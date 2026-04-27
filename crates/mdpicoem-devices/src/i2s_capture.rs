//! I2S capture -> WAV writer.
//!
//! Decodes a stereo 16-bit I2S stream from GPIO pins and writes
//! the captured frames as a canonical 16-bit PCM WAV file. The pin
//! numbers are parameterised at construction time so the same capture
//! logic works for any board mapping (PicoGUS, custom test rigs, etc.).
//!
//! [`I2sCapture::tick`] is called **once per emulator cycle** with the
//! current merged GPIO state (from `emu.bus.gpio_in`) and the caller's
//! system-clock cycle stamp. The cycle stamp must track actual sysclks
//! elapsed (e.g. `Emulator::cycles()`) — not the number of `tick` calls
//! — otherwise `inferred_sample_rate_hz` drifts by a factor of the
//! average cycles-per-instruction (~1.5-2x) when the caller steps one
//! instruction (multi-cycle) per tick. See [`Self::tick`] for details.
//!
//! The capture observes BCLK and LRCLK edges and assembles 16-bit PCM
//! samples MSB first. Channel mapping follows `pico-extras/src/rp2_common/
//! pico_audio_i2s/audio_i2s.pio`: the program's entry-point window sends
//! bits 31..16 of the 32-bit FIFO word under LRCLK=1 and bits 15..0 under
//! LRCLK=0, and PicoGUS firmware packs `(RIGHT << 16) | LEFT`. So the
//! data clocked in under LRCLK=1 is the RIGHT sample, LRCLK=0 is LEFT.
//!
//! **Philips one-BCLK delay.** The PIO aligns each 16-bit word across
//! an LRCLK edge: 15 MSBs are latched on one LRCLK side, and the
//! remaining LSB is latched on the NEXT BCLK rising edge after the
//! LRCLK transition. A naive "16 bits per LRCLK window" receiver would
//! mis-align by exactly 1 bit — observed as `0xFF` pushed →
//! `L=127, R=-32768` (= `0xFF>>1` and `1<<15`). This decoder therefore
//! holds the 15-bit accumulator across the LRCLK edge, latches the
//! 16th (LSB) bit on the first BCLK rising after the edge, and commits
//! the completed word to the channel whose LRCLK level was active during
//! the 15 MSBs (see [`I2sCapture::on_bclk_rising`]).
//!
//! No dependency on an external WAV crate — the file format is
//! documented at <https://soundfile.sapp.org/doc/WaveFormat/>; we write
//! the 44-byte canonical PCM header by hand.

use std::fs;
use std::io::{self, Write};
use std::path::Path;

use tracing::{debug, trace};

/// Which channel the next finalised sample belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Channel {
    Left,
    Right,
}

/// Fixed header length for the canonical 16-bit PCM WAV — 12 bytes
/// "RIFF" chunk + 24 bytes "fmt " sub-chunk + 8 bytes "data" sub-chunk.
pub const WAV_HEADER_BYTES: usize = 44;

/// I2S decoder + WAV writer.
///
/// Call [`Self::tick`] once per emulator cycle with the merged pin
/// state. When done, call [`Self::write_wav`] to persist the captured
/// frames.
#[derive(Debug)]
pub struct I2sCapture {
    // Pin assignments (GPIO numbers).
    bclk_pin: u8,
    lrclk_pin: u8,
    dout_pin: u8,

    // Last observed pin levels, for edge detection.
    prev_bclk: bool,
    prev_lrclk: bool,

    // In-flight sample: MSB-first shift register.
    accumulator: u16,
    /// Number of data bits already shifted in for the current
    /// channel's word. In steady state, rises to 15 over the current
    /// LRCLK window, then an LRCLK edge transfers the 15-bit
    /// accumulator into "finalizing" state and the 16th (LSB) bit is
    /// latched by the first BCLK rising after the edge.
    bit_count: u8,

    /// Channel of the bits currently accumulating (i.e., the new side
    /// after the most recent LRCLK edge). The word being *finalized*
    /// on any pending LSB belongs to the channel in
    /// [`Self::finalizing_channel`], not this field.
    current_channel: Channel,

    /// After an LRCLK edge, the 15-bit `accumulator` holds the MSBs of
    /// the *previous* channel's 16-bit word. The next BCLK rising edge
    /// latches that word's LSB and commits it. `None` between edges
    /// (normal accumulation phase) and at startup before the first
    /// edge has been seen.
    finalizing_channel: Option<Channel>,

    /// Frames emitted so far. Order is insertion-order: each frame is
    /// `(left, right)`.
    frames: Vec<(i16, i16)>,
    /// Half-frame buffers: whichever channel arrives first is parked
    /// here until its mate lands, at which point a frame is emitted.
    /// `pico-extras/audio_i2s.pio` sends RIGHT first per FIFO word
    /// (bits 31..16 under LRCLK=1), so in practice `pending_right`
    /// fills then the subsequent LEFT commit emits. Tracking both
    /// makes the decoder tolerant of the opposite ordering and of
    /// mid-stream resyncs.
    pending_left: Option<i16>,
    pending_right: Option<i16>,

    // Wall-clock / sample-rate inference.
    /// Cycle index at which the first LRCLK edge was observed. Sourced
    /// from the caller's `now_cycles` argument to [`Self::tick`] — so
    /// this is a system-clock timestamp, not a tick counter.
    first_lrclk_cycle: Option<u64>,
    /// Cycle index of the most recent LRCLK edge. Same units as
    /// [`Self::first_lrclk_cycle`].
    last_lrclk_cycle: Option<u64>,
    /// Count of LRCLK edges seen. One full frame = one falling + one
    /// rising edge = 2 edges.
    lrclk_edges: u64,

    /// System clock in Hz; used by sample-rate inference.
    sys_clk_hz: u32,
}

impl I2sCapture {
    /// Build a new capture expecting the emulator to run at
    /// `sys_clk_hz` (used only for sample-rate inference on the
    /// captured output). The three pin numbers identify which GPIO
    /// bits in the `pads` argument to [`Self::tick`] carry the I2S
    /// signals.
    pub fn new(sys_clk_hz: u32, bclk_pin: u8, lrclk_pin: u8, dout_pin: u8) -> Self {
        Self {
            bclk_pin,
            lrclk_pin,
            dout_pin,
            prev_bclk: false,
            prev_lrclk: false,
            accumulator: 0,
            bit_count: 0,
            current_channel: Channel::Left,
            finalizing_channel: None,
            frames: Vec::new(),
            pending_right: None,
            pending_left: None,
            first_lrclk_cycle: None,
            last_lrclk_cycle: None,
            lrclk_edges: 0,
            sys_clk_hz,
        }
    }

    /// Observe the merged GPIO state at system-clock cycle `now_cycles`.
    /// `pads` is the same bit layout as `mdrp2040::Bus::gpio_in` /
    /// `Emulator::gpio_read_all`: bit `N` is the logical level of `GPIOn`.
    ///
    /// `now_cycles` MUST be a monotonic system-clock cycle stamp — pass
    /// `Emulator::cycles()` from a caller that advances one instruction
    /// (potentially multi-sysclk) per `step`. Using the per-call tick
    /// count instead would skew [`Self::inferred_sample_rate_hz`] by the
    /// average cycles-per-instruction and stamp the resulting WAV with
    /// the wrong sample rate.
    pub fn tick(&mut self, pads: u32, now_cycles: u64) {
        let bclk = pads & (1u32 << self.bclk_pin) != 0;
        let lrclk = pads & (1u32 << self.lrclk_pin) != 0;
        let dout = pads & (1u32 << self.dout_pin) != 0;

        // LRCLK edge: finalise the in-flight sample, assign it to the
        // channel matching the *previous* LRCLK level, then reset the
        // accumulator for the new half-frame.
        if lrclk != self.prev_lrclk {
            self.on_lrclk_edge(lrclk, now_cycles);
        }

        // BCLK rising edge: shift DOUT into the accumulator MSB-first.
        if bclk && !self.prev_bclk {
            self.on_bclk_rising(dout);
        }

        self.prev_bclk = bclk;
        self.prev_lrclk = lrclk;
    }

    fn on_lrclk_edge(&mut self, new_lrclk: bool, now_cycles: u64) {
        // Diagnostic trace: every LRCLK edge with the in-flight sample
        // and the bit count latched. Pure observation. `debug!` so it
        // stays out of release builds; the silent-WAV diagnosis runs a
        // debug build of `picogus_diff_rp2040` against this target.
        debug!(
            target: "mdpicoem_devices::i2s_capture",
            now_cycles,
            new_lrclk,
            channel = ?self.current_channel,
            accumulator = format_args!("0x{:04x}", self.accumulator),
            bit_count = self.bit_count,
            edges_seen = self.lrclk_edges,
            "lrclk_edge",
        );

        // Philips one-BCLK delay: the 15 MSBs of the previous channel's
        // 16-bit word are in `accumulator` now; the LSB arrives on the
        // next BCLK rising edge. Mark the word as finalizing and stash
        // which channel it belongs to (the channel active during those
        // 15 MSBs, i.e., the current channel BEFORE this edge).
        if self.bit_count == 15 {
            self.finalizing_channel = Some(self.current_channel);
            // Keep `accumulator` — its 15 MSBs are needed for the LSB
            // shift-in below.
        } else {
            // First edge (priming), malformed frame, or a transmitter
            // that doesn't match the PIO timing — drop any in-flight
            // data and resync cleanly.
            self.finalizing_channel = None;
            self.accumulator = 0;
            self.pending_left = None;
            self.pending_right = None;
        }
        self.bit_count = 0;

        // The new side of LRCLK becomes the current channel. Actual
        // commit of the finalizing word happens on the next BCLK
        // rising edge (which latches that word's LSB).
        self.current_channel = if new_lrclk {
            Channel::Right
        } else {
            Channel::Left
        };

        self.lrclk_edges = self.lrclk_edges.saturating_add(1);
        if self.first_lrclk_cycle.is_none() {
            self.first_lrclk_cycle = Some(now_cycles);
        }
        self.last_lrclk_cycle = Some(now_cycles);
    }

    fn on_bclk_rising(&mut self, dout: bool) {
        // Philips one-BCLK delay: if a previous channel's word is
        // awaiting its LSB, this BCLK rising edge provides it.
        if let Some(finalize) = self.finalizing_channel.take() {
            self.accumulator = (self.accumulator << 1) | (dout as u16);
            let sample = self.accumulator as i16;
            trace!(
                target: "mdpicoem_devices::i2s_capture",
                dout,
                channel = ?finalize,
                committed = format_args!("0x{:04x}", sample as u16),
                "philips_lsb_commit",
            );
            // Pair the commit against whichever channel is already
            // parked in the half-frame buffer; otherwise stash this
            // one and wait for its mate. Works for either
            // RIGHT-then-LEFT (the pico-audio PIO ordering) or the
            // reverse.
            match finalize {
                Channel::Left => {
                    if let Some(right) = self.pending_right.take() {
                        self.frames.push((sample, right));
                    } else {
                        self.pending_left = Some(sample);
                    }
                }
                Channel::Right => {
                    if let Some(left) = self.pending_left.take() {
                        self.frames.push((left, sample));
                    } else {
                        self.pending_right = Some(sample);
                    }
                }
            }
            // Reset accumulator for the new channel. `bit_count` stays
            // at 0 because this BCLK was consumed for the finalizing
            // LSB, not for a new-channel MSB.
            self.accumulator = 0;
            return;
        }

        // Normal accumulation: at most 15 bits per LRCLK window. The
        // 16th bit (LSB) arrives on the first BCLK rising after the
        // LRCLK edge via the `finalizing_channel` branch above.
        if self.bit_count >= 15 {
            return;
        }
        self.accumulator = (self.accumulator << 1) | (dout as u16);
        self.bit_count += 1;
        trace!(
            target: "mdpicoem_devices::i2s_capture",
            dout,
            bit_count = self.bit_count,
            accumulator = format_args!("0x{:04x}", self.accumulator),
            "bclk_rising_latch",
        );
    }

    /// Override the `sys_clk_hz` used by [`Self::inferred_sample_rate_hz`].
    ///
    /// Edge timestamps are stored in cycle-domain, so the sample-rate
    /// inference is simply `sys_clk_hz * (edges-1) / (2 * (last-first))`.
    /// When firmware reprograms PLL mid-capture (e.g. PicoGUS goes from
    /// 125 MHz → 370 MHz early in boot, well before any I2S edges), the
    /// harness should call this with the post-reprogram clock before
    /// reporting, otherwise the inferred rate is wrong by the clock
    /// ratio.
    pub fn set_sys_clk_hz(&mut self, sys_clk_hz: u32) {
        self.sys_clk_hz = sys_clk_hz;
    }

    /// Currently configured `sys_clk_hz` (for diagnostics / tests).
    pub fn sys_clk_hz(&self) -> u32 {
        self.sys_clk_hz
    }

    /// The captured stereo frames, in emit order.
    pub fn frames(&self) -> &[(i16, i16)] {
        &self.frames
    }

    /// LRCLK edge count observed so far (2 per full stereo frame).
    pub fn lrclk_edge_count(&self) -> u64 {
        self.lrclk_edges
    }

    /// Infer the sample rate in Hz from observed LRCLK timing. Returns
    /// `None` if fewer than two edges have been seen — in that case
    /// there is no measurable period yet.
    ///
    /// A single full frame spans two LRCLK edges (one falling + one
    /// rising). Between the first and last edge we observed
    /// `lrclk_edges - 1` half-periods, each of which is
    /// `(last - first) / (edges - 1)` cycles. Converting to frequency
    /// gives `sys_clk_hz * (edges - 1) / (2 * (last - first))`.
    pub fn inferred_sample_rate_hz(&self) -> Option<f64> {
        let first = self.first_lrclk_cycle?;
        let last = self.last_lrclk_cycle?;
        if self.lrclk_edges < 2 || last <= first {
            return None;
        }
        let half_periods = self.lrclk_edges.saturating_sub(1) as f64;
        let total_cycles = (last - first) as f64;
        let freq = self.sys_clk_hz as f64 * half_periods / (2.0 * total_cycles);
        Some(freq)
    }

    /// Duration of the captured audio in seconds, using the sample-
    /// rate estimate from [`Self::inferred_sample_rate_hz`] and falling
    /// back to the given `fallback_rate` when fewer than 2 LRCLK edges
    /// were observed.
    pub fn duration_secs(&self, fallback_rate: u32) -> f64 {
        let rate = self
            .inferred_sample_rate_hz()
            .unwrap_or(fallback_rate as f64);
        if rate <= 0.0 {
            return 0.0;
        }
        self.frames.len() as f64 / rate
    }

    /// Write the captured frames as a 16-bit stereo PCM WAV to `path`.
    ///
    /// `sample_rate_hz` is the rate stamped into the WAV header — the
    /// caller should pass the inferred rate (or a sensible fallback
    /// like 44 100 Hz when nothing was captured).
    ///
    /// Rejects `path` if it is an existing directory. Creates any
    /// missing parent directory components.
    pub fn write_wav(&self, path: &Path, sample_rate_hz: u32) -> io::Result<()> {
        write_wav(path, sample_rate_hz, &self.frames)
    }
}

/// Write a 16-bit stereo WAV file at `path` containing `frames`. Free-
/// standing so callers that already have a `Vec<(i16, i16)>` need not
/// construct an [`I2sCapture`].
pub fn write_wav(path: &Path, sample_rate_hz: u32, frames: &[(i16, i16)]) -> io::Result<()> {
    if path.is_dir() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "refusing to write WAV to existing directory: {}",
                path.display()
            ),
        ));
    }
    if let Some(parent) = path.parent() {
        // Only create if parent has at least one component — empty for
        // plain file names in the CWD.
        if !parent.as_os_str().is_empty() && !parent.exists() {
            fs::create_dir_all(parent)?;
        }
    }

    let channels: u16 = 2;
    let bits_per_sample: u16 = 16;
    let bytes_per_sample: u16 = bits_per_sample / 8;
    let block_align: u16 = channels * bytes_per_sample;
    let byte_rate: u32 = sample_rate_hz * (block_align as u32);
    let data_bytes: u32 = (frames.len() as u32).saturating_mul(block_align as u32);
    let riff_size: u32 = 36u32.saturating_add(data_bytes);

    let mut buf: Vec<u8> = Vec::with_capacity(WAV_HEADER_BYTES + data_bytes as usize);

    // RIFF header
    buf.extend_from_slice(b"RIFF");
    buf.extend_from_slice(&riff_size.to_le_bytes());
    buf.extend_from_slice(b"WAVE");

    // fmt sub-chunk
    buf.extend_from_slice(b"fmt ");
    buf.extend_from_slice(&16u32.to_le_bytes()); // PCM fmt chunk size
    buf.extend_from_slice(&1u16.to_le_bytes()); // audio format = PCM
    buf.extend_from_slice(&channels.to_le_bytes());
    buf.extend_from_slice(&sample_rate_hz.to_le_bytes());
    buf.extend_from_slice(&byte_rate.to_le_bytes());
    buf.extend_from_slice(&block_align.to_le_bytes());
    buf.extend_from_slice(&bits_per_sample.to_le_bytes());

    // data sub-chunk
    buf.extend_from_slice(b"data");
    buf.extend_from_slice(&data_bytes.to_le_bytes());
    debug_assert_eq!(buf.len(), WAV_HEADER_BYTES);

    for (l, r) in frames {
        buf.extend_from_slice(&l.to_le_bytes());
        buf.extend_from_slice(&r.to_le_bytes());
    }

    let mut f = fs::File::create(path)?;
    f.write_all(&buf)?;
    f.sync_all()?;
    Ok(())
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    // Local pin constants for tests — match PicoGUS v4.0.0 mapping.
    const BCLK: u8 = 17;
    const LRCLK: u8 = 18;
    const DOUT: u8 = 16;

    /// One "cycle" of the pin bus, as fed to [`I2sCapture::tick`].
    /// Helper to make the scripted waveform tests readable.
    fn pads(bclk: bool, lrclk: bool, dout: bool) -> u32 {
        let mut p = 0u32;
        if bclk {
            p |= 1u32 << BCLK;
        }
        if lrclk {
            p |= 1u32 << LRCLK;
        }
        if dout {
            p |= 1u32 << DOUT;
        }
        p
    }

    /// Emit one 32-bit FIFO frame using `pico-extras/audio_i2s.pio`
    /// bit timing. Order (MSB first from the FIFO word
    /// `(right << 16) | left`):
    ///   - bits 31..17 under LRCLK=1 (top 15 bits of RIGHT)
    ///   - bit 16 under LRCLK=0 (LSB of RIGHT — LRCLK falling)
    ///   - bits 15..1 under LRCLK=0 (top 15 bits of LEFT)
    ///   - bit 0 under LRCLK=1 (LSB of LEFT — LRCLK rising)
    /// 32 BCLK rising edges per frame, so `cycle` advances by 64.
    fn clock_philips_frame(cap: &mut I2sCapture, cycle: &mut u64, left: u16, right: u16) {
        // 15 bits of RIGHT (bit 15 down to bit 1) with LRCLK=1.
        for i in (1..=15).rev() {
            let bit = (right >> i) & 1 != 0;
            cap.tick(pads(false, true, bit), *cycle);
            *cycle += 1;
            cap.tick(pads(true, true, bit), *cycle);
            *cycle += 1;
        }
        // LSB of RIGHT with LRCLK=0 (LRCLK falling edge on first tick).
        let bit = right & 1 != 0;
        cap.tick(pads(false, false, bit), *cycle);
        *cycle += 1;
        cap.tick(pads(true, false, bit), *cycle);
        *cycle += 1;
        // 15 bits of LEFT (bit 15 down to bit 1) with LRCLK=0.
        for i in (1..=15).rev() {
            let bit = (left >> i) & 1 != 0;
            cap.tick(pads(false, false, bit), *cycle);
            *cycle += 1;
            cap.tick(pads(true, false, bit), *cycle);
            *cycle += 1;
        }
        // LSB of LEFT with LRCLK=1 (LRCLK rising edge on first tick).
        let bit = left & 1 != 0;
        cap.tick(pads(false, true, bit), *cycle);
        *cycle += 1;
        cap.tick(pads(true, true, bit), *cycle);
        *cycle += 1;
    }

    #[test]
    fn decodes_known_square_wave() {
        // Feed three frames using pico-audio-i2s.pio timing and check
        // they decode 1:1. The decoder starts with `prev_lrclk=false`,
        // so the helper's first tick (LRCLK=1) triggers a priming edge
        // that clears accumulator state — frame 0 still decodes because
        // the priming happens before any bits are shifted.
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut cycle: u64 = 0;

        clock_philips_frame(&mut cap, &mut cycle, 0x1234, 0x5678);
        clock_philips_frame(&mut cap, &mut cycle, 0x0001, 0x0002);
        clock_philips_frame(
            &mut cap, &mut cycle, 0xC000, /* -0x4000 as u16 */
            0x7FFF,
        );

        assert_eq!(
            cap.frames(),
            &[
                (0x1234i16, 0x5678i16),
                (0x0001, 0x0002),
                (-0x4000i16, 0x7FFFi16),
            ],
            "Philips timing decode mismatch: frames = {:?}",
            cap.frames()
        );
    }

    /// Exact reproduction of the stub-1 observation from the
    /// 2026-04-23 journal: PicoGUS firmware pushed `0x000000FF` to
    /// TXF constantly, and the naive receiver saw `L=127, R=-32768`.
    /// With the Philips-aware fix, a PIO-accurate transmission of
    /// that same value must decode as `L=0x00FF, R=0x0000`.
    #[test]
    fn stub1_constant_0xff_decodes_correctly() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut cycle: u64 = 0;
        // FIFO word 0x000000FF = (right=0 << 16) | (left=0x00FF).
        for _ in 0..4 {
            clock_philips_frame(&mut cap, &mut cycle, 0x00FF, 0x0000);
        }
        let last = *cap.frames().last().expect("expected frames emitted");
        assert_eq!(
            last,
            (0x00FFi16, 0x0000i16),
            "Philips decoder must recover L=0x00FF, R=0x0000 from PIO \
             output of (right<<16)|left = 0x000000FF; got {:?}",
            last,
        );
    }

    #[test]
    fn wav_file_roundtrip() {
        let tmp = tmp_dir().join("i2s_roundtrip.wav");
        let frames: Vec<(i16, i16)> = (0..100)
            .map(|i| (i as i16 * 10, -(i as i16) * 10))
            .collect();
        write_wav(&tmp, 48_000, &frames).expect("write wav");

        let bytes = fs::read(&tmp).expect("read wav");
        assert!(
            bytes.len() >= WAV_HEADER_BYTES,
            "file smaller than header: {} bytes",
            bytes.len()
        );

        // Magic
        assert_eq!(&bytes[0..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        assert_eq!(&bytes[12..16], b"fmt ");
        assert_eq!(&bytes[36..40], b"data");

        // Audio format PCM = 1, channels = 2, bits = 16.
        let audio_fmt = u16::from_le_bytes([bytes[20], bytes[21]]);
        let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
        let sample_rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
        let byte_rate = u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]);
        let block_align = u16::from_le_bytes([bytes[32], bytes[33]]);
        let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
        assert_eq!(audio_fmt, 1, "audio format != PCM");
        assert_eq!(channels, 2);
        assert_eq!(sample_rate, 48_000);
        assert_eq!(bits, 16);
        assert_eq!(block_align, 4);
        assert_eq!(byte_rate, 48_000 * 4);

        // data chunk size = frames * 4 bytes.
        let data_size = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]);
        assert_eq!(data_size, 100 * 4);

        // Payload round-trip.
        assert_eq!(bytes.len(), WAV_HEADER_BYTES + data_size as usize);
        for (i, (l, r)) in frames.iter().enumerate() {
            let off = WAV_HEADER_BYTES + i * 4;
            let read_l = i16::from_le_bytes([bytes[off], bytes[off + 1]]);
            let read_r = i16::from_le_bytes([bytes[off + 2], bytes[off + 3]]);
            assert_eq!(read_l, *l, "left sample {i} mismatch");
            assert_eq!(read_r, *r, "right sample {i} mismatch");
        }

        let _ = fs::remove_file(&tmp);
    }

    #[test]
    fn no_activity_produces_empty_wav() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        for i in 0..10_000u64 {
            cap.tick(0, i);
        }
        assert_eq!(cap.frames().len(), 0);
        assert_eq!(cap.lrclk_edge_count(), 0);
        assert!(cap.inferred_sample_rate_hz().is_none());

        let tmp = tmp_dir().join("i2s_empty.wav");
        cap.write_wav(&tmp, 44_100).expect("write empty wav");
        let bytes = fs::read(&tmp).expect("read empty wav");
        assert_eq!(
            bytes.len(),
            WAV_HEADER_BYTES,
            "empty WAV must be exactly the 44-byte header"
        );

        // data chunk size field must read zero.
        let data_size = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]);
        assert_eq!(data_size, 0);
        // RIFF size = 36 + data = 36.
        let riff_size = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        assert_eq!(riff_size, 36);

        let _ = fs::remove_file(&tmp);
    }

    #[test]
    fn sample_rate_inferred_from_lrclk() {
        // Script a 48 kHz stream at 125 MHz sys_clk. One full LRCLK
        // period = 125_000_000 / 48_000 cycles ~= 2604.17 cycles. We
        // use an exact-integer alternative: 4 kHz at 64 MHz sys_clk ->
        // period 16000 cycles, half-period 8000.
        //
        // Simpler: fake sys_clk = 32_000, LRCLK period = 2 cycles
        // -> 16 kHz. Generate many LRCLK edges and check the inferred
        // rate lands at 16 kHz.
        //
        // This test explicitly passes a monotonic sysclk counter to
        // `tick` to prove `inferred_sample_rate_hz` consumes the
        // externally supplied stamp (not an internal tick count).
        let sys_clk = 32_000u32;
        let mut cap = I2sCapture::new(sys_clk, BCLK, LRCLK, DOUT);

        // 10 full frames => 20 LRCLK edges, each half-period 1 cycle.
        // We need non-trivial half-periods to avoid the first/last
        // cycle being equal. Use 100 cycles per half-period = 200
        // cycles per frame => 160 Hz.
        let half_period_cycles: u32 = 100;
        let target_rate = sys_clk as f64 / (2.0 * half_period_cycles as f64);
        let mut lrclk = false;
        let mut cycle: u64 = 0;
        for _frame in 0..50 {
            for _ in 0..half_period_cycles {
                cap.tick(pads(false, lrclk, false), cycle);
                cycle += 1;
            }
            lrclk = !lrclk;
        }
        let inferred = cap
            .inferred_sample_rate_hz()
            .expect("should have edges after 50 frames");
        let relative_err = (inferred - target_rate).abs() / target_rate;
        assert!(
            relative_err < 0.05,
            "inferred {inferred:.3} Hz vs target {target_rate:.3} Hz (err {relative_err:.3})"
        );
    }

    #[test]
    fn pad_mask_respects_pin_mapping() {
        // Drive BCLK/LRCLK on the WRONG pins (e.g. GPIO 7 and 8, which
        // are part of the ISA AD bus). Expect zero frames, zero edges —
        // the capture only looks at bits bclk_pin/lrclk_pin/dout_pin.
        const WRONG_BCLK: u8 = 7;
        const WRONG_LRCLK: u8 = 8;
        const WRONG_DOUT: u8 = 9;

        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut lrclk = false;
        let mut cycle: u64 = 0;
        for _ in 0..200 {
            for i in 0..16 {
                let bit = (i & 1) != 0;
                let mut pads_val = 0u32;
                if bit {
                    pads_val |= 1u32 << WRONG_DOUT;
                }
                if lrclk {
                    pads_val |= 1u32 << WRONG_LRCLK;
                }
                // BCLK low
                cap.tick(pads_val, cycle);
                cycle += 1;
                // BCLK high
                cap.tick(pads_val | (1u32 << WRONG_BCLK), cycle);
                cycle += 1;
            }
            lrclk = !lrclk;
        }
        assert_eq!(
            cap.frames().len(),
            0,
            "wrong-pin activity must not produce frames"
        );
        assert_eq!(cap.lrclk_edge_count(), 0);
    }

    #[test]
    fn write_wav_rejects_directory_path() {
        let dir = tmp_dir();
        let _ = fs::create_dir_all(&dir);
        let err = write_wav(&dir, 44_100, &[]).expect_err("writing to dir must fail");
        assert_eq!(err.kind(), io::ErrorKind::InvalidInput);
    }

    #[test]
    fn write_wav_creates_missing_parent_dirs() {
        // Build a unique nested path that doesn't exist yet.
        let root = tmp_dir();
        let nested = root.join("i2s_nested").join("a").join("b").join("out.wav");
        // Ensure a clean slate.
        let _ = fs::remove_dir_all(root.join("i2s_nested"));
        write_wav(&nested, 44_100, &[(1, 2), (3, 4)]).expect("nested write");
        assert!(nested.exists(), "nested WAV not created");
        let bytes = fs::read(&nested).expect("read");
        assert_eq!(bytes.len(), WAV_HEADER_BYTES + 8);
        let _ = fs::remove_dir_all(root.join("i2s_nested"));
    }

    /// A per-test scratch directory under `target/` — the workspace
    /// already .gitignores `target/`, so leaving stray files behind
    /// doesn't pollute git status.
    fn tmp_dir() -> PathBuf {
        let base = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("target").join("i2s_capture_tests"))
            .unwrap_or_else(|| PathBuf::from("target/i2s_capture_tests"));
        let _ = fs::create_dir_all(&base);
        base
    }

    // ---- Branch-coverage additions (Philips-aware) ---------------------------

    /// Covers the resync path in `on_lrclk_edge`: if an LRCLK edge
    /// arrives with `bit_count != 15`, the decoder drops in-flight
    /// state and waits for the next clean frame. Also exercises the
    /// `bit_count >= 15` guard in `on_bclk_rising` (extra BCLKs after
    /// 15 are ignored until the LRCLK edge transfers to finalizing).
    #[test]
    fn short_window_drops_state_then_resyncs() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut cycle: u64 = 0;

        // First: establish a normal frame so pending/buffer logic is
        // warmed up.
        clock_philips_frame(&mut cap, &mut cycle, 0x0AAA, 0x0BBB);
        assert_eq!(cap.frames().len(), 1);

        // Now drive a corrupted window: LRCLK high with only 3 BCLK
        // edges, then an LRCLK edge. This triggers the `bit_count !=
        // 15` branch of on_lrclk_edge: finalizing stays None, any
        // half-pair is dropped.
        for _ in 0..3 {
            cap.tick(pads(false, true, false), cycle);
            cycle += 1;
            cap.tick(pads(true, true, false), cycle);
            cycle += 1;
        }
        cap.tick(pads(false, false, false), cycle); // LRCLK 1→0 edge, bit_count=3
        cycle += 1;

        // Resync: drive a clean frame. It must decode correctly.
        clock_philips_frame(&mut cap, &mut cycle, 0x0123, 0x4567);
        // Extra BCLKs DURING the second clean frame's LRCLK=0 window
        // (more than 15 bits presented) must be ignored per the
        // `bit_count >= 15` early return in on_bclk_rising.
        clock_philips_frame(&mut cap, &mut cycle, 0x00FF, 0x7F00);

        let frames = cap.frames();
        assert_eq!(frames.len(), 3, "got {} frames, expected 3", frames.len());
        assert_eq!(frames[0], (0x0AAAi16, 0x0BBBi16));
        assert_eq!(frames[1], (0x0123i16, 0x4567i16));
        assert_eq!(frames[2], (0x00FFi16, 0x7F00i16));
    }

    /// Covers `on_bclk_rising`'s `bit_count >= 15` early-return branch
    /// by injecting extra BCLK rising edges within a single LRCLK
    /// window (more than the 15 the Philips protocol expects).
    #[test]
    fn extra_bclks_within_window_are_ignored() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut cycle: u64 = 0;

        // One warm-up frame, then an intentionally over-long LRCLK=1
        // window (25 BCLK edges instead of 15 + 1-edge-at-bound).
        clock_philips_frame(&mut cap, &mut cycle, 0, 0);

        // Over-long RIGHT window: 25 BCLK ticks (all DOUT=1) with
        // LRCLK=1. Only the first 15 should land in the accumulator;
        // the rest hit the `bit_count >= 15` guard.
        for _ in 0..25 {
            cap.tick(pads(false, true, true), cycle);
            cycle += 1;
            cap.tick(pads(true, true, true), cycle);
            cycle += 1;
        }
        // Abort with an LRCLK edge → bit_count is 15 (not 25), so
        // finalizing takes effect. Shift in the LSB with LRCLK=0.
        cap.tick(pads(false, false, true), cycle); // edge 1→0
        cycle += 1;
        cap.tick(pads(true, false, true), cycle); // LSB of RIGHT = 1
        cycle += 1;

        // Close the frame with a 15-bit LEFT=0 and LSB under LRCLK=1.
        for _ in 0..15 {
            cap.tick(pads(false, false, false), cycle);
            cycle += 1;
            cap.tick(pads(true, false, false), cycle);
            cycle += 1;
        }
        cap.tick(pads(false, true, false), cycle); // edge 0→1
        cycle += 1;
        cap.tick(pads(true, true, false), cycle); // LSB of LEFT = 0

        // RIGHT was 15 1-bits + 1 LSB=1 = 0xFFFF. LEFT was all zero.
        let last = *cap.frames().last().expect("frame emitted");
        assert_eq!(
            last,
            (0x0000i16, -1i16),
            "over-long window must still decode to (0x0000, 0xFFFF=-1), got {:?}",
            last,
        );
    }

    /// Covers line 277 (edges == 1 → `None`) and line 292 (`rate == 0`
    /// fallback branch when the fallback is zero).
    #[test]
    fn duration_secs_zero_fallback_returns_zero() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        // Drive exactly one LRCLK edge so edges == 1 → the `< 2` guard
        // short-circuits with `None` (line 277 true branch).
        cap.tick(pads(false, false, false), 0);
        cap.tick(pads(false, true, false), 1);
        assert_eq!(cap.lrclk_edge_count(), 1);
        assert!(cap.inferred_sample_rate_hz().is_none());
        // With a zero fallback the `rate <= 0.0` guard returns 0.0.
        assert_eq!(cap.duration_secs(0), 0.0);
    }

    /// Covers line 277's `last <= first` branch (both stamps are equal
    /// → return `None`).
    #[test]
    fn inferred_rate_zero_elapsed_returns_none() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        // Drive three LRCLK edges all at cycle 0 — edges ≥ 2 but
        // `last == first` triggers the second clause of the guard.
        cap.tick(pads(false, false, false), 0);
        cap.tick(pads(false, true, false), 0);
        cap.tick(pads(false, false, false), 0);
        cap.tick(pads(false, true, false), 0);
        assert!(cap.lrclk_edge_count() >= 2);
        assert!(cap.inferred_sample_rate_hz().is_none());
    }

    /// `write_wav` with a bare filename (no parent directory component)
    /// must succeed — covers the `None` arm of `path.parent()` (free
    /// function, line 324 false branch) and the `parent.as_os_str()` empty
    /// check at line 327.
    #[test]
    fn write_wav_bare_filename_no_parent_creation() {
        // Chdir to a temp dir so we don't pollute the workspace root.
        let dir = tmp_dir().join("bare_name");
        let _ = fs::create_dir_all(&dir);
        let prev = std::env::current_dir().expect("cwd");
        std::env::set_current_dir(&dir).expect("chdir into tmp");
        // Plain file name, no path separator → `Path::parent` yields
        // Some("") (empty parent component). Hits line 327's empty-parent
        // branch instead of `fs::create_dir_all`.
        let bare = Path::new("i2s_bare.wav");
        let res = write_wav(bare, 22_050, &[(1, 2)]);
        // Restore cwd BEFORE asserting so a failure doesn't leave the
        // test process in a strange directory.
        std::env::set_current_dir(&prev).expect("restore cwd");
        res.expect("bare filename should succeed");
        let bytes = fs::read(dir.join("i2s_bare.wav")).expect("read");
        assert_eq!(bytes.len(), WAV_HEADER_BYTES + 4);
        let _ = fs::remove_file(dir.join("i2s_bare.wav"));
    }

    /// Covers `set_sys_clk_hz` / `sys_clk_hz` / `I2sCapture::write_wav`
    /// method wrapper — otherwise only exercised via the free function.
    #[test]
    fn set_sys_clk_hz_and_method_write_wav() {
        let mut cap = I2sCapture::new(100_000_000, BCLK, LRCLK, DOUT);
        assert_eq!(cap.sys_clk_hz(), 100_000_000);
        cap.set_sys_clk_hz(125_000_000);
        assert_eq!(cap.sys_clk_hz(), 125_000_000);

        let tmp = tmp_dir().join("i2s_method.wav");
        cap.write_wav(&tmp, 48_000).expect("method write");
        let bytes = fs::read(&tmp).expect("read");
        assert_eq!(bytes.len(), WAV_HEADER_BYTES);
        let _ = fs::remove_file(&tmp);
    }
}
