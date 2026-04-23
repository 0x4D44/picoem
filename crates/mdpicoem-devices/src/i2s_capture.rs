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
//! samples MSB first. On each LRCLK edge, the in-flight sample is
//! finalised and assigned to either the left or right channel based on
//! LRCLK's new level (standard Philips I2S: LRCLK low = left, LRCLK
//! high = right — see
//! `pico-extras/src/rp2_common/pico_audio_i2s/audio_i2s.pio`).
//!
//! Philips I2S also specifies a one-BCLK delay between the LRCLK edge
//! and the first data bit of the new word. That is handled by simply
//! starting with a fresh `accumulator` and ignoring the first BCLK
//! rising edge after an LRCLK transition — in practice, the standard
//! 32-bit-per-word stereo frame already shifts in 16 data bits followed
//! by 16 junk bits, so by the time LRCLK toggles we have a complete
//! 16-bit word stored. See [`I2sCapture::on_bclk_rising`] for the
//! detail.
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

    // In-flight sample: MSB-first shift register (16 bits).
    accumulator: u16,
    /// Number of data bits already shifted in for the current sample.
    /// I2S words are always clocked with at least 16 BCLKs per half-
    /// frame; once we've latched 16 bits we stop appending, and the
    /// remaining BCLKs in the half-frame are ignored until LRCLK
    /// toggles.
    bit_count: u8,

    /// Channel of the sample currently in `accumulator`. Determined by
    /// the LRCLK level after the most recent LRCLK edge.
    current_channel: Channel,

    /// Frames emitted so far. Order is insertion-order: each frame is
    /// `(left, right)`.
    frames: Vec<(i16, i16)>,
    /// Left-channel sample waiting for its matching right sample.
    pending_left: Option<i16>,

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
            frames: Vec::new(),
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

        // Finalise the sample that was being shifted in under the
        // *previous* LRCLK level. Only emit if we actually clocked in
        // 16 bits — fractional words are ignored (the very first half-
        // frame after capture start typically has bit_count < 16 since
        // we missed the leading BCLK edges).
        if self.bit_count >= 16 {
            let sample = self.accumulator as i16;
            match self.current_channel {
                Channel::Left => {
                    self.pending_left = Some(sample);
                }
                Channel::Right => {
                    let left = self.pending_left.take().unwrap_or(0);
                    self.frames.push((left, sample));
                }
            }
        } else {
            // First (or dropped) half-frame — discard any orphaned
            // left sample so we resync cleanly.
            self.pending_left = None;
        }

        // Reset accumulator for the new half-frame. The Philips I2S
        // one-BCLK delay after LRCLK is handled implicitly: after this
        // edge, `bit_count` is 0 and the next BCLK rising edge will
        // latch DOUT as bit 15. In practice the very first data bit
        // after an LRCLK edge lands one BCLK later, so the first BCLK
        // rising that follows is the MSB of the new word.
        self.accumulator = 0;
        self.bit_count = 0;
        self.current_channel = if new_lrclk { Channel::Right } else { Channel::Left };

        self.lrclk_edges = self.lrclk_edges.saturating_add(1);
        if self.first_lrclk_cycle.is_none() {
            self.first_lrclk_cycle = Some(now_cycles);
        }
        self.last_lrclk_cycle = Some(now_cycles);
    }

    fn on_bclk_rising(&mut self, dout: bool) {
        if self.bit_count >= 16 {
            // Already captured a full word for this half-frame — ignore
            // any further BCLKs until LRCLK toggles.
            return;
        }
        // MSB first: bit 15 shifted in first, bit 0 last.
        self.accumulator = (self.accumulator << 1) | (dout as u16);
        self.bit_count += 1;
        // Diagnostic trace per latched bit. `trace!` (very high
        // frequency: BCLK is ~1.5 MHz at 44 kHz stereo) so production
        // release builds compile this to nothing. Useful when the WAV
        // is silent and we need to confirm DOUT=1 ever shows up at all.
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
        let rate = self.inferred_sample_rate_hz().unwrap_or(fallback_rate as f64);
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
    let data_bytes: u32 = (frames.len() as u32)
        .saturating_mul(block_align as u32);
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

    /// Emit one BCLK period for each of the 16 bits of `word` (MSB
    /// first), while `lrclk` is held constant. Each bit takes two ticks
    /// (low half, high half) so BCLK has a visible rising edge. The
    /// caller's mutable cycle counter is bumped one per tick so edge
    /// timestamps remain monotonic across consecutive `clock_word`
    /// calls.
    fn clock_word(cap: &mut I2sCapture, cycle: &mut u64, lrclk_high: bool, word: u16) {
        for i in (0..16).rev() {
            let bit = (word >> i) & 1 != 0;
            // BCLK low, data presented by transmitter.
            cap.tick(pads(false, lrclk_high, bit), *cycle);
            *cycle += 1;
            // BCLK high — rising edge latches `bit` into the shift
            // register.
            cap.tick(pads(true, lrclk_high, bit), *cycle);
            *cycle += 1;
        }
    }

    #[test]
    fn decodes_known_square_wave() {
        // Encode left = 0x1234, right = 0x5678. Standard Philips I2S:
        // LRCLK low = left channel; LRCLK high = right channel. At
        // startup the capture sees LRCLK as 0 (prev_lrclk=false, no
        // edge on the first tick).
        //
        // The first half-frame (LRCLK=0) primes the decoder — because
        // we need an LRCLK edge to finalise a sample, the very first
        // word we clock in is discarded. So encode: discard-left,
        // right=unused, left=0x1234, right=0x5678, then a trailing
        // LRCLK edge to flush the last right sample.
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut cycle: u64 = 0;

        // Prime: LRCLK low, clock 16 bits of junk (discarded).
        clock_word(&mut cap, &mut cycle, false, 0xAAAA);
        // Edge to LRCLK high — finalises the junk word (as left, since
        // current_channel was initialised to Left). Since bit_count=16,
        // it is written to pending_left. This is the contamination we
        // flush below.
        cap.tick(pads(false, true, false), cycle);
        cycle += 1;
        // Clock 16 bits under LRCLK high (right channel).
        clock_word(&mut cap, &mut cycle, true, 0xBBBB);
        // Edge back to LRCLK low — finalises the junk word as right,
        // emitting frame (junk_left, junk_right).
        cap.tick(pads(false, false, false), cycle);
        cycle += 1;

        let baseline_frames = cap.frames().len();

        // Now the real data.
        clock_word(&mut cap, &mut cycle, false, 0x1234);
        // Edge LRCLK high — finalises 0x1234 as left.
        cap.tick(pads(false, true, false), cycle);
        cycle += 1;
        clock_word(&mut cap, &mut cycle, true, 0x5678);
        // Edge LRCLK low — finalises 0x5678 as right, pushes frame.
        cap.tick(pads(false, false, false), cycle);

        let new_frames: Vec<_> = cap.frames()[baseline_frames..].to_vec();
        assert_eq!(
            new_frames,
            vec![(0x1234i16, 0x5678i16)],
            "expected one frame (0x1234, 0x5678), got {:?}",
            new_frames
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

    // ---- Stage 8b branch-coverage additions ---------------------------------

    /// Covers:
    ///   * line 181 `else` branch (`bit_count < 16` → `pending_left = None`);
    ///   * line 206 false branch (`new_lrclk == false` → `Channel::Left`);
    ///   * line 209 false branch (second+ LRCLK edge keeps `first_lrclk_cycle`);
    ///   * line 216 true branch of `on_bclk_rising` (over-16 shift ignored).
    #[test]
    fn lrclk_without_full_word_clears_pending_left() {
        let mut cap = I2sCapture::new(125_000_000, BCLK, LRCLK, DOUT);
        let mut cycle: u64 = 0;
        // Clock 16 bits of a left-channel sample so `pending_left` fills.
        clock_word(&mut cap, &mut cycle, false, 0xBEEF);
        // LRCLK edge to high → finalises the left sample into `pending_left`
        // and flips `current_channel` to Right. Also seeds
        // `first_lrclk_cycle` → covers the true branch of line 209.
        cap.tick(pads(false, true, false), cycle);
        cycle += 1;
        // Now clock only 4 bits on the right half-frame (< 16) then force a
        // drop-back to LRCLK low. Hits the `else` branch at line 181 which
        // clears `pending_left`, and exercises line 206's `false` arm
        // (new_lrclk=false → Channel::Left) plus line 209's false branch
        // (`first_lrclk_cycle.is_none()` is false on the second edge).
        for _ in 0..4 {
            cap.tick(pads(false, true, false), cycle);
            cycle += 1;
            cap.tick(pads(true, true, false), cycle);
            cycle += 1;
        }
        cap.tick(pads(false, false, false), cycle);
        cycle += 1;
        // No frame should have been emitted: the right side never reached 16.
        assert_eq!(cap.frames().len(), 0);

        // Also hit the line 216 guard: clock MORE than 16 bits on the
        // current (low) half-frame. The extra BCLKs are ignored because
        // `bit_count >= 16` — branch `return` covered.
        clock_word(&mut cap, &mut cycle, false, 0x1234);
        // Clock 8 extra bits after the full 16 → exercises the early
        // return at line 216.
        for _ in 0..8 {
            cap.tick(pads(false, false, true), cycle);
            cycle += 1;
            cap.tick(pads(true, false, true), cycle);
            cycle += 1;
        }
        // Flush.
        cap.tick(pads(false, true, false), cycle);
        cycle += 1;
        clock_word(&mut cap, &mut cycle, true, 0x5678);
        cap.tick(pads(false, false, false), cycle);

        // The new left/right pair must survive despite the extra BCLKs.
        let last = *cap.frames().last().expect("one frame emitted");
        assert_eq!(last, (0x1234i16, 0x5678i16));
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
