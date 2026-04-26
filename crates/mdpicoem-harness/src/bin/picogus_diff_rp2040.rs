// PicoGUS trace replayer — mdrp2040 (Cortex-M0+) harness.
//
// Stages 4 + 5 of the PicoGUS Integration HLD
// (`wrk_docs/2026.04.14 - HLD - PicoGUS Integration.md`). Reads a CSV
// trace captured from a patched DOSBox-X and drives synthetic ISA bus
// cycles into our `mdrp2040::Emulator`, stepping virtual time forward
// to match each event's wall-clock offset. Per-cycle samples the I2S
// pins (BCLK / LRCLK / DOUT) and writes the decoded stereo PCM to a
// WAV file at the end of the run.
//
// Trace format (produced by Stage 3):
//
//     # picogus-tap v1
//     ns,port,value,kind
//     0,0x243,0x00,write8
//     50000,0x24b,0x4c,write8
//     ...
//
// `kind` is one of `write8 | write16 | read8 | read16`. Reads are
// logged for diagnostics but **ignored** by the replayer (the DOSBox-X
// guest already materialised whatever status value it cared about;
// we're driving the real firmware open-loop).
//
// CLI:
//
//     picogus_diff_rp2040
//         --flash <path>       (optional; Stage 4 accepts trace-only runs)
//         --bootrom <path>     (optional; default-searches
//                               roms/rp2040/bootrom-rp2040-b2.bin when
//                               --flash is supplied — required for real
//                               SDK firmware to boot past boot2)
//         --trace <path>       (required)
//         --duration <secs>    (optional; caps replay to N sim-seconds)
//
// Injection strategy — idealised ISA waveform
// -------------------------------------------
//
// The authoritative RP2040 GPIO mapping lives in
// [`mdpicoem_harness::picogus_pins`]. It is cross-checked against the
// PicoGUS v4.0.0 firmware (`github.com/polpo/picogus` tag `v4.0.0`) and
// used by both this replayer and the I2S capture module.
//
// Summary for the pins this file drives / reads:
//
//     GPIO  0..3     PSRAM (MISO/CS/SCK/MOSI — owned by on-chip merge)
//     GPIO  4        ISA IOW#           (harness drives)
//     GPIO  5        ISA IOR#           (harness drives)
//     GPIO  6..15    ISA AD0..AD9       (harness drives)
//     GPIO 16..18    I2S DOUT/BCLK/LRCLK (firmware drives — we observe)
//     GPIO 19        ISA DACK
//     GPIO 21        ISA IRQ
//     GPIO 26        ISA IOCHRDY
//     GPIO 27        ISA ADS
//     GPIO 28        UART TX
//
// The PIO `iow` program (isa_io.pio) samples the bus in two phases:
// phase A — waits for IOW falling edge, then reads 10 bits of address
// at AD0_PIN; phase B — flips the ADS sideset high and reads 8 bits
// of data at the same AD0_PIN. The hardware mux on the PCB multiplexes
// address / data onto the same 10 pins; we do not model the mux, we
// just drive the pins to match what the PIO program expects to sample
// each phase.
//
// Our synthetic write looks like this, per event:
//
//   1. Drive address bits A0..A9 on GPIO6..GPIO15, IOW# high (idle).
//      (ADS reflects address phase; the PIO program drives it, not us.)
//   2. Assert IOW# low (GPIO4 = 0). Hold `WRITE_ADDR_HOLD` cycles
//      (address phase — PIO reads 10 address bits).
//   3. Switch the AD0_PIN bus to data bits D0..D7. Hold
//      `WRITE_DATA_HOLD` cycles (data phase — PIO reads 8 data bits).
//   4. Deassert IOW# high. Idle for `WRITE_IDLE_CYCLES`.
//
// At sys_clk = 125 MHz, 37 cycles ≈ 300 ns (an ISA I/O write half-
// cycle). The PIO program runs at the same clock; its autopush-at-18-
// bits config will observe the asserted window easily.
//
// 16-bit writes (`write16`) emit the same address twice with the low
// byte first, high byte second — the real PC ISA bus splits a 16-bit
// I/O into two back-to-back 8-bit cycles at address N and N+1 on
// unaligned boards, or latches the low word on D0..D15 on the few 16-
// bit-capable slots; PicoGUS documents its GUS ports as 8- and 16-bit,
// and the GUS registers it decodes (0x24x) expect the low word at port
// P and the high byte at P+1. For Stage 4 idealisation we split every
// write16 into two write8s; the firmware's PIO program will see them
// as two distinct ISA cycles, which is what it expects anyway.
//
// Reads are ignored entirely (per HLD risks section — the replay is
// open-loop on writes; status reads diverge silently and would need
// a live bridge to resolve).

use std::path::{Path, PathBuf};
use std::time::Instant;

use mdpicoem_devices::{I2sCapture, Psram};
use mdpicoem_harness::picogus_pins::{
    ISA_AD0 as PIN_AD0, ISA_AD_COUNT as PIN_AD_COUNT, ISA_EXTERNAL_PIN_MASK, ISA_IOR as PIN_IOR,
    ISA_IOW as PIN_IOW, I2S_BCLK, I2S_DOUT, I2S_LRCLK,
};
use mdrp2040::{Config, Emulator, EmulatorBuilder};

/// Address width on the ISA bus for the GUS decode window (0x240..0x24F).
/// The PIO program reads 10 bits — we drive all 10, upper bits zero for
/// values inside the GUS range.
const ADDR_BITS: u32 = 10;

/// Data width for an 8-bit write.
const DATA_BITS: u32 = 8;

/// Cycles to hold address on the bus after IOW falls. Must be long
/// enough for the PIO to execute `jmp pin` + `in pins, 10 [3]`
/// (= 5 PIO cycles ≈ 10 sysclks at clkdiv=2). 12 gives margin.
const WRITE_ADDR_HOLD: u32 = 12;

/// Cycles to hold data on the bus after the address→data switch.
/// The PIO reads data via `in pins, 8` after 4 more PIO cycles of
/// `nop [3]` delay (~8 sysclks). 25 cycles gives margin for the PIO
/// to complete the read + autopush + OUT X + JMP.
const WRITE_DATA_HOLD: u32 = 25;

/// Cycles of idle between back-to-back writes.
const WRITE_IDLE_CYCLES: u32 = 12;

/// Initial `clk_sys` seed. Used only at emulator-construction time —
/// `Config.sys_clk_hz` seeds the clock tree and the `I2sCapture`
/// divisor at `CapturingSink::new`. Post-replay, the live `clk_sys` is
/// re-read from `emu.bus.sys_clk_hz()` and pushed into `I2sCapture` via
/// `set_sys_clk_hz` so sample-rate inference reflects any PLL
/// reprogramming firmware did during boot (e.g. PicoGUS 125→370 MHz).
/// The `replay()` loop does not use this constant — it polls
/// `IsaSink::sys_clk_hz()` per chunk instead, so ns↔cycle cadence
/// tracks firmware reprogramming live.
const DEFAULT_SYS_CLK_HZ: u32 = 125_000_000;

// ----------------------------------------------------------------------------
// Trace types
// ----------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceKind {
    Write8,
    Write16,
    Read8,
    Read16,
}

impl TraceKind {
    fn parse(s: &str) -> Result<Self, String> {
        match s {
            "write8" => Ok(Self::Write8),
            "write16" => Ok(Self::Write16),
            "read8" => Ok(Self::Read8),
            "read16" => Ok(Self::Read16),
            other => Err(format!("unknown kind '{other}'")),
        }
    }

    #[inline]
    fn is_write(self) -> bool {
        matches!(self, Self::Write8 | Self::Write16)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceEvent {
    /// Monotonic nanoseconds since capture start.
    pub ns: u64,
    /// 12-bit ISA-decoded port (typ. 0x240..0x24F for the GUS).
    pub port: u16,
    /// Data value — u8 for write8/read8, u16 for write16/read16.
    pub value: u16,
    pub kind: TraceKind,
}

/// Parse a full trace file into an owned vector of events.
///
/// Validates:
///   * The 2-line header (`# picogus-tap v1` + column header).
///   * Every row has 4 CSV fields.
///   * `ns` is u64 and strictly non-decreasing across consecutive rows.
///   * `port` parses as hex and fits in 12 bits (warns stderr if not).
///   * `value` parses as hex and fits in 8 or 16 bits per `kind`.
///   * `kind` is one of the four known strings.
///
/// Returns the first error encountered with a 1-based line number.
pub fn parse_trace(text: &str) -> Result<Vec<TraceEvent>, String> {
    let mut events = Vec::new();
    let mut last_ns: Option<u64> = None;
    let mut saw_magic = false;
    let mut saw_header = false;

    for (idx, raw) in text.lines().enumerate() {
        let line_no = idx + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }

        if !saw_magic {
            if line != "# picogus-tap v1" {
                return Err(format!(
                    "line {line_no}: expected magic '# picogus-tap v1', got '{line}'"
                ));
            }
            saw_magic = true;
            continue;
        }

        if !saw_header {
            if line != "ns,port,value,kind" {
                return Err(format!(
                    "line {line_no}: expected column header 'ns,port,value,kind', got '{line}'"
                ));
            }
            saw_header = true;
            continue;
        }

        // Skip in-file comments after the header.
        if line.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() != 4 {
            return Err(format!(
                "line {line_no}: expected 4 CSV fields, got {} ('{line}')",
                parts.len()
            ));
        }

        let ns: u64 = parts[0]
            .parse()
            .map_err(|e| format!("line {line_no}: invalid ns '{}': {e}", parts[0]))?;

        let port = parse_hex_u32(parts[1])
            .map_err(|e| format!("line {line_no}: invalid port '{}': {e}", parts[1]))?;
        if port > 0xFFF {
            eprintln!(
                "warning: line {line_no}: port {:#06x} outside 12-bit ISA range",
                port
            );
        }
        if port > 0xFFFF {
            return Err(format!(
                "line {line_no}: port {:#x} exceeds u16 range",
                port
            ));
        }

        let value = parse_hex_u32(parts[2])
            .map_err(|e| format!("line {line_no}: invalid value '{}': {e}", parts[2]))?;

        let kind = TraceKind::parse(parts[3])
            .map_err(|e| format!("line {line_no}: {e}"))?;

        let max = match kind {
            TraceKind::Write8 | TraceKind::Read8 => 0xFFu32,
            TraceKind::Write16 | TraceKind::Read16 => 0xFFFFu32,
        };
        if value > max {
            return Err(format!(
                "line {line_no}: value {:#x} too wide for kind {:?}",
                value, kind
            ));
        }

        if let Some(prev) = last_ns {
            if ns < prev {
                return Err(format!(
                    "line {line_no}: non-monotonic timestamp {ns} < {prev}"
                ));
            }
        }
        last_ns = Some(ns);

        events.push(TraceEvent {
            ns,
            port: port as u16,
            value: value as u16,
            kind,
        });
    }

    if !saw_magic || !saw_header {
        return Err("trace file is truncated (missing header)".into());
    }
    Ok(events)
}

fn parse_hex_u32(s: &str) -> Result<u32, String> {
    let stripped = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .unwrap_or(s);
    u32::from_str_radix(stripped, 16).map_err(|e| e.to_string())
}

/// Convert `ns` to cycles at `sys_clk_hz` using u128 math to avoid
/// overflow on multi-second traces.
#[inline]
pub fn ns_to_cycles(ns: u64, sys_clk_hz: u32) -> u64 {
    let cycles = (ns as u128) * (sys_clk_hz as u128) / 1_000_000_000u128;
    cycles as u64
}

/// Convert `cycles` to ns at `sys_clk_hz`. Used by the replay loop to
/// accumulate simulated wall-clock time across variable-rate sysclk
/// segments (e.g. a PicoGUS trace crossing the 125→370 MHz boundary).
#[inline]
pub fn cycles_to_ns(cycles: u64, sys_clk_hz: u32) -> u64 {
    if sys_clk_hz == 0 {
        return 0;
    }
    let ns = (cycles as u128) * 1_000_000_000u128 / (sys_clk_hz as u128);
    ns as u64
}

/// Map a trace-domain timestamp `ev_ns` to its sim-domain target,
/// given a pre-roll offset and a stretch factor. The stretch is applied
/// in trace-domain; the pre-roll is added in sim-domain (they do not
/// compose). With `pre_roll_ns = 0` and `trace_stretch = 1.0`, the
/// result equals `ev_ns` exactly — byte-identical baseline behaviour.
#[inline]
pub fn stretched_target_ns(ev_ns: u64, pre_roll_ns: u64, trace_stretch: f64) -> u64 {
    let stretched = if (trace_stretch - 1.0).abs() < f64::EPSILON {
        ev_ns
    } else {
        ((ev_ns as f64) * trace_stretch) as u64
    };
    pre_roll_ns.saturating_add(stretched)
}

/// Fast-forward `sink` by stepping one bounded chunk at a time until
/// the simulated wall-clock elapsed (carried in `sim_ns`) reaches
/// `target_ns`. Returns the number of stall events observed (0 or 1 —
/// at most one per call, since we bail on the first refusal to advance
/// like the pre-fix code did).
///
/// This is the PLL-aware replacement for the old
/// `target = ns_to_cycles(ev.ns, STATIC_HZ); while sink.cycles() < target`
/// loop. By re-querying `sink.sys_clk_hz()` per chunk and accumulating
/// ns from actual cycles stepped at that clock, the loop stays accurate
/// across firmware PLL reprogramming (e.g. PicoGUS 125→370 MHz).
///
/// `sim_ns` is an in/out running total of simulated wall-clock ns — the
/// caller carries it across events so elapsed time compounds correctly.
///
/// `warn_fn` is invoked at most once (by contract with the caller's
/// dedup) if the sink refuses to advance.
fn advance_to_sim_ns<S: IsaSink>(
    sink: &mut S,
    sim_ns: &mut u64,
    target_ns: u64,
    mut warn_fn: impl FnMut(u64),
) -> usize {
    let mut stalls = 0usize;
    while *sim_ns < target_ns {
        let remaining_ns = target_ns - *sim_ns;
        let hz = sink.sys_clk_hz();
        // Guard against a transient `sys_clk_hz == 0` — firmware could
        // park CLK_SYS briefly while reprogramming PLL. In that window
        // `cycles_needed` would pin to 1 and `cycles_to_ns(stepped, 0)`
        // returns 0 (matches the guard in `cycles_to_ns`), so `sim_ns`
        // would never advance — infinite loop until the sink stalls.
        // Treat zero as "no progress this chunk": step the sink a small
        // amount so firmware can reprogram the clock back above zero,
        // then bail so the caller re-polls on the next event.
        if hz == 0 {
            let before = sink.cycles();
            sink.step(1);
            if sink.cycles() == before {
                // Sink truly refused to advance — standard stall path.
                warn_fn(before);
                stalls += 1;
            }
            break;
        }
        // Cycles needed to cover `remaining_ns` at the *current* clock.
        // Round up so we never fall short and fire the event a fraction
        // too early. Cap per-call chunk at 64 cycles so a mid-chunk PLL
        // reprogram is observed promptly.
        let cycles_needed = (remaining_ns as u128 * hz as u128)
            .div_ceil(1_000_000_000u128)
            .max(1) as u64;
        let chunk = cycles_needed.clamp(1, 64) as u32;
        let before = sink.cycles();
        sink.step(chunk);
        let stepped = sink.cycles().wrapping_sub(before);
        if stepped == 0 {
            warn_fn(before);
            stalls += 1;
            break;
        }
        // Credit the ns actually earned at the clock live during this
        // chunk. If firmware reprogrammed the PLL mid-chunk, this will
        // be slightly off for this chunk only (bounded by 64 cycles).
        *sim_ns = sim_ns.saturating_add(cycles_to_ns(stepped, hz));
    }
    stalls
}

// ----------------------------------------------------------------------------
// GPIO injection
// ----------------------------------------------------------------------------

/// A recorded GPIO poke — used by the test harness to verify ordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Poke {
    pub cycle: u64,
    pub port: u16,
    pub value: u16,
    pub kind: TraceKind,
}

/// Abstract sink for synthetic ISA writes. `Emulator` gets one
/// implementation; the test harness substitutes a recording sink.
pub trait IsaSink {
    fn step(&mut self, cycles: u32);
    fn cycles(&self) -> u64;
    fn drive_pins(&mut self, iow_low: bool, ior_low: bool, ad_bus: u16);
    /// Current merged pad state (the value the firmware's PIO
    /// observes). Default is zero — sinks that don't model pads can
    /// leave this alone. The I2S capture wrapper ([`CapturingSink`])
    /// uses this to sample BCLK / LRCLK / DOUT each cycle.
    fn pad_state(&self) -> u32 {
        0
    }

    /// Current `clk_sys` frequency in Hz as observed by the sink *right
    /// now* (not a snapshot from harness init).
    ///
    /// Why this matters: PicoGUS firmware reprograms PLL_SYS from
    /// 125 MHz → 370 MHz a few ms into boot. The emulator's `ClockTree`
    /// tracks that reprogram correctly, but if the harness keeps using
    /// the 125 MHz constant for ns↔cycle conversions, every event
    /// after the switch fires at the wrong simulated wall-clock time
    /// (by a factor of 370/125 = 2.96×). The `replay()` fast-forward
    /// loop polls this per chunk so the ns→cycle cadence tracks the
    /// firmware's real sysclk across PLL changes.
    ///
    /// Default returns 125 MHz — matches the emulator's power-on
    /// default and keeps mock sinks (which can't observe a clock tree)
    /// behaving the way the tests expect.
    fn sys_clk_hz(&self) -> u32 {
        125_000_000
    }

    /// Number of words currently in PIO0 SM0's RX FIFO (the ISA-IOW
    /// capture FIFO). Default returns 0 — mock sinks that don't model
    /// PIO report empty, disabling backpressure.
    fn pio0_sm0_rx_fifo_level(&self) -> u8 {
        0
    }

    /// Cumulative count of PIO0 SM0 RX FIFO overflow drops. Default
    /// returns 0 — mock sinks don't track overflow. Real emulator
    /// returns the underlying SM's `rx_fifo_drops()`.
    fn pio0_sm0_rx_fifo_drops(&self) -> u64 {
        0
    }
}

impl IsaSink for Emulator {
    #[inline]
    fn step(&mut self, cycles: u32) {
        if cycles == 0 {
            return;
        }
        self.run(cycles as u64).expect("Serial run is infallible");
    }

    #[inline]
    fn cycles(&self) -> u64 {
        self.clock.cycles
    }

    #[inline]
    fn pad_state(&self) -> u32 {
        self.bus.gpio_in
    }

    /// Read the *live* clk_sys from the emulator's `ClockTree`. This
    /// reflects any PLL / mux reprogramming firmware has done since
    /// reset — e.g. PicoGUS's 125→370 MHz overclock at ~33M cycles in.
    #[inline]
    fn sys_clk_hz(&self) -> u32 {
        self.bus.sys_clk_hz()
    }

    #[inline]
    fn pio0_sm0_rx_fifo_level(&self) -> u8 {
        self.bus.pio[0].sm[0].rx_fifo_level()
    }

    #[inline]
    fn pio0_sm0_rx_fifo_drops(&self) -> u64 {
        self.bus.pio[0].sm[0].rx_fifo_drops()
    }

    /// Drive the ISA control + address/data bus by populating the Bus's
    /// `external_gpio_in_override` / `external_gpio_in_mask` fields. The
    /// override is applied on every `update_gpio` (per-system-cycle, see
    /// `Emulator::step`) so harness pokes survive PIO / PSRAM / SIO
    /// merges that previously clobbered direct `gpio_in` writes.
    ///
    /// The mask covers IOW#, IOR#, and AD0..AD9 only — PSRAM pins
    /// (GPIO0..3) remain owned by the on-chip merge so the off-chip
    /// PSRAM model still sees real CS/SCK/MOSI activity from PIO.
    fn drive_pins(&mut self, iow_low: bool, ior_low: bool, ad_bus: u16) {
        let iow_bit = if iow_low { 0u32 } else { 1u32 << PIN_IOW };
        let ior_bit = if ior_low { 0u32 } else { 1u32 << PIN_IOR };
        let ad_val = ((ad_bus as u32) & ((1u32 << PIN_AD_COUNT) - 1)) << PIN_AD0;

        self.bus.external_gpio_in_mask = ISA_EXTERNAL_PIN_MASK;
        self.bus.external_gpio_in_override = iow_bit | ior_bit | ad_val;
        // Reflect the new override immediately in `bus.gpio_in` so a
        // probe reading `gpio_in` between drive_pins() and the next
        // `step()` observes the asserted ISA waveform — matches the
        // pre-override behaviour where drive_pins wrote `gpio_in`
        // directly. The next `update_gpio` (inside `step()`) will
        // recompute from scratch and re-apply the override.
        let ext_mask = self.bus.external_gpio_in_mask;
        self.bus.gpio_in = (self.bus.gpio_in & !ext_mask)
            | (self.bus.external_gpio_in_override & ext_mask);
    }
}

/// Wraps an inner [`IsaSink`] (typically [`Emulator`]) with a per-step
/// tick into an [`I2sCapture`]. The inner sink is stepped in fine
/// increments and the capture is ticked with the true sysclk stamp
/// (`inner.cycles()`) after each, so BCLK / LRCLK edges are timestamped
/// in system-clock cycles rather than in tick-call counts — at 48 kHz
/// stereo with 32 BCLKs per frame the bit clock is ~1.5 MHz on a
/// 125 MHz sys_clk, so fine-grained sampling stays well above Nyquist.
pub struct CapturingSink<S: IsaSink> {
    inner: S,
    capture: I2sCapture,
}

impl<S: IsaSink> CapturingSink<S> {
    pub fn new(inner: S, sys_clk_hz: u32) -> Self {
        Self {
            inner,
            capture: I2sCapture::new(sys_clk_hz, I2S_BCLK, I2S_LRCLK, I2S_DOUT),
        }
    }

    pub fn capture(&self) -> &I2sCapture {
        &self.capture
    }

    /// Read-only access to the wrapped sink — used by the capture-
    /// coverage diagnostic to peek at the inner `Emulator`'s PIO0 SM0
    /// autopush mirror between `drive_write_cycle` calls.
    pub fn inner(&self) -> &S {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut S {
        &mut self.inner
    }

    pub fn into_parts(self) -> (S, I2sCapture) {
        (self.inner, self.capture)
    }
}

impl<S: IsaSink> IsaSink for CapturingSink<S> {
    /// Advance the inner sink until its cycle count reaches or passes
    /// `inner.cycles() + cycles`, ticking the I2S capture after each
    /// inner step with the *actual* sysclk stamp. With
    /// `Emulator::step_quantum(1)` each `inner.step(1)` advances by one
    /// instruction (1–4 sysclks on M0+), so the cycle delta between
    /// consecutive ticks matches the real sysclk delta — not the loop
    /// iteration count. The capture's LRCLK edge timestamps are
    /// therefore in system-clock units, and `inferred_sample_rate_hz`
    /// returns the true rate.
    fn step(&mut self, cycles: u32) {
        let target = self.inner.cycles().wrapping_add(cycles as u64);
        while self.inner.cycles() < target {
            let before = self.inner.cycles();
            self.inner.step(1);
            if self.inner.cycles() == before {
                // Inner sink stalled; stop the sub-loop so the outer
                // stall guard in `replay()` can observe it.
                break;
            }
            self.capture.tick(self.inner.pad_state(), self.inner.cycles());
        }
    }

    fn cycles(&self) -> u64 {
        self.inner.cycles()
    }

    fn drive_pins(&mut self, iow_low: bool, ior_low: bool, ad_bus: u16) {
        self.inner.drive_pins(iow_low, ior_low, ad_bus);
    }

    fn pad_state(&self) -> u32 {
        self.inner.pad_state()
    }

    #[inline]
    fn sys_clk_hz(&self) -> u32 {
        self.inner.sys_clk_hz()
    }

    #[inline]
    fn pio0_sm0_rx_fifo_level(&self) -> u8 {
        self.inner.pio0_sm0_rx_fifo_level()
    }

    #[inline]
    fn pio0_sm0_rx_fifo_drops(&self) -> u64 {
        self.inner.pio0_sm0_rx_fifo_drops()
    }
}

/// One synthetic write cycle: address phase, assert, data phase, deassert.
/// Blocking — returns after idling. Called once per write event.
///
/// Phase timings default to `WRITE_IDLE_CYCLES` / `WRITE_ADDR_HOLD` /
/// `WRITE_DATA_HOLD` but can be overridden at runtime via env vars
/// `PICOGUS_IDLE_CYCLES`, `PICOGUS_ADDR_HOLD`, `PICOGUS_DATA_HOLD` —
/// useful for probing whether the PIO ISA-bus capture is losing
/// events at the default timings.
pub fn drive_write_cycle<S: IsaSink>(sink: &mut S, port: u16, data: u16, wide: bool) {
    let addr_bits = port & ((1u16 << ADDR_BITS) - 1);

    let idle: u32 = std::env::var("PICOGUS_IDLE_CYCLES")
        .ok().and_then(|s| s.parse().ok()).unwrap_or(WRITE_IDLE_CYCLES);
    let addr_hold: u32 = std::env::var("PICOGUS_ADDR_HOLD")
        .ok().and_then(|s| s.parse().ok()).unwrap_or(WRITE_ADDR_HOLD);
    let data_hold: u32 = std::env::var("PICOGUS_DATA_HOLD")
        .ok().and_then(|s| s.parse().ok()).unwrap_or(WRITE_DATA_HOLD);

    // Backpressure: on real ISA hardware the PIO asserts IOCHRDY low when
    // its RX FIFO is full, stretching the bus cycle until firmware drains.
    // We don't model IOCHRDY feedback here, so instead we poll the FIFO
    // level before firing and step the sink until space is available. This
    // mirrors real-hardware backpressure with zero firmware-side changes.
    //
    // Tunables:
    //   PICOGUS_BACKPRESSURE_THRESHOLD  (default 2 of 4) — drain if level ≥ this
    //   PICOGUS_BACKPRESSURE_STEP       (default 256) — cycles per drain tick
    //   PICOGUS_BACKPRESSURE_MAX        (default 200_000) — give-up cap per event
    let bp_threshold: u8 = std::env::var("PICOGUS_BACKPRESSURE_THRESHOLD")
        .ok().and_then(|s| s.parse().ok()).unwrap_or(2);
    let bp_step: u32 = std::env::var("PICOGUS_BACKPRESSURE_STEP")
        .ok().and_then(|s| s.parse().ok()).unwrap_or(256);
    let bp_max: u64 = std::env::var("PICOGUS_BACKPRESSURE_MAX")
        .ok().and_then(|s| s.parse().ok()).unwrap_or(200_000);
    if bp_threshold > 0 {
        let start_cycles = sink.cycles();
        while sink.pio0_sm0_rx_fifo_level() >= bp_threshold {
            let before = sink.cycles();
            sink.step(bp_step);
            if sink.cycles() == before {
                break; // emulator stalled; don't spin forever
            }
            if sink.cycles().wrapping_sub(start_cycles) > bp_max {
                break;
            }
        }
    }

    // Phase 0: idle. Address on the bus, IOW high, IOR high.
    sink.drive_pins(false, false, addr_bits);
    sink.step(idle);

    // Phase 1: assert IOW low with address on the bus. The PIO fires
    // on the IOW falling edge, reads 10 address bits via `in pins, 10`,
    // then flips ADS high via sideset. On real hardware ADS triggers
    // a 74HC mux that switches the AD bus from address to data. We
    // model this by switching the bus to data after WRITE_ADDR_HOLD
    // cycles — enough time for the PIO to have captured the address.
    sink.drive_pins(true, false, addr_bits);
    sink.step(addr_hold);

    // Phase 2: data onto the bus, IOW still asserted. The PIO's
    // `in pins, 8` reads data from the same GPIO pins after a NOP
    // delay. This must happen before the PIO executes its data read.
    //
    // PICOGUS_WRITE16_SWAP: experimental byte-order swap for write16
    // events. When set, the HIGH byte goes to the LOW port first and
    // the LOW byte goes to the HIGH port second. Tests the theory that
    // the trace's `val` encoding differs from the x86 OUTW byte order.
    let data_lo = data & ((1u16 << DATA_BITS) - 1);
    sink.drive_pins(true, false, data_lo);
    sink.step(data_hold);

    // Phase 3: deassert IOW, release the bus. Idle.
    sink.drive_pins(false, false, 0);
    sink.step(idle);

    if wide {
        // Second 8-bit cycle for the high byte at port+1.
        let addr2 = addr_bits.wrapping_add(1) & ((1u16 << ADDR_BITS) - 1);
        let data_hi = (data >> 8) & ((1u16 << DATA_BITS) - 1);

        sink.drive_pins(false, false, addr2);
        sink.step(idle);
        sink.drive_pins(true, false, addr2);
        sink.step(addr_hold);
        sink.drive_pins(true, false, data_hi);
        sink.step(data_hold);
        sink.drive_pins(false, false, 0);
        sink.step(idle);
    }
}

/// Summary of a replay run.
#[derive(Debug, Default, Clone, Copy)]
pub struct ReplaySummary {
    pub events_total: usize,
    pub writes_fired: usize,
    pub reads_skipped: usize,
    pub duration_capped: bool,
    /// Cycle count at the moment the trace finished firing events
    /// (before any post-roll). Equal to `final_cycles` when post-roll
    /// is zero.
    pub final_cycles: u64,
    pub final_sim_ns: u64,
    /// Number of times the inner fast-forward loop broke because the
    /// sink refused to advance. The first stall also emits a one-shot
    /// `eprintln!` warning. Subsequent events fire at the stall cycle.
    pub stall_events: usize,
    /// Cycles spent in the post-roll drain after the last trace event
    /// (or after `--duration` cap was hit). Zero when `post_roll_ns`
    /// is `None` or `Some(0)`.
    pub post_roll_cycles: u64,
}

/// Replay a trace against any `IsaSink`. Returns a summary.
///
/// `duration_ns = Some(n)` stops replay when `event.ns > n`. `None`
/// runs to the end of the trace. The cap is compared against
/// *trace-domain* timestamps (i.e. pre-stretch), so it isn't shifted
/// by `pre_roll_ns` or scaled by `trace_stretch`.
///
/// `post_roll_ns = Some(n)` runs the sink for an additional `n` ns of
/// simulated time after the last fired event (or the duration cap),
/// without firing any further trace events. Lets firmware drain its
/// I2S / DMA pipelines after the last ISA write — Stage 5 needs a few
/// hundred ms of post-roll to capture the trailing audio buffer. `None`
/// or `Some(0)` skips the drain entirely.
///
/// `pre_roll_ns` steps the sink for this many sim-ns *before* firing
/// the first event — gives firmware time to finish boot and arm PIO
/// state machines. `0` skips the pre-roll.
///
/// `trace_stretch` multiplies every event's `ev.ns` target in the
/// sim-domain. `1.0` = unchanged; `>1.0` opens inter-event gaps;
/// `<1.0` compresses. Must be finite and `> 0`.
pub fn replay<S: IsaSink>(
    sink: &mut S,
    events: &[TraceEvent],
    duration_ns: Option<u64>,
    post_roll_ns: Option<u64>,
    pre_roll_ns: u64,
    trace_stretch: f64,
) -> ReplaySummary {
    let mut summary = ReplaySummary {
        events_total: events.len(),
        ..Default::default()
    };

    // Running simulated wall-clock in ns. Accumulated from cycles
    // stepped, using whichever `sys_clk_hz` was live during each chunk
    // — so the timeline stays aligned with trace timestamps even if
    // firmware reprograms PLL mid-run (e.g. PicoGUS 125→370 MHz).
    let mut sim_ns: u64 = 0;
    let mut stall_warned = summary.stall_events > 0;

    if pre_roll_ns > 0 {
        let stalls = advance_to_sim_ns(sink, &mut sim_ns, pre_roll_ns, |cycle| {
            if !stall_warned {
                eprintln!(
                    "warning: emulator stalled at cycle {} during pre-roll",
                    cycle
                );
                stall_warned = true;
            }
        });
        summary.stall_events += stalls;
    }

    // PICOGUS_R44_SWAP: targeted byte-swap for write16 events that
    // update GUS register 0x44 (DRAM address MSW). When enabled, we
    // track the most recently selected register (via write8 to port
    // 0x343) and if a write16 to 0x344 lands while r0x44 is selected,
    // we deliver `val` with its two bytes swapped — testing the
    // theory that the trace's 16-bit val encoding doesn't match the
    // x86 OUTW byte order picogus expects.
    let r44_swap_enabled = std::env::var("PICOGUS_R44_SWAP").is_ok();
    if r44_swap_enabled {
        eprintln!("[diag] PICOGUS_R44_SWAP enabled — swapping write16 bytes when r0x44 selected");
    }
    let mut current_reg_select: u16 = 0;

    for ev in events {
        if let Some(limit) = duration_ns {
            if ev.ns > limit {
                summary.duration_capped = true;
                break;
            }
        }

        let target_ns = stretched_target_ns(ev.ns, pre_roll_ns, trace_stretch);
        let stalls = advance_to_sim_ns(sink, &mut sim_ns, target_ns, |cycle| {
            if !stall_warned {
                eprintln!(
                    "warning: emulator stalled at cycle {} \
                     — subsequent events fire at the stall cycle",
                    cycle
                );
                stall_warned = true;
            }
        });
        summary.stall_events += stalls;

        if ev.kind.is_write() {
            let wide = matches!(ev.kind, TraceKind::Write16);

            // Track gRegSelect via writes to port 0x343.
            if ev.port == 0x343 && !wide {
                current_reg_select = ev.value;
            }

            // Targeted r0x44 swap.
            let data_to_send = if r44_swap_enabled
                && wide
                && ev.port == 0x344
                && current_reg_select == 0x44
            {
                ((ev.value >> 8) & 0xFF) | ((ev.value & 0xFF) << 8)
            } else {
                ev.value
            };

            drive_write_cycle(sink, ev.port, data_to_send, wide);
            summary.writes_fired += 1;
        } else {
            summary.reads_skipped += 1;
        }

        summary.final_sim_ns = ev.ns;
    }

    summary.final_cycles = sink.cycles();

    // Post-roll drain. Step the sink for an additional `post_roll_ns`
    // of *simulated* wall-clock time WITHOUT firing further events, so
    // firmware (e.g. an I2S DMA chain) can flush its trailing buffer.
    if let Some(post_ns) = post_roll_ns {
        if post_ns > 0 {
            let post_start_cycles = sink.cycles();
            let post_target_ns = sim_ns.saturating_add(post_ns);
            let stalls = advance_to_sim_ns(sink, &mut sim_ns, post_target_ns, |cycle| {
                if !stall_warned {
                    eprintln!(
                        "warning: emulator stalled at cycle {} during post-roll",
                        cycle
                    );
                    stall_warned = true;
                }
            });
            summary.stall_events += stalls;
            summary.post_roll_cycles = sink.cycles().wrapping_sub(post_start_cycles);
        }
    }

    summary
}

// ----------------------------------------------------------------------------
// Capture-coverage diagnostic (HLD "PicoGUS Capture Coverage Diagnostic"
// Rev. 1 §3). Per-class `(fired, captured, misattributed)` accounting
// of ISA bus writes against PIO0 SM0 autopushes, plus deciles of the
// fired/captured ratio to expose clustering.
// ----------------------------------------------------------------------------

/// Per-`(port, value)` bucket key. Matches the taxonomy in HLD §3.1
/// and the example output in §5.
#[derive(Copy, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
enum ClassKey {
    /// Port 0x343 exact value (`0x43`, `0x44`, `0x4C`, or other).
    P343(u8),
    /// First half of a write16 at 0x344 (the low-byte sub-event).
    /// Write8 writes to 0x344 also land here.
    P344Lo,
    /// Second half of a write16 at 0x344 (the high-byte sub-event;
    /// fires on the bus at port 0x345 but classes by its origin).
    P344Hi,
    /// Port 0x345 exact value (write8 at 0x345 — distinct from the
    /// write16-origin-at-0x344 second half).
    P345(u8),
    /// Port 0x347 — collapsed into a single `data` bucket regardless
    /// of value, since these are bulk DRAM bytes.
    P347,
    /// Any other port. One row per port, collapsed across values
    /// (setup ports like 0x240, 0x24b, 0x342 tend to appear in small
    /// counts — the bulk of traffic is on 0x343..0x347).
    OtherPort(u16),
}

impl ClassKey {
    /// Render the class as a human-readable label matching HLD §5.
    fn label(self) -> String {
        match self {
            ClassKey::P343(v) => match v {
                0x43 => "(0x343 <- 0x43)".to_string(),
                0x44 => "(0x343 <- 0x44)".to_string(),
                0x4C => "(0x343 <- 0x4C)".to_string(),
                other => format!("(0x343 <- 0x{:02x})", other),
            },
            ClassKey::P344Lo => "(0x344 addr_lo)".to_string(),
            ClassKey::P344Hi => "(0x344 addr_hi)".to_string(),
            ClassKey::P345(v) => format!("(0x345 <- 0x{:02x})", v),
            ClassKey::P347 => "(0x347 <- data)".to_string(),
            ClassKey::OtherPort(p) => format!("(0x{:03x} <- *)", p),
        }
    }
}

/// Classify one sub-event (the PIO-level capture, post write16 split)
/// into a `ClassKey`. `origin_port`/`origin_value`/`is_write16` refer
/// to the trace event; `sub_idx` is 0 for the low-byte sub-event and
/// 1 for the high-byte of a write16.
fn classify_sub_event(
    origin_port: u16,
    origin_value: u16,
    is_write16: bool,
    sub_idx: usize,
) -> ClassKey {
    // Write16 at 0x344 is special-cased per HLD §3.1 — both sub-events
    // attribute to the origin port with `addr_lo`/`addr_hi` sub-keys.
    if is_write16 && origin_port == 0x344 {
        return if sub_idx == 0 {
            ClassKey::P344Lo
        } else {
            ClassKey::P344Hi
        };
    }
    // Otherwise use the sub-event's own fired port and data.
    let fired_port = origin_port.wrapping_add(sub_idx as u16);
    let fired_data = if sub_idx == 0 {
        (origin_value & 0xFF) as u8
    } else {
        ((origin_value >> 8) & 0xFF) as u8
    };
    match fired_port {
        0x343 => ClassKey::P343(fired_data),
        0x344 => ClassKey::P344Lo,
        0x345 => ClassKey::P345(fired_data),
        0x347 => ClassKey::P347,
        other => ClassKey::OtherPort(other),
    }
}

/// Per-class counters.
#[derive(Default, Copy, Clone)]
struct CoverageClass {
    fired: u64,
    captured: u64,
    misattributed: u64,
}

/// Full capture-coverage result.
#[derive(Default)]
struct CaptureCoverage {
    classes: std::collections::BTreeMap<ClassKey, CoverageClass>,
    /// Per-trace-progress decile totals (10 buckets of fired / captured).
    decile_fired: [u64; 10],
    decile_captured: [u64; 10],
    /// Catch-up deltas > 2 whose surplus pushes we couldn't credit to
    /// a specific event (surfaces as a residual bucket).
    catch_up_unattributed: u64,
    /// Post-roll pushes that landed after the last fired sub-event.
    /// Credited to the last sub-event's class if the decoded `(addr,data)`
    /// matches it; otherwise incremented here.
    post_roll_orphans: u64,
    /// Total sub-events fired — exposed for the SUMMARY check
    /// (`fired_sum == summary.writes_fired × (1 + write16_share)`).
    fired_sub_events: u64,
}

impl CaptureCoverage {
    /// Core delta-classifier state transition. Extracted as a pure
    /// method so it can be unit-tested without a live emulator.
    ///
    /// Preconditions:
    /// - Caller has already bumped `classes[class].fired` and
    ///   `decile_fired[decile]`.
    /// - `delta` is the post-cycle `autopush_count` delta (0, 1, or
    ///   more) and `decode_matches_current` is true iff the decoded
    ///   last-pushed `(addr,data)` matches the current sub-event's
    ///   fired `(addr,data)`.
    /// - `pending` carries (class, decile) of the previous sub-event
    ///   whose push may still be in flight.
    ///
    /// Invariant (HLD §7(4)): every physical push must bump exactly
    /// one `captured` or `misattributed` bucket. Surplus pushes in a
    /// `delta > 1` catch-up flow into `catch_up_unattributed`.
    fn classify_sub_event(
        &mut self,
        delta: u64,
        decode_matches_current: bool,
        class: ClassKey,
        decile: usize,
        pending: &mut Option<(ClassKey, usize)>,
    ) {
        match delta {
            0 => {
                // No push observed. Our push may still be in flight —
                // become (or replace) the pending event so the next
                // cycle can reconcile.
                *pending = Some((class, decile));
            }
            1 => {
                if decode_matches_current {
                    // Captured cleanly — attributed to current event.
                    let e = self.classes.entry(class).or_default();
                    e.captured += 1;
                    self.decile_captured[decile] += 1;
                    *pending = None;
                } else if let Some((prev_class, prev_decile)) = pending.take() {
                    // Push landed but decode is for the previous event
                    // — its push arrived late. Credit prev as
                    // misattributed (HLD §3.3 "captured-misattributed")
                    // and record under the PREVIOUS event's decile so
                    // the clustering table counts this drift-captured
                    // push (SHOULD-FIX 2). Current event's own push is
                    // still missing, so current becomes the new pending.
                    let e = self.classes.entry(prev_class).or_default();
                    e.misattributed += 1;
                    self.decile_captured[prev_decile] += 1;
                    *pending = Some((class, decile));
                } else {
                    // Stray push with no pending owner. Credit to
                    // current as misattributed so the invariant still
                    // balances (exactly one bucket bump per push).
                    // Current event's own push is still unaccounted —
                    // keep it pending for the next cycle.
                    let e = self.classes.entry(class).or_default();
                    e.misattributed += 1;
                    self.decile_captured[decile] += 1;
                    *pending = Some((class, decile));
                }
            }
            more => {
                // Catch-up: multiple pushes landed in one drive window.
                // Decode identifies only the last; credit the current
                // event if its decode matches, plus one preceding
                // pending event, and surface any residual as
                // `catch_up_unattributed`. Each credit bumps both the
                // class counter and the decile bucket for the class
                // being credited — NOT for the current decile
                // unconditionally (SHOULD-FIX 2).
                let mut credits: u64 = 0;
                if decode_matches_current {
                    let e = self.classes.entry(class).or_default();
                    e.captured += 1;
                    self.decile_captured[decile] += 1;
                    credits += 1;
                }
                if let Some((prev_class, prev_decile)) = pending.take() {
                    let e = self.classes.entry(prev_class).or_default();
                    e.captured += 1;
                    self.decile_captured[prev_decile] += 1;
                    credits += 1;
                }
                if more > credits {
                    self.catch_up_unattributed += more - credits;
                }
                *pending = None;
            }
        }
    }
}

/// One decoded autopush word — the PIO captured `(addr, data)` pair.
#[inline]
fn decode_push(word: u32) -> (u16, u8) {
    // IN_SHIFTDIR = left on PIO0 SM0 (SHIFTCTRL bit 18 = 0 — verified
    // on both emulator and silicon, 0x012b0000). Shift-in of 10 address
    // bits then 8 data bits leaves `(addr << 8) | data` in the ISR:
    //   bits  7..0 = data
    //   bits 17..8 = addr
    let data = (word & 0xFF) as u8;
    let addr = ((word >> 8) & 0x3FF) as u16;
    (addr, data)
}

/// Replay a trace while capturing per-class PIO0-SM0 autopush coverage.
///
/// Mirrors [`replay`] but specialised for `CapturingSink<Emulator>` so
/// the inner emulator's `pio[0].sm[0].autopush_count` /
/// `.last_autopush_word` can be read between `drive_write_cycle`
/// calls. Classification rules follow HLD §3.3.
fn replay_with_coverage(
    sink: &mut CapturingSink<Emulator>,
    events: &[TraceEvent],
    duration_ns: Option<u64>,
    post_roll_ns: Option<u64>,
    pre_roll_ns: u64,
    trace_stretch: f64,
    uart_drain: &mut UartDrain,
    replay_pokes: &[(u32, u8)],
) -> (ReplaySummary, CaptureCoverage) {
    let apply_replay_pokes = |emu: &mut mdrp2040::Emulator, pokes: &[(u32, u8)]| {
        for &(a, v) in pokes {
            let word_addr = a & !3;
            let shift = (a & 3) * 8;
            let w = emu.peek(word_addr);
            let w_new = (w & !(0xff << shift)) | ((v as u32) << shift);
            emu.poke(word_addr, w_new);
        }
    };
    let mut summary = ReplaySummary {
        events_total: events.len(),
        ..Default::default()
    };
    let mut cov = CaptureCoverage::default();

    // Running simulated wall-clock in ns — see `replay()` for rationale.
    let mut sim_ns: u64 = 0;
    let mut stall_warned = false;

    if pre_roll_ns > 0 {
        let stalls = advance_to_sim_ns(sink, &mut sim_ns, pre_roll_ns, |cycle| {
            if !stall_warned {
                eprintln!(
                    "warning: emulator stalled at cycle {} during pre-roll",
                    cycle
                );
                stall_warned = true;
            }
        });
        summary.stall_events += stalls;
        uart_drain.drain_emu(sink.inner_mut());
    }

    // Running state for the delta-classifier. `pending` carries the
    // class AND decile of the previous sub-event whose push may still
    // be in flight (delta=0 / mismatched-decode cases). Reconciled on
    // the next sub-event. The decile half (SHOULD-FIX 2) ensures that
    // when drift-attribution credits the previous event, the
    // clustering decile table is bumped against the previous event's
    // decile, not the current one.
    let mut pending: Option<(ClassKey, usize)> = None;
    // Track the last fired sub-event class — used by the post-roll
    // reconciliation block to attribute trailing pushes.
    let mut last_fired_class: Option<ClassKey> = None;
    let mut last_fired_port_data: Option<(u16, u8)> = None;

    // Count the sub-events we will actually fire (after duration cap)
    // so decile bucket math works without two passes.
    let mut total_to_fire: u64 = 0;
    for ev in events {
        if let Some(limit) = duration_ns {
            if ev.ns > limit {
                break;
            }
        }
        if !ev.kind.is_write() {
            continue;
        }
        total_to_fire += if matches!(ev.kind, TraceKind::Write16) { 2 } else { 1 };
    }

    let mut fired_so_far: u64 = 0;

    // PICOGUS_R44_SWAP: targeted byte-swap for write16 events updating
    // GUS r0x44. See the standalone `replay` for rationale.
    let r44_swap_enabled = std::env::var("PICOGUS_R44_SWAP").is_ok();
    if r44_swap_enabled {
        eprintln!("[diag] PICOGUS_R44_SWAP enabled (replay_with_coverage)");
    }
    let mut current_reg_select: u16 = 0;
    let mut r44_swap_hits: u64 = 0;

    for ev in events {
        if let Some(limit) = duration_ns {
            if ev.ns > limit {
                summary.duration_capped = true;
                break;
            }
        }

        let target_ns = stretched_target_ns(ev.ns, pre_roll_ns, trace_stretch);
        let stalls = advance_to_sim_ns(sink, &mut sim_ns, target_ns, |cycle| {
            if !stall_warned {
                eprintln!(
                    "warning: emulator stalled at cycle {} \
                     — subsequent events fire at the stall cycle",
                    cycle
                );
                stall_warned = true;
            }
        });
        summary.stall_events += stalls;
        uart_drain.drain_emu(sink.inner_mut());
        if !replay_pokes.is_empty() {
            apply_replay_pokes(sink.inner_mut(), replay_pokes);
        }

        if !ev.kind.is_write() {
            summary.reads_skipped += 1;
            summary.final_sim_ns = ev.ns;
            continue;
        }

        // Track current register select via write8 to port 0x343.
        if !matches!(ev.kind, TraceKind::Write16) && ev.port == 0x343 {
            current_reg_select = ev.value;
        }

        let is_wide = matches!(ev.kind, TraceKind::Write16);
        let sub_count = if is_wide { 2usize } else { 1 };

        // Apply targeted r0x44 byte swap.
        let effective_value = if r44_swap_enabled
            && is_wide
            && ev.port == 0x344
            && current_reg_select == 0x44
        {
            r44_swap_hits += 1;
            ((ev.value >> 8) & 0xFF) | ((ev.value & 0xFF) << 8)
        } else {
            ev.value
        };

        for sub_idx in 0..sub_count {
            let fired_port = ev.port.wrapping_add(sub_idx as u16);
            let fired_data = if sub_idx == 0 {
                (effective_value & 0xFF) as u8
            } else {
                ((effective_value >> 8) & 0xFF) as u8
            };
            let class = classify_sub_event(ev.port, ev.value, is_wide, sub_idx);

            let before_cnt = sink.inner().bus.pio[0].sm[0].autopush_count;
            // Drive one 8-bit cycle. We deliberately call with wide=false
            // and do the split manually so per-sub-event delta checks
            // match the PIO captures one-to-one.
            drive_write_cycle(sink, fired_port, fired_data as u16, false);
            let after_cnt = sink.inner().bus.pio[0].sm[0].autopush_count;
            let pushed_word = sink.inner().bus.pio[0].sm[0].last_autopush_word;
            let delta = after_cnt.wrapping_sub(before_cnt);

            // Decile bucket for this sub-event (based on fired-order
            // progress through the total sub-events we'll fire).
            let decile = if total_to_fire == 0 {
                0
            } else {
                ((fired_so_far * 10) / total_to_fire).min(9) as usize
            };

            // Always bump `fired` for this class / decile.
            {
                let e = cov.classes.entry(class).or_default();
                e.fired += 1;
            }
            cov.decile_fired[decile] += 1;
            cov.fired_sub_events += 1;

            let (decoded_addr, decoded_data) = decode_push(pushed_word);
            let decode_matches_current =
                decoded_addr == fired_port & 0x3FF && decoded_data == fired_data;
            cov.classify_sub_event(
                delta,
                decode_matches_current,
                class,
                decile,
                &mut pending,
            );

            last_fired_class = Some(class);
            last_fired_port_data = Some((fired_port & 0x3FF, fired_data));
            fired_so_far += 1;
            summary.writes_fired += if sub_idx == 0 { 1 } else { 0 };
        }

        summary.final_sim_ns = ev.ns;
    }

    summary.final_cycles = sink.cycles();

    // Post-roll drain — mirror `replay()` but also reconcile any final
    // trailing push by decode match.
    let autopush_before_post_roll = sink.inner().bus.pio[0].sm[0].autopush_count;
    if let Some(post_ns) = post_roll_ns {
        if post_ns > 0 {
            let post_start_cycles = sink.cycles();
            let post_target_ns = sim_ns.saturating_add(post_ns);
            let stalls = advance_to_sim_ns(sink, &mut sim_ns, post_target_ns, |cycle| {
                if !stall_warned {
                    eprintln!(
                        "warning: emulator stalled at cycle {} during post-roll",
                        cycle
                    );
                    stall_warned = true;
                }
            });
            summary.stall_events += stalls;
            summary.post_roll_cycles = sink.cycles().wrapping_sub(post_start_cycles);
            uart_drain.drain_emu(sink.inner_mut());
        }
    }
    let autopush_after_post_roll = sink.inner().bus.pio[0].sm[0].autopush_count;
    let post_roll_delta = autopush_after_post_roll.wrapping_sub(autopush_before_post_roll);
    if post_roll_delta > 0 {
        let last_word = sink.inner().bus.pio[0].sm[0].last_autopush_word;
        let (addr, data) = decode_push(last_word);
        match (last_fired_class, last_fired_port_data) {
            (Some(class), Some((last_port, last_data)))
                if addr == last_port && data == last_data =>
            {
                // The trailing push decodes to the last fired sub-event
                // — credit its class once.
                let e = cov.classes.entry(class).or_default();
                e.captured += 1;
                // Residual pushes in the post-roll delta that don't
                // decode to the last event stay as orphans.
                if post_roll_delta > 1 {
                    cov.post_roll_orphans += post_roll_delta - 1;
                }
            }
            _ => {
                cov.post_roll_orphans += post_roll_delta;
            }
        }
    }

    if r44_swap_enabled {
        eprintln!("[diag] r0x44 swap hits: {}", r44_swap_hits);
    }

    (summary, cov)
}

// ----------------------------------------------------------------------------
// CLI
// ----------------------------------------------------------------------------

struct Args {
    flash: Option<PathBuf>,
    bootrom: Option<PathBuf>,
    trace: PathBuf,
    duration_secs: Option<f64>,
    post_roll_secs: f64,
    pre_roll_secs: f64,
    trace_stretch: f64,
    out: Option<PathBuf>,
    firmware_mode: Option<u32>,
    step_quantum: u32,
}

/// Default `step_quantum` used when constructing the emulator.
///
/// 4 was selected on 2026-04-26 after a quantum sweep against the
/// Monkey Island AdLib trace. Per-quantum results (3 trials each,
/// 2.5 s sim, mode 5):
///
/// | quantum |  wall median | speedup vs q=1 | cake-gate              |
/// |--------:|-------------:|---------------:|------------------------|
/// |       1 |       93.0 s |       baseline | PASS                   |
/// |       4 |       80.9 s |        −13.0 % | PASS — top-5 peaks ≡ 1 |
/// |      16 |       79.6 s |        −14.4 % | PASS but FFT detunes   |
/// |      64 |       67.0 s |        −28.0 % | FAIL — silent          |
///
/// q=4 is spectrally indistinguishable from q=1 (top-5 FFT peaks
/// match exactly, peak/RMS within 0.1 dB) while the ISA-overshoot
/// bound (≤4 cycles) stays well below the 37-cycle ISA write window.
/// `--step-quantum 1` remains available for ISA-edge debugging.
const DEFAULT_STEP_QUANTUM: u32 = 4;

/// Default location searched for a real RP2040 bootrom when `--flash` is
/// supplied and `--bootrom` is absent. Provenance in
/// `roms/rp2040/README.md`.
pub const DEFAULT_BOOTROM_PATH: &str = "roms/rp2040/bootrom-rp2040-b2.bin";

/// Default post-roll duration in seconds. 500 ms gives enough simulated
/// time for firmware's I2S DMA chain to flush its trailing audio buffer
/// after the last ISA write — Stage 5 (WAV capture) will rely on this.
const DEFAULT_POST_ROLL_SECS: f64 = 0.5;

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut flash = None;
    let mut bootrom = None;
    let mut trace = None;
    let mut duration_secs = None;
    let mut post_roll_secs = DEFAULT_POST_ROLL_SECS;
    let mut pre_roll_secs: f64 = 0.0;
    let mut trace_stretch: f64 = 1.0;
    let mut out = None;
    let mut firmware_mode: Option<u32> = None;
    let mut step_quantum: u32 = DEFAULT_STEP_QUANTUM;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--flash" => {
                i += 1;
                if i >= args.len() {
                    return Err("--flash requires a path".into());
                }
                flash = Some(PathBuf::from(&args[i]));
            }
            "--bootrom" => {
                i += 1;
                if i >= args.len() {
                    return Err("--bootrom requires a path".into());
                }
                bootrom = Some(PathBuf::from(&args[i]));
            }
            "--trace" => {
                i += 1;
                if i >= args.len() {
                    return Err("--trace requires a path".into());
                }
                trace = Some(PathBuf::from(&args[i]));
            }
            "--duration" => {
                i += 1;
                if i >= args.len() {
                    return Err("--duration requires seconds".into());
                }
                duration_secs = Some(
                    args[i]
                        .parse::<f64>()
                        .map_err(|e| format!("invalid --duration '{}': {e}", args[i]))?,
                );
            }
            "--post-roll" => {
                i += 1;
                if i >= args.len() {
                    return Err("--post-roll requires seconds".into());
                }
                post_roll_secs = args[i]
                    .parse::<f64>()
                    .map_err(|e| format!("invalid --post-roll '{}': {e}", args[i]))?;
                if post_roll_secs < 0.0 {
                    return Err("--post-roll must be >= 0".into());
                }
            }
            "--pre-roll" => {
                i += 1;
                if i >= args.len() {
                    return Err("--pre-roll requires seconds".into());
                }
                pre_roll_secs = args[i]
                    .parse::<f64>()
                    .map_err(|e| format!("invalid --pre-roll '{}': {e}", args[i]))?;
                if !pre_roll_secs.is_finite() || pre_roll_secs < 0.0 {
                    return Err("--pre-roll must be a finite value >= 0".into());
                }
            }
            "--trace-stretch" => {
                i += 1;
                if i >= args.len() {
                    return Err("--trace-stretch requires a factor".into());
                }
                trace_stretch = args[i]
                    .parse::<f64>()
                    .map_err(|e| format!("invalid --trace-stretch '{}': {e}", args[i]))?;
                if !trace_stretch.is_finite() || trace_stretch <= 0.0 {
                    return Err("--trace-stretch must be a finite value > 0".into());
                }
            }
            "--out" => {
                i += 1;
                if i >= args.len() {
                    return Err("--out requires a path".into());
                }
                out = Some(PathBuf::from(&args[i]));
            }
            "--firmware-mode" => {
                i += 1;
                if i >= args.len() {
                    return Err("--firmware-mode requires a slot index".into());
                }
                let raw = &args[i];
                let parsed = if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
                    u32::from_str_radix(hex, 16)
                } else {
                    raw.parse::<u32>()
                };
                firmware_mode = Some(
                    parsed.map_err(|e| format!("invalid --firmware-mode '{raw}': {e}"))?,
                );
            }
            "--step-quantum" => {
                i += 1;
                if i >= args.len() {
                    return Err("--step-quantum requires a positive integer".into());
                }
                let n: u32 = args[i]
                    .parse()
                    .map_err(|e| format!("invalid --step-quantum '{}': {e}", args[i]))?;
                if n == 0 {
                    return Err("--step-quantum must be >= 1".into());
                }
                step_quantum = n;
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => {
                return Err(format!("unknown argument '{other}'"));
            }
        }
        i += 1;
    }

    let trace = trace.ok_or("--trace is required")?;
    Ok(Args {
        flash,
        bootrom,
        trace,
        duration_secs,
        post_roll_secs,
        pre_roll_secs,
        trace_stretch,
        out,
        firmware_mode,
        step_quantum,
    })
}

/// Resolve the bootrom path given an explicit `--bootrom` flag and the
/// presence (or absence) of `--flash`.
///
/// Rules:
/// * If `--bootrom` is supplied, always honour it (must exist).
/// * Else if `--flash` is absent, return `None` (no firmware to boot).
/// * Else default-search [`DEFAULT_BOOTROM_PATH`]. If the file is
///   present, use it. If not, return `None` — the caller emits a hint.
pub fn resolve_bootrom_path(
    explicit: Option<&Path>,
    flash_present: bool,
) -> Option<PathBuf> {
    if let Some(p) = explicit {
        return Some(p.to_path_buf());
    }
    if !flash_present {
        return None;
    }
    let default = PathBuf::from(DEFAULT_BOOTROM_PATH);
    if default.is_file() { Some(default) } else { None }
}

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         picogus_diff_rp2040 --trace <path>\n                      \
         [--flash <path>] [--bootrom <path>] [--duration <secs>]\n                      \
         [--post-roll <secs>] [--out <path>]\n\
         \n\
         --flash      Optional 2 MB XIP flash image (.bin). Without it the\n              \
                      emulator runs with empty flash; the replayer still\n              \
                      pokes GPIO inputs — useful for harness tests.\n\
         --bootrom    Optional 16 KB RP2040 bootrom image. When --flash is\n              \
                      supplied and this flag is absent, we default-search\n              \
                      `roms/rp2040/bootrom-rp2040-b2.bin`. Required for\n              \
                      real SDK firmware to boot past boot2.\n\
         --trace      Required. CSV file in picogus-tap v1 format.\n\
         --duration   Optional. Stops replay once trace timestamp exceeds\n              \
                      this many simulated seconds.\n\
         --post-roll  Optional (default 0.5 s). After the last trace event\n              \
                      (or the duration cap), step the emulator for this many\n              \
                      additional simulated seconds without firing events —\n              \
                      lets firmware drain trailing I2S / DMA buffers.\n\
         --pre-roll   Optional (default 0 s). Before firing the first trace\n              \
                      event, step the emulator for this many simulated\n              \
                      seconds so firmware can finish boot / arm PIO state\n              \
                      machines. Does NOT shift the duration cap (which is\n              \
                      compared against trace-domain timestamps).\n\
         --trace-stretch\n              \
                      Optional (default 1.0). Multiply every trace event's\n              \
                      timestamp by this factor when advancing sim-time —\n              \
                      stretches inter-event gaps (>1.0) or compresses them\n              \
                      (<1.0). Useful when residual drops suggest PIO can't\n              \
                      keep up with back-to-back ISA writes. Applied in\n              \
                      trace-domain and summed with pre-roll in sim-domain.\n\
         --out        Optional. Path for the captured I2S WAV. Default:\n              \
                      crates/mdpicoem-harness/oracles/picogus_<trace_stem>.wav.\n\
         --firmware-mode N\n              \
                      Optional. After emu.reset(), write N (u32, dec or 0x..)\n              \
                      to watchdog SCRATCH3 (0x4005_8018) so the PicoGUS multifw\n              \
                      bootloader picks slot N instead of falling through to its\n              \
                      flash-settings default.\n\
         --step-quantum N\n              \
                      Optional (default 4). Master-clock cycles per emulator\n              \
                      step() call. q=4 was validated against the Monkey Island\n              \
                      AdLib trace (top-5 FFT peaks match q=1 exactly, 13% wall\n              \
                      reduction). q=1 bounds ISA-waveform overshoot to a single\n              \
                      cycle for ISA-edge debugging. q=16 starts spectral detune;\n              \
                      q=64 silences the WAV — verify with the audio cake-gate."
    );
}

/// Line-buffering drain for the emulator's UART0 TX FIFO. PicoGUS uses
/// UART0 on GPIO 28 (230400 baud) for `stdio_init_all` / `puts`. Every
/// byte the firmware writes to `UARTDR` is captured in
/// `Emulator::drain_uart0_tx_log`; we accumulate bytes here until a
/// newline lands, then flush a full line to stderr prefixed with `[uart]`
/// so it's unambiguous against the harness's own eprintln output.
pub struct UartDrain {
    line: Vec<u8>,
    total_bytes: u64,
}

impl UartDrain {
    pub fn new() -> Self {
        Self { line: Vec::with_capacity(128), total_bytes: 0 }
    }

    pub fn drain_emu(&mut self, emu: &mut mdrp2040::Emulator) {
        let bytes = emu.drain_uart0_tx_log();
        if bytes.is_empty() {
            return;
        }
        self.total_bytes += bytes.len() as u64;
        for b in bytes {
            if b == b'\n' {
                self.flush_line();
            } else if b == b'\r' {
                // stdio's crlf translation often emits CR+LF; swallow the CR.
                continue;
            } else {
                self.line.push(b);
            }
        }
    }

    pub fn flush_line(&mut self) {
        if self.line.is_empty() {
            eprintln!("[uart]");
            return;
        }
        // Replace non-ASCII / control bytes so a misrouted byte doesn't
        // scramble the terminal.
        let rendered: String = self
            .line
            .iter()
            .map(|&b| {
                if (0x20..=0x7e).contains(&b) || b == b'\t' {
                    b as char
                } else {
                    '.'
                }
            })
            .collect();
        eprintln!("[uart] {rendered}");
        self.line.clear();
    }

    pub fn finish(&mut self) {
        if !self.line.is_empty() {
            self.flush_line();
        }
        eprintln!("[uart] total bytes drained: {}", self.total_bytes);
    }

    pub fn total_bytes(&self) -> u64 {
        self.total_bytes
    }
}

fn main() {
    mdpicoem_harness::harness_tracing_init();
    if let Err(e) = run() {
        eprintln!("fatal: {e}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args().map_err(|e| {
        print_usage();
        e
    })?;

    let trace_text = std::fs::read_to_string(&args.trace)
        .map_err(|e| format!("reading {}: {e}", args.trace.display()))?;
    let events = parse_trace(&trace_text)?;
    eprintln!(
        "Loaded trace: {} events, span {} ns ({:.3} s)",
        events.len(),
        events.last().map(|e| e.ns).unwrap_or(0),
        events.last().map(|e| e.ns).unwrap_or(0) as f64 / 1e9,
    );

    // Construct emulator. Seed the clock tree so ns→cycle math matches
    // what firmware (once booted) will eventually see.
    //
    // step_quantum: defaults to 4 since the 2026-04-26 quantum sweep
    // (see DEFAULT_STEP_QUANTUM doc comment). q=4 keeps ISA-overshoot
    // ≤4 cycles (well below the 37-cycle ISA write window) and is
    // spectrally identical to q=1 on the Monkey Island AdLib trace.
    // Pass `--step-quantum 1` to restore single-cycle precision when
    // debugging ISA-edge alignment; larger values trade audio fidelity
    // for throughput.
    let mut builder = EmulatorBuilder::new(Config {
        sys_clk_hz: DEFAULT_SYS_CLK_HZ,
    })
    .step_quantum(args.step_quantum)
    .psram(Psram::picogus());
    if let Some(path) = &args.flash {
        let flash_bytes = load_flash(path)?;
        eprintln!(
            "Loaded flash: {} bytes from {}",
            flash_bytes.len(),
            path.display()
        );
        builder = builder.flash(flash_bytes);
    }
    let mut emu = builder.build().expect("Serial build is infallible");

    // PSRAM pre-seed (diagnostic): fill the entire 8 MB buffer with a
    // triangle-wave pattern so voices reading at arbitrary DRAM
    // addresses (beyond what the trace itself uploaded) still receive
    // non-zero sample data. Use this to prove out the I2S output chain
    // end-to-end when the trace's DRAM upload is incomplete.
    if std::env::var("PICOGUS_PSRAM_PRESEED").is_ok() {
        if let Some(ref mut psram) = emu.bus.psram {
            for (i, byte) in psram.buffer.iter_mut().enumerate() {
                // 8-bit triangle wave, period 256. Signed view:
                // -127..+127 rising, then +127..-127 falling.
                let phase = (i & 0xFF) as i32;
                let sample = if phase < 128 { phase - 64 } else { 192 - phase };
                *byte = (sample & 0xFF) as u8;
            }
            eprintln!(
                "PSRAM pre-seeded with 8-bit triangle wave (PICOGUS_PSRAM_PRESEED set)"
            );
        }
    }

    // Preseed PSRAM from a binary file containing raw DRAM bytes.
    // Byte 0 of the file lands at PSRAM offset 0 (= GUS DRAM address 0).
    // Overrides / follows PICOGUS_PSRAM_PRESEED if both are set.
    // Use `wrk_scratch/extract_dram_from_trace.py` to produce a file
    // from a captured picogus-tap trace.
    if let Ok(path) = std::env::var("PICOGUS_PSRAM_LOAD") {
        let bytes = std::fs::read(&path)
            .map_err(|e| format!("PICOGUS_PSRAM_LOAD: reading {path}: {e}"))?;
        if let Some(ref mut psram) = emu.bus.psram {
            let n = bytes.len().min(psram.buffer.len());
            psram.buffer[..n].copy_from_slice(&bytes[..n]);
            let nz = psram.buffer[..n].iter().filter(|&&b| b != 0).count();
            eprintln!(
                "PSRAM pre-loaded {n} bytes from {path} (nonzero={nz})"
            );
        }
    }

    // Load the RP2040 bootrom before calling reset(): reset() reads SP
    // and the reset vector from ROM word 0 / word 4, so the ROM must be
    // populated first. If no explicit --bootrom is supplied, default-
    // search `roms/rp2040/bootrom-rp2040-b2.bin` when we have flash to
    // boot; otherwise skip (the replayer's unit-test path runs without).
    let resolved_bootrom =
        resolve_bootrom_path(args.bootrom.as_deref(), args.flash.is_some());
    match (&resolved_bootrom, args.flash.is_some()) {
        (Some(path), _) => {
            let bytes = std::fs::read(path)
                .map_err(|e| format!("reading bootrom {}: {e}", path.display()))?;
            eprintln!("Loaded bootrom: {} bytes from {}", bytes.len(), path.display());
            emu.load_bootrom(&bytes);
        }
        (None, true) => {
            eprintln!(
                "warning: --flash supplied but no bootrom found at {} — firmware will wedge at boot2 return",
                DEFAULT_BOOTROM_PATH
            );
        }
        (None, false) => {}
    }

    emu.reset();

    // PicoGUS multifw bootloader reads watchdog scratch[3] at boot to
    // select a firmware slot. SCRATCH0 is at WATCHDOG_BASE + 0x0C per
    // pico-sdk `watchdog_hw_t`, so SCRATCH3 sits at +0x18.
    if let Some(slot) = args.firmware_mode {
        let addr = mdrp2040::bus::WATCHDOG_BASE + 0x18;
        eprintln!(
            "[diag] firmware-mode: direct-field write of slot {slot} to watchdog scratch[3] @ {addr:#x} (bypasses RESETS gate)"
        );
        emu.bus.watchdog_tick.scratch[3] = slot;
    }

    // If flash looks like a pico-sdk image (boot2 at 0x000 + vector
    // table at 0x100 with SP in SRAM and PC in flash), direct-boot into
    // the SDK firmware's own reset handler — bypassing the vendored
    // bootrom's USB-MSC wait loop (we don't model QSPI enough for its
    // flash detection to succeed). See `Emulator::direct_boot_from_flash`
    // for rationale. For hand-crafted flash images that put code at
    // 0x10000000 directly (e.g. `roms/rp2040/i2s_chime.bin`), skip the
    // direct-boot step and let the synthetic bootrom boot normally.
    if args.flash.is_some() {
        const SDK_VTOR_FLASH_OFFSET: u32 = 0x100;
        let sp_at_vtor = emu.bus.memory.xip_read32(SDK_VTOR_FLASH_OFFSET);
        let pc_at_vtor = emu.bus.memory.xip_read32(SDK_VTOR_FLASH_OFFSET + 4);
        // Top-of-SRAM initial SP (0x2004_2000) is the canonical pico-sdk
        // value — one past the last valid SRAM byte, since Thumb pushes
        // pre-decrement. Use an inclusive range so it's accepted.
        let sp_in_sram = (0x2000_0000..=0x2004_2000).contains(&sp_at_vtor);
        let pc_in_flash = (0x1000_0000..0x1020_0000).contains(&(pc_at_vtor & !1));
        if sp_in_sram && pc_in_flash {
            emu.direct_boot_from_flash(SDK_VTOR_FLASH_OFFSET);
            eprintln!(
                "direct-boot: SP={:#010x} PC={:#010x} (flash+{:#x})",
                emu.cores[0].regs.sp(),
                emu.cores[0].regs.pc(),
                SDK_VTOR_FLASH_OFFSET
            );
        } else {
            eprintln!(
                "direct-boot skipped: flash+{:#x} has SP={:#010x} PC={:#010x} (not an SDK vector table) — booting via bootrom reset vector",
                SDK_VTOR_FLASH_OFFSET, sp_at_vtor, pc_at_vtor,
            );
        }
    }

    // Patch SRAM after runtime_init so .data copy is in place.
    // test_psram takes ~1 hour to complete; stub it to return 0.
    //
    // Address defaults to the stock `picogus-v4.0.0.bin` layout
    // (`0x20012FA4`). Override via `PICOGUS_STUB_TEST_PSRAM=0x<addr>` for
    // other builds, or `=0` to skip the patch entirely (letting the real
    // PSRAM test run — useful when debugging PSRAM itself).
    let stub_addr: u32 = match std::env::var("PICOGUS_STUB_TEST_PSRAM") {
        Ok(s) => u32::from_str_radix(s.trim_start_matches("0x"), 16)
            .unwrap_or_else(|_| panic!("PICOGUS_STUB_TEST_PSRAM must be hex, got {s:?}")),
        Err(_) => 0x2001_2FA4,
    };
    let mut uart_drain = UartDrain::new();
    {
        let old_q = emu.step_quantum;
        emu.step_quantum = 64;
        for i in 0..200_000u64 {
            if emu.step().expect("Serial step is infallible") == 0 { break; }
            // Poll the UART every 256 iterations so early-boot puts()
            // are visible interleaved with the warm-up timeline.
            if i & 0xff == 0 {
                uart_drain.drain_emu(&mut emu);
            }
        }
        uart_drain.drain_emu(&mut emu);
        if stub_addr != 0 {
            emu.bus.write32(stub_addr, 0x4770_2000); // MOVS R0,#0; BX LR
            eprintln!("patched SRAM 0x{stub_addr:08X}: test_psram -> return 0");
        } else {
            eprintln!("PICOGUS_STUB_TEST_PSRAM=0: test_psram left live");
        }

        // Diagnostic stub: replace `GUS_sample_stereo` with a constant
        // non-zero return. Used to isolate whether the WAV-silence bug
        // is inside `GUS_sample_stereo` vs. between the function return
        // and the TXF store in `audio_sample_handler`. When enabled, if
        // the WAV contains the constant sample value, the ISR → TXF →
        // I2S path is healthy and the bug is inside GUS_sample_stereo.
        // If the WAV is still silent, the audio chain itself is broken.
        //
        // `PICOGUS_STUB_GUS_SAMPLE_STEREO=0x<addr>`: address of the
        // function (`0x20000cac` in the rebuild v1 ELF). `=0` disables.
        //
        // Patch = `MOVS R0, #0xFF; BX LR` (returns 0xFF → 16-bit signed
        // int +255 on left channel, 0 on right). Audible DC offset.
        let stub_gss_addr: u32 = match std::env::var("PICOGUS_STUB_GUS_SAMPLE_STEREO") {
            Ok(s) => u32::from_str_radix(s.trim_start_matches("0x"), 16)
                .unwrap_or_else(|_| panic!("PICOGUS_STUB_GUS_SAMPLE_STEREO must be hex, got {s:?}")),
            Err(_) => 0,
        };
        if stub_gss_addr != 0 {
            // 0x4770_20FF = BX LR (0x4770 at offset +2) | MOVS R0,#0xFF (0x20FF at +0)
            // little-endian halfword order in a word write: low-half first.
            emu.bus.write32(stub_gss_addr, 0x4770_20FF);
            eprintln!(
                "patched SRAM 0x{stub_gss_addr:08X}: GUS_sample_stereo -> return 0xFF"
            );
        }

        // Diagnostic: stack read/write roundtrip. Writes #77 to [sp, #0],
        // clobbers R0, reads it back, returns. If WAV LEFT = 38 (77>>1),
        // stack at SP+0 works in core 1 IRQ context. If 0, stack broken.
        //
        // `PICOGUS_STUB_STACK_TEST=0x<func_addr>`
        //  +0  SUB sp,#8     (0xB082)
        //  +2  MOVS R0,#77   (0x204D)
        //  +4  STR R0,[sp,#0](0x9000)
        //  +6  MOVS R0,#0    (0x2000)   ; clobber
        //  +8  LDR R0,[sp,#0](0x9800)
        //  +10 ADD sp,#8     (0xB002)
        //  +12 BX LR         (0x4770)
        //  +14 NOP           (0xBF00)
        if let Ok(s) = std::env::var("PICOGUS_STUB_STACK_TEST") {
            let f = u32::from_str_radix(s.trim_start_matches("0x"), 16).unwrap();
            emu.bus.write32(f +  0, (0xB082u32) | (0x204Du32 << 16));
            emu.bus.write32(f +  4, (0x9000u32) | (0x2000u32 << 16));
            emu.bus.write32(f +  8, (0x9800u32) | (0xB002u32 << 16));
            emu.bus.write32(f + 12, (0x4770u32) | (0xBF00u32 << 16));
            eprintln!(
                "patched SRAM 0x{f:08X}: GUS_sample_stereo -> stack roundtrip #77"
            );
        }

        // Diagnostic: stack-based accumulator loop. Sum (VolLeft * 100)
        // over N voices but keep the accumulator on the stack at [sp, #4].
        // Expected: same as PICOGUS_STUB_SUM_MUL_VL (1100 for voice 1).
        // If 0 while STACK_TEST passes → stack writes inside loop drop.
        //
        // `PICOGUS_STUB_STACK_ACCUM=0x<func_addr>:0x<guschan_addr>:<count>`
        //  +0  SUB sp,#8          (0xB082)
        //  +2  MOVS R0,#0         (0x2000)
        //  +4  STR R0,[sp,#4]     (0x9001)  ; init accum on stack
        //  +6  LDR R2,[PC,#24]    (0x4A06)  ; &guschan  (pc+28 aligned)
        //  +8  MOVS R1,#n         (0x2100|n)
        //  +10 MOVS R4,#100       (0x2464)
        //  loop:
        //  +12 LDMIA R2!,{R3}     (0xCA08)
        //  +14 LDR R3,[R3,#52]    (0x6B5B)
        //  +16 MULS R3,R4         (0x4363)
        //  +18 LDR R0,[sp,#4]     (0x9801)
        //  +20 ADDS R0,R0,R3      (0x18C0)
        //  +22 STR R0,[sp,#4]     (0x9001)
        //  +24 SUBS R1,#1         (0x3901)
        //  +26 BNE loop           (0xD1F7) ; offset = 12 - (26+4) = -18; imm8=-9=0xF7
        //  +28 LDR R0,[sp,#4]     (0x9801)
        //  +30 ADD sp,#8          (0xB002)
        //  +32 BX LR              (0x4770)
        //  +34 NOP                (0xBF00)
        //  +36 .word &guschan
        if let Ok(s) = std::env::var("PICOGUS_STUB_STACK_ACCUM") {
            let p: Vec<&str> = s.split(':').collect();
            if p.len() == 3 {
                let f = u32::from_str_radix(p[0].trim_start_matches("0x"), 16).unwrap();
                let g = u32::from_str_radix(p[1].trim_start_matches("0x"), 16).unwrap();
                let n: u16 = p[2].parse().unwrap();
                assert!(n <= 255);
                // Compute literal position. The LDR R2,[PC,#imm] at offset +6
                // has pc = (f+6+4) & !3 = (f+10)&!3. We want the literal at
                // f+36. So imm = ((f+36) - ((f+10)&!3)).
                // If f is word-aligned, f+10 → pc-align → f+8; imm = 28.
                // 0x4A07: Rd=2, imm8 = 28/4 = 7.
                // Ah, I initially put 0x4A06 which is imm8=6 → pc+24 = f+32.
                // Need imm8=7 for pc+28 = f+36.
                let ldr_r2 = 0x4A07u32;
                emu.bus.write32(f +  0, 0x2000_B082);                 // SUB sp,#8 ; MOVS R0,#0
                emu.bus.write32(f +  4, (0x9001u32) | (ldr_r2 << 16)); // STR R0,[sp,#4] ; LDR R2,[PC,#28]
                emu.bus.write32(f +  8, (0x2100u32 | n as u32) | (0x2464u32 << 16)); // MOVS R1,#n ; MOVS R4,#100
                emu.bus.write32(f + 12, 0x6B5B_CA08);                 // LDMIA R2!,{R3} ; LDR R3,[R3,#52]
                emu.bus.write32(f + 16, 0x9801_4363);                 // MULS R3,R4 ; LDR R0,[sp,#4]
                emu.bus.write32(f + 20, 0x9001_18C0);                 // ADDS R0,R0,R3 ; STR R0,[sp,#4]
                emu.bus.write32(f + 24, 0xD1F7_3901);                 // SUBS R1,#1 ; BNE loop
                emu.bus.write32(f + 28, 0xB002_9801);                 // LDR R0,[sp,#4] ; ADD sp,#8
                emu.bus.write32(f + 32, 0xBF00_4770);                 // BX LR ; NOP
                emu.bus.write32(f + 36, g);                           // literal
                eprintln!(
                    "patched SRAM 0x{f:08X}: GUS_sample_stereo -> stack accum sum(VolLeft*100) over {n}"
                );
            }
        }

        // Diagnostic: sum (VolLeft * 100) over 27 voices — simulates the
        // real mixer's `tmpsamp * VolLeft` accumulate, with a constant
        // tmpsamp=100 replacing the PSRAM read. Expected: 100*21 = 2100
        // (voice 1 only). WAV LEFT should be 1050 after 1-bit shift.
        //
        // `PICOGUS_STUB_SUM_MUL_VL=0x<func_addr>:0x<guschan_addr>:<count>`
        if let Ok(s) = std::env::var("PICOGUS_STUB_SUM_MUL_VL") {
            let p: Vec<&str> = s.split(':').collect();
            if p.len() == 3 {
                let f = u32::from_str_radix(p[0].trim_start_matches("0x"), 16).unwrap();
                let g = u32::from_str_radix(p[1].trim_start_matches("0x"), 16).unwrap();
                let n: u16 = p[2].parse().unwrap();
                // Instructions:
                //  +0  MOVS R0,#0      (0x2000)  // accum
                //  +2  LDR  R2,[PC,#20] (0x4A05) // &guschan
                //  +4  MOVS R1,#n      (0x2100|n)
                //  +6  MOVS R4,#100    (0x2464)  // const sample
                //  +8  LDMIA R2!,{R3}  (0xCA08)
                //  +10 LDR  R3,[R3,#52](0x6B5B)  // VolLeft
                //  +12 MULS R3,R4      (0x4363)
                //  +14 ADDS R0,R0,R3   (0x18C0)
                //  +16 SUBS R1,#1      (0x3901)
                //  +18 BNE -14         (0xD1F9)
                //  +20 BX LR           (0x4770)
                //  +22 NOP             (0xBF00)
                //  +24 .word &guschan
                emu.bus.write32(f +  0, 0x4A05_2000);
                emu.bus.write32(f +  4, 0x2464_2100u32 | n as u32);
                emu.bus.write32(f +  8, 0x6B5B_CA08);
                emu.bus.write32(f + 12, 0x18C0_4363);
                emu.bus.write32(f + 16, 0xD1F9_3901);
                emu.bus.write32(f + 20, 0xBF00_4770);
                emu.bus.write32(f + 24, g);
                eprintln!(
                    "patched SRAM 0x{f:08X}: GUS_sample_stereo -> sum(VolLeft*100) over {n} voices"
                );
            }
        }

        // Diagnostic: sum VolLeft over all 27 voices via `guschan[c]->VolLeft`.
        // Closest single-step approximation of the real mixing loop.
        // Expected result ~ 21 (only voice 1 has non-zero VolLeft at end).
        //
        // `PICOGUS_STUB_SUM_VOLLEFTS=0x<func_addr>:0x<guschan_addr>:<count>`
        //  MOVS R0,#0 ; LDR R2,=&guschan[0] ; MOVS R1,#count
        //  loop: LDMIA R2!,{R3} ; LDR R3,[R3,#52] ; ADDS R0,R0,R3 ; SUBS R1,#1 ; BNE loop
        //  BX LR
        if let Ok(s) = std::env::var("PICOGUS_STUB_SUM_VOLLEFTS") {
            let p: Vec<&str> = s.split(':').collect();
            if p.len() == 3 {
                let f = u32::from_str_radix(p[0].trim_start_matches("0x"), 16).unwrap();
                let g = u32::from_str_radix(p[1].trim_start_matches("0x"), 16).unwrap();
                let n: u16 = p[2].parse().unwrap();
                assert!(n <= 255);
                let movs_r0 = 0x2000u16;
                let ldr_r2 = 0x4A04u16;                // LDR R2,[PC,#16]
                let movs_r1 = 0x2100u16 | n;            // MOVS R1,#n
                let ldmia = 0xCA08u16;                  // LDMIA R2!,{R3}
                let ldr_vl = 0x6B5Bu16;                 // LDR R3,[R3,#52]
                let adds = 0x18C0u16;                   // ADDS R0,R0,R3
                let subs = 0x3901u16;                   // SUBS R1,#1
                let bne = 0xD1FAu16;                    // BNE -12
                let bx_lr = 0x4770u16;
                let nop = 0xBF00u16;
                emu.bus.write32(f + 0,  (movs_r0 as u32) | ((ldr_r2 as u32) << 16));
                emu.bus.write32(f + 4,  (movs_r1 as u32) | ((ldmia as u32) << 16));
                emu.bus.write32(f + 8,  (ldr_vl as u32) | ((adds as u32) << 16));
                emu.bus.write32(f + 12, (subs as u32)   | ((bne as u32) << 16));
                emu.bus.write32(f + 16, (bx_lr as u32)  | ((nop as u32) << 16));
                emu.bus.write32(f + 20, g);
                eprintln!(
                    "patched SRAM 0x{f:08X}: GUS_sample_stereo -> sum(guschan[0..{n}]->VolLeft)"
                );
            }
        }

        // Diagnostic: 27-iteration loop sum (1+2+...+27 = 378). Tests
        // whether a tight `ADDS/SUBS/BNE` loop runs correctly on core 1
        // within an IRQ. If WAV shows 378 (or 189 after our 1-bit shift),
        // loop mechanics work.
        //
        // `PICOGUS_STUB_LOOP_TEST=0x<func_addr>`
        //   MOVS R0, #0
        //   MOVS R1, #27
        // loop:
        //   ADDS R0, R0, R1
        //   SUBS R1, #1
        //   BNE loop
        //   BX LR
        if let Ok(s) = std::env::var("PICOGUS_STUB_LOOP_TEST") {
            let f = u32::from_str_radix(s.trim_start_matches("0x"), 16).unwrap();
            // word 0: MOVS R0,#0 (0x2000) | MOVS R1,#27 (0x211B) → 0x211B_2000
            emu.bus.write32(f, 0x211B_2000);
            // word 1: ADDS R0,R0,R1 (0x1840) | SUBS R1,#1 (0x3901) → 0x3901_1840
            emu.bus.write32(f + 4, 0x3901_1840);
            // word 2: BNE -8 (0xD1FC) | BX LR (0x4770) → 0x4770_D1FC
            emu.bus.write32(f + 8, 0x4770_D1FC);
            eprintln!(
                "patched SRAM 0x{f:08X}: GUS_sample_stereo -> sum(1..27) = 378"
            );
        }

        // Diagnostic: chain-deref stub.
        // `PICOGUS_STUB_CHAIN=<func>:<literal>:<off1>:<off2>` — stub at
        // `func` loads `literal` (4-byte word), dereferences [+off1],
        // dereferences [+off2], returns result. Lets us probe e.g.
        // `guschan[1]->VolLeft` without touching firmware.
        //
        // Example: `0x20000cac:0x20017c3c:4:52` →
        //   LDR R0, =0x20017c3c
        //   LDR R0, [R0, #4]   ; guschan[1]
        //   LDR R0, [R0, #52]  ; ->VolLeft
        //   BX LR
        if let Ok(s) = std::env::var("PICOGUS_STUB_CHAIN") {
            let p: Vec<&str> = s.split(':').collect();
            if p.len() == 4 {
                let f = u32::from_str_radix(p[0].trim_start_matches("0x"), 16).unwrap();
                let lit = u32::from_str_radix(p[1].trim_start_matches("0x"), 16).unwrap();
                let off1: u32 = p[2].parse().unwrap();
                let off2: u32 = p[3].parse().unwrap();
                assert!(off1 < 128 && off1 % 4 == 0, "off1 must be 0..128, mult 4");
                assert!(off2 < 128 && off2 % 4 == 0, "off2 must be 0..128, mult 4");
                let imm5_1 = (off1 / 4) as u16;
                let imm5_2 = (off2 / 4) as u16;
                // LDR R0,[R0,#imm5<<2]: 01101 imm5 000 000 = 0x6800 | (imm5 << 6)
                let ldr1 = 0x6800 | (imm5_1 << 6);
                let ldr2 = 0x6800 | (imm5_2 << 6);
                // LDR R0,[PC,#4] = 0x4801 (PC_aligned + 4)
                let ldr_lit: u16 = 0x4801;
                let bx_lr: u16 = 0x4770;
                // word @ f: (ldr_lit | ldr1<<16)
                emu.bus.write32(f, (ldr_lit as u32) | ((ldr1 as u32) << 16));
                // word @ f+4: (ldr2 | bx_lr<<16)
                emu.bus.write32(f + 4, (ldr2 as u32) | ((bx_lr as u32) << 16));
                // word @ f+8: literal
                emu.bus.write32(f + 8, lit);
                eprintln!(
                    "patched SRAM 0x{f:08X}: GUS_sample_stereo -> *(u32*)((*(u32*)0x{lit:08X} +{off1})+{off2})"
                );
            }
        }

        // Diagnostic: stub GUS_sample_stereo to return the product of
        // two constants (21 * 127 = 2667). Verifies that MULS works
        // on core 1 inside an IRQ handler context. If WAV shows 2667
        // (or 1333 = 2667>>1 with our capture's 1-bit shift), MULS is
        // fine. If shows 0, MULS is broken.
        //
        // `PICOGUS_STUB_MULT_TEST=0x<func_addr>` — patch address.
        // Patch: MOVS R0,#21 ; MOVS R1,#127 ; MULS R0,R1 ; BX LR
        if let Ok(s) = std::env::var("PICOGUS_STUB_MULT_TEST") {
            let f = u32::from_str_radix(s.trim_start_matches("0x"), 16).unwrap();
            // word 0: MOVS R0,#21 (0x2015) | MOVS R1,#127 (0x217F)  → 0x217F_2015
            emu.bus.write32(f, 0x217F_2015);
            // word 1: MULS R0,R1 (0x4348)  | BX LR (0x4770)          → 0x4770_4348
            emu.bus.write32(f + 4, 0x4770_4348);
            eprintln!(
                "patched SRAM 0x{f:08X}: GUS_sample_stereo -> return 21 * 127 = 2667"
            );
        }

        // Diagnostic: replace GUS_sample_stereo with a stub that returns
        // `myGUS.ActiveChannels` (as u32). If the WAV then shows sample
        // value 27 (= ActiveChannels), core 1 is reading SRAM correctly
        // from within GUS_sample_stereo. If it shows 0, core 1 can't
        // see core-0 writes to myGUS.
        //
        // `PICOGUS_STUB_RET_ACTIVE=0x<func_addr>:0x<activech_addr>`
        // Writes a 3-word stub at func_addr and a literal pointing to
        // activech_addr.
        //
        // Patch layout at func_addr:
        //   +00: LDR R0, [PC, #4]  (0x4801)      → load literal
        //   +02: LDRB R0, [R0, #0] (0x7800)      → load byte *R0
        //   +04: BX LR             (0x4770)
        //   +06: NOP               (0xBF00)
        //   +08: .word activech_addr
        if let Ok(s) = std::env::var("PICOGUS_STUB_RET_ACTIVE") {
            let parts: Vec<&str> = s.split(':').collect();
            if parts.len() == 2 {
                let f = u32::from_str_radix(parts[0].trim_start_matches("0x"), 16).unwrap();
                let a = u32::from_str_radix(parts[1].trim_start_matches("0x"), 16).unwrap();
                emu.bus.write32(f, 0x7800_4801);     // LDR R0,[PC,#4] ; LDRB R0,[R0]
                emu.bus.write32(f + 4, 0xBF00_4770); // BX LR ; NOP
                emu.bus.write32(f + 8, a);           // literal
                eprintln!(
                    "patched SRAM 0x{f:08X}: GUS_sample_stereo -> return *(u8*)0x{a:08X} (zext)"
                );
            }
        }

        // Diagnostic: bypass the `(GUS_reset_reg & 0x03) != 0x03` early
        // return inside GUS_sample_stereo by NOP'ing the branch at the
        // failure arm (addr 0x20000cc6 in rebuild v1). Falls through to
        // the normal mixing loop regardless of reset_reg value.
        //
        // `PICOGUS_PATCH_BYPASS_GATE=0x<addr>` — address of the
        // `b.n 20001036` instruction (0x20000cc6 in rebuild v1). `=0`
        // disables. Writes NOP (0xBF00) halfword.
        if let Ok(s) = std::env::var("PICOGUS_PATCH_BYPASS_GATE") {
            let addr = u32::from_str_radix(s.trim_start_matches("0x"), 16)
                .unwrap_or_else(|_| panic!("PICOGUS_PATCH_BYPASS_GATE must be hex, got {s:?}"));
            if addr != 0 {
                // 16-bit halfword write: keep the next halfword at +2 intact
                // by reading it, then writing as a 32-bit word with low=0xBF00,
                // high=original_next_halfword.
                let existing = emu.bus.read32(addr & !3);
                // If addr is halfword-aligned (even), we need to write the low
                // halfword. Compute new word preserving the other half.
                let new_word = if addr & 2 == 0 {
                    (existing & 0xFFFF_0000) | 0x0000_BF00
                } else {
                    (existing & 0x0000_FFFF) | 0xBF00_0000
                };
                emu.bus.write32(addr & !3, new_word);
                eprintln!(
                    "patched SRAM 0x{addr:08X}: b.n 20001036 -> NOP (bypass GUS_reset_reg gate)"
                );
            }
        }
        emu.step_quantum = old_q;
    }

    let duration_ns = args
        .duration_secs
        .map(|s| (s * 1e9).max(0.0) as u64);
    let post_roll_ns = (args.post_roll_secs * 1e9).max(0.0) as u64;
    let pre_roll_ns = (args.pre_roll_secs * 1e9).max(0.0) as u64;
    let trace_stretch = args.trace_stretch;
    let out_path = args
        .out
        .clone()
        .unwrap_or_else(|| mdpicoem_harness::default_out_path(&args.trace));

    if pre_roll_ns > 0 || (trace_stretch - 1.0).abs() >= f64::EPSILON {
        eprintln!(
            "replay: pre_roll={:.3} s  trace_stretch={:.3}x",
            pre_roll_ns as f64 / 1e9,
            trace_stretch
        );
    }

    let wall_start = Instant::now();
    let mut sink = CapturingSink::new(emu, DEFAULT_SYS_CLK_HZ);
    // If PICOGUS_POKE_DURING_REPLAY=1, reuse the PICOGUS_POKE list
    // to force the gate open after every trace-event advance. Keeps
    // `GUS_reset_reg` at 0x07 as MIDI events program and trigger
    // voices mid-replay — otherwise voices stay silent because
    // `GUSReset` lands `0x01` (emulator bug; see journal Finding 10).
    let replay_pokes: Vec<(u32, u8)> =
        if std::env::var("PICOGUS_POKE_DURING_REPLAY").ok().as_deref() == Some("1") {
            std::env::var("PICOGUS_POKE")
                .unwrap_or_default()
                .split(',')
                .filter(|s| !s.trim().is_empty())
                .filter_map(|item| {
                    let (a, v) = item.trim().split_once('=')?;
                    let a = u32::from_str_radix(a.trim().trim_start_matches("0x"), 16).ok()?;
                    let v = u8::from_str_radix(v.trim().trim_start_matches("0x"), 16).ok()?;
                    Some((a, v))
                })
                .collect()
        } else {
            Vec::new()
        };
    if !replay_pokes.is_empty() {
        eprintln!(
            "PICOGUS_POKE_DURING_REPLAY: applying {} poke(s) after every trace event",
            replay_pokes.len()
        );
    }
    let (summary, coverage) = replay_with_coverage(
        &mut sink,
        &events,
        duration_ns,
        Some(post_roll_ns),
        pre_roll_ns,
        trace_stretch,
        &mut uart_drain,
        &replay_pokes,
    );
    // --- experimental poke + extra post-roll ---
    // Format: PICOGUS_POKE=0xADDR=0xVAL[,...]  PICOGUS_EXTRA_POSTROLL=<secs>
    // Applies the list of byte pokes to SRAM AFTER the trace-driven
    // replay but BEFORE an optional extra sim-time window during which
    // I2S capture continues. Used to confirm candidate addresses for
    // GUS_reset_reg — poke the candidate to 0x07 and check whether the
    // extra-postroll WAV contains audio.
    let extra_postroll_secs: f64 = std::env::var("PICOGUS_EXTRA_POSTROLL")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(0.0);
    let poke_spec = std::env::var("PICOGUS_POKE").unwrap_or_default();
    if !poke_spec.is_empty() || extra_postroll_secs > 0.0 {
        eprintln!("=== poke + extra post-roll phase ===");
        // Parse poke list once.
        let pokes: Vec<(u32, u8)> = poke_spec
            .split(',')
            .filter(|s| !s.trim().is_empty())
            .filter_map(|item| {
                let (a, v) = item.trim().split_once('=')?;
                let a = u32::from_str_radix(a.trim().trim_start_matches("0x"), 16).ok()?;
                let v = u8::from_str_radix(v.trim().trim_start_matches("0x"), 16).ok()?;
                Some((a, v))
            })
            .collect();
        // Apply byte pokes via read-modify-write on the 32-bit word.
        let apply_pokes = |emu: &mut mdrp2040::Emulator, pokes: &[(u32, u8)]| {
            for &(a, v) in pokes {
                let word_addr = a & !3;
                let shift = (a & 3) * 8;
                let w = emu.peek(word_addr);
                let w_new = (w & !(0xff << shift)) | ((v as u32) << shift);
                emu.poke(word_addr, w_new);
            }
        };
        for &(a, v) in &pokes {
            let word_addr = a & !3;
            let shift = (a & 3) * 8;
            let w = sink.inner_mut().peek(word_addr);
            let w_new = (w & !(0xff << shift)) | ((v as u32) << shift);
            sink.inner_mut().poke(word_addr, w_new);
            eprintln!(
                "  poke 0x{a:08x} byte = 0x{v:02x} (word 0x{word_addr:08x}: 0x{w:08x} -> 0x{w_new:08x}) [initial]"
            );
        }
        if extra_postroll_secs > 0.0 {
            // Re-poke every N steps to survive firmware overwrites.
            let repoke = std::env::var("PICOGUS_REPOKE").ok().as_deref() == Some("1");
            // Step granularity for extra-postroll (default 64).
            let postroll_step: u32 = std::env::var("PICOGUS_POSTROLL_STEP")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(64);
            // Optional: watch a specific SRAM byte — log transitions.
            let watch_addr: Option<u32> = std::env::var("PICOGUS_WATCH")
                .ok()
                .and_then(|s| u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).ok());
            eprintln!(
                "  extra post-roll: {extra_postroll_secs:.3} s  re-poke={repoke}  step={postroll_step}"
            );
            let hz = sink.sys_clk_hz() as u64;
            let extra_cycles = (extra_postroll_secs * hz as f64) as u64;
            let target = sink.cycles().saturating_add(extra_cycles);
            let mut last_watch: Option<u8> = watch_addr.map(|a| {
                let w = sink.inner_mut().peek(a & !3);
                ((w >> ((a & 3) * 8)) & 0xff) as u8
            });
            let mut transitions = 0u64;
            while sink.cycles() < target {
                let before = sink.cycles();
                sink.step(postroll_step);
                if sink.cycles() == before {
                    break;
                }
                if let Some(a) = watch_addr {
                    let w = sink.inner_mut().peek(a & !3);
                    let cur = ((w >> ((a & 3) * 8)) & 0xff) as u8;
                    if last_watch != Some(cur) {
                        if transitions < 40 {
                            eprintln!(
                                "  [watch] cycle {} byte 0x{:08x}: 0x{:02x} -> 0x{:02x}",
                                sink.cycles(), a, last_watch.unwrap_or(0), cur
                            );
                        }
                        transitions += 1;
                        last_watch = Some(cur);
                    }
                }
                if repoke {
                    apply_pokes(sink.inner_mut(), &pokes);
                }
                uart_drain.drain_emu(sink.inner_mut());
            }
            if watch_addr.is_some() {
                eprintln!("  [watch] total transitions: {transitions}");
            }
        }
    }

    let wall_elapsed = wall_start.elapsed();
    let (mut emu, mut capture) = sink.into_parts();
    // Flush any bytes firmware wrote in the final step, then print the
    // running total so the grand total shows up even if the final line
    // didn't end in '\n'.
    uart_drain.drain_emu(&mut emu);
    uart_drain.finish();
    // Snapshot the live `clk_sys` for reporting. PicoGUS firmware
    // reprograms PLL_SYS from 125→370 MHz early in boot; the I2S
    // capture observes all LRCLK edges in the post-reprogram era, so
    // inferring their period against the firmware's *current* sysclk
    // yields the correct sample rate. See the "PicoGUS PLL 370 MHz
    // Reprogram Diagnosis" note in `wrk_scratch/`.
    let final_sys_clk_hz = emu.bus.sys_clk_hz();
    capture.set_sys_clk_hz(final_sys_clk_hz);
    // Core 1 launch check — if `multicore_launch_core1` succeeded, core 1's PC
    // has advanced past its reset state of 0.
    let core1_pc = emu.cores[1].regs.pc();
    let core1_halted = emu.cores[1].is_halted();

    // PSRAM diagnostics — shows whether the ISA → firmware → PSRAM
    // pipeline delivered any data.
    if let Some(ref psram) = emu.bus.psram {
        let nz = psram.buffer.iter().filter(|&&b| b != 0).count();
        println!();
        println!("--- PSRAM diagnostics ---");
        println!("Non-zero bytes:    {} / {}", nz, psram.buffer.len());
        println!("Bytes written:     {}", psram.bytes_written);
        println!("Bytes read:        {}", psram.bytes_read);
        println!(
            "Pin assignment:    MISO=GPIO{}  CS=GPIO{}  SCK=GPIO{}  MOSI=GPIO{}",
            psram.pin_miso(),
            psram.pin_cs(),
            psram.pin_sck(),
            psram.pin_mosi(),
        );
        println!("tick() calls:      {}", psram.tick_count);
        println!("CS# falling edges: {}  (== SPI frames started)", psram.cs_falling_count);

        // DRAM-upload signature probe (2026-04-24 investigation). Three
        // bytes at expected bank-1 locations (addr 65536, 65540, 65544).
        // Under picogus' literal interpretation (val>>8 → dram_high) the
        // game's bank-1 writes collapse onto bank 0 and these bytes stay
        // zero. Under swapped delivery (or a correct r44 interpretation)
        // they hold the game's 16-bit PCM data.
        let buf = &psram.buffer;
        let n = buf.len();
        let probe = |a: usize| if a < n { Some(buf[a]) } else { None };
        println!();
        println!("--- DRAM bank-1 signature probe ---");
        println!(
            "bytes [65532..65544] = {:?}",
            (65532..65544).map(probe).collect::<Vec<_>>()
        );
        println!(
            "bytes [65536..65568] = {:?}",
            (65536..65568).map(probe).collect::<Vec<_>>()
        );
        let bank1_nz = (65536..102_898).filter_map(probe).filter(|&b| b != 0).count();
        println!(
            "nonzero bytes in [65536..102898]: {}/37362",
            bank1_nz
        );
    }

    // ------------------------------------------------------------------
    // PIO1 sweep — the PicoGUS PSRAM SPI lives on PIO1 SM0 (single-SPI
    // bit-banger driving GPIO0..3). Mirrors the PIO0 dump above so the
    // two are directly comparable. PIO1_BASE = 0x5030_0000.
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: PIO1 state ---");
    let p1_ctrl = emu.bus.read32(0x5030_0000);
    println!(
        "PIO1 CTRL:         0x{:08x}  SM enabled mask=0x{:x}",
        p1_ctrl,
        p1_ctrl & 0xF,
    );
    let p1_fstat = emu.bus.read32(0x5030_0000 + 0x004);
    println!("PIO1 FSTAT:        0x{:08x}", p1_fstat);
    for sm in 0..4u32 {
        let rxfull = (p1_fstat >> sm) & 1;
        let rxempty = (p1_fstat >> (8 + sm)) & 1;
        let txfull = (p1_fstat >> (16 + sm)) & 1;
        let txempty = (p1_fstat >> (24 + sm)) & 1;
        println!(
            "PIO1 FSTAT SM{}:    RX full={} empty={}   TX full={} empty={}",
            sm, rxfull, rxempty, txfull, txempty
        );
    }
    println!();
    println!("--- Diag: PIO1 per-SM sweep ---");
    for sm in 0..4u32 {
        let base = 0x5030_0000 + 0x0C8 + sm * 0x18;
        let cd = emu.bus.read32(base);
        let cd_int = (cd >> 16) & 0xFFFF;
        let cd_frac = (cd >> 8) & 0xFF;
        let cd_eff = if cd_int == 0 { 65536 } else { cd_int };
        let ec = emu.bus.read32(base + 0x04);
        let pc = emu.bus.read32(base + 0x0C) & 0x1F;
        let pin = emu.bus.read32(base + 0x14);
        let sideset_count = (pin >> 29) & 0x7;
        let set_count = (pin >> 26) & 0x7;
        let out_count = (pin >> 20) & 0x3F;
        let in_base = (pin >> 15) & 0x1F;
        let sideset_base = (pin >> 10) & 0x1F;
        let set_base = (pin >> 5) & 0x1F;
        let out_base = pin & 0x1F;
        println!(
            "PIO1 SM{} PC: 0x{:02x}  CLKDIV: 0x{:08x} ({}.{:03} eff={})  EXECCTRL: 0x{:08x}",
            sm, pc, cd, cd_int, cd_frac, cd_eff, ec
        );
        println!(
            "         PINCTRL: 0x{:08x}  SIDESET cnt={} base={}  SET cnt={} base={}  OUT cnt={} base={}  IN base={}",
            pin, sideset_count, sideset_base, set_count, set_base, out_count, out_base, in_base
        );
        let sc = emu.bus.read32(base + 0x08);
        let autopush = (sc >> 16) & 1;
        let autopull = (sc >> 17) & 1;
        let in_shiftdir = (sc >> 18) & 1;
        let out_shiftdir = (sc >> 19) & 1;
        let push_thresh_raw = (sc >> 20) & 0x1F;
        let pull_thresh_raw = (sc >> 25) & 0x1F;
        let push_thresh = if push_thresh_raw == 0 { 32 } else { push_thresh_raw };
        let pull_thresh = if pull_thresh_raw == 0 { 32 } else { pull_thresh_raw };
        println!(
            "         SHIFTCTRL: 0x{:08x}  AUTOPUSH={} PUSH_THRESH={}  AUTOPULL={} PULL_THRESH={}  IN_SHIFTDIR={} OUT_SHIFTDIR={}",
            sc, autopush, push_thresh, autopull, pull_thresh, in_shiftdir, out_shiftdir
        );
        let pushes = emu.bus.pio[1].sm[sm as usize].autopush_count;
        println!("         autopush_count: {}", pushes);
    }

    // PIO1 pad state — which GPIOs PIO1 is actually driving.
    let p1_pad_oe = emu.bus.pio[1].pad_oe;
    let p1_pad_out = emu.bus.pio[1].pad_out;
    println!();
    println!("--- Diag: PIO1 pad state ---");
    println!(
        "PIO1 pad_oe:       0x{:08x}  bit0(MISO)={} bit1(CS)={} bit2(SCK)={} bit3(MOSI)={}",
        p1_pad_oe,
        (p1_pad_oe >> 0) & 1,
        (p1_pad_oe >> 1) & 1,
        (p1_pad_oe >> 2) & 1,
        (p1_pad_oe >> 3) & 1,
    );
    println!(
        "PIO1 pad_out:      0x{:08x}  bit0(MISO)={} bit1(CS)={} bit2(SCK)={} bit3(MOSI)={}",
        p1_pad_out,
        (p1_pad_out >> 0) & 1,
        (p1_pad_out >> 1) & 1,
        (p1_pad_out >> 2) & 1,
        (p1_pad_out >> 3) & 1,
    );

    // ------------------------------------------------------------------
    // PIO1 SM0 deep-dive — mirror of the PIO0 SM0 deep-dive below, but
    // for the PSRAM SPI bit-banger. Dumps all 32 instruction slots with
    // a disassembly annotation + PC arrow so we can see exactly which
    // opcode the SM is stalled on. Raw backing is accessed directly
    // because the RP2040 MMIO read of INSTR_MEM returns 0 per the
    // datasheet (write-only register interface).
    //
    // DACK (GPIO19) is reported even though the PSRAM program almost
    // certainly does not gate on it — kept for symmetry with the PIO0
    // deep-dive and as a negative datapoint.
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: PIO1 SM0 deep-dive ---");
    let p1_sm0_clkdiv = emu.bus.read32(0x5030_0000 + 0x0C8);
    let p1_sm0_clkdiv_int = (p1_sm0_clkdiv >> 16) & 0xFFFF;
    let p1_sm0_clkdiv_frac = (p1_sm0_clkdiv >> 8) & 0xFF;
    let p1_sm0_clkdiv_eff = if p1_sm0_clkdiv_int == 0 { 65536 } else { p1_sm0_clkdiv_int };
    println!(
        "PIO1 SM0 CLKDIV raw: 0x{:08x}  ({}.{:03} effective ~{} sysclk/PIO-tick)",
        p1_sm0_clkdiv, p1_sm0_clkdiv_int, p1_sm0_clkdiv_frac, p1_sm0_clkdiv_eff
    );
    let p1_sm0_addr = emu.bus.read32(0x5030_0000 + 0x0D4) & 0x1F;
    println!("PIO1 SM0 PC:         0x{:02x}", p1_sm0_addr);
    let p1_sm0_execctrl = emu.bus.read32(0x5030_0000 + 0x0CC);
    println!(
        "PIO1 SM0 EXECCTRL:   0x{:08x}  (bit31 EXEC_STALLED={})",
        p1_sm0_execctrl,
        (p1_sm0_execctrl >> 31) & 1
    );
    let gpio_in_p1 = emu.bus.gpio_in;
    let dack_high_p1 = (gpio_in_p1 >> 19) & 1 != 0;
    println!(
        "DACK (GPIO19):       {}    (gpio_in=0x{:08x})",
        if dack_high_p1 { "high" } else { "low" },
        gpio_in_p1
    );

    // INSTR_MEM dump — all 32 slots with MMIO (returns 0), raw backing,
    // and the disassembly annotation. Snapshot the backing store first
    // to avoid a borrow overlap with the subsequent MMIO read32() calls
    // on emu.bus.
    println!("INSTR_MEM (all 32 slots — MMIO / raw / DISASM):");
    let p1_im_snapshot: [u16; 32] = *emu.bus.pio[1].instr_mem();
    for i in 0..32usize {
        let mmio = emu.bus.read32(0x5030_0000 + 0x048 + (i as u32) * 4) & 0xFFFF;
        let raw = p1_im_snapshot[i];
        let arrow = if i as u32 == p1_sm0_addr {
            "  <-- PC"
        } else {
            ""
        };
        println!(
            "  [0x{:02x}] mmio=0x{:04x} raw=0x{:04x}  {}{}",
            i,
            mmio,
            raw,
            disasm_pio_instr(raw),
            arrow,
        );
    }

    // Execution counters — per-PC visit table plus stall counters.
    // PC visits bump when a fetched instruction at that slot actually
    // executes without stalling (forced execs excluded). Comparing the
    // visit count at slot 0x19 (OUT PINS, 1 with sideset) to the
    // pad_out CS-fall count and the PSRAM model's CS-fall count
    // localises where the PSRAM SPI pipeline drops edges.
    let p1_sm0 = &emu.bus.pio[1].sm[0];
    let p1_sm0_pc_visits = *p1_sm0.pc_visits();
    let p1_sm0_stall_cycles = p1_sm0.stall_cycles();
    let p1_sm0_stall_at_19 = p1_sm0.cycles_stalled_at_pc_0x19();
    println!("PIO1 SM0 execution counters:");
    println!("  stall cycles:              {}", p1_sm0_stall_cycles);
    println!("  cycles stalled at PC=0x19: {}", p1_sm0_stall_at_19);
    println!("  PC visits:");
    for i in 0x0eusize..=0x1fusize {
        println!("    [0x{:02x}] = {}", i, p1_sm0_pc_visits[i]);
    }
    println!("PIO1 pad_out transition counters (bits 1=CS, 2=SCK, 3=MOSI):");
    println!("  CS  falls:     {}", emu.bus.pio[1].pad_out_cs_falls);
    println!("  CS  rises:     {}", emu.bus.pio[1].pad_out_cs_rises);
    println!("  SCK toggles:   {}", emu.bus.pio[1].pad_out_sck_toggles);
    println!(
        "  MOSI=1 cycles: {}",
        emu.bus.pio[1].pad_out_mosi_writes_of_1
    );

    // ------------------------------------------------------------------
    // DMA dispatch diagnostic (HLD "PicoGUS DMA Dispatch Diagnostic"
    // Rev. 1). Four-way verdict across the 12 DMA channels: (1) no
    // config, (2) configured but never triggered, (3) triggered but the
    // engine never served, (4) engine served but PIO1 didn't consume.
    // All counters live on `DmaChannel`; end-state reads go through the
    // bus so CTRL splices live BUSY correctly.
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: DMA dispatch ---");
    const DMA_BASE_DIAG: u32 = 0x5000_0000;
    println!(
        "CH  READ_ADDR   WRITE_ADDR  COUNT   CTRL       EN TREQ BUSY ever→TXF  \
         trig(CTRL/WADDR/TCNT/AL2T/AL3R/MULTI)  xfers      dreq_seen"
    );
    let mut ever_txf_channels: Vec<usize> = Vec::new();
    let mut triggered_channels: Vec<usize> = Vec::new();
    let mut served_channels: Vec<usize> = Vec::new();
    for ch in 0..12usize {
        let base = DMA_BASE_DIAG + (ch as u32) * 0x40;
        let read_addr = emu.bus.read32(base + 0x00);
        let write_addr = emu.bus.read32(base + 0x04);
        let trans_count = emu.bus.read32(base + 0x08);
        let ctrl = emu.bus.read32(base + 0x0C);
        let en = ctrl & 1;
        let treq = (ctrl >> 15) & 0x3F;
        let busy = (ctrl >> 24) & 1;
        let c = emu.bus.dma.channel(ch);
        let any_trig = c.trig_ctrl
            + c.trig_write_addr
            + c.trig_trans_count
            + c.trig_al2_trans
            + c.trig_al3_read_addr
            + c.trig_multi;
        // Sticky-mask-only "ever targeted PIO1 TXF" check — triggers
        // aren't required; firmware may have programmed WRITE_ADDR
        // without arming yet.
        if c.ever_wrote_pio1_txf_mask != 0 {
            ever_txf_channels.push(ch);
        }
        if any_trig > 0 {
            triggered_channels.push(ch);
        }
        if c.transfers_issued > 0 {
            served_channels.push(ch);
        }
        let dreq_bit_for_ctrl = if treq < 64 {
            (c.dreq_observed_mask >> treq) & 1
        } else {
            0
        };
        println!(
            "{:2}  0x{:08x} 0x{:08x} 0x{:04x}  0x{:08x} {}  {:2}   {}    0x{:x}      \
             {}/{}/{}/{}/{}/{}                              {:<10} {}",
            ch,
            read_addr,
            write_addr,
            trans_count & 0xFFFF,
            ctrl,
            en,
            treq,
            busy,
            c.ever_wrote_pio1_txf_mask,
            c.trig_ctrl,
            c.trig_write_addr,
            c.trig_trans_count,
            c.trig_al2_trans,
            c.trig_al3_read_addr,
            c.trig_multi,
            c.transfers_issued,
            if dreq_bit_for_ctrl != 0 { "YES" } else { "no" },
        );
    }

    // PIO1 SM0 TX FIFO level end-state. `tx_fifo_full()` is one
    // data point; FLEVEL gives the sharp number.
    let pio1_flevel = emu.bus.read32(0x5030_0000 + 0x00C);
    let pio1_sm0_tx_level = pio1_flevel & 0xF;
    let pio1_sm0_autopush = emu.bus.pio[1].sm[0].autopush_count;

    println!();
    println!("Summary:");
    println!(
        "  ever-targeting PIO1 TXF0-3:  {:?}",
        ever_txf_channels
    );
    println!("  channels triggered:           {:?}", triggered_channels);
    println!("  channels served by engine:    {:?}", served_channels);
    println!(
        "  PIO1 SM0 TX FIFO level:       {}  (autopush_count={})",
        pio1_sm0_tx_level, pio1_sm0_autopush,
    );

    // PSRAM non-zero byte count — HLD §7(6) strict rule: the "FULL
    // DISPATCH OK" verdict requires data to have actually landed in
    // PSRAM (PSRAM non-zero OR PIO1 SM0 saw autopushes). Zero means
    // firmware is still losing bytes downstream of PIO1 TX.
    let psram_nonzero_bytes: usize = emu
        .bus
        .psram
        .as_ref()
        .map(|p| p.buffer.iter().filter(|&&b| b != 0).count())
        .unwrap_or(0);

    // Verdict. Priority order (most informative first):
    //   1. No PIO1-TXF config AND no triggers on any channel → NO DISPATCH.
    //   2. Ever-TXF set but no triggers → CONFIGURED, NEVER TRIGGERED.
    //   3. Triggered but no served xfer → TREQ / engine verdicts.
    //   4. Engine served but PIO1 SM0 didn't see FIFO traffic →
    //      ENGINE SERVED, PIO DIDN'T CONSUME.
    //   5. PIO1 SM0 saw transfers → FULL DISPATCH OK.
    //   6. Fallback: only fire if PSRAM actually has data (HLD §7(6)
    //      strict rule — "PIO1-SM0-autopush and PSRAM both zero must
    //      NOT be FULL DISPATCH OK"). This arm is normally unreachable
    //      given the preceding checks, but the explicit PSRAM guard
    //      defends against future refactors that shuffle arm order.
    let verdict = if ever_txf_channels.is_empty() && triggered_channels.is_empty() {
        "NO DISPATCH".to_string()
    } else if !ever_txf_channels.is_empty() && triggered_channels.is_empty() {
        "CONFIGURED, NEVER TRIGGERED".to_string()
    } else if !triggered_channels.is_empty() && served_channels.is_empty() {
        // Pick a representative channel for the verdict text: the first
        // triggered channel is fine — the HLD rule is "if ANY channel
        // fits the pattern".
        let ch = triggered_channels[0];
        let c = emu.bus.dma.channel(ch);
        let treq = ((c.ctrl >> 15) & 0x3F) as u8;
        let dreq_bit = if treq < 64 {
            (c.dreq_observed_mask >> treq) & 1 != 0
        } else {
            false
        };
        if !dreq_bit {
            format!(
                "TREQ never asserted — check firmware CTRL.TREQ_SEL={} or emulator DREQ plumbing for TREQ {}",
                treq, treq,
            )
        } else {
            "DREQ seen but engine did not serve — likely emulator engine gap".to_string()
        }
    } else if !served_channels.is_empty() && pio1_sm0_autopush == 0 {
        let ch = served_channels[0];
        let c = emu.bus.dma.channel(ch);
        format!(
            "ENGINE SERVED, PIO DIDN'T CONSUME (CH{} xfers={}) — \
             investigate PIO1 SM0 pull/shift or TX FIFO sink plumbing",
            ch, c.transfers_issued,
        )
    } else if pio1_sm0_autopush > 0 {
        "FULL DISPATCH OK (PIO1 SM0 saw transfers)".to_string()
    } else if psram_nonzero_bytes > 0 {
        format!(
            "FULL DISPATCH OK (PSRAM has {} non-zero bytes)",
            psram_nonzero_bytes,
        )
    } else {
        // HLD §7(6): PIO1-SM0-autopush == 0 AND PSRAM all-zero — both
        // sinks empty. Must NOT be "FULL DISPATCH OK".
        "DISPATCH UNCLEAR — PIO1 SM0 autopush == 0 and PSRAM all-zero \
         despite upstream counters — check downstream plumbing"
            .to_string()
    };
    println!();
    println!("Verdict: {}", verdict);

    // ------------------------------------------------------------------
    // PIO FIFO push accounting. Per-SM TX/RX push success vs drop
    // counters from `PioFifo`. If DMA is "served" (transfers_issued > 0)
    // but TX push_drop > 0 and push_success == 0 (or far below), the
    // bytes evaporated at the FIFO. Symmetric RX numbers catch
    // autopush-into-full-FIFO drops on the chip-side (firmware not
    // draining RXF fast enough).
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: PIO FIFO push accounting ---");
    for (pio_idx, pio) in emu.bus.pio.iter().enumerate() {
        for sm_idx in 0..4 {
            let sm = &pio.sm[sm_idx];
            println!(
                "PIO{} SM{} TX: push_ok={:<10} push_drop={:<10}    \
                 RX: push_ok={:<10} push_drop={:<10}",
                pio_idx,
                sm_idx,
                sm.tx_push_success(),
                sm.tx_push_drop(),
                sm.rx_push_success(),
                sm.rx_push_drop(),
            );
        }
    }

    // Final GPIO 0..3 state — what the PSRAM model actually sees.
    let gpio_lo = emu.bus.gpio_in & 0xF;
    println!();
    println!("--- Diag: GPIO 0..3 (PSRAM bus) ---");
    println!(
        "gpio_in[3:0]:      0x{:x}  (MISO={} CS={} SCK={} MOSI={})",
        gpio_lo,
        gpio_lo & 1,
        (gpio_lo >> 1) & 1,
        (gpio_lo >> 2) & 1,
        (gpio_lo >> 3) & 1,
    );
    println!();
    println!("--- Diag: PIO + NVIC state ---");
    println!("Core 0 PC:        0x{:08x}", emu.cores[0].regs.pc());
    println!("Core 1 PC:        0x{:08x}", emu.cores[1].regs.pc());
    println!(
        "PIO0 SM enabled:  0x{:x}    PIO1 SM enabled:  0x{:x}",
        emu.bus.read32(0x5020_0000) & 0xF,
        emu.bus.read32(0x5030_0000) & 0xF,
    );
    println!(
        "PIO0 INT0_INTE:   0x{:03x}  INT0_INTS:        0x{:03x}",
        emu.bus.read32(0x5020_012C),
        emu.bus.read32(0x5020_0134),
    );
    println!(
        "PIO0 INT1_INTE:   0x{:03x}  INT1_INTS:        0x{:03x}",
        emu.bus.read32(0x5020_0138),
        emu.bus.read32(0x5020_0140),
    );
    println!(
        "PIO0 INTR:        0x{:03x}  (raw status — IRQ[3:0] | RXNEMPTY[3:0]<<4 | TXNFULL[3:0]<<8)",
        emu.bus.read32(0x5020_0128),
    );
    println!(
        "NVIC[0].pending:  0x{:08x}  NVIC[1].pending:  0x{:08x}",
        emu.bus.nvics[0].pending,
        emu.bus.nvics[1].pending,
    );

    // PWM slice 4 state — this slice drives GUS_sample rate on PicoGUS.
    // `audio_sample_handler` runs on PWM_IRQ_WRAP (NVIC line 4) and pushes
    // the next sample to the I2S PIO TXF. If slice 4 isn't enabled or
    // wrapping, no audio samples are produced regardless of voice state.
    // PWM register offsets (RP2040 datasheet §4.5.3):
    //   EN=0xA0, INTR=0xA4, INTE=0xA8, INTF=0xAC, INTS=0xB0
    let pwm_slice4 = 0x4005_0000u32 + 0x14 * 4;
    let pwm_csr4 = emu.bus.read32(pwm_slice4 + 0x00);
    let pwm_div4 = emu.bus.read32(pwm_slice4 + 0x04);
    let pwm_ctr4 = emu.bus.read32(pwm_slice4 + 0x08);
    let pwm_top4 = emu.bus.read32(pwm_slice4 + 0x10);
    let pwm_en = emu.bus.read32(0x4005_00A0);
    let pwm_intr = emu.bus.read32(0x4005_00A4);
    let pwm_inte = emu.bus.read32(0x4005_00A8);
    let pwm_ints = emu.bus.read32(0x4005_00B0);
    println!(
        "PWM slice4 CSR:   0x{:08x}  EN={} PH_CORRECT={}",
        pwm_csr4,
        pwm_csr4 & 1,
        (pwm_csr4 >> 1) & 1
    );
    println!(
        "PWM slice4 DIV:   0x{:08x}  CTR: 0x{:04x}  TOP: 0x{:04x}",
        pwm_div4, pwm_ctr4, pwm_top4
    );
    println!(
        "PWM EN:           0x{:02x}  INTR: 0x{:02x}  INTE: 0x{:02x}  INTS: 0x{:02x}",
        pwm_en, pwm_intr, pwm_inte, pwm_ints
    );

    // ------------------------------------------------------------------
    // PIO0 SM0 deep-dive — investigates why the IOW capture program
    // never pushes into RX FIFO despite SM0 being enabled.
    //
    // Hypotheses being probed:
    //   H1: clkdiv too high → PIO ticks too slowly to see our 37-cycle
    //       ISA pulse windows.
    //   H2: DACK pin (GPIO19) is high → `jmp pin restart` always taken.
    //   H3: INSTR_MEM never loaded → SM is running NOPs / zeros.
    //
    // Register layout (PIO0_BASE = 0x5020_0000):
    //   0x004 FSTAT          per-SM TX/RX EMPTY/FULL flags
    //   0x048+i*4 INSTR_MEMi  (write-only on silicon → reads return 0)
    //   0x0C8 SM0_CLKDIV     hi16=int, lo16=frac
    //   0x0CC SM0_EXECCTRL   (bit 31 EXEC_STALLED is computed at read)
    //   0x0D4 SM0_ADDR       current PC (5-bit)
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: PIO0 SM0 deep-dive ---");
    let clkdiv = emu.bus.read32(0x5020_0000 + 0x0C8);
    let clkdiv_int = (clkdiv >> 16) & 0xFFFF;
    let clkdiv_frac = (clkdiv >> 8) & 0xFF;
    // RP2040 SMn_CLKDIV: bits [31:16]=INT, [15:8]=FRAC, [7:0]=reserved.
    // INT==0 is treated as 65536 by hardware. FRAC is x/256.
    let effective_int = if clkdiv_int == 0 { 65536 } else { clkdiv_int };
    println!(
        "SM0 CLKDIV raw:   0x{:08x}  ({}.{:03} effective ~{} sysclk/PIO-tick)",
        clkdiv, clkdiv_int, clkdiv_frac, effective_int
    );
    let sm_addr = emu.bus.read32(0x5020_0000 + 0x0D4) & 0x1F;
    println!("SM0 PC:           0x{:02x}", sm_addr);
    let execctrl = emu.bus.read32(0x5020_0000 + 0x0CC);
    println!(
        "SM0 EXECCTRL:     0x{:08x}  (bit31 EXEC_STALLED={})",
        execctrl,
        (execctrl >> 31) & 1
    );

    // INSTR_MEM via MMIO returns 0 (real-silicon-faithful), so also
    // dump the raw backing store via PioBlock::instr_mem() — that gives
    // us the truth on whether firmware actually programmed it.
    println!("INSTR_MEM (first 16 — MMIO read / raw backing):");
    // Snapshot the raw backing store first to avoid an immutable+mutable
    // borrow overlap with the subsequent read32() calls on emu.bus.
    let raw_im_snapshot: [u16; 32] = *emu.bus.pio[0].instr_mem();
    for i in 0..16usize {
        let mmio = emu.bus.read32(0x5020_0000 + 0x048 + (i as u32) * 4) & 0xFFFF;
        println!(
            "  [{:2}] mmio=0x{:04x}  raw=0x{:04x}",
            i, mmio, raw_im_snapshot[i]
        );
    }

    // DACK = GPIO19 (picogus_pins::ISA_DACK). `wait 1 gpio` /
    // `jmp pin` against this pin gates the whole PIO program.
    let gpio_in = emu.bus.gpio_in;
    let dack_high = (gpio_in >> 19) & 1 != 0;
    println!(
        "DACK (GPIO19):    {}    (gpio_in=0x{:08x})",
        if dack_high { "high" } else { "low" },
        gpio_in
    );

    // FSTAT decode for SM0. Bits per RP2040 datasheet §3.7:
    //   [3:0]   RXFULL   (sm0..sm3)
    //   [11:8]  RXEMPTY
    //   [19:16] TXFULL
    //   [27:24] TXEMPTY
    // (NB: prompt's bit map was approximate; this matches the impl in
    //  mdpicoem-common/src/pio/mod.rs::fstat().)
    let fstat = emu.bus.read32(0x5020_0000 + 0x004);
    let sm0_rxfull = (fstat >> 0) & 1;
    let sm0_rxempty = (fstat >> 8) & 1;
    let sm0_txfull = (fstat >> 16) & 1;
    let sm0_txempty = (fstat >> 24) & 1;
    // FLEVEL @ 0x00C: per-SM TX in low nibble, RX in high nibble of
    // each byte; SM0 occupies bits [7:0].
    let flevel = emu.bus.read32(0x5020_0000 + 0x00C);
    let sm0_tx_level = flevel & 0xF;
    let sm0_rx_level = (flevel >> 4) & 0xF;
    println!(
        "FSTAT:            0x{:08x}  SM0 RX lvl={} (full={} empty={})  TX lvl={} (full={} empty={})",
        fstat, sm0_rx_level, sm0_rxfull, sm0_rxempty, sm0_tx_level, sm0_txfull, sm0_txempty
    );
    // FSTAT per-SM bit-decode for all four SMs (RXFULL/RXEMPTY/TXFULL/TXEMPTY).
    for sm in 0..4u32 {
        let rxfull = (fstat >> sm) & 1;
        let rxempty = (fstat >> (8 + sm)) & 1;
        let txfull = (fstat >> (16 + sm)) & 1;
        let txempty = (fstat >> (24 + sm)) & 1;
        println!(
            "FSTAT SM{}:        RX full={} empty={}   TX full={} empty={}",
            sm, rxfull, rxempty, txfull, txempty
        );
    }

    // ------------------------------------------------------------------
    // Per-SM register sweep — the SM0 deep-dive above showed SM0 stuck
    // at WAIT 0 GPIO 4 (PC=0x0b). This block extends the picture to the
    // other three SMs in PIO0 to see whether anything else is running
    // (and thus might be stomping pad_oe[4] or otherwise interfering).
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: PIO0 per-SM sweep ---");
    for sm in 0..4u32 {
        let base = 0x5020_0000 + 0x0C8 + sm * 0x18;
        let cd = emu.bus.read32(base);
        let cd_int = (cd >> 16) & 0xFFFF;
        let cd_frac = (cd >> 8) & 0xFF;
        let cd_eff = if cd_int == 0 { 65536 } else { cd_int };
        let ec = emu.bus.read32(base + 0x04);
        let pc = emu.bus.read32(base + 0x0C) & 0x1F;
        let pin = emu.bus.read32(base + 0x14);
        let sideset_count = (pin >> 29) & 0x7;
        let set_count = (pin >> 26) & 0x7;
        let out_count = (pin >> 20) & 0x3F;
        let in_base = (pin >> 15) & 0x1F;
        let sideset_base = (pin >> 10) & 0x1F;
        let set_base = (pin >> 5) & 0x1F;
        let out_base = pin & 0x1F;
        println!(
            "SM{} PC: 0x{:02x}  CLKDIV: 0x{:08x} ({}.{:03} eff={})  EXECCTRL: 0x{:08x}",
            sm, pc, cd, cd_int, cd_frac, cd_eff, ec
        );
        println!(
            "       PINCTRL: 0x{:08x}  SIDESET cnt={} base={}  SET cnt={} base={}  OUT cnt={} base={}  IN base={}",
            pin, sideset_count, sideset_base, set_count, set_base, out_count, out_base, in_base
        );
        // SHIFTCTRL @ +0x08 from CLKDIV. Bits per RP2040 datasheet §3.7:
        //   [16] AUTOPUSH       [17] AUTOPULL
        //   [18] IN_SHIFTDIR    [19] OUT_SHIFTDIR  (0=left, 1=right)
        //   [20:24] PUSH_THRESH (0 means 32, otherwise N)
        //   [25:29] PULL_THRESH (0 means 32, otherwise N)
        // PIO0 firmware uses autopush at threshold 18 (10 addr + 8 data).
        let sc = emu.bus.read32(base + 0x08);
        let autopush = (sc >> 16) & 1;
        let autopull = (sc >> 17) & 1;
        let in_shiftdir = (sc >> 18) & 1;
        let out_shiftdir = (sc >> 19) & 1;
        let push_thresh_raw = (sc >> 20) & 0x1F;
        let pull_thresh_raw = (sc >> 25) & 0x1F;
        let push_thresh = if push_thresh_raw == 0 { 32 } else { push_thresh_raw };
        let pull_thresh = if pull_thresh_raw == 0 { 32 } else { pull_thresh_raw };
        println!(
            "       SHIFTCTRL: 0x{:08x}  AUTOPUSH={} PUSH_THRESH={}  AUTOPULL={} PULL_THRESH={}  IN_SHIFTDIR={} OUT_SHIFTDIR={}",
            sc, autopush, push_thresh, autopull, pull_thresh, in_shiftdir, out_shiftdir
        );
    }

    // PIO0 pad_out / pad_oe — if PIO0 is driving GPIO 4 as an output
    // (pad_oe bit 4 set), the WAIT 1 / WAIT 0 IOW pattern is being
    // shorted by PIO0 itself, and the harness override gets ignored
    // upstream of PIO's view of gpio_in (PIO sees its own pad_out
    // merged in). Bit 4 highlight is the smoking gun for that case.
    let p0_pad_oe = emu.bus.pio[0].pad_oe;
    let p0_pad_out = emu.bus.pio[0].pad_out;
    let p0_drives_iow = (p0_pad_oe >> 4) & 1 != 0;
    println!();
    println!("--- Diag: PIO0 pad state ---");
    println!(
        "PIO0 pad_oe:      0x{:08x}  bit4(IOW)={} {}",
        p0_pad_oe,
        (p0_pad_oe >> 4) & 1,
        if p0_drives_iow {
            "*** PIO0 driving IOW as output — conflicts with harness override ***"
        } else {
            ""
        }
    );
    println!(
        "PIO0 pad_out:     0x{:08x}  bit4(IOW)={}",
        p0_pad_out,
        (p0_pad_out >> 4) & 1,
    );

    // SIO GPIO at bit 4 — the harness expects SIO not to be driving IOW
    // (SDK firmware should leave the pad SIO-disabled / OE=0). If SIO is
    // driving IOW high, the merged value before the override would also
    // be high, but the override step in `update_gpio` should still win.
    let sio_oe4 = (emu.bus.sio.gpio_oe >> 4) & 1;
    let sio_out4 = (emu.bus.sio.gpio_out >> 4) & 1;
    println!();
    println!("--- Diag: SIO GPIO 4 ---");
    println!(
        "SIO gpio_oe[4]:   {}    SIO gpio_out[4]:  {}",
        sio_oe4, sio_out4
    );

    // External override sanity check — if the harness has dropped its
    // override before this point (e.g. by hitting reset()), `gpio_in`
    // has nothing forcing IOW low and the WAIT 0 GPIO 4 will sit
    // forever. Confirm both mask and override level for bit 4.
    let ext_mask4 = (emu.bus.external_gpio_in_mask >> 4) & 1;
    let ext_ovr4 = (emu.bus.external_gpio_in_override >> 4) & 1;
    println!(
        "ext mask[4]:      {}    ext override[4]:  {}    (full mask=0x{:08x} override=0x{:08x})",
        ext_mask4,
        ext_ovr4,
        emu.bus.external_gpio_in_mask,
        emu.bus.external_gpio_in_override,
    );

    // PIO ticks vs IOW-low ticks — the smoking gun. If
    // pio_tick_iow_low_count == 0 across the whole run, PIO never
    // observed IOW asserted low and the bug is upstream (override
    // merge order, harness sequencing, etc.). If non-zero, PIO did see
    // IOW low and the bug is inside the PIO program / decode itself.
    println!();
    println!("--- Diag: PIO tick counters (slow path only) ---");
    println!("PIO ticks total:           {}", emu.pio_tick_count);
    println!(
        "PIO ticks with IOW low:    {}  ({:.4}%)",
        emu.pio_tick_iow_low_count,
        if emu.pio_tick_count > 0 {
            100.0 * emu.pio_tick_iow_low_count as f64 / emu.pio_tick_count as f64
        } else {
            0.0
        }
    );

    // Triage matrix for the SM0 sample-at-PC=11 question:
    //   max_pc <= 0x0B && advances ≈ 1 → genuinely stuck on WAIT 0
    //   max_pc >  0x0B && autopushes > 0 → SM is running; bug is downstream
    //   max_pc >  0x0B && autopushes == 0 → SM cleared the WAIT but
    //                                        stalls before IN PINS lands
    let sm0_autopush = emu.bus.pio[0].sm[0].autopush_count;
    println!(
        "PIO0 SM0 max PC:           0x{:02x}",
        emu.pio0_sm0_max_pc
    );
    println!(
        "PIO0 SM0 PC advances:      {}",
        emu.pio0_sm0_pc_advances
    );
    println!("PIO0 SM0 autopushes:       {}", sm0_autopush);

    // Persist the captured audio. Reject directories and create any
    // missing parent dirs (handled inside `write_wav`).
    let inferred_rate = capture.inferred_sample_rate_hz();
    let wav_rate = inferred_rate
        .map(|r| r.round() as u32)
        .filter(|r| *r > 0)
        .unwrap_or(44_100);
    capture
        .write_wav(&out_path, wav_rate)
        .map_err(|e| format!("writing WAV to {}: {e}", out_path.display()))?;
    let wav_bytes = std::fs::metadata(&out_path).map(|m| m.len()).unwrap_or(0);

    // ------------------------------------------------------------------
    // PicoGUS ISA capture coverage (HLD "PicoGUS Capture Coverage
    // Diagnostic" Rev. 1 §5). Per-class `(fired, captured, misattr,
    // drop%)` table plus decile clustering and ground-truth cross-check.
    // ------------------------------------------------------------------
    println!();
    println!("--- Diag: PicoGUS ISA capture coverage ---");
    println!(
        "{:<22} {:>9}  {:>9}  {:>8}  {:>7}",
        "Event axis", "fired", "captured", "misattr", "drop%"
    );
    let mut total_fired: u64 = 0;
    let mut total_captured: u64 = 0;
    let mut total_misattr: u64 = 0;
    for (class, stats) in coverage.classes.iter() {
        let drop_pct = if stats.fired > 0 {
            100.0 * (1.0 - (stats.captured as f64 / stats.fired as f64))
        } else {
            0.0
        };
        println!(
            "{:<22} {:>9}  {:>9}  {:>8}  {:>6.1}%",
            class.label(),
            stats.fired,
            stats.captured,
            stats.misattributed,
            drop_pct,
        );
        total_fired += stats.fired;
        total_captured += stats.captured;
        total_misattr += stats.misattributed;
    }
    println!(
        "{:<22} {:>9}  {:>9}  {:>8}",
        "                 ----", "--------", "--------", "-------"
    );
    let total_drop_pct = if total_fired > 0 {
        100.0 * (1.0 - (total_captured as f64 / total_fired as f64))
    } else {
        0.0
    };
    println!(
        "{:<22} {:>9}  {:>9}  {:>8}  {:>6.1}%",
        "total", total_fired, total_captured, total_misattr, total_drop_pct
    );
    println!(
        "post_roll_orphans: {}  catch_up_unattributed: {}",
        coverage.post_roll_orphans, coverage.catch_up_unattributed
    );
    println!();
    println!("Clustering (per trace decile, captured / fired):");
    for d in 0..10usize {
        let fired = coverage.decile_fired[d];
        let captured = coverage.decile_captured[d];
        let pct = if fired > 0 {
            100.0 * captured as f64 / fired as f64
        } else {
            0.0
        };
        println!(
            "  [{:2}-{:2}%]  {:>6} / {:>6}  ({:>5.1}%)",
            d * 10,
            (d + 1) * 10,
            captured,
            fired,
            pct,
        );
    }
    let sm0_autopush_ground_truth = emu.bus.pio[0].sm[0].autopush_count;
    println!(
        "autopush_count ground truth (SM0): {}",
        sm0_autopush_ground_truth
    );
    // HLD §7 acceptance (4): captured_sum + misattr_sum + post_roll_orphans
    // should equal the PIO0 SM0 ground-truth autopush count.
    // HLD §7 (5): fired_sum == summary.writes_fired × (1 + write16_share).
    let reconciliation_total =
        total_captured + total_misattr + coverage.post_roll_orphans;
    println!(
        "reconciliation: captured({}) + misattr({}) + orphans({}) = {}  \
         vs. autopush={}  (Δ={})",
        total_captured,
        total_misattr,
        coverage.post_roll_orphans,
        reconciliation_total,
        sm0_autopush_ground_truth,
        sm0_autopush_ground_truth as i64 - reconciliation_total as i64,
    );
    println!(
        "fired sub-events: {}  writes_fired: {}",
        coverage.fired_sub_events, summary.writes_fired,
    );
    // PIO0 SM0 RX FIFO overflow drops — if non-zero, the harness
    // was firing ISA events faster than the firmware could drain
    // and the trace replay is incomplete. With PICOGUS_IDLE_CYCLES
    // tuned high enough this should stay at 0.
    println!(
        "pio0 sm0 rx fifo drops: {}",
        emu.bus.pio[0].sm[0].rx_fifo_drops()
    );

    // --- SRAM scan: byte candidates for GUS_reset_reg ----------------
    // GUS_reset_reg is a static uint8_t in the PicoGUS firmware
    // (gus-x.cpp). After the init sequence in the Monkey Island trace
    // it should be 0x07. We don't have symbols, so scan SRAM for bytes
    // equal to 0x07 and 0x01 to give ourselves candidate addresses.
    // This is diagnostic only — gated on PICOGUS_SRAM_SCAN=1.
    // Probe specific addresses (comma-separated hex list in PICOGUS_PROBE).
    if let Ok(list) = std::env::var("PICOGUS_PROBE") {
        eprintln!("=== probe list ===");
        for tok in list.split(',') {
            let tok = tok.trim().trim_start_matches("0x");
            if let Ok(a) = u32::from_str_radix(tok, 16) {
                let w = emu.peek(a & !3);
                let b = ((w >> ((a & 3) * 8)) & 0xFF) as u8;
                eprintln!("probe  0x{:08x} = 0x{:02x} (word 0x{:08x}: 0x{:08x})", a, b, a & !3, w);
            }
        }
    }
    // Structured GUS state dump using symbol addresses from
    // wrk_scratch/picogus-rebuild/pg-gus.elf (rebuild v1, 2026-04-22).
    // Gated on PICOGUS_DUMP_GUS=1. Addresses are build-specific —
    // bump them after every firmware rebuild via
    // `arm-none-eabi-nm --defined-only pg-gus.elf | grep -E 'GUS_reset_reg|myGUS|guschan'`
    // and the DWARF member offsets if the class layout changes.
    if std::env::var("PICOGUS_DUMP_GUS").ok().as_deref() == Some("1") {
        // Byte read helper: peek word and slice the byte at `addr`.
        let peek_u8 = |addr: u32| -> u8 {
            let w = emu.peek(addr & !3);
            ((w >> ((addr & 3) * 8)) & 0xff) as u8
        };
        let peek_u16 = |addr: u32| -> u16 {
            let lo = peek_u8(addr) as u16;
            let hi = peek_u8(addr + 1) as u16;
            lo | (hi << 8)
        };
        let peek_u32 = |addr: u32| -> u32 {
            // For aligned u32 reads this is a single word peek; the
            // guschan[] pointers and the GUSChannels u32 fields are
            // aligned on 4 in the ELF layout, so this is safe.
            emu.peek(addr)
        };

        const GUS_RESET_REG: u32 = 0x2001_d976;
        const MYGUS_BASE: u32 = 0x2001_7688;
        // GFGus struct offsets (from DWARF, pg-gus.elf rebuild v1):
        const MYGUS_GCURCHANNEL_OFF: u32 = 12; // u16
        const MYGUS_MIXCONTROL_OFF: u32 = 35;  // u8
        const MYGUS_ACTIVE_CHANNELS_OFF: u32 = 36; // u8
        const MYGUS_IRQSTATUS_OFF: u32 = 100;  // u8
        const GUSCHAN_BASE: u32 = 0x2001_7c3c;
        // volctrl_t struct at 0x20016ba8; `.gus` member at offset 32.
        const VOLUME_BASE: u32 = 0x2001_6ba8;
        const VOLUME_GUS_OFF: u32 = 32;

        eprintln!();
        eprintln!("=== GUS state dump (PICOGUS_DUMP_GUS=1) ===");
        let reset = peek_u8(GUS_RESET_REG);
        let active = peek_u8(MYGUS_BASE + MYGUS_ACTIVE_CHANNELS_OFF);
        let cur_chan = peek_u16(MYGUS_BASE + MYGUS_GCURCHANNEL_OFF);
        let mix_ctrl = peek_u8(MYGUS_BASE + MYGUS_MIXCONTROL_OFF);
        let irq_status = peek_u8(MYGUS_BASE + MYGUS_IRQSTATUS_OFF);
        let volume_gus = peek_u32(VOLUME_BASE + VOLUME_GUS_OFF);
        eprintln!(
            "GUS_reset_reg @ 0x{:08x} = 0x{:02x}  (needs 0x03 set for non-silent output; 0x07 = reset+DAC+IRQ)",
            GUS_RESET_REG, reset,
        );
        eprintln!(
            "myGUS.ActiveChannels @ 0x{:08x} = {} (0x{:02x})",
            MYGUS_BASE + MYGUS_ACTIVE_CHANNELS_OFF,
            active,
            active,
        );
        eprintln!(
            "myGUS.gCurChannel @ 0x{:08x} = {} (0x{:04x})   (last voice-select write)",
            MYGUS_BASE + MYGUS_GCURCHANNEL_OFF,
            cur_chan,
            cur_chan,
        );
        eprintln!(
            "myGUS.mixControl @ 0x{:08x} = 0x{:02x}",
            MYGUS_BASE + MYGUS_MIXCONTROL_OFF,
            mix_ctrl,
        );
        eprintln!(
            "myGUS.IRQStatus @ 0x{:08x} = 0x{:02x}",
            MYGUS_BASE + MYGUS_IRQSTATUS_OFF,
            irq_status,
        );
        eprintln!(
            "volume.gus @ 0x{:08x} = 0x{:08x} ({})    (0 → mixer output forced silent)",
            VOLUME_BASE + VOLUME_GUS_OFF,
            volume_gus,
            volume_gus as i32,
        );
        let gate_open = (reset & 0x03) == 0x03;
        eprintln!(
            "  → audio gate (reset_reg & 0x03 == 0x03): {}",
            if gate_open { "OPEN" } else { "CLOSED — GUS_sample_stereo() returns 0" },
        );

        // Voice 0..3 — enough to catch the Monkey Island config
        // (trace only activates a handful of voices in the first
        // seconds past t=3.486 s). Expand to 32 by adjusting the range.
        let voice_count = std::env::var("PICOGUS_DUMP_GUS_VOICES")
            .ok()
            .and_then(|s| s.parse::<u32>().ok())
            .unwrap_or(4);
        for c in 0..voice_count.min(32) {
            let slot = GUSCHAN_BASE + c * 4;
            let ptr = peek_u32(slot);
            eprintln!();
            eprintln!("voice {}: guschan[{}] @ 0x{:08x} = 0x{:08x}", c, c, slot, ptr);
            if ptr < 0x2000_0000 || ptr >= 0x2004_2000 {
                eprintln!("  (pointer not in SRAM range — voice not constructed)");
                continue;
            }
            // GUSChannels struct members (byte_size = 100)
            let wave_start  = peek_u32(ptr + 0);
            let wave_end    = peek_u32(ptr + 4);
            let wave_addr   = peek_u32(ptr + 8);
            let wave_add    = peek_u32(ptr + 12);
            let wave_ctrl   = peek_u16(ptr + 16);
            let wave_freq   = peek_u16(ptr + 18);
            let ramp_start  = peek_u32(ptr + 20);
            let ramp_end    = peek_u32(ptr + 24);
            let ramp_vol    = peek_u32(ptr + 28);
            let ramp_add    = peek_u32(ptr + 32);
            let ramp_rate   = peek_u8(ptr + 36);
            let ramp_ctrl   = peek_u8(ptr + 37);
            let pan_pot     = peek_u8(ptr + 38);
            let channum     = peek_u8(ptr + 39);
            let pan_left    = peek_u32(ptr + 44);
            let pan_right   = peek_u32(ptr + 48);
            let vol_left    = peek_u32(ptr + 52);
            let vol_right   = peek_u32(ptr + 56);
            // useAddr = WaveAddr >> WAVE_FRACT(10); same for start/end
            let use_addr  = wave_addr  >> 10;
            let use_start = wave_start >> 10;
            let use_end   = wave_end   >> 10;
            eprintln!(
                "  WaveCtrl=0x{:02x}  RampCtrl=0x{:02x}  channum={}  PanPot=0x{:02x}",
                wave_ctrl as u8, ramp_ctrl, channum, pan_pot,
            );
            eprintln!(
                "  WaveCtrl bits: 0x{:04x} (bit0=stopped? bit1=stop_on_end bit6=loop bit7=data_sample16)",
                wave_ctrl,
            );
            eprintln!(
                "  WaveStart=0x{:08x} (useStart=0x{:05x})  WaveEnd=0x{:08x} (useEnd=0x{:05x})",
                wave_start, use_start, wave_end, use_end,
            );
            eprintln!(
                "  WaveAddr =0x{:08x} (useAddr =0x{:05x})  WaveAdd=0x{:08x}  WaveFreq=0x{:04x}",
                wave_addr, use_addr, wave_add, wave_freq,
            );
            eprintln!(
                "  RampStart=0x{:08x}  RampEnd=0x{:08x}  RampVol=0x{:08x} (idx>>10={})",
                ramp_start, ramp_end, ramp_vol, ramp_vol >> 10,
            );
            eprintln!(
                "  RampAdd=0x{:08x}  RampRate=0x{:02x}",
                ramp_add, ramp_rate,
            );
            eprintln!(
                "  PanLeft=0x{:08x}  PanRight=0x{:08x}  VolLeft=0x{:08x}  VolRight=0x{:08x}",
                pan_left, pan_right, vol_left, vol_right,
            );
            // Readable "why silent?" classifier.
            // Per gus-x.cpp:613 GUSChannels::generateSample, the voice
            // ALWAYS contributes `tmpsamp * VolLeft/VolRight` to the
            // output accumulator — there is no early-return on stopped.
            // So if both VolLeft and VolRight are zero, the voice can
            // not contribute a non-zero sample regardless of PSRAM data.
            let wave_stopped = (wave_ctrl & 0x01) != 0;
            let ramp_halted = (ramp_ctrl & 0x01) != 0;
            let in_range = use_addr >= use_start && use_addr <= use_end;
            let vol_zero = vol_left == 0 && vol_right == 0;
            eprintln!(
                "  state: wave_stopped(WaveCtrl&1)={}  ramp_halted(RampCtrl&1)={}  addr_in_range={}",
                wave_stopped, ramp_halted, in_range,
            );
            if vol_zero {
                eprintln!(
                    "  SILENT: VolLeft=VolRight=0  (volume.gus might be 0, or RampVol-PanX clamped to 0, or vol16bit[idx]=0)",
                );
            }
            // sample_cache at offset 60 in GUSChannels:
            //   data[32] at +0, addr (uint32) at +32, addr_next (uint32) at +36.
            // If the voice is actively reading PSRAM, data[] should hold the
            // last 32 bytes of sample data pulled for that voice. If it's all
            // zeros despite PSRAM preseed, the PSRAM read path itself is
            // returning zeros and that's the silence cause.
            let sc_base = ptr + 60;
            let sc_addr = peek_u32(sc_base + 32);
            let sc_addr_next = peek_u32(sc_base + 36);
            let mut data = [0u8; 32];
            for i in 0..32 {
                data[i] = peek_u8(sc_base + i as u32);
            }
            let nz = data.iter().filter(|&&b| b != 0).count();
            eprintln!(
                "  sample_cache.addr=0x{:08x} addr_next=0x{:08x}  data[0..16]: {}",
                sc_addr as i32, sc_addr_next as i32,
                data[..16]
                    .iter()
                    .map(|b| format!("{:02x}", b))
                    .collect::<Vec<_>>()
                    .join(" "),
            );
            eprintln!(
                "  sample_cache.data non-zero: {}/32",
                nz,
            );
        }
        eprintln!();
    }

    if std::env::var("PICOGUS_SRAM_SCAN").ok().as_deref() == Some("1") {
        eprintln!();
        eprintln!("=== SRAM scan (PICOGUS_SRAM_SCAN=1) ===");
        let mut count_by_value: [u32; 256] = [0; 256];
        let mut sevens: Vec<u32> = Vec::new();
        let mut ones: Vec<u32> = Vec::new();
        let sram_base = 0x2000_0000u32;
        let sram_end = 0x2004_2000u32;
        let mut addr = sram_base;
        while addr < sram_end {
            let w = emu.peek(addr);
            for i in 0..4u32 {
                let b = ((w >> (i * 8)) & 0xff) as u8;
                count_by_value[b as usize] += 1;
                if b == 0x07 {
                    sevens.push(addr + i);
                }
                if b == 0x01 {
                    ones.push(addr + i);
                }
            }
            addr += 4;
        }
        eprintln!(
            "SRAM scan: 0x00 count={}  0x01 count={}  0x07 count={}  total bytes={}",
            count_by_value[0x00], count_by_value[0x01], count_by_value[0x07],
            (sram_end - sram_base) as u64
        );
        // Dump candidate address lists to files for easy diffing.
        let dump_path = std::env::var("PICOGUS_SRAM_DUMP")
            .unwrap_or_else(|_| "/tmp/pgd_sram_scan.txt".to_string());
        {
            use std::io::Write;
            if let Ok(mut f) = std::fs::File::create(&dump_path) {
                writeln!(f, "# value=0x07 count={}", sevens.len()).ok();
                for a in &sevens {
                    writeln!(f, "07  0x{:08x}", a).ok();
                }
                writeln!(f, "# value=0x01 count={}", ones.len()).ok();
                for a in &ones {
                    writeln!(f, "01  0x{:08x}", a).ok();
                }
            }
            eprintln!("SRAM scan dumped to {dump_path}");
        }
        // First 32 of each for stderr readability.
        let shown = sevens.len().min(32);
        eprintln!("first {shown} addrs with byte=0x07:");
        for a in sevens.iter().take(shown) {
            eprintln!("  0x{:08x}", a);
        }
    }

    println!();
    println!("=== picogus_diff_rp2040 summary ===");
    println!("Events total:     {}", summary.events_total);
    println!("Writes fired:     {}", summary.writes_fired);
    println!("Reads skipped:    {}", summary.reads_skipped);
    println!("Duration capped:  {}", summary.duration_capped);
    println!("Stall events:     {}", summary.stall_events);
    println!(
        "Final sim time:   {} ns ({:.3} s)",
        summary.final_sim_ns,
        summary.final_sim_ns as f64 / 1e9
    );
    println!(
        "Post-roll:        {} cycles ({:.3} s @ {} Hz)",
        summary.post_roll_cycles,
        summary.post_roll_cycles as f64 / final_sys_clk_hz as f64,
        final_sys_clk_hz
    );
    println!("Final cycles:     {}", summary.final_cycles + summary.post_roll_cycles);
    println!("Core 1 halted:    {}", core1_halted);
    println!("Core 1 PC:        0x{:08x}", core1_pc);
    println!("Wall elapsed:     {:.3} s", wall_elapsed.as_secs_f64());
    if wall_elapsed.as_secs_f64() > 0.0 {
        let sim_s = summary.final_sim_ns as f64 / 1e9;
        println!(
            "Ratio (sim/wall): {:.3}x",
            sim_s / wall_elapsed.as_secs_f64()
        );
    }

    println!();
    println!("--- I2S capture ---");
    println!("WAV path:         {}", out_path.display());
    println!("WAV size:         {} bytes", wav_bytes);
    println!("Frames:           {}", capture.frames().len());
    println!("LRCLK edges:      {}", capture.lrclk_edge_count());
    match inferred_rate {
        Some(rate) => println!(
            "Sample rate:      {:.1} Hz (inferred from LRCLK)",
            rate
        ),
        None => println!(
            "Sample rate:      — (no LRCLK activity; header stamped {} Hz)",
            wav_rate
        ),
    }
    let duration = capture.duration_secs(wav_rate);
    println!("Audio duration:   {:.3} s", duration);
    if capture.lrclk_edge_count() == 0 {
        println!("(no I2S output detected — WAV contains only the 44-byte header)");
    }
    Ok(())
}

fn load_flash(path: &Path) -> Result<Vec<u8>, String> {
    let bytes = std::fs::read(path)
        .map_err(|e| format!("reading flash {}: {e}", path.display()))?;
    if bytes.len() > 2 * 1024 * 1024 {
        eprintln!(
            "warning: flash image {} bytes exceeds 2 MB window; will be clamped",
            bytes.len()
        );
    }
    Ok(bytes)
}

// ============================================================================
// PIO instruction disassembler
// ============================================================================

/// Decode a 16-bit RP2040 PIO instruction word into a human-readable
/// mnemonic. Format follows the RP2040 datasheet §3.4:
///
/// ```text
/// bits[15:13]  OP    (0=JMP, 1=WAIT, 2=IN, 3=OUT, 4=PUSH/PULL,
///                     5=MOV, 6=IRQ, 7=SET)
/// bits[12:8]   DELAY/SIDESET (5 bits)
/// bits[7:0]    instruction body
/// ```
///
/// Side-set/delay is always rendered in `[n]` form even if SIDESET is
/// programmed — this is a *legibility* disassembler, not a faithful
/// reassembler. Fine-grained SIDESET vs DELAY partitioning lives in
/// PIO1 SM0's PINCTRL register and isn't worth threading through here
/// for diagnostic dumps.
pub fn disasm_pio_instr(word: u16) -> String {
    let op = (word >> 13) & 0x7;
    let delay_ss = (word >> 8) & 0x1F;
    let body = word & 0xFF;
    let tail = if delay_ss != 0 {
        format!(" [{}]", delay_ss)
    } else {
        String::new()
    };
    match op {
        0 => {
            // JMP cond, addr
            let cond = (body >> 5) & 0x7;
            let addr = body & 0x1F;
            let cond_s = match cond {
                0 => "",
                1 => "!x, ",
                2 => "x--, ",
                3 => "!y, ",
                4 => "y--, ",
                5 => "x!=y, ",
                6 => "pin, ",
                7 => "!osre, ",
                _ => unreachable!(),
            };
            format!("JMP {}0x{:02x}{}", cond_s, addr, tail)
        }
        1 => {
            // WAIT pol src idx
            let pol = (body >> 7) & 1;
            let src = (body >> 5) & 0x3;
            let idx = body & 0x1F;
            let src_s = match src {
                0 => "GPIO",
                1 => "PIN",
                2 => "IRQ",
                3 => "RSVD3",
                _ => unreachable!(),
            };
            format!("WAIT {} {} {}{}", pol, src_s, idx, tail)
        }
        2 => {
            // IN src, bit_count
            let src = (body >> 5) & 0x7;
            let n_raw = body & 0x1F;
            let n = if n_raw == 0 { 32 } else { n_raw };
            let src_s = match src {
                0 => "PINS",
                1 => "X",
                2 => "Y",
                3 => "NULL",
                4 => "RSVD4",
                5 => "RSVD5",
                6 => "ISR",
                7 => "OSR",
                _ => unreachable!(),
            };
            format!("IN {}, {}{}", src_s, n, tail)
        }
        3 => {
            // OUT dst, bit_count
            let dst = (body >> 5) & 0x7;
            let n_raw = body & 0x1F;
            let n = if n_raw == 0 { 32 } else { n_raw };
            let dst_s = match dst {
                0 => "PINS",
                1 => "X",
                2 => "Y",
                3 => "NULL",
                4 => "PINDIRS",
                5 => "PC",
                6 => "ISR",
                7 => "EXEC",
                _ => unreachable!(),
            };
            format!("OUT {}, {}{}", dst_s, n, tail)
        }
        4 => {
            // PUSH / PULL — bit[7] selects direction
            let is_pull = (body >> 7) & 1 != 0;
            let iff = (body >> 6) & 1 != 0;
            let blk = (body >> 5) & 1 != 0;
            let iff_s = if iff {
                if is_pull {
                    "IFEMPTY "
                } else {
                    "IFFULL "
                }
            } else {
                ""
            };
            let blk_s = if blk { "BLOCK" } else { "NOBLOCK" };
            if is_pull {
                format!("PULL {}{}{}", iff_s, blk_s, tail)
            } else {
                format!("PUSH {}{}{}", iff_s, blk_s, tail)
            }
        }
        5 => {
            // MOV dst, op, src
            let dst = (body >> 5) & 0x7;
            let mop = (body >> 3) & 0x3;
            let src = body & 0x7;
            let dst_s = match dst {
                0 => "PINS",
                1 => "X",
                2 => "Y",
                3 => "RSVD3",
                4 => "EXEC",
                5 => "PC",
                6 => "STATUS",
                7 => "ISR",
                _ => unreachable!(),
            };
            let src_s = match src {
                0 => "PINS",
                1 => "X",
                2 => "Y",
                3 => "NULL",
                4 => "RSVD4",
                5 => "STATUS",
                6 => "ISR",
                7 => "OSR",
                _ => unreachable!(),
            };
            let op_s = match mop {
                0 => "",
                1 => "~",
                2 => "::",
                3 => "RSVD3:",
                _ => unreachable!(),
            };
            format!("MOV {}, {}{}{}", dst_s, op_s, src_s, tail)
        }
        6 => {
            // IRQ — bit[6] CLR, bit[5] WAIT, bit[4] REL, bits[3:0] idx
            let clr = (body >> 6) & 1 != 0;
            let wait = (body >> 5) & 1 != 0;
            let rel = (body >> 4) & 1 != 0;
            let idx = body & 0xF;
            let mode = if clr {
                "CLR"
            } else if wait {
                "WAIT"
            } else {
                "SET"
            };
            let rel_s = if rel { " REL" } else { "" };
            format!("IRQ {} {}{}{}", mode, idx, rel_s, tail)
        }
        7 => {
            // SET dst, val
            let dst = (body >> 5) & 0x7;
            let val = body & 0x1F;
            let dst_s = match dst {
                0 => "PINS",
                1 => "X",
                2 => "Y",
                3 => "RSVD3",
                4 => "PINDIRS",
                5 => "RSVD5",
                6 => "RSVD6",
                7 => "RSVD7",
                _ => unreachable!(),
            };
            format!("SET {}, {}{}", dst_s, val, tail)
        }
        _ => unreachable!(),
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_fixture_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures")
            .join("sample_gus.trace")
    }

    #[test]
    fn parse_sample_fixture() {
        let text = std::fs::read_to_string(sample_fixture_path())
            .expect("sample_gus.trace missing");
        let events = parse_trace(&text).expect("parse");
        assert_eq!(events.len(), 19, "expected 19 events in sample fixture");
        assert_eq!(events[0].ns, 0);

        let mut counts = [0usize; 4];
        for ev in &events {
            match ev.kind {
                TraceKind::Write8 => counts[0] += 1,
                TraceKind::Write16 => counts[1] += 1,
                TraceKind::Read8 => counts[2] += 1,
                TraceKind::Read16 => counts[3] += 1,
            }
        }
        // write8 + write16 + read8 + read16 = 19, all kinds represented.
        assert_eq!(counts.iter().sum::<usize>(), 19);
        assert!(counts[0] > 0, "no write8 events");
        assert!(counts[1] > 0, "no write16 events");
        assert!(counts[2] > 0, "no read8 events");
        assert!(counts[3] > 0, "no read16 events");
    }

    #[test]
    fn parse_monkey_island_fixture() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures")
            .join("monkey_island_theme.trace");
        let text = std::fs::read_to_string(&path)
            .expect("monkey_island_theme.trace missing");
        let events = parse_trace(&text).expect("parse monkey_island_theme.trace");

        // 30 s of Monkey Island theme via GUS: init + ~100 KB DRAM upload
        // + 246 note-on events = ~500K total trace events.
        assert!(
            events.len() > 100_000,
            "expected >100K events, got {}",
            events.len()
        );

        // Timespan: must span at least 20 s of the theme.
        let span_ns = events.last().unwrap().ns - events.first().unwrap().ns;
        assert!(
            span_ns > 20_000_000_000,
            "expected >20 s span, got {:.1} s",
            span_ns as f64 / 1e9
        );

        // Must contain DRAM uploads (port 0x347) — real .pat waveform data.
        let dram_writes = events.iter().filter(|e| e.port == 0x347).count();
        assert!(
            dram_writes >= 50_000,
            "expected >=50K DRAM writes (patch waveforms), got {}",
            dram_writes
        );

        // Must contain both write8 and write16 kinds.
        let w8 = events.iter().filter(|e| e.kind == TraceKind::Write8).count();
        let w16 = events.iter().filter(|e| e.kind == TraceKind::Write16).count();
        assert!(w8 > 0 && w16 > 0, "need both write8 ({w8}) and write16 ({w16})");
    }

    #[test]
    fn parse_rejects_non_monotonic_timestamps() {
        let text = "\
# picogus-tap v1
ns,port,value,kind
100,0x240,0x00,write8
50,0x240,0x01,write8
";
        let err = parse_trace(text).expect_err("should reject backwards ns");
        assert!(
            err.contains("non-monotonic"),
            "unexpected error message: {err}"
        );
    }

    #[test]
    fn parse_rejects_unknown_kind() {
        let text = "\
# picogus-tap v1
ns,port,value,kind
0,0x240,0xdeadbeef,write32
";
        let err = parse_trace(text).expect_err("should reject write32");
        assert!(err.contains("write32"), "unexpected error: {err}");
    }

    #[test]
    fn parse_skips_header_and_comment() {
        let text = "\
# picogus-tap v1
ns,port,value,kind
# mid-file comment, should be ignored
0,0x240,0xff,write8
";
        let events = parse_trace(text).expect("parse");
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].ns, 0);
        assert_eq!(events[0].port, 0x240);
        assert_eq!(events[0].value, 0xff);
        assert_eq!(events[0].kind, TraceKind::Write8);
    }

    #[test]
    fn parse_rejects_missing_header() {
        // Missing the column header line after the magic.
        let text = "\
# picogus-tap v1
0,0x240,0xff,write8
";
        let err = parse_trace(text).expect_err("should reject missing header");
        assert!(err.contains("column header"), "unexpected: {err}");
    }

    #[test]
    fn parse_rejects_value_too_wide_for_kind() {
        let text = "\
# picogus-tap v1
ns,port,value,kind
0,0x240,0x1ff,write8
";
        let err = parse_trace(text).expect_err("value 0x1ff > 0xff for write8");
        assert!(err.contains("too wide"), "unexpected: {err}");
    }

    #[test]
    fn ns_to_cycles_conversion() {
        // At 125 MHz, 1 cycle = 8 ns. ns=8 → cycles=1.
        assert_eq!(ns_to_cycles(8, 125_000_000), 1);
        // Integer truncation is defined — 7 ns rounds down to 0 cycles.
        assert_eq!(ns_to_cycles(7, 125_000_000), 0);
        // 1 s at 125 MHz = 125 million cycles.
        assert_eq!(ns_to_cycles(1_000_000_000, 125_000_000), 125_000_000);
        // 10 s — well within u64, u128 math avoids overflow.
        assert_eq!(ns_to_cycles(10_000_000_000, 125_000_000), 1_250_000_000);
    }

    /// Mock sink that records pokes instead of driving a real emulator —
    /// used to verify replayer ordering without booting the M0+ core.
    struct MockSink {
        cycles: u64,
        pokes: Vec<(u64, u32 /* bus-level packed value */)>,
        last_iow_low: bool,
        recorded: Vec<Poke>,
    }

    impl MockSink {
        fn new() -> Self {
            Self {
                cycles: 0,
                pokes: Vec::new(),
                last_iow_low: false,
                recorded: Vec::new(),
            }
        }

        fn push_write(&mut self, port: u16, value: u16, kind: TraceKind) {
            self.recorded.push(Poke {
                cycle: self.cycles,
                port,
                value,
                kind,
            });
        }
    }

    impl IsaSink for MockSink {
        fn step(&mut self, cycles: u32) {
            self.cycles = self.cycles.wrapping_add(cycles as u64);
        }
        fn cycles(&self) -> u64 {
            self.cycles
        }
        fn drive_pins(&mut self, iow_low: bool, _ior_low: bool, ad_bus: u16) {
            // We approximate a write event as: address phase (IOW high,
            // addr on bus) → assert (IOW low, addr on bus) → data phase
            // (IOW low, data on bus). The transition from IOW-high-with-
            // address to IOW-low marks the start of a write; the
            // subsequent drive-pins while IOW-low latches the data.
            let packed = ((iow_low as u32) << 31) | (ad_bus as u32);
            self.pokes.push((self.cycles, packed));
            self.last_iow_low = iow_low;
        }
    }

    #[test]
    fn replay_advances_emulator_to_target_cycles() {
        // One write at ns=1_000_000 = 125_000 cycles at 125 MHz.
        let mut emu = EmulatorBuilder::new(Config {
            sys_clk_hz: 125_000_000,
        })
        .build().expect("Serial build is infallible");

        // Seed ROM with a minimal vector table + self-branch loop so
        // reset brings core 0 up running instructions (rather than
        // faulting on an unmapped fetch → HardFault-in-HardFault → core
        // lockup → zero-progress infinite loop in `replay`).
        //
        //   Word 0: initial SP (top of SRAM)
        //   Word 1: reset vector → 0x0000_0009 (PC=8, Thumb bit set)
        //   @0x08 : 0xe7fe — `B .` (branch-to-self, 1-3 cycles per taken)
        let mut rom = vec![0u8; 16];
        rom[0..4].copy_from_slice(&0x2004_2000u32.to_le_bytes());
        rom[4..8].copy_from_slice(&0x0000_0009u32.to_le_bytes());
        rom[8..10].copy_from_slice(&0xe7feu16.to_le_bytes());
        emu.load_image(0x0000_0000, &rom);
        emu.reset();
        // `reset()` clobbers the clock tree back to ROSC 6.5 MHz.
        // Re-seed to match the Config so the post-fix PLL-aware replay
        // loop paces events at 125 MHz (matching the old test's
        // implicit assumption, now made explicit by the fix).
        emu.bus.seed_sys_clk_hz(125_000_000);

        let events = vec![TraceEvent {
            ns: 1_000_000,
            port: 0x240,
            value: 0xAB,
            kind: TraceKind::Write8,
        }];

        let summary = replay(&mut emu, &events, None, None, 0, 1.0);
        assert!(
            emu.cycles() >= 125_000,
            "emu cycles {} did not reach 125_000",
            emu.cycles()
        );
        assert_eq!(summary.writes_fired, 1);
        assert_eq!(summary.reads_skipped, 0);
    }

    #[test]
    fn replay_ignores_reads() {
        // One read + one write; expect 1 write fired, 1 read skipped.
        let events = vec![
            TraceEvent {
                ns: 100_000,
                port: 0x246,
                value: 0x20,
                kind: TraceKind::Read8,
            },
            TraceEvent {
                ns: 200_000,
                port: 0x240,
                value: 0xCD,
                kind: TraceKind::Write8,
            },
        ];

        let mut sink = MockSink::new();
        let summary = replay(&mut sink, &events, None, None, 0, 1.0);
        assert_eq!(summary.writes_fired, 1);
        assert_eq!(summary.reads_skipped, 1);
        // MockSink only sees drive_pins during writes; a read emits
        // nothing through the sink.
        assert!(
            !sink.pokes.is_empty(),
            "expected pokes from the write event"
        );
    }

    #[test]
    fn replay_fires_gpio_pokes_in_order() {
        // Three writes with distinct values at distinct times. Record
        // the sequence of (cycle, ad_bus_value) pairs; assert ordering.
        let events = vec![
            TraceEvent {
                ns: 0,
                port: 0x240,
                value: 0x11,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 100_000,
                port: 0x241,
                value: 0x22,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 200_000,
                port: 0x242,
                value: 0x33,
                kind: TraceKind::Write8,
            },
        ];

        // Inject via a hand-wired replay that records per-event the
        // asserted-data bus snapshot. We use the public `drive_write_cycle`
        // against a MockSink that also populates `recorded` so tests can
        // see cleanly what went out per trace event.
        let mut sink = MockSink::new();
        for ev in &events {
            // Fast-forward to event time.
            let target = ns_to_cycles(ev.ns, 125_000_000);
            while sink.cycles() < target {
                sink.step(16);
            }
            if ev.kind.is_write() {
                let wide = matches!(ev.kind, TraceKind::Write16);
                let pre_cycle = sink.cycles();
                drive_write_cycle(&mut sink, ev.port, ev.value, wide);
                sink.push_write(ev.port, ev.value, ev.kind);
                assert!(
                    sink.cycles() > pre_cycle,
                    "drive_write_cycle must advance the sink"
                );
            }
        }

        assert_eq!(sink.recorded.len(), 3);
        assert_eq!(sink.recorded[0].port, 0x240);
        assert_eq!(sink.recorded[0].value, 0x11);
        assert_eq!(sink.recorded[1].port, 0x241);
        assert_eq!(sink.recorded[1].value, 0x22);
        assert_eq!(sink.recorded[2].port, 0x242);
        assert_eq!(sink.recorded[2].value, 0x33);

        // Monotonic cycles.
        assert!(sink.recorded[0].cycle <= sink.recorded[1].cycle);
        assert!(sink.recorded[1].cycle <= sink.recorded[2].cycle);

        // Every write generated 4 drive_pins calls (idle, assert,
        // data, deassert) — so 3 writes × 4 = 12 pokes total.
        assert_eq!(sink.pokes.len(), 12);
    }

    #[test]
    fn drive_write_cycle_drives_expected_pins() {
        // Full pin-level assertion: check the emulator's gpio_in after
        // each phase of a single 8-bit write matches what the PIO
        // program expects to sample.
        let mut emu = EmulatorBuilder::new(Config {
            sys_clk_hz: 125_000_000,
        })
        .build().expect("Serial build is infallible");
        emu.reset();

        // Use a thin wrapper that exposes gpio_in between phases.
        struct Probe<'a> {
            emu: &'a mut Emulator,
            snapshots: Vec<u32>,
        }
        impl<'a> IsaSink for Probe<'a> {
            fn step(&mut self, cycles: u32) {
                // Don't actually step the emulator — we want to observe
                // gpio_in as driven by drive_pins, not have the PIO /
                // PSRAM tick overwrite bit 0 mid-test.
                let _ = cycles;
            }
            fn cycles(&self) -> u64 {
                self.emu.cycles()
            }
            fn drive_pins(&mut self, iow_low: bool, ior_low: bool, ad_bus: u16) {
                self.emu.drive_pins(iow_low, ior_low, ad_bus);
                self.snapshots.push(self.emu.bus.gpio_in);
            }
        }

        let mut probe = Probe {
            emu: &mut emu,
            snapshots: Vec::new(),
        };

        drive_write_cycle(&mut probe, 0x243, 0xCD, false);
        assert_eq!(probe.snapshots.len(), 4);

        let iow_mask = 1u32 << PIN_IOW;
        let ad_mask = ((1u32 << PIN_AD_COUNT) - 1) << PIN_AD0;

        // Phase 0: idle — IOW high, address on AD bus.
        assert_eq!(
            probe.snapshots[0] & iow_mask,
            iow_mask,
            "IOW must be high during idle"
        );
        assert_eq!(
            (probe.snapshots[0] & ad_mask) >> PIN_AD0,
            0x243,
            "AD bus must carry address during idle"
        );

        // Phase 1: assert — IOW low, address still on bus.
        assert_eq!(probe.snapshots[1] & iow_mask, 0, "IOW must be low on assert");
        assert_eq!(
            (probe.snapshots[1] & ad_mask) >> PIN_AD0,
            0x243,
            "AD bus must still carry address during assert"
        );

        // Phase 2: data — IOW low, data on bus.
        assert_eq!(probe.snapshots[2] & iow_mask, 0, "IOW stays low on data");
        assert_eq!(
            (probe.snapshots[2] & ad_mask) >> PIN_AD0,
            0xCD,
            "AD bus must carry data during write"
        );

        // Phase 3: deassert — IOW high, AD bus cleared.
        assert_eq!(
            probe.snapshots[3] & iow_mask,
            iow_mask,
            "IOW must be high after deassert"
        );
    }

    #[test]
    fn parse_rejects_malformed_row() {
        let text = "\
# picogus-tap v1
ns,port,value,kind
0,0x240,0xff
";
        let err = parse_trace(text).expect_err("missing kind field");
        assert!(err.contains("4 CSV fields"), "unexpected: {err}");
    }

    #[test]
    fn duration_cap_stops_replay_early() {
        let events = vec![
            TraceEvent {
                ns: 0,
                port: 0x240,
                value: 0x11,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 500_000_000, // 0.5 s
                port: 0x241,
                value: 0x22,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 2_000_000_000, // 2.0 s — past the cap
                port: 0x242,
                value: 0x33,
                kind: TraceKind::Write8,
            },
        ];

        let mut sink = MockSink::new();
        // Cap at 1 s.
        let summary = replay(&mut sink, &events, Some(1_000_000_000), None, 0, 1.0);
        assert_eq!(summary.writes_fired, 2);
        assert!(summary.duration_capped);
    }

    #[test]
    fn post_roll_advances_sink_after_last_event() {
        // Single event at ns=0; with a 1 ms post-roll @ 125 MHz we
        // expect exactly 125_000 additional cycles past `final_cycles`.
        // Catches B3: trace-end drain missing.
        let events = vec![TraceEvent {
            ns: 0,
            port: 0x240,
            value: 0x11,
            kind: TraceKind::Write8,
        }];

        let mut sink = MockSink::new();
        let summary = replay(
            &mut sink,
            &events,
            None,
            Some(1_000_000), // 1 ms
            0,
            1.0,
        );

        assert_eq!(summary.writes_fired, 1);
        assert!(
            summary.post_roll_cycles >= 125_000,
            "post-roll did not run: {} cycles",
            summary.post_roll_cycles
        );
        // MockSink advances by exactly the requested chunk count, so the
        // post-roll budget should be hit precisely (no overshoot like
        // the real emulator's quantum). ns_to_cycles is exact at this
        // boundary too.
        assert_eq!(
            summary.post_roll_cycles, 125_000,
            "1 ms post-roll @ 125 MHz must be 125_000 cycles"
        );
    }

    #[test]
    fn post_roll_zero_does_not_advance_sink() {
        // Post-roll = Some(0) and post-roll = None must both leave the
        // sink at `final_cycles`. Guards against accidentally always
        // running the drain.
        let events = vec![TraceEvent {
            ns: 0,
            port: 0x240,
            value: 0xAA,
            kind: TraceKind::Write8,
        }];

        let mut sink_zero = MockSink::new();
        let summary_zero = replay(&mut sink_zero, &events, None, Some(0), 0, 1.0);
        assert_eq!(summary_zero.post_roll_cycles, 0);

        let mut sink_none = MockSink::new();
        let summary_none = replay(&mut sink_none, &events, None, None, 0, 1.0);
        assert_eq!(summary_none.post_roll_cycles, 0);
        assert_eq!(sink_zero.cycles(), sink_none.cycles());
    }

    #[test]
    fn pre_roll_advances_sink_before_first_event() {
        // With `pre_roll_ns = 1_000_000` and a single event at ev.ns=0,
        // the sink should advance by the pre-roll AMOUNT (125_000 cyc
        // at 125 MHz) before firing, so final cycles = pre-roll + drive
        // overhead (~60 cycles).
        let events = vec![TraceEvent {
            ns: 0,
            port: 0x240,
            value: 0xAB,
            kind: TraceKind::Write8,
        }];
        let mut sink = MockSink::new();
        let summary = replay(&mut sink, &events, None, None, 1_000_000, 1.0);
        assert_eq!(summary.writes_fired, 1);
        // MockSink's clock is 125 MHz; 1 ms pre-roll → 125_000 cycles
        // before the write fires. drive_write_cycle adds overhead.
        assert!(
            sink.cycles() >= 125_000,
            "pre-roll did not advance sink: cycles = {}",
            sink.cycles()
        );
    }

    #[test]
    fn trace_stretch_scales_inter_event_gap() {
        // Two events at ns=0 and ns=1_000_000 (1 ms apart). Stretch 2.0
        // should make the second event fire at sim-time 2 ms, i.e. the
        // final sink cycle count is ≥ 2 × the stretch=1.0 case.
        let events = vec![
            TraceEvent { ns: 0, port: 0x240, value: 0x01, kind: TraceKind::Write8 },
            TraceEvent { ns: 1_000_000, port: 0x241, value: 0x02, kind: TraceKind::Write8 },
        ];
        let mut sink1 = MockSink::new();
        let _ = replay(&mut sink1, &events, None, None, 0, 1.0);
        let c1 = sink1.cycles();

        let mut sink2 = MockSink::new();
        let _ = replay(&mut sink2, &events, None, None, 0, 2.0);
        let c2 = sink2.cycles();

        // Ignoring drive overhead, c1 ≈ 125_000 cyc, c2 ≈ 250_000 cyc.
        // Check c2 is meaningfully > c1 (> 1.5×) — stretch took effect.
        assert!(
            c2 > c1 * 3 / 2,
            "stretch=2.0 should lengthen sim-time: c1={c1} c2={c2}"
        );
    }

    #[test]
    fn stretched_target_ns_is_identity_with_defaults() {
        // Byte-identical baseline contract: pre_roll=0 & stretch=1.0
        // must leave ev.ns untouched at every input.
        for ev_ns in [0u64, 1, 1_000, 1_000_000, 29_991_219_767] {
            assert_eq!(stretched_target_ns(ev_ns, 0, 1.0), ev_ns);
        }
    }

    /// Mock sink that refuses to advance — used to verify the stall
    /// counter and one-shot warning. Catches the additional "stall
    /// counter" deliverable.
    struct StalledSink {
        cycles: u64,
    }
    impl IsaSink for StalledSink {
        fn step(&mut self, _cycles: u32) {
            // Refuse to advance — simulates a wedged emulator.
        }
        fn cycles(&self) -> u64 {
            self.cycles
        }
        fn drive_pins(&mut self, _iow_low: bool, _ior_low: bool, _ad_bus: u16) {}
    }

    #[test]
    fn replay_counts_stall_events() {
        // Three writes spaced apart in time. With a sink that refuses
        // to step, every fast-forward loop should hit the stall guard
        // and increment the counter.
        let events = vec![
            TraceEvent {
                ns: 100_000,
                port: 0x240,
                value: 0x01,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 200_000,
                port: 0x240,
                value: 0x02,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 300_000,
                port: 0x240,
                value: 0x03,
                kind: TraceKind::Write8,
            },
        ];
        let mut sink = StalledSink { cycles: 0 };
        let summary = replay(&mut sink, &events, None, None, 0, 1.0);
        // Each event needed a fast-forward, each stalled — so 3 stalls.
        assert_eq!(summary.stall_events, 3);
        // Writes still fired even though the sink stalled.
        assert_eq!(summary.writes_fired, 3);
    }

    /// Replay the real emulator end-to-end and check the post-roll path
    /// reports cycles reflecting the drain. Catches B3 against the
    /// production sink path (not just MockSink).
    #[test]
    fn replay_end_to_end_post_roll_reports_cycles() {
        let mut emu = EmulatorBuilder::new(Config {
            sys_clk_hz: 125_000_000,
        })
        .step_quantum(1)
        .build().expect("Serial build is infallible");

        // Same minimal vector + B-to-self loop as
        // replay_advances_emulator_to_target_cycles.
        let mut rom = vec![0u8; 16];
        rom[0..4].copy_from_slice(&0x2004_2000u32.to_le_bytes());
        rom[4..8].copy_from_slice(&0x0000_0009u32.to_le_bytes());
        rom[8..10].copy_from_slice(&0xe7feu16.to_le_bytes());
        emu.load_image(0x0000_0000, &rom);
        emu.reset();
        // `reset()` clobbers clock tree to ROSC (6.5 MHz); re-seed to
        // 125 MHz so 1 ms of post-roll = 125_000 cycles.
        emu.bus.seed_sys_clk_hz(125_000_000);

        let events = vec![TraceEvent {
            ns: 1_000_000,
            port: 0x240,
            value: 0xAB,
            kind: TraceKind::Write8,
        }];

        let summary = replay(
            &mut emu,
            &events,
            None,
            Some(1_000_000), // 1 ms post-roll
            0,
            1.0,
        );
        assert_eq!(summary.writes_fired, 1);
        assert!(
            summary.post_roll_cycles >= 125_000,
            "post-roll cycles {} below 1 ms target",
            summary.post_roll_cycles
        );
        // Final emu cycles must include the drain.
        assert!(
            emu.cycles() >= summary.final_cycles + summary.post_roll_cycles,
            "emu cycles {} did not advance through post-roll (final={}, post={})",
            emu.cycles(),
            summary.final_cycles,
            summary.post_roll_cycles
        );
    }

    /// Inner sink that doesn't actually model audio but exposes a
    /// synthetic `pad_state` whose bits toggle each cycle. If the
    /// CapturingSink wires the tick correctly, the I2sCapture inside
    /// will observe BCLK and LRCLK edges and produce at least a non-
    /// zero LRCLK edge count.
    #[test]
    fn capturing_sink_forwards_ticks_to_i2s_capture() {
        use mdpicoem_harness::picogus_pins::{I2S_BCLK, I2S_LRCLK};

        /// Synthetic pad generator: BCLK toggles every cycle, LRCLK
        /// toggles every 16 cycles (one half-frame).
        struct SynthSink {
            cycles: u64,
        }
        impl IsaSink for SynthSink {
            fn step(&mut self, cycles: u32) {
                self.cycles = self.cycles.wrapping_add(cycles as u64);
            }
            fn cycles(&self) -> u64 {
                self.cycles
            }
            fn drive_pins(&mut self, _iow_low: bool, _ior_low: bool, _ad_bus: u16) {}
            fn pad_state(&self) -> u32 {
                let mut p = 0u32;
                if self.cycles & 1 != 0 {
                    p |= 1u32 << I2S_BCLK;
                }
                if (self.cycles / 16) & 1 != 0 {
                    p |= 1u32 << I2S_LRCLK;
                }
                p
            }
        }

        let mut sink = CapturingSink::new(SynthSink { cycles: 0 }, 125_000_000);
        // Step 1024 cycles — 1024/16 = 64 LRCLK edges.
        sink.step(1024);
        let cap = sink.capture();
        assert!(
            cap.lrclk_edge_count() > 0,
            "expected LRCLK edges from synthetic pads, got {}",
            cap.lrclk_edge_count()
        );
        // LRCLK toggles every 16 cycles; by cycle 1024 we should have
        // ~63 edges (the first toggle is at cycle 16, last at 1008).
        assert!(
            cap.lrclk_edge_count() >= 60 && cap.lrclk_edge_count() <= 65,
            "LRCLK edge count {} out of expected range [60..=65]",
            cap.lrclk_edge_count()
        );
    }

    /// The CapturingSink must advance the inner sink by exactly `n`
    /// cycles when the inner sink is not stalled.
    #[test]
    fn capturing_sink_advances_one_cycle_at_a_time() {
        let mut inner = MockSink::new();
        inner.last_iow_low = false; // ensure no stale state
        let inner_wrapped = CapturingWrapper { inner: &mut inner };
        let mut sink = CapturingSink::new(inner_wrapped, 125_000_000);
        sink.step(100);
        assert_eq!(
            sink.cycles(),
            100,
            "100 single-cycle inner steps must leave inner at 100 cycles"
        );
    }

    /// The I2S capture timestamps must track the inner sink's sysclk
    /// count, not the number of `inner.step(1)` calls. This is the
    /// synthetic repro: a mock whose `step(1)` advances cycles by 4 —
    /// simulating a multi-sysclk M0+ instruction (e.g. BL, LDM {R0..R2}).
    /// Under the broken per-call cycle counter, LRCLK edges would be
    /// stamped at iteration index (1, 2, 3, …) instead of sysclk index
    /// (4, 8, 12, …), inflating `inferred_sample_rate_hz` by 4×.
    #[test]
    fn capturing_sink_stamps_edges_in_sysclks_not_ticks() {
        use mdpicoem_harness::picogus_pins::I2S_LRCLK;

        /// Mock whose `step(1)` reports 4 cycles consumed, with LRCLK
        /// toggling on every inner step boundary. If the CapturingSink
        /// stamps edges in `inner.cycles()` (4, 8, 12, …), the inferred
        /// period is 8 sysclks per full LRCLK cycle. If it stamps in
        /// tick-call counts (1, 2, 3, …), the inferred period would be
        /// 2 units → 4× too fast.
        struct MultiCycleSink {
            cycles: u64,
            lrclk_high: bool,
        }
        impl IsaSink for MultiCycleSink {
            fn step(&mut self, _cycles: u32) {
                // Always report 4 cycles per inner step, regardless of
                // the requested count — models a 4-cycle M0+ BL.
                self.cycles = self.cycles.wrapping_add(4);
                // Flip LRCLK after every step so every inner step is an
                // LRCLK edge.
                self.lrclk_high = !self.lrclk_high;
            }
            fn cycles(&self) -> u64 {
                self.cycles
            }
            fn drive_pins(&mut self, _iow_low: bool, _ior_low: bool, _ad_bus: u16) {}
            fn pad_state(&self) -> u32 {
                if self.lrclk_high { 1u32 << I2S_LRCLK } else { 0 }
            }
        }

        // Scripted run: `sink.step(40)` should loop until inner.cycles()
        // >= 40 (10 inner steps @ 4 cycles each). That gives us 10 LRCLK
        // edges with monotonic sysclk stamps 4, 8, 12, …, 40.
        let sys_clk = 40u32;
        let mut sink = CapturingSink::new(
            MultiCycleSink { cycles: 0, lrclk_high: false },
            sys_clk,
        );
        sink.step(40);
        assert_eq!(
            sink.cycles(),
            40,
            "inner should advance to 40 cycles (10 iterations of +4)"
        );
        let cap = sink.capture();
        assert_eq!(
            cap.lrclk_edge_count(),
            10,
            "expected one LRCLK edge per inner step"
        );
        let inferred = cap
            .inferred_sample_rate_hz()
            .expect("10 edges should infer a rate");
        // `half_periods = edges - 1 = 9`, total cycles = 40 - 4 = 36
        // → rate = sys_clk * 9 / (2 * 36) = 40 * 9 / 72 = 5 Hz.
        // If the fix were reverted and edges were stamped at iteration
        // indices (1, 2, …, 10), total_cycles would be 9 and the
        // inferred rate would be 40 * 9 / 18 = 20 Hz — 4× too high.
        let expected = 5.0f64;
        let rel_err = (inferred - expected).abs() / expected;
        assert!(
            rel_err < 1e-9,
            "expected {expected} Hz (sysclk-stamped edges), got {inferred} Hz \
             — if this fails with ~20 Hz, the capture is counting ticks \
             instead of sysclks"
        );
    }

    /// End-to-end with a real `mdrp2040::Emulator`: a branch-to-self
    /// (`B .` at 3 cycles per iteration) parks core 0 on a multi-cycle
    /// instruction. Each `sink.step(1)` should trigger exactly one
    /// `inner.step(1)` + one I2S tick — but `inner.cycles()` advances
    /// by 3, and the I2S tick stamp must reflect that sysclk delta.
    #[test]
    fn capturing_sink_cycle_matches_emulator_cycles() {
        // Build a minimal ROM: vector table + `B .` at the reset entry
        // so core 0 loops on a 3-cycle taken branch forever.
        let mut emu = EmulatorBuilder::new(Config {
            sys_clk_hz: 125_000_000,
        })
        .step_quantum(1)
        .build().expect("Serial build is infallible");

        let mut rom = vec![0u8; 16];
        // Initial SP at top of SRAM
        rom[0..4].copy_from_slice(&0x2004_2000u32.to_le_bytes());
        // Reset vector → PC = 0x09 (Thumb bit set, entry at 0x08)
        rom[4..8].copy_from_slice(&0x0000_0009u32.to_le_bytes());
        // `B .` at 0x08: encoded 0xE7FE — taken branch, 3 cycles per
        // iteration on M0+.
        rom[8..10].copy_from_slice(&0xe7feu16.to_le_bytes());
        emu.load_image(0x0000_0000, &rom);
        emu.reset();

        // Reference cycle trace: drive a bare emulator through N inner
        // steps, recording `cycles()` after each step. Each step is one
        // taken branch = 3 sysclks.
        let mut reference_cycles: Vec<u64> = Vec::new();
        for _ in 0..8 {
            emu.step().expect("Serial step is infallible");
            reference_cycles.push(emu.cycles());
        }
        assert_eq!(
            reference_cycles.last().copied(),
            Some(24),
            "8 taken branches of 3 cycles each must leave emu at 24 cycles"
        );

        // Now build a fresh emulator (same program) and wrap it. Use
        // `sink.step(1)` N times; each iteration should advance the
        // inner emulator by 3 cycles (not 1, which is what the old
        // CapturingSink::step(n) for-loop implicitly assumed).
        let mut emu2 = EmulatorBuilder::new(Config {
            sys_clk_hz: 125_000_000,
        })
        .step_quantum(1)
        .build().expect("Serial build is infallible");
        emu2.load_image(0x0000_0000, &rom);
        emu2.reset();
        let mut sink = CapturingSink::new(emu2, 125_000_000);

        // Each outer step(1) asks to advance by 1 sysclk; the inner
        // emulator overshoots to 3 cycles on the first `inner.step(1)`
        // (the branch), satisfying the `cycles >= target` exit in
        // CapturingSink::step, so exactly one inner step happens per
        // outer step.
        let mut observed_cycles: Vec<u64> = Vec::new();
        for _ in 0..8 {
            sink.step(1);
            observed_cycles.push(sink.cycles());
        }
        assert_eq!(
            observed_cycles, reference_cycles,
            "CapturingSink must advance inner emulator by true sysclks per \
             instruction, not by the number of step calls. Got {:?} vs \
             expected {:?}.",
            observed_cycles, reference_cycles
        );

        // Now drive the same emulator with an LRCLK-toggle fixture and
        // check the capture records monotonically increasing sysclk
        // stamps. We can't easily inject LRCLK without firmware, but we
        // can assert the capture's internal first/last cycle bounds via
        // `inferred_sample_rate_hz` on a synthetic pad toggle above. The
        // key contract — "tick sees inner.cycles(), not a call counter"
        // — is already proven by the cycle equality above plus the
        // `capturing_sink_stamps_edges_in_sysclks_not_ticks` test.
    }

    /// Small adapter around `&mut MockSink` so we can use a MockSink as
    /// the inner sink of CapturingSink without moving ownership.
    struct CapturingWrapper<'a> {
        inner: &'a mut MockSink,
    }
    impl<'a> IsaSink for CapturingWrapper<'a> {
        fn step(&mut self, cycles: u32) {
            self.inner.step(cycles);
        }
        fn cycles(&self) -> u64 {
            self.inner.cycles()
        }
        fn drive_pins(&mut self, iow_low: bool, ior_low: bool, ad_bus: u16) {
            self.inner.drive_pins(iow_low, ior_low, ad_bus);
        }
    }

    /// `resolve_bootrom_path` always honours an explicit `--bootrom` flag,
    /// even if the file doesn't exist — we leave existence checks to the
    /// caller (which reports the read error with a useful message).
    #[test]
    fn resolve_bootrom_path_honours_explicit_flag() {
        let explicit = PathBuf::from("/nonexistent/custom/bootrom.bin");
        let resolved = resolve_bootrom_path(Some(&explicit), true);
        assert_eq!(resolved, Some(explicit));
    }

    /// Without `--flash`, the default-search is skipped entirely — the
    /// trace-only test mode shouldn't trigger a spurious bootrom load.
    #[test]
    fn resolve_bootrom_path_none_when_no_flash() {
        let resolved = resolve_bootrom_path(None, false);
        assert_eq!(resolved, None);
    }

    /// Hand-crafted I2S chime firmware — 60 bytes of Thumb-16 that
    /// drives GPIO 16/17/18 (DOUT/BCLK/LRCLK) directly via SIO writes.
    /// Generated by `roms/rp2040/gen_i2s_chime.py`; embedded here so the
    /// smoke test doesn't depend on the script having been run. Update
    /// both this array and `i2s_chime.bin` if the generator changes.
    const CHIME_FIRMWARE: &[u8] = &[
        0x0d, 0x4c, 0x07, 0x25, 0x2d, 0x04, 0x01, 0x27, 0x3f, 0x04, 0x01, 0x21,
        0x49, 0x04, 0x01, 0x22, 0x92, 0x04, 0x65, 0x62, 0xa5, 0x61, 0x40, 0x26,
        0x30, 0x46, 0x40, 0x07, 0xc0, 0x0f, 0x00, 0x04, 0xa7, 0x61, 0x60, 0x61,
        0x20, 0x23, 0x61, 0x61, 0xa1, 0x61, 0x01, 0x3b, 0xfb, 0xd1, 0xe2, 0x61,
        0x01, 0x3e, 0xf1, 0xd1, 0xfe, 0xe7, 0x00, 0xbf, 0x00, 0x00, 0x00, 0xd0,
    ];

    /// Minimal ROM image: SP at word 0 = top of SRAM (0x20042000),
    /// reset vector at word 4 = flash base | Thumb. Rest filled with
    /// `B .` traps. Matches `roms/rp2040/gen_blinky.py::build_bootrom`.
    fn minimal_bootrom_for_flash_entry() -> Vec<u8> {
        let mut rom = vec![0u8; 16 * 1024];
        rom[0..4].copy_from_slice(&0x2004_2000u32.to_le_bytes());
        rom[4..8].copy_from_slice(&0x1000_0001u32.to_le_bytes());
        for chunk in rom[8..].chunks_mut(2) {
            chunk.copy_from_slice(&0xE7FEu16.to_le_bytes()); // B .
        }
        rom
    }

    /// End-to-end smoke: load the hand-crafted chime firmware, boot via
    /// the synthetic bootrom (no direct-boot, since chime has no SDK
    /// vector table at flash+0x100), run long enough for all half-frames
    /// to emit, and verify the capture contains non-zero audio. Proves
    /// the full GPIO → SIO → gpio_in → I2sCapture pipeline works.
    #[test]
    fn chime_firmware_produces_nonzero_audio() {
        let mut emu = EmulatorBuilder::new(Config {
            sys_clk_hz: 125_000_000,
        })
        .step_quantum(1)
        .flash(CHIME_FIRMWARE.to_vec())
        .build().expect("Serial build is infallible");
        emu.load_bootrom(&minimal_bootrom_for_flash_entry());
        emu.reset();

        let mut sink = CapturingSink::new(emu, 125_000_000);
        // 1.5 M cycles — empirically well past the chime's ~1.4 M-cycle
        // emission window and into its terminal `B .` self-loop.
        for _ in 0..1_500_000u64 {
            sink.step(1);
        }

        let (_emu, capture) = sink.into_parts();
        assert!(
            capture.lrclk_edge_count() >= 32,
            "chime should emit ≥32 LRCLK edges, got {}",
            capture.lrclk_edge_count()
        );
        let frames = capture.frames();
        assert!(
            !frames.is_empty(),
            "chime should produce at least one decoded frame"
        );
        let nz = frames.iter().filter(|(l, r)| *l != 0 || *r != 0).count();
        assert!(
            nz * 2 >= frames.len(),
            "expected ≥50% non-zero frames, got {nz}/{}",
            frames.len()
        );
    }

    // ----------------------------------------------------------------
    // CaptureCoverage classifier unit tests (HLD §3.3).
    //
    // Guards the delta/pending state machine against regressions —
    // specifically the HLD §7(4) invariant
    // `captured + misattributed + post_roll_orphans == autopush_count`.
    // Each test simulates a sequence of (delta, decode-matches, class,
    // decile) sub-event inputs to `CaptureCoverage::classify_sub_event`,
    // then asserts the per-bucket totals and the global invariant.
    // ----------------------------------------------------------------

    /// Sum `captured` over all `ClassKey` rows.
    fn sum_captured(cov: &CaptureCoverage) -> u64 {
        cov.classes.values().map(|c| c.captured).sum()
    }

    /// Sum `misattributed` over all `ClassKey` rows.
    fn sum_misattr(cov: &CaptureCoverage) -> u64 {
        cov.classes.values().map(|c| c.misattributed).sum()
    }

    /// HLD §7(4) invariant. Excludes `catch_up_unattributed` because
    /// it's a residual bucket by design — `delta > 1` cases with
    /// fewer credits than pushes spill into it.
    fn assert_invariant(cov: &CaptureCoverage, total_pushes: u64) {
        let accounted = sum_captured(cov)
            + sum_misattr(cov)
            + cov.post_roll_orphans
            + cov.catch_up_unattributed;
        assert_eq!(
            accounted, total_pushes,
            "HLD §7(4): captured ({}) + misattr ({}) + orphans ({}) + \
             catch_up_unattributed ({}) = {} != autopush_count ({})",
            sum_captured(cov),
            sum_misattr(cov),
            cov.post_roll_orphans,
            cov.catch_up_unattributed,
            accounted,
            total_pushes,
        );
    }

    /// Helper: mark a sub-event as fired and run the classifier.
    fn record(
        cov: &mut CaptureCoverage,
        delta: u64,
        decode_matches: bool,
        class: ClassKey,
        decile: usize,
        pending: &mut Option<(ClassKey, usize)>,
    ) {
        let e = cov.classes.entry(class).or_default();
        e.fired += 1;
        cov.decile_fired[decile] += 1;
        cov.fired_sub_events += 1;
        cov.classify_sub_event(delta, decode_matches, class, decile, pending);
    }

    /// `delta == 1` with a decode that matches current → `captured`
    /// bumps on current class; pending cleared.
    #[test]
    fn classifier_delta1_match_credits_current_captured() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;
        record(&mut cov, 1, true, ClassKey::P347, 0, &mut pending);
        assert_eq!(cov.classes[&ClassKey::P347].captured, 1);
        assert_eq!(cov.classes[&ClassKey::P347].misattributed, 0);
        assert_eq!(cov.decile_captured[0], 1);
        assert!(pending.is_none(), "pending should clear on clean capture");
        assert_invariant(&cov, 1);
    }

    /// `delta == 0` → no bucket bump; current becomes pending.
    #[test]
    fn classifier_delta0_leaves_current_pending() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;
        record(&mut cov, 0, false, ClassKey::P344Lo, 3, &mut pending);
        assert_eq!(cov.classes[&ClassKey::P344Lo].captured, 0);
        assert_eq!(cov.classes[&ClassKey::P344Lo].misattributed, 0);
        assert_eq!(cov.decile_captured[3], 0);
        assert_eq!(pending, Some((ClassKey::P344Lo, 3)));
        // No pushes landed yet → invariant holds with 0 pushes.
        assert_invariant(&cov, 0);
    }

    /// MUST-FIX 1 regression guard: `delta == 1` with mismatched
    /// decode and a pending previous class → credit the PREVIOUS
    /// class as `misattributed` (NOT captured), bump the previous
    /// event's decile (SHOULD-FIX 2), current becomes new pending.
    /// Only ONE bucket bumps per physical push.
    #[test]
    fn classifier_delta1_mismatch_with_pending_credits_prev_misattr() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;

        // Event A: delta=0 → A pends at decile 2.
        record(&mut cov, 0, false, ClassKey::P347, 2, &mut pending);
        assert_eq!(pending, Some((ClassKey::P347, 2)));
        // No pushes observed so far.
        assert_invariant(&cov, 0);

        // Event B: delta=1, decode mismatches B → A's late push landed.
        // A should get `misattributed += 1` (NOT captured), decile
        // table should bump A's decile (2), B becomes the new pending.
        record(&mut cov, 1, false, ClassKey::P344Lo, 5, &mut pending);
        assert_eq!(
            cov.classes[&ClassKey::P347].misattributed, 1,
            "prev (A) must get misattr += 1"
        );
        assert_eq!(
            cov.classes[&ClassKey::P347].captured, 0,
            "prev (A) must NOT get captured (pre-MUST-FIX bug)"
        );
        assert_eq!(
            cov.classes[&ClassKey::P344Lo].captured, 0,
            "current (B) must NOT double-count as captured"
        );
        assert_eq!(
            cov.classes[&ClassKey::P344Lo].misattributed, 0,
            "current (B) must NOT double-count as misattr \
             (pre-MUST-FIX bug)"
        );
        assert_eq!(
            cov.decile_captured[2], 1,
            "SHOULD-FIX 2: decile must bump at PREV event's decile (2)"
        );
        assert_eq!(
            cov.decile_captured[5], 0,
            "current event's decile must not be credited here"
        );
        assert_eq!(pending, Some((ClassKey::P344Lo, 5)));
        // Exactly 1 push landed.
        assert_invariant(&cov, 1);
    }

    /// `delta == 1` with mismatched decode and NO pending → a stray
    /// push lands; credit current as `misattributed`, current stays
    /// pending for next cycle.
    #[test]
    fn classifier_delta1_mismatch_no_pending_credits_current_misattr() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;
        record(&mut cov, 1, false, ClassKey::P343(0x43), 4, &mut pending);
        assert_eq!(cov.classes[&ClassKey::P343(0x43)].captured, 0);
        assert_eq!(cov.classes[&ClassKey::P343(0x43)].misattributed, 1);
        assert_eq!(cov.decile_captured[4], 1);
        assert_eq!(pending, Some((ClassKey::P343(0x43), 4)));
        // One stray push landed → one bucket bumped.
        assert_invariant(&cov, 1);
    }

    /// `delta == 2` catch-up with both decode-match AND pending
    /// present → credit current (captured) + prev (captured), each
    /// in its own decile (SHOULD-FIX 2). No residual unattributed.
    #[test]
    fn classifier_delta2_catchup_match_with_pending() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;

        // Event A: delta=0 → A pends at decile 1.
        record(&mut cov, 0, false, ClassKey::P347, 1, &mut pending);
        assert_invariant(&cov, 0);

        // Event B: delta=2, decode matches B → 2 pushes landed,
        // credit B (captured at decile 7) and A (captured at decile 1).
        record(&mut cov, 2, true, ClassKey::P344Lo, 7, &mut pending);
        assert_eq!(cov.classes[&ClassKey::P344Lo].captured, 1);
        assert_eq!(cov.classes[&ClassKey::P347].captured, 1);
        assert_eq!(
            cov.decile_captured[7], 1,
            "current event's decile must bump on match"
        );
        assert_eq!(
            cov.decile_captured[1], 1,
            "SHOULD-FIX 2: previous event's decile must bump too"
        );
        assert_eq!(cov.catch_up_unattributed, 0);
        assert!(pending.is_none(), "pending must clear after catch-up");
        // 2 physical pushes landed.
        assert_invariant(&cov, 2);
    }

    /// `delta == 3` catch-up with match + pending → credits 2
    /// (current + prev), surplus 1 → `catch_up_unattributed`. The
    /// invariant still balances because we include
    /// `catch_up_unattributed` in the accounting.
    #[test]
    fn classifier_delta3_catchup_surplus_goes_unattributed() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;
        record(&mut cov, 0, false, ClassKey::P347, 0, &mut pending);
        record(&mut cov, 3, true, ClassKey::P344Lo, 9, &mut pending);
        assert_eq!(cov.classes[&ClassKey::P344Lo].captured, 1);
        assert_eq!(cov.classes[&ClassKey::P347].captured, 1);
        assert_eq!(
            cov.catch_up_unattributed, 1,
            "surplus push (3 - 2 credits) → unattributed"
        );
        assert_invariant(&cov, 3);
    }

    /// `delta == 2` catch-up with mismatch + no pending → 0 credits,
    /// all 2 pushes land in `catch_up_unattributed`. Boundary case
    /// for the surplus-accounting fix.
    #[test]
    fn classifier_delta2_catchup_mismatch_no_pending_all_unattributed() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;
        record(&mut cov, 2, false, ClassKey::P347, 3, &mut pending);
        assert_eq!(cov.classes[&ClassKey::P347].captured, 0);
        assert_eq!(cov.classes[&ClassKey::P347].misattributed, 0);
        assert_eq!(
            cov.catch_up_unattributed, 2,
            "no credits possible → all 2 pushes unattributed"
        );
        assert_invariant(&cov, 2);
    }

    /// End-to-end mixed sequence: interleaves all classifier paths and
    /// checks the HLD §7(4) invariant holds at the end. Models a
    /// realistic steady-state drift pattern:
    ///   - Normal captures (delta=1 match) for bulk 0x347 data.
    ///   - One drift event (delta=0 then delta=1 mismatch).
    ///   - A stray push with no pending (delta=1 mismatch, no pending).
    ///   - A catch-up (delta=2 match with pending).
    ///   - A post-roll orphan injected separately.
    /// Total simulated pushes: 3 + 1 + 1 + 2 + 1 = 8.
    #[test]
    fn classifier_invariant_holds_across_mixed_sequence() {
        let mut cov = CaptureCoverage::default();
        let mut pending: Option<(ClassKey, usize)> = None;

        // 3× clean captures on 0x347 (bulk DRAM bytes).
        record(&mut cov, 1, true, ClassKey::P347, 0, &mut pending);
        record(&mut cov, 1, true, ClassKey::P347, 0, &mut pending);
        record(&mut cov, 1, true, ClassKey::P347, 0, &mut pending);

        // Drift pair: A pends (delta=0), B mismatches → A misattr,
        // B pending.
        record(&mut cov, 0, false, ClassKey::P343(0x43), 1, &mut pending);
        record(&mut cov, 1, false, ClassKey::P344Lo, 2, &mut pending);
        assert_eq!(pending, Some((ClassKey::P344Lo, 2)));

        // C matches → C captured, pending (B) silently dropped
        // (HLD §3.3: clean match clears pending).
        record(&mut cov, 1, true, ClassKey::P347, 3, &mut pending);
        assert!(pending.is_none());

        // Stray push with no pending.
        record(
            &mut cov,
            1,
            false,
            ClassKey::P345(0x01),
            4,
            &mut pending,
        );
        assert!(pending.is_some());

        // Catch-up on a new event D: delta=2, match, with pending from
        // the previous mismatch → 2 credits, no surplus.
        record(&mut cov, 2, true, ClassKey::P347, 5, &mut pending);

        // Post-roll: inject 1 orphan directly (mimics what
        // replay_with_coverage's post-roll path would do).
        cov.post_roll_orphans += 1;

        // Bucket accounting:
        //   captured: 3 (first 3) + 1 (C) + 2 (D catch-up) = 6
        //   misattributed: 1 (A drift) + 1 (stray) = 2
        //   orphans: 1
        // Total = 9 buckets. But physical pushes during main loop =
        // 3 + 0 + 1 + 1 + 1 + 2 = 8, plus 1 orphan = 9 → invariant
        // holds at autopush_count=9.
        assert_eq!(sum_captured(&cov), 6);
        assert_eq!(sum_misattr(&cov), 2);
        assert_eq!(cov.post_roll_orphans, 1);
        assert_eq!(cov.catch_up_unattributed, 0);
        assert_invariant(&cov, 9);
    }

    // ----------------------------------------------------------------
    // `decode_push` — PIO0 SM0 autopush word layout.
    //
    // PIO0 SM0 SHIFTCTRL reads 0x012b0000 on both emulator and silicon;
    // bit 18 (IN_SHIFTDIR) is 0 → shift LEFT. With shift-left, `IN PINS,
    // 10` then `IN PINS, 8` leaves `(addr << 8) | data` in the ISR, so
    // the autopushed word layout is:
    //
    //     bits  7..0 : data  (8 bits)
    //     bits 17..8 : addr  (10 bits)
    // ----------------------------------------------------------------

    #[test]
    fn decode_autopush_word_matches_left_shift_layout() {
        // Sentinel pair chosen to exercise bits across both fields.
        let addr: u16 = 0x347;
        let data: u8 = 0x68;
        let word: u32 = ((addr as u32) << 8) | (data as u32);
        let (decoded_addr, decoded_data) = decode_push(word);
        assert_eq!(decoded_addr, addr, "addr decode under left-shift layout");
        assert_eq!(decoded_data, data, "data decode under left-shift layout");
    }

    #[test]
    fn decode_autopush_word_round_trips_edge_values() {
        for (addr, data) in [
            (0x000u16, 0x00u8),
            (0x3ffu16, 0xffu8),
            (0x000u16, 0xffu8),
            (0x3ffu16, 0x00u8),
        ] {
            let word: u32 = ((addr as u32) << 8) | (data as u32);
            let (decoded_addr, decoded_data) = decode_push(word);
            assert_eq!(
                decoded_addr, addr,
                "addr edge case (addr=0x{:03x}, data=0x{:02x})",
                addr, data
            );
            assert_eq!(
                decoded_data, data,
                "data edge case (addr=0x{:03x}, data=0x{:02x})",
                addr, data
            );
        }
    }

    // ------------------------------------------------------------------
    // PIO disassembler — one focused test per opcode class. The goal
    // isn't a bit-perfect reassembler (see module docs); it's that the
    // human-readable string contains the right mnemonic + key operands.
    // ------------------------------------------------------------------

    #[test]
    fn disasm_jmp_always() {
        // JMP 0x05 (unconditional) — op=0, cond=0, addr=5.
        let s = disasm_pio_instr(0x0005);
        assert!(s.contains("JMP"), "got: {s}");
        assert!(s.contains("0x05"), "got: {s}");
    }

    #[test]
    fn disasm_jmp_conditional_with_delay() {
        // JMP pin, 0x0c, delay 3 — op=0, delay=3, cond=6 (pin), addr=0xc.
        //   [15:13]=000 [12:8]=00011 [7:5]=110 [4:0]=01100 = 0x03cc
        let s = disasm_pio_instr(0x03cc);
        assert!(s.contains("JMP"), "got: {s}");
        assert!(s.contains("pin,"), "got: {s}");
        assert!(s.contains("0x0c"), "got: {s}");
        assert!(s.contains("[3]"), "got: {s}");
    }

    #[test]
    fn disasm_wait_pin() {
        // WAIT 1 PIN 0 — op=1, pol=1, src=1 (PIN), idx=0.
        //   [15:13]=001 [12:8]=00000 [7]=1 [6:5]=01 [4:0]=00000 = 0x20a0
        let s = disasm_pio_instr(0x20a0);
        assert!(s.contains("WAIT"), "got: {s}");
        assert!(s.contains("PIN"), "got: {s}");
        assert!(s.contains(" 1 "), "got: {s}"); // polarity
    }

    #[test]
    fn disasm_wait_gpio() {
        // WAIT 0 GPIO 4 — op=1, pol=0, src=0 (GPIO), idx=4.
        //   [15:13]=001 [12:8]=00000 [7]=0 [6:5]=00 [4:0]=00100 = 0x2004
        let s = disasm_pio_instr(0x2004);
        assert!(s.contains("WAIT"), "got: {s}");
        assert!(s.contains("GPIO"), "got: {s}");
        assert!(s.contains(" 0 "), "got: {s}");
        assert!(s.ends_with("4"), "got: {s}");
    }

    #[test]
    fn disasm_in_pins() {
        // IN PINS, 8 — op=2, src=0 (PINS), n=8.
        //   [15:13]=010 [12:8]=00000 [7:5]=000 [4:0]=01000 = 0x4008
        let s = disasm_pio_instr(0x4008);
        assert!(s.contains("IN"), "got: {s}");
        assert!(s.contains("PINS"), "got: {s}");
        assert!(s.contains("8"), "got: {s}");
    }

    #[test]
    fn disasm_out_pins() {
        // OUT PINS, 1 — op=3, dst=0 (PINS), n=1.
        //   [15:13]=011 [12:8]=00000 [7:5]=000 [4:0]=00001 = 0x6001
        let s = disasm_pio_instr(0x6001);
        assert!(s.contains("OUT"), "got: {s}");
        assert!(s.contains("PINS"), "got: {s}");
        assert!(s.contains("1"), "got: {s}");
    }

    #[test]
    fn disasm_pull_block() {
        // PULL BLOCK — op=4, bit[7]=1 (pull), bit[6]=0, bit[5]=1.
        //   [15:13]=100 [12:8]=00000 [7]=1 [6]=0 [5]=1 [4:0]=00000 = 0x80a0
        let s = disasm_pio_instr(0x80a0);
        assert!(s.contains("PULL"), "got: {s}");
        assert!(s.contains("BLOCK"), "got: {s}");
    }

    #[test]
    fn disasm_push_iffull() {
        // PUSH IFFULL BLOCK — op=4, bit[7]=0 (push), bit[6]=1 (IFFULL),
        //   bit[5]=1 (BLOCK).
        //   [15:13]=100 [12:8]=00000 [7]=0 [6]=1 [5]=1 [4:0]=00000 = 0x8060
        let s = disasm_pio_instr(0x8060);
        assert!(s.contains("PUSH"), "got: {s}");
        assert!(s.contains("IFFULL"), "got: {s}");
        assert!(s.contains("BLOCK"), "got: {s}");
    }

    #[test]
    fn disasm_mov_and_set_and_irq() {
        // MOV Y, X — op=5, dst=2 (Y), mop=0 (none), src=1 (X).
        //   [15:13]=101 [12:8]=00000 [7:5]=010 [4:3]=00 [2:0]=001 = 0xa041
        let s = disasm_pio_instr(0xa041);
        assert!(s.contains("MOV"), "got: {s}");
        assert!(s.contains("Y"), "got: {s}");
        assert!(s.contains("X"), "got: {s}");

        // SET X, 7 — op=7, dst=1 (X), val=7.
        //   [15:13]=111 [12:8]=00000 [7:5]=001 [4:0]=00111 = 0xe027
        let s = disasm_pio_instr(0xe027);
        assert!(s.contains("SET"), "got: {s}");
        assert!(s.contains("X"), "got: {s}");
        assert!(s.contains("7"), "got: {s}");

        // IRQ SET 0 — op=6, CLR=0, WAIT=0, REL=0, idx=0.
        //   [15:13]=110 [12:8]=00000 [7:4]=0000 [3:0]=0000 = 0xc000
        let s = disasm_pio_instr(0xc000);
        assert!(s.contains("IRQ"), "got: {s}");
        assert!(s.contains("SET"), "got: {s}");
    }

    // ------------------------------------------------------------------
    // Dynamic sys_clk_hz — PLL-reprogram harness time-base fix.
    //
    // Regression guard for the bug where the harness used a static
    // 125 MHz constant for ns↔cycle conversions throughout the replay,
    // so events fired after firmware's 125→370 MHz PLL switch landed
    // at the wrong simulated cycle (factor 370/125 = 2.96× off).
    // See `wrk_scratch/picogus-pll-diagnosis.md`.
    // ------------------------------------------------------------------

    /// A mock sink with a mutable `sys_clk_hz` so tests can simulate a
    /// mid-run PLL reprogram. Steps advance `cycles` by the requested
    /// count; the reported clock can be flipped at any time.
    struct VariableClockSink {
        cycles: u64,
        hz: u32,
        /// Cycle at which the reported clock flips to `hz_after`.
        flip_cycle: u64,
        hz_after: u32,
    }

    impl VariableClockSink {
        fn new(initial_hz: u32, flip_cycle: u64, final_hz: u32) -> Self {
            Self {
                cycles: 0,
                hz: initial_hz,
                flip_cycle,
                hz_after: final_hz,
            }
        }
    }

    impl IsaSink for VariableClockSink {
        fn step(&mut self, cycles: u32) {
            self.cycles = self.cycles.wrapping_add(cycles as u64);
        }
        fn cycles(&self) -> u64 {
            self.cycles
        }
        fn drive_pins(&mut self, _iow_low: bool, _ior_low: bool, _ad_bus: u16) {}
        fn sys_clk_hz(&self) -> u32 {
            if self.cycles >= self.flip_cycle {
                self.hz_after
            } else {
                self.hz
            }
        }
    }

    #[test]
    fn replay_adjusts_ns_to_cycles_across_pll_reprogram() {
        // Scenario: sink starts at 125 MHz, flips to 370 MHz when
        // sink.cycles reaches 1_000_000 (= 8 ms of simulated wall
        // time). Two events:
        //   ev[0].ns =  1_000_000 ns (1 ms) — fires purely in the
        //               125 MHz era at ~125_000 cycles.
        //   ev[1].ns = 10_000_000 ns (10 ms) — covers 8 ms pre-flip
        //               (1_000_000 cycles @ 125 MHz) plus 2 ms post-
        //               flip (≈740_000 cycles @ 370 MHz), landing at
        //               ~1_740_000 cycles.
        //
        // The old (buggy) harness used a static 125 MHz for ns↔cycle
        // math, which would have overshot to 10 ms * 125 MHz =
        // 1_250_000 cycles on ev[1] — i.e. it'd fail to model the
        // 2.96× sysclk speedup and effectively give firmware ~34% of
        // the cycles it deserves.
        let mut sink = VariableClockSink::new(125_000_000, 1_000_000, 370_000_000);
        let events = vec![
            TraceEvent {
                ns: 1_000_000,
                port: 0x240,
                value: 0x11,
                kind: TraceKind::Write8,
            },
            TraceEvent {
                ns: 10_000_000,
                port: 0x241,
                value: 0x22,
                kind: TraceKind::Write8,
            },
        ];

        // Post-fix: `replay()` has no sys_clk_hz parameter — the true
        // cadence comes from `IsaSink::sys_clk_hz()` polled per chunk.
        let summary = replay(&mut sink, &events, None, None, 0, 1.0);
        assert_eq!(summary.writes_fired, 2);

        // Post-fix expected cycles:
        //   ev[1] target = 8 ms @ 125 MHz + 2 ms @ 370 MHz
        //                = 1_000_000 + 740_000 = 1_740_000 cycles.
        // Plus drive_write_cycle overhead (~60 cycles per event).
        let final_cycles = sink.cycles();
        let expected_min = 1_700_000u64;
        let expected_max = 1_800_000u64;
        assert!(
            (expected_min..=expected_max).contains(&final_cycles),
            "expected final_cycles in {expected_min}..={expected_max} (post-fix), \
             got {final_cycles} — if it's ≈1.25M the static-clock bug has regressed; \
             if it's ≈3.45M the sim_ns accumulator double-counted"
        );

        // Extra invariant: the clock-flip matters. If we force the
        // same test with no flip (stay at 125 MHz the whole time),
        // ev[1] lands at 10 ms * 125 MHz = 1_250_000 cycles — about
        // 490k fewer.
        let mut sink_noflip = VariableClockSink::new(125_000_000, u64::MAX, 125_000_000);
        let _ = replay(&mut sink_noflip, &events, None, None, 0, 1.0);
        let noflip_cycles = sink_noflip.cycles();
        assert!(
            noflip_cycles < final_cycles,
            "a PLL upclock should cost more cycles of emulation per trace ms \
             (noflip={noflip_cycles}, withflip={final_cycles})"
        );
        assert!(
            (1_240_000..=1_260_000).contains(&noflip_cycles),
            "no-flip run should be ≈10ms × 125 MHz = 1.25M cycles; got {noflip_cycles}"
        );
    }

    #[test]
    fn replay_stable_at_constant_clock_matches_old_behaviour() {
        // Single 125 MHz run — the post-fix loop should land within
        // `drive_write_cycle` overhead of the old fixed-target cycle.
        let mut sink = VariableClockSink::new(125_000_000, u64::MAX, 125_000_000);
        let events = vec![TraceEvent {
            ns: 1_000_000,
            port: 0x240,
            value: 0xAB,
            kind: TraceKind::Write8,
        }];
        let summary = replay(&mut sink, &events, None, None, 0, 1.0);
        assert_eq!(summary.writes_fired, 1);
        // 1 ms @ 125 MHz = 125_000 cycles; plus drive_write_cycle
        // (4 phases × step deltas = 12 + 12 + 25 + 12 = 61 cycles).
        let final_cycles = sink.cycles();
        assert!(
            (125_000..=125_200).contains(&final_cycles),
            "expected ~125_000 cycles, got {final_cycles}"
        );
    }

    #[test]
    fn i2s_capture_set_sys_clk_hz_rescales_rate() {
        use mdpicoem_devices::i2s_capture::I2sCapture;

        // Feed 10 synthetic LRCLK edges with a period of 2604 cycles
        // (= 125M / 48k / 2 × 2 … roughly 48 kHz at 125 MHz). Then
        // switch sys_clk to 370 MHz and expect the reported rate to
        // scale by 370/125.
        let mut cap = I2sCapture::new(125_000_000, 17, 18, 16);

        // Manually script LRCLK edges by toggling the LRCLK pin. We
        // don't care about data bits; just generate edges to populate
        // first_lrclk_cycle / last_lrclk_cycle.
        let half_period: u64 = 1_302; // cycles; 2 × 1302 = 2604 per frame
        let mut cycle: u64 = 0;
        let mut lrclk_high = false;
        for _ in 0..40 {
            // Toggle LRCLK with BCLK low, DOUT low.
            let pads = if lrclk_high { 1u32 << 18 } else { 0 };
            cap.tick(pads, cycle);
            cycle += 1;
            // Hold for half_period-1 cycles without toggling.
            for _ in 1..half_period {
                cap.tick(pads, cycle);
                cycle += 1;
            }
            lrclk_high = !lrclk_high;
        }

        let rate_125 = cap
            .inferred_sample_rate_hz()
            .expect("edges should produce an inferred rate");
        // Flip to 370 MHz — capture timestamps unchanged, only the
        // divisor changes.
        cap.set_sys_clk_hz(370_000_000);
        let rate_370 = cap
            .inferred_sample_rate_hz()
            .expect("edges still present");
        let ratio = rate_370 / rate_125;
        let expected_ratio = 370.0 / 125.0;
        assert!(
            (ratio - expected_ratio).abs() < 1e-6,
            "expected rate to scale by 370/125 = {expected_ratio:.4}, got {ratio:.4} \
             (rate_125={rate_125:.1} Hz, rate_370={rate_370:.1} Hz)"
        );
    }
}
