// PicoGUS trace replayer — mdrp2040 (Cortex-M0+) harness.
//
// Stage 4 of the PicoGUS Integration HLD
// (`wrk_docs/2026.04.14 - HLD - PicoGUS Integration.md`). Reads a CSV
// trace captured from a patched DOSBox-X and drives synthetic ISA bus
// cycles into our `mdrp2040::Emulator`, stepping virtual time forward
// to match each event's wall-clock offset.
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
//         --trace <path>       (required)
//         --duration <secs>    (optional; caps replay to N sim-seconds)
//
// Injection strategy — idealised ISA waveform
// -------------------------------------------
//
// PicoGUS v4.0.0 (`github.com/polpo/picogus` tag `v4.0.0`,
// `sw/isa/isa_io.pio` + `sw/CMakeLists.txt`) pins the ISA bus to the
// following RP2040 GPIOs (via `.define public` directives in
// `isa_io.pio` lines 19–26):
//
//     GPIO  4   IOW_PIN       I/O write strobe  (active low)
//     GPIO  5   IOR_PIN       I/O read strobe   (active low)
//     GPIO  6..15   AD0..AD9  10-bit multiplexed address / 8-bit data
//     GPIO 19   DACK_PIN      DMA acknowledge
//     GPIO 21   IRQ_PIN       firmware-driven IRQ output
//     GPIO 26   IOCHRDY_PIN   sideset output (wait-state handshake)
//     GPIO 27   ADS_PIN       sideset output (address / data mux select)
//     GPIO 28   UART_TX_PIN
//
// PSRAM pins (from `sw/CMakeLists.txt` ~line 260, `build_gus`):
//
//     GPIO  0   PSRAM_PIN_MISO
//     GPIO  1   PSRAM_PIN_CS
//     GPIO  2   PSRAM_PIN_SCK
//     GPIO  3   PSRAM_PIN_MOSI
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
//   2. Assert IOW# low (GPIO4 = 0). Hold `WRITE_ASSERT_CYCLES` cycles.
//   3. Switch the AD0_PIN bus to data bits D0..D7 (or D0..D15 mirrored
//      for write16). Hold another `WRITE_ASSERT_CYCLES`.
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

use mdrp2040::{Config, Emulator, EmulatorBuilder};

// ----------------------------------------------------------------------------
// Pin mapping (sourced from PicoGUS v4.0.0 firmware — see module comment).
// ----------------------------------------------------------------------------

const PIN_IOW: u8 = 4;
const PIN_IOR: u8 = 5;
const PIN_AD0: u8 = 6; // AD0..AD9 occupy GPIO6..GPIO15
const PIN_AD_COUNT: u8 = 10;

/// Address width on the ISA bus for the GUS decode window (0x240..0x24F).
/// The PIO program reads 10 bits — we drive all 10, upper bits zero for
/// values inside the GUS range.
const ADDR_BITS: u32 = 10;

/// Data width for an 8-bit write.
const DATA_BITS: u32 = 8;

/// Cycles to hold a write phase asserted. 37 cycles ≈ 300 ns at 125 MHz.
const WRITE_ASSERT_CYCLES: u32 = 37;

/// Cycles of idle between back-to-back writes.
const WRITE_IDLE_CYCLES: u32 = 12;

/// Default system clock for cycle math. PicoGUS firmware overclocks to
/// 366 MHz but our idealised waveform only requires consistent ns→cycle
/// conversion. 125 MHz matches the HLD's arithmetic ("37 cycles ≈ 300 ns
/// at 125 MHz sysclk") and keeps the math human-readable.
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
}

/// Bitmask of GPIO pins the harness drives externally — IOW#, IOR#, and
/// the 10-bit AD bus AD0..AD9. PSRAM pins (GPIO0..3) are deliberately
/// **not** in this mask; they remain owned by the on-chip SIO/PIO/PSRAM
/// merge inside `Emulator::update_gpio`.
pub const ISA_EXTERNAL_PIN_MASK: u32 = (1u32 << PIN_IOW)
    | (1u32 << PIN_IOR)
    | (((1u32 << PIN_AD_COUNT) - 1) << PIN_AD0);

impl IsaSink for Emulator {
    #[inline]
    fn step(&mut self, cycles: u32) {
        if cycles == 0 {
            return;
        }
        self.run(cycles as u64);
    }

    #[inline]
    fn cycles(&self) -> u64 {
        self.clock.cycles
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

/// One synthetic write cycle: address phase, assert, data phase, deassert.
/// Blocking — returns after idling. Called once per write event.
pub fn drive_write_cycle<S: IsaSink>(sink: &mut S, port: u16, data: u16, wide: bool) {
    let addr_bits = port & ((1u16 << ADDR_BITS) - 1);

    // Phase 0: idle. Address on the bus, IOW high, IOR high.
    sink.drive_pins(false, false, addr_bits);
    sink.step(WRITE_IDLE_CYCLES);

    // Phase 1: assert IOW low with address still on the bus — PIO
    // latches the address on the IOW falling edge.
    sink.drive_pins(true, false, addr_bits);
    sink.step(WRITE_ASSERT_CYCLES);

    // Phase 2: data onto the bus, IOW still asserted. For write16 we
    // also drive the high byte on the low pins — the PIO program only
    // samples 8 bits per cycle (18-bit autopush = 10 addr + 8 data);
    // 16-bit writes split into two 8-bit cycles back-to-back below.
    let data_lo = data & ((1u16 << DATA_BITS) - 1);
    sink.drive_pins(true, false, data_lo);
    sink.step(WRITE_ASSERT_CYCLES);

    // Phase 3: deassert IOW, release the bus. Idle.
    sink.drive_pins(false, false, 0);
    sink.step(WRITE_IDLE_CYCLES);

    if wide {
        // Second 8-bit cycle for the high byte at port+1.
        let addr2 = addr_bits.wrapping_add(1) & ((1u16 << ADDR_BITS) - 1);
        let data_hi = (data >> 8) & ((1u16 << DATA_BITS) - 1);

        sink.drive_pins(false, false, addr2);
        sink.step(WRITE_IDLE_CYCLES);
        sink.drive_pins(true, false, addr2);
        sink.step(WRITE_ASSERT_CYCLES);
        sink.drive_pins(true, false, data_hi);
        sink.step(WRITE_ASSERT_CYCLES);
        sink.drive_pins(false, false, 0);
        sink.step(WRITE_IDLE_CYCLES);
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
/// runs to the end of the trace.
///
/// `post_roll_ns = Some(n)` runs the sink for an additional `n` ns of
/// simulated time after the last fired event (or the duration cap),
/// without firing any further trace events. Lets firmware drain its
/// I2S / DMA pipelines after the last ISA write — Stage 5 needs a few
/// hundred ms of post-roll to capture the trailing audio buffer. `None`
/// or `Some(0)` skips the drain entirely.
pub fn replay<S: IsaSink>(
    sink: &mut S,
    events: &[TraceEvent],
    sys_clk_hz: u32,
    duration_ns: Option<u64>,
    post_roll_ns: Option<u64>,
) -> ReplaySummary {
    let mut summary = ReplaySummary {
        events_total: events.len(),
        ..Default::default()
    };

    for ev in events {
        if let Some(limit) = duration_ns {
            if ev.ns > limit {
                summary.duration_capped = true;
                break;
            }
        }

        // Step the emulator forward to this event's target cycle.
        let target = ns_to_cycles(ev.ns, sys_clk_hz);
        while sink.cycles() < target {
            let remaining = target - sink.cycles();
            // Cap per-call step to a reasonable chunk so we don't hand
            // the emulator absurd `run(n)` values on big gaps — the
            // emulator's own quantum handling already deals with this
            // but smaller chunks keep cycle overshoot bounded.
            let chunk = remaining.clamp(1, 64) as u32;
            let before = sink.cycles();
            sink.step(chunk);
            if sink.cycles() == before {
                // Sink refused to advance (e.g. emulator locked up with
                // no firmware loaded, or both cores halted). Bail out of
                // the fast-forward rather than spin forever — the event
                // still fires below, just at the stalled cycle count.
                if summary.stall_events == 0 {
                    eprintln!(
                        "warning: emulator stalled at cycle {} \
                         — subsequent events fire at the stall cycle",
                        before
                    );
                }
                summary.stall_events += 1;
                break;
            }
        }

        if ev.kind.is_write() {
            let wide = matches!(ev.kind, TraceKind::Write16);
            drive_write_cycle(sink, ev.port, ev.value, wide);
            summary.writes_fired += 1;
        } else {
            summary.reads_skipped += 1;
        }

        summary.final_sim_ns = ev.ns;
    }

    summary.final_cycles = sink.cycles();

    // Post-roll drain. Step the sink an additional `post_roll_ns` worth
    // of cycles WITHOUT firing further trace events, so firmware (e.g.
    // an I2S DMA chain) has wall-time to flush its trailing buffer.
    if let Some(post_ns) = post_roll_ns {
        if post_ns > 0 {
            let target_cycles = ns_to_cycles(post_ns, sys_clk_hz);
            let post_start = sink.cycles();
            let target = post_start.wrapping_add(target_cycles);
            while sink.cycles() < target {
                let remaining = target - sink.cycles();
                let chunk = remaining.clamp(1, 64) as u32;
                let before = sink.cycles();
                sink.step(chunk);
                if sink.cycles() == before {
                    // Same stall guard as above — refused to advance.
                    if summary.stall_events == 0 {
                        eprintln!(
                            "warning: emulator stalled at cycle {} during post-roll",
                            before
                        );
                    }
                    summary.stall_events += 1;
                    break;
                }
            }
            summary.post_roll_cycles = sink.cycles().wrapping_sub(post_start);
        }
    }

    summary
}

// ----------------------------------------------------------------------------
// CLI
// ----------------------------------------------------------------------------

struct Args {
    flash: Option<PathBuf>,
    trace: PathBuf,
    duration_secs: Option<f64>,
    post_roll_secs: f64,
}

/// Default post-roll duration in seconds. 500 ms gives enough simulated
/// time for firmware's I2S DMA chain to flush its trailing audio buffer
/// after the last ISA write — Stage 5 (WAV capture) will rely on this.
const DEFAULT_POST_ROLL_SECS: f64 = 0.5;

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut flash = None;
    let mut trace = None;
    let mut duration_secs = None;
    let mut post_roll_secs = DEFAULT_POST_ROLL_SECS;
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
        trace,
        duration_secs,
        post_roll_secs,
    })
}

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         picogus_diff_rp2040 --trace <path>\n                      \
         [--flash <path>] [--duration <secs>] [--post-roll <secs>]\n\
         \n\
         --flash      Optional 2 MB XIP flash image (.bin). Without it the\n              \
                      emulator runs with empty flash; the replayer still\n              \
                      pokes GPIO inputs — useful for harness tests.\n\
         --trace      Required. CSV file in picogus-tap v1 format.\n\
         --duration   Optional. Stops replay once trace timestamp exceeds\n              \
                      this many simulated seconds.\n\
         --post-roll  Optional (default 0.5 s). After the last trace event\n              \
                      (or the duration cap), step the emulator for this many\n              \
                      additional simulated seconds without firing events —\n              \
                      lets firmware drain trailing I2S / DMA buffers."
    );
}

fn main() {
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
    // step_quantum(1) — per-cycle precision is required for ISA waveform
    // timing. With the default quantum (64), each `sink.step(chunk)` can
    // run up to 63 cycles past the requested target, which can span a
    // full ISA write cycle (37 cycles asserted). Per-cycle stepping
    // bounds overshoot to one cycle, ensuring drive_pins() transitions
    // line up with PIO sampling on the cycle they were issued.
    let mut builder = EmulatorBuilder::new(Config {
        sys_clk_hz: DEFAULT_SYS_CLK_HZ,
    })
    .step_quantum(1);
    if let Some(path) = &args.flash {
        let flash_bytes = load_flash(path)?;
        eprintln!(
            "Loaded flash: {} bytes from {}",
            flash_bytes.len(),
            path.display()
        );
        builder = builder.flash(flash_bytes);
    }
    let mut emu = builder.build();
    emu.reset();

    let duration_ns = args
        .duration_secs
        .map(|s| (s * 1e9).max(0.0) as u64);
    let post_roll_ns = (args.post_roll_secs * 1e9).max(0.0) as u64;

    let wall_start = Instant::now();
    let summary = replay(
        &mut emu,
        &events,
        DEFAULT_SYS_CLK_HZ,
        duration_ns,
        Some(post_roll_ns),
    );
    let wall_elapsed = wall_start.elapsed();

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
        summary.post_roll_cycles as f64 / DEFAULT_SYS_CLK_HZ as f64,
        DEFAULT_SYS_CLK_HZ
    );
    println!("Final cycles:     {}", summary.final_cycles + summary.post_roll_cycles);
    println!("Wall elapsed:     {:.3} s", wall_elapsed.as_secs_f64());
    if wall_elapsed.as_secs_f64() > 0.0 {
        let sim_s = summary.final_sim_ns as f64 / 1e9;
        println!(
            "Ratio (sim/wall): {:.3}x",
            sim_s / wall_elapsed.as_secs_f64()
        );
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
        .build();

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

        let events = vec![TraceEvent {
            ns: 1_000_000,
            port: 0x240,
            value: 0xAB,
            kind: TraceKind::Write8,
        }];

        let summary = replay(&mut emu, &events, 125_000_000, None, None);
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
        let summary = replay(&mut sink, &events, 125_000_000, None, None);
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
        .build();
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
        let summary = replay(&mut sink, &events, 125_000_000, Some(1_000_000_000), None);
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
            125_000_000,
            None,
            Some(1_000_000), // 1 ms
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
        let summary_zero = replay(&mut sink_zero, &events, 125_000_000, None, Some(0));
        assert_eq!(summary_zero.post_roll_cycles, 0);

        let mut sink_none = MockSink::new();
        let summary_none = replay(&mut sink_none, &events, 125_000_000, None, None);
        assert_eq!(summary_none.post_roll_cycles, 0);
        assert_eq!(sink_zero.cycles(), sink_none.cycles());
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
        let summary = replay(&mut sink, &events, 125_000_000, None, None);
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
        .build();

        // Same minimal vector + B-to-self loop as
        // replay_advances_emulator_to_target_cycles.
        let mut rom = vec![0u8; 16];
        rom[0..4].copy_from_slice(&0x2004_2000u32.to_le_bytes());
        rom[4..8].copy_from_slice(&0x0000_0009u32.to_le_bytes());
        rom[8..10].copy_from_slice(&0xe7feu16.to_le_bytes());
        emu.load_image(0x0000_0000, &rom);
        emu.reset();

        let events = vec![TraceEvent {
            ns: 1_000_000,
            port: 0x240,
            value: 0xAB,
            kind: TraceKind::Write8,
        }];

        let summary = replay(
            &mut emu,
            &events,
            125_000_000,
            None,
            Some(1_000_000), // 1 ms post-roll
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
}
