//! OneROM full-system oracle — boot real firmware end-to-end.
//!
//! Loads the RP2350 bootrom and an unmodified OneROM `.bin` into flash,
//! runs the emulator, watches for OneROM's init to complete (PIO1 +
//! PIO2 both have SMs enabled), snapshots PIO state, decides which
//! oracle branch to run, then drives pin stimulus and observes the
//! served data byte.
//!
//! Stage F from the master PIO differential LLD. Design:
//! `wrk_docs/2026.04.14 - HLD - OneROM Full-System Oracle.md`.
//!
//! Milestones: F.1 (boot without crash) ✔, F.2 (sync) ✔,
//! F.3 (state dump + oracle decision) ✔, F.4 (stimulus + observation) ✔.
//!
//! Usage:
//!   cargo run -p picoem-harness --bin onerom_full_system_rp2350 --release

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::Ordering;

use picoem_harness::{onerom_snapshot_fmt, onerom_sync};
use rp2350_emu::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str = "crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0.bin";

/// Cycle cap for boot. Rough budget: a few million cycles should be
/// more than enough for bootrom + OneROM init at our default
/// emulated clock.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// CTRL register offset within a PIO block.
const PIO_CTRL: u32 = 0x000;

// ---------------------------------------------------------------------------
// test-sdrr-0 pin map (parsed from the bundled fixture at file offset 0x80FC;
// see journal entry 2026-04-15).
//
// CAUTION — pin-map collision:
//   CS2 (GPIO 12) overlaps A12 (ADDR_PINS[12] = GPIO 12).
//   CS3 (GPIO 15) overlaps A11 (ADDR_PINS[11] = GPIO 15).
//
// The fire-24-a fixture multiplexes CS and high-address lanes onto the same
// GPIOs. Consequence for stimulus: when the harness drives both CS2/CS3 AND
// A11/A12, the chosen address determines those CS bits (and vice versa).
//
// The MVP uses address=0 — so A11 = A12 = 0, which means driving CS2 and
// CS3 HIGH (to deassert them) simultaneously with A11/A12 LOW is **not
// physically representable** in this pin layout. We choose to honour the
// CS semantics (CS2/CS3 high = deasserted) and accept that the address on
// pins 12 and 15 will be 1 — the stimulus is still consistent with what
// silicon would see if a host drove CS2/CS3 high while the A11/A12 lanes
// were idle.
//
// If a future fixture needs to assert a specific non-zero address with
// distinct CS levels, this collision must be resolved — likely by changing
// the test fixture to a pin map without the overlap.
// ---------------------------------------------------------------------------

/// Data bus base — D0..D7 ride on GPIO 16..23.
const GPIO_DATA_BASE: u8 = 16;

/// CS lanes. OneROM's config uses CS1 as /OE (low = asserted).
const GPIO_CS1: u8 = 13;
const GPIO_CS2: u8 = 12;
const GPIO_CS3: u8 = 15;

/// A0..A12 wired across these GPIOs (A0..A7 + A8..A9 + A10..A12).
/// A13..A15 unused for this fixture.
///
/// See the module-level CAUTION above: A11 (GPIO 15) and A12 (GPIO 12)
/// overlap CS3 and CS2 respectively.
const ADDR_PINS: [u8; 13] = [7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 14, 15, 12];

/// How many post-sync cycles we drive stimulus for before giving up
/// (or hitting a second WFI).
const POST_SYNC_STIMULUS_CYCLES: u64 = 40;

fn repo_root_relative(rel: &str) -> PathBuf {
    // Harness is invoked from the workspace root via `cargo run`; that's
    // the cwd, and all paths in this file are workspace-relative.
    Path::new(rel).to_path_buf()
}

fn main() -> ExitCode {
    picoem_harness::harness_tracing_init();

    let bootrom_path = repo_root_relative(BOOTROM_PATH);
    let flash_path = repo_root_relative(FLASH_PATH);

    let bootrom = match std::fs::read(&bootrom_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!(
                "failed to read bootrom at {}: {}",
                bootrom_path.display(),
                e
            );
            return ExitCode::from(2);
        }
    };

    let flash = match std::fs::read(&flash_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!(
                "failed to read flash image at {}: {}",
                flash_path.display(),
                e
            );
            return ExitCode::from(2);
        }
    };

    println!(
        "loaded bootrom ({} bytes) and flash ({} bytes)",
        bootrom.len(),
        flash.len()
    );

    // step_quantum=1 so every emu.run(1) advances exactly one CPU
    // instruction — gives a faithful per-instruction trace for
    // diagnosing where main() returns early.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Bootrom bypass.
    //
    // OneROM's `.bin` is a raw flash image whose first 8 bytes are the
    // standard ARM vector table (SP, then Reset). The RP2350 bootrom
    // expects an IMAGE_DEF / PARTITION_TABLE block layout instead; our
    // bootrom run rejects OneROM's image (PC falls to an invalid
    // address ~27 000 cycles in). Working around this for the full-
    // system test by jumping straight to OneROM's reset vector, same
    // as §9 "bootrom + image format" of the LLD.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    // LSB = Thumb indicator; we execute Thumb only, so clear it.
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);
    println!(
        "bypassing bootrom: SP=0x{:08X} PC=0x{:08X}",
        initial_sp, initial_pc
    );

    // OneROM's serving loop is single-core (core 0 runs, core 1 sleeps).
    // Keep core 1 halted so we don't trace its NMI/HardFault noise.
    emu.core_mut(1).halt();

    // Control-flow trace. Log PC whenever it jumps by more than
    // a "natural" amount (a few sequential instructions or a short
    // back-branch) — that captures function entries / exits / long
    // branches and ignores the noise of sequential execution and
    // tight loops. Also force-log the first K steps so we see the
    // very earliest flow. Ring buffer so we can print the last N
    // events at the end.
    let mut trace: Vec<(u64, u32, u32)> = Vec::new(); // (cycle, prev_pc, new_pc)
    let trace_cap: usize = 400;
    const LONG_JUMP_BYTES: u32 = 32; // treat jumps > this as "interesting"
    let mut last_pc: u32 = emu.core(0).regs.pc();
    let record = |cycle: u64, prev: u32, new: u32, trace: &mut Vec<(u64, u32, u32)>| {
        if trace.len() == trace_cap {
            trace.remove(0);
        }
        trace.push((cycle, prev, new));
    };

    // Dense per-cycle PC log. Keeps the last N (pre_pc, post_pc) entries so
    // we can reconstruct the exact instruction sequence that led to the
    // WFI idle loop.
    let mut dense: Vec<(u64, u32, u32)> = Vec::new();
    let dense_cap: usize = 250;

    // Peripheral state change log. Samples key registers periodically and
    // logs any diff vs the last snapshot, tagged with the cycle and PC.
    // This surfaces:
    //   - when RESETS is cleared for PIO/DMA (bringing peripherals out of reset)
    //   - any write to PIO0 CTRL or INSTR_MEM[0..8]
    //   - clock-tree writes
    // Sampling rate: every 16 cycles (covers any write since we're at
    // step_quantum=1, i.e. ~16 instructions).
    #[derive(Default, Clone, Copy, PartialEq)]
    struct PeriphSnapshot {
        resets: u32, // RESETS.RESET (bits set = in reset)
        pio0_ctrl: u32,
        pio1_ctrl: u32,
        pio2_ctrl: u32,
        pio0_im0: u32,
        pio1_im0: u32,
        pio2_im0: u32,
        clk_sys_ctrl: u32,
        clk_sys_sel: u32,
    }
    let mut last_snap = PeriphSnapshot::default();
    let mut periph_log: Vec<(u64, u32, &'static str, u32, u32)> = Vec::new();
    let mut last_sample_cycle: u64 = 0;
    let periph_sample_interval: u64 = 16;

    // Step one instruction at a time for a while, so we can observe
    // each PC transition. This is slow but we're bounded at the
    // boot cycle cap and this is a diagnostic run, not production.
    let mut synced_at: Option<u64> = None;
    let mut sync_report: Option<onerom_sync::SyncReport> = None;
    let mut wfi_loop_hits: u32 = 0;
    // Snapshot of the real DMA's CH1 push-count at sync time. Used to
    // compute the post-sync push delta below, replacing the harness's
    // GlueDma::ch1_pushes() observable (which was a duplicate of the
    // real DMA's `ChannelTransferEvent.push_count`, exposed via
    // `Bus::dma_channel_transfer_event` behind the `testing` feature).
    let mut ch1_pushes_at_sync: u32 = 0;

    // Post-sync observation log: (relative cycle, data_byte, pio2_oe_data_mask).
    let mut obs_log: Vec<(u64, u8, u8)> = Vec::new();
    let mut sync_detect_cycle: Option<u64> = None;

    while emu.cycles() < BOOT_CYCLE_CAP {
        let before_cycles = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after_cycles = emu.cycles();
        let pc = emu.core(0).regs.pc();

        // Safety: cycle counter must advance.
        if after_cycles == before_cycles {
            eprintln!("cycle counter stalled at {} pc=0x{:08X}", before_cycles, pc);
            break;
        }

        // Log a trace entry on any "long jump" (function-call-ish
        // transition) or early warm-up.
        let pc_delta = pc.wrapping_sub(last_pc);
        let is_long_jump =
            !(pc_delta <= LONG_JUMP_BYTES || pc_delta >= 0u32.wrapping_sub(LONG_JUMP_BYTES));
        if is_long_jump || trace.len() < 40 {
            record(after_cycles, last_pc, pc, &mut trace);
        }

        if dense.len() == dense_cap {
            dense.remove(0);
        }
        dense.push((after_cycles, last_pc, pc));

        // Periodic peripheral-state sampling.
        if after_cycles >= last_sample_cycle + periph_sample_interval {
            last_sample_cycle = after_cycles;
            // INSTR_MEM is write-only via MMIO (`read32(0x048..=0x0C4)`
            // returns 0). Use the direct backing-storage accessor so we
            // actually see when firmware programs each PIO block's first
            // instruction slot.
            let snap = PeriphSnapshot {
                resets: emu.bus.read32(0x4002_0000, 0),
                pio0_ctrl: emu.bus.pio[0].read32(0x000),
                pio1_ctrl: emu.bus.pio[1].read32(0x000),
                pio2_ctrl: emu.bus.pio[2].read32(0x000),
                pio0_im0: emu.bus.pio[0].instr_mem()[0] as u32,
                pio1_im0: emu.bus.pio[1].instr_mem()[0] as u32,
                pio2_im0: emu.bus.pio[2].instr_mem()[0] as u32,
                clk_sys_ctrl: emu.bus.read32(0x4001_003C, 0),
                clk_sys_sel: emu.bus.read32(0x4001_0044, 0),
            };
            let mut push = |tag: &'static str, old: u32, new: u32| {
                if old != new {
                    periph_log.push((after_cycles, pc, tag, old, new));
                }
            };
            push("RESETS", last_snap.resets, snap.resets);
            push("PIO0.CTRL", last_snap.pio0_ctrl, snap.pio0_ctrl);
            push("PIO1.CTRL", last_snap.pio1_ctrl, snap.pio1_ctrl);
            push("PIO2.CTRL", last_snap.pio2_ctrl, snap.pio2_ctrl);
            push("PIO0.INSTR[0]", last_snap.pio0_im0, snap.pio0_im0);
            push("PIO1.INSTR[0]", last_snap.pio1_im0, snap.pio1_im0);
            push("PIO2.INSTR[0]", last_snap.pio2_im0, snap.pio2_im0);
            push("CLK_SYS_CTRL", last_snap.clk_sys_ctrl, snap.clk_sys_ctrl);
            push("CLK_SYS_SEL", last_snap.clk_sys_sel, snap.clk_sys_sel);
            last_snap = snap;
        }

        last_pc = pc;

        // Detect WFI loop at 0x10001404 — PC sits between 0x10001404
        // and 0x10001406. Once we've seen this 4 cycles in a row, the
        // CPU has reached its post-main idle state.
        if pc == 0x10001404 || pc == 0x10001406 {
            wfi_loop_hits += 1;
            if wfi_loop_hits > 4 {
                eprintln!(
                    "WFI idle loop reached at cycle {} — main() returned as expected? (see trace)",
                    after_cycles
                );
                break;
            }
        } else {
            wfi_loop_hits = 0;
        }

        // PIO sync check (F.2). Real OneROM uses PIO1 (BLOCK_ADDR, SM0 =
        // address reader) + PIO2 (BLOCK_DATA, SM0+1 = data writer + CS
        // handler). PIO0 is left unused (BLOCK_MONITOR). Sync = "address
        // and data blocks both have SMs enabled".
        if sync_report.is_none() && onerom_sync::is_synced(&mut emu.bus) {
            synced_at = Some(after_cycles);
            sync_detect_cycle = Some(after_cycles);
            let report = onerom_sync::capture_snapshot(&mut emu.bus, after_cycles);
            // Capture the real DMA's CH1 push count so the post-sync
            // delta below is a clean count of pushes inside the
            // observation window. The real DMA peripheral is now the
            // single source of truth — the previous glue-DMA prime
            // is no longer required.
            ch1_pushes_at_sync = emu.bus.dma_channel_transfer_event(1).push_count;
            sync_report = Some(report);

            // Install external-input stimulus: CS1 low, CS2/CS3 high,
            // address=0. Using the external-mask override (see the
            // `gpio_external_*` docs on `Bus`) rather than poking
            // `gpio_in` directly, which would be clobbered by every
            // subsequent `update_gpio` call.
            //
            // NB: per the module-level CAUTION above, driving CS3/CS2
            // HIGH simultaneously forces A11/A12 bits to 1. Data pins
            // (D0..D7 on GPIO 16..23) are PIO-driven and must NOT be
            // masked — that's what we're observing. The stimulus mask
            // therefore covers CS1/CS2/CS3 + all address pins only.
            let stim_mask = (1u32 << GPIO_CS1)
                | (1u32 << GPIO_CS2)
                | (1u32 << GPIO_CS3)
                | ADDR_PINS.iter().fold(0u32, |a, &p| a | (1u32 << p));
            let stim_level = (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
            emu.bus.gpio_external_mask = stim_mask;
            emu.bus
                .gpio_external_in
                .store(stim_level, Ordering::Relaxed);
        }

        // Post-sync: log observations (F.4). The real DMA peripheral
        // drives the chain on its own; no harness-side pump required.
        if let Some(sync_cycle) = sync_detect_cycle {
            let rel_cycle = after_cycles.saturating_sub(sync_cycle);
            let data_byte =
                ((emu.bus.gpio_in.load(Ordering::Relaxed) >> GPIO_DATA_BASE) & 0xFF) as u8;
            let pio2_drives_data = ((emu.bus.pio[2].pad_oe >> GPIO_DATA_BASE) & 0xFF) as u8;
            obs_log.push((rel_cycle, data_byte, pio2_drives_data));

            if rel_cycle >= POST_SYNC_STIMULUS_CYCLES {
                println!();
                println!(
                    "post-sync stimulus window complete ({} cycles)",
                    POST_SYNC_STIMULUS_CYCLES
                );
                break;
            }
        }
    }

    // Dump the trace.
    println!();
    println!(
        "CONTROL-FLOW TRACE (last {} long-jumps, cycle / prev → new):",
        trace.len()
    );
    for (cyc, prev, new) in &trace {
        println!("  cycle {:>10}  0x{:08X} -> 0x{:08X}", cyc, prev, new);
    }

    println!();
    println!(
        "DENSE PC LOG (last {} cycles, every instruction):",
        dense.len()
    );
    for (cyc, prev, new) in &dense {
        println!("  cycle {:>10}  0x{:08X} -> 0x{:08X}", cyc, prev, new);
    }

    println!();
    println!(
        "PERIPHERAL STATE CHANGES ({} events, sampled every {} cycles):",
        periph_log.len(),
        periph_sample_interval
    );
    for (cyc, pc, tag, old, new) in &periph_log {
        println!(
            "  cycle {:>10}  pc=0x{:08X}  {:<14} 0x{:08X} -> 0x{:08X}",
            cyc, pc, tag, old, new
        );
    }

    println!();
    println!("CORE 0 REGISTER DUMP AT STOP:");
    let regs = &emu.core(0).regs;
    println!("  PC  = 0x{:08X}    SP  = 0x{:08X}", regs.pc(), regs.sp());
    println!(
        "  IPSR = 0x{:08X}   (exception number; 0 = thread mode)",
        regs.ipsr()
    );
    for r in 0..8u8 {
        print!("  R{}  = 0x{:08X}  ", r, regs.r[r as usize]);
        if (r + 1) % 4 == 0 {
            println!();
        }
    }
    println!("  LR  = 0x{:08X}", regs.r[14]);

    // Sanity-check: read back what's actually at the last few "interesting"
    // PCs via the bus. If XIP mapping is wrong, the instruction bytes the
    // CPU saw will differ from the .bin contents.
    println!();
    println!("XIP READBACK (what our CPU saw at key PCs):");
    for &(label, addr) in &[
        ("0x10001400 (BL site)", 0x10001400u32),
        ("0x10005090 (BL target)", 0x10005090u32),
        ("0x10005094 (CBZ)", 0x10005094u32),
        ("0x10005098 (prologue?)", 0x10005098u32),
    ] {
        let w = emu.bus.read32(addr, 0);
        println!("  {:32} = 0x{:08X}", label, w);
    }

    // Final state dump.
    let final_cycles = emu.cycles();
    let final_pc = emu.core(0).regs.pc();
    let final_ctrl = emu.bus.pio[0].read32(PIO_CTRL);
    println!();
    println!("FINAL STATE:");
    println!("  cycles      = {}", final_cycles);
    println!("  core 0 pc   = 0x{:08X}", final_pc);
    println!("  PIO0.CTRL   = 0x{:08X}", final_ctrl);

    // Diagnostic: dump PIO0/1/2 (OneROM uses BLOCK_ADDR=1 and BLOCK_DATA=2).
    // INSTR_MEM is write-only via MMIO, so we use the debug accessor to
    // verify programs were actually loaded.
    for b in 0..3 {
        println!();
        println!("PIO{} DIAGNOSTICS:", b);
        println!("  CTRL       = 0x{:08X}", emu.bus.pio[b].read32(0x000));
        println!("  FSTAT      = 0x{:08X}", emu.bus.pio[b].read32(0x004));
        println!("  FLEVEL     = 0x{:08X}", emu.bus.pio[b].read32(0x00C));
        let im = emu.bus.pio[b].instr_mem();
        for i in 0..32usize {
            if i % 8 == 0 {
                print!("  INSTR[{:02}..{:02}]:", i, (i + 7).min(31));
            }
            print!(" {:04X}", im[i]);
            if (i + 1) % 8 == 0 || i == 31 {
                println!();
            }
        }
    }

    // Clock state.
    println!();
    println!("CLOCKS DIAGNOSTICS:");
    println!(
        "  CLK_SYS_CTRL = 0x{:08X}  CLK_SYS_SELECTED = 0x{:08X}",
        emu.bus.read32(0x4001_003C, 0),
        emu.bus.read32(0x4001_0044, 0)
    );
    println!("  sys_clk_hz (computed) = {}", emu.bus.sys_clk_hz());

    // F.3: print snapshot captured at sync + oracle-branch decision.
    if let Some(report) = &sync_report {
        println!();
        println!("SNAPSHOT AT SYNC (cycle {}):", report.cycle);
        print!("{}", onerom_snapshot_fmt::format_snapshot(report));

        let oracle_path = Path::new("crates/picoem-harness/oracles/onerom_2364.trace");
        let (branch, reason) = onerom_snapshot_fmt::decide_oracle_branch(report, oracle_path);
        println!();
        println!("ORACLE DECISION: branch={:?} reason=\"{}\"", branch, reason);
    }

    // F.4: smoke-test verdict on the observation log.
    if !obs_log.is_empty() {
        println!();
        println!(
            "POST-SYNC OBSERVATIONS ({} cycles, columns: rel_cycle data_byte pio2_oe_data):",
            obs_log.len()
        );
        // Delta from the snapshot taken at sync — counts only pushes
        // produced inside the post-sync observation window.
        // `wrapping_sub` defends against u32 wrap (theoretical only;
        // observation windows are well under 2^32 cycles).
        let ch1_pushes = emu
            .bus
            .dma_channel_transfer_event(1)
            .push_count
            .wrapping_sub(ch1_pushes_at_sync);
        let verdict = evaluate_smoke_test(&obs_log, ch1_pushes);
        for (cyc, byte, oe) in &obs_log {
            println!(
                "  rel {:>3}  data=0x{:02X}  pio2_oe=0x{:02X}",
                cyc, byte, oe
            );
        }
        println!();
        println!("  DMA CH1 pushes during observation: {}", ch1_pushes);
        match verdict {
            SmokeVerdict::Pass { byte, start, end } => {
                println!(
                    "SMOKE TEST PASS — stable byte 0x{:02X} observed on D0..D7 at cycles {}..{} \
                     (ch1_pushes={})",
                    byte, start, end, ch1_pushes
                );
            }
            SmokeVerdict::Fail(reason) => {
                println!("SMOKE TEST FAIL — {}", reason);
            }
        }
    }

    match synced_at {
        Some(c) => {
            println!();
            println!(
                "SUCCESS — PIO1 (addr) + PIO2 (data) both have SMs enabled at cycle {}",
                c
            );
            println!(
                "  PIO1.CTRL = 0x{:08X}, PIO2.CTRL = 0x{:08X}",
                emu.bus.pio[1].read32(PIO_CTRL),
                emu.bus.pio[2].read32(PIO_CTRL),
            );
            ExitCode::SUCCESS
        }
        None => {
            println!();
            println!("FAILURE — boot did not reach PIO1+PIO2 SM-enable sync condition");
            ExitCode::FAILURE
        }
    }
}

/// Result of evaluating the post-sync observation log against the
/// piorom.c timing-envelope oracle.
enum SmokeVerdict {
    /// A stable byte was observed on D0..D7 for at least `MIN_STABLE_CYCLES`
    /// consecutive cycles within the expected 8..30 post-CS window, with
    /// PIO2 driving all 8 data lanes (pad_oe mask == 0xFF) throughout.
    Pass {
        byte: u8,
        start: u64,
        end: u64,
    },
    Fail(String),
}

/// Smoke test: within relative cycles 8..30, find a ≥ 3-cycle span where
/// `pio2_drives_data == 0xFF` AND `data_byte` is constant, and verify
/// that the glue DMA CH1 actually pushed at least one byte into PIO2's
/// TX FIFO during the observation window.
///
/// `ch1_pushes` is the count of successful CH1 → PIO2 TX0 pushes since
/// sync — the `> 0` requirement rules out false positives where the
/// observed byte is just PIO2's reset state (default `0xFF` on pad_oe).
///
/// `obs_log` rows: (relative cycle, data byte on D0..D7, PIO2 pad_oe
/// over D0..D7).
fn evaluate_smoke_test(obs_log: &[(u64, u8, u8)], ch1_pushes: u32) -> SmokeVerdict {
    const MIN_STABLE_CYCLES: usize = 3;
    const WINDOW_START: u64 = 8;
    const WINDOW_END: u64 = 30;

    if ch1_pushes == 0 {
        return SmokeVerdict::Fail(
            "DMA never pumped during observation window — glue DMA arming issue. \
             Any stable byte on D0..D7 would be the PIO2 reset state, not served data."
                .to_string(),
        );
    }

    let window: Vec<&(u64, u8, u8)> = obs_log
        .iter()
        .filter(|(c, _, _)| *c >= WINDOW_START && *c <= WINDOW_END)
        .collect();

    if window.is_empty() {
        return SmokeVerdict::Fail(format!(
            "observation log does not cover window {}..{} (got {} rows)",
            WINDOW_START,
            WINDOW_END,
            obs_log.len()
        ));
    }

    let mut run_start: Option<usize> = None;
    let mut run_byte: u8 = 0;
    for (i, (_, byte, oe)) in window.iter().enumerate() {
        let drives_all = *oe == 0xFF;
        match run_start {
            Some(s) if drives_all && *byte == run_byte => {
                let len = i - s + 1;
                if len >= MIN_STABLE_CYCLES {
                    return SmokeVerdict::Pass {
                        byte: run_byte,
                        start: window[s].0,
                        end: window[i].0,
                    };
                }
            }
            _ => {
                if drives_all {
                    run_start = Some(i);
                    run_byte = *byte;
                } else {
                    run_start = None;
                }
            }
        }
    }

    SmokeVerdict::Fail(format!(
        "no {}-cycle stable byte with PIO2 driving all D0..D7 lanes in window {}..{}; \
         max pio2_oe seen = 0x{:02X}",
        MIN_STABLE_CYCLES,
        WINDOW_START,
        WINDOW_END,
        window.iter().map(|(_, _, oe)| *oe).max().unwrap_or(0),
    ))
}
