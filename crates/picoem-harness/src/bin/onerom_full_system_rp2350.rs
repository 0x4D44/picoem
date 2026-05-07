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

use picoem_harness::onerom_fixture::FixtureSpec;
use picoem_harness::{onerom_snapshot_fmt, onerom_sync};
use rp2350_emu::{Config, EmulatorBuilder};

/// SRAM base where the firmware's `preload_rom_image` DMA would deposit
/// the shadow buffer on real silicon — and where the smoke harness now
/// pre-populates a deterministic pattern post-boot-sync. Mirrors
/// [`crate::onerom_serving_oracle::SHADOW_BASE`].
const SHADOW_BASE: u32 = 0x2000_0000;

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
/// (or hitting a second WFI). Bumped from 40 to 200 to give the
/// address-pin sweep below room to drive several distinct addresses
/// with enough dwell time per address for the PIO+DMA chain to
/// propagate the change (PIO1 SM samples → RXF → CH0 → CH1.READ_ADDR
/// update → CH1 read → push edge).
const POST_SYNC_STIMULUS_CYCLES: u64 = 200;

/// Number of master-clock cycles to hold each address-pin pattern
/// before advancing to the next. Picked so the PIO+DMA chain has a
/// steady-state window per pattern even at high latency.
const ADDR_DWELL_CYCLES: u64 = 40;

fn repo_root_relative(rel: &str) -> PathBuf {
    // Harness is invoked from the workspace root via `cargo run`; that's
    // the cwd, and all paths in this file are workspace-relative.
    Path::new(rel).to_path_buf()
}

/// Map a 13-bit address word `addr` (A0..A12) to the GPIO-level mask
/// the harness should drive. `ADDR_PINS[i]` is the GPIO that carries
/// `A[i]`; if bit `i` of `addr` is set, the corresponding GPIO bit is
/// set in the returned mask.
fn addr_word_to_gpio_levels(addr: u16) -> u32 {
    let mut levels = 0u32;
    for (bit_idx, &pin) in ADDR_PINS.iter().enumerate() {
        if (addr >> bit_idx) & 1 == 1 {
            levels |= 1u32 << pin;
        }
    }
    levels
}

/// Address sweep pattern. Each entry is a 13-bit address word the
/// harness drives onto the A0..A12 lanes during the post-sync window.
///
/// Constraint (see CAUTION at file head): CS2/CS3 must remain HIGH
/// while serving (chip selects deasserted) — and CS3=GPIO15 overlaps
/// A11, CS2=GPIO12 overlaps A12. So every address we drive must have
/// **A11=A12=1** (i.e. bits 11 and 12 set, value & 0x1800 == 0x1800).
/// That leaves 11 bits of variation in A0..A10.
///
/// We pick 4 distinct addresses, spaced across the legal range, that
/// produce visibly different `(addr & 0xFF)` byte residues so the
/// downstream byte-equality check (`(last_src_addr & 0xFF) ^ 0x55`)
/// is meaningfully different per address. The scan also starts at the
/// pre-sweep "all-zero address" (0x1800) so the first dwell window
/// behaves identically to the pre-sweep harness.
const ADDR_SWEEP: &[u16] = &[
    0x1800, // baseline: A11=A12=1, A0..A10=0
    0x1855, // A0,A2,A4,A6 set
    0x18AA, // A1,A3,A5,A7 set
    0x1903, // A0,A1,A8 set
    0x1980, // A7,A8 set
];

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

    // Parse the fixture metadata so we know `shadow_size` ahead of the
    // post-sync SHADOW pre-population step. Failure here is fatal — without
    // the size we can't validate per-push `last_src_addr` ranges.
    let fixture_spec = match FixtureSpec::from_flash(&flash) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to parse OneROM fixture metadata: {}", e);
            return ExitCode::from(2);
        }
    };
    println!(
        "parsed fixture spec: shadow_size = 0x{:X} bytes ({} KiB)",
        fixture_spec.shadow_size,
        fixture_spec.shadow_size / 1024
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

    // Per-push-edge log for byte-correctness validation. Each entry records
    // a CH1 push detected during the observation window:
    //   (relative cycle, last_src_addr, byte read back from that address,
    //    address-pin word being driven at this cycle).
    // Populated when `bus.dma_channel_transfer_event(1).push_count`
    // increments cycle-over-cycle. The trailing address-pin word is
    // captured so the verdict can require `last_src_addr` to track the
    // stimulus — a stuck CH1.READ_ADDR (the bug fixed in rp2350-emu
    // 0.2.3) cannot satisfy that requirement.
    let mut ch1_push_edges: Vec<(u64, u32, u8, u16)> = Vec::new();
    // Tracks `push_count` between cycles so we can detect single-cycle
    // edges. Initialised at sync time below.
    let mut ch1_push_count_prev: u32 = 0;

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
            ch1_push_count_prev = ch1_pushes_at_sync;
            sync_report = Some(report);

            // Pre-populate SHADOW with a deterministic pattern so we can
            // validate that CH1 reads from the correct address AND returns
            // the correct byte. The firmware's `preload_rom_image` DMA
            // does not actually run in EMU today, so without this
            // population SHADOW would be all zeros and any "stable byte"
            // observed on D0..D7 cannot be served data — see review
            // feedback for Stage 7. Pattern: byte = (addr & 0xFF) ^ 0x55.
            // The XOR ensures we don't confuse it with literal address
            // bytes; the function is deterministic so the reader can
            // recompute the expected byte from `last_src_addr` alone.
            //
            // Population happens AFTER boot-sync (so we don't race the
            // firmware's own SRAM writes during init) but BEFORE the
            // observation window logs any pushes — that's why this lives
            // in the sync-detect arm, not at start-of-main.
            let shadow_size = fixture_spec.shadow_size as u32;
            for offset in 0..shadow_size {
                let addr = SHADOW_BASE.wrapping_add(offset);
                let byte = ((addr & 0xFF) ^ 0x55) as u8;
                emu.bus.write8(addr, byte, 0);
            }
            println!(
                "pre-populated SHADOW [{:#010X}..{:#010X}) with pattern (addr & 0xFF) ^ 0x55",
                SHADOW_BASE,
                SHADOW_BASE + shadow_size
            );

            // Install external-input stimulus: CS1 low, CS2/CS3 high,
            // address sweep follows below. Using the external-mask
            // override (see the `gpio_external_*` docs on `Bus`)
            // rather than poking `gpio_in` directly, which would be
            // clobbered by every subsequent `update_gpio` call.
            //
            // NB: per the module-level CAUTION above, driving CS3/CS2
            // HIGH simultaneously forces A11/A12 bits to 1. Data pins
            // (D0..D7 on GPIO 16..23) are PIO-driven and must NOT be
            // masked — that's what we're observing. The stimulus mask
            // therefore covers CS1/CS2/CS3 + all address pins only.
            //
            // Pre-2026-05-07 the harness drove a fixed all-zero
            // address for the entire post-sync window, which made
            // `last_src_addr` trivially constant — even when CH1 was
            // never updating its `READ_ADDR` register (the
            // `Bus::tick_dma` `mem::take` borrow trap, fixed in
            // rp2350-emu 0.2.3). The sweep below drives several
            // distinct address words so a stuck `READ_ADDR` cannot
            // hide behind a single uniform stimulus.
            let stim_mask = (1u32 << GPIO_CS1)
                | (1u32 << GPIO_CS2)
                | (1u32 << GPIO_CS3)
                | ADDR_PINS.iter().fold(0u32, |a, &p| a | (1u32 << p));
            // CS2/CS3 high (deasserted) is the constant; address bits
            // get OR'd in per-step from the sweep schedule. CS1 stays
            // low (asserted = serving).
            let cs_level = (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
            let initial_addr_level = addr_word_to_gpio_levels(ADDR_SWEEP[0]);
            emu.bus.gpio_external_mask = stim_mask;
            emu.bus
                .gpio_external_in
                .store(cs_level | initial_addr_level, Ordering::Relaxed);
        }

        // Post-sync: log observations (F.4). The real DMA peripheral
        // drives the chain on its own; no harness-side pump required.
        if let Some(sync_cycle) = sync_detect_cycle {
            let rel_cycle = after_cycles.saturating_sub(sync_cycle);

            // Address-pin sweep: pick which address word should be
            // driven during this rel_cycle and update the external-in
            // override. Re-asserting the same level is a cheap atomic
            // store, so we don't bother gating on "changed" here.
            let sweep_idx =
                ((rel_cycle / ADDR_DWELL_CYCLES) as usize).min(ADDR_SWEEP.len() - 1);
            let active_addr_word = ADDR_SWEEP[sweep_idx];
            let cs_level = (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
            let addr_level = addr_word_to_gpio_levels(active_addr_word);
            emu.bus
                .gpio_external_in
                .store(cs_level | addr_level, Ordering::Relaxed);

            let data_byte =
                ((emu.bus.gpio_in.load(Ordering::Relaxed) >> GPIO_DATA_BASE) & 0xFF) as u8;
            let pio2_drives_data = ((emu.bus.pio[2].pad_oe >> GPIO_DATA_BASE) & 0xFF) as u8;
            obs_log.push((rel_cycle, data_byte, pio2_drives_data));

            // Detect a CH1 push edge: `push_count` is monotonically
            // increasing per `dma::ChannelTransferEvent`'s reader contract,
            // so any cycle-over-cycle delta is one or more new pushes.
            // Capture `last_src_addr` and re-read the SHADOW byte at that
            // address (SHADOW is immutable post-population, so this is
            // exactly the byte CH1 fed to PIO2's TX FIFO). Also record
            // the address word the harness was driving at this cycle so
            // the verdict can correlate `last_src_addr` with the stimulus.
            let ev = emu.bus.dma_channel_transfer_event(1);
            if ev.push_count != ch1_push_count_prev {
                let src = ev.last_src_addr;
                let byte = emu.bus.read8(src, 0);
                ch1_push_edges.push((rel_cycle, src, byte, active_addr_word));
                ch1_push_count_prev = ev.push_count;
            }

            if rel_cycle >= POST_SYNC_STIMULUS_CYCLES {
                println!();
                println!(
                    "post-sync stimulus window complete ({} cycles, {} sweep steps)",
                    POST_SYNC_STIMULUS_CYCLES,
                    ADDR_SWEEP.len()
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

    // F.4: smoke-test verdict — byte-correctness against the SHADOW
    // pattern populated at sync. The verdict no longer relies on
    // sampling the GPIO data bus (that was a liveness check at best,
    // and "stable 0xFF" was just PIO2's reset-state pad_oe). Instead,
    // every CH1 push edge captured during the observation window must:
    //   1. Have `last_src_addr` inside `[SHADOW_BASE, SHADOW_BASE + shadow_size)`
    //   2. Carry the byte `(last_src_addr & 0xFF) ^ 0x55`
    let mut smoke_passed = false;
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
        for (cyc, byte, oe) in &obs_log {
            println!(
                "  rel {:>3}  data=0x{:02X}  pio2_oe=0x{:02X}",
                cyc, byte, oe
            );
        }
        println!();
        println!("  DMA CH1 pushes during observation: {}", ch1_pushes);
        if !ch1_push_edges.is_empty() {
            println!(
                "POST-SYNC CH1 PUSH EDGES ({} edges, columns: rel_cycle src_addr byte_at_src expected addr_pin_word):",
                ch1_push_edges.len()
            );
            for (cyc, src, byte, addr_word) in &ch1_push_edges {
                let expected = ((src & 0xFF) ^ 0x55) as u8;
                println!(
                    "  rel {:>3}  src=0x{:08X}  byte=0x{:02X}  expected=0x{:02X}  addr_pins=0x{:04X}",
                    cyc, src, byte, expected, addr_word
                );
            }
        }

        let verdict = evaluate_smoke_test(
            &ch1_push_edges,
            ch1_pushes,
            SHADOW_BASE,
            fixture_spec.shadow_size as u32,
            &obs_log,
            ADDR_SWEEP,
            ADDR_DWELL_CYCLES,
        );
        match verdict {
            SmokeVerdict::Pass {
                edges,
                distinct_src_addrs,
                pin_matches,
            } => {
                println!(
                    "SMOKE TEST PASS — {} CH1 push edges across {} distinct \
                     src addresses, all from SHADOW range and all bytes \
                     matched (addr & 0xFF) ^ 0x55 (ch1_pushes={}); \
                     {}/{} sweep addresses produced the correct data byte \
                     on D0..D7 with PIO2 driving",
                    edges, distinct_src_addrs, ch1_pushes,
                    pin_matches, ADDR_SWEEP.len()
                );
                smoke_passed = true;
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
                "{} — PIO1 (addr) + PIO2 (data) both have SMs enabled at cycle {}",
                if smoke_passed { "SUCCESS" } else { "PARTIAL" },
                c
            );
            println!(
                "  PIO1.CTRL = 0x{:08X}, PIO2.CTRL = 0x{:08X}",
                emu.bus.pio[1].read32(PIO_CTRL),
                emu.bus.pio[2].read32(PIO_CTRL),
            );
            // Smoke verdict feeds the exit code now (was previously
            // SUCCESS-on-sync, FAIL-otherwise — which let byte-wrong
            // serves slip through). Sync without a passing smoke
            // verdict is reported as PARTIAL above and exits FAILURE.
            if smoke_passed {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        None => {
            println!();
            println!("FAILURE — boot did not reach PIO1+PIO2 SM-enable sync condition");
            ExitCode::FAILURE
        }
    }
}

/// Result of the byte-correctness smoke test on CH1 push edges.
enum SmokeVerdict {
    /// All verdict criteria passed:
    ///   1. At least one CH1 push edge was observed, every edge's
    ///      `last_src_addr` was inside the SHADOW range, every pushed
    ///      byte matched the deterministic pattern `(addr & 0xFF) ^ 0x55`,
    ///      and the harness saw `last_src_addr` take more than one
    ///      distinct value.
    ///   2. For every sweep address word that produced at least one CH1
    ///      push during its dwell window, the data pins (D0..D7) showed
    ///      a byte from that dwell's push set on at least one cycle
    ///      where PIO2 was actively driving the pads. Dwells with no
    ///      pushes are skipped (inconclusive, not failure). At least
    ///      **two** dwells must have produced a positive match —
    ///      mirrors criterion 4's distinct-src-addrs cardinality
    ///      requirement. A single match is too easy to satisfy with a
    ///      stuck-byte regression that happens to align with one
    ///      dwell.
    Pass {
        edges: usize,
        distinct_src_addrs: usize,
        pin_matches: usize,
    },
    Fail(String),
}

/// Smoke verdict: byte-correctness validation on CH1 push edges
/// captured during the observation window.
///
/// `push_edges` rows: (relative cycle, last_src_addr, byte read back
/// from `last_src_addr` at the cycle of the push, address-pin word
/// being driven at the time of the push). SHADOW is populated at
/// sync time and never written again, so the byte read here is
/// exactly the byte CH1 fed to PIO2's TX FIFO.
///
/// `ch1_pushes` is the total `push_count` delta over the window —
/// included for the zero-push fast path so we can give a more useful
/// diagnostic than "0 edges captured".
///
/// PASS criteria:
///   1. At least one push edge.
///   2. Every edge's `last_src_addr` ∈ [shadow_base, shadow_base + shadow_size).
///   3. Every pushed byte equals `((last_src_addr & 0xFF) ^ 0x55) as u8`.
///   4. The set of distinct `last_src_addr` values across all push
///      edges has cardinality > 1.
///
/// Criterion 4 is what specifically catches a stuck CH1.READ_ADDR
/// (the rp2350-emu pre-0.2.3 `Bus::tick_dma` borrow trap, where every
/// DMA-to-DMA write was swallowed by the `mem::take` stand-in). The
/// address-pin sweep drives several distinct address words; if the
/// CH0 → CH1.READ_ADDR pipe is alive, push edges land on multiple
/// distinct SHADOW offsets. If the pipe is broken, every push lands
/// on the same firmware-initialised offset and the cardinality
/// collapses to 1.
///
/// Criterion 5 (observation-based pin-data correctness): for each
/// sweep address word, find the set of `byte_at_src` values pushed
/// by CH1 during that dwell window, then look for at least one cycle
/// in the dwell where PIO2 drove a byte from that set onto D0..D7
/// (`pio2_oe_data_mask == 0xFF`). Dwells with no CH1 pushes are
/// skipped (inconclusive); at least **two** dwells must produce a
/// positive match (mirrors criterion 4's distinct-src-addrs
/// cardinality requirement — a single positive match is satisfiable
/// by "PIO2 stuck driving the baseline byte" regressions that align
/// with one dwell by chance, whereas requiring two matches across the
/// sweep proves the SDRR pipe is genuinely tracking address-pin
/// changes). We can't compute the firmware's pin-pattern → SHADOW-
/// offset translation from the outside, but we can observe what
/// offset CH1 actually read from per dwell and assert PIO2 put that
/// byte on the pins — which is the same claim the brief's formula
/// tried to make.
fn evaluate_smoke_test(
    push_edges: &[(u64, u32, u8, u16)],
    ch1_pushes: u32,
    shadow_base: u32,
    shadow_size: u32,
    obs_log: &[(u64, u8, u8)],
    addr_sweep: &[u16],
    addr_dwell_cycles: u64,
) -> SmokeVerdict {
    if ch1_pushes == 0 {
        return SmokeVerdict::Fail(
            "SMOKE TEST FAIL — DMA CH1 produced 0 pushes during the \
             observation window. Possible causes:\n  \
               - Boot-sync occurred but the firmware never armed CH1.\n  \
               - CH1 paced on a DREQ source that never asserts (PIO1 or \
             PIO2 SM not pushing data — check FSTAT).\n  \
               - Per-cycle DMA tick gate (RESETS_RESET bit 2 set, or \
             Bus::tick_peripherals's needs_tick() short-circuit \
             misfiring).\n  \
               - Test hook bug — channel_transfer_event not being \
             updated by Dma::issue_transfer."
                .to_string(),
        );
    }

    if push_edges.is_empty() {
        return SmokeVerdict::Fail(format!(
            "ch1_pushes={} but no push edges were captured by the \
             cycle-by-cycle edge detector. Likely a harness bug: \
             multiple pushes happened in a single observed cycle and \
             we sampled `last_src_addr` after the final one. Tighten \
             the per-cycle detection.",
            ch1_pushes
        ));
    }

    let shadow_end = shadow_base.wrapping_add(shadow_size);
    for (cyc, src, byte, addr_word) in push_edges {
        if !(*src >= shadow_base && *src < shadow_end) {
            return SmokeVerdict::Fail(format!(
                "CH1 push at rel cycle {} read from src=0x{:08X}, which \
                 is outside SHADOW range [0x{:08X}..0x{:08X}). \
                 Address pins were 0x{:04X} at this cycle. \
                 Clue: CH1 is firing but reading from the wrong address \
                 — possible address-deposit-from-CH0 issue, wrong PIO \
                 program output, or stale `last_src_addr` from a prior \
                 transfer.",
                cyc, src, shadow_base, shadow_end, addr_word
            ));
        }
        let expected = ((*src & 0xFF) ^ 0x55) as u8;
        if *byte != expected {
            return SmokeVerdict::Fail(format!(
                "CH1 push at rel cycle {} from src=0x{:08X} returned \
                 byte=0x{:02X}, expected=0x{:02X}. Address pins were \
                 0x{:04X} at this cycle. Clue: CH1 is reading the right \
                 address but the byte came back wrong — possible SHADOW \
                 corruption (something else is writing to the populated \
                 range) or a wrong-byte-lane bug in the DMA read path.",
                cyc, src, byte, expected, addr_word
            ));
        }
    }

    // Criterion 4: the address-pin sweep should have driven CH1's
    // `READ_ADDR` register through multiple distinct values. Count
    // distinct `last_src_addr` entries and require > 1.
    let mut distinct: Vec<u32> = push_edges.iter().map(|(_, src, _, _)| *src).collect();
    distinct.sort_unstable();
    distinct.dedup();
    if distinct.len() <= 1 {
        let stuck_at = distinct.first().copied().unwrap_or(0);
        return SmokeVerdict::Fail(format!(
            "CH1 fired {} times during the address-pin sweep but \
             `last_src_addr` only ever took {} distinct value(s) \
             (stuck at 0x{:08X}). The sweep drove {} distinct address \
             words on A0..A12, so CH1's READ_ADDR register should be \
             tracking the stimulus. Almost certainly the CH0 → \
             CH1.READ_ADDR DMA-to-DMA write is being dropped — exactly \
             the rp2350-emu pre-0.2.3 `Bus::tick_dma` `mem::take` \
             borrow trap. Check the regression test \
             `dma::tests::dma_to_dma_write_during_tick_lands_on_live_dma`.",
            push_edges.len(),
            distinct.len(),
            stuck_at,
            ADDR_SWEEP.len(),
        ));
    }

    // Criterion 5 (observation-based pin-data correctness): for each
    // address word in the sweep, the data pins (D0..D7, GPIO 16..23)
    // must — at least once during the dwell window, while PIO2 is
    // actively driving the pads (`pio2_oe_data_mask == 0xFF`) — show a
    // byte that CH1 actually pushed during the same window.
    //
    // We can't compute the firmware's internal pin-pattern → SHADOW-
    // offset lift function from the outside (when the harness drives
    // pin pattern 0x1855, CH1 reads from 0x200090AA, not 0x20001855 —
    // OneROM does its own bit-shuffle/bank-offset translation), so we
    // can't compare against `((shadow_base + addr_word) & 0xFF) ^ 0x55`.
    // What we CAN observe is the byte CH1 read from `last_src_addr` at
    // each push (`byte_at_src`) — and that's exactly the byte PIO2
    // should have shifted out for this address word. The contract
    // shifts from "pin byte equals SHADOW[lift(pattern)]" (formula we
    // can't compute) to the equivalent observable: "pin byte equals one
    // of the bytes CH1 pushed during this dwell".
    //
    // Dwells with no CH1 pushes are skipped (inconclusive — CH1 may not
    // have caught up yet, particularly at the tail of an early-exit
    // run), but at least one dwell must produce a positive match —
    // passing without ANY observation that exercised the serve pipeline
    // is meaningless.
    let mut pin_matches = 0usize;
    for (sweep_idx, &addr_word) in addr_sweep.iter().enumerate() {
        let dwell_start = (sweep_idx as u64) * addr_dwell_cycles;
        let dwell_end = dwell_start + addr_dwell_cycles;

        // Collect the set of bytes CH1 pushed during this dwell. The
        // edges are stored with the address-pin word that was being
        // driven at the time; we filter by relative cycle within the
        // dwell window rather than by `addr_word` so we're tolerant of
        // edges captured one cycle late (the harness samples
        // `last_src_addr` after the GPIO-in store).
        let mut push_bytes: Vec<u8> = push_edges
            .iter()
            .filter(|(cyc, _, _, _)| *cyc >= dwell_start && *cyc < dwell_end)
            .map(|(_, _, byte, _)| *byte)
            .collect();
        push_bytes.sort_unstable();
        push_bytes.dedup();

        if push_bytes.is_empty() {
            // Inconclusive but not failure — no CH1 push activity in
            // this dwell to validate against. Move on.
            continue;
        }

        let mut hit = false;
        let mut cycles_with_oe = 0usize;
        let mut last_byte = 0u8;
        for (cyc, byte, oe) in obs_log {
            if *cyc >= dwell_start && *cyc < dwell_end && *oe == 0xFF {
                cycles_with_oe += 1;
                last_byte = *byte;
                if push_bytes.contains(byte) {
                    hit = true;
                    break;
                }
            }
        }
        if !hit {
            let push_bytes_disp: Vec<String> =
                push_bytes.iter().map(|b| format!("0x{:02X}", b)).collect();
            return SmokeVerdict::Fail(format!(
                "address-pin sweep word #{}/0x{:04X} (dwell rel cycles \
                 {}..{}): CH1 pushed bytes {{{}}} during this dwell, \
                 but no cycle in the dwell window showed any of those \
                 bytes on D0..D7 with PIO2 driving (pad_oe == 0xFF). \
                 Cycles with PIO2 driving in this window: {}; last byte \
                 seen on D0..D7: 0x{:02X}. Clue: CH1 produced \
                 byte-correct pushes (criteria 1–4 passed) but PIO2 \
                 either did not assert OE in this window, or shifted out \
                 a byte that didn't come from CH1's recent pushes — \
                 check PIO2's program, FSTAT/FLEVEL, and the SM clock \
                 divisor.",
                sweep_idx,
                addr_word,
                dwell_start,
                dwell_end,
                push_bytes_disp.join(", "),
                cycles_with_oe,
                last_byte,
            ));
        }
        pin_matches += 1;
    }

    if pin_matches < 2 {
        return SmokeVerdict::Fail(format!(
            "address-pin sweep produced fewer than 2 positive pin-data \
             matches ({}/{} dwells matched). Mirrors criterion 4's \
             distinct-src-addrs cardinality requirement: passing the \
             smoke verdict on a single dwell match would let \"PIO2 \
             stuck driving the baseline byte\" failure modes slip \
             through — a stuck-byte regression looks identical to a \
             working harness for whichever single dwell happens to \
             match by chance. Requiring >= 2 matches across the sweep \
             ensures the SDRR pipe is genuinely tracking address-pin \
             changes. Sweep config: {} sweep words × {} cycles, total \
             pushes captured: {}.",
            pin_matches,
            addr_sweep.len(),
            addr_sweep.len(),
            addr_dwell_cycles,
            push_edges.len(),
        ));
    }

    SmokeVerdict::Pass {
        edges: push_edges.len(),
        distinct_src_addrs: distinct.len(),
        pin_matches,
    }
}
