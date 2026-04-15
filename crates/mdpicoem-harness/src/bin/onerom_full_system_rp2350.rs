//! OneROM full-system oracle — boot real firmware end-to-end.
//!
//! Loads the RP2350 bootrom and an unmodified OneROM `.bin` into flash,
//! runs the emulator, watches for OneROM's init to complete (detected
//! via PIO CTRL.SM_ENABLE bits going high on block 0), and then — in
//! follow-up stages — drives input-pin stimulus and observes the
//! served data byte.
//!
//! This is Stage F from the master PIO differential LLD. Design:
//! `wrk_docs/2026.04.14 - HLD - OneROM Full-System Oracle.md`.
//!
//! Current milestone: F.1 (boot without crash). F.2 (sync), F.3 (state
//! dump), F.4 (stimulus) land as follow-up commits in the same binary.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin onerom_full_system_rp2350 --release

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use mdrp2350::{Config, Emulator, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0.bin";

/// Cycle cap for boot. Rough budget: a few million cycles should be
/// more than enough for bootrom + OneROM init at our default
/// emulated clock.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// CTRL register offset within a PIO block.
const PIO_CTRL: u32 = 0x000;

fn repo_root_relative(rel: &str) -> PathBuf {
    // Harness is invoked from the workspace root via `cargo run`; that's
    // the cwd, and all paths in this file are workspace-relative.
    Path::new(rel).to_path_buf()
}

fn main() -> ExitCode {
    let bootrom_path = repo_root_relative(BOOTROM_PATH);
    let flash_path = repo_root_relative(FLASH_PATH);

    let bootrom = match std::fs::read(&bootrom_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", bootrom_path.display(), e);
            return ExitCode::from(2);
        }
    };

    let flash = match std::fs::read(&flash_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read flash image at {}: {}", flash_path.display(), e);
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
        .build();
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

    // Step one instruction at a time for a while, so we can observe
    // each PC transition. This is slow but we're bounded at the
    // boot cycle cap and this is a diagnostic run, not production.
    let mut synced_at: Option<u64> = None;
    let mut wfi_loop_hits: u32 = 0;

    while emu.cycles() < BOOT_CYCLE_CAP {
        let before_cycles = emu.cycles();
        emu.run(1);
        let after_cycles = emu.cycles();
        let pc = emu.core(0).regs.pc();

        // Safety: cycle counter must advance.
        if after_cycles == before_cycles {
            eprintln!(
                "cycle counter stalled at {} pc=0x{:08X}",
                before_cycles, pc
            );
            break;
        }

        // Log a trace entry on any "long jump" (function-call-ish
        // transition) or early warm-up.
        let pc_delta = pc.wrapping_sub(last_pc);
        let is_long_jump = !(pc_delta <= LONG_JUMP_BYTES
            || pc_delta >= 0u32.wrapping_sub(LONG_JUMP_BYTES));
        if is_long_jump || trace.len() < 40 {
            record(after_cycles, last_pc, pc, &mut trace);
        }

        if dense.len() == dense_cap {
            dense.remove(0);
        }
        dense.push((after_cycles, last_pc, pc));

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

        // PIO sync check (original goal).
        if after_cycles % 1024 == 0 {
            let ctrl = emu.bus.pio[0].read32(PIO_CTRL);
            if ctrl & 0x7 == 0x7 {
                synced_at = Some(after_cycles);
                break;
            }
        }
    }

    // Dump the trace.
    println!();
    println!("CONTROL-FLOW TRACE (last {} long-jumps, cycle / prev → new):", trace.len());
    for (cyc, prev, new) in &trace {
        println!("  cycle {:>10}  0x{:08X} -> 0x{:08X}", cyc, prev, new);
    }

    println!();
    println!("DENSE PC LOG (last {} cycles, every instruction):", dense.len());
    for (cyc, prev, new) in &dense {
        println!("  cycle {:>10}  0x{:08X} -> 0x{:08X}", cyc, prev, new);
    }

    println!();
    println!("CORE 0 REGISTER DUMP AT STOP:");
    let regs = &emu.core(0).regs;
    println!("  PC  = 0x{:08X}    SP  = 0x{:08X}", regs.pc(), regs.sp());
    println!("  IPSR = 0x{:08X}   (exception number; 0 = thread mode)", regs.ipsr());
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
        let w = emu.bus.read32(addr);
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

    // Diagnostic: dump a handful of PIO0 registers to see what got
    // configured.
    println!();
    println!("PIO0 DIAGNOSTICS:");
    println!("  CTRL       = 0x{:08X}", emu.bus.pio[0].read32(0x000));
    println!("  FSTAT      = 0x{:08X}", emu.bus.pio[0].read32(0x004));
    println!("  FLEVEL     = 0x{:08X}", emu.bus.pio[0].read32(0x00C));
    for i in 0..9 {
        let insn = emu.bus.pio[0].read32(0x048 + i * 4) & 0xFFFF;
        print!("  INSTR_MEM[{}]=0x{:04X}", i, insn);
        if (i + 1) % 3 == 0 {
            println!();
        }
    }

    // Clock state.
    println!();
    println!("CLOCKS DIAGNOSTICS:");
    println!(
        "  CLK_SYS_CTRL = 0x{:08X}  CLK_SYS_SELECTED = 0x{:08X}",
        emu.bus.read32(0x4001_003C),
        emu.bus.read32(0x4001_0044)
    );
    println!("  sys_clk_hz (computed) = {}", emu.bus.sys_clk_hz());

    match synced_at {
        Some(c) => {
            println!();
            println!("SUCCESS — PIO0 serving config (SM_ENABLE=0x7) reached at cycle {}", c);
            ExitCode::SUCCESS
        }
        None => {
            println!();
            println!("FAILURE — boot did not reach PIO0.CTRL sync condition");
            ExitCode::FAILURE
        }
    }
}
