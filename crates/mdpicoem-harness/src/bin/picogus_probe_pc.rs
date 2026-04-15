// Diagnostic-only: load PicoGUS firmware + real bootrom, step for a
// while, and periodically print core-0 PC + cycle count. Helps locate
// where firmware settles (e.g. the main loop, a HardFault spin, a
// SLEEP-WFI, or PIO init).
//
// This is a scratch tool — not part of the supported harness surface.
// Usage:
//   cargo run -p mdpicoem-harness --release --bin picogus_probe_pc -- \
//       --flash third_party/picogus/picogus-v4.0.0.bin \
//       --bootrom roms/rp2040/bootrom-rp2040-b2.bin \
//       --steps 20000000 --sample-every 500000

use mdrp2040::{Config, EmulatorBuilder};
use std::path::PathBuf;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut flash = None;
    let mut bootrom = None;
    let mut total_steps: u64 = 5_000_000;
    let mut sample_every: u64 = 250_000;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--flash" => { i += 1; flash = Some(PathBuf::from(&args[i])); }
            "--bootrom" => { i += 1; bootrom = Some(PathBuf::from(&args[i])); }
            "--steps" => { i += 1; total_steps = args[i].parse().unwrap(); }
            "--sample-every" => { i += 1; sample_every = args[i].parse().unwrap(); }
            other => { eprintln!("unknown arg: {other}"); std::process::exit(1); }
        }
        i += 1;
    }
    let flash = flash.expect("--flash required");
    let bootrom = bootrom.expect("--bootrom required");

    let flash_bytes = std::fs::read(&flash).expect("read flash");
    let bootrom_bytes = std::fs::read(&bootrom).expect("read bootrom");

    let mut emu = EmulatorBuilder::new(Config { sys_clk_hz: 125_000_000 })
        .flash(flash_bytes)
        .step_quantum(64)
        .build();
    emu.load_bootrom(&bootrom_bytes);
    emu.reset();
    emu.direct_boot_from_flash(0x100);

    eprintln!(
        "reset: core0 pc={:#010x} sp={:#010x} lr={:#010x}",
        emu.cores[0].regs.pc(),
        emu.cores[0].regs.sp(),
        emu.cores[0].regs.lr(),
    );

    // Single-step all instructions, but only log when PC leaves the
    // inner data-init copy loop or a fault trampoline is entered.
    emu.step_quantum = 1;
    let mut last_pc_logged = 0u32;
    let copy_loop = 0x1000022eu32..=0x10000234u32;
    for s in 0..5_000_000u64 {
        let before_pc = emu.cores[0].regs.pc();
        let before_lr = emu.cores[0].regs.lr();
        let before_sp = emu.cores[0].regs.sp();
        let r = emu.cores[0].regs.r;
        let consumed = emu.step();
        let after_pc = emu.cores[0].regs.pc();
        // Log when we exit the copy loop, enter a fault handler, or at
        // a fixed sampling cadence.
        let interesting = !copy_loop.contains(&after_pc)
            && copy_loop.contains(&before_pc);
        let fault = after_pc < 0x1000 && before_pc >= 0x10000000;
        if interesting || fault || s < 16 || (s % 50000 == 0) {
            eprintln!(
                "#{s:>7} pc={before_pc:#010x}->{after_pc:#010x} sp={before_sp:#010x} lr={before_lr:#010x} r1={:#010x} r2={:#010x} r3={:#010x}",
                r[1], r[2], r[3],
            );
            last_pc_logged = after_pc;
        }
        if consumed == 0 { eprintln!("HALT @ {s}"); break; }
        if fault {
            eprintln!("FAULT at step {s}, jumped to {after_pc:#010x}");
            // Trace a few more to see what happens
            for k in 0..8 {
                let p = emu.cores[0].regs.pc();
                let _ = emu.step();
                eprintln!("  fault+{k}: {p:#010x} -> {:#010x}", emu.cores[0].regs.pc());
            }
            break;
        }
    }
    let _ = last_pc_logged;
    emu.step_quantum = 64;

    let mut steps = trace_first;
    let mut last_sample_cycles = emu.cycles();
    let mut last_pc = 0u32;
    let mut same_pc_count = 0u64;
    while steps < total_steps {
        let consumed = emu.step();
        if consumed == 0 {
            eprintln!(
                "HALTED at step {steps}, cycles={}, core0 pc={:#010x} halted={} core1 halted={}",
                emu.cycles(),
                emu.cores[0].regs.pc(),
                emu.cores[0].is_halted(),
                emu.cores[1].is_halted(),
            );
            break;
        }
        steps += 1;
        if steps % sample_every == 0 {
            let pc = emu.cores[0].regs.pc();
            let cycles = emu.cycles();
            let dc = cycles - last_sample_cycles;
            eprintln!(
                "step {:>8} cycles={:>12} (+{:>8}) pc0={:#010x} lr0={:#010x} pc1={:#010x} halted1={}",
                steps, cycles, dc, pc, emu.cores[0].regs.lr(),
                emu.cores[1].regs.pc(), emu.cores[1].is_halted(),
            );
            last_sample_cycles = cycles;
            if pc == last_pc {
                same_pc_count += 1;
                if same_pc_count >= 4 {
                    eprintln!("  PC has not moved across {} samples — likely tight loop", same_pc_count);
                }
            } else {
                same_pc_count = 0;
            }
            last_pc = pc;
        }
    }
    eprintln!(
        "final: cycles={} core0 pc={:#010x} lr={:#010x}",
        emu.cycles(),
        emu.cores[0].regs.pc(),
        emu.cores[0].regs.lr(),
    );
}
