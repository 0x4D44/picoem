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

    // Single-step all instructions, but only log at function-call
    // transitions and exception entry. Exception entry is the real
    // signal we want — LR becoming 0xFFFFFFxx (EXC_RETURN) after a
    // step means the core vectored to an exception handler.
    emu.step_quantum = 1;
    let mut last_in_exception = false;
    for s in 0..5_000_000u64 {
        let before_pc = emu.cores[0].regs.pc();
        let before_lr = emu.cores[0].regs.lr();
        let before_sp = emu.cores[0].regs.sp();
        let r = emu.cores[0].regs.r;
        let consumed = emu.step();
        let after_pc = emu.cores[0].regs.pc();
        let after_lr = emu.cores[0].regs.lr();
        let in_exception = (after_lr & 0xFFFF_FFF0) == 0xFFFF_FFF0;
        let entered_exception = in_exception && !last_in_exception;
        let left_exception = !in_exception && last_in_exception;
        last_in_exception = in_exception;
        if entered_exception {
            eprintln!(
                "*** EXC ENTRY #{s}: pc {before_pc:#010x} -> {after_pc:#010x}, lr {before_lr:#010x} -> {after_lr:#010x}, sp {before_sp:#010x}",
            );
            eprintln!(
                "    r0={:#010x} r1={:#010x} r2={:#010x} r3={:#010x} r4={:#010x} r5={:#010x} r6={:#010x} r7={:#010x}",
                r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7],
            );
            break;
        }
        if left_exception {
            eprintln!(
                "EXC RETURN #{s}: pc {before_pc:#010x} -> {after_pc:#010x}",
            );
        }
        if consumed == 0 { eprintln!("HALT @ {s}"); break; }
    }
    emu.step_quantum = 64;

    let mut steps = 0u64;
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
