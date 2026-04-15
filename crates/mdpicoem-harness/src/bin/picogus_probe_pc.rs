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
    // Ring-buffer of recent call transitions. Each entry captures
    // (step, from_pc, to_pc, lr, r0, r1, vtor).
    const RING_LEN: usize = 64;
    #[derive(Clone, Copy)]
    struct CallRec { step: u64, from: u32, to: u32, lr: u32, r0: u32, r1: u32, vtor: u32 }
    let mut ring: Vec<CallRec> = Vec::with_capacity(RING_LEN);
    let mut ring_head: usize = 0;
    for s in 0..5_000_000u64 {
        let before_pc = emu.cores[0].regs.pc();
        let before_lr = emu.cores[0].regs.lr();
        let before_sp = emu.cores[0].regs.sp();
        let r = emu.cores[0].regs.r;
        let consumed = emu.step();
        let after_pc = emu.cores[0].regs.pc();
        let after_lr = emu.cores[0].regs.lr();
        // BL/BLX detection: LR changed AND new LR points just past the
        // prior instruction (thumb-tagged). If so, record a call.
        if after_lr != before_lr && (after_lr & 1) == 1 {
            let expected_ret = (before_pc + 2) | 1;
            let expected_ret_wide = (before_pc + 4) | 1;
            if after_lr == expected_ret || after_lr == expected_ret_wide {
                let vtor = emu.bus.read32(0xe000_ed08);
                let r0 = emu.cores[0].regs.r[0];
                let r1 = emu.cores[0].regs.r[1];
                let rec = CallRec { step: s, from: before_pc, to: after_pc, lr: after_lr, r0, r1, vtor };
                if ring.len() < RING_LEN {
                    ring.push(rec);
                } else {
                    ring[ring_head] = rec;
                    ring_head = (ring_head + 1) % RING_LEN;
                }
            }
        }
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
            // Dump SRAM around the crash-time SP to reveal the call chain.
            // Pico-sdk panic() at 0x20000274 pushes {r0,r1,r2,r3} then {lr},
            // so the immediate caller's LR is near SP at panic time.
            let dump_base = before_sp.saturating_sub(16);
            eprint!("    stack@sp-16: ");
            for i in 0..24 {
                let addr = dump_base + (i * 4);
                let w = emu.bus.read32(addr);
                eprint!("{w:#010x} ");
                if i % 4 == 3 { eprint!("\n                  "); }
            }
            eprintln!();
            // Dump instructions around pre-PC (the BKPT/panic site) and
            // panic wrapper. Each Thumb-16 halfword printed as u16.
            let dump_windows = [
                ("crash site", before_pc.saturating_sub(16), before_pc.saturating_add(16)),
                ("panic wrapper", 0x2000_0290, 0x2000_02a8),
                ("hard_assertion_failure", 0x2000_0db0, 0x2000_0de0),
                ("caller @ r5/stack+36", (r[5] & !1).saturating_sub(16), (r[5] & !1).saturating_add(16)),
                ("stack chain 0x20000449", 0x2000_0440, 0x2000_0460),
                ("stack chain 0x20000393", 0x2000_0380, 0x2000_03a8),
            ];
            // Dump the ring buffer of recent calls in chronological order.
            eprintln!("--- last {} BL/BLX calls (chronological) ---", ring.len());
            let order: Box<dyn Iterator<Item = usize>> = if ring.len() < RING_LEN {
                Box::new(0..ring.len())
            } else {
                Box::new((ring_head..RING_LEN).chain(0..ring_head))
            };
            for idx in order {
                let CallRec { step: s, from, to, lr, r0, r1, vtor } = ring[idx];
                eprintln!(
                    "    step={s:>6}  {from:#010x} -> {to:#010x}  r0={r0:#010x} r1={r1:#010x} vtor={vtor:#010x}"
                );
            }
            for (label, lo, hi) in dump_windows {
                eprintln!("--- {label} [{lo:#010x}..{hi:#010x}]");
                let mut addr = lo & !1;
                while addr < hi {
                    let hw = emu.bus.read32(addr & !3);
                    let low = hw as u16;
                    let high = (hw >> 16) as u16;
                    eprintln!("    {:#010x}: {:04x}  {:#010x}: {:04x}", addr, low, addr+2, high);
                    addr += 4;
                }
            }
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
