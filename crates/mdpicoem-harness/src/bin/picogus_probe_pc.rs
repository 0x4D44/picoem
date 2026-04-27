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

use mdpicoem_harness::picogus_pins::{ISA_EXTERNAL_PIN_MASK, ISA_IOR, ISA_IOW};
use mdrp2040::bus::{CLOCKS_BASE, PIO0_BASE, PIO1_BASE, PLL_SYS_BASE, PLL_USB_BASE};
use mdrp2040::{Config, EmulatorBuilder};
use std::path::PathBuf;

fn main() {
    mdpicoem_harness::harness_tracing_init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut flash = None;
    let mut bootrom = None;
    let mut total_steps: u64 = 5_000_000;
    let mut sample_every: u64 = 250_000;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--flash" => {
                i += 1;
                flash = Some(PathBuf::from(&args[i]));
            }
            "--bootrom" => {
                i += 1;
                bootrom = Some(PathBuf::from(&args[i]));
            }
            "--steps" => {
                i += 1;
                total_steps = args[i].parse().unwrap();
            }
            "--sample-every" => {
                i += 1;
                sample_every = args[i].parse().unwrap();
            }
            other => {
                eprintln!("unknown arg: {other}");
                std::process::exit(1);
            }
        }
        i += 1;
    }
    let flash = flash.expect("--flash required");
    let bootrom = bootrom.expect("--bootrom required");

    let flash_bytes = std::fs::read(&flash).expect("read flash");
    let bootrom_bytes = std::fs::read(&bootrom).expect("read bootrom");

    let mut emu = EmulatorBuilder::new(Config {
        sys_clk_hz: 125_000_000,
    })
    .flash(flash_bytes)
    .step_quantum(64)
    .build()
    .expect("Serial build is infallible");
    emu.load_bootrom(&bootrom_bytes);
    emu.reset();
    emu.direct_boot_from_flash(0x100);

    // Prime ISA pins to idle (IOW# = IOR# = HIGH) so firmware probes of
    // the ISA bus don't see phantom writes / phantom reads. This mirrors
    // picogus_diff_rp2040's replay sink state between events.
    emu.bus.external_gpio_in_mask = ISA_EXTERNAL_PIN_MASK;
    emu.bus.external_gpio_in_override = (1u32 << ISA_IOW) | (1u32 << ISA_IOR);

    // Run past runtime_init so .data copy completes, then patch SRAM.
    // test_psram at SRAM 0x20012FA4 takes ~1 hour to complete; stub it
    // to return 0 (success) immediately so we can debug downstream init.
    emu.step_quantum = 64;
    for _ in 0..200_000u64 {
        if emu.step().expect("Serial step is infallible") == 0 {
            break;
        }
    }
    // Patch: MOVS R0, #0 (0x2000) + BX LR (0x4770) at test_psram entry
    emu.bus.write32(0x2001_2FA4, 0x4770_2000);
    eprintln!("patched SRAM 0x20012FA4: test_psram -> return 0");

    eprintln!(
        "reset: core0 pc={:#010x} sp={:#010x} lr={:#010x} (ISA idle primed)",
        emu.cores[0].regs.pc(),
        emu.cores[0].regs.sp(),
        emu.cores[0].regs.lr(),
    );

    // Single-step all instructions, but only log at function-call
    // transitions and exception entry. Exception entry is the real
    // signal we want — LR becoming 0xFFFFFFxx (EXC_RETURN) after a
    // step means the core vectored to an exception handler.
    //
    // Phase 5 tweak: only run the per-instruction probe for the first
    // ~100k instructions. That's enough to observe the post-direct-boot
    // call chain without burning minutes on the main loop. If firmware
    // hasn't faulted by then, the bulk-step outer loop below runs.
    emu.step_quantum = 1;
    let mut last_in_exception = false;
    // Ring-buffer of recent call transitions. Each entry captures
    // (step, from_pc, to_pc, lr, r0, r1, vtor).
    const RING_LEN: usize = 64;
    #[derive(Clone, Copy)]
    #[allow(dead_code)]
    struct CallRec {
        step: u64,
        from: u32,
        to: u32,
        lr: u32,
        r0: u32,
        r1: u32,
        vtor: u32,
    }
    let mut ring: Vec<CallRec> = Vec::with_capacity(RING_LEN);
    let mut ring_head: usize = 0;
    for s in 0..100_000u64 {
        let before_pc = emu.cores[0].regs.pc();
        let before_lr = emu.cores[0].regs.lr();
        let before_sp = emu.cores[0].regs.sp();
        let r = emu.cores[0].regs.r;
        let consumed = emu.step().expect("Serial step is infallible");
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
                let rec = CallRec {
                    step: s,
                    from: before_pc,
                    to: after_pc,
                    lr: after_lr,
                    r0,
                    r1,
                    vtor,
                };
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
                if i % 4 == 3 {
                    eprint!("\n                  ");
                }
            }
            eprintln!();
            // Dump instructions around pre-PC (the BKPT/panic site) and
            // panic wrapper. Each Thumb-16 halfword printed as u16.
            let dump_windows = [
                (
                    "crash site",
                    before_pc.saturating_sub(16),
                    before_pc.saturating_add(16),
                ),
                ("panic wrapper", 0x2000_0290, 0x2000_02a8),
                ("hard_assertion_failure", 0x2000_0db0, 0x2000_0de0),
                (
                    "caller @ r5/stack+36",
                    (r[5] & !1).saturating_sub(16),
                    (r[5] & !1).saturating_add(16),
                ),
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
                let CallRec {
                    step: s,
                    from,
                    to,
                    lr: _,
                    r0,
                    r1,
                    vtor,
                } = ring[idx];
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
                    eprintln!(
                        "    {:#010x}: {:04x}  {:#010x}: {:04x}",
                        addr,
                        low,
                        addr + 2,
                        high
                    );
                    addr += 4;
                }
            }
            break;
        }
        if left_exception {
            eprintln!("EXC RETURN #{s}: pc {before_pc:#010x} -> {after_pc:#010x}",);
        }
        if consumed == 0 {
            eprintln!("HALT @ {s}");
            break;
        }
    }
    emu.step_quantum = 64;

    let mut steps = 0u64;
    let mut last_sample_cycles = emu.cycles();
    let mut last_pc = 0u32;
    let mut same_pc_count = 0u64;
    while steps < total_steps {
        let consumed = emu.step().expect("Serial step is infallible");
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
                steps,
                cycles,
                dc,
                pc,
                emu.cores[0].regs.lr(),
                emu.cores[1].regs.pc(),
                emu.cores[1].is_halted(),
            );
            last_sample_cycles = cycles;
            if pc == last_pc {
                same_pc_count += 1;
                if same_pc_count >= 4 {
                    eprintln!(
                        "  PC has not moved across {} samples — likely tight loop",
                        same_pc_count
                    );
                }
            } else {
                same_pc_count = 0;
            }
            last_pc = pc;
        }
    }
    eprintln!(
        "final: cycles={} core0 pc={:#010x} lr={:#010x} sp={:#010x}",
        emu.cycles(),
        emu.cores[0].regs.pc(),
        emu.cores[0].regs.lr(),
        emu.cores[0].regs.sp(),
    );
    eprintln!(
        "final: core1 pc={:#010x} lr={:#010x} halted={}",
        emu.cores[1].regs.pc(),
        emu.cores[1].regs.lr(),
        emu.cores[1].is_halted(),
    );

    // Disassemble a window around core0 PC to identify the busy loop.
    let pc = emu.cores[0].regs.pc();
    let win_lo = pc.saturating_sub(48);
    let win_hi = pc.saturating_add(48);
    eprintln!("-- core0 PC window [{win_lo:#010x}..{win_hi:#010x}]");
    let mut addr = win_lo & !1;
    while addr < win_hi {
        let word = emu.bus.read32(addr & !3);
        let lo = word as u16;
        let hi = (word >> 16) as u16;
        let marker_lo = if addr == (pc & !1) { " <-- PC" } else { "" };
        let marker_hi = if (addr + 2) == (pc & !1) {
            " <-- PC"
        } else {
            ""
        };
        eprintln!(
            "    {:#010x}: {:04x}{}  {:#010x}: {:04x}{}",
            addr,
            lo,
            marker_lo,
            addr + 2,
            hi,
            marker_hi
        );
        addr += 4;
    }
    // Dump r0..r7 and stack to correlate with the loop variables.
    eprintln!(
        "    r0={:#010x} r1={:#010x} r2={:#010x} r3={:#010x} r4={:#010x} r5={:#010x} r6={:#010x} r7={:#010x}",
        emu.cores[0].regs.r[0],
        emu.cores[0].regs.r[1],
        emu.cores[0].regs.r[2],
        emu.cores[0].regs.r[3],
        emu.cores[0].regs.r[4],
        emu.cores[0].regs.r[5],
        emu.cores[0].regs.r[6],
        emu.cores[0].regs.r[7],
    );
    let sp = emu.cores[0].regs.sp();
    eprintln!("-- stack [sp-16..sp+48]");
    for i in 0..16u32 {
        let a = sp.wrapping_sub(16).wrapping_add(i * 4);
        let w = emu.bus.read32(a);
        eprintln!("    {a:#010x}: {w:#010x}");
    }

    // LR chain: if we're inside a non-leaf function, LR points at the
    // return address. Disassemble around LR too — the *caller* is
    // frequently the thing doing the waiting.
    let lr = emu.cores[0].regs.lr() & !1;
    let lrlo = lr.saturating_sub(32);
    let lrhi = lr.saturating_add(16);
    eprintln!("-- LR window [{lrlo:#010x}..{lrhi:#010x}] (caller's next insn)");
    let mut addr = lrlo & !1;
    while addr < lrhi {
        let word = emu.bus.read32(addr & !3);
        let lo = word as u16;
        let hi = (word >> 16) as u16;
        let marker_lo = if addr == lr { " <-- LR" } else { "" };
        let marker_hi = if (addr + 2) == lr { " <-- LR" } else { "" };
        eprintln!(
            "    {:#010x}: {:04x}{}  {:#010x}: {:04x}{}",
            addr,
            lo,
            marker_lo,
            addr + 2,
            hi,
            marker_hi
        );
        addr += 4;
    }

    // ----------------------------------------------------------------
    // Phase 5: peripheral state dump. Reads via `emu.bus.read32` so we
    // get the same view the SDK firmware would see through MMIO.
    // ----------------------------------------------------------------
    eprintln!();
    eprintln!("=== peripheral state dump ===");

    eprintln!("-- CLOCKS @ {CLOCKS_BASE:#010x}");
    // CLK_REF_CTRL / DIV / SELECTED, CLK_SYS_CTRL / DIV / SELECTED,
    // CLK_PERI_CTRL / SELECTED, CLK_USB_CTRL / DIV / SELECTED.
    let clk_names: &[(&str, u32)] = &[
        ("CLK_GPOUT0_CTRL", 0x00),
        ("CLK_GPOUT0_DIV", 0x04),
        ("CLK_REF_CTRL", 0x30),
        ("CLK_REF_DIV", 0x34),
        ("CLK_REF_SELECTED", 0x38),
        ("CLK_SYS_CTRL", 0x3c),
        ("CLK_SYS_DIV", 0x40),
        ("CLK_SYS_SELECTED", 0x44),
        ("CLK_PERI_CTRL", 0x48),
        ("CLK_PERI_SELECTED", 0x50),
        ("CLK_USB_CTRL", 0x54),
        ("CLK_USB_DIV", 0x58),
        ("CLK_USB_SELECTED", 0x5c),
        ("CLK_ADC_CTRL", 0x60),
        ("CLK_ADC_DIV", 0x64),
        ("CLK_ADC_SELECTED", 0x68),
        ("CLK_RTC_CTRL", 0x6c),
        ("CLK_RTC_DIV", 0x70),
        ("CLK_RTC_SELECTED", 0x74),
        ("CLK_SYS_RESUS_CTRL", 0x78),
        ("FC0_REF_KHZ", 0x80),
        ("FC0_SRC", 0x94),
    ];
    for (name, off) in clk_names {
        let v = emu.bus.read32(CLOCKS_BASE + off);
        eprintln!("    {off:#06x} {name:<22} = {v:#010x}");
    }
    eprintln!(
        "    emu.bus.clock_tree.sys_clk_hz = {} Hz",
        emu.bus.clock_tree.sys_clk_hz
    );
    eprintln!(
        "    emu.bus.clock_tree.ref_clk_hz = {} Hz",
        emu.bus.clock_tree.ref_clk_hz
    );

    eprintln!("-- PLL_SYS @ {PLL_SYS_BASE:#010x}");
    for (name, off) in &[("CS", 0u32), ("PWR", 4), ("FBDIV_INT", 8), ("PRIM", 12)] {
        let v = emu.bus.read32(PLL_SYS_BASE + off);
        eprintln!("    {off:#06x} {name:<12} = {v:#010x}");
    }
    eprintln!("-- PLL_USB @ {PLL_USB_BASE:#010x}");
    for (name, off) in &[("CS", 0u32), ("PWR", 4), ("FBDIV_INT", 8), ("PRIM", 12)] {
        let v = emu.bus.read32(PLL_USB_BASE + off);
        eprintln!("    {off:#06x} {name:<12} = {v:#010x}");
    }

    // PIO0 / PIO1. Registers of interest: CTRL (SM_ENABLE), FSTAT,
    // FDEBUG, SM0..SM3 { CLKDIV, EXECCTRL, SHIFTCTRL, ADDR, INSTR,
    // PINCTRL }, instruction memory INSTR_MEM[0..32].
    for (label, base) in [("PIO0", PIO0_BASE), ("PIO1", PIO1_BASE)] {
        eprintln!("-- {label} @ {base:#010x}");
        let ctrl = emu.bus.read32(base + 0x000);
        let fstat = emu.bus.read32(base + 0x004);
        let fdebug = emu.bus.read32(base + 0x008);
        let flevel = emu.bus.read32(base + 0x00c);
        eprintln!(
            "    CTRL   = {ctrl:#010x} (SM_ENABLE bits[3:0]={:04b})",
            ctrl & 0xf
        );
        eprintln!("    FSTAT  = {fstat:#010x}");
        eprintln!("    FDEBUG = {fdebug:#010x}");
        eprintln!("    FLEVEL = {flevel:#010x}");
        // Dump instruction memory as 32 halfwords.
        eprint!("    INSTR_MEM:");
        for i in 0u32..32 {
            let w = emu.bus.read32(base + 0x048 + i * 4);
            if i % 8 == 0 {
                eprint!("\n       ");
            }
            eprint!(" {:04x}", w & 0xffff);
        }
        eprintln!();
        // Per-SM registers (CLKDIV, EXECCTRL, SHIFTCTRL, ADDR, INSTR,
        // PINCTRL). These start at 0x0c8 for SM0, stride 0x018.
        for sm in 0u32..4 {
            let base_sm = base + 0x0c8 + sm * 0x018;
            let clkdiv = emu.bus.read32(base_sm + 0x00);
            let execctrl = emu.bus.read32(base_sm + 0x04);
            let shiftctrl = emu.bus.read32(base_sm + 0x08);
            let addr = emu.bus.read32(base_sm + 0x0c);
            let instr = emu.bus.read32(base_sm + 0x10);
            let pinctrl = emu.bus.read32(base_sm + 0x14);
            eprintln!(
                "    SM{sm}: clkdiv={clkdiv:#010x} execctrl={execctrl:#010x} \
                 shiftctrl={shiftctrl:#010x} addr={addr:#010x} instr={instr:04x} \
                 pinctrl={pinctrl:#010x} enabled={}",
                (ctrl >> sm) & 1,
            );
        }
    }

    // DMA — not modelled as a real peripheral; writes land in the
    // Bus's generic `peripheral_regs` HashMap, which `read32` surfaces
    // as a pass-through. If the firmware programmed DMA channels 0/1
    // (expected for I2S), we'll see non-zero READ/WRITE/TRANS/CTRL
    // fields. If everything is zero, the HashMap never saw any DMA
    // traffic either.
    eprintln!("-- DMA @ 0x50000000 (peripheral_regs pass-through; no simulation)");
    for ch in 0u32..2 {
        let base_ch = 0x5000_0000u32 + ch * 0x40;
        let read_addr = emu.bus.read32(base_ch + 0x00);
        let write_addr = emu.bus.read32(base_ch + 0x04);
        let trans_count = emu.bus.read32(base_ch + 0x08);
        let ctrl_trig = emu.bus.read32(base_ch + 0x0c);
        let al1_ctrl = emu.bus.read32(base_ch + 0x10);
        eprintln!(
            "    CH{ch}: READ={read_addr:#010x} WRITE={write_addr:#010x} \
             TRANS={trans_count:#010x} CTRL_TRIG={ctrl_trig:#010x} AL1_CTRL={al1_ctrl:#010x}"
        );
    }
    // DMA INTR0 / INTS0 / INTE0 (0x400 / 0x40c / 0x404).
    let dma_intr = emu.bus.read32(0x5000_0400);
    let dma_ints = emu.bus.read32(0x5000_040c);
    let dma_inte = emu.bus.read32(0x5000_0404);
    eprintln!("    INTR={dma_intr:#010x} INTE={dma_inte:#010x} INTS={dma_ints:#010x}");

    // GPIO pad state for the I2S pins — if anything is driving them,
    // we should see non-default `bus.gpio_in` bits.
    let gpio_in = emu.bus.gpio_in;
    eprintln!("-- GPIO");
    eprintln!(
        "    bus.gpio_in = {gpio_in:#010x}, I2S_DOUT(16)={}, I2S_BCLK(17)={}, I2S_LRCLK(18)={}",
        (gpio_in >> 16) & 1,
        (gpio_in >> 17) & 1,
        (gpio_in >> 18) & 1,
    );
    // SIO gpio_out / gpio_oe.
    eprintln!(
        "    sio.gpio_out = {:#010x}, sio.gpio_oe = {:#010x}",
        emu.bus.sio.gpio_out, emu.bus.sio.gpio_oe,
    );
    // PIO pad_out / pad_oe — who is driving the I2S pins?
    for (label, i) in [("PIO0", 0usize), ("PIO1", 1)] {
        let pout = emu.bus.pio[i].pad_out;
        let poe = emu.bus.pio[i].pad_oe;
        eprintln!("    {label}: pad_out={pout:#010x} pad_oe={poe:#010x}");
    }
}
