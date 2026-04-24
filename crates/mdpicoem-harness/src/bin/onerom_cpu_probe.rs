//! Disposable diagnostic: boot the CPU-mode OneROM fixture and trace the
//! CPU serve loop PC range, so the CPU oracle's `is_synced_cpu` sync
//! detector can pin the exact PC range. Delete this binary once the
//! oracle is landed.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin onerom_cpu_probe --release

use std::collections::BTreeMap;
use std::process::ExitCode;
use std::sync::atomic::Ordering;

use mdrp2350::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0-cpu.bin";

/// Generous boot budget — CPU mode may take longer to settle than PIO
/// because there's no PIO handoff, the CPU must iterate to the serve
/// loop itself.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// After reaching steady state, profile PC for this many instructions.
const PROFILE_INSTRUCTIONS: u64 = 5_000;

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    let bootrom = match std::fs::read(BOOTROM_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", BOOTROM_PATH, e);
            return ExitCode::from(2);
        }
    };
    let flash = match std::fs::read(FLASH_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read flash at {}: {}", FLASH_PATH, e);
            return ExitCode::from(2);
        }
    };

    println!(
        "loaded bootrom ({} bytes) and flash ({} bytes)",
        bootrom.len(),
        flash.len()
    );

    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().unwrap();
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Bootrom bypass.
    let sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let pc = pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(sp);
    emu.core_mut(0).regs.set_pc(pc);
    emu.core_mut(1).halt();

    println!("bypass: SP=0x{:08X} PC=0x{:08X}", sp, pc);

    // Drive CS1 low + CS2/CS3 high + addr pins 1 (baseline stim) so the
    // CPU's serve-loop doesn't stall on "no CS asserted". We reuse the
    // pin-map from the PIO oracle. This matches the baseline stim the
    // CPU oracle will drive before sync.
    const GPIO_CS1: u8 = 13;
    const GPIO_CS2: u8 = 12;
    const GPIO_CS3: u8 = 15;
    const ADDR_PINS: [u8; 13] = [7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 14, 15, 12];

    let ext_mask: u32 = (1u32 << GPIO_CS1)
        | (1u32 << GPIO_CS2)
        | (1u32 << GPIO_CS3)
        | ADDR_PINS.iter().fold(0u32, |a, &p| a | (1u32 << p));
    // Baseline stimulus: CS1 low, CS2/CS3 high (A11=A12=1), all low bits 0.
    let stim: u32 = (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
    emu.bus.gpio_external_mask = ext_mask;
    emu.bus.gpio_external_in.store(stim, Ordering::Relaxed);

    // Run until PIO1/PIO2 CTRL stay 0 AND PC is circulating in a narrow
    // range for many instructions. Track OEN transitions to surface
    // where/when the CPU sets the data-pin OEN.
    let warm_up_cap: u64 = BOOT_CYCLE_CAP / 2;
    let mut last_oe_data: u8 = 0;
    let mut oe_change_count = 0u32;
    while emu.cycles() < warm_up_cap {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        if emu.cycles() == before {
            eprintln!("cycle stalled at {}", before);
            return ExitCode::FAILURE;
        }
        let oe = emu.bus.read32(0xD000_0020, 0);
        let oe_data = ((oe >> 16) & 0xFF) as u8;
        if oe_data != last_oe_data && oe_change_count < 20 {
            let pc = emu.core(0).regs.pc();
            println!(
                "OEN change at cycle {} PC=0x{:08X}  oe[16..23]: 0x{:02X} -> 0x{:02X}  full_oe=0x{:08X}",
                emu.cycles(),
                pc,
                last_oe_data,
                oe_data,
                oe,
            );
            oe_change_count += 1;
            last_oe_data = oe_data;
        }
    }

    let pio1_ctrl = emu.bus.read32(0x5030_0000, 0);
    let pio2_ctrl = emu.bus.read32(0x5040_0000, 0);
    println!(
        "after warm-up at cycle {}: PIO1.CTRL=0x{:08X} PIO2.CTRL=0x{:08X}",
        emu.cycles(),
        pio1_ctrl,
        pio2_ctrl
    );
    println!("core 0 PC = 0x{:08X}", emu.core(0).regs.pc());

    // PC histogram.
    let mut hist: BTreeMap<u32, u64> = BTreeMap::new();
    for _ in 0..PROFILE_INSTRUCTIONS {
        let p = emu.core(0).regs.pc();
        *hist.entry(p).or_insert(0) += 1;
        emu.run(1).expect("Serial run is infallible");
    }

    // Disassemble a wider range around the serve loop. Print every halfword
    // with a quick mnemonic guess so we can see the full flow around 0x926.
    println!();
    println!("Dump around serve loop (0x10000900..=0x10000960):");
    for addr in (0x10000900u32..=0x10000960u32).step_by(2) {
        let hw = emu.bus.read8(addr, 0) as u32 | ((emu.bus.read8(addr + 1, 0) as u32) << 8);
        // Minimal classifier — just show the encoding family.
        let mnemonic = match hw & 0xF800 {
            0x6000 => format!("STR R{}, [R{}, #{}]", hw & 7, (hw >> 3) & 7, ((hw >> 6) & 0x1F) * 4),
            0x6800 => format!("LDR R{}, [R{}, #{}]", hw & 7, (hw >> 3) & 7, ((hw >> 6) & 0x1F) * 4),
            0x7000 => format!("STRB R{}, [R{}, #{}]", hw & 7, (hw >> 3) & 7, (hw >> 6) & 0x1F),
            0x7800 => format!("LDRB R{}, [R{}, #{}]", hw & 7, (hw >> 3) & 7, (hw >> 6) & 0x1F),
            0x8000 => format!("STRH R{}, [R{}, #{}]", hw & 7, (hw >> 3) & 7, ((hw >> 6) & 0x1F) * 2),
            0x8800 => format!("LDRH R{}, [R{}, #{}]", hw & 7, (hw >> 3) & 7, ((hw >> 6) & 0x1F) * 2),
            _ => format!("0x{:04X}", hw),
        };
        if hw != 0 {
            println!("  0x{:08X}: 0x{:04X}   // {}", addr, hw, mnemonic);
        }
    }

    // Check SRAM population at the shadow base.
    let mut nonzero = 0usize;
    let mut first_values = [0u8; 16];
    for i in 0..0x1_0000u32 {
        let b = emu.bus.read8(0x2000_0000 + i, 0);
        if b != 0 {
            nonzero += 1;
        }
        if (i as usize) < 16 {
            first_values[i as usize] = b;
        }
    }
    println!();
    println!("SRAM shadow scan: {} non-zero bytes out of 65536", nonzero);
    println!("  first 16 bytes at 0x20000000: {:02X?}", first_values);
    // Spot-check key shadow offsets that the oracle will look up:
    println!("  shadow[0x9000] = 0x{:02X}  (walk1 baseline pin pattern)", emu.bus.read8(0x2000_9000, 0));
    println!("  shadow[0x9080] = 0x{:02X}  (walk1 A0 pin pattern)", emu.bus.read8(0x2000_9080, 0));
    println!("  shadow[0x9040] = 0x{:02X}  (walk1 A1 pin pattern)", emu.bus.read8(0x2000_9040, 0));
    println!("  shadow[0x9020] = 0x{:02X}  (walk1 A2 pin pattern)", emu.bus.read8(0x2000_9020, 0));
    println!("  shadow[0x9010] = 0x{:02X}  (walk1 A3 pin pattern)", emu.bus.read8(0x2000_9010, 0));

    // Inspect the serve loop bytes.
    println!();
    println!("Serve loop disassembly (instruction words at each hot PC):");
    for pc in [0x10000926, 0x10000928, 0x1000092A, 0x1000092C, 0x1000092E, 0x10000930u32] {
        let w = emu.bus.read32(pc, 0);
        println!("  PC=0x{:08X}: raw 0x{:08X}  [lo=0x{:04X} hi=0x{:04X}]", pc, w, w & 0xFFFF, (w >> 16) & 0xFFFF);
    }

    // Inspect registers in the serve loop.
    println!();
    println!("Core 0 regs (in serve loop):");
    let regs = &emu.core(0).regs;
    for i in 0..16 {
        println!("  R{:2} = 0x{:08X}", i, regs.r[i]);
    }

    // Try applying CS1-low stimulus and see if the serve loop reacts.
    println!();
    println!("Applying CS1-low stim (0x1800 baseline)...");
    let stim_level_cs1_low: u32 = (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3); // CS1 at bit 13 clear
    emu.bus.gpio_external_in.store(stim_level_cs1_low, Ordering::Relaxed);
    for n in 0..500u64 {
        emu.run(1).expect("Serial run is infallible");
        let oe = emu.bus.read32(0xD000_0020, 0);
        let out = emu.bus.read32(0xD000_0010, 0);
        if (oe >> 16) & 0xFF != 0 || (out >> 16) & 0xFF != 0 {
            println!(
                "  tick {}: PC=0x{:08X}  SIO_OE[16..23]=0x{:02X}  SIO_OUT[16..23]=0x{:02X}  gpio_in=0x{:08X}",
                n,
                emu.core(0).regs.pc(),
                (oe >> 16) & 0xFF,
                (out >> 16) & 0xFF,
                emu.bus.gpio_in.load(Ordering::Relaxed),
            );
            break;
        }
    }

    // === write8 regression check ===================================
    // Hypothesis: emulator's write8 has no arm for region 0xD (SIO).
    // Sanity test: write8 a byte to SIO_GPIO_OUT and check it landed.
    println!();
    println!("write8 regression check (writes 0xA5 to SIO_GPIO_OUT via write8):");
    let before = emu.bus.read32(0xD000_0010, 0);
    emu.bus.write8(0xD000_0010, 0xA5, 0);
    let after_byte = emu.bus.read32(0xD000_0010, 0);
    println!("  before write8: SIO_GPIO_OUT = 0x{:08X}", before);
    println!("  after  write8: SIO_GPIO_OUT = 0x{:08X}  (expected 0x{:08X} if byte 0 sticks)",
             after_byte, (before & !0xFFu32) | 0xA5);
    emu.bus.write32(0xD000_0010, before, 0); // restore

    // Now compare to word write:
    emu.bus.write32(0xD000_0010, 0x0000_00A5, 0);
    let after_word = emu.bus.read32(0xD000_0010, 0);
    println!("  after  write32 0xA5: SIO_GPIO_OUT = 0x{:08X}", after_word);
    emu.bus.write32(0xD000_0010, before, 0);

    // Run FIRST with pure CS1-low stim (no gap drives) — see if OEN
    // gets set once and held.
    println!();
    println!("Steady-state CS1-low stim for 200 ticks (tracking OEN + PC + OUT):");
    let cs1_low_stim: u32 = (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
    emu.bus.gpio_external_in.store(cs1_low_stim, Ordering::Relaxed);
    for t in 0..200u32 {
        emu.run(1).expect("Serial run is infallible");
        let oe = emu.bus.read32(0xD000_0030, 0);
        let out = emu.bus.read32(0xD000_0010, 0);
        let pc = emu.core(0).regs.pc();
        // Print every tick for the first 40, then every 10 afterwards.
        if t < 40 || (t % 10) == 0 {
            let r0 = emu.core(0).regs.r[0];
            let r1 = emu.core(0).regs.r[1];
            println!(
                "  t={:>3} PC=0x{:08X} OE=0x{:08X} OUT=0x{:08X}  R0=0x{:08X} R1=0x{:02X}  gpio_in=0x{:08X}",
                t,
                pc,
                oe,
                out,
                r0,
                r1 & 0xFF,
                emu.bus.gpio_in.load(Ordering::Relaxed),
            );
        }
    }

    // Sweep stim: apply different addr bits and print the observed byte.
    println!();
    println!("Address sweep (applying stim + waiting, observing data pins):");
    for &addr_bits in &[0x1801u16, 0x1802, 0x1804, 0x1808, 0x1810, 0x1820, 0x1AAA, 0x1D55, 0x1FFF] {
        // Compose stim.
        let mut stim: u32 = 0;
        for (i, &pin) in ADDR_PINS.iter().enumerate() {
            if (addr_bits >> i) & 1 != 0 {
                stim |= 1u32 << pin;
            }
        }
        // CS1 stays low (bit 13 clear); CS2/CS3 high are already set by A11/A12 in addr.
        emu.bus.gpio_external_in.store(stim, Ordering::Relaxed);
        // Tick until output stabilises (or give up after 200 cycles).
        let mut last_byte = 0u8;
        let mut stable_ticks = 0u32;
        let mut decided_at: Option<u32> = None;
        let mut first_r0_change_at: Option<(u32, u32)> = None;
        let initial_r0 = emu.core(0).regs.r[0];
        for c in 0..200u32 {
            emu.run(1).expect("Serial run is infallible");
            let oe = emu.bus.read32(0xD000_0030, 0); // CORRECTED — was reading GPIO_OUT_CLR
            let out = emu.bus.read32(0xD000_0010, 0);
            let oe_data = ((oe >> 16) & 0xFF) as u8;
            let out_data = ((out >> 16) & 0xFF) as u8;
            let r0 = emu.core(0).regs.r[0];
            if first_r0_change_at.is_none() && r0 != initial_r0 {
                first_r0_change_at = Some((c, r0));
            }
            if oe_data == 0xFF && out_data == last_byte {
                stable_ticks += 1;
                if stable_ticks >= 4 {
                    decided_at = Some(c + 1 - 4);
                    break;
                }
            } else {
                last_byte = out_data;
                stable_ticks = 0;
            }
        }
        let oe_final = emu.bus.read32(0xD000_0030, 0);
        let out_final = emu.bus.read32(0xD000_0010, 0);
        let gpio_in_final = emu.bus.gpio_in.load(Ordering::Relaxed);
        let r0_final = emu.core(0).regs.r[0];
        let r1_final = emu.core(0).regs.r[1];
        println!(
            "  addr=0x{:04X}  stim=0x{:08X}  byte_out=0x{:02X} oe=0x{:02X}  gpio_in=0x{:08X}  R0=0x{:08X} R1=0x{:02X}  r0_first_change={:?}  latency={:?}",
            addr_bits,
            stim,
            (out_final >> 16) & 0xFF,
            (oe_final >> 16) & 0xFF,
            gpio_in_final,
            r0_final,
            r1_final & 0xFF,
            first_r0_change_at,
            decided_at
        );
        // De-assert CS1 between cases.
        let cs_high = (1u32 << GPIO_CS1) | (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
        emu.bus.gpio_external_in.store(cs_high, Ordering::Relaxed);
        for _ in 0..12 {
            emu.run(1).expect("Serial run is infallible");
        }
    }

    println!();
    println!("PC histogram (top 20, over {} instructions):", PROFILE_INSTRUCTIONS);
    let mut entries: Vec<_> = hist.iter().map(|(k, v)| (*k, *v)).collect();
    entries.sort_by(|a, b| b.1.cmp(&a.1));
    for (pc, n) in entries.iter().take(20) {
        let pct = 100.0 * (*n as f64) / (PROFILE_INSTRUCTIONS as f64);
        println!("  PC=0x{:08X}  count={:>5}  ({:>5.2}%)", pc, n, pct);
    }

    // Contiguous block detection: find the longest run of PCs whose counts
    // are non-trivial (> PROFILE_INSTRUCTIONS/500) and report min/max.
    let min_count = (PROFILE_INSTRUCTIONS / 500).max(1);
    let mut hot_pcs: Vec<u32> = hist
        .iter()
        .filter(|(_, c)| **c >= min_count)
        .map(|(p, _)| *p)
        .collect();
    hot_pcs.sort();
    if let (Some(&lo), Some(&hi)) = (hot_pcs.first(), hot_pcs.last()) {
        println!();
        println!("Hot-PC range (count >= {}): 0x{:08X}..=0x{:08X}", min_count, lo, hi);
        let span = hi.wrapping_sub(lo);
        println!("  span: {} bytes", span);
    }

    // Final snapshot: GPIO_IN/OUT/OEN
    let gpio_in = emu.bus.gpio_in.load(Ordering::Relaxed);
    let gpio_out = emu.bus.read32(0xD000_0010, 0);
    let gpio_oe = emu.bus.read32(0xD000_0020, 0);
    println!();
    println!(
        "Final GPIO snapshot: gpio_in=0x{:08X} SIO_OUT=0x{:08X} SIO_OE=0x{:08X}",
        gpio_in, gpio_out, gpio_oe
    );
    println!("  data bits (pins 16..23): 0x{:02X}", (gpio_in >> 16) & 0xFF);
    println!("  data OEN (pins 16..23): 0x{:02X}", (gpio_oe >> 16) & 0xFF);

    ExitCode::SUCCESS
}
