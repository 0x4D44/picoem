/// Phase 0a workload study: synthetic dual-core RP2350 workload with
/// per-core access counter metrics.
///
/// Core 0: GPIO toggle + shared SRAM write + FIFO push (tight loop)
/// Core 1: shared SRAM read + FIFO read (tight loop)
///
/// Reports `CoreCounters` breakdown and decision-gate thresholds.
use rp2350_emu::{Config, Emulator};
use std::time::Instant;

const CORE0_BASE: u32 = 0x2000_0000;
const CORE1_BASE: u32 = 0x2000_1000;
const SHARED_SRAM: u32 = 0x2004_0000;
const RUN_QUANTA: u64 = 1_000_000;

fn main() {
    println!("=== Workload Study (Phase 0a) ===\n");

    let mut emu = Emulator::new(Config::default());

    // --- Core 0 program: GPIO toggle + shared SRAM write + FIFO push ---
    //
    // Pre-loaded registers:
    //   r0 = 0xD000_0014 (SIO_GPIO_OUT_XOR)
    //   r1 = 1
    //   r2 = 0 (counter)
    //   r3 = 0x2004_0000 (shared SRAM midpoint)
    //   r4 = 0xD000_0050 (SIO_FIFO_WR)
    //
    // Instructions:
    //   0x2000_0000: STR r1, [r0, #0]   ; GPIO XOR toggle  (0x6001)
    //   0x2000_0002: STR r2, [r3, #0]   ; shared SRAM write (0x601A)
    //   0x2000_0004: STR r2, [r4, #0]   ; FIFO push         (0x6022)
    //   0x2000_0006: ADDS r2, r2, #1    ; increment counter  (0x3201)
    //   0x2000_0008: B .-8              ; loop back           (0xE7FA)
    emu.poke(CORE0_BASE, 0x601A_6001); // STR r1,[r0] | STR r2,[r3]
    emu.poke(CORE0_BASE + 4, 0x3201_6022); // STR r2,[r4] | ADDS r2,#1
    emu.poke(CORE0_BASE + 8, 0x0000_E7FA); // B .-8       | (pad)

    emu.core_mut(0).set_reg(0, 0xD000_0014); // SIO_GPIO_OUT_XOR
    emu.core_mut(0).set_reg(1, 1);
    emu.core_mut(0).set_reg(2, 0);
    emu.core_mut(0).set_reg(3, SHARED_SRAM);
    emu.core_mut(0).set_reg(4, 0xD000_0050); // SIO_FIFO_WR
    emu.core_mut(0).regs.msp = 0x2008_0000;
    emu.core_mut(0).regs.r[13] = 0x2008_0000;
    emu.core_mut(0).regs.set_pc(CORE0_BASE);
    emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb mode

    // --- Core 1 program: shared SRAM read + FIFO read ---
    //
    // Pre-loaded registers:
    //   r3 = 0x2004_0000 (shared SRAM — same address core 0 writes)
    //   r6 = 0xD000_0058 (SIO_FIFO_RD)
    //   r7 = 0 (counter)
    //
    // Instructions:
    //   0x2000_1000: LDR r0, [r3, #0]   ; shared SRAM read   (0x6818)
    //   0x2000_1002: LDR r5, [r6, #0]   ; FIFO read          (0x6835)
    //   0x2000_1004: ADDS r7, r7, #1    ; increment counter   (0x3701)
    //   0x2000_1006: B .-6              ; loop back            (0xE7FB)
    emu.poke(CORE1_BASE, 0x6835_6818); // LDR r0,[r3] | LDR r5,[r6]
    emu.poke(CORE1_BASE + 4, 0xE7FB_3701); // ADDS r7,#1  | B .-6

    emu.core_mut(1).set_reg(3, SHARED_SRAM);
    emu.core_mut(1).set_reg(6, 0xD000_0058); // SIO_FIFO_RD
    emu.core_mut(1).set_reg(7, 0);
    emu.core_mut(1).regs.msp = 0x2007_0000;
    emu.core_mut(1).regs.r[13] = 0x2007_0000;
    emu.core_mut(1).regs.set_pc(CORE1_BASE);
    emu.core_mut(1).regs.xpsr = 1 << 24; // Thumb mode

    // Reset counters before the run
    emu.reset_counters();

    // Run
    let start = Instant::now();
    for _ in 0..RUN_QUANTA {
        emu.step().unwrap();
    }
    let elapsed = start.elapsed();

    // Report
    println!(
        "Run: {} quanta in {:.2}s\n",
        RUN_QUANTA,
        elapsed.as_secs_f64()
    );

    for core_id in 0..2 {
        let c = emu.core_counters(core_id);
        let total_accesses =
            c.sram_reads + c.sram_writes + c.sio_accesses + c.peripheral_accesses + c.ppb_accesses;

        println!("--- Core {} ---", core_id);
        println!("  Decode/execute cycles:  {}", c.decode_execute_cycles);
        println!("  WFI cycles:             {}", c.wfi_cycles);
        println!("  WFE cycles:             {}", c.wfe_cycles);
        println!(
            "  SRAM reads:             {} ({:.1}%)",
            c.sram_reads,
            pct(c.sram_reads, total_accesses)
        );
        println!(
            "  SRAM writes:            {} ({:.1}%)",
            c.sram_writes,
            pct(c.sram_writes, total_accesses)
        );
        println!(
            "  SIO accesses:           {} ({:.1}%)",
            c.sio_accesses,
            pct(c.sio_accesses, total_accesses)
        );
        println!(
            "  Peripheral accesses:    {} ({:.1}%)",
            c.peripheral_accesses,
            pct(c.peripheral_accesses, total_accesses)
        );
        println!(
            "  PPB accesses:           {} ({:.1}%)",
            c.ppb_accesses,
            pct(c.ppb_accesses, total_accesses)
        );
        println!();
    }

    // Decision gate
    let c0 = emu.core_counters(0);
    let c1 = emu.core_counters(1);
    let c0_active = c0.decode_execute_cycles;
    let c1_active = c1.decode_execute_cycles;
    let total_active = c0_active + c1_active;

    let c1_pct = if total_active > 0 {
        c1_active as f64 / total_active as f64 * 100.0
    } else {
        0.0
    };
    let cross_core_writes_pct = if c0.sram_writes + c1.sram_writes > 0 {
        // Cross-core SRAM writes: core 0 writes to shared region that core 1 reads.
        // In this synthetic workload, ALL core 0 SRAM writes are cross-core by design.
        c0.sram_writes as f64
            / (c0.sram_writes + c0.sram_reads + c1.sram_writes + c1.sram_reads) as f64
            * 100.0
    } else {
        0.0
    };

    println!("=== Decision Gate ===");
    println!(
        "Core 1 active: {:.1}% {} (threshold: >=20%)",
        c1_pct,
        if c1_pct >= 20.0 { "[PASS]" } else { "[FAIL]" }
    );
    println!(
        "Cross-core SRAM writes: {:.1}% {} (threshold: <10%)",
        cross_core_writes_pct,
        if cross_core_writes_pct < 10.0 {
            "[PASS]"
        } else {
            "[WARN]"
        }
    );
}

fn pct(n: u64, total: u64) -> f64 {
    if total == 0 {
        0.0
    } else {
        n as f64 / total as f64 * 100.0
    }
}
