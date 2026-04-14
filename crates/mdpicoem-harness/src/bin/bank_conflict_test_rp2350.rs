// SRAM bank-conflict test for RP2354 (OneROM) via SWD single-step.
//
// Tests the hypothesis: when instruction fetch and data access hit the SAME
// SRAM bank, the Cortex-M33 bus multiplexer stalls +1 cycle.
//
// SRAM bank formula: bank = (byte_address >> 2) & 7 (bits [4:2]).
// TEST_SLOT = 0x20000100 → bank = (0x100 >> 2) & 7 = 0.
//
// Run: cargo run -p mdpicoem-harness --bin bank_conflict_test_rp2350

use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// SRAM slot for injected instructions — always bank 0.
const TEST_SLOT: u64 = 0x2000_0100;

// ARM Cortex-M register IDs.
const R0: RegisterId = RegisterId(0);
const R1: RegisterId = RegisterId(1);
const PC: RegisterId = RegisterId(15);

// DWT / CoreDebug MMIO addresses.
const DEMCR: u64 = 0xE000_EDFC;
const DWT_CTRL: u64 = 0xE000_1000;
const DWT_CYCCNT: u64 = 0xE000_1004;

// DEMCR / DWT bits.
const TRCENA: u32 = 1 << 24;
const CYCCNTENA: u32 = 1 << 0;

// Thumb-2 encoded instructions (16-bit, little-endian).
const LDR_R0_R1: u32 = 0x6808; // LDR R0, [R1, #0]
const STR_R0_R1: u32 = 0x6008; // STR R0, [R1, #0]

// Data addresses for each SRAM bank (fetch is always from bank 0 at TEST_SLOT).
const DATA_BANK0: u64 = 0x2000_0200; // bank = (0x200 >> 2) & 7 = 0
const DATA_BANK1: u64 = 0x2000_0204; // bank = (0x204 >> 2) & 7 = 1
const DATA_BANK2: u64 = 0x2000_0208; // bank = (0x208 >> 2) & 7 = 2
const DATA_BANK3: u64 = 0x2000_020C; // bank = (0x20C >> 2) & 7 = 3
const DATA_BANK4: u64 = 0x2000_0210; // bank = (0x210 >> 2) & 7 = 4

const NUM_SAMPLES: usize = 20;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Write a 16-bit Thumb instruction at `addr` in SRAM.
fn write_thumb(core: &mut Core, addr: u64, hw: u32) -> Result<(), probe_rs::Error> {
    let bytes = (hw as u16).to_le_bytes();
    core.write_8(addr, &bytes)?;
    Ok(())
}

/// Enable DWT CYCCNT.
fn enable_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    let demcr: u32 = core.read_word_32(DEMCR)?;
    core.write_word_32(DEMCR, demcr | TRCENA)?;
    let ctrl: u32 = core.read_word_32(DWT_CTRL)?;
    core.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
    Ok(())
}

fn reset_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    core.write_word_32(DWT_CYCCNT, 0)?;
    Ok(())
}

fn read_cyccnt(core: &mut Core) -> Result<u32, probe_rs::Error> {
    core.read_word_32(DWT_CYCCNT)
}

/// Measure cycle count for a single instruction at TEST_SLOT that accesses
/// memory at `data_addr`. Sets up R1 = data_addr, writes test data, then
/// single-steps and reads CYCCNT.
fn measure_mem_insn(
    core: &mut Core,
    insn: u32,
    data_addr: u64,
) -> Result<u32, probe_rs::Error> {
    // Write instruction at TEST_SLOT.
    write_thumb(core, TEST_SLOT, insn)?;

    // Point R1 at the data address and ensure valid data is there.
    core.write_core_reg(R1, data_addr)?;
    core.write_word_32(data_addr, 0xCAFE_BABE)?;

    // For STR, put something in R0 to store.
    core.write_core_reg(R0, 0x1234_5678u64)?;

    // Set PC, reset counter, step, read counter.
    core.write_core_reg(PC, TEST_SLOT)?;
    reset_cyccnt(core)?;
    core.step()?;
    read_cyccnt(core)
}

/// Collect NUM_SAMPLES measurements.
fn collect_samples(
    core: &mut Core,
    insn: u32,
    data_addr: u64,
) -> Result<Vec<u32>, probe_rs::Error> {
    let mut samples = Vec::with_capacity(NUM_SAMPLES);
    for _ in 0..NUM_SAMPLES {
        samples.push(measure_mem_insn(core, insn, data_addr)?);
    }
    Ok(samples)
}

fn median(samples: &[u32]) -> u32 {
    let mut sorted = samples.to_vec();
    sorted.sort();
    sorted[sorted.len() / 2]
}

fn bank_of(addr: u64) -> u32 {
    ((addr >> 2) & 7) as u32
}

fn print_result(label: &str, samples: &[u32]) {
    let med = median(samples);
    let min = *samples.iter().min().unwrap();
    let max = *samples.iter().max().unwrap();
    println!("{label}");
    println!("  samples: {samples:?}");
    println!("  median={med}  min={min}  max={max}");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("bank_conflict_test_rp2350: SRAM bank-conflict hypothesis checker");
    println!("=================================================================");
    println!("TEST_SLOT = 0x{TEST_SLOT:08X}  (bank {})", bank_of(TEST_SLOT));
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;
    core.reset_and_halt(std::time::Duration::from_millis(500))?;
    enable_cyccnt(&mut core)?;

    let t0 = Instant::now();

    // -- Test A: LDR same bank (fetch bank 0, data bank 0) --
    let a = collect_samples(&mut core, LDR_R0_R1, DATA_BANK0)?;
    print_result(
        &format!(
            "Test A  LDR same-bank  fetch=bank{}  data=0x{:08X} bank{}",
            bank_of(TEST_SLOT),
            DATA_BANK0,
            bank_of(DATA_BANK0),
        ),
        &a,
    );

    // -- Test B: LDR different bank (fetch bank 0, data bank 1) --
    let b = collect_samples(&mut core, LDR_R0_R1, DATA_BANK1)?;
    print_result(
        &format!(
            "Test B  LDR diff-bank  fetch=bank{}  data=0x{:08X} bank{}",
            bank_of(TEST_SLOT),
            DATA_BANK1,
            bank_of(DATA_BANK1),
        ),
        &b,
    );

    // -- Test D: LDR more bank combinations --
    let d_addrs = [
        (DATA_BANK2, "D1"),
        (DATA_BANK3, "D2"),
        (DATA_BANK4, "D3"),
    ];
    let mut d_results = Vec::new();
    for (addr, tag) in &d_addrs {
        let samples = collect_samples(&mut core, LDR_R0_R1, *addr)?;
        print_result(
            &format!(
                "Test {tag}  LDR  fetch=bank{}  data=0x{addr:08X} bank{}",
                bank_of(TEST_SLOT),
                bank_of(*addr),
            ),
            &samples,
        );
        d_results.push((*addr, samples));
    }

    // -- Test E: STR same-bank vs different-bank --
    let e_same = collect_samples(&mut core, STR_R0_R1, DATA_BANK0)?;
    print_result(
        &format!(
            "Test E1 STR same-bank  fetch=bank{}  data=0x{:08X} bank{}",
            bank_of(TEST_SLOT),
            DATA_BANK0,
            bank_of(DATA_BANK0),
        ),
        &e_same,
    );

    let e_diff = collect_samples(&mut core, STR_R0_R1, DATA_BANK1)?;
    print_result(
        &format!(
            "Test E2 STR diff-bank  fetch=bank{}  data=0x{:08X} bank{}",
            bank_of(TEST_SLOT),
            DATA_BANK1,
            bank_of(DATA_BANK1),
        ),
        &e_diff,
    );

    // -- Test F: Sweep all 8 banks with LDR to see which ones are +1 --
    println!();
    println!("Test F: LDR sweep across all 8 banks (data addresses 0x200..0x21C)");
    let sweep_base: u64 = 0x2000_0200;
    let mut sweep_medians = Vec::new();
    for bank in 0..8u64 {
        let addr = sweep_base + bank * 4;
        let samples = collect_samples(&mut core, LDR_R0_R1, addr)?;
        let med = median(&samples);
        sweep_medians.push((bank, addr, med));
        println!(
            "  bank {bank}: data=0x{addr:08X}  median={med}  samples={samples:?}"
        );
    }

    // -- Test G: Confirm bank 2 at multiple different base addresses --
    println!();
    println!("Test G: LDR bank 2 at different base addresses");
    let bank2_addrs: Vec<u64> = vec![
        0x2000_0208, // bank 2
        0x2000_0228, // bank 2 (+ 0x20)
        0x2000_0248, // bank 2 (+ 0x40)
        0x2000_0308, // bank 2 (different 256-byte region)
        0x2000_0408, // bank 2 (yet another region)
    ];
    for addr in &bank2_addrs {
        let bk = bank_of(*addr);
        let samples = collect_samples(&mut core, LDR_R0_R1, *addr)?;
        let med = median(&samples);
        println!(
            "  data=0x{addr:08X} bank={bk}  median={med}  samples={samples:?}"
        );
    }

    // -- Test H: Also try addresses NOT bank 2 but nearby --
    println!();
    println!("Test H: LDR non-bank-2 addresses near 0x208");
    let near_addrs: Vec<u64> = vec![
        0x2000_0200, // bank 0
        0x2000_0204, // bank 1
        0x2000_0208, // bank 2
        0x2000_020C, // bank 3
        0x2000_0210, // bank 4
        0x2000_0214, // bank 5
        0x2000_0218, // bank 6
        0x2000_021C, // bank 7
    ];
    for addr in &near_addrs {
        let bk = bank_of(*addr);
        let samples = collect_samples(&mut core, LDR_R0_R1, *addr)?;
        let med = median(&samples);
        println!(
            "  data=0x{addr:08X} bank={bk}  median={med}  samples={:?}",
            &samples[..5],
        );
    }

    // -- Test I: Try different fetch addresses (move TEST_SLOT) --
    println!();
    println!("Test I: Move instruction to different banks, data at 0x20000300");
    let data_fixed: u64 = 0x2000_0300; // bank = (0x300 >> 2) & 7 = 0
    core.write_word_32(data_fixed, 0xDEAD_BEEF)?;
    for fetch_bank in 0..8u64 {
        let fetch_addr = 0x2000_0100 + fetch_bank * 4; // banks 0..7
        write_thumb(&mut core, fetch_addr, LDR_R0_R1)?;
        let mut samples = Vec::with_capacity(NUM_SAMPLES);
        for _ in 0..NUM_SAMPLES {
            core.write_core_reg(R1, data_fixed)?;
            core.write_core_reg(R0, 0u64)?;
            core.write_core_reg(PC, fetch_addr)?;
            reset_cyccnt(&mut core)?;
            core.step()?;
            samples.push(read_cyccnt(&mut core)?);
        }
        let med = median(&samples);
        println!(
            "  fetch=0x{fetch_addr:08X} bank={}  data=0x{data_fixed:08X} bank={}  median={med}  samples={:?}",
            bank_of(fetch_addr),
            bank_of(data_fixed),
            &samples[..5],
        );
    }

    let elapsed = t0.elapsed();

    // -- Summary --
    println!();
    println!("==========================================================");
    println!("Summary (medians):");

    let med_a = median(&a);
    let med_b = median(&b);
    let med_e_same = median(&e_same);
    let med_e_diff = median(&e_diff);

    println!("  LDR same-bank  (0→0): {med_a}");
    println!("  LDR diff-bank  (0→1): {med_b}");
    for (addr, samples) in &d_results {
        println!(
            "  LDR diff-bank  (0→{}): {}",
            bank_of(*addr),
            median(samples)
        );
    }
    println!("  STR same-bank  (0→0): {med_e_same}");
    println!("  STR diff-bank  (0→1): {med_e_diff}");

    // Determine if bank conflict is confirmed.
    // If same-bank is consistently higher than all diff-bank, it's confirmed.
    let all_diff_ldr: Vec<u32> = std::iter::once(med_b)
        .chain(d_results.iter().map(|(_, s)| median(s)))
        .collect();
    let max_diff_ldr = *all_diff_ldr.iter().max().unwrap();

    let ldr_conflict = med_a > max_diff_ldr;
    let str_conflict = med_e_same > med_e_diff;

    println!();
    if ldr_conflict && str_conflict {
        println!("Conclusion: bank conflict CONFIRMED (both LDR and STR show +1 for same-bank)");
    } else if ldr_conflict {
        println!("Conclusion: bank conflict CONFIRMED for LDR only (STR unaffected)");
    } else if str_conflict {
        println!("Conclusion: bank conflict CONFIRMED for STR only (LDR unaffected)");
    } else {
        println!("Conclusion: bank conflict NOT CONFIRMED (same-bank and diff-bank show equal cycles)");
    }
    println!("  LDR delta (same - max_diff): {}", med_a as i32 - max_diff_ldr as i32);
    println!("  STR delta (same - diff):     {}", med_e_same as i32 - med_e_diff as i32);
    println!();
    println!("Completed in {:.1}s", elapsed.as_secs_f64());

    Ok(())
}
