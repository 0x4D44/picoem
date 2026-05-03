//! Disposable diagnostic: dump full 64-byte `sdrr_rom_set_t` descriptor
//! for every ROM set in a fixture and diff them pairwise. Also runs a
//! small bank of speculative patch experiments — each candidate flips
//! one byte in the failing set's descriptor and tries to boot it.
//!
//! Once analysis is complete this binary should be removed.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use mdpicoem_harness::{onerom_serving_oracle, onerom_serving_oracle_cpu};
use mdrp2350::{Config, EmulatorBuilder};

const FLASH_BASE: u32 = 0x1000_0000;
const SDRR_INFO_OFFSET: usize = 0x0200;
const SDRR_INFO_METADATA_PTR_OFFSET: usize = 44;
const METADATA_HEADER_ROM_SETS_PTR_OFFSET: usize = 24;
const METADATA_HEADER_ROM_SET_COUNT_OFFSET: usize = 20;
const ROM_SET_STRIDE: usize = 64;
const DEFAULT_FIXTURE: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

fn ptr_to_off(flash: &[u8], ptr: u32) -> Option<usize> {
    let off = ptr.checked_sub(FLASH_BASE)? as usize;
    if off >= flash.len() { None } else { Some(off) }
}

fn read_u32(flash: &[u8], off: usize) -> Option<u32> {
    let bytes = flash.get(off..off + 4)?;
    Some(u32::from_le_bytes(bytes.try_into().ok()?))
}

fn locate_rom_sets(flash: &[u8]) -> Option<(usize, usize)> {
    let metadata_ptr = read_u32(flash, SDRR_INFO_OFFSET + SDRR_INFO_METADATA_PTR_OFFSET)?;
    let metadata_off = ptr_to_off(flash, metadata_ptr)?;
    let count = *flash.get(metadata_off + METADATA_HEADER_ROM_SET_COUNT_OFFSET)? as usize;
    let rom_sets_ptr = read_u32(flash, metadata_off + METADATA_HEADER_ROM_SETS_PTR_OFFSET)?;
    let rom_sets_off = ptr_to_off(flash, rom_sets_ptr)?;
    Some((rom_sets_off, count))
}

fn dump_descriptor(idx: usize, bytes: &[u8]) {
    println!("=== ROM set {} ===", idx);
    for row in 0..(ROM_SET_STRIDE / 16) {
        print!("  +0x{:02X}: ", row * 16);
        for col in 0..16 {
            print!("{:02X} ", bytes[row * 16 + col]);
        }
        print!(" |");
        for col in 0..16 {
            let b = bytes[row * 16 + col];
            print!("{}", if (0x20..=0x7E).contains(&b) { b as char } else { '.' });
        }
        println!("|");
    }
    // Field decode for the known prefix
    let data_ptr = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
    let size = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
    println!("  data_ptr=0x{:08X}  size=0x{:08X} ({} bytes)", data_ptr, size, size);
}

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const BOOT_CYCLE_CAP: u64 = 5_000_000;

fn try_boot(bootrom: &[u8], flash: &[u8], rom_set_index: u32) -> Result<u64, String> {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .map_err(|e| format!("emulator build failed: {e:?}"))?;
    emu.load_bootrom(bootrom);
    emu.load_flash(flash);
    emu.reset();

    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);
    emu.core_mut(1).halt();

    onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins(&mut emu, flash, rom_set_index)?;

    // Phase 1: PC enters the serve-loop range.
    let mut phase1: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        if emu.cycles() == before { return Err(format!("stalled at {before}")); }
        if onerom_serving_oracle_cpu::is_synced_cpu(&emu, None) {
            phase1 = Some(emu.cycles());
            break;
        }
    }
    phase1.ok_or_else(|| format!("Phase1 timeout @ {} cycles", BOOT_CYCLE_CAP))?;

    // Phase 2: PC + sentinel.
    const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
    const ROM_SET_INDEX_OFFSET: u32 = 6;
    let live_index = emu.bus.memory.sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);
    let sentinel = onerom_serving_oracle::lift_shadow_from_flash_pub(flash, live_index)
        .and_then(|s| onerom_serving_oracle_cpu::find_shadow_sentinel(&s));
    while !onerom_serving_oracle_cpu::is_synced_cpu(&emu, sentinel) && emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        if emu.cycles() == before { return Err(format!("stalled at {before}")); }
    }
    if !onerom_serving_oracle_cpu::is_synced_cpu(&emu, sentinel) {
        return Err(format!("Phase2 timeout @ {} cycles", BOOT_CYCLE_CAP));
    }
    Ok(emu.cycles())
}

fn run_patch_trial(label: &str, bootrom: &[u8], base_flash: &[u8], rom_set_index: u32,
                    patches: &[(usize, u8)]) {
    let mut flash = base_flash.to_vec();
    for &(off, val) in patches {
        flash[off] = val;
    }
    let t0 = Instant::now();
    match try_boot(bootrom, &flash, rom_set_index) {
        Ok(c) => println!("  [{label}] SYNCED at cycle {} ({} ms)", c, t0.elapsed().as_millis()),
        Err(e) => println!("  [{label}] TIMEOUT — {} ({} ms)", e, t0.elapsed().as_millis()),
    }
}

fn main() -> ExitCode {
    let fixture = std::env::args().nth(1).map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_FIXTURE));

    let flash = match std::fs::read(&fixture) {
        Ok(b) => b,
        Err(e) => { eprintln!("read {}: {e}", fixture.display()); return ExitCode::from(2); }
    };

    let (rom_sets_off, count) = match locate_rom_sets(&flash) {
        Some(v) => v,
        None => { eprintln!("metadata parse failed"); return ExitCode::from(3); }
    };

    println!("fixture: {}", fixture.display());
    println!("flash bytes: {}", flash.len());
    println!("rom_sets array offset: 0x{:X}  count: {}", rom_sets_off, count);
    // Quick layout cross-check via parse_rom_set_layout:
    if let Some(layout) = onerom_serving_oracle::parse_rom_set_layout(&flash) {
        for (k, slot) in layout.iter().enumerate() {
            println!("  parse_rom_set_layout[{}]: data_offset=0x{:X} size={}", k, slot.data_offset, slot.size);
        }
    }
    println!();

    // Capture each descriptor
    let mut descs: Vec<[u8; ROM_SET_STRIDE]> = Vec::new();
    for k in 0..count {
        let off = rom_sets_off + k * ROM_SET_STRIDE;
        let bytes: &[u8] = &flash[off..off + ROM_SET_STRIDE];
        let arr: [u8; ROM_SET_STRIDE] = bytes.try_into().unwrap();
        dump_descriptor(k, &arr);
        descs.push(arr);
        println!();
    }

    // Per-byte diff table (skip data_ptr+size — we already know they vary).
    println!("=== Per-byte diff (offsets 8..64) ===");
    println!("offset  set0 set1 set2 set3 ...   note");
    for off in 8..ROM_SET_STRIDE {
        let v0 = descs[0][off];
        let all_equal = descs.iter().all(|d| d[off] == v0);
        if all_equal {
            // Suppress all-equal rows to keep output tight; mark only if non-zero.
            if v0 != 0 {
                let row: String = descs.iter()
                    .map(|d| format!("{:02X} ", d[off]))
                    .collect();
                println!("  +0x{:02X}  {}  all-equal (nz)", off, row);
            }
            continue;
        }
        let row: String = descs.iter()
            .map(|d| format!("{:02X} ", d[off]))
            .collect();
        // Identify which sets disagree with set 0.
        let differ: Vec<usize> = (1..descs.len())
            .filter(|&i| descs[i][off] != v0).collect();
        println!("  +0x{:02X}  {}  differ_from_0={:?}", off, row, differ);
    }
    println!();
    println!("(rows where all sets share the same value of 0x00 are suppressed)");

    // ---------- Patch trials ----------
    let bootrom = match std::fs::read(BOOTROM_PATH) {
        Ok(b) => b,
        Err(e) => { eprintln!("read bootrom: {e}"); return ExitCode::from(2); }
    };

    println!();
    println!("=== Baseline: try set 3 unmodified (sanity) ===");
    run_patch_trial("baseline_set3", &bootrom, &flash, 3, &[]);
    println!();
    println!("=== Speculative patches on set 3 (descriptor at 0x{:X}) ===", rom_sets_off + 3 * ROM_SET_STRIDE);
    let set3_off = rom_sets_off + 3 * ROM_SET_STRIDE;
    let set0_off = rom_sets_off;
    let set1_off = rom_sets_off + ROM_SET_STRIDE;
    // Trial 1: copy set 0's +0x08 byte (0x10) onto set 3's +0x08 (was 0x04).
    //          Makes set 3's roms[] pointer == set 0's.
    run_patch_trial("patch_+0x08_to_set0_value (0x04 -> 0x10)",
                    &bootrom, &flash, 3, &[(set3_off + 0x08, flash[set0_off + 0x08])]);
    // Trial 2: copy set 1's +0x08 byte (0x0C) onto set 3.
    run_patch_trial("patch_+0x08_to_set1_value (0x04 -> 0x0C)",
                    &bootrom, &flash, 3, &[(set3_off + 0x08, flash[set1_off + 0x08])]);
    // Trial 3: copy ALL of set 0's descriptor tail (+0x08..+0x40) onto set 3.
    //          Leaves data_ptr/size intact but normalises every other field.
    let mut full_tail_patches: Vec<(usize, u8)> = Vec::new();
    for off in 8..ROM_SET_STRIDE {
        full_tail_patches.push((set3_off + off, flash[set0_off + off]));
    }
    run_patch_trial("patch_entire_+0x08..+0x40_from_set0",
                    &bootrom, &flash, 3, &full_tail_patches);

    ExitCode::SUCCESS
}
