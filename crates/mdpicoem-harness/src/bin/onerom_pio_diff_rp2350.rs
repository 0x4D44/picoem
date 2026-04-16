//! OneROM PIO differential — pure-Rust diff binary.
//!
//! Reads `crates/mdpicoem-harness/oracles/onerom_2364.trace`, rebuilds
//! the scenario in our PIO (`mdpicoem-common::pio::PioBlock`), replays
//! the committed input-pin stimulus, and diffs the output pin state
//! cycle-by-cycle against the trace body.
//!
//! Design: `wrk_docs/2026.04.14 - HLD - OneROM PIO Differential.md` §§7-9.
//!
//! Zero C dependencies — the trace file is the only interface to epio.

use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

use mdpicoem_common::pio::PioBlock;
use mdpicoem_harness::onerom_trace::{self, Trace};

/// Start PCs for OneROM's three SMs (derived from `setup_onerom`'s
/// instruction layout: SM0 = 6 instrs at 0-5, SM1 = 2 instrs at 6-7,
/// SM2 = 1 instr at 8).
const SM_STARTS: [u8; 4] = [0, 6, 8, 0];

/// Enabled SMs — SM3 is unused by OneROM.
const SM_ENABLE_MASK: u32 = 0b0111;

/// Value preloaded into SM1's TX FIFO by `setup_onerom`'s `APIO_TXF`.
const SM1_TXF_PRELOAD: u32 = 0x0000_2000;

/// Pre-init instructions replayed on SM1 by `setup_onerom`'s
/// `APIO_SM_EXEC_INSTR` calls, in order.
const SM1_PRE_INSTRS: &[u16] = &[
    0x80A0, // PULL BLOCK    (100_0_0000_1010_0000)
    0xA027, // MOV X, OSR    (101_0_0000_0010_0111)
];

// Trace file model (parsing lives in `mdpicoem_harness::onerom_trace`).

// ---------------------------------------------------------------------------
// Our-PIO setup — reproduce epio's post-`epio_from_apio()` state
// ---------------------------------------------------------------------------

fn setup_pio(trace: &Trace) -> PioBlock {
    let mut pio = PioBlock::new();

    // Load instr_mem slot-by-slot.
    for (i, &insn) in trace.instrs.iter().enumerate() {
        let offset = 0x048 + (i as u32) * 4;
        pio.write32(offset, insn as u32, 0);
    }

    // Configure each SM (SM3 regs are zero — skipping them avoids
    // `clkdiv=0` landmines, though write32 tolerates it).
    for sm in 0..4u32 {
        let reg = &trace.regs[sm as usize];
        let base = 0x0C8 + sm * 0x18;
        pio.write32(base + 0x00, reg.clkdiv, 0);
        pio.write32(base + 0x04, reg.execctrl, 0);
        pio.write32(base + 0x08, reg.shiftctrl, 0);
        pio.write32(base + 0x14, reg.pinctrl, 0);
    }

    // SM1: preload TX FIFO with 0x2000 (`APIO_TXF` in setup_onerom).
    pio.write32(0x014, SM1_TXF_PRELOAD, 0);

    // SM1: replay pre-init instructions from `APIO_SM_EXEC_INSTR`.
    let sm1_instr_offset = 0x0C8 + 1 * 0x18 + 0x10;
    for &insn in SM1_PRE_INSTRS {
        pio.write32(sm1_instr_offset, insn as u32, 0);
    }

    // `APIO_SM_JMP_TO_START()` — force each SM's PC to its program start
    // by force-executing a JMP to that target. JMP with no condition is
    // opcode 0x0000 | (target & 0x1F).
    for (sm, &start) in SM_STARTS.iter().enumerate() {
        if start == 0 {
            continue; // no-op: PC is already 0 after reset
        }
        let jmp = (start as u32) & 0x1F;
        let instr_offset = 0x0C8 + (sm as u32) * 0x18 + 0x10;
        pio.write32(instr_offset, jmp, 0);
    }

    // Enable SMs 0, 1, 2.
    pio.write32(0x000, SM_ENABLE_MASK, 0);

    pio
}

// ---------------------------------------------------------------------------
// Glue DMA (see LLD §8)
// ---------------------------------------------------------------------------

/// Replicates `epio_dma_setup_read_pio_chain(epio, 0, 0, 1, 4, 0, 2, 4, 8)`.
///
/// epio's chain: 4-cycle read latency + 4-cycle write latency = 8 cycles
/// from "SM1 RX FIFO has data" to "SM2 TX FIFO has the corresponding byte".
/// `bit_mode=8` reads one byte from SRAM and replicates across the 32-bit
/// word — on an all-zero SRAM that's `read_value = 0`.
///
/// Driven through the PIO register bus (FSTAT / RXF1 / TXF2) — no reach-in
/// to the `pub(crate)` FIFO accessors.
#[derive(Default)]
struct GlueDma {
    read_delay: u8,
    write_delay: u8,
    read_value: u32,
    has_pending: bool,
}

const DMA_READ_CYCLES: u8 = 4;
const DMA_WRITE_CYCLES: u8 = 4;

impl GlueDma {
    fn tick(&mut self, pio: &mut PioBlock) {
        // Writes first (to make room for new reads).
        if self.write_delay > 0 {
            self.write_delay -= 1;
            if self.write_delay == 0 && self.has_pending {
                // Check SM2 TX full — if full, retry next cycle.
                let fstat = pio.read32(0x004);
                let sm2_tx_full = (fstat >> (16 + 2)) & 1 != 0;
                if sm2_tx_full {
                    self.write_delay = 1;
                } else {
                    pio.write32(0x018, self.read_value, 0); // TXF2
                    self.has_pending = false;
                    self.read_value = 0;
                }
            }
        }

        // Read completion.
        if self.read_delay > 0 {
            self.read_delay -= 1;
            if self.read_delay == 0 {
                if self.write_delay > 0 {
                    // Write still in flight — retry next cycle.
                    self.read_delay = 1;
                } else {
                    // SRAM zero-init ⇒ byte 0, replicated 4× per bit_mode=8.
                    self.read_value = 0;
                    self.has_pending = true;
                    self.write_delay = DMA_WRITE_CYCLES;
                }
            }
        }

        // New read trigger.
        if self.read_delay == 0 && !self.has_pending {
            let fstat = pio.read32(0x004);
            let sm1_rx_empty = (fstat >> (8 + 1)) & 1 != 0;
            if !sm1_rx_empty {
                let _ = pio.read32(0x024); // RXF1 pops, address discarded
                self.read_delay = DMA_READ_CYCLES;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Pin-state composition
// ---------------------------------------------------------------------------

/// Compose the `gpio_in` value we hand to `PioBlock::step` given the
/// host's drive/level masks. Host-driven pins take their forced level;
/// every other pin reads as pulled-high (1), matching epio's
/// undriven-pin convention.
fn compose_gpio_in(input_drive: u32, input_level: u32) -> u32 {
    (input_level & input_drive) | !input_drive
}

/// Observable pin state from our side, composed to match epio's
/// `epio_read_driven_pins` + `epio_read_pin_states` semantics:
///
/// - Driven bits = host-driven | PIO-driven (pad_oe).
/// - Level bits:
///   - host-driven bits → `input_level`
///   - PIO-driven bits (and not host-driven) → `pad_out`
///   - undriven bits → 1 (pullup)
fn observe(pio: &PioBlock, input_drive: u32, input_level: u32) -> (u32, u32) {
    let pad_oe = pio.pad_oe;
    let pad_out = pio.pad_out;
    let drive = input_drive | pad_oe;
    let level = (input_drive & input_level)
        | (!input_drive & pad_oe & pad_out)
        | !(input_drive | pad_oe);
    (drive, level)
}

// ---------------------------------------------------------------------------
// Main diff loop
// ---------------------------------------------------------------------------

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    let args: Vec<String> = env::args().collect();
    let trace_path = if args.len() == 2 {
        PathBuf::from(&args[1])
    } else {
        PathBuf::from("crates/mdpicoem-harness/oracles/onerom_2364.trace")
    };

    let trace = match onerom_trace::parse_trace(&trace_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("failed to parse {}: {}", trace_path.display(), e);
            return ExitCode::from(2);
        }
    };

    println!(
        "loaded {} instrs, {} body rows from {}",
        trace.instrs.len(),
        trace.body.len(),
        trace_path.display()
    );

    let mut pio = setup_pio(&trace);

    let mut divergences = 0usize;
    let mut glue_dma = GlueDma::default();
    for row in &trace.body {
        let gpio_in = compose_gpio_in(row.input_drive, row.input_level);
        pio.step(gpio_in);
        glue_dma.tick(&mut pio);

        let (our_drive, our_level) = observe(&pio, row.input_drive, row.input_level);
        let drive_ok = our_drive == row.out_drive;
        let level_ok = our_level == row.out_level;

        if !(drive_ok && level_ok) {
            if divergences == 0 {
                println!();
                println!("FIRST DIVERGENCE:");
            }
            println!(
                "  cycle {:2}: expected drive=0x{:08X} level=0x{:08X}",
                row.cycle, row.out_drive, row.out_level
            );
            println!(
                "            got      drive=0x{:08X} level=0x{:08X}",
                our_drive, our_level
            );
            divergences += 1;
            if divergences >= 5 {
                println!("  ...stopping after 5 divergences");
                break;
            }
        }
    }

    if divergences == 0 {
        println!("PASS — all {} cycles match the committed trace", trace.body.len());
        ExitCode::SUCCESS
    } else {
        println!();
        println!("FAIL — {} divergences", divergences);
        ExitCode::FAILURE
    }
}
