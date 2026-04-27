//! OneROM full-system oracle — sync detection and PIO snapshot capture (F.2).
//!
//! The real `sdrr`/OneROM firmware splits its serving pipeline across two
//! PIO blocks (`BLOCK_ADDR = PIO1`, `BLOCK_DATA = PIO2`); `BLOCK_MONITOR
//! = PIO0` is unused in the typical serving mode. "Sync" = both serving
//! blocks have at least one state-machine enabled, i.e. OneROM's
//! `setup_onerom` has finished `apio_sm_set_enabled()` on both.
//!
//! `capture_snapshot` reads all readback-safe registers from PIO0/1/2
//! into a typed report. RXF offsets (`0x020..0x02C`) are deliberately
//! **not** read — those pop the FIFO.
//!
//! Design: `wrk_docs/2026.04.14 - HLD - OneROM Full-System Oracle.md` §5.

use mdrp2350::Bus;

/// MMIO base addresses of PIO0, PIO1, PIO2.
pub const PIO_BASES: [u32; 3] = [0x5020_0000, 0x5030_0000, 0x5040_0000];

/// PIO `CTRL` register offset within a block.
pub const PIO_CTRL: u32 = 0x000;

/// Per-SM register block base inside a PIO block (`SMn` starts at `SM_BASE + n*SM_STRIDE`).
pub const SM_BASE: u32 = 0x0C8;

/// Stride between per-SM register blocks.
pub const SM_STRIDE: u32 = 0x18;

/// `DBG_PADOUT` register offset.
pub const DBG_PADOUT: u32 = 0x03C;

/// `DBG_PADOE` register offset.
pub const DBG_PADOE: u32 = 0x040;

/// Snapshot of one state machine's readback-safe registers.
#[derive(Clone, Copy, Debug, Default)]
pub struct SmSnapshot {
    pub block: u8,
    pub sm: u8,
    pub clkdiv: u32,
    pub execctrl: u32,
    pub shiftctrl: u32,
    pub pinctrl: u32,
    pub addr: u32,
    pub last_insn: u32,
}

/// Snapshot of one PIO block — CTRL, program memory, four SMs, and debug
/// pad latches.
#[derive(Clone, Debug)]
pub struct PioSnapshot {
    pub block: u8,
    pub ctrl: u32,
    pub instr_mem: [u16; 32],
    pub sms: [SmSnapshot; 4],
    pub dbg_padout: u32,
    pub dbg_padoe: u32,
}

/// Full three-block snapshot plus the emulator cycle count it was taken at.
#[derive(Clone, Debug)]
pub struct SyncReport {
    pub cycle: u64,
    pub pio0: PioSnapshot,
    pub pio1: PioSnapshot,
    pub pio2: PioSnapshot,
}

/// Cheap sync check — two `bus.read32` calls.
///
/// Returns true once `BLOCK_ADDR` (PIO1) and `BLOCK_DATA` (PIO2) both have
/// at least one SM enabled. PIO0 is ignored (monitor block, optional).
pub fn is_synced(bus: &mut Bus) -> bool {
    let ctrl1 = bus.read32(PIO_BASES[1] + PIO_CTRL, 0);
    let ctrl2 = bus.read32(PIO_BASES[2] + PIO_CTRL, 0);
    (ctrl1 & 0xF) != 0 && (ctrl2 & 0xF) != 0
}

/// Capture a full `SyncReport` from all three PIO blocks.
///
/// Reads only readback-safe registers (skips RXF — reading it pops the
/// FIFO). Called once on the cycle sync is first detected.
pub fn capture_snapshot(bus: &mut Bus, cycle: u64) -> SyncReport {
    SyncReport {
        cycle,
        pio0: capture_block(bus, 0),
        pio1: capture_block(bus, 1),
        pio2: capture_block(bus, 2),
    }
}

fn capture_block(bus: &mut Bus, block: u8) -> PioSnapshot {
    let base = PIO_BASES[block as usize];
    let ctrl = bus.read32(base + PIO_CTRL, 0);
    let dbg_padout = bus.read32(base + DBG_PADOUT, 0);
    let dbg_padoe = bus.read32(base + DBG_PADOE, 0);

    // INSTR_MEM is write-only via MMIO: the PIO block returns 0 for
    // reads at `0x048..=0x0C4` (see `mdpicoem_common::pio::PioBlock::read32`).
    // Use the dedicated `instr_mem()` accessor, which exposes the backing
    // storage for tooling exactly like this.
    let instr_mem = *bus.pio[block as usize].instr_mem();

    let mut sms = [SmSnapshot::default(); 4];
    for sm in 0..4u8 {
        let sm_base = base + SM_BASE + (sm as u32) * SM_STRIDE;
        sms[sm as usize] = SmSnapshot {
            block,
            sm,
            clkdiv: bus.read32(sm_base + 0x00, 0),
            execctrl: bus.read32(sm_base + 0x04, 0),
            shiftctrl: bus.read32(sm_base + 0x08, 0),
            addr: bus.read32(sm_base + 0x0C, 0),
            last_insn: bus.read32(sm_base + 0x10, 0),
            pinctrl: bus.read32(sm_base + 0x14, 0),
        };
    }

    PioSnapshot {
        block,
        ctrl,
        instr_mem,
        sms,
        dbg_padout,
        dbg_padoe,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mdrp2350::{Config, EmulatorBuilder};

    /// PIO out-of-reset bit in the RESETS.RESET register.
    ///
    /// RESETS bit layout (see `mdrp2350/src/bus/peripherals.rs`): bit 15 =
    /// PIO0, bit 16 = PIO1, bit 17 = PIO2. We clear all three so the
    /// `pio[b].read32` path behaves.
    const RESETS_RESET_OFFSET: u32 = 0x4002_0000;
    const PIO_RESET_MASK: u32 = (1 << 15) | (1 << 16) | (1 << 17);

    fn new_bus() -> mdrp2350::Emulator {
        // Build an emulator so we can use its `Bus` — the bus alone can't
        // be constructed directly from outside the crate.
        EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build is infallible")
    }

    fn release_pio_reset(bus: &mut Bus) {
        // RESETS register at 0x4002_0000 is the reset *hold* mask; write
        // alias=3 (CLR) to clear PIO bits.
        bus.write32(RESETS_RESET_OFFSET | (3 << 12), PIO_RESET_MASK, 0);
    }

    #[test]
    fn is_synced_false_at_reset() {
        let mut emu = new_bus();
        assert!(!is_synced(&mut emu.bus));
    }

    #[test]
    fn is_synced_requires_both_blocks() {
        let mut emu = new_bus();
        release_pio_reset(&mut emu.bus);

        // Only PIO1 enabled → not synced yet.
        emu.bus.write32(PIO_BASES[1] + PIO_CTRL, 0b0001, 0);
        assert!(!is_synced(&mut emu.bus));

        // Now PIO2 too — synced.
        emu.bus.write32(PIO_BASES[2] + PIO_CTRL, 0b0011, 0);
        assert!(is_synced(&mut emu.bus));
    }

    /// Bringing PIO1 SM0 through the standard init dance and verifying
    /// `capture_snapshot` reads back the configured register values
    /// cleanly. This is the F.2 end-to-end readback check.
    #[test]
    fn capture_snapshot_reads_back_clean() {
        let mut emu = new_bus();
        release_pio_reset(&mut emu.bus);

        // Program a single instruction: JMP 0 (opcode 0x0000) in slot 0.
        emu.bus.write32(PIO_BASES[1] + 0x048, 0x0000, 0);

        // SM0 configuration.
        let sm_base = PIO_BASES[1] + SM_BASE;
        let clkdiv_val = (1302u32 << 16) | (128u32 << 8); // int=1302, frac=128
        emu.bus.write32(sm_base + 0x00, clkdiv_val, 0);
        let pinctrl_val = (5u32 << 26) | (3u32 << 20); // SET_COUNT=5, OUT_COUNT=3
        emu.bus.write32(sm_base + 0x14, pinctrl_val, 0);

        // Enable SM0.
        emu.bus.write32(PIO_BASES[1] + PIO_CTRL, 0b0001, 0);

        let report = capture_snapshot(&mut emu.bus, 12345);
        assert_eq!(report.cycle, 12345);
        assert_eq!(report.pio1.ctrl & 0xF, 0b0001);
        assert_eq!(report.pio1.sms[0].clkdiv, clkdiv_val);
        assert_eq!(report.pio1.sms[0].pinctrl, pinctrl_val);
        assert_eq!(report.pio1.instr_mem[0], 0x0000);
        // PIO0/2 untouched → CTRL=0.
        assert_eq!(report.pio0.ctrl & 0xF, 0);
        assert_eq!(report.pio2.ctrl & 0xF, 0);
    }
}
