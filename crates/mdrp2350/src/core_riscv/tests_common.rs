//! Shared test fixtures used by `tests_p2`/`p3`/`p4`/`p5`. Lifted out of
//! per-file duplication on 2026-04-26 (tech-debt sweep, action plan C3).
//! Add helpers here as new ones become 2+-way duplicated; kept narrow on
//! purpose so each `tests_pN.rs` still owns its own per-phase scaffolding.

#![cfg(test)]

use super::Hazard3;
use crate::Bus;

/// Construct a fresh Hazard3 + Bus pair. The executor tests load
/// instruction words into SRAM at offset 0 and step the hart with its
/// reset PC still pointing at SRAM base (`0x2000_0000`).
pub(super) fn fresh() -> (Hazard3, Bus) {
    (Hazard3::new(0), Bus::new())
}

/// Write a 32-bit instruction word into SRAM at `sram_offset`
/// (offset from `0x2000_0000` / SRAM base).
pub(super) fn write_insn(bus: &mut Bus, sram_offset: u32, insn: u32) {
    bus.memory.sram_write32(sram_offset, insn);
}

/// Write a 16-bit halfword (e.g. an RV32C compressed instruction) into
/// SRAM at `sram_offset`.
pub(super) fn write_hw(bus: &mut Bus, sram_offset: u32, hw: u16) {
    bus.memory.sram_write16(sram_offset, hw);
}
