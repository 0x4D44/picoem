//! RP2350-specific memory bank topology.
//!
//! The storage primitive (`Memory`) and its scalar accessors live in
//! [`picoem_common::memory`]. The bank-mapping function below encodes
//! the RP2350 SRAM layout (8-stripe + 2-scratch); it is *not* chip-agnostic
//! and therefore sits alongside the RP2350 contention model in this crate.

pub use picoem_common::memory::{Memory, ROM_SIZE, SRAM_SIZE};

/// Returns the SRAM bank number (0-9) for a given address in the SRAM region.
/// SRAM0-7: word-striped, bank = (word_offset) % 8
/// SRAM8: offset 0x80000-0x80FFF (4KB)
/// SRAM9: offset 0x81000-0x81FFF (4KB)
/// Returns None if the offset is outside SRAM range.
///
/// Accepts a full address (0x20xx_xxxx through 0x23xx_xxxx); strips alias
/// and base bits internally. Alias addresses resolve to the same bank.
pub fn bank_for_address(addr: u32) -> Option<u8> {
    if (addr >> 28) != 0x2 {
        return None;
    }
    let offset = addr & 0x00FF_FFFF; // strip alias bits [27:24]
    if offset < 0x8_0000 {
        // Striped region: 0x00000-0x7FFFF (512KB)
        Some(((offset >> 2) & 7) as u8)
    } else if offset <= 0x8_0FFF {
        // SRAM8: 0x80000-0x80FFF
        Some(8)
    } else if offset <= 0x8_1FFF {
        // SRAM9: 0x81000-0x81FFF
        Some(9)
    } else {
        None
    }
}
