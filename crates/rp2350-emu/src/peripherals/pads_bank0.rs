//! RP2350 PADS_BANK0 — pad drive strength / pull / input-enable
//! (datasheet §9.11.3, base `0x4003_8000`).
//!
//! Phase 2 plain-storage per HLD V5 §5.8: one u32 per pad plus
//! `VOLTAGE_SELECT` at offset 0. 48 pads after the VSEL word.
//!
//! V5 scope: writes land, reads return stored; pad behaviour is not
//! modelled — firmware that programs pull-ups simply sees the
//! round-trip. Scenarios observe UART/SPI/I2C via peripheral register
//! state, not pin state (§4 constraint), so pad modelling is
//! unnecessary for corpus coverage.

use super::apply_alias_rmw;

/// PADS_BANK0 base (RP2350 datasheet §9.11.3).
pub const PADS_BANK0_BASE: u32 = 0x4003_8000;

/// Offset: `VOLTAGE_SELECT` (0x00).
pub const VOLTAGE_SELECT_OFFSET: u32 = 0x00;

/// First GPIO pad register (offset 0x04).
const GPIO_PAD_START: u32 = 0x04;

/// Number of user GPIO pads.
pub const PAD_COUNT: usize = 48;

/// One word per pad → `GPIO_PAD_START + 48*4 = 0x04 + 0xC0 = 0xC4`.
const GPIO_PAD_END: u32 = GPIO_PAD_START + (PAD_COUNT as u32) * 4;

/// Per-pad register mask. Bits: bit 0=slewfast, bit 1=schmitt,
/// bit 2=pde, bit 3=pue, bits 4:5=drive, bit 6=ie, bit 7=od, bit 8=iso.
const PAD_MASK: u32 = 0x1FF;

pub struct PadsBank0Regs {
    voltage_select: u32,
    pads: [u32; PAD_COUNT],
}

impl PadsBank0Regs {
    pub fn new() -> Self {
        Self {
            voltage_select: 0,
            pads: [0; PAD_COUNT],
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Decode an offset to a pad index, or `None` for VSEL / OOB.
    fn pad_index(offset: u32) -> Option<usize> {
        if !(GPIO_PAD_START..GPIO_PAD_END).contains(&offset) {
            return None;
        }
        Some(((offset - GPIO_PAD_START) / 4) as usize)
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        if offset == VOLTAGE_SELECT_OFFSET {
            return self.voltage_select;
        }
        if let Some(idx) = Self::pad_index(offset) {
            return self.pads[idx];
        }
        0
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        if offset == VOLTAGE_SELECT_OFFSET {
            let mut stored = self.voltage_select;
            apply_alias_rmw(&mut stored, value, alias);
            self.voltage_select = stored & 0x1;
            return;
        }
        if let Some(idx) = Self::pad_index(offset) {
            let mut stored = self.pads[idx];
            apply_alias_rmw(&mut stored, value, alias);
            self.pads[idx] = stored & PAD_MASK;
        }
    }
}

impl Default for PadsBank0Regs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pad_offset(pin: u32) -> u32 {
        GPIO_PAD_START + pin * 4
    }

    #[test]
    fn reset_defaults() {
        let p = PadsBank0Regs::new();
        assert_eq!(p.voltage_select, 0);
        assert!(p.pads.iter().all(|&v| v == 0));
    }

    #[test]
    fn vsel_roundtrips_one_bit() {
        let mut p = PadsBank0Regs::new();
        p.write32(VOLTAGE_SELECT_OFFSET, 1, 0);
        assert_eq!(p.read32(VOLTAGE_SELECT_OFFSET), 1);
    }

    #[test]
    fn pad_register_roundtrip_gpio0() {
        let mut p = PadsBank0Regs::new();
        // Drive=4 mA (bits 4:5=10), IE=1 (bit 6).
        p.write32(pad_offset(0), 0x56, 0);
        assert_eq!(p.read32(pad_offset(0)), 0x56);
    }

    #[test]
    fn pad_register_roundtrip_last() {
        let mut p = PadsBank0Regs::new();
        p.write32(pad_offset(47), 0xFF, 0);
        assert_eq!(p.read32(pad_offset(47)), PAD_MASK & 0xFF);
    }

    #[test]
    fn pad_writes_masked_to_9_bits() {
        let mut p = PadsBank0Regs::new();
        p.write32(pad_offset(0), 0xFFFF_FFFF, 0);
        assert_eq!(p.read32(pad_offset(0)), PAD_MASK);
    }

    #[test]
    fn pad_bitset_alias() {
        let mut p = PadsBank0Regs::new();
        p.write32(pad_offset(0), 0x40, 0);
        p.write32(pad_offset(0), 0x01, 2); // BITSET slewfast
        assert_eq!(p.read32(pad_offset(0)), 0x41);
    }
}
