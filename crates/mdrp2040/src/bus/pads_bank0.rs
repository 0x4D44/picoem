//! PADS_BANK0 GPIO pad control (base 0x4001_C000).
//!
//! RP2040 datasheet §2.19.4. Layout:
//!
//! * `VOLTAGE_SELECT` at offset 0x00 — 0 = 3.3 V, 1 = 1.8 V.
//! * 30 × `GPIO<n>` at offsets 0x04..0x7C — 4 bytes per pin: OD / IE / DRIVE /
//!   PUE / PDE / SCHMITT / SLEWFAST.
//! * 2 × `SWCLK` / `SWD` at 0x80 / 0x84 — stubbed.
//!
//! Phase 5.A stores the per-pin configuration as opaque 32-bit words;
//! firmware reads what it wrote. Pad input-enable (IE, bit 6) is not
//! used to gate GPIO_IN in 5.A — input routing is managed entirely by
//! the SIO GPIO_IN merge path.

pub(crate) const NUM_PADS: usize = 30;

/// Default per-pin PADS register value (IE = 1, DRIVE = 4 mA, PUE = 1).
/// Matches Pico SDK defaults so firmware init round-trips cleanly.
pub(crate) const PAD_RESET: u32 = 0x0000_0056;

pub struct PadsBank0 {
    pub voltage_select: u32,
    pub pads: [u32; NUM_PADS],
    pub swclk: u32,
    pub swd: u32,
}

impl PadsBank0 {
    pub fn new() -> Self {
        Self {
            voltage_select: 0,
            pads: [PAD_RESET; NUM_PADS],
            swclk: PAD_RESET,
            swd: PAD_RESET,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn read32(&self, offset: u32) -> u32 {
        let off = offset & 0xFFF;
        match off {
            0x00 => self.voltage_select,
            0x04..=0x7C => {
                let idx = ((off - 0x04) >> 2) as usize;
                if idx < NUM_PADS {
                    self.pads[idx]
                } else {
                    0
                }
            }
            0x80 => self.swclk,
            0x84 => self.swd,
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        let off = offset & 0xFFF;
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        match off {
            0x00 => self.voltage_select = apply(self.voltage_select, val) & 0x1,
            0x04..=0x7C => {
                let idx = ((off - 0x04) >> 2) as usize;
                if idx < NUM_PADS {
                    self.pads[idx] = apply(self.pads[idx], val);
                }
            }
            0x80 => self.swclk = apply(self.swclk, val),
            0x84 => self.swd = apply(self.swd, val),
            _ => {}
        }
    }
}

impl Default for PadsBank0 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_pico_sdk() {
        let p = PadsBank0::new();
        for pad in p.pads.iter() {
            assert_eq!(*pad, PAD_RESET);
        }
    }

    #[test]
    fn write_roundtrips() {
        let mut p = PadsBank0::new();
        p.write32(0x04, 0x11, 0); // GPIO0
        assert_eq!(p.read32(0x04), 0x11);
    }

    #[test]
    fn voltage_select_masks_to_1_bit() {
        let mut p = PadsBank0::new();
        p.write32(0x00, 0xFFFF_FFFF, 0);
        assert_eq!(p.voltage_select, 1);
    }

    #[test]
    fn gpio29_is_last() {
        let mut p = PadsBank0::new();
        // GPIO29 offset = 0x04 + 29 * 4 = 0x78.
        p.write32(0x78, 0x5A, 0);
        assert_eq!(p.read32(0x78), 0x5A);
    }
}
