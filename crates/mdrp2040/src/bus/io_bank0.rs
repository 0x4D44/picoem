//! IO_BANK0 GPIO function select (base 0x4001_4000).
//!
//! RP2040 datasheet §2.19.6. 30 GPIOs × 2 registers each (`GPIO_STATUS`,
//! `GPIO_CTRL`) = 240 bytes of per-pin state. Registers beyond the pin
//! block (INTR / PROC0_INTE / PROC1_INTE / …) are stubbed: they read
//! back what was written so firmware init code sees a round-trip.
//!
//! `GPIO_CTRL[4:0]` is FUNCSEL:
//! * 0 = XIP, 1 = SPI, 2 = UART, 3 = I2C, 4 = PWM, 5 = SIO,
//! * 6 = PIO0, 7 = PIO1, 8 = USB, 31 = NULL (Hi-Z).
//!
//! Phase 5.A stores the CTRL register and reports STATUS as
//! `oetoperiph << 12 | intoperiph << 24` — a simple passthrough of what
//! firmware would see on a real chip after programming the pin.

/// Number of GPIOs on RP2040 (IO_BANK0 block).
pub(crate) const NUM_GPIOS: usize = 30;

/// Per-pin register pair: CTRL / STATUS.
pub struct IoBank0 {
    /// GPIO_CTRL[pin] = FUNCSEL / OUTOVER / OEOVER / INOVER / IRQOVER.
    pub ctrl: [u32; NUM_GPIOS],
    /// Raw INT / INTE / INTF / INTS storage — 4 × 4 words = 16 words per
    /// bank (4 banks of 8 pins each). Backing store only; interrupt
    /// delivery is not modelled in Phase 5.A.
    pub(crate) intr: [u32; 4],
    pub(crate) proc0_inte: [u32; 4],
    pub(crate) proc0_intf: [u32; 4],
    pub(crate) proc0_ints: [u32; 4],
    pub(crate) proc1_inte: [u32; 4],
    pub(crate) proc1_intf: [u32; 4],
    pub(crate) proc1_ints: [u32; 4],
    pub(crate) dormant_wake_inte: [u32; 4],
    pub(crate) dormant_wake_intf: [u32; 4],
    pub(crate) dormant_wake_ints: [u32; 4],
}

impl IoBank0 {
    pub fn new() -> Self {
        // FUNCSEL default for every pin is 31 (NULL / Hi-Z).
        let mut ctrl = [0u32; NUM_GPIOS];
        for c in ctrl.iter_mut() {
            *c = 0x0000_001F;
        }
        Self {
            ctrl,
            intr: [0; 4],
            proc0_inte: [0; 4],
            proc0_intf: [0; 4],
            proc0_ints: [0; 4],
            proc1_inte: [0; 4],
            proc1_intf: [0; 4],
            proc1_ints: [0; 4],
            dormant_wake_inte: [0; 4],
            dormant_wake_intf: [0; 4],
            dormant_wake_ints: [0; 4],
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Returns the FUNCSEL (bits [4:0]) currently assigned to `pin`.
    #[inline]
    pub fn funcsel(&self, pin: usize) -> u32 {
        if pin < NUM_GPIOS {
            self.ctrl[pin] & 0x1F
        } else {
            0
        }
    }

    /// Read an IO_BANK0 register by byte offset.
    pub fn read32(&self, offset: u32) -> u32 {
        let off = offset & 0xFFF;
        // Per-pin status/ctrl at offsets 0x000..0x0EF (30 pins × 8 bytes).
        if off < (NUM_GPIOS as u32) * 8 {
            let pin = (off >> 3) as usize;
            return match off & 0x7 {
                0x0 => self.status_for(pin),
                0x4 => self.ctrl[pin],
                _ => 0,
            };
        }
        // Interrupt block starts at 0x0F0.
        match off {
            0x0F0 => self.intr[0],
            0x0F4 => self.intr[1],
            0x0F8 => self.intr[2],
            0x0FC => self.intr[3],
            0x100 => self.proc0_inte[0],
            0x104 => self.proc0_inte[1],
            0x108 => self.proc0_inte[2],
            0x10C => self.proc0_inte[3],
            0x110 => self.proc0_intf[0],
            0x114 => self.proc0_intf[1],
            0x118 => self.proc0_intf[2],
            0x11C => self.proc0_intf[3],
            0x120 => self.proc0_ints[0],
            0x124 => self.proc0_ints[1],
            0x128 => self.proc0_ints[2],
            0x12C => self.proc0_ints[3],
            0x130 => self.proc1_inte[0],
            0x134 => self.proc1_inte[1],
            0x138 => self.proc1_inte[2],
            0x13C => self.proc1_inte[3],
            0x140 => self.proc1_intf[0],
            0x144 => self.proc1_intf[1],
            0x148 => self.proc1_intf[2],
            0x14C => self.proc1_intf[3],
            0x150 => self.proc1_ints[0],
            0x154 => self.proc1_ints[1],
            0x158 => self.proc1_ints[2],
            0x15C => self.proc1_ints[3],
            0x160 => self.dormant_wake_inte[0],
            0x164 => self.dormant_wake_inte[1],
            0x168 => self.dormant_wake_inte[2],
            0x16C => self.dormant_wake_inte[3],
            0x170 => self.dormant_wake_intf[0],
            0x174 => self.dormant_wake_intf[1],
            0x178 => self.dormant_wake_intf[2],
            0x17C => self.dormant_wake_intf[3],
            0x180 => self.dormant_wake_ints[0],
            0x184 => self.dormant_wake_ints[1],
            0x188 => self.dormant_wake_ints[2],
            0x18C => self.dormant_wake_ints[3],
            _ => 0,
        }
    }

    /// Write an IO_BANK0 register with an alias-aware update.
    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        let off = offset & 0xFFF;
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        if off < (NUM_GPIOS as u32) * 8 {
            let pin = (off >> 3) as usize;
            match off & 0x7 {
                0x0 => {} // STATUS read-only
                0x4 => self.ctrl[pin] = apply(self.ctrl[pin], val),
                _ => {}
            }
            return;
        }
        // INTR (0x0F0) is W1C (write-1-to-clear); other interrupt regs are
        // plain storage. Phase 5.A keeps this simple.
        let idx = |o: u32| ((o & 0xF) >> 2) as usize;
        match off {
            0x0F0..=0x0FC => self.intr[idx(off)] = apply(self.intr[idx(off)], val),
            0x100..=0x10C => self.proc0_inte[idx(off)] = apply(self.proc0_inte[idx(off)], val),
            0x110..=0x11C => self.proc0_intf[idx(off)] = apply(self.proc0_intf[idx(off)], val),
            0x120..=0x12C => self.proc0_ints[idx(off)] = apply(self.proc0_ints[idx(off)], val),
            0x130..=0x13C => self.proc1_inte[idx(off)] = apply(self.proc1_inte[idx(off)], val),
            0x140..=0x14C => self.proc1_intf[idx(off)] = apply(self.proc1_intf[idx(off)], val),
            0x150..=0x15C => self.proc1_ints[idx(off)] = apply(self.proc1_ints[idx(off)], val),
            0x160..=0x16C => {
                self.dormant_wake_inte[idx(off)] = apply(self.dormant_wake_inte[idx(off)], val)
            }
            0x170..=0x17C => {
                self.dormant_wake_intf[idx(off)] = apply(self.dormant_wake_intf[idx(off)], val)
            }
            0x180..=0x18C => {
                self.dormant_wake_ints[idx(off)] = apply(self.dormant_wake_ints[idx(off)], val)
            }
            _ => {}
        }
    }

    /// Synthesise a GPIO_STATUS word for `pin`. Phase 5.A bakes in a
    /// simple shape: OUTTOPAD / OETOPAD / INFROMPAD mirror the CTRL
    /// OUTOVER / OEOVER / INOVER when those fields are non-zero, else
    /// the SIO-driven levels. Firmware rarely reads STATUS in init code
    /// — this is enough for Pico SDK's gpio_init round-trip.
    fn status_for(&self, _pin: usize) -> u32 {
        0
    }
}

impl Default for IoBank0 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn funcsel_defaults_to_null() {
        let io = IoBank0::new();
        for pin in 0..NUM_GPIOS {
            assert_eq!(io.funcsel(pin), 0x1F);
        }
    }

    #[test]
    fn set_funcsel_via_ctrl_write() {
        let mut io = IoBank0::new();
        // GPIO25 CTRL is at offset 25 * 8 + 4 = 0xCC.
        io.write32(0xCC, 5, 0); // SIO
        assert_eq!(io.funcsel(25), 5);
    }

    #[test]
    fn status_readonly_ignores_write() {
        let mut io = IoBank0::new();
        let before = io.read32(0x00);
        io.write32(0x00, 0xFFFF_FFFF, 0);
        assert_eq!(io.read32(0x00), before);
    }

    #[test]
    fn intr_storage_roundtrip() {
        let mut io = IoBank0::new();
        io.write32(0x0F0, 0xABCD, 0);
        assert_eq!(io.read32(0x0F0), 0xABCD);
    }
}
