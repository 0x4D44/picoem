/// Single-cycle IO block.
///
/// GPIO output/OE/input registers + CPUID dispatch.
/// Phase 5 adds spinlocks, FIFOs, doorbells, divider, interpolators.
pub struct Sio {
    /// 32 hardware spinlocks.
    pub spinlocks: [u32; 32],
    /// SIO GPIO output register (offset 0x010).
    pub gpio_out: u32,
    /// SIO GPIO output enable register (offset 0x030).
    pub gpio_oe: u32,
    /// SIO GPIO input register (offset 0x004, always 0 — no external pin model).
    pub gpio_in: u32,
}

impl Sio {
    pub fn new() -> Self {
        Self {
            spinlocks: [0; 32],
            gpio_out: 0,
            gpio_oe: 0,
            gpio_in: 0,
        }
    }

    /// Explicitly reset all SIO state. Called from `Emulator::reset()`.
    pub fn reset(&mut self) {
        self.gpio_out = 0;
        self.gpio_oe = 0;
        self.gpio_in = 0;
    }

    /// 32-bit register read. `offset` is already masked to 12 bits by Bus.
    /// GPIO_HI_IN (0x008) is handled by Bus before calling this.
    pub fn read32(&self, offset: u32, core: usize) -> u32 {
        match offset {
            0x000 => core as u32,   // CPUID
            0x004 => self.gpio_in,  // GPIO_IN
            0x010 => self.gpio_out, // GPIO_OUT
            0x030 => self.gpio_oe,  // GPIO_OE
            _ => 0,
        }
    }

    /// 32-bit register write. `offset` is already masked to 12 bits by Bus.
    pub fn write32(&mut self, offset: u32, val: u32, _core: usize) {
        match offset {
            // GPIO_OUT: RP2350 offsets (8-byte spacing)
            0x010 => self.gpio_out = val,
            0x018 => self.gpio_out |= val,    // GPIO_OUT_SET
            0x020 => self.gpio_out &= !val,   // GPIO_OUT_CLR
            0x028 => self.gpio_out ^= val,    // GPIO_OUT_XOR
            // GPIO_OE: RP2350 offsets (8-byte spacing)
            0x030 => self.gpio_oe = val,
            0x038 => self.gpio_oe |= val,     // GPIO_OE_SET
            0x040 => self.gpio_oe &= !val,    // GPIO_OE_CLR
            0x048 => self.gpio_oe ^= val,     // GPIO_OE_XOR
            _ => {}
        }
    }
}

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}
