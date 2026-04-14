//! RP2040 bus skeleton. Phase 3 ships just enough fields and methods to
//! let `Emulator::reset` compile and clear visible state. The AHB-Lite
//! fabric, address decode, CLOCKS/RESETS/SIO/PADS/IO/PLL register
//! storage, XIP_CTRL+SSI path, PIO blocks (2 vs RP2350's 3), and the
//! RP2040 SRAM bank / contention model are all Phase 5.

use mdpicoem_common::Memory;

/// RP2040 memory sizes.
///
/// - ROM: 16 KB (half of RP2350's 32 KB bootrom).
/// - SRAM: 264 KB (4×64 KB striped SRAM0-3 + 2×4 KB scratch SRAM4-5).
pub const ROM_SIZE: usize = 16 * 1024;
pub const SRAM_SIZE: usize = 264 * 1024;

/// RP2040 AHB-Lite bus fabric (skeleton).
///
/// Holds the chip's memory backing store plus the minimum bookkeeping
/// fields that `Emulator::reset` needs to clear. Everything else is
/// Phase 5.
pub struct Bus {
    pub memory: Memory,
    /// GPIO input pin state (post-mux). Cleared on reset.
    pub gpio_in: u32,
}

impl Bus {
    /// Construct a fresh RP2040 bus with RP2040-sized memory.
    pub fn new() -> Self {
        Self {
            memory: Memory::with_sizes(ROM_SIZE, SRAM_SIZE),
            gpio_in: 0,
        }
    }

    /// Direct memory read (bypasses bus timing). Delegates to the
    /// common `Memory` peek path — no decode of RP2040-specific
    /// addresses (peripherals, XIP_CTRL, etc.) here.
    pub fn peek32(&self, addr: u32) -> u32 {
        self.memory.peek32(addr)
    }

    /// Direct memory write (bypasses bus timing). Delegates to the
    /// common `Memory` poke path.
    pub fn poke32(&mut self, addr: u32, value: u32) {
        self.memory.poke32(addr, value);
    }

    /// Load XIP flash image. On RP2040 the chip has no onboard flash;
    /// this stores the image in `Memory.xip` for XIP access via
    /// `0x1500_0000` (Phase 5 will wire up the XIP_CTRL+SSI decode).
    pub fn load_flash(&mut self, data: &[u8]) {
        self.memory.load_flash(data);
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}
