//! RP2040 bus skeleton. Phase 3 ships just enough fields and methods to
//! let `Emulator::reset` compile and clear visible state. The AHB-Lite
//! fabric, address decode, CLOCKS/RESETS/SIO/PADS/IO/PLL register
//! storage, XIP_CTRL+SSI path, PIO blocks (2 vs RP2350's 3), and the
//! RP2040 SRAM bank / contention model are all Phase 5.

pub mod ppb;

use mdpicoem_common::Memory;
use ppb::Ppb;

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
    /// Per-core PPB state (VTOR, SHPR, ICSR, active-exception bitmap).
    /// Indexed by core id (0 or 1). Phase 4.B uses index 0 exclusively
    /// since `Emulator::step` only drives core 0; Phase 5 will wire up
    /// core 1 once SIO/NVIC dispatch lands.
    pub ppb: [Ppb; 2],
    /// Identifies which core is currently executing on the bus. Kept as
    /// a field so PPB-indexed helpers (`bus.ppb[bus.active_core()]`)
    /// mirror the mdrp2350 call pattern even though Phase 4.B only uses
    /// core 0.
    active_core: usize,
}

impl Bus {
    /// Construct a fresh RP2040 bus with RP2040-sized memory.
    pub fn new() -> Self {
        Self {
            memory: Memory::with_sizes(ROM_SIZE, SRAM_SIZE),
            gpio_in: 0,
            ppb: [Ppb::new(), Ppb::new()],
            active_core: 0,
        }
    }

    /// Currently-executing core (0 or 1).
    #[inline]
    pub fn active_core(&self) -> usize {
        self.active_core
    }

    /// Set the active core. Phase 4.B only drives core 0; Phase 5 will
    /// alternate between cores each quantum.
    #[inline]
    pub fn set_active_core(&mut self, core: usize) {
        debug_assert!(core < 2);
        self.active_core = core;
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

    // --- Phase 4.A bus read/write stubs ----------------------------------
    //
    // The AHB-Lite fabric, address decode, cycle accounting, bus-fault
    // reporting, contention model, and peripheral routing all land in
    // Phase 5. Phase 4.A only needs somewhere to send CPU load/store
    // traffic during unit tests — so these methods route straight to the
    // common `Memory` backing store with no timing and no decode beyond
    // what `Memory::peek*/poke*` already does.

    pub fn read8(&mut self, addr: u32) -> u8 {
        self.memory.peek8(addr)
    }

    pub fn read16(&mut self, addr: u32) -> u16 {
        let lo = self.memory.peek8(addr) as u16;
        let hi = self.memory.peek8(addr.wrapping_add(1)) as u16;
        lo | (hi << 8)
    }

    pub fn read32(&mut self, addr: u32) -> u32 {
        self.memory.peek32(addr)
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        self.memory.poke8(addr, val);
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        let bytes = val.to_le_bytes();
        self.memory.poke8(addr, bytes[0]);
        self.memory.poke8(addr.wrapping_add(1), bytes[1]);
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        self.memory.poke32(addr, val);
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}
