//! RP2040 AHB-Lite bus fabric.
//!
//! Phase 5.A: full address decode + peripheral routing for the registers
//! firmware actually touches (CLOCKS, RESETS, PLL_SYS, PLL_USB, XOSC, ROSC,
//! SIO, IO_BANK0, PADS_BANK0, XIP_CTRL stub, SSI stub). SRAM bank routing
//! uses the RP2040 4+2 layout from [`crate::memory::bank_for_address`];
//! bank contention is modelled simply (+1 cycle on SRAM access when
//! the companion core has already touched SRAM this quantum).
//!
//! Phase 5.B: PIO0 / PIO1 wired into the AHB decode at `0x5020_0000` and
//! `0x5030_0000`. Register access goes through `PioBlock::read32` /
//! `write32` (mirrors `mdrp2350::Bus`). Sub-word writes to PIO ranges are
//! ignored — several PIO registers have side-effects on read (RXF pop) or
//! write (TXF push, CTRL bit flags) that would behave incorrectly under a
//! synthetic read-modify-write. Sub-word reads still go through `read32`
//! (matching mdrp2350) and so observe those same side-effects on the
//! enclosing word — firmware that only touches PIO with word-sized
//! accesses (the supported path in the datasheet) is unaffected.

pub mod clocks;
pub mod io_bank0;
pub mod pads_bank0;
pub mod ppb;
pub mod resets;
pub mod sio;

use std::collections::HashMap;

use mdpicoem_common::PioBlock;

use crate::memory::{FLASH_SIZE, Memory, ROM_SIZE, SRAM_SIZE, bank_for_address};
use clocks::{ClockTree, ClocksRegs, PLL_RESET, PllRegs, ROSC_FREQ_HZ, RoscRegs, XoscRegs};
use io_bank0::IoBank0;
use pads_bank0::PadsBank0;
use ppb::Ppb;
use resets::Resets;
use sio::Sio;

/// Peripheral base addresses (see RP2040 datasheet §2.2).
pub const APB_BASE: u32 = 0x4000_0000;
pub const SIO_BASE: u32 = 0xD000_0000;

// APB peripheral base addresses (RP2040 datasheet §2.2).
pub const SYSINFO_BASE: u32 = 0x4000_0000;
pub const SYSCFG_BASE: u32 = 0x4000_4000;
pub const CLOCKS_BASE: u32 = 0x4000_8000;
pub const RESETS_BASE: u32 = 0x4000_C000;
pub const PSM_BASE: u32 = 0x4001_0000;
pub const IO_BANK0_BASE: u32 = 0x4001_4000;
pub const IO_QSPI_BASE: u32 = 0x4001_8000;
pub const PADS_BANK0_BASE: u32 = 0x4001_C000;
pub const PADS_QSPI_BASE: u32 = 0x4002_0000;
pub const XOSC_BASE: u32 = 0x4002_4000;
pub const PLL_SYS_BASE: u32 = 0x4002_8000;
pub const PLL_USB_BASE: u32 = 0x4002_C000;
pub const BUSCTRL_BASE: u32 = 0x4003_0000;
pub const ROSC_BASE: u32 = 0x4006_0000;
pub const XIP_CTRL_BASE: u32 = 0x1400_0000;
pub const SSI_BASE: u32 = 0x1800_0000;
pub const XIP_SRAM_BASE: u32 = 0x1500_0000;
pub const XIP_SRAM_END: u32 = 0x1500_4000; // 16 KB
/// XIP flash window base. Aliases at `+0x0100_0000`, `+0x0200_0000`,
/// `+0x0300_0000` mirror the same 2 MB flash buffer.
pub const XIP_FLASH_BASE: u32 = 0x1000_0000;

/// Returns `Some(offset)` if `addr` falls inside one of the four 2 MB
/// XIP flash alias windows (`0x10`, `0x11`, `0x12`, `0x13` at bits
/// [27:24]). The returned offset is the byte offset into the flash
/// buffer (in `0..FLASH_SIZE`).
#[inline]
pub(crate) fn xip_flash_offset(addr: u32) -> Option<u32> {
    // Region selector (bits [31:28]) must be 0x1 for XIP. Alias bits
    // [27:24] (values 0..3 for the four 2 MB aliases) are validated
    // below.
    if (addr & 0xF000_0000) != XIP_FLASH_BASE {
        return None;
    }
    // Alias select bits [27:24]: 0x0, 0x1, 0x2, 0x3.
    let alias = (addr >> 24) & 0xF;
    if alias > 3 {
        return None;
    }
    // Offset inside the 2 MB alias window.
    let offset = addr & 0x00FF_FFFF;
    if (offset as usize) < FLASH_SIZE {
        Some(offset)
    } else {
        None
    }
}

// PIO AHB windows (RP2040 datasheet §3 — two PIO blocks).
pub const PIO0_BASE: u32 = 0x5020_0000;
pub const PIO1_BASE: u32 = 0x5030_0000;

/// XIP SRAM size (16 KB on RP2040 — the cache RAM exposed as scratch).
pub const XIP_SRAM_SIZE: usize = 16 * 1024;

/// RP2040 AHB-Lite bus fabric.
pub struct Bus {
    pub memory: Memory,
    /// GPIO input state after merging SIO output with PIO outputs
    /// (Phase 5.A: SIO only). Read by firmware via SIO_GPIO_IN.
    pub gpio_in: u32,
    /// Per-core PPB (VTOR, SHPR, ICSR, active bitmap).
    pub ppb: [Ppb; 2],
    /// Single-cycle IO block.
    pub sio: Sio,
    /// RESETS peripheral.
    pub resets: Resets,
    /// CLOCKS register storage.
    pub clocks_regs: ClocksRegs,
    /// XOSC register storage.
    pub xosc_regs: XoscRegs,
    /// ROSC register storage.
    pub rosc_regs: RoscRegs,
    /// PLL_SYS register image (`[CS, PWR, FBDIV_INT, PRIM]`).
    pub pll_sys_regs: PllRegs,
    /// PLL_USB register image.
    pub pll_usb_regs: PllRegs,
    /// Derived clock tree frequencies (recomputed on any CLOCKS/PLL write).
    pub clock_tree: ClockTree,
    /// IO_BANK0 per-pin function select.
    pub io_bank0: IoBank0,
    /// PADS_BANK0 per-pin pad control.
    pub pads_bank0: PadsBank0,
    /// XIP SRAM (16 KB cache memory usable as SRAM, 0x1500_0000..0x1500_4000).
    xip_sram: Box<[u8; XIP_SRAM_SIZE]>,
    /// XIP_CTRL register backing store (stub — firmware round-trips only).
    xip_ctrl_regs: HashMap<u32, u32>,
    /// SSI register backing store (stub).
    ssi_regs: HashMap<u32, u32>,
    /// Catch-all APB peripheral register backing store for blocks we
    /// don't model in detail (PSM / BUSCTRL / SYSCFG / IO_QSPI / PADS_QSPI /
    /// UART / SPI / I2C / ADC / PWM / TIMER / WATCHDOG / RTC / VREG / TBMAN).
    /// Keyed by canonical word address (alias bits stripped).
    peripheral_regs: HashMap<u32, u32>,
    /// PIO0 / PIO1. Wired into the AHB decode at `0x5020_0000` /
    /// `0x5030_0000` (see [`PIO0_BASE`] / [`PIO1_BASE`]); output pins are
    /// merged into [`Self::gpio_in`] by [`crate::Emulator::update_gpio`].
    pub pio: [PioBlock; 2],
    /// Per-core event flag for WFE/SEV / FIFO event protocol.
    pub event_flag: [bool; 2],
    /// Which core is currently executing on the bus.
    active_core: usize,
    /// Cycle cost of the most recent bus access.
    last_access_cycles: u32,
    /// Bus fault sticky flags.
    bus_fault: bool,
    bus_fault_addr: u32,
    /// Per-quantum bank-touched bitmap — bit N = bank N was accessed by
    /// the core 0 step. Read by core 1 to compute +1 cycle contention.
    core0_bank_touched: u8,
    /// True while the currently-running core is core 1 and it is looking
    /// up contention against `core0_bank_touched`. Set/cleared by the
    /// dual-core scheduler via `begin_core1_step` / `end_core1_step`.
    contention_check_active: bool,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Memory::with_flash(ROM_SIZE, SRAM_SIZE, FLASH_SIZE),
            gpio_in: 0,
            ppb: [Ppb::new(), Ppb::new()],
            sio: Sio::new(),
            resets: Resets::new(),
            clocks_regs: ClocksRegs::new(),
            xosc_regs: XoscRegs::new(),
            rosc_regs: RoscRegs::new(),
            pll_sys_regs: PLL_RESET,
            pll_usb_regs: PLL_RESET,
            clock_tree: ClockTree::default(),
            io_bank0: IoBank0::new(),
            pads_bank0: PadsBank0::new(),
            xip_sram: Box::new([0u8; XIP_SRAM_SIZE]),
            xip_ctrl_regs: HashMap::new(),
            ssi_regs: HashMap::new(),
            peripheral_regs: HashMap::new(),
            pio: [PioBlock::new(), PioBlock::new()],
            event_flag: [false; 2],
            active_core: 0,
            last_access_cycles: 0,
            bus_fault: false,
            bus_fault_addr: 0,
            core0_bank_touched: 0,
            contention_check_active: false,
        }
    }

    // --- Active-core / scheduler plumbing ---------------------------------

    #[inline]
    pub fn active_core(&self) -> usize {
        self.active_core
    }

    #[inline]
    pub fn set_active_core(&mut self, core: usize) {
        debug_assert!(core < 2);
        self.active_core = core;
    }

    /// Called before core 1 steps each quantum — enables the contention
    /// check that adds +1 cycle when core 1 touches an SRAM bank already
    /// touched by core 0.
    #[inline]
    pub fn begin_core1_step(&mut self) {
        self.contention_check_active = true;
    }

    /// Called after core 1 has finished its slice. Clears the
    /// contention window and wipes the core-0 bank map for the next
    /// quantum.
    #[inline]
    pub fn end_core1_step(&mut self) {
        self.contention_check_active = false;
        self.core0_bank_touched = 0;
    }

    // --- Clock-tree accessors --------------------------------------------

    #[inline]
    pub fn sys_clk_hz(&self) -> u32 {
        self.clock_tree.sys_clk_hz
    }

    #[inline]
    pub fn ref_clk_hz(&self) -> u32 {
        self.clock_tree.ref_clk_hz
    }

    /// Seed the derived clock tree with an initial frequency. First
    /// write to CLOCKS / PLL replaces the seed with the derived value.
    pub fn seed_sys_clk_hz(&mut self, hz: u32) {
        self.clock_tree.sys_clk_hz = hz;
        self.clock_tree.ref_clk_hz = hz;
    }

    fn recompute_clock_tree(&mut self) {
        clocks::recompute(
            &self.clocks_regs,
            &self.pll_sys_regs,
            &self.pll_usb_regs,
            &mut self.clock_tree,
        );
    }

    // --- Flash / XIP management ------------------------------------------

    /// Copy `data` into the 2 MB XIP flash window at offset 0. Oversized
    /// images are clamped by [`Memory::load_flash`]; the mapped window
    /// is always 2 MB so reads past the image length return 0.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.memory.load_flash(data);
    }

    // --- Bus-fault plumbing -----------------------------------------------

    pub fn bus_fault(&self) -> bool {
        self.bus_fault
    }

    pub fn bus_fault_addr(&self) -> u32 {
        self.bus_fault_addr
    }

    pub fn clear_bus_fault(&mut self) {
        self.bus_fault = false;
    }

    // --- Direct peek/poke (bypasses decode, still routes through regions)

    pub fn peek32(&self, addr: u32) -> u32 {
        if (addr >> 28) == 0x2 {
            // SRAM
            self.memory.sram_read32(addr & 0x00FF_FFFF)
        } else if addr >= XIP_SRAM_BASE && addr < XIP_SRAM_END {
            let off = (addr - XIP_SRAM_BASE) as usize;
            u32::from_le_bytes([
                self.xip_sram[off],
                self.xip_sram[off + 1],
                self.xip_sram[off + 2],
                self.xip_sram[off + 3],
            ])
        } else {
            self.memory.peek32(addr)
        }
    }

    pub fn poke32(&mut self, addr: u32, value: u32) {
        if (addr >> 28) == 0x2 {
            self.memory.sram_write32(addr & 0x00FF_FFFF, value);
        } else if addr >= XIP_SRAM_BASE && addr < XIP_SRAM_END {
            let off = (addr - XIP_SRAM_BASE) as usize;
            let bytes = value.to_le_bytes();
            self.xip_sram[off..off + 4].copy_from_slice(&bytes);
        } else {
            self.memory.poke32(addr, value);
        }
    }

    // --- Raw ROM loader (used by `Emulator::load_bootrom`) ----------------

    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.memory.load_rom(data);
    }

    // --- Latency helpers --------------------------------------------------

    #[inline]
    pub fn last_access_cycles(&self) -> u32 {
        self.last_access_cycles
    }

    /// Base read latency for an address region (cycles).
    #[inline]
    fn read_latency(region: u32) -> u32 {
        match region {
            0x0 => 1, // ROM
            0x1 => 1, // XIP / XIP_CTRL / SSI
            0x2 => 1, // SRAM
            0x4 => 3, // APB peripherals
            0x5 => 1, // AHB peripherals
            0xD => 1, // SIO
            0xE => 1, // PPB
            _ => 1,
        }
    }

    #[inline]
    fn write_latency(region: u32) -> u32 {
        match region {
            0x4 => 4, // APB writes
            _ => 1,
        }
    }

    /// Record an SRAM bank touch for the active core and return any
    /// contention wait states (simple +1 model).
    #[inline]
    fn note_sram_access(&mut self, addr: u32) -> u32 {
        if let Some(bank) = bank_for_address(addr) {
            let bit = 1u8 << (bank & 7);
            let wait = if self.contention_check_active && self.core0_bank_touched & bit != 0 {
                1
            } else {
                0
            };
            if self.active_core == 0 {
                self.core0_bank_touched |= bit;
            }
            wait
        } else {
            0
        }
    }

    // --- XIP SRAM scratch helpers ----------------------------------------

    fn xip_sram_read(&self, addr: u32, width: usize) -> u32 {
        let off = (addr - XIP_SRAM_BASE) as usize;
        let end = off + width;
        if end <= self.xip_sram.len() {
            match width {
                1 => self.xip_sram[off] as u32,
                2 => u16::from_le_bytes([self.xip_sram[off], self.xip_sram[off + 1]]) as u32,
                4 => u32::from_le_bytes([
                    self.xip_sram[off],
                    self.xip_sram[off + 1],
                    self.xip_sram[off + 2],
                    self.xip_sram[off + 3],
                ]),
                _ => 0,
            }
        } else {
            0
        }
    }

    fn xip_sram_write(&mut self, addr: u32, val: u32, width: usize) {
        let off = (addr - XIP_SRAM_BASE) as usize;
        let end = off + width;
        if end <= self.xip_sram.len() {
            let bytes = val.to_le_bytes();
            for i in 0..width {
                self.xip_sram[off + i] = bytes[i];
            }
        }
    }

    // --- Peripheral read dispatch ----------------------------------------

    fn peripheral_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        match base {
            SYSINFO_BASE => self.sysinfo_read(offset),
            CLOCKS_BASE => self.clocks_regs.read32(offset),
            RESETS_BASE => self.resets.read32(offset),
            XOSC_BASE => self.xosc_regs.read32(offset),
            PLL_SYS_BASE => clocks::pll_read(&self.pll_sys_regs, offset),
            PLL_USB_BASE => clocks::pll_read(&self.pll_usb_regs, offset),
            ROSC_BASE => self.rosc_regs.read32(offset),
            IO_BANK0_BASE => self.io_bank0.read32(offset),
            PADS_BANK0_BASE => self.pads_bank0.read32(offset),
            PIO0_BASE => self.pio[0].read32(offset),
            PIO1_BASE => self.pio[1].read32(offset),
            _ => *self.peripheral_regs.get(&canonical).unwrap_or(&0),
        }
    }

    fn peripheral_write32(&mut self, addr: u32, val: u32, alias: u32) {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        match base {
            SYSINFO_BASE => {} // read-only
            CLOCKS_BASE => {
                if self.clocks_regs.write32(offset, val, alias) {
                    self.recompute_clock_tree();
                }
            }
            RESETS_BASE => self.resets.write32(offset, val, alias),
            XOSC_BASE => self.xosc_regs.write32(offset, val, alias),
            PLL_SYS_BASE => {
                if clocks::pll_write(&mut self.pll_sys_regs, offset, val, alias) {
                    self.recompute_clock_tree();
                }
            }
            PLL_USB_BASE => {
                if clocks::pll_write(&mut self.pll_usb_regs, offset, val, alias) {
                    self.recompute_clock_tree();
                }
            }
            ROSC_BASE => self.rosc_regs.write32(offset, val, alias),
            IO_BANK0_BASE => self.io_bank0.write32(offset, val, alias),
            PADS_BANK0_BASE => self.pads_bank0.write32(offset, val, alias),
            PIO0_BASE => self.pio[0].write32(offset, val, alias),
            PIO1_BASE => self.pio[1].write32(offset, val, alias),
            _ => {
                // Catch-all: store with alias semantics so firmware round-trips.
                let old = *self.peripheral_regs.get(&canonical).unwrap_or(&0);
                let new = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => val,
                };
                self.peripheral_regs.insert(canonical, new);
            }
        }
    }

    fn sysinfo_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => 0x0000_0001, // CHIP_ID: RP2040 manufacturer (placeholder)
            0x004 => 0x0000_0000, // PLATFORM
            _ => 0,
        }
    }

    // --- XIP_CTRL + SSI stubs --------------------------------------------

    fn xip_ctrl_read(&self, offset: u32) -> u32 {
        // XIP_CTRL_CTRL (offset 0x00) reports EN=1 so the bootrom's check
        // for "XIP cache enabled" succeeds immediately.
        match offset {
            0x00 => *self.xip_ctrl_regs.get(&0).unwrap_or(&1),
            _ => *self.xip_ctrl_regs.get(&offset).unwrap_or(&0),
        }
    }

    fn xip_ctrl_write(&mut self, offset: u32, val: u32) {
        self.xip_ctrl_regs.insert(offset, val);
    }

    fn ssi_read(&self, offset: u32) -> u32 {
        // SSI_SR (offset 0x28) is polled frequently; report TFE|BF set so
        // firmware transmit-wait loops terminate. Other regs return 0.
        match offset {
            0x28 => 0x04 | 0x01, // TFE | BUSY-cleared
            _ => *self.ssi_regs.get(&offset).unwrap_or(&0),
        }
    }

    fn ssi_write(&mut self, offset: u32, val: u32) {
        self.ssi_regs.insert(offset, val);
    }

    // ======================================================================
    // Read / write entry points
    // ======================================================================

    pub fn read8(&mut self, addr: u32) -> u8 {
        let region = addr >> 28;
        self.last_access_cycles = Self::read_latency(region);
        match region {
            0x0 if (addr & 0x0FFF_FFFF) < ROM_SIZE as u32 => {
                self.memory.rom_read8(addr & 0x0FFF_FFFF)
            }
            0x1 => self.region1_read(addr, 1) as u8,
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if (off as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    self.memory.sram_read8(off)
                } else {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                    0
                }
            }
            0x4 | 0x5 => {
                let word = self.peripheral_read32(addr & !3);
                word.to_le_bytes()[(addr & 3) as usize]
            }
            0xD => {
                let word = self.sio_read32(addr & !3);
                word.to_le_bytes()[(addr & 3) as usize]
            }
            0xE => {
                let word = self.ppb[self.active_core].read32(addr & !3);
                word.to_le_bytes()[(addr & 3) as usize]
            }
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
                0
            }
        }
    }

    pub fn read16(&mut self, addr: u32) -> u16 {
        let region = addr >> 28;
        self.last_access_cycles = Self::read_latency(region);
        match region {
            0x0 if (addr & 0x0FFF_FFFF) + 1 < ROM_SIZE as u32 => {
                self.memory.rom_read16(addr & 0x0FFF_FFFF)
            }
            0x1 => self.region1_read(addr, 2) as u16,
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 1) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    self.memory.sram_read16(off)
                } else {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                    0
                }
            }
            0x4 | 0x5 => {
                let word = self.peripheral_read32(addr & !3);
                let half = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half]
            }
            0xD => {
                let word = self.sio_read32(addr & !3);
                let half = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half]
            }
            0xE => {
                let word = self.ppb[self.active_core].read32(addr & !3);
                let half = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half]
            }
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
                0
            }
        }
    }

    pub fn read32(&mut self, addr: u32) -> u32 {
        let region = addr >> 28;
        self.last_access_cycles = Self::read_latency(region);
        match region {
            0x0 if (addr & 0x0FFF_FFFF) + 3 < ROM_SIZE as u32 => {
                self.memory.rom_read32(addr & 0x0FFF_FFFF)
            }
            0x1 => self.region1_read(addr, 4),
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 3) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    self.memory.sram_read32(off)
                } else {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                    0
                }
            }
            0x4 | 0x5 => self.peripheral_read32(addr),
            0xD => self.sio_read32(addr),
            0xE => self.ppb[self.active_core].read32(addr),
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
                0
            }
        }
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        let region = addr >> 28;
        self.last_access_cycles = Self::write_latency(region);
        match region {
            0x1 if addr >= XIP_SRAM_BASE && addr < XIP_SRAM_END => {
                self.xip_sram_write(addr, val as u32, 1);
            }
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if (off as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    // All four SRAM alias windows (0x20/0x21/0x22/0x23)
                    // map to the same backing storage — RP2040 datasheet
                    // §2.1.2 calls out alias bits [25:24] as bank-striping
                    // flavours for DMA, not peripheral XOR/SET/CLR.
                    self.memory.sram_write8(off, val);
                } else {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                }
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                if base == PIO0_BASE || base == PIO1_BASE {
                    // PIO is 32-bit access only (matches mdrp2350) — byte
                    // writes would trigger spurious RXF pops via the RMW
                    // read. Silently ignore.
                    return;
                }
                let alias = (addr >> 12) & 3;
                // Byte-level RMW into the word, preserving alias semantics.
                let word_addr = canonical & !3;
                let byte_idx = (canonical & 3) as usize;
                let old = self.peripheral_read32(word_addr);
                let mut bytes = old.to_le_bytes();
                bytes[byte_idx] = val;
                let new_word = u32::from_le_bytes(bytes);
                // For an alias access, convert the byte to a positioned
                // word and defer alias math to the peripheral layer.
                if alias == 0 {
                    self.peripheral_write32(word_addr, new_word, 0);
                } else {
                    let shifted = (val as u32) << (byte_idx * 8);
                    self.peripheral_write32(word_addr, shifted, alias);
                }
            }
            0xD => {
                let word_addr = addr & !3;
                let byte_idx = (addr & 3) as usize;
                let old = self.sio_read32(word_addr);
                let mut bytes = old.to_le_bytes();
                bytes[byte_idx] = val;
                self.sio_write32(word_addr, u32::from_le_bytes(bytes));
            }
            0xE => {
                let word_addr = addr & !3;
                let byte_idx = (addr & 3) as usize;
                let old = self.ppb[self.active_core].read32(word_addr);
                let mut bytes = old.to_le_bytes();
                bytes[byte_idx] = val;
                self.ppb[self.active_core].write32(word_addr, u32::from_le_bytes(bytes));
            }
            0x0 | 0x1 => {} // ROM / XIP flash — silently ignored at any width
            _ => {
                // Unmapped at any width sets the sticky bus-fault flag so
                // step() can escalate to HardFault.
                self.bus_fault = true;
                self.bus_fault_addr = addr;
            }
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        let region = addr >> 28;
        self.last_access_cycles = Self::write_latency(region);
        match region {
            0x1 if addr >= XIP_SRAM_BASE && addr < XIP_SRAM_END => {
                self.xip_sram_write(addr, val as u32, 2);
            }
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 1) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    // All four SRAM alias windows map to the same storage
                    // (RP2040 datasheet §2.1.2).
                    self.memory.sram_write16(off, val);
                } else {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                }
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                if base == PIO0_BASE || base == PIO1_BASE {
                    // PIO is 32-bit access only (matches mdrp2350).
                    return;
                }
                let alias = (addr >> 12) & 3;
                let word_addr = canonical & !3;
                let half_idx = ((canonical >> 1) & 1) as usize;
                let old = self.peripheral_read32(word_addr);
                let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                halves[half_idx] = val;
                let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                if alias == 0 {
                    self.peripheral_write32(word_addr, new_word, 0);
                } else {
                    let shifted = (val as u32) << (half_idx * 16);
                    self.peripheral_write32(word_addr, shifted, alias);
                }
            }
            0xD => {
                let word_addr = addr & !3;
                let half_idx = ((addr >> 1) & 1) as usize;
                let old = self.sio_read32(word_addr);
                let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                halves[half_idx] = val;
                let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                self.sio_write32(word_addr, new_word);
            }
            0xE => {
                let word_addr = addr & !3;
                let half_idx = ((addr >> 1) & 1) as usize;
                let old = self.ppb[self.active_core].read32(word_addr);
                let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                halves[half_idx] = val;
                let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                self.ppb[self.active_core].write32(word_addr, new_word);
            }
            0x0 | 0x1 => {} // ROM / XIP flash — silently ignored at any width
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
            }
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        let region = addr >> 28;
        self.last_access_cycles = Self::write_latency(region);
        match region {
            0x1 if addr >= XIP_SRAM_BASE && addr < XIP_SRAM_END => {
                self.xip_sram_write(addr, val, 4);
            }
            0x1 => {
                // Region 0x1 at XIP_CTRL (0x1400_0000) or SSI (0x1800_0000).
                let base = addr & 0xFFFF_F000;
                let offset = addr & 0x0FFF;
                if base == XIP_CTRL_BASE {
                    self.xip_ctrl_write(offset, val);
                } else if base == SSI_BASE {
                    self.ssi_write(offset, val);
                }
            }
            0x2 => {
                let off = addr & 0x00FF_FFFF;
                if ((off + 3) as usize) < SRAM_SIZE {
                    self.last_access_cycles += self.note_sram_access(addr);
                    // All four SRAM alias windows map to the same storage
                    // (RP2040 datasheet §2.1.2).
                    self.memory.sram_write32(off, val);
                } else {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                }
            }
            0x4 | 0x5 => {
                let alias = (addr >> 12) & 3;
                self.peripheral_write32(addr, val, alias);
            }
            0xD => self.sio_write32(addr, val),
            0xE => self.ppb[self.active_core].write32(addr, val),
            0x0 => {} // ROM — silently ignored at any width
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
            }
        }
    }

    // --- Region 0x1 read dispatch (XIP flash / XIP SRAM / XIP_CTRL / SSI)

    fn region1_read(&mut self, addr: u32, width: usize) -> u32 {
        if addr >= XIP_SRAM_BASE && addr < XIP_SRAM_END {
            return self.xip_sram_read(addr, width);
        }
        let base = addr & 0xFFFF_F000;
        let offset = addr & 0x0FFF;
        if base == XIP_CTRL_BASE {
            return self.xip_ctrl_read(offset);
        }
        if base == SSI_BASE {
            return self.ssi_read(offset);
        }
        // XIP flash window (0x10/0x11/0x12/0x13, each a 2 MB mirror).
        // PicoGUS Integration HLD (Stage 1): flash is a plain mapped
        // window — no wait states, no cache, no fault before load.
        if let Some(flash_off) = xip_flash_offset(addr) {
            return match width {
                1 => self.memory.xip_read8(flash_off) as u32,
                2 => self.memory.xip_read16(flash_off) as u32,
                4 => self.memory.xip_read32(flash_off),
                _ => 0,
            };
        }
        0
    }

    // --- SIO read/write dispatch -----------------------------------------
    //
    // GPIO_IN (0x004) is owned by Bus so the SIO crate has no direct
    // dependency on PIO (Phase 5.B lifts this out). All other offsets
    // delegate to `Sio`.

    fn sio_read32(&mut self, addr: u32) -> u32 {
        let offset = addr & 0xFFF;
        match offset {
            0x004 => self.gpio_in,
            _ => {
                let core = self.active_core;
                self.sio.read32(offset, core)
            }
        }
    }

    fn sio_write32(&mut self, addr: u32, val: u32) {
        let offset = addr & 0xFFF;
        let core = self.active_core;
        self.sio.write32(offset, val, core);
        if let Some(receiver) = self.sio.pending_fifo_event.take() {
            self.event_flag[receiver] = true;
        }
    }

    // --- Back-compat accessors for Phase 3 / 4 tests ---------------------
    //
    // The previous Phase 3 stub exposed a `gpio_in` field; keep that
    // interface stable so tests don't need updating.
    #[inline]
    pub fn gpio_in(&self) -> u32 {
        self.gpio_in
    }

    /// Signal SEV to both cores.
    pub fn signal_sev(&mut self) {
        self.event_flag[0] = true;
        self.event_flag[1] = true;
    }

    /// ROSC nominal frequency re-export.
    pub const ROSC_FREQ_HZ_CONST: u32 = ROSC_FREQ_HZ;
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_bus_all_peripherals_in_reset() {
        let bus = Bus::new();
        assert_eq!(bus.resets.state, resets::RESET_MASK);
    }

    #[test]
    fn sram_write_read_roundtrip() {
        let mut bus = Bus::new();
        bus.write32(0x2000_0100, 0xDEAD_BEEF);
        assert_eq!(bus.read32(0x2000_0100), 0xDEAD_BEEF);
    }

    #[test]
    fn sram_aliases_mirror_same_storage() {
        // RP2040 datasheet §2.1.2: all four SRAM alias windows
        // (0x20/0x21/0x22/0x23) address the same backing bytes. Aliases
        // are bank-striping flavours for DMA, not peripheral XOR/SET/CLR.
        let mut bus = Bus::new();
        bus.write32(0x2000_0100, 0xF0F0_F0F0);
        // A write via 0x21xxxxxx overwrites the same bytes, not XORs.
        bus.write32(0x2100_0100, 0x0F0F_0F0F);
        assert_eq!(bus.read32(0x2000_0100), 0x0F0F_0F0F);
        // Reads through every alias observe the identical word.
        assert_eq!(bus.read32(0x2100_0100), 0x0F0F_0F0F);
        assert_eq!(bus.read32(0x2200_0100), 0x0F0F_0F0F);
        assert_eq!(bus.read32(0x2300_0100), 0x0F0F_0F0F);
        // Writing through 0x22 / 0x23 also just overwrites.
        bus.write32(0x2200_0200, 0xAAAA_AAAA);
        bus.write32(0x2300_0200, 0x5555_5555);
        assert_eq!(bus.read32(0x2000_0200), 0x5555_5555);
    }

    #[test]
    fn resets_clr_deasserts() {
        let mut bus = Bus::new();
        // CLR alias at RESETS base 0x4000_C000 + alias 3 → offset 0x3000.
        bus.write32(0x4000_F000, 0x0000_0001);
        assert_eq!(bus.read32(0x4000_C000) & 1, 0);
        assert_eq!(bus.read32(0x4000_C008) & 1, 1);
    }

    #[test]
    fn clocks_ref_mux_switch_to_xosc() {
        let mut bus = Bus::new();
        // CLK_REF_CTRL at 0x4000_8030, write SRC=2 (XOSC).
        bus.write32(0x4000_8030, 2);
        assert_eq!(bus.clock_tree.ref_clk_hz, clocks::XOSC_FREQ_HZ);
    }

    #[test]
    fn clocks_sys_div_write_at_0x40_recomputes_tree() {
        // RP2040 datasheet §2.15.7: CLK_SYS_DIV is at CLOCKS_BASE + 0x40.
        // A write to 0x4000_8040 must land on `clk_sys_div` and feed
        // through `recompute_clock_tree()` — confirming the constants
        // aren't swapped with CLK_SYS_SELECTED (0x44, mux indicator).
        let mut bus = Bus::new();
        bus.seed_sys_clk_hz(clocks::ROSC_FREQ_HZ);
        // DIV integer field lives in bits [31:16]; write /4.
        bus.write32(0x4000_8040, 4 << 16);
        assert_eq!(bus.clocks_regs.clk_sys_div, 4 << 16);
        assert_eq!(bus.clock_tree.sys_clk_hz, clocks::ROSC_FREQ_HZ / 4);
    }

    #[test]
    fn pll_lock_bit_forced_high() {
        let mut bus = Bus::new();
        let cs = bus.read32(PLL_SYS_BASE);
        assert_ne!(cs & (1 << 31), 0);
    }

    #[test]
    fn sio_gpio_out_roundtrip() {
        let mut bus = Bus::new();
        bus.write32(SIO_BASE + 0x010, 0x5A);
        assert_eq!(bus.read32(SIO_BASE + 0x010), 0x5A);
    }

    #[test]
    fn sio_cpuid_reflects_active_core() {
        let mut bus = Bus::new();
        bus.set_active_core(1);
        assert_eq!(bus.read32(SIO_BASE), 1);
        bus.set_active_core(0);
        assert_eq!(bus.read32(SIO_BASE), 0);
    }

    #[test]
    fn gpio_in_is_owned_by_bus() {
        let mut bus = Bus::new();
        bus.gpio_in = 0x42;
        assert_eq!(bus.read32(SIO_BASE + 0x004), 0x42);
    }

    #[test]
    fn xip_fresh_bus_reads_zero_without_fault() {
        // PicoGUS integration (Stage 1 HLD): flash is a plain mapped
        // window. Reads before `load_flash` must return 0 without
        // setting bus_fault so a firmware that probes XIP during boot
        // doesn't take a spurious HardFault.
        let mut bus = Bus::new();
        assert_eq!(bus.read32(0x1000_0000), 0);
        assert_eq!(bus.read8(0x1000_0001), 0);
        assert_eq!(bus.read16(0x1000_0002), 0);
        assert!(!bus.bus_fault());
    }

    #[test]
    fn xip_read_after_flash_load() {
        let mut bus = Bus::new();
        bus.load_flash(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(bus.read32(0x1000_0000), 0xDDCCBBAA);
        assert_eq!(bus.read8(0x1000_0000), 0xAA);
        assert_eq!(bus.read8(0x1000_0003), 0xDD);
        assert_eq!(bus.read16(0x1000_0002), 0xDDCC);
    }

    #[test]
    fn xip_aliases_mirror_flash_base() {
        // RP2040 XIP has three read-only aliases at 0x11/0x12/0x13 that
        // map to the same 2 MB flash window. All four addresses must
        // observe identical bytes after `load_flash`.
        let mut bus = Bus::new();
        bus.load_flash(&[0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE]);
        let words_at = |bus: &mut Bus, base: u32| {
            (
                bus.read32(base),
                bus.read32(base + 4),
                bus.read8(base + 1),
                bus.read16(base + 6),
            )
        };
        let canonical = words_at(&mut bus, 0x1000_0000);
        assert_eq!(words_at(&mut bus, 0x1100_0000), canonical);
        assert_eq!(words_at(&mut bus, 0x1200_0000), canonical);
        assert_eq!(words_at(&mut bus, 0x1300_0000), canonical);
        assert_eq!(canonical.0, 0xEFBEADDE);
    }

    #[test]
    fn xip_read_past_loaded_length_returns_zero() {
        // Within the mapped 2 MB window, addresses past the loaded
        // image length must read 0 (pre-allocated zero bytes in the
        // backing buffer). No bus fault.
        let mut bus = Bus::new();
        bus.load_flash(&[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(bus.read32(0x1000_0004), 0);
        assert_eq!(bus.read32(0x1010_0000), 0); // 1 MB in
        assert_eq!(bus.read32(0x101F_FFFC), 0); // last word of window
        assert_eq!(bus.read32(0x1110_0000), 0); // alias 0x11, mid-window
        assert!(!bus.bus_fault());
    }

    #[test]
    fn xip_writes_silently_ignored_at_every_width() {
        // Real flash needs erase/program via XIP_SSI; at the AHB layer
        // writes to the flash window must not fault and must not alter
        // the loaded bytes.
        let mut bus = Bus::new();
        bus.load_flash(&[0x55, 0x66, 0x77, 0x88]);
        bus.write8(0x1000_0000, 0xAA);
        bus.write16(0x1000_0002, 0xBBBB);
        bus.write32(0x1000_0000, 0xDEAD_BEEF);
        // Aliases must also swallow writes.
        bus.write8(0x1100_0000, 0xAA);
        bus.write16(0x1200_0002, 0xBBBB);
        bus.write32(0x1300_0000, 0xDEAD_BEEF);
        assert!(!bus.bus_fault(), "flash writes must not raise bus_fault");
        assert_eq!(bus.read32(0x1000_0000), 0x88776655);
        assert_eq!(bus.read32(0x1100_0000), 0x88776655);
    }

    #[test]
    fn xip_sram_scratch() {
        let mut bus = Bus::new();
        bus.write32(XIP_SRAM_BASE, 0xCAFE_BABE);
        assert_eq!(bus.read32(XIP_SRAM_BASE), 0xCAFE_BABE);
    }

    #[test]
    fn xip_ctrl_roundtrip() {
        let mut bus = Bus::new();
        bus.write32(XIP_CTRL_BASE + 0x4, 0x1234);
        assert_eq!(bus.read32(XIP_CTRL_BASE + 0x4), 0x1234);
    }

    #[test]
    fn unmapped_region_faults() {
        let mut bus = Bus::new();
        bus.read32(0x7000_0000);
        assert!(bus.bus_fault());
    }

    #[test]
    fn unmapped_writes_fault_at_every_width() {
        // Consistent policy (see write8/write16/write32): unmapped writes
        // at any width set the sticky bus-fault flag.
        let mut bus = Bus::new();
        bus.write8(0x7000_0000, 0xAA);
        assert!(bus.bus_fault(), "write8 to unmapped region must fault");
        bus.clear_bus_fault();

        bus.write16(0x7000_0000, 0xAABB);
        assert!(bus.bus_fault(), "write16 to unmapped region must fault");
        bus.clear_bus_fault();

        bus.write32(0x7000_0000, 0xAABB_CCDD);
        assert!(bus.bus_fault(), "write32 to unmapped region must fault");
    }

    #[test]
    fn rom_writes_silently_ignored_at_every_width() {
        // ROM is read-only — writes at any width must NOT raise bus_fault.
        let mut bus = Bus::new();
        bus.write8(0x0000_0100, 0xAA);
        assert!(!bus.bus_fault(), "write8 to ROM is silent");
        bus.write16(0x0000_0100, 0xAABB);
        assert!(!bus.bus_fault(), "write16 to ROM is silent");
        bus.write32(0x0000_0100, 0xAABB_CCDD);
        assert!(!bus.bus_fault(), "write32 to ROM is silent");
    }

    #[test]
    fn sram_bank_contention_plus_one_cycle() {
        let mut bus = Bus::new();
        // Core 0 touches bank 0.
        bus.set_active_core(0);
        let _ = bus.read32(0x2000_0000);
        // Core 1 touches the same bank — expect +1 cycle latency.
        bus.set_active_core(1);
        bus.begin_core1_step();
        let _ = bus.read32(0x2000_0000);
        assert_eq!(bus.last_access_cycles, 2);
        bus.end_core1_step();
    }

    #[test]
    fn sram_bank_no_contention_different_banks() {
        let mut bus = Bus::new();
        bus.set_active_core(0);
        let _ = bus.read32(0x2000_0000); // bank 0
        bus.set_active_core(1);
        bus.begin_core1_step();
        let _ = bus.read32(0x2000_0004); // bank 1
        assert_eq!(bus.last_access_cycles, 1);
        bus.end_core1_step();
    }
}
