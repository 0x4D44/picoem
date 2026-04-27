//! `Peripherals` — Mutex-guarded peripheral-state bundle shared across
//! the coordinator and CPU workers in the RP2040 threaded runtime.
//!
//! Stage 3b.3 (dual-execution HLD V1 §6.4): parallel home for the cold-path
//! peripheral register banks that on the serial `Bus` are plain struct
//! fields. WorkerBus routes MMIO into these mutexes; the coordinator
//! (Stage 3b.4) refreshes snapshot-backed read paths (timer, PIO) under
//! the same locks.
//!
//! The component `State` structs reuse the existing serial `Bus`
//! peripheral types directly (`ClocksRegs`, `Resets`, `XoscRegs`,
//! `RoscRegs`, `IoBank0`, `PadsBank0`). No behavioural change between
//! paths — just a relocation into Mutex-guarded storage.
//!
//! ## Lock order
//!
//! The serial `Bus::peripheral_read32` / `peripheral_write32` path
//! checks RESETS first and returns / drops the access if the peripheral
//! is held in reset, then dispatches to the peripheral module. The
//! threaded WorkerBus matches that sequence — RESETS first, then the
//! destination peripheral — which is why the documented lock order for
//! a single MMIO dispatch is:
//!
//! `resets < clocks < io < timer < legacy`
//!
//! Stage 3b.3 has zero nested lock sites per dispatch (each branch
//! acquires and releases one lock), so this is a forward-looking
//! invariant for any future nested call.
//!
//! ## Poisoning
//!
//! Call sites use `.lock().unwrap()` — panic on poison. A poisoned
//! mutex implies the previous lock-holder panicked, which already left
//! the emulator in an indeterminate state; fail loud rather than
//! silently continue on stale data.
//!
//! Dropped vs mdrp2350's `Peripherals`:
//! - No QMI block (RP2040 has no on-chip XIP through QMI).
//! - UART / SPI / I2C / ADC / PWM / DMA / WATCHDOG: carried in the
//!   `legacy` HashMap for Stage 3b.3 — the serial types pull in a lot
//!   of state (IRQ register side-effects for UART/SPI/I2C, DMA trigger
//!   channels) the threaded path would need to forward. Stage 3b.4 is
//!   the place to refactor the hot ones. WorkerBus fall-throughs for
//!   those regions match the serial HashMap RAW semantics so firmware
//!   sees a clean round-trip.
//! - TIMER: typed state — `TimerRegs` has read-side state (TIMELR read
//!   latches TIMEHR) the HashMap cannot replicate.

use std::collections::HashMap;
use std::sync::Mutex;

use mdpicoem_common::clocks::{pll_cs_read_with_lock, pll_should_arm_lock};

use crate::bus::clocks::{
    ClockTree, ClocksRegs, PLL_RESET, PllRegs, RoscRegs, XoscRegs, pll_read, pll_write, recompute,
};
use crate::bus::io_bank0::IoBank0;
use crate::bus::pads_bank0::PadsBank0;
use crate::bus::resets::Resets;
use crate::peripherals::timer::TimerRegs;

// =======================================================================
// ClocksState — CLOCKS + PLL_SYS + PLL_USB + XOSC + ROSC + derived tree
// =======================================================================

/// CLOCKS / PLL_SYS / PLL_USB / XOSC / ROSC aggregate + derived
/// [`ClockTree`] cache. Reuses the serial `Bus` storage types 1:1.
pub struct ClocksState {
    pub clocks_regs: ClocksRegs,
    pub xosc_regs: XoscRegs,
    pub rosc_regs: RoscRegs,
    pub pll_sys_regs: PllRegs,
    pub pll_usb_regs: PllRegs,
    pub pll_sys_lock_at_cycle: Option<u64>,
    pub pll_usb_lock_at_cycle: Option<u64>,
    pub clock_tree: ClockTree,
}

impl ClocksState {
    /// Fresh `ClocksState` at power-on defaults. Mirrors `Bus::new()`
    /// exactly for the relevant fields.
    pub fn new_default() -> Self {
        Self {
            clocks_regs: ClocksRegs::new(),
            xosc_regs: XoscRegs::new(),
            rosc_regs: RoscRegs::new(),
            pll_sys_regs: PLL_RESET,
            pll_usb_regs: PLL_RESET,
            pll_sys_lock_at_cycle: None,
            pll_usb_lock_at_cycle: None,
            clock_tree: ClockTree::default(),
        }
    }

    /// CLOCKS read. Mirrors `ClocksRegs::read32` — plain storage.
    pub fn clocks_read(&self, offset: u32) -> u32 {
        self.clocks_regs.read32(offset)
    }

    /// CLOCKS write + ClockTree recompute on relevant fields.
    pub fn clocks_write(&mut self, offset: u32, val: u32, alias: u32) {
        if self.clocks_regs.write32(offset, val, alias) {
            self.recompute();
        }
    }

    /// XOSC read / write — plain storage + STATUS synthesis.
    pub fn xosc_read(&self, offset: u32) -> u32 {
        self.xosc_regs.read32(offset)
    }
    pub fn xosc_write(&mut self, offset: u32, val: u32, alias: u32) {
        self.xosc_regs.write32(offset, val, alias);
    }

    /// ROSC read / write.
    pub fn rosc_read(&self, offset: u32) -> u32 {
        self.rosc_regs.read32(offset)
    }
    pub fn rosc_write(&mut self, offset: u32, val: u32, alias: u32) {
        self.rosc_regs.write32(offset, val, alias);
    }

    /// Read a PLL_SYS register with LOCK-bit synthesis on CS.
    pub fn pll_sys_read_at(&self, offset: u32, master_cycle: u64) -> u32 {
        if offset == 0x00 {
            pll_cs_read_with_lock(&self.pll_sys_regs, self.pll_sys_lock_at_cycle, master_cycle)
        } else {
            pll_read(&self.pll_sys_regs, offset)
        }
    }

    /// Read a PLL_USB register with LOCK-bit synthesis on CS.
    pub fn pll_usb_read_at(&self, offset: u32, master_cycle: u64) -> u32 {
        if offset == 0x00 {
            pll_cs_read_with_lock(&self.pll_usb_regs, self.pll_usb_lock_at_cycle, master_cycle)
        } else {
            pll_read(&self.pll_usb_regs, offset)
        }
    }

    /// Alias-aware PLL_SYS write + lock-arm refresh + ClockTree recompute.
    pub fn pll_sys_write_at(&mut self, offset: u32, val: u32, alias: u32, master_cycle: u64) {
        let old_regs = self.pll_sys_regs;
        if pll_write(&mut self.pll_sys_regs, offset, val, alias) {
            self.pll_sys_lock_at_cycle = pll_should_arm_lock(
                &old_regs,
                &self.pll_sys_regs,
                self.pll_sys_lock_at_cycle,
                master_cycle,
            );
            self.recompute();
        }
    }

    /// Alias-aware PLL_USB write + lock-arm refresh + ClockTree recompute.
    pub fn pll_usb_write_at(&mut self, offset: u32, val: u32, alias: u32, master_cycle: u64) {
        let old_regs = self.pll_usb_regs;
        if pll_write(&mut self.pll_usb_regs, offset, val, alias) {
            self.pll_usb_lock_at_cycle = pll_should_arm_lock(
                &old_regs,
                &self.pll_usb_regs,
                self.pll_usb_lock_at_cycle,
                master_cycle,
            );
            self.recompute();
        }
    }

    /// Recompute derived `ClockTree` — delegate to the shared helper.
    fn recompute(&mut self) {
        recompute(
            &self.clocks_regs,
            &self.pll_sys_regs,
            &self.pll_usb_regs,
            &mut self.clock_tree,
        );
    }
}

impl Default for ClocksState {
    fn default() -> Self {
        Self::new_default()
    }
}

// =======================================================================
// ResetsState — RESETS peripheral
// =======================================================================

pub struct ResetsState {
    pub resets: Resets,
}

impl ResetsState {
    pub fn new_default() -> Self {
        Self {
            resets: Resets::new(),
        }
    }

    pub fn read(&self, offset: u32) -> u32 {
        self.resets.read32(offset)
    }

    pub fn write(&mut self, offset: u32, val: u32, alias: u32) {
        self.resets.write32(offset, val, alias);
    }

    /// True iff the peripheral whose bus base is `base` is currently held
    /// in `RESETS.RESET`. Matches the serial `peripheral_dispatch::
    /// is_held_in_reset` lookup.
    pub fn is_held_in_reset_base(&self, base: u32) -> bool {
        use crate::bus::peripheral_dispatch::BASE_RESET_MAP;
        for &(b, bit) in BASE_RESET_MAP {
            if b == base {
                return self.resets.is_held(bit);
            }
        }
        false
    }
}

impl Default for ResetsState {
    fn default() -> Self {
        Self::new_default()
    }
}

// =======================================================================
// IoState — IO_BANK0 + PADS_BANK0
// =======================================================================

pub struct IoState {
    pub io_bank0: IoBank0,
    pub pads_bank0: PadsBank0,
}

impl IoState {
    pub fn new_default() -> Self {
        Self {
            io_bank0: IoBank0::new(),
            pads_bank0: PadsBank0::new(),
        }
    }
}

impl Default for IoState {
    fn default() -> Self {
        Self::new_default()
    }
}

// =======================================================================
// TimerState — typed TIMER peripheral (preserves TIMELR→TIMEHR latching)
// =======================================================================

/// Thin wrapper around the serial [`TimerRegs`] so the threaded path
/// can preserve the read-side state — specifically, TIMELR read
/// latches TIMEHR, and that latched value must survive until the next
/// TIMELR read. A plain `HashMap<u32, u32>` cannot model this.
///
/// Stage 3b.3 seeds `master_cycle` / `sys_hz` at each access from
/// `SharedState.master_cycle` + the cached `ClockTree.sys_clk_hz`;
/// Stage 3b.4's coordinator will advance these values as part of its
/// quantum tick so alarms also fire.
pub struct TimerState {
    pub regs: TimerRegs,
}

impl TimerState {
    pub fn new_default() -> Self {
        Self {
            regs: TimerRegs::new(),
        }
    }

    /// Route a TIMER read. `master_cycle` + `sys_hz` come from the
    /// worker's `SharedState` snapshot.
    pub fn read32(&mut self, offset: u32, master_cycle: u64, sys_hz: u32) -> u32 {
        self.regs.read32(offset, master_cycle, sys_hz)
    }

    /// Route a TIMER write. Alias is the 2-bit APB normalized form
    /// (0=plain / 1=XOR / 2=BITSET / 3=BITCLR).
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, master_cycle: u64, sys_hz: u32) {
        self.regs
            .write32(offset, value, alias, master_cycle, sys_hz);
    }
}

impl Default for TimerState {
    fn default() -> Self {
        Self::new_default()
    }
}

// =======================================================================
// Peripherals aggregate
// =======================================================================

/// Mutex-guarded bundle of RP2040 peripheral state shared across worker
/// threads. Lives behind an `Arc` on `SharedState`.
///
/// Lock order (matches the dispatch order in `WorkerBus::apb_{read,write}32`):
/// `resets < clocks < io < timer < legacy`.
pub struct Peripherals {
    pub clocks: Mutex<ClocksState>,
    pub resets: Mutex<ResetsState>,
    pub io: Mutex<IoState>,
    /// TIMER peripheral. Typed state because TIMELR read latches TIMEHR —
    /// a HashMap cannot replicate that side effect.
    pub timer: Mutex<TimerState>,
    /// Untyped fallback HashMap for peripheral regions we have not
    /// migrated to typed storage on the threaded path yet: UART / SPI /
    /// I2C / ADC / PWM / DMA / WATCHDOG / XIP_CTRL / SSI / SYSINFO /
    /// BUSCTRL / SYSCFG / PSM / IO_QSPI / PADS_QSPI. Serial `Bus` stores
    /// these in a similar `peripheral_regs: HashMap<u32, u32>` — the
    /// RAW (read-after-write) semantics are preserved via the same
    /// alias-aware update rule (normal / XOR / OR / AND-NOT).
    ///
    /// See `tech_debt.md` for the follow-up to type the hot register-
    /// side-effect paths (UART/SPI/I2C IRQ RIS, DMA trigger).
    pub legacy: Mutex<HashMap<u32, u32>>,
}

impl Peripherals {
    /// Construct a fresh `Peripherals` at power-on defaults.
    pub fn new_default() -> Self {
        Self {
            clocks: Mutex::new(ClocksState::new_default()),
            resets: Mutex::new(ResetsState::new_default()),
            io: Mutex::new(IoState::new_default()),
            timer: Mutex::new(TimerState::new_default()),
            legacy: Mutex::new(HashMap::new()),
        }
    }
}

impl Default for Peripherals {
    fn default() -> Self {
        Self::new_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn peripherals_default_construction_locks_succeed() {
        let p = Peripherals::new_default();
        // None of the locks should be poisoned on a freshly built bundle.
        assert!(p.clocks.lock().is_ok());
        assert!(p.resets.lock().is_ok());
        assert!(p.io.lock().is_ok());
        assert!(p.timer.lock().is_ok());
        assert!(p.legacy.lock().is_ok());
    }

    #[test]
    fn clocks_state_post_bootrom_defaults_mirror_bus() {
        let c = ClocksState::new_default();
        assert_eq!(c.pll_sys_regs, PLL_RESET);
        assert_eq!(c.pll_usb_regs, PLL_RESET);
        assert_eq!(c.pll_sys_lock_at_cycle, None);
        assert_eq!(c.pll_usb_lock_at_cycle, None);
        // Default clk_sys_div (RP2040 SVD) = 0x0001_0000.
        assert_eq!(c.clocks_regs.clk_sys_div, 0x0001_0000);
    }

    #[test]
    fn resets_state_default_holds_everything() {
        let r = ResetsState::new_default();
        assert_eq!(r.read(0x00), crate::bus::resets::RESET_MASK);
        assert!(r.is_held_in_reset_base(crate::bus::TIMER_BASE));
        assert!(r.is_held_in_reset_base(crate::bus::UART0_BASE));
    }

    #[test]
    fn clocks_write_triggers_recompute_via_tree_update() {
        let mut c = ClocksState::new_default();
        // ROSC default tree is unset until first write — after a CLK_SYS
        // write the cached tree should reflect post-recompute state.
        c.clocks_write(0x3C /* CLK_SYS_CTRL */, 0, 0);
        // No src change → default src is clk_ref → still ROSC.
        assert_eq!(c.clock_tree.sys_clk_hz, c.clock_tree.ref_clk_hz);
    }

    #[test]
    fn pll_sys_write_arms_lock_from_correct_cycle() {
        let mut c = ClocksState::new_default();
        // Power-up path: clear PWR, set FBDIV.
        c.pll_sys_write_at(0x04, 0, 0, 100);
        c.pll_sys_write_at(0x08, 125, 0, 100);
        // Lock should be armed at some cycle ≥ start.
        assert!(c.pll_sys_lock_at_cycle.is_some());
    }
}
