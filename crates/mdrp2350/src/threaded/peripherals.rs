//! `Peripherals` — Mutex-guarded peripheral-state bundle shared across
//! the coordinator, CPU workers, and PIO worker in the threaded runtime.
//!
//! Phase 3 Stage 4 (LLD V7 §7): creates the parallel scaffolding only.
//! Stage 5 (`WorkerBus`) routes MMIO to these mutexes; Stage 6's
//! `ThreadedEmulator::from_emulator` constructs this from the existing
//! single-threaded `Bus` field storage.
//!
//! The component `State` structs mirror — 1:1 — the fields already on
//! `crate::bus::Bus` (same names, same types). No functional change in
//! Stage 4: the Bus field ownership stays put; this is a parallel home
//! ready for Stage 6 to populate.
//!
//! ## Lock order
//!
//! Acquire in this order to avoid deadlock (Phase 3 has zero nested
//! lock sites, so this is a forward-looking invariant):
//!
//! `clocks < qmi < resets < apb < timers < dma < legacy`
//!
//! ## Poisoning
//!
//! Call sites use `.lock().unwrap()` — panic on poison. A poisoned
//! mutex implies the previous lock-holder panicked, which already left
//! the emulator in an indeterminate state; fail loud rather than
//! silently continue on stale data.
//!
//! ## PLL offset dispatch
//!
//! `ClocksState::pll_sys_read_at(offset, master_cycle)` /
//! `pll_usb_read_at(offset, master_cycle)` mirror the offset dispatch
//! on `Bus::pll_sys_read_at` / `Bus::pll_usb_read_at` exactly —
//! `0x000` returns CS with the LOCK bit derived from the supplied
//! `master_cycle`; `0x004`/`0x008`/`0x00C` return the raw register
//! image; other offsets return 0. The master-cycle snapshot is taken
//! *outside* the lock so a concurrent coordinator `fetch_add` doesn't
//! serialize with CPU reads.

use std::collections::HashMap;
use std::sync::Mutex;

use mdpicoem_common::clocks::{ClockTree, pll_cs_read_with_lock};

use crate::bus::RESETS_POST_BOOTROM;
use crate::dma::Dma;
use crate::peripherals::adc::AdcRegs;
use crate::peripherals::i2c::I2cRegs;
use crate::peripherals::io_bank0::IoBank0Regs;
use crate::peripherals::pads_bank0::PadsBank0Regs;
use crate::peripherals::pwm::PwmRegs;
use crate::peripherals::spi::SpiRegs;
use crate::peripherals::ticks::TicksRegs;
use crate::peripherals::timer::TimerRegs;
use crate::peripherals::uart::UartRegs;

// =======================================================================
// Component state structs
// =======================================================================

/// CLOCKS + PLL_SYS + PLL_USB + ROSC + XOSC + GPIO-hi noise.
///
/// Field types mirror `crate::bus::Bus` exactly. See `bus/mod.rs:285..338`.
pub struct ClocksState {
    /// CLK_REF_CTRL register (CLOCKS offset 0x030).
    pub clk_ref_ctrl: u32,
    /// CLK_SYS_CTRL register (CLOCKS offset 0x03C).
    pub clk_sys_ctrl: u32,
    /// CLK_SYS_DIV register (CLOCKS offset 0x040).
    pub clk_sys_div: u32,
    /// Derived clock-tree frequencies.
    pub clock_tree: ClockTree,
    /// PLL_SYS register image `[CS, PWR, FBDIV_INT, PRIM]`.
    pub pll_sys_regs: [u32; 4],
    /// PLL_USB register image `[CS, PWR, FBDIV_INT, PRIM]`.
    pub pll_usb_regs: [u32; 4],
    /// Master cycle at which PLL_SYS's lock-detect counter expires.
    pub pll_sys_lock_at_cycle: Option<u64>,
    /// Master cycle at which PLL_USB's lock-detect counter expires.
    pub pll_usb_lock_at_cycle: Option<u64>,
    /// ROSC register image (9 words).
    pub rosc: [u32; 9],
    /// XOSC register image (5 words).
    pub xosc: [u32; 5],
    /// SIO GPIO_HI_IN noise seed (QSPI pin activity simulation).
    pub gpio_hi_noise_state: u32,
}

impl ClocksState {
    /// Mirror `Bus::new()` / `Bus::with_atomics()` defaults at
    /// `bus/mod.rs:430..442`. Post-bootrom state — no PLL lock armed,
    /// 150 MHz sys / 12 MHz ref / 150 MHz peri.
    pub fn post_bootrom() -> Self {
        use mdpicoem_common::clocks::{RP2350_SYS_CLK_HZ, XOSC_FREQ_HZ};
        let clock_tree = ClockTree {
            sys_clk_hz: RP2350_SYS_CLK_HZ,
            ref_clk_hz: XOSC_FREQ_HZ,
            peri_clk_hz: RP2350_SYS_CLK_HZ,
        };
        Self {
            clk_ref_ctrl: 0,
            clk_sys_ctrl: 0,
            clk_sys_div: 0x0001_0000,
            clock_tree,
            pll_sys_regs: [0x0000_0001, 0x0000_002D, 0, 0x0007_7000],
            pll_usb_regs: [0x0000_0001, 0x0000_002D, 0, 0x0007_7000],
            pll_sys_lock_at_cycle: None,
            pll_usb_lock_at_cycle: None,
            rosc: [0u32; 9],
            xosc: [0u32; 5],
            gpio_hi_noise_state: 0xA5A5_A5A5,
        }
    }

    /// Read PLL_SYS register by offset with the LOCK bit (CS[31])
    /// derived from the supplied master-cycle snapshot. Mirrors
    /// `Bus::pll_sys_read_at` in `bus/peripherals.rs` exactly:
    /// `0x000` → CS with live LOCK, `0x004`/`0x008`/`0x00C` → raw
    /// register image, other offsets → 0. Caller snapshots
    /// `master_cycle` from `SharedState.master_cycle` (lock-free)
    /// before taking the clocks lock so the helper does not serialize
    /// with the coordinator's `fetch_add`.
    pub fn pll_sys_read_at(&self, offset: u32, master_cycle: u64) -> u32 {
        pll_read_from(
            &self.pll_sys_regs,
            offset,
            self.pll_sys_lock_at_cycle,
            master_cycle,
        )
    }

    /// Read PLL_USB register by offset with the LOCK bit (CS[31])
    /// derived from the supplied master-cycle snapshot. Mirrors
    /// `Bus::pll_usb_read_at` in `bus/peripherals.rs` exactly.
    pub fn pll_usb_read_at(&self, offset: u32, master_cycle: u64) -> u32 {
        pll_read_from(
            &self.pll_usb_regs,
            offset,
            self.pll_usb_lock_at_cycle,
            master_cycle,
        )
    }
}

/// Map a PLL register offset to its index in the `[u32; 4]` register
/// image. Returns `None` for unknown offsets. Duplicated (by design,
/// ~6 LOC) from `bus/peripherals.rs::pll_reg_index` — keeps Stage 4
/// minimally invasive; Stage 5 can revisit if the duplication becomes
/// a maintenance burden.
fn pll_reg_index(offset: u32) -> Option<usize> {
    match offset {
        0x000 => Some(0),
        0x004 => Some(1),
        0x008 => Some(2),
        0x00C => Some(3),
        _ => None,
    }
}

/// Read a PLL register with LOCK-bit synthesis on CS (offset 0x000).
/// Duplicated (~6 LOC) from `bus/peripherals.rs::pll_read_from` for
/// Stage 4 self-containment. The LOCK-bit logic lives in
/// `mdpicoem_common::clocks::pll_cs_read_with_lock` and is shared.
fn pll_read_from(regs: &[u32; 4], offset: u32, lock_at: Option<u64>, now: u64) -> u32 {
    match pll_reg_index(offset) {
        Some(0) => pll_cs_read_with_lock(regs, lock_at, now),
        Some(i) => regs[i],
        None => 0,
    }
}

/// QMI QSPI memory interface state. See `bus/mod.rs:283, 338`.
pub struct QmiState {
    /// QMI register backing store (28 words, offsets 0x000..0x06C).
    pub qmi_regs: [u32; 28],
    /// XIP cache window offset (set by QMI M0_RFMT writes).
    pub xip_cache_offset: u32,
}

impl QmiState {
    /// Mirror `Bus::new()` defaults.
    pub fn post_bootrom() -> Self {
        Self {
            qmi_regs: [0u32; 28],
            xip_cache_offset: 0,
        }
    }
}

/// RESETS block state. See `bus/mod.rs:246`.
pub struct ResetsState {
    /// RESETS.RESET register. Bits set = peripheral held in reset.
    pub resets_state: u32,
}

impl ResetsState {
    /// Mirror `Bus::new()` — post-bootrom peripherals released.
    pub fn post_bootrom() -> Self {
        Self {
            resets_state: RESETS_POST_BOOTROM,
        }
    }
}

/// APB peripheral register state. Mirrors `bus/mod.rs:253..266`.
pub struct ApbState {
    /// UART0 — PL011-derived UART at 0x4007_0000.
    pub uart0: UartRegs,
    /// SPI0 — PL022-derived SPI at 0x4008_0000.
    pub spi0: SpiRegs,
    /// I2C0 — DesignWare DW_apb_i2c at 0x4009_0000.
    pub i2c0: I2cRegs,
    /// ADC — single instance at 0x400A_0000.
    pub adc: AdcRegs,
    /// PWM — 12-slice block at 0x4005_0000.
    pub pwm: PwmRegs,
    /// IO_BANK0 plain-storage GPIO control.
    pub io_bank0: IoBank0Regs,
    /// PADS_BANK0 plain-storage pad drive/pull control.
    pub pads_bank0: PadsBank0Regs,
}

impl ApbState {
    /// Mirror `Bus::new()` — same IRQ constants wired into each
    /// peripheral as the single-threaded path.
    pub fn post_bootrom() -> Self {
        use crate::irq::{
            IRQ_ADC_IRQ_FIFO, IRQ_I2C0_IRQ, IRQ_PWM_IRQ_WRAP_0, IRQ_PWM_IRQ_WRAP_1,
            IRQ_SPI0_IRQ, IRQ_UART0_IRQ,
        };
        Self {
            uart0: UartRegs::new(IRQ_UART0_IRQ),
            spi0: SpiRegs::new(IRQ_SPI0_IRQ),
            i2c0: I2cRegs::new(IRQ_I2C0_IRQ),
            adc: AdcRegs::new(IRQ_ADC_IRQ_FIFO),
            pwm: PwmRegs::new(IRQ_PWM_IRQ_WRAP_0, IRQ_PWM_IRQ_WRAP_1),
            io_bank0: IoBank0Regs::new(),
            pads_bank0: PadsBank0Regs::new(),
        }
    }
}

/// TICKS + TIMER0 + TIMER1 state. See `bus/mod.rs:248..252`.
pub struct TimersState {
    /// TICKS block — six-domain 1 µs tick generator.
    pub ticks: TicksRegs,
    /// TIMER0 — 64-bit µs counter + four alarms.
    pub timer0: TimerRegs,
    /// TIMER1 — same shape as TIMER0.
    pub timer1: TimerRegs,
}

impl TimersState {
    /// Mirror `Bus::new()`.
    pub fn post_bootrom() -> Self {
        use crate::irq::{IRQ_TIMER0_IRQ_0, IRQ_TIMER1_IRQ_0};
        Self {
            ticks: TicksRegs::post_bootrom(),
            timer0: TimerRegs::new(IRQ_TIMER0_IRQ_0),
            timer1: TimerRegs::new(IRQ_TIMER1_IRQ_0),
        }
    }
}

/// DMA controller state. See `bus/mod.rs:268`.
pub struct DmaState {
    /// 16-channel DMA controller.
    pub dma: Dma,
}

impl DmaState {
    /// Mirror `Bus::new()`.
    pub fn post_bootrom() -> Self {
        Self { dma: Dma::new() }
    }
}

// =======================================================================
// Peripherals aggregate
// =======================================================================

/// Mutex-guarded bundle of peripheral state shared across worker
/// threads. Instances live behind an `Arc` on `SharedState`.
///
/// Lock order: `clocks < qmi < resets < apb < timers < dma < legacy`.
pub struct Peripherals {
    pub clocks: Mutex<ClocksState>,
    pub qmi: Mutex<QmiState>,
    pub resets: Mutex<ResetsState>,
    pub apb: Mutex<ApbState>,
    pub timers: Mutex<TimersState>,
    pub dma: Mutex<DmaState>,
    /// Legacy untyped register HashMap. Mirrors `Bus::peripheral_regs`
    /// (9–11 live call sites in `bus/mod.rs`). Phase 5 migrates the
    /// remaining sites and deletes this field.
    pub legacy: Mutex<HashMap<u32, u32>>,
}

impl Peripherals {
    /// Construct a fresh `Peripherals` with every component in its
    /// post-bootrom state, matching `Bus::new()` / `Bus::with_atomics()`.
    ///
    /// Stage 6's `ThreadedEmulator::from_emulator` uses a struct-literal
    /// form that consumes the existing Bus field storage instead; this
    /// constructor exists for unit tests and any future standalone use.
    pub fn new_default() -> Self {
        Self {
            clocks: Mutex::new(ClocksState::post_bootrom()),
            qmi: Mutex::new(QmiState::post_bootrom()),
            resets: Mutex::new(ResetsState::post_bootrom()),
            apb: Mutex::new(ApbState::post_bootrom()),
            timers: Mutex::new(TimersState::post_bootrom()),
            dma: Mutex::new(DmaState::post_bootrom()),
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
    fn peripherals_default_construction() {
        let p = Peripherals::new_default();
        // None of the locks should be poisoned on a freshly built
        // Peripherals — attempting each `.lock()` succeeds.
        assert!(p.clocks.lock().is_ok());
        assert!(p.qmi.lock().is_ok());
        assert!(p.resets.lock().is_ok());
        assert!(p.apb.lock().is_ok());
        assert!(p.timers.lock().is_ok());
        assert!(p.dma.lock().is_ok());
        assert!(p.legacy.lock().is_ok());
    }

    #[test]
    fn clocks_state_mirrors_bus_post_bootrom() {
        // Defaults must match `Bus::new()` exactly so Stage 6 can
        // populate via struct-literal from existing Bus fields without
        // a semantic drift.
        let c = ClocksState::post_bootrom();
        assert_eq!(c.clk_sys_div, 0x0001_0000);
        assert_eq!(c.pll_sys_regs, [0x0000_0001, 0x0000_002D, 0, 0x0007_7000]);
        assert_eq!(c.pll_usb_regs, [0x0000_0001, 0x0000_002D, 0, 0x0007_7000]);
        assert_eq!(c.pll_sys_lock_at_cycle, None);
        assert_eq!(c.pll_usb_lock_at_cycle, None);
        assert_eq!(c.gpio_hi_noise_state, 0xA5A5_A5A5);
    }

    #[test]
    fn pll_cs_helper_derives_lock_bit_from_master_cycle() {
        // Regression harness for the Stage 4 PLL CS helper refactor:
        // caller threads the master-cycle snapshot through; the helper
        // derives CS[31] = (master_cycle >= lock_at_cycle).
        let mut c = ClocksState::post_bootrom();
        // Power the PLL enough that the base-predicate (FBDIV != 0)
        // can hold once we program it, and arm the lock at cycle 100.
        // We only need the lock_at and the CS->master_cycle comparison
        // here; exhaustive base-predicate coverage lives in
        // mdpicoem-common.
        c.pll_sys_regs[0] = 0x0000_0001; // CS image stays all-zero in LOCK bit slot
        c.pll_sys_regs[1] = 0; // PWR cleared → base predicate can fire
        c.pll_sys_regs[2] = 125; // FBDIV != 0
        c.pll_sys_lock_at_cycle = Some(100);

        // Before the deadline — LOCK bit must be 0.
        let cs_before = c.pll_sys_read_at(0x000, 50);
        assert_eq!(cs_before & (1 << 31), 0, "LOCK must be 0 before deadline");
        // After the deadline — LOCK bit must be 1.
        let cs_after = c.pll_sys_read_at(0x000, 150);
        assert_ne!(
            cs_after & (1 << 31),
            0,
            "LOCK must be 1 at/after deadline"
        );
    }

    #[test]
    fn pll_read_at_offsets_match_bus_dispatch() {
        // Non-CS offsets must return the raw register image; unknown
        // offsets must return 0. Mirrors Bus::pll_sys_read_at /
        // Bus::pll_usb_read_at exactly.
        let mut c = ClocksState::post_bootrom();
        c.pll_sys_regs = [0xAAAA_AAAA, 0x1111_1111, 0x2222_2222, 0x3333_3333];
        c.pll_usb_regs = [0x5555_5555, 0x6666_6666, 0x7777_7777, 0x8888_8888];
        c.pll_sys_lock_at_cycle = None; // CS LOCK bit stays 0, CS returns raw
        c.pll_usb_lock_at_cycle = None;

        // 0x004 / 0x008 / 0x00C → raw register image, untouched.
        assert_eq!(c.pll_sys_read_at(0x004, 0), 0x1111_1111);
        assert_eq!(c.pll_sys_read_at(0x008, 0), 0x2222_2222);
        assert_eq!(c.pll_sys_read_at(0x00C, 0), 0x3333_3333);
        assert_eq!(c.pll_usb_read_at(0x004, 0), 0x6666_6666);
        assert_eq!(c.pll_usb_read_at(0x008, 0), 0x7777_7777);
        assert_eq!(c.pll_usb_read_at(0x00C, 0), 0x8888_8888);

        // 0x000 (CS) returns base image with LOCK bit force-cleared
        // when no lock is armed — the pll_cs_read_with_lock helper
        // clears CS[31] unless the base predicate AND master_cycle >=
        // lock_at both hold. pll_sys CS input 0xAAAA_AAAA → 0x2AAA_AAAA
        // (bit 31 cleared). pll_usb CS input 0x5555_5555 already has
        // bit 31 clear → returns unchanged.
        assert_eq!(c.pll_sys_read_at(0x000, 0), 0x2AAA_AAAA);
        assert_eq!(c.pll_usb_read_at(0x000, 0), 0x5555_5555);

        // Unknown offsets return 0.
        assert_eq!(c.pll_sys_read_at(0x010, 0), 0);
        assert_eq!(c.pll_sys_read_at(0x020, 0), 0);
        assert_eq!(c.pll_usb_read_at(0x010, 0), 0);
        assert_eq!(c.pll_usb_read_at(0xFFF, 0), 0);
    }
}
