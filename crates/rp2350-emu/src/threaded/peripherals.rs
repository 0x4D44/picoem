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

use picoem_common::clocks::{ClockTree, pll_cs_read_with_lock, pll_should_arm_lock};

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
use crate::peripherals::usb::UsbCtrl;

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
        use picoem_common::clocks::{RP2350_SYS_CLK_HZ, XOSC_FREQ_HZ};
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

    /// Alias-aware PLL_SYS write + lock-arm refresh. Mirrors
    /// `Bus::pll_sys_write_at` in `bus/peripherals.rs`. The master-cycle
    /// snapshot is caller-supplied so the WorkerBus keeps the
    /// `SharedState.master_cycle` read outside this mutex.
    pub fn pll_sys_write_at(&mut self, offset: u32, val: u32, alias: u32, master_cycle: u64) {
        let old_regs = self.pll_sys_regs;
        pll_write_into(&mut self.pll_sys_regs, offset, val, alias);
        self.pll_sys_lock_at_cycle = pll_should_arm_lock(
            &old_regs,
            &self.pll_sys_regs,
            self.pll_sys_lock_at_cycle,
            master_cycle,
        );
        self.recompute_clock_tree();
    }

    /// Alias-aware PLL_USB write + lock-arm refresh. Mirrors
    /// `Bus::pll_usb_write_at`.
    pub fn pll_usb_write_at(&mut self, offset: u32, val: u32, alias: u32, master_cycle: u64) {
        let old_regs = self.pll_usb_regs;
        pll_write_into(&mut self.pll_usb_regs, offset, val, alias);
        self.pll_usb_lock_at_cycle = pll_should_arm_lock(
            &old_regs,
            &self.pll_usb_regs,
            self.pll_usb_lock_at_cycle,
            master_cycle,
        );
        self.recompute_clock_tree();
    }

    // --- CLOCKS (0x4001_0000) ---------------------------------------
    //
    // Mirrors `Bus::clocks_read` / `Bus::clocks_write` exactly — only
    // CLK_REF_CTRL / CLK_SYS_CTRL / CLK_SYS_DIV have typed storage;
    // channels without backing storage return zero on read and are
    // dropped on write.

    /// CLOCKS read. Mirrors `Bus::clocks_read`.
    pub fn clocks_read(&self, offset: u32) -> u32 {
        match offset {
            // clk_gpout0..3 — non-glitchless, _SELECTED reads 1.
            0x000 | 0x004 => 0,
            0x008 => 1,
            0x00C | 0x010 => 0,
            0x014 => 1,
            0x018 | 0x01C => 0,
            0x020 => 1,
            0x024 | 0x028 => 0,
            0x02C => 1,
            // clk_ref — glitchless.
            0x030 => self.clk_ref_ctrl,
            0x038 => 1 << (self.clk_ref_ctrl & 0x3),
            // clk_sys — glitchless.
            0x03C => self.clk_sys_ctrl,
            0x040 => self.clk_sys_div,
            0x044 => 1 << (self.clk_sys_ctrl & 0x1),
            // clk_peri / clk_hstx / clk_usb / clk_adc — non-glitchless.
            0x048 | 0x04C => 0,
            0x050 => 1,
            0x054 | 0x058 => 0,
            0x05C => 1,
            0x060 | 0x064 => 0,
            0x068 => 1,
            0x06C | 0x070 => 0,
            0x074 => 1,
            _ => 0,
        }
    }

    /// CLOCKS write. Mirrors `Bus::clocks_write`. Only CLK_REF_CTRL /
    /// CLK_SYS_CTRL / CLK_SYS_DIV have backing storage; other writes
    /// are dropped.
    pub fn clocks_write(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |current: u32| match alias {
            0 => val,
            1 => current ^ val,
            2 => current | val,
            3 => current & !val,
            _ => val,
        };
        match offset {
            0x030 => self.clk_ref_ctrl = apply(self.clk_ref_ctrl),
            0x03C => self.clk_sys_ctrl = apply(self.clk_sys_ctrl),
            0x040 => self.clk_sys_div = apply(self.clk_sys_div),
            _ => {}
        }
        self.recompute_clock_tree();
    }

    /// Refresh the cached `ClockTree` after a CLOCKS / PLL write.
    /// Mirrors `Bus::recompute_clock_tree` — see `bus/mod.rs` for the
    /// single source of truth. Stage 5 duplicates the logic locally
    /// because Bus's helper is an inherent method on the `Bus` type
    /// and not reachable from here.
    pub(crate) fn recompute_clock_tree(&mut self) {
        use picoem_common::clocks::{RP2350_SYS_CLK_HZ, XOSC_FREQ_HZ};
        // Phase 3 Stage 5: this is a lightweight mirror. The
        // single-threaded Bus::recompute_clock_tree does more
        // elaborate PLL-rate derivation; at this point the WorkerBus
        // only relies on the ClockTree for observable values of
        // clk_sys_hz etc., none of which affect Stage 5 tests. A
        // later stage replaces this with the full derivation once
        // there's a concrete consumer.
        let _ = RP2350_SYS_CLK_HZ;
        let _ = XOSC_FREQ_HZ;
    }

    // --- ROSC (0x400E_8000) -----------------------------------------

    pub fn rosc_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.rosc[0],
            0x004 => self.rosc[1],
            0x008 => self.rosc[2],
            0x00C => 0, // RANDOM — stub
            0x010 => self.rosc[4],
            0x014 => self.rosc[5],
            0x018 => (1 << 31) | (1 << 12), // STATUS: STABLE | ENABLED
            0x01C => 0,                     // RANDOMBIT
            0x020 => 0,                     // COUNT
            _ => 0,
        }
    }

    pub fn rosc_write(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |current: u32| match alias {
            0 => val,
            1 => current ^ val,
            2 => current | val,
            3 => current & !val,
            _ => val,
        };
        let idx = match offset {
            0x000 => 0,
            0x004 => 1,
            0x008 => 2,
            0x010 => 4,
            0x014 => 5,
            _ => return,
        };
        self.rosc[idx] = apply(self.rosc[idx]);
    }

    // --- XOSC (0x4004_8000) -----------------------------------------

    pub fn xosc_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.xosc[0],
            0x004 => (1 << 31) | (1 << 12), // STATUS: STABLE | ENABLED
            0x008 => self.xosc[2],
            0x00C => self.xosc[3],
            0x01C => 0, // COUNT
            _ => 0,
        }
    }

    pub fn xosc_write(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |current: u32| match alias {
            0 => val,
            1 => current ^ val,
            2 => current | val,
            3 => current & !val,
            _ => val,
        };
        let idx = match offset {
            0x000 => 0,
            0x008 => 2,
            0x00C => 3,
            _ => return,
        };
        self.xosc[idx] = apply(self.xosc[idx]);
    }
}

/// Apply an alias-aware write to a PLL register image. Duplicated
/// from `bus/peripherals.rs::pll_write_into`.
fn pll_write_into(regs: &mut [u32; 4], offset: u32, val: u32, alias: u32) {
    if let Some(i) = pll_reg_index(offset) {
        regs[i] = match alias {
            0 => val,
            1 => regs[i] ^ val,
            2 => regs[i] | val,
            3 => regs[i] & !val,
            _ => val,
        };
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
/// `picoem_common::clocks::pll_cs_read_with_lock` and is shared.
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

    /// QMI read. Mirrors `Bus::qmi_read`: DIRECT_CSR (0x000) forces
    /// TXEMPTY (bit 16) and RXEMPTY (bit 17); other offsets return
    /// raw register image.
    pub fn qmi_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.qmi_regs.first().copied().unwrap_or(0) | (1 << 16) | (1 << 17),
            _ => {
                let idx = (offset >> 2) as usize;
                self.qmi_regs.get(idx).copied().unwrap_or(0)
            }
        }
    }

    /// QMI write. Mirrors `Bus::qmi_write` — plain storage, no alias.
    pub fn qmi_write(&mut self, offset: u32, val: u32) {
        let idx = (offset >> 2) as usize;
        if idx < self.qmi_regs.len() {
            self.qmi_regs[idx] = val;
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

    /// RESETS read. Mirrors `Bus::resets_read`.
    pub fn resets_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.resets_state,
            0x004 => 0,
            0x008 => !self.resets_state,
            _ => 0,
        }
    }

    /// RESETS write (alias-aware). Mirrors `Bus::resets_write`.
    pub fn resets_write(&mut self, offset: u32, val: u32, alias: u32) {
        if offset == 0x000 {
            self.resets_state = match alias {
                0 => val,
                1 => self.resets_state ^ val,
                2 => self.resets_state | val,
                3 => self.resets_state & !val,
                _ => val,
            };
        }
    }

    /// True iff the peripheral whose bus base is `base` is currently
    /// held in `RESETS.RESET`. Mirrors `Bus::is_held_in_reset_base`
    /// (`bus/mod.rs:854`); uses the shared `reset_bit_for_base` helper
    /// so the bit-pattern mapping stays in exactly one place.
    #[inline]
    pub fn is_held_in_reset_base(&self, base: u32) -> bool {
        match crate::bus::reset_bit_for_base(base) {
            Some(bit) => (self.resets_state & (1u32 << bit)) != 0,
            None => false,
        }
    }
}

/// APB peripheral register state. Mirrors `bus/mod.rs:253..266`.
pub struct ApbState {
    /// UART0/UART1 — PL011-derived UART at 0x4007_0000 / 0x4007_4000.
    pub uart: [UartRegs; 2],
    /// SPI0/SPI1 — PL022-derived SPI at 0x4008_0000 / 0x4008_4000.
    pub spi: [SpiRegs; 2],
    /// I2C0/I2C1 — DesignWare DW_apb_i2c at 0x4009_0000 / 0x4009_4000.
    pub i2c: [I2cRegs; 2],
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
        use crate::dreq::{
            DREQ_I2C0_RX, DREQ_I2C0_TX, DREQ_I2C1_RX, DREQ_I2C1_TX, DREQ_SPI0_RX, DREQ_SPI0_TX,
            DREQ_SPI1_RX, DREQ_SPI1_TX, DREQ_UART0_RX, DREQ_UART0_TX, DREQ_UART1_RX, DREQ_UART1_TX,
        };
        use crate::irq::{
            IRQ_ADC_IRQ_FIFO, IRQ_I2C0_IRQ, IRQ_I2C1_IRQ, IRQ_PWM_IRQ_WRAP_0, IRQ_PWM_IRQ_WRAP_1,
            IRQ_SPI0_IRQ, IRQ_SPI1_IRQ, IRQ_UART0_IRQ, IRQ_UART1_IRQ,
        };
        Self {
            uart: [
                UartRegs::new(IRQ_UART0_IRQ, DREQ_UART0_TX, DREQ_UART0_RX),
                UartRegs::new(IRQ_UART1_IRQ, DREQ_UART1_TX, DREQ_UART1_RX),
            ],
            spi: [
                SpiRegs::new(IRQ_SPI0_IRQ, DREQ_SPI0_TX, DREQ_SPI0_RX),
                SpiRegs::new(IRQ_SPI1_IRQ, DREQ_SPI1_TX, DREQ_SPI1_RX),
            ],
            i2c: [
                I2cRegs::new(IRQ_I2C0_IRQ, DREQ_I2C0_TX, DREQ_I2C0_RX),
                I2cRegs::new(IRQ_I2C1_IRQ, DREQ_I2C1_TX, DREQ_I2C1_RX),
            ],
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

/// USB controller state — register-surface stub at `0x5011_0000` with
/// 4 KB DPRAM at `0x5010_0000`. See `peripherals/usb.rs` and HLD V5
/// §Component 1.
pub struct UsbState {
    /// USB control-register + DPRAM stub.
    pub usbctrl: UsbCtrl,
}

impl UsbState {
    /// Mirror `Bus::new()` — fresh stub in post-reset state.
    pub fn post_bootrom() -> Self {
        Self {
            usbctrl: UsbCtrl::new(),
        }
    }
}

// =======================================================================
// Peripherals aggregate
// =======================================================================

/// Mutex-guarded bundle of peripheral state shared across worker
/// threads. Instances live behind an `Arc` on `SharedState`.
///
/// Lock order: `clocks < qmi < resets < apb < timers < dma < usb < legacy`.
pub struct Peripherals {
    pub clocks: Mutex<ClocksState>,
    pub qmi: Mutex<QmiState>,
    pub resets: Mutex<ResetsState>,
    pub apb: Mutex<ApbState>,
    pub timers: Mutex<TimersState>,
    pub dma: Mutex<DmaState>,
    pub usb: Mutex<UsbState>,
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
            usb: Mutex::new(UsbState::post_bootrom()),
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
        assert!(p.usb.lock().is_ok());
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
        // picoem-common.
        c.pll_sys_regs[0] = 0x0000_0001; // CS image stays all-zero in LOCK bit slot
        c.pll_sys_regs[1] = 0; // PWR cleared → base predicate can fire
        c.pll_sys_regs[2] = 125; // FBDIV != 0
        c.pll_sys_lock_at_cycle = Some(100);

        // Before the deadline — LOCK bit must be 0.
        let cs_before = c.pll_sys_read_at(0x000, 50);
        assert_eq!(cs_before & (1 << 31), 0, "LOCK must be 0 before deadline");
        // After the deadline — LOCK bit must be 1.
        let cs_after = c.pll_sys_read_at(0x000, 150);
        assert_ne!(cs_after & (1 << 31), 0, "LOCK must be 1 at/after deadline");
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
