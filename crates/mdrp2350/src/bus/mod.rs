pub mod clocks;
pub mod peripherals;
pub mod ppb;

use std::collections::HashMap;
use std::io::Write;
use std::sync::Arc;

use tracing::debug;

use crate::bus::clocks::{ClockTree, ROSC_FREQ_HZ, XOSC_FREQ_HZ, pll_output_hz};
use crate::threaded::CoreAtomics;
use crate::dma::{DMA_BASE, Dma};
use crate::dreq::{
    DREQ_I2C0_RX, DREQ_I2C0_TX, DREQ_I2C1_RX, DREQ_I2C1_TX, DREQ_SPI0_RX, DREQ_SPI0_TX,
    DREQ_SPI1_RX, DREQ_SPI1_TX, DREQ_UART0_RX, DREQ_UART0_TX, DREQ_UART1_RX, DREQ_UART1_TX,
};
use crate::irq::{
    IRQ_ADC_IRQ_FIFO, IRQ_I2C0_IRQ, IRQ_I2C1_IRQ, IRQ_PWM_IRQ_WRAP_0,
    IRQ_PWM_IRQ_WRAP_1, IRQ_SPI0_IRQ, IRQ_SPI1_IRQ, IRQ_TIMER0_IRQ_0, IRQ_TIMER1_IRQ_0,
    IRQ_UART0_IRQ, IRQ_UART1_IRQ, PERIPH_IRQ_MASK,
};
use crate::memory::{Memory, SRAM_SIZE, bank_for_address};
use crate::peripherals::adc::{ADC_BASE, AdcRegs};
use crate::peripherals::coresight_trace::{CORESIGHT_TRACE_BASE, CoresightTraceRegs};
use crate::peripherals::i2c::{I2C0_BASE, I2C1_BASE, I2cRegs};
use crate::peripherals::inert::{GLITCH_DETECTOR_BASE, GlitchDetector, SYSCFG_BASE, SysCfg, TBMAN_BASE, Tbman};
use crate::peripherals::io_bank0::{IO_BANK0_BASE, IoBank0Regs};
use crate::peripherals::otp::{OTP_DATA_BASE, OTP_DATA_SIZE, Otp};
use crate::peripherals::pads_bank0::{PADS_BANK0_BASE, PadsBank0Regs};
use crate::peripherals::powman::{POWMAN_BASE, PowmanRegs};
use crate::peripherals::psm::{PSM_BASE, Psm};
use crate::peripherals::pwm::{PWM_BASE, PwmRegs};
use crate::peripherals::sha256::{SHA256_BASE, Sha256Regs};
use crate::peripherals::spi::{SPI0_BASE, SPI1_BASE, SpiRegs};
use crate::peripherals::ticks::{TICKS_BASE, TicksRegs};
use crate::peripherals::timer::{TIMER0_BASE, TIMER1_BASE, TimerRegs};
use crate::peripherals::trng::{TRNG_BASE, Trng};
use crate::peripherals::uart::{UART0_BASE, UART1_BASE, UartRegs};
use crate::peripherals::watchdog::{WATCHDOG_BASE, WatchdogRegs};
use crate::pio::PioBlock;
use crate::sio::Sio;

/// Number of entries in the PC-keyed decoded-op cache.
/// Direct-mapped, indexed by `(pc >> 1) & (DECODE_CACHE_SIZE - 1)`.
/// See HLD `2026.04.14 - HLD - Decoded-Op Cache.md` §3.
pub(crate) const DECODE_CACHE_SIZE: usize = 16384;

/// One decoded instruction. 12 bytes. `Copy`.
///
/// Populated lazily on a cache miss by `CortexM33::populate_decode_cache`.
/// An entry with `tag == u32::MAX` is empty (that value is odd and cannot
/// match a halfword-aligned PC).
///
/// See HLD §2.
#[derive(Clone, Copy, Debug)]
pub(crate) struct DecodedOp {
    /// PC this entry is valid for. Full tag (no shift). `u32::MAX` = empty.
    pub tag: u32,
    /// First halfword (the one at PC).
    pub hw0: u16,
    /// Second halfword (at PC+2). Zero for narrow instructions.
    pub hw1: u16,
    /// Extra wait states the bus charged when these halfwords were last
    /// fetched (bank 2/6 SRAM `+1`; other regions 0). Max observed = 2
    /// (wide instruction straddling bank 2/6). Replayed on every hit so
    /// the fast path matches the non-cached cycle count exactly.
    pub fetch_wait: u8,
    /// Packed flags.
    ///   bit 0 — `is_wide`
    ///   bit 1 — `is_pure` (handler does not touch the bus wait-state
    ///           accumulator nor raise a synchronous fault)
    ///   bit 2 — `is_thumb16_flag_only` (CMP/CMN/TST — always set flags
    ///           even inside an IT block; pre-computed to avoid a nested
    ///           match on every narrow-in-IT execution)
    ///   bits 3..7 — reserved
    pub flags: u8,
}

impl DecodedOp {
    pub(crate) const FLAG_WIDE: u8 = 0b0000_0001;
    pub(crate) const FLAG_PURE: u8 = 0b0000_0010;
    pub(crate) const FLAG_FLAG_ONLY: u8 = 0b0000_0100;

    #[inline(always)]
    pub(crate) fn empty() -> Self {
        Self { tag: u32::MAX, hw0: 0, hw1: 0, fetch_wait: 0, flags: 0 }
    }

    #[inline(always)]
    pub(crate) fn is_wide(&self) -> bool {
        self.flags & Self::FLAG_WIDE != 0
    }

    #[inline(always)]
    pub(crate) fn is_pure(&self) -> bool {
        self.flags & Self::FLAG_PURE != 0
    }

    #[inline(always)]
    pub(crate) fn is_flag_only(&self) -> bool {
        self.flags & Self::FLAG_FLAG_ONLY != 0
    }
}

/// True if `pc` lies in an executable region the cache may index.
/// Only ROM (0x0), XIP/XIP-SRAM (0x1), and SRAM (0x2) qualify. Other
/// regions either cannot legitimately contain code or are dynamic and
/// not worth caching.
#[inline(always)]
pub(crate) fn is_cacheable_pc(pc: u32) -> bool {
    matches!(canon_oracle_addr(pc) >> 28, 0x0 | 0x1 | 0x2)
}

/// Region bits for [`Bus::pending_invalidation_regions`] /
/// [`crate::core::CortexM33::invalidate_decode_cache_regions`]. The
/// meaningful regions are the three cacheable ones (ROM / XIP / SRAM
/// — see [`is_cacheable_pc`]). `REGION_BULK` is the universal-bulk
/// escape hatch used when the caller doesn't know (or doesn't care)
/// which region changed.
pub mod invalidation_regions {
    /// Region `0x0` — ROM (the 32 KB bootrom).
    pub const ROM: u8 = 1 << 0;
    /// Region `0x1` — XIP / XIP-SRAM (flash window).
    pub const XIP: u8 = 1 << 1;
    /// Region `0x2` — on-chip SRAM (all 10 banks).
    pub const SRAM: u8 = 1 << 2;
    /// Bulk bit — drain every slot regardless of tag region. Used when
    /// the caller can't attribute the change to a specific region
    /// (e.g. `Emulator::poke`, legacy code paths, or an `ISB` that
    /// must flush the entire decode pipeline).
    pub const BULK: u8 = 1 << 7;
}

// --- RESETS bit assignments (RP2350 datasheet §7.5 Table 486) ----------
//
// RP2350 RESETS.RESET is a 29-bit field (bits 0..=28); bits 29..=31 are
// RAZ/WI. Constants here are cross-checked against
// `target/tmp/src_clone/one-rom/sdrr/include/reg-rp235x.h` (which
// quotes the relevant subset from datasheet Table 486). Only the
// peripherals Phase 1+ actually model are named; extend as each new
// peripheral lands.
//
// TICKS is **not** gated by RESETS — the tick generator is a bus-level
// block that silicon does not put behind a reset line. Pico-SDK runtime
// init programs TICKS before any TIMER use without touching RESETS for
// it.

/// Canonicalise a QEMU-RV32 oracle address into the native RP2350 map.
///
/// QEMU `virt` places SRAM at `0x8000_0000`; RP2350 silicon places SRAM
/// at `0x2000_0000`. Firmware exercised by the RV32 fuzz oracle issues
/// `0x8xxx_xxxx` addresses; this helper remaps region 0x8 onto the
/// existing SRAM backing so both sides of the oracle resolve to the
/// same bytes. No-op outside region 0x8.
#[inline(always)]
pub fn canon_oracle_addr(addr: u32) -> u32 {
    if (addr >> 28) == 0x8 {
        (addr & 0x0FFF_FFFF) | 0x2000_0000
    } else {
        addr
    }
}

/// RESETS bit for ADC (datasheet §7.5).
pub const RESET_ADC: u8 = 0;
/// RESETS bit for DMA.
pub const RESET_DMA: u8 = 2;
/// RESETS bit for HSTX.
pub const RESET_HSTX: u8 = 3;
/// RESETS bit for I2C0 (RP2350 datasheet §7.5 Table 486).
pub const RESET_I2C0: u8 = 4;
/// RESETS bit for I2C1.
pub const RESET_I2C1: u8 = 5;
/// RESETS bit for IO_BANK0.
pub const RESET_IO_BANK0: u8 = 6;
/// RESETS bit for PADS_BANK0.
pub const RESET_PADS_BANK0: u8 = 9;
/// RESETS bit for PIO0.
pub const RESET_PIO0: u8 = 11;
/// RESETS bit for PIO1.
pub const RESET_PIO1: u8 = 12;
/// RESETS bit for PIO2.
pub const RESET_PIO2: u8 = 13;
/// RESETS bit for PLL_SYS.
pub const RESET_PLL_SYS: u8 = 14;
/// RESETS bit for PLL_USB.
pub const RESET_PLL_USB: u8 = 15;
/// RESETS bit for PWM.
pub const RESET_PWM: u8 = 16;
/// RESETS bit for POWMAN (RP2350 datasheet §2.14.3 RESETS_RESET table —
/// slot between RESET_PWM = 16 and RESET_SPI0 = 18).
pub const RESET_POWMAN: u8 = 17;
/// RESETS bit for SPI0.
pub const RESET_SPI0: u8 = 18;
/// RESETS bit for SPI1.
pub const RESET_SPI1: u8 = 19;
/// RESETS bit for SYSCFG.
pub const RESET_SYSCFG: u8 = 20;
/// RESETS bit for SYSINFO.
pub const RESET_SYSINFO: u8 = 21;
/// RESETS bit for TIMER0 (datasheet §7.5, `RESET_TIMER0` in
/// `reg-rp235x.h`).
pub const RESET_TIMER0: u8 = 23;
/// RESETS bit for TIMER1 (datasheet §7.5; alphabetical slot after
/// TIMER0 / TBMAN).
pub const RESET_TIMER1: u8 = 24;
/// RESETS bit for UART0.
pub const RESET_UART0: u8 = 26;
/// RESETS bit for UART1.
pub const RESET_UART1: u8 = 27;
/// RESETS bit for USBCTRL.
pub const RESET_USBCTRL: u8 = 28;

/// Post-bootrom `RESETS.RESET` state — peripherals released by pico-sdk
/// `runtime_init_bootrom_reset`. See HLD V5 §5.7.
///
/// Held-in-reset bits (bits 0..=28 minus the released set) cover
/// OTP, SHA256, TRNG, GLITCH_DETECTOR, POWMAN — blocks the emulator
/// does not model; keeping them held means firmware that accidentally
/// pokes those windows gets 0/noop via the Bus-level guard rather
/// than the HashMap fallthrough.
pub const RESETS_POST_BOOTROM: u32 = {
    let released = (1u32 << RESET_PLL_SYS)
        | (1u32 << RESET_PLL_USB)
        | (1u32 << RESET_IO_BANK0)
        | (1u32 << RESET_PADS_BANK0)
        | (1u32 << RESET_TIMER0)
        | (1u32 << RESET_TIMER1)
        | (1u32 << RESET_SYSCFG)
        | (1u32 << RESET_SYSINFO)
        // Phase 2 peripherals: V5 §5.7 lists these as post-bootrom
        // released (pico-sdk `runtime_init_bootrom_reset` covers them).
        | (1u32 << RESET_UART0)
        | (1u32 << RESET_SPI0)
        | (1u32 << RESET_I2C0)
        | (1u32 << RESET_ADC)
        | (1u32 << RESET_PWM)
        // Phase 3: DMA released post-bootrom.
        | (1u32 << RESET_DMA);
    // Field width is 29 bits (datasheet §7.5).
    let mask: u32 = 0x1FFF_FFFF;
    mask & !released
};

/// Maps a peripheral base address to the RESETS bit gating it. Used
/// by [`Bus::is_held_in_reset_base`] to inline the RESETS guard on
/// `read32` / `write32` dispatch (HLD V5 §5.3, inline, no separate
/// `peripheral_dispatch.rs` file).
///
/// Peripherals not listed here are either not reset-gated (SIO, PPB,
/// memory, TICKS) or not yet modelled at the Bus level.
#[inline]
pub(crate) fn reset_bit_for_base(base: u32) -> Option<u8> {
    match base {
        TIMER0_BASE => Some(RESET_TIMER0),
        TIMER1_BASE => Some(RESET_TIMER1),
        UART0_BASE => Some(RESET_UART0),
        UART1_BASE => Some(RESET_UART1),
        SPI0_BASE => Some(RESET_SPI0),
        SPI1_BASE => Some(RESET_SPI1),
        I2C0_BASE => Some(RESET_I2C0),
        I2C1_BASE => Some(RESET_I2C1),
        ADC_BASE => Some(RESET_ADC),
        PWM_BASE => Some(RESET_PWM),
        IO_BANK0_BASE => Some(RESET_IO_BANK0),
        PADS_BANK0_BASE => Some(RESET_PADS_BANK0),
        DMA_BASE => Some(RESET_DMA),
        SYSCFG_BASE => Some(RESET_SYSCFG),
        POWMAN_BASE => Some(RESET_POWMAN),
        _ => None,
    }
}

/// Bus fabric — address decode and cycle accounting.
///
/// Phase 1: flat memory, single-cycle access everywhere.
/// Phase 2 adds AHB5 arbitration, APB bridge latency, bus contention.
pub struct Bus {
    pub memory: Memory,
    /// Total cycles of the most recent bus access (for testing/debug).
    pub(crate) last_access_cycles: u32,
    /// Accumulated extra wait states beyond 1-cycle baseline during current instruction.
    /// Reset by decode_execute before dispatch, added to cycle count after.
    pub(crate) extra_wait_states: u32,
    /// Stub backing store for peripheral registers (APB + AHB).
    /// Keyed by canonical address (alias bits stripped).
    /// TODO: Replace with direct Peripheral trait dispatch when real peripherals are added.
    pub(crate) peripheral_regs: HashMap<u32, u32>,
    /// Cross-core atomics (halted/WFE/event_flag/irq_pending/RCP/bus_fault).
    /// Shared via `Arc` with the two `CortexM33` cores — Phase 3 Stage 1
    /// (LLD V7 §2). In the single-threaded path, Bus is the sole owner
    /// of the inner state; the threaded runtime clones this `Arc` onto
    /// `SharedState` and the CPU workers.
    pub atomics: Arc<CoreAtomics>,
    /// RESETS peripheral state: bits set = peripheral in reset.
    /// Default [`RESETS_POST_BOOTROM`] — peripherals released by
    /// pico-sdk `runtime_init_bootrom_reset` per HLD V5 §5.7.
    /// `Emulator::reset` restores this value. The underlying RP2350
    /// hardware-reset value is 0x1FFF_FFFF (all 29 peripherals held),
    /// but the emulator starts from post-bootrom state because
    /// `load_image` bypasses the bootrom.
    pub resets_state: u32,
    /// TICKS block (HLD V5 §5.4). Six-domain 1 µs tick generator.
    pub(crate) ticks: TicksRegs,
    /// TIMER0 — 64-bit microsecond counter + four alarms (HLD V5 §5.4).
    pub(crate) timer0: TimerRegs,
    /// TIMER1 — same shape as TIMER0, driven by the TIMER1 TICKS domain.
    pub(crate) timer1: TimerRegs,
    /// UART0/1 — PL011-derived UARTs at `0x4007_0000` / `0x4007_4000`
    /// (HLD V5 §6 row 2). Indexed `[0]=UART0`, `[1]=UART1`.
    pub(crate) uart: [UartRegs; 2],
    /// SPI0/1 — PL022-derived SPIs at `0x4008_0000` / `0x4008_4000`.
    pub(crate) spi: [SpiRegs; 2],
    /// I2C0/1 — DesignWare DW_apb_i2c at `0x4009_0000` / `0x4009_4000`.
    pub(crate) i2c: [I2cRegs; 2],
    /// ADC — single instance at `0x400A_0000`.
    pub(crate) adc: AdcRegs,
    /// PWM — 12-slice block at `0x4005_0000`.
    pub(crate) pwm: PwmRegs,
    /// IO_BANK0 plain-storage GPIO control (HLD V5 §5.8).
    pub(crate) io_bank0: IoBank0Regs,
    /// PADS_BANK0 plain-storage pad drive/pull control.
    pub(crate) pads_bank0: PadsBank0Regs,
    /// DMA controller — 16 channels (HLD V5 §5.6, Phase 3).
    pub(crate) dma: Dma,
    /// SYSCFG — storage-only inert peripheral (HLD V5 §7.D.1).
    pub(crate) syscfg: SysCfg,
    /// TBMAN — storage-only inert peripheral.
    pub(crate) tbman: Tbman,
    /// GLITCH_DETECTOR — ARM RW-with-0x5bad-reset + TRIG_STATUS reads-as-zero / W1C.
    pub(crate) glitch: GlitchDetector,
    /// PSM — power-on state machine (HLD V5 §7.D.2). Instant handshake
    /// model: `DONE` mirrors `FRCE_ON`.
    pub(crate) psm: Psm,
    /// WATCHDOG — countdown with reset trigger (HLD V5 §7.D.3).
    pub(crate) watchdog: WatchdogRegs,
    /// OTP — 16 KB fuse array, OR-only writes (HLD V5 §7.D.4).
    pub(crate) otp: Otp,
    /// TRNG — 32-bit counter model (HLD V5 §7.D.5).
    pub(crate) trng: Trng,
    /// SHA-256 — 16-word block compressor (HLD V5 §7.D.6).
    pub(crate) sha256: Sha256Regs,
    /// POWMAN — AON timer + VREG + ARCHSEL storage (HLD V5 §8.E.1).
    /// No tick advancement; warn-once on COUNT/MATCH/non-Arm ARCHSEL.
    pub(crate) powman: PowmanRegs,
    /// CORESIGHT_TRACE — storage-only CoreSight block at `0xE004_1000`
    /// (HLD V5 §8.E.2). No trace data produced.
    pub(crate) coresight_trace: CoresightTraceRegs,
    /// Warn-once latch for `CLOCKS.CLK_*_CTRL.ENABLE` clear (HLD V5
    /// §4.A2 site 9). Keyed on CLOCKS offset of the CTRL register.
    pub(crate) warned_clk_enable_clear: std::collections::HashSet<u32>,
    /// Word-aligned MMIO addresses for which an "unmodelled access" warn
    /// has already been emitted. HLD V5 §4.A1 — warn once per address,
    /// per `Bus` instance, so firmware or tests that hammer an
    /// unmodelled peripheral don't drown the trace.
    pub(crate) warned_addrs: std::collections::HashSet<u32>,
    /// Watchdog reset requested flag (HLD V5 §7.D.3). Set by the
    /// WATCHDOG peripheral when its countdown fires; polled by the CPU
    /// step loop (wiring lands with the WATCHDOG peripheral itself).
    /// Kept here so that landing order §10 step 3a does not re-touch
    /// `Bus`.
    pub(crate) watchdog_reset_requested: bool,
    /// Whether flash (XIP) content has been loaded.
    pub(crate) flash_loaded: bool,
    /// Suppress per-word SRAM bank wait states during burst transfers
    /// (STM/LDM/PUSH/POP). The SRAM controller handles sequential word
    /// accesses without per-word bank penalties.
    pub(crate) burst_mode: bool,
    /// Address of the most recently fetched instruction. Used to determine
    /// whether the current fetch is sequential (prefetch buffer absorbs
    /// bank penalty) or non-sequential (bank penalty applies).
    /// Initialized to `u32::MAX` so the first fetch is non-sequential.
    pub(crate) last_fetch_addr: u32,
    /// 4 KB boot RAM at 0xEFFF_F000..0xF000_0000.
    /// RP2350 maps this as the secure boot stack (USB DPRAM secure alias).
    /// Initial SP = 0xF000_0000 (top of this region).
    pub(crate) boot_ram: Box<[u8; 4096]>,
    /// 16 KB XIP SRAM at 0x1C00_0000..0x1C00_3FFF.
    /// RP2350 XIP cache memory accessible as SRAM.
    pub(crate) xip_sram: Box<[u8; 16384]>,
    /// QMI register backing store (offsets 0x000..0x06C, 28 words).
    pub(crate) qmi_regs: [u32; 28],
    /// CLK_REF_CTRL register (CLOCKS offset 0x030).
    pub(crate) clk_ref_ctrl: u32,
    /// CLK_SYS_CTRL register (CLOCKS offset 0x060).
    pub(crate) clk_sys_ctrl: u32,
    /// CLK_SYS_DIV register (CLOCKS offset 0x064).
    /// [31:16] integer divider (0 treated as 1), [15:0] fractional (ignored).
    /// Reset value 0x0001_0000 = integer div 1.
    pub(crate) clk_sys_div: u32,
    /// Derived clock-tree frequencies. Recomputed after each write to
    /// CLK_REF_CTRL / CLK_SYS_CTRL / CLK_SYS_DIV or any PLL_SYS /
    /// PLL_USB register.
    pub(crate) clock_tree: ClockTree,
    /// PLL_SYS register image: `[CS, PWR, FBDIV_INT, PRIM]` at offsets
    /// `0x000`, `0x004`, `0x008`, `0x00C` respectively. Reset values
    /// per LLD V2 §4.3: CS=0x01 (REFDIV=1), PWR=0x2D (powered down),
    /// FBDIV=0 (PLL off → `pll_output_hz` returns 0), PRIM=0x77000
    /// (POSTDIV1=7, POSTDIV2=7).
    pub(crate) pll_sys_regs: [u32; 4],
    /// PLL_USB register image — same layout and reset values as
    /// `pll_sys_regs`. Separate storage so configuring one PLL does
    /// not affect the other.
    pub(crate) pll_usb_regs: [u32; 4],
    /// Master cycle count at the start of the current step. Populated by
    /// `Emulator::step` / `Emulator::run` before any core dispatch so that
    /// PLL CS reads and write-time lock-arm transitions observe a fresh
    /// cycle. See `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 P2.
    pub(crate) master_cycle: u64,
    /// Master cycle at which PLL_SYS's lock-detect counter expires. `None`
    /// means the PLL is not currently armed (powered down, unconfigured,
    /// or hasn't been powered up yet). Managed by `pll_sys_write` via
    /// `mdpicoem_common::clocks::pll_should_arm_lock`.
    pub(crate) pll_sys_lock_at_cycle: Option<u64>,
    /// Master cycle at which PLL_USB's lock-detect counter expires. Same
    /// semantics as `pll_sys_lock_at_cycle`.
    pub(crate) pll_usb_lock_at_cycle: Option<u64>,
    /// ROSC register image (LLD V2 §4.11). Indices map to offsets:
    /// `0=CTRL (0x000)`, `1=FREQA (0x004)`, `2=FREQB (0x008)`,
    /// `3=RANDOM (0x00C)`, `4=DORMANT (0x010)`, `5=DIV (0x014)`,
    /// `6=STATUS (0x018)`, `7=RANDOMBIT (0x01C)`, `8=COUNT (0x020)`.
    /// Storage-only — none of these affect the fixed 6.5 MHz ROSC
    /// output. Read-only offsets (RANDOM, STATUS, RANDOMBIT, COUNT)
    /// return synthesised values and ignore writes.
    pub(crate) rosc_regs: [u32; 9],
    /// XOSC register image (LLD V2 §4.12). Indices map to offsets:
    /// `0=CTRL (0x000)`, `1=STATUS (0x004)`, `2=DORMANT (0x008)`,
    /// `3=STARTUP (0x00C)`, `4=COUNT (0x01C)`.
    /// Storage-only; STATUS and COUNT are read-only.
    pub(crate) xosc_regs: [u32; 5],
    /// SIO GPIO_HI_IN (offset 0x008). Upper QSPI GPIO pins.
    /// When flash is loaded, returns pseudo-random noise to simulate
    /// QSPI pin activity (the bootrom samples this to detect flash).
    pub(crate) gpio_hi_noise_state: u32,
    /// XIP cache window offset: maps XIP SRAM reads (0x1C00_0000)
    /// to flash content at this byte offset. Set by QMI M0_RFMT writes.
    pub(crate) xip_cache_offset: u32,
    /// Single-cycle IO block (GPIO, CPUID, spinlocks, FIFO, divider, etc.).
    pub sio: Sio,
    /// Three PIO blocks (PIO0, PIO1, PIO2).
    pub pio: [PioBlock; 3],
    /// Combined GPIO pin state (readable by SIO and PIO).
    pub gpio_in: u32,
    /// External-input stimulus value. Bits selected by
    /// [`Self::gpio_external_mask`] are forced to the corresponding
    /// bits of this value after `update_gpio` merges SIO/PIO outputs.
    /// Lets the harness drive pins (CS, address bus, etc.) that the
    /// emulator otherwise recomputes every tick. Defaults to 0.
    pub gpio_external_in: u32,
    /// External-input stimulus mask. Bit `i` set = the harness dictates
    /// `gpio_in[i]`; bit `i` clear = PIO/SIO dictates. Defaults to 0
    /// (no stimulus — legacy behaviour).
    pub gpio_external_mask: u32,
    /// Dirty-range log for per-core decode caches. Every SRAM / ROM /
    /// XIP write pushes the target halfword address(es) here; the driver
    /// (`Emulator::step` in the single-threaded path) drains this into
    /// the core that just ran, evicting stale entries from its
    /// per-core decode cache. Mirrors the threaded
    /// `WorkerBus::pending_cache_invalidations` pattern (LLD V7 §9).
    ///
    /// Width-aware push: `write32` pushes `{addr, addr+2}`, `write16` /
    /// `write8` push `{addr}`. The drainer also evicts the slot at
    /// `addr - 2` so a wide instruction whose `hw1` is rewritten is
    /// evicted along with the narrow slot. Parity with the pre-migration
    /// `Bus::invalidate_pc_range` which cleared `{addr-2, addr [, addr+2]}`.
    pub pending_cache_invalidations: Vec<u32>,
    /// Region-scoped bulk-invalidation bitmask. Set by [`Self::load_bootrom`]
    /// (bit [`invalidation_regions::ROM`]), [`Self::load_flash`] (bit
    /// [`invalidation_regions::XIP`]), and [`Self::invalidate_all`] (bit
    /// [`invalidation_regions::BULK`]) when a write has replaced
    /// executable bytes wholesale. The driver drains the mask on the
    /// next observation by calling
    /// [`crate::core::CortexM33::invalidate_decode_cache_regions`] on
    /// each core and then resets it to `0`.
    ///
    /// Complements [`Self::pending_cache_invalidations`] — that Vec
    /// covers narrow writes observed per-instruction; this mask covers
    /// bulk loads that predate any `step()` call and can preserve slots
    /// outside the touched region (a `load_flash` no longer blows away
    /// SRAM cache entries — regression fixed after Task #10 review).
    pub pending_invalidation_regions: u8,
    /// MMIO trace toggle (see `wrk_docs/2026.04.15 - HLD - RP2350 Peripheral
    /// Coverage V5.md` §4 / §4.2.7). When `true`, each byte/half/word bus
    /// access emits one line to [`Self::mmio_trace_sink`] (defaults to stdout
    /// when `None`). Zero overhead when `false` — the hot path
    /// short-circuits before any formatting. Mirrors the mdrp2040 V7 idiom.
    pub mmio_trace_enabled: bool,
    /// Per-core, per-instruction PC snapshot. Indexed by the core id
    /// passed to `set_active_pc(pc, core)` so a core switch does not
    /// alias one core's decode PC onto the other. Set by the core's
    /// decode path (`CortexM33::decode_execute`) immediately before
    /// instruction fetch, so every read/write during that instruction
    /// carries the correct architectural PC. Also set to sentinel
    /// values (`0xFFFF_FFFE` / `0xFFFF_FFFD`) by `enter_exception` /
    /// `exit_exception` so stacking / unstacking lines are
    /// distinguishable from ordinary instruction-driven access.
    /// Default `[0, 0]`; only meaningful while a core is executing.
    pub(crate) active_pc: [u32; 2],
    /// Optional override sink for trace output. `None` routes to stdout
    /// via `println!`. Unit tests inject a `Vec<u8>`-backed sink to
    /// capture lines without wrestling with fd 1 redirection.
    pub(crate) mmio_trace_sink: Option<Box<dyn Write>>,
    /// Per-core LR/SC reservation (RV32A). `Some(addr)` holds a
    /// word-aligned SRAM address the core has reserved via `lr.w`; any
    /// write to that word by any master clears the corresponding
    /// reservation. See HLD §4.7.
    pub reservation: [Option<u32>; 2],
}

impl Bus {
    /// Construct a stand-alone `Bus` with its own `CoreAtomics`. The
    /// returned atomics are not shared with any `CortexM33`; callers
    /// that build an `Emulator` should use [`Bus::with_atomics`] to
    /// keep the Bus and cores in the same atomic state.
    pub fn new() -> Self {
        Self::with_atomics(Arc::new(CoreAtomics::default()))
    }

    /// Construct a `Bus` that shares the supplied `CoreAtomics` with
    /// the (to-be-constructed) `CortexM33` cores. Phase 3 Stage 1.
    pub fn with_atomics(atomics: Arc<CoreAtomics>) -> Self {
        // HLD V5 §5.7: construction alone produces post-bootrom state.
        // `Bus::new()`, `Emulator::new(...)`, and `Emulator::reset()` all
        // land on the same clock / RESETS / TICKS table, so `load_image`
        // firmware (which bypasses the bootrom) observes the same state
        // real silicon would see after pico-sdk `runtime_init_*`.
        use mdpicoem_common::clocks::{RP2350_SYS_CLK_HZ, XOSC_FREQ_HZ};
        let post_bootrom_tree = ClockTree {
            sys_clk_hz: RP2350_SYS_CLK_HZ,
            ref_clk_hz: XOSC_FREQ_HZ,
            peri_clk_hz: RP2350_SYS_CLK_HZ,
        };
        Self {
            memory: Memory::new(),
            last_access_cycles: 0,
            extra_wait_states: 0,
            peripheral_regs: HashMap::new(),
            resets_state: RESETS_POST_BOOTROM,
            ticks: TicksRegs::post_bootrom(),
            timer0: TimerRegs::new(IRQ_TIMER0_IRQ_0),
            timer1: TimerRegs::new(IRQ_TIMER1_IRQ_0),
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
            dma: Dma::new(),
            atomics,
            syscfg: SysCfg::new(),
            tbman: Tbman::new(),
            glitch: GlitchDetector::new(),
            psm: Psm::new(),
            watchdog: WatchdogRegs::new(),
            otp: Otp::new(),
            trng: Trng::new(),
            sha256: Sha256Regs::new(),
            powman: PowmanRegs::new(),
            coresight_trace: CoresightTraceRegs::new(),
            warned_clk_enable_clear: std::collections::HashSet::new(),
            warned_addrs: std::collections::HashSet::new(),
            watchdog_reset_requested: false,
            flash_loaded: false,
            burst_mode: false,
            last_fetch_addr: u32::MAX,
            boot_ram: Box::new([0u8; 4096]),
            xip_sram: Box::new([0u8; 16384]),
            qmi_regs: [0u32; 28],
            clk_ref_ctrl: 0,
            clk_sys_ctrl: 0,
            clk_sys_div: 0x0001_0000,
            clock_tree: post_bootrom_tree,
            pll_sys_regs: [0x0000_0001, 0x0000_002D, 0, 0x0007_7000],
            pll_usb_regs: [0x0000_0001, 0x0000_002D, 0, 0x0007_7000],
            master_cycle: 0,
            pll_sys_lock_at_cycle: None,
            pll_usb_lock_at_cycle: None,
            rosc_regs: [0u32; 9],
            xosc_regs: [0u32; 5],
            gpio_hi_noise_state: 0xA5A5_A5A5,
            xip_cache_offset: 0,
            sio: Sio::new(),
            pio: [PioBlock::new(), PioBlock::new(), PioBlock::new()],
            gpio_in: 0,
            gpio_external_in: 0,
            gpio_external_mask: 0,
            // Dirty-range log for per-core decode caches. 16 entries up
            // front — STM tops out at 13 registers, FPU context push
            // spills 16 words. Matches `WorkerBus::pending_cache_invalidations`.
            pending_cache_invalidations: Vec::with_capacity(16),
            pending_invalidation_regions: 0,
            mmio_trace_enabled: false,
            active_pc: [0; 2],
            mmio_trace_sink: None,
            reservation: [None, None],
        }
    }

    // --- Clock tree accessors (see bus/clocks.rs and LLD V2 §4) ---

    /// Current effective system clock frequency in Hz.
    ///
    /// Derived from CLK_SYS_CTRL / CLK_REF_CTRL / CLK_SYS_DIV and the
    /// PLL registers. The Pacer reads this after each quantum to follow
    /// firmware clock changes.
    pub fn sys_clk_hz(&self) -> u32 {
        self.clock_tree.sys_clk_hz
    }

    /// Seed the clock-tree frequencies (both `sys_clk_hz` and
    /// `ref_clk_hz`) without writing to any register. The first
    /// subsequent call to [`Self::recompute_clock_tree`] — triggered
    /// by any write to a CLOCKS or PLL register — overwrites the seed
    /// with the register-derived value.
    ///
    /// Used by `EmulatorBuilder::build` to forward a non-default
    /// `Config::sys_clk_hz` into the Bus as the vestigial seed
    /// (LLD V2 §4.9). `Bus::new` already installs the HLD V5 §5.7
    /// post-bootrom table, so only non-default configs need this
    /// override — hence the builder's call is conditional.
    pub fn seed_sys_clk_hz(&mut self, hz: u32) {
        self.clock_tree.sys_clk_hz = hz;
        self.clock_tree.ref_clk_hz = hz;
    }

    /// Seed the clock tree to the RP2350 post-bootrom state per HLD
    /// V5 §5.7: `clk_sys = 150 MHz`, `clk_ref = 12 MHz`,
    /// `clk_peri = clk_sys`. Idempotent with [`Self::new`], which
    /// already installs this table — called again from
    /// `Emulator::reset` so a reset that ran firmware first (and
    /// mutated the `ClockTree` via register writes) returns to a known
    /// baseline.
    ///
    /// A subsequent write to any CLOCKS / PLL register triggers
    /// [`Self::recompute_clock_tree`], which replaces these seeded
    /// values with register-derived ones — so firmware that actually
    /// reprograms the clock tree at boot still produces the right
    /// post-reprogram frequencies.
    ///
    /// `clk_adc` is not yet carried on `ClockTree`; when Phase 2 adds
    /// it, seed it here to `RP2350_ADC_CLK_HZ` (48 MHz).
    pub fn seed_post_bootrom_clocks(&mut self) {
        use mdpicoem_common::clocks::{RP2350_SYS_CLK_HZ, XOSC_FREQ_HZ};
        self.clock_tree.sys_clk_hz = RP2350_SYS_CLK_HZ;
        self.clock_tree.ref_clk_hz = XOSC_FREQ_HZ;
        self.clock_tree.peri_clk_hz = RP2350_SYS_CLK_HZ;
    }

    /// Current effective reference clock frequency in Hz.
    pub fn ref_clk_hz(&self) -> u32 {
        self.clock_tree.ref_clk_hz
    }

    /// Recompute `clock_tree.sys_clk_hz` / `ref_clk_hz` from the
    /// current CLOCKS and PLL register state. Called after any write
    /// to CLK_REF_CTRL / CLK_SYS_CTRL / CLK_SYS_DIV or any PLL_SYS /
    /// PLL_USB register.
    ///
    /// See LLD V2 §4.5 for the formulas.
    pub(crate) fn recompute_clock_tree(&mut self) {
        // --- ref_clk_hz -------------------------------------------------
        let ref_hz = match self.clk_ref_ctrl & 0x3 {
            0 => ROSC_FREQ_HZ,
            1 => match (self.clk_ref_ctrl >> 5) & 0x7 {
                0 => pll_output_hz(&self.pll_usb_regs), // aux: PLL_USB
                _ => 0,                                 // clksrc_gpin0/1 — unmodeled
            },
            2 => XOSC_FREQ_HZ,
            _ => ROSC_FREQ_HZ, // reserved — safe fallback
        };

        // --- sys_clk_hz -------------------------------------------------
        let sys_src_hz = match self.clk_sys_ctrl & 0x1 {
            0 => ref_hz, // clk_ref path
            _ => match (self.clk_sys_ctrl >> 5) & 0x7 {
                0 => pll_output_hz(&self.pll_sys_regs),
                1 => pll_output_hz(&self.pll_usb_regs),
                2 => ROSC_FREQ_HZ,
                3 => XOSC_FREQ_HZ,
                _ => 0, // clksrc_gpin0/1 — unmodeled
            },
        };

        // CLK_SYS_DIV[31:16] integer divider; 0 is reserved → treat as 1.
        let int_div = ((self.clk_sys_div >> 16) & 0xFFFF).max(1);
        let sys_hz = sys_src_hz / int_div;

        self.clock_tree.ref_clk_hz = ref_hz;
        self.clock_tree.sys_clk_hz = sys_hz;

        debug!(
            sys_clk_hz = sys_hz,
            ref_clk_hz = ref_hz,
            peri_clk_hz = self.clock_tree.peri_clk_hz,
            "clock tree recomputed",
        );
    }

    // --- XIP SRAM helpers (0x1C00_0000..0x1C00_3FFF) ---

    fn is_xip_sram(addr: u32) -> bool {
        addr >= 0x1C00_0000 && addr < 0x1C00_4000
    }

    fn xip_sram_read8(&self, addr: u32) -> u8 {
        self.xip_sram[(addr - 0x1C00_0000) as usize]
    }

    fn xip_sram_write8(&mut self, addr: u32, val: u8) {
        self.xip_sram[(addr - 0x1C00_0000) as usize] = val;
    }

    fn xip_sram_read16(&self, addr: u32) -> u16 {
        let off = (addr - 0x1C00_0000) as usize;
        u16::from_le_bytes([self.xip_sram[off], self.xip_sram[off + 1]])
    }

    fn xip_sram_write16(&mut self, addr: u32, val: u16) {
        let off = (addr - 0x1C00_0000) as usize;
        self.xip_sram[off..off + 2].copy_from_slice(&val.to_le_bytes());
    }

    fn xip_sram_read32(&self, addr: u32) -> u32 {
        let off = (addr - 0x1C00_0000) as usize;
        u32::from_le_bytes([
            self.xip_sram[off], self.xip_sram[off + 1],
            self.xip_sram[off + 2], self.xip_sram[off + 3],
        ])
    }

    fn xip_sram_write32(&mut self, addr: u32, val: u32) {
        let off = (addr - 0x1C00_0000) as usize;
        self.xip_sram[off..off + 4].copy_from_slice(&val.to_le_bytes());
    }

    // --- Boot RAM helpers (0xEFFF_F000..0xF000_0000) ---

    /// Check if address is in the 4KB boot RAM region.
    /// Returns true if `addr` lies in the CORESIGHT_TRACE aperture
    /// (`0xE004_1000..0xE004_2000`). 4 KB window — HLD V5 §8.E.2.
    #[inline]
    pub fn is_coresight_trace(addr: u32) -> bool {
        addr >= CORESIGHT_TRACE_BASE && addr < CORESIGHT_TRACE_BASE + 0x1000
    }

    pub fn is_boot_ram(addr: u32) -> bool {
        addr >= 0xEFFF_F000 && addr < 0xF000_0000
    }

    /// True if `reg_offset` (already masked to the SIO 12-bit window)
    /// is a GPIO_OUT-family register. Sub-word writes to these
    /// registers are replicated across all lanes on RP2350 silicon,
    /// rather than doing byte-lane RMW — see the `write8` SIO arm
    /// for the full rationale and datasheet reference.
    ///
    /// Covers GPIO_OUT (0x010), GPIO_OUT_SET/CLR/XOR (0x018/0x020/
    /// 0x028) and the OE mirrors GPIO_OE / GPIO_OE_SET/CLR/XOR
    /// (0x030/0x038/0x040/0x048). GPIO_HI_OUT mirrors are not
    /// modelled on mdrp2350 (see wrk_docs Phase 5 LLD §Known
    /// Limitations) so this helper does not include their offsets.
    #[inline]
    pub fn is_sio_gpio_out_replicating_reg(reg_offset: u32) -> bool {
        matches!(
            reg_offset,
            0x010 | 0x018 | 0x020 | 0x028 | 0x030 | 0x038 | 0x040 | 0x048
        )
    }

    fn boot_ram_read8(&self, addr: u32) -> u8 {
        let off = (addr - 0xEFFF_F000) as usize;
        self.boot_ram[off]
    }

    fn boot_ram_write8(&mut self, addr: u32, val: u8) {
        let off = (addr - 0xEFFF_F000) as usize;
        self.boot_ram[off] = val;
    }

    pub fn boot_ram_read32(&self, addr: u32) -> u32 {
        let off = (addr - 0xEFFF_F000) as usize;
        u32::from_le_bytes([
            self.boot_ram[off],
            self.boot_ram[off + 1],
            self.boot_ram[off + 2],
            self.boot_ram[off + 3],
        ])
    }

    pub fn boot_ram_write32(&mut self, addr: u32, val: u32) {
        let off = (addr - 0xEFFF_F000) as usize;
        let bytes = val.to_le_bytes();
        self.boot_ram[off..off + 4].copy_from_slice(&bytes);
    }

    fn boot_ram_read16(&self, addr: u32) -> u16 {
        let off = (addr - 0xEFFF_F000) as usize;
        u16::from_le_bytes([self.boot_ram[off], self.boot_ram[off + 1]])
    }

    fn boot_ram_write16(&mut self, addr: u32, val: u16) {
        let off = (addr - 0xEFFF_F000) as usize;
        let bytes = val.to_le_bytes();
        self.boot_ram[off..off + 2].copy_from_slice(&bytes);
    }

    // --- Bus arbitration ---

    /// Determine the downstream port ID for an address.
    /// Two addresses that return the same port ID will contend.
    /// Returns None for core-local ports (SIO, PPB) that never contend.
    pub fn downstream_port(addr: u32) -> Option<u8> {
        match addr >> 28 {
            0x0 => Some(0),  // ROM — single port
            0x1 => Some(1),  // XIP — single port
            0x2 => {
                // SRAM — per-bank ports
                match bank_for_address(addr) {
                    Some(bank) => Some(2 + bank), // ports 2-11
                    None => Some(2),              // out-of-range SRAM, treat as bank 0
                }
            }
            0x4 => Some(12), // APB bridge — single port
            0x5 => Some(13), // AHB peripherals — single port
            0xD => None,     // SIO — core-local, no contention
            0xE => None,     // PPB — core-local, no contention
            _ => Some(14),   // unmapped — treat as single port
        }
    }

    /// Check if a single core's access has any stall from contention.
    /// With only one core accessing, there's never contention.
    pub fn arbitrate_stall(&self, _core: u8, _addr: u32) -> u32 {
        0 // single core never stalls
    }

    /// Given two simultaneous accesses (core 0 and core 1), determine stall
    /// cycles for each. Core 0 has higher priority (wins ties).
    /// Returns (core0_stall, core1_stall).
    pub fn arbitrate_pair(&self, core0_addr: u32, core1_addr: u32) -> (u32, u32) {
        let port0 = Self::downstream_port(core0_addr);
        let port1 = Self::downstream_port(core1_addr);

        match (port0, port1) {
            (Some(p0), Some(p1)) if p0 == p1 => {
                // Same downstream port — core 1 stalls (core 0 wins)
                (0, 1)
            }
            _ => {
                // Different ports, or one/both are core-local — no contention
                (0, 0)
            }
        }
    }

    /// Stash the instruction PC of the currently-executing instruction
    /// on the specified core. Called by
    /// [`crate::core::CortexM33::decode_execute`] before instruction
    /// fetch so the MMIO trace can report a meaningful PC for every
    /// access that instruction performs. Also called by exception
    /// entry / exit with sentinel values (`0xFFFF_FFFE`,
    /// `0xFFFF_FFFD`) so stacking / unstacking lines are distinguishable
    /// from ordinary instruction-driven access. See HLD V5 §4.2.7.
    #[inline]
    pub fn set_active_pc(&mut self, pc: u32, core: u8) {
        self.active_pc[core as usize] = pc;
    }

    /// Emit a single trace line. `rw` is `'R'` or `'W'`, `size` is 1/2/4
    /// bytes, `val` is the value read or written. Called from the six
    /// outer bus access methods only when [`Self::mmio_trace_enabled`] is
    /// `true`; the caller gates with `if self.mmio_trace_enabled` so the
    /// formatting cost is paid only when tracing.
    ///
    /// Routes to [`Self::mmio_trace_sink`] if set, else `println!` (stdout).
    /// No buffering — each line flushes at the `writeln!` boundary.
    ///
    /// Coverage note (mirrors mdrp2040 V7 §4.3). The trace is emitted
    /// only from the six outer access methods ([`Self::read8`] …
    /// [`Self::write32`]). The internal peripheral dispatch helpers
    /// (`sysinfo_read`, `clocks_read/write`, `pll_sys_read/write`, PIO
    /// block `read32`/`write32`, SIO dispatch, PPB dispatch) are **only
    /// reachable** from those six methods — they have no other callers
    /// in the crate and are `pub(crate)`. So outer-only tracing covers
    /// 100% of the MMIO surface firmware can touch, at one line per
    /// architectural access. Hooking the inner helpers as well would
    /// double-emit on word-sized peripheral access and surface the
    /// byte/half RMW-through-word32 artefact on narrow peripheral
    /// access — neither of which helps the "what does firmware touch
    /// next?" workflow.
    ///
    /// `#[cold]` + `#[inline(never)]` keeps the cold path out of the
    /// caller's register allocation so the `if self.mmio_trace_enabled`
    /// fast-path stays branch-predicted-not-taken and decoded-op-cache
    /// hot paths are unaffected when tracing is off. This is the
    /// "V2 reverted to runtime flag" decision in V4's review history.
    #[cold]
    #[inline(never)]
    pub(crate) fn emit_mmio_trace(&mut self, rw: char, size: u32, addr: u32, val: u32, core: u8) {
        let line = format!(
            "TRACE {} {} 0x{:08X} val=0x{:08X} core={} pc=0x{:08X}",
            rw, size, addr, val, core as usize, self.active_pc[core as usize]
        );
        if let Some(sink) = self.mmio_trace_sink.as_mut() {
            let _ = writeln!(sink, "{}", line);
        } else {
            println!("{}", line);
        }
    }

    /// Install a captured trace sink (used by unit tests). `None` routes
    /// back to stdout. This is `pub(crate)` to keep it off the public
    /// surface — the binary toggles `mmio_trace_enabled` only.
    #[cfg(test)]
    pub(crate) fn set_mmio_trace_sink(&mut self, sink: Option<Box<dyn Write>>) {
        self.mmio_trace_sink = sink;
    }

    /// Assert an external IRQ at a specific core. `core` names the NVIC
    /// **receiver** — not the writer. Example: when core 1 writes the
    /// core-0-bound FIFO, SIO calls `bus.assert_irq_core(0, IRQ_SIO_IRQ_FIFO)`
    /// so the latch lands on core 0's pending mask. This matches the
    /// HLD V5 §5.3 direction and mirrors the mdrp2040 V7 pattern.
    ///
    /// **Contract**: this helper is for IRQs listed in
    /// [`crate::irq::CORE_LOCAL_IRQS`] — lines that are routed to one
    /// specific core by peripheral design (SIO per-core FIFO/BELL/MTIMECMP,
    /// GPIO bank-0, GPIO QSPI). For IRQs that should fire on both cores
    /// (every shared peripheral — TIMER, DMA, UART, SPI, I2C, PIO, etc.)
    /// use [`Self::assert_irq_shared`]. A `debug_assert!` sanity-checks
    /// that callers of this helper are targeting a core-local IRQ.
    ///
    /// Out-of-range arguments are silent no-ops:
    /// * `core >= 2` — only two cores exist on RP2350.
    /// * `irq >= IRQ_COUNT (52)` — NVIC has 52 inputs; asserting beyond
    ///   is a peripheral bug the emulator silently drops rather than
    ///   latching somewhere unexpected.
    ///
    /// The assert mirrors the pending bit into both `irq_pending[core]`
    /// (a test/observability side-channel) and the target core's
    /// NVIC_ISPR (the architectural latch the dispatch path walks).
    pub fn assert_irq_core(&mut self, core: usize, irq: u32) {
        debug_assert!(
            irq >= crate::irq::IRQ_COUNT || Self::is_core_local_irq(irq),
            "assert_irq_core called with shared IRQ {irq}; use assert_irq_shared(irq)"
        );
        if core < 2 && irq < crate::irq::IRQ_COUNT {
            // Phase 3 Stage 1: irq_pending moved onto `CoreAtomics`. The
            // non-zero return of `take_irq_pending` on the consumer side
            // replaces the dropped `irq_pending_dirty` flag.
            self.atomics.assert_irq(core, irq);
        }
    }

    /// Assert an external IRQ on every core for a shared peripheral line.
    /// Peripherals that do not route their IRQ to a specific core (every
    /// non-SIO / non-GPIO line on RP2350) call this so both NVICs see the
    /// pending bit and dispatch picks it up on whichever core has the
    /// lowest current execution priority.
    ///
    /// **Contract**: `irq` must NOT be in [`crate::irq::CORE_LOCAL_IRQS`].
    /// A `debug_assert!` guards that invariant; release builds silently
    /// latch on both cores.
    ///
    /// Out-of-range arguments are silent no-ops (see
    /// [`Self::assert_irq_core`]).
    pub fn assert_irq_shared(&mut self, irq: u32) {
        debug_assert!(
            !Self::is_core_local_irq(irq),
            "assert_irq_shared called with core-local IRQ {irq}; use assert_irq_core(core, irq)"
        );
        if irq < crate::irq::IRQ_COUNT {
            self.atomics.assert_irq_shared(irq);
        }
    }

    /// Clear a core-local IRQ's pending bit on one core. Mirror of
    /// [`Self::assert_irq_core`]. Peripherals call this when a level-
    /// triggered source de-asserts; they own the latch lifecycle.
    /// Out-of-range arguments are silent no-ops.
    ///
    /// Phase 0b.1 Commit B: no dirty-flag is set on clear. The forward
    /// merge is a union (`|=`), and a stale `nvic_ispr` bit does not
    /// re-fire on its own — only the dispatch path and explicit ICPR
    /// writes clear `nvic_ispr`. See the "dual-clear invariant" docs at
    /// `core/exceptions.rs::try_take_any_pending_exception`.
    pub fn clear_irq_core(&mut self, core: usize, irq: u32) {
        if core < 2 && irq < crate::irq::IRQ_COUNT {
            self.atomics.clear_irq(core, irq);
        }
    }

    /// Clear a shared IRQ's pending bit on both cores. Mirror of
    /// [`Self::assert_irq_shared`]. Out-of-range arguments are silent
    /// no-ops. No dirty-flag (see [`Self::clear_irq_core`]).
    pub fn clear_irq_shared(&mut self, irq: u32) {
        if irq < crate::irq::IRQ_COUNT {
            self.atomics.clear_irq(0, irq);
            self.atomics.clear_irq(1, irq);
        }
    }

    /// Internal: is this IRQ a core-local line? Used by the debug-assert
    /// guards on [`Self::assert_irq_core`] / [`Self::assert_irq_shared`].
    #[inline]
    fn is_core_local_irq(irq: u32) -> bool {
        let mut i = 0;
        while i < crate::irq::CORE_LOCAL_IRQS.len() {
            if crate::irq::CORE_LOCAL_IRQS[i] == irq {
                return true;
            }
            i += 1;
        }
        false
    }

    // --- RESETS guard / peripheral tick (HLD V5 §5.3 / §5.5) ------------

    /// True iff the peripheral whose bus base is `base` is currently
    /// held in `RESETS.RESET`. Called inline from `read32` / `write32`
    /// dispatch before routing to the peripheral module. HLD V5 §5.3.
    ///
    /// Returns `false` for unmapped bases — they fall through to the
    /// non-reset-gated HashMap / peripheral path.
    #[inline]
    pub(crate) fn is_held_in_reset_base(&self, base: u32) -> bool {
        match reset_bit_for_base(base) {
            Some(bit) => (self.resets_state & (1u32 << bit)) != 0,
            None => false,
        }
    }

    /// True iff the peripheral whose RESETS bit is `bit` is currently
    /// held. Used by the tick path to skip reset-held peripherals.
    #[inline]
    pub(crate) fn is_held_in_reset_bit(&self, bit: u8) -> bool {
        (self.resets_state & (1u32 << bit)) != 0
    }

    /// Advance every stateful peripheral by `sys_clks` system-clock
    /// cycles, then route any latched IRQs into the NVIC pending masks.
    /// Called at quantum end from [`crate::Emulator::step`] per HLD
    /// V5 §5.3 / §5.5.
    ///
    /// V5 does NOT gate this call — the prompt is explicit: "tick
    /// every cycle, unconditionally. A follow-up HLD will add a gate
    /// if `paced_bench_rp2350` regression exceeds the §9.8 threshold."
    pub(crate) fn tick_peripherals(&mut self, sys_clks: u32) {
        // TICKS runs unconditionally — there is no RESETS bit for the
        // tick generator (it is bus-level plumbing). Advance all six
        // domains; consumers (TIMER0/TIMER1/RISCV-MTIME) drain edges.
        self.ticks.advance_all(sys_clks);

        // MTIME (SIO §3.1.8) — drain RISCV TICKS edges and advance the
        // RISC-V platform timer. `Sio::tick_mtime_from_ticks` picks
        // between the edge count and `sys_clks` based on
        // `MTIME_CTRL.FULLSPEED`; it also gates on `MTIME_CTRL.EN`.
        // See HLD `2026.04.17 - HLD - Residual A.2.1 MTIME
        // WATCHDOG_TICK Fix.md`.
        let riscv_edges = self.ticks.take_riscv_edges();
        self.sio.tick_mtime_from_ticks(riscv_edges, sys_clks);

        // TIMER0 — advance microsecond counter by the edges accumulated
        // on the TIMER0 TICKS domain, poll alarms, route shared IRQ.
        if !self.is_held_in_reset_bit(RESET_TIMER0) {
            let edges = self.ticks.take_timer0_edges();
            if edges > 0 {
                self.timer0.advance_us(edges);
            }
            let bits = self.timer0.poll_alarms();
            self.raise_timer_irqs(bits);
        }

        // TIMER1 — same as TIMER0 against its own domain + IRQ base.
        if !self.is_held_in_reset_bit(RESET_TIMER1) {
            let edges = self.ticks.take_timer1_edges();
            if edges > 0 {
                self.timer1.advance_us(edges);
            }
            let bits = self.timer1.poll_alarms();
            self.raise_timer_irqs(bits);
        }

        // Phase 2 peripherals — each advances per sys_clk unless held
        // in reset. Any raised NVIC lines get folded into the per-core
        // pending masks via `raise_irqs_u64`.
        let mut ext_irqs = 0u64;
        if !self.is_held_in_reset_bit(RESET_UART0) {
            self.uart[0].tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_UART1) {
            self.uart[1].tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_SPI0) {
            self.spi[0].tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_SPI1) {
            self.spi[1].tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_I2C0) {
            self.i2c[0].tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_I2C1) {
            self.i2c[1].tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_ADC) {
            self.adc.tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        if !self.is_held_in_reset_bit(RESET_PWM) {
            self.pwm.tick(sys_clks, &self.clock_tree, &mut ext_irqs);
        }
        // POWMAN — AON timer; advance COUNT and pend TIMER IRQ on match.
        // HLD "2026.04.17 - HLD - RP2350 Coverage Gap Fill V11.md" §3.2.
        if !self.is_held_in_reset_bit(RESET_POWMAN) {
            ext_irqs |= self.powman.advance(sys_clks, &self.clock_tree);
        }
        self.raise_irqs_u64(ext_irqs);

        // DMA ticks after peripherals produce DREQ (HLD V5 §5.6).
        if !self.is_held_in_reset_bit(RESET_DMA) {
            self.tick_dma();
        }

        // WATCHDOG countdown — one cycle per `tick_peripherals` invocation
        // (HLD V5 §7.D.3). Simplification: not scaled by `sys_clks`; the
        // granularity matches the paced step-quantum tick loop closely
        // enough for the LOAD=100 / fire-within-130-cycles test bound.
        if self.watchdog.tick() {
            self.set_watchdog_reset();
        }
    }

    /// Raise the IRQ lines encoded in `bits` via `assert_irq_shared`.
    /// TIMER0/1 IRQs are all shared (both cores' NVIC see the pend).
    #[inline]
    fn raise_timer_irqs(&mut self, bits: u64) {
        if bits == 0 {
            return;
        }
        let mut remaining = bits;
        while remaining != 0 {
            let irq = remaining.trailing_zeros();
            self.assert_irq_shared(irq);
            remaining &= remaining - 1;
        }
    }

    /// Invalidate any per-core LR/SC reservation that covers the word
    /// containing `addr`. Called from every public write path (any master
    /// writing anywhere in the reservable region breaks the reservation,
    /// per HLD §4.7 / RV-priv A-extension).
    #[inline]
    pub(crate) fn invalidate_reservation_at(&mut self, addr: u32) {
        let word = addr & !3;
        for slot in self.reservation.iter_mut() {
            if *slot == Some(word) {
                *slot = None;
            }
        }
    }

    /// Returns true if a bus fault was detected on `core`'s last access.
    /// Phase 3 Stage 1: bus-fault state migrated to `CoreAtomics` and
    /// gained a per-core `core` arg (LLD V7 §2). Single-threaded callers
    /// pass their `self.core_id`.
    pub fn bus_fault(&self, core: usize) -> bool {
        self.atomics.is_bus_fault(core)
    }

    /// Returns the address that caused `core`'s most recent bus fault.
    pub fn bus_fault_addr(&self, core: usize) -> u32 {
        self.atomics.bus_fault_addr(core)
    }

    /// Clear `core`'s bus-fault flag.
    pub fn clear_bus_fault(&mut self, core: usize) {
        self.atomics.clear_bus_fault(core);
    }

    /// Request a system-wide watchdog reset (HLD V5 §7.D.3). Called by
    /// the WATCHDOG peripheral when its countdown fires. The CPU step
    /// loop polls [`Self::watchdog_reset_requested`] before instruction
    /// fetch and reseeds state + jumps to the reset vector on true.
    pub fn set_watchdog_reset(&mut self) {
        self.watchdog_reset_requested = true;
    }

    /// Returns true if a watchdog reset has been requested.
    pub fn watchdog_reset_requested(&self) -> bool {
        self.watchdog_reset_requested
    }

    /// Clear the watchdog-reset flag.
    pub fn clear_watchdog_reset(&mut self) {
        self.watchdog_reset_requested = false;
    }

    /// Clear the warn-once address budget. Call from `Emulator::reset()`
    /// (and the step-3a watchdog handler) alongside `clear_bus_fault()`
    /// and `clear_watchdog_reset()` so post-reset firmware sees a fresh
    /// warn-once slate.
    pub fn clear_warned_addrs(&mut self) {
        self.warned_addrs.clear();
    }

    /// Emit a single `tracing::warn!` per unique word-aligned MMIO
    /// address for HashMap-fallthrough (unmodelled) accesses. HLD V5
    /// §4.A1. Returns nothing; callers proceed with the existing
    /// fallthrough path.
    #[inline]
    fn warn_unmodelled_mmio(&mut self, word_addr: u32) {
        if self.warned_addrs.insert(word_addr) {
            tracing::warn!(
                addr = format_args!("{:#010X}", word_addr),
                "unmodelled MMIO access"
            );
        }
    }

    /// Set whether flash (XIP) content has been loaded.
    pub fn set_flash_loaded(&mut self, loaded: bool) {
        self.flash_loaded = loaded;
    }

    /// Read GPIO_HI_IN (SIO offset 0x008). Returns QSPI pin state.
    /// When flash is loaded, returns noise with bit 29 frequently set.
    /// The bootrom's flash-detect loop reads this 21 times, extracting
    /// bit 29 via `lsrs (gpio>>28), #2` and accumulating with `adcs`.
    /// The threshold is 0xF1 (241); without carry the sum is 231, so we
    /// need bit 29 set in ~11 of 21 reads.
    fn read_gpio_hi_in(&mut self) -> u32 {
        if !self.flash_loaded {
            return 0;
        }
        // Advance simple LFSR for variation, then force bit 29 on
        // most reads. Real QSPI lines are noisy — bias toward "alive".
        let s = self.gpio_hi_noise_state;
        self.gpio_hi_noise_state = s.wrapping_mul(1103515245).wrapping_add(12345);
        // Set bits 29-31 (QSPI data lines) to simulate flash responses.
        // Keep bit 28 toggling for additional entropy.
        self.gpio_hi_noise_state | 0xE000_0000
    }

    /// Load flash data into XIP memory and mark flash as loaded. Sets
    /// the XIP bit in [`Self::pending_invalidation_regions`] so the
    /// driver drains only XIP-region decode-cache slots before resuming
    /// execution — flash bytes have been replaced wholesale, but SRAM /
    /// ROM slots are untouched and stay hot.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.memory.load_flash(data);
        self.flash_loaded = true;
        self.pending_invalidation_regions |= invalidation_regions::XIP;
    }

    // --- Latency accounting ---

    /// Returns the cycle cost of the most recent bus access.
    pub fn last_access_cycles(&self) -> u32 {
        self.last_access_cycles
    }

    /// Returns accumulated extra wait states for the current instruction.
    pub fn extra_wait_states(&self) -> u32 {
        self.extra_wait_states
    }

    /// Reset extra wait state accumulator. Called at start of each instruction.
    pub fn reset_extra_wait_states(&mut self) {
        self.extra_wait_states = 0;
    }

    /// Adds `n` to the extra-wait-states accumulator. Used by the slow
    /// path in `decode_execute` to re-inject the cache entry's `fetch_wait`
    /// after `reset_extra_wait_states`, preserving cycle-count identity
    /// with the pre-cache behaviour.
    #[inline(always)]
    pub fn add_extra_wait_states(&mut self, n: u32) {
        self.extra_wait_states += n;
    }

    /// Return the current extra-wait-states accumulator and reset it to
    /// zero atomically. Backs the `CoreBus::take_extra_wait_states` trait
    /// method (Phase 3 Stage 2) — combined drain-and-read semantics are
    /// cheaper than a `extra_wait_states()` getter followed by
    /// `reset_extra_wait_states()`.
    #[inline(always)]
    pub fn take_extra_wait_states(&mut self) -> u32 {
        let n = self.extra_wait_states;
        self.extra_wait_states = 0;
        n
    }

    // --- Decoded-op cache invalidation (see HLD §7) ----------------------

    /// Queue cache invalidations covering `[addr, addr+len)` on the
    /// per-core decode caches. `len` is 1, 2, or 4 bytes for the three
    /// write widths. Pushes into [`Self::pending_cache_invalidations`];
    /// the driver drains it into the core that ran. Mirrors the
    /// [`crate::threaded::bus::WorkerBus`] pattern (LLD V7 §9).
    ///
    /// The drainer
    /// ([`crate::core::CortexM33::invalidate_decode_cache_entries`])
    /// evicts both the slot for `addr` and the slot for `addr - 2`
    /// (covering a wide instruction whose `hw1` landed at `addr`). For
    /// a 4-byte write we push `{addr, addr+2}` so the combined coverage
    /// is `{addr-2, addr, addr+2}` — parity with the pre-migration
    /// `invalidate_pc_range(addr, 4)` sweep.
    #[inline]
    fn invalidate_pc_range(&mut self, addr: u32, len: u8) {
        debug_assert!(len == 1 || len == 2 || len == 4);
        if matches!(addr >> 28, 0x0..=0x2) {
            self.pending_cache_invalidations.push(addr);
            if len == 4 {
                // `write32` spans `{addr-2, addr, addr+2}`; one push only
                // covers `{addr-2, addr}`. Push `addr+2` to reach the third
                // slot.
                self.pending_cache_invalidations.push(addr.wrapping_add(2));
            }
        }
    }

    /// Request a bulk invalidation of both cores' decode caches. Escape
    /// hatch for tools / tests that write executable bytes through paths
    /// that bypass the usual invalidation hooks (e.g. `Emulator::poke`,
    /// direct `bus.memory.sram_write*`). Sets the
    /// [`invalidation_regions::BULK`] bit so every slot is drained
    /// regardless of tag region. The driver (`Emulator::step` /
    /// `Emulator::load_*`) clears [`Self::pending_invalidation_regions`]
    /// after invalidating both cores.
    pub fn invalidate_all(&mut self) {
        self.pending_invalidation_regions |= invalidation_regions::BULK;
    }

    /// Load the bootrom (32 KB ROM image at 0x0000_0000). Sets the
    /// [`invalidation_regions::ROM`] bit in
    /// [`Self::pending_invalidation_regions`] so the driver drains only
    /// ROM-region decode-cache slots before resuming execution.
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.memory.load_rom(data);
        self.pending_invalidation_regions |= invalidation_regions::ROM;
    }

    /// Enable burst mode — suppresses per-word SRAM bank wait states.
    /// Used by multi-word instructions (STM/LDM/PUSH/POP).
    pub fn set_burst_mode(&mut self) {
        self.burst_mode = true;
    }

    /// Disable burst mode after multi-word transfer completes.
    pub fn clear_burst_mode(&mut self) {
        self.burst_mode = false;
    }

    /// Compute read latency for an address region.
    #[inline(always)]
    fn read_latency(region: u32) -> (u32, u32) {
        match region {
            0x0 => (1, 0), // ROM
            0x1 => (1, 0), // XIP cache hit
            0x2 => (1, 0), // SRAM
            0x4 => (3, 2), // APB peripherals
            0x5 => (1, 0), // AHB peripherals
            0xD => (1, 0), // SIO
            0xE => (1, 0), // PPB
            _   => (1, 0), // unmapped
        }
    }

    /// Compute write latency for an address region.
    #[inline(always)]
    fn write_latency(region: u32) -> (u32, u32) {
        match region {
            0x2 => (1, 0), // SRAM
            0x4 => (4, 3), // APB peripherals
            0x5 => (1, 0), // AHB peripherals
            0xD => (1, 0), // SIO
            0xE => (1, 0), // PPB
            _   => (1, 0), // unmapped/ROM
        }
    }

    // --- 8-bit access ---

    pub fn read8(&mut self, addr: u32, core: u8) -> u8 {
        let addr = canon_oracle_addr(addr);
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = match region {
            0x2 => addr & 0x00FF_FFFF, // strip SRAM alias bits [27:24]
            _   => addr & 0x0FFF_FFFF,
        };
        let val = match region {
            0x0 if offset < 0x8000 => self.memory.rom_read8(offset),
            0x1 if Self::is_xip_sram(addr) && self.flash_loaded => {
                self.memory.xip_read8((addr - 0x1C00_0000) + self.xip_cache_offset)
            }
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_read8(addr),
            0x1 => {
                if !self.flash_loaded {
                    self.atomics.set_bus_fault(core as usize, addr);
                    if self.mmio_trace_enabled {
                        self.emit_mmio_trace('R', 1, addr, 0, core);
                    }
                    return 0;
                }
                self.memory.xip_read8(offset)
            }
            0x2 if offset < SRAM_SIZE as u32 => {
                let v = self.memory.sram_read8(offset);
                // sram_bank_wait removed: bank 2/6 penalty is modeled on
                // instruction fetch only (in decode.rs), not data accesses.
                v
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let word_addr = canonical & !3;
                let offset = word_addr & 0x0000_0FFF;
                // Narrow-access dispatch for byte-significant Phase 2
                // registers: UARTDR pops one RX byte per access; SSPDR
                // pops one RX word per access (low byte here).
                if !self.is_held_in_reset_base(base) {
                    match (base, offset) {
                        (UART0_BASE, crate::peripherals::uart::UARTDR) => {
                            let v = self.uart[0].read8(crate::peripherals::uart::UARTDR);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        (UART1_BASE, crate::peripherals::uart::UARTDR) => {
                            let v = self.uart[1].read8(crate::peripherals::uart::UARTDR);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        (SPI0_BASE, crate::peripherals::spi::SSPDR) => {
                            let v = self.spi[0].read8(crate::peripherals::spi::SSPDR);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        (SPI1_BASE, crate::peripherals::spi::SSPDR) => {
                            let v = self.spi[1].read8(crate::peripherals::spi::SSPDR);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                            let v = self.i2c[0].read8(crate::peripherals::i2c::IC_DATA_CMD);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        (I2C1_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                            let v = self.i2c[1].read8(crate::peripherals::i2c::IC_DATA_CMD);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        (ADC_BASE, crate::peripherals::adc::FIFO) => {
                            let v = self.adc.read8(crate::peripherals::adc::FIFO);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 1, addr, v as u32, core);
                            }
                            return v;
                        }
                        _ => {}
                    }
                }
                let word = if self.is_held_in_reset_base(base) {
                    0
                } else {
                    match base {
                        0x4000_0000 => self.sysinfo_read(offset),
                        0x4002_0000 => self.resets_read(offset),
                        0x4001_0000 => self.clocks_read(offset),
                        0x4004_8000 => self.xosc_read(offset),
                        0x400E_8000 => self.rosc_read(offset),
                        0x4005_0000 => self.pll_sys_read(offset),
                        0x4005_8000 => self.pll_usb_read(offset),
                        0x400D_0000 => self.qmi_read(offset),
                        TIMER0_BASE => self.timer0.read32(offset),
                        TIMER1_BASE => self.timer1.read32(offset),
                        TICKS_BASE => self.ticks.read32(offset),
                        UART0_BASE => self.uart[0].read32(offset),
                        UART1_BASE => self.uart[1].read32(offset),
                        SPI0_BASE => self.spi[0].read32(offset),
                        SPI1_BASE => self.spi[1].read32(offset),
                        I2C0_BASE => self.i2c[0].read32(offset),
                        I2C1_BASE => self.i2c[1].read32(offset),
                        ADC_BASE => self.adc.read32(offset),
                        PWM_BASE => self.pwm.read32(offset),
                        IO_BANK0_BASE => self.io_bank0.read32(offset),
                        PADS_BANK0_BASE => self.pads_bank0.read32(offset),
                        SYSCFG_BASE => self.syscfg.read32(offset),
                        TBMAN_BASE => self.tbman.read32(offset),
                        GLITCH_DETECTOR_BASE => self.glitch.read32(offset),
                        PSM_BASE => self.psm.read32(offset),
                        WATCHDOG_BASE => self.watchdog.read32(offset),
                        OTP_DATA_BASE => {
                            let word_off = (addr - OTP_DATA_BASE) & (OTP_DATA_SIZE - 1) & !3;
                            self.otp.read32(word_off)
                        }
                        TRNG_BASE => self.trng.read32(offset),
                        SHA256_BASE => self.sha256.read32(offset),
                        POWMAN_BASE => self.powman.read32(offset),
                        0x5020_0000 => self.pio[0].read32(offset),
                        0x5030_0000 => self.pio[1].read32(offset),
                        0x5040_0000 => self.pio[2].read32(offset),
                        _ => {
                            self.warn_unmodelled_mmio(word_addr);
                            *self.peripheral_regs.get(&word_addr).unwrap_or(&0)
                        }
                    }
                };
                let byte_idx = (canonical & 3) as usize;
                word.to_le_bytes()[byte_idx]
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                let word_offset = reg_offset & !3;
                debug_assert!(
                    !crate::core::PerCoreSio::owns_offset(word_offset),
                    "DIV/INTERP addr 0x{:08X} reached Bus::read8 — use CortexM33::bus_read8 wrapper",
                    addr
                );
                let word = match word_offset {
                    0x004 => self.gpio_in,
                    0x008 => self.read_gpio_hi_in(),
                    _ => {
                        self.sio.read32(word_offset, core as usize)
                    }
                };
                word.to_le_bytes()[(addr & 3) as usize]
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_read8(addr),
            0xE if Self::is_coresight_trace(addr) => {
                let word_off = (addr - CORESIGHT_TRACE_BASE) & !3;
                let byte_idx = (addr & 3) as usize;
                self.coresight_trace.read32(word_off).to_le_bytes()[byte_idx]
            }
            0xE => 0, // PPB (stub)
            _ => {
                self.atomics.set_bus_fault(core as usize, addr);
                0
            }
        };
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('R', 1, addr, val as u32, core);
        }
        val
    }

    pub fn write8(&mut self, addr: u32, val: u8, core: u8) {
        let addr = canon_oracle_addr(addr);
        // RV32A: any write can invalidate an LR/SC reservation on either
        // core. HLD §4.7.
        self.invalidate_reservation_at(addr);
        let region = addr >> 28;
        debug_assert!(
            region != 0xD || !crate::core::PerCoreSio::owns_offset(addr & 0xFFF),
            "DIV/INTERP addr 0x{:08X} reached Bus::write8 — use CortexM33::bus_write8 wrapper",
            addr
        );
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x00FF_FFFF;
        match region {
            0x1 if Self::is_xip_sram(addr) => {
                self.xip_sram_write8(addr, val);
                self.invalidate_pc_range(addr, 1);
            }
            0x2 if offset < SRAM_SIZE as u32 => {
                let sram_alias = (addr >> 24) & 0x3;
                if sram_alias == 0 {
                    self.memory.sram_write8(offset, val);
                } else {
                    let old = self.memory.sram_read8(offset);
                    let new_val = match sram_alias {
                        1 => old ^ val,
                        2 => old | val,
                        3 => old & !val,
                        _ => unreachable!(),
                    };
                    self.memory.sram_write8(offset, new_val);
                }
                // sram_bank_wait removed: bank penalty on instruction fetch only.
                self.invalidate_pc_range(addr, 1);
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let word_offset_for_narrow = (canonical & !3) & 0x0000_0FFF;
                // RESETS Bus-level guard (HLD V5 §5.3). Held
                // peripherals drop the write silently.
                if self.is_held_in_reset_base(base) {
                    // no-op
                } else {
                    // Narrow-access dispatch for byte-significant Phase 2
                    // registers: UARTDR pushes one TX byte per access;
                    // SSPDR pushes one TX word per access; IC_DATA_CMD
                    // triggers one transaction per access. Bypass the
                    // word-RMW path so these side-effect registers aren't
                    // double-fired.
                    // Map the two possible bases for each peripheral to
                    // an instance index so we can share one narrow-access
                    // block.
                    let uart_instance: Option<usize> = match base {
                        UART0_BASE => Some(0),
                        UART1_BASE => Some(1),
                        _ => None,
                    };
                    let spi_instance: Option<usize> = match base {
                        SPI0_BASE => Some(0),
                        SPI1_BASE => Some(1),
                        _ => None,
                    };
                    let i2c_instance: Option<usize> = match base {
                        I2C0_BASE => Some(0),
                        I2C1_BASE => Some(1),
                        _ => None,
                    };
                    if let Some(idx) = uart_instance {
                        if word_offset_for_narrow == crate::peripherals::uart::UARTDR {
                            let mut ext_irqs = 0u64;
                            self.uart[idx].write8(
                                crate::peripherals::uart::UARTDR,
                                val,
                                &mut ext_irqs,
                            );
                            self.raise_irqs_u64(ext_irqs);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 1, addr, val as u32, core);
                            }
                            return;
                        }
                    }
                    if let Some(idx) = spi_instance {
                        if word_offset_for_narrow == crate::peripherals::spi::SSPDR {
                            let mut ext_irqs = 0u64;
                            self.spi[idx].write8(
                                crate::peripherals::spi::SSPDR,
                                val,
                                &mut ext_irqs,
                            );
                            self.raise_irqs_u64(ext_irqs);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 1, addr, val as u32, core);
                            }
                            return;
                        }
                    }
                    if let Some(idx) = i2c_instance {
                        if word_offset_for_narrow == crate::peripherals::i2c::IC_DATA_CMD {
                            let mut ext_irqs = 0u64;
                            self.i2c[idx].write8(
                                crate::peripherals::i2c::IC_DATA_CMD,
                                val,
                                &mut ext_irqs,
                            );
                            self.raise_irqs_u64(ext_irqs);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 1, addr, val as u32, core);
                            }
                            return;
                        }
                    }
                    match (base, word_offset_for_narrow) {
                        // ADC FIFO is a side-effect register: `adc.read32(FIFO)`
                        // pops a sample. A byte write through the RMW path
                        // would read-then-write-back and silently pop the
                        // FIFO. The FIFO has no architected narrow-write
                        // semantics on real silicon (datasheet §12.4.5 lists
                        // FIFO as read-only) — swallow the access. Mirrors
                        // the RP2040 `narrow_peripheral_write8` ADC arm
                        // (`crates/mdrp2040/src/bus/mod.rs:877-878`). Note:
                        // byte lanes >0 within other narrow registers will
                        // also pop via the RMW path — silicon firmware
                        // doesn't hit this; matches RP2040 idiom.
                        (ADC_BASE, crate::peripherals::adc::FIFO) => {
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 1, addr, val as u32, core);
                            }
                            return;
                        }
                        _ => {}
                    }
                    match base {
                        0x4000_0000 => {
                            // SYSINFO: read-only, ignore byte writes
                        }
                        0x400D_0000 => {
                            // QMI: do RMW on the word
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let old_word = self.qmi_read(reg_offset);
                            let mut bytes = old_word.to_le_bytes();
                            bytes[byte_idx] = val;
                            self.qmi_write(reg_offset, u32::from_le_bytes(bytes));
                        }
                        0x4001_0000 | 0x4005_0000 | 0x4005_8000
                        | 0x4004_8000 | 0x400E_8000 => {
                            // CLOCKS / PLL_SYS / PLL_USB / XOSC / ROSC:
                            // peripherals that handle the atomic alias
                            // internally. For a subword SET/CLR/XOR we
                            // must preserve the alias semantic — passing
                            // alias=0 after an RMW merge would turn SET
                            // into plain overwrite (see LLD V2 §4.8 note
                            // on the pre-existing subword bug). Strategy:
                            //   • alias == 0 → RMW the word, pass alias=0.
                            //   • alias != 0 → expand byte to `byte << shift`
                            //     and let the peripheral's alias logic
                            //     apply SET / CLR / XOR bit-wise.
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    0x4001_0000 => self.clocks_read(reg_offset),
                                    0x4005_0000 => self.pll_sys_read(reg_offset),
                                    0x4005_8000 => self.pll_usb_read(reg_offset),
                                    0x4004_8000 => self.xosc_read(reg_offset),
                                    _ => self.rosc_read(reg_offset),
                                };
                                let mut bytes = old_word.to_le_bytes();
                                bytes[byte_idx] = val;
                                (u32::from_le_bytes(bytes), 0)
                            } else {
                                ((val as u32) << (byte_idx * 8), alias)
                            };
                            match base {
                                0x4001_0000 => self.clocks_write(reg_offset, word_val, pass_alias),
                                0x4005_0000 => self.pll_sys_write(reg_offset, word_val, pass_alias),
                                0x4005_8000 => self.pll_usb_write(reg_offset, word_val, pass_alias),
                                0x4004_8000 => self.xosc_write(reg_offset, word_val, pass_alias),
                                _ => self.rosc_write(reg_offset, word_val, pass_alias),
                            }
                        }
                        TIMER0_BASE | TIMER1_BASE | TICKS_BASE => {
                            // TIMER / TICKS: same subword-alias
                            // strategy as CLOCKS — preserve SET/CLR/XOR
                            // semantics when the access was an alias.
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    TIMER0_BASE => self.timer0.read32(reg_offset),
                                    TIMER1_BASE => self.timer1.read32(reg_offset),
                                    _ => self.ticks.read32(reg_offset),
                                };
                                let mut bytes = old_word.to_le_bytes();
                                bytes[byte_idx] = val;
                                (u32::from_le_bytes(bytes), 0)
                            } else {
                                ((val as u32) << (byte_idx * 8), alias)
                            };
                            match base {
                                TIMER0_BASE => self.timer0.write32(reg_offset, word_val, pass_alias),
                                TIMER1_BASE => self.timer1.write32(reg_offset, word_val, pass_alias),
                                _ => {
                                    if self.ticks.write32(reg_offset, word_val, pass_alias) {
                                        self.timer0.invalidate_lazy();
                                        self.timer1.invalidate_lazy();
                                    }
                                }
                            }
                        }
                        0x4002_0000 => {
                            // RESETS: only word-aligned writes meaningful, ignore byte
                        }
                        SYSCFG_BASE | TBMAN_BASE | GLITCH_DETECTOR_BASE
                        | PSM_BASE | WATCHDOG_BASE => {
                            // Inert / PSM / WATCHDOG narrow byte: same
                            // subword-alias strategy as CLOCKS. WATCHDOG
                            // TRIGGER write from a byte lane is not a real
                            // firmware pattern (TRIGGER is bit 31 — lane 3
                            // MSB) but handled correctly by preserving alias.
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    SYSCFG_BASE => self.syscfg.read32(reg_offset),
                                    TBMAN_BASE => self.tbman.read32(reg_offset),
                                    GLITCH_DETECTOR_BASE => self.glitch.read32(reg_offset),
                                    PSM_BASE => self.psm.read32(reg_offset),
                                    _ => self.watchdog.read32(reg_offset),
                                };
                                let mut bytes = old_word.to_le_bytes();
                                bytes[byte_idx] = val;
                                (u32::from_le_bytes(bytes), 0u32)
                            } else {
                                ((val as u32) << (byte_idx * 8), alias)
                            };
                            match base {
                                SYSCFG_BASE => self.syscfg.write32(reg_offset, word_val, pass_alias),
                                TBMAN_BASE => self.tbman.write32(reg_offset, word_val, pass_alias),
                                GLITCH_DETECTOR_BASE => self.glitch.write32(reg_offset, word_val, pass_alias),
                                PSM_BASE => self.psm.write32(reg_offset, word_val, pass_alias),
                                _ => {
                                    if self.watchdog.write32(reg_offset, word_val, pass_alias) {
                                        self.set_watchdog_reset();
                                    }
                                }
                            }
                        }
                        OTP_DATA_BASE => {
                            // OTP narrow byte write — OR-only fuse
                            // semantics apply at the word level. Pack
                            // the byte into the correct lane and OR-merge.
                            let otp_word_off =
                                (addr - OTP_DATA_BASE) & (OTP_DATA_SIZE - 1) & !3;
                            let byte_idx = ((addr - OTP_DATA_BASE) & 3) as usize;
                            let word_val = (val as u32) << (byte_idx * 8);
                            self.otp.write32(otp_word_off, word_val);
                        }
                        TRNG_BASE | SHA256_BASE | POWMAN_BASE => {
                            // TRNG / SHA / POWMAN narrow byte — subword-alias strategy.
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    TRNG_BASE => self.trng.read32(reg_offset),
                                    SHA256_BASE => self.sha256.read32(reg_offset),
                                    _ => self.powman.read32(reg_offset),
                                };
                                let mut bytes = old_word.to_le_bytes();
                                bytes[byte_idx] = val;
                                (u32::from_le_bytes(bytes), 0u32)
                            } else {
                                ((val as u32) << (byte_idx * 8), alias)
                            };
                            match base {
                                TRNG_BASE => self.trng.write32(reg_offset, word_val, pass_alias),
                                SHA256_BASE => self.sha256.write32(reg_offset, word_val, pass_alias),
                                _ => {
                                    let mask = self.powman.write32(reg_offset, word_val, pass_alias);
                                    self.raise_irqs_u64(mask);
                                }
                            }
                        }
                        UART0_BASE | UART1_BASE | SPI0_BASE | SPI1_BASE | I2C0_BASE
                        | I2C1_BASE | ADC_BASE | PWM_BASE | IO_BANK0_BASE
                        | PADS_BANK0_BASE => {
                            // Phase 2 peripherals that don't need narrow
                            // byte dispatch (already intercepted above for
                            // UART_DR / SSPDR / IC_DATA_CMD). Use the same
                            // subword-alias pattern as CLOCKS/TIMER: preserve
                            // SET/CLR/XOR semantics.
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    UART0_BASE => self.uart[0].read32(reg_offset),
                                    UART1_BASE => self.uart[1].read32(reg_offset),
                                    SPI0_BASE => self.spi[0].read32(reg_offset),
                                    SPI1_BASE => self.spi[1].read32(reg_offset),
                                    I2C0_BASE => self.i2c[0].read32(reg_offset),
                                    I2C1_BASE => self.i2c[1].read32(reg_offset),
                                    ADC_BASE => self.adc.read32(reg_offset),
                                    PWM_BASE => self.pwm.read32(reg_offset),
                                    IO_BANK0_BASE => self.io_bank0.read32(reg_offset),
                                    _ => self.pads_bank0.read32(reg_offset),
                                };
                                let mut bytes = old_word.to_le_bytes();
                                bytes[byte_idx] = val;
                                (u32::from_le_bytes(bytes), 0u32)
                            } else {
                                ((val as u32) << (byte_idx * 8), alias)
                            };
                            let mut ext_irqs = 0u64;
                            match base {
                                UART0_BASE => self.uart[0].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                UART1_BASE => self.uart[1].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                SPI0_BASE => self.spi[0].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                SPI1_BASE => self.spi[1].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                I2C0_BASE => self.i2c[0].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                I2C1_BASE => self.i2c[1].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                ADC_BASE => self.adc.write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                PWM_BASE => self.pwm.write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                IO_BANK0_BASE => self.io_bank0.write32(reg_offset, word_val, pass_alias),
                                _ => self.pads_bank0.write32(reg_offset, word_val, pass_alias),
                            }
                            self.raise_irqs_u64(ext_irqs);
                        }
                        0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {} // PIO: 32-bit access only
                        _ => {
                            let word_addr = canonical & !3;
                            let byte_idx = (canonical & 3) as usize;
                            self.warn_unmodelled_mmio(word_addr);
                            let old_word = *self.peripheral_regs.get(&word_addr).unwrap_or(&0);
                            let mut bytes = old_word.to_le_bytes();
                            let old_byte = bytes[byte_idx];
                            bytes[byte_idx] = match alias {
                                0 => val,
                                1 => old_byte ^ val,
                                2 => old_byte | val,
                                3 => old_byte & !val,
                                _ => unreachable!(),
                            };
                            self.peripheral_regs.insert(word_addr, u32::from_le_bytes(bytes));
                        }
                    }
                }
            }
            0xD => {
                // SIO has no APB alias encoding — every access is
                // alias 0, word-only at the peripheral. Narrow byte
                // writes are fielded by read-modify-write at the word
                // level: read the current word, splice in the new
                // byte at its lane, write back. This matches how the
                // CORESIGHT_TRACE arm below handles byte writes.
                //
                // Without this arm, `STRB Rn, [Rm, #0]` to
                // SIO_GPIO_OUT (and every other SIO register) fell
                // through the catch-all and was silently dropped —
                // blocking OneROM's CPU-serve loop where the final
                // store that drives the data pins is a byte write.
                //
                // GPIO_OUT family (offsets 0x010 / 0x018 / 0x020 /
                // 0x028 and OE 0x030 / 0x038 / 0x040 / 0x048): real
                // RP2350 silicon replicates the byte across all 4
                // lanes of the 32-bit register (the single-cycle IO
                // fabric latches the full 32-bit bus without
                // byte-lane enables for the GPIO_OUT path). OneROM's
                // CPU-serve loop relies on this: a single STRB at
                // offset 0 lights up pins 16..23 as well as pins 0..7.
                let word_addr = addr & !3;
                let reg_offset = word_addr & 0xFFF;
                if Self::is_sio_gpio_out_replicating_reg(reg_offset) {
                    let replicated = u32::from(val) * 0x0101_0101;
                    self.write32(word_addr, replicated, core);
                } else {
                    let byte_idx = (addr & 3) as usize;
                    // core is the outer write8/16/32 param
                    // Read via the low-level SIO path rather than
                    // `self.read32(word_addr, core)` so we don't trip the
                    // GPIO_IN mirror short-circuit — the merged
                    // write we emit goes back to
                    // `self.write32(word_addr, ..., core)`, which
                    // preserves FIFO_WR / doorbell / softirq
                    // semantics for any byte-lane access that hits
                    // a side-effect offset.
                    let old_word = match reg_offset {
                        0x004 => self.gpio_in,
                        0x008 => self.read_gpio_hi_in(),
                        _ => self.sio.read32(reg_offset, core as usize),
                    };
                    let mut bytes = old_word.to_le_bytes();
                    bytes[byte_idx] = val;
                    self.write32(word_addr, u32::from_le_bytes(bytes), core);
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_write8(addr, val),
            0xE if Self::is_coresight_trace(addr) => {
                // Narrow byte write — pack into the correct lane,
                // merge at word granularity.
                let word_off = (addr - CORESIGHT_TRACE_BASE) & !3;
                let byte_idx = (addr & 3) as usize;
                let old = self.coresight_trace.read32(word_off);
                let mut bytes = old.to_le_bytes();
                bytes[byte_idx] = val;
                self.coresight_trace
                    .write32(word_off, u32::from_le_bytes(bytes), 0);
            }
            _ => {} // ROM read-only, others unmapped/stub
        }
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('W', 1, addr, val as u32, core);
        }
    }

    /// Fold an `irqs: u64` mask from a peripheral's write-path into
    /// the per-core pending banks via [`Self::assert_irq_shared`]. Used
    /// by the narrow-access dispatch and the word-RMW dispatch paths;
    /// unit-tested via the Phase 2 integration tests.
    ///
    /// Bits outside the peripheral-driven range (`PERIPH_IRQ_MASK` —
    /// lines 46..=51 are software-only, writable only via `NVIC_ISPR`)
    /// are filtered out. A peripheral `mask |= 1 << IRQ_*` typo on an
    /// out-of-range constant would otherwise silently misassert a
    /// software-only line.
    #[inline]
    pub(crate) fn raise_irqs_u64(&mut self, irqs: u64) {
        let mut remaining = irqs & PERIPH_IRQ_MASK;
        if remaining == 0 {
            return;
        }
        while remaining != 0 {
            let irq = remaining.trailing_zeros();
            self.assert_irq_shared(irq);
            remaining &= remaining - 1;
        }
    }

    // --- 16-bit access ---

    pub fn read16(&mut self, addr: u32, core: u8) -> u16 {
        let addr = canon_oracle_addr(addr);
        // Phase 0b.1 Commit B: PPB addresses route through
        // `CortexM33::bus_read16`. Bus-level read16 is still reachable
        // from decode.rs (opcode fetch) and non-PPB tests.
        debug_assert!(addr >> 28 != 0xE || Self::is_boot_ram(addr),
            "PPB address 0x{:08X} reached Bus::read16 — use CortexM33::bus_read16 wrapper",
            addr);
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = match region {
            0x2 => addr & 0x00FF_FFFF, // strip SRAM alias bits [27:24]
            _   => addr & 0x0FFF_FFFF,
        };
        let val = match region {
            0x0 if offset + 1 < 0x8000 => self.memory.rom_read16(offset),
            0x1 if Self::is_xip_sram(addr) && self.flash_loaded => {
                self.memory.xip_read16((addr - 0x1C00_0000) + self.xip_cache_offset)
            }
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_read16(addr),
            0x1 => {
                if !self.flash_loaded {
                    self.atomics.set_bus_fault(core as usize, addr);
                    if self.mmio_trace_enabled {
                        self.emit_mmio_trace('R', 2, addr, 0, core);
                    }
                    return 0;
                }
                self.memory.xip_read16(offset)
            }
            0x2 if (offset + 1) < SRAM_SIZE as u32 => {
                let v = self.memory.sram_read16(offset);
                // sram_bank_wait removed: bank penalty on instruction fetch only.
                v
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let word_addr = canonical & !3;
                let offset = word_addr & 0x0000_0FFF;
                // Narrow halfword path: SPI SSPDR is the only half-significant
                // register (8..16-bit frames pop one word/pop one word).
                if !self.is_held_in_reset_base(base) {
                    // SPI halfword side-effect registers (both instances).
                    let spi_idx: Option<usize> = match base {
                        SPI0_BASE => Some(0),
                        SPI1_BASE => Some(1),
                        _ => None,
                    };
                    if let Some(idx) = spi_idx {
                        if offset == crate::peripherals::spi::SSPDR {
                            let v = self.spi[idx].read16(crate::peripherals::spi::SSPDR);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 2, addr, v as u32, core);
                            }
                            return v;
                        }
                    }
                    // UARTDR and IC_DATA_CMD: halfword read collapses to
                    // byte via narrow path (zero-extended).
                    let uart_idx: Option<usize> = match base {
                        UART0_BASE => Some(0),
                        UART1_BASE => Some(1),
                        _ => None,
                    };
                    if let Some(idx) = uart_idx {
                        if offset == crate::peripherals::uart::UARTDR {
                            let v = self.uart[idx].read8(crate::peripherals::uart::UARTDR) as u16;
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 2, addr, v as u32, core);
                            }
                            return v;
                        }
                    }
                    let i2c_idx: Option<usize> = match base {
                        I2C0_BASE => Some(0),
                        I2C1_BASE => Some(1),
                        _ => None,
                    };
                    if let Some(idx) = i2c_idx {
                        if offset == crate::peripherals::i2c::IC_DATA_CMD {
                            let v =
                                self.i2c[idx].read32(crate::peripherals::i2c::IC_DATA_CMD) as u16;
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('R', 2, addr, v as u32, core);
                            }
                            return v;
                        }
                    }
                    if (base, offset) == (ADC_BASE, crate::peripherals::adc::FIFO) {
                        let v = self.adc.read16(crate::peripherals::adc::FIFO);
                        if self.mmio_trace_enabled {
                            self.emit_mmio_trace('R', 2, addr, v as u32, core);
                        }
                        return v;
                    }
                }
                let word = if self.is_held_in_reset_base(base) {
                    0
                } else {
                    match base {
                        0x4000_0000 => self.sysinfo_read(offset),
                        0x4002_0000 => self.resets_read(offset),
                        0x4001_0000 => self.clocks_read(offset),
                        0x4004_8000 => self.xosc_read(offset),
                        0x400E_8000 => self.rosc_read(offset),
                        0x4005_0000 => self.pll_sys_read(offset),
                        0x4005_8000 => self.pll_usb_read(offset),
                        0x400D_0000 => self.qmi_read(offset),
                        TIMER0_BASE => self.timer0.read32(offset),
                        TIMER1_BASE => self.timer1.read32(offset),
                        TICKS_BASE => self.ticks.read32(offset),
                        UART0_BASE => self.uart[0].read32(offset),
                        UART1_BASE => self.uart[1].read32(offset),
                        SPI0_BASE => self.spi[0].read32(offset),
                        SPI1_BASE => self.spi[1].read32(offset),
                        I2C0_BASE => self.i2c[0].read32(offset),
                        I2C1_BASE => self.i2c[1].read32(offset),
                        ADC_BASE => self.adc.read32(offset),
                        PWM_BASE => self.pwm.read32(offset),
                        IO_BANK0_BASE => self.io_bank0.read32(offset),
                        PADS_BANK0_BASE => self.pads_bank0.read32(offset),
                        SYSCFG_BASE => self.syscfg.read32(offset),
                        TBMAN_BASE => self.tbman.read32(offset),
                        GLITCH_DETECTOR_BASE => self.glitch.read32(offset),
                        PSM_BASE => self.psm.read32(offset),
                        WATCHDOG_BASE => self.watchdog.read32(offset),
                        OTP_DATA_BASE => {
                            let word_off = (addr - OTP_DATA_BASE) & (OTP_DATA_SIZE - 1) & !3;
                            self.otp.read32(word_off)
                        }
                        TRNG_BASE => self.trng.read32(offset),
                        SHA256_BASE => self.sha256.read32(offset),
                        POWMAN_BASE => self.powman.read32(offset),
                        0x5020_0000 => self.pio[0].read32(offset),
                        0x5030_0000 => self.pio[1].read32(offset),
                        0x5040_0000 => self.pio[2].read32(offset),
                        _ => {
                            self.warn_unmodelled_mmio(word_addr);
                            *self.peripheral_regs.get(&word_addr).unwrap_or(&0)
                        }
                    }
                };
                let half_idx = ((canonical >> 1) & 1) as usize;
                let halves: [u16; 2] = [word as u16, (word >> 16) as u16];
                halves[half_idx]
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                let word_offset = reg_offset & !3;
                debug_assert!(
                    !crate::core::PerCoreSio::owns_offset(word_offset),
                    "DIV/INTERP addr 0x{:08X} reached Bus::read16 — use CortexM33::bus_read16 wrapper",
                    addr
                );
                let word = match word_offset {
                    0x004 => self.gpio_in,
                    0x008 => self.read_gpio_hi_in(),
                    _ => {
                        self.sio.read32(word_offset, core as usize)
                    }
                };
                let half_idx = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half_idx]
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_read16(addr),
            0xE if Self::is_coresight_trace(addr) => {
                let word_off = (addr - CORESIGHT_TRACE_BASE) & !3;
                let word = self.coresight_trace.read32(word_off);
                let half_idx = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half_idx]
            }
            _ => {
                self.atomics.set_bus_fault(core as usize, addr);
                0
            }
        };
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('R', 2, addr, val as u32, core);
        }
        val
    }

    pub fn write16(&mut self, addr: u32, val: u16, core: u8) {
        let addr = canon_oracle_addr(addr);
        // Phase 0b.1 Commit B: PPB addresses route through
        // `CortexM33::bus_write16`.
        debug_assert!(addr >> 28 != 0xE || Self::is_boot_ram(addr),
            "PPB address 0x{:08X} reached Bus::write16 — use CortexM33::bus_write16 wrapper",
            addr);
        debug_assert!(
            addr >> 28 != 0xD || !crate::core::PerCoreSio::owns_offset(addr & 0xFFF),
            "DIV/INTERP addr 0x{:08X} reached Bus::write16 — use CortexM33::bus_write16 wrapper",
            addr
        );
        // RV32A: invalidate any LR/SC reservation that covers this word
        // (a 16-bit write within the word still clears the full-word
        // reservation). HLD §4.7.
        self.invalidate_reservation_at(addr);
        if addr & 3 == 3 {
            // Crosses a word boundary — also break the next word's
            // reservation.
            self.invalidate_reservation_at(addr.wrapping_add(1));
        }
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x00FF_FFFF;
        match region {
            0x1 if Self::is_xip_sram(addr) => {
                self.xip_sram_write16(addr, val);
                self.invalidate_pc_range(addr, 2);
            }
            0x2 if (offset + 1) < SRAM_SIZE as u32 => {
                let sram_alias = (addr >> 24) & 0x3;
                if sram_alias == 0 {
                    self.memory.sram_write16(offset, val);
                } else {
                    let old = self.memory.sram_read16(offset);
                    let new_val = match sram_alias {
                        1 => old ^ val,
                        2 => old | val,
                        3 => old & !val,
                        _ => unreachable!(),
                    };
                    self.memory.sram_write16(offset, new_val);
                }
                // sram_bank_wait removed: bank penalty on instruction fetch only.
                self.invalidate_pc_range(addr, 2);
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let word_offset_for_narrow = (canonical & !3) & 0x0000_0FFF;
                // RESETS Bus-level guard (HLD V5 §5.3).
                if self.is_held_in_reset_base(base) {
                    // no-op
                } else {
                    // Narrow halfword dispatch for side-effect registers
                    // (UART0/1, SPI0/1, I2C0/1).
                    let uart_instance: Option<usize> = match base {
                        UART0_BASE => Some(0),
                        UART1_BASE => Some(1),
                        _ => None,
                    };
                    let spi_instance: Option<usize> = match base {
                        SPI0_BASE => Some(0),
                        SPI1_BASE => Some(1),
                        _ => None,
                    };
                    let i2c_instance: Option<usize> = match base {
                        I2C0_BASE => Some(0),
                        I2C1_BASE => Some(1),
                        _ => None,
                    };
                    if let Some(idx) = uart_instance {
                        if word_offset_for_narrow == crate::peripherals::uart::UARTDR {
                            let mut ext_irqs = 0u64;
                            self.uart[idx].write8(
                                crate::peripherals::uart::UARTDR,
                                val as u8,
                                &mut ext_irqs,
                            );
                            self.raise_irqs_u64(ext_irqs);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 2, addr, val as u32, core);
                            }
                            return;
                        }
                    }
                    if let Some(idx) = spi_instance {
                        if word_offset_for_narrow == crate::peripherals::spi::SSPDR {
                            let mut ext_irqs = 0u64;
                            self.spi[idx].write16(
                                crate::peripherals::spi::SSPDR,
                                val,
                                &mut ext_irqs,
                            );
                            self.raise_irqs_u64(ext_irqs);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 2, addr, val as u32, core);
                            }
                            return;
                        }
                    }
                    if let Some(idx) = i2c_instance {
                        if word_offset_for_narrow == crate::peripherals::i2c::IC_DATA_CMD {
                            let mut ext_irqs = 0u64;
                            self.i2c[idx].write32(
                                crate::peripherals::i2c::IC_DATA_CMD,
                                val as u32,
                                0,
                                &mut ext_irqs,
                            );
                            self.raise_irqs_u64(ext_irqs);
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 2, addr, val as u32, core);
                            }
                            return;
                        }
                    }
                    match (base, word_offset_for_narrow) {
                        // ADC FIFO read-only: see matching comment in
                        // `write8` above. Swallow halfword writes so the
                        // RMW path doesn't silently pop a sample.
                        (ADC_BASE, crate::peripherals::adc::FIFO) => {
                            if self.mmio_trace_enabled {
                                self.emit_mmio_trace('W', 2, addr, val as u32, core);
                            }
                            return;
                        }
                        _ => {}
                    }
                    match base {
                        0x4000_0000 => {
                            // SYSINFO: read-only, ignore halfword writes
                        }
                        0x400D_0000 => {
                            // QMI: do RMW on the word
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let old_word = self.qmi_read(reg_offset);
                            let mut halves: [u16; 2] = [old_word as u16, (old_word >> 16) as u16];
                            halves[half_idx] = val;
                            self.qmi_write(reg_offset, (halves[0] as u32) | ((halves[1] as u32) << 16));
                        }
                        0x4001_0000 | 0x4005_0000 | 0x4005_8000
                        | 0x4004_8000 | 0x400E_8000 => {
                            // CLOCKS / PLL_SYS / PLL_USB / XOSC / ROSC:
                            // same subword-alias strategy as `write8`
                            // (see the comment there).
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    0x4001_0000 => self.clocks_read(reg_offset),
                                    0x4005_0000 => self.pll_sys_read(reg_offset),
                                    0x4005_8000 => self.pll_usb_read(reg_offset),
                                    0x4004_8000 => self.xosc_read(reg_offset),
                                    _ => self.rosc_read(reg_offset),
                                };
                                let mut halves: [u16; 2] =
                                    [old_word as u16, (old_word >> 16) as u16];
                                halves[half_idx] = val;
                                (
                                    (halves[0] as u32) | ((halves[1] as u32) << 16),
                                    0,
                                )
                            } else {
                                ((val as u32) << (half_idx * 16), alias)
                            };
                            match base {
                                0x4001_0000 => self.clocks_write(reg_offset, word_val, pass_alias),
                                0x4005_0000 => self.pll_sys_write(reg_offset, word_val, pass_alias),
                                0x4005_8000 => self.pll_usb_write(reg_offset, word_val, pass_alias),
                                0x4004_8000 => self.xosc_write(reg_offset, word_val, pass_alias),
                                _ => self.rosc_write(reg_offset, word_val, pass_alias),
                            }
                        }
                        TIMER0_BASE | TIMER1_BASE | TICKS_BASE => {
                            // TIMER / TICKS halfword access: same
                            // subword-alias strategy as CLOCKS.
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    TIMER0_BASE => self.timer0.read32(reg_offset),
                                    TIMER1_BASE => self.timer1.read32(reg_offset),
                                    _ => self.ticks.read32(reg_offset),
                                };
                                let mut halves: [u16; 2] =
                                    [old_word as u16, (old_word >> 16) as u16];
                                halves[half_idx] = val;
                                (
                                    (halves[0] as u32) | ((halves[1] as u32) << 16),
                                    0,
                                )
                            } else {
                                ((val as u32) << (half_idx * 16), alias)
                            };
                            match base {
                                TIMER0_BASE => self.timer0.write32(reg_offset, word_val, pass_alias),
                                TIMER1_BASE => self.timer1.write32(reg_offset, word_val, pass_alias),
                                _ => {
                                    if self.ticks.write32(reg_offset, word_val, pass_alias) {
                                        self.timer0.invalidate_lazy();
                                        self.timer1.invalidate_lazy();
                                    }
                                }
                            }
                        }
                        0x4002_0000 => {
                            // RESETS: only word-aligned writes meaningful, ignore halfword
                        }
                        SYSCFG_BASE | TBMAN_BASE | GLITCH_DETECTOR_BASE
                        | PSM_BASE | WATCHDOG_BASE => {
                            // Inert / PSM / WATCHDOG halfword path — subword
                            // alias preservation.
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    SYSCFG_BASE => self.syscfg.read32(reg_offset),
                                    TBMAN_BASE => self.tbman.read32(reg_offset),
                                    GLITCH_DETECTOR_BASE => self.glitch.read32(reg_offset),
                                    PSM_BASE => self.psm.read32(reg_offset),
                                    _ => self.watchdog.read32(reg_offset),
                                };
                                let mut halves: [u16; 2] =
                                    [old_word as u16, (old_word >> 16) as u16];
                                halves[half_idx] = val;
                                ((halves[0] as u32) | ((halves[1] as u32) << 16), 0u32)
                            } else {
                                ((val as u32) << (half_idx * 16), alias)
                            };
                            match base {
                                SYSCFG_BASE => self.syscfg.write32(reg_offset, word_val, pass_alias),
                                TBMAN_BASE => self.tbman.write32(reg_offset, word_val, pass_alias),
                                GLITCH_DETECTOR_BASE => self.glitch.write32(reg_offset, word_val, pass_alias),
                                PSM_BASE => self.psm.write32(reg_offset, word_val, pass_alias),
                                _ => {
                                    if self.watchdog.write32(reg_offset, word_val, pass_alias) {
                                        self.set_watchdog_reset();
                                    }
                                }
                            }
                        }
                        OTP_DATA_BASE => {
                            // OTP narrow halfword write — OR-only fuse
                            // semantics at the word level.
                            let otp_word_off =
                                (addr - OTP_DATA_BASE) & (OTP_DATA_SIZE - 1) & !3;
                            let half_idx = (((addr - OTP_DATA_BASE) >> 1) & 1) as usize;
                            let word_val = (val as u32) << (half_idx * 16);
                            self.otp.write32(otp_word_off, word_val);
                        }
                        TRNG_BASE | SHA256_BASE | POWMAN_BASE => {
                            // TRNG / SHA / POWMAN halfword path — subword alias.
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    TRNG_BASE => self.trng.read32(reg_offset),
                                    SHA256_BASE => self.sha256.read32(reg_offset),
                                    _ => self.powman.read32(reg_offset),
                                };
                                let mut halves: [u16; 2] =
                                    [old_word as u16, (old_word >> 16) as u16];
                                halves[half_idx] = val;
                                ((halves[0] as u32) | ((halves[1] as u32) << 16), 0u32)
                            } else {
                                ((val as u32) << (half_idx * 16), alias)
                            };
                            match base {
                                TRNG_BASE => self.trng.write32(reg_offset, word_val, pass_alias),
                                SHA256_BASE => self.sha256.write32(reg_offset, word_val, pass_alias),
                                _ => {
                                    let mask = self.powman.write32(reg_offset, word_val, pass_alias);
                                    self.raise_irqs_u64(mask);
                                }
                            }
                        }
                        UART0_BASE | UART1_BASE | SPI0_BASE | SPI1_BASE | I2C0_BASE
                        | I2C1_BASE | ADC_BASE | PWM_BASE | IO_BANK0_BASE
                        | PADS_BANK0_BASE => {
                            // Phase 2 peripherals halfword path — subword
                            // alias preservation.
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            let reg_offset = word_addr & 0x0000_0FFF;
                            let (word_val, pass_alias) = if alias == 0 {
                                let old_word = match base {
                                    UART0_BASE => self.uart[0].read32(reg_offset),
                                    UART1_BASE => self.uart[1].read32(reg_offset),
                                    SPI0_BASE => self.spi[0].read32(reg_offset),
                                    SPI1_BASE => self.spi[1].read32(reg_offset),
                                    I2C0_BASE => self.i2c[0].read32(reg_offset),
                                    I2C1_BASE => self.i2c[1].read32(reg_offset),
                                    ADC_BASE => self.adc.read32(reg_offset),
                                    PWM_BASE => self.pwm.read32(reg_offset),
                                    IO_BANK0_BASE => self.io_bank0.read32(reg_offset),
                                    _ => self.pads_bank0.read32(reg_offset),
                                };
                                let mut halves: [u16; 2] =
                                    [old_word as u16, (old_word >> 16) as u16];
                                halves[half_idx] = val;
                                ((halves[0] as u32) | ((halves[1] as u32) << 16), 0u32)
                            } else {
                                ((val as u32) << (half_idx * 16), alias)
                            };
                            let mut ext_irqs = 0u64;
                            match base {
                                UART0_BASE => self.uart[0].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                UART1_BASE => self.uart[1].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                SPI0_BASE => self.spi[0].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                SPI1_BASE => self.spi[1].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                I2C0_BASE => self.i2c[0].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                I2C1_BASE => self.i2c[1].write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                ADC_BASE => self.adc.write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                PWM_BASE => self.pwm.write32(reg_offset, word_val, pass_alias, &mut ext_irqs),
                                IO_BANK0_BASE => self.io_bank0.write32(reg_offset, word_val, pass_alias),
                                _ => self.pads_bank0.write32(reg_offset, word_val, pass_alias),
                            }
                            self.raise_irqs_u64(ext_irqs);
                        }
                        0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {} // PIO: 32-bit access only
                        _ => {
                            let word_addr = canonical & !3;
                            let half_idx = ((canonical >> 1) & 1) as usize;
                            self.warn_unmodelled_mmio(word_addr);
                            let old_word = *self.peripheral_regs.get(&word_addr).unwrap_or(&0);
                            let mut halves: [u16; 2] = [old_word as u16, (old_word >> 16) as u16];
                            let old_half = halves[half_idx];
                            halves[half_idx] = match alias {
                                0 => val,
                                1 => old_half ^ val,
                                2 => old_half | val,
                                3 => old_half & !val,
                                _ => unreachable!(),
                            };
                            let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                            self.peripheral_regs.insert(word_addr, new_word);
                        }
                    }
                }
            }
            0xD => {
                // SIO narrow halfword write — mirror the `write8` SIO
                // arm above. See that comment for rationale including
                // GPIO_OUT-family replication across both halves.
                let word_addr = addr & !3;
                let reg_offset = word_addr & 0xFFF;
                if Self::is_sio_gpio_out_replicating_reg(reg_offset) {
                    let replicated = u32::from(val) * 0x0001_0001;
                    self.write32(word_addr, replicated, core);
                } else {
                    let half_idx = ((addr >> 1) & 1) as usize;
                    // core is the outer write8/16/32 param
                    let old_word = match reg_offset {
                        0x004 => self.gpio_in,
                        0x008 => self.read_gpio_hi_in(),
                        _ => self.sio.read32(reg_offset, core as usize),
                    };
                    let mut halves: [u16; 2] =
                        [old_word as u16, (old_word >> 16) as u16];
                    halves[half_idx] = val;
                    let merged = (halves[0] as u32) | ((halves[1] as u32) << 16);
                    self.write32(word_addr, merged, core);
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_write16(addr, val),
            0xE if Self::is_coresight_trace(addr) => {
                let word_off = (addr - CORESIGHT_TRACE_BASE) & !3;
                let half_idx = ((addr >> 1) & 1) as usize;
                let old = self.coresight_trace.read32(word_off);
                let mut halves: [u16; 2] = [old as u16, (old >> 16) as u16];
                halves[half_idx] = val;
                let merged = (halves[0] as u32) | ((halves[1] as u32) << 16);
                self.coresight_trace.write32(word_off, merged, 0);
            }
            _ => {}
        }
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('W', 2, addr, val as u32, core);
        }
    }

    // --- 32-bit access ---

    pub fn read32(&mut self, addr: u32, core: u8) -> u32 {
        let addr = canon_oracle_addr(addr);
        // Phase 0b.1 Commit B: PPB addresses are routed through
        // `CortexM33::bus_read32` before reaching here. Anything at
        // `0xE0..0xEF` that is not boot RAM is a caller bug.
        debug_assert!(addr >> 28 != 0xE || Self::is_boot_ram(addr),
            "PPB address 0x{:08X} reached Bus::read32 — use CortexM33::bus_read32 wrapper",
            addr);
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = match region {
            0x2 => addr & 0x00FF_FFFF, // strip SRAM alias bits [27:24]
            _   => addr & 0x0FFF_FFFF,
        };
        let val = match region {
            0x0 if offset + 3 < 0x8000 => self.memory.rom_read32(offset),
            0x1 if Self::is_xip_sram(addr) && self.flash_loaded => {
                // XIP SRAM (0x1C00_0000): when flash is loaded, the bootrom
                // reads flash through this window. Map reads to flash content
                // using the current window offset tracked by QMI configuration.
                let xip_offset = (addr - 0x1C00_0000) + self.xip_cache_offset;
                self.memory.xip_read32(xip_offset)
            }
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_read32(addr),
            0x1 => {
                if !self.flash_loaded {
                    self.atomics.set_bus_fault(core as usize, addr);
                    if self.mmio_trace_enabled {
                        self.emit_mmio_trace('R', 4, addr, 0, core);
                    }
                    return 0;
                }
                self.memory.xip_read32(offset)
            }
            0x2 if (offset + 3) < SRAM_SIZE as u32 => {
                let v = self.memory.sram_read32(offset);
                // sram_bank_wait removed: bank penalty on instruction fetch only.
                v
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let offset = canonical & 0x0000_0FFF;
                // RESETS Bus-level guard (HLD V5 §5.3). Reset-gated
                // peripherals return 0 without reaching the peripheral
                // module. Inline — no separate peripheral_dispatch.rs
                // file per V5 §8.
                if self.is_held_in_reset_base(base) {
                    0
                } else {
                    match base {
                        0x4000_0000 => self.sysinfo_read(offset),
                        0x4002_0000 => self.resets_read(offset),
                        0x4001_0000 => self.clocks_read(offset),
                        0x4004_8000 => self.xosc_read(offset),
                        0x400E_8000 => self.rosc_read(offset),
                        0x4005_0000 => self.pll_sys_read(offset),
                        0x4005_8000 => self.pll_usb_read(offset),
                        0x400D_0000 => self.qmi_read(offset),
                        TIMER0_BASE => self.timer0.read32(offset),
                        TIMER1_BASE => self.timer1.read32(offset),
                        TICKS_BASE => self.ticks.read32(offset),
                        UART0_BASE => self.uart[0].read32(offset),
                        UART1_BASE => self.uart[1].read32(offset),
                        SPI0_BASE => self.spi[0].read32(offset),
                        SPI1_BASE => self.spi[1].read32(offset),
                        I2C0_BASE => self.i2c[0].read32(offset),
                        I2C1_BASE => self.i2c[1].read32(offset),
                        ADC_BASE => self.adc.read32(offset),
                        PWM_BASE => self.pwm.read32(offset),
                        IO_BANK0_BASE => self.io_bank0.read32(offset),
                        PADS_BANK0_BASE => self.pads_bank0.read32(offset),
                        DMA_BASE => self.dma.read32(offset),
                        SYSCFG_BASE => self.syscfg.read32(offset),
                        TBMAN_BASE => self.tbman.read32(offset),
                        GLITCH_DETECTOR_BASE => self.glitch.read32(offset),
                        PSM_BASE => self.psm.read32(offset),
                        WATCHDOG_BASE => self.watchdog.read32(offset),
                        // OTP_DATA is a 16 KB flat aperture (HLD V5 §7.D.4).
                        // No APB aliases — bits 12,13 are real OTP offset
                        // bits. All four 4 KB sub-windows collapse into the
                        // same `base` after the `!0x3000` alias strip, so we
                        // recover the true byte offset from the raw `addr`.
                        OTP_DATA_BASE => {
                            self.otp.read32((addr - OTP_DATA_BASE) & (OTP_DATA_SIZE - 1))
                        }
                        TRNG_BASE => self.trng.read32(offset),
                        SHA256_BASE => self.sha256.read32(offset),
                        POWMAN_BASE => self.powman.read32(offset),
                        0x5020_0000 => self.pio[0].read32(offset),
                        0x5030_0000 => self.pio[1].read32(offset),
                        0x5040_0000 => self.pio[2].read32(offset),
                        _ => {
                            self.warn_unmodelled_mmio(canonical);
                            *self.peripheral_regs.get(&canonical).unwrap_or(&0)
                        }
                    }
                }
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                debug_assert!(
                    !crate::core::PerCoreSio::owns_offset(reg_offset),
                    "DIV/INTERP addr 0x{:08X} reached Bus::read32 — use CortexM33::bus_read32 wrapper",
                    addr
                );
                match reg_offset {
                    0x004 => self.gpio_in,
                    0x008 => self.read_gpio_hi_in(),
                    _ => {
                        self.sio.read32(reg_offset, core as usize)
                    }
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_read32(addr),
            0xE if Self::is_coresight_trace(addr) => {
                self.coresight_trace.read32(addr - CORESIGHT_TRACE_BASE)
            }
            _ => {
                self.atomics.set_bus_fault(core as usize, addr);
                0
            }
        };
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('R', 4, addr, val, core);
        }
        val
    }

    pub fn write32(&mut self, addr: u32, val: u32, core: u8) {
        let addr = canon_oracle_addr(addr);
        // Phase 0b.1 Commit B: PPB addresses are routed through
        // `CortexM33::bus_write32` before reaching here.
        debug_assert!(addr >> 28 != 0xE || Self::is_boot_ram(addr),
            "PPB address 0x{:08X} reached Bus::write32 — use CortexM33::bus_write32 wrapper",
            addr);
        // RV32A: invalidate any LR/SC reservation at this word.
        // HLD §4.7.
        self.invalidate_reservation_at(addr);
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x00FF_FFFF;
        match region {
            0x1 if Self::is_xip_sram(addr) => {
                self.xip_sram_write32(addr, val);
                self.invalidate_pc_range(addr, 4);
            }
            0x2 if (offset + 3) < SRAM_SIZE as u32 => {
                let sram_alias = (addr >> 24) & 0x3;
                if sram_alias == 0 {
                    self.memory.sram_write32(offset, val);
                } else {
                    let old = self.memory.sram_read32(offset);
                    let new_val = match sram_alias {
                        1 => old ^ val,
                        2 => old | val,
                        3 => old & !val,
                        _ => unreachable!(),
                    };
                    self.memory.sram_write32(offset, new_val);
                }
                // sram_bank_wait removed: bank penalty on instruction fetch only.
                self.invalidate_pc_range(addr, 4);
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let offset = canonical & 0x0000_0FFF;
                // RESETS Bus-level guard (HLD V5 §5.3). Reset-gated
                // peripherals drop writes silently (inline per V5 §8).
                if self.is_held_in_reset_base(base) {
                    // no-op
                } else {
                    match base {
                        0x4002_0000 => self.resets_write(offset, val, alias),
                        0x400D_0000 => self.qmi_write(offset, val),
                        0x4001_0000 => self.clocks_write(offset, val, alias),
                        0x4005_0000 => self.pll_sys_write(offset, val, alias),
                        0x4005_8000 => self.pll_usb_write(offset, val, alias),
                        0x4004_8000 => self.xosc_write(offset, val, alias),
                        0x400E_8000 => self.rosc_write(offset, val, alias),
                        // SYSINFO (0x4000_0000): read-only, ignore writes
                        0x4000_0000 => {}
                        TIMER0_BASE => self.timer0.write32(offset, val, alias),
                        TIMER1_BASE => self.timer1.write32(offset, val, alias),
                        TICKS_BASE => {
                            // HLD V5 §5.4: a TICKS write that can shift
                            // the tick rate must invalidate TIMER0/1
                            // cached match cycles. TicksRegs::write32
                            // returns `true` for any TIMER0/1 domain
                            // CTRL/CYCLES/COUNT touch.
                            let invalidate = self.ticks.write32(offset, val, alias);
                            if invalidate {
                                self.timer0.invalidate_lazy();
                                self.timer1.invalidate_lazy();
                            }
                        }
                        UART0_BASE => {
                            let mut ext_irqs = 0u64;
                            self.uart[0].write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        UART1_BASE => {
                            let mut ext_irqs = 0u64;
                            self.uart[1].write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        SPI0_BASE => {
                            let mut ext_irqs = 0u64;
                            self.spi[0].write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        SPI1_BASE => {
                            let mut ext_irqs = 0u64;
                            self.spi[1].write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        I2C0_BASE => {
                            let mut ext_irqs = 0u64;
                            self.i2c[0].write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        I2C1_BASE => {
                            let mut ext_irqs = 0u64;
                            self.i2c[1].write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        ADC_BASE => {
                            let mut ext_irqs = 0u64;
                            self.adc.write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        PWM_BASE => {
                            let mut ext_irqs = 0u64;
                            self.pwm.write32(offset, val, alias, &mut ext_irqs);
                            self.raise_irqs_u64(ext_irqs);
                        }
                        IO_BANK0_BASE => self.io_bank0.write32(offset, val, alias),
                        PADS_BANK0_BASE => self.pads_bank0.write32(offset, val, alias),
                        DMA_BASE => self.dma.write32(offset, val, alias),
                        SYSCFG_BASE => self.syscfg.write32(offset, val, alias),
                        TBMAN_BASE => self.tbman.write32(offset, val, alias),
                        GLITCH_DETECTOR_BASE => self.glitch.write32(offset, val, alias),
                        PSM_BASE => self.psm.write32(offset, val, alias),
                        WATCHDOG_BASE => {
                            if self.watchdog.write32(offset, val, alias) {
                                self.set_watchdog_reset();
                            }
                        }
                        // OTP 16 KB flat aperture (HLD V5 §7.D.4). OR-only
                        // fuse semantics — alias ignored (see module doc).
                        OTP_DATA_BASE => {
                            let otp_off = (addr - OTP_DATA_BASE) & (OTP_DATA_SIZE - 1);
                            self.otp.write32(otp_off, val);
                        }
                        TRNG_BASE => self.trng.write32(offset, val, alias),
                        SHA256_BASE => self.sha256.write32(offset, val, alias),
                        POWMAN_BASE => {
                            // POWMAN write32 returns a NVIC raise mask
                            // when a write transitions level-sensitive
                            // INTS.TIMER from 0 → 1 (e.g. enabling
                            // INTE.TIMER while INTR.TIMER is already
                            // latched). V12 §3.2 INTE-gating fix.
                            let mask = self.powman.write32(offset, val, alias);
                            self.raise_irqs_u64(mask);
                        }
                        0x5020_0000 => self.pio[0].write32(offset, val, alias),
                        0x5030_0000 => self.pio[1].write32(offset, val, alias),
                        0x5040_0000 => self.pio[2].write32(offset, val, alias),
                        _ => {
                            // Existing HashMap path with alias logic
                            self.warn_unmodelled_mmio(canonical);
                            let old = *self.peripheral_regs.get(&canonical).unwrap_or(&0);
                            let new_val = match alias {
                                0 => val,
                                1 => old ^ val,
                                2 => old | val,
                                3 => old & !val,
                                _ => unreachable!(),
                            };
                            self.peripheral_regs.insert(canonical, new_val);
                        }
                    }
                }
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                debug_assert!(
                    !crate::core::PerCoreSio::owns_offset(reg_offset),
                    "DIV/INTERP addr 0x{:08X} reached Bus::write32 — use CortexM33::bus_write32 wrapper",
                    addr
                );
                self.sio.write32(reg_offset, val, core as usize);
                // FIFO_WR event signaling: set event_flag for receiver core.
                if let Some(receiver) = self.sio.pending_fifo_event.take() {
                    self.atomics.set_event_flag(receiver);
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_write32(addr, val),
            0xE if Self::is_coresight_trace(addr) => {
                // CORESIGHT_TRACE 4 KB aperture — plain storage
                // round-trip. No APB alias encoding; writes always
                // replace the stored word.
                let offset = addr - CORESIGHT_TRACE_BASE;
                self.coresight_trace.write32(offset, val, 0);
            }
            // Unmapped regions raise a precise bus fault so flush-style
            // writers (Phase 7 Stage B lazy FP) and other speculative
            // stores see the failure. Mirrors the read32 unmapped path.
            _ => {
                self.atomics.set_bus_fault(core as usize, addr);
            }
        }
        if self.mmio_trace_enabled {
            self.emit_mmio_trace('W', 4, addr, val, core);
        }
    }

    // -----------------------------------------------------------------
    // DMA wiring (HLD V5 §5.6, Phase 3)
    // -----------------------------------------------------------------

    /// Snapshot every peripheral's DREQ condition into a 64-bit bitmap.
    /// Bit positions follow `dreq.rs` constants — RP2350 datasheet
    /// §12.6.4.2 Table 124. Called by `Dma::tick` before arbitration so
    /// the DMA sees a consistent snapshot across all channels.
    pub fn collect_dreqs(&self) -> u64 {
        let mut bits = 0u64;

        // PIO0 / PIO1 / PIO2 — four SM × (TX | RX) per block.
        for sm in 0..4 {
            if self.pio[0].tx_dreq(sm) {
                bits |= 1u64 << (sm as u64);          // DREQ 0..3
            }
            if self.pio[0].rx_dreq(sm) {
                bits |= 1u64 << (4 + sm as u64);      // DREQ 4..7
            }
            if self.pio[1].tx_dreq(sm) {
                bits |= 1u64 << (8 + sm as u64);      // DREQ 8..11
            }
            if self.pio[1].rx_dreq(sm) {
                bits |= 1u64 << (12 + sm as u64);     // DREQ 12..15
            }
            if self.pio[2].tx_dreq(sm) {
                bits |= 1u64 << (16 + sm as u64);     // DREQ 16..19
            }
            if self.pio[2].rx_dreq(sm) {
                bits |= 1u64 << (20 + sm as u64);     // DREQ 20..23
            }
        }

        // SPI0/1 TX/RX (DREQ 24..27).
        if self.spi[0].tx_dreq() {
            bits |= 1u64 << DREQ_SPI0_TX;
        }
        if self.spi[0].rx_dreq() {
            bits |= 1u64 << DREQ_SPI0_RX;
        }
        if self.spi[1].tx_dreq() {
            bits |= 1u64 << DREQ_SPI1_TX;
        }
        if self.spi[1].rx_dreq() {
            bits |= 1u64 << DREQ_SPI1_RX;
        }

        // UART0/1 TX/RX (DREQ 28..31).
        if self.uart[0].tx_dreq() {
            bits |= 1u64 << DREQ_UART0_TX;
        }
        if self.uart[0].rx_dreq() {
            bits |= 1u64 << DREQ_UART0_RX;
        }
        if self.uart[1].tx_dreq() {
            bits |= 1u64 << DREQ_UART1_TX;
        }
        if self.uart[1].rx_dreq() {
            bits |= 1u64 << DREQ_UART1_RX;
        }

        // PWM wrap DREQs (32..43) — one-shot-per-wrap, not modelled in V1.

        // I2C0/1 TX/RX (DREQ 44..47).
        if self.i2c[0].tx_dreq() {
            bits |= 1u64 << DREQ_I2C0_TX;
        }
        if self.i2c[0].rx_dreq() {
            bits |= 1u64 << DREQ_I2C0_RX;
        }
        if self.i2c[1].tx_dreq() {
            bits |= 1u64 << DREQ_I2C1_TX;
        }
        if self.i2c[1].rx_dreq() {
            bits |= 1u64 << DREQ_I2C1_RX;
        }

        // ADC (DREQ 48).
        if self.adc.dreq() {
            bits |= 1u64 << 48;
        }

        // XIP stream/QMI (49..51), HSTX (52), CORESIGHT (53), SHA256 (54)
        // — not modelled in V1.

        // FORCE (bit 63) — always asserted.
        bits |= 1u64 << 63;

        bits
    }

    /// Drive the DMA by one cycle. Swaps the DMA out of `self` to avoid
    /// cross-borrows while it issues transfers through the bus, then
    /// restores it and routes any pending IRQs through `irq_pending`.
    ///
    /// Per HLD V5 §5.6 ordering contract: peripherals tick first (to
    /// produce DREQ), then `tick_dma` consumes the snapshot.
    pub fn tick_dma(&mut self) {
        let mut dma = std::mem::take(&mut self.dma);
        dma.tick(self);
        dma.route_irqs(&self.atomics);
        self.dma = dma;
    }
}

/// Extra wait-state for SRAM bank access (retained for documentation).
/// Banks 2 and 6 have +1 cycle on RP2350 (measured on silicon via DWT CYCCNT).
/// No longer called from data-access paths. Bank 2/6 penalty is now
/// modeled on instruction fetch only, conditional on sequentiality.
/// Returns 0 during burst mode (STM/LDM/PUSH/POP) — the SRAM controller
/// handles sequential accesses without per-word bank penalties.
#[allow(dead_code)]
fn sram_bank_wait(addr: u32, burst: bool) -> u32 {
    if burst {
        return 0;
    }
    let offset = addr & 0x000F_FFFF;
    if offset < 0x8_0000 {
        // Striped SRAM0-7
        let bank = (offset >> 2) & 7;
        if bank == 2 || bank == 6 {
            1
        } else {
            0
        }
    } else {
        0 // SRAM8-9 non-striped: no extra wait
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

// ===================================================================
// `CoreBus` impl — Phase 3 Stage 2 (LLD V7 §1).
//
// Every method is a one-liner forward to an existing inherent `Bus`
// method or field. The trait is the generic surface used by
// `CortexM33::step<B: CoreBus>`; in Stage 2 the only implementor is
// `Bus`. Stage 5 adds `WorkerBus`.
// ===================================================================

use crate::core::bus_trait::CoreBus;

impl CoreBus for Bus {
    #[inline(always)]
    fn read8(&mut self, addr: u32, core: u8) -> u8 {
        Bus::read8(self, addr, core)
    }
    #[inline(always)]
    fn read16(&mut self, addr: u32, core: u8) -> u16 {
        Bus::read16(self, addr, core)
    }
    #[inline(always)]
    fn read32(&mut self, addr: u32, core: u8) -> u32 {
        Bus::read32(self, addr, core)
    }
    #[inline(always)]
    fn write8(&mut self, addr: u32, val: u8, core: u8) {
        Bus::write8(self, addr, val, core)
    }
    #[inline(always)]
    fn write16(&mut self, addr: u32, val: u16, core: u8) {
        Bus::write16(self, addr, val, core)
    }
    #[inline(always)]
    fn write32(&mut self, addr: u32, val: u32, core: u8) {
        Bus::write32(self, addr, val, core)
    }

    #[inline(always)]
    fn set_active_pc(&mut self, pc: u32, core: u8) {
        Bus::set_active_pc(self, pc, core)
    }

    #[inline(always)]
    fn bus_fault(&self, core: u8) -> bool {
        Bus::bus_fault(self, core as usize)
    }
    #[inline(always)]
    fn bus_fault_addr(&self, core: u8) -> u32 {
        Bus::bus_fault_addr(self, core as usize)
    }
    #[inline(always)]
    fn clear_bus_fault(&mut self, core: u8) {
        Bus::clear_bus_fault(self, core as usize)
    }

    #[inline(always)]
    fn set_burst_mode(&mut self, on: bool) {
        if on {
            Bus::set_burst_mode(self);
        } else {
            Bus::clear_burst_mode(self);
        }
    }

    #[inline(always)]
    fn add_extra_wait_states(&mut self, n: u32) {
        Bus::add_extra_wait_states(self, n)
    }

    #[inline(always)]
    fn take_extra_wait_states(&mut self) -> u32 {
        Bus::take_extra_wait_states(self)
    }

    // --- TRANSIENT (Stage 2) ------------------------------------------

    #[inline(always)]
    fn atomics(&self) -> &Arc<crate::threaded::CoreAtomics> {
        &self.atomics
    }

    // --- GPIO OUT / OE / IN (Phase 3 Stage 6a) -----------------------
    //
    // Forward straight to the inherent `Sio` fields and the Bus-level
    // `gpio_in` so the `Bus` path behaves identically to the direct
    // `bus.sio.gpio_out = x` accesses the call sites used before.

    #[inline(always)]
    fn gpio_read_out(&self) -> u32 {
        self.sio.gpio_out
    }
    #[inline(always)]
    fn gpio_write_out(&mut self, val: u32) {
        self.sio.gpio_out = val;
    }
    #[inline(always)]
    fn gpio_set_out(&mut self, mask: u32) {
        self.sio.gpio_out |= mask;
    }
    #[inline(always)]
    fn gpio_clear_out(&mut self, mask: u32) {
        self.sio.gpio_out &= !mask;
    }
    #[inline(always)]
    fn gpio_xor_out(&mut self, mask: u32) {
        self.sio.gpio_out ^= mask;
    }

    #[inline(always)]
    fn gpio_read_oe(&self) -> u32 {
        self.sio.gpio_oe
    }
    #[inline(always)]
    fn gpio_write_oe(&mut self, val: u32) {
        self.sio.gpio_oe = val;
    }
    #[inline(always)]
    fn gpio_set_oe(&mut self, mask: u32) {
        self.sio.gpio_oe |= mask;
    }
    #[inline(always)]
    fn gpio_clear_oe(&mut self, mask: u32) {
        self.sio.gpio_oe &= !mask;
    }
    #[inline(always)]
    fn gpio_xor_oe(&mut self, mask: u32) {
        self.sio.gpio_oe ^= mask;
    }

    #[inline(always)]
    fn gpio_read_in(&self) -> u32 {
        self.gpio_in
    }

    #[inline(always)]
    fn extra_wait_states(&self) -> u32 {
        Bus::extra_wait_states(self)
    }
    #[inline(always)]
    fn reset_extra_wait_states(&mut self) {
        Bus::reset_extra_wait_states(self)
    }

    #[inline(always)]
    fn last_fetch_addr(&self) -> u32 {
        self.last_fetch_addr
    }
    #[inline(always)]
    fn set_last_fetch_addr(&mut self, addr: u32) {
        self.last_fetch_addr = addr;
    }

    #[inline(always)]
    fn mmio_trace_enabled(&self) -> bool {
        self.mmio_trace_enabled
    }
    #[inline(always)]
    fn emit_mmio_trace(&mut self, rw: char, size: u32, addr: u32, val: u32, core: u8) {
        Bus::emit_mmio_trace(self, rw, size, addr, val, core)
    }
}

#[cfg(test)]
mod corebus_trait_tests {
    use super::*;
    use crate::core::CoreBus;

    /// Compile-time + smoke check that `CoreBus for Bus` covers every
    /// method the trait declares and that the trait is reachable via a
    /// `dyn CoreBus` coercion. Phase 3 Stage 2 (LLD V7 §1).
    #[test]
    fn bus_core_bus_impl_covers_all_methods() {
        let atomics = Arc::new(CoreAtomics::default());
        let mut bus = Bus::with_atomics(Arc::clone(&atomics));

        // dyn-dispatch path — compile-time check that every trait method
        // is dyn-safe and reachable through the trait object.
        let bus_dyn: &mut dyn CoreBus = &mut bus;

        // Canonical 13-method surface.
        let _ = bus_dyn.read32(0, 0);
        bus_dyn.write32(0, 0, 0);
        let _ = bus_dyn.read16(0, 0);
        bus_dyn.write16(0, 0, 0);
        let _ = bus_dyn.read8(0, 0);
        bus_dyn.write8(0, 0, 0);
        bus_dyn.set_active_pc(0x2000_0000, 0);
        let _fault = bus_dyn.bus_fault(0);
        let _addr = bus_dyn.bus_fault_addr(0);
        bus_dyn.clear_bus_fault(0);
        bus_dyn.set_burst_mode(true);
        bus_dyn.set_burst_mode(false);
        bus_dyn.add_extra_wait_states(3);
        let n = bus_dyn.take_extra_wait_states();
        assert_eq!(n, 3, "take_extra_wait_states should return the added 3");
        assert_eq!(
            bus_dyn.take_extra_wait_states(),
            0,
            "take_extra_wait_states should drain to zero"
        );

        // Transient accessors (removed in later Phase 3 stages — see
        // `core/bus_trait.rs` for the teardown schedule).
        let _atomics: &Arc<CoreAtomics> = bus_dyn.atomics();
        // GPIO OUT/OE/IN typed accessors (Phase 3 Stage 6a).
        let _ = bus_dyn.gpio_read_out();
        bus_dyn.gpio_write_out(0);
        bus_dyn.gpio_set_out(0);
        bus_dyn.gpio_clear_out(0);
        bus_dyn.gpio_xor_out(0);
        let _ = bus_dyn.gpio_read_oe();
        bus_dyn.gpio_write_oe(0);
        bus_dyn.gpio_set_oe(0);
        bus_dyn.gpio_clear_oe(0);
        bus_dyn.gpio_xor_oe(0);
        let _ = bus_dyn.gpio_read_in();
        let _ = bus_dyn.extra_wait_states();
        bus_dyn.reset_extra_wait_states();
        let _ = bus_dyn.mmio_trace_enabled();
        // emit_mmio_trace is a no-op unless mmio_trace_enabled is true, but we
        // still call it to validate the signature.
        bus_dyn.emit_mmio_trace('R', 4, 0x2000_0000, 0, 0);
    }
}

// ============================================================================
// Bus observability: warn-once on unmodelled MMIO (HLD V5 §4.A1) +
// watchdog-reset flag (HLD V5 §7.D.3)
// ============================================================================
// ============================================================================
// Bus observability: warn-once on unmodelled MMIO (HLD V5 §4.A1) +
// watchdog-reset flag (HLD V5 §7.D.3)
// ============================================================================

/// Warn-assertion tests use `tracing::subscriber::with_default`, which
/// installs a *thread-local* subscriber. If future parallel-test
/// infrastructure shares subscribers across threads, these tests would
/// need `#[serial]` via the `serial_test` crate.
#[cfg(test)]
mod bus_observability {
    use crate::bus::Bus;
    use std::sync::{Arc, Mutex};
    use tracing::{Event, Metadata, Subscriber};
    use tracing::span::{Attributes, Id, Record};

    /// Capture `tracing` events (name + level + fields) into a shared
    /// `Vec<String>` so tests can assert on warn-once semantics.
    ///
    /// The subscriber ignores spans (tests only care about events) and
    /// records each event as `"<LEVEL> <format!(fields)>"`, where the
    /// fields are flattened into a single string via a one-shot
    /// `field::Visit`.
    #[derive(Default)]
    struct CaptureSubscriber {
        events: Arc<Mutex<Vec<String>>>,
    }

    struct FieldRecorder(String);
    impl tracing::field::Visit for FieldRecorder {
        fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
            use std::fmt::Write;
            if !self.0.is_empty() {
                self.0.push(' ');
            }
            let _ = write!(self.0, "{}={:?}", field.name(), value);
        }
    }

    impl Subscriber for CaptureSubscriber {
        fn enabled(&self, _metadata: &Metadata<'_>) -> bool { true }
        fn new_span(&self, _span: &Attributes<'_>) -> Id { Id::from_u64(1) }
        fn record(&self, _span: &Id, _values: &Record<'_>) {}
        fn record_follows_from(&self, _span: &Id, _follows: &Id) {}
        fn event(&self, event: &Event<'_>) {
            let mut visitor = FieldRecorder(String::new());
            event.record(&mut visitor);
            let meta = event.metadata();
            let line = format!("{} {}", meta.level(), visitor.0);
            self.events.lock().unwrap().push(line);
        }
        fn enter(&self, _span: &Id) {}
        fn exit(&self, _span: &Id) {}
    }

    fn count_warns_matching(events: &[String], addr_hex: &str) -> usize {
        events
            .iter()
            .filter(|line| line.starts_with("WARN"))
            .filter(|line| line.contains(addr_hex))
            .count()
    }

    #[test]
    fn unmodelled_mmio_warn_fires_once_per_address_on_repeated_writes() {
        // HLD V5 §4.A1 testing paragraph: write twice to an unmodelled
        // address, capture events, assert exactly one WARN fired for
        // that address. HLD V5 §4 originally pointed at `0x400F_8000`
        // (SHA-256 base); step 3b models SHA-256, so the warn-once
        // probe moved to `0x4012_0000` — the OTP SBPI controller
        // aperture, distinct from the OTP_DATA aperture at
        // `0x4013_0000` that step 3b does model.
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut bus = Bus::new();
            bus.write32(0x4012_0000, 0xDEAD_BEEF, 0);
            bus.write32(0x4012_0000, 0xCAFE_BABE, 0);
        });
        let events = captured.lock().unwrap();
        let matches = count_warns_matching(&events, "0x40120000");
        assert_eq!(matches, 1,
            "expected exactly one WARN for 0x4012_0000; got {} in {:?}",
            matches, *events);
    }

    #[test]
    fn unmodelled_mmio_warn_fires_on_first_read_too() {
        // Reads at an unmodelled address should also warn-once.
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut bus = Bus::new();
            let _ = bus.read32(0x4012_0000, 0);
            let _ = bus.read32(0x4012_0000, 0);
        });
        let events = captured.lock().unwrap();
        let matches = count_warns_matching(&events, "0x40120000");
        assert_eq!(matches, 1);
    }

    #[test]
    fn unmodelled_mmio_warn_fires_once_per_unique_address() {
        // Two distinct unmodelled addresses → two distinct WARNs. Both
        // live inside the OTP SBPI controller aperture (unmodelled —
        // see note above).
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut bus = Bus::new();
            bus.write32(0x4012_0000, 0, 0);
            bus.write32(0x4012_0004, 0, 0);
        });
        let events = captured.lock().unwrap();
        assert_eq!(count_warns_matching(&events, "0x40120000"), 1);
        assert_eq!(count_warns_matching(&events, "0x40120004"), 1);
    }

    // --- Watchdog reset request flag (HLD V5 §7.D.3) ---------------------

    #[test]
    fn watchdog_reset_flag_default_false() {
        let bus = Bus::new();
        assert!(!bus.watchdog_reset_requested());
    }

    #[test]
    fn watchdog_reset_flag_set_poll_clear() {
        let mut bus = Bus::new();
        bus.set_watchdog_reset();
        assert!(bus.watchdog_reset_requested());
        bus.clear_watchdog_reset();
        assert!(!bus.watchdog_reset_requested());
    }
}
