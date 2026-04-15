//! RP2040 UART peripheral (PL011-derived; datasheet §4.2).
//!
//! Phase 2 of the RP2040 peripheral coverage plan (HLD V7 §5.3 / §6).
//! Two instances live at `0x4003_4000` (UART0) and `0x4003_8000` (UART1).
//! This module implements the observed-register subset pico-sdk's
//! `hello_uart` and `uart/*` examples actually touch, not full PL011
//! coverage.
//!
//! # Register map (offsets relative to `UARTn_BASE`)
//!
//! | Offset  | Name             | Access | Notes                                       |
//! |---------|------------------|--------|---------------------------------------------|
//! | `0x000` | `UARTDR`         | R/W    | Data; byte/halfword side-effect on FIFOs    |
//! | `0x004` | `UARTRSR_ECR`    | R/W    | Receive status/error clear                  |
//! | `0x018` | `UARTFR`         | RO     | Flags (TXFE/TXFF/RXFE/RXFF/BUSY/CTS/DCD/..) |
//! | `0x024` | `UARTIBRD`       | R/W    | Integer baud-rate divisor                   |
//! | `0x028` | `UARTFBRD`       | R/W    | Fractional baud-rate divisor                |
//! | `0x02C` | `UARTLCR_H`      | R/W    | Line control (FEN, WLEN, STP2, PEN, ..)     |
//! | `0x030` | `UARTCR`         | R/W    | Control (UARTEN, TXE, RXE, ..)              |
//! | `0x034` | `UARTIFLS`       | R/W    | FIFO interrupt-level select                 |
//! | `0x038` | `UARTIMSC`       | R/W    | Interrupt mask                              |
//! | `0x03C` | `UARTRIS`        | RO     | Raw interrupt status                        |
//! | `0x040` | `UARTMIS`        | RO     | Masked = RIS & IMSC                         |
//! | `0x044` | `UARTICR`        | W1C    | Interrupt clear                             |
//! | `0x048` | `UARTDMACR`      | R/W    | DMA control                                 |
//! | `0xFE0..0xFEC` | `UARTPERIPHID0..3` | RO | PrimeCell peripheral ID constants    |
//! | `0xFF0..0xFFC` | `UARTPCELLID0..3`  | RO | PrimeCell ID constants                |
//!
//! # Transmit model
//!
//! The 16-entry TX FIFO is modelled as a `VecDeque<u8>`. `UARTDR` byte
//! writes push into the FIFO when FEN=1; when FEN=0 only a single holding
//! register is simulated (the FIFO is bypassed but we still use the
//! VecDeque as backing storage with a `fifos_enabled` gate on capacity).
//!
//! [`UartRegs::tick`] consumes TX FIFO bytes based on the baud rate
//! derived from `UARTIBRD` + `UARTFBRD` + `clk_peri`. The fractional
//! divisor uses the PL011 formula: `baud = clk_peri / (16 * (IBRD +
//! FBRD/64))`. The emulator accumulates "sysclk cycles since last byte"
//! in [`UartRegs::tx_cycle_accum`]; when it crosses
//! `sysclks_per_byte`, one byte is popped.
//!
//! When TX transitions fill-level under `UARTIFLS.TXIFLSEL` (defaulting
//! to ≤ 1/2 full), `UART_TXIS` is raised in `UARTRIS`. `UARTMIS` =
//! `UARTRIS & UARTIMSC` — when masked-enabled, the combined IRQ line
//! is asserted into `bus.irq_pending` via the caller-supplied `irqs`
//! word.
//!
//! # Receive model
//!
//! RX FIFO is present as `VecDeque<u8>` but no stimulus source is wired
//! in Phase 2 (per the plan's deferral). Reads of `UARTDR` return 0 with
//! `UARTFR.RXFE=1`. External RX stimulus belongs to a future phase.
//!
//! # IRQ line aggregation
//!
//! PL011 raises an ORed IRQ line from all sources in `MIS`. On RP2040
//! that line maps to NVIC IRQ 20 (UART0) or IRQ 21 (UART1). The
//! [`UartRegs::tick`] signature takes `nvic_irq` so the same struct
//! serves both instances.
//!
//! # Deferred from Phase 2
//!
//! * External RX stimulus path (FIFO source).
//! * CTS / DCD / DSR / RI modem-flow-control pins.
//! * Break-condition timing (`LBE` in `UARTLCR_H.BRK`).
//! * DMA DREQ generation (`UARTDMACR` is storage-only; Phase 4).
//! * Overrun / framing / parity error insertion.

use std::collections::VecDeque;

use mdpicoem_common::clocks::ClockTree;

/// Offset: `UARTDR` — data register (byte side-effect: FIFO push/pop).
pub const UARTDR: u32 = 0x000;
/// Offset: `UARTRSR_ECR` — receive status / error clear.
pub const UARTRSR_ECR: u32 = 0x004;
/// Offset: `UARTFR` — flag register (read-only).
pub const UARTFR: u32 = 0x018;
/// Offset: `UARTILPR` — IrDA low-power counter. Reads as 0.
pub const UARTILPR: u32 = 0x020;
/// Offset: `UARTIBRD` — integer baud divisor.
pub const UARTIBRD: u32 = 0x024;
/// Offset: `UARTFBRD` — fractional baud divisor (6 bits).
pub const UARTFBRD: u32 = 0x028;
/// Offset: `UARTLCR_H` — line control.
pub const UARTLCR_H: u32 = 0x02C;
/// Offset: `UARTCR` — control.
pub const UARTCR: u32 = 0x030;
/// Offset: `UARTIFLS` — FIFO interrupt level select.
pub const UARTIFLS: u32 = 0x034;
/// Offset: `UARTIMSC` — interrupt mask set/clear.
pub const UARTIMSC: u32 = 0x038;
/// Offset: `UARTRIS` — raw interrupt status (read-only).
pub const UARTRIS: u32 = 0x03C;
/// Offset: `UARTMIS` — masked interrupt status (read-only).
pub const UARTMIS: u32 = 0x040;
/// Offset: `UARTICR` — interrupt clear (W1C).
pub const UARTICR: u32 = 0x044;
/// Offset: `UARTDMACR` — DMA control.
pub const UARTDMACR: u32 = 0x048;

// PrimeCell ID constants (PL011 reset values). Firmware occasionally
// probes these to confirm the block is a PL011; we report the canonical
// values the ARM PL011 TRM documents.
pub const UARTPERIPHID0: u32 = 0xFE0;
pub const UARTPERIPHID1: u32 = 0xFE4;
pub const UARTPERIPHID2: u32 = 0xFE8;
pub const UARTPERIPHID3: u32 = 0xFEC;
pub const UARTPCELLID0: u32 = 0xFF0;
pub const UARTPCELLID1: u32 = 0xFF4;
pub const UARTPCELLID2: u32 = 0xFF8;
pub const UARTPCELLID3: u32 = 0xFFC;

// --- UARTCR bits ------------------------------------------------------
const UARTCR_UARTEN: u32 = 1 << 0;
const UARTCR_TXE: u32 = 1 << 8;
#[allow(dead_code)] // documented bit; RX path not yet wired (Phase 2 deferral)
const UARTCR_RXE: u32 = 1 << 9;

// --- UARTLCR_H bits ---------------------------------------------------
const UARTLCR_H_FEN: u32 = 1 << 4;

// --- UARTFR bits ------------------------------------------------------
const UARTFR_CTS: u32 = 1 << 0;
const UARTFR_BUSY: u32 = 1 << 3;
const UARTFR_RXFE: u32 = 1 << 4;
const UARTFR_TXFF: u32 = 1 << 5;
const UARTFR_RXFF: u32 = 1 << 6;
const UARTFR_TXFE: u32 = 1 << 7;

// --- Interrupt source bits (shared across RIS / IMSC / MIS / ICR) -----
//
// Names follow PL011 TRM §3.3. The full RP2040 set uses 11 bits; we
// surface the ones firmware in the corpus actually touches.
/// CTS modem status change.
pub const UART_INT_CTS: u32 = 1 << 1;
/// Receive IRQ — RX FIFO crossed up over its trigger level.
pub const UART_INT_RX: u32 = 1 << 4;
/// Transmit IRQ — TX FIFO crossed down under its trigger level.
pub const UART_INT_TX: u32 = 1 << 5;
/// Receive timeout.
pub const UART_INT_RT: u32 = 1 << 6;
/// Framing error.
pub const UART_INT_FE: u32 = 1 << 7;
/// Parity error.
pub const UART_INT_PE: u32 = 1 << 8;
/// Break error.
pub const UART_INT_BE: u32 = 1 << 9;
/// Overrun error.
pub const UART_INT_OE: u32 = 1 << 10;
/// Combined mask of all interrupt sources firmware can observe.
const UART_INT_MASK: u32 = UART_INT_CTS
    | UART_INT_RX
    | UART_INT_TX
    | UART_INT_RT
    | UART_INT_FE
    | UART_INT_PE
    | UART_INT_BE
    | UART_INT_OE;

/// FIFO depth (both TX and RX) when `UARTLCR_H.FEN=1`.
pub const UART_FIFO_DEPTH: usize = 16;

/// PL011 peripheral ID bytes (PID0..3). PL011 r1p5 canonical values.
const PERIPH_ID: [u32; 4] = [0x11, 0x10, 0x34, 0x00];
/// PrimeCell ID constants — identical across all PrimeCell peripherals.
const PCELL_ID: [u32; 4] = [0x0D, 0xF0, 0x05, 0xB1];

/// Register storage + state for one PL011-derived UART (RP2040 §4.2).
pub struct UartRegs {
    // -- programmable registers --------------------------------------
    rsr_ecr: u32,
    ibrd: u32,
    fbrd: u32,
    lcr_h: u32,
    cr: u32,
    ifls: u32,
    imsc: u32,
    ris: u32,
    dmacr: u32,
    // -- FIFOs --------------------------------------------------------
    tx_fifo: VecDeque<u8>,
    rx_fifo: VecDeque<u8>,
    /// Accumulated sysclk cycles since the last byte was popped from
    /// the TX FIFO. When `>= sysclks_per_byte`, one byte drains.
    tx_cycle_accum: u64,
    /// NVIC IRQ number this UART raises into `bus.irq_pending`
    /// (UART0=20, UART1=21).
    nvic_irq: u32,
}

impl UartRegs {
    /// Construct a fresh UART at power-on default state. `nvic_irq` is
    /// the NVIC line (20 for UART0, 21 for UART1 on RP2040).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            rsr_ecr: 0,
            ibrd: 0,
            fbrd: 0,
            lcr_h: 0,
            cr: 0,
            // IFLS reset value per PL011 TRM: TX/RX trigger level = 1/2
            // (field encoding 0b010 for each lane).
            ifls: (0b010 << 3) | 0b010,
            imsc: 0,
            ris: 0,
            dmacr: 0,
            tx_fifo: VecDeque::with_capacity(UART_FIFO_DEPTH),
            rx_fifo: VecDeque::with_capacity(UART_FIFO_DEPTH),
            tx_cycle_accum: 0,
            nvic_irq,
        }
    }

    /// Reset every field to post-init defaults.
    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        *self = Self::new(irq);
    }

    /// True iff the UART has no outstanding work (TX FIFO drained, RX
    /// FIFO empty). Used by `Bus::all_peripherals_idle` to keep the
    /// fast-path taken whenever firmware isn't actively transmitting.
    pub fn is_idle(&self) -> bool {
        self.tx_fifo.is_empty() && self.rx_fifo.is_empty() && self.ris == 0
    }

    /// Enabled state: UARTEN bit in UARTCR.
    #[inline]
    fn is_enabled(&self) -> bool {
        (self.cr & UARTCR_UARTEN) != 0
    }

    /// TX-enabled state: UARTEN && TXE.
    #[inline]
    fn is_tx_enabled(&self) -> bool {
        self.is_enabled() && (self.cr & UARTCR_TXE) != 0
    }

    /// FIFO-enabled state: LCR_H.FEN. When clear the "FIFOs" collapse
    /// to 1-deep holding registers.
    #[inline]
    fn fifos_enabled(&self) -> bool {
        (self.lcr_h & UARTLCR_H_FEN) != 0
    }

    /// Effective TX FIFO capacity. 16 when FEN=1, 1 when FEN=0.
    #[inline]
    fn tx_capacity(&self) -> usize {
        if self.fifos_enabled() {
            UART_FIFO_DEPTH
        } else {
            1
        }
    }

    /// Build the UARTFR flag word from the live TX/RX FIFO state. The
    /// BUSY flag reflects TX FIFO non-empty only — the real silicon
    /// also drops BUSY after the shift register finishes, but the
    /// per-cycle drain model surfaces that through the TX FIFO being
    /// empty, so the two collapse into one signal here.
    fn fr_read(&self) -> u32 {
        let mut fr = 0u32;
        // CTS is tied high because we don't model flow control.
        fr |= UARTFR_CTS;
        let cap = self.tx_capacity();
        if self.tx_fifo.is_empty() {
            fr |= UARTFR_TXFE;
        } else {
            fr |= UARTFR_BUSY;
            if self.tx_fifo.len() >= cap {
                fr |= UARTFR_TXFF;
            }
        }
        if self.rx_fifo.is_empty() {
            fr |= UARTFR_RXFE;
        } else if self.rx_fifo.len() >= cap {
            fr |= UARTFR_RXFF;
        }
        fr
    }

    /// Translate `UARTIFLS.TXIFLSEL` (bits [2:0]) into the "drain below"
    /// fill threshold. PL011 TRM §3.3.10: 0=1/8, 1=1/4, 2=1/2, 3=3/4,
    /// 4=7/8, >=5 reserved → fall back to 1/2.
    fn tx_fill_threshold(&self) -> usize {
        let sel = self.ifls & 0x7;
        let cap = UART_FIFO_DEPTH;
        match sel {
            0 => cap / 8,
            1 => cap / 4,
            2 => cap / 2,
            3 => (cap * 3) / 4,
            4 => (cap * 7) / 8,
            _ => cap / 2,
        }
    }

    /// Recompute `ris` from the live state. TX interrupt latches when
    /// the FIFO level drops to at or below the configured trigger level.
    /// RX interrupt latches when FIFO level rises above the threshold.
    ///
    /// Phase 2 only models TX. RX path is deferred.
    fn refresh_tx_interrupt(&mut self) {
        let lvl = self.tx_fifo.len();
        let thresh = self.tx_fill_threshold();
        // PL011 raises TXIS when the FIFO level drops to ≤ threshold.
        if lvl <= thresh {
            self.ris |= UART_INT_TX;
        }
    }

    /// Compute sysclks per transmitted byte given the current
    /// `UARTIBRD` / `UARTFBRD` + `clk_peri` state.
    ///
    /// The PL011 baud rate is `baud = peri_hz / (16 * (IBRD + FBRD/64))`.
    /// At 10 bits/byte (8 data + 1 start + 1 stop) the byte-time in
    /// sysclks is `sys_hz * 10 / baud`.
    ///
    /// When IBRD=0 the UART is effectively unconfigured; we fall back
    /// to 1 cycle/byte so `tick()` still drains the FIFO (tests that
    /// skip baud programming stay deterministic). In production
    /// firmware IBRD is always programmed before data, so this path
    /// isn't load-bearing.
    fn sysclks_per_byte(&self, clock_tree: &ClockTree) -> u64 {
        let ibrd = self.ibrd & 0xFFFF;
        let fbrd = self.fbrd & 0x3F;
        if ibrd == 0 && fbrd == 0 {
            return 1;
        }
        let peri = clock_tree.peri_hz().max(1);
        let sys = clock_tree.sys_clk_hz.max(1) as u64;
        // PL011 baud = peri / (16 * (IBRD + FBRD/64)).
        // Collapse into integer math by scaling by 64:
        //   div_64 = IBRD*64 + FBRD
        //   baud   = peri * 4 / div_64 (since 16 * 64 / (64/4) = 4).
        let div_64 = (ibrd as u64) * 64 + fbrd as u64;
        if div_64 == 0 {
            return 1;
        }
        let baud = peri.saturating_mul(4) / div_64;
        if baud == 0 {
            return 1;
        }
        // 10 bits/byte (8 data + 1 start + 1 stop).
        // sysclks_per_byte = sys_hz * 10 / baud.
        (sys.saturating_mul(10) / baud).max(1)
    }

    // -------------------------------------------------------------------
    // Register dispatch
    // -------------------------------------------------------------------

    /// Read a UART register by offset.
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            UARTDR => self.read_dr() as u32,
            UARTRSR_ECR => self.rsr_ecr & 0xF,
            UARTFR => self.fr_read(),
            UARTILPR => 0,
            UARTIBRD => self.ibrd,
            UARTFBRD => self.fbrd,
            UARTLCR_H => self.lcr_h,
            UARTCR => self.cr,
            UARTIFLS => self.ifls,
            UARTIMSC => self.imsc,
            UARTRIS => self.ris,
            UARTMIS => self.ris & self.imsc,
            UARTICR => 0, // W1C, reads as 0
            UARTDMACR => self.dmacr,
            UARTPERIPHID0 => PERIPH_ID[0],
            UARTPERIPHID1 => PERIPH_ID[1],
            UARTPERIPHID2 => PERIPH_ID[2],
            UARTPERIPHID3 => PERIPH_ID[3],
            UARTPCELLID0 => PCELL_ID[0],
            UARTPCELLID1 => PCELL_ID[1],
            UARTPCELLID2 => PCELL_ID[2],
            UARTPCELLID3 => PCELL_ID[3],
            _ => 0,
        }
    }

    /// Write a UART register. Alias semantics apply to plain-storage
    /// registers (IBRD / FBRD / LCR_H / CR / IFLS / IMSC / DMACR);
    /// side-effect registers (DR push, ICR clear) handle alias
    /// themselves.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u32) {
        match offset {
            UARTDR => {
                // DR write pushes the low byte into the TX FIFO.
                self.push_tx(value as u8, irqs);
            }
            UARTRSR_ECR => {
                // Any write clears all four error bits.
                self.rsr_ecr = 0;
            }
            UARTFR | UARTMIS | UARTRIS => {} // read-only
            UARTILPR => {} // not modelled
            UARTIBRD => {
                let mut stored = self.ibrd;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.ibrd = stored & 0xFFFF;
            }
            UARTFBRD => {
                let mut stored = self.fbrd;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.fbrd = stored & 0x3F;
            }
            UARTLCR_H => {
                let mut stored = self.lcr_h;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.lcr_h = stored;
                // A write that clears FEN collapses both FIFOs to one
                // holding entry — drop anything that overflowed the
                // new capacity so `UARTFR.TXFF` reflects reality.
                if !self.fifos_enabled() {
                    self.tx_fifo.truncate(1);
                    self.rx_fifo.truncate(1);
                }
            }
            UARTCR => {
                let mut stored = self.cr;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cr = stored;
                // Disabling the UART resets the TX cycle accumulator so
                // a subsequent enable starts a fresh bit-time window.
                if !self.is_enabled() {
                    self.tx_cycle_accum = 0;
                }
            }
            UARTIFLS => {
                let mut stored = self.ifls;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.ifls = stored & 0x3F;
            }
            UARTIMSC => {
                let mut stored = self.imsc;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.imsc = stored & UART_INT_MASK;
                // Mask change may expose a latched RIS bit — re-fire
                // the aggregate IRQ if MIS is now non-zero.
                self.route_irq(irqs);
            }
            UARTICR => {
                // W1C: alias semantics applied first, then every set
                // bit clears the matching RIS bit.
                let mut clr = self.ris;
                super::apply_alias_rmw(&mut clr, value, alias);
                let mask = clr & UART_INT_MASK;
                self.ris &= !mask;
                self.route_irq(irqs);
            }
            UARTDMACR => {
                let mut stored = self.dmacr;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.dmacr = stored & 0x7;
            }
            _ => {}
        }
    }

    /// Byte-accessible read of `UARTDR`. Other offsets fall back to
    /// word reads via the bus.
    pub fn read8(&mut self, offset: u32) -> u8 {
        if offset == UARTDR {
            self.read_dr()
        } else {
            self.read32(offset) as u8
        }
    }

    /// Byte-accessible write of `UARTDR`. Bypasses the word-RMW that
    /// would spuriously pop the RX FIFO or re-emit the DR value into
    /// the TX FIFO on sub-word access.
    pub fn write8(&mut self, offset: u32, value: u8, irqs: &mut u32) {
        if offset == UARTDR {
            self.push_tx(value, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    /// Read UARTDR. If the RX FIFO has data, pop the head byte;
    /// otherwise return 0. Side-effect: clears the RX path's error
    /// flags in the low nibble of RSR (not modelled in Phase 2).
    fn read_dr(&mut self) -> u8 {
        self.rx_fifo.pop_front().unwrap_or(0)
    }

    /// Push a byte into the TX FIFO. If the UART is disabled or
    /// TX-disabled, the write is silently dropped — matches the
    /// PL011 datasheet (UARTEN=0 leaves the FIFO in reset state).
    fn push_tx(&mut self, byte: u8, irqs: &mut u32) {
        if !self.is_tx_enabled() {
            return;
        }
        let cap = self.tx_capacity();
        if self.tx_fifo.len() >= cap {
            // Overflow drops the byte. The PL011 also latches an
            // overrun error — we don't model that in Phase 2.
            return;
        }
        self.tx_fifo.push_back(byte);
        // FIFO rising past the TX trigger level clears the TXIS
        // condition (re-assertion happens as the FIFO drains under
        // the threshold in `refresh_tx_interrupt`). Do not raise TX
        // IRQ on push.
        if self.tx_fifo.len() > self.tx_fill_threshold() {
            self.ris &= !UART_INT_TX;
        }
        self.route_irq(irqs);
    }

    /// Aggregate IRQ routing. PL011 ORs every MIS bit into a single
    /// NVIC line; we raise that line into `irqs` by OR-ing
    /// `1 << nvic_irq` when `ris & imsc` is non-zero.
    fn route_irq(&self, irqs: &mut u32) {
        if (self.ris & self.imsc) != 0 {
            *irqs |= 1u32 << self.nvic_irq;
        }
    }

    /// Advance the UART by `cycles` system-clock cycles. The baud-rate
    /// model pops one byte from the TX FIFO every `sysclks_per_byte`
    /// cycles; level-crossings below `UARTIFLS.TXIFLSEL` latch TXIS.
    pub fn tick(&mut self, cycles: u32, clock_tree: &ClockTree, irqs: &mut u32) {
        if cycles == 0 || !self.is_tx_enabled() || self.tx_fifo.is_empty() {
            return;
        }
        let sysclks_per_byte = self.sysclks_per_byte(clock_tree);
        self.tx_cycle_accum = self.tx_cycle_accum.saturating_add(cycles as u64);
        while self.tx_cycle_accum >= sysclks_per_byte && !self.tx_fifo.is_empty() {
            self.tx_cycle_accum -= sysclks_per_byte;
            self.tx_fifo.pop_front();
        }
        self.refresh_tx_interrupt();
        self.route_irq(irqs);
    }
}

impl Default for UartRegs {
    fn default() -> Self {
        // Defaults to UART0 IRQ; callers that need UART1 override via
        // `new(21)`. This only matters for standalone test fixtures —
        // the bus constructs both instances explicitly.
        Self::new(20)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const UART0_IRQ: u32 = 20;
    const SYS_HZ: u32 = 125_000_000;

    fn tree(peri: u32) -> ClockTree {
        let mut t = ClockTree::default();
        t.sys_clk_hz = SYS_HZ;
        t.peri_clk_hz = peri;
        t.ref_clk_hz = SYS_HZ;
        t
    }

    // --- reset / defaults ---------------------------------------------

    #[test]
    fn reset_defaults_all_zero_except_ifls() {
        let u = UartRegs::new(UART0_IRQ);
        assert_eq!(u.ibrd, 0);
        assert_eq!(u.fbrd, 0);
        assert_eq!(u.lcr_h, 0);
        assert_eq!(u.cr, 0);
        assert_eq!(u.imsc, 0);
        assert_eq!(u.ris, 0);
        // IFLS resets to (1/2, 1/2).
        assert_eq!(u.ifls, (0b010 << 3) | 0b010);
    }

    #[test]
    fn reset_clears_runtime_state() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.cr = UARTCR_UARTEN | UARTCR_TXE;
        u.lcr_h = UARTLCR_H_FEN;
        u.push_tx(0x5A, &mut irqs);
        assert!(!u.tx_fifo.is_empty());
        u.reset();
        assert!(u.tx_fifo.is_empty());
        assert_eq!(u.cr, 0);
    }

    #[test]
    fn fr_reads_txfe_rxfe_at_reset() {
        let mut u = UartRegs::new(UART0_IRQ);
        let fr = u.read32(UARTFR);
        assert!(fr & UARTFR_TXFE != 0, "TX FIFO empty at reset");
        assert!(fr & UARTFR_RXFE != 0, "RX FIFO empty at reset");
        assert!(fr & UARTFR_BUSY == 0, "BUSY clear at reset");
    }

    // --- IBRD / FBRD round-trip ---------------------------------------

    #[test]
    fn ibrd_fbrd_roundtrip() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTIBRD, 67, 0, &mut irqs); // 115200 baud at 125MHz clk_peri
        u.write32(UARTFBRD, 52, 0, &mut irqs);
        assert_eq!(u.read32(UARTIBRD), 67);
        assert_eq!(u.read32(UARTFBRD), 52);
    }

    #[test]
    fn ibrd_truncated_to_16_bits() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTIBRD, 0xDEAD_BEEF, 0, &mut irqs);
        assert_eq!(u.read32(UARTIBRD), 0xBEEF);
    }

    #[test]
    fn fbrd_truncated_to_6_bits() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTFBRD, 0xFF, 0, &mut irqs);
        assert_eq!(u.read32(UARTFBRD), 0x3F);
    }

    // --- TX data path -------------------------------------------------

    #[test]
    fn dr_write_before_enable_is_dropped() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTDR, 0xA5, 0, &mut irqs);
        assert!(u.tx_fifo.is_empty());
    }

    #[test]
    fn dr_write_after_enable_pushes_into_tx_fifo() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        u.write32(UARTDR, 0xA5, 0, &mut irqs);
        assert_eq!(u.tx_fifo.len(), 1);
        assert_eq!(u.tx_fifo.front().copied(), Some(0xA5));
    }

    #[test]
    fn byte_write_to_dr_uses_narrow_path() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        u.write8(UARTDR, 0x5A, &mut irqs);
        assert_eq!(u.tx_fifo.front().copied(), Some(0x5A));
    }

    #[test]
    fn tx_fifo_caps_at_16_when_fen_set() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        for i in 0..20u8 {
            u.write32(UARTDR, i as u32, 0, &mut irqs);
        }
        assert_eq!(u.tx_fifo.len(), 16, "FIFO must cap at 16 with FEN=1");
    }

    #[test]
    fn tx_fifo_caps_at_1_when_fen_clear() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        // FEN cleared.
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        for i in 0..5u8 {
            u.write32(UARTDR, i as u32, 0, &mut irqs);
        }
        assert_eq!(u.tx_fifo.len(), 1, "holding register only with FEN=0");
    }

    // --- Baud-rate cadence --------------------------------------------

    #[test]
    fn tick_drains_fifo_at_derived_cadence() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        // 115200 baud at 125 MHz clk_peri: IBRD=67, FBRD=52 per pico-sdk.
        u.write32(UARTIBRD, 67, 0, &mut irqs);
        u.write32(UARTFBRD, 52, 0, &mut irqs);
        // Push 4 bytes.
        for b in [0x11, 0x22, 0x33, 0x44] {
            u.write32(UARTDR, b, 0, &mut irqs);
        }
        assert_eq!(u.tx_fifo.len(), 4);
        let t = tree(SYS_HZ);
        // Step a generous window — 115200 baud × 10 bits = 86.8 µs/byte.
        // At 125 MHz that's ~10850 cycles/byte; 4 bytes = ~43400 cycles.
        u.tick(50_000, &t, &mut irqs);
        assert!(
            u.tx_fifo.is_empty(),
            "FIFO must drain after 4 × byte-time at configured baud"
        );
    }

    #[test]
    fn tick_ignored_when_uart_disabled() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        // TXE but not UARTEN.
        u.write32(UARTCR, UARTCR_TXE, 0, &mut irqs);
        // Force FIFO contents (bypassing push_tx gate via direct access).
        u.tx_fifo.push_back(0xFF);
        let t = tree(SYS_HZ);
        u.tick(1_000_000, &t, &mut irqs);
        assert_eq!(u.tx_fifo.len(), 1, "disabled UART must not drain");
    }

    // --- IRQ routing --------------------------------------------------

    #[test]
    fn tx_empty_raises_txis_when_imsc_set() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        u.write32(UARTIMSC, UART_INT_TX, 0, &mut irqs);
        u.write32(UARTIBRD, 67, 0, &mut irqs);
        u.write32(UARTFBRD, 52, 0, &mut irqs);
        // Queue a byte.
        u.write32(UARTDR, 0x5A, 0, &mut irqs);
        let t = tree(SYS_HZ);
        u.tick(50_000, &t, &mut irqs);
        // After drain, TXIS latched in RIS.
        assert_eq!(
            u.ris & UART_INT_TX,
            UART_INT_TX,
            "TXIS must latch after FIFO drains under threshold"
        );
        // MIS = RIS & IMSC.
        assert_eq!(u.read32(UARTMIS) & UART_INT_TX, UART_INT_TX);
        // irqs bit for UART0 is set.
        assert!(
            irqs & (1u32 << UART0_IRQ) != 0,
            "irqs must carry the NVIC bit for UART0"
        );
    }

    #[test]
    fn icr_is_write_one_to_clear() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.ris = UART_INT_TX | UART_INT_RX;
        u.write32(UARTICR, UART_INT_TX, 0, &mut irqs);
        assert_eq!(u.ris & UART_INT_TX, 0, "TX bit cleared");
        assert_eq!(u.ris & UART_INT_RX, UART_INT_RX, "RX bit preserved");
    }

    #[test]
    fn ris_and_mis_readonly_writes_are_dropped() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.ris = UART_INT_TX;
        u.write32(UARTRIS, 0xFF, 0, &mut irqs);
        u.write32(UARTMIS, 0xFF, 0, &mut irqs);
        assert_eq!(u.ris, UART_INT_TX, "RIS must be read-only");
    }

    #[test]
    fn mis_is_ris_masked_by_imsc() {
        let mut u = UartRegs::new(UART0_IRQ);
        u.ris = UART_INT_TX | UART_INT_RX;
        u.imsc = UART_INT_RX;
        assert_eq!(u.read32(UARTMIS), UART_INT_RX);
    }

    // --- is_idle ------------------------------------------------------

    #[test]
    fn is_idle_true_at_reset() {
        let u = UartRegs::new(UART0_IRQ);
        assert!(u.is_idle());
    }

    #[test]
    fn is_idle_false_with_pending_tx() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        u.write32(UARTDR, 0xA5, 0, &mut irqs);
        assert!(!u.is_idle(), "pending TX byte breaks idle");
    }

    // --- Fifo disable truncates ---------------------------------------

    #[test]
    fn clearing_fen_truncates_tx_fifo_to_one() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTLCR_H, UARTLCR_H_FEN, 0, &mut irqs);
        u.write32(UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0, &mut irqs);
        for i in 0..5u8 {
            u.write32(UARTDR, i as u32, 0, &mut irqs);
        }
        // Clear FEN — collapse.
        u.write32(UARTLCR_H, 0, 0, &mut irqs);
        assert!(u.tx_fifo.len() <= 1, "FEN=0 collapses FIFO to 1");
    }

    // --- PrimeCell ID -------------------------------------------------

    #[test]
    fn peripheral_and_pcell_id_match_pl011() {
        let mut u = UartRegs::new(UART0_IRQ);
        assert_eq!(u.read32(UARTPERIPHID0), 0x11);
        assert_eq!(u.read32(UARTPERIPHID1), 0x10);
        assert_eq!(u.read32(UARTPERIPHID2), 0x34);
        assert_eq!(u.read32(UARTPERIPHID3), 0x00);
        assert_eq!(u.read32(UARTPCELLID0), 0x0D);
        assert_eq!(u.read32(UARTPCELLID1), 0xF0);
        assert_eq!(u.read32(UARTPCELLID2), 0x05);
        assert_eq!(u.read32(UARTPCELLID3), 0xB1);
    }

    // --- alias semantics ----------------------------------------------

    #[test]
    fn imsc_bitset_alias_works() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.write32(UARTIMSC, UART_INT_TX, 2, &mut irqs); // BITSET
        u.write32(UARTIMSC, UART_INT_RX, 2, &mut irqs);
        assert_eq!(u.imsc, UART_INT_TX | UART_INT_RX);
    }

    #[test]
    fn imsc_bitclr_alias_works() {
        let mut u = UartRegs::new(UART0_IRQ);
        let mut irqs = 0;
        u.imsc = UART_INT_MASK;
        u.write32(UARTIMSC, UART_INT_TX, 3, &mut irqs); // BITCLR
        assert_eq!(u.imsc & UART_INT_TX, 0);
    }
}
