//! RP2040 clock infrastructure — CLOCKS / XOSC / ROSC / PLL_SYS / PLL_USB
//! register storage plus the derived [`ClockTree`] cache.
//!
//! Layout differs from RP2350:
//! * CLOCKS base is `0x4000_8000` (not `0x4001_0000`).
//! * CLK_REF_CTRL = 0x30, CLK_REF_DIV = 0x34, CLK_REF_SELECTED = 0x38.
//! * CLK_SYS_CTRL = 0x3C, CLK_SYS_DIV = 0x40, CLK_SYS_SELECTED = 0x44.
//! * PLL_SYS at 0x4002_8000, PLL_USB at 0x4002_C000.
//! * XOSC at 0x4002_4000, ROSC at 0x4006_0000.
//!
//! The pure PLL math (`pll_output_hz`) and the [`ClockTree`] struct live
//! in [`mdpicoem_common::clocks`]; this module owns the RP2040 register
//! *storage* and the `recompute` step that turns register writes into
//! frequency updates.

pub use mdpicoem_common::clocks::{ClockTree, ROSC_FREQ_HZ, XOSC_FREQ_HZ, pll_output_hz};

// --- CLOCKS offsets (RP2040 datasheet §2.15.7) -----------------------------
//
// All 10 clocks follow a CTRL / DIV / SELECTED triple, except that clk_peri
// has no divider hardware — offset 0x4C reads zero. Offsets below match the
// datasheet.
pub(crate) const CLK_GPOUT0_CTRL: u32 = 0x00;
pub(crate) const CLK_GPOUT0_DIV: u32 = 0x04;
pub(crate) const CLK_GPOUT0_SELECTED: u32 = 0x08;
pub(crate) const CLK_GPOUT1_CTRL: u32 = 0x0C;
pub(crate) const CLK_GPOUT1_DIV: u32 = 0x10;
pub(crate) const CLK_GPOUT1_SELECTED: u32 = 0x14;
pub(crate) const CLK_GPOUT2_CTRL: u32 = 0x18;
pub(crate) const CLK_GPOUT2_DIV: u32 = 0x1C;
pub(crate) const CLK_GPOUT2_SELECTED: u32 = 0x20;
pub(crate) const CLK_GPOUT3_CTRL: u32 = 0x24;
pub(crate) const CLK_GPOUT3_DIV: u32 = 0x28;
pub(crate) const CLK_GPOUT3_SELECTED: u32 = 0x2C;
pub(crate) const CLK_REF_CTRL: u32 = 0x30;
pub(crate) const CLK_REF_DIV: u32 = 0x34;
pub(crate) const CLK_REF_SELECTED: u32 = 0x38;
pub(crate) const CLK_SYS_CTRL: u32 = 0x3C;
pub(crate) const CLK_SYS_DIV: u32 = 0x40;
pub(crate) const CLK_SYS_SELECTED: u32 = 0x44;
pub(crate) const CLK_PERI_CTRL: u32 = 0x48;
// 0x4C: CLK_PERI_DIV — no divider hardware, reads-as-zero
pub(crate) const CLK_PERI_SELECTED: u32 = 0x50;
pub(crate) const CLK_USB_CTRL: u32 = 0x54;
pub(crate) const CLK_USB_DIV: u32 = 0x58;
pub(crate) const CLK_USB_SELECTED: u32 = 0x5C;
pub(crate) const CLK_ADC_CTRL: u32 = 0x60;
pub(crate) const CLK_ADC_DIV: u32 = 0x64;
pub(crate) const CLK_ADC_SELECTED: u32 = 0x68;
pub(crate) const CLK_RTC_CTRL: u32 = 0x6C;
pub(crate) const CLK_RTC_DIV: u32 = 0x70;
pub(crate) const CLK_RTC_SELECTED: u32 = 0x74;

/// RP2040 CLOCKS register storage.
///
/// Only the fields firmware actually pokes at are backed by real storage;
/// the rest read-as-zero. All `CLK_*_SELECTED` registers are synthesised on
/// read — pico-sdk's `clock_configure` busy-waits on this handshake:
///
/// * Glitchless muxes (`clk_ref`, `clk_sys`): `_SELECTED` is `1 << SRC`,
///   mirroring `CTRL[SRC]` immediately.
/// * Non-glitchless clocks (`clk_gpout{0..3}`, `clk_peri`, `clk_usb`,
///   `clk_adc`, `clk_rtc`): `_SELECTED` reads as `1` unconditionally. This
///   matches silicon — the mux is a simple AUXSRC demux, always "selected"
///   from firmware's perspective — and satisfies pico-sdk's
///   `while (!(selected & (1u << 0)))` after each CTRL write.
pub struct ClocksRegs {
    pub clk_gpout0_ctrl: u32,
    pub clk_gpout0_div: u32,
    pub clk_gpout1_ctrl: u32,
    pub clk_gpout1_div: u32,
    pub clk_gpout2_ctrl: u32,
    pub clk_gpout2_div: u32,
    pub clk_gpout3_ctrl: u32,
    pub clk_gpout3_div: u32,
    pub clk_ref_ctrl: u32,
    pub clk_ref_div: u32,
    pub clk_sys_ctrl: u32,
    pub clk_sys_div: u32,
    pub clk_peri_ctrl: u32,
    pub clk_usb_ctrl: u32,
    pub clk_usb_div: u32,
    pub clk_adc_ctrl: u32,
    pub clk_adc_div: u32,
    pub clk_rtc_ctrl: u32,
    pub clk_rtc_div: u32,
}

impl ClocksRegs {
    pub fn new() -> Self {
        Self {
            clk_gpout0_ctrl: 0,
            clk_gpout0_div: 0x0000_0100,
            clk_gpout1_ctrl: 0,
            clk_gpout1_div: 0x0000_0100,
            clk_gpout2_ctrl: 0,
            clk_gpout2_div: 0x0000_0100,
            clk_gpout3_ctrl: 0,
            clk_gpout3_div: 0x0000_0100,
            clk_ref_ctrl: 0,
            clk_ref_div: 0x0000_0100, // default int div = 1 (bits [11:8])
            clk_sys_ctrl: 0,
            clk_sys_div: 0x0001_0000, // default int div = 1 (bits [31:16])
            clk_peri_ctrl: 0,
            clk_usb_ctrl: 0,
            clk_usb_div: 0x0000_0100,
            clk_adc_ctrl: 0,
            clk_adc_div: 0x0000_0100,
            clk_rtc_ctrl: 0,
            clk_rtc_div: 0x0000_0100,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Read a CLOCKS register by byte offset.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            // clk_gpout0..3 — no SRC mux, `_SELECTED` reads 1.
            CLK_GPOUT0_CTRL => self.clk_gpout0_ctrl,
            CLK_GPOUT0_DIV => self.clk_gpout0_div,
            CLK_GPOUT0_SELECTED => 1,
            CLK_GPOUT1_CTRL => self.clk_gpout1_ctrl,
            CLK_GPOUT1_DIV => self.clk_gpout1_div,
            CLK_GPOUT1_SELECTED => 1,
            CLK_GPOUT2_CTRL => self.clk_gpout2_ctrl,
            CLK_GPOUT2_DIV => self.clk_gpout2_div,
            CLK_GPOUT2_SELECTED => 1,
            CLK_GPOUT3_CTRL => self.clk_gpout3_ctrl,
            CLK_GPOUT3_DIV => self.clk_gpout3_div,
            CLK_GPOUT3_SELECTED => 1,
            // clk_ref — glitchless, 2-bit SRC field in [1:0].
            CLK_REF_CTRL => self.clk_ref_ctrl,
            CLK_REF_DIV => self.clk_ref_div,
            CLK_REF_SELECTED => 1 << (self.clk_ref_ctrl & 0x3),
            // clk_sys — glitchless, 1-bit SRC field in [0].
            CLK_SYS_CTRL => self.clk_sys_ctrl,
            CLK_SYS_DIV => self.clk_sys_div,
            CLK_SYS_SELECTED => 1 << (self.clk_sys_ctrl & 0x1),
            // clk_peri — no DIV field on RP2040; `_SELECTED` reads 1.
            CLK_PERI_CTRL => self.clk_peri_ctrl,
            CLK_PERI_SELECTED => 1,
            CLK_USB_CTRL => self.clk_usb_ctrl,
            CLK_USB_DIV => self.clk_usb_div,
            CLK_USB_SELECTED => 1,
            CLK_ADC_CTRL => self.clk_adc_ctrl,
            CLK_ADC_DIV => self.clk_adc_div,
            CLK_ADC_SELECTED => 1,
            CLK_RTC_CTRL => self.clk_rtc_ctrl,
            CLK_RTC_DIV => self.clk_rtc_div,
            CLK_RTC_SELECTED => 1,
            _ => 0,
        }
    }

    /// Write a CLOCKS register with an alias-aware update.
    /// Returns `true` if the write affected a field that feeds the
    /// derived [`ClockTree`] (only `clk_ref` / `clk_sys` CTRL/DIV in V1).
    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) -> bool {
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        match offset {
            CLK_GPOUT0_CTRL => {
                self.clk_gpout0_ctrl = apply(self.clk_gpout0_ctrl, val);
                false
            }
            CLK_GPOUT0_DIV => {
                self.clk_gpout0_div = apply(self.clk_gpout0_div, val);
                false
            }
            CLK_GPOUT1_CTRL => {
                self.clk_gpout1_ctrl = apply(self.clk_gpout1_ctrl, val);
                false
            }
            CLK_GPOUT1_DIV => {
                self.clk_gpout1_div = apply(self.clk_gpout1_div, val);
                false
            }
            CLK_GPOUT2_CTRL => {
                self.clk_gpout2_ctrl = apply(self.clk_gpout2_ctrl, val);
                false
            }
            CLK_GPOUT2_DIV => {
                self.clk_gpout2_div = apply(self.clk_gpout2_div, val);
                false
            }
            CLK_GPOUT3_CTRL => {
                self.clk_gpout3_ctrl = apply(self.clk_gpout3_ctrl, val);
                false
            }
            CLK_GPOUT3_DIV => {
                self.clk_gpout3_div = apply(self.clk_gpout3_div, val);
                false
            }
            CLK_REF_CTRL => {
                self.clk_ref_ctrl = apply(self.clk_ref_ctrl, val);
                true
            }
            CLK_REF_DIV => {
                self.clk_ref_div = apply(self.clk_ref_div, val);
                true
            }
            CLK_SYS_CTRL => {
                self.clk_sys_ctrl = apply(self.clk_sys_ctrl, val);
                true
            }
            CLK_SYS_DIV => {
                self.clk_sys_div = apply(self.clk_sys_div, val);
                true
            }
            CLK_PERI_CTRL => {
                self.clk_peri_ctrl = apply(self.clk_peri_ctrl, val);
                // Peripheral clock derivation follows AUXSRC + ENABLE,
                // so the clock tree must be recomputed whenever CLK_PERI
                // changes — UART/SPI/I2C baud-rate models read
                // `ClockTree::peri_hz()` on every cadence decision.
                true
            }
            CLK_USB_CTRL => {
                self.clk_usb_ctrl = apply(self.clk_usb_ctrl, val);
                false
            }
            CLK_USB_DIV => {
                self.clk_usb_div = apply(self.clk_usb_div, val);
                false
            }
            CLK_ADC_CTRL => {
                self.clk_adc_ctrl = apply(self.clk_adc_ctrl, val);
                false
            }
            CLK_ADC_DIV => {
                self.clk_adc_div = apply(self.clk_adc_div, val);
                false
            }
            CLK_RTC_CTRL => {
                self.clk_rtc_ctrl = apply(self.clk_rtc_ctrl, val);
                false
            }
            CLK_RTC_DIV => {
                self.clk_rtc_div = apply(self.clk_rtc_div, val);
                false
            }
            _ => false,
        }
    }
}

impl Default for ClocksRegs {
    fn default() -> Self {
        Self::new()
    }
}

// --- XOSC register storage (base 0x4002_4000) ------------------------------
//
// Offsets (RP2040 datasheet §2.16):
//   0x00 CTRL, 0x04 STATUS, 0x08 DORMANT, 0x0C STARTUP, 0x1C COUNT.
//
// STATUS reports STABLE | ENABLED unconditionally so firmware's wait-for-
// stable loops fall through on the first read.

pub struct XoscRegs {
    pub ctrl: u32,
    pub dormant: u32,
    pub startup: u32,
}

impl XoscRegs {
    pub fn new() -> Self {
        Self { ctrl: 0, dormant: 0, startup: 0 }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.ctrl,
            0x04 => (1 << 31) | (1 << 12), // STABLE | ENABLED
            0x08 => self.dormant,
            0x0C => self.startup,
            0x1C => 0, // COUNT — stub
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        match offset {
            0x00 => self.ctrl = apply(self.ctrl, val),
            0x08 => self.dormant = apply(self.dormant, val),
            0x0C => self.startup = apply(self.startup, val),
            _ => {} // STATUS / COUNT are read-only
        }
    }
}

impl Default for XoscRegs {
    fn default() -> Self {
        Self::new()
    }
}

// --- ROSC register storage (base 0x4006_0000) ------------------------------
//
// Offsets (RP2040 datasheet §2.17):
//   0x00 CTRL, 0x04 FREQA, 0x08 FREQB, 0x0C DORMANT, 0x10 DIV, 0x14 PHASE,
//   0x18 STATUS, 0x1C RANDOMBIT, 0x20 COUNT.
//
// STATUS reports STABLE | ENABLED unconditionally. RANDOMBIT reads as 0
// (no PRNG modelled); COUNT reads as 0. All storage-only — writes do not
// change the fixed 6.5 MHz ROSC output.

pub struct RoscRegs {
    pub ctrl: u32,
    pub freqa: u32,
    pub freqb: u32,
    pub dormant: u32,
    pub div: u32,
    pub phase: u32,
}

impl RoscRegs {
    pub fn new() -> Self {
        Self { ctrl: 0, freqa: 0, freqb: 0, dormant: 0, div: 0, phase: 0 }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.ctrl,
            0x04 => self.freqa,
            0x08 => self.freqb,
            0x0C => self.dormant,
            0x10 => self.div,
            0x14 => self.phase,
            0x18 => (1 << 31) | (1 << 12), // STATUS: STABLE | ENABLED
            0x1C => 0, // RANDOMBIT
            0x20 => 0, // COUNT
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        match offset {
            0x00 => self.ctrl = apply(self.ctrl, val),
            0x04 => self.freqa = apply(self.freqa, val),
            0x08 => self.freqb = apply(self.freqb, val),
            0x0C => self.dormant = apply(self.dormant, val),
            0x10 => self.div = apply(self.div, val),
            0x14 => self.phase = apply(self.phase, val),
            _ => {} // STATUS / RANDOMBIT / COUNT are read-only
        }
    }
}

impl Default for RoscRegs {
    fn default() -> Self {
        Self::new()
    }
}

// --- PLL register storage (PLL_SYS 0x4002_8000 / PLL_USB 0x4002_C000) ------
//
// Both PLLs share the same layout:
//   0x00 CS (REFDIV in [5:0], LOCK in [31]),
//   0x04 PWR (power-gating bits, treated as storage only),
//   0x08 FBDIV_INT ([11:0]),
//   0x0C PRIM (POSTDIV1 in [18:16], POSTDIV2 in [14:12]).
//
// Reset values per RP2040 SVD:
//   CS = 0x0000_0001 (REFDIV = 1, LOCK = 0).
//   PWR = 0x0000_002D (powered down).
//   FBDIV_INT = 0.
//   PRIM = 0x0007_7000 (POSTDIV1 = 7, POSTDIV2 = 7).
//
// CS[31] (LOCK) is now derived from register image + per-PLL arm state +
// master cycle count via `mdpicoem_common::clocks::pll_cs_read_with_lock`
// at the Bus dispatch layer; `pll_read` below returns the raw stored CS
// value (bit 31 reflects whatever was written, i.e. 0 at reset). See
// `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md`.

/// PLL register image stored as `[CS, PWR, FBDIV_INT, PRIM]`.
pub type PllRegs = [u32; 4];

/// Power-on default for a PLL register image (matches the RP2040 SVD).
pub const PLL_RESET: PllRegs = [0x0000_0001, 0x0000_002D, 0, 0x0007_7000];

fn pll_reg_index(offset: u32) -> Option<usize> {
    match offset {
        0x00 => Some(0),
        0x04 => Some(1),
        0x08 => Some(2),
        0x0C => Some(3),
        _ => None,
    }
}

/// Read a raw PLL register word. CS[31] (LOCK) is returned exactly as
/// stored — Bus-level dispatch overlays the modelled lock status via
/// `mdpicoem_common::clocks::pll_cs_read_with_lock`.
pub fn pll_read(regs: &PllRegs, offset: u32) -> u32 {
    match pll_reg_index(offset) {
        Some(i) => regs[i],
        None => 0,
    }
}

/// Write a PLL register with an alias-aware update. Returns `true` if
/// the offset was recognised (and a recompute may be needed).
pub fn pll_write(regs: &mut PllRegs, offset: u32, val: u32, alias: u32) -> bool {
    if let Some(i) = pll_reg_index(offset) {
        regs[i] = match alias {
            0 => val,
            1 => regs[i] ^ val,
            2 => regs[i] | val,
            3 => regs[i] & !val,
            _ => val,
        };
        true
    } else {
        false
    }
}

/// Recompute the derived system / reference clock frequencies from the
/// current RP2040 register state.
///
/// RP2040 CLK_REF mux (CLK_REF_CTRL[1:0]):
///   0 = ROSC, 1 = AUX (PLL_USB via SRC bits [5]), 2 = XOSC.
/// RP2040 CLK_SYS mux (CLK_SYS_CTRL[0]):
///   0 = clk_ref, 1 = AUX (SRC in bits [7:5]): 0 = PLL_SYS, 1 = PLL_USB,
///   2 = ROSC, 3 = XOSC.
///
/// Dividers:
/// * CLK_REF_DIV[11:8] — integer divider (0 → treat as 1).
/// * CLK_SYS_DIV[31:16] — integer divider (0 → treat as 1). Fractional
///   bits [15:0] are ignored.
pub fn recompute(
    clocks: &ClocksRegs,
    pll_sys: &PllRegs,
    pll_usb: &PllRegs,
    tree: &mut ClockTree,
) {
    let ref_src_hz = match clocks.clk_ref_ctrl & 0x3 {
        0 => ROSC_FREQ_HZ,
        1 => {
            // AUX mux: only PLL_USB is meaningful on RP2040; clksrc_gpin0/1
            // are unmodeled.
            match (clocks.clk_ref_ctrl >> 5) & 0x3 {
                0 => pll_output_hz(pll_usb),
                _ => 0,
            }
        }
        2 => XOSC_FREQ_HZ,
        _ => ROSC_FREQ_HZ,
    };
    let ref_div = ((clocks.clk_ref_div >> 8) & 0x3).max(1);
    let ref_hz = ref_src_hz / ref_div;

    let sys_src_hz = match clocks.clk_sys_ctrl & 0x1 {
        0 => ref_hz,
        _ => match (clocks.clk_sys_ctrl >> 5) & 0x7 {
            0 => pll_output_hz(pll_sys),
            1 => pll_output_hz(pll_usb),
            2 => ROSC_FREQ_HZ,
            3 => XOSC_FREQ_HZ,
            _ => 0,
        },
    };

    let int_div = ((clocks.clk_sys_div >> 16) & 0xFFFF).max(1);
    let sys_hz = sys_src_hz / int_div;

    // RP2040 CLK_PERI mux (CLK_PERI_CTRL[7:5] = AUXSRC):
    //   0 = clk_sys, 1 = PLL_SYS, 2 = PLL_USB, 3 = ROSC, 4 = XOSC,
    //   5 = clksrc_gpin0, 6 = clksrc_gpin1, 7 = reserved.
    // pico-sdk default is AUXSRC=0 (clk_sys); firmware often overrides
    // to AUXSRC=1 (PLL_SYS direct) to detach clk_peri from clk_sys DIV.
    let peri_enable = (clocks.clk_peri_ctrl & (1 << 11)) != 0;
    let peri_src_hz = match (clocks.clk_peri_ctrl >> 5) & 0x7 {
        0 => sys_hz,
        1 => pll_output_hz(pll_sys),
        2 => pll_output_hz(pll_usb),
        3 => ROSC_FREQ_HZ,
        4 => XOSC_FREQ_HZ,
        _ => 0, // gpin0 / gpin1 / reserved — unmodelled
    };
    // CLK_PERI has no DIV on RP2040 (offset 0x4C reads zero). Treat the
    // ENABLE bit as a gate: when clear, the peripheral clock defaults to
    // clk_sys so a firmware that pokes at UART/SPI/I2C without
    // programming CLK_PERI_CTRL still gets cadence. pico-sdk's
    // `clock_configure` writes ENABLE along with AUXSRC, so the gate is
    // benign in practice.
    let peri_hz = if peri_enable {
        peri_src_hz.max(1)
    } else {
        sys_hz.max(1)
    };

    tree.ref_clk_hz = ref_hz;
    tree.sys_clk_hz = sys_hz;
    tree.peri_clk_hz = peri_hz;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clocks_default_is_rosc() {
        let c = ClocksRegs::new();
        let ps = PLL_RESET;
        let pu = PLL_RESET;
        let mut tree = ClockTree::default();
        recompute(&c, &ps, &pu, &mut tree);
        assert_eq!(tree.sys_clk_hz, ROSC_FREQ_HZ);
        assert_eq!(tree.ref_clk_hz, ROSC_FREQ_HZ);
    }

    #[test]
    fn clocks_switch_to_xosc() {
        let mut c = ClocksRegs::new();
        let ps = PLL_RESET;
        let pu = PLL_RESET;
        // Select XOSC as REF source.
        c.clk_ref_ctrl = 2;
        let mut tree = ClockTree::default();
        recompute(&c, &ps, &pu, &mut tree);
        assert_eq!(tree.ref_clk_hz, XOSC_FREQ_HZ);
        assert_eq!(tree.sys_clk_hz, XOSC_FREQ_HZ);
    }

    #[test]
    fn clocks_sys_via_pll_sys() {
        let mut c = ClocksRegs::new();
        let mut ps = PLL_RESET;
        let pu = PLL_RESET;
        // Typical Pico SDK config: REFDIV=1, FBDIV=125, POSTDIV1=6, POSTDIV2=2
        // → 12 MHz * 125 / 1 / 6 / 2 = 125 MHz.
        ps[0] = 1;
        ps[2] = 125;
        ps[3] = (6 << 16) | (2 << 12);
        // Switch CLK_SYS to AUX/PLL_SYS.
        c.clk_sys_ctrl = 1;
        let mut tree = ClockTree::default();
        recompute(&c, &ps, &pu, &mut tree);
        assert_eq!(tree.sys_clk_hz, 125_000_000);
    }

    #[test]
    fn pll_forces_lock_bit_on_read() {
        // Post-`2026.04.15 HLD - PLL LOCK Modelling` fix: `pll_read`
        // now returns the raw stored CS value. CS[31] is only set when
        // the Bus dispatch layer composes it from (regs, lock_at, now)
        // via `pll_cs_read_with_lock`. See the clocks.rs top-of-file
        // comment for the architectural note.
        let regs: PllRegs = [0; 4];
        let cs = pll_read(&regs, 0x00);
        assert_eq!(cs & (1 << 31), 0, "CS[31] must not be forced — raw read");
    }

    #[test]
    fn pll_write_alias_clr() {
        let mut regs: PllRegs = [0xF0F0_F0F0; 4];
        assert!(pll_write(&mut regs, 0x04, 0x0F0F_0F0F, 3));
        assert_eq!(regs[1], 0xF0F0_F0F0);
        assert!(pll_write(&mut regs, 0x04, 0x0F0F_0F00, 3));
        assert_eq!(regs[1], 0xF0F0_F0F0);
    }

    #[test]
    fn xosc_stable_by_default() {
        let x = XoscRegs::new();
        assert_ne!(x.read32(0x04) & (1 << 31), 0);
    }

    #[test]
    fn rosc_div_write_stores() {
        let mut r = RoscRegs::new();
        r.write32(0x10, 0xAA02, 0);
        assert_eq!(r.read32(0x10), 0xAA02);
    }

    /// CLK_SYS_DIV lives at offset 0x40 (RP2040 datasheet §2.15.7). A
    /// write to 0x40 must route to the integer divider field [31:16] and
    /// feed through `recompute()` into `tree.sys_clk_hz`. CLK_SYS_SELECTED
    /// is at 0x44 and read-only.
    #[test]
    fn clk_sys_div_offset_0x40_feeds_recompute() {
        let mut c = ClocksRegs::new();
        let ps = PLL_RESET;
        let pu = PLL_RESET;
        // SRC=ROSC (default) → int divider applies directly.
        // Write DIV with integer part = 2 (bits [31:16]).
        c.write32(CLK_SYS_DIV, 2 << 16, 0);
        assert_eq!(CLK_SYS_DIV, 0x40);
        assert_eq!(c.clk_sys_div, 2 << 16);
        let mut tree = ClockTree::default();
        recompute(&c, &ps, &pu, &mut tree);
        assert_eq!(tree.sys_clk_hz, ROSC_FREQ_HZ / 2);
        // CLK_SYS_SELECTED is a read-only mux indicator at 0x44.
        assert_eq!(CLK_SYS_SELECTED, 0x44);
    }

    // --- CLOCKS `_SELECTED` handshake (HLD V7 §4.4 point 5 / §5.3) ---------
    //
    // pico-sdk's `clock_configure` busy-waits on `_SELECTED` reflecting the
    // new source after each `_CTRL` write. The handshake is single-cycle
    // (no state machine).

    #[test]
    fn clk_sys_selected_mirrors_src_bit_zero() {
        // Datasheet-specified semantic: `CLK_SYS_SELECTED = 1 << SRC`.
        // Default SRC=0 (clk_ref) → `_SELECTED = 1`.
        let mut c = ClocksRegs::new();
        assert_eq!(c.read32(CLK_SYS_SELECTED), 1);
        // Writing SRC=1 (AUX) → `_SELECTED = 2`.
        c.write32(CLK_SYS_CTRL, 1, 0);
        assert_eq!(c.read32(CLK_SYS_SELECTED), 2);
    }

    #[test]
    fn clk_ref_selected_mirrors_src_two_bits() {
        let mut c = ClocksRegs::new();
        // SRC=2 (XOSC) → `_SELECTED = 1 << 2 = 4`.
        c.write32(CLK_REF_CTRL, 2, 0);
        assert_eq!(c.read32(CLK_REF_SELECTED), 4);
        // SRC=1 (AUX) → `_SELECTED = 2`.
        c.write32(CLK_REF_CTRL, 1, 0);
        assert_eq!(c.read32(CLK_REF_SELECTED), 2);
    }

    #[test]
    fn clk_peri_selected_after_ctrl_write_is_one() {
        let mut c = ClocksRegs::new();
        // pico-sdk passes CTRL = AUXSRC_BITS | ENABLE_BIT (0x0800_0000). The
        // exact value is irrelevant to the non-glitchless mux — any write
        // results in `_SELECTED = 1`.
        c.write32(CLK_PERI_CTRL, 0x0800_0000, 0);
        assert_eq!(c.read32(CLK_PERI_SELECTED), 1);
    }

    #[test]
    fn all_ten_clocks_handshake_after_ctrl_write() {
        // Each clock's `_CTRL` write must leave the matching `_SELECTED`
        // register satisfying pico-sdk's `selected & (1u << div_input)` test
        // on the very next read — no intermediate cycle required.
        //
        // Glitchless clocks use `div_input = CTRL.SRC`; non-glitchless clocks
        // use `div_input = 0`, so `_SELECTED = 1` is always accepted.
        let cases: &[(u32, u32, u32, u32)] = &[
            // (CTRL offset, SELECTED offset, CTRL value, expected SELECTED)
            (CLK_GPOUT0_CTRL, CLK_GPOUT0_SELECTED, 0x0000_0820, 1),
            (CLK_GPOUT1_CTRL, CLK_GPOUT1_SELECTED, 0x0000_0830, 1),
            (CLK_GPOUT2_CTRL, CLK_GPOUT2_SELECTED, 0x0000_0840, 1),
            (CLK_GPOUT3_CTRL, CLK_GPOUT3_SELECTED, 0x0000_0850, 1),
            (CLK_REF_CTRL, CLK_REF_SELECTED, 0x0000_0002, 1 << 2), // SRC=2 (XOSC)
            (CLK_SYS_CTRL, CLK_SYS_SELECTED, 0x0000_0001, 1 << 1), // SRC=1 (AUX)
            (CLK_PERI_CTRL, CLK_PERI_SELECTED, 0x0800_0000, 1),
            (CLK_USB_CTRL, CLK_USB_SELECTED, 0x0800_0000, 1),
            (CLK_ADC_CTRL, CLK_ADC_SELECTED, 0x0800_0000, 1),
            (CLK_RTC_CTRL, CLK_RTC_SELECTED, 0x0800_0000, 1),
        ];
        for &(ctrl_off, sel_off, ctrl_val, expected) in cases {
            let mut c = ClocksRegs::new();
            c.write32(ctrl_off, ctrl_val, 0);
            assert_eq!(
                c.read32(sel_off),
                expected,
                "CTRL=0x{:02x} SELECTED=0x{:02x} after write 0x{:08x}",
                ctrl_off,
                sel_off,
                ctrl_val
            );
        }
    }

    #[test]
    fn clk_sys_selected_idempotent_on_same_src() {
        // Writing the same SRC twice must not perturb `_SELECTED` — the
        // handshake is stateless.
        let mut c = ClocksRegs::new();
        c.write32(CLK_SYS_CTRL, 1, 0);
        let first = c.read32(CLK_SYS_SELECTED);
        c.write32(CLK_SYS_CTRL, 1, 0);
        let second = c.read32(CLK_SYS_SELECTED);
        assert_eq!(first, second);
        assert_eq!(first, 2);
    }

    #[test]
    fn clk_peri_selected_idempotent_on_repeated_ctrl_writes() {
        let mut c = ClocksRegs::new();
        c.write32(CLK_PERI_CTRL, 0x0800_0000, 0);
        let first = c.read32(CLK_PERI_SELECTED);
        c.write32(CLK_PERI_CTRL, 0x0800_0000, 0);
        let second = c.read32(CLK_PERI_SELECTED);
        assert_eq!(first, 1);
        assert_eq!(second, 1);
    }

    #[test]
    fn non_glitchless_selected_unaffected_by_ctrl_value() {
        // Regardless of what bits firmware writes into CLK_*_CTRL (AUXSRC,
        // ENABLE, phase, nudge), the `_SELECTED` read for a non-glitchless
        // clock must be 1 — there is no SRC to mirror.
        let mut c = ClocksRegs::new();
        for ctrl_val in [0u32, 0xFFFF_FFFF, 0x0800_0000, 0x0000_0AA0] {
            c.write32(CLK_USB_CTRL, ctrl_val, 0);
            assert_eq!(c.read32(CLK_USB_SELECTED), 1);
        }
    }

    #[test]
    fn non_sys_peri_ctrl_does_not_trigger_recompute() {
        // `write32` returning `false` for non-tree-relevant clocks keeps
        // the Bus from recomputing the ClockTree on every unrelated CTRL
        // poke. Phase 2: CLK_PERI_CTRL DOES trigger recompute (unlike
        // other non-glitchless clocks) because UART/SPI/I2C peripherals
        // depend on `ClockTree::peri_clk_hz` for their baud-rate /
        // bit-rate models.
        let mut c = ClocksRegs::new();
        assert!(!c.write32(CLK_GPOUT0_CTRL, 0x0800_0000, 0));
        assert!(
            c.write32(CLK_PERI_CTRL, 0x0800_0000, 0),
            "CLK_PERI_CTRL must trigger recompute (Phase 2 UART/SPI/I2C cadence)"
        );
        assert!(!c.write32(CLK_USB_CTRL, 0x0800_0000, 0));
        assert!(!c.write32(CLK_ADC_CTRL, 0x0800_0000, 0));
        assert!(!c.write32(CLK_RTC_CTRL, 0x0800_0000, 0));
        // Glitchless clocks still affect the tree.
        assert!(c.write32(CLK_REF_CTRL, 2, 0));
        assert!(c.write32(CLK_SYS_CTRL, 1, 0));
    }

    #[test]
    fn alias_rmw_applies_to_new_clock_fields() {
        // pico-sdk reaches the CLOCKS block via alias offsets as well as the
        // plain base: `*(CLK_PERI_CTRL_BITSET) = 0x800` is an hw_set_bits
        // macro expansion. The CTRL/DIV backing storage for the seven clocks
        // added in Wave 1 (gpout0..3, peri, usb, adc, rtc) must honour the
        // same alias semantics as clk_ref / clk_sys: alias=0 plain store,
        // alias=1 XOR, alias=2 BITSET, alias=3 BITCLR.
        let mut c = ClocksRegs::new();

        // CTRL-field path: CLK_PERI_CTRL.
        c.write32(CLK_PERI_CTRL, 0x0800_0000, 0);
        assert_eq!(c.read32(CLK_PERI_CTRL), 0x0800_0000);
        c.write32(CLK_PERI_CTRL, 0x0000_00FF, 2); // BITSET
        assert_eq!(c.read32(CLK_PERI_CTRL), 0x0800_00FF);
        c.write32(CLK_PERI_CTRL, 0x0000_000F, 3); // BITCLR
        assert_eq!(c.read32(CLK_PERI_CTRL), 0x0800_00F0);

        // DIV-field path: CLK_ADC_DIV. Reset value is 0x0000_0100.
        assert_eq!(c.read32(CLK_ADC_DIV), 0x0000_0100);
        c.write32(CLK_ADC_DIV, 0x0000_00FF, 2); // BITSET
        assert_eq!(c.read32(CLK_ADC_DIV), 0x0000_01FF);
        c.write32(CLK_ADC_DIV, 0x0000_000F, 3); // BITCLR
        assert_eq!(c.read32(CLK_ADC_DIV), 0x0000_01F0);
    }
}
