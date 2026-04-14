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
pub(crate) const CLK_REF_CTRL: u32 = 0x30;
pub(crate) const CLK_REF_DIV: u32 = 0x34;
pub(crate) const CLK_REF_SELECTED: u32 = 0x38;
pub(crate) const CLK_SYS_CTRL: u32 = 0x3C;
pub(crate) const CLK_SYS_DIV: u32 = 0x40;
pub(crate) const CLK_SYS_SELECTED: u32 = 0x44;

/// RP2040 CLOCKS register storage.
///
/// Only the fields firmware actually pokes at are backed by real storage;
/// the rest read-as-zero. `CLK_REF_SELECTED` / `CLK_SYS_SELECTED` are
/// synthesised from their respective CTRL SRC fields on read (one-hot mux).
pub struct ClocksRegs {
    pub clk_ref_ctrl: u32,
    pub clk_ref_div: u32,
    pub clk_sys_ctrl: u32,
    pub clk_sys_div: u32,
}

impl ClocksRegs {
    pub fn new() -> Self {
        Self {
            clk_ref_ctrl: 0,
            clk_ref_div: 0x0000_0100, // default int div = 1 (bits [11:8])
            clk_sys_ctrl: 0,
            clk_sys_div: 0x0001_0000, // default int div = 1 (bits [31:16])
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Read a CLOCKS register by byte offset.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            CLK_REF_CTRL => self.clk_ref_ctrl,
            CLK_REF_DIV => self.clk_ref_div,
            CLK_REF_SELECTED => 1 << (self.clk_ref_ctrl & 0x3),
            CLK_SYS_CTRL => self.clk_sys_ctrl,
            CLK_SYS_DIV => self.clk_sys_div,
            CLK_SYS_SELECTED => 1 << (self.clk_sys_ctrl & 0x1),
            _ => 0,
        }
    }

    /// Write a CLOCKS register with an alias-aware update.
    /// Returns `true` if the write affected a field that feeds the
    /// derived [`ClockTree`].
    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) -> bool {
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        match offset {
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
//   CS = 0x0000_0001 (REFDIV = 1, LOCK = 0 — forced to 1 on read for
//   firmware convenience).
//   PWR = 0x0000_002D (powered down).
//   FBDIV_INT = 0.
//   PRIM = 0x0007_7000 (POSTDIV1 = 7, POSTDIV2 = 7).

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

/// Read a PLL register, forcing CS[31] (LOCK) high so firmware lock-poll
/// loops fall through immediately.
pub fn pll_read(regs: &PllRegs, offset: u32) -> u32 {
    match pll_reg_index(offset) {
        Some(0) => regs[0] | (1 << 31),
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

    tree.ref_clk_hz = ref_hz;
    tree.sys_clk_hz = sys_hz;
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
        let regs: PllRegs = [0; 4];
        let cs = pll_read(&regs, 0x00);
        assert_ne!(cs & (1 << 31), 0, "CS[31] LOCK must read as 1");
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
}
