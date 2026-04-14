use super::Bus;

/// Map a PLL register offset (`0x000`, `0x004`, `0x008`, `0x00C`) to
/// its index in a `[u32; 4]` register image. Returns `None` for
/// unknown offsets — callers should ignore those.
fn pll_reg_index(offset: u32) -> Option<usize> {
    match offset {
        0x000 => Some(0),
        0x004 => Some(1),
        0x008 => Some(2),
        0x00C => Some(3),
        _ => None,
    }
}

/// Read a PLL register, forcing the LOCK bit (`CS[31]`) on the CS
/// register so firmware poll loops succeed immediately. Shared
/// between PLL_SYS and PLL_USB.
fn pll_read_from(regs: &[u32; 4], offset: u32) -> u32 {
    match pll_reg_index(offset) {
        Some(0) => regs[0] | (1 << 31), // CS: always report LOCK=1
        Some(i) => regs[i],
        None => 0,
    }
}

/// Apply an alias-aware write to a PLL register image. `alias`
/// follows the usual APB convention: 0=normal, 1=XOR, 2=SET, 3=CLR.
/// Unknown offsets are silently dropped — real hardware also ignores
/// accesses outside the 16-byte window.
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

impl Bus {
    // --- SYSINFO (0x40000000) — read-only ---
    pub(crate) fn sysinfo_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => 0x0000_0002, // CHIP_ID: RP2350
            0x004 => 0x0000_0000, // PACKAGE_SEL: RP2350A (QFN60)
            0x008 => 0x0000_0001, // PLATFORM: ASIC
            _ => 0,
        }
    }

    // --- RESETS (0x40020000) ---
    pub(crate) fn resets_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.resets_state,  // RESET
            0x004 => 0,                  // WDSEL (ignored)
            0x008 => !self.resets_state, // RESET_DONE (instant completion)
            _ => 0,
        }
    }

    pub(crate) fn resets_write(&mut self, offset: u32, val: u32, alias: u32) {
        if offset == 0x000 {
            self.resets_state = match alias {
                0 => val,
                1 => self.resets_state ^ val,
                2 => self.resets_state | val,
                3 => self.resets_state & !val,
                _ => unreachable!(),
            };
        }
    }

    // --- CLOCKS (0x40010000) ---
    pub(crate) fn clocks_read(&self, offset: u32) -> u32 {
        match offset {
            0x030 => self.clk_ref_ctrl,
            0x038 => 1 << (self.clk_ref_ctrl & 0x3), // CLK_REF_SELECTED
            0x060 => self.clk_sys_ctrl,
            0x064 => self.clk_sys_div,
            0x068 => 1 << (self.clk_sys_ctrl & 0x1), // CLK_SYS_SELECTED
            _ => 0,
        }
    }

    /// Apply an alias-aware write to one of the CLOCKS registers.
    ///
    /// `alias` encodes the atomic-access kind (0 = normal, 1 = XOR,
    /// 2 = SET, 3 = CLR), matching the RP2350 APB aperture convention
    /// already used by `resets_write`. After the underlying register
    /// is updated, `recompute_clock_tree` refreshes the derived
    /// `sys_clk_hz` / `ref_clk_hz` values.
    pub(crate) fn clocks_write(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |current: u32| match alias {
            0 => val,
            1 => current ^ val,
            2 => current | val,
            3 => current & !val,
            _ => val,
        };
        match offset {
            0x030 => self.clk_ref_ctrl = apply(self.clk_ref_ctrl),
            0x060 => self.clk_sys_ctrl = apply(self.clk_sys_ctrl),
            0x064 => self.clk_sys_div = apply(self.clk_sys_div),
            _ => {}
        }
        self.recompute_clock_tree();
    }

    // --- ROSC (0x400E8000) ---
    pub(crate) fn rosc_read(&self, offset: u32) -> u32 {
        match offset {
            0x018 => (1 << 31) | (1 << 12), // STATUS: STABLE | ENABLED
            _ => 0,
        }
    }

    // --- XOSC (0x40048000) ---
    pub(crate) fn xosc_read(&self, offset: u32) -> u32 {
        match offset {
            0x004 => (1 << 31) | (1 << 12), // STATUS: STABLE + ENABLED
            _ => 0,
        }
    }

    // --- PLL_SYS (0x40050000) / PLL_USB (0x40058000) ---
    //
    // Both PLLs share the same register layout: CS (0x000), PWR (0x004),
    // FBDIV_INT (0x008), PRIM (0x00C). We always force the LOCK bit
    // (CS[31]) so firmware polling for lock succeeds on the first read
    // — see LLD V2 §9 risk 2 for the known fidelity gap.
    pub(crate) fn pll_sys_read(&self, offset: u32) -> u32 {
        pll_read_from(&self.pll_sys_regs, offset)
    }

    pub(crate) fn pll_sys_write(&mut self, offset: u32, val: u32, alias: u32) {
        pll_write_into(&mut self.pll_sys_regs, offset, val, alias);
        self.recompute_clock_tree();
    }

    pub(crate) fn pll_usb_read(&self, offset: u32) -> u32 {
        pll_read_from(&self.pll_usb_regs, offset)
    }

    pub(crate) fn pll_usb_write(&mut self, offset: u32, val: u32, alias: u32) {
        pll_write_into(&mut self.pll_usb_regs, offset, val, alias);
        self.recompute_clock_tree();
    }

    // --- QMI (0x400D0000) --- QSPI memory interface
    pub(crate) fn qmi_read(&self, offset: u32) -> u32 {
        match offset {
            // DIRECT_CSR: force TXEMPTY (bit 16) + RXEMPTY (bit 17) always set
            0x000 => self.qmi_regs.get(0).copied().unwrap_or(0) | (1 << 16) | (1 << 17),
            _ => {
                let idx = (offset >> 2) as usize;
                self.qmi_regs.get(idx).copied().unwrap_or(0)
            }
        }
    }

    pub(crate) fn qmi_write(&mut self, offset: u32, val: u32) {
        let idx = (offset >> 2) as usize;
        if idx < self.qmi_regs.len() {
            self.qmi_regs[idx] = val;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::bus::Bus;

    #[test]
    fn test_sysinfo_chip_id() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(0x4000_0000), 0x0000_0002);
    }

    #[test]
    fn test_sysinfo_platform() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(0x4000_0008), 0x0000_0001);
    }

    #[test]
    fn test_resets_default_all_in_reset() {
        let bus = Bus::new();
        assert_eq!(bus.resets_state, 0x1FFF_FFFF);
    }

    #[test]
    fn test_resets_clear_deassert() {
        let mut bus = Bus::new();
        // Write via CLR alias (alias 3) to deassert all resets
        // CLR alias address: base 0x4002_0000 + offset 0x000 + alias 3 => 0x4002_3000
        bus.write32(0x4002_3000, 0x1FFF_FFFF);
        // RESET register should now be 0
        assert_eq!(bus.read32(0x4002_0000), 0x0000_0000);
        // RESET_DONE should be all 1s
        assert_eq!(bus.read32(0x4002_0008), 0xFFFF_FFFF);
    }

    #[test]
    fn test_xosc_stable() {
        let mut bus = Bus::new();
        let status = bus.read32(0x4004_8004);
        assert_ne!(status & (1 << 31), 0, "STABLE bit should be set");
    }

    #[test]
    fn test_pll_locked() {
        let mut bus = Bus::new();
        let cs = bus.read32(0x4005_0000);
        assert_ne!(cs & (1 << 31), 0, "LOCK bit should be set");
    }

    #[test]
    fn test_clk_sys_selected() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(0x4001_0068), 0x1);
    }
}
