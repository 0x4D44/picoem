use super::Bus;

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
            0x038 => 0x1, // CLK_REF_SELECTED
            0x068 => 0x1, // CLK_SYS_SELECTED
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

    // --- PLL_SYS (0x40050000) ---
    pub(crate) fn pll_sys_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => 1 << 31, // CS: LOCK bit set
            _ => 0,
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
