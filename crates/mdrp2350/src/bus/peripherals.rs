use super::Bus;
use mdpicoem_common::clocks::{pll_cs_read_with_lock, pll_should_arm_lock};

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

/// Read a PLL register. For CS (offset 0x000), the LOCK bit (`CS[31]`)
/// is derived from `(regs, lock_at, now)` via
/// `mdpicoem_common::clocks::pll_cs_read_with_lock` — see
/// `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md`. Other offsets
/// return the raw stored value.
fn pll_read_from(regs: &[u32; 4], offset: u32, lock_at: Option<u64>, now: u64) -> u32 {
    match pll_reg_index(offset) {
        Some(0) => pll_cs_read_with_lock(regs, lock_at, now),
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
    //
    // RP2350 layout (datasheet §8.1). Differs from RP2040: drops RTC,
    // adds HSTX. Full offset map:
    //   0x00-0x2C GPOUT0-3 (CTRL / DIV / SELECTED each)
    //   0x30 CLK_REF_CTRL   / 0x34 CLK_REF_DIV   / 0x38 CLK_REF_SELECTED
    //   0x3C CLK_SYS_CTRL   / 0x40 CLK_SYS_DIV   / 0x44 CLK_SYS_SELECTED
    //   0x48 CLK_PERI_CTRL  / 0x4C CLK_PERI_DIV  / 0x50 CLK_PERI_SELECTED
    //   0x54 CLK_HSTX_CTRL  / 0x58 CLK_HSTX_DIV  / 0x5C CLK_HSTX_SELECTED
    //   0x60 CLK_USB_CTRL   / 0x64 CLK_USB_DIV   / 0x68 CLK_USB_SELECTED
    //   0x6C CLK_ADC_CTRL   / 0x70 CLK_ADC_DIV   / 0x74 CLK_ADC_SELECTED
    //
    // `_SELECTED` handshake (HLD V5 §4.2.10 / §5.7, pico-sdk
    // `clock_configure` busy-wait):
    //   * Glitchless (clk_ref, clk_sys): `_SELECTED = 1 << (CTRL & SRC_MASK)`.
    //   * Non-glitchless (clk_gpout*, clk_peri, clk_hstx, clk_usb,
    //     clk_adc): `_SELECTED` reads `1` unconditionally. Silicon
    //     always reports the AUXSRC path as "selected" from firmware's
    //     perspective; satisfies pico-sdk's
    //     `while (!(selected & (1u << 0)))` after each CTRL write.
    //
    // Ported from mdrp2040 commit `b1a40e4` (RP2040 Phase 0 Wave 1).
    // The mdrp2350 implementation prior to this audit returned 0 for
    // every non-REF/non-SYS `_SELECTED` offset — pico-sdk firmware
    // reprogramming clk_peri / clk_hstx / clk_usb / clk_adc would spin
    // forever.
    pub(crate) fn clocks_read(&self, offset: u32) -> u32 {
        match offset {
            // clk_gpout0..3 — non-glitchless, `_SELECTED` reads 1.
            // CTRL/DIV are plain-storage from `peripheral_regs` (see
            // the HashMap fallthrough in write32 for APB peripherals
            // that don't have a dedicated backing store yet).
            0x000 | 0x004 => 0, // CLK_GPOUT0_CTRL / DIV — storage not modelled
            0x008 => 1,         // CLK_GPOUT0_SELECTED
            0x00C | 0x010 => 0, // CLK_GPOUT1_CTRL / DIV
            0x014 => 1,         // CLK_GPOUT1_SELECTED
            0x018 | 0x01C => 0, // CLK_GPOUT2_CTRL / DIV
            0x020 => 1,         // CLK_GPOUT2_SELECTED
            0x024 | 0x028 => 0, // CLK_GPOUT3_CTRL / DIV
            0x02C => 1,         // CLK_GPOUT3_SELECTED
            // clk_ref — glitchless, 2-bit SRC in CTRL[1:0].
            0x030 => self.clk_ref_ctrl,
            0x038 => 1 << (self.clk_ref_ctrl & 0x3), // CLK_REF_SELECTED
            // clk_sys — glitchless, 1-bit SRC in CTRL[0].
            0x03C => self.clk_sys_ctrl,
            0x040 => self.clk_sys_div,
            0x044 => 1 << (self.clk_sys_ctrl & 0x1), // CLK_SYS_SELECTED
            // clk_peri — non-glitchless.
            0x048 | 0x04C => 0, // CTRL / DIV — storage not modelled
            0x050 => 1,         // CLK_PERI_SELECTED
            // clk_hstx — non-glitchless (RP2350-only block).
            0x054 | 0x058 => 0, // CTRL / DIV
            0x05C => 1,         // CLK_HSTX_SELECTED
            // clk_usb — non-glitchless.
            0x060 | 0x064 => 0,
            0x068 => 1,         // CLK_USB_SELECTED
            // clk_adc — non-glitchless.
            0x06C | 0x070 => 0,
            0x074 => 1,         // CLK_ADC_SELECTED
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
    ///
    /// Only CLK_REF_CTRL / CLK_SYS_CTRL / CLK_SYS_DIV write to real
    /// backing storage today; the other clocks' writes are dropped
    /// (the `_SELECTED` handshake reads `1` regardless of CTRL). This
    /// matches silicon behaviour for firmware that only cares about
    /// completing `clock_configure`'s busy-wait. Per-channel CTRL
    /// storage lands in Phase 2 when clk_peri / clk_usb frequencies
    /// feed real peripheral models.
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
            0x03C => self.clk_sys_ctrl = apply(self.clk_sys_ctrl),
            0x040 => self.clk_sys_div = apply(self.clk_sys_div),
            _ => {}
        }
        self.recompute_clock_tree();
    }

    // --- ROSC (0x400E8000) --- see LLD V2 §4.11
    //
    // Register layout (`rosc_regs` index → offset):
    //   0=CTRL (0x000), 1=FREQA (0x004), 2=FREQB (0x008),
    //   3=RANDOM (0x00C), 4=DORMANT (0x010), 5=DIV (0x014),
    //   6=STATUS (0x018), 7=RANDOMBIT (0x01C), 8=COUNT (0x020).
    //
    // Writes to writable offsets are stored but have no side effect on
    // the fixed 6.5 MHz ROSC output. Reads from RANDOM, STATUS,
    // RANDOMBIT, COUNT return synthesised values (writes are dropped).
    pub(crate) fn rosc_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.rosc_regs[0], // CTRL
            0x004 => self.rosc_regs[1], // FREQA
            0x008 => self.rosc_regs[2], // FREQB
            0x00C => 0,                 // RANDOM — stub (no PRNG)
            0x010 => self.rosc_regs[4], // DORMANT
            0x014 => self.rosc_regs[5], // DIV
            0x018 => (1 << 31) | (1 << 12), // STATUS: STABLE | ENABLED
            0x01C => 0,                 // RANDOMBIT
            0x020 => 0,                 // COUNT
            _ => 0,
        }
    }

    /// Apply an alias-aware write to a ROSC register. `alias` matches
    /// the APB convention (0=normal, 1=XOR, 2=SET, 3=CLR). Read-only
    /// offsets (RANDOM, STATUS, RANDOMBIT, COUNT) ignore writes.
    pub(crate) fn rosc_write(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |current: u32| match alias {
            0 => val,
            1 => current ^ val,
            2 => current | val,
            3 => current & !val,
            _ => val,
        };
        let idx = match offset {
            0x000 => 0, // CTRL
            0x004 => 1, // FREQA
            0x008 => 2, // FREQB
            0x010 => 4, // DORMANT
            0x014 => 5, // DIV
            // 0x00C RANDOM / 0x018 STATUS / 0x01C RANDOMBIT / 0x020 COUNT
            // are read-only — ignore writes.
            _ => return,
        };
        self.rosc_regs[idx] = apply(self.rosc_regs[idx]);
    }

    // --- XOSC (0x40048000) --- see LLD V2 §4.12
    //
    // Register layout (`xosc_regs` index → offset):
    //   0=CTRL (0x000), 1=STATUS (0x004), 2=DORMANT (0x008),
    //   3=STARTUP (0x00C), 4=COUNT (0x01C).
    //
    // STATUS and COUNT are read-only.
    pub(crate) fn xosc_read(&self, offset: u32) -> u32 {
        match offset {
            0x000 => self.xosc_regs[0], // CTRL
            0x004 => (1 << 31) | (1 << 12), // STATUS: STABLE | ENABLED
            0x008 => self.xosc_regs[2], // DORMANT
            0x00C => self.xosc_regs[3], // STARTUP
            0x01C => 0,                 // COUNT
            _ => 0,
        }
    }

    /// Apply an alias-aware write to an XOSC register. STATUS (0x004)
    /// and COUNT (0x01C) are read-only and ignored.
    pub(crate) fn xosc_write(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |current: u32| match alias {
            0 => val,
            1 => current ^ val,
            2 => current | val,
            3 => current & !val,
            _ => val,
        };
        let idx = match offset {
            0x000 => 0, // CTRL
            0x008 => 2, // DORMANT
            0x00C => 3, // STARTUP
            // 0x004 STATUS / 0x01C COUNT are read-only — ignore writes.
            _ => return,
        };
        self.xosc_regs[idx] = apply(self.xosc_regs[idx]);
    }

    // --- PLL_SYS (0x40050000) / PLL_USB (0x40058000) ---
    //
    // Both PLLs share the same register layout: CS (0x000), PWR (0x004),
    // FBDIV_INT (0x008), PRIM (0x00C). CS[31] (LOCK) is derived from
    // register image + `pll_*_lock_at_cycle` + caller-supplied
    // `master_cycle` on each read; writes consult `pll_should_arm_lock`
    // to rearm / drop the arm per HLD §4. See
    // `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md`.
    //
    // Phase 3 Stage 4 (LLD V7 §12): the master-cycle snapshot is now a
    // caller-provided parameter. Single-threaded Bus callers pass
    // `self.master_cycle`; the future WorkerBus caller snapshots
    // `SharedState.master_cycle` lock-free with `load(Acquire)` before
    // calling — the helper then takes the peripherals.clocks lock
    // without serializing on the coordinator's `fetch_add`.
    pub(crate) fn pll_sys_read_at(&self, offset: u32, master_cycle: u64) -> u32 {
        pll_read_from(
            &self.pll_sys_regs,
            offset,
            self.pll_sys_lock_at_cycle,
            master_cycle,
        )
    }

    /// Convenience wrapper: single-threaded Bus read that supplies its
    /// own `self.master_cycle` snapshot. Preserves the legacy call
    /// shape for `bus/mod.rs` dispatch.
    pub(crate) fn pll_sys_read(&self, offset: u32) -> u32 {
        self.pll_sys_read_at(offset, self.master_cycle)
    }

    pub(crate) fn pll_sys_write_at(
        &mut self,
        offset: u32,
        val: u32,
        alias: u32,
        master_cycle: u64,
    ) {
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

    pub(crate) fn pll_sys_write(&mut self, offset: u32, val: u32, alias: u32) {
        let master_cycle = self.master_cycle;
        self.pll_sys_write_at(offset, val, alias, master_cycle);
    }

    pub(crate) fn pll_usb_read_at(&self, offset: u32, master_cycle: u64) -> u32 {
        pll_read_from(
            &self.pll_usb_regs,
            offset,
            self.pll_usb_lock_at_cycle,
            master_cycle,
        )
    }

    pub(crate) fn pll_usb_read(&self, offset: u32) -> u32 {
        self.pll_usb_read_at(offset, self.master_cycle)
    }

    pub(crate) fn pll_usb_write_at(
        &mut self,
        offset: u32,
        val: u32,
        alias: u32,
        master_cycle: u64,
    ) {
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

    pub(crate) fn pll_usb_write(&mut self, offset: u32, val: u32, alias: u32) {
        let master_cycle = self.master_cycle;
        self.pll_usb_write_at(offset, val, alias, master_cycle);
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
        assert_eq!(bus.read32(0x4000_0000, 0), 0x0000_0002);
    }

    #[test]
    fn test_sysinfo_platform() {
        let mut bus = Bus::new();
        assert_eq!(bus.read32(0x4000_0008, 0), 0x0000_0001);
    }

    #[test]
    fn test_resets_default_post_bootrom_state() {
        // HLD V5 §5.7: `Bus::new()` seeds the post-bootrom RESETS
        // state (TIMER0/1, PLL_SYS/USB, IO_BANK0, PADS_BANK0,
        // SYSCFG, SYSINFO released). Hardware-reset value is
        // `0x1FFF_FFFF` (all held); the emulator starts beyond
        // bootrom because `load_image` doesn't run the bootrom.
        let bus = Bus::new();
        assert_eq!(bus.resets_state, crate::bus::RESETS_POST_BOOTROM);
        // Peripherals landed in Phase 1 must be released.
        assert_eq!(
            bus.resets_state & (1u32 << crate::bus::RESET_TIMER0),
            0,
            "TIMER0 must be released at post-bootrom"
        );
        assert_eq!(
            bus.resets_state & (1u32 << crate::bus::RESET_TIMER1),
            0,
            "TIMER1 must be released at post-bootrom"
        );
    }

    #[test]
    fn test_resets_clear_deassert() {
        let mut bus = Bus::new();
        // Write via CLR alias (alias 3) to deassert all resets
        // CLR alias address: base 0x4002_0000 + offset 0x000 + alias 3 => 0x4002_3000
        bus.write32(0x4002_3000, 0x1FFF_FFFF, 0);
        // RESET register should now be 0
        assert_eq!(bus.read32(0x4002_0000, 0), 0x0000_0000);
        // RESET_DONE should be all 1s
        assert_eq!(bus.read32(0x4002_0008, 0), 0xFFFF_FFFF);
    }

    #[test]
    fn test_xosc_stable() {
        let mut bus = Bus::new();
        let status = bus.read32(0x4004_8004, 0);
        assert_ne!(status & (1 << 31), 0, "STABLE bit should be set");
    }

    #[test]
    fn test_pll_locked() {
        // Post-`2026.04.15 HLD - PLL LOCK Modelling` fix: at reset PWR=0x2D
        // (PD+VCOPD set), FBDIV=0 → lock base predicate is false, CS[31]
        // must read 0. This inverts the pre-fix assertion, which was
        // locking in the known bug (see tech_debt.md).
        let mut bus = Bus::new();
        let cs = bus.read32(0x4005_0000, 0);
        assert_eq!(cs & (1 << 31), 0, "LOCK bit must be 0 at reset (PLL unpowered)");
    }

    #[test]
    fn test_clk_sys_selected() {
        let mut bus = Bus::new();
        // RP2350 CLK_SYS_SELECTED at 0x040_10044.
        assert_eq!(bus.read32(0x4001_0044, 0), 0x1);
    }
}
