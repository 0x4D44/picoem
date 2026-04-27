//! RP2350 USB controller — register-surface stub (Component 1 of the
//! V5 USB+HSTX+Bootrom HLD).
//!
//! No protocol, no host/device enumeration. Goal is "pico-sdk firmware
//! that touches USB init runs through it without faulting." Anything
//! that polls live USB status (BUS_RESET, line state) is out of scope
//! and parks where the firmware would park on real silicon with no
//! device attached — matching V5 HLD §Component 1 "Static 'no device'"
//! contract.
//!
//! # Address map
//!
//! | Region                | Base         | Size  | Purpose                            |
//! |-----------------------|--------------|-------|------------------------------------|
//! | `USBCTRL_DPRAM_BASE`  | `0x5010_0000`| 4 KB  | Endpoint buffers + buffer-control  |
//! | `USBCTRL_REGS_BASE`   | `0x5011_0000`| 16 KB | MAIN_CTRL / SIE_* / INTR / INTS / … |
//!
//! DPRAM is plain RAM (4096 bytes). The lower `0x000..0x180` is the
//! buffer-control + setup-packet region; `0x180..0x1000` is endpoint
//! data buffers. `reset()` zeros only the buffer-control window per
//! stub policy (V5 HLD §Component 1 — a stub's reset behaviour is
//! deliberate, not a silicon claim, since real DPRAM contents are
//! unspecified post-reset).
//!
//! # SIE_STATUS — stub policy
//!
//! Stub holds SIE_STATUS at `0` unconditionally. Reads return `0`;
//! writes — W1C or otherwise — are silently dropped. This is stricter
//! than silicon (which has W1C bits firmware would write `1` to clear)
//! but observationally correct for a stub that never reports a device
//! event: there is no non-zero state to clear. The HLD-cited W1C bits
//! (V5 §Component 1) are `{SUSPENDED=4, RESUME=11, VBUS_DETECTED=17,
//! TRANS_COMPLETE=18, BUS_RESET=19}`; additional W1C bits described
//! in datasheet §12.6.5 are deferred until a silicon oracle
//! (`silicon_periph_diff_rp2350` USB scenario) confirms them.
//!
//! # Risks (V5 HLD §Risks)
//!
//! - Permanently-low `IRQ_USBCTRL_IRQ` — safe for device-mode init
//!   only. Host-mode init that polls `BUS_RESET` will hang. No in-tree
//!   RP2350 firmware currently exercises host mode.

/// USB control-register base — `0x5011_0000` (RP2350 datasheet §Appendix A).
pub const USBCTRL_REGS_BASE: u32 = 0x5011_0000;
/// USB DPRAM base — `0x5010_0000`, 4 KB.
pub const USBCTRL_DPRAM_BASE: u32 = 0x5010_0000;
/// DPRAM size in bytes.
pub const USBCTRL_DPRAM_SIZE: u32 = 0x1000;
/// Buffer-control window cleared on `reset()` (V5 HLD §Component 1).
pub const DPRAM_BUFFER_CONTROL_END: u32 = 0x180;

// --- Register offsets (datasheet §12.6.5 / pico-sdk hardware/regs/usb.h) ---
const ADDR_ENDP: u32 = 0x000;
const INT_EP_CTRL: u32 = 0x008;
const MAIN_CTRL: u32 = 0x040;
const SOF_WR: u32 = 0x044;
const SOF_RD: u32 = 0x048;
const SIE_CTRL: u32 = 0x04C;
const SIE_STATUS: u32 = 0x050;
const INT_EP_STALL_ARM: u32 = 0x054;
const BUFF_STATUS: u32 = 0x058;
const BUFF_CPU_SHOULD_HANDLE: u32 = 0x05C;
const EP_ABORT: u32 = 0x060;
const EP_ABORT_DONE: u32 = 0x064;
const EP_STALL_ARM: u32 = 0x068;
const NAK_POLL: u32 = 0x06C;
const EP_STATUS_STALL_NAK: u32 = 0x070;
const USB_MUXING: u32 = 0x074;
const USB_PWR: u32 = 0x078;
const USBPHY_DIRECT: u32 = 0x07C;
const USBPHY_DIRECT_OVERRIDE: u32 = 0x080;
const USBPHY_TRIM: u32 = 0x084;
const LINESTATE_TUNING: u32 = 0x088;
const INTR: u32 = 0x08C;
const INTE: u32 = 0x090;
const INTF: u32 = 0x094;
const INTS: u32 = 0x098;

/// USB controller — register-surface stub. See module docs.
pub struct UsbCtrl {
    addr_endp: u32,
    int_ep_ctrl: u32,
    main_ctrl: u32,
    sof_wr: u32,
    sie_ctrl: u32,
    int_ep_stall_arm: u32,
    buff_cpu_should_handle: u32,
    ep_abort: u32,
    ep_abort_done: u32,
    ep_stall_arm: u32,
    nak_poll: u32,
    ep_status_stall_nak: u32,
    usb_muxing: u32,
    usb_pwr: u32,
    usbphy_direct: u32,
    usbphy_direct_override: u32,
    usbphy_trim: u32,
    linestate_tuning: u32,
    inte: u32,
    intf: u32,
    /// Dual-port RAM — 4 KB plain memory at `USBCTRL_DPRAM_BASE`.
    /// Boxed because `[u8; 4096]` is too large for a stack-resident
    /// struct (mirrors `Bus::boot_ram`).
    dpram: Box<[u8; 4096]>,
}

impl UsbCtrl {
    /// Construct a fresh USB controller in post-reset state.
    pub fn new() -> Self {
        Self {
            addr_endp: 0,
            int_ep_ctrl: 0,
            main_ctrl: 0,
            sof_wr: 0,
            sie_ctrl: 0,
            int_ep_stall_arm: 0,
            buff_cpu_should_handle: 0,
            ep_abort: 0,
            ep_abort_done: 0,
            ep_stall_arm: 0,
            nak_poll: 0,
            ep_status_stall_nak: 0,
            usb_muxing: 0,
            usb_pwr: 0,
            usbphy_direct: 0,
            usbphy_direct_override: 0,
            usbphy_trim: 0,
            linestate_tuning: 0,
            inte: 0,
            intf: 0,
            dpram: Box::new([0u8; 4096]),
        }
    }

    /// Restore post-reset state. Clears every register and the
    /// `0x000..0x180` DPRAM buffer-control window. The `0x180..0x1000`
    /// data region is left as last-written (V5 HLD §Component 1).
    pub fn reset(&mut self) {
        self.addr_endp = 0;
        self.int_ep_ctrl = 0;
        self.main_ctrl = 0;
        self.sof_wr = 0;
        self.sie_ctrl = 0;
        self.int_ep_stall_arm = 0;
        self.buff_cpu_should_handle = 0;
        self.ep_abort = 0;
        self.ep_abort_done = 0;
        self.ep_stall_arm = 0;
        self.nak_poll = 0;
        self.ep_status_stall_nak = 0;
        self.usb_muxing = 0;
        self.usb_pwr = 0;
        self.usbphy_direct = 0;
        self.usbphy_direct_override = 0;
        self.usbphy_trim = 0;
        self.linestate_tuning = 0;
        self.inte = 0;
        self.intf = 0;
        for byte in &mut self.dpram[..DPRAM_BUFFER_CONTROL_END as usize] {
            *byte = 0;
        }
    }

    /// Read a USB control register by 12-bit offset.
    ///
    /// `&mut self` mirrors other peripheral signatures (UART/SPI need
    /// it for FIFO pops); USB stub never mutates on read.
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            ADDR_ENDP => self.addr_endp,
            INT_EP_CTRL => self.int_ep_ctrl,
            MAIN_CTRL => self.main_ctrl,
            SOF_WR => self.sof_wr,
            // SOF_RD is a free-running counter on silicon; with no SOFs
            // ever generated, returning 0 is the steady-state value.
            SOF_RD => 0,
            SIE_CTRL => self.sie_ctrl,
            // SIE_STATUS: static "no device". Every observable bit
            // (VBUS_DETECTED, CONNECTED, SUSPENDED, RESUME, BUS_RESET,
            // TRANS_COMPLETE, …) reads 0.
            SIE_STATUS => 0,
            INT_EP_STALL_ARM => self.int_ep_stall_arm,
            BUFF_STATUS => 0,
            BUFF_CPU_SHOULD_HANDLE => self.buff_cpu_should_handle,
            EP_ABORT => self.ep_abort,
            EP_ABORT_DONE => self.ep_abort_done,
            EP_STALL_ARM => self.ep_stall_arm,
            NAK_POLL => self.nak_poll,
            EP_STATUS_STALL_NAK => self.ep_status_stall_nak,
            USB_MUXING => self.usb_muxing,
            USB_PWR => self.usb_pwr,
            USBPHY_DIRECT => self.usbphy_direct,
            USBPHY_DIRECT_OVERRIDE => self.usbphy_direct_override,
            USBPHY_TRIM => self.usbphy_trim,
            LINESTATE_TUNING => self.linestate_tuning,
            // INTR / INTS: stub never raises — both read 0 always.
            INTR | INTS => 0,
            INTE => self.inte,
            INTF => self.intf,
            _ => 0,
        }
    }

    /// Write a USB control register. Last-write-wins on plain-storage
    /// registers; W1C registers (SIE_STATUS) silently swallow the
    /// write — they read 0 on the next access regardless. `_irqs`
    /// follows the UART convention (out-mask of 64 NVIC bits) but
    /// this stub never sets a bit.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, _irqs: &mut u64) {
        match offset {
            ADDR_ENDP => super::apply_alias_rmw(&mut self.addr_endp, value, alias),
            INT_EP_CTRL => super::apply_alias_rmw(&mut self.int_ep_ctrl, value, alias),
            MAIN_CTRL => super::apply_alias_rmw(&mut self.main_ctrl, value, alias),
            SOF_WR => super::apply_alias_rmw(&mut self.sof_wr, value, alias),
            // SOF_RD is read-only on silicon.
            SOF_RD => {}
            SIE_CTRL => super::apply_alias_rmw(&mut self.sie_ctrl, value, alias),
            // SIE_STATUS is W1C on real silicon; stub holds it at 0 so
            // any write — W1C or not — is a no-op.
            SIE_STATUS => {}
            INT_EP_STALL_ARM => super::apply_alias_rmw(&mut self.int_ep_stall_arm, value, alias),
            // BUFF_STATUS is W1C in silicon; stub holds at 0.
            BUFF_STATUS => {}
            BUFF_CPU_SHOULD_HANDLE => {
                super::apply_alias_rmw(&mut self.buff_cpu_should_handle, value, alias)
            }
            EP_ABORT => super::apply_alias_rmw(&mut self.ep_abort, value, alias),
            EP_ABORT_DONE => super::apply_alias_rmw(&mut self.ep_abort_done, value, alias),
            EP_STALL_ARM => super::apply_alias_rmw(&mut self.ep_stall_arm, value, alias),
            NAK_POLL => super::apply_alias_rmw(&mut self.nak_poll, value, alias),
            EP_STATUS_STALL_NAK => {
                super::apply_alias_rmw(&mut self.ep_status_stall_nak, value, alias)
            }
            USB_MUXING => super::apply_alias_rmw(&mut self.usb_muxing, value, alias),
            USB_PWR => super::apply_alias_rmw(&mut self.usb_pwr, value, alias),
            USBPHY_DIRECT => super::apply_alias_rmw(&mut self.usbphy_direct, value, alias),
            USBPHY_DIRECT_OVERRIDE => {
                super::apply_alias_rmw(&mut self.usbphy_direct_override, value, alias)
            }
            USBPHY_TRIM => super::apply_alias_rmw(&mut self.usbphy_trim, value, alias),
            LINESTATE_TUNING => super::apply_alias_rmw(&mut self.linestate_tuning, value, alias),
            // INTR / INTS: read-only on silicon.
            INTR | INTS => {}
            INTE => super::apply_alias_rmw(&mut self.inte, value, alias),
            INTF => super::apply_alias_rmw(&mut self.intf, value, alias),
            _ => {}
        }
    }

    /// Read 1, 2, or 4 bytes from DPRAM at byte `offset`. Returns the
    /// value zero-extended into a u32; sub-word reads do not touch
    /// adjacent bytes. Bus alias masking is applied by the caller —
    /// `offset` here is already canonical (`0..0x1000`).
    pub fn read_dpram(&self, offset: u32, size: u8) -> u32 {
        let off = (offset & (USBCTRL_DPRAM_SIZE - 1)) as usize;
        match size {
            1 => self.dpram[off] as u32,
            2 => {
                let lo = self.dpram[off] as u32;
                let hi = self.dpram[off + 1] as u32;
                lo | (hi << 8)
            }
            4 => {
                let b0 = self.dpram[off] as u32;
                let b1 = self.dpram[off + 1] as u32;
                let b2 = self.dpram[off + 2] as u32;
                let b3 = self.dpram[off + 3] as u32;
                b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            }
            _ => 0,
        }
    }

    /// Write 1, 2, or 4 bytes into DPRAM at byte `offset`. Plain
    /// last-write-wins; APB-style alias semantics do not apply
    /// (DPRAM is AHB-Lite plain memory).
    pub fn write_dpram(&mut self, offset: u32, value: u32, size: u8) {
        let off = (offset & (USBCTRL_DPRAM_SIZE - 1)) as usize;
        match size {
            1 => self.dpram[off] = value as u8,
            2 => {
                self.dpram[off] = value as u8;
                self.dpram[off + 1] = (value >> 8) as u8;
            }
            4 => {
                self.dpram[off] = value as u8;
                self.dpram[off + 1] = (value >> 8) as u8;
                self.dpram[off + 2] = (value >> 16) as u8;
                self.dpram[off + 3] = (value >> 24) as u8;
            }
            _ => {}
        }
    }
}

impl Default for UsbCtrl {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bus;
    use crate::bus::RESET_USBCTRL;
    use crate::irq::IRQ_USBCTRL_IRQ;

    const RESETS_BASE: u32 = 0x4002_0000;
    const RESETS_CLR: u32 = RESETS_BASE + 0x3000;

    fn release_usbctrl(bus: &mut Bus) {
        // USBCTRL is held in reset post-bootrom; firmware must release
        // it before any register read returns non-zero. Mirrors the
        // `runtime_init_bootrom_reset` step pico-sdk emits.
        bus.write32(RESETS_CLR, 1 << RESET_USBCTRL, 0);
    }

    // --- Module-level (no Bus) -------------------------------------------

    #[test]
    fn default_reads_static_no_device() {
        let mut u = UsbCtrl::new();
        assert_eq!(
            u.read32(SIE_STATUS),
            0,
            "VBUS_DETECTED, CONNECTED, etc all 0"
        );
        assert_eq!(u.read32(BUFF_STATUS), 0);
        assert_eq!(u.read32(INTS), 0);
        assert_eq!(u.read32(INTR), 0);
    }

    #[test]
    fn pico_sdk_init_sequence_no_fault() {
        let mut u = UsbCtrl::new();
        let mut irqs = 0u64;
        // Slim transcription of pico-sdk's `usb_device_init`: program
        // USB_MUXING + USB_PWR, set MAIN_CTRL.CONTROLLER_EN, then
        // SIE_CTRL with PULLUP_EN. Read SIE_STATUS — must report no
        // VBUS, no IRQ.
        u.write32(USB_MUXING, 0x00000009, 0, &mut irqs); // TO_PHY | SOFTCON
        u.write32(USB_PWR, 0x000000A0, 0, &mut irqs); // VBUS_DETECT_OVERRIDE_EN | VBUS_EN_OVERRIDE_EN
        u.write32(MAIN_CTRL, 0x00000001, 0, &mut irqs); // CONTROLLER_EN
        u.write32(SIE_CTRL, 0x00010000, 0, &mut irqs); // PULLUP_EN
        let status = u.read32(SIE_STATUS);
        assert_eq!(status & (1 << 17), 0, "VBUS_DETECTED clear");
        assert_eq!(status & (1 << 16), 0, "CONNECTED clear");
        assert_eq!(status, 0, "every status bit must read 0 on stub");
        assert_eq!(irqs, 0, "init sequence raises no NVIC bit");
    }

    #[test]
    fn sie_status_writes_drop_silently_reads_return_zero() {
        let mut u = UsbCtrl::new();
        let mut irqs = 0u64;
        // The HLD-cited W1C bits (V5 §Component 1): {SUSPENDED=4,
        // RESUME=11, VBUS_DETECTED=17, TRANS_COMPLETE=18, BUS_RESET=19}.
        // For each, writing `1` must leave the register at 0.
        for bit in [4, 11, 17, 18, 19] {
            u.write32(SIE_STATUS, 1u32 << bit, 0, &mut irqs);
            let status = u.read32(SIE_STATUS);
            assert_eq!(
                status & (1u32 << bit),
                0,
                "SIE_STATUS bit {bit} must read 0 after write",
            );
        }
        // A write with all-ones still reads 0 (writes drop silently).
        u.write32(SIE_STATUS, 0xFFFF_FFFF, 0, &mut irqs);
        assert_eq!(u.read32(SIE_STATUS), 0);
        assert_eq!(irqs, 0);
    }

    #[test]
    fn plain_storage_registers_round_trip() {
        let mut u = UsbCtrl::new();
        let mut irqs = 0u64;
        u.write32(MAIN_CTRL, 0x0000_0001, 0, &mut irqs);
        u.write32(SIE_CTRL, 0x4000_0000, 0, &mut irqs);
        u.write32(USB_MUXING, 0xDEAD_BEEF, 0, &mut irqs);
        assert_eq!(u.read32(MAIN_CTRL), 0x0000_0001);
        assert_eq!(u.read32(SIE_CTRL), 0x4000_0000);
        assert_eq!(u.read32(USB_MUXING), 0xDEAD_BEEF);
    }

    #[test]
    fn alias_bitset_works_on_main_ctrl() {
        let mut u = UsbCtrl::new();
        let mut irqs = 0u64;
        u.write32(MAIN_CTRL, 0x1, 2 /* BITSET */, &mut irqs);
        u.write32(MAIN_CTRL, 0x4, 2, &mut irqs);
        assert_eq!(u.read32(MAIN_CTRL), 0x5);
        u.write32(MAIN_CTRL, 0x4, 3 /* BITCLR */, &mut irqs);
        assert_eq!(u.read32(MAIN_CTRL), 0x1);
    }

    #[test]
    fn dpram_word_byte_halfword_round_trip() {
        let mut u = UsbCtrl::new();
        // Word-write at the data region (>= 0x180), then byte/halfword
        // reads should pull the right lanes.
        u.write_dpram(0x200, 0xDEAD_BEEF, 4);
        assert_eq!(u.read_dpram(0x200, 4), 0xDEAD_BEEF);
        assert_eq!(u.read_dpram(0x200, 1), 0xEF);
        assert_eq!(u.read_dpram(0x201, 1), 0xBE);
        assert_eq!(u.read_dpram(0x202, 2), 0xDEAD);

        // Byte writes at scattered offsets, then word read merges them.
        u.write_dpram(0x300, 0x11, 1);
        u.write_dpram(0x301, 0x22, 1);
        u.write_dpram(0x302, 0x33, 1);
        u.write_dpram(0x303, 0x44, 1);
        assert_eq!(u.read_dpram(0x300, 4), 0x4433_2211);

        // Halfword write at 0x400.
        u.write_dpram(0x400, 0xCAFE, 2);
        assert_eq!(u.read_dpram(0x400, 2), 0xCAFE);
        assert_eq!(u.read_dpram(0x400, 1), 0xFE);
        assert_eq!(u.read_dpram(0x401, 1), 0xCA);
    }

    #[test]
    fn reset_clears_buffer_control_region_only() {
        let mut u = UsbCtrl::new();
        // Fill all DPRAM with 0xAA via byte writes.
        for off in 0..USBCTRL_DPRAM_SIZE {
            u.write_dpram(off, 0xAA, 1);
        }
        // Sanity: filled.
        assert_eq!(u.read_dpram(0x000, 1), 0xAA);
        assert_eq!(u.read_dpram(0x180, 1), 0xAA);
        assert_eq!(u.read_dpram(0xFFF, 1), 0xAA);

        // Set a control register so we can prove reset clears it too.
        let mut irqs = 0u64;
        u.write32(MAIN_CTRL, 0x1, 0, &mut irqs);
        assert_eq!(u.read32(MAIN_CTRL), 0x1);

        u.reset();

        // Buffer-control window cleared.
        for off in 0..DPRAM_BUFFER_CONTROL_END {
            assert_eq!(
                u.read_dpram(off, 1),
                0,
                "DPRAM[{off:#X}] must be 0 after reset",
            );
        }
        // Data region preserved.
        for off in DPRAM_BUFFER_CONTROL_END..USBCTRL_DPRAM_SIZE {
            assert_eq!(
                u.read_dpram(off, 1),
                0xAA,
                "DPRAM[{off:#X}] must be 0xAA after reset (data region)",
            );
        }
        // Registers cleared.
        assert_eq!(u.read32(MAIN_CTRL), 0);
    }

    // --- Bus-level integration -------------------------------------------

    #[test]
    fn bus_routes_main_ctrl_round_trip_at_canonical_alias() {
        let mut bus = Bus::new();
        release_usbctrl(&mut bus);
        bus.write32(USBCTRL_REGS_BASE + MAIN_CTRL, 0x0000_0001, 0);
        assert_eq!(bus.read32(USBCTRL_REGS_BASE + MAIN_CTRL, 0), 0x0000_0001);
    }

    #[test]
    fn bus_routes_sie_status_w1c_via_bitset_alias() {
        let mut bus = Bus::new();
        release_usbctrl(&mut bus);
        // Write SIE_STATUS via APB BITSET alias (+0x2000). Stub still
        // reads 0.
        bus.write32(
            USBCTRL_REGS_BASE + SIE_STATUS + 0x2000,
            (1 << 19) | (1 << 18),
            0,
        );
        assert_eq!(bus.read32(USBCTRL_REGS_BASE + SIE_STATUS, 0), 0);
    }

    #[test]
    fn bus_dpram_word_round_trip() {
        let mut bus = Bus::new();
        release_usbctrl(&mut bus);
        bus.write32(USBCTRL_DPRAM_BASE + 0x200, 0xDEAD_BEEF, 0);
        assert_eq!(bus.read32(USBCTRL_DPRAM_BASE + 0x200, 0), 0xDEAD_BEEF);
    }

    #[test]
    fn bus_dpram_byte_halfword_round_trip() {
        let mut bus = Bus::new();
        release_usbctrl(&mut bus);
        bus.write8(USBCTRL_DPRAM_BASE + 0x300, 0x11, 0);
        bus.write8(USBCTRL_DPRAM_BASE + 0x301, 0x22, 0);
        bus.write16(USBCTRL_DPRAM_BASE + 0x302, 0x4433, 0);
        assert_eq!(bus.read32(USBCTRL_DPRAM_BASE + 0x300, 0), 0x4433_2211);
        assert_eq!(bus.read16(USBCTRL_DPRAM_BASE + 0x300, 0), 0x2211);
        assert_eq!(bus.read8(USBCTRL_DPRAM_BASE + 0x303, 0), 0x44);
    }

    #[test]
    fn bus_reads_zero_while_held_in_reset() {
        let mut bus = Bus::new();
        // Don't release USBCTRL; reads must return 0.
        bus.write32(USBCTRL_REGS_BASE + MAIN_CTRL, 0x1, 0);
        assert_eq!(
            bus.read32(USBCTRL_REGS_BASE + MAIN_CTRL, 0),
            0,
            "writes dropped + reads return 0 while USBCTRL held in reset",
        );
    }

    /// Emulator-level: run the pico-sdk init sequence through the
    /// real `Bus`, assert NVIC line 14 (IRQ_USBCTRL_IRQ) is never
    /// pending on either core. Stub never raises, so this is the
    /// invariant we lock in.
    #[test]
    fn usb_irq_stays_low_through_full_init() {
        use crate::{Config, Emulator};
        let mut emu = Emulator::new(Config::default());
        emu.bus.write32(RESETS_CLR, 1 << RESET_USBCTRL, 0);
        emu.bus.write32(USBCTRL_REGS_BASE + USB_MUXING, 0x9, 0);
        emu.bus.write32(USBCTRL_REGS_BASE + USB_PWR, 0xA0, 0);
        emu.bus.write32(USBCTRL_REGS_BASE + MAIN_CTRL, 0x1, 0);
        emu.bus.write32(USBCTRL_REGS_BASE + SIE_CTRL, 0x10000, 0);
        // Read SIE_STATUS via the bus to give the stub a chance to
        // mistakenly raise — it must not.
        let _ = emu.bus.read32(USBCTRL_REGS_BASE + SIE_STATUS, 0);
        let _ = emu.bus.read32(USBCTRL_REGS_BASE + SIE_STATUS, 0);
        for core in 0..2usize {
            let pending = emu.bus.atomics.irq_pending_load(core);
            assert_eq!(
                pending & (1u64 << IRQ_USBCTRL_IRQ),
                0,
                "core {core}: USB IRQ must stay low through init (irq_pending = {pending:#X})",
            );
        }
    }
}
