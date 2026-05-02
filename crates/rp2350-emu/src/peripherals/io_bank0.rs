//! RP2350 IO_BANK0 — GPIO pin-mux and interrupt registers (datasheet
//! §9.11, base `0x4002_8000`).
//!
//! Phase 2 of the RP2350 peripheral coverage plan (HLD V5 §5.8). 48
//! user GPIOs — each has:
//!
//! * `GPIOn_STATUS` at `0x00 + n*0x08` — read-only mirror of CTRL.
//!   For V5 scope we return the stored CTRL word; firmware writes to
//!   STATUS are ignored.
//! * `GPIOn_CTRL` at `0x04 + n*0x08` — plain-storage 32-bit word
//!   covering FUNCSEL, OUTOVER, OEOVER, INOVER, IRQOVER (datasheet
//!   §9.11.2.1). **No active pin routing** — peripherals do not drive
//!   pin state through FUNCSEL in V5 (§4 observability constraint).
//!
//! After the 48-pin CTRL/STATUS pairs, the per-proc interrupt registers:
//!
//! * `INTRn` (0x230..0x240) — interrupt raw status per 8 GPIOs.
//! * `PROC0_INTEn` / `PROC0_INTFn` / `PROC0_INTSn` — proc0 mask / force
//!   / status for 8-GPIO groups.
//! * `PROC1_INTEn` / `PROC1_INTFn` / `PROC1_INTSn` — ditto for proc1.
//! * `DORMANT_WAKE_INTEn` / `DORMANT_WAKE_INTFn` / `DORMANT_WAKE_INTSn`.
//!
//! For V5 scope these are all plain-storage u32 arrays. No GPIO IRQ
//! routing — firmware that reads them back sees the stored bits. When a
//! scenario needs active GPIO IRQs, that's a later phase.

use super::apply_alias_rmw;

/// IO_BANK0 base (RP2350 datasheet §9.11).
pub const IO_BANK0_BASE: u32 = 0x4002_8000;

/// Number of user GPIOs on RP2350.
pub const GPIO_COUNT: usize = 48;

/// Stride between consecutive GPIO banks.
const GPIO_STRIDE: u32 = 0x08;
/// `GPIOn_STATUS` offset within a bank.
const GPIO_STATUS_OFFSET: u32 = 0x00;
/// `GPIOn_CTRL` offset within a bank.
const GPIO_CTRL_OFFSET: u32 = 0x04;

/// End of the per-GPIO bank region. `0x04 + 47*0x08 = 0x17C + 4 = 0x180`.
const GPIO_BANK_END: u32 = (GPIO_COUNT as u32) * GPIO_STRIDE;

/// Start of the interrupt-register block (INTR[0..5]). Datasheet §9.11.2
/// places it immediately after the 48-GPIO bank; each INTRn covers 8
/// GPIOs (4 bits per pin = 32 bits per group).
const INT_BLOCK_START: u32 = 0x230;
/// Number of 8-GPIO INT groups (48 ÷ 8).
const INT_GROUP_COUNT: usize = 6;
const INT_GROUP_BYTES: u32 = 4;

/// Offsets for each grouped IRQ register set. Six words per section.
const INTR_OFFSET: u32 = INT_BLOCK_START;
const PROC0_INTE_OFFSET: u32 = INT_BLOCK_START + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const PROC0_INTF_OFFSET: u32 = PROC0_INTE_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const PROC0_INTS_OFFSET: u32 = PROC0_INTF_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const PROC1_INTE_OFFSET: u32 = PROC0_INTS_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const PROC1_INTF_OFFSET: u32 = PROC1_INTE_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const PROC1_INTS_OFFSET: u32 = PROC1_INTF_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const DORMANT_WAKE_INTE_OFFSET: u32 = PROC1_INTS_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const DORMANT_WAKE_INTF_OFFSET: u32 =
    DORMANT_WAKE_INTE_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
const DORMANT_WAKE_INTS_OFFSET: u32 =
    DORMANT_WAKE_INTF_OFFSET + INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;

/// Plain-storage IO_BANK0 register set.
pub struct IoBank0Regs {
    /// CTRL per GPIO.
    ctrl: [u32; GPIO_COUNT],
    /// INTR / per-proc / dormant interrupt words. 6 groups × 9 sections
    /// = 54 words total.
    intr: [u32; INT_GROUP_COUNT],
    proc0_inte: [u32; INT_GROUP_COUNT],
    proc0_intf: [u32; INT_GROUP_COUNT],
    proc1_inte: [u32; INT_GROUP_COUNT],
    proc1_intf: [u32; INT_GROUP_COUNT],
    dormant_wake_inte: [u32; INT_GROUP_COUNT],
    dormant_wake_intf: [u32; INT_GROUP_COUNT],
}

impl IoBank0Regs {
    pub fn new() -> Self {
        Self {
            ctrl: [0; GPIO_COUNT],
            intr: [0; INT_GROUP_COUNT],
            proc0_inte: [0; INT_GROUP_COUNT],
            proc0_intf: [0; INT_GROUP_COUNT],
            proc1_inte: [0; INT_GROUP_COUNT],
            proc1_intf: [0; INT_GROUP_COUNT],
            dormant_wake_inte: [0; INT_GROUP_COUNT],
            dormant_wake_intf: [0; INT_GROUP_COUNT],
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Decode an offset into `(gpio, field)` if it lands in the per-GPIO
    /// bank area. `field` is 0 for STATUS and 4 for CTRL.
    fn decode_gpio_offset(offset: u32) -> Option<(usize, u32)> {
        if offset >= GPIO_BANK_END {
            return None;
        }
        let gpio = (offset / GPIO_STRIDE) as usize;
        let field = offset % GPIO_STRIDE;
        Some((gpio, field))
    }

    /// Decode an interrupt-block offset into `(section, group)`. Section
    /// tells which array (INTR/PROC0_INTE/..); group indexes 0..6.
    fn decode_int_offset(offset: u32) -> Option<(u8, usize)> {
        let group_size = INT_GROUP_COUNT as u32 * INT_GROUP_BYTES;
        let table = [
            (0u8, INTR_OFFSET),
            (1, PROC0_INTE_OFFSET),
            (2, PROC0_INTF_OFFSET),
            (3, PROC0_INTS_OFFSET),
            (4, PROC1_INTE_OFFSET),
            (5, PROC1_INTF_OFFSET),
            (6, PROC1_INTS_OFFSET),
            (7, DORMANT_WAKE_INTE_OFFSET),
            (8, DORMANT_WAKE_INTF_OFFSET),
            (9, DORMANT_WAKE_INTS_OFFSET),
        ];
        for (section, start) in table {
            let end = start + group_size;
            if offset >= start && offset < end {
                let group = ((offset - start) / INT_GROUP_BYTES) as usize;
                return Some((section, group));
            }
        }
        None
    }

    /// Derive PROCn_INTS = (INTR | PROCn_INTF) & PROCn_INTE per group.
    fn proc_ints(&self, proc: usize, group: usize) -> u32 {
        let (inte, intf) = match proc {
            0 => (self.proc0_inte[group], self.proc0_intf[group]),
            _ => (self.proc1_inte[group], self.proc1_intf[group]),
        };
        (self.intr[group] | intf) & inte
    }

    fn dormant_wake_ints(&self, group: usize) -> u32 {
        (self.intr[group] | self.dormant_wake_intf[group]) & self.dormant_wake_inte[group]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        if let Some((gpio, field)) = Self::decode_gpio_offset(offset) {
            return match field {
                GPIO_STATUS_OFFSET => self.ctrl[gpio], // STATUS mirrors CTRL in V5
                GPIO_CTRL_OFFSET => self.ctrl[gpio],
                _ => 0,
            };
        }
        if let Some((section, group)) = Self::decode_int_offset(offset) {
            return match section {
                0 => self.intr[group],
                1 => self.proc0_inte[group],
                2 => self.proc0_intf[group],
                3 => self.proc_ints(0, group),
                4 => self.proc1_inte[group],
                5 => self.proc1_intf[group],
                6 => self.proc_ints(1, group),
                7 => self.dormant_wake_inte[group],
                8 => self.dormant_wake_intf[group],
                9 => self.dormant_wake_ints(group),
                _ => 0,
            };
        }
        0
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        if let Some((gpio, field)) = Self::decode_gpio_offset(offset) {
            match field {
                GPIO_STATUS_OFFSET => {} // read-only
                GPIO_CTRL_OFFSET => {
                    let mut stored = self.ctrl[gpio];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.ctrl[gpio] = stored;
                }
                _ => {}
            }
            return;
        }
        if let Some((section, group)) = Self::decode_int_offset(offset) {
            match section {
                0 => {
                    // INTR is nominally W1C on the "edge" bits (0,1,4,5)
                    // per datasheet §9.11.2.6. Plain storage in V5: we
                    // accept writes as alias-applied storage updates so
                    // firmware round-trip works; the per-bit W1C
                    // semantics are a future enhancement once a scenario
                    // depends on it.
                    let mut stored = self.intr[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.intr[group] = stored;
                }
                1 => {
                    let mut stored = self.proc0_inte[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.proc0_inte[group] = stored;
                }
                2 => {
                    let mut stored = self.proc0_intf[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.proc0_intf[group] = stored;
                }
                3 => {} // INTS is read-only (derived)
                4 => {
                    let mut stored = self.proc1_inte[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.proc1_inte[group] = stored;
                }
                5 => {
                    let mut stored = self.proc1_intf[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.proc1_intf[group] = stored;
                }
                6 => {} // read-only
                7 => {
                    let mut stored = self.dormant_wake_inte[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.dormant_wake_inte[group] = stored;
                }
                8 => {
                    let mut stored = self.dormant_wake_intf[group];
                    apply_alias_rmw(&mut stored, value, alias);
                    self.dormant_wake_intf[group] = stored;
                }
                9 => {} // read-only
                _ => {}
            }
        }
    }
}

impl Default for IoBank0Regs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gpio_ctrl_offset(pin: u32) -> u32 {
        pin * GPIO_STRIDE + GPIO_CTRL_OFFSET
    }

    fn gpio_status_offset(pin: u32) -> u32 {
        pin * GPIO_STRIDE + GPIO_STATUS_OFFSET
    }

    #[test]
    fn reset_defaults_all_zero() {
        let b = IoBank0Regs::new();
        assert_eq!(b.ctrl.iter().sum::<u32>(), 0);
    }

    #[test]
    fn gpio_ctrl_roundtrips_for_pin_0() {
        let mut b = IoBank0Regs::new();
        // FUNCSEL=2 (UART0) in the low 5 bits.
        b.write32(gpio_ctrl_offset(0), 0x0000_0002, 0);
        assert_eq!(b.read32(gpio_ctrl_offset(0)), 0x0000_0002);
    }

    #[test]
    fn gpio_ctrl_roundtrips_for_last_pin() {
        let mut b = IoBank0Regs::new();
        b.write32(gpio_ctrl_offset(47), 0xDEAD_BEEF, 0);
        assert_eq!(b.read32(gpio_ctrl_offset(47)), 0xDEAD_BEEF);
    }

    #[test]
    fn status_reads_cap_ctrl_in_v5() {
        let mut b = IoBank0Regs::new();
        b.write32(gpio_ctrl_offset(5), 0x0000_1234, 0);
        // STATUS mirrors CTRL — V5 plain-storage model.
        assert_eq!(b.read32(gpio_status_offset(5)), 0x0000_1234);
    }

    #[test]
    fn status_writes_dropped() {
        let mut b = IoBank0Regs::new();
        b.write32(gpio_status_offset(0), 0xFFFF_FFFF, 0);
        assert_eq!(b.read32(gpio_status_offset(0)), 0);
    }

    #[test]
    fn ctrl_bitset_alias() {
        let mut b = IoBank0Regs::new();
        b.write32(gpio_ctrl_offset(0), 0x1, 0);
        b.write32(gpio_ctrl_offset(0), 0x6, 2); // BITSET
        assert_eq!(b.read32(gpio_ctrl_offset(0)), 0x7);
    }

    #[test]
    fn ctrl_bitclr_alias() {
        let mut b = IoBank0Regs::new();
        b.write32(gpio_ctrl_offset(0), 0x0000_000F, 0);
        b.write32(gpio_ctrl_offset(0), 0x0000_0006, 3); // BITCLR
        assert_eq!(b.read32(gpio_ctrl_offset(0)), 0x0000_0009);
    }

    #[test]
    fn int_registers_plain_storage_roundtrip() {
        let mut b = IoBank0Regs::new();
        b.write32(PROC0_INTE_OFFSET, 0x1234, 0);
        assert_eq!(b.read32(PROC0_INTE_OFFSET), 0x1234);
        b.write32(DORMANT_WAKE_INTF_OFFSET + 4, 0x5678, 0);
        assert_eq!(b.read32(DORMANT_WAKE_INTF_OFFSET + 4), 0x5678);
    }

    #[test]
    fn proc0_ints_is_derived_read_only() {
        let mut b = IoBank0Regs::new();
        b.write32(INTR_OFFSET, 0xF0, 0);
        b.write32(PROC0_INTE_OFFSET, 0xC0, 0);
        // INTS = (INTR | INTF) & INTE = 0xC0.
        assert_eq!(b.read32(PROC0_INTS_OFFSET), 0xC0);
        // Write to INTS is ignored.
        b.write32(PROC0_INTS_OFFSET, 0xFFFF_FFFF, 0);
        assert_eq!(b.read32(PROC0_INTS_OFFSET), 0xC0);
    }
}
