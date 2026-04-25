//! Private Peripheral Bus (PPB) for Cortex-M0+.
//!
//! M0+ has a much smaller PPB than M33: no MPU, no SAU, no CPACR, no
//! FPCCR. Phase 4.B only needs the fields the exception model reads or
//! writes:
//!
//! * `vtor` — Vector Table Offset Register. Holds the base address of
//!   the vector table. Reset value is 0.
//! * `shpr` — System Handler Priority Registers. Stores priorities for
//!   exceptions 4..15. On M0+ only a subset is configurable (SVCall,
//!   PendSV, SysTick); see [`Ppb::exception_priority`].
//! * `icsr` — Interrupt Control and State Register. Phase 4.B uses the
//!   PENDSVSET / PENDSTSET bits; the nested-exception bookkeeping lives
//!   in the NVIC helpers.
//!
//! Priority format on M0+: the register stores 8-bit values per
//! exception, but only bits [7:6] are implemented. That gives 4 priority
//! levels (0, 0x40, 0x80, 0xC0). Priority 0 is the highest configurable.
//! Fixed-priority exceptions: Reset = -3, NMI = -2, HardFault = -1.

/// System-exception priority register index. SHPR is stored as 12 bytes
/// so exception N ∈ {4..15} maps to index `N - 4`. Keep arithmetic
/// explicit at the call sites — nothing fancy here.
const SHPR_LEN: usize = 12;

/// Fixed priority for Reset (exception 1).
pub(crate) const PRIO_RESET: i16 = -3;
/// Fixed priority for NMI (exception 2).
pub(crate) const PRIO_NMI: i16 = -2;
/// Fixed priority for HardFault (exception 3).
pub(crate) const PRIO_HARDFAULT: i16 = -1;

/// Private Peripheral Bus state — exception-relevant fields only.
#[derive(Clone)]
pub struct Ppb {
    /// Vector Table Offset Register. Must be aligned to 128 bytes
    /// (implementation-defined on M0+, typically a power-of-two ≥ table
    /// size). We do not enforce alignment on write — firmware is
    /// responsible. Exception entry reads `mem[vtor + 4*exc_num]`.
    pub vtor: u32,
    /// System Handler Priority Registers. Only bytes covering exceptions
    /// 11 (SVCall), 14 (PendSV) and 15 (SysTick) are architecturally
    /// defined on M0+; other bytes read-as-zero / write-ignored.
    pub shpr: [u8; SHPR_LEN],
    /// Interrupt Control and State Register. Phase 4.B only uses bits
    /// 28 (PENDSVSET) and 26 (PENDSTSET) — set by firmware to trigger
    /// PendSV / SysTick. Clearing bits and read-as-active bits land in
    /// Phase 5 once the NVIC is wired in.
    pub icsr: u32,
    /// Active-exception bitmap. Bit N = 1 means exception N is currently
    /// executing (has been entered but not yet returned from). Used by
    /// the nested-exception return path to clear the bit on `exit`.
    pub active: u64,
}

impl Ppb {
    /// Construct a reset-state PPB.
    pub fn new() -> Self {
        Self {
            vtor: 0,
            shpr: [0; SHPR_LEN],
            icsr: 0,
            active: 0,
        }
    }

    /// Effective priority for exception `exc_num`. Fixed-priority
    /// exceptions return their architectural constants; configurable
    /// ones come from SHPR bytes 7 / 10 / 11 (for SVCall / PendSV /
    /// SysTick respectively). Bits [5:0] of the priority byte are
    /// RAZ/WI on M0+ — we ignore them here.
    #[inline]
    #[allow(dead_code)] // used starting Phase 5 NVIC wiring
    pub fn exception_priority(&self, exc_num: u16) -> i16 {
        debug_assert!(
            exc_num < 16,
            "exception_priority is for system exceptions only; use Nvic::priority for external IRQs"
        );
        match exc_num {
            1 => PRIO_RESET,
            2 => PRIO_NMI,
            3 => PRIO_HARDFAULT,
            // Exceptions 4..15 — configurable via SHPR.
            4..=15 => {
                let idx = (exc_num - 4) as usize;
                // Only top two bits count → 4 levels.
                (self.shpr[idx] & 0xC0) as i16
            }
            // External IRQs (Phase 5 will plumb NVIC_IPR here).
            _ => 0xFF,
        }
    }

    /// Mark exception `exc_num` as active (entering the handler).
    #[inline]
    pub fn mark_active(&mut self, exc_num: u16) {
        if exc_num < 64 {
            self.active |= 1u64 << exc_num;
        }
    }

    /// Mark exception `exc_num` as no longer active (exception return).
    #[inline]
    pub fn clear_active(&mut self, exc_num: u16) {
        if exc_num < 64 {
            self.active &= !(1u64 << exc_num);
        }
    }

    /// True if any exception is currently active (for nested-exception
    /// return handling).
    #[inline]
    pub fn any_active(&self) -> bool {
        self.active != 0
    }

    /// Read a 32-bit PPB register.
    ///
    /// PPB base is `0xE000_0000`; the SCB lives at `0xE000_ED00..`.
    /// Registers modelled on M0+:
    /// * `CPUID`    at `0xE000_ED00` — read-only constant.
    /// * `ICSR`     at `0xE000_ED04`.
    /// * `VTOR`     at `0xE000_ED08`.
    /// * `AIRCR`    at `0xE000_ED0C` — stub (VECTKEY echoes 0x05FA).
    /// * `SHPR2/3`  at `0xE000_ED1C/20` (SHPR1 is RAZ on M0+).
    /// Other offsets read-as-zero.
    ///
    /// Addresses are decoded by `addr & 0xFFFF`, matching mdrp2350's
    /// idiom — the top nibble (0xE) is already guaranteed by the bus
    /// region decoder, and bits [27:16] are all zero for the SCB
    /// window, so the low 16 bits uniquely identify the register.
    pub fn read32(&self, addr: u32) -> u32 {
        match addr & 0xFFFF {
            0xED00 => 0x410C_C601, // CPUID: Cortex-M0+ r0p1
            0xED04 => self.icsr,
            0xED08 => self.vtor,
            0xED0C => 0xFA05_0000, // AIRCR stub (VECTKEY echo)
            0xED1C => {
                // SHPR2 covers exceptions 8..11 (SVCall at byte 3).
                (self.shpr[7] as u32) << 24
            }
            0xED20 => {
                // SHPR3 covers exceptions 12..15 (PendSV byte 2, SysTick byte 3).
                ((self.shpr[10] as u32) << 16) | ((self.shpr[11] as u32) << 24)
            }
            _ => 0,
        }
    }

    /// Write a 32-bit PPB register.
    pub fn write32(&mut self, addr: u32, val: u32) {
        match addr & 0xFFFF {
            0xED04 => {
                // ICSR: PENDSVSET / PENDSVCLR / PENDSTSET / PENDSTCLR
                // are W1S/W1C bits; Phase 5.A stores the raw value so
                // firmware round-trips, honouring clear bits as well.
                if val & (1 << 27) != 0 {
                    self.icsr &= !(1 << 28);
                }
                if val & (1 << 28) != 0 {
                    self.icsr |= 1 << 28;
                }
                if val & (1 << 25) != 0 {
                    self.icsr &= !(1 << 26);
                }
                if val & (1 << 26) != 0 {
                    self.icsr |= 1 << 26;
                }
            }
            0xED08 => self.vtor = val,
            0xED1C => {
                // SHPR2: byte 3 → SVCall priority (exception 11 → idx 7).
                self.shpr[7] = ((val >> 24) & 0xFF) as u8;
            }
            0xED20 => {
                // SHPR3: byte 2 → PendSV (exc 14 → idx 10),
                //         byte 3 → SysTick (exc 15 → idx 11).
                self.shpr[10] = ((val >> 16) & 0xFF) as u8;
                self.shpr[11] = ((val >> 24) & 0xFF) as u8;
            }
            _ => {}
        }
    }
}

impl Default for Ppb {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpuid_read_returns_m0plus_r0p1() {
        let ppb = Ppb::default();
        // Cortex-M0+ r0p1 CPUID constant per ARM DDI 0484C §B3.2.3.
        assert_eq!(ppb.read32(0xE000_ED00), 0x410C_C601);
    }

    #[test]
    fn vtor_roundtrip_through_memory_mapped_path() {
        // Regression for the Phase-2 mask/pattern bug: Phase 1's
        // direct_boot_from_flash set the field, but pico-sdk's
        // get_vtable() reads VTOR via 0xE000_ED08 and saw 0.
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED08, 0x1000_0100);
        assert_eq!(ppb.read32(0xE000_ED08), 0x1000_0100);
    }

    #[test]
    fn icsr_pendsvset_is_sticky_through_read32() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED04, 1 << 28); // PENDSVSET
        assert_eq!(ppb.read32(0xE000_ED04) & (1 << 28), 1 << 28);
    }

    #[test]
    fn aircr_stub_returns_vectkey_echo() {
        // SDK reset code reads AIRCR to preserve VECTKEYSTAT; stub returns
        // 0xFA05_0000 so a subsequent RMW writes a valid VECTKEY of 0x05FA.
        let ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_ED0C), 0xFA05_0000);
    }

    #[test]
    fn shpr3_roundtrip_pendsv_and_systick() {
        let mut ppb = Ppb::default();
        // byte 2 = PendSV (exc 14 → idx 10), byte 3 = SysTick (exc 15 → idx 11)
        let val = (0xC0u32 << 16) | (0x80u32 << 24);
        ppb.write32(0xE000_ED20, val);
        assert_eq!(ppb.read32(0xE000_ED20), val);
        assert_eq!(ppb.shpr[10], 0xC0);
        assert_eq!(ppb.shpr[11], 0x80);
    }
}
