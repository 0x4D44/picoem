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
}

impl Default for Ppb {
    fn default() -> Self {
        Self::new()
    }
}
