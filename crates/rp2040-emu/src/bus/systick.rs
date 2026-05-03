//! ARMv6-M SysTick — CPU-local 24-bit countdown timer.
//!
//! Per ARMv6-M ARM §B3.3.1, when the counter reaches 0, on the *next*
//! clock cycle the value is reloaded from `RVR`; if `TICKINT` is set,
//! that reload-tick asserts the SysTick exception (sets
//! `ICSR.PENDSTSET`). The period is therefore `RVR + 1` cycles. With
//! `CVR = 0` after firmware's clearing write, the very first tick
//! reloads and fires.
//!
//! V5 §5.2: One `SysTick` instance per core lives on `Bus` (and on
//! the threaded `WorkerBus`). The slow-path inner loop ticks the
//! active-core's SysTick once per master cycle; on a TICKINT-arm
//! underflow the caller ORs `ICSR.PENDSTSET` (bit 26) onto the
//! active-core PPB.
//!
//! MMIO map (relative to PPB base `0xE000_E000`):
//!
//! | Offset | Name | Notes |
//! |--------|------|-------|
//! | 0x010  | `SYST_CSR`   | ENABLE (0), TICKINT (1), CLKSOURCE (2), COUNTFLAG (16, RC) |
//! | 0x014  | `SYST_RVR`   | reload value, writes truncated to bits[23:0] |
//! | 0x018  | `SYST_CVR`   | current value, any write zeroes CVR + clears COUNTFLAG |
//! | 0x01C  | `SYST_CALIB` | RAZ stub (returns 0)                |

/// ARMv6-M SysTick — CPU-local 24-bit countdown timer.
#[derive(Debug, Default)]
pub struct SysTick {
    /// Control / Status: ENABLE | TICKINT | CLKSOURCE | COUNTFLAG (RC).
    pub(crate) csr: u32,
    /// Reload value (24-bit; high byte forced to zero).
    pub(crate) rvr: u32,
    /// Current value (24-bit; high byte forced to zero).
    pub(crate) cvr: u32,
}

impl SysTick {
    /// All registers zeroed (matches reset state per ARMv6-M ARM
    /// §B3.3.1).
    pub fn new() -> Self {
        Self {
            csr: 0,
            rvr: 0,
            cvr: 0,
        }
    }

    /// True iff `CSR.ENABLE` (bit 0) is set.
    pub fn is_enabled(&self) -> bool {
        self.csr & 1 != 0
    }

    /// MMIO read at `0xE000_Exxx`. Mutates self because reading `CSR`
    /// clears the `COUNTFLAG` (bit 16) per ARMv6-M ARM §B3.3.2 —
    /// "reading SYST_CSR clears bit 16".
    pub fn read32(&mut self, addr: u32) -> u32 {
        match addr & 0xFFFF {
            0xE010 => {
                // CSR: read returns current value, then clears
                // COUNTFLAG (bit 16). Read-clear semantics.
                let v = self.csr;
                self.csr &= !(1 << 16);
                v
            }
            0xE014 => self.rvr,
            0xE018 => self.cvr,
            0xE01C => 0, // CALIB — RAZ stub
            _ => 0,
        }
    }

    /// MMIO write at `0xE000_Exxx`.
    pub fn write32(&mut self, addr: u32, val: u32) {
        match addr & 0xFFFF {
            0xE010 => {
                // CSR: only ENABLE / TICKINT / CLKSOURCE writable;
                // COUNTFLAG (bit 16) untouched on write.
                let preserved = self.csr & (1 << 16);
                self.csr = (val & 0x7) | preserved;
            }
            0xE014 => {
                // RVR: 24-bit reload register. Writes to high bits
                // are truncated.
                self.rvr = val & 0x00FF_FFFF;
            }
            0xE018 => {
                // CVR: any write zeroes CVR *and* clears
                // COUNTFLAG. The write data is ignored.
                let _ = val;
                self.cvr = 0;
                self.csr &= !(1 << 16);
            }
            // CALIB / unknown — RAZ/WI.
            _ => {}
        }
    }

    /// Tick once per master cycle. Returns `true` iff this tick
    /// generated a SysTick exception (caller sets
    /// `ICSR.PENDSTSET` (bit 26) on the active-core PPB).
    ///
    /// Per ARMv6-M ARM §B3.3.1: when the counter reaches 0, on the
    /// *next* clock cycle the value is reloaded from RVR and (if
    /// `TICKINT` is set) the SysTick exception is asserted. Period is
    /// therefore `RVR + 1` cycles; with `CVR = 0` after firmware's
    /// clearing write, the very first tick fires.
    pub fn tick(&mut self) -> bool {
        if !self.is_enabled() {
            return false;
        }
        if self.cvr == 0 {
            // Reload + (optional) interrupt assert.
            self.cvr = self.rvr & 0x00FF_FFFF;
            self.csr |= 1 << 16; // COUNTFLAG
            return self.csr & 2 != 0; // TICKINT
        }
        self.cvr -= 1;
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: enable SysTick with TICKINT and CLKSOURCE=processor
    /// (`CSR = 0b111`), with `RVR = rvr` and `CVR = 0`.
    fn arm(s: &mut SysTick, rvr: u32) {
        s.write32(0xE000_E014, rvr); // RVR
        s.write32(0xE000_E018, 0xDEAD_BEEF); // CVR (any write zeroes)
        s.write32(0xE000_E010, 0b111); // CSR: ENABLE | TICKINT | CLKSOURCE
    }

    #[test]
    fn enable_then_underflow_sets_countflag_and_returns_tickint() {
        let mut s = SysTick::new();
        arm(&mut s, 0); // RVR=0 — period 1, fire every tick
        let fired = s.tick();
        assert!(fired, "TICKINT-armed first tick must fire");
        // COUNTFLAG must have latched.
        assert_eq!(s.csr & (1 << 16), 1 << 16);
    }

    #[test]
    fn disable_during_countdown_freezes_cvr() {
        let mut s = SysTick::new();
        arm(&mut s, 10);
        // First tick: CVR was 0, reload+fire. CVR is now 10.
        assert!(s.tick());
        assert_eq!(s.cvr, 10);
        // Two countdown ticks: 10 -> 9 -> 8.
        assert!(!s.tick());
        assert!(!s.tick());
        assert_eq!(s.cvr, 8);
        // Disable mid-countdown.
        s.write32(0xE000_E010, 0b110); // clear ENABLE, keep TICKINT|CLKSOURCE
        // Further ticks must not change CVR or fire.
        let frozen = s.cvr;
        for _ in 0..5 {
            assert!(!s.tick());
            assert_eq!(s.cvr, frozen);
        }
    }

    #[test]
    fn cvr_write_clears_to_zero_and_clears_countflag() {
        let mut s = SysTick::new();
        arm(&mut s, 4);
        // Fire once to set COUNTFLAG.
        assert!(s.tick());
        assert_ne!(s.csr & (1 << 16), 0, "COUNTFLAG must be set after fire");
        // Any write to CVR zeroes it and clears COUNTFLAG.
        s.write32(0xE000_E018, 0x12345);
        assert_eq!(s.cvr, 0);
        assert_eq!(s.csr & (1 << 16), 0, "CVR write must clear COUNTFLAG");
    }

    #[test]
    fn rvr_write_truncated_to_24_bits() {
        let mut s = SysTick::new();
        s.write32(0xE000_E014, 0xFFFF_FFFF);
        assert_eq!(s.read32(0xE000_E014), 0x00FF_FFFF);
    }

    #[test]
    fn csr_read_clears_countflag() {
        let mut s = SysTick::new();
        arm(&mut s, 0);
        assert!(s.tick(), "first tick fires");
        // First CSR read returns COUNTFLAG=1.
        let v1 = s.read32(0xE000_E010);
        assert_ne!(v1 & (1 << 16), 0, "first read sees COUNTFLAG=1");
        // Second CSR read returns COUNTFLAG=0 (read-clear).
        let v2 = s.read32(0xE000_E010);
        assert_eq!(v2 & (1 << 16), 0, "second read sees COUNTFLAG=0");
    }

    #[test]
    fn period_is_rvr_plus_one_cycles() {
        // Preamble: CVR=0, RVR=4, ENABLE+TICKINT+CLKSOURCE.
        // Tick 10 times. Per ARMv6-M ARM §B3.3.1, period = RVR+1 = 5
        // cycles. With CVR pre-cleared, fires at tick 1 and tick 6.
        let mut s = SysTick::new();
        arm(&mut s, 4);
        let mut fires = Vec::new();
        for i in 1..=10 {
            if s.tick() {
                fires.push(i);
            }
        }
        assert_eq!(fires, vec![1, 6], "fires at t=1 and t=6 (period 5)");
    }
}
