//! Exclusive-monitor tracking for LDREX/STREX on shared SRAM.
//!
//! Each core owns one monitor slot (an `AtomicU64`). The low bit is a
//! valid flag; bits [31:2] hold the word-aligned SRAM address. The bus
//! layer calls [`ExclusiveMonitors::snoop`] on every SRAM write to
//! invalidate any monitor that matches the written word address.
//!
//! All operations use `Relaxed` ordering — correctness relies on the
//! global step lock that serialises instruction execution, not on
//! store-buffer visibility between hardware threads.

use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

/// Bit 0 marks a monitor slot as valid.
const VALID: u64 = 1;

/// Pack a word-aligned address + valid bit into a single u64 tag.
fn encode(addr: u32) -> u64 {
    ((addr & !3) as u64) | VALID
}

/// Per-core exclusive-monitor state for LDREX/STREX.
///
/// Shared (typically behind an `Arc`) between the two core threads and
/// the bus snoop path.
pub struct ExclusiveMonitors {
    monitors: [AtomicU64; 2],
}

impl ExclusiveMonitors {
    pub fn new() -> Self {
        Self {
            monitors: [AtomicU64::new(0), AtomicU64::new(0)],
        }
    }

    /// Mark a reservation. `addr` should be a SRAM bus address;
    /// encoding normalizes to word-aligned offset.
    pub fn set(&self, core: usize, addr: u32) {
        debug_assert!(core < 2);
        self.monitors[core].store(encode(addr), Relaxed);
    }

    /// Clear a reservation (CLREX, exception entry/exit, STREX).
    pub fn clear(&self, core: usize) {
        debug_assert!(core < 2);
        self.monitors[core].store(0, Relaxed);
    }

    /// Check if a reservation is still valid for this address.
    pub fn check(&self, core: usize, addr: u32) -> bool {
        debug_assert!(core < 2);
        self.monitors[core].load(Relaxed) == encode(addr)
    }

    /// Returns true if any core has an active reservation.
    fn any_valid(&self) -> bool {
        self.monitors[0].load(Relaxed) & VALID != 0 || self.monitors[1].load(Relaxed) & VALID != 0
    }

    /// Snoop: invalidate any monitor matching this word address.
    /// Called by the bus layer on every SRAM write.
    pub fn snoop(&self, addr: u32) {
        if !self.any_valid() {
            return;
        }
        let tag = encode(addr);
        for m in &self.monitors {
            let _ = m.compare_exchange(tag, 0, Relaxed, Relaxed);
        }
    }
}

impl Default for ExclusiveMonitors {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_check_clear() {
        let mon = ExclusiveMonitors::new();
        let addr = 0x2000_1000;

        // Initially no reservation.
        assert!(!mon.check(0, addr));

        // Set → check succeeds.
        mon.set(0, addr);
        assert!(mon.check(0, addr));

        // Clear → check fails.
        mon.clear(0);
        assert!(!mon.check(0, addr));
    }

    #[test]
    fn snoop_invalidates() {
        let mon = ExclusiveMonitors::new();
        let addr = 0x2000_2000;

        mon.set(0, addr);
        assert!(mon.check(0, addr));

        mon.snoop(addr);
        assert!(!mon.check(0, addr));
    }

    #[test]
    fn snoop_wrong_addr_no_effect() {
        let mon = ExclusiveMonitors::new();
        let addr_a = 0x2000_3000;
        let addr_b = 0x2000_4000;

        mon.set(0, addr_a);
        mon.snoop(addr_b);
        assert!(mon.check(0, addr_a));
    }

    #[test]
    fn snoop_clears_both_cores() {
        let mon = ExclusiveMonitors::new();
        let addr = 0x2000_5000;

        mon.set(0, addr);
        mon.set(1, addr);
        assert!(mon.check(0, addr));
        assert!(mon.check(1, addr));

        mon.snoop(addr);
        assert!(!mon.check(0, addr));
        assert!(!mon.check(1, addr));
    }

    #[test]
    fn clear_is_independent() {
        let mon = ExclusiveMonitors::new();
        let addr = 0x2000_6000;

        mon.set(0, addr);
        mon.set(1, addr);

        mon.clear(0);
        assert!(!mon.check(0, addr));
        assert!(mon.check(1, addr)); // core 1 unaffected
    }

    #[test]
    fn any_valid_fast_path() {
        let mon = ExclusiveMonitors::new();
        let addr = 0x2000_7000;

        // No monitors set — snoop should be a no-op (fast path).
        // We verify indirectly: set on core 0, clear it, then snoop
        // a different address. If any_valid were broken and returned
        // true after clear, the CAS would still miss (wrong tag), so
        // correctness holds either way — but the fast path means zero
        // CAS operations.
        mon.set(0, addr);
        mon.clear(0);
        mon.set(1, addr);
        mon.clear(1);

        // Both cleared — any_valid should be false.
        // Snoop with a live address should be a no-op.
        let probe_addr = 0x2000_8000;
        mon.set(0, probe_addr);
        mon.clear(0);
        mon.snoop(probe_addr);
        // Nothing to assert beyond "didn't panic"; the real
        // verification is that snoop returns early before touching
        // the CAS path.  We can at least confirm no monitor is set.
        assert!(!mon.check(0, probe_addr));
        assert!(!mon.check(1, probe_addr));
    }

    #[test]
    fn word_aligned_encoding() {
        let mon = ExclusiveMonitors::new();

        // Set with a misaligned address (byte 1 within a word).
        mon.set(0, 0x2000_0001);

        // Check with the word-aligned base — should match because
        // both normalize to the same word (bits [1:0] masked off).
        assert!(mon.check(0, 0x2000_0000));
        assert!(mon.check(0, 0x2000_0001));
        assert!(mon.check(0, 0x2000_0002));
        assert!(mon.check(0, 0x2000_0003));

        // Next word should NOT match.
        assert!(!mon.check(0, 0x2000_0004));
    }
}
