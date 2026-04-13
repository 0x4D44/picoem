// QEMU differential test harness — foundation types and test generation.
//
// Validates Thumb-2 instruction semantics by executing identical instructions
// in both QEMU (Cortex-M33 model) and our emulator, then diffing state.

// Re-export emulator types the harness needs.
pub use mdrp2354::{Bus, CortexM33};

// ============================================================================
// Address constants — QEMU side (MPS2-AN505 ssram-0)
// ============================================================================

/// QEMU: instruction slot in ssram-0.
pub const QEMU_TEST_SLOT: u32 = 0x0000_0100;
/// QEMU: stack pointer for push/pop/load/store tests.
pub const QEMU_TEST_STACK: u32 = 0x0004_0000;
/// QEMU: scratch SRAM for load/store data.
pub const QEMU_TEST_SCRATCH: u32 = 0x0000_0200;

// ============================================================================
// Address constants — Emulator side (our SRAM address space)
// ============================================================================

/// Emulator: instruction slot in SRAM.
pub const EMU_TEST_SLOT: u32 = 0x2000_0100;
/// Emulator: stack pointer.
pub const EMU_TEST_STACK: u32 = 0x2004_0000;
/// Emulator: scratch SRAM.
pub const EMU_TEST_SCRATCH: u32 = 0x2000_0200;

// ============================================================================
// GDB register indices (stable across QEMU >= 7.0)
// ============================================================================

/// R0-R12 are indices 0-12.
pub const REG_R0: u8 = 0;
pub const REG_SP: u8 = 13;
pub const REG_LR: u8 = 14;
pub const REG_PC: u8 = 15;
/// Indices 16-24 are legacy FPA (unused). xPSR is at index 25.
pub const REG_XPSR: u8 = 25;

// ============================================================================
// xPSR comparison masks
// ============================================================================

/// N, Z, C, V, Q — all condition flags.
pub const MASK_ALL_FLAGS: u32 = 0xF800_0000;
/// N, Z only — for MUL where C and V are UNPREDICTABLE.
pub const MASK_NZ_ONLY: u32 = 0xC000_0000;
/// No flags — for MOV/ADD (high register) which don't update flags.
pub const MASK_NO_FLAGS: u32 = 0x0000_0000;

// ============================================================================
// Test case model
// ============================================================================

/// A single differential test case: one instruction with preconditions.
pub struct TestCase {
    /// Human-readable name (e.g., "ADDS R0, R1, R2 (overflow)").
    pub name: String,
    /// Instruction opcode (16-bit for Phase A).
    pub opcode: u16,
    /// Register preconditions: (index, value). Unset registers default to 0.
    pub reg_pre: Vec<(u8, u32)>,
    /// xPSR precondition. Default: 0x01000000 (T bit set, flags clear).
    pub xpsr_pre: u32,
    /// Whether this instruction accesses memory (use execute_one_with_bus).
    pub needs_bus: bool,
    /// Registers whose values are addresses (offsets from scratch base).
    /// The runner translates these by adding the per-side TEST_SCRATCH base.
    pub addr_regs: Vec<u8>,
    /// Memory preconditions as offsets from scratch area.
    /// Written to QEMU_TEST_SCRATCH+offset and EMU_TEST_SCRATCH+offset.
    pub mem_pre: Vec<(u32, u8)>,
    /// Memory offsets to compare after execution.
    pub mem_check: Vec<u32>,
    /// xPSR flag mask for comparison. Default: MASK_ALL_FLAGS.
    pub xpsr_mask: u32,
}

impl Default for TestCase {
    fn default() -> Self {
        Self {
            name: String::new(),
            opcode: 0,
            reg_pre: Vec::new(),
            xpsr_pre: 0x0100_0000, // T bit set, flags clear
            needs_bus: false,
            addr_regs: Vec::new(),
            mem_pre: Vec::new(),
            mem_check: Vec::new(),
            xpsr_mask: MASK_ALL_FLAGS,
        }
    }
}

/// Generate all test cases. Placeholder — returns empty Vec until generators
/// are implemented in Stage 3.
pub fn generate_all() -> Vec<TestCase> {
    Vec::new()
}

// ============================================================================
// Unit tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- TestCase::default() --

    #[test]
    fn default_xpsr_has_thumb_bit() {
        let tc = TestCase::default();
        assert_eq!(tc.xpsr_pre, 0x0100_0000, "T bit must be set");
    }

    #[test]
    fn default_mask_is_all_flags() {
        let tc = TestCase::default();
        assert_eq!(tc.xpsr_mask, MASK_ALL_FLAGS);
    }

    #[test]
    fn default_no_bus() {
        let tc = TestCase::default();
        assert!(!tc.needs_bus);
    }

    #[test]
    fn default_empty_preconditions() {
        let tc = TestCase::default();
        assert!(tc.reg_pre.is_empty());
        assert!(tc.addr_regs.is_empty());
        assert!(tc.mem_pre.is_empty());
        assert!(tc.mem_check.is_empty());
    }

    // -- Mask constants --

    #[test]
    fn mask_all_flags_covers_nzcvq() {
        // N=bit31, Z=bit30, C=bit29, V=bit28, Q=bit27
        assert_eq!(MASK_ALL_FLAGS, 0xF800_0000);
        assert_ne!(MASK_ALL_FLAGS & (1 << 31), 0, "N bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 30), 0, "Z bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 29), 0, "C bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 28), 0, "V bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 27), 0, "Q bit");
    }

    #[test]
    fn mask_nz_only_covers_nz() {
        assert_eq!(MASK_NZ_ONLY, 0xC000_0000);
        assert_ne!(MASK_NZ_ONLY & (1 << 31), 0, "N bit");
        assert_ne!(MASK_NZ_ONLY & (1 << 30), 0, "Z bit");
        assert_eq!(MASK_NZ_ONLY & (1 << 29), 0, "C bit excluded");
        assert_eq!(MASK_NZ_ONLY & (1 << 28), 0, "V bit excluded");
        assert_eq!(MASK_NZ_ONLY & (1 << 27), 0, "Q bit excluded");
    }

    #[test]
    fn mask_no_flags_is_zero() {
        assert_eq!(MASK_NO_FLAGS, 0);
    }

    // -- Address constants --

    #[test]
    fn qemu_addresses_non_overlapping() {
        // TEST_SLOT at 0x100, scratch at 0x200. Slot occupies at most a few
        // bytes; scratch is 256 bytes starting at 0x200.
        assert!(QEMU_TEST_SLOT < QEMU_TEST_SCRATCH);
        // Stack is above both.
        assert!(QEMU_TEST_STACK > QEMU_TEST_SCRATCH);
    }

    #[test]
    fn emu_addresses_non_overlapping() {
        assert!(EMU_TEST_SLOT < EMU_TEST_SCRATCH);
        assert!(EMU_TEST_STACK > EMU_TEST_SCRATCH);
    }

    #[test]
    fn qemu_addresses_correct() {
        assert_eq!(QEMU_TEST_SLOT, 0x0000_0100);
        assert_eq!(QEMU_TEST_STACK, 0x0004_0000);
        assert_eq!(QEMU_TEST_SCRATCH, 0x0000_0200);
    }

    #[test]
    fn emu_addresses_correct() {
        assert_eq!(EMU_TEST_SLOT, 0x2000_0100);
        assert_eq!(EMU_TEST_STACK, 0x2004_0000);
        assert_eq!(EMU_TEST_SCRATCH, 0x2000_0200);
    }

    #[test]
    fn emu_addresses_in_sram() {
        // Emulator SRAM starts at 0x2000_0000.
        assert!(EMU_TEST_SLOT >= 0x2000_0000);
        assert!(EMU_TEST_STACK >= 0x2000_0000);
        assert!(EMU_TEST_SCRATCH >= 0x2000_0000);
    }

    #[test]
    fn slot_scratch_separation() {
        // Scratch must start past the test slot to avoid instruction/data
        // overlap. Slot at 0x100, scratch at 0x200 — 256 bytes of space for
        // instructions + BKPT.
        assert_eq!(QEMU_TEST_SCRATCH - QEMU_TEST_SLOT, 0x100);
        assert_eq!(EMU_TEST_SCRATCH - EMU_TEST_SLOT, 0x100);
    }

    // -- GDB register indices --

    #[test]
    fn reg_indices_correct() {
        assert_eq!(REG_R0, 0);
        assert_eq!(REG_SP, 13);
        assert_eq!(REG_LR, 14);
        assert_eq!(REG_PC, 15);
        assert_eq!(REG_XPSR, 25);
    }

    // -- generate_all() placeholder --

    #[test]
    fn generate_all_returns_empty() {
        assert!(generate_all().is_empty());
    }
}
