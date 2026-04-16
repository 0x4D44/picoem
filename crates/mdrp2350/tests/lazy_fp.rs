//! Phase 7 Stage B — Lazy FP context save integration tests.
//!
//! These exercise the exception-entry/exit FP frame logic against the
//! Armv8-M architecture spec (DDI0553 §B3.4.3-4 / §D1.2.32-34) and the
//! HLD invariants in `wrk_docs/2026.04.14 - HLD - Phase 7 Coprocessors
//! and FPU.md` §B.9.
//!
//! The CPU and PPB are public; integration tests poke FPCCR/FPCAR/FPDSCR
//! through PPB MMIO and call `test_enter_exception` / `test_exit_exception`
//! to simulate the architecturally-defined paths.

use mdrp2350::{Bus, CortexM33};

// ---------------------------------------------------------------------------
// FPCCR bit positions — duplicated locally to avoid cross-crate coupling.
// Identical to the crate-internal constants in `bus/ppb.rs`.
const FPCCR_LSPACT:    u32 = 1 << 0;
const FPCCR_MMRDY:     u32 = 1 << 5;
const FPCCR_BFRDY:     u32 = 1 << 6;
const FPCCR_SPLIMVIOL: u32 = 1 << 9;
const FPCCR_LSPEN:     u32 = 1 << 30;
const FPCCR_ASPEN:     u32 = 1 << 31;

const CONTROL_FPCA: u32 = 1 << 2;

const VT_BASE: u32 = 0x2000_4000;
const HANDLER_ADDR: u32 = 0x2000_4200;
const HANDLER_VEC: u32 = HANDLER_ADDR | 1;

/// Build a CPU + Bus with: MSP at 0x2000_2000, vector table at VT_BASE
/// (NMI/HardFault/MemManage/UsageFault/SVC/PendSV/SysTick all pointing to
/// HANDLER_ADDR). UsageFault/MemManage/BusFault are enabled in SHCSR by
/// default (real silicon resets them disabled, but tests want to observe
/// the non-escalated fault paths).
fn fixture() -> (CortexM33, Bus) {
    let mut cpu = CortexM33::new();
    cpu.regs.msp = 0x2000_2000;
    cpu.regs.r[13] = cpu.regs.msp;

    let mut bus = Bus::default();
    // Phase 0b.1 Commit B: per-core PPB (VTOR / SHCSR / FPCCR etc.) now
    // lives on CortexM33, not Bus.
    cpu.ppb.vtor = VT_BASE;
    // Vectors 2 (NMI), 3 (HardFault), 4 (MemManage), 5 (BusFault),
    // 6 (UsageFault), 11 (SVC), 14 (PendSV), 15 (SysTick).
    for &exc in &[2u32, 3, 4, 5, 6, 11, 14, 15] {
        bus.write32(VT_BASE + exc * 4, HANDLER_VEC, 0);
    }
    // Enable MEMFAULTENA (16), BUSFAULTENA (17), USGFAULTENA (18).
    cpu.ppb.shcsr |= (1 << 16) | (1 << 17) | (1 << 18);

    (cpu, bus)
}

/// Encode VADD.F32 Sd, Sn, Sm.
fn enc_vadd(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (0 << 7) | (d << 6) | (0b11 << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (0 << 6) | (m << 5) | vm;
    (hw0, hw1)
}

// ===========================================================================
// Test 1 — No-FP handler (HLD §B.9 #1)
//
// Thread mode without prior FP activity: CONTROL.FPCA=0. Take an exception,
// observe FType=1 (no FP frame), no FP-region writes, FPCA stays 0 across
// entry. On return, S0-S15 untouched and FPCA still 0.
// ===========================================================================
#[test]
fn no_fp_handler_no_fp_frame_writes() {
    let (mut cpu, mut bus) = fixture();
    assert_eq!(cpu.regs.control & CONTROL_FPCA, 0, "FPCA must start clear");

    // Seed S0..S15 with a known pattern so we can assert they aren't
    // disturbed by entry.
    for i in 0..16 {
        cpu.regs.s[i] = f32::from_bits(0xCAFE_0000 + i as u32);
    }

    let pre_msp = cpu.regs.msp;
    cpu.test_enter_exception(11, &mut bus); // SVC
    let post_msp = cpu.regs.msp;
    

    // Frame is 32 bytes (basic only) — no FP region.
    assert_eq!(pre_msp - post_msp, 32, "no-FP frame is exactly 32 bytes");
    // EXC_RETURN[4]=1 (FType=1, no FP frame).
    assert_eq!(cpu.regs.r[14], 0xFFFF_FFF9, "FType=1 EXC_RETURN");
    assert_eq!(cpu.regs.control & CONTROL_FPCA, 0, "FPCA stays clear");
    // No FP region was written, so LSPACT must be clear and FPCAR
    // never set.
    assert_eq!(cpu.ppb.fpccr & FPCCR_LSPACT, 0);
    assert_eq!(cpu.ppb.fpcar, 0);
    // S0..S15 unchanged.
    for i in 0..16 {
        assert_eq!(cpu.regs.s[i].to_bits(), 0xCAFE_0000 + i as u32);
    }

    // Return.
    cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, 0);
    for i in 0..16 {
        assert_eq!(cpu.regs.s[i].to_bits(), 0xCAFE_0000 + i as u32);
    }
}

// ===========================================================================
// Test 2 — FP handler with lazy flush (HLD §B.9 #2)
// ===========================================================================
#[test]
fn fp_handler_lazy_flush_then_return() {
    let (mut cpu, mut bus) = fixture();

    // Set CONTROL.FPCA=1 by executing a VFP op (only legal writer).
    cpu.regs.s[2] = 1.5;
    cpu.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA, "FPU op sets FPCA");
    assert_eq!(cpu.regs.s[0], 4.0);

    // Seed S0..S15 with a known pre-exception pattern.
    let mut pre_s = [0.0f32; 16];
    for i in 0..16 {
        pre_s[i] = f32::from_bits(0xBEEF_0000 + i as u32);
        cpu.regs.s[i] = pre_s[i];
    }
    cpu.regs.fpscr = 0x4000_0000; // arbitrary marker

    let pre_msp = cpu.regs.msp;
    cpu.test_enter_exception(11, &mut bus);
    

    // Frame is 32 + 72 = 104 bytes.
    assert_eq!(pre_msp - cpu.regs.msp, 104);
    // FType=0 EXC_RETURN.
    assert_eq!(cpu.regs.r[14], 0xFFFF_FFE9);
    // Lazy: LSPACT set, FPCAR points at the FP region (basic frame + 32).
    assert_eq!(cpu.ppb.fpccr & FPCCR_LSPACT, FPCCR_LSPACT);
    assert_eq!(cpu.ppb.fpcar, cpu.regs.msp + 32);
    // FPCA cleared by entry.
    assert_eq!(cpu.regs.control & CONTROL_FPCA, 0);

    // Now execute an FP op in handler mode — should trigger the lazy
    // flush of S0-S15 + FPSCR, clear LSPACT, and set FPCA=1 again.
    let fpcar = cpu.ppb.fpcar;
    cpu.regs.s[10] = 99.0; // sentinel — will overwrite S10 post-flush
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "LSPACT cleared");
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA, "FPCA set");

    // The flushed frame should hold the *pre-exception* S registers.
    // We seeded S10 with a sentinel after entry; the flush must capture
    // the live S0..S15 at the time of the flush — which now includes the
    // 99.0 sentinel. So check S10 == 99.0 in the flushed slot AFTER the
    // ADD overwrote S0 and S10 was poked.
    assert_eq!(bus.read32(fpcar + 10 * 4, 0), (99.0f32).to_bits());

    // Return — LSPACT=0 now, so the pop path runs and restores S0..S15
    // from the frame. (The pre-exception values are gone — they were
    // overwritten by the flush of the in-handler register file.)
    cpu.test_exit_exception(0xFFFF_FFE9, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA, "FPCA restored");
    assert_eq!(cpu.regs.msp, pre_msp);
}

// ===========================================================================
// Test 3 — Nested interrupts with FP activity at both levels (HLD §B.9 #3)
//
// Exercises LSPACT lifecycle across two nested exception frames where each
// level actually reserves + flushes its own FP region. This is stricter
// than the naive nesting test (which would let the inner entry run with
// FPCA=0 and therefore never exercise nested LSPACT / FPCAR reservations).
// ===========================================================================
#[test]
fn nested_interrupts_capture_fpca_independently() {
    let (mut cpu, mut bus) = fixture();
    

    // Light up FPCA via an FPU op.
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.regs.s[2] = 1.0;
    cpu.regs.s[4] = 2.0;
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA);

    // Outer exception — entry captures FPCA into EXC_RETURN[4]=0, reserves
    // an FP frame (LSPACT=1), and clears FPCA.
    let outer_msp_pre = cpu.regs.msp;
    cpu.test_enter_exception(11, &mut bus);
    assert_eq!(cpu.regs.r[14], 0xFFFF_FFE9, "outer FType=0");
    assert_eq!(cpu.regs.control & CONTROL_FPCA, 0);
    let outer_lr = cpu.regs.r[14];
    let outer_fpcar = cpu.ppb.fpcar;
    assert_ne!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "outer lazy reserve");

    // Execute a VFP op in the outer handler. This flushes the outer lazy
    // reservation (LSPACT→0) and sets FPCA=1 in outer-handler context,
    // so the nested inner entry will also reserve an FP region.
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "outer LSPACT flushed");
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA,
        "outer handler now FP-active");

    // Inner exception — taken with FPCA=1, so inner entry reserves its
    // OWN FP frame (wider frame, FType=0 EXC_RETURN, fresh LSPACT).
    let inner_msp_pre = cpu.regs.msp;
    cpu.test_enter_exception(14, &mut bus); // PendSV
    assert_eq!(cpu.regs.r[14], 0xFFFF_FFE1,
        "inner FType=0, return-handler (MSP)");
    assert_eq!(inner_msp_pre - cpu.regs.msp, 104,
        "inner reserves basic + FP region (104B)");
    assert_ne!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "inner lazy reserve");
    let inner_fpcar = cpu.ppb.fpcar;
    assert_ne!(inner_fpcar, outer_fpcar,
        "inner FPCAR is distinct from outer FPCAR");
    assert_eq!(cpu.regs.control & CONTROL_FPCA, 0, "FPCA cleared on inner entry");

    // Execute a VFP op in the inner handler — flushes the inner lazy
    // reservation and sets FPCA=1 in inner-handler context.
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "inner LSPACT flushed");
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA,
        "inner handler FP-active");

    // Return from inner (FType=0). Inner's FP frame is popped; FPCA
    // restored to 1 by the FType=0 rule.
    cpu.test_exit_exception(cpu.regs.r[14], &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA,
        "inner return restores FPCA=1");
    assert_eq!(cpu.regs.msp, inner_msp_pre, "inner pop restores SP");
    assert_eq!(cpu.regs.r[14], outer_lr, "inner pop restored outer LR");

    // Return from outer (FType=0). Since the outer handler already
    // flushed its lazy reservation earlier, LSPACT=0 here and the FP
    // frame pop restores S0-S15 + FPSCR from memory. FPCA restored to 1.
    cpu.test_exit_exception(outer_lr, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA,
        "outer return restores pre-outer FPCA=1");
    assert_eq!(cpu.regs.msp, outer_msp_pre, "outer pop restores SP");
}

// ===========================================================================
// Test 4 — Eager mode (HLD §B.9 #4)
// ===========================================================================
#[test]
fn eager_mode_writes_fp_frame_on_entry() {
    let (mut cpu, mut bus) = fixture();
    // Disable lazy stacking: clear FPCCR.LSPEN.
    
    cpu.ppb.fpccr &= !FPCCR_LSPEN;

    // Light up FPCA.
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.regs.s[2] = 1.0;
    cpu.regs.s[4] = 2.0;
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA);

    // Seed S0..S15 with a recognizable pattern.
    for i in 0..16 {
        cpu.regs.s[i] = f32::from_bits(0xFACE_0000 + i as u32);
    }
    cpu.regs.fpscr = 0xC000_0000;

    cpu.test_enter_exception(11, &mut bus);

    // LSPACT must be clear (eager wrote the frame already).
    assert_eq!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "eager: LSPACT clear");
    // FPCAR was still recorded.
    let fpcar = cpu.ppb.fpcar;
    assert_eq!(fpcar, cpu.regs.msp + 32);

    // Verify each saved word.
    for i in 0..16 {
        let expected = 0xFACE_0000 + i as u32;
        assert_eq!(bus.read32(fpcar + (i as u32) * 4, 0), expected,
            "S{} eager save mismatch", i);
    }
    assert_eq!(bus.read32(fpcar + 64, 0), 0xC000_0000, "FPSCR eager save");
    assert_eq!(bus.read32(fpcar + 68, 0), 0, "reserved word");
}

// ===========================================================================
// Test 5 — Stack-limit during FP reserve raises UsageFault (HLD §B.9 #5)
//
// Per DDI0553 §D1.2.32, FPCCR.SPLIMVIOL is set *only* when the FP region
// specifically caused the underflow — i.e. the basic frame alone would
// have fit. When the basic frame already underflows, SPLIMVIOL stays
// clear (the violation is not attributable to FP context) but UFSR.STKOF
// is set in both cases. This pair of tests pins the distinction.
// ===========================================================================

/// Basic frame alone already underflows → UFSR.STKOF set, SPLIMVIOL clear.
/// The FP region didn't cause this — the basic 32-byte frame was already
/// impossible.
#[test]
fn stack_limit_basic_frame_underflow_no_splimviol() {
    let (mut cpu, mut bus) = fixture();
    // Light up FPCA so the entry path *would* reserve an FP region.
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.regs.s[2] = 1.0;
    cpu.regs.s[4] = 2.0;
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    // Set MSPLIM such that even the basic frame (32B) underflows.
    // Pick 16 bytes of headroom (< 32).
    cpu.regs.msplim = cpu.regs.msp - 16;

    cpu.test_enter_exception(11, &mut bus);
    

    // SPLIMVIOL must be CLEAR — the basic frame alone underflowed, so
    // the FP region isn't the cause.
    assert_eq!(cpu.ppb.fpccr & FPCCR_SPLIMVIOL, 0,
        "SPLIMVIOL must stay clear when basic frame alone underflows");
    // CFSR.STKOF must be set regardless.
    assert_ne!(cpu.ppb.cfsr & (1 << 20), 0, "CFSR.STKOF must be set");
    // UsageFault is pending.
    assert!(cpu.has_pending_fault(), "UsageFault must be pending");
}

/// Basic frame fits, FP region causes the underflow → SPLIMVIOL set.
#[test]
fn stack_limit_fp_region_underflow_sets_splimviol() {
    let (mut cpu, mut bus) = fixture();
    // Light up FPCA so the entry path reserves an FP region.
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.regs.s[2] = 1.0;
    cpu.regs.s[4] = 2.0;
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    // Set MSPLIM such that the basic frame fits but adding the FP region
    // (72 bytes) underflows past the limit. Basic frame is 32B; pick a
    // limit that leaves 40 bytes of headroom (between 32 and 32+72).
    cpu.regs.msplim = cpu.regs.msp - 40;

    cpu.test_enter_exception(11, &mut bus);
    

    // SPLIMVIOL set on FPCCR — FP region specifically drove the violation.
    assert_ne!(cpu.ppb.fpccr & FPCCR_SPLIMVIOL, 0,
        "SPLIMVIOL must be set when basic fits but FP region underflows");
    // CFSR.STKOF (bit 20) set.
    assert_ne!(cpu.ppb.cfsr & (1 << 20), 0, "CFSR.STKOF must be set");
    // UsageFault is pending.
    assert!(cpu.has_pending_fault(), "UsageFault must be pending");
}

// ===========================================================================
// Test 6 — Bus fault during lazy flush (HLD §B.9 #6 — adapted)
//
// The HLD originally specified MPU fault (MMRDY); the emulator's MPU is
// not enforced on data writes (Stage E work), so we exercise the bus-fault
// equivalent (BFRDY). Both share the same flush-abort + LSPACT-retained
// invariant, so this verifies the structural contract.
//
// TODO(Phase 7 Stage E): once MPU is enforced on data writes, add a
// symmetric `mpu_fault_during_lazy_flush_sets_mmrdy_keeps_lspact` test
// that points FPCAR at an MPU-protected region and asserts FPCCR.MMRDY
// (not BFRDY) is set and IPSR ends up in MemManage (4).
// ===========================================================================
#[test]
fn bus_fault_during_lazy_flush_delivers_busfault() {
    let (mut cpu, mut bus) = fixture();

    // Light up FPCA.
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.regs.s[2] = 1.0;
    cpu.regs.s[4] = 2.0;
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    // Take an exception — entry reserves the lazy frame on MSP, sets
    // LSPACT, sets FPCAR.
    cpu.test_enter_exception(11, &mut bus);
    
    assert_ne!(cpu.ppb.fpccr & FPCCR_LSPACT, 0);

    // Force the flush target to an unmapped address (region 0xB has no
    // backing). Manually overwrite FPCAR to point there.
    cpu.ppb.fpcar = 0xB000_0000;

    // Write a VADD.F32 at the handler PC so step() will decode+execute it,
    // triggering the lazy flush + bus fault through the normal dispatch
    // path. This exercises the end-to-end fault-delivery coupling where
    // the flush sets bus.bus_fault, step() picks it up, and enters
    // exception 5 (BusFault).
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    bus.write32(HANDLER_ADDR, ((hw1 as u32) << 16) | hw0 as u32, 0);
    assert_eq!(cpu.regs.pc(), HANDLER_ADDR, "PC sits at handler after entry");

    // One step: decode fetches VADD, fpu_execute runs the lazy flush which
    // bus-faults, step() observes bus.bus_fault and delivers BusFault.
    cpu.step(&mut bus);

    // BFRDY must be set on the flush.
    assert_ne!(cpu.ppb.fpccr & FPCCR_BFRDY, 0,
        "FPCCR.BFRDY must be set on flush bus fault");
    // LSPACT must be retained so the next attempt sees the unflushed
    // state.
    assert_ne!(cpu.ppb.fpccr & FPCCR_LSPACT, 0,
        "LSPACT must remain set after bus-faulting flush");
    // MMRDY must be clear (we reached BusFault, not MemManage).
    assert_eq!(cpu.ppb.fpccr & FPCCR_MMRDY, 0);

    // End-to-end: the BusFault exception (vector 5) must have been taken.
    assert_eq!(cpu.regs.ipsr(), 5,
        "IPSR must be BusFault (5) after step() delivers the fault");
    // BusFault-specific CFSR bits: PRECISERR (bit 9) and BFARVALID (bit 15).
    assert_ne!(cpu.ppb.cfsr & (1 << 9), 0, "CFSR.BFSR.PRECISERR set");
    assert_ne!(cpu.ppb.cfsr & (1 << 15), 0, "CFSR.BFSR.BFARVALID set");
    // BFAR should record the faulting address (the first S0 write target).
    assert_eq!(cpu.ppb.bfar, 0xB000_0000, "BFAR points at flush target");
}

// ===========================================================================
// Test 7 — EXC_RETURN[4]=0 with no FP context raises UsageFault (HLD §B.9 #7)
// ===========================================================================
#[test]
fn fabricated_fp_exc_return_raises_usage_fault() {
    let (mut cpu, mut bus) = fixture();
    // Take a non-FP exception so we enter handler mode with a
    // well-formed FType=1 EXC_RETURN.
    cpu.test_enter_exception(11, &mut bus);
    assert_eq!(cpu.regs.r[14], 0xFFFF_FFF9);

    // Now fabricate an EXC_RETURN that *claims* an FP frame is present,
    // but no FP activity has occurred since reset (FPCAR=0, LSPACT=0).
    // The integrity check at exit must reject this.
    cpu.test_exit_exception(0xFFFF_FFE9, &mut bus);

    
    // CFSR.UFSR.INVPC (bit 17).
    assert_ne!(cpu.ppb.cfsr & (1 << 17), 0,
        "CFSR.UFSR.INVPC must be set on bogus FType=0 EXC_RETURN");
    assert!(cpu.has_pending_fault(), "UsageFault must be pending");
}

// ===========================================================================
// Test 7b — FType=1 EXC_RETURN with LSPACT=1 (devil's-advocate coverage)
//
// DDI0553 §B3.4.4 ExceptionReturn: EXC_RETURN[4] must match the FP state
// reserved at entry. If a lazy-stacked entry (FPCA=1 → LSPACT=1, FType=0
// EXC_RETURN) returns with LR rewritten so bit 4 is set (fabricating
// FType=1), LSPACT stays dangling in thread mode and the next thread-mode
// FP op would flush into stale FPCAR memory. The integrity check in
// exit_exception must fire UsageFault.INVPC rather than silently clear
// LSPACT (silent clear would mask the handler bug).
// ===========================================================================
#[test]
fn exc_return_ftype1_with_lspact_mismatch() {
    let (mut cpu, mut bus) = fixture();

    // 1. Thread-mode FP activity to set FPCA=1.
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    cpu.regs.s[2] = 1.0;
    cpu.regs.s[4] = 2.0;
    cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA);

    // 2. Take an exception (lazy mode is default — LSPEN=1 at reset). Entry
    // reserves the FP region, sets LSPACT, and writes an FType=0
    // EXC_RETURN into LR.
    cpu.test_enter_exception(11, &mut bus);
    
    assert_ne!(cpu.ppb.fpccr & FPCCR_LSPACT, 0, "LSPACT set by lazy entry");
    assert_eq!(cpu.regs.r[14], 0xFFFF_FFE9, "FType=0 EXC_RETURN on entry");

    // 3. Handler manipulates LR to set bit 4 — fabricating FType=1 on an
    // entry that actually reserved an FP frame.
    let fabricated = cpu.regs.r[14] | 0x10;
    assert_eq!(fabricated, 0xFFFF_FFF9, "fabricated FType=1 EXC_RETURN");

    // 4. Return with the fabricated value.
    cpu.test_exit_exception(fabricated, &mut bus);

    // 5. UsageFault pending with CFSR.UFSR.INVPC (bit 17) set.
    assert_ne!(cpu.ppb.cfsr & (1 << 17), 0,
        "CFSR.UFSR.INVPC must be set on FType=1/LSPACT=1 mismatch");
    assert!(cpu.has_pending_fault(),
        "UsageFault must be pending on integrity-check failure");
    // LSPACT is retained — the silent-clear alternative would mask the bug.
    assert_ne!(cpu.ppb.fpccr & FPCCR_LSPACT, 0,
        "LSPACT must NOT be silently cleared by the integrity check");
}

// ===========================================================================
// Test 8 — Phase 3/4/5 exception regression (HLD §B.9 #8)
// ===========================================================================
#[test]
fn exception_replay_under_fpca_zero_and_one() {
    // Under FPCA=0, the path is identical to pre-Phase-7 (basic frame
    // only, FType=1, etc). Under FPCA=1, the same exception sequence
    // must produce a wider frame and FType=0 EXC_RETURN.
    for &fpca in &[false, true] {
        let (mut cpu, mut bus) = fixture();
        if fpca {
            let (hw0, hw1) = enc_vadd(0, 2, 4);
            cpu.regs.s[2] = 1.0;
            cpu.regs.s[4] = 2.0;
            cpu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
            assert_eq!(cpu.regs.control & CONTROL_FPCA, CONTROL_FPCA);
        } else {
            assert_eq!(cpu.regs.control & CONTROL_FPCA, 0);
        }

        let pre_msp = cpu.regs.msp;
        cpu.test_enter_exception(11, &mut bus);

        let expected_frame: u32 = if fpca { 104 } else { 32 };
        let expected_lr: u32 = if fpca { 0xFFFF_FFE9 } else { 0xFFFF_FFF9 };
        assert_eq!(pre_msp - cpu.regs.msp, expected_frame,
            "fpca={}: frame size", fpca);
        assert_eq!(cpu.regs.r[14], expected_lr, "fpca={}: EXC_RETURN", fpca);
        assert_eq!(cpu.regs.ipsr(), 11, "fpca={}: IPSR=SVC", fpca);

        // Return must restore FPCA to its pre-exception value.
        cpu.test_exit_exception(cpu.regs.r[14], &mut bus);
        assert_eq!(cpu.regs.msp, pre_msp, "fpca={}: SP restored", fpca);
        let post_fpca = cpu.regs.control & CONTROL_FPCA;
        assert_eq!(post_fpca,
            if fpca { CONTROL_FPCA } else { 0 },
            "fpca={}: FPCA restored", fpca);
    }
}

// ===========================================================================
// Test 9 — Core 1 FPU (HLD §B.9 #9)
// ===========================================================================
#[test]
fn core1_fpu_entry_exit_isolated_from_core0() {
    // Two independent CortexM33 instances (Core 0 and Core 1) sharing
    // one Bus. Each has its own register file and its own PPB lane.
    // Exercise FPU activity on Core 1 only and verify the FP frame is
    // taken on Core 1's PPB without leaking onto Core 0.
    let mut core0 = CortexM33::with_id(0);
    core0.regs.msp = 0x2000_2000;
    core0.regs.r[13] = core0.regs.msp;

    let mut core1 = CortexM33::with_id(1);
    core1.regs.msp = 0x2000_3000;
    core1.regs.r[13] = core1.regs.msp;

    let mut bus = Bus::default();
    // Vectors for both PPB lanes. Phase 0b.1 Commit B: per-core PPB
    // now lives on CortexM33, so VTOR/SHCSR are programmed per core.
    for &exc in &[2u32, 3, 4, 5, 6, 11, 14, 15] {
        bus.write32(VT_BASE + exc * 4, HANDLER_VEC, 0);
    }
    core0.ppb.vtor = VT_BASE;
    core1.ppb.vtor = VT_BASE;
    core0.ppb.shcsr |= (1 << 16) | (1 << 17) | (1 << 18);
    core1.ppb.shcsr |= (1 << 16) | (1 << 17) | (1 << 18);

    // Core 1 executes VADD — sets FPCA on Core 1, leaves Core 0 untouched.
    core1.regs.s[2] = 7.0;
    core1.regs.s[4] = 11.0;
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    core1.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(core1.regs.s[0], 18.0);
    assert_eq!(core1.regs.control & CONTROL_FPCA, CONTROL_FPCA);
    assert_eq!(core0.regs.control & CONTROL_FPCA, 0,
        "Core 0 FPCA must remain clear");

    // Core 1 takes an exception. FP frame must be allocated on Core 1's
    // stack and recorded on Core 1's PPB lane.
    let pre_msp1 = core1.regs.msp;
    core1.test_enter_exception(11, &mut bus);
    assert_eq!(pre_msp1 - core1.regs.msp, 104);
    assert_ne!(core1.ppb.fpccr & FPCCR_LSPACT, 0, "Core 1 PPB.LSPACT");
    assert_eq!(core0.ppb.fpccr & FPCCR_LSPACT, 0, "Core 0 PPB.LSPACT clear");
    assert_eq!(core1.ppb.fpcar, core1.regs.msp + 32);
    assert_eq!(core0.ppb.fpcar, 0, "Core 0 FPCAR unchanged");

    // Return — Core 1's FPCA must come back to 1.
    core1.test_exit_exception(core1.regs.r[14], &mut bus);
    assert_eq!(core1.regs.control & CONTROL_FPCA, CONTROL_FPCA);
    assert_eq!(core1.regs.msp, pre_msp1);
}

// ===========================================================================
// Smoke test — confirm the FPCCR reset value matches the architecture.
// ===========================================================================
#[test]
fn fpccr_reset_matches_architecture() {
    // Phase 0b.1 Commit B: reset value is produced by
    // `CortexM33::with_id` → `Ppb::default()`; check both cores.
    for id in 0..2u8 {
        let cpu = CortexM33::with_id(id);
        let v = cpu.ppb.fpccr;
        assert_eq!(v & FPCCR_ASPEN, FPCCR_ASPEN, "core {}: ASPEN at reset", id);
        assert_eq!(v & FPCCR_LSPEN, FPCCR_LSPEN, "core {}: LSPEN at reset", id);
        assert_eq!(v & FPCCR_LSPACT, 0, "core {}: LSPACT clear at reset", id);
    }
}
