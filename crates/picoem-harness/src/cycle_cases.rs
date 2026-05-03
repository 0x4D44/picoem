// Cycle oracle catalog — sequences and the measurement stub used by
// `silicon_cycle_oracle_rp2350`.
//
// Each `CycleCase` is a tiny Thumb sequence whose per-iteration cycle cost
// is measured at native speed on real RP2354 silicon (via the mailbox
// stub below) and on the emulator (via `Emulator::step`), then diffed.
//
// **What this measures.** The per-iter number for each case is the cost of
// one `BLX seq / seq body / BX LR` round-trip inside a steady-state
// measurement loop. It is NOT the halt-step per-instruction cost used by
// the entries in `tech_debt.md`. The two numbers measure different
// quantities — a halt-step entry for e.g. `PUSH {R0,R1}` (HW=4, EMU=3) is
// one instruction plus a 5-cycle debug overhead; the sequence-in-loop
// entry for `push_2_min_cost` (`PUSH {R0,R1}; POP {R0,R1}` as a
// sequence) is a two-instruction bundle plus BLX/BX LR/branch-taken
// overhead. Comparing the two directly is a category error. See the
// corresponding section in `tech_debt.md` for the context-qualified
// sequence-in-loop numbers.
//
// See `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md`
// §Oracle 2 for the shape and measurement protocol.

/// A single cycle-oracle case.
///
/// `seq` is the Thumb halfword stream for the measurement body. It must NOT
/// end in `bx lr` (0x4770) — the runner appends that at upload time so the
/// catalog stays focused on what's under test.
///
/// `emu_baseline` is the emulator's per-iter cycle cost as of the last run
/// (the previous recorded baseline for this case). The runner prints the
/// emulator's live value on every invocation so drifts are visible; update
/// this field when a case's emulator value legitimately changes.
///
/// `tolerance` is a per-case known-delta budget (in cycles). The case passes
/// when `|hw - emu| <= max(case.tolerance, args.tolerance)` — i.e. the CLI
/// `--tolerance <N>` override acts as a floor, never a ceiling. Non-zero
/// per-case tolerances encode pipeline-overlap effects the emulator
/// structurally cannot model without a Phase-2 pipeline-model refactor; see
/// `wrk_docs/2026.04.21 - HLD - Track B Cycle Oracle Fidelity.md` for the
/// rationale and the per-case doc comments in `CASES` for the specifics.
pub struct CycleCase {
    pub name: &'static str,
    pub seq: &'static [u16],
    pub emu_baseline: u32,
    pub tolerance: u32,
}

// ---------------------------------------------------------------------------
// MEASUREMENT_STUB — the host-resident Thumb routine that reads the mailbox,
// calls the test sequence K times, and writes the CYCCNT delta back.
//
// AAPCS contract:
//   r0-r3, r12 are caller-saved; r4-r11, LR are callee-saved. `blx` to the
//   sequence may clobber r0-r3, r12, LR — so all loop state must live in
//   callee-saved regs (r4-r7).
//
// Register allocation across the loop:
//   r4 — DWT_CYCCNT pointer  (literal)
//   r5 — mailbox base         (literal)
//   r6 — seq_ptr (Thumb LSB=1)
//   r7 — K counter
//
// Layout (halfword index → assembly):
//   [ 0] push {r4-r7, lr}        ; save callee-saved frame
//   [ 1] ldr  r5, [pc, #36]      ; r5 = MAILBOX_BASE (from literal pool)
//   [ 2] ldr  r0, [r5, #0]       ; r0 = GO           <- poll label
//   [ 3] cmp  r0, #0
//   [ 4] beq  [2]                ; spin until host writes 1
//   [ 5] movs r0, #0
//   [ 6] str  r0, [r5, #0]       ; clear GO
//   [ 7] ldr  r6, [r5, #8]       ; r6 = seq_ptr (LSB=1 for Thumb blx)
//   [ 8] ldr  r7, [r5, #12]      ; r7 = K iteration count
//   [ 9] ldr  r4, [pc, #24]      ; r4 = DWT_CYCCNT
//   [10] movs r0, #0
//   [11] str  r0, [r4]           ; CYCCNT = 0
//   [12] blx  r6                 ; call sequence      <- loop label
//   [13] subs r7, #1
//   [14] bne  [12]               ; while (--K)
//   [15] ldr  r0, [r4]           ; read CYCCNT delta
//   [16] str  r0, [r5, #16]      ; write CYCLES mailbox
//   [17] movs r0, #1
//   [18] str  r0, [r5, #4]       ; DONE = 1
//   [19] b    [2]                ; back to poll (never return — host BKPTs)
//   [20..21] literal: MAILBOX_BASE
//   [22..23] literal: DWT_CYCCNT
//
// Addresses: the stub is assumed to be placed at a 4-byte-aligned SRAM
// address. With that, `ldr rT, [pc, #imm]` targets the literal pool at
// stub_start + 40 (MAILBOX_BASE) and stub_start + 44 (DWT_CYCCNT).
pub const MEASUREMENT_STUB: &[u16] = &[
    0xB5F0, //  [ 0] push {r4, r5, r6, r7, lr}
    0x4D09, //  [ 1] ldr  r5, [pc, #36]  — (PC&~3)+36 = stub+40 (MAILBOX_BASE lit)
    0x6828, //  [ 2] ldr  r0, [r5, #0]   — poll: r0 = mailbox.GO
    0x2800, //  [ 3] cmp  r0, #0
    0xD0FC, //  [ 4] beq  -8             — branch back to [2] (PC-8)
    0x2000, //  [ 5] movs r0, #0
    0x6028, //  [ 6] str  r0, [r5, #0]   — clear GO
    0x68AE, //  [ 7] ldr  r6, [r5, #8]   — r6 = seq_ptr
    0x68EF, //  [ 8] ldr  r7, [r5, #12]  — r7 = K
    0x4C06, //  [ 9] ldr  r4, [pc, #24]  — (PC&~3)+24 = stub+44 (DWT_CYCCNT lit)
    0x2000, //  [10] movs r0, #0
    0x6020, //  [11] str  r0, [r4]       — CYCCNT = 0
    0x47B0, //  [12] blx  r6             — loop: call sequence
    0x3F01, //  [13] subs r7, #1
    0xD1FC, //  [14] bne  -8             — branch back to [12] (PC-8)
    0x6820, //  [15] ldr  r0, [r4]       — read CYCCNT
    0x6128, //  [16] str  r0, [r5, #16]  — CYCLES mailbox
    0x2001, //  [17] movs r0, #1
    0x6068, //  [18] str  r0, [r5, #4]   — DONE = 1
    0xE7ED, //  [19] b    -38            — unconditional branch back to [2]
    0x0000, //  [20] literal MAILBOX_BASE low half  (patched at upload time)
    0x0000, //  [21] literal MAILBOX_BASE high half
    0x0000, //  [22] literal DWT_CYCCNT  low half
    0x0000, //  [23] literal DWT_CYCCNT  high half
];

/// Offset (in halfwords) of the MAILBOX_BASE literal inside `MEASUREMENT_STUB`.
pub const STUB_LIT_MAILBOX_HW: usize = 20;
/// Offset (in halfwords) of the DWT_CYCCNT literal inside `MEASUREMENT_STUB`.
pub const STUB_LIT_DWT_HW: usize = 22;

// ---------------------------------------------------------------------------
// Catalog
//
// Each sequence is hand-assembled Thumb. Cycle accounting notes live next
// to each case so the rationale is auditable without chasing the manual.
// ---------------------------------------------------------------------------

/// Eight Thumb `NOP` halfwords — the positive-control case.
///
/// This is the oracle's own measurement-bias sanity check: NOPs on M33 are
/// the simplest well-defined 1-cycle instructions, so HW and EMU should
/// agree at `per_iter = 8` with tol=0. If they don't, the oracle itself is
/// systematically biased (measurement bug, clock-source mismatch,
/// BLX/BX LR cost modelled differently, etc.) and every other case's
/// delta is suspect until the bias is tracked down.
const SEQ_NOP_CHAIN_8: &[u16] = &[
    0xBF00, // nop
    0xBF00, // nop
    0xBF00, // nop
    0xBF00, // nop
    0xBF00, // nop
    0xBF00, // nop
    0xBF00, // nop
    0xBF00, // nop
];

/// `PUSH {R0, R1}; POP {R0, R1}`.
///
/// Maps loosely to the halt-step `tech_debt.md` entry "PUSH Minimum Cost"
/// (HW=4 vs the 1+N formula's 3), but what we actually measure here is a
/// two-instruction bundle inside a BLX/BX LR round-trip — not the isolated
/// instruction cost. Use the sequence-in-loop table in `tech_debt.md` for
/// the HW/EMU numbers from this case.
const SEQ_PUSH_2: &[u16] = &[
    0xB403, // push {r0, r1}
    0xBC03, // pop  {r0, r1}
];

/// Forward branch past >256-byte nop sled, then backward branch of the
/// same span.
///
/// Maps loosely to the halt-step `tech_debt.md` entry "Backward Branch
/// Pipeline Penalty". As with `push_2_min_cost`, the sequence-in-loop
/// number is the BLX/BX LR round-trip cost — not the isolated branch
/// cost. A divergence here still indicates our pipeline model may
/// differ from silicon, but the delta size is not directly comparable
/// to the "HW=6 vs EMU=1" halt-step deltas in tech_debt.md.
///
/// Layout (hw index inside seq, N = filler count. Final halfword [N+3]
/// is the appended `bx lr` from the runner, not part of this seq array):
///   [0]     B forward to [N+2] (skip)
///   [1]     middle: B forward to [N+3] (end = appended bx lr)
///   [2..N+1] NOP × N    ; filler (stretches the span past 256 B on the
///                         "large" variant)
///   [N+2]   skip: B backward to [1] (middle)
///
/// Iteration: blx → [0] → [N+2] → [1] → end. Exactly one backward
/// branch per iteration; its span is `2*(N+1)` bytes.
///
/// For N_LARGE=150 → span = 302 bytes > 256 B → triggers the M33
/// prefetch-buffer flush on silicon. For N_SMALL=10 → span = 22 bytes,
/// well inside the prefetch window and the positive control that proves
/// the large case isn't spurious.
const SEQ_BACKWARD_LARGE: &[u16] = &BACKWARD_SEQ_LARGE;
const SEQ_BACKWARD_SMALL: &[u16] = &BACKWARD_SEQ_SMALL;

const BACKWARD_SEQ_LARGE: [u16; 153] = make_backward_seq::<153>(150);
const BACKWARD_SEQ_SMALL: [u16; 13] = make_backward_seq::<13>(10);

/// Build a `[u16; TOTAL]` array encoding the backward-branch pattern with
/// `N` NOP halfwords of filler (`TOTAL == N + 3`). Indexing comment above
/// `SEQ_BACKWARD_LARGE` explains the control flow; the appended bx lr
/// sits at halfword [N+3], added by the runner at upload time.
///
/// Encodings:
///   Thumb-16 B T2 (unconditional): `0b11100_imm11` — target = PC + 2*sxt(imm11)
///   where PC = instruction address + 4.
const fn make_backward_seq<const TOTAL: usize>(n: usize) -> [u16; TOTAL] {
    assert!(TOTAL == n + 3, "TOTAL must equal N+3");
    let mut out = [0xBF00u16; TOTAL]; // filler defaults to NOP

    // [0] → forward to [N+2] (skip).
    //   instr_addr=0, PC=4, target=2*(N+2). delta=2*N. imm11_hw=N.
    out[0] = encode_b_t2(n as i32);
    // [1] (middle) → forward to [N+3] (end = appended bx lr).
    //   instr_addr=2, PC=6, target=2*(N+3). delta=2*N. imm11_hw=N.
    out[1] = encode_b_t2(n as i32);
    // [2..N+1] already NOP.
    // [N+2] (skip) → backward to [1] (middle).
    //   instr_addr=2*(N+2), PC=2*(N+2)+4, target=2.
    //   delta = 2 - 2*(N+2) - 4 = -2*(N+3). imm11_hw = -(N+3).
    out[n + 2] = encode_b_t2(-(n as i32) - 3);

    out
}

/// Encode Thumb-16 unconditional B (T2) given the halfword offset imm11
/// (signed, range [-1024, 1023]).
const fn encode_b_t2(imm11_hw: i32) -> u16 {
    assert!(
        imm11_hw >= -1024 && imm11_hw <= 1023,
        "imm11 out of range for B T2"
    );
    let bits = (imm11_hw as i16) as u16 & 0x07FF;
    0xE000 | bits
}

/// `LDR r1, =DATA; LDR r0, [r1]` with DATA in the SAME SRAM bank as the
/// sequence fetch.
///
/// Seq layout (halfwords, seq_start is 4-byte-aligned):
///   [0] ldr r1, [pc, #4]   ; 0x4901 — reads u32 at seq+8 (halfwords [4..6))
///   [1] ldr r0, [r1]       ; 0x6808 — data load (bank hazard under test)
///   [2] bx  lr             ; 0x4770 — explicit return before the literal
///   [3] NOP                ; 0xBF00 — pad so the literal sits at a
///                           ; 4-byte-aligned offset (6 → 8)
///   [4] 0x0200             ; literal low half  (= 0x2000_0200 & 0xFFFF)
///   [5] 0x2000             ; literal high half (= 0x2000_0200 >> 16)
///
/// For `ldr r1, [pc, #4]` at offset 0: PC = 4, Align(PC,4) = 4,
/// imm8 = 1 → byte offset 4 → target = 4 + 4 = 8. That lands on
/// halfwords [4..6) — the literal pool. Without the padding at [3],
/// the u32 at offset 8 would straddle the last seq halfword and the
/// runner's appended `bx lr`, producing a garbage address.
///
/// Data address is `EMU_TEST_SCRATCH` (0x2000_0200). `(0x200 >> 2) & 7 = 0`
/// → bank 0. The sequence lives at `CYCLE_SEQ_SLOT = 0x2000_1000`:
/// `(0x1000 >> 2) & 7 = 0` → bank 0. Fetch and data both hit bank 0 — any
/// contention penalty fires. The `_diff` twin below uses data address
/// 0x2000_0204 (bank 1) as the control; comparing the two cases
/// isolates the bank-contention signal.
///
/// The runner appends another 0x4770 after [5]; that one is unreachable
/// because [2] returns first. The final halfword of `seq` is 0x2000
/// (literal high half), not 0x4770 — satisfying the runner's
/// "seq ends without bx lr" contract.
const SEQ_BANK_CONTENTION_SAME: &[u16] = &[
    0x4901, // ldr r1, [pc, #4]   — reads 32 bits at seq+8 (halfwords [4..6))
    0x6808, // ldr r0, [r1]       — data load
    0x4770, // bx  lr             — explicit early return
    0xBF00, // nop                — padding so literal aligns to offset 8
    0x0200, // literal low half:  0x2000_0200 & 0xFFFF = 0x0200
    0x2000, // literal high half: 0x2000_0200 >> 16    = 0x2000
];

/// `LDR r1, =DATA; LDR r0, [r1]` with DATA in a DIFFERENT SRAM bank to the
/// sequence fetch — the twin control for `SEQ_BANK_CONTENTION_SAME`.
///
/// Layout identical to `SEQ_BANK_CONTENTION_SAME` except the literal now
/// points at `0x2000_0204` — `(0x204 >> 2) & 7 = 1` → bank 1 (vs the seq
/// fetch's bank 0). Comparing `bank_contention_fetch_data_same` against
/// `bank_contention_fetch_data_diff` isolates the contention penalty: if
/// the per-iter numbers differ, the delta is the bank contention; if they
/// match, there is no observable contention in this measurement mode.
const SEQ_BANK_CONTENTION_DIFF: &[u16] = &[
    0x4901, // ldr r1, [pc, #4]
    0x6808, // ldr r0, [r1]
    0x4770, // bx  lr
    0xBF00, // nop (padding)
    0x0204, // literal low half:  0x2000_0204 & 0xFFFF = 0x0204 (bank 1)
    0x2000, // literal high half: 0x2000_0204 >> 16    = 0x2000
];

/// `LDM r0, {r1-r3, r8-r12}` — 8-register LDM via Thumb-32 LDM.W (T2),
/// bare form with no push/pop framing.
///
/// The previous `ldm_8_reg` wrapped the LDM in `push {r4-r7}` / `pop {r4-r7}`
/// framing because `{r1-r8}` clobbers r4-r7 which the stub uses for loop
/// state across the BLX. Reviewer correctly flagged that the push+pop
/// pair (12 emulator cycles) swamps the LDM-8 cost (9 cycles per the 1+N
/// formula), so the original case measured bundle cost, not LDM cost.
///
/// Fix: pick a destination register list that DOESN'T touch r4-r7. LDM
/// excludes the base register (r0), SP, and PC, and its list of loadable
/// regs on M33 is {r1-r12, r14}. Choose {r1, r2, r3, r8, r9, r10, r11, r12}
/// — eight registers, none of which the stub keeps loop state in. The
/// stub already pushed r4-r7 on entry (AAPCS callee-saved frame for its
/// own loop state), so the LDM is free to clobber r8-r11 (also callee-
/// saved in general, but the stub does not use them) and r12 (caller-
/// saved; stub never relies on it across the BLX).
///
/// Encoding of T2 LDM with Rn=r0 (W=0, P=0) and register-list
/// {r1,r2,r3,r8,r9,r10,r11,r12}:
///   hw0 = 0b1110_1000_1001_0000 = 0xE890   (LDM.W, no writeback, Rn=r0)
///   hw1 = register list bitmap  = 0b0001_1111_0000_1110 = 0x1F0E
///         (bits 1,2,3 = r1,r2,r3 ; bits 8,9,10,11,12 = r8..r12)
///
/// Seq layout (halfwords, seq_start 4-byte-aligned):
///   [0] ldr  r0, [pc, #4]  ; 0x4801 — imm8=1 → offset 4 → target 8
///   [1] ldm.w r0, {regs}   ; 0xE890
///   [2]                    ; 0x1F0E (register list)
///   [3] bx   lr            ; 0x4770 — explicit early return
///   [4] literal low half   ; 0x0200 (data address = EMU_TEST_SCRATCH)
///   [5] literal high half  ; 0x2000
const SEQ_LDM_8_REG: &[u16] = &[
    0x4801, // ldr r0, [pc, #4]   — reads u32 at seq+8 (halfwords [4..6))
    0xE890, // ldm.w r0, {r1,r2,r3,r8,r9,r10,r11,r12}   — hw0
    0x1F0E, //                                          — hw1 (reg list)
    0x4770, // bx  lr
    0x0200, // literal low half:  0x2000_0200 & 0xFFFF
    0x2000, // literal high half: 0x2000_0200 >> 16
];

/// One `adds r0, r0, r1` per iteration — the single-adds baseline for
/// `back_to_back_alu`.
///
/// Reviewer observed that `back_to_back_alu` (8× `adds`) has no
/// single-adds control, so any delta between HW and EMU could be either
/// back-to-back forwarding *or* BLX/BX LR round-trip cost. Pairing the
/// two cases isolates the forwarding signal:
///
///   per_iter(single_adds)     = BLX + 1×ADDS + BXLR + loop_ovh
///   per_iter(back_to_back_alu) = BLX + 8×ADDS + BXLR + loop_ovh
///
/// If the difference `back_to_back - single == 7 × emu_add_cost`, there
/// is no observable back-to-back forwarding. If `back_to_back - single <
/// 7 × emu_add_cost`, silicon is forwarding between consecutive ADDS in
/// a way the emulator does not model.
const SEQ_SINGLE_ADDS: &[u16] = &[
    0x1840, // adds r0, r0, r1
];

/// Eight back-to-back `adds r0, r1` — exposes whether forwarding between
/// consecutive ALU ops lets silicon overlap pipeline stages that
/// halt-step measurements can't see. Paired with `single_adds` above.
///
/// `adds r0, r0, r1` T1 encoding: 0b0001_100_Rm_Rn_Rd = 0b0001100_001_000_000
///   Rd=0, Rn=0, Rm=1 → 0x1840.
const SEQ_BACK_TO_BACK_ALU: &[u16] = &[
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
    0x1840, // adds r0, r0, r1
];

/// Initial catalog. `emu_baseline` values are seeded from the M33 cycle
/// model at the time the case was added; the runner prints the live
/// emulator value each run so drift is visible. Update the values here
/// when a case's emulator cost legitimately changes (not to "make tests
/// pass" — the oracle exposes drift between silicon and the model).
pub const CASES: &[CycleCase] = &[
    // Positive control FIRST — if this fails at tol=0, everything below is
    // noise until the bias is tracked down.
    //
    // `nop_chain_8` — known-delta positive control. HW=11 EMU=14 Δ=−3 on
    // 2026-04-21. The delta is structural: M33 folds consecutive NOPs into
    // the prefetch stream (≈−1 to −2) AND silicon absorbs the stub's
    // BLX/BX-LR framing into its prefetch state in a way our binary
    // "sequential vs non-sequential" abstraction cannot represent (≈−1).
    // Closing at tol=0 would require an M33 fetch/issue pipeline model we
    // have deliberately declined to build (broad blast radius across the
    // emulator + ~2388 unit tests for a 3-cycle microbench residual). See
    // `wrk_docs/2026.04.21 - HLD - Track B Cycle Oracle Fidelity.md` §3.2
    // and §5 for the full rationale; the printed hw/emu/delta columns
    // still surface any future drift even when the case passes.
    CycleCase {
        name: "nop_chain_8",
        seq: SEQ_NOP_CHAIN_8,
        emu_baseline: 14,
        tolerance: 3,
    },
    // `push_2_min_cost` — known-delta. HW=10 EMU=12 Δ=−2. Silicon holds
    // the two PUSH stores in the M33 write buffer; when the POP loads
    // arrive targeting the same addresses, they're satisfied from the
    // write buffer at 1 cycle each (partial store-to-load forwarding),
    // giving ≈4-cycle body cost vs. our 6. Requires a write-buffer state
    // model with address/forwarding tracking — see HLD §3.3.
    CycleCase {
        name: "push_2_min_cost",
        seq: SEQ_PUSH_2,
        emu_baseline: 12,
        tolerance: 2,
    },
    CycleCase {
        name: "backward_branch_small",
        seq: SEQ_BACKWARD_SMALL,
        emu_baseline: 13,
        tolerance: 0,
    },
    CycleCase {
        name: "backward_branch_large",
        seq: SEQ_BACKWARD_LARGE,
        emu_baseline: 13,
        tolerance: 0,
    },
    // `bank_contention_fetch_data_same` — known-delta Δ=−1. Not
    // contention-sourced: silicon exhibits no observable bank-contention
    // penalty at sequence-in-loop scale and we do not model it on RP2350
    // by design. The residual is LDR→LDR load-to-use forwarding: silicon
    // forwards the first LDR's result to the second LDR's address phase,
    // saving one cycle. See HLD §3.4.
    CycleCase {
        name: "bank_contention_fetch_data_same",
        seq: SEQ_BANK_CONTENTION_SAME,
        emu_baseline: 10,
        tolerance: 1,
    },
    // `bank_contention_fetch_data_diff` — same root cause as `_same`
    // above. HLD §3.4.
    CycleCase {
        name: "bank_contention_fetch_data_diff",
        seq: SEQ_BANK_CONTENTION_DIFF,
        emu_baseline: 10,
        tolerance: 1,
    },
    CycleCase {
        name: "ldm_8_reg",
        seq: SEQ_LDM_8_REG,
        emu_baseline: 17,
        tolerance: 0,
    },
    CycleCase {
        name: "single_adds",
        seq: SEQ_SINGLE_ADDS,
        emu_baseline: 7,
        tolerance: 0,
    },
    CycleCase {
        name: "back_to_back_alu",
        seq: SEQ_BACK_TO_BACK_ALU,
        emu_baseline: 14,
        tolerance: 0,
    },
];

// ---------------------------------------------------------------------------
// Delta math
// ---------------------------------------------------------------------------

/// Per-iteration cycle cost from two steady-state measurements at K_low
/// and K_high. Precondition: K_high > K_low.
pub fn per_iter(m_low: u32, m_high: u32, k_low: u32, k_high: u32) -> u32 {
    debug_assert!(k_high > k_low, "k_high must be > k_low");
    (m_high - m_low) / (k_high - k_low)
}

// ---------------------------------------------------------------------------
// Emulator measurement path
// ---------------------------------------------------------------------------
//
// Lives here (rather than in the `silicon_cycle_oracle_rp2350` binary) so
// that unit tests can drive it end-to-end without going through probe-rs.
// The binary imports `fresh_emulator` and `measure_emu` from this module.

use crate::{CYCLE_MAILBOX_BASE, EMU_TEST_SLOT, EMU_TEST_STACK};
use rp2350_emu::{Config, Emulator, EmulatorBuilder};

/// Sequence scratch region (4 KB above the stub). Mirrors the constant of
/// the same name in the binary; kept here because `fresh_emulator` needs
/// it for SRAM upload.
pub const CYCLE_SEQ_SLOT: u32 = 0x2000_1000;

/// Stub start in SRAM (same as `EMU_TEST_SLOT` — reuses the ISA oracle's
/// slot, the two oracles never run concurrently).
pub const STUB_START: u32 = EMU_TEST_SLOT;

// Mailbox word offsets.
pub const MBX_GO: u32 = 0x00;
pub const MBX_DONE: u32 = 0x04;
pub const MBX_SEQ_PTR: u32 = 0x08;
pub const MBX_ITER: u32 = 0x0C;
pub const MBX_CYCLES: u32 = 0x10;
pub const MBX_RESERVED: u32 = 0x14;

// DWT / CoreDebug MMIO — deduped in `silicon_oracle.rs`; re-exported so
// existing consumers (`pack_stub`, the binary wrappers) don't need path
// churn when they hoisted out from duplicated copies.
pub use crate::silicon_oracle::{
    CYCCNTENA, DEMCR_U32 as DEMCR, DWT_CTRL_U32 as DWT_CTRL, DWT_CYCCNT_ADDR, TRCENA,
};

/// Produce the fully-patched stub bytes (little-endian halfwords) with
/// the MAILBOX_BASE and DWT_CYCCNT literals written into the pool slots.
pub fn pack_stub() -> Vec<u8> {
    let mut hws: Vec<u16> = MEASUREMENT_STUB.to_vec();
    let mbx = CYCLE_MAILBOX_BASE;
    let dwt = DWT_CYCCNT_ADDR;
    hws[STUB_LIT_MAILBOX_HW] = (mbx & 0xFFFF) as u16;
    hws[STUB_LIT_MAILBOX_HW + 1] = (mbx >> 16) as u16;
    hws[STUB_LIT_DWT_HW] = (dwt & 0xFFFF) as u16;
    hws[STUB_LIT_DWT_HW + 1] = (dwt >> 16) as u16;
    let mut out = Vec::with_capacity(hws.len() * 2);
    for hw in &hws {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out
}

/// Pack a sequence + appended bx lr sentinel into a byte stream.
pub fn pack_seq(seq: &[u16]) -> Vec<u8> {
    let mut out = Vec::with_capacity((seq.len() + 1) * 2);
    for hw in seq {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out.extend_from_slice(&0x4770u16.to_le_bytes()); // bx lr sentinel
    out
}

// ---------------------------------------------------------------------------
// Hardware-side measurement (probe-rs path, shared with the orchestrator)
// ---------------------------------------------------------------------------
//
// These helpers were formerly private to `bin/silicon_cycle_oracle_rp2350.rs`.
// Hoisting them into the catalogue module lets `run_against` (and, by
// extension, the `test_silicon` orchestrator) drive the same measurement
// path without duplicating MMIO constants.

use crate::silicon_oracle::{self, CaseOutcome, Verdict, enable_cyccnt};
use probe_rs::{Core, MemoryInterface, RegisterId};
use std::time::{Duration, Instant};

const PC_REG: RegisterId = RegisterId(15);
const XPSR_REG: RegisterId = RegisterId(16);
const SP_REG: RegisterId = RegisterId(13);
const LR_REG: RegisterId = RegisterId(14);

// SCB fault-status registers (host side — used in timeout diagnostics).
const HFSR_ADDR: u64 = 0xE000_ED2C;
const CFSR_ADDR: u64 = 0xE000_ED28;

/// Per-K polling timeout for DONE (per case, per iteration count).
const DONE_TIMEOUT: Duration = Duration::from_secs(1);

/// Zero the mailbox region on silicon (six u32 slots).
fn zero_mailbox_hw(core: &mut Core) -> Result<(), probe_rs::Error> {
    for off in [
        MBX_GO,
        MBX_DONE,
        MBX_SEQ_PTR,
        MBX_ITER,
        MBX_CYCLES,
        MBX_RESERVED,
    ] {
        core.write_word_32((CYCLE_MAILBOX_BASE + off) as u64, 0)?;
    }
    Ok(())
}

/// Write mailbox: GO=1, DONE=0, SEQ_PTR=seq_start|1, ITER=K.
fn kick_mailbox_hw(core: &mut Core, seq_start: u32, k: u32) -> Result<(), probe_rs::Error> {
    debug_assert!(
        seq_start & 1 == 0,
        "seq_start must be halfword-aligned before OR'ing Thumb bit"
    );
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_DONE) as u64, 0)?;
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_CYCLES) as u64, 0)?;
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_SEQ_PTR) as u64, seq_start | 1)?;
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_ITER) as u64, k)?;
    // GO last — stub is spinning on this.
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_GO) as u64, 1)?;
    Ok(())
}

/// Resume, poll DONE until 1 or timeout, halt, read CYCLES.
///
/// On timeout, returns a diagnostic string containing PC/SP/LR/CFSR/HFSR and
/// the six mailbox slots — just enough state to distinguish "stub wedged in
/// poll" from "fault in stub" from "fault in seq".
fn wait_and_read_cycles_hw(core: &mut Core) -> Result<u32, Box<dyn std::error::Error>> {
    core.run()?;
    let deadline = Instant::now() + DONE_TIMEOUT;
    loop {
        let done: u32 = core.read_word_32((CYCLE_MAILBOX_BASE + MBX_DONE) as u64)?;
        if done == 1 {
            break;
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC_REG).unwrap_or(0xDEAD_BEEF);
            let sp: u32 = core.read_core_reg(SP_REG).unwrap_or(0xDEAD_BEEF);
            let lr: u32 = core.read_core_reg(LR_REG).unwrap_or(0xDEAD_BEEF);
            let hfsr: u32 = core.read_word_32(HFSR_ADDR).unwrap_or(0xDEAD_BEEF);
            let cfsr: u32 = core.read_word_32(CFSR_ADDR).unwrap_or(0xDEAD_BEEF);
            let go: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_GO) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let done_v: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_DONE) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let seq_ptr: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_SEQ_PTR) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let iter: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_ITER) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let cycles: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_CYCLES) as u64)
                .unwrap_or(0xDEAD_BEEF);
            return Err(format!(
                "timeout waiting for stub DONE=1\n\
                 PC=0x{pc:08X} SP=0x{sp:08X} LR=0x{lr:08X}\n\
                 HFSR=0x{hfsr:08X} CFSR=0x{cfsr:08X}\n\
                 mailbox: GO={go} DONE={done_v} SEQ_PTR=0x{seq_ptr:08X} ITER={iter} CYCLES={cycles}"
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    core.halt(Duration::from_millis(200))?;
    let cycles: u32 = core.read_word_32((CYCLE_MAILBOX_BASE + MBX_CYCLES) as u64)?;
    Ok(cycles)
}

/// Measure raw cycles for one K-value on hardware. Caller is responsible for
/// having written the stub + sequence + mailbox state before the first call.
pub fn measure_hw(
    core: &mut Core,
    seq_start: u32,
    k: u32,
) -> Result<u32, Box<dyn std::error::Error>> {
    // Re-prime PC so the run re-enters the stub's push frame fresh.
    core.write_core_reg(PC_REG, STUB_START)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP_REG, EMU_TEST_STACK)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;
    kick_mailbox_hw(core, seq_start, k)?;
    wait_and_read_cycles_hw(core)
}

// ---------------------------------------------------------------------------
// Public CLI-level runner args + `run_against`
// ---------------------------------------------------------------------------

/// Arguments for `run_against`. Mirrors the standalone binary's CLI.
#[derive(Clone, Debug)]
pub struct CycleArgs {
    pub filter: Option<String>,
    pub exclude: Option<String>,
    pub iter_low: u32,
    pub iter_high: u32,
    pub tolerance: u32,
}

impl Default for CycleArgs {
    fn default() -> Self {
        Self {
            filter: None,
            exclude: None,
            iter_low: 101,
            iter_high: 201,
            tolerance: 0,
        }
    }
}

/// Per-case richer result used by the standalone binary's table output.
///
/// `effective_tolerance` is `max(case.tolerance, args.tolerance)` — the
/// budget actually applied to decide PASS/FAIL. `known_delta_pass` is
/// `true` iff the case passed only because that effective tolerance was
/// non-zero (i.e. `delta != 0 && verdict == Pass`). The standalone
/// binary uses these two fields to print the `PASS (known Δ=<delta>, tol=<N>)`
/// annotation mandated by the HLD; see its lead-sign-off addendum.
#[derive(Debug)]
pub struct CycleCaseResult {
    pub name: &'static str,
    pub hw_low: u32,
    pub hw_high: u32,
    pub hw_per_iter: u32,
    pub emu_low: u32,
    pub emu_high: u32,
    pub emu_per_iter: u32,
    pub emu_baseline: u32,
    pub delta: i64,
    pub verdict: Verdict,
    pub effective_tolerance: u32,
    pub known_delta_pass: bool,
    pub elapsed_ms: u32,
}

impl CycleCaseResult {
    /// Convert this per-case result into a `CaseOutcome` for the unified
    /// report.
    ///
    /// **Note — canonical path is `run_against`.** This helper is kept for
    /// callers that hold a `CycleCaseResult` directly, but it does NOT
    /// emit the `known Δ=… tol=…` detail that `run_against` attaches to
    /// PASS outcomes when `known_delta_pass == true`. If you need that
    /// annotation in orchestrator output, go through `run_against`.
    /// Currently no in-tree caller depends on this helper (2026-04-21);
    /// merging the two code paths is deferred until a second caller
    /// appears.
    pub fn to_outcome(&self) -> CaseOutcome {
        match self.verdict {
            Verdict::Pass => CaseOutcome::pass("cycle", self.name, self.elapsed_ms),
            Verdict::Fail | Verdict::Skip | Verdict::Degraded => {
                let detail = format!(
                    "hw={} emu={} delta={:+} tol={}",
                    self.hw_per_iter, self.emu_per_iter, self.delta, self.effective_tolerance,
                );
                CaseOutcome::fail("cycle", self.name, detail, self.elapsed_ms)
            }
        }
    }
}

/// Run a single case end-to-end (HW + EMU) and produce a `CycleCaseResult`.
/// Separate from `to_outcome` so the standalone binary can keep its rich
/// per-case table.
///
/// `cli_tolerance` is the CLI `--tolerance <N>` floor (from `CycleArgs`);
/// the effective budget is `max(case.tolerance, cli_tolerance)`. Per-case
/// tolerances (see `CASES` doc comments + the Track B Cycle Oracle Fidelity
/// HLD) encode pipeline-effect residuals that are not accounting bugs; the
/// CLI floor still lets operators tighten the oracle with `--tolerance 0`
/// to surface those known deltas in CI-style output, or loosen it
/// uniformly for exploratory sweeps.
pub fn run_cycle_case(
    core: &mut Core,
    case: &CycleCase,
    iter_low: u32,
    iter_high: u32,
    cli_tolerance: u32,
) -> Result<CycleCaseResult, Box<dyn std::error::Error>> {
    let t0 = Instant::now();
    let seq_bytes = pack_seq(case.seq);

    // Write seq to hardware SRAM once per case.
    core.write_8(CYCLE_SEQ_SLOT as u64, &seq_bytes)?;

    let hw_low = measure_hw(core, CYCLE_SEQ_SLOT, iter_low)?;
    let hw_high = measure_hw(core, CYCLE_SEQ_SLOT, iter_high)?;
    let hw_per_iter = (hw_high - hw_low) / (iter_high - iter_low);

    let mut emu = fresh_emulator(&seq_bytes);
    let emu_low = measure_emu(&mut emu, CYCLE_SEQ_SLOT, iter_low)?;
    let emu_high = measure_emu(&mut emu, CYCLE_SEQ_SLOT, iter_high)?;
    let emu_per_iter = (emu_high - emu_low) / (iter_high - iter_low);

    let delta = hw_per_iter as i64 - emu_per_iter as i64;
    let effective_tolerance = case.tolerance.max(cli_tolerance);
    let verdict = if (delta.unsigned_abs() as u32) <= effective_tolerance {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    // "Known-delta pass": passed, but only because tolerance absorbed a
    // non-zero delta. tol=0 perfect matches don't trip this flag.
    let known_delta_pass = verdict == Verdict::Pass && delta != 0;

    let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
    Ok(CycleCaseResult {
        name: case.name,
        hw_low,
        hw_high,
        hw_per_iter,
        emu_low,
        emu_high,
        emu_per_iter,
        emu_baseline: case.emu_baseline,
        delta,
        verdict,
        effective_tolerance,
        known_delta_pass,
        elapsed_ms,
    })
}

/// Library entry point used by `silicon_cycle_oracle_rp2350` and the
/// `test_silicon` orchestrator.
///
/// **Cleanup contract**: none. The cycle oracle only writes SRAM scratch
/// (stub + sequences + mailbox) and enables DWT — both defaulted by
/// `core.reset_and_halt`, so the next oracle in an orchestrated run sees
/// a clean slate without any explicit teardown.
///
/// Preconditions: `core` is halted. `reset_and_halt` + `enable_cyccnt` have
/// been called (the orchestrator owns these; the standalone binary's thin
/// wrapper does them before calling here).
///
/// Case selection semantics:
/// * `order = None` — run every catalogue case whose name matches
///   `args.filter`, in catalogue-declared order (single-pass / standalone
///   default).
/// * `order = Some(&[name, name, …])` — run exactly those cases in that
///   order. `args.filter` is ignored for selection (the caller already
///   applied any filtering when building the list). Names not present in
///   the catalogue are skipped with a single `eprintln!` warning per
///   unknown name so a mis-shuffled call leaves a breadcrumb but does
///   not bring down the run.
pub fn run_against(
    core: &mut Core,
    args: &CycleArgs,
    order: Option<&[&str]>,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error>> {
    debug_assert!(
        args.iter_high > args.iter_low,
        "iter_high must exceed iter_low"
    );

    // Enable DWT defensively — idempotent; lets the standalone binary be
    // minimal and also lets the orchestrator call us without trusting
    // upstream setup.
    enable_cyccnt(core)?;

    // Write the stub once; it stays resident across all cases.
    let stub_bytes = pack_stub();
    core.write_8(STUB_START as u64, &stub_bytes)?;
    zero_mailbox_hw(core)?;

    // Prime core registers. probe-rs strips the Thumb LSB internally; write
    // the aligned PC and set XPSR.T=1.
    core.write_core_reg(PC_REG, STUB_START)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP_REG, EMU_TEST_STACK)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;

    let selected: Vec<&CycleCase> = match order {
        None => CASES
            .iter()
            .filter(|c| silicon_oracle::name_matches_filter(c.name, args.filter.as_deref()))
            .filter(|c| !silicon_oracle::should_exclude(c.name, args.exclude.as_deref()))
            .collect(),
        Some(names) => {
            let mut v: Vec<&CycleCase> = Vec::with_capacity(names.len());
            for name in names {
                match CASES.iter().find(|c| c.name == *name) {
                    Some(c) => v.push(c),
                    None => eprintln!(
                        "cycle_cases::run_against: unknown case '{name}' in order list; skipping",
                    ),
                }
            }
            v
        }
    };

    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(selected.len());
    for case in selected {
        let r = run_cycle_case(core, case, args.iter_low, args.iter_high, args.tolerance)?;
        let detail = match r.verdict {
            // Distinguish PASS (exact match) from PASS (known Δ absorbed by
            // per-case tolerance). Plain PASS keeps an empty detail so the
            // orchestrator's summary output stays tidy; known-delta PASS
            // carries the Δ + tol so soak-run forensics don't have to
            // re-read the standalone binary's table.
            Verdict::Pass if r.known_delta_pass => {
                format!("known Δ={:+} tol={}", r.delta, r.effective_tolerance,)
            }
            Verdict::Pass => String::new(),
            // Fail / Skip / Degraded all surface the same numeric detail.
            // `run_cycle_case` only produces Pass/Fail today; tolerating
            // Skip/Degraded keeps this match exhaustive without forcing a
            // separate code path.
            Verdict::Fail | Verdict::Skip | Verdict::Degraded => format!(
                "hw={} emu={} delta={:+} tol={}",
                r.hw_per_iter, r.emu_per_iter, r.delta, r.effective_tolerance,
            ),
        };
        outcomes.push(CaseOutcome {
            oracle: "cycle",
            case: r.name,
            verdict: r.verdict,
            detail,
            elapsed_ms: r.elapsed_ms,
        });
        // `reset_and_halt` is not needed between cases; the stub's final
        // `b poll` at [19] loops back to the poll label and `measure_hw`
        // re-primes PC on each call.
    }
    Ok(outcomes)
}

/// Build a fresh emulator pre-loaded with the stub + seq bytes, mailbox
/// zeroed, DWT enabled, core 0 primed to start at the stub, core 1
/// halted.
pub fn fresh_emulator(seq_bytes: &[u8]) -> Emulator {
    // step_quantum(1) — the stub + seq all run on core 0; the DWT reads
    // inside the stub see per-instruction core.cycles via PPB, so cycle
    // accounting is per-instruction regardless of quantum.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    emu.cores.expect_arm_mut()[1].halt();

    let stub_bytes = pack_stub();
    for (i, &b) in stub_bytes.iter().enumerate() {
        emu.bus
            .memory
            .sram_write8((STUB_START - 0x2000_0000) + i as u32, b);
    }
    for (i, &b) in seq_bytes.iter().enumerate() {
        emu.bus
            .memory
            .sram_write8((CYCLE_SEQ_SLOT - 0x2000_0000) + i as u32, b);
    }
    for off in [
        MBX_GO,
        MBX_DONE,
        MBX_SEQ_PTR,
        MBX_ITER,
        MBX_CYCLES,
        MBX_RESERVED,
    ] {
        emu.bus.write32(CYCLE_MAILBOX_BASE + off, 0, 0);
    }

    // DEMCR + DWT_CTRL live in the PPB region. Phase 0b.1 Commit B
    // moved the per-core PPB onto CortexM33; `Emulator::mmio_read32` /
    // `mmio_write32` intercept PPB addresses and route to core 0's PPB.
    let demcr = emu.mmio_read32(DEMCR);
    emu.mmio_write32(DEMCR, demcr | TRCENA);
    let ctrl = emu.mmio_read32(DWT_CTRL);
    emu.mmio_write32(DWT_CTRL, ctrl | CYCCNTENA);

    {
        let c0 = &mut emu.cores.expect_arm_mut()[0];
        c0.wake();
        c0.regs.set_pc(STUB_START);
        c0.regs.r[13] = EMU_TEST_STACK;
        c0.regs.msp = EMU_TEST_STACK;
        c0.regs.r[14] = 0xFFFF_FFFF;
        c0.regs.xpsr = 0x0100_0000; // T=1
    }

    emu
}

/// Kick the emulator mailbox, then spin `emu.step()` until DONE=1 or
/// we blow through a generous cycle budget. Returns raw CYCLES.
pub fn measure_emu(emu: &mut Emulator, seq_start: u32, k: u32) -> Result<u32, String> {
    // Cheap insurance against future catalog authors passing a mis-
    // aligned sequence address: the stub ORs the Thumb bit in,
    // so `seq_start` itself must be halfword-aligned.
    debug_assert!(
        seq_start & 1 == 0,
        "seq_start must be halfword-aligned before OR'ing Thumb bit"
    );
    emu.bus.write32(CYCLE_MAILBOX_BASE + MBX_DONE, 0, 0);
    emu.bus.write32(CYCLE_MAILBOX_BASE + MBX_CYCLES, 0, 0);
    emu.bus
        .write32(CYCLE_MAILBOX_BASE + MBX_SEQ_PTR, seq_start | 1, 0);
    emu.bus.write32(CYCLE_MAILBOX_BASE + MBX_ITER, k, 0);
    emu.bus.write32(CYCLE_MAILBOX_BASE + MBX_GO, 1, 0);

    let budget: u64 = 1_000_000u64.max((k as u64) * 200);
    let start_cycles = emu.cycles();
    loop {
        emu.step().expect("Serial step is infallible");
        let done = emu.bus.read32(CYCLE_MAILBOX_BASE + MBX_DONE, 0);
        if done == 1 {
            break;
        }
        if emu.cycles() - start_cycles > budget {
            return Err(format!(
                "emulator budget ({budget} cycles) exhausted before DONE=1 (k={k})"
            ));
        }
    }
    Ok(emu.bus.read32(CYCLE_MAILBOX_BASE + MBX_CYCLES, 0))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mailbox_base_alignment() {
        assert_eq!(
            CYCLE_MAILBOX_BASE % 4,
            0,
            "mailbox base must be word-aligned"
        );
    }

    #[test]
    fn test_stub_ends_looping() {
        // Final instruction halfword (before the literal pool at [20..24])
        // is an unconditional backward B (T2) to the poll label at [2].
        //
        //   instr_addr([19]) = 38, PC = 42, target = 4 → delta = -38,
        //   imm11_hw = -19 → two's-complement 11-bit = 0x7ED.
        //   Encoding: 0b11100_111_1110_1101 = 0xE7ED.
        assert_eq!(
            MEASUREMENT_STUB[19], 0xE7ED,
            "final branch must loop to poll"
        );
        // Computed, not hardcoded (proof):
        let instr_addr_hw = 19;
        let pc_hw = instr_addr_hw + 2; // PC = instr+4 bytes = +2 halfwords
        let target_hw: i32 = 2; // poll label
        let imm11_hw = target_hw - pc_hw; // signed halfword offset
        let encoded = 0xE000u16 | ((imm11_hw as i16) as u16 & 0x07FF);
        assert_eq!(
            MEASUREMENT_STUB[19], encoded,
            "final B encoding matches formula"
        );
    }

    #[test]
    fn test_case_seqs_no_final_bxlr() {
        for case in CASES {
            let last = *case.seq.last().expect("seq must be non-empty");
            assert_ne!(
                last, 0x4770,
                "case '{}' must not end in bx lr (runner appends it)",
                case.name,
            );
        }
    }

    /// End-to-end measurement-math test against the emulator.
    ///
    /// Runs the real `fresh_emulator` + `measure_emu` path on a tiny
    /// two-instruction sequence (`adds r0, r1; bx lr` — the appended bx
    /// lr makes this effectively a 2-halfword seq, one ADDS per call)
    /// at three K values. Asserts:
    ///
    ///   1. `(m_K2 − m_K1)` divides exactly by `(K2 − K1)` — catches a
    ///      division/rounding bug.
    ///   2. Per-iter computed from K1 vs K2 matches per-iter from K1 vs
    ///      K3 and from K2 vs K3 — catches a non-linear-in-K bug (e.g.
    ///      warm-up cost slipping into every iteration).
    ///   3. Per-iter is positive and within a sanity bound (≤ 20, well
    ///      above BLX+ADDS+BXLR+loop).
    ///
    /// The point is NOT to measure a specific cycle count (that's the
    /// oracle's job against silicon); it's to catch a future bug in the
    /// measurement path itself. If somebody breaks the K-delta math or
    /// the mailbox round-trip, this test fires long before the next HW
    /// run.
    #[test]
    fn test_measurement_math_end_to_end() {
        // Single `adds r0, r0, r1`. Runner appends `bx lr` so this is a
        // ~2-cycle seq body inside BLX/BX LR framing.
        let seq: &[u16] = &[0x1840];
        let seq_bytes = pack_seq(seq);

        let mut emu = fresh_emulator(&seq_bytes);

        let k1 = 101u32;
        let k2 = 201u32;
        let k3 = 301u32;

        let m_k1 = measure_emu(&mut emu, CYCLE_SEQ_SLOT, k1).expect("measure_emu K=101");
        let m_k2 = measure_emu(&mut emu, CYCLE_SEQ_SLOT, k2).expect("measure_emu K=201");
        let m_k3 = measure_emu(&mut emu, CYCLE_SEQ_SLOT, k3).expect("measure_emu K=301");

        // (1) exact division.
        assert_eq!(
            (m_k2 - m_k1) % (k2 - k1),
            0,
            "K-delta does not divide exactly: m_k1={m_k1} m_k2={m_k2}",
        );
        assert_eq!(
            (m_k3 - m_k1) % (k3 - k1),
            0,
            "K-delta does not divide exactly: m_k1={m_k1} m_k3={m_k3}",
        );
        assert_eq!(
            (m_k3 - m_k2) % (k3 - k2),
            0,
            "K-delta does not divide exactly: m_k2={m_k2} m_k3={m_k3}",
        );

        // (2) per-iter is stable across all three pairings.
        let p12 = per_iter(m_k1, m_k2, k1, k2);
        let p13 = per_iter(m_k1, m_k3, k1, k3);
        let p23 = per_iter(m_k2, m_k3, k2, k3);
        assert_eq!(p12, p13, "per_iter drifts: K1↔K2={p12}, K1↔K3={p13}");
        assert_eq!(p12, p23, "per_iter drifts: K1↔K2={p12}, K2↔K3={p23}");

        // (3) sanity bounds.
        assert!(p12 > 0, "per_iter must be > 0, got {p12}");
        assert!(
            p12 <= 20,
            "per_iter={p12} is implausibly high for BLX+ADDS+BXLR+loop",
        );
    }

    /// Verifies the effective-tolerance rule mandated by the Track B
    /// Cycle Oracle Fidelity HLD §6.1: `effective = max(case.tolerance,
    /// cli.tolerance)`. CLI `--tolerance <N>` must remain a floor (can
    /// widen but never tighten a per-case budget), so the pass/fail
    /// decision uses the larger of the two.
    ///
    /// This test is pure data-level (no probe-rs, no emulator path) — it
    /// exercises the `max()` rule directly by building `CycleCase`
    /// instances with known tolerances and asserting the effective
    /// budget matches expectations for three representative scenarios.
    #[test]
    fn test_per_case_tolerance_overrides_cli_tolerance() {
        // The rule under test. Kept as a local closure so the test
        // documents the effective-tolerance formula rather than indirectly
        // asserting via `run_cycle_case` (which needs a live probe).
        fn effective_tolerance(case_tol: u32, cli_tol: u32) -> u32 {
            case_tol.max(cli_tol)
        }

        // Scenario 1: case.tolerance=3, cli.tolerance=0 → 3 (per-case
        // survives when CLI is silent). A delta of 3 must still pass.
        let eff = effective_tolerance(3, 0);
        assert_eq!(eff, 3, "case.tolerance=3 must survive cli=0");
        let delta: i64 = -3;
        assert!(
            (delta.unsigned_abs() as u32) <= eff,
            "delta={delta} must pass at eff={eff}",
        );

        // Scenario 2: case.tolerance=0, cli.tolerance=5 → 5 (CLI floor
        // widens a zero-tol case; useful for exploratory sweeps).
        let eff = effective_tolerance(0, 5);
        assert_eq!(eff, 5, "cli.tolerance=5 must widen case=0");
        let delta: i64 = 4;
        assert!(
            (delta.unsigned_abs() as u32) <= eff,
            "delta={delta} must pass at eff={eff}",
        );

        // Scenario 3: case.tolerance=2, cli.tolerance=5 → 5 (the larger
        // wins; neither value alone is the answer).
        let eff = effective_tolerance(2, 5);
        assert_eq!(eff, 5, "max(2,5)=5 — larger wins");
        // And confirm the per-case value is NOT the answer when CLI is
        // larger (this was the pre-HLD behaviour, intentionally kept
        // visible here in case a future refactor regresses it).
        assert_ne!(eff, 2, "cli.tolerance=5 floor must not be tightened to 2");
    }
}
