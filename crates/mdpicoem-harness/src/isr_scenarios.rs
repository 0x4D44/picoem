// ISR oracle catalogue — exception entry + lazy/eager FP save + tail-
// chaining, diffed by `silicon_isr_diff_rp2350` against real RP2354
// silicon. Each scenario owns a hand-assembled SRAM image (vector table
// + default handler + PendSV/SysTick handler + main routine) and a set
// of observables read post-BKPT on both sides. See
// `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md` §Component 2 for the full design — image layout, scenario
// type, and initial catalogue all live there.
//
// **Why a new oracle, not a peripheral scenario?** Exception entry needs
// code in SRAM to *run* when the exception fires — the peripheral
// oracle's countdown sled has no room for handler dispatch. This module
// owns its own sled equivalent: a per-scenario vector table, handler
// stub, and main routine, all in SRAM, VTOR reprogrammed to point at the
// SRAM vector table.
//
// **Expected EMU behaviour.** As of Stage 6 landing, the mdrp2350 core's
// `step()` does NOT poll ICSR.PENDSVSET / PENDSTSET between
// instructions — an `str icsr, #PENDSVSET` write sets the latch but no
// exception is dispatched. That is exactly the tech-debt item this
// oracle is designed to expose: see `tech_debt.md:295` ("Exception
// entry/exit not differentially validated") and HLD §Component 2 rationale.
// v1 scenarios are expected to FAIL on the EMU side until pending-
// exception dispatch lands; every FAIL should surface a concrete
// divergence (CYCCNT delta, stacked frame layout, FPCCR state).
//
// **Known EMU limitation — ICSR W1S/W1C semantics.** Per ARMv8-M, writing
// a 1 to ICSR.PENDSVSET (bit 28) or ICSR.PENDSTSET (bit 26) SETS the
// corresponding pending bit, and writing a 1 to PENDSVCLR (bit 27) or
// PENDSTCLR (bit 25) CLEARS it. `crates/mdrp2350/src/bus/ppb.rs` does
// not currently implement these write-1-to-set / write-1-to-clear
// semantics — a plain `str` to ICSR lands as a direct register write,
// not a latch update. The consequence for this oracle is that any EMU
// run that depends on ICSR write semantics to trigger an exception
// will miss the trigger entirely; this compounds with the pending-
// exception dispatch gap above and is part of the same tech-debt fix
// (tech_debt.md:295, "Exception entry/exit not differentially
// validated"). TODO: resolve alongside the CortexM33 exception-dispatch
// landing; when that lands, add PENDSVSET / PENDSTSET / PENDSVCLR /
// PENDSTCLR write-side handling in ppb.rs.

// ---------------------------------------------------------------------------
// Absolute MMIO constants (RP2350 M33)
// ---------------------------------------------------------------------------

/// Secure SCB ICSR (Interrupt Control State Register). Bit 28 = PENDSVSET
/// (pend PendSV), bit 26 = PENDSTSET (pend SysTick). RP2354 boots secure.
pub const SCB_ICSR_ADDR: u32 = 0xE000_ED04;
/// Secure SCB VTOR. Vector-table offset; low 7 bits must be zero
/// (VTOR is 128-byte aligned).
pub const S_VTOR_ADDR: u32 = 0xE000_ED08;
/// Non-Secure SCB VTOR alias (`SCB_NS` base + 0xD08 = 0xE002_ED08).
pub const NS_VTOR_ADDR: u32 = 0xE002_ED08;
/// Secure SCB CPACR. FPU access enable lives in CP10/CP11 (bits 20–23).
pub const S_CPACR_ADDR: u32 = 0xE000_ED88;
/// Non-Secure SCB CPACR alias (`SCB_NS` base + 0xD88 = 0xE002_ED88).
pub const NS_CPACR_ADDR: u32 = 0xE002_ED88;
/// FPCCR — FP Context Control Register. Bit 0 = LSPACT (lazy active),
/// bit 30 = LSPEN (lazy enable), bit 31 = ASPEN (automatic state preservation).
pub const FPCCR_ADDR: u32 = 0xE000_EF34;
/// FPCAR — FP Context Address Register (8-byte aligned pointer at lazy save).
pub const FPCAR_ADDR: u32 = 0xE000_EF38;
/// NVIC ICPR0 — NVIC Interrupt Clear Pending Register 0. Used by the
/// cleanup block to drop any IRQs the scenarios may have armed. RP2354
/// has more than 32 external IRQs, but v1 scenarios never touch any of
/// them — only SysTick and PendSV (both internal). Kept anyway so a
/// future scenario that arms a real IRQ has a documented place to
/// extend the cleanup.
pub const NVIC_ICPR0_ADDR: u32 = 0xE000_E280;

/// ICSR.PENDSVSET bit (pend PendSV exception).
pub const ICSR_PENDSVSET: u32 = 1 << 28;
/// ICSR.PENDSTSET bit (pend SysTick exception).
pub const ICSR_PENDSTSET: u32 = 1 << 26;
/// CPACR bits enabling full access to CP10 + CP11 (FPU).
pub const CPACR_CP10_CP11_FULL: u32 = (0b11 << 20) | (0b11 << 22);
/// FPCCR.LSPACT — lazy FP save is active (set at exception entry when
/// LSPEN=1 and CONTROL.FPCA=1).
pub const FPCCR_LSPACT: u32 = 1 << 0;
/// FPCCR.LSPEN — lazy FP save enabled (reset value).
pub const FPCCR_LSPEN: u32 = 1 << 30;
/// FPCCR.ASPEN — automatic state preservation (reset value).
pub const FPCCR_ASPEN: u32 = 1 << 31;

/// SysTick CSR / RVR / CVR addresses (core-local). Used by the tail-chain
/// scenario to pre-arm a 4-cycle reload.
pub const SYST_CSR_ADDR: u32 = 0xE000_E010;
pub const SYST_RVR_ADDR: u32 = 0xE000_E014;
pub const SYST_CVR_ADDR: u32 = 0xE000_E018;
/// SYST_CSR bits: ENABLE | TICKINT | CLKSOURCE (processor clock).
pub const SYST_CSR_ENABLE_TICKINT_CORE: u32 = 0b111;

// ---------------------------------------------------------------------------
// Scenario type
// ---------------------------------------------------------------------------

/// Which register to prime on entry.
///
/// R0..R12 go through probe-rs `write_core_reg` on the HW side and
/// `core_mut(0).regs.r[n]` on the EMU side. MSP/PSP/CONTROL/CPACR/VTOR
/// are MMIO or architecturally-addressed; the runner knows how to route
/// each to the right write path for HW / EMU.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum IsrReg {
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    /// Main Stack Pointer.
    Msp,
    /// Process Stack Pointer.
    Psp,
    /// CONTROL register (thread mode priv + SPSEL + FPCA).
    Control,
    /// CPACR — FP access enable. Written to both S and NS aliases.
    Cpacr,
    /// VTOR — vector table offset. Written to both S and NS aliases.
    Vtor,
}

/// Slot in the stacked exception frame.
///
/// Basic frame layout (8 words at MSP after exception entry):
///
/// ```text
///   [0] R0
///   [4] R1
///   [8] R2
///  [12] R3
///  [16] R12
///  [20] LR
///  [24] ReturnAddress (PC)
///  [28] xPSR
/// ```
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StackedReg {
    R0,
    R1,
    R2,
    R3,
    R12,
    Lr,
    Pc,
    Xpsr,
}

impl StackedReg {
    /// Byte offset of this slot within the basic stacked frame.
    pub fn offset(self) -> u32 {
        match self {
            StackedReg::R0 => 0,
            StackedReg::R1 => 4,
            StackedReg::R2 => 8,
            StackedReg::R3 => 12,
            StackedReg::R12 => 16,
            StackedReg::Lr => 20,
            StackedReg::Pc => 24,
            StackedReg::Xpsr => 28,
        }
    }
}

/// Observable the runner reads post-BKPT on both HW and EMU, then
/// diffs. First mismatch wins.
#[derive(Copy, Clone, Debug)]
pub enum IsrObservable {
    /// Absolute-address MMIO read, masked by the second field.
    Mmio(u32, u32),
    /// Word at `MSP + StackedReg::offset()`. MSP is read post-halt
    /// (after the handler's BKPT #0) — the handler enters on MSP for
    /// all v1 scenarios, so the frame sits where the exception entry
    /// wrote it.
    Stacked(StackedReg),
    /// The u32 the handler stored into `ISR_MAILBOX_CYCCNT`. Reads 0
    /// if the handler never reached its CYCCNT store — that itself is
    /// a useful failure signal (the exception never fired, or the
    /// handler BKPT'd before the store).
    CycleDelta,
}

/// A single ISR oracle scenario. All fields are `&'static` so the
/// catalogue stays pure data; the runner is the only code path.
pub struct IsrScenario {
    pub name: &'static str,
    /// Bytes to upload at `ISR_IMAGE_BASE`. Follows the fixed image
    /// layout documented in `wrk_docs/2026.04.15 - HLD - test_silicon
    /// Orchestrator and Coverage Expansion.md` §"Image layout":
    /// vector table at +0x000, default handler at +0x040, PendSV /
    /// SysTick handler at +0x044, main routine at +0x100, literal pool
    /// at the end.
    pub image: &'static [u8],
    /// Offset of the main routine's first instruction (relative to
    /// `ISR_IMAGE_BASE`). Runner sets PC = `ISR_IMAGE_BASE +
    /// entry_offset` before resuming. For the v1 catalogue this is
    /// always 0x100 (the fixed main_offset), but kept configurable
    /// so a future scenario with a taller handler can shift main.
    pub entry_offset: u32,
    /// Pre-resume register snapshot. Applied in order — later entries
    /// win if a register is listed twice.
    pub init_regs: &'static [(IsrReg, u32)],
    /// Maximum sysclks the main + handler are allowed to run before
    /// BKPT halts the core. On HW this backs a wall-clock timeout; on
    /// EMU this drives `Emulator::step` count. Over-bound = TIMEOUT
    /// failure (no diff attempted).
    pub max_sysclks: u32,
    /// Observables. `(label, observable)` — label used in diff messages.
    pub observe: &'static [(&'static str, IsrObservable)],
}

// ---------------------------------------------------------------------------
// Image layout helper constants (match HLD §"Image layout")
// ---------------------------------------------------------------------------

/// Byte offset within an image of the PendSV/SysTick handler body.
/// Fixed convention — every scenario's image places the handler here so
/// the vector-table slot points at the same address regardless of how
/// tall the handler is.
pub const HANDLER_OFFSET: u32 = 0x044;
/// Byte offset within an image of the main routine's first instruction.
/// 0x100 is enough headroom above HANDLER_OFFSET to fit any plausible
/// handler body; v1 scenarios all settle well under 0xC0.
pub const MAIN_OFFSET: u32 = 0x100;

// Re-export `ISR_DEFAULT_HANDLER_OFF` from the crate root via the runner;
// the catalogue itself never needs the numeric value — it always writes
// `0xBE01` (bkpt #1) at the canonical offset during image construction.

// ---------------------------------------------------------------------------
// Image builders
// ---------------------------------------------------------------------------

/// Image size shared by all v1 scenarios. 0x180 bytes = 0x100 prologue
/// (vector table + default handler + handler stub + padding) + 0x80 of
/// main routine + literal pool. Kept constant so the runner's upload
/// path doesn't need scenario-specific sizing.
pub const ISR_IMAGE_SIZE: usize = 0x180;

/// Assemble a complete scenario image from its hand-assembled handler
/// body, main body, and literal pool.
///
/// Layout:
///
/// ```text
///   + 0x000 .. 0x040  vector table
///                       [0] initial MSP      = ISR_STACK_TOP
///                       [1] Reset_Handler    = base | 1 | MAIN_OFFSET
///                       [2..13] default      = base | 1 | 0x040
///                       [14] PendSV          = base | 1 | HANDLER_OFFSET
///                       [15] SysTick         = base | 1 | HANDLER_OFFSET
///   + 0x040 .. 0x044  default handler: bkpt #1
///   + 0x044 .. 0x100  handler body (copied from `handler_hw`)
///   + 0x100 .. 0x180  main body + literal pool
/// ```
///
/// `handler_hw` is the Thumb halfword stream for the PendSV/SysTick
/// handler body (starting at offset 0x044). `main_hw` is the main
/// routine body including any trailing literal pool (starting at 0x100,
/// must fit within 0x80 bytes).
///
/// Any unused bytes are zero-padded. The function is `const` only as
/// far as byte concatenation is concerned; in practice it's invoked at
/// catalogue-construction time to produce the `&'static [u8]` images.
const fn build_image<const N_HANDLER_HW: usize, const N_MAIN_HW: usize>(
    image_base: u32,
    stack_top: u32,
    handler_hw: [u16; N_HANDLER_HW],
    main_hw: [u16; N_MAIN_HW],
) -> [u8; ISR_IMAGE_SIZE] {
    let mut out = [0u8; ISR_IMAGE_SIZE];

    // Vector table (16 entries × 4 bytes = 64 bytes).
    //   [0] initial MSP
    //   [1] Reset_Handler
    //   [2..13] default handler for all others
    //   [14] PendSV
    //   [15] SysTick
    let reset_vec = (image_base + MAIN_OFFSET) | 1;
    let default_vec = (image_base + 0x040) | 1;
    let pendsv_vec = (image_base + HANDLER_OFFSET) | 1;
    let systick_vec = (image_base + HANDLER_OFFSET) | 1;

    // Word 0: initial MSP
    let msp_bytes = stack_top.to_le_bytes();
    out[0] = msp_bytes[0];
    out[1] = msp_bytes[1];
    out[2] = msp_bytes[2];
    out[3] = msp_bytes[3];

    // Word 1: Reset_Handler
    let rv = reset_vec.to_le_bytes();
    out[4] = rv[0];
    out[5] = rv[1];
    out[6] = rv[2];
    out[7] = rv[3];

    // Words 2..13: default handler
    let mut i = 2;
    while i < 14 {
        let off = i * 4;
        let b = default_vec.to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        out[off + 2] = b[2];
        out[off + 3] = b[3];
        i += 1;
    }

    // Word 14: PendSV
    let pv = pendsv_vec.to_le_bytes();
    out[56] = pv[0];
    out[57] = pv[1];
    out[58] = pv[2];
    out[59] = pv[3];

    // Word 15: SysTick
    let sv = systick_vec.to_le_bytes();
    out[60] = sv[0];
    out[61] = sv[1];
    out[62] = sv[2];
    out[63] = sv[3];

    // Default handler at offset 0x040: bkpt #1 (0xBE01). A halt here
    // means the trigger hit the wrong vector slot — distinct halt
    // reason from the expected `bkpt #0` inside the real handler.
    out[0x040] = 0x01;
    out[0x041] = 0xBE;
    out[0x042] = 0x00;
    out[0x043] = 0x00; // padding to keep handler 4-byte aligned

    // Handler body at offset 0x044.
    let mut h = 0;
    while h < N_HANDLER_HW {
        let off = 0x044 + h * 2;
        let b = handler_hw[h].to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        h += 1;
    }

    // Main body at offset 0x100.
    let mut m = 0;
    while m < N_MAIN_HW {
        let off = 0x100 + m * 2;
        let b = main_hw[m].to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        m += 1;
    }

    out
}

// ---------------------------------------------------------------------------
// Hand-assembled handlers
// ---------------------------------------------------------------------------

// Each handler body is placed at offset 0x044 within the image. The
// handler is entered with LR = EXC_RETURN (architectural) and MSP
// pointing at the freshly stacked exception frame.
//
// Shared handler shape (baseline + tail-chain scenarios):
//
// ```text
//   [0] ldr r0, [pc, #8]     ; r0 = DWT_CYCCNT (from literal)
//   [1] ldr r0, [r0]         ; r0 = current CYCCNT
//   [2] ldr r1, [pc, #8]     ; r1 = ISR_MAILBOX_CYCCNT (from literal)
//   [3] str r0, [r1]         ; mailbox[CYCCNT] = r0
//   [4] bkpt #0              ; halt for host readback
//   [5] bkpt #0              ; padding for literal alignment (4-byte)
//   [6..7] literal: DWT_CYCCNT_ADDR (0xE000_1004)
//   [8..9] literal: ISR_MAILBOX_CYCCNT
// ```
//
// Literal-pool math:
//   `ldr r0, [pc, #8]` at halfword [0]:
//     instr_addr = 0x044, PC = 0x048, Align(PC, 4) = 0x048,
//     target = 0x048 + 8 = 0x050 → halfword [6] (0x050 - 0x044 = 0x0C = 12,
//     12 / 2 = 6). Correct.
//   `ldr r1, [pc, #8]` at halfword [2]:
//     instr_addr = 0x048, PC = 0x04C, Align(PC, 4) = 0x04C,
//     target = 0x04C + 8 = 0x054 → halfword [8]. Correct.
//
// Encoding: `ldr rD, [pc, #imm8*4]` T1 = 0b01001_RRR_IIIIIIII.
//   `ldr r0, [pc, #8]` → imm8 = 2 → 0x4802.
//   `ldr r1, [pc, #8]` → imm8 = 2 → 0x4902.
//
// Per-scenario handler bodies replace just the first few halfwords
// when they need to observe FPCCR etc. without any FP instruction.

/// Baseline handler: read CYCCNT, mailbox it, BKPT #0.
///
/// Used by `isr_pendsv_cold` and `isr_tail_chain_pendsv_systick`.
/// Critical invariants:
///   - Touches NO FP register: this scenario specifically tests cold
///     (non-FP) ISR entry; firing an FP op here would dirty CONTROL.FPCA
///     on the way back out and corrupt the baseline measurement.
///   - Exits via BKPT #0, not a valid EXC_RETURN: v1 does NOT test
///     exception return; the handler halts for readback and the runner
///     resets the core between scenarios.
const HANDLER_BASELINE: [u16; 10] = [
    0x4802, //  [0] ldr r0, [pc, #8]    — r0 = DWT_CYCCNT_ADDR
    0x6800, //  [1] ldr r0, [r0]        — r0 = *DWT_CYCCNT (CYCCNT value)
    0x4902, //  [2] ldr r1, [pc, #8]    — r1 = ISR_MAILBOX_CYCCNT
    0x6008, //  [3] str r0, [r1]        — mailbox[CYCCNT] = r0
    0xBE00, //  [4] bkpt #0             — halt for host readback
    0xBE00, //  [5] bkpt #0             — padding (literal pool at +8)
    0x1004, //  [6] lit: DWT_CYCCNT_ADDR low  = 0xE000_1004 & 0xFFFF
    0xE000, //  [7] lit: DWT_CYCCNT_ADDR high = 0xE000_1004 >> 16
    0x3FF8, //  [8] lit: ISR_MAILBOX_CYCCNT low  = 0x2000_3FF8 & 0xFFFF
    0x2000, //  [9] lit: ISR_MAILBOX_CYCCNT high = 0x2000_3FF8 >> 16
];

/// Lazy FP handler: read FPCCR via MMIO (NO FP instruction) and BKPT.
///
/// **Critical lazy-FP trap avoidance.** On M33 with LSPEN=1 and
/// CONTROL.FPCA=1 at exception entry, FPCCR.LSPACT is set but S0–S15
/// are NOT flushed to the reserved FP stack region — the flush happens
/// on the first FP op executed in the handler. If the handler reads
/// FPCCR via a VMRS or touches any S register, the lazy save fires and
/// LSPACT is cleared, defeating the observable we're trying to check.
/// Reading FPCCR via plain `ldr` from `0xE000_EF34` is a bus access,
/// not an FP instruction, so LSPACT stays set.
///
/// Body:
/// ```text
///   [0] ldr r0, [pc, #16]    ; r0 = FPCCR_ADDR
///   [1] ldr r0, [r0]         ; r0 = *FPCCR
///   [2] ldr r1, [pc, #16]    ; r1 = FPCAR_ADDR
///   [3] ldr r1, [r1]         ; r1 = *FPCAR
///   [4] ldr r2, [pc, #16]    ; r2 = ISR_MAILBOX_CYCCNT (mailbox r0 here)
///   [5] str r0, [r2]         ; mailbox[0] = FPCCR snapshot
///   [6] bkpt #0              ; halt for host readback
///   [7] bkpt #0              ; padding
///   [8..9] lit: FPCCR_ADDR       (0xE000_EF34)
///   [10..11] lit: FPCAR_ADDR     (0xE000_EF38)
///   [12..13] lit: ISR_MAILBOX_CYCCNT
/// ```
///
/// Literal math for `ldr r0, [pc, #16]` at halfword [0]:
///   PC = 0x048, Align(PC, 4) = 0x048, target = 0x048 + 16 = 0x058 →
///   halfword (0x058 - 0x044)/2 = 10. Wait — let me recompute: we want
///   halfword [8]. 0x044 + 8*2 = 0x054. So offset from PC = 0x054 -
///   0x048 = 0x0C = 12 → imm8=3 → encoding 0x4803. Let me redo below.
///
/// Corrected encoding:
///   [0] `ldr r0, [pc, #12]` — imm8=3 → 0x4803 — target halfword [8]
///   [2] `ldr r1, [pc, #12]` — imm8=3 → 0x4903 — instr_addr=0x048,
///       PC=0x04C, target = 0x04C + 12 = 0x058 → halfword [10]
///   [4] `ldr r2, [pc, #12]` — imm8=3 → 0x4A03 — instr_addr=0x04C,
///       PC=0x050, target = 0x050 + 12 = 0x05C → halfword [12]
///
/// The mailbox slot carries the FPCCR snapshot (not CYCCNT) for this
/// scenario — `CycleDelta` is not observed; the scenario uses
/// `Mmio(FPCCR_ADDR)` and `Mmio(FPCAR_ADDR)` directly.
const HANDLER_LAZY_FP_OBSERVE: [u16; 14] = [
    0x4803, //  [0] ldr r0, [pc, #12]   — r0 = FPCCR_ADDR
    0x6800, //  [1] ldr r0, [r0]        — r0 = *FPCCR (plain load, NOT VMRS)
    0x4903, //  [2] ldr r1, [pc, #12]   — r1 = FPCAR_ADDR
    0x6809, //  [3] ldr r1, [r1]        — r1 = *FPCAR
    0x4A03, //  [4] ldr r2, [pc, #12]   — r2 = ISR_MAILBOX_CYCCNT
    0x6010, //  [5] str r0, [r2]        — mailbox = FPCCR snapshot (for trace)
    0xBE00, //  [6] bkpt #0             — halt
    0xBE00, //  [7] bkpt #0             — padding
    0xEF34, //  [8] lit: FPCCR_ADDR  low  = 0xE000_EF34 & 0xFFFF
    0xE000, //  [9] lit: FPCCR_ADDR  high = 0xE000_EF34 >> 16
    0xEF38, // [10] lit: FPCAR_ADDR low  = 0xE000_EF38 & 0xFFFF
    0xE000, // [11] lit: FPCAR_ADDR high = 0xE000_EF38 >> 16
    0x3FF8, // [12] lit: ISR_MAILBOX_CYCCNT low
    0x2000, // [13] lit: ISR_MAILBOX_CYCCNT high
];

// ---------------------------------------------------------------------------
// Hand-assembled main routines
// ---------------------------------------------------------------------------

// Main routines all live at offset 0x100. They set up the trigger
// (ICSR.PENDSVSET or SYST_CSR enable), then either busy-wait until
// preempted, or BKPT #0 (the handler won't run if the exception doesn't
// dispatch, so the BKPT guards against a wedge).
//
// Register conventions inside main (all scenarios):
//   r4 — scratch
//   r5 — DWT_CYCCNT address
//   r6 — ICSR address
//   r7 — PENDSVSET constant
//
// Literal pool starts at halfword index 16 (main_offset = 0x100,
// literal starts at 0x120). Each literal is 4 bytes (two halfwords).

/// Baseline main: reset CYCCNT, pend PendSV, busy-wait.
///
/// ```text
///   [ 0] movs r4, #0
///   [ 1] ldr  r5, [pc, #24]     ; r5 = DWT_CYCCNT_ADDR  (lit at +28 bytes)
///   [ 2] str  r4, [r5]          ; CYCCNT = 0
///   [ 3] ldr  r6, [pc, #24]     ; r6 = SCB_ICSR_ADDR    (lit)
///   [ 4] ldr  r7, [pc, #24]     ; r7 = ICSR_PENDSVSET   (lit)
///   [ 5] str  r7, [r6]          ; *ICSR = PENDSVSET  -- TRIGGER
///   [ 6] b    .                 ; wait for PendSV
///   [ 7] bkpt #0                ; safety net (never reached if exception fires)
///   ... 8 halfwords of NOP pad to 16 hw = 0x20 bytes, literal pool at +32.
///   [ 8..15] NOP padding
///   [16..17] lit: DWT_CYCCNT_ADDR
///   [18..19] lit: SCB_ICSR_ADDR
///   [20..21] lit: ICSR_PENDSVSET
/// ```
///
/// Literal-pool math for `ldr r5, [pc, #24]` at halfword [1]:
///   instr_addr = 0x102, PC = 0x106, Align(PC, 4) = 0x104,
///   target = 0x104 + 24 = 0x11C → halfword (0x11C - 0x100) / 2 = 14.
///   We want halfword [16] (= 0x120). Let me fix: imm8 for target 0x120
///   is (0x120 - 0x104) = 0x1C = 28 → imm8 = 7 → 0x4D07.
///
/// Corrected math (imm8 multiplied by 4):
///   [1] `ldr r5, [pc, #imm8*4]` where target hw [16] = 0x120
///       instr_addr = 0x102, PC = 0x106, Align = 0x104
///       imm8*4 = 0x120 - 0x104 = 0x1C → imm8 = 7 → 0x4D07
///   [3] `ldr r6, [pc, #imm8*4]` where target hw [18] = 0x124
///       instr_addr = 0x106, PC = 0x10A, Align = 0x108
///       imm8*4 = 0x124 - 0x108 = 0x1C → imm8 = 7 → 0x4E07
///   [4] `ldr r7, [pc, #imm8*4]` where target hw [20] = 0x128
///       instr_addr = 0x108, PC = 0x10C, Align = 0x10C
///       imm8*4 = 0x128 - 0x10C = 0x1C → imm8 = 7 → 0x4F07
const MAIN_BASELINE: [u16; 22] = [
    0x2400, // [ 0] movs r4, #0
    0x4D07, // [ 1] ldr  r5, [pc, #28]     — r5 = DWT_CYCCNT_ADDR
    0x602C, // [ 2] str  r4, [r5]          — *CYCCNT = 0  (reset counter)
    0x4E07, // [ 3] ldr  r6, [pc, #28]     — r6 = SCB_ICSR_ADDR
    0x4F07, // [ 4] ldr  r7, [pc, #28]     — r7 = ICSR_PENDSVSET (0x1000_0000)
    0x6037, // [ 5] str  r7, [r6]          — *ICSR = PENDSVSET  (TRIGGER)
    0xE7FE, // [ 6] b    .                 — busy-wait (branch-to-self)
    0xBE00, // [ 7] bkpt #0                — safety net
    0xBF00, // [ 8] nop                    — literal-pool alignment padding
    0xBF00, // [ 9] nop
    0xBF00, // [10] nop
    0xBF00, // [11] nop
    0xBF00, // [12] nop
    0xBF00, // [13] nop
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0x1004, // [16] lit: DWT_CYCCNT_ADDR low  = 0xE000_1004 & 0xFFFF
    0xE000, // [17] lit: DWT_CYCCNT_ADDR high
    0xED04, // [18] lit: SCB_ICSR_ADDR  low  = 0xE000_ED04 & 0xFFFF
    0xE000, // [19] lit: SCB_ICSR_ADDR  high
    0x0000, // [20] lit: ICSR_PENDSVSET low  = 0x1000_0000 & 0xFFFF
    0x1000, // [21] lit: ICSR_PENDSVSET high
];

/// Lazy FP main: dirty V0 with a known sentinel BEFORE triggering
/// PendSV, then pend and busy-wait.
///
/// The `vmov s0, r4` instruction sets CONTROL.FPCA=1 which is the
/// architectural precondition for FPCCR.LSPACT to fire at exception
/// entry under LSPEN=1.
///
/// ```text
///   [ 0] movs r4, #0xCA           ; low byte of the sentinel
///   [ 1] lsls r4, r4, #8          ; r4 = 0xCA00
///   [ 2] adds r4, r4, #0xFE       ; r4 = 0xCAFE
///   [ 3] ee00_0a10                ; vmov s0, r4 — DIRTY the FP state
///       (Thumb-32 MCR form: `vmov s0, r4` encodes as EE00 0A10)
///   [ 4] movs r4, #0
///   [ 5] ldr  r5, [pc, #28]       ; r5 = DWT_CYCCNT_ADDR
///   [ 6] str  r4, [r5]            ; CYCCNT = 0
///   [ 7] ldr  r6, [pc, #28]       ; r6 = SCB_ICSR_ADDR
///   [ 8] ldr  r7, [pc, #28]       ; r7 = ICSR_PENDSVSET
///   [ 9] str  r7, [r6]            ; TRIGGER
///   [10] b    .                   ; wait
///   [11] bkpt #0                  ; safety net
///   [12..15] NOP pad
///   [16..17] lit: DWT_CYCCNT_ADDR
///   [18..19] lit: SCB_ICSR_ADDR
///   [20..21] lit: ICSR_PENDSVSET
/// ```
///
/// `vmov s0, r4` is a Thumb-32 instruction: hw0=0xEE00, hw1=0x4A10.
/// The Rt field of VMOV(R->S) T1 lives in hw1 bits[15:12]; for r4 that's
/// the leading `0x4` of `0x4A10`. A historical version of this image
/// used `0x0A10` (Rt=0, i.e. `vmov s0, r0`), which silently fed
/// whatever r0 happened to contain into s0 instead of the CAFE sentinel.
/// We split the wide instruction into two halfwords in the image stream.
///
/// Literal-pool math shifts because the image starts with an extra 4
/// halfwords of sentinel setup + 1 wide Thumb-32 (2 hws). So halfword
/// [5] = `ldr r5, [pc, #imm]`:
///   instr_addr = 0x100 + 5*2 = 0x10A, PC = 0x10E, Align = 0x10C
///   target hw [16] = 0x100 + 16*2 = 0x120
///   imm8*4 = 0x120 - 0x10C = 0x14 → imm8 = 5 → 0x4D05
///
/// Similarly:
///   [8] `ldr r6, [pc, #imm]` at instr 0x110, PC=0x114, Align=0x114
///       target hw [18] = 0x124 → imm8*4 = 0x10 → imm8 = 4 → 0x4E04
///   [9] `ldr r7, [pc, #imm]` at instr 0x112, PC=0x116, Align=0x114
///       target hw [20] = 0x128 → imm8*4 = 0x14 → imm8 = 5 → 0x4F05
const MAIN_LAZY_FP: [u16; 22] = [
    0x24CA, // [ 0] movs r4, #0xCA         — sentinel byte
    0x0224, // [ 1] lsls r4, r4, #8        — r4 = 0xCA00
    0x34FE, // [ 2] adds r4, #0xFE         — r4 = 0xCAFE
    0xEE00, // [ 3] hw0: vmov s0, r4       — Thumb-32 MCR form
    0x4A10, // [ 4] hw1: vmov s0, r4       — (Rt=4; sets CONTROL.FPCA = 1)
    0x4D05, // [ 5] ldr  r5, [pc, #20]     — r5 = DWT_CYCCNT_ADDR
    0x2400, // [ 6] movs r4, #0
    0x602C, // [ 7] str  r4, [r5]          — *CYCCNT = 0 (reset)
    0x4E04, // [ 8] ldr  r6, [pc, #16]     — r6 = SCB_ICSR_ADDR
    0x4F05, // [ 9] ldr  r7, [pc, #20]     — r7 = ICSR_PENDSVSET
    0x6037, // [10] str  r7, [r6]          — *ICSR = PENDSVSET (TRIGGER)
    0xE7FE, // [11] b    .                 — busy-wait
    0xBE00, // [12] bkpt #0                — safety net
    0xBF00, // [13] nop                    — padding
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0x1004, // [16] lit: DWT_CYCCNT_ADDR low
    0xE000, // [17] lit: DWT_CYCCNT_ADDR high
    0xED04, // [18] lit: SCB_ICSR_ADDR low
    0xE000, // [19] lit: SCB_ICSR_ADDR high
    0x0000, // [20] lit: ICSR_PENDSVSET low
    0x1000, // [21] lit: ICSR_PENDSVSET high
];

/// Eager FP main. Identical to MAIN_LAZY_FP — the eager-vs-lazy
/// distinction is made by FPCCR.LSPEN, which the runner programs via
/// MMIO during scenario setup (not inside the main routine). Sharing
/// the same main body keeps the literal-pool offsets identical.
const MAIN_EAGER_FP: [u16; 22] = MAIN_LAZY_FP;

/// Tail-chain main: arm SysTick, pend PendSV, busy-wait.
///
/// SysTick is pre-armed via MMIO in `init_regs` (RVR=4, CVR=0, CSR
/// enabled) so the first SysTick underflow fires within a handful of
/// cycles after main begins. PendSV is pended immediately; both
/// exceptions race and should tail-chain on silicon (not modelled on
/// EMU today).
///
/// Same code as MAIN_BASELINE — the scenario difference is in the
/// init_regs programming.
const MAIN_TAIL_CHAIN: [u16; 22] = MAIN_BASELINE;

// ---------------------------------------------------------------------------
// Scenario images (built at compile time from handlers + mains)
// ---------------------------------------------------------------------------

use crate::{ISR_IMAGE_BASE, ISR_MAILBOX_CYCCNT, ISR_STACK_TOP};

const IMAGE_PENDSV_COLD: [u8; ISR_IMAGE_SIZE] =
    build_image(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_BASELINE, MAIN_BASELINE);

const IMAGE_LAZY_FP_SAVE: [u8; ISR_IMAGE_SIZE] =
    build_image(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_LAZY_FP_OBSERVE, MAIN_LAZY_FP);

const IMAGE_EAGER_FP_SAVE: [u8; ISR_IMAGE_SIZE] =
    build_image(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_BASELINE, MAIN_EAGER_FP);

const IMAGE_TAIL_CHAIN: [u8; ISR_IMAGE_SIZE] =
    build_image(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_BASELINE, MAIN_TAIL_CHAIN);

// ---------------------------------------------------------------------------
// Observables + init_regs per scenario
// ---------------------------------------------------------------------------

// -- Scenario 1: isr_pendsv_cold --
const INIT_PENDSV_COLD: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
];
const OBS_PENDSV_COLD: &[(&str, IsrObservable)] = &[
    // The CYCCNT delta is the load-bearing baseline — everything else
    // compares against it. Cold entry on M33 is ~12 cycles (exception-
    // entry base) plus handler prologue. HW and EMU must match; a delta
    // here means our exception-entry cycle accounting is wrong.
    ("cyccnt_delta", IsrObservable::CycleDelta),
    // Stacked PC = the instruction that was about to execute when the
    // exception fired. For the baseline scenario, main busy-waits at
    // offset 0x10C (the `b .` at halfword [6]); HW and EMU must agree.
    ("stacked_pc", IsrObservable::Stacked(StackedReg::Pc)),
    // Stacked xPSR top bits must include the PendSV exception number
    // (14) that was pending; the handler's IPSR reflects the active
    // exception, not the stacked frame. Mask the exception number
    // field via the Mmio path below instead.
    ("stacked_xpsr", IsrObservable::Stacked(StackedReg::Xpsr)),
];

// -- Scenario 2: isr_lazy_fp_save --
const INIT_LAZY_FP_SAVE: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
    // CPACR: enable CP10 + CP11 (FPU). Written to BOTH S and NS aliases
    // by the runner so the scenario works regardless of which security
    // state the core ends up in. Value 0x00F0_0000 = full access both.
    (IsrReg::Cpacr, CPACR_CP10_CP11_FULL),
];
const OBS_LAZY_FP_SAVE: &[(&str, IsrObservable)] = &[
    // FPCCR: LSPACT bit MUST be set (lazy save reserved the frame but
    // didn't flush S0-S15). Mask includes LSPACT (bit 0) + LSPEN
    // (bit 30) — both are part of the observable state.
    ("fpccr", IsrObservable::Mmio(FPCCR_ADDR, FPCCR_LSPACT | FPCCR_LSPEN | FPCCR_ASPEN)),
    // FPCAR: must be non-zero (points at the reserved 18-word FP region
    // inside the stacked frame). Runner compares the raw word, masking
    // the low 3 bits (FPCAR is 8-byte aligned).
    ("fpcar", IsrObservable::Mmio(FPCAR_ADDR, !0x7)),
    // Stacked basic-frame slot R0 — reads through because the basic
    // frame is always written, lazy FP only affects the *extended* region.
    ("stacked_r0", IsrObservable::Stacked(StackedReg::R0)),
];

// -- Scenario 3: isr_eager_fp_save --
const INIT_EAGER_FP_SAVE: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
    (IsrReg::Cpacr, CPACR_CP10_CP11_FULL),
    // FPCCR: clear LSPEN, keep ASPEN. After this write, exception
    // entry flushes S0-S15 immediately.
    //
    // NOTE: init_regs can't write FPCCR directly (no IsrReg::Fpccr
    // variant — writing FPCCR lives in the scenario-specific setup
    // block below). Handled via an extra Mmio MMIO write in the
    // runner's per-scenario preamble. For v1 we encode the FPCCR
    // write as a second Cpacr-like slot — no; the cleaner path is to
    // extend init_regs' range with an `Mmio(addr, val)` variant or
    // to hand-write FPCCR in the runner. Chosen: runner's preamble
    // detects the `isr_eager_fp_save` name and clears FPCCR.LSPEN
    // before resuming. Documented in run_against.
];
const OBS_EAGER_FP_SAVE: &[(&str, IsrObservable)] = &[
    // LSPACT MUST be clear (eager save already happened).
    ("fpccr", IsrObservable::Mmio(FPCCR_ADDR, FPCCR_LSPACT | FPCCR_LSPEN)),
    // FPCAR may be non-zero (exception entry still records it) but
    // this scenario doesn't care about its exact value.
    // Stacked basic-frame R0 — the runner primes R0 = 0x11111111 before
    // triggering so the stacked slot has a known sentinel.
    ("stacked_r0", IsrObservable::Stacked(StackedReg::R0)),
];

// -- Scenario 4: isr_tail_chain_pendsv_systick --
const INIT_TAIL_CHAIN: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
    // Runner's scenario-specific preamble also writes SYST_RVR = 4 and
    // sets SYST_CSR = ENABLE|TICKINT|CLKSOURCE so SysTick arms with a
    // fast-underflow period. v1 doesn't use init_regs to route MMIO
    // writes; the preamble handles it — documented in run_against.
];
const OBS_TAIL_CHAIN: &[(&str, IsrObservable)] = &[
    // CycleDelta is the oracle's load-bearing observable: tail-chained
    // entry should take noticeably fewer cycles than two independent
    // cold entries (exit + re-entry is optimised to a single chain).
    // The diff reports the raw value; interpretation is left to the
    // operator's catalogue note.
    ("cyccnt_delta", IsrObservable::CycleDelta),
    // Stacked PC — both PendSV and SysTick stack the same interrupted
    // address under tail-chain, so HW and EMU must agree.
    ("stacked_pc", IsrObservable::Stacked(StackedReg::Pc)),
];

// ---------------------------------------------------------------------------
// Catalogue
// ---------------------------------------------------------------------------

/// Initial catalogue. 4 scenarios per HLD §Component 2 §"Initial catalogue".
/// All names have the `isr_` prefix for orchestrator substring-uniqueness.
pub const SCENARIOS: &[IsrScenario] = &[
    IsrScenario {
        name: "isr_pendsv_cold",
        image: &IMAGE_PENDSV_COLD,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_PENDSV_COLD,
        max_sysclks: 800,
        observe: OBS_PENDSV_COLD,
    },
    IsrScenario {
        name: "isr_lazy_fp_save",
        image: &IMAGE_LAZY_FP_SAVE,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_LAZY_FP_SAVE,
        max_sysclks: 1000,
        observe: OBS_LAZY_FP_SAVE,
    },
    IsrScenario {
        name: "isr_eager_fp_save",
        image: &IMAGE_EAGER_FP_SAVE,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_EAGER_FP_SAVE,
        max_sysclks: 1200,
        observe: OBS_EAGER_FP_SAVE,
    },
    IsrScenario {
        name: "isr_tail_chain_pendsv_systick",
        image: &IMAGE_TAIL_CHAIN,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_TAIL_CHAIN,
        max_sysclks: 1200,
        observe: OBS_TAIL_CHAIN,
    },
];

// ---------------------------------------------------------------------------
// Runner (library API)
// ---------------------------------------------------------------------------

use crate::silicon_oracle::{
    self, enable_cyccnt, reset_cyccnt, CaseOutcome, Verdict,
};
use mdrp2350::{Config, EmulatorBuilder};
use probe_rs::{Core, MemoryInterface, RegisterId};
use std::time::{Duration, Instant};

const PC_REG: RegisterId = RegisterId(15);
const XPSR_REG: RegisterId = RegisterId(16);
const SP_REG: RegisterId = RegisterId(13);
const LR_REG: RegisterId = RegisterId(14);
const MSP_REG: RegisterId = RegisterId(17);
const PSP_REG: RegisterId = RegisterId(18);
/// Cortex-M debug register id 20 = {CONTROL, FAULTMASK, BASEPRI, PRIMASK}
/// packed. Writing 0 zeroes all four — thread-mode, MSP, privileged, no FPCA.
const EXTRA_REG: RegisterId = RegisterId(0b10100);

const BKPT_TIMEOUT: Duration = Duration::from_secs(5);

/// Arguments for `run_against`. Shape mirrors `PeriphArgs`.
#[derive(Clone, Debug, Default)]
pub struct IsrArgs {
    pub filter: Option<String>,
    pub verbose: bool,
}

/// Map `IsrReg` to its probe-rs `RegisterId` where applicable. Returns
/// `None` for regs routed via MMIO (CPACR, VTOR) — the caller handles
/// those separately.
fn isr_reg_id(r: IsrReg) -> Option<RegisterId> {
    Some(match r {
        IsrReg::R0 => RegisterId(0),
        IsrReg::R1 => RegisterId(1),
        IsrReg::R2 => RegisterId(2),
        IsrReg::R3 => RegisterId(3),
        IsrReg::R4 => RegisterId(4),
        IsrReg::R5 => RegisterId(5),
        IsrReg::R6 => RegisterId(6),
        IsrReg::R7 => RegisterId(7),
        IsrReg::R8 => RegisterId(8),
        IsrReg::R9 => RegisterId(9),
        IsrReg::R10 => RegisterId(10),
        IsrReg::R11 => RegisterId(11),
        IsrReg::R12 => RegisterId(12),
        IsrReg::Msp => MSP_REG,
        IsrReg::Psp => PSP_REG,
        IsrReg::Control => EXTRA_REG,
        IsrReg::Cpacr | IsrReg::Vtor => return None,
    })
}

/// Apply init_regs on HW. CPACR / VTOR are routed via MMIO (both S and
/// NS aliases) so the scenario works regardless of security state.
fn apply_init_regs_hw(
    core: &mut Core,
    init_regs: &[(IsrReg, u32)],
) -> Result<(), Box<dyn std::error::Error>> {
    for &(reg, val) in init_regs {
        match reg {
            IsrReg::Cpacr => {
                core.write_word_32(S_CPACR_ADDR as u64, val)?;
                core.write_word_32(NS_CPACR_ADDR as u64, val)?;
            }
            IsrReg::Vtor => {
                core.write_word_32(S_VTOR_ADDR as u64, val)?;
                core.write_word_32(NS_VTOR_ADDR as u64, val)?;
            }
            r => {
                if let Some(id) = isr_reg_id(r) {
                    core.write_core_reg(id, val)?;
                }
            }
        }
    }
    Ok(())
}

/// Apply init_regs on EMU. Same routing contract as the HW path.
fn apply_init_regs_emu(emu: &mut mdrp2350::Emulator, init_regs: &[(IsrReg, u32)]) {
    for &(reg, val) in init_regs {
        match reg {
            IsrReg::Cpacr => {
                emu.mmio_write32(S_CPACR_ADDR, val);
                emu.mmio_write32(NS_CPACR_ADDR, val);
            }
            IsrReg::Vtor => {
                emu.mmio_write32(S_VTOR_ADDR, val);
                emu.mmio_write32(NS_VTOR_ADDR, val);
            }
            IsrReg::R0 => emu.core_mut(0).regs.r[0] = val,
            IsrReg::R1 => emu.core_mut(0).regs.r[1] = val,
            IsrReg::R2 => emu.core_mut(0).regs.r[2] = val,
            IsrReg::R3 => emu.core_mut(0).regs.r[3] = val,
            IsrReg::R4 => emu.core_mut(0).regs.r[4] = val,
            IsrReg::R5 => emu.core_mut(0).regs.r[5] = val,
            IsrReg::R6 => emu.core_mut(0).regs.r[6] = val,
            IsrReg::R7 => emu.core_mut(0).regs.r[7] = val,
            IsrReg::R8 => emu.core_mut(0).regs.r[8] = val,
            IsrReg::R9 => emu.core_mut(0).regs.r[9] = val,
            IsrReg::R10 => emu.core_mut(0).regs.r[10] = val,
            IsrReg::R11 => emu.core_mut(0).regs.r[11] = val,
            IsrReg::R12 => emu.core_mut(0).regs.r[12] = val,
            IsrReg::Msp => emu.core_mut(0).regs.msp = val,
            IsrReg::Psp => emu.core_mut(0).regs.psp = val,
            IsrReg::Control => emu.core_mut(0).regs.control = val,
        }
    }
}

/// Scenario-specific MMIO preamble. Handles the two cases that don't
/// fit the `init_regs` schema: eager-FP needs FPCCR.LSPEN cleared, and
/// tail-chain needs SysTick pre-armed.
fn scenario_preamble_hw(
    core: &mut Core,
    name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    match name {
        "isr_eager_fp_save" => {
            // Clear FPCCR.LSPEN. Keep ASPEN so the exception-entry path
            // still records FPCAR — it's the lazy-vs-eager split we're
            // testing, not ASPEN behaviour.
            let fpccr: u32 = core.read_word_32(FPCCR_ADDR as u64)?;
            core.write_word_32(
                FPCCR_ADDR as u64,
                (fpccr & !FPCCR_LSPEN) | FPCCR_ASPEN,
            )?;
        }
        "isr_tail_chain_pendsv_systick" => {
            // Pre-arm SysTick: RVR=4, CVR=0, then enable | tickint |
            // clksource. The underflow fires within a few cycles of the
            // main routine starting; PendSV is pended in main, giving the
            // tail-chain condition.
            core.write_word_32(SYST_RVR_ADDR as u64, 4)?;
            core.write_word_32(SYST_CVR_ADDR as u64, 0)?;
            core.write_word_32(SYST_CSR_ADDR as u64, SYST_CSR_ENABLE_TICKINT_CORE)?;
        }
        _ => {}
    }
    Ok(())
}

fn scenario_preamble_emu(emu: &mut mdrp2350::Emulator, name: &str) {
    match name {
        "isr_eager_fp_save" => {
            let fpccr = emu.mmio_read32(FPCCR_ADDR);
            emu.mmio_write32(FPCCR_ADDR, (fpccr & !FPCCR_LSPEN) | FPCCR_ASPEN);
        }
        "isr_tail_chain_pendsv_systick" => {
            emu.mmio_write32(SYST_RVR_ADDR, 4);
            emu.mmio_write32(SYST_CVR_ADDR, 0);
            emu.mmio_write32(SYST_CSR_ADDR, SYST_CSR_ENABLE_TICKINT_CORE);
        }
        _ => {}
    }
}

fn read_observable_hw(
    core: &mut Core,
    obs: IsrObservable,
    msp: u32,
) -> Result<u32, Box<dyn std::error::Error>> {
    Ok(match obs {
        IsrObservable::Mmio(addr, _mask) => core.read_word_32(addr as u64)?,
        IsrObservable::Stacked(slot) => core.read_word_32((msp + slot.offset()) as u64)?,
        IsrObservable::CycleDelta => core.read_word_32(ISR_MAILBOX_CYCCNT as u64)?,
    })
}

fn read_observable_emu(emu: &mut mdrp2350::Emulator, obs: IsrObservable, msp: u32) -> u32 {
    match obs {
        IsrObservable::Mmio(addr, _mask) => emu.mmio_read32(addr),
        IsrObservable::Stacked(slot) => emu.peek(msp + slot.offset()),
        IsrObservable::CycleDelta => emu.mmio_read32(ISR_MAILBOX_CYCCNT),
    }
}

fn observable_mask(obs: IsrObservable) -> u32 {
    match obs {
        IsrObservable::Mmio(_, mask) => mask,
        // Stacked reads compare full-word by default. Scenarios could
        // add per-slot masks if alignment bits matter; none do in v1.
        IsrObservable::Stacked(_) => !0,
        // CycleDelta is a raw cycle count — full-word compare.
        IsrObservable::CycleDelta => !0,
    }
}

/// Run one scenario end-to-end (HW + EMU) and produce the outcome.
fn run_one_scenario(
    core: &mut Core,
    sc: &IsrScenario,
    verbose: bool,
) -> Result<(Verdict, Option<String>, Duration), Box<dyn std::error::Error>> {
    let t0 = Instant::now();

    // --------------------------------------------------------------
    // HW side
    // --------------------------------------------------------------

    if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }

    // Upload the scenario image to SRAM.
    core.write_8(ISR_IMAGE_BASE as u64, sc.image)?;

    // Clear the mailbox so a stale value from a prior scenario can't
    // masquerade as a valid handler write.
    core.write_word_32(ISR_MAILBOX_CYCCNT as u64, 0)?;

    // Reset FPCCR to the architectural default (ASPEN=1, LSPEN=1,
    // LSPACT=0) before every scenario. Without this, an earlier lazy-FP
    // scenario that left LSPACT=1 can pollute a later scenario's
    // observables — e.g. scenario 2's lazy save asserts LSPACT, scenario
    // 3 runs without touching FPCCR, and its observable snapshot still
    // reports LSPACT=1 because nothing cleared it. The cleanup path at
    // `run_against` exit already does this once per invocation, but
    // here we do it per-scenario to decouple ordering.
    core.write_word_32(FPCCR_ADDR as u64, FPCCR_ASPEN | FPCCR_LSPEN)?;

    // Apply init_regs (includes VTOR + CPACR to both S/NS aliases).
    apply_init_regs_hw(core, sc.init_regs)?;

    // Scenario-specific MMIO preamble (eager-FP FPCCR clear,
    // tail-chain SysTick arm).
    scenario_preamble_hw(core, sc.name)?;

    // Prime PC / SP / xPSR / LR / EXTRA.
    core.write_core_reg(PC_REG, ISR_IMAGE_BASE + sc.entry_offset)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1 (Thumb)
    core.write_core_reg(SP_REG, ISR_STACK_TOP)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;
    // Clear CONTROL/PRIMASK/BASEPRI/FAULTMASK so the core runs in
    // privileged thread mode on MSP with interrupts enabled.
    core.write_core_reg(EXTRA_REG, 0u32)?;

    // Reset CYCCNT so the mailbox snapshot inside the handler reflects
    // the cost from main's reset store to handler entry.
    reset_cyccnt(core)?;

    core.run()?;

    // Wait for BKPT halt.
    let deadline = Instant::now() + BKPT_TIMEOUT;
    loop {
        if core.status()?.is_halted() {
            break;
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC_REG).unwrap_or(0xDEAD_BEEF);
            return Err(format!(
                "scenario '{}' BKPT timeout: PC=0x{pc:08X}",
                sc.name
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }

    // Read HW observables. MSP after exception entry points at the
    // stacked frame — exactly the location the Stacked(_) reads need.
    let hw_msp: u32 = core.read_core_reg(MSP_REG)?;
    let hw_obs: Vec<u32> = sc
        .observe
        .iter()
        .map(|(_, o)| read_observable_hw(core, *o, hw_msp))
        .collect::<Result<_, _>>()?;

    // --------------------------------------------------------------
    // EMU side — fresh emulator so scenarios don't interfere.
    // --------------------------------------------------------------

    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    emu.core_mut(1).halt();

    // Pin bus.active_core to core 0 before any harness MMIO. The PPB
    // state (SCB, NVIC, SysTick, FPCCR, MPU, fault regs) is per-core;
    // `Emulator::mmio_{read,write}32` dispatches via `bus.active_core`,
    // which only gets refreshed inside the `step()` loop. A fresh
    // emulator defaults to 0, but we set it explicitly so later code
    // (e.g. post-step observable reads) that expects core 0 state
    // doesn't accidentally hit core 1's PPB. Core 1 is halted for the
    // entire scenario, so there is no contention.
    emu.bus.set_active_core(0);

    // Upload image via poke (word-aligned) to avoid byte-by-byte
    // overhead. The image is always a multiple of 4 bytes.
    debug_assert_eq!(sc.image.len() % 4, 0, "image must be word-aligned");
    for chunk_off in (0..sc.image.len()).step_by(4) {
        let word = u32::from_le_bytes([
            sc.image[chunk_off],
            sc.image[chunk_off + 1],
            sc.image[chunk_off + 2],
            sc.image[chunk_off + 3],
        ]);
        emu.poke(ISR_IMAGE_BASE + chunk_off as u32, word);
    }
    // Invalidate any decoded-op cache entries — poke() bypasses the
    // bus write path, so executable SRAM must be flushed before step.
    emu.bus.invalidate_all();

    // Clear mailbox.
    emu.mmio_write32(ISR_MAILBOX_CYCCNT, 0);

    // Reset FPCCR to the architectural default (ASPEN=1, LSPEN=1,
    // LSPACT=0) before every scenario. Mirrors the HW-side reset; keeps
    // scenario ordering from leaking lazy-FP state across cases.
    emu.mmio_write32(FPCCR_ADDR, FPCCR_ASPEN | FPCCR_LSPEN);

    // init_regs for EMU.
    apply_init_regs_emu(&mut emu, sc.init_regs);
    scenario_preamble_emu(&mut emu, sc.name);

    // Enable DWT CYCCNT on EMU side (mirrors the HW-side enable done
    // once at oracle startup).
    let demcr = emu.mmio_read32(silicon_oracle::DEMCR_U32);
    emu.mmio_write32(silicon_oracle::DEMCR_U32, demcr | silicon_oracle::TRCENA);
    let dwt_ctrl = emu.mmio_read32(silicon_oracle::DWT_CTRL_U32);
    emu.mmio_write32(silicon_oracle::DWT_CTRL_U32, dwt_ctrl | silicon_oracle::CYCCNTENA);
    emu.mmio_write32(silicon_oracle::DWT_CYCCNT_ADDR, 0);

    // Prime core-0 state on EMU.
    {
        let c = emu.core_mut(0);
        c.wake();
        c.regs.set_pc(ISR_IMAGE_BASE + sc.entry_offset);
        c.regs.xpsr = 0x0100_0000;
        c.regs.r[13] = ISR_STACK_TOP;
        c.regs.msp = ISR_STACK_TOP;
        c.regs.r[14] = 0xFFFF_FFFF;
        c.regs.control = 0;
        c.regs.primask = 0;
        c.regs.basepri = 0;
        c.regs.faultmask = 0;
    }

    // Step until any core halts (BKPT) or we exhaust the max_sysclks
    // budget. The handler's BKPT #0 puts core 0 into the halted state,
    // which `is_halted()` picks up.
    let budget = sc.max_sysclks as u64;
    let start_cycles = emu.cycles();
    while !emu.core(0).is_halted() && emu.cycles().saturating_sub(start_cycles) < budget {
        emu.step();
    }
    // Force-halt regardless — if the budget ran out without a BKPT,
    // the observables below will register a divergence (mailbox=0,
    // MSP unchanged).
    emu.core_mut(0).halt();

    // Restore `bus.active_core` to 0 before reading observables. The
    // `step()` loop iterates both cores and leaves `active_core` at
    // whichever core ran last (core 1 for two-iteration quanta, even
    // though its inner loop was a no-op because it's halted). PPB
    // observables (FPCCR, SysTick regs) must route to core 0 where the
    // scenario actually executed — otherwise `mmio_read32(FPCCR_ADDR)`
    // returns core 1's untouched default and falsely diffs against HW.
    emu.bus.set_active_core(0);

    let emu_msp = emu.core(0).regs.msp;
    let emu_obs: Vec<u32> = sc
        .observe
        .iter()
        .map(|(_, o)| read_observable_emu(&mut emu, *o, emu_msp))
        .collect();

    // --------------------------------------------------------------
    // Diff — first mismatch wins.
    // --------------------------------------------------------------

    let mut first_div: Option<String> = None;
    for (i, (label, obs)) in sc.observe.iter().enumerate() {
        let mask = observable_mask(*obs);
        let h = hw_obs[i] & mask;
        let e = emu_obs[i] & mask;
        if h != e {
            let msg = format!(
                "{label} (HW=0x{h:08X} EMU=0x{e:08X} xor=0x{:08X} mask=0x{mask:08X})",
                h ^ e
            );
            if first_div.is_none() {
                first_div = Some(msg.clone());
            }
            if verbose {
                println!("    DIFF {msg}");
            }
        } else if verbose {
            println!("    ok   {label}: 0x{h:08X}");
        }
    }

    let verdict = if first_div.is_none() { Verdict::Pass } else { Verdict::Fail };
    Ok((verdict, first_div, t0.elapsed()))
}

/// Library entry point.
///
/// **Cleanup contract (HLD §Cross-oracle state-cleanup contract, isr
/// row)**: on exit — pass or fail — restore VTOR=0 (both S and NS),
/// clear NVIC ICPR0 pending bits, reset FPCCR to its default
/// (ASPEN=1, LSPEN=1, LSPACT=0), and clear CONTROL via EXTRA_REG.
/// Cleanup failures are logged to stderr but do not alter the return.
///
/// Preconditions: `core` is live (auto-attached). The function halts
/// + enables CYCCNT on entry; individual scenarios reset CYCCNT
/// themselves via the main routine.
///
/// Case selection:
/// * `order = None` — run every scenario whose name matches
///   `args.filter`, in catalogue-declared order.
/// * `order = Some(&[name, …])` — run exactly those scenarios in that
///   order; `args.filter` is ignored. Unknown names skipped with one
///   `eprintln!` per name.
pub fn run_against(
    core: &mut Core,
    args: &IsrArgs,
    order: Option<&[&str]>,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error>> {
    // Ensure CYCCNT is enabled; harmless if already on.
    if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }
    enable_cyccnt(core)?;

    let selected: Vec<&IsrScenario> = match order {
        None => SCENARIOS
            .iter()
            .filter(|s| silicon_oracle::name_matches_filter(s.name, args.filter.as_deref()))
            .collect(),
        Some(names) => {
            let mut v: Vec<&IsrScenario> = Vec::with_capacity(names.len());
            for name in names {
                match SCENARIOS.iter().find(|s| s.name == *name) {
                    Some(sc) => v.push(sc),
                    None => eprintln!(
                        "isr_scenarios::run_against: unknown scenario '{name}' in order list; skipping",
                    ),
                }
            }
            v
        }
    };

    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(selected.len());
    let mut loop_err: Option<Box<dyn std::error::Error>> = None;

    for sc in &selected {
        match run_one_scenario(core, sc, args.verbose) {
            Ok((verdict, detail, elapsed)) => {
                let elapsed_ms = elapsed.as_millis().min(u32::MAX as u128) as u32;
                outcomes.push(match verdict {
                    Verdict::Pass => CaseOutcome::pass("isr", sc.name, elapsed_ms),
                    Verdict::Fail => CaseOutcome::fail(
                        "isr",
                        sc.name,
                        detail.unwrap_or_default(),
                        elapsed_ms,
                    ),
                });
            }
            Err(e) => {
                loop_err = Some(e);
                break;
            }
        }
    }

    // ------------------------------------------------------------------
    // Cleanup (HLD §Cross-oracle state-cleanup contract: isr row).
    // Runs unconditionally. Failures logged but do not alter the return.
    // ------------------------------------------------------------------
    if let Err(e) = core.halt(Duration::from_millis(200)) {
        eprintln!("warning: isr cleanup halt failed: {e}");
    }
    // Restore VTOR=0 (both S and NS aliases).
    if let Err(e) = core.write_word_32(S_VTOR_ADDR as u64, 0) {
        eprintln!("warning: isr cleanup S_VTOR write failed: {e}");
    }
    if let Err(e) = core.write_word_32(NS_VTOR_ADDR as u64, 0) {
        eprintln!("warning: isr cleanup NS_VTOR write failed: {e}");
    }
    // Clear NVIC pending bits (ICPR0). Write-1-to-clear — writing all-
    // ones drops every pending bit in the 32 IRQs covered. v1 never
    // touches non-SysTick/PendSV interrupts, but the write is idempotent
    // and documents the extension point.
    if let Err(e) = core.write_word_32(NVIC_ICPR0_ADDR as u64, 0xFFFF_FFFF) {
        eprintln!("warning: isr cleanup NVIC ICPR0 write failed: {e}");
    }
    // Restore FPCCR to ASPEN=1 | LSPEN=1 | LSPACT=0 (reset default).
    if let Err(e) = core.write_word_32(FPCCR_ADDR as u64, FPCCR_ASPEN | FPCCR_LSPEN) {
        eprintln!("warning: isr cleanup FPCCR write failed: {e}");
    }
    // Clear ICSR.PENDSVSET / PENDSTSET. ICSR has write-1-to-clear
    // semantics on bits 27 (PENDSVCLR) and 25 (PENDSTCLR); writing
    // both clears any lingering pend.
    if let Err(e) = core.write_word_32(SCB_ICSR_ADDR as u64, (1 << 27) | (1 << 25)) {
        eprintln!("warning: isr cleanup ICSR clear failed: {e}");
    }
    // Disable SysTick so a leftover tail-chain arm doesn't fire on the
    // next scenario.
    if let Err(e) = core.write_word_32(SYST_CSR_ADDR as u64, 0) {
        eprintln!("warning: isr cleanup SYST_CSR write failed: {e}");
    }
    // CONTROL=0 via EXTRA_REG — thread mode, MSP, privileged, no FPCA.
    if let Err(e) = core.write_core_reg(EXTRA_REG, 0u32) {
        eprintln!("warning: isr cleanup CONTROL write failed: {e}");
    }

    if let Some(e) = loop_err {
        return Err(e);
    }
    Ok(outcomes)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    // (1) Catalogue presence — exactly 4 scenarios, all `isr_*` prefix.
    #[test]
    fn test_catalogue_size_and_prefix() {
        assert_eq!(SCENARIOS.len(), 4, "catalogue must carry 4 scenarios in v1");
        for s in SCENARIOS {
            assert!(
                s.name.starts_with("isr_"),
                "scenario '{}' must start with 'isr_' prefix for substring-uniqueness",
                s.name,
            );
        }
    }

    // (2) Image-layout invariants — every image is ISR_IMAGE_SIZE bytes
    //     and the first word of the vector table = ISR_STACK_TOP (initial
    //     MSP), word 1 has Thumb LSB, word 14 (PendSV) has Thumb LSB,
    //     word 15 (SysTick) has Thumb LSB.
    #[test]
    fn test_image_layout_invariants() {
        for sc in SCENARIOS {
            assert_eq!(
                sc.image.len(),
                ISR_IMAGE_SIZE,
                "scenario '{}' image must be {} bytes",
                sc.name,
                ISR_IMAGE_SIZE,
            );

            // Word 0 — initial MSP.
            let msp = u32::from_le_bytes([
                sc.image[0],
                sc.image[1],
                sc.image[2],
                sc.image[3],
            ]);
            assert_eq!(
                msp, ISR_STACK_TOP,
                "scenario '{}' vector[0] must be ISR_STACK_TOP",
                sc.name,
            );

            // Word 1 — Reset_Handler. Thumb LSB set, base matches.
            let rv = u32::from_le_bytes([
                sc.image[4],
                sc.image[5],
                sc.image[6],
                sc.image[7],
            ]);
            assert_eq!(
                rv & 1,
                1,
                "scenario '{}' vector[1] missing Thumb LSB",
                sc.name,
            );
            assert_eq!(
                rv & !1,
                ISR_IMAGE_BASE + MAIN_OFFSET,
                "scenario '{}' vector[1] must point at main",
                sc.name,
            );

            // Word 14 — PendSV handler.
            let pv = u32::from_le_bytes([
                sc.image[56],
                sc.image[57],
                sc.image[58],
                sc.image[59],
            ]);
            assert_eq!(
                pv & 1,
                1,
                "scenario '{}' vector[14] (PendSV) missing Thumb LSB",
                sc.name,
            );
            assert_eq!(
                pv & !1,
                ISR_IMAGE_BASE + HANDLER_OFFSET,
                "scenario '{}' vector[14] must point at handler",
                sc.name,
            );

            // Word 15 — SysTick handler.
            let sv = u32::from_le_bytes([
                sc.image[60],
                sc.image[61],
                sc.image[62],
                sc.image[63],
            ]);
            assert_eq!(
                sv & 1,
                1,
                "scenario '{}' vector[15] (SysTick) missing Thumb LSB",
                sc.name,
            );

            // Default handler at offset 0x040 must be bkpt #1 (0xBE01).
            let dh = u16::from_le_bytes([
                sc.image[0x040],
                sc.image[0x041],
            ]);
            assert_eq!(
                dh, 0xBE01,
                "scenario '{}' default handler must be bkpt #1",
                sc.name,
            );
        }
    }

    // (3) Main routine fits within the image. Every scenario must have
    //     at least MAIN_OFFSET + 4 bytes of content (room for one
    //     instruction + terminator).
    #[test]
    fn test_main_routine_fits_image() {
        for sc in SCENARIOS {
            assert!(
                sc.image.len() >= (MAIN_OFFSET as usize) + 4,
                "scenario '{}' image too small to contain main at MAIN_OFFSET",
                sc.name,
            );
            // Main at MAIN_OFFSET must NOT be zero (catches a malformed
            // image where main never got written).
            let main_hw0 = u16::from_le_bytes([
                sc.image[MAIN_OFFSET as usize],
                sc.image[MAIN_OFFSET as usize + 1],
            ]);
            assert_ne!(
                main_hw0, 0,
                "scenario '{}' main routine starts with zero halfword",
                sc.name,
            );
        }
    }

    // (4) Substring-uniqueness within the catalogue. Orchestrator fires
    //     the whole-catalogue validator on boot; this one fires at
    //     `cargo test` time so a bad rename inside this module fails
    //     before the orchestrator ever sees it.
    #[test]
    fn test_catalogue_names_substring_unique() {
        let mut seen: HashSet<&str> = HashSet::new();
        for sc in SCENARIOS {
            assert!(seen.insert(sc.name), "duplicate isr scenario '{}'", sc.name);
        }
        for s1 in SCENARIOS {
            for s2 in SCENARIOS {
                if s1.name == s2.name {
                    continue;
                }
                assert!(
                    !s1.name.contains(s2.name),
                    "scenario '{}' is a substring of '{}'; filter aliasing",
                    s2.name, s1.name,
                );
            }
        }
    }

    // (5) VTOR address constants pinned. A future rename of the
    //     constant must update every scenario that relies on it.
    #[test]
    fn test_vtor_address_constants_pinned() {
        assert_eq!(S_VTOR_ADDR, 0xE000_ED08, "secure VTOR address");
        assert_eq!(NS_VTOR_ADDR, 0xE002_ED08, "non-secure VTOR address");
        assert_eq!(S_CPACR_ADDR, 0xE000_ED88, "secure CPACR address");
        assert_eq!(NS_CPACR_ADDR, 0xE002_ED88, "non-secure CPACR address");
        assert_eq!(FPCCR_ADDR, 0xE000_EF34, "FPCCR address");
        assert_eq!(FPCAR_ADDR, 0xE000_EF38, "FPCAR address");
    }

    // (6) Image base / stack top / mailbox constants form a coherent
    //     layout. Stack grows down from ISR_STACK_TOP; image ends well
    //     below ISR_STACK_TOP; mailbox sits above the stack window.
    #[test]
    fn test_image_constants_layout() {
        assert!(
            ISR_IMAGE_BASE + ISR_IMAGE_SIZE as u32 <= ISR_STACK_TOP,
            "image must end at or below ISR_STACK_TOP",
        );
        assert!(
            ISR_MAILBOX_CYCCNT >= ISR_STACK_TOP,
            "mailbox must live above the stack window",
        );
        // VTOR alignment — RP2350 M33 requires 128-byte alignment
        // (low 7 bits clear).
        assert_eq!(
            ISR_IMAGE_BASE & 0x7F,
            0,
            "ISR_IMAGE_BASE must be 128-byte aligned for VTOR",
        );
    }

    // (7) StackedReg offsets match the basic-frame layout.
    #[test]
    fn test_stacked_reg_offsets() {
        assert_eq!(StackedReg::R0.offset(), 0);
        assert_eq!(StackedReg::R1.offset(), 4);
        assert_eq!(StackedReg::R2.offset(), 8);
        assert_eq!(StackedReg::R3.offset(), 12);
        assert_eq!(StackedReg::R12.offset(), 16);
        assert_eq!(StackedReg::Lr.offset(), 20);
        assert_eq!(StackedReg::Pc.offset(), 24);
        assert_eq!(StackedReg::Xpsr.offset(), 28);
    }

    // (8) Every scenario's init_regs set includes VTOR — without it the
    //     scenario runs against the reset vector table and would
    //     never reach the in-image handler.
    #[test]
    fn test_every_scenario_sets_vtor() {
        for sc in SCENARIOS {
            let has_vtor = sc
                .init_regs
                .iter()
                .any(|(r, v)| *r == IsrReg::Vtor && *v == ISR_IMAGE_BASE);
            assert!(
                has_vtor,
                "scenario '{}' missing VTOR = ISR_IMAGE_BASE in init_regs",
                sc.name,
            );
        }
    }

    // (9) Handler at offset 0x044 must end in bkpt #0 somewhere in the
    //     first 12 halfwords — the runner relies on the handler halting.
    #[test]
    fn test_handler_contains_bkpt0() {
        for sc in SCENARIOS {
            let mut saw_bkpt0 = false;
            for hw in 0..12 {
                let off = (HANDLER_OFFSET as usize) + hw * 2;
                let half = u16::from_le_bytes([sc.image[off], sc.image[off + 1]]);
                if half == 0xBE00 {
                    saw_bkpt0 = true;
                    break;
                }
            }
            assert!(
                saw_bkpt0,
                "scenario '{}' handler body must contain bkpt #0",
                sc.name,
            );
        }
    }

    // (10) Main routine must contain a branch-to-self (`b .` = 0xE7FE)
    //      so if the exception never fires, main spins instead of
    //      falling through into the literal pool.
    #[test]
    fn test_main_contains_busy_wait() {
        for sc in SCENARIOS {
            let mut saw_busy_wait = false;
            for hw in 0..16 {
                let off = (MAIN_OFFSET as usize) + hw * 2;
                let half = u16::from_le_bytes([sc.image[off], sc.image[off + 1]]);
                if half == 0xE7FE {
                    saw_busy_wait = true;
                    break;
                }
            }
            assert!(
                saw_busy_wait,
                "scenario '{}' main routine must contain branch-to-self (0xE7FE)",
                sc.name,
            );
        }
    }

    // ---------------- Literal-pool encoding invariants (11) ----------------

    /// Walk `image[start..end]` and, for every T1 `ldr rD, [pc, #imm8*4]`
    /// encoding, return `(instr_byte_off, rD, target_byte_off, target_word)`.
    /// `start` / `end` are byte offsets within the image. The target
    /// address follows ARMv8-M semantics: `Align(instr_addr + 4, 4) +
    /// imm8*4`.
    ///
    /// Handles Thumb-32 wide instructions: a halfword whose top 5 bits
    /// are in {11101, 11110, 11111} is the high half of a 32-bit Thumb
    /// encoding — skip both it and the following halfword. Without this
    /// the walker would misparse `vmov s0, r4` (hw0=0xEE00 hw1=0x4A10)
    /// as a T1 LDR because 0x4A10 satisfies `(hw & 0xF800) == 0x4800`.
    fn collect_ldr_literal_loads(
        image: &[u8],
        start: usize,
        end: usize,
    ) -> Vec<(usize, u8, usize, u32)> {
        let mut out = Vec::new();
        let mut off = start;
        while off + 1 < end {
            let hw = u16::from_le_bytes([image[off], image[off + 1]]);
            // Thumb-32 wide-instruction prefix detection (ARMv8-M
            // A6.3): hw[15:13] == 111 and hw[12:11] != 00. I.e.
            // top 5 bits in {11101, 11110, 11111}.
            let top5 = (hw >> 11) & 0x1F;
            if top5 == 0b11101 || top5 == 0b11110 || top5 == 0b11111 {
                off += 4;
                continue;
            }
            // T1 LDR literal: top 5 bits = 01001
            if (hw & 0xF800) == 0x4800 {
                let rd = ((hw >> 8) & 0x7) as u8;
                let imm8 = (hw & 0xFF) as usize;
                let instr_addr = off;
                // Align(instr_addr + 4, 4)
                let pc_aligned = (instr_addr + 4) & !3;
                let target = pc_aligned + imm8 * 4;
                if target + 3 < image.len() {
                    let word = u32::from_le_bytes([
                        image[target],
                        image[target + 1],
                        image[target + 2],
                        image[target + 3],
                    ]);
                    out.push((instr_addr, rd, target, word));
                }
            }
            off += 2;
        }
        out
    }

    /// Every PC-relative LDR in the handler body (0x044..0x100) must
    /// target a word inside the handler's literal pool region, not
    /// anywhere else. We define the handler literal pool region as the
    /// tail of the handler body past the last BKPT padding halfword —
    /// concretely, `[0x050..0x100]` for the 10-halfword baseline
    /// handler and `[0x054..0x100]` for the 14-halfword observe
    /// handler. A stricter invariant would enumerate every `(hw, slot)`
    /// pair, but the catch we care about is "off-by-one-imm8 caused an
    /// LDR to resolve to the wrong literal"; bound-based checking is
    /// enough because the region beyond the pool is zero-padded and
    /// would never satisfy the expected-literal check in (12).
    #[test]
    fn test_handler_literal_loads_target_in_pool() {
        for sc in SCENARIOS {
            let loads = collect_ldr_literal_loads(
                sc.image,
                HANDLER_OFFSET as usize,
                MAIN_OFFSET as usize,
            );
            for (instr_off, rd, target, word) in loads {
                // Target must be word-aligned.
                assert_eq!(
                    target & 3,
                    0,
                    "scenario '{}' handler LDR at 0x{instr_off:03X} (r{rd}) \
                     computes non-word-aligned target 0x{target:03X}",
                    sc.name,
                );
                // Target must be inside handler body, past the handler
                // instructions. A handler body cannot load a literal
                // from the main routine (different PC base) without
                // being blatantly wrong.
                assert!(
                    target >= HANDLER_OFFSET as usize && target < MAIN_OFFSET as usize,
                    "scenario '{}' handler LDR at 0x{instr_off:03X} (r{rd}) \
                     resolves to 0x{target:03X} (word 0x{word:08X}) \
                     outside handler literal pool [0x{:03X}..0x{:03X})",
                    sc.name,
                    HANDLER_OFFSET,
                    MAIN_OFFSET,
                );
            }
        }
    }

    /// Every PC-relative LDR in the main body (0x100..image_end) must
    /// resolve to a word in the main routine's literal pool. Main
    /// routines in v1 place their pool at hw[16..22] (byte 0x120..0x12C)
    /// — the three scenario literals are DWT_CYCCNT_ADDR, SCB_ICSR_ADDR,
    /// and ICSR_PENDSVSET.
    ///
    /// This test is the specific regression check for the "0x4F04 vs
    /// 0x4F05" bug: an off-by-one imm8 in MAIN_LAZY_FP[9] would resolve
    /// to 0x124 (SCB_ICSR_ADDR) instead of 0x128 (ICSR_PENDSVSET),
    /// silently writing ICSR with the wrong value and firing NMI
    /// instead of PendSV. Both targets are inside the literal pool
    /// bounds, so bounds-only checking is not enough — this test uses
    /// the per-register expected literal to pin the exact word.
    #[test]
    fn test_main_literal_loads_match_expected() {
        // Per-register expected-literal map, shared by all v1 main
        // routines (they all reuse the same three literals).
        //
        // We match on the stored word, not on the slot index, because
        // the layout constants are scenario-wide.
        fn expected_for_reg(rd: u8) -> u32 {
            match rd {
                5 => 0xE000_1004, // DWT_CYCCNT_ADDR
                6 => 0xE000_ED04, // SCB_ICSR_ADDR
                7 => 0x1000_0000, // ICSR_PENDSVSET
                _ => unreachable!(
                    "unexpected LDR target register r{rd} in main routine"
                ),
            }
        }

        for sc in SCENARIOS {
            let loads = collect_ldr_literal_loads(
                sc.image,
                MAIN_OFFSET as usize,
                ISR_IMAGE_SIZE,
            );
            assert!(
                !loads.is_empty(),
                "scenario '{}' main routine has no LDR literal loads — \
                 literal pool walk is dead code",
                sc.name,
            );
            for (instr_off, rd, target, word) in loads {
                // Target must be word-aligned.
                assert_eq!(
                    target & 3,
                    0,
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     computes non-word-aligned target 0x{target:03X}",
                    sc.name,
                );
                // Target must lie inside the main routine's literal
                // pool, which starts after the final non-literal
                // halfword. Pool byte range: [0x120..0x12C].
                assert!(
                    (0x120..0x12C).contains(&target),
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     resolves to 0x{target:03X} outside the literal pool \
                     [0x120..0x12C)",
                    sc.name,
                );
                // The loaded word must match the per-register expected
                // literal. This catches the hw[9] 0x4F04 bug: imm8=4
                // would give target 0x124 = SCB_ICSR_ADDR, but r7's
                // expected literal is ICSR_PENDSVSET = 0x1000_0000.
                let expected = expected_for_reg(rd);
                assert_eq!(
                    word, expected,
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     loads 0x{word:08X}, expected 0x{expected:08X}",
                    sc.name,
                );
            }
        }
    }
}
