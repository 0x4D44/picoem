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
// **EMU dispatch status.** ICSR dispatch IS wired: `step()` calls
// `try_take_any_pending_exception` (exceptions.rs:422) at each
// instruction boundary (mod.rs:110), which polls ICSR.PENDSVSET /
// PENDSTSET / NMI / NVIC and takes the highest-priority pending
// exception. ICSR W1S/W1C write semantics are also implemented
// (ppb.rs:454-465). Remaining scenario FAILs surface real divergences
// in stacked-frame layout, FPCCR state, or CYCCNT delta — not a
// missing dispatch path. See tech_debt.md:314 ("Exception entry/exit
// not differentially validated") for the broader validation roadmap.

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

/// Extended image size for Phase 0a external-IRQ scenarios.
///
/// The baseline image reserves only 16 vector-table entries (64 bytes)
/// and places the default handler opcode at 0x040 — overlapping
/// external-IRQ vector slots 16+. External-IRQ scenarios need room for
/// the full NVIC input range 0..=47 (slots 16..=63) plus SPARE lines
/// up to IRQ 47, so this variant reserves the first 256 bytes (0x100)
/// for a 64-entry vector table and then lays out the default handler,
/// primary handler, alternate handler, and main body after it.
///
/// Layout:
///
/// ```text
///   + 0x000 .. 0x100  vector table — 64 entries, slots 0..=63
///                       [0] initial MSP = ISR_STACK_TOP
///                       [1] Reset_Handler = base | 1 | IRQ_MAIN_OFFSET
///                       [2..13] default handler = base | 1 | IRQ_DEFAULT_HANDLER_OFFSET
///                       [14] PendSV / [15] SysTick → IRQ_HANDLER_OFFSET
///                       [16..63] extra_slots populate via modeset;
///                                unpopulated slots fall through to the
///                                default handler (bkpt #1)
///   + 0x100 .. 0x104  default handler: bkpt #1 (0xBE01)
///   + 0x104 .. 0x140  primary handler body (30 halfwords max)
///   + 0x140 .. 0x180  alternate handler body (32 halfwords max)
///   + 0x180 .. 0x200  main body + literal pool (64 halfwords)
/// ```
pub const IRQ_IMAGE_SIZE: usize = 0x200;

/// Byte offset of the default handler (bkpt #1) inside an
/// `IRQ_IMAGE_SIZE` image.
pub const IRQ_DEFAULT_HANDLER_OFFSET: u32 = 0x100;
/// Byte offset of the primary handler body inside an `IRQ_IMAGE_SIZE`
/// image. Used for PendSV, SysTick, and any extra slots that point at
/// `IRQ_HANDLER_OFFSET`.
pub const IRQ_HANDLER_OFFSET: u32 = 0x104;
/// Byte offset of the alternate handler body inside an `IRQ_IMAGE_SIZE`
/// image. Scenarios that need a second handler body (e.g. priority
/// preemption with distinct low-priority and high-priority handlers)
/// place it here and list it in their `extra_slots`.
pub const IRQ_ALT_HANDLER_OFFSET: u32 = 0x140;
/// Byte offset of the main routine inside an `IRQ_IMAGE_SIZE` image.
pub const IRQ_MAIN_OFFSET: u32 = 0x180;

/// One vector-table slot populator for [`build_image_modeset`].
///
/// Each slot maps a vector-table index (word offset in the image's
/// first 0x100 bytes) to a handler offset within the image. The builder
/// writes `(image_base + handler_offset) | 1` into vector word `index`
/// so the entered handler has the Thumb bit set.
///
/// Index range is 0..=63 — the default image carries a 16-entry vector
/// table but `ISR_IMAGE_SIZE` leaves room for up to 64 vector entries
/// before colliding with the default-handler opcode at 0x040. Callers
/// that need more slots should bump `ISR_IMAGE_SIZE` first.
#[derive(Copy, Clone, Debug)]
pub struct VectorSlot {
    /// Vector-table word index (0..=63). Exceptions 14 (PendSV) and 15
    /// (SysTick) are populated by default even when `extra_slots` is
    /// empty — listing them here again overrides the handler offset.
    pub index: usize,
    /// Byte offset within the image where the handler body lives. The
    /// builder records `(image_base + handler_offset) | 1` in the
    /// vector slot.
    pub handler_offset: u32,
}

impl VectorSlot {
    /// Construct a slot. `const` so the mode-sets can live in `static`
    /// items.
    pub const fn new(index: usize, handler_offset: u32) -> Self {
        Self { index, handler_offset }
    }
}

/// Default mode-set for scenarios that touch only PendSV + SysTick —
/// preserves the v1 image shape. Every slot maps to [`HANDLER_OFFSET`]
/// so the single handler body `handler_hw` serves both.
pub const DEFAULT_MODESET: &[VectorSlot] = &[
    VectorSlot::new(14, HANDLER_OFFSET),
    VectorSlot::new(15, HANDLER_OFFSET),
];

/// Assemble a complete scenario image from its hand-assembled handler
/// body, main body, and literal pool — with no extra vector-slot
/// populators. Equivalent to [`build_image_modeset`] with
/// `extra_slots = &[]`, i.e. only PendSV (14) and SysTick (15) are
/// populated at `HANDLER_OFFSET`.
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
    build_image_modeset(image_base, stack_top, handler_hw, main_hw, &[])
}

/// Mode-set aware image builder (HLD V5 §4.1.6).
///
/// Produces the same layout as [`build_image`] but accepts an
/// `extra_slots` slice of additional vector-table entries to populate.
/// Use this variant for external-IRQ scenarios that need a second
/// handler body in the image (e.g. TIMER0_IRQ_0 alongside PendSV).
///
/// PendSV (14) and SysTick (15) always point at `HANDLER_OFFSET` —
/// extra slots may override them by listing the same index. The builder
/// applies `extra_slots` after the default PendSV/SysTick writes so
/// overrides win, and listing an index outside 0..=63 is a no-op (the
/// `while` guard clips it).
///
/// The handler body at `HANDLER_OFFSET` is shared by every slot that
/// references it, so external-IRQ scenarios that reuse the baseline
/// handler do not need to carry a second body. Scenarios that need a
/// distinct second handler can place its bytes further into the image
/// (e.g. at 0x080) and list that offset in `extra_slots`.
const fn build_image_modeset<const N_HANDLER_HW: usize, const N_MAIN_HW: usize>(
    image_base: u32,
    stack_top: u32,
    handler_hw: [u16; N_HANDLER_HW],
    main_hw: [u16; N_MAIN_HW],
    extra_slots: &[VectorSlot],
) -> [u8; ISR_IMAGE_SIZE] {
    let mut out = [0u8; ISR_IMAGE_SIZE];

    // Vector table defaults. PendSV (14) and SysTick (15) go through
    // the same `HANDLER_OFFSET` so the single shared handler body
    // services both.
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

    // Apply extra-slot populators from the mode-set. Each slot writes
    // a vector-table entry at `index * 4` pointing at
    // `(image_base + handler_offset) | 1`. Applied after the defaults
    // so a slot listing index 14 or 15 overrides PendSV / SysTick.
    let mut s = 0;
    while s < extra_slots.len() {
        let slot = extra_slots[s];
        if slot.index < 64 {
            let off = slot.index * 4;
            if off + 4 <= ISR_IMAGE_SIZE {
                let vec = (image_base + slot.handler_offset) | 1;
                let b = vec.to_le_bytes();
                out[off] = b[0];
                out[off + 1] = b[1];
                out[off + 2] = b[2];
                out[off + 3] = b[3];
            }
        }
        s += 1;
    }

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

/// Extended-layout image builder for Phase 0a external-IRQ scenarios
/// (HLD V5 §4.1.5 + §4.1.6).
///
/// Produces an [`IRQ_IMAGE_SIZE`] image with a 48-entry vector table,
/// supporting vector slots for IRQs 0..=31 without colliding with the
/// default-handler opcode. Callers supply:
///
/// * `handler_hw` — primary handler body at [`IRQ_HANDLER_OFFSET`].
///   Shared by PendSV, SysTick, and any `extra_slots` that reference
///   this offset.
/// * `alt_handler_hw` — alternate handler body at [`IRQ_ALT_HANDLER_OFFSET`].
///   Length may be zero (the scenario uses only one handler body).
/// * `main_hw` — main routine at [`IRQ_MAIN_OFFSET`].
/// * `extra_slots` — additional vector-table populators beyond the
///   default PendSV (14) + SysTick (15).
const fn build_image_irq<
    const N_HANDLER_HW: usize,
    const N_ALT_HW: usize,
    const N_MAIN_HW: usize,
>(
    image_base: u32,
    stack_top: u32,
    handler_hw: [u16; N_HANDLER_HW],
    alt_handler_hw: [u16; N_ALT_HW],
    main_hw: [u16; N_MAIN_HW],
    extra_slots: &[VectorSlot],
) -> [u8; IRQ_IMAGE_SIZE] {
    let mut out = [0u8; IRQ_IMAGE_SIZE];

    let reset_vec = (image_base + IRQ_MAIN_OFFSET) | 1;
    let default_vec = (image_base + IRQ_DEFAULT_HANDLER_OFFSET) | 1;
    let pendsv_vec = (image_base + IRQ_HANDLER_OFFSET) | 1;
    let systick_vec = (image_base + IRQ_HANDLER_OFFSET) | 1;

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

    // Word 14: PendSV / Word 15: SysTick
    let pv = pendsv_vec.to_le_bytes();
    out[56] = pv[0]; out[57] = pv[1]; out[58] = pv[2]; out[59] = pv[3];
    let sv = systick_vec.to_le_bytes();
    out[60] = sv[0]; out[61] = sv[1]; out[62] = sv[2]; out[63] = sv[3];

    // Default external-IRQ entries 16..=63: default handler (bkpt #1
    // halt). Scenarios override via extra_slots. Covers all 48 NVIC
    // inputs (IRQs 0..=47 map to exception numbers 16..=63) so a
    // scenario that pends a high-numbered IRQ without an explicit
    // extra_slots entry still lands on the halt handler instead of
    // jumping to address 0.
    let mut ext_slot = 16;
    while ext_slot < 64 {
        let off = ext_slot * 4;
        let b = default_vec.to_le_bytes();
        out[off] = b[0]; out[off + 1] = b[1]; out[off + 2] = b[2]; out[off + 3] = b[3];
        ext_slot += 1;
    }

    // Extra-slot overrides. Accept indices 0..=63 so external-IRQ
    // scenarios can populate any NVIC input line.
    let mut s = 0;
    while s < extra_slots.len() {
        let slot = extra_slots[s];
        if slot.index < 64 {
            let off = slot.index * 4;
            let vec = (image_base + slot.handler_offset) | 1;
            let b = vec.to_le_bytes();
            out[off] = b[0]; out[off + 1] = b[1]; out[off + 2] = b[2]; out[off + 3] = b[3];
        }
        s += 1;
    }

    // Default handler at 0x0C0: bkpt #1.
    out[IRQ_DEFAULT_HANDLER_OFFSET as usize] = 0x01;
    out[IRQ_DEFAULT_HANDLER_OFFSET as usize + 1] = 0xBE;
    out[IRQ_DEFAULT_HANDLER_OFFSET as usize + 2] = 0x00;
    out[IRQ_DEFAULT_HANDLER_OFFSET as usize + 3] = 0x00;

    // Primary handler body at IRQ_HANDLER_OFFSET.
    let mut h = 0;
    while h < N_HANDLER_HW {
        let off = IRQ_HANDLER_OFFSET as usize + h * 2;
        let b = handler_hw[h].to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        h += 1;
    }

    // Alternate handler body at IRQ_ALT_HANDLER_OFFSET.
    let mut a = 0;
    while a < N_ALT_HW {
        let off = IRQ_ALT_HANDLER_OFFSET as usize + a * 2;
        let b = alt_handler_hw[a].to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        a += 1;
    }

    // Main body at IRQ_MAIN_OFFSET.
    let mut m = 0;
    while m < N_MAIN_HW {
        let off = IRQ_MAIN_OFFSET as usize + m * 2;
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
// Phase 0a external-IRQ handlers + mains (HLD V5 §4.1.5)
// ---------------------------------------------------------------------------
//
// Three scenarios cover:
//   (a) cold TIMER0_IRQ_0 assert + handler entry
//   (b) masked-pending → unmask delivery
//   (c) priority preemption (two IRQs)
//
// All three use the extended IRQ_IMAGE_SIZE layout (HLD V5 §4.1.6) so
// vector slots 16+ (external IRQs) don't collide with the default-
// handler opcode at 0x040 in the baseline layout.

/// Baseline IRQ handler — reads CYCCNT, writes to mailbox, BKPT #0.
/// Same shape as [`HANDLER_BASELINE`] but laid out for the extended
/// `IRQ_IMAGE_SIZE` image — handler sits at `IRQ_HANDLER_OFFSET =
/// 0x104` so the literal-pool math differs from the 0x044 variant.
///
/// ```text
///   [0] ldr r0, [pc, #8]     ; r0 = DWT_CYCCNT_ADDR (lit at hw[6])
///   [1] ldr r0, [r0]         ; r0 = *DWT_CYCCNT
///   [2] ldr r1, [pc, #8]     ; r1 = ISR_MAILBOX_CYCCNT (lit at hw[8])
///   [3] str r0, [r1]         ; mailbox = CYCCNT
///   [4] bkpt #0              ; halt
///   [5] bkpt #0              ; padding
///   [6..7] lit: DWT_CYCCNT_ADDR
///   [8..9] lit: ISR_MAILBOX_CYCCNT
/// ```
///
/// Literal-pool math (hw[0] at byte 0x104, PC=0x108, Align(PC,4)=0x108,
/// target = 0x108 + 8 = 0x110 → hw[6]; hw[2] at 0x108, PC=0x10C,
/// Align=0x10C, target = 0x10C + 8 = 0x114 → hw[8]). Both opcodes
/// encode imm8=2 (offset 8) so nothing changes at the opcode level
/// when the handler base moves — PC-relative LDR is offset-agnostic.
const HANDLER_IRQ_CYCCNT: [u16; 10] = [
    0x4802, //  [0] ldr r0, [pc, #8]
    0x6800, //  [1] ldr r0, [r0]
    0x4902, //  [2] ldr r1, [pc, #8]
    0x6008, //  [3] str r0, [r1]
    0xBE00, //  [4] bkpt #0
    0xBE00, //  [5] bkpt #0 (padding)
    0x1004, //  [6] lit: DWT_CYCCNT_ADDR low
    0xE000, //  [7] lit: DWT_CYCCNT_ADDR high
    0x3FF8, //  [8] lit: ISR_MAILBOX_CYCCNT low
    0x2000, //  [9] lit: ISR_MAILBOX_CYCCNT high
];

/// High-priority preempting handler — writes 0xBBBBBBBB to mailbox and
/// BKPTs. Used by scenario C's primary (IRQ 1) handler. Placed at
/// IRQ_HANDLER_OFFSET = 0x104.
///
/// ```text
///   [0] ldr r0, [pc, #8]     ; r0 = 0xBBBBBBBB (lit at hw[6])
///   [1] ldr r1, [pc, #12]    ; r1 = ISR_MAILBOX_CYCCNT (lit at hw[8])
///   [2] str r0, [r1]         ; mailbox = sentinel B
///   [3] bkpt #0              ; halt
///   [4] bkpt #0              ; padding
///   [5] bkpt #0              ; padding
///   [6..7] lit: 0xBBBBBBBB
///   [8..9] lit: ISR_MAILBOX_CYCCNT
/// ```
const HANDLER_IRQ_PREEMPT: [u16; 10] = [
    0x4802, //  [0] ldr r0, [pc, #8]    — r0 = 0xBBBB_BBBB
    0x4903, //  [1] ldr r1, [pc, #12]   — r1 = ISR_MAILBOX_CYCCNT
    0x6008, //  [2] str r0, [r1]
    0xBE00, //  [3] bkpt #0
    0xBE00, //  [4] bkpt #0             — padding
    0xBE00, //  [5] bkpt #0             — padding
    0xBBBB, //  [6] lit: sentinel_b low
    0xBBBB, //  [7] lit: sentinel_b high
    0x3FF8, //  [8] lit: ISR_MAILBOX_CYCCNT low
    0x2000, //  [9] lit: ISR_MAILBOX_CYCCNT high
];

/// Low-priority preempted handler — writes 0xAAAAAAAA to mailbox, pends
/// IRQ 1 via NVIC_ISPR, then busy-waits (expecting IRQ 1 to preempt).
/// Used by scenario C's alternate (IRQ 0) handler at
/// IRQ_ALT_HANDLER_OFFSET = 0x140.
///
/// ```text
///   [0] ldr r0, [pc, #12]    ; r0 = 0xAAAAAAAA (lit hw[8])
///   [1] ldr r1, [pc, #16]    ; r1 = ISR_MAILBOX_CYCCNT (hw[10])
///   [2] str r0, [r1]         ; mailbox = sentinel A
///   [3] ldr r2, [pc, #16]    ; r2 = NVIC_ISPR0 (hw[12])
///   [4] movs r3, #2          ; r3 = 0x2 (bit for IRQ 1)
///   [5] str r3, [r2]         ; NVIC_ISPR |= 2 → pend IRQ 1
///   [6] b .                  ; busy-wait; IRQ 1 should preempt here
///   [7] bkpt #0              ; safety (reached only if no preemption)
///   [8..9] lit: 0xAAAAAAAA
///   [10..11] lit: ISR_MAILBOX_CYCCNT
///   [12..13] lit: NVIC_ISPR0
/// ```
///
/// Literal math (hw[0] at byte 0x140, PC=0x144, Align=0x144, target
/// hw[8] = 0x150, offset = 0x0C → imm8 = 3 → 0x4803). PC-relative LDR
/// is offset-agnostic, so opcodes remain the same regardless of the
/// handler base address.
const HANDLER_IRQ_LOW_PRIO: [u16; 14] = [
    0x4803, //  [0] ldr r0, [pc, #12]   — r0 = 0xAAAA_AAAA
    0x4904, //  [1] ldr r1, [pc, #16]   — r1 = ISR_MAILBOX_CYCCNT
    0x6008, //  [2] str r0, [r1]
    0x4A04, //  [3] ldr r2, [pc, #16]   — r2 = NVIC_ISPR0
    0x2302, //  [4] movs r3, #2
    0x6013, //  [5] str r3, [r2]        — pend IRQ 1
    0xE7FE, //  [6] b .                 — busy-wait; IRQ 1 preempts
    0xBE00, //  [7] bkpt #0             — safety
    0xAAAA, //  [8] lit: sentinel_a low
    0xAAAA, //  [9] lit: sentinel_a high
    0x3FF8, // [10] lit: ISR_MAILBOX_CYCCNT low
    0x2000, // [11] lit: ISR_MAILBOX_CYCCNT high
    0xE200, // [12] lit: NVIC_ISPR0 low
    0xE000, // [13] lit: NVIC_ISPR0 high
];

/// Scenario A main — cold TIMER0_IRQ_0. Enable IRQ 0 then pend.
///
/// ```text
///   [ 0] movs r4, #1
///   [ 1] ldr  r5, [pc, #28]   ; r5 = DWT_CYCCNT_ADDR
///   [ 2] ldr  r6, [pc, #28]   ; r6 = NVIC_ISER0
///   [ 3] ldr  r7, [pc, #32]   ; r7 = NVIC_ISPR0
///   [ 4] str  r4, [r6]        ; NVIC_ISER |= 1 (enable IRQ 0)
///   [ 5] movs r4, #0
///   [ 6] str  r4, [r5]        ; CYCCNT = 0 (reset)
///   [ 7] movs r4, #1
///   [ 8] str  r4, [r7]        ; NVIC_ISPR |= 1 (pend — TRIGGER)
///   [ 9] b    .               ; busy-wait
///   [10] bkpt #0              ; safety
///   [11..15] nop padding
///   [16..17] lit: DWT_CYCCNT_ADDR
///   [18..19] lit: NVIC_ISER0
///   [20..21] lit: NVIC_ISPR0
/// ```
///
/// Literal-pool math is PC-relative, so moving `IRQ_MAIN_OFFSET`
/// doesn't change the opcodes. At `IRQ_MAIN_OFFSET = 0x180`:
///   hw[1] at 0x182: PC = 0x186, Align = 0x184, target hw[16] = 0x1A0,
///     offset = 0x1C → imm8 = 7 → 0x4D07.
///   hw[2] at 0x184: PC = 0x188, Align = 0x188, target hw[18] = 0x1A4,
///     offset = 0x1C → imm8 = 7 → 0x4E07.
///   hw[3] at 0x186: PC = 0x18A, Align = 0x188, target hw[20] = 0x1A8,
///     offset = 0x20 → imm8 = 8 → 0x4F08.
const MAIN_IRQ_COLD: [u16; 22] = [
    0x2401, // [ 0] movs r4, #1
    0x4D07, // [ 1] ldr  r5, [pc, #28]   — r5 = DWT_CYCCNT_ADDR
    0x4E07, // [ 2] ldr  r6, [pc, #28]   — r6 = NVIC_ISER0
    0x4F08, // [ 3] ldr  r7, [pc, #32]   — r7 = NVIC_ISPR0
    0x6034, // [ 4] str  r4, [r6]        — enable IRQ 0
    0x2400, // [ 5] movs r4, #0
    0x602C, // [ 6] str  r4, [r5]        — reset CYCCNT
    0x2401, // [ 7] movs r4, #1
    0x603C, // [ 8] str  r4, [r7]        — pend IRQ 0 (TRIGGER)
    0xE7FE, // [ 9] b    .
    0xBE00, // [10] bkpt #0              — safety
    0xBF00, // [11] nop
    0xBF00, // [12] nop
    0xBF00, // [13] nop
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0x1004, // [16] lit: DWT_CYCCNT_ADDR low
    0xE000, // [17] lit: DWT_CYCCNT_ADDR high
    0xE100, // [18] lit: NVIC_ISER0 low
    0xE000, // [19] lit: NVIC_ISER0 high
    0xE200, // [20] lit: NVIC_ISPR0 low
    0xE000, // [21] lit: NVIC_ISPR0 high
];

/// Scenario B main — masked-pending, then unmask. Pend IRQ 0 with
/// ISER=0 (no delivery), a few cycles pass, then enable (trigger).
///
/// Identical literal pool to [`MAIN_IRQ_COLD`]; the only difference is
/// the order of the ISPR and ISER writes (line [4] vs [8]).
const MAIN_IRQ_MASKED_PEND: [u16; 22] = [
    0x2401, // [ 0] movs r4, #1
    0x4D07, // [ 1] ldr  r5, [pc, #28]   — r5 = DWT_CYCCNT_ADDR
    0x4E07, // [ 2] ldr  r6, [pc, #28]   — r6 = NVIC_ISER0
    0x4F08, // [ 3] ldr  r7, [pc, #32]   — r7 = NVIC_ISPR0
    0x603C, // [ 4] str  r4, [r7]        — pend IRQ 0 (latches, NO delivery)
    0x2400, // [ 5] movs r4, #0
    0x602C, // [ 6] str  r4, [r5]        — reset CYCCNT
    0x2401, // [ 7] movs r4, #1
    0x6034, // [ 8] str  r4, [r6]        — enable IRQ 0 (TRIGGER)
    0xE7FE, // [ 9] b    .
    0xBE00, // [10] bkpt #0              — safety
    0xBF00, // [11] nop
    0xBF00, // [12] nop
    0xBF00, // [13] nop
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0x1004, // [16] lit: DWT_CYCCNT_ADDR low
    0xE000, // [17] lit: DWT_CYCCNT_ADDR high
    0xE100, // [18] lit: NVIC_ISER0 low
    0xE000, // [19] lit: NVIC_ISER0 high
    0xE200, // [20] lit: NVIC_ISPR0 low
    0xE000, // [21] lit: NVIC_ISPR0 high
];

/// Scenario C main — priority preemption. Configure IRQ 0 priority =
/// 0xC0 (low) and IRQ 1 priority = 0x40 (high), enable both, pend IRQ 0.
/// The IRQ 0 handler (alt, at IRQ_ALT_HANDLER_OFFSET) then pends IRQ 1
/// from inside itself, triggering a preemption.
///
/// ```text
///   [ 0] ldr  r4, [pc, #28]   ; r4 = NVIC_IPR0_addr
///   [ 1] ldr  r5, [pc, #32]   ; r5 = 0x0000_40C0 (IPR priorities)
///   [ 2] str  r5, [r4]        ; NVIC_IPR0 = priorities
///   [ 3] ldr  r4, [pc, #32]   ; r4 = NVIC_ISER0
///   [ 4] movs r5, #3
///   [ 5] str  r5, [r4]        ; enable IRQ 0 + 1
///   [ 6] ldr  r4, [pc, #28]   ; r4 = DWT_CYCCNT_ADDR
///   [ 7] movs r5, #0
///   [ 8] str  r5, [r4]        ; CYCCNT = 0
///   [ 9] ldr  r4, [pc, #28]   ; r4 = NVIC_ISPR0
///   [10] movs r5, #1
///   [11] str  r5, [r4]        ; pend IRQ 0 — TRIGGER
///   [12] b    .
///   [13] bkpt #0              ; safety
///   [14..15] nop padding
///   [16..17] lit: NVIC_IPR0_addr
///   [18..19] lit: IPR priorities = 0x0000_40C0
///   [20..21] lit: NVIC_ISER0
///   [22..23] lit: DWT_CYCCNT_ADDR
///   [24..25] lit: NVIC_ISPR0
/// ```
///
/// Literal math is PC-relative; main moves to `IRQ_MAIN_OFFSET = 0x180`:
///   hw[0] at 0x180: PC = 0x184, Align = 0x184, target hw[16] = 0x1A0,
///     offset = 0x1C → imm8 = 7 → 0x4C07.
///   hw[1] at 0x182: PC = 0x186, Align = 0x184, target hw[18] = 0x1A4,
///     offset = 0x20 → imm8 = 8 → 0x4D08.
///   hw[3] at 0x186: PC = 0x18A, Align = 0x188, target hw[20] = 0x1A8,
///     offset = 0x20 → imm8 = 8 → 0x4C08.
///   hw[6] at 0x18C: PC = 0x190, Align = 0x190, target hw[22] = 0x1AC,
///     offset = 0x1C → imm8 = 7 → 0x4C07.
///   hw[9] at 0x192: PC = 0x196, Align = 0x194, target hw[24] = 0x1B0,
///     offset = 0x1C → imm8 = 7 → 0x4C07.
const MAIN_IRQ_PRIORITY_PREEMPT: [u16; 26] = [
    0x4C07, // [ 0] ldr  r4, [pc, #28]   — r4 = NVIC_IPR0_addr
    0x4D08, // [ 1] ldr  r5, [pc, #32]   — r5 = 0x0000_40C0
    0x6025, // [ 2] str  r5, [r4]        — NVIC_IPR0 = priorities
    0x4C08, // [ 3] ldr  r4, [pc, #32]   — r4 = NVIC_ISER0
    0x2503, // [ 4] movs r5, #3
    0x6025, // [ 5] str  r5, [r4]        — enable IRQ 0 + 1
    0x4C07, // [ 6] ldr  r4, [pc, #28]   — r4 = DWT_CYCCNT_ADDR
    0x2500, // [ 7] movs r5, #0
    0x6025, // [ 8] str  r5, [r4]        — CYCCNT = 0
    0x4C07, // [ 9] ldr  r4, [pc, #28]   — r4 = NVIC_ISPR0
    0x2501, // [10] movs r5, #1
    0x6025, // [11] str  r5, [r4]        — pend IRQ 0 (TRIGGER)
    0xE7FE, // [12] b    .
    0xBE00, // [13] bkpt #0              — safety
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0xE400, // [16] lit: NVIC_IPR0_addr low
    0xE000, // [17] lit: NVIC_IPR0_addr high
    0x40C0, // [18] lit: priorities low  (bytes [0xC0, 0x40, 0x00, 0x00])
    0x0000, // [19] lit: priorities high
    0xE100, // [20] lit: NVIC_ISER0 low
    0xE000, // [21] lit: NVIC_ISER0 high
    0x1004, // [22] lit: DWT_CYCCNT_ADDR low
    0xE000, // [23] lit: DWT_CYCCNT_ADDR high
    0xE200, // [24] lit: NVIC_ISPR0 low
    0xE000, // [25] lit: NVIC_ISPR0 high
];

/// No alternate handler body — scenarios A and B use only the primary
/// handler at IRQ_HANDLER_OFFSET. Length-0 array keeps `build_image_irq`'s
/// const-generic instantiation happy.
const HANDLER_NONE: [u16; 0] = [];

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

// Mode-set slices for external-IRQ scenarios. Each maps the relevant
// external IRQ slot to its handler offset inside the image.
//
// Scenarios A + B populate only slot 16 (TIMER0_IRQ_0) and point it at
// `IRQ_HANDLER_OFFSET` — the shared CYCCNT-mailbox handler.
const MODESET_IRQ_TIMER0: &[VectorSlot] = &[
    VectorSlot::new(16, IRQ_HANDLER_OFFSET),
];

// Scenario C populates slot 16 (TIMER0_IRQ_0 → low-priority alt handler)
// and slot 17 (TIMER0_IRQ_1 → high-priority primary handler). The slot
// order encodes the preemption relationship: 17 is numerically later but
// architecturally higher-priority (0x40 < 0xC0).
const MODESET_IRQ_PREEMPT: &[VectorSlot] = &[
    VectorSlot::new(16, IRQ_ALT_HANDLER_OFFSET),
    VectorSlot::new(17, IRQ_HANDLER_OFFSET),
];

const IMAGE_IRQ_COLD: [u8; IRQ_IMAGE_SIZE] =
    build_image_irq(ISR_IMAGE_BASE, ISR_STACK_TOP,
        HANDLER_IRQ_CYCCNT, HANDLER_NONE, MAIN_IRQ_COLD, MODESET_IRQ_TIMER0);

const IMAGE_IRQ_MASKED_PEND: [u8; IRQ_IMAGE_SIZE] =
    build_image_irq(ISR_IMAGE_BASE, ISR_STACK_TOP,
        HANDLER_IRQ_CYCCNT, HANDLER_NONE, MAIN_IRQ_MASKED_PEND, MODESET_IRQ_TIMER0);

const IMAGE_IRQ_PRIORITY_PREEMPT: [u8; IRQ_IMAGE_SIZE] =
    build_image_irq(ISR_IMAGE_BASE, ISR_STACK_TOP,
        HANDLER_IRQ_PREEMPT, HANDLER_IRQ_LOW_PRIO, MAIN_IRQ_PRIORITY_PREEMPT, MODESET_IRQ_PREEMPT);

// ---------------------------------------------------------------------------
// POWMAN TIMER IRQ scenario (Coverage Gap Fill V11 §3.2)
// ---------------------------------------------------------------------------
//
// `powman_match_irq_timer_line_45` — release POWMAN from reset, program
// AON alarm = 100, enable `TIMER.RUN | TIMER.ALARM_ENAB`, enable NVIC
// line 45 (bank 1 bit 13). Handler writes `0xCAFE_BABE` to
// `ISR_MAILBOX_CYCCNT` (pre-seeded to 0 by the runner — see
// `run_one_scenario`'s `write_word_32(ISR_MAILBOX_CYCCNT, 0)` and
// `mmio_write32(ISR_MAILBOX_CYCCNT, 0)` calls) and BKPT #0. The HLD
// suggests a `0xDEAD_BEEF` sentinel, but the existing oracle runner
// uses 0; a post-run read of 0 means the handler never ran.
//
// Vector slot math: 16 + 45 = 61 → byte offset 61 * 4 = 0xF4. Below
// `IRQ_IMAGE_SIZE`'s 64-slot vector table (0x100 bytes), so no layout
// change needed. `MODESET_IRQ_POWMAN` populates the slot.
//
// NVIC line 45 is in bank 1: ISER1 at 0xE000_E104, bit (45 - 32) = 13.
// Value 1 << 13 = 0x2000.

const MODESET_IRQ_POWMAN: &[VectorSlot] = &[
    VectorSlot::new(61, IRQ_HANDLER_OFFSET),
];

/// POWMAN handler — V13 Stage 2. Captures CYCCNT into
/// `ISR_MAILBOX_CYCCNT` (for the `cyccnt_delta` observable) *and*
/// stores the `0xCAFE_BABE` sentinel into `ISR_MAILBOX_RESERVED` (for
/// the deterministic `mailbox_sentinel` observable), then `bkpt #0`.
///
/// Two mailbox slots so pass/fail and timing drift are diffed
/// independently: the sentinel is exact-compared (HW = EMU =
/// 0xCAFE_BABE), while CYCCNT values differ modulo XOSC/sys_clk phase
/// and are inspected as an informational delta in the oracle's FAIL
/// output. The single-slot V12 handler couldn't carry both signals.
///
/// Handler body at `IRQ_HANDLER_OFFSET = 0x104`. PC-relative LDR on
/// Thumb resolves its target as `Align(PC+4, 4) + imm8*4`. All three
/// literal loads in this handler sit at *word-aligned* instruction
/// offsets (hw[0], hw[2], hw[4] at 0x104/0x108/0x10C), so PC aligns
/// cleanly and each picks up the 0x4800 | 3 encoding.
///
/// Resolution table:
///
/// * `hw[0]` at 0x104: PC=0x108, Align=0x108, target hw[8]=0x114
///   (= DWT_CYCCNT_ADDR). imm8=3 → `0x4803` (`ldr r0, [pc, #12]`).
/// * `hw[2]` at 0x108: PC=0x10C, Align=0x10C, target hw[10]=0x118
///   (= ISR_MAILBOX_BASE). imm8=3 → `0x4903` (`ldr r1, [pc, #12]`).
/// * `hw[4]` at 0x10C: PC=0x110, Align=0x110, target hw[12]=0x11C
///   (= 0xCAFE_BABE). imm8=3 → `0x4803` (`ldr r0, [pc, #12]`).
/// * `hw[5]` is `str r0, [r1, #4]` (0x6048) — stores the sentinel at
///   `ISR_MAILBOX_BASE + 4 = ISR_MAILBOX_RESERVED`.
///
/// ```text
///   [0] ldr r0, [pc, #12]    ; r0 = DWT_CYCCNT_ADDR (hw[8..9])
///   [1] ldr r0, [r0]         ; r0 = *CYCCNT
///   [2] ldr r1, [pc, #12]    ; r1 = ISR_MAILBOX_BASE (hw[10..11])
///   [3] str r0, [r1]         ; mailbox_cyccnt = CYCCNT
///   [4] ldr r0, [pc, #12]    ; r0 = 0xCAFEBABE (hw[12..13])
///   [5] str r0, [r1, #4]     ; mailbox_reserved = sentinel
///   [6] bkpt #0              ; halt
///   [7] bkpt #0              ; padding (keep literal pool aligned)
///   [8..9]   lit: DWT_CYCCNT_ADDR
///   [10..11] lit: ISR_MAILBOX_BASE
///   [12..13] lit: 0xCAFEBABE
/// ```
const HANDLER_POWMAN_SENTINEL: [u16; 14] = [
    0x4803, //  [ 0] ldr r0, [pc, #12]  — r0 = DWT_CYCCNT_ADDR
    0x6800, //  [ 1] ldr r0, [r0]       — r0 = *CYCCNT
    0x4903, //  [ 2] ldr r1, [pc, #12]  — r1 = ISR_MAILBOX_BASE
    0x6008, //  [ 3] str r0, [r1]       — mailbox_cyccnt = CYCCNT
    0x4803, //  [ 4] ldr r0, [pc, #12]  — r0 = 0xCAFEBABE
    0x6048, //  [ 5] str r0, [r1, #4]   — mailbox_reserved = sentinel
    0xBE00, //  [ 6] bkpt #0            — halt for host readback
    0xBE00, //  [ 7] bkpt #0            — padding
    0x1004, //  [ 8] lit: DWT_CYCCNT_ADDR low  = 0xE000_1004 & 0xFFFF
    0xE000, //  [ 9] lit: DWT_CYCCNT_ADDR high = 0xE000_1004 >> 16
    0x3FF8, //  [10] lit: ISR_MAILBOX_BASE low  = 0x2000_3FF8 & 0xFFFF
    0x2000, //  [11] lit: ISR_MAILBOX_BASE high = 0x2000_3FF8 >> 16
    0xBABE, //  [12] lit: 0xCAFEBABE low
    0xCAFE, //  [13] lit: 0xCAFEBABE high
];

/// POWMAN scenario main — release POWMAN, program alarm = 100, enable
/// TIMER.RUN | TIMER.ALARM_ENAB, enable NVIC IRQ 45.
///
/// **POWMAN password requirement.** Per pico-sdk at the pinned commit
/// `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779` (`powman.h`), every
/// password-gated POWMAN register (including ALARM_TIME_* and TIMER)
/// silently drops writes unless bits [31:16] equal `0x5AFE`; a wrong
/// password also latches `BADPASSWD`. The literal-pool values for
/// ALARM=100 and TIMER=RUN|ALARM_ENAB therefore bake `0x5AFE` into
/// the upper halfword: `0x5AFE_0064` and `0x5AFE_0012`. On silicon
/// POWMAN stores only the low 16 bits; the emulator mirrors this by
/// masking password-gated writes to `value & 0xFFFF` (see
/// `peripherals::powman::PowmanRegs::write32`). RESETS and NVIC writes
/// do NOT require the password.
///
/// All constants live in the literal pool starting at halfword [20]
/// (= `IRQ_MAIN_OFFSET + 40` = 0x1A8). Each `ldr` uses PC-relative
/// addressing; the encoding is offset-agnostic (PC-rel LDR opcodes
/// depend only on *relative* position, not the absolute main offset).
///
/// V12 §3.2: silicon gates NVIC line 45 on `INTE.TIMER`. Three
/// instructions added at hw[9..11] write `POWMAN_INTE = 0x5AFE_0002`
/// before the NVIC ISER1 enable. Existing pre-NVIC instruction slots
/// hw[0..8] are unchanged in shape; their LDR `imm8` fields are
/// recomputed because the literal pool shifted by 4 halfwords (from
/// hw[16..31] to hw[20..35]). New literals append at hw[36..39].
///
/// ```text
///   [ 0] ldr  r4, [pc, #36]      ; r4 = 1 << 17  (RESET_POWMAN)
///   [ 1] ldr  r5, [pc, #40]      ; r5 = RESETS_RESET_CLR = 0x4002_3000
///   [ 2] str  r4, [r5]           ; release POWMAN
///   [ 3] ldr  r4, [pc, #40]      ; r4 = 100 | password = 0x5AFE_0064
///   [ 4] ldr  r5, [pc, #40]      ; r5 = POWMAN_ALARM_TIME_15TO0
///   [ 5] str  r4, [r5]           ; ALARM = 100
///   [ 6] ldr  r4, [pc, #40]      ; r4 = RUN|ALARM_ENAB|pwd = 0x5AFE_0012
///   [ 7] ldr  r5, [pc, #44]      ; r5 = POWMAN_TIMER
///   [ 8] str  r4, [r5]           ; TIMER = RUN | ALARM_ENAB
///   [ 9] ldr  r4, [pc, #52]      ; r4 = INT_TIMER_BIT|pwd = 0x5AFE_0002 (NEW)
///   [10] ldr  r5, [pc, #52]      ; r5 = POWMAN_INTE = 0x4010_00E4 (NEW)
///   [11] str  r4, [r5]           ; INTE.TIMER = 1                     (NEW)
///   [12] ldr  r4, [pc, #40]      ; r4 = 1 << 13 = 0x2000
///   [13] ldr  r5, [pc, #40]      ; r5 = NVIC_ISER1
///   [14] str  r4, [r5]           ; enable IRQ 45
///   [15] b    .                  ; busy-wait
///   [16] bkpt #0                 ; safety (unreachable if IRQ fires)
///   [17..19] nop                 ; padding (keep literal pool word-
///                                ;          aligned at hw[20] = 0x1A8)
///   [20..21] lit: 1 << 17                  0x0002_0000
///   [22..23] lit: RESETS_RESET_CLR         0x4002_3000
///   [24..25] lit: 100 + POWMAN password    0x5AFE_0064
///   [26..27] lit: POWMAN_ALARM_TIME_15TO0  0x4010_0084
///   [28..29] lit: RUN|ALARM_ENAB + pwd     0x5AFE_0012
///   [30..31] lit: POWMAN_TIMER             0x4010_0088
///   [32..33] lit: 1 << 13                  0x0000_2000
///   [34..35] lit: NVIC_ISER1               0xE000_E104
///   [36..37] lit: INT_TIMER_BIT + pwd      0x5AFE_0002 (NEW)
///   [38..39] lit: POWMAN_INTE              0x4010_00E4 (NEW)
/// ```
///
/// Literal-pool math at `IRQ_MAIN_OFFSET = 0x180`. Each LDR (T1)
/// encodes `imm8 = (target - (PC & ~3)) / 4` where PC = inst_addr + 4.
///   hw[ 0] at 0x180: PC=0x184, Align=0x184, target hw[20]=0x1A8
///          → offset 0x24 = 36 → imm8=9  → `0x4C09`
///   hw[ 1] at 0x182: PC=0x186, Align=0x184, target hw[22]=0x1AC
///          → offset 0x28 = 40 → imm8=10 → `0x4D0A`
///   hw[ 3] at 0x186: PC=0x18A, Align=0x188, target hw[24]=0x1B0
///          → offset 0x28 = 40 → imm8=10 → `0x4C0A`
///   hw[ 4] at 0x188: PC=0x18C, Align=0x18C, target hw[26]=0x1B4
///          → offset 0x28 = 40 → imm8=10 → `0x4D0A`
///   hw[ 6] at 0x18C: PC=0x190, Align=0x190, target hw[28]=0x1B8
///          → offset 0x28 = 40 → imm8=10 → `0x4C0A`
///   hw[ 7] at 0x18E: PC=0x192, Align=0x190, target hw[30]=0x1BC
///          → offset 0x2C = 44 → imm8=11 → `0x4D0B`
///   hw[ 9] at 0x192: PC=0x196, Align=0x194, target hw[36]=0x1C8 (NEW)
///          → offset 0x34 = 52 → imm8=13 → `0x4C0D`
///   hw[10] at 0x194: PC=0x198, Align=0x198, target hw[38]=0x1CC (NEW)
///          → offset 0x34 = 52 → imm8=13 → `0x4D0D`
///   hw[12] at 0x198: PC=0x19C, Align=0x19C, target hw[32]=0x1C0
///          → offset 0x24 = 36 → imm8=9  → `0x4C09`
///   hw[13] at 0x19A: PC=0x19E, Align=0x19C, target hw[34]=0x1C4
///          → offset 0x28 = 40 → imm8=10 → `0x4D0A`
const MAIN_IRQ_POWMAN: [u16; 40] = [
    0x4C09, // [ 0] ldr  r4, [pc, #36]    — r4 = 1 << 17
    0x4D0A, // [ 1] ldr  r5, [pc, #40]    — r5 = RESETS_RESET_CLR
    0x602C, // [ 2] str  r4, [r5]         — release POWMAN
    0x4C0A, // [ 3] ldr  r4, [pc, #40]    — r4 = 100 | pwd
    0x4D0A, // [ 4] ldr  r5, [pc, #40]    — r5 = POWMAN_ALARM_TIME_15TO0
    0x602C, // [ 5] str  r4, [r5]
    0x4C0A, // [ 6] ldr  r4, [pc, #40]    — r4 = RUN | ALARM_ENAB | pwd
    0x4D0B, // [ 7] ldr  r5, [pc, #44]    — r5 = POWMAN_TIMER
    0x602C, // [ 8] str  r4, [r5]
    0x4C0D, // [ 9] ldr  r4, [pc, #52]    — r4 = INT_TIMER_BIT | pwd  (NEW)
    0x4D0D, // [10] ldr  r5, [pc, #52]    — r5 = POWMAN_INTE          (NEW)
    0x602C, // [11] str  r4, [r5]         — INTE.TIMER = 1            (NEW)
    0x4C09, // [12] ldr  r4, [pc, #36]    — r4 = 1 << 13
    0x4D0A, // [13] ldr  r5, [pc, #40]    — r5 = NVIC_ISER1
    0x602C, // [14] str  r4, [r5]
    0xE7FE, // [15] b    .                — busy-wait
    0xBE00, // [16] bkpt #0               — safety
    0xBF00, // [17] nop
    0xBF00, // [18] nop
    0xBF00, // [19] nop                   — pad to keep hw[20] word-aligned
    0x0000, // [20] lit: 1 << 17 low  = 0x0002_0000 & 0xFFFF
    0x0002, // [21] lit: 1 << 17 high = 0x0002_0000 >> 16
    0x3000, // [22] lit: RESETS_RESET_CLR low  = 0x4002_3000 & 0xFFFF
    0x4002, // [23] lit: RESETS_RESET_CLR high
    0x0064, // [24] lit: (100 | POWMAN password) low  = 0x5AFE_0064 & 0xFFFF
    0x5AFE, // [25] lit: (100 | POWMAN password) high = 0x5AFE_0064 >> 16
    0x0084, // [26] lit: POWMAN_ALARM_TIME_15TO0 low
    0x4010, // [27] lit: POWMAN_ALARM_TIME_15TO0 high
    0x0012, // [28] lit: (RUN | ALARM_ENAB | POWMAN password) low  = 0x5AFE_0012 & 0xFFFF
    0x5AFE, // [29] lit: (RUN | ALARM_ENAB | POWMAN password) high = 0x5AFE_0012 >> 16
    0x0088, // [30] lit: POWMAN_TIMER low
    0x4010, // [31] lit: POWMAN_TIMER high
    0x2000, // [32] lit: 1 << 13 low
    0x0000, // [33] lit: 1 << 13 high
    0xE104, // [34] lit: NVIC_ISER1 low
    0xE000, // [35] lit: NVIC_ISER1 high
    0x0002, // [36] lit: (INT_TIMER_BIT | POWMAN password) low  = 0x5AFE_0002 & 0xFFFF (NEW)
    0x5AFE, // [37] lit: (INT_TIMER_BIT | POWMAN password) high = 0x5AFE_0002 >> 16    (NEW)
    0x00E4, // [38] lit: POWMAN_INTE = 0x4010_00E4 low                                  (NEW)
    0x4010, // [39] lit: POWMAN_INTE = 0x4010_00E4 high                                 (NEW)
];

const IMAGE_IRQ_POWMAN: [u8; IRQ_IMAGE_SIZE] =
    build_image_irq(ISR_IMAGE_BASE, ISR_STACK_TOP,
        HANDLER_POWMAN_SENTINEL, HANDLER_NONE, MAIN_IRQ_POWMAN, MODESET_IRQ_POWMAN);

// -- Scenario: powman_match_irq_timer_line_45 (Coverage Gap Fill V11 §3.2) --
const INIT_IRQ_POWMAN: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
];
const OBS_IRQ_POWMAN: &[(&str, IsrObservable)] = &[
    // V13 Stage 2 — two observables in parallel:
    //
    // `cyccnt_delta` — reads ISR_MAILBOX_CYCCNT, which the handler
    //   populates with the CYCCNT snapshot at entry. HW and EMU
    //   CYCCNT values differ modulo XOSC-to-sys_clk phase (POWMAN
    //   ticks on XOSC/4 while exception entry timing is sys_clk-
    //   domain), so the oracle reports both so timing drift is
    //   visible even when the exact compare fails.
    // `mailbox_sentinel` — reads ISR_MAILBOX_RESERVED, a deterministic
    //   `0xCAFE_BABE` proof that the handler ran and reached the
    //   sentinel store. HW = EMU = 0xCAFE_BABE; this is the primary
    //   pass/fail signal.
    ("cyccnt_delta", IsrObservable::CycleDelta),
    (
        "mailbox_sentinel",
        IsrObservable::Mmio(crate::ISR_MAILBOX_RESERVED, !0),
    ),
];

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

// -- Scenario 5: isr_ext_irq_timer0_cold (Phase 0a / HLD V5 §4.1.5 a) --
//
// Main enables IRQ 0 (TIMER0_IRQ_0) in NVIC_ISER then pends it via
// NVIC_ISPR. The handler reads CYCCNT and halts at BKPT #0. Observables:
//
//   * CycleDelta — cold external-IRQ entry is ~12 cycles plus handler
//     prologue. HW and EMU should agree.
//   * Stacked PC — the `b .` at main hw[9] = byte 0x152.
//   * Stacked xPSR — carries IRQ 0's active exception number (16).
//
// VTOR points at ISR_IMAGE_BASE; no other preamble required.
const INIT_EXT_IRQ_COLD: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
];
const OBS_EXT_IRQ_COLD: &[(&str, IsrObservable)] = &[
    ("cyccnt_delta", IsrObservable::CycleDelta),
    ("stacked_pc", IsrObservable::Stacked(StackedReg::Pc)),
    ("stacked_xpsr", IsrObservable::Stacked(StackedReg::Xpsr)),
];

// -- Scenario 6: isr_ext_irq_masked_pending (Phase 0a / HLD V5 §4.1.5 b) --
//
// Main pends IRQ 0 BEFORE enabling NVIC_ISER. Pending bit latches but
// no delivery happens. Then main writes NVIC_ISER → unmask triggers
// delivery. Same observables as scenario 5 since the end state is
// identical — the point is that delivery is deferred until ISER is set.
const INIT_EXT_IRQ_MASKED_PEND: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
];
const OBS_EXT_IRQ_MASKED_PEND: &[(&str, IsrObservable)] = &[
    ("cyccnt_delta", IsrObservable::CycleDelta),
    ("stacked_pc", IsrObservable::Stacked(StackedReg::Pc)),
    ("stacked_xpsr", IsrObservable::Stacked(StackedReg::Xpsr)),
];

// -- Scenario 7: isr_ext_irq_priority_preempt (Phase 0a / HLD V5 §4.1.5 c) --
//
// Two IRQs: IRQ 0 at priority 0xC0 (low), IRQ 1 at priority 0x40 (high).
// Main pends IRQ 0 → low-priority handler fires. Low-priority handler
// writes sentinel 0xAAAAAAAA to mailbox, then pends IRQ 1. High-priority
// IRQ 1 preempts before the low-priority busy-wait completes. IRQ 1
// handler writes sentinel 0xBBBBBBBB to mailbox and BKPTs.
//
// Observable: `mailbox (CycleDelta slot)` — should read 0xBBBB_BBBB iff
// preemption occurred. If preemption is broken (no dispatch of IRQ 1
// while IRQ 0's handler is running), the scenario times out with
// mailbox = 0xAAAA_AAAA.
const INIT_EXT_IRQ_PRIORITY_PREEMPT: &[(IsrReg, u32)] = &[
    (IsrReg::Vtor, ISR_IMAGE_BASE),
];
const OBS_EXT_IRQ_PRIORITY_PREEMPT: &[(&str, IsrObservable)] = &[
    // `CycleDelta` reads ISR_MAILBOX_CYCCNT; scenario C reuses the slot
    // for the sentinel byte. The expected value on both HW and EMU is
    // 0xBBBB_BBBB (high-priority handler overwrote low's write).
    ("mailbox_sentinel", IsrObservable::CycleDelta),
];

// ---------------------------------------------------------------------------
// Catalogue
// ---------------------------------------------------------------------------

/// Initial catalogue + Phase 0a external-IRQ additions.
///
/// v1 covered 4 scenarios (PendSV/SysTick/FP). Phase 0a adds three
/// external-IRQ scenarios (HLD V5 §4.1.5) that validate the new NVIC
/// dispatch path:
///
///   5. `isr_ext_irq_timer0_cold` — cold TIMER0_IRQ_0 assert + entry.
///   6. `isr_ext_irq_masked_pending` — pend-before-unmask delivery.
///   7. `isr_ext_irq_priority_preempt` — high-priority IRQ preempts
///       low-priority handler mid-execution.
///
/// All names keep the `isr_` prefix for orchestrator substring-
/// uniqueness; the `_ext_irq_` mid-segment distinguishes them from the
/// v1 PendSV/SysTick scenarios.
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
    IsrScenario {
        name: "isr_ext_irq_timer0_cold",
        image: &IMAGE_IRQ_COLD,
        entry_offset: IRQ_MAIN_OFFSET,
        init_regs: INIT_EXT_IRQ_COLD,
        max_sysclks: 1000,
        observe: OBS_EXT_IRQ_COLD,
    },
    IsrScenario {
        name: "isr_ext_irq_masked_pending",
        image: &IMAGE_IRQ_MASKED_PEND,
        entry_offset: IRQ_MAIN_OFFSET,
        init_regs: INIT_EXT_IRQ_MASKED_PEND,
        max_sysclks: 1200,
        observe: OBS_EXT_IRQ_MASKED_PEND,
    },
    IsrScenario {
        name: "isr_ext_irq_priority_preempt",
        image: &IMAGE_IRQ_PRIORITY_PREEMPT,
        entry_offset: IRQ_MAIN_OFFSET,
        init_regs: INIT_EXT_IRQ_PRIORITY_PREEMPT,
        max_sysclks: 2000,
        observe: OBS_EXT_IRQ_PRIORITY_PREEMPT,
    },
    // Coverage Gap Fill V11 §3.2: POWMAN AON match fires IRQ 45. Main
    // releases POWMAN from reset, programs ALARM=100 + TIMER.{RUN,
    // ALARM_ENAB}, enables NVIC line 45 in bank 1. Handler writes
    // 0xCAFE_BABE to ISR_MAILBOX_CYCCNT (pre-seeded 0 by the runner —
    // see the HANDLER_POWMAN_SENTINEL docs). The scenario also runs on
    // live RP2354 via `test_rp2350_silicon_isr_diff`.
    //
    // `max_sysclks = 100 * POWMAN_SYS_PER_TICK + 500 = 5500`. At the
    // default clock tree (sys=150 MHz, POWMAN=3 MHz) this covers the
    // 100-tick count-up plus exception entry + handler prologue.
    // Stage 5 pre-flight (`smoke_powman_pacing_rp2350`) measures the
    // real ratio; update if silicon disagrees.
    // Name prefixed with `isr_` per the orchestrator's substring-
    // uniqueness contract (see `test_rp2350_silicon.rs`); the
    // informative suffix `powman_match_timer_line_45` traces back to
    // HLD V11 §3.2's logical scenario name.
    IsrScenario {
        name: "isr_powman_match_timer_line_45",
        image: &IMAGE_IRQ_POWMAN,
        entry_offset: IRQ_MAIN_OFFSET,
        init_regs: INIT_IRQ_POWMAN,
        max_sysclks: 5500,
        observe: OBS_IRQ_POWMAN,
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
    pub exclude: Option<String>,
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

    // Reset + halt between scenarios so NVIC pending/active state,
    // SysTick configuration, SHCSR.PENDSVACT, and other SCB state from
    // the prior scenario can't leak into this one. Observed in practice:
    // `isr_lazy_fp_save` hangs when run right after `isr_pendsv_cold` in
    // catalogue order but passes in isolation — prior scenario left the
    // CPU in a state where the new scenario's entry executes a stale
    // exception before reaching its own BKPT. A plain `halt` alone
    // doesn't clear that. SYSRESETREQ disables DWT/CYCCNT, so re-enable
    // before the observation logic reaches `reset_cyccnt`.
    core.reset_and_halt(Duration::from_millis(500))?;
    enable_cyccnt(core)?;

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

    // Phase 0b.1 Commit B: per-core PPB (SCB, NVIC, SysTick, FPCCR,
    // MPU, fault regs) lives on CortexM33 now; `Emulator::mmio_*` route
    // PPB addresses to core 0's PPB directly, so no `set_active_core`
    // bookkeeping is needed. Core 1 is halted for the entire scenario.

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

    // Phase 0b.1 Commit B: `Emulator::mmio_read32` routes PPB addresses
    // directly to `self.cores[0].ppb`; no `set_active_core` rewind is
    // needed. Core 0 is where the scenario executed; observables read
    // from core 0's PPB.

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
            .filter(|s| !silicon_oracle::should_exclude(s.name, args.exclude.as_deref()))
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

    /// Per-scenario layout constants. Phase 0a introduces a second
    /// image size (`IRQ_IMAGE_SIZE`) for external-IRQ scenarios, so
    /// tests that assert layout invariants pick constants based on the
    /// scenario's image length.
    struct Layout {
        image_size: usize,
        main_offset: u32,
        handler_offset: u32,
        default_handler_offset: u32,
    }

    fn layout_for(sc: &IsrScenario) -> Layout {
        if sc.image.len() == IRQ_IMAGE_SIZE {
            Layout {
                image_size: IRQ_IMAGE_SIZE,
                main_offset: IRQ_MAIN_OFFSET,
                handler_offset: IRQ_HANDLER_OFFSET,
                default_handler_offset: IRQ_DEFAULT_HANDLER_OFFSET,
            }
        } else {
            Layout {
                image_size: ISR_IMAGE_SIZE,
                main_offset: MAIN_OFFSET,
                handler_offset: HANDLER_OFFSET,
                default_handler_offset: 0x040,
            }
        }
    }

    // (1) Catalogue presence — v1 + Phase 0a = 7 scenarios, all `isr_*`
    //     prefix.
    #[test]
    fn test_catalogue_size_and_prefix() {
        assert_eq!(SCENARIOS.len(), 8,
            "catalogue must carry v1 (4) + Phase 0a (3) + POWMAN (1) = 8 scenarios");
        for s in SCENARIOS {
            assert!(
                s.name.starts_with("isr_"),
                "scenario '{}' must start with 'isr_' prefix for substring-uniqueness",
                s.name,
            );
        }
    }

    // (2) Image-layout invariants — per-scenario layout table (baseline
    //     = ISR_IMAGE_SIZE, extended = IRQ_IMAGE_SIZE). Word 0 is
    //     ISR_STACK_TOP, word 1 points at main with Thumb LSB, words
    //     14/15 point at the primary handler, and the default handler
    //     is `bkpt #1`.
    #[test]
    fn test_image_layout_invariants() {
        for sc in SCENARIOS {
            let layout = layout_for(sc);
            assert_eq!(
                sc.image.len(),
                layout.image_size,
                "scenario '{}' image must be {} bytes",
                sc.name,
                layout.image_size,
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
                ISR_IMAGE_BASE + layout.main_offset,
                "scenario '{}' vector[1] must point at main (offset 0x{:X})",
                sc.name,
                layout.main_offset,
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
                ISR_IMAGE_BASE + layout.handler_offset,
                "scenario '{}' vector[14] must point at primary handler",
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

            // Default handler must be bkpt #1 (0xBE01).
            let dh_off = layout.default_handler_offset as usize;
            let dh = u16::from_le_bytes([
                sc.image[dh_off],
                sc.image[dh_off + 1],
            ]);
            assert_eq!(
                dh, 0xBE01,
                "scenario '{}' default handler at 0x{:03X} must be bkpt #1",
                sc.name,
                dh_off,
            );
        }
    }

    // (3) Main routine fits within the image. Every scenario must have
    //     at least `main_offset + 4` bytes of content and a non-zero
    //     leading halfword (catches a malformed image where main never
    //     got written).
    #[test]
    fn test_main_routine_fits_image() {
        for sc in SCENARIOS {
            let layout = layout_for(sc);
            assert!(
                sc.image.len() >= (layout.main_offset as usize) + 4,
                "scenario '{}' image too small to contain main at main_offset",
                sc.name,
            );
            let main_hw0 = u16::from_le_bytes([
                sc.image[layout.main_offset as usize],
                sc.image[layout.main_offset as usize + 1],
            ]);
            assert_ne!(
                main_hw0, 0,
                "scenario '{}' main routine starts with zero halfword",
                sc.name,
            );
        }
    }

    // (3a) Phase 0a mode-set invariants — external-IRQ scenarios
    //      populate vector slot 16 (TIMER0_IRQ_0) and, for scenario C,
    //      slot 17 (TIMER0_IRQ_1). Assert the entries are non-default
    //      (i.e. the mode-set override took effect).
    #[test]
    fn test_external_irq_scenarios_populate_slot_16() {
        for sc in SCENARIOS {
            if !sc.name.starts_with("isr_ext_irq_") {
                continue;
            }
            // Slot 16 at byte offset 64. Must have Thumb LSB set and
            // point past the default handler (otherwise the mode-set
            // write didn't land).
            let vec16 = u32::from_le_bytes([
                sc.image[64],
                sc.image[65],
                sc.image[66],
                sc.image[67],
            ]);
            assert_eq!(vec16 & 1, 1,
                "scenario '{}' slot 16 missing Thumb LSB", sc.name);
            let target = vec16 & !1;
            assert_ne!(
                target,
                ISR_IMAGE_BASE + IRQ_DEFAULT_HANDLER_OFFSET,
                "scenario '{}' slot 16 still points at the default handler — \
                 mode-set override did not apply",
                sc.name,
            );
            // Target must be inside the image.
            let image_end = ISR_IMAGE_BASE + IRQ_IMAGE_SIZE as u32;
            assert!(
                target >= ISR_IMAGE_BASE && target < image_end,
                "scenario '{}' slot 16 target 0x{target:08X} is outside image",
                sc.name,
            );
        }
    }

    // (3b) Scenario C specifically populates both slot 16 and 17 (the
    //      preemption scenario's two distinct IRQs). Slot 16 points at
    //      the alternate handler; slot 17 points at the primary.
    #[test]
    fn test_priority_preempt_scenario_populates_two_slots() {
        let sc = SCENARIOS
            .iter()
            .find(|s| s.name == "isr_ext_irq_priority_preempt")
            .expect("priority_preempt scenario must be in catalogue");
        let vec16 = u32::from_le_bytes([
            sc.image[64],  sc.image[65],  sc.image[66],  sc.image[67],
        ]);
        let vec17 = u32::from_le_bytes([
            sc.image[68],  sc.image[69],  sc.image[70],  sc.image[71],
        ]);
        assert_eq!(vec16 & !1,
            ISR_IMAGE_BASE + IRQ_ALT_HANDLER_OFFSET,
            "slot 16 must point at alt handler (low-priority IRQ 0)");
        assert_eq!(vec17 & !1,
            ISR_IMAGE_BASE + IRQ_HANDLER_OFFSET,
            "slot 17 must point at primary handler (high-priority IRQ 1)");
    }

    // (3c) `build_image_modeset` with an empty slice must produce
    //      an image bit-identical to `build_image` — the API migration
    //      contract.
    #[test]
    fn test_build_image_modeset_empty_slice_matches_build_image() {
        const HW: [u16; 2] = [0xBE00, 0xBE00];
        let a = build_image(ISR_IMAGE_BASE, ISR_STACK_TOP, HW, HW);
        let b = build_image_modeset(ISR_IMAGE_BASE, ISR_STACK_TOP, HW, HW, &[]);
        assert_eq!(a, b, "empty mode-set must behave identically to build_image");
    }

    // (3d) DEFAULT_MODESET (PendSV + SysTick at HANDLER_OFFSET) must
    //      produce an image bit-identical to `build_image` — callers
    //      migrating from `build_image` to `build_image_modeset` with
    //      the default set shouldn't change behaviour.
    #[test]
    fn test_build_image_modeset_default_matches_build_image() {
        const HW: [u16; 2] = [0xBE00, 0xBE00];
        let a = build_image(ISR_IMAGE_BASE, ISR_STACK_TOP, HW, HW);
        let b = build_image_modeset(
            ISR_IMAGE_BASE, ISR_STACK_TOP, HW, HW, DEFAULT_MODESET,
        );
        assert_eq!(a, b, "DEFAULT_MODESET must behave identically to build_image");
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

    // (9) Handler must end in bkpt #0 somewhere in the first 12
    //     halfwords — the runner relies on the handler halting. Layout-
    //     aware: baseline scenarios check 0x044, external-IRQ scenarios
    //     check IRQ_HANDLER_OFFSET.
    #[test]
    fn test_handler_contains_bkpt0() {
        for sc in SCENARIOS {
            let layout = layout_for(sc);
            let mut saw_bkpt0 = false;
            for hw in 0..12 {
                let off = (layout.handler_offset as usize) + hw * 2;
                let half = u16::from_le_bytes([sc.image[off], sc.image[off + 1]]);
                if half == 0xBE00 {
                    saw_bkpt0 = true;
                    break;
                }
            }
            assert!(
                saw_bkpt0,
                "scenario '{}' handler body at 0x{:03X} must contain bkpt #0",
                sc.name,
                layout.handler_offset,
            );
        }
    }

    // (10) Main routine must contain a branch-to-self (`b .` = 0xE7FE)
    //      so if the exception never fires, main spins instead of
    //      falling through into the literal pool.
    #[test]
    fn test_main_contains_busy_wait() {
        for sc in SCENARIOS {
            let layout = layout_for(sc);
            let mut saw_busy_wait = false;
            for hw in 0..16 {
                let off = (layout.main_offset as usize) + hw * 2;
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

    /// Every PC-relative LDR in the handler body must target a
    /// word-aligned address inside the handler body (before the main
    /// routine). Layout-aware: baseline scenarios check [0x044..0x100),
    /// external-IRQ scenarios check [IRQ_HANDLER_OFFSET..IRQ_MAIN_OFFSET).
    #[test]
    fn test_handler_literal_loads_target_in_pool() {
        for sc in SCENARIOS {
            let layout = layout_for(sc);
            // External-IRQ scenarios place a second handler body at
            // IRQ_ALT_HANDLER_OFFSET = 0x100 which also emits LDR
            // literals. Walk through IRQ_MAIN_OFFSET so both handler
            // bodies are covered; they share the same pool structure.
            let end = layout.main_offset as usize;
            let loads = collect_ldr_literal_loads(
                sc.image,
                layout.handler_offset as usize,
                end,
            );
            for (instr_off, rd, target, word) in loads {
                assert_eq!(
                    target & 3,
                    0,
                    "scenario '{}' handler LDR at 0x{instr_off:03X} (r{rd}) \
                     computes non-word-aligned target 0x{target:03X}",
                    sc.name,
                );
                assert!(
                    target >= layout.handler_offset as usize && target < end,
                    "scenario '{}' handler LDR at 0x{instr_off:03X} (r{rd}) \
                     resolves to 0x{target:03X} (word 0x{word:08X}) \
                     outside handler pool [0x{:03X}..0x{:03X})",
                    sc.name,
                    layout.handler_offset,
                    end,
                );
            }
        }
    }

    /// POWMAN handler literal-pool strengthening (V13 Stage 2 layout).
    /// The handler must load three literals — `DWT_CYCCNT_ADDR` (into
    /// r0 at hw[0]), `ISR_MAILBOX_BASE` (into r1 at hw[2]), and
    /// `0xCAFE_BABE` (into r0 at hw[4]) — to populate both mailbox
    /// slots. Originally guarded the V11 `0x4902` bug (imm8=2 was
    /// mis-aligned past a non-word-aligned LDR at hw[1], pointing at
    /// the sentinel literal instead of the mailbox address); V13
    /// restructures the handler so all three LDRs sit at word-aligned
    /// positions and imm8 is uniformly 3, but keeps the same "walk the
    /// assembled image and assert literal targets" shape so a future
    /// encoding regression can't silently ship.
    #[test]
    fn test_handler_powman_sentinel_literals_are_mailbox_and_sentinel() {
        let sc = SCENARIOS
            .iter()
            .find(|s| s.name == "isr_powman_match_timer_line_45")
            .expect("POWMAN scenario must exist in the catalogue");
        let layout = layout_for(sc);
        let loads = collect_ldr_literal_loads(
            sc.image,
            layout.handler_offset as usize,
            layout.main_offset as usize,
        );

        // V13 Stage 2: three literal loads in order — r0=DWT_CYCCNT_ADDR,
        // r1=ISR_MAILBOX_BASE, r0=0xCAFE_BABE. Match by address so the
        // `find` calls bind to specific instruction offsets, not just
        // "first LDR r0".
        const DWT_CYCCNT_ADDR: u32 = 0xE000_1004;
        let r0_loads: Vec<u32> = loads
            .iter()
            .filter(|(_, rd, _, _)| *rd == 0)
            .map(|(_, _, _, w)| *w)
            .collect();
        let r1_loads: Vec<u32> = loads
            .iter()
            .filter(|(_, rd, _, _)| *rd == 1)
            .map(|(_, _, _, w)| *w)
            .collect();

        assert!(
            r0_loads.iter().any(|w| *w == DWT_CYCCNT_ADDR),
            "POWMAN handler must LDR r0 ← DWT_CYCCNT_ADDR (0x{:08X}); \
             got r0 loads: {:X?}",
            DWT_CYCCNT_ADDR,
            r0_loads,
        );
        assert!(
            r0_loads.iter().any(|w| *w == 0xCAFE_BABE),
            "POWMAN handler must LDR r0 ← 0xCAFE_BABE sentinel; got \
             r0 loads: {:X?}",
            r0_loads,
        );
        assert!(
            r1_loads.iter().any(|w| *w == crate::ISR_MAILBOX_BASE),
            "POWMAN handler must LDR r1 ← ISR_MAILBOX_BASE \
             (= 0x{:08X}); got r1 loads: {:X?}",
            crate::ISR_MAILBOX_BASE,
            r1_loads,
        );
    }

    /// V12 §3.2 regression: the POWMAN main routine's INTE write must
    /// load `0x5AFE_0002` (`INT_TIMER_BIT | POWMAN password`) into r4
    /// and `0x4010_00E4` (`POWMAN_INTE` address) into r5, then store.
    /// These are at hw[9..11] in [`MAIN_IRQ_POWMAN`] and were appended
    /// in V12 to gate NVIC line 45 — a wrong literal would silently
    /// either (a) miss the password and store nothing on silicon, or
    /// (b) write the wrong bit and never enable INTE.TIMER, so the
    /// scenario would TIMEOUT exactly as it did under V11. Lock the
    /// literal-pool offsets in by walking the assembled image.
    #[test]
    fn test_main_powman_inte_literals_are_inte_addr_and_value() {
        let sc = SCENARIOS
            .iter()
            .find(|s| s.name == "isr_powman_match_timer_line_45")
            .expect("POWMAN scenario must exist in the catalogue");
        let layout = layout_for(sc);
        let main_off = layout.main_offset as usize;

        // The new INTE write triple is hw[9], hw[10], hw[11] in
        // MAIN_IRQ_POWMAN. In the assembled image those land at
        // main_off + 18, main_off + 20, main_off + 22.
        let inte_val_load_off = main_off + 9 * 2;
        let inte_addr_load_off = main_off + 10 * 2;

        let loads = collect_ldr_literal_loads(
            sc.image,
            main_off,
            sc.image.len(),
        );
        let r4_inte_word = loads
            .iter()
            .find(|(addr, rd, _, _)| *addr == inte_val_load_off && *rd == 4)
            .map(|(_, _, _, w)| *w)
            .expect(
                "POWMAN main hw[9] must be `ldr r4, [pc, #X]` resolving \
                 to the INTE write value",
            );
        let r5_inte_addr_word = loads
            .iter()
            .find(|(addr, rd, _, _)| *addr == inte_addr_load_off && *rd == 5)
            .map(|(_, _, _, w)| *w)
            .expect(
                "POWMAN main hw[10] must be `ldr r5, [pc, #X]` resolving \
                 to the POWMAN_INTE register address",
            );

        assert_eq!(
            r4_inte_word, 0x5AFE_0002,
            "POWMAN main r4 INTE write value must be \
             `INT_TIMER_BIT | POWMAN password` = 0x5AFE_0002; \
             got 0x{r4_inte_word:08X}",
        );
        assert_eq!(
            r5_inte_addr_word, 0x4010_00E4,
            "POWMAN main r5 INTE address must be POWMAN_INTE = \
             0x4010_00E4; got 0x{r5_inte_addr_word:08X}",
        );
    }

    /// Every PC-relative LDR in the main body of a **baseline** scenario
    /// must resolve to one of the three v1 literals (DWT_CYCCNT_ADDR,
    /// SCB_ICSR_ADDR, ICSR_PENDSVSET). This is the specific regression
    /// check for the "0x4F04 vs 0x4F05" bug — an off-by-one imm8 in
    /// MAIN_LAZY_FP[9] would load SCB_ICSR_ADDR where ICSR_PENDSVSET
    /// was expected, silently writing ICSR with the wrong value.
    ///
    /// External-IRQ scenarios use a different literal pool layout
    /// (NVIC_ISER / NVIC_ISPR / NVIC_IPR / DWT_CYCCNT_ADDR) and are
    /// validated by [`test_ext_irq_main_literals_resolve_inside_pool`].
    #[test]
    fn test_main_literal_loads_match_expected() {
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
            // Scoped to baseline scenarios only; external-IRQ scenarios
            // have a different literal set.
            if sc.image.len() != ISR_IMAGE_SIZE {
                continue;
            }
            let loads = collect_ldr_literal_loads(
                sc.image,
                MAIN_OFFSET as usize,
                ISR_IMAGE_SIZE,
            );
            assert!(
                !loads.is_empty(),
                "scenario '{}' main routine has no LDR literal loads",
                sc.name,
            );
            for (instr_off, rd, target, word) in loads {
                assert_eq!(
                    target & 3,
                    0,
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     computes non-word-aligned target 0x{target:03X}",
                    sc.name,
                );
                assert!(
                    (0x120..0x12C).contains(&target),
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     resolves to 0x{target:03X} outside the literal pool \
                     [0x120..0x12C)",
                    sc.name,
                );
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

    /// External-IRQ scenarios: every PC-relative LDR in the main body
    /// must resolve to a word-aligned address inside the image and
    /// inside the main routine's literal pool window (past the
    /// branch-to-self). Looser than the baseline check because the
    /// external-IRQ scenarios use varying literal counts (3 for A/B,
    /// 5 for C); the strong form pins each load to a known NVIC MMIO
    /// address or to DWT_CYCCNT_ADDR.
    #[test]
    fn test_ext_irq_main_literals_resolve_inside_pool() {
        // Accept the addresses/constants the external-IRQ scenarios
        // use. Any other word indicates an off-by-one imm8.
        const EXPECTED_WORDS: &[u32] = &[
            0xE000_1004, // DWT_CYCCNT_ADDR
            0xE000_E100, // NVIC_ISER0
            0xE000_E104, // NVIC_ISER1 (POWMAN IRQ 45 sits in bank 1)
            0xE000_E200, // NVIC_ISPR0
            0xE000_E400, // NVIC_IPR0 addr
            0x0000_40C0, // packed priorities: IRQ0=0xC0, IRQ1=0x40
            // POWMAN scenario literals — Coverage Gap Fill V11 §3.2.
            // Password-gated POWMAN writes carry `0x5AFE` in bits [31:16]
            // per pico-sdk `powman.h` (commit a1438dff); silicon drops
            // writes without it and the emulator masks bits [31:16] off
            // on store. RESETS/NVIC writes are not password-gated.
            0x0002_0000, // 1 << RESET_POWMAN (bit 17)
            0x4002_3000, // RESETS_RESET_CLR alias
            0x5AFE_0064, // ALARM value (100 dec) + POWMAN password
            0x4010_0084, // POWMAN_ALARM_TIME_15TO0
            0x5AFE_0012, // TIMER.RUN | TIMER.ALARM_ENAB + POWMAN password
            0x4010_0088, // POWMAN_TIMER
            0x0000_2000, // 1 << 13 (NVIC IRQ 45 bit in bank 1)
            // V12 §3.2: silicon gates NVIC line 45 on `INTE.TIMER`.
            // Scenario writes `INTE = INT_TIMER_BIT | password` before
            // the NVIC enable.
            0x5AFE_0002, // INT_TIMER_BIT (= 1 << 1) + POWMAN password
            0x4010_00E4, // POWMAN_INTE
        ];

        for sc in SCENARIOS {
            if sc.image.len() != IRQ_IMAGE_SIZE {
                continue;
            }
            let loads = collect_ldr_literal_loads(
                sc.image,
                IRQ_MAIN_OFFSET as usize,
                IRQ_IMAGE_SIZE,
            );
            assert!(
                !loads.is_empty(),
                "scenario '{}' ext-IRQ main has no LDR literal loads",
                sc.name,
            );
            for (instr_off, rd, target, word) in loads {
                assert_eq!(
                    target & 3,
                    0,
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     computes non-word-aligned target 0x{target:03X}",
                    sc.name,
                );
                // Target inside the image — specifically in the main-
                // routine region (above IRQ_MAIN_OFFSET).
                assert!(
                    target >= IRQ_MAIN_OFFSET as usize && target < IRQ_IMAGE_SIZE,
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     resolves to 0x{target:03X} outside main pool window",
                    sc.name,
                );
                // Word must be one of the expected NVIC/CYCCNT literals.
                assert!(
                    EXPECTED_WORDS.contains(&word),
                    "scenario '{}' main LDR at 0x{instr_off:03X} (r{rd}) \
                     loads unexpected word 0x{word:08X}",
                    sc.name,
                );
            }
        }
    }

    // ---------------- ISR oracle delta investigation (2026-04-16) -----------

    /// Run a scenario EMU-side and return the mailbox CYCCNT value the
    /// handler wrote. Used by the investigation test below to compare
    /// `isr_pendsv_cold` and `isr_tail_chain_pendsv_systick` in isolation.
    ///
    /// Mirrors the EMU-side half of `run_one_scenario` but skips HW.
    fn run_scenario_emu_mailbox(sc: &IsrScenario) -> u32 {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.core_mut(1).halt();

        // Upload image.
        for chunk_off in (0..sc.image.len()).step_by(4) {
            let word = u32::from_le_bytes([
                sc.image[chunk_off],
                sc.image[chunk_off + 1],
                sc.image[chunk_off + 2],
                sc.image[chunk_off + 3],
            ]);
            emu.poke(crate::ISR_IMAGE_BASE + chunk_off as u32, word);
        }
        emu.bus.invalidate_all();

        emu.mmio_write32(crate::ISR_MAILBOX_CYCCNT, 0);
        emu.mmio_write32(FPCCR_ADDR, FPCCR_ASPEN | FPCCR_LSPEN);

        apply_init_regs_emu(&mut emu, sc.init_regs);
        scenario_preamble_emu(&mut emu, sc.name);

        // Enable DWT CYCCNT.
        let demcr = emu.mmio_read32(crate::silicon_oracle::DEMCR_U32);
        emu.mmio_write32(crate::silicon_oracle::DEMCR_U32,
            demcr | crate::silicon_oracle::TRCENA);
        let dwt_ctrl = emu.mmio_read32(crate::silicon_oracle::DWT_CTRL_U32);
        emu.mmio_write32(crate::silicon_oracle::DWT_CTRL_U32,
            dwt_ctrl | crate::silicon_oracle::CYCCNTENA);
        emu.mmio_write32(crate::silicon_oracle::DWT_CYCCNT_ADDR, 0);

        // Prime core 0.
        {
            let c = emu.core_mut(0);
            c.wake();
            c.regs.set_pc(crate::ISR_IMAGE_BASE + sc.entry_offset);
            c.regs.xpsr = 0x0100_0000;
            c.regs.r[13] = crate::ISR_STACK_TOP;
            c.regs.msp = crate::ISR_STACK_TOP;
            c.regs.r[14] = 0xFFFF_FFFF;
            c.regs.control = 0;
            c.regs.primask = 0;
            c.regs.basepri = 0;
            c.regs.faultmask = 0;
        }

        let budget = sc.max_sysclks as u64;
        let start_cycles = emu.cycles();
        while !emu.core(0).is_halted() && emu.cycles().saturating_sub(start_cycles) < budget {
            emu.step();
        }
        emu.core_mut(0).halt();

        emu.mmio_read32(crate::ISR_MAILBOX_CYCCNT)
    }

    /// Investigation: measure EMU-side mailbox CYCCNT for both scenarios,
    /// confirm the ~10-cycle spread between them (previous session
    /// reported EMU=25 for cold, EMU=15 for tail-chain, both HW=19).
    ///
    /// The two scenarios use identical images — same main, same handler
    /// — so the only EMU-side difference is `scenario_preamble_emu`'s
    /// SysTick arming for the tail-chain case. Whatever mechanism makes
    /// the CYCCNT readings diverge is a function of that state.
    ///
    /// This test prints the values so `cargo test -- --nocapture` shows
    /// the current spread; no assertion is load-bearing on cycle count
    /// (cycle models evolve).
    #[test]
    fn test_investigate_cold_vs_tail_chain_emu_cyccnt() {
        let cold = SCENARIOS.iter().find(|s| s.name == "isr_pendsv_cold").unwrap();
        let tail = SCENARIOS.iter().find(|s| s.name == "isr_tail_chain_pendsv_systick").unwrap();

        let cold_mbx = run_scenario_emu_mailbox(cold);
        let tail_mbx = run_scenario_emu_mailbox(tail);

        eprintln!("=== ISR oracle delta investigation ===");
        eprintln!("isr_pendsv_cold                  mailbox_cyccnt = {cold_mbx}");
        eprintln!("isr_tail_chain_pendsv_systick    mailbox_cyccnt = {tail_mbx}");
        eprintln!("delta (cold - tail)              = {}", cold_mbx as i64 - tail_mbx as i64);
        eprintln!("HW reference (previous session)  = 19 for both");

        // Sanity-check both scenarios actually reached their handler.
        assert!(cold_mbx > 0, "cold scenario never wrote mailbox");
        assert!(tail_mbx > 0, "tail-chain scenario never wrote mailbox");
    }

    // --- V13 Stage 2 — POWMAN scenario CYCCNT observable -----------
    //
    // Red test: the POWMAN scenario must surface both observables —
    // a deterministic sentinel (so handler-ran is binary-clear) AND a
    // CYCCNT delta (so timing drift between HW and EMU is visible in
    // the oracle output, even if not a pass/fail discriminator in its
    // own right). V11/V12 only surfaced the sentinel, mapped through
    // the CycleDelta slot; that loses the timing signal.

    #[test]
    fn powman_scenario_exposes_both_cyccnt_and_sentinel_observables() {
        use crate::{ISR_MAILBOX_CYCCNT, ISR_MAILBOX_RESERVED};

        let sc = SCENARIOS
            .iter()
            .find(|s| s.name == "isr_powman_match_timer_line_45")
            .expect("POWMAN scenario must exist in the catalogue");

        let cyccnt_obs = sc
            .observe
            .iter()
            .find(|(name, _)| *name == "cyccnt_delta")
            .map(|(_, obs)| *obs)
            .expect(
                "POWMAN scenario must expose a `cyccnt_delta` observable \
                 (V13 Stage 2)",
            );
        match cyccnt_obs {
            IsrObservable::CycleDelta => {
                // CycleDelta reads ISR_MAILBOX_CYCCNT; the handler
                // must mailbox its CYCCNT reading there.
            }
            other => panic!(
                "POWMAN `cyccnt_delta` must be the CycleDelta variant \
                 (reads ISR_MAILBOX_CYCCNT = {:#010X}); got {:?}",
                ISR_MAILBOX_CYCCNT, other
            ),
        }

        let sentinel_obs = sc
            .observe
            .iter()
            .find(|(name, _)| *name == "mailbox_sentinel")
            .map(|(_, obs)| *obs)
            .expect(
                "POWMAN scenario must expose a `mailbox_sentinel` \
                 observable (V13 Stage 2)",
            );
        match sentinel_obs {
            IsrObservable::Mmio(addr, mask) => {
                assert_eq!(
                    addr, ISR_MAILBOX_RESERVED,
                    "POWMAN `mailbox_sentinel` must read ISR_MAILBOX_RESERVED \
                     ({:#010X}) — the sentinel slot moved off the CYCCNT \
                     mailbox when cyccnt_delta was added (V13 Stage 2)",
                    ISR_MAILBOX_RESERVED,
                );
                assert_eq!(
                    mask, !0,
                    "sentinel compare must be full-word (exact 0xCAFE_BABE)",
                );
            }
            other => panic!(
                "POWMAN `mailbox_sentinel` must be the Mmio variant at \
                 ISR_MAILBOX_RESERVED; got {:?}",
                other
            ),
        }
    }
}
