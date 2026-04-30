// ISR oracle catalogue for RP2040 / Cortex-M0+ (ARMv6-M). Ports the
// RP2350 oracle (`crates/mdpicoem-harness/src/isr_scenarios.rs`) to the
// simpler M0+ profile: no lazy/eager FP, no CPACR, no Secure /
// Non-Secure world. See `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral
// Coverage V7.md` §6.1 (scenario catalogue) and §4.4 (oracle plumbing).
//
// **Why a separate module.** The RP2350 `build_image` is private (const
// fn) and its 16-entry vector table does not leave room for external
// IRQ vectors — the TIMER_IRQ_0 scenario needs vector[16] (= byte
// offset 0x40), which is exactly where the M33 builder plants its
// `bkpt #1` default-handler landing pad. Rather than modify the M33
// module (explicitly forbidden by the sub-task spec), this module owns
// its own 17-entry vector-table builder and its own scenario images.
// The `pub const` MMIO addresses (ICSR, NVIC_ICPR0, SYST_*) *are*
// imported from the M33 module — they name architectural registers
// that are identical across M33 and M0+.
//
// **Status.** The Phase 1 IRQ plumbing (`Bus::irq_pending` +
// `tick_peripherals` + pending-exception dispatch in
// `CortexM0Plus::step`, plus NVIC ISER/ICER/ISPR/ICPR/IPR0..7 in
// `bus/mod.rs::nvic_mmio_write32` + `nvic_mmio_read32`) has landed.
// All six scenarios (V1 × 2 + V2 × 4) dispatch correctly on the EMU
// side and have been validated against real RP2040 silicon under
// `silicon_isr_diff_rp2040`.

#[cfg(test)]
use crate::ISR_MAILBOX_CYCCNT;
use crate::isr_scenarios::{
    ICSR_PENDSVSET, MAIN_OFFSET as M33_MAIN_OFFSET, NVIC_ICPR0_ADDR, SCB_ICSR_ADDR, SYST_CSR_ADDR,
    SYST_CSR_ENABLE_TICKINT_CORE, SYST_CVR_ADDR, SYST_RVR_ADDR,
};
use crate::silicon_oracle::{self, CaseOutcome, Verdict, enable_cyccnt, reset_cyccnt};
use crate::{ISR_IMAGE_BASE, ISR_STACK_TOP};
use mdrp2040::{Config, EmulatorBuilder};
use probe_rs::{Core, MemoryInterface, RegisterId};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// M0+-specific MMIO constants
// ---------------------------------------------------------------------------

/// VTOR (single alias — M0+ has no Secure / Non-Secure split).
pub const VTOR_ADDR: u32 = 0xE000_ED08;

/// NVIC_ISER0 — Interrupt Set-Enable Register 0 (IRQs 0..31).
pub const NVIC_ISER0_ADDR: u32 = 0xE000_E100;
/// NVIC_ICER0 — Interrupt Clear-Enable Register 0.
pub const NVIC_ICER0_ADDR: u32 = 0xE000_E180;
/// NVIC_ISPR0 — Interrupt Set-Pending Register 0.
pub const NVIC_ISPR0_ADDR: u32 = 0xE000_E200;
/// NVIC_IPR0 — Interrupt Priority Register 0 (priority bytes for IRQs
/// 0..3; M0+ implements bits [7:6] only). Used by the V2 §3.1 priority-
/// preempt scenario to give IRQ #1 a numerically-lower priority value
/// than IRQ #0.
pub const NVIC_IPR0_ADDR: u32 = 0xE000_E400;

/// RP2040 TIMER peripheral base.
pub const TIMER_BASE: u32 = 0x4005_4000;
/// TIMER ALARM0 (write to arm a deadline against TIMERAWL).
pub const TIMER_ALARM0_ADDR: u32 = TIMER_BASE + 0x10;
/// TIMER ALARM1 (write to arm a deadline against TIMERAWL). Used by the
/// V2 §3.1 priority-preempt scenario to fire ALARM0 + ALARM1 at the same
/// TIMERAWL deadline so both NVIC pending bits set in lock-step.
pub const TIMER_ALARM1_ADDR: u32 = TIMER_BASE + 0x14;
/// TIMER TIMERAWL — low 32 bits of the live timer (no latch). Read by
/// the V2 §3.3 WFI scenario to compute a near-future ALARM0 deadline.
pub const TIMER_TIMERAWL_ADDR: u32 = TIMER_BASE + 0x28;
/// TIMER ARMED — W1C disarm register (one bit per ALARM, write-1-to-
/// clear). Used by `reset_scenario_state_*` to disarm any leftover
/// alarms from the previous scenario; without this, `alarm_fire_cycle`
/// slots stay live across the reset cycle and can re-pend INTR.
pub const TIMER_ARMED_ADDR: u32 = TIMER_BASE + 0x20;
/// TIMER INTR — W1C pending alarm bits (bit 0 = ALARM0).
pub const TIMER_INTR_ADDR: u32 = TIMER_BASE + 0x34;
/// TIMER INTE — interrupt enable mask (bit 0 = ALARM0).
pub const TIMER_INTE_ADDR: u32 = TIMER_BASE + 0x38;

/// TIMER_IRQ_0 (IRQ #0). Matches `mdrp2040::irq::IRQ_TIMER_IRQ_0`.
pub const IRQ_TIMER_IRQ_0: u32 = 0;
/// TIMER_IRQ_1 (IRQ #1). Matches `mdrp2040::irq::IRQ_TIMER_IRQ_1`. Used
/// by the V2 §3.1 priority-preempt scenario.
pub const IRQ_TIMER_IRQ_1: u32 = 1;

// ---------------------------------------------------------------------------
// Image layout (M0+ variant)
// ---------------------------------------------------------------------------

/// Size of a scenario image in bytes. Same 0x180 shape as the M33
/// oracle: 0x100 prologue (vector table + handler) + 0x80 main.
pub const ISR_IMAGE_SIZE: usize = 0x180;

/// Byte offset within an image of the PendSV / SysTick / TIMER handler
/// body. Shifted up from the M33 layout (0x044 → 0x04C) to leave room
/// for a 17-entry vector table (68 bytes = 0x44) plus the default-
/// handler landing pad (bkpt #1 + padding = 4 bytes at 0x048).
pub const HANDLER_OFFSET: u32 = 0x04C;

/// Byte offset of the default-handler `bkpt #1`. Any unused vector
/// entry points here so a misfire lands on a distinct halt reason.
pub const DEFAULT_HANDLER_OFFSET: u32 = 0x048;

/// Byte offset within an image of main routine's first instruction.
/// Kept at 0x100 so literal-pool offsets match the M33 main bodies.
pub const MAIN_OFFSET: u32 = 0x100;

// Sanity-check that we didn't drift from the M33's main offset (the
// literal-pool offsets in `MAIN_BASELINE_M0` depend on it).
const _: () = assert!(MAIN_OFFSET == M33_MAIN_OFFSET);

const _: () = assert!(
    (HANDLER_OFFSET as usize) + HANDLER_TAIL.len() * 2 <= MAIN_OFFSET as usize,
    "HANDLER_TAIL must fit between HANDLER_OFFSET and MAIN_OFFSET",
);

/// Number of vector-table entries this oracle allocates. 17 covers
/// everything the scenarios use: [0..15] = standard M-profile + [16] =
/// TIMER_IRQ_0. Rounded up to a 4-byte boundary for trailing code.
const VECTOR_TABLE_ENTRIES: usize = 17;

// ---------------------------------------------------------------------------
// Image builder (M0+ variant — 17-entry vector table)
// ---------------------------------------------------------------------------

/// Assemble a scenario image. Layout:
///
/// ```text
///   + 0x000 .. 0x044  vector table (17 entries × 4 bytes)
///                       [0]  initial MSP    = ISR_STACK_TOP
///                       [1]  Reset_Handler  = base | 1 | MAIN_OFFSET
///                       [2..13] default     = base | 1 | DEFAULT_HANDLER_OFFSET
///                       [14] PendSV         = base | 1 | HANDLER_OFFSET
///                       [15] SysTick        = base | 1 | HANDLER_OFFSET
///                       [16] TIMER_IRQ_0    = base | 1 | HANDLER_OFFSET
///   + 0x044 .. 0x048  zero padding
///   + 0x048 .. 0x04C  default handler: bkpt #1 + padding
///   + 0x04C .. 0x100  handler body (copied from `handler_hw`)
///   + 0x100 .. 0x180  main body + literal pool
/// ```
///
/// Any unused vector-table slot [2..13] points at the default handler
/// at 0x048, so a misfire lands on `bkpt #1` instead of random bytes.
///
/// The builder is `const` to allow the scenarios to hold `&'static
/// [u8]` images built at compile time.
const fn build_image_m0plus<const N_HANDLER_HW: usize, const N_MAIN_HW: usize>(
    image_base: u32,
    stack_top: u32,
    handler_hw: [u16; N_HANDLER_HW],
    main_hw: [u16; N_MAIN_HW],
) -> [u8; ISR_IMAGE_SIZE] {
    let mut out = [0u8; ISR_IMAGE_SIZE];

    let reset_vec = (image_base + MAIN_OFFSET) | 1;
    let default_vec = (image_base + DEFAULT_HANDLER_OFFSET) | 1;
    let handler_vec = (image_base + HANDLER_OFFSET) | 1;

    // Word 0: initial MSP.
    let msp_bytes = stack_top.to_le_bytes();
    out[0] = msp_bytes[0];
    out[1] = msp_bytes[1];
    out[2] = msp_bytes[2];
    out[3] = msp_bytes[3];

    // Word 1: Reset_Handler.
    let rv = reset_vec.to_le_bytes();
    out[4] = rv[0];
    out[5] = rv[1];
    out[6] = rv[2];
    out[7] = rv[3];

    // Words 2..13: default handler.
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

    // Word 14 (PendSV), 15 (SysTick), 16 (TIMER_IRQ_0) all point at
    // the same handler body — scenarios differentiate by which path
    // triggered them, not by handler identity.
    let mut j = 14;
    while j < VECTOR_TABLE_ENTRIES {
        let off = j * 4;
        let b = handler_vec.to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        out[off + 2] = b[2];
        out[off + 3] = b[3];
        j += 1;
    }
    // Vector-table rows 17..18 stay zero (unused on M0+).

    // Default handler at 0x048: bkpt #1 + zero padding to keep the
    // handler body 4-byte aligned.
    out[DEFAULT_HANDLER_OFFSET as usize] = 0x01;
    out[DEFAULT_HANDLER_OFFSET as usize + 1] = 0xBE;
    out[DEFAULT_HANDLER_OFFSET as usize + 2] = 0x00;
    out[DEFAULT_HANDLER_OFFSET as usize + 3] = 0x00;

    // Handler body at HANDLER_OFFSET.
    let mut h = 0;
    while h < N_HANDLER_HW {
        let off = HANDLER_OFFSET as usize + h * 2;
        let b = handler_hw[h].to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        h += 1;
    }

    // Main body at MAIN_OFFSET.
    let mut m = 0;
    while m < N_MAIN_HW {
        let off = MAIN_OFFSET as usize + m * 2;
        let b = main_hw[m].to_le_bytes();
        out[off] = b[0];
        out[off + 1] = b[1];
        m += 1;
    }

    out
}

// ---------------------------------------------------------------------------
// Hand-assembled handler — M0+ / ARMv6-M
// ---------------------------------------------------------------------------
//
// The handler runs at HANDLER_OFFSET = 0x04C. Instead of the M33
// CYCCNT-mailbox handler, this one increments a per-case u32 counter
// in SRAM and (for the TIMER scenario) W1C's the peripheral pending
// flag. M0+ cannot synthesise a wide immediate with `movw`, so the
// addresses come from a tail literal pool.
//
// Two handlers:
//
// * `HANDLER_TIMER` — for the TIMER_IRQ_0 cold-entry scenario. It
//   increments *counter_addr, writes `1` to `TIMER.INTR` (W1C clears
//   the pending alarm bit), and `BX LR` so the core returns via
//   EXC_RETURN. No BKPT — main's trailing BKPT will halt after the
//   handler returns.
//
// * `HANDLER_TAIL` — shared between PendSV and SysTick. It reads IPSR
//   to disambiguate which exception fired, increments the matching
//   counter, and `BX LR`.
//
// The literal pool layout for both variants places the counter base
// address in the pool; the handler reads IPSR[8:0] (via `mrs`) to
// compute the counter offset.
//
// Every halfword is hand-assembled. Encoding notes:
//
//   ldr rD, [pc, #imm8*4]         T1 = 0b01001_RRR_IIIIIIII
//   ldr rD, [rN, #imm5*4]         T1 = 0b01101_III_NNN_DDD
//   str rD, [rN, #imm5*4]         T1 = 0b01100_III_NNN_DDD
//   adds rD, rN, #imm3            T1 = 0b0001110_III_NNN_DDD
//   mrs rD, IPSR                  T1 wide, hw0=0xF3EF hw1=0x80_05 (Rd=0)
//                                 hw0=0xF3EF hw1=0x8005 | (Rd<<8)
//   bx rN                         T1 = 0b010001110_NNNN_000 = 0x4700|(N<<3)

// Counter addresses — distinct per-scenario SRAM slots above the
// stack. Placed in the top 16 bytes of the mailbox page so they sit
// well clear of exception-stacked frames (ISR_STACK_TOP = 0x2000_3000
// grows down).
//
// Layout inside the mailbox page:
//   + 0xFC8 — V2 §3.3 phase cell (main writes 1, wfi, then 2)
//   + 0xFCC — V2 §3.3 phase_at_entry (handler captures phase on entry)
//   + 0xFD0 — V2 §3.4 ISER readback (RAZ/WI scenario, primary obs)
//   + 0xFD4 — V2 §3.4 ISPR readback
//   + 0xFD8 — V2 §3.4 ICER readback
//   + 0xFDC — V2 §3.4 ICPR readback
//   + 0xFE0 — TIMER scenario counter
//   + 0xFE4 — PendSV scenario counter
//   + 0xFE8 — SysTick scenario counter
//   + 0xFEC — V2 §3.2 gate cell (main writes 1 then 2)
//   + 0xFF0 — V2 §3.2 gate_at_entry (handler captures gate on entry)
//   + 0xFF4 — reserved (free slot)
//   + 0xFF8 — ISR_MAILBOX_CYCCNT (defined in lib.rs; reserved for the
//             RP2350 oracle, kept clear here for parity)
//
// Stage 4 placed PHASE/PHASE_AT_ENTRY *below* the existing block at
// 0xFC8/0xFCC because moving any of the existing cells would force
// edits to Stage 2 / Stage 3 handlers' literal pools, which is
// explicitly out of scope for Stage 4. Stage 5 followed the same
// downward-grow pattern, placing the priority-preempt cells contiguously
// below at 0xFB8/0xFBC/0xFC0.
//
// Stage 5 layout (priority-preempt scenario — V2 §3.1):
//   + 0xFB8 — CTR_IRQ_0
//   + 0xFBC — CTR_IRQ_1
//   + 0xFC0 — ORDER_FIRST_IRQ (primary observable; written once with
//             a non-zero sentinel by whichever handler ran first)
/// V2 §3.1 — counter for HANDLER_IRQ0 (TIMER_IRQ_0 / ALARM0).
pub const CTR_IRQ_0_ADDR: u32 = 0x2000_3FB8;
/// V2 §3.1 — counter for HANDLER_IRQ1 (TIMER_IRQ_1 / ALARM1).
pub const CTR_IRQ_1_ADDR: u32 = 0x2000_3FBC;
/// V2 §3.1 — observable: which handler ran first. Each handler writes
/// its sentinel (`HANDLER_IRQ0 = 0xA0`, `HANDLER_IRQ1 = 0xA1`) only if
/// this cell is currently zero. PASS on `order_first_irq == 0xA1` —
/// IRQ_1 has lower priority value (= higher priority) and must dispatch
/// first; IRQ_0 then runs via tail-chain.
pub const ORDER_FIRST_IRQ_ADDR: u32 = 0x2000_3FC0;
/// V2 §3.3 — phase cell. Main writes `1` before `wfi`, then `2` after
/// the handler returns and main resumes. PASS on `phase == 2`.
pub const PHASE_ADDR: u32 = 0x2000_3FC8;
/// V2 §3.3 — observable: phase value at handler entry. The handler
/// stores `*phase` into this cell. PASS on `phase_at_entry == 1`,
/// proving the handler dispatched during the WFI window (between the
/// `phase=1` and `phase=2` stores).
pub const PHASE_AT_ENTRY_ADDR: u32 = 0x2000_3FCC;
pub const ISER_READBACK_ADDR: u32 = 0x2000_3FD0;
pub const ISPR_READBACK_ADDR: u32 = 0x2000_3FD4;
pub const ICER_READBACK_ADDR: u32 = 0x2000_3FD8;
pub const ICPR_READBACK_ADDR: u32 = 0x2000_3FDC;
pub const CTR_TIMER_ADDR: u32 = 0x2000_3FE0;
pub const CTR_PENDSV_ADDR: u32 = 0x2000_3FE4;
pub const CTR_SYSTICK_ADDR: u32 = 0x2000_3FE8;
/// V2 §3.2 — gate cell. Main writes `1` before `cpsie i`, `2` after. On
/// architecturally-correct M0+ the handler runs at the `cpsie i`
/// boundary, so the handler's read of `gate` returns `1`.
pub const GATE_ADDR: u32 = 0x2000_3FEC;
/// V2 §3.2 — observable: gate value at handler entry. The handler
/// stores `*gate` into this cell. PASS on `gate_at_entry == 1`.
pub const GATE_AT_ENTRY_ADDR: u32 = 0x2000_3FF0;

/// Handler for the TIMER_IRQ_0 scenario.
///
/// **Order matters** (mirroring pico-sdk's standard timer-ISR pattern):
///   1. W1C TIMER.INTR — drops the peripheral's level-asserted line
///      so `poll_alarms`'s level re-assert path stops re-pending NVIC.
///   2. W1C NVIC_ICPR0 bit 0 — clears any NVIC.pending state set by
///      the level re-assert during the handler's pre-W1C prefix
///      (~3 instr × 1-2 cycles = several slow-path ticks where
///      `tick_peripherals` re-OR'd `1<<IRQ_TIMER_IRQ_0` into
///      `bus.irq_pending`).
///   3. Counter increment — observable.
///   4. `BX LR` — return via EXC_RETURN.
///
/// Without steps 1+2, `exit_exception`'s tail-chain poll sees
/// NVIC.pending bit 0 still set from level re-assertion during
/// the handler body, and re-dispatches the same alarm — making
/// the V1 oracle's `ctr_timer == 1` assertion unsatisfiable. Real
/// RP2040 silicon has the same level-pending semantics; this
/// pattern mirrors `pico-sdk/src/rp2_common/hardware_timer/timer.c`.
///
/// Encoding: hw[0/3/5] are all `ldr r0, [pc, #28]` (different
/// instruction PCs but the same literal-pool offset of 0x1C from
/// the post-fetch aligned PC). Verified literal math below.
const HANDLER_TIMER: [u16; 22] = [
    // Phase 1: W1C TIMER.INTR (drops level immediately)
    0x4807, // [ 0] ldr  r0, [pc, #28]   — TIMER_INTR_ADDR (lit hw[16])
    0x2101, // [ 1] movs r1, #1
    0x6001, // [ 2] str  r1, [r0]        — TIMER.INTR = 1 (W1C bit 0)
    // Phase 2: W1C NVIC_ICPR0 bit 0 (clears stale pending from
    // level re-assertions during phase 1's prefix)
    0x4807, // [ 3] ldr  r0, [pc, #28]   — NVIC_ICPR0_ADDR (lit hw[18])
    0x6001, // [ 4] str  r1, [r0]        — NVIC_ICPR0 = 1 (r1 still 1)
    // Phase 3: Increment counter
    0x4807, // [ 5] ldr  r0, [pc, #28]   — CTR_TIMER_ADDR (lit hw[20])
    0x6801, // [ 6] ldr  r1, [r0]
    0x3101, // [ 7] adds r1, #1
    0x6001, // [ 8] str  r1, [r0]
    // Phase 4: Return via EXC_RETURN
    0x4770, // [ 9] bx   lr
    0xBF00, // [10] nop padding
    0xBF00, // [11] nop padding
    0xBF00, // [12] nop padding
    0xBF00, // [13] nop padding
    0xBF00, // [14] nop padding
    0xBF00, // [15] nop padding
    0x4034, // [16] lit: TIMER_INTR_ADDR  low  (0x4005_4034)
    0x4005, // [17] lit: TIMER_INTR_ADDR  high
    0xE280, // [18] lit: NVIC_ICPR0_ADDR  low  (0xE000_E280)
    0xE000, // [19] lit: NVIC_ICPR0_ADDR  high
    0x3FE0, // [20] lit: CTR_TIMER_ADDR   low  (0x2000_3FE0)
    0x2000, // [21] lit: CTR_TIMER_ADDR   high
];

// Pin literal-pool byte offsets to keep hw[0/3/5] ldr math stable.
const _: () = assert!(
    HANDLER_TIMER[16] == 0x4034 && HANDLER_TIMER[17] == 0x4005,
    "TIMER_INTR_ADDR literal must remain at hw[16..=17]",
);
const _: () = assert!(
    HANDLER_TIMER[18] == 0xE280 && HANDLER_TIMER[19] == 0xE000,
    "NVIC_ICPR0_ADDR literal must remain at hw[18..=19]",
);
const _: () = assert!(
    HANDLER_TIMER[20] == 0x3FE0 && HANDLER_TIMER[21] == 0x2000,
    "CTR_TIMER_ADDR literal must remain at hw[20..=21]",
);
const _: () = assert!(
    (HANDLER_OFFSET as usize) + HANDLER_TIMER.len() * 2 <= MAIN_OFFSET as usize,
    "HANDLER_TIMER must fit between HANDLER_OFFSET and MAIN_OFFSET",
);

/// Shared handler for PendSV + SysTick in the tail-chain scenario.
///
/// Dispatch on IPSR — on M0+ `mrs rD, IPSR` reads the low 9 bits of
/// xPSR. PendSV = 14, SysTick = 15. The handler increments whichever
/// counter matches, disables SysTick, clears any pending PendSV +
/// SysTick bits in ICSR, and returns via `BX LR`.
///
/// The ICSR pend-clear is load-bearing: SysTick will re-fire during
/// the ~22-cycle handler before the SYST_CSR=0 disable lands at hw[13],
/// latching PENDSTSET. Without an explicit W1C clear, `exit_exception`'s
/// tail-chain poll would dispatch SysTick a second time and the
/// `ctr_systick == 1` invariant would fail. PENDSTCLR (bit 25) +
/// PENDSVCLR (bit 27) → mask 0x0A00_0000.
///
/// ```text
///   [ 0] mrs  r2, IPSR       ; r2 = exception number  (hw0=0xF3EF, hw1=0x8205)
///   [ 2] cmp  r2, #14        ; PendSV?
///   [ 3] bne  hw[6]          ; skip PendSV increment if r2 != 14
///   [ 4] ldr  r0, [pc, #32]  ; r0 = CTR_PENDSV_ADDR    (lit [22])
///   [ 5] b    hw[8]          ; jump to common increment
///   [ 6] ldr  r0, [pc, #32]  ; r0 = CTR_SYSTICK_ADDR   (lit [24])
///   [ 7] nop                 ; alignment padding
///   [ 8] ldr  r1, [r0]       ; common increment
///   [ 9] adds r1, #1
///   [10] str  r1, [r0]
///   [11] ldr  r3, [pc, #28]  ; r3 = SYST_CSR_ADDR      (lit [26])
///   [12] movs r4, #0
///   [13] str  r4, [r3]       ; *SYST_CSR = 0 (disable SysTick)
///   [14] ldr  r3, [pc, #24]  ; r3 = ICSR_ADDR          (lit [28])
///   [15] ldr  r4, [pc, #28]  ; r4 = PENDST/PENDSV clear mask (lit [30])
///   [16] str  r4, [r3]       ; *ICSR = PENDSTCLR | PENDSVCLR
///   [17] bx   lr
///   [18..21] padding
///   [22..23] lit: CTR_PENDSV_ADDR
///   [24..25] lit: CTR_SYSTICK_ADDR
///   [26..27] lit: SYST_CSR_ADDR
///   [28..29] lit: ICSR_ADDR (0xE000_ED04)
///   [30..31] lit: ICSR clear mask (0x0A00_0000)
/// ```
///
/// Wide instruction: `mrs r2, IPSR` = hw0=0xF3EF, hw1=0x8205 (Rd=2,
/// SYSm=5 for IPSR).
///
/// Literal math — `ldr rD, [pc, #imm8*4]` on ARMv6-M uses PC rounded
/// down to a 4-byte boundary plus imm8*4. hw[i] byte = 0x04C + 2*i:
///
///   hw[4]  `ldr r0, [pc, #32]`: instr 0x054, PC 0x058, Align 0x058,
///     target hw[22] at 0x078 → imm8*4 = 32 → imm8 = 8. Encoding 0x4808.
///   hw[6]  `ldr r0, [pc, #32]`: instr 0x058, PC 0x05C, Align 0x05C,
///     target hw[24] at 0x07C → imm8*4 = 32 → imm8 = 8. Encoding 0x4808.
///   hw[11] `ldr r3, [pc, #28]`: instr 0x062, PC 0x066, Align 0x064,
///     target hw[26] at 0x080 → imm8*4 = 28 → imm8 = 7. Encoding 0x4B07.
///   hw[14] `ldr r3, [pc, #24]`: instr 0x068, PC 0x06C, Align 0x06C,
///     target hw[28] at 0x084 → imm8*4 = 24 → imm8 = 6. Encoding 0x4B06.
///   hw[15] `ldr r4, [pc, #28]`: instr 0x06A, PC 0x06E, Align 0x06C,
///     target hw[30] at 0x088 → imm8*4 = 28 → imm8 = 7. Encoding 0x4C07.
///
/// Branches on ARMv6-M — ARMv6-M T1 `bne` uses signed imm8 scaled by 2,
/// target = PC + imm8*2, with PC = branch_addr + 4 (the usual ARM rule).
/// HANDLER_TAIL lives at image offset 0x04C, so byte addresses of the
/// halfwords are:
///
///   hw[3] bne at 0x052, PC = 0x056
///   hw[4] ldr at 0x054
///   hw[5] b   at 0x056
///   hw[6] ldr at 0x058
///
///   * `bne` at hw[3]: skip the two PendSV halfwords (hw[4] `ldr` and
///     hw[5] `b`) and land at hw[6] = 0x058. imm8*2 = 0x058 - 0x056 = 2
///     → imm8 = 1. Encoding: 0b1101_0001_00000001 = 0xD101. (An earlier
///     draft used 0xD102, which lands at hw[7] = 0x05A = the nop; the
///     SysTick path then jumped into the common increment with an
///     uninitialised r0 — a silent regression caught in review.)
///   * `b` at hw[5] (byte 0x056, PC = 0x05A): target hw[8] (common
///     increment) at 0x05C. imm11*2 = 0x05C - 0x05A = 2 → imm11 = 1.
///     T2 encoding: 0b11100_00000000001 = 0xE001. (This is `b .+2` in
///     the standard ARM "relative to PC" notation — PC already includes
///     the pipeline offset.)
const HANDLER_TAIL: [u16; 32] = [
    0xF3EF, // [ 0] mrs r2, IPSR — hw0
    0x8205, // [ 1] mrs r2, IPSR — hw1 (Rd=2, SYSm=5)
    0x2A0E, // [ 2] cmp r2, #14   — PendSV number
    0xD101, // [ 3] bne hw[6]    — skip PendSV ldr/b if not PendSV
    0x4808, // [ 4] ldr r0, [pc, #32] — CTR_PENDSV_ADDR
    0xE001, // [ 5] b   .+2       — jump to common [8]
    0x4808, // [ 6] ldr r0, [pc, #32] — CTR_SYSTICK_ADDR
    0xBF00, // [ 7] nop           — alignment padding
    0x6801, // [ 8] ldr r1, [r0]  — common increment
    0x3101, // [ 9] adds r1, #1
    0x6001, // [10] str r1, [r0]
    0x4B07, // [11] ldr r3, [pc, #28] — SYST_CSR_ADDR
    0x2400, // [12] movs r4, #0
    0x601C, // [13] str r4, [r3]   — *SYST_CSR = 0 (disable SysTick)
    0x4B06, // [14] ldr r3, [pc, #24] — ICSR_ADDR
    0x4C07, // [15] ldr r4, [pc, #28] — PENDST/PENDSV clear mask
    0x601C, // [16] str r4, [r3]   — *ICSR = PENDSTCLR | PENDSVCLR
    0x4770, // [17] bx  lr
    0xBF00, // [18] nop padding
    0xBF00, // [19] nop padding
    0xBF00, // [20] nop padding
    0xBF00, // [21] nop padding
    0x3FE4, // [22] lit: CTR_PENDSV_ADDR low
    0x2000, // [23] lit: CTR_PENDSV_ADDR high
    0x3FE8, // [24] lit: CTR_SYSTICK_ADDR low
    0x2000, // [25] lit: CTR_SYSTICK_ADDR high
    0xE010, // [26] lit: SYST_CSR_ADDR low
    0xE000, // [27] lit: SYST_CSR_ADDR high
    0xED04, // [28] lit: ICSR_ADDR low (0xE000_ED04)
    0xE000, // [29] lit: ICSR_ADDR high
    0x0000, // [30] lit: ICSR clear mask low (0x0A00_0000)
    0x0A00, // [31] lit: ICSR clear mask high
];

// Pin byte offset of the literal-pool entries that hw[11], hw[14], and
// hw[15]'s `ldr [pc, #imm]` math depend on.
const _: () = assert!(
    HANDLER_TAIL[26] == 0xE010 && HANDLER_TAIL[27] == 0xE000,
    "SYST_CSR_ADDR literal must remain at hw[26..=27] for hw[11] ldr math",
);
const _: () = assert!(
    HANDLER_TAIL[28] == 0xED04 && HANDLER_TAIL[29] == 0xE000,
    "ICSR_ADDR literal must remain at hw[28..=29] for hw[14] ldr math",
);
const _: () = assert!(
    HANDLER_TAIL[30] == 0x0000 && HANDLER_TAIL[31] == 0x0A00,
    "ICSR clear mask must remain at hw[30..=31] for hw[15] ldr math",
);

/// Handler for V2 §3.2 `isr_m0_masked_pending_unmask`. Mirrors
/// HANDLER_TIMER's "W1C TIMER.INTR → W1C NVIC_ICPR0 → counter" prefix
/// (so a re-pend during the handler body doesn't re-dispatch — although
/// for this scenario TIMER.INTE is intentionally left disabled by main,
/// see MAIN_MASKED), but adds an extra observable: load `*GATE_ADDR`
/// into r5 then store it into `GATE_AT_ENTRY_ADDR`. That captures
/// whether the handler ran *between* main's `gate=1` and `gate=2`
/// stores — the load-bearing assertion for the PRIMASK-gate / `cpsie i`
/// dispatch-boundary check.
///
/// Layout — image bytes 0x04C..0x080:
/// ```text
///   [ 0] ldr  r0, [pc, #28]    ; r0 = TIMER_INTR_ADDR        (lit hw[16])
///   [ 1] movs r1, #1
///   [ 2] str  r1, [r0]         ; W1C TIMER.INTR bit 0
///   [ 3] ldr  r0, [pc, #28]    ; r0 = NVIC_ICPR0_ADDR        (lit hw[18])
///   [ 4] str  r1, [r0]         ; W1C NVIC pending bit 0
///   [ 5] ldr  r2, [pc, #28]    ; r2 = GATE_ADDR              (lit hw[20])
///   [ 6] ldr  r5, [r2]         ; r5 = *gate
///   [ 7] ldr  r3, [pc, #28]    ; r3 = GATE_AT_ENTRY_ADDR     (lit hw[22])
///   [ 8] str  r5, [r3]         ; *gate_at_entry = r5
///   [ 9] ldr  r0, [pc, #28]    ; r0 = CTR_TIMER_ADDR         (lit hw[24])
///   [10] ldr  r1, [r0]
///   [11] adds r1, #1
///   [12] str  r1, [r0]         ; ctr_timer += 1
///   [13] bx   lr
///   [14..15] nop padding
///   [16..17] lit: TIMER_INTR_ADDR     (0x4005_4034)
///   [18..19] lit: NVIC_ICPR0_ADDR     (0xE000_E280)
///   [20..21] lit: GATE_ADDR           (0x2000_3FEC)
///   [22..23] lit: GATE_AT_ENTRY_ADDR  (0x2000_3FF0)
///   [24..25] lit: CTR_TIMER_ADDR      (0x2000_3FE0)
/// ```
///
/// Literal math — handler body lives at HANDLER_OFFSET = 0x04C, so
/// hw[i] byte = 0x04C + 2*i. Each `ldr [pc, #imm8*4]` aligns PC down
/// to a 4-byte boundary first.
///
///   hw[ 0] addr 0x04C, PC 0x050, Align 0x050, target hw[16]=0x06C → imm=0x1C → imm8=7 → 0x4807
///   hw[ 3] addr 0x052, PC 0x056, Align 0x054, target hw[18]=0x070 → imm=0x1C → imm8=7 → 0x4807
///   hw[ 5] addr 0x056, PC 0x05A, Align 0x058, target hw[20]=0x074 → imm=0x1C → imm8=7 → 0x4A07
///   hw[ 7] addr 0x05A, PC 0x05E, Align 0x05C, target hw[22]=0x078 → imm=0x1C → imm8=7 → 0x4B07
///   hw[ 9] addr 0x05E, PC 0x062, Align 0x060, target hw[24]=0x07C → imm=0x1C → imm8=7 → 0x4807
const HANDLER_MASKED: [u16; 26] = [
    // Phase 1: W1C TIMER.INTR (defensive — INTE is left off by main, but
    // mirrors HANDLER_TIMER's pico-sdk-shaped prefix so a future variant
    // that enables INTE doesn't re-dispatch).
    0x4807, // [ 0] ldr  r0, [pc, #28]   — TIMER_INTR_ADDR
    0x2101, // [ 1] movs r1, #1
    0x6001, // [ 2] str  r1, [r0]        — TIMER.INTR = 1 (W1C bit 0)
    // Phase 2: W1C NVIC_ICPR0 bit 0 (clears the firmware-set pending bit
    // so EXC_RETURN's tail-chain poll doesn't re-dispatch).
    0x4807, // [ 3] ldr  r0, [pc, #28]   — NVIC_ICPR0_ADDR
    0x6001, // [ 4] str  r1, [r0]        — NVIC_ICPR0 = 1 (r1 still 1)
    // Phase 3: capture *gate into gate_at_entry (LOAD-BEARING).
    0x4A07, // [ 5] ldr  r2, [pc, #28]   — GATE_ADDR
    0x6815, // [ 6] ldr  r5, [r2]        — r5 = *gate
    0x4B07, // [ 7] ldr  r3, [pc, #28]   — GATE_AT_ENTRY_ADDR
    0x601D, // [ 8] str  r5, [r3]        — *gate_at_entry = r5
    // Phase 4: ctr_timer += 1.
    0x4807, // [ 9] ldr  r0, [pc, #28]   — CTR_TIMER_ADDR
    0x6801, // [10] ldr  r1, [r0]
    0x3101, // [11] adds r1, #1
    0x6001, // [12] str  r1, [r0]
    // Phase 5: return via EXC_RETURN.
    0x4770, // [13] bx   lr
    0xBF00, // [14] nop padding
    0xBF00, // [15] nop padding
    0x4034, // [16] lit: TIMER_INTR_ADDR     low  (0x4005_4034)
    0x4005, // [17] lit: TIMER_INTR_ADDR     high
    0xE280, // [18] lit: NVIC_ICPR0_ADDR     low  (0xE000_E280)
    0xE000, // [19] lit: NVIC_ICPR0_ADDR     high
    0x3FEC, // [20] lit: GATE_ADDR           low  (0x2000_3FEC)
    0x2000, // [21] lit: GATE_ADDR           high
    0x3FF0, // [22] lit: GATE_AT_ENTRY_ADDR  low  (0x2000_3FF0)
    0x2000, // [23] lit: GATE_AT_ENTRY_ADDR  high
    0x3FE0, // [24] lit: CTR_TIMER_ADDR      low  (0x2000_3FE0)
    0x2000, // [25] lit: CTR_TIMER_ADDR      high
];

// Pin literal-pool byte offsets — every `ldr [pc, #imm]` above depends
// on these slots staying put.
const _: () = assert!(
    HANDLER_MASKED[16] == 0x4034 && HANDLER_MASKED[17] == 0x4005,
    "TIMER_INTR_ADDR literal must remain at hw[16..=17]",
);
const _: () = assert!(
    HANDLER_MASKED[18] == 0xE280 && HANDLER_MASKED[19] == 0xE000,
    "NVIC_ICPR0_ADDR literal must remain at hw[18..=19]",
);
const _: () = assert!(
    HANDLER_MASKED[20] == 0x3FEC && HANDLER_MASKED[21] == 0x2000,
    "GATE_ADDR literal must remain at hw[20..=21]",
);
const _: () = assert!(
    HANDLER_MASKED[22] == 0x3FF0 && HANDLER_MASKED[23] == 0x2000,
    "GATE_AT_ENTRY_ADDR literal must remain at hw[22..=23]",
);
const _: () = assert!(
    HANDLER_MASKED[24] == 0x3FE0 && HANDLER_MASKED[25] == 0x2000,
    "CTR_TIMER_ADDR literal must remain at hw[24..=25]",
);
const _: () = assert!(
    (HANDLER_OFFSET as usize) + HANDLER_MASKED.len() * 2 <= MAIN_OFFSET as usize,
    "HANDLER_MASKED must fit between HANDLER_OFFSET and MAIN_OFFSET",
);

/// Handler for V2 §3.3 `isr_m0_wfi_wake`. Identical structural shape to
/// HANDLER_MASKED — same prefix, same instruction encodings, just the
/// PHASE / PHASE_AT_ENTRY literals replace GATE / GATE_AT_ENTRY. The
/// handler's load-bearing observable is `*phase` captured at entry,
/// proving the dispatch happened during the WFI window (between main's
/// `*phase=1` and `*phase=2` stores).
///
/// Layout — image bytes 0x04C..0x080:
/// ```text
///   [ 0] ldr  r0, [pc, #28]    ; r0 = TIMER_INTR_ADDR        (lit hw[16])
///   [ 1] movs r1, #1
///   [ 2] str  r1, [r0]         ; W1C TIMER.INTR bit 0 (load-bearing —
///                              ;   ALARM0 INTE is enabled by main, so
///                              ;   without this the level re-asserts)
///   [ 3] ldr  r0, [pc, #28]    ; r0 = NVIC_ICPR0_ADDR        (lit hw[18])
///   [ 4] str  r1, [r0]         ; W1C NVIC pending bit 0
///   [ 5] ldr  r2, [pc, #28]    ; r2 = PHASE_ADDR             (lit hw[20])
///   [ 6] ldr  r5, [r2]         ; r5 = *phase
///   [ 7] ldr  r3, [pc, #28]    ; r3 = PHASE_AT_ENTRY_ADDR    (lit hw[22])
///   [ 8] str  r5, [r3]         ; *phase_at_entry = r5
///   [ 9] ldr  r0, [pc, #28]    ; r0 = CTR_TIMER_ADDR         (lit hw[24])
///   [10] ldr  r1, [r0]
///   [11] adds r1, #1
///   [12] str  r1, [r0]         ; ctr_timer += 1
///   [13] bx   lr
///   [14..15] nop padding
///   [16..17] lit: TIMER_INTR_ADDR     (0x4005_4034)
///   [18..19] lit: NVIC_ICPR0_ADDR     (0xE000_E280)
///   [20..21] lit: PHASE_ADDR          (0x2000_3FC8)
///   [22..23] lit: PHASE_AT_ENTRY_ADDR (0x2000_3FCC)
///   [24..25] lit: CTR_TIMER_ADDR      (0x2000_3FE0)
/// ```
///
/// Literal math is identical to HANDLER_MASKED — every `ldr [pc, #imm]`
/// targets the next-but-one literal slot at imm8=7 (28 bytes). See
/// HANDLER_MASKED above for the per-instruction derivation.
const HANDLER_WFI: [u16; 26] = [
    // Phase 1: W1C TIMER.INTR — drops level immediately so the alarm
    // doesn't re-pend NVIC after the handler clears ICPR0 below.
    0x4807, // [ 0] ldr  r0, [pc, #28]   — TIMER_INTR_ADDR
    0x2101, // [ 1] movs r1, #1
    0x6001, // [ 2] str  r1, [r0]        — TIMER.INTR = 1 (W1C bit 0)
    // Phase 2: W1C NVIC_ICPR0 bit 0 (clears NVIC pending state set by
    // the alarm fire so EXC_RETURN's tail-chain poll doesn't re-dispatch).
    0x4807, // [ 3] ldr  r0, [pc, #28]   — NVIC_ICPR0_ADDR
    0x6001, // [ 4] str  r1, [r0]        — NVIC_ICPR0 = 1 (r1 still 1)
    // Phase 3: capture *phase into phase_at_entry (LOAD-BEARING).
    0x4A07, // [ 5] ldr  r2, [pc, #28]   — PHASE_ADDR
    0x6815, // [ 6] ldr  r5, [r2]        — r5 = *phase
    0x4B07, // [ 7] ldr  r3, [pc, #28]   — PHASE_AT_ENTRY_ADDR
    0x601D, // [ 8] str  r5, [r3]        — *phase_at_entry = r5
    // Phase 4: ctr_timer += 1.
    0x4807, // [ 9] ldr  r0, [pc, #28]   — CTR_TIMER_ADDR
    0x6801, // [10] ldr  r1, [r0]
    0x3101, // [11] adds r1, #1
    0x6001, // [12] str  r1, [r0]
    // Phase 5: return via EXC_RETURN.
    0x4770, // [13] bx   lr
    0xBF00, // [14] nop padding
    0xBF00, // [15] nop padding
    0x4034, // [16] lit: TIMER_INTR_ADDR     low  (0x4005_4034)
    0x4005, // [17] lit: TIMER_INTR_ADDR     high
    0xE280, // [18] lit: NVIC_ICPR0_ADDR     low  (0xE000_E280)
    0xE000, // [19] lit: NVIC_ICPR0_ADDR     high
    0x3FC8, // [20] lit: PHASE_ADDR          low  (0x2000_3FC8)
    0x2000, // [21] lit: PHASE_ADDR          high
    0x3FCC, // [22] lit: PHASE_AT_ENTRY_ADDR low  (0x2000_3FCC)
    0x2000, // [23] lit: PHASE_AT_ENTRY_ADDR high
    0x3FE0, // [24] lit: CTR_TIMER_ADDR      low  (0x2000_3FE0)
    0x2000, // [25] lit: CTR_TIMER_ADDR      high
];

// Pin literal-pool byte offsets — every `ldr [pc, #imm]` above depends
// on these slots staying put.
const _: () = assert!(
    HANDLER_WFI[16] == 0x4034 && HANDLER_WFI[17] == 0x4005,
    "TIMER_INTR_ADDR literal must remain at hw[16..=17]",
);
const _: () = assert!(
    HANDLER_WFI[18] == 0xE280 && HANDLER_WFI[19] == 0xE000,
    "NVIC_ICPR0_ADDR literal must remain at hw[18..=19]",
);
const _: () = assert!(
    HANDLER_WFI[20] == 0x3FC8 && HANDLER_WFI[21] == 0x2000,
    "PHASE_ADDR literal must remain at hw[20..=21]",
);
const _: () = assert!(
    HANDLER_WFI[22] == 0x3FCC && HANDLER_WFI[23] == 0x2000,
    "PHASE_AT_ENTRY_ADDR literal must remain at hw[22..=23]",
);
const _: () = assert!(
    HANDLER_WFI[24] == 0x3FE0 && HANDLER_WFI[25] == 0x2000,
    "CTR_TIMER_ADDR literal must remain at hw[24..=25]",
);
const _: () = assert!(
    (HANDLER_OFFSET as usize) + HANDLER_WFI.len() * 2 <= MAIN_OFFSET as usize,
    "HANDLER_WFI must fit between HANDLER_OFFSET and MAIN_OFFSET",
);

/// V2 §3.1 — Two distinct handler bodies concatenated into one array
/// for image-builder convenience.
///
/// HANDLER_IRQ0 occupies hw[0..24] (24 halfwords), HANDLER_IRQ1
/// occupies hw[24..48] (24 halfwords). The image-builder plants
/// vector entry [16] at HANDLER_OFFSET (HANDLER_IRQ0 entry) and the
/// scenario const-time-patches vector entry [17] to point at
/// HANDLER_OFFSET + 24*2 = HANDLER_OFFSET + 48 (HANDLER_IRQ1 entry).
/// See `IMAGE_PRIORITY_PREEMPT` and `patch_vector_entry`.
///
/// Layout choice. Two distinct handlers (rather than one IPSR-reading
/// handler) is per HLD §3.1: simpler, no `mrs` in the handler hot path,
/// no shared counter array. We keep both bodies in a single `[u16; N]`
/// because `build_image_m0plus` already handles a single handler array
/// — patching only vector entry [17] avoids modifying the builder's
/// signature (and so cannot regress V1 / Stage 2-4 scenarios).
///
/// **Sentinel values for `order_first_irq`.** The cell starts at zero
/// (cleared by `reset_scenario_state_*`). Each handler's "is this the
/// first to run?" check is `cmp r3, #0; bne skip; ...`, so we must use
/// a non-zero sentinel — IRQ_0 writes `0xA0`, IRQ_1 writes `0xA1`. PASS
/// is `order_first_irq == 0xA1` (IRQ_1 ran first).
///
/// **HANDLER_IRQ0 layout** — bytes 0x04C..0x07C inside the image, hw
/// indices [0..24] in this array:
/// ```text
///   [ 0] ldr  r0, [pc, #28]     ; r0 = TIMER_INTR_ADDR        (lit hw[16])
///   [ 1] movs r1, #1             ; W1C bit value for ALARM0
///   [ 2] str  r1, [r0]           ; W1C TIMER.INTR bit 0
///   [ 3] ldr  r0, [pc, #28]      ; r0 = NVIC_ICPR0_ADDR        (lit hw[18])
///   [ 4] str  r1, [r0]           ; W1C NVIC bit 0  (r1 still 1)
///   [ 5] ldr  r2, [pc, #28]      ; r2 = ORDER_FIRST_IRQ_ADDR   (lit hw[20])
///   [ 6] ldr  r3, [r2]           ; r3 = *order_first_irq
///   [ 7] cmp  r3, #0
///   [ 8] bne  hw[11]             ; skip movs+str if non-zero
///   [ 9] movs r3, #0xA0          ; sentinel — IRQ_0 ran first
///   [10] str  r3, [r2]
///   [11] ldr  r0, [pc, #20]      ; r0 = CTR_IRQ_0_ADDR         (lit hw[22])
///   [12] ldr  r1, [r0]
///   [13] adds r1, #1
///   [14] str  r1, [r0]           ; ctr_irq_0 += 1
///   [15] bx   lr                 ; return via EXC_RETURN
///   [16..17] lit: TIMER_INTR_ADDR        (0x4005_4034)
///   [18..19] lit: NVIC_ICPR0_ADDR        (0xE000_E280)
///   [20..21] lit: ORDER_FIRST_IRQ_ADDR   (0x2000_3FC0)
///   [22..23] lit: CTR_IRQ_0_ADDR         (0x2000_3FB8)
/// ```
///
/// Literal math — HANDLER_IRQ0 starts at byte 0x04C, so absolute hw[i]
/// byte = 0x04C + 2*i. `ldr [pc, #imm8*4]` rounds PC down to a 4-byte
/// boundary first.
///
///   hw[ 0] addr 0x04C, PC 0x050, Align 0x050, target hw[16]=0x06C → imm=0x1C → imm8=7 → 0x4807
///   hw[ 3] addr 0x052, PC 0x056, Align 0x054, target hw[18]=0x070 → imm=0x1C → imm8=7 → 0x4807
///   hw[ 5] addr 0x056, PC 0x05A, Align 0x058, target hw[20]=0x074 → imm=0x1C → imm8=7 → 0x4A07
///   hw[11] addr 0x062, PC 0x066, Align 0x064, target hw[22]=0x078 → imm=0x14 → imm8=5 → 0x4805
///
/// Branch math — `bne` at hw[8], byte 0x05C, branch_PC = 0x060.
/// Target hw[11] at 0x062. offset = 0x062 - 0x060 = 2 → imm8 = 1.
/// Encoding: 0xD101.
///
/// **HANDLER_IRQ1 layout** — bytes 0x07C..0x0AC, hw indices [24..48]
/// in this array. Identical structure to HANDLER_IRQ0; literal-pool
/// offsets are the same (each handler self-contained). Differences:
///   - movs r1 = #2  (W1C bit 1 for ALARM1)
///   - sentinel = #0xA1
///   - lits point at CTR_IRQ_1_ADDR / TIMER_INTR / NVIC_ICPR0 / ORDER
///
/// Literal math — HANDLER_IRQ1 starts at byte 0x07C (= 0x04C + 24*2).
/// hw[24+k] is at byte 0x07C + 2*k. The relative-from-PC math therefore
/// yields the same encodings as HANDLER_IRQ0.
///
///   hw[24+ 0] addr 0x07C, PC 0x080, Align 0x080, target hw[24+16]=0x09C → imm8=7 → 0x4807
///   hw[24+ 3] addr 0x082, PC 0x086, Align 0x084, target hw[24+18]=0x0A0 → imm8=7 → 0x4807
///   hw[24+ 5] addr 0x086, PC 0x08A, Align 0x088, target hw[24+20]=0x0A4 → imm8=7 → 0x4A07
///   hw[24+11] addr 0x092, PC 0x096, Align 0x094, target hw[24+22]=0x0A8 → imm8=5 → 0x4805
///
/// `bne` at hw[24+8], byte 0x08C, branch_PC = 0x090, target byte 0x092
/// → imm8 = 1 → 0xD101.
const HANDLER_PRIORITY_PREEMPT: [u16; 48] = [
    // ---- HANDLER_IRQ0 (entry at HANDLER_OFFSET = 0x04C) -----------------
    // Phase 1: W1C TIMER.INTR bit 0 (drops level for ALARM0).
    0x4807, // [ 0] ldr  r0, [pc, #28]   — TIMER_INTR_ADDR
    0x2101, // [ 1] movs r1, #1
    0x6001, // [ 2] str  r1, [r0]        — TIMER.INTR = 1 (W1C bit 0)
    // Phase 2: W1C NVIC_ICPR0 bit 0.
    0x4807, // [ 3] ldr  r0, [pc, #28]   — NVIC_ICPR0_ADDR
    0x6001, // [ 4] str  r1, [r0]        — NVIC_ICPR0 = 1 (r1 still 1)
    // Phase 3: order_first_irq race-free first-write.
    0x4A07, // [ 5] ldr  r2, [pc, #28]   — ORDER_FIRST_IRQ_ADDR
    0x6813, // [ 6] ldr  r3, [r2]        — r3 = *order
    0x2B00, // [ 7] cmp  r3, #0
    0xD101, // [ 8] bne  hw[11]          — skip if non-zero
    0x23A0, // [ 9] movs r3, #0xA0       — sentinel for IRQ_0
    0x6013, // [10] str  r3, [r2]        — *order = 0xA0
    // Phase 4: ctr_irq_0 += 1.
    0x4805, // [11] ldr  r0, [pc, #20]   — CTR_IRQ_0_ADDR
    0x6801, // [12] ldr  r1, [r0]
    0x3101, // [13] adds r1, #1
    0x6001, // [14] str  r1, [r0]
    // Phase 5: return via EXC_RETURN.
    0x4770, // [15] bx   lr
    // ---- HANDLER_IRQ0 literal pool (hw[16..24]) ------------------------
    0x4034, // [16] lit: TIMER_INTR_ADDR        low  (0x4005_4034)
    0x4005, // [17] lit: TIMER_INTR_ADDR        high
    0xE280, // [18] lit: NVIC_ICPR0_ADDR        low  (0xE000_E280)
    0xE000, // [19] lit: NVIC_ICPR0_ADDR        high
    0x3FC0, // [20] lit: ORDER_FIRST_IRQ_ADDR   low  (0x2000_3FC0)
    0x2000, // [21] lit: ORDER_FIRST_IRQ_ADDR   high
    0x3FB8, // [22] lit: CTR_IRQ_0_ADDR         low  (0x2000_3FB8)
    0x2000, // [23] lit: CTR_IRQ_0_ADDR         high
    // ---- HANDLER_IRQ1 (entry at HANDLER_OFFSET + 48 = 0x07C) -----------
    // Phase 1: W1C TIMER.INTR bit 1 (drops level for ALARM1).
    0x4807, // [24] ldr  r0, [pc, #28]   — TIMER_INTR_ADDR
    0x2102, // [25] movs r1, #2
    0x6001, // [26] str  r1, [r0]        — TIMER.INTR = 2 (W1C bit 1)
    // Phase 2: W1C NVIC_ICPR0 bit 1.
    0x4807, // [27] ldr  r0, [pc, #28]   — NVIC_ICPR0_ADDR
    0x6001, // [28] str  r1, [r0]        — NVIC_ICPR0 = 2 (r1 still 2)
    // Phase 3: order_first_irq race-free first-write.
    0x4A07, // [29] ldr  r2, [pc, #28]   — ORDER_FIRST_IRQ_ADDR
    0x6813, // [30] ldr  r3, [r2]        — r3 = *order
    0x2B00, // [31] cmp  r3, #0
    0xD101, // [32] bne  hw[24+11]       — skip if non-zero
    0x23A1, // [33] movs r3, #0xA1       — sentinel for IRQ_1
    0x6013, // [34] str  r3, [r2]        — *order = 0xA1
    // Phase 4: ctr_irq_1 += 1.
    0x4805, // [35] ldr  r0, [pc, #20]   — CTR_IRQ_1_ADDR
    0x6801, // [36] ldr  r1, [r0]
    0x3101, // [37] adds r1, #1
    0x6001, // [38] str  r1, [r0]
    // Phase 5: return via EXC_RETURN.
    0x4770, // [39] bx   lr
    // ---- HANDLER_IRQ1 literal pool (hw[40..48]) ------------------------
    0x4034, // [40] lit: TIMER_INTR_ADDR        low  (0x4005_4034)
    0x4005, // [41] lit: TIMER_INTR_ADDR        high
    0xE280, // [42] lit: NVIC_ICPR0_ADDR        low  (0xE000_E280)
    0xE000, // [43] lit: NVIC_ICPR0_ADDR        high
    0x3FC0, // [44] lit: ORDER_FIRST_IRQ_ADDR   low  (0x2000_3FC0)
    0x2000, // [45] lit: ORDER_FIRST_IRQ_ADDR   high
    0x3FBC, // [46] lit: CTR_IRQ_1_ADDR         low  (0x2000_3FBC)
    0x2000, // [47] lit: CTR_IRQ_1_ADDR         high
];

// HANDLER_IRQ1 entry offset (relative to HANDLER_OFFSET) — used by
// `IMAGE_PRIORITY_PREEMPT` to patch vector entry [17].
const HANDLER_IRQ0_LEN_HW: usize = 24;
const HANDLER_IRQ1_OFFSET_BYTES: u32 = (HANDLER_IRQ0_LEN_HW as u32) * 2;

// Pin every literal-pool slot — every `ldr [pc, #imm]` above depends on
// these offsets staying put.
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[16] == 0x4034 && HANDLER_PRIORITY_PREEMPT[17] == 0x4005,
    "TIMER_INTR_ADDR literal must remain at HANDLER_IRQ0 hw[16..=17]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[18] == 0xE280 && HANDLER_PRIORITY_PREEMPT[19] == 0xE000,
    "NVIC_ICPR0_ADDR literal must remain at HANDLER_IRQ0 hw[18..=19]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[20] == 0x3FC0 && HANDLER_PRIORITY_PREEMPT[21] == 0x2000,
    "ORDER_FIRST_IRQ_ADDR literal must remain at HANDLER_IRQ0 hw[20..=21]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[22] == 0x3FB8 && HANDLER_PRIORITY_PREEMPT[23] == 0x2000,
    "CTR_IRQ_0_ADDR literal must remain at HANDLER_IRQ0 hw[22..=23]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[40] == 0x4034 && HANDLER_PRIORITY_PREEMPT[41] == 0x4005,
    "TIMER_INTR_ADDR literal must remain at HANDLER_IRQ1 hw[40..=41]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[42] == 0xE280 && HANDLER_PRIORITY_PREEMPT[43] == 0xE000,
    "NVIC_ICPR0_ADDR literal must remain at HANDLER_IRQ1 hw[42..=43]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[44] == 0x3FC0 && HANDLER_PRIORITY_PREEMPT[45] == 0x2000,
    "ORDER_FIRST_IRQ_ADDR literal must remain at HANDLER_IRQ1 hw[44..=45]",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[46] == 0x3FBC && HANDLER_PRIORITY_PREEMPT[47] == 0x2000,
    "CTR_IRQ_1_ADDR literal must remain at HANDLER_IRQ1 hw[46..=47]",
);
// Pin the HANDLER_IRQ0 / HANDLER_IRQ1 boundary with a bracketed
// pair of anchors. The head anchor (hw[24] == 0x4807) catches a
// boundary that drifted *forward*; the tail anchor (hw[15] == 0x4770,
// IRQ0's `bx lr`) catches a boundary that drifted *backward*. Without
// the tail anchor, IRQ0 growing by one halfword and IRQ1 contracting
// by one would leave hw[24] still 0x4807 (IRQ1's *second* `ldr` looks
// identical to its first) while vector entry [17] silently lands one
// instruction past IRQ1's true entry point. The length-to-offset pin
// makes the same invariant explicit in symbolic form.
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[15] == 0x4770,
    "HANDLER_IRQ0 must end at hw[15] with bx lr (0x4770)",
);
const _: () = assert!(
    HANDLER_PRIORITY_PREEMPT[24] == 0x4807,
    "HANDLER_IRQ1 entry must remain at hw[24] (ldr r0, [pc, #28])",
);
const _: () = assert!(
    HANDLER_IRQ0_LEN_HW * 2 == HANDLER_IRQ1_OFFSET_BYTES as usize,
    "HANDLER_IRQ1_OFFSET_BYTES must equal HANDLER_IRQ0_LEN_HW * 2",
);
const _: () = assert!(
    (HANDLER_OFFSET as usize) + HANDLER_PRIORITY_PREEMPT.len() * 2 <= MAIN_OFFSET as usize,
    "HANDLER_PRIORITY_PREEMPT must fit between HANDLER_OFFSET and MAIN_OFFSET",
);

// ---------------------------------------------------------------------------
// Hand-assembled main routines
// ---------------------------------------------------------------------------
//
// Main routines live at MAIN_OFFSET = 0x100. Literal-pool math uses
// the same instruction-PC alignment as the M33 module's MAIN_BASELINE:
// `ldr rD, [pc, #imm8*4]` rounds PC down to a 4-byte boundary before
// adding the offset.
//
// Two main routines:
//
// * `MAIN_TIMER`    — arm TIMER ALARM0 to fire quickly, enable
//   TIMER.INTE, enable TIMER_IRQ_0 in NVIC_ISER, busy-wait.
// * `MAIN_TAIL` — pre-arm SysTick via MMIO (handled in the
//   runner's preamble), then pend PendSV via ICSR and busy-wait.

/// TIMER_IRQ_0 main. Writes `0x0000_0010` to `ALARM0` (a near-future
/// deadline relative to `TIMERAWL` = 0 at boot), `1` to `INTE`, `1 <<
/// IRQ_TIMER_IRQ_0` to `NVIC_ISER`, then branch-to-self.
///
/// Register map inside main:
///   r0 — scratch (ALARM0 deadline, NVIC bit value)
///   r1 — scratch (peripheral address, mask)
///   r4 — ALARM0 address
///   r5 — INTE address
///   r6 — NVIC_ISER address
///
/// ```text
///   [ 0] movs r0, #0x10            ; near-future alarm deadline
///   [ 1] ldr  r4, [pc, #28]        ; r4 = TIMER_ALARM0_ADDR   (lit [16])
///   [ 2] str  r0, [r4]             ; arm alarm
///   [ 3] movs r0, #1
///   [ 4] ldr  r5, [pc, #24]        ; r5 = TIMER_INTE_ADDR      (lit [18])
///   [ 5] str  r0, [r5]             ; enable ALARM0 IRQ
///   [ 6] ldr  r6, [pc, #24]        ; r6 = NVIC_ISER0_ADDR      (lit [20])
///   [ 7] str  r0, [r6]             ; NVIC: enable IRQ #0
///   [ 8] b    .                    ; busy-wait until TIMER fires
///   [ 9] bkpt #0                   ; safety net — handler returns past this
///   [10..15] padding
///   [16..17] lit: TIMER_ALARM0_ADDR
///   [18..19] lit: TIMER_INTE_ADDR
///   [20..21] lit: NVIC_ISER0_ADDR
/// ```
///
/// Literal math:
///   hw[1] `ldr r4, [pc, #imm]`: addr 0x102, PC 0x106, Align 0x104.
///     target hw[16] = 0x120 → imm8*4 = 0x1C → imm8 = 7. Encoding: 0x4C07.
///   hw[4] `ldr r5, [pc, #imm]`: addr 0x108, PC 0x10C, Align 0x10C.
///     target hw[18] = 0x124 → imm8*4 = 0x18 → imm8 = 6. Encoding: 0x4D06.
///   hw[6] `ldr r6, [pc, #imm]`: addr 0x10C, PC 0x110, Align 0x110.
///     target hw[20] = 0x128 → imm8*4 = 0x18 → imm8 = 6. Encoding: 0x4E06.
///
/// After the handler returns (BX LR → EXC_RETURN), the CPU resumes at
/// the `b .` busy-wait. A host-side SWD halt then stops the core for
/// observable readback; the runner issues the halt once the handler
/// has had time to run by polling the counter every few ms and
/// halting once it increments (or a wall-clock watchdog fires).
const MAIN_TIMER: [u16; 22] = [
    0x2010, // [ 0] movs r0, #0x10         — ALARM0 deadline
    0x4C07, // [ 1] ldr  r4, [pc, #28]     — TIMER_ALARM0_ADDR
    0x6020, // [ 2] str  r0, [r4]          — arm alarm
    0x2001, // [ 3] movs r0, #1
    0x4D06, // [ 4] ldr  r5, [pc, #24]     — TIMER_INTE_ADDR
    0x6028, // [ 5] str  r0, [r5]          — enable ALARM0 IRQ
    0x4E06, // [ 6] ldr  r6, [pc, #24]     — NVIC_ISER0_ADDR
    0x6030, // [ 7] str  r0, [r6]          — enable IRQ #0 in NVIC
    0xE7FE, // [ 8] b    .                 — busy-wait
    0xBE00, // [ 9] bkpt #0                — safety net
    0xBF00, // [10] nop                    — padding
    0xBF00, // [11] nop
    0xBF00, // [12] nop
    0xBF00, // [13] nop
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0x4010, // [16] lit: TIMER_ALARM0_ADDR low   = 0x4005_4010 & 0xFFFF
    0x4005, // [17] lit: TIMER_ALARM0_ADDR high
    0x4038, // [18] lit: TIMER_INTE_ADDR  low    = 0x4005_4038 & 0xFFFF
    0x4005, // [19] lit: TIMER_INTE_ADDR  high
    0xE100, // [20] lit: NVIC_ISER0_ADDR low     = 0xE000_E100 & 0xFFFF
    0xE000, // [21] lit: NVIC_ISER0_ADDR high
];

/// Tail-chain main. SysTick is pre-armed via MMIO in the runner's
/// preamble; main's job is just to pend PendSV and busy-wait.
///
/// Literal-pool layout matches the M33 MAIN_BASELINE at
/// `isr_scenarios.rs:533` — hw[16..17] ICSR address, hw[18..19]
/// PENDSVSET mask. DWT_CYCCNT is unused on M0+ (the RP2040 PPB has no
/// DWT block modelled, and silicon M0+ typically doesn't implement
/// DWT either).
///
/// ```text
///   [ 0] ldr  r6, [pc, #56]        ; r6 = SCB_ICSR_ADDR        (lit [16])
///   [ 1] ldr  r7, [pc, #56]        ; r7 = ICSR_PENDSVSET       (lit [18])
///   [ 2] str  r7, [r6]             ; *ICSR = PENDSVSET (TRIGGER)
///   [ 3] b    .                    ; busy-wait
///   [ 4] bkpt #0                   ; safety net
///   [ 5..15] NOP padding
///   [16..17] lit: SCB_ICSR_ADDR
///   [18..19] lit: ICSR_PENDSVSET
///   [20..21] reserved (kept 0)
/// ```
///
/// Literal math:
///   hw[0] `ldr r6, [pc, #imm]`: addr 0x100, PC 0x104, Align 0x104.
///     target hw[16] = 0x120 → imm8*4 = 0x1C → imm8 = 7. Encoding: 0x4E07.
///   hw[1] `ldr r7, [pc, #imm]`: addr 0x102, PC 0x106, Align 0x104.
///     target hw[18] = 0x124 → imm8*4 = 0x20 → imm8 = 8. Encoding: 0x4F08.
const MAIN_TAIL: [u16; 22] = [
    0x4E07, // [ 0] ldr  r6, [pc, #28]     — SCB_ICSR_ADDR
    0x4F08, // [ 1] ldr  r7, [pc, #32]     — ICSR_PENDSVSET
    0x6037, // [ 2] str  r7, [r6]          — *ICSR = PENDSVSET
    0xE7FE, // [ 3] b    .                 — busy-wait
    0xBE00, // [ 4] bkpt #0                — safety net
    0xBF00, // [ 5] nop                    — padding
    0xBF00, // [ 6] nop
    0xBF00, // [ 7] nop
    0xBF00, // [ 8] nop
    0xBF00, // [ 9] nop
    0xBF00, // [10] nop
    0xBF00, // [11] nop
    0xBF00, // [12] nop
    0xBF00, // [13] nop
    0xBF00, // [14] nop
    0xBF00, // [15] nop
    0xED04, // [16] lit: SCB_ICSR_ADDR low  = 0xE000_ED04 & 0xFFFF
    0xE000, // [17] lit: SCB_ICSR_ADDR high
    0x0000, // [18] lit: ICSR_PENDSVSET low = 0x1000_0000 & 0xFFFF
    0x1000, // [19] lit: ICSR_PENDSVSET high
    0x0000, // [20] reserved
    0x0000, // [21] reserved
];

/// V2 §3.4 main: write 0xFFFFFFFF to each of NVIC ISER0/ISPR0/ICER0/ICPR0,
/// read back, and store readback into the four `*_READBACK` SRAM cells.
/// Pre-seeds before ICER0 / ICPR0 use 0x0000_FFFF (within the mask
/// range) so the masked-clear can demonstrate it covers the full
/// pre-seed and produces a final readback of 0. No handler dispatch.
///
/// Register convention:
///   r0 — scratch (MMIO/SRAM target address per step)
///   r1 — scratch (readback value)
///   r2 — held: 0xFFFF_FFFF (the all-ones write value)
///   r3 — held: 0x0000_FFFF (within-mask pre-seed value)
///   r4 — held: NVIC_ISER0_ADDR
///   r5 — held: NVIC_ISPR0_ADDR
///
/// Literal-pool layout matches the M0+ `ldr [pc, #imm8*4]` math: PC
/// rounded down to a 4-byte boundary plus imm8*4. hw[i] byte address =
/// 0x100 + 2*i. Pool packed at hw[44..64].
///
/// ```text
///   [ 0] ldr  r2, [pc, #84]   ; r2 = 0xFFFF_FFFF              (lit hw[44])
///   [ 1] ldr  r3, [pc, #88]   ; r3 = 0x0000_FFFF              (lit hw[46])
///   [ 2] ldr  r4, [pc, #88]   ; r4 = NVIC_ISER0_ADDR          (lit hw[48])
///   [ 3] ldr  r5, [pc, #92]   ; r5 = NVIC_ISPR0_ADDR          (lit hw[50])
///   ; -- Step 1: ISER0 RAZ/WI
///   [ 4] str  r2, [r4]        ; *NVIC_ISER0 = 0xFFFFFFFF
///   [ 5] ldr  r1, [r4]        ; r1 = readback (expect 0x03FF_FFFF)
///   [ 6] ldr  r0, [pc, #88]   ; r0 = ISER_READBACK_ADDR       (lit hw[52])
///   [ 7] str  r1, [r0]
///   [ 8] str  r3, [r4]        ; pre-seed ISER0 = 0x0000_FFFF (within mask)
///   ; -- Step 2: ISPR0 RAZ/WI
///   [ 9] str  r2, [r5]        ; *NVIC_ISPR0 = 0xFFFFFFFF
///   [10] ldr  r1, [r5]        ; r1 = readback (expect 0x03FF_FFFF)
///   [11] ldr  r0, [pc, #84]   ; r0 = ISPR_READBACK_ADDR       (lit hw[54])
///   [12] str  r1, [r0]
///   [13] str  r3, [r5]        ; pre-seed ISPR0 = 0x0000_FFFF
///   ; -- Step 3: ICER0 — masked-clear covers full pre-seed
///   [14] ldr  r0, [pc, #80]   ; r0 = NVIC_ICER0_ADDR          (lit hw[56])
///   [15] str  r2, [r0]        ; *NVIC_ICER0 = 0xFFFFFFFF
///   [16] ldr  r1, [r4]        ; r1 = ISER0 readback (expect 0)
///   [17] ldr  r0, [pc, #80]   ; r0 = ICER_READBACK_ADDR       (lit hw[58])
///   [18] str  r1, [r0]
///   ; -- Step 4: ICPR0 — masked-clear covers full pre-seed
///   [19] ldr  r0, [pc, #80]   ; r0 = NVIC_ICPR0_ADDR          (lit hw[60])
///   [20] str  r2, [r0]        ; *NVIC_ICPR0 = 0xFFFFFFFF
///   [21] ldr  r1, [r5]        ; r1 = ISPR0 readback (expect 0)
///   [22] ldr  r0, [pc, #76]   ; r0 = ICPR_READBACK_ADDR       (lit hw[62])
///   [23] str  r1, [r0]
///   [24] b    .               ; busy-wait
///   [25..43] nop padding
///   [44..45] lit: 0xFFFF_FFFF
///   [46..47] lit: 0x0000_FFFF
///   [48..49] lit: NVIC_ISER0_ADDR (0xE000_E100)
///   [50..51] lit: NVIC_ISPR0_ADDR (0xE000_E200)
///   [52..53] lit: ISER_READBACK_ADDR (0x2000_3FD0)
///   [54..55] lit: ISPR_READBACK_ADDR (0x2000_3FD4)
///   [56..57] lit: NVIC_ICER0_ADDR (0xE000_E180)
///   [58..59] lit: ICER_READBACK_ADDR (0x2000_3FD8)
///   [60..61] lit: NVIC_ICPR0_ADDR (0xE000_E280)
///   [62..63] lit: ICPR_READBACK_ADDR (0x2000_3FDC)
/// ```
///
/// Literal math (per `ldr Rt, [pc, #imm8*4]`, PC = instr_addr+4 rounded
/// down to 4-byte boundary):
///   hw[ 0] addr 0x100, PC 0x104, Align 0x104, target hw[44]=0x158 → imm=0x54  → imm8=21 → 0x4A15
///   hw[ 1] addr 0x102, PC 0x106, Align 0x104, target hw[46]=0x15C → imm=0x58  → imm8=22 → 0x4B16
///   hw[ 2] addr 0x104, PC 0x108, Align 0x108, target hw[48]=0x160 → imm=0x58  → imm8=22 → 0x4C16
///   hw[ 3] addr 0x106, PC 0x10A, Align 0x108, target hw[50]=0x164 → imm=0x5C  → imm8=23 → 0x4D17
///   hw[ 6] addr 0x10C, PC 0x110, Align 0x110, target hw[52]=0x168 → imm=0x58  → imm8=22 → 0x4816
///   hw[11] addr 0x116, PC 0x11A, Align 0x118, target hw[54]=0x16C → imm=0x54  → imm8=21 → 0x4815
///   hw[14] addr 0x11C, PC 0x120, Align 0x120, target hw[56]=0x170 → imm=0x50  → imm8=20 → 0x4814
///   hw[17] addr 0x122, PC 0x126, Align 0x124, target hw[58]=0x174 → imm=0x50  → imm8=20 → 0x4814
///   hw[19] addr 0x126, PC 0x12A, Align 0x128, target hw[60]=0x178 → imm=0x50  → imm8=20 → 0x4814
///   hw[22] addr 0x12C, PC 0x130, Align 0x130, target hw[62]=0x17C → imm=0x4C  → imm8=19 → 0x4813
const MAIN_NVIC_RAZWI: [u16; 64] = [
    0x4A15, // [ 0] ldr  r2, [pc, #84]   — 0xFFFF_FFFF
    0x4B16, // [ 1] ldr  r3, [pc, #88]   — 0x0000_FFFF
    0x4C16, // [ 2] ldr  r4, [pc, #88]   — NVIC_ISER0_ADDR
    0x4D17, // [ 3] ldr  r5, [pc, #92]   — NVIC_ISPR0_ADDR
    0x6022, // [ 4] str  r2, [r4]        — *NVIC_ISER0 = 0xFFFFFFFF
    0x6821, // [ 5] ldr  r1, [r4]        — readback
    0x4816, // [ 6] ldr  r0, [pc, #88]   — ISER_READBACK_ADDR
    0x6001, // [ 7] str  r1, [r0]
    0x6023, // [ 8] str  r3, [r4]        — pre-seed *NVIC_ISER0 = 0x0000FFFF
    0x602A, // [ 9] str  r2, [r5]        — *NVIC_ISPR0 = 0xFFFFFFFF
    0x6829, // [10] ldr  r1, [r5]        — readback
    0x4815, // [11] ldr  r0, [pc, #84]   — ISPR_READBACK_ADDR
    0x6001, // [12] str  r1, [r0]
    0x602B, // [13] str  r3, [r5]        — pre-seed *NVIC_ISPR0 = 0x0000FFFF
    0x4814, // [14] ldr  r0, [pc, #80]   — NVIC_ICER0_ADDR
    0x6002, // [15] str  r2, [r0]        — *NVIC_ICER0 = 0xFFFFFFFF
    0x6821, // [16] ldr  r1, [r4]        — readback ISER0 (expect 0)
    0x4814, // [17] ldr  r0, [pc, #80]   — ICER_READBACK_ADDR
    0x6001, // [18] str  r1, [r0]
    0x4814, // [19] ldr  r0, [pc, #80]   — NVIC_ICPR0_ADDR
    0x6002, // [20] str  r2, [r0]        — *NVIC_ICPR0 = 0xFFFFFFFF
    0x6829, // [21] ldr  r1, [r5]        — readback ISPR0 (expect 0)
    0x4813, // [22] ldr  r0, [pc, #76]   — ICPR_READBACK_ADDR
    0x6001, // [23] str  r1, [r0]
    0xE7FE, // [24] b    .               — busy-wait
    0xBF00, // [25] nop                  — padding
    0xBF00, // [26] nop
    0xBF00, // [27] nop
    0xBF00, // [28] nop
    0xBF00, // [29] nop
    0xBF00, // [30] nop
    0xBF00, // [31] nop
    0xBF00, // [32] nop
    0xBF00, // [33] nop
    0xBF00, // [34] nop
    0xBF00, // [35] nop
    0xBF00, // [36] nop
    0xBF00, // [37] nop
    0xBF00, // [38] nop
    0xBF00, // [39] nop
    0xBF00, // [40] nop
    0xBF00, // [41] nop
    0xBF00, // [42] nop
    0xBF00, // [43] nop
    0xFFFF, // [44] lit: 0xFFFF_FFFF low
    0xFFFF, // [45] lit: 0xFFFF_FFFF high
    0xFFFF, // [46] lit: 0x0000_FFFF low
    0x0000, // [47] lit: 0x0000_FFFF high
    0xE100, // [48] lit: NVIC_ISER0_ADDR low
    0xE000, // [49] lit: NVIC_ISER0_ADDR high
    0xE200, // [50] lit: NVIC_ISPR0_ADDR low
    0xE000, // [51] lit: NVIC_ISPR0_ADDR high
    0x3FD0, // [52] lit: ISER_READBACK_ADDR low
    0x2000, // [53] lit: ISER_READBACK_ADDR high
    0x3FD4, // [54] lit: ISPR_READBACK_ADDR low
    0x2000, // [55] lit: ISPR_READBACK_ADDR high
    0xE180, // [56] lit: NVIC_ICER0_ADDR low
    0xE000, // [57] lit: NVIC_ICER0_ADDR high
    0x3FD8, // [58] lit: ICER_READBACK_ADDR low
    0x2000, // [59] lit: ICER_READBACK_ADDR high
    0xE280, // [60] lit: NVIC_ICPR0_ADDR low
    0xE000, // [61] lit: NVIC_ICPR0_ADDR high
    0x3FDC, // [62] lit: ICPR_READBACK_ADDR low
    0x2000, // [63] lit: ICPR_READBACK_ADDR high
];

// Pin literal-pool byte offsets — every `ldr [pc, #imm]` above depends
// on these slots staying put.
const _: () = assert!(
    MAIN_NVIC_RAZWI[44] == 0xFFFF && MAIN_NVIC_RAZWI[45] == 0xFFFF,
    "0xFFFF_FFFF literal must remain at hw[44..=45]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[46] == 0xFFFF && MAIN_NVIC_RAZWI[47] == 0x0000,
    "0x0000_FFFF literal must remain at hw[46..=47]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[48] == 0xE100 && MAIN_NVIC_RAZWI[49] == 0xE000,
    "NVIC_ISER0_ADDR literal must remain at hw[48..=49]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[50] == 0xE200 && MAIN_NVIC_RAZWI[51] == 0xE000,
    "NVIC_ISPR0_ADDR literal must remain at hw[50..=51]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[52] == 0x3FD0 && MAIN_NVIC_RAZWI[53] == 0x2000,
    "ISER_READBACK_ADDR literal must remain at hw[52..=53]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[54] == 0x3FD4 && MAIN_NVIC_RAZWI[55] == 0x2000,
    "ISPR_READBACK_ADDR literal must remain at hw[54..=55]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[56] == 0xE180 && MAIN_NVIC_RAZWI[57] == 0xE000,
    "NVIC_ICER0_ADDR literal must remain at hw[56..=57]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[58] == 0x3FD8 && MAIN_NVIC_RAZWI[59] == 0x2000,
    "ICER_READBACK_ADDR literal must remain at hw[58..=59]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[60] == 0xE280 && MAIN_NVIC_RAZWI[61] == 0xE000,
    "NVIC_ICPR0_ADDR literal must remain at hw[60..=61]",
);
const _: () = assert!(
    MAIN_NVIC_RAZWI[62] == 0x3FDC && MAIN_NVIC_RAZWI[63] == 0x2000,
    "ICPR_READBACK_ADDR literal must remain at hw[62..=63]",
);
const _: () = assert!(
    (MAIN_OFFSET as usize) + MAIN_NVIC_RAZWI.len() * 2 <= ISR_IMAGE_SIZE,
    "MAIN_NVIC_RAZWI must fit inside the image's main region",
);

/// V2 §3.2 main: PRIMASK-gated pend then `cpsie i` unmask.
///
/// Sequence:
///   1. `cpsid i` — set PRIMASK=1 (block exception dispatch).
///   2. `*NVIC_ISPR0 = 1` — set TIMER_IRQ_0 pending in NVIC.
///   3. `*NVIC_ISER0 = 1` — enable TIMER_IRQ_0 in NVIC.
///   4. `*gate = 1`        — pre-unmask gate value.
///   5. `cpsie i`          — clear PRIMASK; on architecturally-correct
///      M0+ the dispatch happens on this boundary.
///   6. `*gate = 2`        — main resumes after handler returns.
///   7. `b .`              — busy-wait.
///
/// **Caveat re: TIMER level re-pend.** TIMER_IRQ_0 has level-driven
/// re-pend behaviour in `peripherals/timer.rs` — `tick_peripherals`
/// re-asserts NVIC bit 0 every cycle while `(timer.intr & timer.inte) != 0`.
/// Main therefore does NOT enable `TIMER.INTE`: the IRQ is set pending
/// purely via `NVIC_ISPR0`, so once the handler clears `NVIC_ICPR0` bit 0
/// there's no level signal to re-assert it. Single-shot dispatch.
///
/// Register convention (matching MAIN_TIMER's shape):
///   r0 — scratch (1, then 2 — written into the gate / NVIC bits)
///   r4 — held: NVIC_ISPR0_ADDR
///   r5 — held: NVIC_ISER0_ADDR
///   r6 — held: GATE_ADDR
///
/// Layout — image bytes 0x100..0x130:
/// ```text
///   [ 0] cpsid i                ; 0xB672 — PRIMASK=1
///   [ 1] ldr  r4, [pc, #28]     ; r4 = NVIC_ISPR0_ADDR    (lit hw[16])
///   [ 2] movs r0, #1
///   [ 3] str  r0, [r4]          ; pend TIMER_IRQ_0
///   [ 4] ldr  r5, [pc, #24]     ; r5 = NVIC_ISER0_ADDR    (lit hw[18])
///   [ 5] str  r0, [r5]          ; enable TIMER_IRQ_0 in NVIC
///   [ 6] ldr  r6, [pc, #24]     ; r6 = GATE_ADDR          (lit hw[20])
///   [ 7] str  r0, [r6]          ; *gate = 1   (PRE-UNMASK probe)
///   [ 8] cpsie i                ; 0xB662 — PRIMASK=0  (DISPATCH BOUNDARY)
///   [ 9] movs r0, #2
///   [10] str  r0, [r6]          ; *gate = 2   (post-handler resume)
///   [11] b    .                 ; 0xE7FE — busy-wait
///   [12] bkpt #0                ; safety net
///   [13..15] nop padding
///   [16..17] lit: NVIC_ISPR0_ADDR (0xE000_E200)
///   [18..19] lit: NVIC_ISER0_ADDR (0xE000_E100)
///   [20..21] lit: GATE_ADDR       (0x2000_3FEC)
/// ```
///
/// Literal math — main lives at MAIN_OFFSET = 0x100, so hw[i] byte =
/// 0x100 + 2*i. `ldr [pc, #imm8*4]` rounds PC down to a 4-byte boundary
/// before adding imm8*4.
///
///   hw[ 1] addr 0x102, PC 0x106, Align 0x104, target hw[16]=0x120 → imm=0x1C → imm8=7 → 0x4C07
///   hw[ 4] addr 0x108, PC 0x10C, Align 0x10C, target hw[18]=0x124 → imm=0x18 → imm8=6 → 0x4D06
///   hw[ 6] addr 0x10C, PC 0x110, Align 0x110, target hw[20]=0x128 → imm=0x18 → imm8=6 → 0x4E06
const MAIN_MASKED: [u16; 22] = [
    0xB672, // [ 0] cpsid i               — PRIMASK=1
    0x4C07, // [ 1] ldr  r4, [pc, #28]    — NVIC_ISPR0_ADDR
    0x2001, // [ 2] movs r0, #1
    0x6020, // [ 3] str  r0, [r4]         — pend TIMER_IRQ_0
    0x4D06, // [ 4] ldr  r5, [pc, #24]    — NVIC_ISER0_ADDR
    0x6028, // [ 5] str  r0, [r5]         — enable TIMER_IRQ_0
    0x4E06, // [ 6] ldr  r6, [pc, #24]    — GATE_ADDR
    0x6030, // [ 7] str  r0, [r6]         — *gate = 1
    0xB662, // [ 8] cpsie i               — PRIMASK=0  (dispatch boundary)
    0x2002, // [ 9] movs r0, #2
    0x6030, // [10] str  r0, [r6]         — *gate = 2  (post-handler)
    0xE7FE, // [11] b    .                — busy-wait
    0xBE00, // [12] bkpt #0               — safety net
    0xBF00, // [13] nop padding
    0xBF00, // [14] nop padding
    0xBF00, // [15] nop padding
    0xE200, // [16] lit: NVIC_ISPR0_ADDR low  (0xE000_E200)
    0xE000, // [17] lit: NVIC_ISPR0_ADDR high
    0xE100, // [18] lit: NVIC_ISER0_ADDR low  (0xE000_E100)
    0xE000, // [19] lit: NVIC_ISER0_ADDR high
    0x3FEC, // [20] lit: GATE_ADDR        low  (0x2000_3FEC)
    0x2000, // [21] lit: GATE_ADDR        high
];

// Pin literal-pool byte offsets.
const _: () = assert!(
    MAIN_MASKED[16] == 0xE200 && MAIN_MASKED[17] == 0xE000,
    "NVIC_ISPR0_ADDR literal must remain at hw[16..=17]",
);
const _: () = assert!(
    MAIN_MASKED[18] == 0xE100 && MAIN_MASKED[19] == 0xE000,
    "NVIC_ISER0_ADDR literal must remain at hw[18..=19]",
);
const _: () = assert!(
    MAIN_MASKED[20] == 0x3FEC && MAIN_MASKED[21] == 0x2000,
    "GATE_ADDR literal must remain at hw[20..=21]",
);
const _: () = assert!(
    (MAIN_OFFSET as usize) + MAIN_MASKED.len() * 2 <= ISR_IMAGE_SIZE,
    "MAIN_MASKED must fit inside the image's main region",
);

/// V2 §3.3 main: enable TIMER_IRQ_0, arm ALARM0 at TIMERAWL+200, mark
/// `phase=1`, WFI, mark `phase=2`, busy-wait.
///
/// Sequence:
///   1. Enable TIMER_IRQ_0 in NVIC (`*NVIC_ISER0 = 1`).
///   2. Enable ALARM0 INTE (`*TIMER_INTE = 1`) — this is what makes the
///      alarm fire raise NVIC bit 0 (level-driven).
///   3. Read TIMERAWL, add a 200-tick (~200 µs at default 1µs cadence)
///      delta to compute the deadline.
///   4. Write deadline to `TIMER_ALARM0`.
///   5. `*phase = 1`            — pre-WFI gate.
///   6. `wfi`                   — park the core; alarm will wake.
///   7. `*phase = 2`            — main resumed after handler returned.
///   8. `b .`                   — busy-wait.
///
/// Register convention:
///   r0 — scratch (1, 2, mask)
///   r1 — scratch (timer addresses, deadline source/dest)
///   r2 — scratch (deadline value)
///   r4 — held: NVIC_ISER0_ADDR
///   r5 — held: TIMER_INTE_ADDR
///   r6 — held: PHASE_ADDR
///
/// Layout — image bytes 0x100..0x13C:
/// ```text
///   [ 0] ldr  r4, [pc, #36]    ; r4 = NVIC_ISER0_ADDR     (lit hw[20])
///   [ 1] movs r0, #1
///   [ 2] str  r0, [r4]         ; enable IRQ #0 in NVIC
///   [ 3] ldr  r5, [pc, #36]    ; r5 = TIMER_INTE_ADDR     (lit hw[22])
///   [ 4] str  r0, [r5]         ; enable ALARM0 INTE
///   [ 5] ldr  r1, [pc, #36]    ; r1 = TIMER_TIMERAWL_ADDR (lit hw[24])
///   [ 6] ldr  r2, [r1]         ; r2 = TIMERAWL (now)
///   [ 7] adds r2, #200         ; r2 = deadline (now + 200 ticks)
///   [ 8] ldr  r1, [pc, #32]    ; r1 = TIMER_ALARM0_ADDR   (lit hw[26])
///   [ 9] str  r2, [r1]         ; arm alarm
///   [10] ldr  r6, [pc, #32]    ; r6 = PHASE_ADDR          (lit hw[28])
///   [11] str  r0, [r6]         ; *phase = 1   (PRE-WFI probe)
///   [12] wfi                   ; 0xBF30 — park core; alarm wake
///   [13] movs r0, #2
///   [14] str  r0, [r6]         ; *phase = 2   (post-handler resume)
///   [15] b    .                ; 0xE7FE — busy-wait
///   [16] bkpt #0               ; safety net
///   [17..19] nop padding
///   [20..21] lit: NVIC_ISER0_ADDR     (0xE000_E100)
///   [22..23] lit: TIMER_INTE_ADDR     (0x4005_4038)
///   [24..25] lit: TIMER_TIMERAWL_ADDR (0x4005_4028)
///   [26..27] lit: TIMER_ALARM0_ADDR   (0x4005_4010)
///   [28..29] lit: PHASE_ADDR          (0x2000_3FC8)
/// ```
///
/// Literal math — main lives at MAIN_OFFSET = 0x100, hw[i] byte =
/// 0x100 + 2*i. `ldr [pc, #imm8*4]` rounds PC down to a 4-byte boundary
/// before adding imm8*4.
///
///   hw[ 0] addr 0x100, PC 0x104, Align 0x104, target hw[20]=0x128 → imm=0x24 → imm8=9 → 0x4C09
///   hw[ 3] addr 0x106, PC 0x10A, Align 0x108, target hw[22]=0x12C → imm=0x24 → imm8=9 → 0x4D09
///   hw[ 5] addr 0x10A, PC 0x10E, Align 0x10C, target hw[24]=0x130 → imm=0x24 → imm8=9 → 0x4909
///   hw[ 8] addr 0x110, PC 0x114, Align 0x114, target hw[26]=0x134 → imm=0x20 → imm8=8 → 0x4908
///   hw[10] addr 0x114, PC 0x118, Align 0x118, target hw[28]=0x138 → imm=0x20 → imm8=8 → 0x4E08
const MAIN_WFI: [u16; 30] = [
    0x4C09, // [ 0] ldr  r4, [pc, #36]    — NVIC_ISER0_ADDR
    0x2001, // [ 1] movs r0, #1
    0x6020, // [ 2] str  r0, [r4]         — *NVIC_ISER0 = 1
    0x4D09, // [ 3] ldr  r5, [pc, #36]    — TIMER_INTE_ADDR
    0x6028, // [ 4] str  r0, [r5]         — *TIMER.INTE = 1
    0x4909, // [ 5] ldr  r1, [pc, #36]    — TIMER_TIMERAWL_ADDR
    0x680A, // [ 6] ldr  r2, [r1]         — r2 = TIMERAWL
    0x32C8, // [ 7] adds r2, #200         — r2 = deadline
    0x4908, // [ 8] ldr  r1, [pc, #32]    — TIMER_ALARM0_ADDR
    0x600A, // [ 9] str  r2, [r1]         — *TIMER.ALARM0 = deadline
    0x4E08, // [10] ldr  r6, [pc, #32]    — PHASE_ADDR
    0x6030, // [11] str  r0, [r6]         — *phase = 1  (r0 still 1)
    0xBF30, // [12] wfi                   — park core
    0x2002, // [13] movs r0, #2
    0x6030, // [14] str  r0, [r6]         — *phase = 2  (post-handler)
    0xE7FE, // [15] b    .                — busy-wait
    0xBE00, // [16] bkpt #0               — safety net
    0xBF00, // [17] nop padding
    0xBF00, // [18] nop padding
    0xBF00, // [19] nop padding
    0xE100, // [20] lit: NVIC_ISER0_ADDR     low  (0xE000_E100)
    0xE000, // [21] lit: NVIC_ISER0_ADDR     high
    0x4038, // [22] lit: TIMER_INTE_ADDR     low  (0x4005_4038)
    0x4005, // [23] lit: TIMER_INTE_ADDR     high
    0x4028, // [24] lit: TIMER_TIMERAWL_ADDR low  (0x4005_4028)
    0x4005, // [25] lit: TIMER_TIMERAWL_ADDR high
    0x4010, // [26] lit: TIMER_ALARM0_ADDR   low  (0x4005_4010)
    0x4005, // [27] lit: TIMER_ALARM0_ADDR   high
    0x3FC8, // [28] lit: PHASE_ADDR          low  (0x2000_3FC8)
    0x2000, // [29] lit: PHASE_ADDR          high
];

// Pin literal-pool byte offsets.
const _: () = assert!(
    MAIN_WFI[20] == 0xE100 && MAIN_WFI[21] == 0xE000,
    "NVIC_ISER0_ADDR literal must remain at hw[20..=21]",
);
const _: () = assert!(
    MAIN_WFI[22] == 0x4038 && MAIN_WFI[23] == 0x4005,
    "TIMER_INTE_ADDR literal must remain at hw[22..=23]",
);
const _: () = assert!(
    MAIN_WFI[24] == 0x4028 && MAIN_WFI[25] == 0x4005,
    "TIMER_TIMERAWL_ADDR literal must remain at hw[24..=25]",
);
const _: () = assert!(
    MAIN_WFI[26] == 0x4010 && MAIN_WFI[27] == 0x4005,
    "TIMER_ALARM0_ADDR literal must remain at hw[26..=27]",
);
const _: () = assert!(
    MAIN_WFI[28] == 0x3FC8 && MAIN_WFI[29] == 0x2000,
    "PHASE_ADDR literal must remain at hw[28..=29]",
);
const _: () = assert!(
    (MAIN_OFFSET as usize) + MAIN_WFI.len() * 2 <= ISR_IMAGE_SIZE,
    "MAIN_WFI must fit inside the image's main region",
);

/// V2 §3.1 main: program NVIC priorities, enable both IRQs, arm both
/// alarms at the same TIMERAWL deadline, enable both INTE bits, spin.
///
/// Sequence:
///   1. `*NVIC_IPR0 = 0x0000_40C0` — IRQ #0 priority byte = 0xC0
///      (effective 3); IRQ #1 priority byte = 0x40 (effective 1). Lower
///      numeric value wins, so IRQ #1 dispatches first.
///   2. `*NVIC_ISER0 = 3` — enable IRQ #0 + IRQ #1.
///   3. Read TIMERAWL, add 200-tick delta → deadline.
///   4. Write deadline to `TIMER_ALARM0` AND `TIMER_ALARM1` so both
///      alarms fire on the same `tick_peripherals` call.
///   5. `*TIMER_INTE = 3` — route both alarms to NVIC bits 0 and 1.
///   6. `b .` — busy-wait. Both alarms fire → both NVIC pending bits set
///      → `try_take_any_pending_exception` picks IRQ #1 first (lower
///      priority value); on its return, tail-chain poll picks IRQ #0.
///
/// Register convention:
///   r0 — scratch (MMIO target address per step)
///   r1 — scratch (priority pattern, then enable mask `3`, then INTE mask)
///   r2 — held: deadline value
///
/// Layout — image bytes 0x100..0x144:
/// ```text
///   [ 0] ldr  r0, [pc, #36]    ; r0 = NVIC_IPR0_ADDR             (lit hw[20])
///   [ 1] ldr  r1, [pc, #40]    ; r1 = 0x0000_40C0                (lit hw[22])
///   [ 2] str  r1, [r0]         ; *NVIC_IPR0 = 0x40C0 (priority array)
///   [ 3] ldr  r0, [pc, #40]    ; r0 = NVIC_ISER0_ADDR            (lit hw[24])
///   [ 4] movs r1, #3
///   [ 5] str  r1, [r0]         ; *NVIC_ISER0 = 3 (enable IRQ#0+#1)
///   [ 6] ldr  r0, [pc, #36]    ; r0 = TIMER_TIMERAWL_ADDR        (lit hw[26])
///   [ 7] ldr  r2, [r0]         ; r2 = TIMERAWL
///   [ 8] adds r2, #200         ; r2 = deadline
///   [ 9] ldr  r0, [pc, #36]    ; r0 = TIMER_ALARM0_ADDR          (lit hw[28])
///   [10] str  r2, [r0]         ; arm ALARM0
///   [11] ldr  r0, [pc, #36]    ; r0 = TIMER_ALARM1_ADDR          (lit hw[30])
///   [12] str  r2, [r0]         ; arm ALARM1 (same deadline)
///   [13] ldr  r0, [pc, #36]    ; r0 = TIMER_INTE_ADDR            (lit hw[32])
///   [14] str  r1, [r0]         ; *INTE = 3 (r1 still 3)
///   [15] b    .                ; busy-wait
///   [16] bkpt #0               ; safety net
///   [17..19] nop padding
///   [20..21] lit: NVIC_IPR0_ADDR        (0xE000_E400)
///   [22..23] lit: 0x0000_40C0           — priority pattern
///   [24..25] lit: NVIC_ISER0_ADDR        (0xE000_E100)
///   [26..27] lit: TIMER_TIMERAWL_ADDR   (0x4005_4028)
///   [28..29] lit: TIMER_ALARM0_ADDR     (0x4005_4010)
///   [30..31] lit: TIMER_ALARM1_ADDR     (0x4005_4014)
///   [32..33] lit: TIMER_INTE_ADDR       (0x4005_4038)
/// ```
///
/// Literal math — main lives at MAIN_OFFSET = 0x100, hw[i] byte =
/// 0x100 + 2*i. `ldr [pc, #imm8*4]` rounds PC down to a 4-byte boundary
/// before adding imm8*4.
///
///   hw[ 0] addr 0x100, PC 0x104, Align 0x104, target hw[20]=0x128 → imm=0x24 → imm8=9 → 0x4809
///   hw[ 1] addr 0x102, PC 0x106, Align 0x104, target hw[22]=0x12C → imm=0x28 → imm8=10 → 0x490A
///   hw[ 3] addr 0x106, PC 0x10A, Align 0x108, target hw[24]=0x130 → imm=0x28 → imm8=10 → 0x480A
///   hw[ 6] addr 0x10C, PC 0x110, Align 0x110, target hw[26]=0x134 → imm=0x24 → imm8=9 → 0x4809
///   hw[ 9] addr 0x112, PC 0x116, Align 0x114, target hw[28]=0x138 → imm=0x24 → imm8=9 → 0x4809
///   hw[11] addr 0x116, PC 0x11A, Align 0x118, target hw[30]=0x13C → imm=0x24 → imm8=9 → 0x4809
///   hw[13] addr 0x11A, PC 0x11E, Align 0x11C, target hw[32]=0x140 → imm=0x24 → imm8=9 → 0x4809
const MAIN_PRIORITY_PREEMPT: [u16; 34] = [
    0x4809, // [ 0] ldr  r0, [pc, #36]   — NVIC_IPR0_ADDR
    0x490A, // [ 1] ldr  r1, [pc, #40]   — 0x0000_40C0
    0x6001, // [ 2] str  r1, [r0]        — *NVIC_IPR0 = 0x40C0
    0x480A, // [ 3] ldr  r0, [pc, #40]   — NVIC_ISER0_ADDR
    0x2103, // [ 4] movs r1, #3
    0x6001, // [ 5] str  r1, [r0]        — *NVIC_ISER0 = 3
    0x4809, // [ 6] ldr  r0, [pc, #36]   — TIMER_TIMERAWL_ADDR
    0x6802, // [ 7] ldr  r2, [r0]        — r2 = TIMERAWL
    0x32C8, // [ 8] adds r2, #200        — r2 = deadline
    0x4809, // [ 9] ldr  r0, [pc, #36]   — TIMER_ALARM0_ADDR
    0x6002, // [10] str  r2, [r0]        — arm ALARM0
    0x4809, // [11] ldr  r0, [pc, #36]   — TIMER_ALARM1_ADDR
    0x6002, // [12] str  r2, [r0]        — arm ALARM1 (same deadline)
    0x4809, // [13] ldr  r0, [pc, #36]   — TIMER_INTE_ADDR
    0x6001, // [14] str  r1, [r0]        — *INTE = 3 (r1 still 3)
    0xE7FE, // [15] b    .               — busy-wait
    0xBE00, // [16] bkpt #0              — safety net
    0xBF00, // [17] nop padding
    0xBF00, // [18] nop padding
    0xBF00, // [19] nop padding
    0xE400, // [20] lit: NVIC_IPR0_ADDR        low  (0xE000_E400)
    0xE000, // [21] lit: NVIC_IPR0_ADDR        high
    0x40C0, // [22] lit: 0x0000_40C0           low  — priority pattern
    0x0000, // [23] lit: 0x0000_40C0           high
    0xE100, // [24] lit: NVIC_ISER0_ADDR        low  (0xE000_E100)
    0xE000, // [25] lit: NVIC_ISER0_ADDR        high
    0x4028, // [26] lit: TIMER_TIMERAWL_ADDR   low  (0x4005_4028)
    0x4005, // [27] lit: TIMER_TIMERAWL_ADDR   high
    0x4010, // [28] lit: TIMER_ALARM0_ADDR     low  (0x4005_4010)
    0x4005, // [29] lit: TIMER_ALARM0_ADDR     high
    0x4014, // [30] lit: TIMER_ALARM1_ADDR     low  (0x4005_4014)
    0x4005, // [31] lit: TIMER_ALARM1_ADDR     high
    0x4038, // [32] lit: TIMER_INTE_ADDR       low  (0x4005_4038)
    0x4005, // [33] lit: TIMER_INTE_ADDR       high
];

// Pin literal-pool byte offsets.
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[20] == 0xE400 && MAIN_PRIORITY_PREEMPT[21] == 0xE000,
    "NVIC_IPR0_ADDR literal must remain at hw[20..=21]",
);
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[22] == 0x40C0 && MAIN_PRIORITY_PREEMPT[23] == 0x0000,
    "priority pattern 0x0000_40C0 literal must remain at hw[22..=23]",
);
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[24] == 0xE100 && MAIN_PRIORITY_PREEMPT[25] == 0xE000,
    "NVIC_ISER0_ADDR literal must remain at hw[24..=25]",
);
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[26] == 0x4028 && MAIN_PRIORITY_PREEMPT[27] == 0x4005,
    "TIMER_TIMERAWL_ADDR literal must remain at hw[26..=27]",
);
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[28] == 0x4010 && MAIN_PRIORITY_PREEMPT[29] == 0x4005,
    "TIMER_ALARM0_ADDR literal must remain at hw[28..=29]",
);
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[30] == 0x4014 && MAIN_PRIORITY_PREEMPT[31] == 0x4005,
    "TIMER_ALARM1_ADDR literal must remain at hw[30..=31]",
);
const _: () = assert!(
    MAIN_PRIORITY_PREEMPT[32] == 0x4038 && MAIN_PRIORITY_PREEMPT[33] == 0x4005,
    "TIMER_INTE_ADDR literal must remain at hw[32..=33]",
);
const _: () = assert!(
    (MAIN_OFFSET as usize) + MAIN_PRIORITY_PREEMPT.len() * 2 <= ISR_IMAGE_SIZE,
    "MAIN_PRIORITY_PREEMPT must fit inside the image's main region",
);

// ---------------------------------------------------------------------------
// Scenario images
// ---------------------------------------------------------------------------

const IMAGE_TIMER_COLD: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_TIMER, MAIN_TIMER);

const IMAGE_TAIL_CHAIN: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_TAIL, MAIN_TAIL);

/// V2 §3.4 image. No handler dispatch — passes an empty `[u16; 0]` so
/// the handler region stays zero. Vector slots [14..16] still point at
/// HANDLER_OFFSET (as the builder mandates), but no scenario instruction
/// pends those exceptions, so the zero-filled handler region is never
/// reached. Slots [2..13] still point at the `bkpt #1` default handler
/// for any architecturally-required entries.
const IMAGE_NVIC_RAZWI: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, [], MAIN_NVIC_RAZWI);

/// V2 §3.2 image — PRIMASK-gated pend then `cpsie i` unmask.
const IMAGE_MASKED_PENDING: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_MASKED, MAIN_MASKED);

/// V2 §3.3 image — WFI wake on TIMER ALARM0.
const IMAGE_WFI_WAKE: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_WFI, MAIN_WFI);

/// Const helper — overwrite one vector-table entry's u32. Used by the V2
/// §3.1 image so vector entry [17] (TIMER_IRQ_1) lands on HANDLER_IRQ1's
/// first instruction inside the concatenated `HANDLER_PRIORITY_PREEMPT`
/// array, while vector entry [16] (TIMER_IRQ_0) keeps the
/// builder-default of HANDLER_OFFSET (HANDLER_IRQ0's entry).
///
/// `entry_idx` indexes 4-byte vector-table slots starting at byte 0.
/// `build_image_m0plus` only fills entries [0..VECTOR_TABLE_ENTRIES] with
/// non-zero values; entries 17..=N occupy bytes that are part of the
/// post-vector "zero padding" region (HLD layout) and so are still inside
/// the vector-table-shaped prefix of the image up to DEFAULT_HANDLER_OFFSET.
/// The bounds check therefore allows entry indices in the range
/// `[0, DEFAULT_HANDLER_OFFSET / 4)` so the patch can land in the padding
/// gap without colliding with the default-handler `bkpt #1`.
const fn patch_vector_entry(
    mut image: [u8; ISR_IMAGE_SIZE],
    entry_idx: usize,
    target: u32,
) -> [u8; ISR_IMAGE_SIZE] {
    assert!(
        entry_idx * 4 < DEFAULT_HANDLER_OFFSET as usize,
        "patch_vector_entry: entry_idx must land inside the vector-table prefix",
    );
    let off = entry_idx * 4;
    let b = target.to_le_bytes();
    image[off] = b[0];
    image[off + 1] = b[1];
    image[off + 2] = b[2];
    image[off + 3] = b[3];
    image
}

/// V2 §3.1 image — priority preemption. Built by `build_image_m0plus`,
/// then patched so vector entry [17] (TIMER_IRQ_1) points at
/// HANDLER_IRQ1's entry inside `HANDLER_PRIORITY_PREEMPT`
/// (= HANDLER_OFFSET + HANDLER_IRQ1_OFFSET_BYTES). Vector entry [16]
/// (TIMER_IRQ_0) is left at the builder default of HANDLER_OFFSET,
/// where HANDLER_IRQ0 starts.
const IMAGE_PRIORITY_PREEMPT: [u8; ISR_IMAGE_SIZE] = patch_vector_entry(
    build_image_m0plus(
        ISR_IMAGE_BASE,
        ISR_STACK_TOP,
        HANDLER_PRIORITY_PREEMPT,
        MAIN_PRIORITY_PREEMPT,
    ),
    17,
    (ISR_IMAGE_BASE + HANDLER_OFFSET + HANDLER_IRQ1_OFFSET_BYTES) | 1,
);

// ---------------------------------------------------------------------------
// Scenario type
// ---------------------------------------------------------------------------

/// Which register to prime on entry.
///
/// Strict M0+ subset of the M33 `IsrReg` enum: no CPACR (no FPU on
/// M0+), no Secure / Non-Secure VTOR split.
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
    /// Main Stack Pointer.
    Msp,
    /// Process Stack Pointer.
    Psp,
    /// CONTROL register (thread mode priv + SPSEL).
    Control,
    /// VTOR — vector table offset.
    Vtor,
}

/// Observable the runner reads post-halt on both HW and EMU.
#[derive(Copy, Clone, Debug)]
pub enum IsrObservable {
    /// Absolute-address MMIO read, masked by the second field.
    Mmio(u32, u32),
    /// SRAM u32 read at the given address (used for counters).
    Memory(u32),
}

/// A single ISR oracle scenario.
pub struct IsrScenario {
    pub name: &'static str,
    pub image: &'static [u8],
    pub entry_offset: u32,
    pub init_regs: &'static [(IsrReg, u32)],
    /// Wall-clock budget in milliseconds for the EMU side. Dispatch is
    /// modelled (see the module-level Status block); kept short so the
    /// oracle completes quickly.
    pub max_millis: u32,
    pub observe: &'static [(&'static str, IsrObservable)],
}

// ---------------------------------------------------------------------------
// Per-scenario init_regs + observables
// ---------------------------------------------------------------------------

const INIT_TIMER_COLD: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_TIMER_COLD: &[(&str, IsrObservable)] = &[
    // Primary load-bearing observable: the handler ran exactly once so
    // the counter == 1. Dispatch path is silicon-validated; EMU and
    // silicon both PASS.
    ("ctr_timer", IsrObservable::Memory(CTR_TIMER_ADDR)),
    // TIMER.INTR bit 0 clear after the W1C inside the handler.
    ("timer_intr", IsrObservable::Mmio(TIMER_INTR_ADDR, 0x1)),
];

const INIT_TAIL_CHAIN: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_TAIL_CHAIN: &[(&str, IsrObservable)] = &[
    // Both handlers ran at least once — tail-chain invariant. Widened
    // to "== 1" via full-word compare (the handler path is idempotent
    // but main busy-waits forever so each exception fires exactly
    // once before the runner halts).
    ("ctr_pendsv", IsrObservable::Memory(CTR_PENDSV_ADDR)),
    ("ctr_systick", IsrObservable::Memory(CTR_SYSTICK_ADDR)),
];

const INIT_MASKED: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_MASKED: &[(&str, IsrObservable)] = &[
    // Primary load-bearing observable: gate snapshot at handler entry.
    // PASS condition is `gate_at_entry == 1` — the handler ran AFTER
    // the gate=1 store and BEFORE the gate=2 store, i.e. dispatch
    // happened on the `cpsie i` boundary (not later).
    ("gate_at_entry", IsrObservable::Memory(GATE_AT_ENTRY_ADDR)),
    // Secondary: the handler ran exactly once.
    ("ctr_timer", IsrObservable::Memory(CTR_TIMER_ADDR)),
    // Secondary: main resumed after handler returned (gate==2).
    ("gate", IsrObservable::Memory(GATE_ADDR)),
];

const INIT_WFI: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_WFI: &[(&str, IsrObservable)] = &[
    // Primary load-bearing observable: phase snapshot at handler entry.
    // PASS condition is `phase_at_entry == 1` — the handler ran during
    // the WFI window, AFTER `*phase=1` but BEFORE main resumed past
    // `wfi` and stored `*phase=2`.
    ("phase_at_entry", IsrObservable::Memory(PHASE_AT_ENTRY_ADDR)),
    // Secondary: the handler ran exactly once.
    ("ctr_timer", IsrObservable::Memory(CTR_TIMER_ADDR)),
    // Secondary: main resumed past WFI after the handler returned.
    ("phase", IsrObservable::Memory(PHASE_ADDR)),
];

const INIT_PRIORITY_PREEMPT: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_PRIORITY_PREEMPT: &[(&str, IsrObservable)] = &[
    // Primary load-bearing observable: which handler ran first. PASS
    // condition is `order_first_irq == 0xA1` (IRQ_1 sentinel) — IRQ_1
    // has the lower priority value, so it must dispatch first; IRQ_0
    // then runs via tail-chain. Listed first so `primary_observable_addr`
    // polls it.
    ("order_first_irq", IsrObservable::Memory(ORDER_FIRST_IRQ_ADDR)),
    // Secondary: each handler ran exactly once.
    ("ctr_irq_0", IsrObservable::Memory(CTR_IRQ_0_ADDR)),
    ("ctr_irq_1", IsrObservable::Memory(CTR_IRQ_1_ADDR)),
];

const INIT_NVIC_RAZWI: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_NVIC_RAZWI: &[(&str, IsrObservable)] = &[
    // Primary load-bearing observable: ISER0 high bits read as zero.
    // Listed first so `primary_observable_addr` polls it.
    ("iser_readback", IsrObservable::Memory(ISER_READBACK_ADDR)),
    ("ispr_readback", IsrObservable::Memory(ISPR_READBACK_ADDR)),
    // ICER/ICPR readback expected to be 0 (whole pre-seed cleared by the
    // masked write). ICER/ICPR readbacks are stored later in the main
    // body than the ISER/ISPR ones; the runner's one-extra-chunk grace
    // window covers the whole tail before halting — see `run_emu_scenario`.
    ("icer_readback", IsrObservable::Memory(ICER_READBACK_ADDR)),
    ("icpr_readback", IsrObservable::Memory(ICPR_READBACK_ADDR)),
];

// ---------------------------------------------------------------------------
// Catalogue
// ---------------------------------------------------------------------------

/// Phase 1 minimum catalogue per V7 §6.1: cold TIMER_IRQ_0 +
/// PendSV+SysTick tail-chain. All names prefixed `isr_m0_` so the
/// filter substring doesn't alias the RP2350 catalogue.
pub const SCENARIOS: &[IsrScenario] = &[
    IsrScenario {
        name: "isr_m0_timer_cold",
        image: &IMAGE_TIMER_COLD,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_TIMER_COLD,
        max_millis: 1500,
        observe: OBS_TIMER_COLD,
    },
    IsrScenario {
        name: "isr_m0_tail_chain_pendsv_systick",
        image: &IMAGE_TAIL_CHAIN,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_TAIL_CHAIN,
        max_millis: 1500,
        observe: OBS_TAIL_CHAIN,
    },
    // V2 §3.4: NVIC ISER/ICER/ISPR/ICPR high-bits RAZ/WI. No handler
    // dispatch — pure register-shape assertion.
    IsrScenario {
        name: "isr_m0_nvic_high_bits_razwi",
        image: &IMAGE_NVIC_RAZWI,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_NVIC_RAZWI,
        max_millis: 1500,
        observe: OBS_NVIC_RAZWI,
    },
    // V2 §3.2: PRIMASK-gated pend, then `cpsie i` unmask. Verifies the
    // PRIMASK gate inside `try_take_any_pending_exception` and proves
    // dispatch happens on the `cpsie i` boundary, not later.
    IsrScenario {
        name: "isr_m0_masked_pending_unmask",
        image: &IMAGE_MASKED_PENDING,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_MASKED,
        max_millis: 1500,
        observe: OBS_MASKED,
    },
    // V2 §3.3: WFI wake on TIMER ALARM0. Main parks the core via WFI
    // after arming the alarm; the alarm fires, NVIC re-pends, the core
    // un-halts, dispatches the handler, and main resumes.
    IsrScenario {
        name: "isr_m0_wfi_wake",
        image: &IMAGE_WFI_WAKE,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_WFI,
        max_millis: 1500,
        observe: OBS_WFI,
    },
    // V2 §3.1: priority-preempt / tail-chain. Two TIMER alarms fire in
    // lock-step on the same TIMERAWL deadline; IPR0 gives IRQ #1 a lower
    // priority value than IRQ #0, so IRQ #1's handler dispatches first
    // and IRQ #0's runs as a tail-chain. Validates the priority array
    // path through `Nvic::highest_priority_pending` and
    // `try_take_any_pending_exception`.
    IsrScenario {
        name: "isr_m0_priority_preempt",
        image: &IMAGE_PRIORITY_PREEMPT,
        entry_offset: MAIN_OFFSET,
        init_regs: INIT_PRIORITY_PREEMPT,
        max_millis: 1500,
        observe: OBS_PRIORITY_PREEMPT,
    },
];

// ---------------------------------------------------------------------------
// Runner
// ---------------------------------------------------------------------------

const PC_REG: RegisterId = RegisterId(15);
const XPSR_REG: RegisterId = RegisterId(16);
const SP_REG: RegisterId = RegisterId(13);
const LR_REG: RegisterId = RegisterId(14);
const MSP_REG: RegisterId = RegisterId(17);
const PSP_REG: RegisterId = RegisterId(18);
/// Debug register 20 packs {CONTROL, FAULTMASK, BASEPRI, PRIMASK}.
/// On M0+ only CONTROL (byte 3) and PRIMASK (byte 0) are defined;
/// writing 0 zeroes both.
const EXTRA_REG: RegisterId = RegisterId(0b10100);

/// Arguments for `run_against`.
#[derive(Clone, Debug, Default)]
pub struct IsrArgs {
    pub filter: Option<String>,
    pub verbose: bool,
}

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
        IsrReg::Msp => MSP_REG,
        IsrReg::Psp => PSP_REG,
        IsrReg::Control => EXTRA_REG,
        IsrReg::Vtor => return None,
    })
}

fn apply_init_regs_hw(
    core: &mut Core,
    init_regs: &[(IsrReg, u32)],
) -> Result<(), Box<dyn std::error::Error>> {
    for &(reg, val) in init_regs {
        match reg {
            IsrReg::Vtor => {
                core.write_word_32(VTOR_ADDR as u64, val)?;
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

fn apply_init_regs_emu(emu: &mut mdrp2040::Emulator, init_regs: &[(IsrReg, u32)]) {
    for &(reg, val) in init_regs {
        match reg {
            IsrReg::Vtor => emu.mmio_write32(VTOR_ADDR, val),
            IsrReg::R0 => emu.core_mut(0).regs.r[0] = val,
            IsrReg::R1 => emu.core_mut(0).regs.r[1] = val,
            IsrReg::R2 => emu.core_mut(0).regs.r[2] = val,
            IsrReg::R3 => emu.core_mut(0).regs.r[3] = val,
            IsrReg::R4 => emu.core_mut(0).regs.r[4] = val,
            IsrReg::R5 => emu.core_mut(0).regs.r[5] = val,
            IsrReg::R6 => emu.core_mut(0).regs.r[6] = val,
            IsrReg::R7 => emu.core_mut(0).regs.r[7] = val,
            IsrReg::Msp => emu.core_mut(0).regs.msp = val,
            IsrReg::Psp => emu.core_mut(0).regs.psp = val,
            IsrReg::Control => emu.core_mut(0).regs.control = val,
        }
    }
}

/// Scenario-specific MMIO preamble. Handles the one case (tail-chain)
/// that needs SysTick pre-armed via MMIO rather than through init_regs.
fn scenario_preamble_hw(core: &mut Core, name: &str) -> Result<(), Box<dyn std::error::Error>> {
    if name == "isr_m0_tail_chain_pendsv_systick" {
        // Arm SysTick with a short reload so the underflow fires quickly.
        core.write_word_32(SYST_RVR_ADDR as u64, 4)?;
        core.write_word_32(SYST_CVR_ADDR as u64, 0)?;
        core.write_word_32(SYST_CSR_ADDR as u64, SYST_CSR_ENABLE_TICKINT_CORE)?;
    }
    Ok(())
}

fn scenario_preamble_emu(emu: &mut mdrp2040::Emulator, name: &str) {
    if name == "isr_m0_tail_chain_pendsv_systick" {
        emu.mmio_write32(SYST_RVR_ADDR, 4);
        emu.mmio_write32(SYST_CVR_ADDR, 0);
        emu.mmio_write32(SYST_CSR_ADDR, SYST_CSR_ENABLE_TICKINT_CORE);
    }
}

fn read_observable_hw(
    core: &mut Core,
    obs: IsrObservable,
) -> Result<u32, Box<dyn std::error::Error>> {
    Ok(match obs {
        IsrObservable::Mmio(addr, _mask) => core.read_word_32(addr as u64)?,
        IsrObservable::Memory(addr) => core.read_word_32(addr as u64)?,
    })
}

fn read_observable_emu(emu: &mut mdrp2040::Emulator, obs: IsrObservable) -> u32 {
    match obs {
        IsrObservable::Mmio(addr, _mask) => emu.mmio_read32(addr),
        IsrObservable::Memory(addr) => emu.peek(addr),
    }
}

fn observable_mask(obs: IsrObservable) -> u32 {
    match obs {
        IsrObservable::Mmio(_, mask) => mask,
        IsrObservable::Memory(_) => !0,
    }
}

/// Clear the per-scenario counter cells and TIMER state in SRAM / MMIO.
/// Done before every scenario on both sides so a previous scenario's
/// counter doesn't bleed through.
fn reset_scenario_state_hw(core: &mut Core) -> Result<(), Box<dyn std::error::Error>> {
    core.write_word_32(CTR_TIMER_ADDR as u64, 0)?;
    core.write_word_32(CTR_PENDSV_ADDR as u64, 0)?;
    core.write_word_32(CTR_SYSTICK_ADDR as u64, 0)?;
    // V2 §3.4 — NVIC RAZ/WI scenario observable cells.
    core.write_word_32(ISER_READBACK_ADDR as u64, 0)?;
    core.write_word_32(ISPR_READBACK_ADDR as u64, 0)?;
    core.write_word_32(ICER_READBACK_ADDR as u64, 0)?;
    core.write_word_32(ICPR_READBACK_ADDR as u64, 0)?;
    // V2 §3.2 — PRIMASK-gate scenario observable cells.
    core.write_word_32(GATE_ADDR as u64, 0)?;
    core.write_word_32(GATE_AT_ENTRY_ADDR as u64, 0)?;
    // V2 §3.3 — WFI-wake scenario observable cells.
    core.write_word_32(PHASE_ADDR as u64, 0)?;
    core.write_word_32(PHASE_AT_ENTRY_ADDR as u64, 0)?;
    // V2 §3.1 — priority-preempt scenario observable cells.
    core.write_word_32(CTR_IRQ_0_ADDR as u64, 0)?;
    core.write_word_32(CTR_IRQ_1_ADDR as u64, 0)?;
    core.write_word_32(ORDER_FIRST_IRQ_ADDR as u64, 0)?;
    // Clear TIMER.INTR (W1C both alarm flags), disable INTE, and disarm
    // every ALARM. ARMED is W1C — write 0xF to disarm all four alarms;
    // without this, `alarm_fire_cycle` slots from a prior scenario stay
    // live and can re-pend INTR after this reset.
    core.write_word_32(TIMER_ARMED_ADDR as u64, 0xF)?;
    core.write_word_32(TIMER_INTR_ADDR as u64, 0xFFFF_FFFF)?;
    core.write_word_32(TIMER_INTE_ADDR as u64, 0)?;
    // Clear any pre-seeded NVIC state so the V2 §3.4 / §3.2 / §3.1
    // scenarios start from a known zero pending/enabled/priority mask.
    // ICER0/ICPR0 use W1C semantics — writing all-ones clears every bit.
    // ISER0 has no direct W1C, so ICER0 alone is what disables enabled
    // IRQs. NVIC_IPR0 must be zeroed too — V2 §3.1 leaves a non-zero
    // priority pattern there which would otherwise bleed into V1 / Stage
    // 2-4 scenarios.
    core.write_word_32(NVIC_ICER0_ADDR as u64, 0xFFFF_FFFF)?;
    core.write_word_32(NVIC_ICPR0_ADDR as u64, 0xFFFF_FFFF)?;
    core.write_word_32(NVIC_IPR0_ADDR as u64, 0)?;
    Ok(())
}

fn reset_scenario_state_emu(emu: &mut mdrp2040::Emulator) {
    emu.poke(CTR_TIMER_ADDR, 0);
    emu.poke(CTR_PENDSV_ADDR, 0);
    emu.poke(CTR_SYSTICK_ADDR, 0);
    emu.poke(ISER_READBACK_ADDR, 0);
    emu.poke(ISPR_READBACK_ADDR, 0);
    emu.poke(ICER_READBACK_ADDR, 0);
    emu.poke(ICPR_READBACK_ADDR, 0);
    // V2 §3.2 — PRIMASK-gate scenario observable cells.
    emu.poke(GATE_ADDR, 0);
    emu.poke(GATE_AT_ENTRY_ADDR, 0);
    // V2 §3.3 — WFI-wake scenario observable cells.
    emu.poke(PHASE_ADDR, 0);
    emu.poke(PHASE_AT_ENTRY_ADDR, 0);
    // V2 §3.1 — priority-preempt scenario observable cells.
    emu.poke(CTR_IRQ_0_ADDR, 0);
    emu.poke(CTR_IRQ_1_ADDR, 0);
    emu.poke(ORDER_FIRST_IRQ_ADDR, 0);
    // Disarm every ALARM (W1C — write 0xF) before clearing INTR so a
    // pending fire-cycle can't re-pend INTR after this reset. Mirrors
    // the HW-side reset above.
    emu.mmio_write32(TIMER_ARMED_ADDR, 0xF);
    emu.mmio_write32(TIMER_INTR_ADDR, 0xFFFF_FFFF);
    emu.mmio_write32(TIMER_INTE_ADDR, 0);
    emu.mmio_write32(NVIC_ICER0_ADDR, 0xFFFF_FFFF);
    emu.mmio_write32(NVIC_ICPR0_ADDR, 0xFFFF_FFFF);
    // V2 §3.1 — clear NVIC priority array (defends against bleed-through
    // from a prior priority-preempt run leaving non-zero priorities).
    emu.mmio_write32(NVIC_IPR0_ADDR, 0);
}

/// Result of running a scenario's EMU half. Per HLD V5 §6.2.
#[derive(Debug)]
pub enum EmuOutcome {
    /// Run completed; observables collected per `sc.observe`.
    Completed(Vec<u32>),
    /// CPU entered HardFault during the run — distinct failure class
    /// so a misdispatch surfaces clearly vs a generic counter mismatch.
    HardFault { pc: u32, ipsr: u8 },
    /// Cycle budget exhausted with no observable progress.
    Timeout,
}

/// Pick the primary success observable for a scenario. The runner
/// polls this address every chunk; once it's non-zero, one extra
/// chunk runs (so the tail-chain scenario can fire its second
/// exception) and the run returns `Completed`.
fn primary_observable_addr(name: &str) -> u32 {
    match name {
        "isr_m0_timer_cold" => CTR_TIMER_ADDR,
        "isr_m0_tail_chain_pendsv_systick" => CTR_PENDSV_ADDR,
        "isr_m0_nvic_high_bits_razwi" => ISER_READBACK_ADDR,
        "isr_m0_masked_pending_unmask" => GATE_AT_ENTRY_ADDR,
        "isr_m0_wfi_wake" => PHASE_AT_ENTRY_ADDR,
        "isr_m0_priority_preempt" => ORDER_FIRST_IRQ_ADDR,
        other => {
            panic!("primary_observable_addr: unknown scenario '{other}'; add it to this match")
        }
    }
}

/// Set up an `Emulator` for an ISR scenario: image upload, scenario-
/// state reset, init regs, scenario preamble, DWT/CYCCNT priming, and
/// core-0 thread-mode register init. No probe-rs.
///
/// Caller is responsible for building the `Emulator`, halting core 1
/// (M0+ V1 oracle assumes single-core), and setting the active core
/// to 0 before calling this helper.
pub fn setup_emulator_image(emu: &mut mdrp2040::Emulator, sc: &IsrScenario) {
    // Bus::new() defaults all peripherals to held-in-reset, but real
    // RP2040 silicon releases TIMER + WATCHDOG via bootrom before user
    // code runs. Mirror that here so MAIN_TIMER's writes to ALARM0 /
    // INTE / NVIC_ISER0 actually land. Address 0x4000_F000 is the
    // RESETS_CLR alias (offset 0x3000 from RESETS_BASE 0x4000_C000);
    // W1S clears the named bits per `Bus::resets_clr_deasserts` test
    // at bus/mod.rs:1937.
    emu.mmio_write32(
        0x4000_F000,
        (1 << 21)   // RESET_TIMER
            | (1 << 24), // RESET_WATCHDOG  (TIMER's 1µs tick counts watchdog ticks)
    );

    // Upload the image word-by-word.
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

    reset_scenario_state_emu(emu);
    apply_init_regs_emu(emu, sc.init_regs);
    scenario_preamble_emu(emu, sc.name);

    // Parity with the HW side: enable DWT CYCCNT if modelled. M0+
    // typically has no DWT, but the emulator may model it; the writes
    // are harmless either way.
    let demcr = emu.mmio_read32(silicon_oracle::DEMCR_U32);
    emu.mmio_write32(silicon_oracle::DEMCR_U32, demcr | silicon_oracle::TRCENA);
    let dwt_ctrl = emu.mmio_read32(silicon_oracle::DWT_CTRL_U32);
    emu.mmio_write32(
        silicon_oracle::DWT_CTRL_U32,
        dwt_ctrl | silicon_oracle::CYCCNTENA,
    );
    emu.mmio_write32(silicon_oracle::DWT_CYCCNT_ADDR, 0);

    // Prime core-0 thread-mode state. T-bit set, MSP at stack top, LR
    // sentinel, PRIMASK/CONTROL = 0 (privileged thread mode on MSP,
    // IRQs un-masked).
    let c = emu.core_mut(0);
    c.wake();
    c.regs.set_pc(ISR_IMAGE_BASE + sc.entry_offset);
    c.regs.xpsr = 0x0100_0000;
    c.regs.r[13] = ISR_STACK_TOP;
    c.regs.msp = ISR_STACK_TOP;
    c.regs.r[14] = 0xFFFF_FFFF;
    c.regs.control = 0;
    c.regs.primask = 0;
}

/// Run the EMU side of a scenario forward. Returns observable readouts
/// per `sc.observe` once the primary observable advances past zero, plus
/// a status flag distinguishing normal completion, HardFault, and
/// timeout. Per HLD V5 §6.2:
///
/// * Stepped in chunks of `cycles_per_ms / 4`; after each chunk the
///   primary observable is checked and `is_in_hardfault()` is polled.
/// * On primary observable progress (> 0) one extra chunk runs (so
///   tail-chain has a window to fire the second exception), then the
///   run breaks and returns `Completed`.
/// * On `is_in_hardfault()` true, the run returns `HardFault { pc,
///   ipsr }` immediately — bails on first divergence.
/// * If the cycle budget (`sc.max_millis * cycles_per_ms`) exhausts
///   with no progress, returns `Timeout`.
pub fn run_emu_scenario(emu: &mut mdrp2040::Emulator, sc: &IsrScenario) -> EmuOutcome {
    let cycles_per_ms = (Config::default().sys_clk_hz as u64) / 1000;
    let chunk_cycles = (cycles_per_ms / 4).max(1);
    let total_budget = (sc.max_millis as u64).saturating_mul(cycles_per_ms);
    let primary_addr = primary_observable_addr(sc.name);

    let mut spent: u64 = 0;
    let mut saw_progress = false;

    while spent < total_budget {
        let this_chunk = chunk_cycles.min(total_budget - spent);
        emu.run(this_chunk).expect("Serial run is infallible");
        spent = spent.saturating_add(this_chunk);

        // HardFault check: between chunks, before observable poll, so a
        // misdispatched IRQ landing on the default-handler `bkpt #1`
        // path (which also escalates to HardFault) surfaces here as a
        // distinct failure class.
        if emu.core(0).is_in_hardfault() {
            let pc = emu.core(0).regs.pc();
            let ipsr = (emu.core(0).regs.xpsr & 0x1FF) as u8;
            return EmuOutcome::HardFault { pc, ipsr };
        }

        let primary = emu.peek(primary_addr);
        if primary > 0 {
            saw_progress = true;
            // Run one more chunk so tail-chain has a window to fire its
            // second exception, then break.
            let extra = chunk_cycles.min(total_budget.saturating_sub(spent));
            if extra > 0 {
                emu.run(extra).expect("Serial run is infallible");
                if emu.core(0).is_in_hardfault() {
                    let pc = emu.core(0).regs.pc();
                    let ipsr = (emu.core(0).regs.xpsr & 0x1FF) as u8;
                    return EmuOutcome::HardFault { pc, ipsr };
                }
            }
            break;
        }
    }

    // If we exited via budget exhaustion without progress, that's
    // Timeout. A run that *did* see progress counts as Completed — the
    // primary observable is non-zero and we collect whatever the
    // secondary observables read.
    if !saw_progress {
        return EmuOutcome::Timeout;
    }

    let obs: Vec<u32> = sc
        .observe
        .iter()
        .map(|(_, o)| read_observable_emu(emu, *o))
        .collect();

    EmuOutcome::Completed(obs)
}

/// Run one scenario end-to-end (HW + EMU) and produce the outcome.
///
/// HW halt strategy. Unlike the RP2350 oracle, this one doesn't rely
/// on an in-handler BKPT — the handlers return via `BX LR` so the
/// core keeps running after the ISR fires. The runner polls the
/// counter every few ms and halts the core once it has ticked (or
/// hits `max_millis`). This avoids corrupting the exception-entry /
/// exit measurement with a handler-side BKPT.
fn run_one_scenario(
    core: &mut Core,
    sc: &IsrScenario,
    verbose: bool,
) -> Result<(Verdict, Option<String>, Duration), Box<dyn std::error::Error>> {
    let t0 = Instant::now();

    // ---- HW side -----------------------------------------------------

    if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }

    core.write_8(ISR_IMAGE_BASE as u64, sc.image)?;
    reset_scenario_state_hw(core)?;
    apply_init_regs_hw(core, sc.init_regs)?;
    scenario_preamble_hw(core, sc.name)?;

    // Prime CPU state — Thumb bit set, SP at stack top, LR sentinel,
    // PRIMASK/CONTROL=0 (privileged thread mode on MSP, IRQs enabled).
    core.write_core_reg(PC_REG, ISR_IMAGE_BASE + sc.entry_offset)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?;
    core.write_core_reg(SP_REG, ISR_STACK_TOP)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;
    core.write_core_reg(EXTRA_REG, 0u32)?;

    // Enable CYCCNT for parity with the RP2350 oracle. Harmless on
    // M0+ silicon that lacks DWT — `read_word_32` will return 0 or
    // error out, which the caller tolerates (CYCCNT isn't observed).
    let _ = reset_cyccnt(core);

    core.run()?;

    // Poll until any counter ticks or the wall-clock budget expires.
    let deadline = Instant::now() + Duration::from_millis(sc.max_millis as u64);
    let primary_counter_addr = match sc.name {
        "isr_m0_timer_cold" => CTR_TIMER_ADDR,
        "isr_m0_tail_chain_pendsv_systick" => CTR_PENDSV_ADDR,
        "isr_m0_nvic_high_bits_razwi" => ISER_READBACK_ADDR,
        "isr_m0_masked_pending_unmask" => GATE_AT_ENTRY_ADDR,
        "isr_m0_wfi_wake" => PHASE_AT_ENTRY_ADDR,
        "isr_m0_priority_preempt" => ORDER_FIRST_IRQ_ADDR,
        _ => CTR_TIMER_ADDR,
    };
    loop {
        if Instant::now() > deadline {
            break;
        }
        if !core.status()?.is_halted() {
            // Still running — check counter.
            let cnt: u32 = core.read_word_32(primary_counter_addr as u64).unwrap_or(0);
            if cnt > 0 {
                // Give the second exception a short window to fire
                // (relevant for the tail-chain scenario).
                std::thread::sleep(Duration::from_millis(20));
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        } else {
            // Core halted on its own (BKPT safety net or fault).
            break;
        }
    }
    if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }

    let hw_obs: Vec<u32> = sc
        .observe
        .iter()
        .map(|(_, o)| read_observable_hw(core, *o))
        .collect::<Result<_, _>>()?;

    // ---- EMU side ----------------------------------------------------

    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .expect("Serial build is infallible");
    emu.core_mut(1).halt();
    emu.bus.set_active_core(0);

    setup_emulator_image(&mut emu, sc);
    let outcome = run_emu_scenario(&mut emu, sc);
    emu.core_mut(0).halt();
    emu.bus.set_active_core(0);

    // ---- Diff --------------------------------------------------------
    //
    // HardFault and Timeout are distinct failure classes — they short-
    // circuit the per-observable diff so a regression points directly
    // at the misdispatch / hang instead of looking like a generic
    // counter-mismatch.
    let emu_obs: Vec<u32> = match &outcome {
        EmuOutcome::Completed(obs) => obs.clone(),
        EmuOutcome::HardFault { pc, ipsr } => {
            let msg = format!("EMU hardfault at pc=0x{pc:08X} ipsr={ipsr}");
            if verbose {
                println!("    FAIL {msg}");
            }
            return Ok((Verdict::Fail, Some(msg), t0.elapsed()));
        }
        EmuOutcome::Timeout => {
            let msg = "EMU cycle budget exhausted before primary observable advanced".to_string();
            if verbose {
                println!("    FAIL {msg}");
            }
            return Ok((Verdict::Fail, Some(msg), t0.elapsed()));
        }
    };

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

    let verdict = if first_div.is_none() {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    Ok((verdict, first_div, t0.elapsed()))
}

/// Library entry point. Runs every scenario whose name matches
/// `args.filter` in catalogue order; performs a best-effort cleanup
/// (VTOR=0, SysTick disabled, NVIC ICPR cleared, TIMER.INTE cleared,
/// ICSR pend-clears) before returning.
pub fn run_against(
    core: &mut Core,
    args: &IsrArgs,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error>> {
    if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }
    // Enable CYCCNT for parity — harmless if absent on M0+ silicon.
    let _ = enable_cyccnt(core);

    let selected: Vec<&IsrScenario> = SCENARIOS
        .iter()
        .filter(|s| silicon_oracle::name_matches_filter(s.name, args.filter.as_deref()))
        .collect();

    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(selected.len());
    let mut loop_err: Option<Box<dyn std::error::Error>> = None;

    for sc in &selected {
        match run_one_scenario(core, sc, args.verbose) {
            Ok((verdict, detail, elapsed)) => {
                let elapsed_ms = elapsed.as_millis().min(u32::MAX as u128) as u32;
                outcomes.push(match verdict {
                    Verdict::Pass => CaseOutcome::pass("isr_m0", sc.name, elapsed_ms),
                    Verdict::Fail => {
                        CaseOutcome::fail("isr_m0", sc.name, detail.unwrap_or_default(), elapsed_ms)
                    }
                });
            }
            Err(e) => {
                loop_err = Some(e);
                break;
            }
        }
    }

    // Best-effort cleanup.
    if let Err(e) = core.halt(Duration::from_millis(200)) {
        eprintln!("warning: isr_m0 cleanup halt failed: {e}");
    }
    if let Err(e) = core.write_word_32(VTOR_ADDR as u64, 0) {
        eprintln!("warning: isr_m0 cleanup VTOR write failed: {e}");
    }
    if let Err(e) = core.write_word_32(NVIC_ICPR0_ADDR as u64, 0xFFFF_FFFF) {
        eprintln!("warning: isr_m0 cleanup NVIC ICPR0 write failed: {e}");
    }
    if let Err(e) = core.write_word_32(NVIC_ICER0_ADDR as u64, 0xFFFF_FFFF) {
        eprintln!("warning: isr_m0 cleanup NVIC ICER0 write failed: {e}");
    }
    if let Err(e) = core.write_word_32(SCB_ICSR_ADDR as u64, (1 << 27) | (1 << 25)) {
        eprintln!("warning: isr_m0 cleanup ICSR clear failed: {e}");
    }
    if let Err(e) = core.write_word_32(SYST_CSR_ADDR as u64, 0) {
        eprintln!("warning: isr_m0 cleanup SYST_CSR write failed: {e}");
    }
    if let Err(e) = core.write_word_32(TIMER_INTE_ADDR as u64, 0) {
        eprintln!("warning: isr_m0 cleanup TIMER.INTE write failed: {e}");
    }
    if let Err(e) = core.write_word_32(TIMER_INTR_ADDR as u64, 0xFFFF_FFFF) {
        eprintln!("warning: isr_m0 cleanup TIMER.INTR clear failed: {e}");
    }
    if let Err(e) = core.write_core_reg(EXTRA_REG, 0u32) {
        eprintln!("warning: isr_m0 cleanup CONTROL write failed: {e}");
    }

    // Silence the pend-clear helper (unused until Phase 1 lands).
    let _ = ICSR_PENDSVSET;

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

    #[test]
    fn catalogue_size_and_prefix() {
        assert_eq!(
            SCENARIOS.len(),
            6,
            "Phase 1 (V1×2) + V2 stage 2-5 (×4) = 6 scenarios",
        );
        for s in SCENARIOS {
            assert!(
                s.name.starts_with("isr_m0_"),
                "scenario '{}' must start with 'isr_m0_' prefix",
                s.name,
            );
        }
    }

    #[test]
    fn image_layout_invariants() {
        for sc in SCENARIOS {
            assert_eq!(sc.image.len(), ISR_IMAGE_SIZE, "image size");

            // Word 0 — initial MSP.
            let msp = u32::from_le_bytes([sc.image[0], sc.image[1], sc.image[2], sc.image[3]]);
            assert_eq!(msp, ISR_STACK_TOP, "vector[0]");

            // Word 1 — Reset_Handler.
            let rv = u32::from_le_bytes([sc.image[4], sc.image[5], sc.image[6], sc.image[7]]);
            assert_eq!(rv & 1, 1, "reset vector Thumb LSB");
            assert_eq!(rv & !1, ISR_IMAGE_BASE + MAIN_OFFSET, "reset vector target");

            // Word 14 — PendSV, points at handler.
            let pv = u32::from_le_bytes([sc.image[56], sc.image[57], sc.image[58], sc.image[59]]);
            assert_eq!(pv & 1, 1, "PendSV Thumb LSB");
            assert_eq!(
                pv & !1,
                ISR_IMAGE_BASE + HANDLER_OFFSET,
                "PendSV vector target",
            );

            // Word 15 — SysTick.
            let sv = u32::from_le_bytes([sc.image[60], sc.image[61], sc.image[62], sc.image[63]]);
            assert_eq!(sv & 1, 1, "SysTick Thumb LSB");
            assert_eq!(
                sv & !1,
                ISR_IMAGE_BASE + HANDLER_OFFSET,
                "SysTick vector target",
            );

            // Word 16 — TIMER_IRQ_0.
            let tv = u32::from_le_bytes([sc.image[64], sc.image[65], sc.image[66], sc.image[67]]);
            assert_eq!(tv & 1, 1, "TIMER_IRQ_0 Thumb LSB");
            assert_eq!(
                tv & !1,
                ISR_IMAGE_BASE + HANDLER_OFFSET,
                "TIMER_IRQ_0 vector target",
            );

            // Default handler at 0x048 — bkpt #1.
            let dh = u16::from_le_bytes([
                sc.image[DEFAULT_HANDLER_OFFSET as usize],
                sc.image[DEFAULT_HANDLER_OFFSET as usize + 1],
            ]);
            assert_eq!(dh, 0xBE01, "default handler bkpt #1");
        }
    }

    #[test]
    fn main_routine_fits_image() {
        for sc in SCENARIOS {
            assert!(sc.image.len() >= (MAIN_OFFSET as usize) + 4);
            let main_hw0 = u16::from_le_bytes([
                sc.image[MAIN_OFFSET as usize],
                sc.image[MAIN_OFFSET as usize + 1],
            ]);
            assert_ne!(main_hw0, 0, "main routine starts with zero");
        }
    }

    #[test]
    fn catalogue_names_substring_unique() {
        let mut seen: HashSet<&str> = HashSet::new();
        for sc in SCENARIOS {
            assert!(seen.insert(sc.name), "duplicate name '{}'", sc.name);
        }
        for s1 in SCENARIOS {
            for s2 in SCENARIOS {
                if s1.name == s2.name {
                    continue;
                }
                assert!(
                    !s1.name.contains(s2.name),
                    "'{}' is a substring of '{}' — filter aliasing",
                    s2.name,
                    s1.name,
                );
            }
        }
    }

    #[test]
    fn address_constants_pinned() {
        assert_eq!(VTOR_ADDR, 0xE000_ED08);
        assert_eq!(NVIC_ISER0_ADDR, 0xE000_E100);
        assert_eq!(TIMER_BASE, 0x4005_4000);
        assert_eq!(TIMER_ALARM0_ADDR, 0x4005_4010);
        assert_eq!(TIMER_TIMERAWL_ADDR, 0x4005_4028);
        assert_eq!(TIMER_INTR_ADDR, 0x4005_4034);
        assert_eq!(TIMER_INTE_ADDR, 0x4005_4038);
        assert_eq!(IRQ_TIMER_IRQ_0, 0);
    }

    #[test]
    fn counter_addresses_inside_mailbox_page() {
        // Counters live in the mailbox page (0x2000_3000..0x2000_4000)
        // so the scenario image and stack never collide with them.
        const _: () = {
            assert!(CTR_TIMER_ADDR >= ISR_STACK_TOP);
            assert!(CTR_TIMER_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(CTR_PENDSV_ADDR >= ISR_STACK_TOP);
            assert!(CTR_PENDSV_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(CTR_SYSTICK_ADDR >= ISR_STACK_TOP);
            assert!(CTR_SYSTICK_ADDR < ISR_STACK_TOP + 0x1000);
            // V2 §3.4 readback cells — same constraint.
            assert!(ISER_READBACK_ADDR >= ISR_STACK_TOP);
            assert!(ISER_READBACK_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(ISPR_READBACK_ADDR >= ISR_STACK_TOP);
            assert!(ISPR_READBACK_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(ICER_READBACK_ADDR >= ISR_STACK_TOP);
            assert!(ICER_READBACK_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(ICPR_READBACK_ADDR >= ISR_STACK_TOP);
            assert!(ICPR_READBACK_ADDR < ISR_STACK_TOP + 0x1000);
            // V2 §3.2 gate cells — same constraint.
            assert!(GATE_ADDR >= ISR_STACK_TOP);
            assert!(GATE_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(GATE_AT_ENTRY_ADDR >= ISR_STACK_TOP);
            assert!(GATE_AT_ENTRY_ADDR < ISR_STACK_TOP + 0x1000);
            // V2 §3.3 phase cells — same constraint.
            assert!(PHASE_ADDR >= ISR_STACK_TOP);
            assert!(PHASE_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(PHASE_AT_ENTRY_ADDR >= ISR_STACK_TOP);
            assert!(PHASE_AT_ENTRY_ADDR < ISR_STACK_TOP + 0x1000);
            // And clear of the mailbox words themselves.
            assert!(CTR_TIMER_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(CTR_PENDSV_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(CTR_SYSTICK_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(ISER_READBACK_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(ISPR_READBACK_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(ICER_READBACK_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(ICPR_READBACK_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(GATE_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(GATE_AT_ENTRY_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(PHASE_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(PHASE_AT_ENTRY_ADDR < ISR_MAILBOX_CYCCNT);
            // No collision with the existing CTR cells.
            assert!(ISER_READBACK_ADDR != CTR_TIMER_ADDR);
            assert!(ISPR_READBACK_ADDR != CTR_TIMER_ADDR);
            assert!(ICER_READBACK_ADDR != CTR_TIMER_ADDR);
            assert!(ICPR_READBACK_ADDR != CTR_TIMER_ADDR);
            assert!(GATE_ADDR != CTR_TIMER_ADDR);
            assert!(GATE_ADDR != CTR_PENDSV_ADDR);
            assert!(GATE_ADDR != CTR_SYSTICK_ADDR);
            assert!(GATE_AT_ENTRY_ADDR != GATE_ADDR);
            // V2 §3.3 — PHASE cells distinct from every other slot.
            assert!(PHASE_ADDR != PHASE_AT_ENTRY_ADDR);
            assert!(PHASE_ADDR != ISER_READBACK_ADDR);
            assert!(PHASE_ADDR != ISPR_READBACK_ADDR);
            assert!(PHASE_ADDR != ICER_READBACK_ADDR);
            assert!(PHASE_ADDR != ICPR_READBACK_ADDR);
            assert!(PHASE_ADDR != CTR_TIMER_ADDR);
            assert!(PHASE_ADDR != CTR_PENDSV_ADDR);
            assert!(PHASE_ADDR != CTR_SYSTICK_ADDR);
            assert!(PHASE_ADDR != GATE_ADDR);
            assert!(PHASE_ADDR != GATE_AT_ENTRY_ADDR);
            // V2 §3.1 — priority-preempt cells inside the mailbox page,
            // clear of the mailbox words, and pairwise-distinct from every
            // other observable cell.
            assert!(CTR_IRQ_0_ADDR >= ISR_STACK_TOP);
            assert!(CTR_IRQ_0_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(CTR_IRQ_1_ADDR >= ISR_STACK_TOP);
            assert!(CTR_IRQ_1_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(ORDER_FIRST_IRQ_ADDR >= ISR_STACK_TOP);
            assert!(ORDER_FIRST_IRQ_ADDR < ISR_STACK_TOP + 0x1000);
            assert!(CTR_IRQ_0_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(CTR_IRQ_1_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(ORDER_FIRST_IRQ_ADDR < ISR_MAILBOX_CYCCNT);
            assert!(CTR_IRQ_0_ADDR != CTR_IRQ_1_ADDR);
            assert!(CTR_IRQ_0_ADDR != ORDER_FIRST_IRQ_ADDR);
            assert!(CTR_IRQ_1_ADDR != ORDER_FIRST_IRQ_ADDR);
            assert!(CTR_IRQ_0_ADDR != CTR_TIMER_ADDR);
            assert!(CTR_IRQ_0_ADDR != CTR_PENDSV_ADDR);
            assert!(CTR_IRQ_0_ADDR != CTR_SYSTICK_ADDR);
            assert!(CTR_IRQ_0_ADDR != PHASE_ADDR);
            assert!(CTR_IRQ_0_ADDR != PHASE_AT_ENTRY_ADDR);
            assert!(CTR_IRQ_0_ADDR != GATE_ADDR);
            assert!(CTR_IRQ_0_ADDR != GATE_AT_ENTRY_ADDR);
            assert!(CTR_IRQ_1_ADDR != CTR_TIMER_ADDR);
            assert!(ORDER_FIRST_IRQ_ADDR != CTR_TIMER_ADDR);
            assert!(ORDER_FIRST_IRQ_ADDR != PHASE_ADDR);
        };
    }

    /// Decode the T1 `bne` in HANDLER_TAIL and assert the branch target
    /// resolves to hw[6] (the SysTick-path `ldr r0`). This guards against
    /// off-by-one regressions like the earlier 0xD102 (which landed on
    /// hw[7] = nop and let the SysTick path fall through with an
    /// uninitialised r0 into the common increment).
    #[test]
    fn handler_tail_bne_target_is_hw6() {
        let bne = HANDLER_TAIL[3];
        // T1 bne encoding: 0b1101_0001_IIIIIIII. Verify upper byte.
        assert_eq!(bne & 0xFF00, 0xD100, "hw[3] is not a T1 bne");
        // imm8 is signed 8-bit, scaled by 2, added to PC = branch_addr + 4.
        let imm8 = (bne & 0xFF) as i8 as i32;
        // hw[3] is at byte offset HANDLER_OFFSET + 3*2.
        let branch_byte = (HANDLER_OFFSET as i32) + 3 * 2;
        let pc = branch_byte + 4;
        let target = pc + imm8 * 2;
        let hw6_byte = (HANDLER_OFFSET as i32) + 6 * 2;
        assert_eq!(
            target, hw6_byte,
            "bne target byte 0x{:03X} != hw[6] byte 0x{:03X} (imm8={})",
            target, hw6_byte, imm8,
        );
    }
}
