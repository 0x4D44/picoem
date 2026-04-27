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
// **Expected EMU behaviour.** As of this write, the `mdrp2040` core's
// `step()` does not poll ICSR.PENDSVSET / PENDSTSET between
// instructions, and NVIC ISER/ISPR/ICPR registers are unmodelled — an
// `str` to ICSR latches the bit (ppb.rs handles W1S/W1C correctly) but
// no exception is dispatched. A write to NVIC_ISER / NVIC_ISPR falls
// through harmlessly. Both scenarios therefore FAIL on the EMU side
// until the Phase 1 IRQ plumbing (`Bus::irq_pending` +
// `tick_peripherals` + pending-exception dispatch in `CortexM0Plus::
// step`) lands. This is the same posture the RP2350 oracle takes
// against `tech_debt.md` § "Exception entry/exit not differentially
// validated"; the oracle is the surfacing tool.

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

/// RP2040 TIMER peripheral base.
pub const TIMER_BASE: u32 = 0x4005_4000;
/// TIMER ALARM0 (write to arm a deadline against TIMERAWL).
pub const TIMER_ALARM0_ADDR: u32 = TIMER_BASE + 0x10;
/// TIMER INTR — W1C pending alarm bits (bit 0 = ALARM0).
pub const TIMER_INTR_ADDR: u32 = TIMER_BASE + 0x34;
/// TIMER INTE — interrupt enable mask (bit 0 = ALARM0).
pub const TIMER_INTE_ADDR: u32 = TIMER_BASE + 0x38;

/// TIMER_IRQ_0 (IRQ #0). Matches `mdrp2040::irq::IRQ_TIMER_IRQ_0`.
pub const IRQ_TIMER_IRQ_0: u32 = 0;

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
//   + 0xFE0 — TIMER scenario counter
//   + 0xFE4 — PendSV scenario counter
//   + 0xFE8 — SysTick scenario counter
pub const CTR_TIMER_ADDR: u32 = 0x2000_3FE0;
pub const CTR_PENDSV_ADDR: u32 = 0x2000_3FE4;
pub const CTR_SYSTICK_ADDR: u32 = 0x2000_3FE8;

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

// ---------------------------------------------------------------------------
// Scenario images
// ---------------------------------------------------------------------------

const IMAGE_TIMER_COLD: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_TIMER, MAIN_TIMER);

const IMAGE_TAIL_CHAIN: [u8; ISR_IMAGE_SIZE] =
    build_image_m0plus(ISR_IMAGE_BASE, ISR_STACK_TOP, HANDLER_TAIL, MAIN_TAIL);

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
    /// Wall-clock budget in milliseconds. Exception dispatch isn't
    /// modelled yet on EMU, so this runs out on the EMU side — which
    /// is the expected FAIL signal. Kept short so the oracle completes
    /// quickly even with two FAIL scenarios.
    pub max_millis: u32,
    pub observe: &'static [(&'static str, IsrObservable)],
}

// ---------------------------------------------------------------------------
// Per-scenario init_regs + observables
// ---------------------------------------------------------------------------

const INIT_TIMER_COLD: &[(IsrReg, u32)] = &[(IsrReg::Vtor, ISR_IMAGE_BASE)];
const OBS_TIMER_COLD: &[(&str, IsrObservable)] = &[
    // Primary load-bearing observable: the handler ran exactly once so
    // the counter == 1. On silicon this should PASS; on EMU it FAILS
    // because the core never dispatches the IRQ.
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
    // Clear TIMER.INTR (W1C both alarm flags) and disable INTE.
    core.write_word_32(TIMER_INTR_ADDR as u64, 0xFFFF_FFFF)?;
    core.write_word_32(TIMER_INTE_ADDR as u64, 0)?;
    Ok(())
}

fn reset_scenario_state_emu(emu: &mut mdrp2040::Emulator) {
    emu.poke(CTR_TIMER_ADDR, 0);
    emu.poke(CTR_PENDSV_ADDR, 0);
    emu.poke(CTR_SYSTICK_ADDR, 0);
    emu.mmio_write32(TIMER_INTR_ADDR, 0xFFFF_FFFF);
    emu.mmio_write32(TIMER_INTE_ADDR, 0);
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
        assert_eq!(SCENARIOS.len(), 2, "Phase 1 minimum = 2 scenarios");
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
        assert_eq!(TIMER_INTR_ADDR, 0x4005_4034);
        assert_eq!(TIMER_INTE_ADDR, 0x4005_4038);
        assert_eq!(IRQ_TIMER_IRQ_0, 0);
    }

    #[test]
    fn counter_addresses_inside_mailbox_page() {
        // Counters live in the mailbox page (0x2000_3000..0x2000_4000)
        // so the scenario image and stack never collide with them.
        assert!(CTR_TIMER_ADDR >= ISR_STACK_TOP);
        assert!(CTR_TIMER_ADDR < ISR_STACK_TOP + 0x1000);
        assert!(CTR_PENDSV_ADDR >= ISR_STACK_TOP);
        assert!(CTR_PENDSV_ADDR < ISR_STACK_TOP + 0x1000);
        assert!(CTR_SYSTICK_ADDR >= ISR_STACK_TOP);
        assert!(CTR_SYSTICK_ADDR < ISR_STACK_TOP + 0x1000);
        // And clear of the mailbox words themselves.
        assert!(CTR_TIMER_ADDR < ISR_MAILBOX_CYCCNT);
        assert!(CTR_PENDSV_ADDR < ISR_MAILBOX_CYCCNT);
        assert!(CTR_SYSTICK_ADDR < ISR_MAILBOX_CYCCNT);
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
