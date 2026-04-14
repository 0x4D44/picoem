#!/usr/bin/env python3
"""
Generate an RP2350 ARM benchmark firmware for the mdrp2350app showcase app.

The firmware runs six timed sections, each exercising a different M33
instruction class. Before and after each section it writes a phase
sentinel to SRAM address 0x2003_FF00 so the host (sim thread) can
measure the emulator cycle count at the transitions and compute
per-section deltas (see LLD §8).

Phase values:
    0x00    Not yet started (initial SRAM content)
    0x1N    Section N started (N = 1..=6)
    0x2N    Section N done
    0xFF    All sections complete, firmware spins forever

Sections (LLD §8.3):
    1. arith_add   — ADDS Rd, Rd, #1                 — 1,000,000 iters
    2. arith_mul   — MUL  Rd, Rm                      — 1,000,000 iters
    3. arith_sdiv  — SDIV Rd, Rn, Rm                  —   100,000 iters
    4. mem_seq_ld  — LDR  Rd, [Rn, #0] (fixed ptr)    —   500,000 iters
    5. bit_clz     — CLZ  Rd, Rm                      — 1,000,000 iters
    6. ldm_8regs   — LDMIA Rn!, {r0-r7} (reset Rn)    —    50,000 iters

The loop body is always the instruction under test + SUBS counter + BNE,
so each section reports "triplet cycles per iteration" rather than the
instruction's cost in isolation (LLD §8.4). The Δ column in the bench
panel still surfaces fidelity.

Notes:
  * Section 4 (mem_seq_ld) uses a fixed LDR with immediate offset rather
    than a post-indexed LDR with pointer advance. A post-indexed LDR
    would still need to reset the pointer periodically to avoid running
    off the SRAM region; a fixed load avoids that without affecting
    what the bench is actually measuring (LDR.W timing). The buffer
    word lives at 0x2005_0000 (well above the stack at 0x2003_FF00).
  * Section 6 caches the LDM base in r9 and restores r8 each iteration
    with a low-to-high MOV so the LDMIA always hits the same 32 bytes.

References:
  - roms/rp2350/gen_blinky.py / roms/rp2350/gen_lcd_demo.py (builder template)
  - wrk_docs/2026.04.13 - LLD - Emulator Showcase App.md §8, §10.2
  - crates/mdrp2350/src/tests.rs (encoder reference for Thumb-32
    LDMIA.W, MUL.W, SDIV, CLZ, LDR.W imm12)
"""

import struct
import sys

# =============================================================================
# Constants
# =============================================================================

FLASH_BASE = 0x10000000
SRAM_BASE = 0x20000000
SRAM_SIZE = 520 * 1024
STACK_TOP = SRAM_BASE + SRAM_SIZE  # 0x20082000

# Phase sentinel lives near the top of SRAM, safely above stack/heap.
PHASE_ADDR = 0x2003FF00

# LDR buffer for section 4 — must be in SRAM, well away from the stack
# and the phase sentinel. 0x2005_0000 is inside SRAM (which runs to
# 0x2008_2000) but nowhere near anything else we touch.
LDR_BUF_ADDR = 0x20050000

# Scratch buffer for the LDM section — needs to hold 8 words (32 bytes)
# at a fixed address; the firmware resets the base register before each
# LDMIA iteration so we never run off the end.
LDM_BUF_ADDR = 0x20050040

# Picobin IMAGE_DEF metadata (copied verbatim from gen_blinky.py)
PICOBIN_BLOCK_MARKER_START = 0xFFFFDED3
PICOBIN_BLOCK_MARKER_END = 0xAB123579
PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE = 0x42
PICOBIN_BLOCK_ITEM_2BS_LAST = 0xFF
IMAGE_TYPE_VALUE = 0x0001 | (0x2 << 4) | (0x0 << 8) | (0x1 << 12)  # 0x1021

# Iteration counts per section (LLD §8.3).
ITER_ADD = 1_000_000
ITER_MUL = 1_000_000
ITER_SDIV = 100_000
ITER_LDR = 500_000
ITER_CLZ = 1_000_000
ITER_LDM = 50_000


# =============================================================================
# Thumb-2 instruction encoding helpers
# =============================================================================
# The 16-bit and simple 32-bit encoders are copied verbatim from
# gen_lcd_demo.py. The specialised 32-bit encoders (MUL.W, SDIV, CLZ,
# LDMIA.W, LDR.W imm12) match crates/mdrp2350/src/tests.rs so their
# behaviour is already covered by the upstream unit tests.

def thumb_movw(rd, imm16):
    assert 0 <= rd <= 15 and 0 <= imm16 <= 0xFFFF
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF240 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)


def thumb_movt(rd, imm16):
    assert 0 <= rd <= 15 and 0 <= imm16 <= 0xFFFF
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF2C0 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)


def thumb_mov_imm32(rd, val32):
    """Load a 32-bit immediate into Rd using MOVW+MOVT (always both halves)."""
    lo = val32 & 0xFFFF
    hi = (val32 >> 16) & 0xFFFF
    code = thumb_movw(rd, lo)
    if hi != 0:
        code += thumb_movt(rd, hi)
    return code


def thumb_str_imm12(rt, rn, imm12):
    """STR.W Rt, [Rn, #imm12] (Thumb-32, unsigned 12-bit offset)."""
    assert 0 <= rt <= 15 and 0 <= rn <= 15 and 0 <= imm12 <= 4095
    hw1 = 0xF8C0 | rn
    hw2 = (rt << 12) | imm12
    return struct.pack('<HH', hw1, hw2)


def thumb_ldr_imm12(rt, rn, imm12):
    """LDR.W Rt, [Rn, #imm12] (Thumb-32, unsigned 12-bit offset)."""
    assert 0 <= rt <= 15 and 0 <= rn <= 15 and 0 <= imm12 <= 4095
    hw1 = 0xF8D0 | rn
    hw2 = (rt << 12) | imm12
    return struct.pack('<HH', hw1, hw2)


def thumb_movs_imm8(rd, imm8):
    """MOVS Rd, #imm8 (Thumb-16, Rd must be R0-R7)."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)


def thumb_subs_imm8(rd, imm8):
    """SUBS Rd, #imm8 (Thumb-16, Rd must be R0-R7)."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3800 | (rd << 8) | imm8)


def thumb_mov_high(rd, rm):
    """MOV Rd, Rm (Thumb-16 high-register form; supports R0-R15)."""
    assert 0 <= rd <= 15 and 0 <= rm <= 15
    dn = (rd >> 3) & 1
    rd_low = rd & 0x7
    return struct.pack('<H', 0x4600 | (dn << 7) | (rm << 3) | rd_low)


def thumb_bne(offset):
    """BNE label (Thumb-16). offset is signed byte offset from PC+4."""
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD100 | imm8)


def thumb_b(offset):
    """Unconditional B label (Thumb-16). offset from PC+4, -2048..+2046."""
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)


def thumb_b_w(offset):
    """
    Unconditional B.W (T4). 25-bit signed byte offset, must be even.
    Encoding matches crates/mdrp2350/src/tests.rs::encode_b_w_uncond
    and roms/rp2350/gen_lcd_demo.py::thumb_b_w.
    """
    assert offset % 2 == 0 and -(1 << 24) <= offset < (1 << 24)
    uoffset = offset & 0xFFFFFFFF
    s = (uoffset >> 24) & 1
    i1 = (uoffset >> 23) & 1
    i2 = (uoffset >> 22) & 1
    imm10 = (uoffset >> 12) & 0x3FF
    imm11 = (uoffset >> 1) & 0x7FF
    j1 = (i1 ^ s) ^ 1
    j2 = (i2 ^ s) ^ 1
    hw0 = 0xF000 | (s << 10) | imm10
    hw1 = 0x9000 | (j1 << 13) | (j2 << 11) | imm11
    return struct.pack('<HH', hw0, hw1)


# -----------------------------------------------------------------------------
# Thumb-32 encoders for the instructions under test. Opcode layouts match
# crates/mdrp2350/src/tests.rs, which has unit tests for each of these.
# -----------------------------------------------------------------------------

def thumb_mul_w(rd, rn, rm):
    """
    MUL.W Rd, Rn, Rm — 32-bit multiply (Thumb-32).
    Encoding: 1111_1011_0000_Rn | 1111_Rd_0000_Rm  (Ra=0xF, op1=000, op2=00).
    Matches crates/mdrp2350/src/tests.rs::encode_mul_w.
    """
    assert 0 <= rd <= 15 and 0 <= rn <= 15 and 0 <= rm <= 15
    hw0 = 0xFB00 | rn
    hw1 = 0xF000 | (rd << 8) | rm
    return struct.pack('<HH', hw0, hw1)


def thumb_sdiv(rd, rn, rm):
    """
    SDIV Rd, Rn, Rm — signed integer divide (Thumb-32).
    Encoding: 1111_1011_1001_Rn | 1111_Rd_1111_Rm  (op1=001, op2=1111).
    Matches crates/mdrp2350/src/tests.rs::encode_sdiv.
    """
    assert 0 <= rd <= 15 and 0 <= rn <= 15 and 0 <= rm <= 15
    hw0 = 0xFB90 | rn
    hw1 = 0xF000 | (rd << 8) | 0x00F0 | rm
    return struct.pack('<HH', hw0, hw1)


def thumb_clz(rd, rm):
    """
    CLZ Rd, Rm — count leading zeros (Thumb-32).
    Encoding: 1111_1010_1011_Rm | 1111_Rd_1000_Rm.
    Matches crates/mdrp2350/src/tests.rs::clz_w (which shows the raw
    halfwords 0xFAB1 / 0xF081 for CLZ R0, R1).
    """
    assert 0 <= rd <= 15 and 0 <= rm <= 15
    hw0 = 0xFAB0 | rm
    hw1 = 0xF080 | (rd << 8) | rm
    return struct.pack('<HH', hw0, hw1)


def thumb_ldmia_w(rn, writeback, reglist):
    """
    LDMIA.W Rn{!}, <reglist> (Thumb-32).
    Encoding: 1110_1000_10W1_Rn | P_M_0_reglist[12:0], where P is PC in
    the register list, M is LR. Matches crates/mdrp2350/src/tests.rs::
    encode_ldmia_w — hw0 = 0xE890 | (W<<5) | Rn, hw1 = reglist.
    """
    assert 0 <= rn <= 15 and 0 <= reglist <= 0xFFFF
    hw0 = 0xE890 | ((1 << 5) if writeback else 0) | rn
    hw1 = reglist
    return struct.pack('<HH', hw0, hw1)


# =============================================================================
# Section builders
# =============================================================================

def emit_write_phase(phase):
    """
    Write `phase` (a u32) to the sentinel address 0x2003_FF00 using R10/R11
    as scratch. R10 holds the sentinel address across all sections so we
    don't reload it; this keeps the inter-section setup cost out of the
    measurement window (it's only the STR that bumps the counter, which
    is between the two phase writes).

    Precondition: R10 already contains PHASE_ADDR and R11 is free to clobber.
    """
    # R11 = phase, STR.W R11, [R10, #0]
    code = thumb_mov_imm32(11, phase)
    code += thumb_str_imm12(11, 10, 0)
    return code


def emit_counter_load(rn, iters):
    """
    MOVW/MOVT Rn, #iters. Rn must be a general-purpose register; when
    Rn is a low register (R0-R7) this is a 4-byte MOVW + (optional) MOVT.
    """
    return thumb_mov_imm32(rn, iters)


def emit_loop_back(code, loop_top):
    """
    Append a BNE (Thumb-16) that branches back to `loop_top` from the
    current end of `code`. Returns the new `code` with the BNE appended.
    The BNE is at offset len(code), and PC during execution is
    (len(code) + 4). Offset must be in -256..+254 and even.
    """
    bne_pos = len(code)
    bne_offset = loop_top - (bne_pos + 4)
    return code + thumb_bne(bne_offset)


def build_section_1_arith_add():
    """
    Section 1: arith_add — ADDS Rd, Rd, #1 under a decrement-and-branch loop.
        phase = 0x11
        movw r2, #<iters-lo>; movt r2, #<iters-hi>
        movs r3, #0
      .L:
        adds r3, r3, #1       ; instruction under test
        subs r2, #1
        bne  .L
        phase = 0x21
    """
    code = b''
    code += emit_write_phase(0x11)
    code += thumb_mov_imm32(2, ITER_ADD)
    code += thumb_movs_imm8(3, 0)
    loop_top = len(code)
    # ADDS R3, R3, #1 — Thumb-16 ADDS Rd, Rn, #imm3: 0001110_imm3_Rn_Rd
    code += struct.pack('<H', 0x1C00 | (1 << 6) | (3 << 3) | 3)
    code += thumb_subs_imm8(2, 1)
    code = emit_loop_back(code, loop_top)
    code += emit_write_phase(0x21)
    return code


def build_section_2_arith_mul():
    """
    Section 2: arith_mul — MUL.W R3, R3, R4 with a constant multiplier.
        phase = 0x12
        movw r2, #<iters>
        movs r3, #1
        movs r4, #1              ; multiplier — 1 to avoid runaway growth
      .L:
        mul  r3, r3, r4          ; instruction under test (Thumb-32)
        subs r2, #1
        bne  .L
        phase = 0x22
    """
    code = b''
    code += emit_write_phase(0x12)
    code += thumb_mov_imm32(2, ITER_MUL)
    code += thumb_movs_imm8(3, 1)
    code += thumb_movs_imm8(4, 1)
    loop_top = len(code)
    code += thumb_mul_w(3, 3, 4)
    code += thumb_subs_imm8(2, 1)
    code = emit_loop_back(code, loop_top)
    code += emit_write_phase(0x22)
    return code


def build_section_3_arith_sdiv():
    """
    Section 3: arith_sdiv — SDIV R3, R5, R6 with constants that keep the
    dividend's significant-bit count steady (so every iteration has the
    same cycle cost).
        phase = 0x13
        movw r2, #<iters>
        movw r5, #12345          ; dividend
        movs r6, #7              ; divisor
      .L:
        sdiv r3, r5, r6          ; instruction under test
        subs r2, #1
        bne  .L
        phase = 0x23
    """
    code = b''
    code += emit_write_phase(0x13)
    code += thumb_mov_imm32(2, ITER_SDIV)
    code += thumb_mov_imm32(5, 12345)
    code += thumb_movs_imm8(6, 7)
    loop_top = len(code)
    code += thumb_sdiv(3, 5, 6)
    code += thumb_subs_imm8(2, 1)
    code = emit_loop_back(code, loop_top)
    code += emit_write_phase(0x23)
    return code


def build_section_4_mem_seq_ld():
    """
    Section 4: mem_seq_ld — LDR.W R0, [R1, #0] with R1 fixed at LDR_BUF_ADDR.
        phase = 0x14
        movw r2, #<iters>
        movw r1, #<buf-lo>; movt r1, #<buf-hi>
      .L:
        ldr  r0, [r1, #0]         ; instruction under test
        subs r2, #1
        bne  .L
        phase = 0x24

    The LLD §8.3 originally envisioned a post-indexed LDR that auto-
    advances the pointer. That would eventually run off SRAM and would
    need the pointer reset anyway; a fixed base gives the same timing
    signal (LDR.W immediate is what the M33 does for single-word loads)
    without the bookkeeping. The deviation is documented in the module
    header.
    """
    code = b''
    code += emit_write_phase(0x14)
    code += thumb_mov_imm32(2, ITER_LDR)
    code += thumb_mov_imm32(1, LDR_BUF_ADDR)
    loop_top = len(code)
    code += thumb_ldr_imm12(0, 1, 0)
    code += thumb_subs_imm8(2, 1)
    code = emit_loop_back(code, loop_top)
    code += emit_write_phase(0x24)
    return code


def build_section_5_bit_clz():
    """
    Section 5: bit_clz — CLZ R3, R4 with a constant input.
        phase = 0x15
        movw r2, #<iters>
        movw r4, #0x1000         ; bit 12 set → CLZ result = 19
      .L:
        clz  r3, r4              ; instruction under test
        subs r2, #1
        bne  .L
        phase = 0x25
    """
    code = b''
    code += emit_write_phase(0x15)
    code += thumb_mov_imm32(2, ITER_CLZ)
    code += thumb_mov_imm32(4, 0x1000)
    loop_top = len(code)
    code += thumb_clz(3, 4)
    code += thumb_subs_imm8(2, 1)
    code = emit_loop_back(code, loop_top)
    code += emit_write_phase(0x25)
    return code


def build_section_6_ldm_8regs():
    """
    Section 6: ldm_8regs — LDMIA.W R8!, {R0-R7} over a fixed 32-byte buffer.

    To avoid running the base pointer off the buffer, R9 caches the
    constant buffer address and R8 is reset from R9 at the top of every
    iteration. The MOV r8, r9 is a Thumb-16 high-register move (1 cycle).

        phase = 0x16
        movw r2, #<iters>
        movw r9, #<buf-lo>; movt r9, #<buf-hi>   ; cached buffer address
      .L:
        mov  r8, r9                               ; reset base
        ldmia r8!, {r0-r7}                        ; instruction under test
        subs r2, #1
        bne  .L
        phase = 0x26

    Note the dual purpose of r2 as the outer iteration counter: because
    LDMIA destroys r0-r7 we cannot use a low register for the counter
    without also loading it from the buffer. R2 is one of the registers
    loaded by the LDM, so the LDM value has to be benign — we rely on
    buffer[2] being whatever we put there at init time (see
    `build_buffer_init` below).

    A cleaner alternative would be a high-register counter, but that
    requires a high-register SUBS (Thumb-32) and a long BNE. The chosen
    approach keeps the loop small: r2 is reloaded by LDMIA but we
    subtract `ITER_LDM` directly from it via pre-initialising buffer[2]
    to the target counter minus one, then comparing against 0. That's
    fiddly. Simpler: use a disjoint counter register, r11.

    Revised pattern (implemented below):
        phase = 0x16
        movw r11, #<iters-lo>; movt r11, #<iters-hi>
        movw r9,  #<buf-lo>;  movt r9,  #<buf-hi>
      .L:
        mov   r8, r9
        ldmia r8!, {r0-r7}            ; loads into r0-r7 (discarded)
        subs  r11, r11, #1            ; Thumb-32 SUBS imm on high reg
        bne.w .L                       ; or short BNE if in range
        phase = 0x26

    SUBS R11, R11, #1 is a Thumb-32 encoding. Simpler: decrement R2 (low)
    externally and rebuild each iter via a fresh MOV. Actually the
    simplest thing is: use R12 (IP) for the counter — SUBS imm on R12
    is also Thumb-32. To avoid another Thumb-32 encoder, we can instead
    pick a low register that the LDMIA does NOT load. But LDMIA {r0-r7}
    loads all eight low registers.

    Final design: keep the loop counter in R11, decrement it with a
    Thumb-32 SUBS.W, and branch with a Thumb-16 BNE. We encode SUBS.W
    inline as a data-processing-modified-immediate instruction (T3 of
    SUBS). Counter width is well under 16 bits → fits in imm12.
    """
    code = b''
    code += emit_write_phase(0x16)

    # R11 = iteration counter. SUBS.W R11, R11, #1 is T3 encoding of SUB.
    code += thumb_mov_imm32(11, ITER_LDM)
    # R9 = LDM buffer base (cached).
    code += thumb_mov_imm32(9, LDM_BUF_ADDR)

    loop_top = len(code)
    # MOV R8, R9 (Thumb-16 high-register move).
    code += thumb_mov_high(8, 9)
    # LDMIA.W R8!, {R0-R7}  — reglist bits 0..7 = 0x00FF.
    code += thumb_ldmia_w(8, writeback=True, reglist=0x00FF)
    # SUBS.W R11, R11, #1 — T3 data-processing modified immediate, S=1,
    # op=0b1101 (SUB), Rn=11, Rd=11, i=0, imm3=0, imm8=1 → imm12=1.
    # hw0 = 1111_0_i_01_101_S_Rn  = 0xF1B0 | Rn  (S=1, i=0, op=1101)
    #     = 0xF1B0 | 11 = 0xF1BB
    # hw1 = 0_imm3_Rd_imm8         = 0x0B01 (Rd=11, imm8=1, imm3=0)
    subs_w = struct.pack('<HH', 0xF1BB, 0x0B01)
    code += subs_w
    code = emit_loop_back(code, loop_top)

    code += emit_write_phase(0x26)
    return code


def build_phase_base_setup():
    """
    One-time setup: load R10 with PHASE_ADDR so every `emit_write_phase`
    is a tight 3-word MOVW/MOVT/STR sequence that doesn't rebuild the
    address each time. This runs before section 1 and outside any
    timed window.
    """
    return thumb_mov_imm32(10, PHASE_ADDR)


def build_buffer_init():
    """
    Zero the 32-byte LDM scratch buffer (LDM_BUF_ADDR..LDM_BUF_ADDR+0x20).
    The LDM section doesn't care what the values are — it discards the
    loaded data — but we want deterministic memory contents for cross-
    run reproducibility. One STR per word is enough.
    """
    code = b''
    code += thumb_mov_imm32(0, LDM_BUF_ADDR)
    code += thumb_movs_imm8(1, 0)
    for i in range(8):
        code += thumb_str_imm12(1, 0, i * 4)
    # Also zero the single LDR word at LDR_BUF_ADDR.
    code += thumb_mov_imm32(0, LDR_BUF_ADDR)
    code += thumb_str_imm12(1, 0, 0)
    return code


def build_halt():
    """
    Final state: write phase = 0xFF, then spin forever.
    `b .` is a Thumb-16 unconditional branch with offset -4 (loop to self).
    """
    code = emit_write_phase(0xFF)
    code += thumb_b(-4)
    return code


def emit_inter_section_delay():
    """
    Spin for ~300 cycles between the section-done and next-section-start
    phase writes so that the host poller (which samples once per quantum =
    150 cycles) always observes the 0x2N "done" sentinel before it's
    overwritten by the next section's 0x1(N+1) "start" sentinel.

    Uses R2 as a scratch counter (safe: every section finishes with R2=0
    and we haven't entered the next section's preamble yet). At 3 cycles
    per iteration (SUBS + BNE taken), MOVS R2, #100 gives ~300 cycles
    of dead time — two full quanta, guaranteeing the poller sees the
    transition.
    """
    code = thumb_movs_imm8(2, 100)       # MOVS R2, #100
    loop_top = len(code)
    code += thumb_subs_imm8(2, 1)         # SUBS R2, #1
    code = emit_loop_back(code, loop_top) # BNE  loop_top
    return code


def build_reset_handler():
    """
    Full reset handler: one-time setup, six benchmark sections, halt.
    Each section pair is separated by a short delay so the host poller
    can observe every phase sentinel transition individually.
    """
    code = b''
    code += build_phase_base_setup()
    code += build_buffer_init()
    code += build_section_1_arith_add()
    code += emit_inter_section_delay()
    code += build_section_2_arith_mul()
    code += emit_inter_section_delay()
    code += build_section_3_arith_sdiv()
    code += emit_inter_section_delay()
    code += build_section_4_mem_seq_ld()
    code += emit_inter_section_delay()
    code += build_section_5_bit_clz()
    code += emit_inter_section_delay()
    code += build_section_6_ldm_8regs()
    code += emit_inter_section_delay()
    code += build_halt()
    return code


def build_fault_handler():
    """Infinite loop for exception vectors."""
    return thumb_b(-4)


def build_picobin_block():
    block = b''
    block += struct.pack('<I', PICOBIN_BLOCK_MARKER_START)
    block += struct.pack('<B', PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE)
    block += struct.pack('<B', 0x01)
    block += struct.pack('<H', IMAGE_TYPE_VALUE)
    block += struct.pack('<B', PICOBIN_BLOCK_ITEM_2BS_LAST)
    block += struct.pack('<H', 0x0001)
    block += struct.pack('<B', 0x00)
    block += struct.pack('<I', 0x00000000)
    block += struct.pack('<I', PICOBIN_BLOCK_MARKER_END)
    assert len(block) == 20
    return block


# =============================================================================
# Binary assembly
# =============================================================================

def main():
    code_offset = 0x60
    reset_code = build_reset_handler()
    fault_code = build_fault_handler()

    fault_offset = code_offset + len(reset_code)
    if fault_offset % 2 != 0:
        reset_code += b'\x00'
        fault_offset += 1

    reset_vector = (FLASH_BASE + code_offset) | 1
    fault_vector = (FLASH_BASE + fault_offset) | 1

    vectors = struct.pack(
        '<16I',
        STACK_TOP,
        reset_vector,
        fault_vector, fault_vector, fault_vector, fault_vector,
        fault_vector, fault_vector,
        0, 0, 0,
        fault_vector, fault_vector,
        0,
        fault_vector, fault_vector,
    )
    assert len(vectors) == 64

    picobin_block = build_picobin_block()
    used = len(vectors) + len(picobin_block)
    padding = code_offset - used
    assert padding >= 0

    binary = vectors + picobin_block + (b'\x00' * padding) + reset_code + fault_code

    while len(binary) % 256 != 0:
        binary += b'\x00'

    outpath = sys.argv[1] if len(sys.argv) > 1 else 'benchmark.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  Flash base:      {FLASH_BASE:#010x}")
    print(f"  Reset handler:   {FLASH_BASE + code_offset:#010x}")
    print(f"  Reset code size: {len(reset_code)} bytes")
    print(f"  Phase sentinel:  {PHASE_ADDR:#010x}")
    print(f"  LDR buffer:      {LDR_BUF_ADDR:#010x}")
    print(f"  LDM buffer:      {LDM_BUF_ADDR:#010x}")
    print("  Sections:")
    print(f"    1 arith_add   iters={ITER_ADD}")
    print(f"    2 arith_mul   iters={ITER_MUL}")
    print(f"    3 arith_sdiv  iters={ITER_SDIV}")
    print(f"    4 mem_seq_ld  iters={ITER_LDR}")
    print(f"    5 bit_clz     iters={ITER_CLZ}")
    print(f"    6 ldm_8regs   iters={ITER_LDM}")


if __name__ == '__main__':
    main()
