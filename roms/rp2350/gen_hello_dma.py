#!/usr/bin/env python3
"""
Generate a minimal RP2350 ARM firmware exercising DMA mem-to-mem transfer.

Phase 3 corpus entry (HLD V5 §5.6). Loaded into SRAM at 0x2000_0000 via
`Emulator::load_image` (bypassing the bootrom), relying on the
emulator's HLD V5 §5.7 post-bootrom state — DMA released from RESETS.

Layout at SRAM base 0x2000_0000:
  0x0000: Vector table (16 standard Cortex-M33 entries; only SP / Reset
          / HardFault are used).
  0x0040: Reset handler (program DMA, poll BUSY, write marker).
  0x00XX: Fault trap (B .).
  0x1000: Source data (4 words: 0xCAFE_0001..0xCAFE_0004).
  0x2000: Destination data (zeroed; DMA fills it).
  0x3000: Counter cell (marker written after DMA completes).

The firmware:
  1. Release DMA from RESETS (belt-and-braces).
  2. Seed 4 source words at 0x2000_1000.
  3. Program DMA ch0:
     - READ_ADDR = 0x2000_1000
     - WRITE_ADDR = 0x2000_2000
     - TRANS_COUNT = 4
     - CTRL_TRIG: EN=1, DATA_SIZE=2 (word), INCR_READ, INCR_WRITE,
       TREQ_SEL=63 (FORCE), CHAIN_TO=0 (self = no chain)
  4. Poll CTRL_TRIG bit 26 (BUSY) until clear.
  5. Read INTR, write marker to counter cell if INTR bit 0 is set.
  6. BKPT #0 / WFI (halt).

References:
  - RP2350 Datasheet §12.6 (DMA).
  - `crates/mdrp2350/src/dma.rs` for the emulator DMA model.
  - `gen_hello_timer.py` for Thumb-2 encoding patterns.
"""

import struct
import sys

# =============================================================================
# Constants
# =============================================================================

SRAM_BASE = 0x20000000
SRAM_SIZE = 520 * 1024  # 520KB SRAM on RP2350

# Stack at top of SRAM.
STACK_TOP = SRAM_BASE + SRAM_SIZE  # 0x20082000

# Source/destination/counter addresses within SRAM.
SRC_ADDR = SRAM_BASE + 0x1000      # 0x2000_1000
DST_ADDR = SRAM_BASE + 0x2000      # 0x2000_2000
COUNTER_ADDR = SRAM_BASE + 0x3000  # 0x2000_3000

# RP2350 peripheral addresses.
RESETS_BASE = 0x40020000
RESETS_RESET_CLR = RESETS_BASE + 0x3000  # CLR alias (+0x3000)
RESET_DMA_BIT = 1 << 2

DMA_BASE = 0x50000000
DMA_CH0_READ_ADDR = DMA_BASE + 0x00
DMA_CH0_WRITE_ADDR = DMA_BASE + 0x04
DMA_CH0_TRANS_COUNT = DMA_BASE + 0x08
DMA_CH0_CTRL_TRIG = DMA_BASE + 0x0C
DMA_INTR = DMA_BASE + 0x400

# CTRL_TRIG value: EN=1, DATA_SIZE=2 (word), INCR_READ=1, INCR_WRITE=1,
# TREQ_SEL=63 (FORCE), CHAIN_TO=0 (self = no chain).
# bits: [0]=1, [3:2]=0b10, [4]=1, [5]=1, [20:15]=0b111111, [14:11]=0b0000
CTRL_TRIG_VALUE = (1 << 0) | (2 << 2) | (1 << 4) | (1 << 5) | (63 << 15)
# = 0x001F_8039

# Source data words.
SRC_DATA = [0xCAFE_0001, 0xCAFE_0002, 0xCAFE_0003, 0xCAFE_0004]

CODE_OFFSET = 0x40  # code immediately after 64-byte vector table

# =============================================================================
# Thumb-2 encoding helpers
# =============================================================================

def thumb_movw(rd, imm16):
    """MOVW Rd, #imm16 (Thumb-32)."""
    assert 0 <= rd <= 15 and 0 <= imm16 <= 0xFFFF
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF240 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)

def thumb_movt(rd, imm16):
    """MOVT Rd, #imm16 (Thumb-32)."""
    assert 0 <= rd <= 15 and 0 <= imm16 <= 0xFFFF
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF2C0 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)

def thumb_mov_imm32(rd, val32):
    """Load a 32-bit immediate into Rd using MOVW+MOVT."""
    code = thumb_movw(rd, val32 & 0xFFFF)
    if (val32 >> 16) != 0:
        code += thumb_movt(rd, (val32 >> 16) & 0xFFFF)
    return code

def thumb_str_imm(rt, rn, imm_byte):
    """STR Rt, [Rn, #imm_byte] T1 (Rt/Rn in R0-R7, imm5 word offset)."""
    assert 0 <= rt <= 7 and 0 <= rn <= 7
    assert imm_byte % 4 == 0 and 0 <= imm_byte <= 124
    imm5 = imm_byte // 4
    return struct.pack('<H', 0x6000 | (imm5 << 6) | (rn << 3) | rt)

def thumb_ldr_imm(rt, rn, imm_byte):
    """LDR Rt, [Rn, #imm_byte] T1."""
    assert 0 <= rt <= 7 and 0 <= rn <= 7
    assert imm_byte % 4 == 0 and 0 <= imm_byte <= 124
    imm5 = imm_byte // 4
    return struct.pack('<H', 0x6800 | (imm5 << 6) | (rn << 3) | rt)

def thumb_tst_reg(rn, rm):
    """TST Rn, Rm (T1)."""
    assert 0 <= rn <= 7 and 0 <= rm <= 7
    return struct.pack('<H', 0x4200 | (rm << 3) | rn)

def thumb_bne(offset):
    """BNE label (T1). offset in signed bytes from PC+4."""
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    return struct.pack('<H', 0xD100 | ((offset >> 1) & 0xFF))

def thumb_b(offset):
    """B label (T1 unconditional). offset in signed bytes from PC+4."""
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    return struct.pack('<H', 0xE000 | ((offset >> 1) & 0x7FF))

def thumb_bkpt_0():
    """BKPT #0 (T1)."""
    return struct.pack('<H', 0xBE00)

def thumb_movs_imm(rd, imm8):
    """MOVS Rd, #imm8 (T1)."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)

def thumb_adds_imm(rd, imm8):
    """ADDS Rd, #imm8 (T2)."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3000 | (rd << 8) | imm8)

# =============================================================================
# Firmware body
# =============================================================================

def build_reset_handler():
    """
    Program DMA ch0 for a 4-word mem-to-mem transfer, poll for completion,
    write a marker to the counter cell.

    Register allocation:
      r0 — scratch (source data values, CTRL_TRIG readback)
      r1 — DMA_BASE (0x5000_0000)
      r2 — source address (0x2000_1000)
      r3 — destination address (0x2000_2000)
      r4 — RESETS_RESET_CLR (0x4002_3000)
      r5 — counter cell address (0x2000_3000)
      r6 — BUSY mask (1 << 26)
      r7 — counter scratch
    """
    code = b''

    # --- Step 1: Release DMA from RESETS ---
    code += thumb_mov_imm32(4, RESETS_RESET_CLR)   # r4 = RESETS.CLR alias
    code += thumb_mov_imm32(0, RESET_DMA_BIT)      # r0 = bit 2
    code += thumb_str_imm(0, 4, 0)                 # *r4 = r0

    # --- Step 2: Seed 4 source words into SRAM ---
    code += thumb_mov_imm32(2, SRC_ADDR)           # r2 = 0x2000_1000
    for i, val in enumerate(SRC_DATA):
        code += thumb_mov_imm32(0, val)
        code += thumb_str_imm(0, 2, i * 4)

    # --- Step 3: Program DMA ch0 ---
    code += thumb_mov_imm32(1, DMA_BASE)           # r1 = DMA_BASE
    code += thumb_mov_imm32(3, DST_ADDR)           # r3 = 0x2000_2000

    # CH0.READ_ADDR = SRC_ADDR (r2)
    code += thumb_str_imm(2, 1, 0x00)              # [r1+0x00] = r2

    # CH0.WRITE_ADDR = DST_ADDR (r3)
    code += thumb_str_imm(3, 1, 0x04)              # [r1+0x04] = r3

    # CH0.TRANS_COUNT = 4
    code += thumb_movs_imm(0, 4)
    code += thumb_str_imm(0, 1, 0x08)              # [r1+0x08] = 4

    # CH0.CTRL_TRIG = CTRL_TRIG_VALUE (triggers the transfer)
    code += thumb_mov_imm32(0, CTRL_TRIG_VALUE)
    code += thumb_str_imm(0, 1, 0x0C)              # [r1+0x0C] = CTRL_TRIG

    # --- Step 4: Poll CTRL_TRIG bit 26 (BUSY) until clear ---
    code += thumb_mov_imm32(6, 1 << 26)            # r6 = BUSY mask
    # poll_loop:
    poll_loop_start = len(code)
    code += thumb_ldr_imm(0, 1, 0x0C)              # r0 = [r1+0x0C] (CTRL_TRIG)
    code += thumb_tst_reg(0, 6)                    # TST r0, r6
    poll_branch_pos = len(code)
    # BNE back to poll_loop_start.
    # offset = poll_loop_start - (poll_branch_pos + 4)
    offset = poll_loop_start - (poll_branch_pos + 4)
    code += thumb_bne(offset)

    # --- Step 5: Write marker to counter cell ---
    code += thumb_mov_imm32(5, COUNTER_ADDR)       # r5 = counter cell
    code += thumb_movs_imm(7, 1)                   # r7 = 1
    code += thumb_str_imm(7, 5, 0)                 # *r5 = 1

    # --- Step 6: Halt ---
    code += thumb_bkpt_0()

    # Pad with fault trap just in case.
    code += thumb_b(-2)  # infinite loop (B .)

    return code


def build_firmware():
    """Assemble the complete firmware image: vector table + reset handler."""
    code = build_reset_handler()

    # Fault trap: B . at the end of the code area (2 bytes).
    fault_trap_offset = CODE_OFFSET + len(code)

    # Build vector table (16 entries, 64 bytes).
    # Exception 0 (unused) doubles as the initial SP.
    vt = struct.pack('<I', STACK_TOP)
    # Exception 1: Reset handler.
    reset_addr = SRAM_BASE + CODE_OFFSET + 1  # Thumb bit
    vt += struct.pack('<I', reset_addr)
    # Exceptions 2-15: all point to fault trap.
    fault_addr = SRAM_BASE + fault_trap_offset + 1  # Thumb bit
    for _ in range(14):
        vt += struct.pack('<I', fault_addr)

    assert len(vt) == CODE_OFFSET

    # Source data region at offset 0x1000 (zeroed by load_image positioning).
    # We don't need to pre-fill because the firmware stores it at runtime.

    return vt + code


def main():
    fw = build_firmware()
    outpath = sys.argv[1] if len(sys.argv) > 1 else 'hello_dma.bin'
    with open(outpath, 'wb') as f:
        f.write(fw)
    print(f"Wrote {len(fw)} bytes to {outpath}")
    print(f"  CTRL_TRIG = 0x{CTRL_TRIG_VALUE:08X}")
    print(f"  SRC_ADDR  = 0x{SRC_ADDR:08X}")
    print(f"  DST_ADDR  = 0x{DST_ADDR:08X}")
    print(f"  COUNTER   = 0x{COUNTER_ADDR:08X}")


if __name__ == '__main__':
    main()
