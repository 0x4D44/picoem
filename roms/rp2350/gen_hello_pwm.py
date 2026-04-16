#!/usr/bin/env python3
"""
Generate a minimal RP2350 ARM firmware that exercises PWM slice 0 wrap.

Phase 2 corpus entry (HLD V5 §3 / §6 row 2). The firmware:

  1. Configures PWM slice 0: TOP=100, CSR.EN=1.
  2. Enables slice 0 globally (EN = 1).
  3. In a loop:
     a. Polls PWM.INTR for bit 0.
     b. Once set: W1Cs INTR bit 0 and increments a counter at
        0x2000_3000.
     c. Loops forever.

Test oracle: the counter at 0x2000_3000 advances beyond 1 within a
wall-clock budget, proving PWM slice 0 wrapped at least once and the
W1C path works.

PWM base on RP2350 is 0x400A_8000 (datasheet §12.5 / §2.3 Table 6,
and the pico-sdk-pico2 `addressmap.h`). The HLD V5 §6 Phase 2 table's
`0x4005_0000` is the RP2040 PWM base and — on RP2350 — is actually
PLL_SYS; the correct RP2350 PWM base is used here.

References:
  - RP2350 Datasheet §12.5 (PWM).
  - `crates/mdrp2350/src/peripherals/pwm.rs` for the emulator model.
  - `gen_hello_timer.py` for Thumb-2 encoding.
"""

import struct
import sys

SRAM_BASE = 0x20000000
SRAM_SIZE = 520 * 1024
STACK_TOP = SRAM_BASE + SRAM_SIZE
COUNTER_ADDR = 0x20003000

RESETS_BASE = 0x40020000
RESETS_RESET_CLR = RESETS_BASE + 0x3000
RESET_PWM_BIT = 1 << 16

PWM_BASE = 0x400A8000
# Slice 0 registers.
SLICE0_CSR = 0x00
SLICE0_TOP = 0x10
# Global registers.
PWM_EN_OFFSET = 0xF0
PWM_INTR_OFFSET = 0xF4

CSR_EN = 1 << 0

CODE_OFFSET = 0x40

# --- Thumb-2 encoding helpers (same as gen_hello_uart.py) ---

def thumb_movw(rd, imm16):
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF240 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)

def thumb_movt(rd, imm16):
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF2C0 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)

def thumb_mov_imm32(rd, val32):
    code = thumb_movw(rd, val32 & 0xFFFF)
    if (val32 >> 16) != 0:
        code += thumb_movt(rd, (val32 >> 16) & 0xFFFF)
    return code

def thumb_str_imm(rt, rn, imm_byte):
    assert imm_byte % 4 == 0 and 0 <= imm_byte <= 124
    imm5 = imm_byte // 4
    return struct.pack('<H', 0x6000 | (imm5 << 6) | (rn << 3) | rt)

def thumb_ldr_imm(rt, rn, imm_byte):
    assert imm_byte % 4 == 0 and 0 <= imm_byte <= 124
    imm5 = imm_byte // 4
    return struct.pack('<H', 0x6800 | (imm5 << 6) | (rn << 3) | rt)

def thumb_movs_imm(rd, imm8):
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)

def thumb_adds_reg(rd, rn, rm):
    return struct.pack('<H', 0x1800 | (rm << 6) | (rn << 3) | rd)

def thumb_tst_reg(rn, rm):
    return struct.pack('<H', 0x4200 | (rm << 3) | rn)

def thumb_beq(offset):
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    return struct.pack('<H', 0xD000 | ((offset >> 1) & 0xFF))

def thumb_b(offset):
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    return struct.pack('<H', 0xE000 | ((offset >> 1) & 0x7FF))


def build_reset_handler():
    """
    Program slice 0 (CSR.EN, TOP=100), global EN=1, then loop polling
    the wrap-IRQ latch.

    Register layout:
      r3 — PWM_BASE
      r4 — counter cell address
      r5 — RESETS_RESET_CLR
      r1 — scratch / constants
      r2 — INTR read scratch
    """
    code = b''

    # Release PWM from RESETS.
    code += thumb_mov_imm32(5, RESETS_RESET_CLR)
    code += thumb_mov_imm32(1, RESET_PWM_BIT)
    code += thumb_str_imm(1, 5, 0)

    # Preload PWM_BASE. r3 carries it for the entire sled.
    code += thumb_mov_imm32(3, PWM_BASE)

    # SLICE0.CSR = CSR_EN (1).
    code += thumb_movs_imm(1, CSR_EN)
    code += thumb_str_imm(1, 3, SLICE0_CSR)

    # SLICE0.TOP = 100.
    code += thumb_movs_imm(1, 100)
    code += thumb_str_imm(1, 3, SLICE0_TOP)

    # Global PWM.EN = 1. Offset 0xF0 is > 124 so can't use STR T1.
    # Use a secondary base in r6 = PWM_BASE + 0xF0.
    code += thumb_mov_imm32(6, PWM_BASE + PWM_EN_OFFSET)
    code += thumb_movs_imm(1, 1)
    code += thumb_str_imm(1, 6, 0)

    # r7 = INTR address (PWM_BASE + 0xF4).
    code += thumb_mov_imm32(7, PWM_BASE + PWM_INTR_OFFSET)

    # Counter cell.
    code += thumb_mov_imm32(4, COUNTER_ADDR)

    # loop_top: poll INTR until bit 0 set, W1C, increment counter.
    loop_top = len(code)
    # r1 = 1 (mask + W1C value + increment).
    code += thumb_movs_imm(1, 1)
    poll_top = len(code)
    # r2 = *r7
    code += thumb_ldr_imm(2, 7, 0)
    code += thumb_tst_reg(2, 1)
    # BEQ poll_top (while Z=1 → INTR bit 0 not yet set).
    beq_off = poll_top - (len(code) + 4)
    code += thumb_beq(beq_off)
    # W1C: *r7 = r1 (= 1).
    code += thumb_str_imm(1, 7, 0)
    # Counter increment.
    code += thumb_ldr_imm(2, 4, 0)
    code += thumb_adds_reg(2, 2, 1)
    code += thumb_str_imm(2, 4, 0)
    # Loop back.
    b_off = loop_top - (len(code) + 4)
    code += thumb_b(b_off)
    return code


def build_fault_handler():
    return thumb_b(-4)


def main():
    reset_code = build_reset_handler()
    fault_code = build_fault_handler()

    reset_offset = CODE_OFFSET
    fault_offset = reset_offset + len(reset_code)
    if fault_offset % 2 != 0:
        reset_code += b'\x00'
        fault_offset += 1

    reset_vector = (SRAM_BASE + reset_offset) | 1
    fault_vector = (SRAM_BASE + fault_offset) | 1

    vectors = struct.pack('<16I',
        STACK_TOP,
        reset_vector,
        fault_vector,
        fault_vector,
        fault_vector,
        fault_vector,
        fault_vector,
        fault_vector,
        0, 0, 0,
        fault_vector,
        fault_vector,
        0,
        fault_vector,
        fault_vector,
    )
    assert len(vectors) == 64

    padding = reset_offset - len(vectors)
    assert padding >= 0

    binary = vectors + (b'\x00' * padding) + reset_code + fault_code
    while len(binary) % 256 != 0:
        binary += b'\x00'

    outpath = sys.argv[1] if len(sys.argv) > 1 else 'hello_pwm.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  Reset handler:   {SRAM_BASE + reset_offset:#010x}")
    print(f"  Counter cell:    {COUNTER_ADDR:#010x}")
    print(f"  Code size:       {len(reset_code)} bytes")


if __name__ == '__main__':
    main()
