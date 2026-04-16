#!/usr/bin/env python3
"""
Generate a minimal RP2350 ARM firmware that exercises UART0 TX.

Phase 2 corpus entry (HLD V5 §3 / §6 row 2). The firmware:

  1. Enables the UART0 FIFO (LCR_H.FEN=1).
  2. Sets UART0 CR = UARTEN | TXE (enable + tx-enable).
  3. In a loop:
     a. Writes one byte to UARTDR ('A' = 0x41).
     b. Waits for UARTFR.TXFE (TX FIFO empty) — busy-poll.
     c. Increments a counter at 0x2000_3000.
     d. Loops forever.

Test oracle: the counter at 0x2000_3000 advances beyond 1 within a
wall-clock budget, proving the emulator processed at least one
complete TX-and-wait cycle. UARTFR.TXFE rising to 1 after the
drain validates the tick-path drain and FIFO state machine.

Layout at SRAM base 0x2000_0000:
  0x0000: Vector table (16 Cortex-M33 entries).
  0x0040: Reset handler (UART setup + TX loop).

References:
  - RP2350 Datasheet §12.1 (UART — PL011).
  - `crates/mdrp2350/src/peripherals/uart.rs` for the emulator model.
  - `gen_hello_timer.py` for Thumb-2 encoding patterns (reused below).
"""

import struct
import sys

SRAM_BASE = 0x20000000
SRAM_SIZE = 520 * 1024
STACK_TOP = SRAM_BASE + SRAM_SIZE
COUNTER_ADDR = 0x20003000

RESETS_BASE = 0x40020000
RESETS_RESET_CLR = RESETS_BASE + 0x3000
RESET_UART0_BIT = 1 << 26

UART0_BASE = 0x40070000
UART0_DR = 0x00
UART0_FR = 0x18
UART0_LCR_H = 0x2C
UART0_CR = 0x30
UARTCR_UARTEN = 1 << 0
UARTCR_TXE = 1 << 8
UARTLCR_H_FEN = 1 << 4
UARTFR_TXFE = 1 << 7

CODE_OFFSET = 0x40

# --- Thumb-2 encoding helpers ---

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
    Enable UART0, TX bytes in a loop, increment counter on each.

    Register layout:
      r3 — UART0_BASE
      r4 — counter addr (0x2000_3000)
      r5 — RESETS_RESET_CLR
      r0 — scratch / UART register read
      r1 — constant mask (TXFE bit / byte value)
      r2 — polling scratch
    """
    code = b''

    # Release UART0 from RESETS (belt-and-braces; post-bootrom already
    # releases it, but firmware exercising the real reset sequence is
    # the honest test path).
    code += thumb_mov_imm32(5, RESETS_RESET_CLR)
    code += thumb_mov_imm32(1, RESET_UART0_BIT)
    code += thumb_str_imm(1, 5, 0)

    # Preload UART0_BASE.
    code += thumb_mov_imm32(3, UART0_BASE)

    # LCR_H = FEN (enable FIFOs).
    code += thumb_movs_imm(1, UARTLCR_H_FEN)
    code += thumb_str_imm(1, 3, UART0_LCR_H)

    # CR = UARTEN | TXE.  0x101 — use mov_imm32.
    code += thumb_mov_imm32(1, UARTCR_UARTEN | UARTCR_TXE)
    code += thumb_str_imm(1, 3, UART0_CR)

    # Counter cell pointer.
    code += thumb_mov_imm32(4, COUNTER_ADDR)

    # Main loop.
    loop_top = len(code)
    # Write one byte (0x41 = 'A') to UARTDR.
    code += thumb_movs_imm(1, 0x41)
    code += thumb_str_imm(1, 3, UART0_DR)

    # Poll UARTFR.TXFE — wait for drain.
    # r1 = TXFE mask (0x80).
    code += thumb_movs_imm(1, UARTFR_TXFE)
    poll_top = len(code)
    code += thumb_ldr_imm(2, 3, UART0_FR)
    code += thumb_tst_reg(2, 1)
    # If (FR & TXFE) == 0 (Z=1 after AND), loop.
    # We want "loop while TXFE clear" — i.e. beq poll_top. But tst sets
    # Z = (operand1 AND operand2 == 0). So when TXFE is clear, Z=1 →
    # BEQ back; when TXFE is set, Z=0 → fall through.
    beq_off = poll_top - (len(code) + 4)
    code += thumb_beq(beq_off)

    # Increment counter.
    code += thumb_ldr_imm(2, 4, 0)
    code += thumb_movs_imm(1, 1)
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

    outpath = sys.argv[1] if len(sys.argv) > 1 else 'hello_uart.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  Reset handler:   {SRAM_BASE + reset_offset:#010x}")
    print(f"  Counter cell:    {COUNTER_ADDR:#010x}")
    print(f"  Code size:       {len(reset_code)} bytes")


if __name__ == '__main__':
    main()
