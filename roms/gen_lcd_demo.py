#!/usr/bin/env python3
"""
Generate an RP2350 ARM firmware that bit-bangs a 20x2 character LCD over
three GPIO pins for the mdrp2354 showcase app.

Wire protocol (matches crates/mdrp2354-app/src/devices/lcd.rs and
LLD §7.3):
  GPIO 14 = SCLK  (rising edge samples DATA)
  GPIO 15 = DATA  (MSB first, stable while SCLK is high)
  GPIO 16 = CS    (active low — frames one transaction)

A frame is the sequence of bytes shifted in between a CS falling edge and
the next CS rising edge. The first byte of a frame is the opcode:
  0x01                         CLEAR      — fill with spaces, cursor=(0,0)
  0x02, col, row               SET_CURSOR — move cursor
  0x03, char+...               WRITE      — write characters at the cursor
Any other first byte is an unknown opcode and is silently dropped.

Layout at flash base 0x10000000:
  0x000: Vector table (16 standard Cortex-M33 entries)
  0x040: Picobin IMAGE_DEF block (20 bytes)
  0x054: Padding
  0x060: Reset handler + bit-bang main loop

Timing contract (LLD §4.1): the emulator samples GPIO once per quantum
(default 150 cycles). Any edge the decoder must observe has to be held
for at least 2 x quantum_cycles = 300 cycles before the next transition
on the same signal. `emit_delay(min_cycles)` clamps to a minimum of 300
cycles and is invoked around every edge.

References:
  - roms/gen_blinky.py (template)
  - wrk_docs/2026.04.13 - LLD - Emulator Showcase App.md
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

# LCD pins
SCLK_PIN = 14
DATA_PIN = 15
CS_PIN = 16

SCLK_MASK = 1 << SCLK_PIN
DATA_MASK = 1 << DATA_PIN
CS_MASK = 1 << CS_PIN

# SIO registers
SIO_BASE = 0xD0000000
SIO_GPIO_OUT_SET = SIO_BASE + 0x018
SIO_GPIO_OUT_CLR = SIO_BASE + 0x020
SIO_GPIO_OE_SET = SIO_BASE + 0x038

# IO_BANK0 / PADS_BANK0 register layouts (same formula as gen_blinky.py)
IO_BANK0_BASE = 0x40028000
PADS_BANK0_BASE = 0x40038000

def io_bank0_ctrl(n):
    """GPIO{n}_CTRL register address."""
    # GPIO25 is at 0x40028000 + 0x0CC per gen_blinky.py → 0xCC = 0x4 + 25*8.
    return IO_BANK0_BASE + 0x004 + n * 8

def pads_bank0_gpio(n):
    """PADS_BANK0_GPIO{n} register address."""
    # GPIO25 is at 0x40038000 + 0x068 per gen_blinky.py → 0x68 = 0x4 + 25*4.
    return PADS_BANK0_BASE + 0x04 + n * 4

# Picobin IMAGE_DEF metadata (copied verbatim from gen_blinky.py)
PICOBIN_BLOCK_MARKER_START = 0xFFFFDED3
PICOBIN_BLOCK_MARKER_END = 0xAB123579
PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE = 0x42
PICOBIN_BLOCK_ITEM_2BS_LAST = 0xFF
IMAGE_TYPE_VALUE = 0x0001 | (0x2 << 4) | (0x0 << 8) | (0x1 << 12)  # 0x1021

# Timing: LLD §4.1 says every edge must be held at least 2 * quantum_cycles
# = 300 cycles. The delay loop body is roughly 2 cycles per iteration on
# the emulator (SUBS + BNE taken). DELAY_ITERS = 200 burns ~400 cycles per
# edge — 33% above the minimum, so we have headroom if cycle counting is
# ever tightened. Kept identical for every edge to simplify reasoning.
DELAY_MIN_CYCLES = 300
DELAY_ITER_CYCLES = 2            # conservative per-iteration estimate
DELAY_OVERHEAD_CYCLES = 4        # MOVW + final BNE not-taken slack
DELAY_ITERS = max(
    200,
    (DELAY_MIN_CYCLES - DELAY_OVERHEAD_CYCLES + DELAY_ITER_CYCLES - 1)
    // DELAY_ITER_CYCLES,
)  # 200 iters ≈ 400 cycles — 33% above the 300-cycle floor

# =============================================================================
# Thumb-2 instruction encoding helpers (subset of gen_blinky.py)
# =============================================================================

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
    lo = val32 & 0xFFFF
    hi = (val32 >> 16) & 0xFFFF
    code = thumb_movw(rd, lo)
    if hi != 0:
        code += thumb_movt(rd, hi)
    return code

def thumb_str(rt, rn, imm12):
    assert 0 <= rt <= 15 and 0 <= rn <= 15 and 0 <= imm12 <= 4095
    hw1 = 0xF8C0 | rn
    hw2 = (rt << 12) | imm12
    return struct.pack('<HH', hw1, hw2)

def thumb_subs_imm8(rd, imm8):
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3800 | (rd << 8) | imm8)

def thumb_bne(offset):
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD100 | imm8)

def thumb_b(offset):
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)

def thumb_b_w(offset):
    """
    Unconditional B.W (T4). 25-bit signed byte offset, must be even.
    Encoding matches crates/mdrp2354/src/tests.rs::encode_b_w_uncond.
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

# =============================================================================
# Code generation helpers
# =============================================================================

def emit_gpio_init(pin):
    """Pad + CTRL + OE init sequence for one pin."""
    code = b''
    # PADS_BANK0_GPIO{pin} <- 0x34 (IE=1, DRIVE=4mA, …)
    code += thumb_mov_imm32(0, pads_bank0_gpio(pin))
    code += thumb_mov_imm32(1, 0x34)
    code += thumb_str(1, 0, 0)
    # IO_BANK0_GPIO{pin}_CTRL <- 5 (SIO function)
    code += thumb_mov_imm32(0, io_bank0_ctrl(pin))
    code += thumb_mov_imm32(1, 5)
    code += thumb_str(1, 0, 0)
    # SIO_GPIO_OE_SET <- 1 << pin
    code += thumb_mov_imm32(0, SIO_GPIO_OE_SET)
    code += thumb_mov_imm32(1, 1 << pin)
    code += thumb_str(1, 0, 0)
    return code

def emit_delay(min_cycles=DELAY_MIN_CYCLES):
    """
    Burn at least `min_cycles` emulator cycles.
    Uses r7 as the loop counter to keep r0..r6 free for the caller.
    """
    n = max(DELAY_ITERS,
            (max(min_cycles, DELAY_MIN_CYCLES) - DELAY_OVERHEAD_CYCLES
             + DELAY_ITER_CYCLES - 1) // DELAY_ITER_CYCLES)
    code = thumb_mov_imm32(7, n)        # MOVW r7, #n
    loop_start = len(code)
    code += thumb_subs_imm8(7, 1)       # SUBS r7, #1
    # BNE back to loop_start. PC during BNE = (loop_start + 2) + 4.
    bne_pos = loop_start + 2
    bne_offset = loop_start - (bne_pos + 4)  # -6
    code += thumb_bne(bne_offset)
    return code

def emit_gpio_set(mask):
    """STR #mask to SIO_GPIO_OUT_SET. Clobbers r0, r1."""
    code = thumb_mov_imm32(0, SIO_GPIO_OUT_SET)
    code += thumb_mov_imm32(1, mask)
    code += thumb_str(1, 0, 0)
    return code

def emit_gpio_clr(mask):
    """STR #mask to SIO_GPIO_OUT_CLR. Clobbers r0, r1."""
    code = thumb_mov_imm32(0, SIO_GPIO_OUT_CLR)
    code += thumb_mov_imm32(1, mask)
    code += thumb_str(1, 0, 0)
    return code

def emit_bit(bit_value):
    """
    Shift one bit out of DATA, clocked by SCLK.
    Timing contract: every level held ≥ DELAY_MIN_CYCLES cycles.
    Sequence:
      DATA <- bit, delay     (stabilise DATA with SCLK low)
      SCLK <- high, delay    (rising edge — the decoder samples)
      SCLK <- low, delay     (falling edge, prepare for next bit)
    """
    code = b''
    if bit_value:
        code += emit_gpio_set(DATA_MASK)
    else:
        code += emit_gpio_clr(DATA_MASK)
    code += emit_delay()
    code += emit_gpio_set(SCLK_MASK)
    code += emit_delay()
    code += emit_gpio_clr(SCLK_MASK)
    code += emit_delay()
    return code

def emit_byte(byte):
    """Shift one byte MSB-first onto DATA, clocked by SCLK."""
    code = b''
    for i in range(8):
        bit_value = (byte >> (7 - i)) & 1
        code += emit_bit(bit_value)
    return code

def emit_frame(payload_bytes):
    """
    Wrap a byte sequence in a CS-low / CS-high frame.
    LLD §7.3: the decoder distinguishes command frames from write frames
    by the first byte, so the caller is responsible for the payload layout.
    """
    code = b''
    code += emit_gpio_clr(CS_MASK)        # CS low: start frame
    code += emit_delay()
    for b in payload_bytes:
        code += emit_byte(b)
    code += emit_delay()                  # settle before CS high
    code += emit_gpio_set(CS_MASK)        # CS high: end frame
    code += emit_delay()
    return code

def emit_cmd_clear():
    return emit_frame(bytes([0x01]))

def emit_cmd_set_cursor(col, row):
    return emit_frame(bytes([0x02, col, row]))

def emit_cmd_write(text):
    # LLD §7.3: WRITE frames start with the 0x03 opcode byte, followed by
    # the characters to draw at the current cursor.
    data = text.encode('ascii') if isinstance(text, str) else bytes(text)
    return emit_frame(bytes([0x03]) + data)

# =============================================================================
# Reset handler
# =============================================================================

def build_reset_handler():
    code = b''

    # 1. Configure the three LCD pins (SCLK, DATA, CS).
    for pin in (SCLK_PIN, DATA_PIN, CS_PIN):
        code += emit_gpio_init(pin)

    # 2. Idle state: CS high, SCLK low, DATA low.
    code += emit_gpio_set(CS_MASK)
    code += emit_gpio_clr(SCLK_MASK | DATA_MASK)
    code += emit_delay()

    # 3. Main loop.
    main_loop_offset = len(code)

    code += emit_cmd_clear()
    code += emit_cmd_set_cursor(0, 0)
    code += emit_cmd_write("Hello from")
    code += emit_cmd_set_cursor(0, 1)
    code += emit_cmd_write("mdrp2354!")

    # 4. Long pause between refresh cycles: emit several delays.
    for _ in range(8):
        code += emit_delay(2000)

    # 5. Branch back to the top of the main loop. B.W because the loop
    # body is well over the ±2KB range of a short B.
    b_pos = len(code)
    b_offset = main_loop_offset - (b_pos + 4)
    code += thumb_b_w(b_offset)
    return code

def build_fault_handler():
    return thumb_b(-4)  # B . (spin)

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

    # Handle unconditional branch being out of range: split the main loop
    # to land within the ±2KB window, or fall back to a B.W. For now assert
    # and let a future extension deal with it.
    fault_offset = code_offset + len(reset_code)
    if fault_offset % 2 != 0:
        reset_code += b'\x00'
        fault_offset += 1

    reset_vector = (FLASH_BASE + code_offset) | 1
    fault_vector = (FLASH_BASE + fault_offset) | 1

    vectors = struct.pack('<16I',
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

    outpath = sys.argv[1] if len(sys.argv) > 1 else 'lcd_demo.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  Flash base:      {FLASH_BASE:#010x}")
    print(f"  Reset handler:   {FLASH_BASE + code_offset:#010x}")
    print(f"  Reset code size: {len(reset_code)} bytes")
    print(f"  LCD pins:        SCLK=GPIO{SCLK_PIN}, DATA=GPIO{DATA_PIN}, "
          f"CS=GPIO{CS_PIN}")
    print(f"  Delay iters:     {DELAY_ITERS} (~{DELAY_ITERS * DELAY_ITER_CYCLES} cycles)")

if __name__ == '__main__':
    main()
