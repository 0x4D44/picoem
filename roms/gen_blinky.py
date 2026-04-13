#!/usr/bin/env python3
"""
Generate a minimal RP2350 ARM blinky binary for emulator testing.

Layout at flash base 0x10000000:
  0x000: Vector table (16 standard Cortex-M33 entries)
  0x040: Picobin IMAGE_DEF block (20 bytes, must be within first 4KB)
  0x054: Padding to align code
  0x060: Reset handler code (blinky main loop)
  0x0xx: Fault handler (infinite loop)

The binary blinks GPIO25 (Pico 2 onboard LED) by toggling it via
SIO GPIO_OUT_XOR register. The delay loop is a simple countdown.

References:
  - RP2350 Datasheet Section 5.9.5 (Minimum Viable Image Metadata)
  - pico-sdk/src/common/boot_picobin_headers/include/boot/picobin.h
  - metebalci/rp2350-bare-metal-build (MIT-style reference implementation)
"""

import struct
import sys

# =============================================================================
# Constants
# =============================================================================

FLASH_BASE = 0x10000000
SRAM_BASE = 0x20000000
SRAM_SIZE = 520 * 1024  # 520KB SRAM on RP2350

# Stack at top of SRAM
STACK_TOP = SRAM_BASE + SRAM_SIZE  # 0x20082000

# RP2350 peripheral addresses
SIO_BASE = 0xD0000000
SIO_GPIO_OUT_SET = SIO_BASE + 0x018
SIO_GPIO_OUT_CLR = SIO_BASE + 0x020
SIO_GPIO_OUT_XOR = SIO_BASE + 0x028
SIO_GPIO_OE_SET = SIO_BASE + 0x038

IO_BANK0_BASE = 0x40028000
IO_BANK0_GPIO25_CTRL = IO_BANK0_BASE + 0x0CC  # GPIO25 ctrl register

PADS_BANK0_BASE = 0x40038000
PADS_BANK0_GPIO25 = PADS_BANK0_BASE + 0x68  # GPIO25 pad register

# Picobin constants (from picobin.h)
PICOBIN_BLOCK_MARKER_START = 0xFFFFDED3
PICOBIN_BLOCK_MARKER_END = 0xAB123579

PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE = 0x42
PICOBIN_BLOCK_ITEM_2BS_LAST = 0xFF

# IMAGE_TYPE value for: EXE | Secure | ARM | RP2350
# Bits: [3:0]=IMAGE_TYPE_EXE(1), [5:4]=EXE_SECURITY_S(2), [10:8]=EXE_CPU_ARM(0), [14:12]=EXE_CHIP_RP2350(1)
IMAGE_TYPE_VALUE = (
    0x0001  # IMAGE_TYPE_EXE
    | (0x2 << 4)  # EXE_SECURITY_S
    | (0x0 << 8)  # EXE_CPU_ARM
    | (0x1 << 12)  # EXE_CHIP_RP2350
)
# = 0x1021

# =============================================================================
# Thumb-2 instruction encoding helpers
# =============================================================================

def thumb_nop():
    """NOP (Thumb-16)"""
    return struct.pack('<H', 0xBF00)

def thumb_movw(rd, imm16):
    """MOVW Rd, #imm16 (Thumb-32): places imm16 in lower halfword of Rd"""
    assert 0 <= rd <= 15 and 0 <= imm16 <= 0xFFFF
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF240 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)

def thumb_movt(rd, imm16):
    """MOVT Rd, #imm16 (Thumb-32): places imm16 in upper halfword of Rd"""
    assert 0 <= rd <= 15 and 0 <= imm16 <= 0xFFFF
    imm4 = (imm16 >> 12) & 0xF
    i = (imm16 >> 11) & 0x1
    imm3 = (imm16 >> 8) & 0x7
    imm8 = imm16 & 0xFF
    hw1 = 0xF2C0 | (i << 10) | imm4
    hw2 = (imm3 << 12) | (rd << 8) | imm8
    return struct.pack('<HH', hw1, hw2)

def thumb_mov_imm32(rd, val32):
    """Load a 32-bit immediate into Rd using MOVW+MOVT"""
    lo = val32 & 0xFFFF
    hi = (val32 >> 16) & 0xFFFF
    code = thumb_movw(rd, lo)
    if hi != 0:
        code += thumb_movt(rd, hi)
    return code

def thumb_str(rt, rn, imm12):
    """STR.W Rt, [Rn, #imm12] (Thumb-32)"""
    assert 0 <= rt <= 15 and 0 <= rn <= 15 and 0 <= imm12 <= 4095
    hw1 = 0xF8C0 | rn
    hw2 = (rt << 12) | imm12
    return struct.pack('<HH', hw1, hw2)

def thumb_ldr_imm(rt, rn, imm12):
    """LDR.W Rt, [Rn, #imm12] (Thumb-32)"""
    assert 0 <= rt <= 15 and 0 <= rn <= 15 and 0 <= imm12 <= 4095
    hw1 = 0xF8D0 | rn
    hw2 = (rt << 12) | imm12
    return struct.pack('<HH', hw1, hw2)

def thumb_subs_imm8(rd, imm8):
    """SUBS Rd, #imm8 (Thumb-16, Rd must be R0-R7)"""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3800 | (rd << 8) | imm8)

def thumb_bne(offset):
    """BNE label (Thumb-16). offset is signed byte offset from PC+4."""
    # offset must be even, in range -256..+254
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD100 | imm8)

def thumb_b(offset):
    """B label (Thumb-16 unconditional). offset from PC+4, range -2048..+2046"""
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)

def thumb_bx_lr():
    """BX LR (Thumb-16): return from function"""
    return struct.pack('<H', 0x4770)

# =============================================================================
# Build the binary
# =============================================================================

def build_picobin_block():
    """Build the 20-byte picobin IMAGE_DEF block."""
    block = b''
    # Marker start
    block += struct.pack('<I', PICOBIN_BLOCK_MARKER_START)
    # Item 0: IMAGE_TYPE (1BS = 1-byte-size)
    block += struct.pack('<B', PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE)
    block += struct.pack('<B', 0x01)  # size: 1 word
    block += struct.pack('<H', IMAGE_TYPE_VALUE)
    # Item 1: LAST (2BS = 2-byte-size)
    block += struct.pack('<B', PICOBIN_BLOCK_ITEM_2BS_LAST)
    block += struct.pack('<H', 0x0001)  # total block size in words (1 = minimal)
    block += struct.pack('<B', 0x00)  # pad
    # Next block offset (0 = self-loop, single block)
    block += struct.pack('<I', 0x00000000)
    # Marker end
    block += struct.pack('<I', PICOBIN_BLOCK_MARKER_END)
    assert len(block) == 20, f"Block is {len(block)} bytes, expected 20"
    return block

def build_reset_handler():
    """
    Build the blinky reset handler.

    Sequence:
    1. Configure GPIO25 pad (output enable, drive strength)
    2. Configure GPIO25 function select to SIO (function 5)
    3. Enable GPIO25 output via SIO OE_SET
    4. Set GPIO25 high initially
    5. Loop: delay, toggle GPIO25 via XOR, repeat
    """
    code = b''

    # --- Step 1: Configure PADS_BANK0 GPIO25 ---
    # Write 0x34 (output disable=0, input enable=1, drive=4mA, PUE=0, PDE=0, schmitt=1, slewfast=0)
    # Actually for output: 0x30 is fine (IE=1, OD=0, DRIVE=2x4mA)
    # Default pad value 0x56 has OD=1 which disables output. We need OD=0.
    # Pico SDK blinky uses 0x34: IE=1, DRIVE=4mA, PUE=0, PDE=1, SCHMITT=1, SLEWFAST=0
    code += thumb_mov_imm32(0, PADS_BANK0_GPIO25)  # R0 = pad register address
    code += thumb_mov_imm32(1, 0x34)  # R1 = pad config value
    code += thumb_str(1, 0, 0)  # STR R1, [R0, #0]

    # --- Step 2: Configure IO_BANK0 GPIO25_CTRL = 5 (SIO function) ---
    code += thumb_mov_imm32(0, IO_BANK0_GPIO25_CTRL)  # R0 = ctrl register
    code += thumb_mov_imm32(1, 5)  # R1 = function 5 (SIO)
    code += thumb_str(1, 0, 0)  # STR R1, [R0, #0]

    # --- Step 3: Enable GPIO25 output via SIO_GPIO_OE_SET ---
    code += thumb_mov_imm32(0, SIO_GPIO_OE_SET)  # R0 = OE_SET register
    code += thumb_mov_imm32(1, 1 << 25)  # R1 = bit 25
    code += thumb_str(1, 0, 0)  # STR R1, [R0, #0]

    # --- Step 4: Set GPIO25 high initially ---
    code += thumb_mov_imm32(0, SIO_GPIO_OUT_SET)  # R0 = OUT_SET register
    # R1 still = (1 << 25)
    code += thumb_str(1, 0, 0)  # STR R1, [R0, #0]

    # --- Step 5: Main blink loop ---
    # Pre-load R4 = XOR register address, R5 = bit 25 mask
    code += thumb_mov_imm32(4, SIO_GPIO_OUT_XOR)
    code += thumb_mov_imm32(5, 1 << 25)

    # loop_top:
    loop_top_offset = len(code)

    # Delay loop: R2 = 500000, count down to 0
    code += thumb_mov_imm32(2, 500000)  # ~500k iterations

    # delay_loop:
    delay_loop_offset = len(code)
    code += thumb_subs_imm8(2, 1)  # SUBS R2, #1
    # BNE back to delay_loop: offset is from (current_pc + 4) to delay_loop
    # Current instruction is at delay_loop_offset + 2 bytes (the SUBS is 2 bytes)
    # PC during BNE = delay_loop_offset + 2 + 4 = delay_loop_offset + 6
    # Target = delay_loop_offset
    # offset = delay_loop_offset - (delay_loop_offset + 2 + 4) = -6
    # Wait, need to account for the SUBS being 2 bytes
    bne_pos = delay_loop_offset + 2  # position of BNE instruction
    bne_offset = delay_loop_offset - (bne_pos + 4)  # -6
    code += thumb_bne(bne_offset)

    # Toggle GPIO25
    code += thumb_str(5, 4, 0)  # STR R5, [R4, #0] -- XOR toggle

    # Branch back to loop_top
    b_pos = len(code)
    b_offset = loop_top_offset - (b_pos + 4)
    code += thumb_b(b_offset)

    return code

def build_fault_handler():
    """Infinite loop for fault handlers."""
    # B . (branch to self)
    return thumb_b(-4)  # offset -4 = branch to self (PC+4-4 = PC)

def main():
    # Code starts at offset 0x60 from flash base (after vectors + picobin block + padding)
    CODE_OFFSET = 0x60
    RESET_HANDLER_ADDR = FLASH_BASE + CODE_OFFSET
    # Fault handler at a known offset after reset handler
    # We'll place it right after the reset handler code

    # Build code first to know its size
    reset_code = build_reset_handler()
    fault_code = build_fault_handler()

    # Fault handler address (right after reset code, aligned to 2 bytes)
    fault_offset = CODE_OFFSET + len(reset_code)
    if fault_offset % 2 != 0:
        reset_code += b'\x00'  # pad
        fault_offset += 1
    FAULT_HANDLER_ADDR = FLASH_BASE + fault_offset

    # Thumb bit must be set in vector table entries (bit 0 = 1)
    reset_vector = RESET_HANDLER_ADDR | 1
    fault_vector = FAULT_HANDLER_ADDR | 1

    # =================================================================
    # Vector Table (16 entries = 64 bytes at offset 0x000)
    # =================================================================
    vectors = struct.pack('<16I',
        STACK_TOP,          # 0x00: Initial SP
        reset_vector,       # 0x04: Reset
        fault_vector,       # 0x08: NMI
        fault_vector,       # 0x0C: HardFault
        fault_vector,       # 0x10: MemManage
        fault_vector,       # 0x14: BusFault
        fault_vector,       # 0x18: UsageFault
        fault_vector,       # 0x1C: SecureFault
        0,                  # 0x20: Reserved
        0,                  # 0x24: Reserved
        0,                  # 0x28: Reserved
        fault_vector,       # 0x2C: SVCall
        fault_vector,       # 0x30: DebugMon
        0,                  # 0x34: Reserved
        fault_vector,       # 0x38: PendSV
        fault_vector,       # 0x3C: SysTick
    )
    assert len(vectors) == 64

    # =================================================================
    # Picobin IMAGE_DEF block (20 bytes at offset 0x040)
    # =================================================================
    picobin_block = build_picobin_block()

    # =================================================================
    # Padding to CODE_OFFSET
    # =================================================================
    used = len(vectors) + len(picobin_block)
    padding = CODE_OFFSET - used
    assert padding >= 0, f"Code offset {CODE_OFFSET:#x} too small, need {used:#x}"

    # =================================================================
    # Assemble full binary
    # =================================================================
    binary = vectors + picobin_block + (b'\x00' * padding) + reset_code + fault_code

    # Pad to a nice size (256 byte aligned)
    while len(binary) % 256 != 0:
        binary += b'\x00'

    # Write output
    outpath = sys.argv[1] if len(sys.argv) > 1 else 'blinky.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  Flash base:      {FLASH_BASE:#010x}")
    print(f"  Stack top:       {STACK_TOP:#010x}")
    print(f"  Reset handler:   {RESET_HANDLER_ADDR:#010x} (vector: {reset_vector:#010x})")
    print(f"  Fault handler:   {FAULT_HANDLER_ADDR:#010x} (vector: {fault_vector:#010x})")
    print(f"  Picobin block:   offset 0x{len(vectors):03x}, IMAGE_TYPE={IMAGE_TYPE_VALUE:#06x}")
    print(f"  Code region:     offset {CODE_OFFSET:#05x} - {CODE_OFFSET + len(reset_code) + len(fault_code) - 1:#05x}")

if __name__ == '__main__':
    main()
