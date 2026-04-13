#!/usr/bin/env python3
"""
Generate a minimal RP2350 dual-core test binary for emulator testing.

Layout at flash base 0x10000000:
  0x000: Core 0 vector table (16 standard Cortex-M33 entries)
  0x040: Picobin IMAGE_DEF block (20 bytes)
  0x054: Padding to 0x060
  0x060: Core 0 reset handler (launch Core 1, set GPIO 25, loop)
  0x100: Core 1 vector table (used as VTOR in launch sequence)
  0x140: Core 1 entry code (set GPIO 0, loop)

Core 0 runs the standard multicore_launch_core1 protocol:
  1. Drain RX FIFO
  2. Send {0, 0, 1, vtor, sp, entry1} with SEV after each write
  3. Read echo for non-zero values (1, vtor, sp, entry1)
  4. Set GPIO 25 on success
  5. Infinite loop

Core 1 (launched by bootrom after receiving the sequence):
  1. Set GPIO 0
  2. Infinite loop

References:
  - RP2350 Datasheet Section 5.9.5 (Minimum Viable Image Metadata)
  - pico-sdk multicore_launch_core1 protocol
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

# Core 1 stack: 4KB below Core 0's stack top
CORE1_SP = STACK_TOP - 4096  # 0x20081000

# RP2350 SIO addresses
SIO_BASE       = 0xD0000000
SIO_GPIO_OUT_SET = SIO_BASE + 0x018
SIO_FIFO_ST    = SIO_BASE + 0x050
SIO_FIFO_WR    = SIO_BASE + 0x054
SIO_FIFO_RD    = SIO_BASE + 0x058

# Binary offsets
CORE0_CODE_OFFSET = 0x060
CORE1_VTOR_OFFSET = 0x200
CORE1_CODE_OFFSET = 0x240

# Computed addresses
CORE1_VTOR  = FLASH_BASE + CORE1_VTOR_OFFSET
CORE1_ENTRY = FLASH_BASE + CORE1_CODE_OFFSET

# Picobin constants
PICOBIN_BLOCK_MARKER_START = 0xFFFFDED3
PICOBIN_BLOCK_MARKER_END = 0xAB123579
PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE = 0x42
PICOBIN_BLOCK_ITEM_2BS_LAST = 0xFF

IMAGE_TYPE_VALUE = (
    0x0001        # IMAGE_TYPE_EXE
    | (0x2 << 4)  # EXE_SECURITY_S
    | (0x0 << 8)  # EXE_CPU_ARM
    | (0x1 << 12) # EXE_CHIP_RP2350
)

# =============================================================================
# Thumb instruction encoding helpers
# =============================================================================

def thumb_nop():
    """NOP (Thumb-16)"""
    return struct.pack('<H', 0xBF00)

def thumb_sev():
    """SEV (Thumb-16): Send Event"""
    return struct.pack('<H', 0xBF40)

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

def thumb_tst_imm(rn, imm12):
    """TST.W Rn, #imm12 (Thumb-32): test bits, sets Z if (Rn & imm) == 0
    Uses the simple unrotated encoding for small immediates (0-255)."""
    assert 0 <= rn <= 15 and 0 <= imm12 <= 255
    # TST.W Rn, #imm is encoded as: 0xF010 Rn, 0x0F00 | imm8
    hw1 = 0xF010 | rn
    hw2 = 0x0F00 | (imm12 & 0xFF)
    return struct.pack('<HH', hw1, hw2)

def thumb_cmp_reg(rn, rm):
    """CMP Rn, Rm (Thumb-16, high regs OK): compare two registers"""
    # Encoding T2: 0100 0101 N Rm Rn (where N is high bit of Rn)
    assert 0 <= rn <= 15 and 0 <= rm <= 15
    n_hi = (rn >> 3) & 1
    rn_lo = rn & 0x7
    return struct.pack('<H', 0x4500 | (n_hi << 7) | (rm << 3) | rn_lo)

def thumb_beq(offset):
    """BEQ label (Thumb-16). offset is signed byte offset from PC+4."""
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD000 | imm8)

def thumb_bne(offset):
    """BNE label (Thumb-16). offset is signed byte offset from PC+4."""
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD100 | imm8)

def thumb_b(offset):
    """B label (Thumb-16 unconditional). offset from PC+4, range -2048..+2046"""
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)

# =============================================================================
# Build the binary
# =============================================================================

def build_picobin_block():
    """Build the 20-byte picobin IMAGE_DEF block (same as gen_blinky.py)."""
    block = b''
    block += struct.pack('<I', PICOBIN_BLOCK_MARKER_START)
    block += struct.pack('<B', PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE)
    block += struct.pack('<B', 0x01)  # size: 1 word
    block += struct.pack('<H', IMAGE_TYPE_VALUE)
    block += struct.pack('<B', PICOBIN_BLOCK_ITEM_2BS_LAST)
    block += struct.pack('<H', 0x0001)
    block += struct.pack('<B', 0x00)
    block += struct.pack('<I', 0x00000000)
    block += struct.pack('<I', PICOBIN_BLOCK_MARKER_END)
    assert len(block) == 20
    return block

def build_core0_handler():
    """
    Core 0 reset handler: launch Core 1 via FIFO, then set GPIO 25.

    Register plan:
      R6 = SIO_FIFO_ST address (0xD0000050)
      R7 = SIO_FIFO_WR address (0xD0000054)
      R8 = SIO_FIFO_RD address (0xD0000058)

    Launch sequence: {0, 0, 1, vtor, sp, entry1}
    For each word: write FIFO_WR, SEV, poll FIFO_ST VLD, read FIFO_RD echo
    """
    code = b''

    # Load SIO FIFO register addresses into high registers
    code += thumb_mov_imm32(6, SIO_FIFO_ST)   # R6 = FIFO_ST
    code += thumb_mov_imm32(7, SIO_FIFO_WR)   # R7 = FIFO_WR
    code += thumb_mov_imm32(8, SIO_FIFO_RD)   # R8 = FIFO_RD

    # --- Step 1: Drain RX FIFO ---
    # drain_loop:
    drain_loop_offset = len(code)
    # Read FIFO_ST, test VLD (bit 0)
    code += thumb_ldr_imm(0, 6, 0)          # LDR R0, [R6, #0]  ; R0 = FIFO_ST
    code += thumb_tst_imm(0, 1)             # TST R0, #1        ; test VLD
    beq_pos = len(code)
    code += thumb_beq(0)                    # BEQ drain_done (placeholder)
    # VLD set: read and discard
    code += thumb_ldr_imm(0, 8, 0)          # LDR R0, [R8, #0]  ; read FIFO_RD
    b_drain_pos = len(code)
    b_drain_offset = drain_loop_offset - (b_drain_pos + 4)
    code += thumb_b(b_drain_offset)          # B drain_loop
    drain_done_offset = len(code)
    # Patch the BEQ
    beq_offset = drain_done_offset - (beq_pos + 4)
    code = code[:beq_pos] + thumb_beq(beq_offset) + code[beq_pos + 2:]

    # --- Step 2: Send launch sequence ---
    # The 6-word sequence: {0, 0, 1, vtor, sp, entry1}
    # The bootrom echoes ALL values back (including zeros).
    # Core 0 must read each echo to keep the FIFO from filling up and
    # to satisfy the bootrom's handshake state machine.
    launch_words = [0, 0, 1, CORE1_VTOR, CORE1_SP, CORE1_ENTRY | 1]
    # entry1 has thumb bit set

    for i, word in enumerate(launch_words):
        # Load value to send into R0
        code += thumb_mov_imm32(0, word)       # R0 = word

        # Write to FIFO_WR
        code += thumb_str(0, 7, 0)              # STR R0, [R7, #0]

        # SEV to wake Core 1
        code += thumb_sev()                     # SEV

        # Poll FIFO_ST until VLD (bit 0) is set — wait for echo
        poll_loop_offset = len(code)
        code += thumb_ldr_imm(1, 6, 0)     # LDR R1, [R6, #0]  ; R1 = FIFO_ST
        code += thumb_tst_imm(1, 1)        # TST R1, #1        ; test VLD
        beq_poll_pos = len(code)
        beq_poll_target = poll_loop_offset - (beq_poll_pos + 4)
        code += thumb_beq(beq_poll_target)  # BEQ poll_loop (spin until VLD)

        # Read echo (discard — we trust the emulator)
        code += thumb_ldr_imm(1, 8, 0)     # LDR R1, [R8, #0]  ; R1 = echo

    # --- Step 3: Set GPIO 25 ---
    code += thumb_mov_imm32(0, SIO_GPIO_OUT_SET)   # R0 = GPIO_OUT_SET
    code += thumb_mov_imm32(1, 1 << 25)            # R1 = (1 << 25)
    code += thumb_str(1, 0, 0)                     # STR R1, [R0, #0]

    # --- Step 4: Infinite loop ---
    code += thumb_b(-4)                            # B . (branch to self)

    return code

def build_core1_code():
    """
    Core 1 entry: set GPIO 0 via SIO_GPIO_OUT_SET, then loop forever.
    """
    code = b''

    # Set GPIO 0
    code += thumb_mov_imm32(0, SIO_GPIO_OUT_SET)   # R0 = GPIO_OUT_SET
    code += thumb_mov_imm32(1, 1)                  # R1 = 1  (bit 0)
    code += thumb_str(1, 0, 0)                     # STR R1, [R0, #0]

    # Infinite loop
    code += thumb_b(-4)                            # B .

    return code

def build_fault_handler():
    """Infinite loop for fault handlers."""
    return thumb_b(-4)

def main():
    # Build code sections first to know sizes
    core0_code = build_core0_handler()
    core1_code = build_core1_code()
    fault_code = build_fault_handler()

    # Core 0 addresses
    CORE0_RESET_ADDR = FLASH_BASE + CORE0_CODE_OFFSET

    # Fault handler lives right after Core 0 code
    fault_offset = CORE0_CODE_OFFSET + len(core0_code)
    if fault_offset % 2 != 0:
        core0_code += b'\x00'
        fault_offset += 1
    FAULT_HANDLER_ADDR = FLASH_BASE + fault_offset

    # Core 1 addresses
    CORE1_RESET_ADDR = FLASH_BASE + CORE1_CODE_OFFSET

    # Thumb bit in vectors
    core0_reset_vector = CORE0_RESET_ADDR | 1
    core1_reset_vector = CORE1_RESET_ADDR | 1
    fault_vector = FAULT_HANDLER_ADDR | 1

    # =================================================================
    # Core 0 Vector Table (16 entries = 64 bytes at offset 0x000)
    # =================================================================
    core0_vectors = struct.pack('<16I',
        STACK_TOP,          # 0x00: Initial SP
        core0_reset_vector, # 0x04: Reset
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
    assert len(core0_vectors) == 64

    # =================================================================
    # Picobin IMAGE_DEF block (20 bytes at offset 0x040)
    # =================================================================
    picobin_block = build_picobin_block()

    # =================================================================
    # Core 1 Vector Table (16 entries = 64 bytes at offset 0x100)
    # This is the VTOR address Core 0 sends in the launch sequence.
    # The bootrom uses entries 0 (SP) and 1 (reset vector).
    # =================================================================
    core1_vectors = struct.pack('<16I',
        CORE1_SP,           # 0x00: Initial SP for Core 1
        core1_reset_vector, # 0x04: Reset vector for Core 1
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
    assert len(core1_vectors) == 64

    # =================================================================
    # Assemble full binary
    # =================================================================
    binary = bytearray()

    # Core 0 vectors (0x000 - 0x03F)
    binary += core0_vectors
    assert len(binary) == 0x040

    # Picobin IMAGE_DEF (0x040 - 0x053)
    binary += picobin_block
    assert len(binary) == 0x054

    # Padding to Core 0 code (0x054 - 0x05F)
    binary += b'\x00' * (CORE0_CODE_OFFSET - len(binary))
    assert len(binary) == CORE0_CODE_OFFSET

    # Core 0 code (0x060 - ...)
    binary += core0_code
    binary += fault_code

    # Pad to Core 1 vector table offset
    if len(binary) > CORE1_VTOR_OFFSET:
        print(f"ERROR: Core 0 code extends past Core 1 VTOR offset "
              f"({len(binary):#x} > {CORE1_VTOR_OFFSET:#x})")
        sys.exit(1)
    binary += b'\x00' * (CORE1_VTOR_OFFSET - len(binary))
    assert len(binary) == CORE1_VTOR_OFFSET

    # Core 1 vector table (0x100 - 0x13F)
    binary += core1_vectors
    assert len(binary) == CORE1_CODE_OFFSET

    # Core 1 code (0x140 - ...)
    binary += core1_code

    # Pad to 512-byte boundary
    while len(binary) % 512 != 0:
        binary += b'\x00'

    # Write output
    outpath = sys.argv[1] if len(sys.argv) > 1 else 'dualcore.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  Flash base:        {FLASH_BASE:#010x}")
    print(f"  Stack top (Core0): {STACK_TOP:#010x}")
    print(f"  Core 0 reset:      {CORE0_RESET_ADDR:#010x} (vector: {core0_reset_vector:#010x})")
    print(f"  Fault handler:     {FAULT_HANDLER_ADDR:#010x} (vector: {fault_vector:#010x})")
    print(f"  Core 1 VTOR:       {CORE1_VTOR:#010x}")
    print(f"  Core 1 SP:         {CORE1_SP:#010x}")
    print(f"  Core 1 entry:      {CORE1_ENTRY:#010x} (vector: {core1_reset_vector:#010x})")
    print(f"  Core 0 code size:  {len(core0_code)} bytes")
    print(f"  Core 1 code size:  {len(core1_code)} bytes")

if __name__ == '__main__':
    main()
