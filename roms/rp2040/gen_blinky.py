#!/usr/bin/env python3
"""
Generate a minimal RP2040 Cortex-M0+ blinky binary for the mdrp2040app
emulator demo.

The RP2040 reset sequence in `mdrp2040::Emulator::reset` pulls the
initial SP from ROM word 0 and the reset vector from ROM word 4. In the
real Pico, the bootrom at 0x0000_0000 does that job; here we generate a
minimal `bootrom.bin` that just contains those two words plus infinite-
loop filler, and a `blinky.bin` placed at flash base 0x1000_0000 whose
first bytes are the reset handler.

Layout:
  bootrom.bin (ROM @ 0x00000000):
    0x000: Initial SP = top of SRAM = 0x20042000
    0x004: Reset vector = 0x10000001 (flash base + Thumb bit)
    0x008: Remaining ROM filled with B . (self-loop) halfwords.

  blinky.bin (flash @ 0x10000000):
    0x000..0x0xx: Reset handler code (MOVS / LSLS / LDR / STR / B)
    0x0xx..0x0yy: Literal pool (SIO_BASE, bit 25 mask)

The reset handler:
  1. r4 = SIO_BASE (via LDR literal)
  2. r5 = 1 << 25  (GPIO25 mask, built via MOVS + LSLS)
  3. r6 = 1 << 25  for writing to SET registers
  4. [r4 + 0x024] = r6  -> SIO_GPIO_OE_SET  (enable output)
  5. [r4 + 0x014] = r6  -> SIO_GPIO_OUT_SET (drive HIGH)
  6. loop {
       delay countdown (r2)
       [r4 + 0x01C] = r6  -> SIO_GPIO_OUT_XOR (toggle)
     }

Cortex-M0+ instruction notes:
  - No MOVW/MOVT (Thumb-32) — use LDR [PC, #imm8] from literal pool.
  - STR Rt, [Rn, #imm5] where imm5 is word-scaled (range 0..124).
  - BNE/B with Thumb-16 encoding only.
"""

import struct
import sys
from pathlib import Path

# =============================================================================
# Constants
# =============================================================================

FLASH_BASE = 0x10000000
SRAM_BASE  = 0x20000000
SRAM_SIZE  = 264 * 1024       # 264KB SRAM on RP2040
STACK_TOP  = SRAM_BASE + SRAM_SIZE  # 0x20042000

ROM_SIZE   = 16 * 1024        # 16KB ROM

# RP2040 SIO register offsets (from base 0xD0000000).
SIO_BASE          = 0xD0000000
SIO_GPIO_OUT      = 0x010
SIO_GPIO_OUT_SET  = 0x014
SIO_GPIO_OUT_XOR  = 0x01C
SIO_GPIO_OE       = 0x020
SIO_GPIO_OE_SET   = 0x024

# =============================================================================
# Thumb-16 instruction encoding helpers (Cortex-M0+ subset)
# =============================================================================

def thumb_movs_imm8(rd, imm8):
    """MOVS Rd, #imm8 — T1 encoding. Rd in r0..r7."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)

def thumb_lsls_imm5(rd, rm, imm5):
    """LSLS Rd, Rm, #imm5 — T1 encoding. imm5 in 0..31."""
    assert 0 <= rd <= 7 and 0 <= rm <= 7 and 0 <= imm5 <= 31
    return struct.pack('<H', 0x0000 | (imm5 << 6) | (rm << 3) | rd)

def thumb_ldr_pc(rt, imm8):
    """LDR Rt, [PC, #imm8] — T1 (word load from literal pool).

    imm8 is word-scaled. Effective address = (Align(PC, 4)) + (imm8 * 4),
    where PC = current instruction address + 4.
    """
    assert 0 <= rt <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x4800 | (rt << 8) | imm8)

def thumb_str_imm5(rt, rn, imm5_words):
    """STR Rt, [Rn, #imm5*4] — T1 encoding. imm5 is word-scaled (0..31)."""
    assert 0 <= rt <= 7 and 0 <= rn <= 7 and 0 <= imm5_words <= 31
    return struct.pack('<H', 0x6000 | (imm5_words << 6) | (rn << 3) | rt)

def thumb_mov_r_lo(rd, rm):
    """MOV Rd, Rm — T1 encoding (any low register to any low register).
    Uses the MOV (register) encoding from ARMv6-M."""
    assert 0 <= rd <= 7 and 0 <= rm <= 7
    return struct.pack('<H', 0x4600 | (rm << 3) | rd)

def thumb_subs_imm8(rd, imm8):
    """SUBS Rd, Rd, #imm8 — T2 encoding. Rd in r0..r7."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3800 | (rd << 8) | imm8)

def thumb_bne(offset):
    """BNE label — T1 encoding. offset from (PC+4), range -256..+254, even."""
    assert -256 <= offset <= 254 and offset % 2 == 0
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD100 | imm8)

def thumb_b(offset):
    """B label — T2 encoding. offset from (PC+4), range -2048..+2046, even."""
    assert -2048 <= offset <= 2046 and offset % 2 == 0
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)

def thumb_nop():
    """NOP — T1 encoding (MOV r8, r8 aliased as NOP)."""
    return struct.pack('<H', 0xBF00)

# =============================================================================
# Code generation
# =============================================================================

def build_blinky():
    """
    Build the reset handler + literal pool for blinky.bin.

    Register usage:
      r0 = scratch (used to build the bit 25 mask)
      r2 = delay counter
      r4 = SIO_BASE (loaded from literal pool)
      r5 = 1 << 25 (GPIO25 mask)

    Layout:
      offset 0: code
      offset N: literal pool (SIO_BASE aligned to 4 bytes)
    """
    code = b''

    # --- Build bit 25 mask in r5 via MOVS + LSLS ---
    code += thumb_movs_imm8(5, 1)       # MOVS r5, #1
    code += thumb_lsls_imm5(5, 5, 25)   # LSLS r5, r5, #25

    # --- Load SIO_BASE into r4 from literal pool ---
    # We don't yet know the literal-pool offset. Place a placeholder and
    # fix up below. We'll put the literal pool immediately after the
    # code body, word-aligned.
    ldr_fixup = len(code)
    code += thumb_ldr_pc(4, 0)          # LDR r4, [PC, #?] (patched later)

    # --- SIO_GPIO_OE_SET (+0x024) = r5 ---
    # STR Rt, [Rn, #imm5*4]: imm5 = 0x024 / 4 = 9.
    code += thumb_str_imm5(5, 4, 9)     # STR r5, [r4, #0x24]

    # --- SIO_GPIO_OUT_SET (+0x014) = r5 ---
    # imm5 = 0x014 / 4 = 5.
    code += thumb_str_imm5(5, 4, 5)     # STR r5, [r4, #0x14]

    # --- Main blink loop ---
    loop_top = len(code)

    # Delay init: MOVS r0, #0xFA; LSLS r2, r0, #8  (r2 = 0xFA00 ≈ 64000)
    # We want ~64K inner-loop iterations for a visible toggle period at
    # ~6.5 MHz ROSC (about 40ms).
    code += thumb_movs_imm8(0, 0xFA)    # MOVS r0, #0xFA
    code += thumb_lsls_imm5(2, 0, 8)    # LSLS r2, r0, #8   (r2 = 0xFA00)

    # Inner delay loop: SUBS r2, r2, #1; BNE back.
    delay_loop = len(code)
    code += thumb_subs_imm8(2, 1)       # SUBS r2, r2, #1
    # BNE target is `delay_loop`. At BNE, PC points to BNE+4, so offset
    # is (delay_loop) - (delay_loop + 2 + 4) = -6.
    bne_instr_off = delay_loop + 2
    bne_offset = delay_loop - (bne_instr_off + 4)
    code += thumb_bne(bne_offset)

    # Toggle GPIO25 via SIO_GPIO_OUT_XOR (+0x01C). imm5 = 0x01C / 4 = 7.
    code += thumb_str_imm5(5, 4, 7)     # STR r5, [r4, #0x1C]

    # B back to loop_top. PC at B = b_pos + 4, offset = loop_top - (b_pos+4).
    b_pos = len(code)
    b_offset = loop_top - (b_pos + 4)
    code += thumb_b(b_offset)

    # --- Literal pool (word-aligned) ---
    # Align to 4 bytes before placing the SIO_BASE literal.
    if len(code) % 4 != 0:
        code += thumb_nop()             # 2-byte padding

    literal_offset = len(code)
    code += struct.pack('<I', SIO_BASE)

    # --- Fix up the LDR offset ---
    # LDR Rt, [PC, #imm8]: PC during the load = Align(PC, 4) where
    # PC = ldr_instr_addr + 4. Target = literal_offset (bytes from start).
    # imm8 is scaled by 4.
    ldr_pc_aligned = (ldr_fixup + 4) & ~3
    imm8_bytes = literal_offset - ldr_pc_aligned
    assert imm8_bytes >= 0 and imm8_bytes % 4 == 0, \
        f"literal pool before LDR? literal_offset={literal_offset:#x}, ldr_pc_aligned={ldr_pc_aligned:#x}"
    imm8 = imm8_bytes // 4
    assert 0 <= imm8 <= 255, f"LDR offset out of range: {imm8}"

    # Patch the LDR halfword.
    old = struct.unpack('<H', code[ldr_fixup:ldr_fixup + 2])[0]
    patched = (old & ~0xFF) | imm8
    code = code[:ldr_fixup] + struct.pack('<H', patched) + code[ldr_fixup + 2:]

    return code

def build_bootrom():
    """
    Build a minimal ROM image:
      word 0: initial SP = STACK_TOP
      word 1: reset vector = FLASH_BASE | 1
      rest:   infinite self-loop halfwords (B .) to trap stray fetches
    """
    rom = b''
    rom += struct.pack('<I', STACK_TOP)
    rom += struct.pack('<I', FLASH_BASE | 1)

    # Fill the rest with `B .` — halfword encoding: 0xE7FE.
    # (offset -4 from PC+4 = branch to self).
    trap = struct.pack('<H', 0xE7FE)
    while len(rom) < ROM_SIZE:
        rom += trap

    assert len(rom) == ROM_SIZE
    return rom

def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent

    blinky = build_blinky()
    bootrom = build_bootrom()

    blinky_path = out_dir / 'blinky.bin'
    bootrom_path = out_dir / 'bootrom.bin'

    blinky_path.write_bytes(blinky)
    bootrom_path.write_bytes(bootrom)

    print(f"Wrote {blinky_path} ({len(blinky)} bytes)")
    print(f"  flash base:    {FLASH_BASE:#010x}")
    print(f"  reset handler: {FLASH_BASE:#010x} (Thumb entry {FLASH_BASE|1:#010x})")
    print(f"Wrote {bootrom_path} ({len(bootrom)} bytes)")
    print(f"  stack top:     {STACK_TOP:#010x}")
    print(f"  reset vector:  {FLASH_BASE|1:#010x}")

if __name__ == '__main__':
    main()
