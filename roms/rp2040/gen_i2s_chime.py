#!/usr/bin/env python3
"""
Generate a minimal RP2040 Cortex-M0+ I2S chime binary for the
`picogus_diff_rp2040` smoke-test path.

Purpose
-------
`picogus_diff_rp2040` expects to load firmware that drives the I2S pins
(GPIO 16 DOUT, 17 BCLK, 18 LRCLK) so the harness can decode the audio
and write a WAV. Real PicoGUS v4.0.0 firmware is what we'd eventually
like to boot, but it depends on pico-sdk runtime + peripherals our
emulator stubs — investigation in progress, tracked in `tech_debt.md`.

This script generates a **synthetic** firmware that bypasses any SDK
runtime and drives the I2S pins directly via the SIO GPIO_OUT register.
It produces a 16-frame square-wave burst — enough for our `I2sCapture`
decoder to observe LRCLK edges, finalise frames, and infer a plausible
sample rate. This is the "end-to-end audio pipeline smoke test" —
demonstrating the emulator correctly routes pin writes through to the
I2S decoder.

Layout:
  bootrom.bin (ROM @ 0x00000000):        synthetic stub (reuse existing)
  i2s_chime.bin (flash @ 0x10000000):    this script's output

Firmware logic (Thumb-16 only):
  1. Load SIO_BASE into r4.
  2. Set GPIO16..18 as outputs (SIO GPIO_OE_SET = pin_mask).
  3. In a tight loop:
     - For each of N LRCLK half-frames:
       - Set LRCLK low (GPIO_OUT_CLR GPIO18)
       - For 32 bits: toggle BCLK with DOUT=bit(i)
       - Set LRCLK high (GPIO_OUT_SET GPIO18)
       - For 32 bits: toggle BCLK with DOUT=bit(i)
  4. Infinite loop when done.

Sample content: we emit a 4-period square wave (MSB alternates every
8 frames) so the captured WAV has non-zero energy — identifiable as
"produced a tone".

Cortex-M0+ instruction notes:
  - No MOVW/MOVT; use LDR [PC, #imm8] for 32-bit immediates.
  - STR Rt, [Rn, #imm5] is word-scaled (range 0..124).
  - All control flow via B / BNE / BL.
"""

import struct
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

FLASH_BASE = 0x10000000
SRAM_BASE = 0x20000000
SRAM_SIZE = 264 * 1024
STACK_TOP = SRAM_BASE + SRAM_SIZE

# I2S pins (must match crates/mdpicoem-harness/src/picogus_pins.rs).
I2S_DOUT = 16
I2S_BCLK = 17
I2S_LRCLK = 18
I2S_MASK = (1 << I2S_DOUT) | (1 << I2S_BCLK) | (1 << I2S_LRCLK)
BCLK_MASK = 1 << I2S_BCLK
LRCLK_MASK = 1 << I2S_LRCLK
DOUT_MASK = 1 << I2S_DOUT

# SIO registers (RP2040 SIO base 0xD0000000).
SIO_BASE = 0xD0000000
SIO_GPIO_OUT_SET = 0x014  # atomic set
SIO_GPIO_OUT_CLR = 0x018  # atomic clear
SIO_GPIO_OUT_XOR = 0x01C
SIO_GPIO_OE_SET = 0x024   # output-enable atomic set

NUM_FRAMES = 64  # LRCLK half-frames to emit (32 stereo samples)


# ---------------------------------------------------------------------------
# Thumb-16 encoding helpers (Cortex-M0+ subset)
# ---------------------------------------------------------------------------

def th_movs_imm8(rd, imm8):
    """MOVS Rd, #imm8 — T1."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)


def th_lsls_imm5(rd, rm, imm5):
    """LSLS Rd, Rm, #imm5 — T1."""
    assert 0 <= rd <= 7 and 0 <= rm <= 7 and 0 <= imm5 <= 31
    return struct.pack('<H', (imm5 << 6) | (rm << 3) | rd)


def th_ldr_pc(rt, imm8):
    """LDR Rt, [PC, #imm8*4] — T1 literal load."""
    assert 0 <= rt <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x4800 | (rt << 8) | imm8)


def th_str_imm5(rt, rn, imm5_words):
    """STR Rt, [Rn, #imm5*4] — T1."""
    assert 0 <= rt <= 7 and 0 <= rn <= 7 and 0 <= imm5_words <= 31
    return struct.pack('<H', 0x6000 | (imm5_words << 6) | (rn << 3) | rt)


def th_subs_imm8(rd, imm8):
    """SUBS Rd, Rd, #imm8 — T2."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3800 | (rd << 8) | imm8)


def th_adds_imm8(rd, imm8):
    """ADDS Rd, Rd, #imm8 — T2."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x3000 | (rd << 8) | imm8)


def th_cmp_imm8(rn, imm8):
    """CMP Rn, #imm8."""
    assert 0 <= rn <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2800 | (rn << 8) | imm8)


def th_bne(offset):
    """BNE label — T1; offset from PC+4 in bytes, range -256..+254 even."""
    assert -256 <= offset <= 254 and offset % 2 == 0
    imm8 = (offset >> 1) & 0xFF
    return struct.pack('<H', 0xD100 | imm8)


def th_bl(offset):
    """B label — T2; offset from PC+4, range -2048..+2046 even."""
    assert -2048 <= offset <= 2046 and offset % 2 == 0
    imm11 = (offset >> 1) & 0x7FF
    return struct.pack('<H', 0xE000 | imm11)


def th_nop():
    return struct.pack('<H', 0xBF00)


# ---------------------------------------------------------------------------
# Firmware assembly
# ---------------------------------------------------------------------------

def build_chime():
    """
    Emits an I2S waveform: for each half-frame we pre-set DOUT to 1 or 0
    (based on a slow counter bit so samples vary over time), then pulse
    BCLK 32 times with DOUT held stable. At the end of each half-frame
    we toggle LRCLK.

    Registers:
      r0 = scratch
      r1 = BCLK mask only (= 0x20000) — used for BCLK-only SET/CLR
      r2 = LRCLK mask (= 0x40000)     — used for LRCLK XOR
      r3 = bit counter
      r4 = SIO_BASE
      r5 = I2S_MASK (= 0x70000) — for initial OE_SET + one-shot CLR
      r6 = outer half-frame counter
      r7 = DOUT mask (= 0x10000) — used for DOUT SET/CLR
    """
    code = b''

    # --- Prologue: load SIO_BASE + constants ---
    ldr_sio_fixup = len(code)
    code += th_ldr_pc(4, 0)               # LDR r4, [PC, #?]  (patched below)

    # r5 = I2S_MASK = 0x70000
    code += th_movs_imm8(5, 7)
    code += th_lsls_imm5(5, 5, 16)

    # r7 = DOUT mask = 0x10000
    code += th_movs_imm8(7, 1)
    code += th_lsls_imm5(7, 7, 16)

    # r1 = BCLK mask = 0x20000
    code += th_movs_imm8(1, 1)
    code += th_lsls_imm5(1, 1, 17)

    # r2 = LRCLK mask = 0x40000
    code += th_movs_imm8(2, 1)
    code += th_lsls_imm5(2, 2, 18)

    # SIO_GPIO_OE_SET = I2S_MASK (enable outputs on GPIO 16/17/18)
    code += th_str_imm5(5, 4, 9)          # STR r5, [r4, #0x24]

    # SIO_GPIO_OUT_CLR = I2S_MASK (start with all pins low)
    code += th_str_imm5(5, 4, 6)          # STR r5, [r4, #0x18]

    # r6 = outer loop counter
    code += th_movs_imm8(6, NUM_FRAMES)

    # ==== Outer loop: each iteration emits one half-frame ====
    outer_top = len(code)

    # Compute DOUT bit from r6's bit 2 (slow square wave: period 8 half-
    # frames = 4 stereo samples per square-wave cycle). MOV r0, r6;
    # LSLS r0, r0, #29; LSRS r0, r0, #31 → r0 = (r6 >> 2) & 1.
    code += bytes.fromhex('30 46')        # MOV r0, r6  (T1 MOV low-to-low)
    code += bytes.fromhex('40 07')        # LSLS r0, r0, #29  -- isolate bit 2 to bit 31
    code += bytes.fromhex('c0 0f')        # LSRS r0, r0, #31  -- shift to bit 0

    # r0 now ∈ {0, 1}. Shift into DOUT position (bit 16).
    code += th_lsls_imm5(0, 0, 16)        # r0 <<= 16

    # Pre-clear DOUT, then conditionally set it.
    code += th_str_imm5(7, 4, 6)          # SIO_OUT_CLR = DOUT mask (r7)
    code += th_str_imm5(0, 4, 5)          # SIO_OUT_SET = r0 (DOUT bit or 0)

    # ==== BCLK burst: 32 rising+falling edges ====
    code += th_movs_imm8(3, 32)           # r3 = 32 BCLK cycles

    bclk_loop_top = len(code)
    code += th_str_imm5(1, 4, 5)          # SIO_OUT_SET = BCLK (r1) — BCLK rises
    code += th_str_imm5(1, 4, 6)          # SIO_OUT_CLR = BCLK (r1) — BCLK falls

    code += th_subs_imm8(3, 1)
    bne_pos = len(code)
    code += th_bne(bclk_loop_top - (bne_pos + 4))

    # Toggle LRCLK after the half-frame.
    code += th_str_imm5(2, 4, 7)          # SIO_OUT_XOR = LRCLK (r2)

    # Outer loop footer.
    code += th_subs_imm8(6, 1)
    bne_pos = len(code)
    outer_delta = outer_top - (bne_pos + 4)
    assert -256 <= outer_delta <= 254, f"outer BNE out of range: {outer_delta}"
    code += th_bne(outer_delta)

    # Halt (self-loop) once the burst completes.
    code += struct.pack('<H', 0xE7FE)     # B .

    # --- Literal pool (word-aligned) ---
    while len(code) % 4 != 0:
        code += th_nop()

    literal_offset = len(code)
    code += struct.pack('<I', SIO_BASE)

    ldr_pc_aligned = (ldr_sio_fixup + 4) & ~3
    imm8_bytes = literal_offset - ldr_pc_aligned
    assert imm8_bytes >= 0 and imm8_bytes % 4 == 0, \
        f"SIO literal before LDR? literal_offset={literal_offset:#x}, ldr_pc_aligned={ldr_pc_aligned:#x}"
    imm8 = imm8_bytes // 4
    assert 0 <= imm8 <= 255, f"LDR offset out of range: {imm8}"

    old = struct.unpack('<H', code[ldr_sio_fixup:ldr_sio_fixup + 2])[0]
    patched = (old & ~0xFF) | imm8
    code = code[:ldr_sio_fixup] + struct.pack('<H', patched) + code[ldr_sio_fixup + 2:]

    return code


def build_bootrom():
    """
    Minimal ROM: SP at offset 0, reset vector at offset 4, rest = B . trap.
    Identical layout to gen_blinky.py's bootrom.
    """
    rom = b''
    rom += struct.pack('<I', STACK_TOP)
    rom += struct.pack('<I', FLASH_BASE | 1)
    trap = struct.pack('<H', 0xE7FE)
    while len(rom) < 16 * 1024:
        rom += trap
    return rom


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent

    chime = build_chime()
    bootrom = build_bootrom()

    chime_path = out_dir / 'i2s_chime.bin'
    bootrom_path = out_dir / 'bootrom-i2s-chime.bin'

    chime_path.write_bytes(chime)
    bootrom_path.write_bytes(bootrom)

    print(f"Wrote {chime_path} ({len(chime)} bytes)")
    print(f"  reset handler:  {FLASH_BASE:#010x}")
    print(f"  frames emitted: {NUM_FRAMES}")
    print(f"Wrote {bootrom_path} ({len(bootrom)} bytes, reuses gen_blinky layout)")


if __name__ == '__main__':
    main()
