#!/usr/bin/env python3
"""
Generate a minimal RP2350 ARM firmware exercising TIMER0 alarm + busy_wait.

Phase 1 corpus entry (HLD V5 §3). Loaded into SRAM at 0x2000_0000 via
`Emulator::load_image` (bypassing the bootrom), relying on the
emulator's HLD V5 §5.7 post-bootrom state — TICKS.TIMER0.CYCLES = 12,
TIMER0 released from RESETS, clk_sys = 150 MHz, clk_ref = 12 MHz.

Layout at SRAM base 0x2000_0000:
  0x0000: Vector table (16 standard Cortex-M33 entries; only SP / Reset
          / HardFault are used — the rest point to the fault trap).
  0x0040: Reset handler (configure TIMER0, loop alarm + increment).
  0x00XX: Fault trap (B .).

The firmware:
  1. Ensures TIMER0 released from RESETS (belt-and-braces; EMU already
     starts released).
  2. Ensures TICKS.TIMER0.CTRL.ENABLE = 1 (post-bootrom state disables
     TIMER0 TICKS until firmware enables it — the SDK does this too).
  3. In a loop:
     - Read TIMELR into R0.
     - R0 += 1000 (next alarm at +1000 µs).
     - Write R0 to TIMER0.ALARM_0.
     - Poll INTR for bit 0.
     - W1C INTR bit 0.
     - Increment counter at 0x2000_3000 (chosen to sit above the vector
       table + code region, with room for test-side polling).
     - Loop back (forever).

The test harness polls the counter cell at 0x2000_3000 and asserts it
advances within a wall-clock budget.

References:
  - RP2350 Datasheet §12.8 (TIMER), §8.5 (TICKS).
  - `crates/mdrp2350/src/peripherals/{ticks,timer}.rs` for the emulator
    model the firmware runs against.
  - `gen_blinky.py`, `gen_benchmark.py` for Thumb-2 encoding patterns.
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

# Counter cell. Must live above the code region + below STACK_TOP.
# 0x2000_3000 = offset 0x3000; well past the ~512-byte firmware body.
COUNTER_ADDR = 0x20003000

# RP2350 peripheral addresses.
RESETS_BASE = 0x40020000
RESETS_RESET_CLR = RESETS_BASE + 0x3000  # CLR alias (+0x3000)
RESET_TIMER0_BIT = 1 << 23

TICKS_BASE = 0x40108000
TICKS_TIMER0_CTRL = TICKS_BASE + 0x18
TICKS_CTRL_ENABLE = 1 << 0

TIMER0_BASE = 0x400B0000
TIMER0_TIMELR = 0x0C   # offset from TIMER0_BASE
TIMER0_ALARM0 = 0x10
TIMER0_INTR   = 0x3C
TIMER0_INTE   = 0x40
TIMER0_INTS   = 0x48

CODE_OFFSET = 0x40  # code immediately after 64-byte vector table

# =============================================================================
# Thumb-2 encoding helpers (cribbed from gen_blinky.py)
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

def thumb_movs_imm(rd, imm8):
    """MOVS Rd, #imm8 (T1)."""
    assert 0 <= rd <= 7 and 0 <= imm8 <= 255
    return struct.pack('<H', 0x2000 | (rd << 8) | imm8)

def thumb_adds_reg(rd, rn, rm):
    """ADDS Rd, Rn, Rm (T1)."""
    assert 0 <= rd <= 7 and 0 <= rn <= 7 and 0 <= rm <= 7
    return struct.pack('<H', 0x1800 | (rm << 6) | (rn << 3) | rd)

def thumb_tst_reg(rn, rm):
    """TST Rn, Rm (T1)."""
    assert 0 <= rn <= 7 and 0 <= rm <= 7
    return struct.pack('<H', 0x4200 | (rm << 3) | rn)

def thumb_beq(offset):
    """BEQ label (T1). offset in signed bytes from PC+4."""
    assert -256 <= offset <= 254 and (offset % 2 == 0)
    return struct.pack('<H', 0xD000 | ((offset >> 1) & 0xFF))

def thumb_b(offset):
    """B label (T1 unconditional). offset in signed bytes from PC+4."""
    assert -2048 <= offset <= 2046 and (offset % 2 == 0)
    return struct.pack('<H', 0xE000 | ((offset >> 1) & 0x7FF))

def thumb_bkpt_0():
    """BKPT #0 (T1)."""
    return struct.pack('<H', 0xBE00)

# =============================================================================
# Firmware body
# =============================================================================

def build_reset_handler():
    """
    Configure TIMER0, run an alarm-fire loop, increment a counter.

    Register allocation (caller-saved, kept live across the whole loop):
      r3 — TIMER0_BASE (0x400B_0000)
      r4 — TICKS_TIMER0_CTRL (0x4010_8018)
      r5 — RESETS_RESET_CLR (0x4002_3000)
      r6 — counter cell address (0x2000_3000)
      r7 — counter scratch (read-modify-write)
      r0 — TIMELR read / ALARM target
      r1 — constant 1 (INTE, INTR W1C, TICKS ENABLE, RESETS mask word)
      r2 — INTS poll scratch
    """
    code = b''

    # --- Step 1: Release TIMER0 from RESETS (belt-and-braces; EMU
    # already starts released per HLD V5 §5.7). ---
    code += thumb_mov_imm32(5, RESETS_RESET_CLR)  # r5 = RESETS.CLR alias
    code += thumb_mov_imm32(1, RESET_TIMER0_BIT)  # r1 = bit 23 mask
    code += thumb_str_imm(1, 5, 0)                # *r5 = r1 (clear TIMER0 reset)

    # --- Step 2: Enable TICKS.TIMER0 ---
    code += thumb_mov_imm32(4, TICKS_TIMER0_CTRL) # r4 = TICKS.TIMER0.CTRL
    code += thumb_movs_imm(1, TICKS_CTRL_ENABLE)  # r1 = 1
    code += thumb_str_imm(1, 4, 0)                # CTRL = 1

    # --- Step 3: Preload TIMER0_BASE, counter addr, INTE=1 ---
    code += thumb_mov_imm32(3, TIMER0_BASE)       # r3 = 0x400B_0000
    code += thumb_mov_imm32(6, COUNTER_ADDR)      # r6 = 0x2000_3000
    # Enable TIMER0 alarm-0 interrupt (INTE bit 0). r1 already = 1 so
    # this is a small code-size win over reloading r1.
    code += thumb_str_imm(1, 3, TIMER0_INTE)      # TIMER0.INTE = 1

    # --- Step 4: Main loop ---
    # loop_top:
    #   r0 = TIMER0.TIMELR
    #   r0 += 1000
    #   TIMER0.ALARM_0 = r0   (arms alarm 0)
    # poll:
    #   r2 = TIMER0.INTS
    #   tst r2, r1            (Z=1 if INTS.bit0==0)
    #   beq poll
    #   TIMER0.INTR = r1      (W1C — clear latch)
    #   r7 = *r6 ; r7 += 1 ; *r6 = r7
    #   b loop_top
    loop_top_off = len(code)
    # r0 = TIMELR
    code += thumb_ldr_imm(0, 3, TIMER0_TIMELR)
    # r1 still = 1 — we need 1000 in a different reg. Load r2 = 1000
    # temporarily.
    code += thumb_mov_imm32(2, 1000)
    # r0 = r0 + r2
    code += thumb_adds_reg(0, 0, 2)
    # Store to ALARM_0
    code += thumb_str_imm(0, 3, TIMER0_ALARM0)
    # Re-load r1 = 1 (adds_reg doesn't touch r1, so this is redundant
    # except that r2 got clobbered — we kept r1 intact, so drop the
    # movs; keep the contract explicit for readability).
    # poll_top:
    poll_top_off = len(code)
    code += thumb_ldr_imm(2, 3, TIMER0_INTS)
    code += thumb_tst_reg(2, 1)
    # BEQ poll_top: offset = (poll_top - (PC+4)) bytes
    # Current instruction is at len(code); PC at BEQ = start + len(code)
    # PC+4 = start + len(code) + 4
    # target = poll_top_off
    # offset = poll_top_off - (len(code) + 4)
    beq_bytes = poll_top_off - (len(code) + 4)
    code += thumb_beq(beq_bytes)
    # W1C INTR bit 0
    code += thumb_str_imm(1, 3, TIMER0_INTR)
    # Increment counter
    code += thumb_ldr_imm(7, 6, 0)
    code += thumb_movs_imm(2, 1)  # r2 = 1 scratch
    code += thumb_adds_reg(7, 7, 2)
    code += thumb_str_imm(7, 6, 0)
    # Branch back to loop_top
    b_bytes = loop_top_off - (len(code) + 4)
    code += thumb_b(b_bytes)

    return code

def build_fault_handler():
    """Infinite loop (B .) for fault trap."""
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

    used = len(vectors)
    padding = reset_offset - used
    assert padding >= 0, f"Reset offset {reset_offset:#x} too small, need {used:#x}"

    binary = vectors + (b'\x00' * padding) + reset_code + fault_code

    # Pad to 256-byte alignment.
    while len(binary) % 256 != 0:
        binary += b'\x00'

    outpath = sys.argv[1] if len(sys.argv) > 1 else 'hello_timer.bin'
    with open(outpath, 'wb') as f:
        f.write(binary)

    print(f"Generated {outpath}: {len(binary)} bytes")
    print(f"  SRAM base:       {SRAM_BASE:#010x}")
    print(f"  Stack top:       {STACK_TOP:#010x}")
    print(f"  Reset handler:   {SRAM_BASE + reset_offset:#010x} (vector: {reset_vector:#010x})")
    print(f"  Fault handler:   {SRAM_BASE + fault_offset:#010x} (vector: {fault_vector:#010x})")
    print(f"  Counter cell:    {COUNTER_ADDR:#010x}")
    print(f"  Code region:     offset {reset_offset:#05x}-{reset_offset + len(reset_code) - 1:#05x}")

if __name__ == '__main__':
    main()
