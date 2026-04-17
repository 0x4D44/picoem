#!/usr/bin/env python3
"""
Generate a minimal RP2350 RISC-V (Hazard3) blinky binary for emulator testing.

Layout at SRAM base 0x20000000:
  0x000: Reset entry (firmware start — Hazard3 resets with PC = 0x20000000
         for images loaded via `Emulator::load_image`, per HLD V6 §8 Q1).
  0x0xx: Pad / GPIO config + blink main loop (bare rv32i, no C/M/A).

The binary blinks GPIO25 (Pico 2 onboard LED) by writing the SIO
GPIO_OUT XOR alias. The delay loop is a simple countdown. No toolchain
dependency — each instruction is hand-encoded here with a disassembly
comment. Matches the style of `gen_blinky.py`.

Bare-asm landmines (per HLD V6 §4.5 + §6 P5):
  (a) `gp` is left at 0 — we never emit linker-relaxation-dependent code.
  (b) Vectored `mtvec` table not used — blinky never takes a trap, so
      `mtvec` is left at its reset value (`0x0000_1FFD`).
  (c) `mscratch` unused — no trap entry path.
  (d) `mcountinhibit` untouched — blinky never reads mcycle/minstret.

Optional round-trip: if `riscv32-unknown-elf-objdump` is on PATH the
script disassembles the emitted binary and prints it for eyeball
comparison with the comment track. Absent toolchain → skipped silently.

ISA scope: rv32i only. No compressed (C), multiply (M), or atomics (A)
are required for blinky. Keeping it rv32i side-steps the Zcmp ↔ C
encoding collision called out in HLD V6 §4.5.

References:
  - RP2350 Datasheet Section 3.8 (Hazard3 core), Section 8 (SIO/pads).
  - HLD V6: `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`.
"""

import struct
import subprocess
import sys
import tempfile
import os

# =============================================================================
# Constants
# =============================================================================

SRAM_BASE = 0x20000000
LOAD_ADDR = SRAM_BASE  # HLD §8 Q1 — firmware linked at SRAM base.

# RP2350 peripheral addresses (same map as the Arm blinky — peripherals are
# arch-shared per HLD §5).
SIO_BASE = 0xD0000000
# Per RP2350 datasheet 3.1.7.3 SIO register map — GPIO_OUT at +0x010,
# GPIO_OUT_SET +0x018, GPIO_OUT_CLR +0x020, GPIO_OUT_XOR +0x028.
SIO_GPIO_OUT = SIO_BASE + 0x010
SIO_GPIO_OUT_SET = SIO_BASE + 0x018
SIO_GPIO_OUT_CLR = SIO_BASE + 0x020
SIO_GPIO_OUT_XOR = SIO_BASE + 0x028
SIO_GPIO_OE      = SIO_BASE + 0x030
SIO_GPIO_OE_SET  = SIO_BASE + 0x038

IO_BANK0_BASE = 0x40028000
IO_BANK0_GPIO25_CTRL = IO_BANK0_BASE + 0x0CC

PADS_BANK0_BASE = 0x40038000
PADS_BANK0_GPIO25 = PADS_BANK0_BASE + 0x68

LED_PIN = 25
LED_MASK = 1 << LED_PIN

# =============================================================================
# RV32I encoder helpers — every emitter returns 4 bytes in little-endian.
# =============================================================================

def rv_word(insn: int) -> bytes:
    assert 0 <= insn <= 0xFFFFFFFF
    return struct.pack('<I', insn)

def r_type(funct7, rs2, rs1, funct3, rd, opcode) -> int:
    return ((funct7 & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | \
           ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | \
           ((rd & 0x1F) << 7) | (opcode & 0x7F)

def i_type(imm12, rs1, funct3, rd, opcode) -> int:
    imm = imm12 & 0xFFF
    return (imm << 20) | ((rs1 & 0x1F) << 15) | \
           ((funct3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

def s_type(imm12, rs2, rs1, funct3, opcode) -> int:
    imm = imm12 & 0xFFF
    imm_hi = (imm >> 5) & 0x7F
    imm_lo = imm & 0x1F
    return (imm_hi << 25) | ((rs2 & 0x1F) << 20) | \
           ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | \
           (imm_lo << 7) | (opcode & 0x7F)

def b_type(imm13, rs2, rs1, funct3, opcode) -> int:
    # imm13 is signed, LSB is implicit zero.
    assert imm13 % 2 == 0, "B-type immediate must be even"
    assert -4096 <= imm13 <= 4095
    v = imm13 & 0x1FFE  # 13-bit range, low bit zero
    # Bits: [12|10:5|4:1|11] at [31|30:25|11:8|7]
    b12   = (v >> 12) & 0x1
    b10_5 = (v >> 5) & 0x3F
    b4_1  = (v >> 1) & 0xF
    b11   = (v >> 11) & 0x1
    return (b12 << 31) | (b10_5 << 25) | ((rs2 & 0x1F) << 20) | \
           ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | \
           (b4_1 << 8) | (b11 << 7) | (opcode & 0x7F)

def u_type(imm20, rd, opcode) -> int:
    # imm20 is the upper 20 bits of a 32-bit value (bits [31:12]).
    return ((imm20 & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

def j_type(imm21, rd, opcode) -> int:
    assert imm21 % 2 == 0
    assert -(1 << 20) <= imm21 < (1 << 20)
    v = imm21 & 0x1FFFFE
    # Bits: [20|10:1|11|19:12] at [31|30:21|20|19:12]
    b20   = (v >> 20) & 0x1
    b10_1 = (v >> 1) & 0x3FF
    b11   = (v >> 11) & 0x1
    b19_12 = (v >> 12) & 0xFF
    return (b20 << 31) | (b10_1 << 21) | (b11 << 20) | \
           (b19_12 << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)

# --- Concrete instructions (one helper per mnemonic used) ---

def lui(rd, imm20) -> bytes:
    """LUI rd, imm20 — rd = imm20 << 12."""
    return rv_word(u_type(imm20, rd, 0b0110111))

def addi(rd, rs1, imm12) -> bytes:
    """ADDI rd, rs1, imm — rd = rs1 + sext(imm12)."""
    assert -2048 <= imm12 <= 2047
    return rv_word(i_type(imm12, rs1, 0b000, rd, 0b0010011))

def sw(rs2, imm12, rs1) -> bytes:
    """SW rs2, imm(rs1) — mem[rs1 + sext(imm12)] = rs2[31:0]."""
    assert -2048 <= imm12 <= 2047
    return rv_word(s_type(imm12, rs2, rs1, 0b010, 0b0100011))

def bne(rs1, rs2, imm13) -> bytes:
    """BNE rs1, rs2, pc+imm13."""
    return rv_word(b_type(imm13, rs2, rs1, 0b001, 0b1100011))

def beq(rs1, rs2, imm13) -> bytes:
    """BEQ rs1, rs2, pc+imm13."""
    return rv_word(b_type(imm13, rs2, rs1, 0b000, 0b1100011))

def jal(rd, imm21) -> bytes:
    """JAL rd, pc+imm21 — rd = pc+4; pc += imm21."""
    return rv_word(j_type(imm21, rd, 0b1101111))

def csrrs(rd, csr, rs1) -> bytes:
    """CSRRS rd, csr, rs1 — read csr to rd and OR rs1 into csr.
    With rs1=x0 this is a plain read (no write side effect)."""
    assert 0 <= csr <= 0xFFF
    insn = ((csr & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | (0b010 << 12) | \
           ((rd & 0x1F) << 7) | 0b1110011
    return rv_word(insn)

# =============================================================================
# 32-bit immediate loader
# =============================================================================

def li32(rd: int, value: int) -> bytes:
    """Load a 32-bit constant into rd via LUI + ADDI, correcting for ADDI's
    sign extension. One or two instructions depending on the low 12 bits.

    Equivalent to the GNU `li` macro."""
    value &= 0xFFFFFFFF
    lo = value & 0xFFF
    # ADDI sign-extends imm12, so if bit 11 is set the effective addend is
    # negative — pre-bias the high 20 bits by +1 to compensate.
    if lo & 0x800:
        hi = ((value >> 12) + 1) & 0xFFFFF
        lo_signed = lo - 0x1000
    else:
        hi = (value >> 12) & 0xFFFFF
        lo_signed = lo

    out = b''
    if hi != 0:
        out += lui(rd, hi)
        if lo_signed != 0:
            out += addi(rd, rd, lo_signed)
    else:
        # Pure small immediate. ADDI from x0 is the canonical `li`.
        out += addi(rd, 0, lo_signed)
    return out

# =============================================================================
# Build the blinky image
# =============================================================================

class Asm:
    """Collect emitted bytes + a parallel disassembly comment track so the
    round-trip step can diff them against objdump output."""

    def __init__(self, base_addr: int):
        self.code = bytearray()
        self.base = base_addr
        self.annotations = []  # list of (offset, text)

    def pc(self) -> int:
        return self.base + len(self.code)

    def emit(self, bytes_: bytes, comment: str):
        self.annotations.append((len(self.code), comment))
        self.code.extend(bytes_)

    def emit_li32(self, rd: int, val: int, label: str):
        """Emit the 1-or-2 instructions of `li rd, val` with a combined
        comment anchored at the first instruction."""
        before = len(self.code)
        self.annotations.append((before, f"li  x{rd}, {val:#x}  ({label})"))
        self.code.extend(li32(rd, val))
        # Back-fill any second instruction without an extra annotation —
        # the single combined comment describes the pair.

def build_image() -> Asm:
    asm = Asm(LOAD_ADDR)

    # Register allocation:
    #   x1  = mhartid (read once at boot)
    #   x5  = base address scratch (reused across MMIO writes)
    #   x6  = value scratch         (reused)
    #   x10 = SIO_GPIO_OUT_XOR      (hot path — loop-invariant)
    #   x11 = LED_MASK              (hot path — loop-invariant)
    #   x12 = delay countdown
    #
    # Dual-core guard: RP2350 boots both harts at the same PC. Without a
    # hart-id dispatch, core 1 would race core 0's XOR toggles and cancel
    # them out (each core XOR-writes the same mask on each loop iteration,
    # so the net edge rate on `gpio_in` collapses to zero). `mhartid` !=
    # 0 parks core 1 in an infinite self-branch before any GPIO setup.

    # --- Hart-id guard: core 1 parks ---------------------------------------
    #
    #   0: csrrs x1, mhartid, x0  ; x1 = hart id
    #   4: beq   x1, x0, +8       ; core 0 -> skip park (taken)
    #   8: jal   x0, 0            ; core 1 -> infinite self-loop
    #  12: (blinky setup begins)
    #
    # RISC-V branch offsets are relative to the branch instruction's own
    # PC (not PC+4). BEQ at PC=4 with imm=+8 therefore targets PC+8 = 12,
    # which is the first setup instruction immediately past the park JAL.
    asm.emit(csrrs(1, 0xF14, 0),
             "csrrs x1, mhartid, x0  ; x1 = hart id")
    asm.emit(beq(1, 0, 8),
             "beq  x1, x0, +8        ; core 0 skips park")
    asm.emit(jal(0, 0),
             "jal  x0, 0             ; park (core 1, infinite self-loop)")

    # --- Step 1: PADS_BANK0_GPIO25 = 0x56 ---------------------------------
    #   Bit layout for PADS_BANK0.GPIOn:
    #     bit 7 OD (output disable)  — cleared
    #     bit 6 IE (input enable)    — set (1, unchanged from default)
    #     bit 4 DRIVE[1:0]           — 01 = 4mA
    #     bit 2 PDE (pull-down en)   — set (benign for output)
    #     bit 1 SCHMITT              — set
    #   0x56 = 0101_0110 — OD clear, IE set, 4mA drive, SCHMITT on.
    asm.emit_li32(5, PADS_BANK0_GPIO25, "PADS_BANK0_GPIO25 addr")
    asm.emit(addi(6, 0, 0x56),
             "addi x6, x0, 0x56  ; pad config (OD=0, IE=1, drive=4mA)")
    asm.emit(sw(6, 0, 5),
             "sw   x6, 0(x5)     ; *PADS_BANK0_GPIO25 = 0x56")

    # --- Step 2: IO_BANK0_GPIO25_CTRL = 5 (FUNCSEL = SIO) ------------------
    asm.emit_li32(5, IO_BANK0_GPIO25_CTRL, "IO_BANK0_GPIO25_CTRL addr")
    asm.emit(addi(6, 0, 5),
             "addi x6, x0, 5     ; FUNCSEL = SIO")
    asm.emit(sw(6, 0, 5),
             "sw   x6, 0(x5)     ; *GPIO25_CTRL = 5")

    # --- Step 3: SIO_GPIO_OE_SET |= (1 << 25) -----------------------------
    #   Writing to GPIO_OE_SET is a 1-hot set — the ones we write turn on.
    asm.emit_li32(5, SIO_GPIO_OE_SET, "SIO_GPIO_OE_SET addr")
    asm.emit_li32(6, LED_MASK, "LED bit mask (1<<25)")
    asm.emit(sw(6, 0, 5),
             "sw   x6, 0(x5)     ; enable GPIO25 output")

    # --- Step 4: Pre-load hot-path constants ------------------------------
    #   x10 = SIO_GPIO_OUT_XOR, x11 = LED_MASK so the inner loop is just
    #   delay + one SW + branch.
    asm.emit_li32(10, SIO_GPIO_OUT_XOR, "SIO_GPIO_OUT_XOR addr (hot)")
    asm.emit_li32(11, LED_MASK, "LED mask (hot)")

    # --- Step 5: main blink loop ------------------------------------------
    #
    #   loop_top:
    #       li   x12, DELAY_COUNT
    #   delay:
    #       addi x12, x12, -1
    #       bne  x12, x0, delay
    #       sw   x11, 0(x10)       # XOR toggle GPIO25
    #       jal  x0, loop_top      # unconditional
    #
    # Delay count: firmware time units (~instructions). Hazard3 emulator
    # runs ~1 cycle/insn; choose a count that fits in a 12-bit immediate
    # so the inner loop stays flat (3 instructions — addi + bne + fall
    # through). 0x7FF = 2047 iterations is plenty short per toggle, lots
    # of headroom to observe many edges in 1M cycles.
    DELAY_COUNT = 0x7FF

    loop_top = asm.pc()
    asm.emit(addi(12, 0, DELAY_COUNT),
             f"addi x12, x0, {DELAY_COUNT:#x}  ; delay count")

    delay_top = asm.pc()
    asm.emit(addi(12, 12, -1),
             "addi x12, x12, -1  ; delay--")

    # BNE offset = delay_top - pc_of_bne
    bne_pc = asm.pc()
    bne_offset = delay_top - bne_pc
    asm.emit(bne(12, 0, bne_offset),
             f"bne  x12, x0, {bne_offset:+d}  ; loop while x12 != 0")

    asm.emit(sw(11, 0, 10),
             "sw   x11, 0(x10)   ; *SIO_GPIO_OUT_XOR = LED_MASK (toggle)")

    jal_pc = asm.pc()
    jal_offset = loop_top - jal_pc
    asm.emit(jal(0, jal_offset),
             f"jal  x0, {jal_offset:+d}  ; -> loop_top")

    return asm

# =============================================================================
# Optional objdump round-trip
# =============================================================================

def try_objdump(raw: bytes, base: int) -> None:
    """If riscv32-unknown-elf-objdump is present, print its disassembly
    for eyeball comparison with the emit-track comments. No assertion —
    this is a dev aid, not a CI gate."""
    tool = "riscv32-unknown-elf-objdump"
    try:
        subprocess.run([tool, "--version"], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        print(f"(skipping objdump round-trip — {tool} not on PATH)")
        return

    # objdump needs an ELF or raw with --target. Use raw binary form:
    with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as f:
        f.write(raw)
        tmp = f.name
    try:
        result = subprocess.run(
            [tool,
             "-D", "-b", "binary",
             "-m", "riscv:rv32",
             "-M", "no-aliases,numeric",
             f"--adjust-vma={base:#x}",
             tmp],
            capture_output=True, text=True, check=True,
        )
        print("--- objdump -D round-trip ---")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"(objdump invocation failed: {e.stderr})", file=sys.stderr)
    finally:
        os.unlink(tmp)

# =============================================================================
# Entry point
# =============================================================================

def main() -> int:
    asm = build_image()

    # Pad to a nice 256-byte boundary — matches gen_blinky.py style.
    while len(asm.code) % 256 != 0:
        asm.code.append(0x00)

    outpath = sys.argv[1] if len(sys.argv) > 1 else 'blinky-riscv.bin'
    with open(outpath, 'wb') as f:
        f.write(asm.code)

    print(f"Generated {outpath}: {len(asm.code)} bytes")
    print(f"  Load address:  {LOAD_ADDR:#010x}")
    print(f"  Reset PC:      {LOAD_ADDR:#010x} (set by Emulator::reset, HLD §4.3)")
    print(f"  LED pin:       GPIO{LED_PIN}")
    print()
    print("--- instruction trace -------------------------------------------")
    for off, text in asm.annotations:
        addr = LOAD_ADDR + off
        # Width-guard: if the next annotation starts within 4 bytes of this
        # one, this is a single instruction; otherwise it's an `li` pair.
        next_off = next((o for o, _ in asm.annotations if o > off), len(asm.code))
        nbytes = min(next_off - off, 8)
        hx = ' '.join(f"{b:02x}" for b in asm.code[off:off+nbytes])
        print(f"  {addr:08x}:  {hx:<24}  {text}")

    try_objdump(bytes(asm.code), LOAD_ADDR)
    return 0

if __name__ == '__main__':
    sys.exit(main())
