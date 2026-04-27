// P3 per-op semantic tests. Covers RV32M (mul/div), RV32A (atomics),
// and RV32C (compressed). The RV32I + Zicsr + Zifencei baseline stays
// in `tests_p2.rs`.
//
// Golden vectors for M-extension overflow / div-by-zero come straight
// from RV-priv §7.1–§7.2:
//   DIV/DIVU by 0 -> quotient = all-ones (all bits set).
//   DIV overflow (INT_MIN / -1) -> quotient = INT_MIN, remainder = 0.
//   REM/REMU by 0 -> remainder = dividend.
//
// A-extension semantics come from HLD §4.7:
//   reservable region = 0x2000_0000..0x2008_2000 (the 520 KB SRAM).
//   LR outside reservable = silent no-op (no trap, no reservation).
//   SC outside reservable = silent write-skipped (rd=1, no trap).
//   AMO*.W outside reservable = mcause=7.
//
// C-extension vectors come from the RISC-V unprivileged spec
// compressed-instruction table. Each test plants a 16-bit word at the
// reset PC and single-steps.

use super::Hazard3;
use super::decode::{self, AmoKind, MulDivKind, Op};
use crate::Bus;

// ---------- helpers ----------

use super::tests_common::{fresh, write_hw, write_insn};

// R-type encoder — matches tests_p2.
fn enc_r(opcode: u32, rd: u8, f3: u32, rs1: u8, rs2: u8, f7: u32) -> u32 {
    (f7 << 25)
        | ((rs2 as u32) << 20)
        | ((rs1 as u32) << 15)
        | (f3 << 12)
        | ((rd as u32) << 7)
        | (opcode << 2)
        | 0b11
}

// AMO encoder: funct5 + aq + rl + rs2 + rs1 + 010 + rd + 0101111
fn enc_amo(funct5: u32, aq: bool, rl: bool, rd: u8, rs1: u8, rs2: u8) -> u32 {
    (funct5 << 27)
        | ((aq as u32) << 26)
        | ((rl as u32) << 25)
        | ((rs2 as u32) << 20)
        | ((rs1 as u32) << 15)
        | (0b010 << 12)
        | ((rd as u32) << 7)
        | (0b01_011 << 2) // OPCODE_AMO
        | 0b11
}

const OP: u32 = 0b01_100;

// -------------------------------------------------------------------
// RV32M
// -------------------------------------------------------------------

#[test]
fn mul_basic() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 7;
    c.x[2] = 6;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Mul,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 42);
}

#[test]
fn mul_wraps_low_32() {
    // 0x1_0000 * 0x1_0000 = 0x1_0000_0000 -> low 32 = 0.
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x0001_0000;
    c.x[2] = 0x0001_0000;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Mul,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0);
}

#[test]
fn mulh_signed_signed() {
    // (-2) * (-3) = 6 -> high 32 = 0
    let (mut c, mut bus) = fresh();
    c.x[1] = (-2i32) as u32;
    c.x[2] = (-3i32) as u32;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Mulh,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0);
    // INT_MIN * INT_MIN = 2^62 -> high 32 = 2^30 = 0x4000_0000.
    c.x[1] = i32::MIN as u32;
    c.x[2] = i32::MIN as u32;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Mulh,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0x4000_0000);
}

#[test]
fn mulhsu_signed_times_unsigned() {
    // (-1) * 0xFFFF_FFFF = -(2^32 - 1) = 0xFFFF_FFFF_0000_0001_...
    // Actually: i64 product of -1 * 0xFFFF_FFFF = -0xFFFF_FFFF = 0xFFFF_FFFF_0000_0001
    // high 32 = 0xFFFF_FFFF.
    let (mut c, mut bus) = fresh();
    c.x[1] = (-1i32) as u32;
    c.x[2] = 0xFFFF_FFFF;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Mulhsu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xFFFF_FFFF);
}

#[test]
fn mulhu_unsigned() {
    // 0xFFFF_FFFF * 0xFFFF_FFFF = 0xFFFF_FFFE_0000_0001 -> high = 0xFFFF_FFFE
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xFFFF_FFFF;
    c.x[2] = 0xFFFF_FFFF;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Mulhu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xFFFF_FFFE);
}

#[test]
fn div_basic_and_negative() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 20;
    c.x[2] = 3;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3] as i32, 6);
    c.x[1] = (-20i32) as u32;
    c.x[2] = 3;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3] as i32, -6);
}

#[test]
fn div_by_zero_yields_all_ones() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 42;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xFFFF_FFFF);
}

#[test]
fn div_overflow_intmin_div_minus1() {
    let (mut c, mut bus) = fresh();
    c.x[1] = i32::MIN as u32;
    c.x[2] = (-1i32) as u32;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], i32::MIN as u32);
}

#[test]
fn divu_basic_and_by_zero() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xFFFF_FFFE;
    c.x[2] = 2;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Divu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0x7FFF_FFFF);
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Divu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xFFFF_FFFF);
}

#[test]
fn rem_basic_and_overflow() {
    let (mut c, mut bus) = fresh();
    c.x[1] = (-20i32) as u32;
    c.x[2] = 3;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Rem,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3] as i32, -2, "sign of dividend");

    // Overflow -> rem = 0.
    c.x[1] = i32::MIN as u32;
    c.x[2] = (-1i32) as u32;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Rem,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0);
}

#[test]
fn rem_by_zero_returns_dividend() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xCAFE_BABE;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Rem,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xCAFE_BABE);
}

#[test]
fn remu_by_zero_returns_dividend() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xDEAD_BEEF;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Remu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xDEAD_BEEF);
}

#[test]
fn mul_decoded_from_word_and_runs_via_step() {
    // End-to-end: encode MUL, plant in SRAM, step.
    let (mut c, mut bus) = fresh();
    c.x[1] = 5;
    c.x[2] = 7;
    let insn = enc_r(OP, 3, 0b000, 1, 2, 0b000_0001);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[3], 35);
    assert_eq!(c.pc, 0x2000_0004);
}

// -------------------------------------------------------------------
// RV32A
// -------------------------------------------------------------------

#[test]
fn lr_w_records_reservation_and_loads() {
    let (mut c, mut bus) = fresh();
    bus.memory.sram_write32(0x100, 0x1234_5678);
    c.x[1] = 0x2000_0100;
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[2], 0x1234_5678);
    assert_eq!(bus.reservation[0], Some(0x2000_0100));
}

#[test]
fn sc_w_succeeds_when_reservation_matches() {
    let (mut c, mut bus) = fresh();
    bus.memory.sram_write32(0x100, 0xAAAA_AAAA);
    c.x[1] = 0x2000_0100;
    // LR
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    // SC new value
    c.x[3] = 0xDEAD_BEEF;
    c.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 4,
            rs1: 1,
            rs2: 3,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[4], 0, "SC success returns 0");
    assert_eq!(bus.memory.sram_read32(0x100), 0xDEAD_BEEF);
    assert_eq!(bus.reservation[0], None, "successful SC clears reservation");
}

#[test]
fn sc_w_fails_when_reservation_cleared_by_other_core() {
    let (mut c0, mut bus) = fresh();
    c0.x[1] = 0x2000_0200; // Core 0 LR
    c0.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(bus.reservation[0], Some(0x2000_0200));

    // Now some other master writes the same word — simulate by direct
    // bus.write32 (this is what a DMA or the other hart's ST would do).
    bus.write32(0x2000_0200, 0xF00D_CAFE, 0);

    // Reservation should be cleared.
    assert_eq!(bus.reservation[0], None);

    // SC fails (rd=1).
    c0.x[3] = 0x1234_5678;
    c0.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 4,
            rs1: 1,
            rs2: 3,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c0.x[4], 1, "SC fails after reservation cleared");
    // Memory unchanged by the failing SC.
    assert_eq!(bus.memory.sram_read32(0x200), 0xF00D_CAFE);
}

#[test]
fn lr_w_outside_reservable_silent_noop() {
    let (mut c, mut bus) = fresh();
    // XIP SRAM region — outside reservable per HLD §4.7.
    c.x[1] = 0x1C00_0000;
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    // No reservation recorded.
    assert_eq!(bus.reservation[0], None);
    // No trap.
    assert_eq!(c.csrs.mcause, 0);
    // rd untouched.
    assert_eq!(c.x[2], 0);
}

#[test]
fn sc_w_outside_reservable_silent_fail() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x1C00_0000;
    c.x[3] = 0xBEEF_BEEF;
    c.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 4,
            rs1: 1,
            rs2: 3,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[4], 1, "SC outside reservable returns 1");
    assert_eq!(c.csrs.mcause, 0, "no trap");
}

#[test]
fn amo_outside_reservable_traps_cause7() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x1C00_0000; // XIP SRAM — outside reservable
    c.x[2] = 1;
    c.execute(
        Op::Amo {
            kind: AmoKind::Add,
            rd: 3,
            rs1: 1,
            rs2: 2,
            aq: false,
            rl: false,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(
        c.csrs.mcause, 7,
        "amo outside reservable -> store access fault"
    );
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn lr_w_misaligned_traps_cause4() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_0101; // word-misaligned
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 4);
}

#[test]
fn sc_w_misaligned_traps_cause6() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_0101;
    c.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 6);
}

#[test]
fn amoadd_w_misaligned_traps_cause6() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_0101;
    c.execute(
        Op::Amo {
            kind: AmoKind::Add,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 6);
}

// All AMO ops: each sets up memory, runs the AMO, checks rd == old and
// memory == expected new.
fn amo_case(kind: AmoKind, old: u32, src: u32, expected_new: u32) {
    let (mut c, mut bus) = fresh();
    let off: u32 = 0x300;
    bus.memory.sram_write32(off, old);
    c.x[1] = 0x2000_0000 + off;
    c.x[2] = src;
    c.execute(
        Op::Amo {
            kind,
            rd: 3,
            rs1: 1,
            rs2: 2,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], old, "rd = original value");
    assert_eq!(
        bus.memory.sram_read32(off),
        expected_new,
        "mem after amo[{:?}]",
        kind,
    );
}

#[test]
fn amo_swap_add_and_or_xor() {
    amo_case(AmoKind::Swap, 0x1111_1111, 0x2222_2222, 0x2222_2222);
    amo_case(AmoKind::Add, 100, 23, 123);
    amo_case(AmoKind::And, 0xF0F0_F0F0, 0xFF00_FF00, 0xF000_F000);
    amo_case(AmoKind::Or, 0x0F0F_0F0F, 0xF0F0_F0F0, 0xFFFF_FFFF);
    amo_case(AmoKind::Xor, 0xAAAA_AAAA, 0xFFFF_FFFF, 0x5555_5555);
}

#[test]
fn amo_min_max_signed_and_unsigned() {
    // Signed: -2 min 1 -> -2.
    amo_case(AmoKind::Min, (-2i32) as u32, 1, (-2i32) as u32);
    amo_case(AmoKind::Max, (-2i32) as u32, 1, 1);
    // Unsigned: 0xFFFF_FFFF min 1 -> 1.
    amo_case(AmoKind::Minu, 0xFFFF_FFFF, 1, 1);
    amo_case(AmoKind::Maxu, 0xFFFF_FFFF, 1, 0xFFFF_FFFF);
}

#[test]
fn amo_decoded_from_word_and_runs_via_step() {
    let (mut c, mut bus) = fresh();
    bus.memory.sram_write32(0x400, 10);
    c.x[1] = 0x2000_0400;
    c.x[2] = 32;
    // AMOADD.W x3, x2, (x1). funct5=00000.
    let insn = enc_amo(0b00000, false, false, 3, 1, 2);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[3], 10);
    assert_eq!(bus.memory.sram_read32(0x400), 42);
    assert_eq!(c.pc, 0x2000_0004);
}

// Property-style invariants per HLD §7, driven across two harts sharing
// one Bus. This is the one multi-core test in P3 — it documents the
// contract of `Bus::reservation` and the write-path invalidation hook.
#[test]
fn lrsc_property_invariants_two_harts() {
    let mut harts = [Hazard3::new(0), Hazard3::new(1)];
    let mut bus = Bus::new();

    let addr = 0x2000_1000u32;
    bus.memory.sram_write32(0x1000, 0xBAAD_F00D);

    // --- Invariant: after lr.w on core 0, reservation[0] == Some(addr).
    harts[0].x[1] = addr;
    harts[0].execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(bus.reservation[0], Some(addr));
    assert_eq!(bus.reservation[1], None);

    // --- Invariant: sc.w with matching reservation succeeds AND clears
    // all reservations that covered the same word.
    harts[1].x[1] = addr;
    harts[1].x[2] = 0x1111_1111;
    // Core 1 also reserves the same word.
    harts[1].execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 3,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(bus.reservation[1], Some(addr));
    // Now core 1 does a successful SC.
    harts[1].execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 4,
            rs1: 1,
            rs2: 2,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(harts[1].x[4], 0, "SC on core 1 succeeded");
    assert_eq!(bus.memory.sram_read32(0x1000), 0x1111_1111);
    // Successful SC's underlying write invalidated core 0's
    // reservation at the same word too.
    assert_eq!(bus.reservation[0], None);
    assert_eq!(bus.reservation[1], None);

    // --- Invariant: sc.w with no prior reservation fails and leaves
    // memory unchanged.    harts[0].x[1] = addr;
    harts[0].x[2] = 0xDEAD_DEAD;
    harts[0].execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 5,
            rs1: 1,
            rs2: 2,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(harts[0].x[5], 1, "SC fails without prior LR");
    assert_eq!(
        bus.memory.sram_read32(0x1000),
        0x1111_1111,
        "memory unchanged"
    );

    // --- Invariant: amo*.w outside reservable -> mcause=7, memory
    // unchanged.
    let xip_addr = 0x1C00_0010u32;
    let before = bus.memory.sram_read32(0x1000);
    harts[0].csrs.mtvec = 0x2000_2000;
    harts[0].csrs.mcause = 0;
    harts[0].x[1] = xip_addr;
    harts[0].x[2] = 5;
    harts[0].execute(
        Op::Amo {
            kind: AmoKind::Add,
            rd: 6,
            rs1: 1,
            rs2: 2,
            aq: false,
            rl: false,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(harts[0].csrs.mcause, 7);
    assert_eq!(bus.memory.sram_read32(0x1000), before);
}

// -------------------------------------------------------------------
// RV32C
// -------------------------------------------------------------------

// Single-step a 16-bit instruction plant. Returns nothing; caller
// inspects state via the hart ref. Each test plants at sram_offset 0.
fn step16(c: &mut Hazard3, bus: &mut Bus, hw: u16) {
    write_hw(bus, 0, hw);
    c.step(bus);
}

#[test]
fn c_nop_is_addi_x0_x0_0() {
    // C.NOP = 0x0001
    let (mut c, mut bus) = fresh();
    step16(&mut c, &mut bus, 0x0001);
    assert_eq!(c.pc, 0x2000_0002, "compressed PC advance is 2 bytes");
    assert_eq!(c.x[0], 0);
}

#[test]
fn c_addi_sign_extends_and_writes_rd() {
    // C.ADDI x5, -1  -> rd/rs1=5, imm[5]=1, imm[4:0]=11111 -> imm = -1.
    //
    // Bit layout: 000 1 00101 11111 01
    //             15..13:001(f3) 12:1(imm5) 11..7:00101(rd=5) 6..2:11111 1..0:01
    let hw: u16 = 0b000_1_00101_11111_01;
    let (mut c, mut bus) = fresh();
    c.x[5] = 10;
    step16(&mut c, &mut bus, hw);
    assert_eq!(c.x[5], 9);
    assert_eq!(c.pc, 0x2000_0002);
}

#[test]
fn c_li_writes_sign_extended_immediate() {
    // C.LI x6, -2  -> rs1=0, rd=6.
    // f3=010, imm[5]=1, rd=00110, imm[4:0]=11110
    let hw: u16 = 0b010_1_00110_11110_01;
    let (mut c, mut bus) = fresh();
    step16(&mut c, &mut bus, hw);
    assert_eq!(c.x[6] as i32, -2);
}

#[test]
fn c_lui_writes_upper_immediate() {
    // C.LUI x7, 1  -> rd=7, nzimm6=1 -> imm = 1<<12 = 0x1000.
    // f3=011, imm[17]=0, rd=00111, imm[16:12]=00001, op=01
    let hw: u16 = 0b011_0_00111_00001_01;
    let (mut c, mut bus) = fresh();
    step16(&mut c, &mut bus, hw);
    assert_eq!(c.x[7], 0x1000);
}

#[test]
fn c_addi4spn_adds_to_sp() {
    // C.ADDI4SPN x8, x2, 8 -> nzuimm[3]=1, rd'=000 (=x8).
    // nzuimm encoding: bits 12:11=nzuimm[5:4], 10:7=nzuimm[9:6],
    //                  6=nzuimm[2], 5=nzuimm[3], rd'=bits 4:2.
    // For nzuimm=8 (0b0000_1000): bit 3 = 1. So bits 12..5 = 00000001 0
    //                                                        ^b5_4=00 b9_6=0000 b2=0 b3=1
    // f3=000, bits 12:11=00, bits 10:7=0000, bit 6=0, bit 5=1, rd'=000, op=00
    let hw: u16 = 0b000_00_0000_0_1_000_00;
    let (mut c, mut bus) = fresh();
    c.x[2] = 0x2000_0100; // sp
    step16(&mut c, &mut bus, hw);
    assert_eq!(c.x[8], 0x2000_0108);
}

#[test]
fn c_addi16sp_adjusts_sp() {
    // C.ADDI16SP sp, -16. nzimm is 10-bit sign-extended. -16 in 10-bit
    // two's complement = 0b1111110000 -> bits{b9,b8,b7,b6,b5,b4} all 1.
    // Insn packing: bit12=b9, bit6=b4, bit5=b6, bits4:3=b8:7, bit2=b5.
    //   15:13=011 (f3)
    //   12=1 (b9)
    //   11:7=00010 (rd=2, sp)
    //   6=1 (b4)
    //   5=1 (b6)
    //   4:3=11 (b8:7)
    //   2=1 (b5)
    //   1:0=01 (quadrant)
    let hw: u16 = 0b011_1_00010_1_1_11_1_01;
    let (mut c, mut bus) = fresh();
    c.x[2] = 0x2000_0100;
    step16(&mut c, &mut bus, hw);
    assert_eq!(c.x[2], 0x2000_0100 - 16);
}

#[test]
fn c_lw_c_sw_stack_offsets() {
    // Round-trip via C.SW then C.LW.
    // Pick rs1'=x8 (bits 9:7=000). rs2'=x9 (bits 4:2=001). rd'=x10 (001).
    //
    // SW encoding: f3=110, uimm{b5:3,b2,b6} at bits {12:10,6,5}, op=00.
    // Use uimm=0. So bits 12:10=000, bit 6=0, bit 5=0.
    //   f3=110, uimm=0, rs1'=000, uimm=0, rs2'=001.
    // Result: 110_000_000_0_0_001_00
    //           f3 uimm rs1' u6 u2 rs2' op
    // Wait positions: 15:13 f3=110; 12:10 u[5:3]=000; 9:7 rs1'=000;
    //  6 u[2]=0; 5 u[6]=0; 4:2 rs2'=001; 1:0 op=00.
    let sw: u16 = 0b110_000_000_0_0_001_00;
    // LW: f3=010, same uimm=0, rs1'=000, rd'=010 (bits 4:2=010 -> x10).
    let lw: u16 = 0b010_000_000_0_0_010_00;

    let (mut c, mut bus) = fresh();
    c.x[8] = 0x2000_0200; // rs1
    c.x[9] = 0xCAFE_F00D; // rs2
    // Two compressed insns at offsets 0 and 2.
    write_hw(&mut bus, 0, sw);
    write_hw(&mut bus, 2, lw);
    c.step(&mut bus);
    c.step(&mut bus);
    assert_eq!(c.x[10], 0xCAFE_F00D);
    assert_eq!(c.pc, 0x2000_0004);
}

#[test]
fn c_lwsp_swsp_stack_pointer() {
    // C.SWSP x5, 0(sp); C.LWSP x6, 0(sp)
    // SWSP: f3=110. uimm{b5:2,b7:6} at bits{12:9,8:7}. rs2 at bits 6:2.
    //   uimm=0 -> bits 12:7 all 0. rs2=00101.
    let swsp: u16 = 0b110_000000_00101_10;
    // LWSP: f3=010. uimm{b5,b4:2,b7:6} at bits{12,6:4,3:2}. rd at bits 11:7.
    //   uimm=0 -> all imm bits 0. rd=00110.
    let lwsp: u16 = 0b010_0_00110_00000_10;

    let (mut c, mut bus) = fresh();
    c.x[2] = 0x2000_0300; // sp
    c.x[5] = 0xBEEF_D00D;
    write_hw(&mut bus, 0, swsp);
    write_hw(&mut bus, 2, lwsp);
    c.step(&mut bus);
    c.step(&mut bus);
    assert_eq!(c.x[6], 0xBEEF_D00D);
}

#[test]
fn c_mv_and_c_add() {
    // C.MV x5, x6 -> add x5, x0, x6. f3=100, bit12=0, rd=00101,
    // rs2=00110, op=10.
    let cmv: u16 = 0b100_0_00101_00110_10;
    // C.ADD x5, x7 -> add x5, x5, x7. bit12=1, rs2=00111.
    let cadd: u16 = 0b100_1_00101_00111_10;

    let (mut c, mut bus) = fresh();
    c.x[6] = 100;
    c.x[7] = 23;
    write_hw(&mut bus, 0, cmv);
    write_hw(&mut bus, 2, cadd);
    c.step(&mut bus);
    assert_eq!(c.x[5], 100);
    c.step(&mut bus);
    assert_eq!(c.x[5], 123);
}

#[test]
fn c_jr_and_c_jalr() {
    // C.JR x5 -> jalr x0, 0(x5). bit12=0, rd_field=00101, rs2=00000.
    let cjr: u16 = 0b100_0_00101_00000_10;
    let (mut c, mut bus) = fresh();
    c.x[5] = 0x2000_0020;
    write_hw(&mut bus, 0, cjr);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0020);
    assert_eq!(c.x[1], 0, "C.JR writes x0 as link, not x1");

    // C.JALR x7 -> jalr x1, 0(x7). bit12=1, rd_field=00111, rs2=00000.
    let cjalr: u16 = 0b100_1_00111_00000_10;
    let (mut c, mut bus) = fresh();
    c.x[7] = 0x2000_0030;
    write_hw(&mut bus, 0, cjalr);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0030);
    assert_eq!(c.x[1], 0x2000_0002, "C.JALR link = pc + 2");
}

#[test]
fn c_j_unconditional_jump_pc_plus_2() {
    // C.J imm=4. bits: f3=101, imm layout as in c_jimm for 4 -> b2=?  b5=0 b4=0 b3_1=010.
    // imm=4 means raw bits [11..1] = 00000000100.
    // b11=0 b10=0 b9:8=00 b7=0 b6=0 b5=0 b4=0 b3_1=010.
    // At insn bits: 12=b11=0, 11=b4=0, 10:9=b9:8=00, 8=b10=0, 7=b6=0,
    // 6=b7=0, 5:3=b3:1=010, 2=b5=0.
    let cj: u16 = 0b101_0_0_00_0_0_0_010_0_01;
    let (mut c, mut bus) = fresh();
    write_hw(&mut bus, 0, cj);
    c.step(&mut bus);
    assert_eq!(
        c.pc, 0x2000_0004,
        "c.j with imm=4 jumps from 0x..0000 to 0x..0004"
    );
}

#[test]
fn c_jal_writes_link_x1() {
    // C.JAL imm=4. Same layout as C.J but f3=001, rd=x1.
    let cjal: u16 = 0b001_0_0_00_0_0_0_010_0_01;
    let (mut c, mut bus) = fresh();
    write_hw(&mut bus, 0, cjal);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0004);
    assert_eq!(c.x[1], 0x2000_0002, "link PC is epc+2 for compressed");
}

#[test]
fn c_beqz_taken_and_not_taken() {
    // C.BEQZ rs1'=x8, imm=4.
    // Layout: f3=110, bit12=b8=0, bits11:10=b4:3=00, rs1'=000 (x8),
    // bits6:5=b7:6=00, bits4:3=b2:1=10, bit2=b5=0, op=01.
    let cbeqz: u16 = 0b110_0_00_000_00_10_0_01;
    // Taken: x8 == 0.
    let (mut c, mut bus) = fresh();
    c.x[8] = 0;
    write_hw(&mut bus, 0, cbeqz);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0004, "beqz taken -> pc + 4");

    // Not taken: x8 != 0.
    let (mut c, mut bus) = fresh();
    c.x[8] = 1;
    write_hw(&mut bus, 0, cbeqz);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0002, "beqz not taken -> pc + 2");
}

#[test]
fn c_bnez_inverse_of_beqz() {
    let cbnez: u16 = 0b111_0_00_000_00_10_0_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 1; // nonzero -> taken
    write_hw(&mut bus, 0, cbnez);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0004);

    let (mut c, mut bus) = fresh();
    c.x[8] = 0; // zero -> not taken
    write_hw(&mut bus, 0, cbnez);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0002);
}

#[test]
fn c_slli_shifts_in_place() {
    // C.SLLI x5, 3 -> slli x5, x5, 3. f3=000, bit12=0, rd=00101, shamt[4:0]=00011, op=10.
    let hw: u16 = 0b000_0_00101_00011_10;
    let (mut c, mut bus) = fresh();
    c.x[5] = 1;
    step16(&mut c, &mut bus, hw);
    assert_eq!(c.x[5], 8);
}

#[test]
fn c_srli_srai_andi_and_register_forms() {
    // C.SRLI x8, 2 -> rs1'=000.  bits[12]=0 (shamt5), bits[11:10]=00 (SRLI),
    // bits[9:7]=000 (rs1'=x8), bits[6:2]=00010 (shamt[4:0]), op=01, f3=100.
    let csrli: u16 = 0b100_0_00_000_00010_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 0x8000_0000;
    step16(&mut c, &mut bus, csrli);
    assert_eq!(c.x[8], 0x2000_0000);

    // C.SRAI x8, 2. bits[11:10]=01.
    let csrai: u16 = 0b100_0_01_000_00010_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 0x8000_0000;
    step16(&mut c, &mut bus, csrai);
    assert_eq!(c.x[8], 0xE000_0000);

    // C.ANDI x8, -1. bits[11:10]=10, imm raw = 0b111111 sign-ext to -1.
    // imm[5]=1 (bit12), imm[4:0]=11111 (bits 6:2).
    let candi: u16 = 0b100_1_10_000_11111_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 0xDEAD_BEEF;
    step16(&mut c, &mut bus, candi);
    assert_eq!(c.x[8], 0xDEAD_BEEF);

    // C.SUB x8, x9. bits[12]=0 bits[11:10]=11, sel=0 sub. rs2'=001 (x9).
    let csub: u16 = 0b100_0_11_000_00_001_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 30;
    c.x[9] = 10;
    step16(&mut c, &mut bus, csub);
    assert_eq!(c.x[8], 20);

    // C.XOR x8, x9 -> sel=001.
    let cxor: u16 = 0b100_0_11_000_01_001_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 0xAAAA_AAAA;
    c.x[9] = 0x5555_5555;
    step16(&mut c, &mut bus, cxor);
    assert_eq!(c.x[8], 0xFFFF_FFFF);

    // C.OR x8, x9 -> sel=010.
    let c_or: u16 = 0b100_0_11_000_10_001_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 0xAAAA_0000;
    c.x[9] = 0x0000_5555;
    step16(&mut c, &mut bus, c_or);
    assert_eq!(c.x[8], 0xAAAA_5555);

    // C.AND x8, x9 -> sel=011.
    let cand: u16 = 0b100_0_11_000_11_001_01;
    let (mut c, mut bus) = fresh();
    c.x[8] = 0xFFFF_0000;
    c.x[9] = 0x0F0F_0F0F;
    step16(&mut c, &mut bus, cand);
    assert_eq!(c.x[8], 0x0F0F_0000);
}

#[test]
fn c_ebreak_traps_cause_3() {
    // C.EBREAK -> bit12=1, rd=0, rs2=0, f3=100, op=10. -> 0x9002.
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    step16(&mut c, &mut bus, 0x9002);
    assert_eq!(c.csrs.mcause, 3);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn c_illegal_all_zero_traps() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    step16(&mut c, &mut bus, 0x0000);
    assert_eq!(c.csrs.mcause, 2, "all-zero compressed is illegal");
}

#[test]
fn c_jalr_rs1_aliasing_link_wins() {
    // C.JALR x5. If link rd (x1) shares the source-register index? Not
    // possible here since rd is hard-coded x1 and rs1=x5. But let's test
    // the canonical case — x1 already holds something; link overrides.
    let cjalr: u16 = 0b100_1_00101_00000_10;
    let (mut c, mut bus) = fresh();
    c.x[5] = 0x2000_0050;
    c.x[1] = 0xDEAD_DEAD;
    write_hw(&mut bus, 0, cjalr);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0050);
    assert_eq!(c.x[1], 0x2000_0002);
}

// Decode sanity: confirm the 32-bit `decode` path still rejects 16-bit
// words so callers can't accidentally route them through the wrong
// decoder.
#[test]
fn decode_rejects_16bit_word_for_base_isa_entry() {
    assert!(matches!(decode::decode(0x0001), Op::Illegal { .. }));
}

// End-to-end: encode AluKind::Add via step using a compressed C.ADD.
#[test]
fn compressed_instruction_ticks_pc_by_two() {
    // Plant two C.NOPs and step twice; PC should advance 4 total.
    let (mut c, mut bus) = fresh();
    write_hw(&mut bus, 0, 0x0001);
    write_hw(&mut bus, 2, 0x0001);
    c.step(&mut bus);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0004);
    assert_eq!(c.cycles(), 2);
}

// HLD §7 invariant #4: sc.w targeting a different word than the prior
// matching lr.w must fail (rd=1) and leave the target word unchanged.
// This closes the "address-mismatch" corner in the LR/SC property set —
// the existing multi-hart test covers other-master-wrote and no-LR
// paths, but not address-mismatch on the same hart.
#[test]
fn sc_w_different_addr_fails() {
    let (mut c, mut bus) = fresh();
    // Distinct words, both reservable.
    bus.memory.sram_write32(0x1000, 0xAAAA_AAAA);
    bus.memory.sram_write32(0x1004, 0xBBBB_BBBB);

    // LR.W at 0x2000_1000: reservation[0] = Some(0x2000_1000).
    c.x[1] = 0x2000_1000;
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[2], 0xAAAA_AAAA, "LR loaded the word at 0x2000_1000");
    assert_eq!(bus.reservation[0], Some(0x2000_1000));

    // SC.W at 0x2000_1004 (different word, still reservable) with new
    // data. Must fail: reservation holds 0x2000_1000, not 0x2000_1004.
    c.x[3] = 0x2000_1004;
    c.x[4] = 0xDEAD_BEEF;
    c.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 5,
            rs1: 3,
            rs2: 4,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[5], 1, "SC at a different addr must fail (rd=1)");
    // Target word unchanged.
    assert_eq!(
        bus.memory.sram_read32(0x1004),
        0xBBBB_BBBB,
        "SC failure must not write memory",
    );
    // Original reservation cleared on mismatch (per executor contract).
    assert_eq!(bus.reservation[0], None);
}

// RV32C spec: C.JR with rs1=0 is reserved / illegal. Encoding
// `1000 0000 0000 0010` (0x8002): f3=100, bit12=0, rd_field=0, rs2=0,
// quadrant=10. The decoder must produce `Op::Illegal` so the executor
// traps with mcause=ILLEGAL_INSTRUCTION.
#[test]
fn c_jr_rs1_zero_illegal() {
    let op = decode::decode16(0x8002);
    assert!(
        matches!(op, Op::Illegal { .. }),
        "C.JR with rs1=0 must decode as Illegal, got {:?}",
        op,
    );
}
