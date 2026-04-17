// RV32I + Zicsr + Zifencei executor. Dispatch from `Hazard3::step`
// lands here with an already-decoded `Op` and the faulting-instruction
// PC stashed in `self.pc`. Every bus access checks `bus.bus_fault(self.hart_id as usize)`
// and maps to the §4.5 trap table on fault.
//
// The executor mutates `self.pc` to the next-sequential PC *before*
// branches/jumps/traps potentially override it. Branch/jump ops overwrite
// with the taken target; trap entry overwrites with mtvec.

use crate::Bus;

use super::Hazard3;
use super::csr::{csr_access, CsrAccess};
use super::decode::{
    AluImmKind, AluKind, AmoKind, BranchKind, CsrKind, LoadKind, MulDivKind, Op, ShiftKind,
    StoreKind,
};
use super::trap::cause;

/// Reservable-memory bounds for RV32A LR/SC/AMO. HLD §4.7 — the entire
/// 528 KB SRAM (`0x2000_0000..0x2008_2000`). Everything outside this
/// range is non-reservable: LR/SC silently fail; AMO traps mcause=7.
const RESERVABLE_LO: u32 = 0x2000_0000;
const RESERVABLE_HI: u32 = 0x2008_2000;

#[inline]
fn in_reservable(addr: u32) -> bool {
    addr >= RESERVABLE_LO && addr < RESERVABLE_HI
}

impl Hazard3 {
    /// Execute one decoded op. `epc` is the PC of the faulting
    /// instruction (captured before self.pc advance). Convenience wrapper
    /// that assumes a 32-bit-wide instruction — used by tests that
    /// construct `Op` variants directly without going through `step`.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn execute(&mut self, op: Op, bus: &mut Bus, epc: u32) {
        self.execute_sized(op, bus, epc, 4);
    }

    /// Execute one decoded op with an explicit instruction width (`width`
    /// is `2` for RV32C compressed, `4` for base-ISA 32-bit). The pre-
    /// advanced PC is `epc + width`; branch/jump ops override.
    pub(crate) fn execute_sized(&mut self, op: Op, bus: &mut Bus, epc: u32, width: u32) {
        // `set_active_pc(epc)` is published by `Hazard3::step` before the
        // fetch (HLD §4.6). Don't duplicate it here.

        // Pre-advance PC to next sequential; branch/jump ops override.
        // Trap entry overrides too. Matches the "execute overrides pc"
        // pattern used by the M33 executor.
        self.pc = epc.wrapping_add(width);

        match op {
            Op::Lui { rd, imm } => self.wr(rd, imm),
            Op::Auipc { rd, imm } => self.wr(rd, epc.wrapping_add(imm)),

            Op::Jal { rd, imm } => {
                let target = epc.wrapping_add(imm as u32);
                // With C-extension, targets only need to be 2-byte aligned
                // (bit 0 clear). Bit 1 of target is acceptable. HLD §4.5.
                if target & 1 != 0 {
                    self.enter_trap(cause::INSTR_ADDR_MISALIGNED, target, epc, bus);
                    return;
                }
                // Link PC is the next-sequential PC. `self.pc` was already
                // pre-advanced to `epc + 4` (or +2 for compressed) by the
                // step wrapper; use that value so C.JAL produces a +2 link.
                let link = self.pc;
                self.wr(rd, link);
                self.pc = target;
            }
            Op::Jalr { rd, rs1, imm } => {
                // JALR: target = (rs1 + imm) with low bit cleared (RV-priv).
                let target = self.rd_x(rs1).wrapping_add(imm as u32) & !1;
                // RV-priv: write link BEFORE jumping — but also must
                // tolerate rd == rs1 (common JALR t0, t0 pattern). Compute
                // link first, then write.
                let link = self.pc;
                self.wr(rd, link);
                self.pc = target;
                // Target bit 0 is already cleared; bit 1 can be set with
                // C. No further alignment check is needed.
            }

            Op::Branch { kind, rs1, rs2, imm } => {
                let a = self.rd_x(rs1);
                let b = self.rd_x(rs2);
                let taken = match kind {
                    BranchKind::Beq  => a == b,
                    BranchKind::Bne  => a != b,
                    BranchKind::Blt  => (a as i32) <  (b as i32),
                    BranchKind::Bge  => (a as i32) >= (b as i32),
                    BranchKind::Bltu => a <  b,
                    BranchKind::Bgeu => a >= b,
                };
                if taken {
                    let target = epc.wrapping_add(imm as u32);
                    // 2-byte alignment with C.
                    if target & 1 != 0 {
                        self.enter_trap(cause::INSTR_ADDR_MISALIGNED, target, epc, bus);
                        return;
                    }
                    self.pc = target;
                }
            }

            Op::Load { kind, rd, rs1, imm } => {
                let addr = self.rd_x(rs1).wrapping_add(imm as u32);
                let (size, aligned) = match kind {
                    LoadKind::Lb | LoadKind::Lbu => (1u32, true),
                    LoadKind::Lh | LoadKind::Lhu => (2u32, addr & 1 == 0),
                    LoadKind::Lw => (4u32, addr & 3 == 0),
                };
                if !aligned {
                    self.enter_trap(cause::LOAD_ADDR_MISALIGNED, addr, epc, bus);
                    return;
                }
                // Issue the access.
                let val: u32 = match kind {
                    LoadKind::Lb => {
                        let v = bus.read8(addr, self.hart_id) as i8 as i32 as u32;
                        v
                    }
                    LoadKind::Lbu => bus.read8(addr, self.hart_id) as u32,
                    LoadKind::Lh => {
                        let v = bus.read16(addr, self.hart_id) as i16 as i32 as u32;
                        v
                    }
                    LoadKind::Lhu => bus.read16(addr, self.hart_id) as u32,
                    LoadKind::Lw  => bus.read32(addr, self.hart_id),
                };
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    self.enter_trap(cause::LOAD_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
                let _ = size;
                self.wr(rd, val);
            }

            Op::Store { kind, rs1, rs2, imm } => {
                let addr = self.rd_x(rs1).wrapping_add(imm as u32);
                let val = self.rd_x(rs2);
                let aligned = match kind {
                    StoreKind::Sb => true,
                    StoreKind::Sh => addr & 1 == 0,
                    StoreKind::Sw => addr & 3 == 0,
                };
                if !aligned {
                    self.enter_trap(cause::STORE_ADDR_MISALIGNED, addr, epc, bus);
                    return;
                }
                match kind {
                    StoreKind::Sb => bus.write8(addr,  val as u8, self.hart_id),
                    StoreKind::Sh => bus.write16(addr, val as u16, self.hart_id),
                    StoreKind::Sw => bus.write32(addr, val, self.hart_id),
                }
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    self.enter_trap(cause::STORE_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
            }

            Op::OpImm { kind, rd, rs1, imm } => {
                let a = self.rd_x(rs1);
                let b = imm as u32;
                let r = match kind {
                    AluImmKind::Addi  => a.wrapping_add(b),
                    AluImmKind::Slti  => if (a as i32) < imm { 1 } else { 0 },
                    AluImmKind::Sltiu => if a < b { 1 } else { 0 },
                    AluImmKind::Xori  => a ^ b,
                    AluImmKind::Ori   => a | b,
                    AluImmKind::Andi  => a & b,
                };
                self.wr(rd, r);
            }
            Op::ShiftImm { kind, rd, rs1, shamt } => {
                let a = self.rd_x(rs1);
                let s = shamt & 0x1F;
                let r = match kind {
                    ShiftKind::Slli => a.wrapping_shl(s as u32),
                    ShiftKind::Srli => a.wrapping_shr(s as u32),
                    ShiftKind::Srai => ((a as i32).wrapping_shr(s as u32)) as u32,
                };
                self.wr(rd, r);
            }

            Op::Op { kind, rd, rs1, rs2 } => {
                let a = self.rd_x(rs1);
                let b = self.rd_x(rs2);
                let r = match kind {
                    AluKind::Add  => a.wrapping_add(b),
                    AluKind::Sub  => a.wrapping_sub(b),
                    AluKind::Sll  => a.wrapping_shl(b & 0x1F),
                    AluKind::Slt  => if (a as i32) < (b as i32) { 1 } else { 0 },
                    AluKind::Sltu => if a < b { 1 } else { 0 },
                    AluKind::Xor  => a ^ b,
                    AluKind::Srl  => a.wrapping_shr(b & 0x1F),
                    AluKind::Sra  => ((a as i32).wrapping_shr(b & 0x1F)) as u32,
                    AluKind::Or   => a | b,
                    AluKind::And  => a & b,
                };
                self.wr(rd, r);
            }

            Op::MulDiv { kind, rd, rs1, rs2 } => {
                let a = self.rd_x(rs1);
                let b = self.rd_x(rs2);
                let r = exec_muldiv(kind, a, b);
                self.wr(rd, r);
            }

            Op::Amo { kind, rd, rs1, rs2, aq: _, rl: _ } => {
                self.exec_amo(kind, rd, rs1, rs2, bus, epc);
            }

            Op::Fence => {
                // No-op — single-threaded emulation (HLD §3, §4.5).
            }
            Op::FenceI => {
                // No-op on RISC-V today. HLD §4.8 tripwire: when a RISC-V
                // decoded-op cache lands, this path must invalidate. The
                // debug_assert below is the tripwire — it fires in debug
                // builds on FENCE.I execution so the cache-add PR notices.
                // Feel free to toggle this constant to `true` once the
                // cache path wires invalidation.
                const RISCV_DECODE_CACHE_EXISTS: bool = false;
                debug_assert!(
                    !RISCV_DECODE_CACHE_EXISTS,
                    "fence.i is no-op; wire invalidation first (HLD §4.8)"
                );
            }

            Op::Ecall => {
                self.enter_trap(cause::ECALL_FROM_M, 0, epc, bus);
            }
            Op::Ebreak => {
                self.enter_trap(cause::BREAKPOINT, 0, epc, bus);
            }
            Op::Mret => {
                self.mret();
            }
            Op::Wfi => {
                // HLD §4.6: hart parks; wake when `(mip & mie) != 0`. The
                // wake side of the predicate is P4 — P2 just sets the
                // flag so the scheduler skips this hart. Firmware that
                // needs the wake semantics in P2 will block here forever,
                // matching the HLD's documented scope.
                self.wfi_parked = true;
            }

            Op::Csr { kind, rd, rs1_or_zimm, csr } => {
                let rs1_val = if matches!(kind, CsrKind::Csrrw | CsrKind::Csrrs | CsrKind::Csrrc) {
                    self.rd_x(rs1_or_zimm)
                } else {
                    0 // immediate forms — csr_access uses rs1_or_zimm directly
                };
                match csr_access(self, kind, csr, rs1_or_zimm, rs1_val) {
                    CsrAccess::Ok(old) => self.wr(rd, old),
                    CsrAccess::Trap => {
                        self.enter_trap(cause::ILLEGAL_INSTRUCTION, 0, epc, bus);
                    }
                }
            }

            Op::Illegal { insn: _ } => {
                self.enter_trap(cause::ILLEGAL_INSTRUCTION, 0, epc, bus);
            }
        }
    }

    /// Read a general-purpose register. `x[0]` always reads as zero.
    #[inline(always)]
    pub(crate) fn rd_x(&self, idx: u8) -> u32 {
        if idx == 0 { 0 } else { self.x[idx as usize] }
    }

    /// Write a general-purpose register. Writes to `x[0]` are no-ops.
    #[inline(always)]
    pub(crate) fn wr(&mut self, idx: u8, val: u32) {
        if idx != 0 {
            self.x[idx as usize] = val;
        }
    }

    /// RV32A: word-sized atomics (LR/SC/AMO*). All follow HLD §4.7
    /// semantics: misalign -> mcause 4/6/6, reservable-region gating,
    /// bus-fault -> mcause 5/7/7. AMO-outside-reservable is the only
    /// case that traps (mcause=7) — LR/SC outside reservable silently
    /// fail.
    fn exec_amo(
        &mut self,
        kind: AmoKind,
        rd: u8,
        rs1: u8,
        rs2: u8,
        bus: &mut Bus,
        epc: u32,
    ) {
        let addr = self.rd_x(rs1);
        let core = (self.hart_id as usize);
        match kind {
            AmoKind::Lr => {
                if addr & 3 != 0 {
                    self.enter_trap(cause::LOAD_ADDR_MISALIGNED, addr, epc, bus);
                    return;
                }
                if !in_reservable(addr) {
                    // HLD §4.7: LR outside reservable silently fails —
                    // reservation is NOT recorded, no read performed, no
                    // trap. Real silicon spins forever here.
                    return;
                }
                let val = bus.read32(addr, self.hart_id);
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    self.enter_trap(cause::LOAD_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
                bus.reservation[core] = Some(addr);
                self.wr(rd, val);
            }
            AmoKind::Sc => {
                if addr & 3 != 0 {
                    self.enter_trap(cause::STORE_ADDR_MISALIGNED, addr, epc, bus);
                    return;
                }
                if !in_reservable(addr) {
                    // Silent fail per HLD §4.7 — rd = 1, no write, no
                    // trap. Reservation is *not* cleared by a no-op SC.
                    self.wr(rd, 1);
                    return;
                }
                if bus.reservation[core] != Some(addr) {
                    // Mismatch: fail; clear this core's reservation.
                    bus.reservation[core] = None;
                    self.wr(rd, 1);
                    return;
                }
                let val = self.rd_x(rs2);
                bus.write32(addr, val, self.hart_id);
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    // Defensive: clear this core's reservation before
                    // trap entry. `write32` already calls
                    // `invalidate_reservation_at(addr)` before issuing
                    // the bus transaction, so this is belt-and-suspenders
                    // — but explicit here so the fault corner doesn't
                    // leak a stale reservation if the invalidation hook
                    // ever changes.
                    bus.reservation[core] = None;
                    self.enter_trap(cause::STORE_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
                // The `write32` call above already invalidated every
                // matching reservation (both cores) via
                // `invalidate_reservation_at`.
                debug_assert!(
                    bus.reservation[core].is_none(),
                    "successful sc.w should have cleared this core's reservation via write32 invalidation hook"
                );
                self.wr(rd, 0);
            }
            _ => {
                // AMO*.W. Misalign -> mcause=6. Outside reservable ->
                // mcause=7 (datasheet §2.1.6.2 distinguishes AMO from
                // lr/sc).
                if addr & 3 != 0 {
                    self.enter_trap(cause::STORE_ADDR_MISALIGNED, addr, epc, bus);
                    return;
                }
                if !in_reservable(addr) {
                    self.enter_trap(cause::STORE_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
                let old = bus.read32(addr, self.hart_id);
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    self.enter_trap(cause::STORE_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
                let src = self.rd_x(rs2);
                let new = match kind {
                    AmoKind::Swap => src,
                    AmoKind::Add  => old.wrapping_add(src),
                    AmoKind::And  => old & src,
                    AmoKind::Or   => old | src,
                    AmoKind::Xor  => old ^ src,
                    AmoKind::Min  => ((old as i32).min(src as i32)) as u32,
                    AmoKind::Max  => ((old as i32).max(src as i32)) as u32,
                    AmoKind::Minu => old.min(src),
                    AmoKind::Maxu => old.max(src),
                    AmoKind::Lr | AmoKind::Sc => unreachable!(),
                };
                bus.write32(addr, new, self.hart_id);
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    self.enter_trap(cause::STORE_ACCESS_FAULT, addr, epc, bus);
                    return;
                }
                self.wr(rd, old);
            }
        }
    }
}

/// RV32M operation semantics. Extracted as a free fn because it only
/// needs operand arithmetic — no bus / CSR touch.
#[inline]
fn exec_muldiv(kind: MulDivKind, a: u32, b: u32) -> u32 {
    match kind {
        MulDivKind::Mul => a.wrapping_mul(b),
        MulDivKind::Mulh => {
            let p = (a as i32 as i64).wrapping_mul(b as i32 as i64);
            (p >> 32) as u32
        }
        MulDivKind::Mulhsu => {
            let p = (a as i32 as i64).wrapping_mul(b as u64 as i64);
            (p >> 32) as u32
        }
        MulDivKind::Mulhu => {
            let p = (a as u64).wrapping_mul(b as u64);
            (p >> 32) as u32
        }
        MulDivKind::Div => {
            // DIV by zero -> quotient = -1 (all bits set).
            // Overflow INT_MIN / -1 -> INT_MIN.
            if b == 0 {
                0xFFFF_FFFF
            } else if (a as i32) == i32::MIN && (b as i32) == -1 {
                i32::MIN as u32
            } else {
                ((a as i32).wrapping_div(b as i32)) as u32
            }
        }
        MulDivKind::Divu => {
            if b == 0 { 0xFFFF_FFFF } else { a / b }
        }
        MulDivKind::Rem => {
            if b == 0 {
                a // remainder = dividend
            } else if (a as i32) == i32::MIN && (b as i32) == -1 {
                0
            } else {
                ((a as i32).wrapping_rem(b as i32)) as u32
            }
        }
        MulDivKind::Remu => {
            if b == 0 { a } else { a % b }
        }
    }
}
