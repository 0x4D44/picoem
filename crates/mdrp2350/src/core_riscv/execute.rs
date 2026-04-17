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
    AluImmKind, AluKind, BranchKind, CsrKind, LoadKind, Op, ShiftKind, StoreKind,
};
use super::trap::cause;

impl Hazard3 {
    /// Execute one decoded op. `epc` is the PC of the faulting
    /// instruction (captured before self.pc advance). Returns `true` if
    /// the instruction completed; `false` is reserved for future use
    /// (wfi stall signalling etc.) — P2 always returns `true` because
    /// `wfi_parked` is checked by the step wrapper.
    pub(crate) fn execute(&mut self, op: Op, bus: &mut Bus, epc: u32) {
        // `set_active_pc(epc)` is published by `Hazard3::step` before the
        // fetch (HLD §4.6). Don't duplicate it here.

        // Pre-advance PC to next sequential; branch/jump ops override.
        // Trap entry overrides too. Matches the "execute overrides pc"
        // pattern used by the M33 executor.
        self.pc = epc.wrapping_add(4);

        match op {
            Op::Lui { rd, imm } => self.wr(rd, imm),
            Op::Auipc { rd, imm } => self.wr(rd, epc.wrapping_add(imm)),

            Op::Jal { rd, imm } => {
                let target = epc.wrapping_add(imm as u32);
                // Instruction-address misaligned check: HLD §4.5 cause 0.
                // Without C-extension, the minimum alignment is 4 bytes,
                // so any non-4-aligned target traps.
                // TODO: when C lands in P3, relax to (target & 1) != 0.
                if target & 0b11 != 0 {
                    self.enter_trap(cause::INSTR_ADDR_MISALIGNED, target, epc);
                    return;
                }
                self.wr(rd, epc.wrapping_add(4));
                self.pc = target;
            }
            Op::Jalr { rd, rs1, imm } => {
                // JALR: target = (rs1 + imm) with low bit cleared (RV-priv).
                let target = self.rd_x(rs1).wrapping_add(imm as u32) & !1;
                if target & 0b11 != 0 {
                    self.enter_trap(cause::INSTR_ADDR_MISALIGNED, target, epc);
                    return;
                }
                // RV-priv: write link BEFORE jumping — but also must
                // tolerate rd == rs1 (common JALR t0, t0 pattern). Compute
                // link first, then write.
                let link = epc.wrapping_add(4);
                self.wr(rd, link);
                self.pc = target;
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
                    if target & 0b11 != 0 {
                        self.enter_trap(cause::INSTR_ADDR_MISALIGNED, target, epc);
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
                    self.enter_trap(cause::LOAD_ADDR_MISALIGNED, addr, epc);
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
                    self.enter_trap(cause::LOAD_ACCESS_FAULT, addr, epc);
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
                    self.enter_trap(cause::STORE_ADDR_MISALIGNED, addr, epc);
                    return;
                }
                match kind {
                    StoreKind::Sb => bus.write8(addr,  val as u8, self.hart_id),
                    StoreKind::Sh => bus.write16(addr, val as u16, self.hart_id),
                    StoreKind::Sw => bus.write32(addr, val, self.hart_id),
                }
                if bus.bus_fault(self.hart_id as usize) {
                    bus.clear_bus_fault(self.hart_id as usize);
                    self.enter_trap(cause::STORE_ACCESS_FAULT, addr, epc);
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
                self.enter_trap(cause::ECALL_FROM_M, 0, epc);
            }
            Op::Ebreak => {
                self.enter_trap(cause::BREAKPOINT, 0, epc);
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
                        self.enter_trap(cause::ILLEGAL_INSTRUCTION, 0, epc);
                    }
                }
            }

            Op::Illegal { insn: _ } => {
                self.enter_trap(cause::ILLEGAL_INSTRUCTION, 0, epc);
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
}
