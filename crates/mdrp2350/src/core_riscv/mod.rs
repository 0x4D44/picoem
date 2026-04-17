// Hazard3 RISC-V core. P1b landed the struct + reset + CSR storage;
// P2 wires real fetch-decode-execute for RV32I + Zicsr + Zifencei,
// trap entry / mret, and `wfi`-park semantics. IRQ controller and
// atomics live in P3/P4 per
// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`
// §4.4.

pub(crate) mod regs;
pub(crate) mod decode;
pub(crate) mod csr;
pub(crate) mod trap;
pub(crate) mod execute;
pub(crate) mod irq;

#[cfg(test)]
mod tests_p2;
#[cfg(test)]
mod tests_p3;
#[cfg(test)]
mod tests_p4;

use crate::Bus;
use regs::CsrFile;
use trap::cause;
use irq::Xh3Irq;

/// `misa` hardwired value — MXL=01 (bit 30) + X (bit 23) + I (bit 8) +
/// M (bit 12) + A (bit 0) + C (bit 2). No U/S/B. Per HLD §4.3.
const MISA_VALUE: u32 = 0x4080_1105;

/// Reset PC for V1 firmware loaded via `Emulator::load_image` into SRAM
/// (HLD §4.3 / §8 Q1).
const RESET_PC: u32 = 0x2000_0000;

/// Hazard3 core (single hart). Dual-core complex holds two of these.
pub struct Hazard3 {
    /// Integer register file. `x[0]` is architecturally wired to zero —
    /// the P2 executor ignores writes to index 0 via `rd_x`/`wr`.
    pub(crate) x: [u32; 32],
    /// Program counter.
    pub(crate) pc: u32,
    /// Monotonically increasing per-core cycle count; drives the
    /// quantum scheduler. Distinct from `csrs.mcycle` (CSR-visible,
    /// gated by `mcountinhibit.CY`).
    pub(crate) cycles: u64,
    /// Hart ID (0 or 1). Exposed as `mhartid`.
    hart_id: u8,
    /// Halt flag — observed by `step_pair_riscv`. P2/P4 populate this.
    pub(crate) halted: bool,
    /// `wfi`-parked flag. P4 wake logic clears this when
    /// `(mip & mie) != 0` (HLD §4.6).
    pub(crate) wfi_parked: bool,
    /// M-mode CSR storage.
    pub(crate) csrs: CsrFile,
    /// Hazard3 external-IRQ controller (Xh3irq CSR window at 0xBE0..0xBE5).
    /// P4 wires this to drive `mip[11]` (MEIP).
    pub(crate) xh3irq: Xh3Irq,
}

impl Hazard3 {
    /// Construct a hart at reset with the given hart ID. Applies every
    /// HLD §4.3 reset value.
    pub fn new(hart_id: u8) -> Self {
        Self {
            x: [0; 32],
            pc: RESET_PC,
            cycles: 0,
            hart_id,
            halted: false,
            wfi_parked: false,
            csrs: CsrFile::new(),
            xh3irq: Xh3Irq::new(),
        }
    }

    /// Reset the hart to its §4.3 power-on state, preserving the hart
    /// ID.
    pub fn reset(&mut self) {
        *self = Self::new(self.hart_id);
    }

    /// Execute one RV32I[MAC] + Zicsr + Zifencei instruction. Flat
    /// 1-cycle cost (HLD §8 Q5; silicon cycle oracle is P5/P6 work).
    /// Parked / halted harts are no-ops.
    pub fn step(&mut self, bus: &mut Bus) {
        if self.halted || self.wfi_parked {
            return;
        }

        // P4 trap delivery at instruction boundary (HLD §4.6). Interrupts
        // dispatch only when mstatus.MIE=1 and at least one `mip & mie`
        // bit is set. RV-priv §3.1.9 priority: MEI > MSI > MTI > internal
        // > external in general, but Hazard3/RP2350 fixes on the standard
        // RV-priv relative priorities. We check MEIP/MSIP/MTIP in that
        // order.
        let mie_global = (self.csrs.mstatus >> 3) & 1 != 0;
        let pending = self.csrs.mip & self.csrs.mie;
        if mie_global && pending != 0 {
            // Priority: MEIP (bit 11), MSIP (bit 3), MTIP (bit 7). This
            // matches the Hazard3 csr.adoc "priority order" note — MEI
            // highest, then MSI, then MTI.
            //
            // CRITICAL: MEIP only triggers trap delivery when xh3irq
            // arbitration produces a winning IRQ. If `mip[11]` is set but
            // every pending IRQ is masked by `meicontext.ppreempt` (and no
            // firmware-writable MEIP path exists — see `MIP_MASK`),
            // `arbitrate()` returns `None`. In that case we must fall
            // through to MSIP/MTIP/fetch rather than deliver a MEIP trap
            // with no IRQ context: the handler would see `meinext.noirq=1`,
            // `mret` back, and re-trigger immediately → infinite loop.
            let irq_pending = bus.atomics.irq_pending_load(self.hart_id as usize);
            let meip_arb = if pending & (1 << 11) != 0 {
                self.xh3irq.arbitrate(irq_pending)
            } else {
                None
            };
            let chosen: Option<(u32, Option<(u8, u8)>)> = if meip_arb.is_some() {
                Some((11u32, meip_arb))
            } else if pending & (1 << 3) != 0 {
                Some((3u32, None))
            } else if pending & (1 << 7) != 0 {
                Some((7u32, None))
            } else {
                // MEIP was set but arbitration returned None, and no
                // MSIP/MTIP pending. Fall through to fetch — equivalent to
                // no deliverable interrupt this cycle.
                None
            };
            if let Some((cause_code, irq_ctx)) = chosen {
                // Interrupt cause has bit 31 set.
                let mcause = 0x8000_0000 | cause_code;
                // mepc = PC of the *next* instruction that would have run —
                // the instruction hasn't started. That's the current self.pc.
                let epc = self.pc;
                self.enter_trap(mcause, 0, epc, bus);
                // On MEIP, install preempt level from xh3irq.
                if let Some((irq, pri)) = irq_ctx {
                    self.xh3irq.on_ext_irq_entry(irq, pri);
                }
                self.cycles = self.cycles.wrapping_add(1);
                return;
            }
        }

        let epc = self.pc;

        // Instruction-address misalignment. With C-extension PC only needs
        // bit 0 clear; bit 1 is acceptable. HLD §4.5 cause 0.
        if epc & 1 != 0 {
            self.enter_trap(cause::INSTR_ADDR_MISALIGNED, epc, epc, bus);
            self.cycles = self.cycles.wrapping_add(1);
            return;
        }

        // Publish current-instruction PC for the MMIO trace (HLD §4.6).
        // Must precede the fetch so fetch-side bus transactions attribute
        // to the correct PC.
        bus.set_active_pc(epc, self.hart_id);

        // Fetch first halfword. Low two bits select 16-vs-32-bit.
        let hw0 = bus.read16(epc, self.hart_id);
        if bus.bus_fault(self.hart_id as usize) {
            bus.clear_bus_fault(self.hart_id as usize);
            self.enter_trap(cause::INSTR_ACCESS_FAULT, epc, epc, bus);
            self.cycles = self.cycles.wrapping_add(1);
            return;
        }
        let (op, width) = if (hw0 & 0b11) != 0b11 {
            // Compressed.
            (decode::decode16(hw0), 2u32)
        } else {
            // Base-ISA 32-bit. Fetch the second halfword and combine.
            let hw1 = bus.read16(epc.wrapping_add(2), self.hart_id);
            if bus.bus_fault(self.hart_id as usize) {
                bus.clear_bus_fault(self.hart_id as usize);
                self.enter_trap(cause::INSTR_ACCESS_FAULT, epc, epc, bus);
                self.cycles = self.cycles.wrapping_add(1);
                return;
            }
            let insn = (hw0 as u32) | ((hw1 as u32) << 16);
            (decode::decode(insn), 4u32)
        };
        self.execute_sized(op, bus, epc, width);

        // Flat-cycle cost. The HLD's M/load mult-cycle budget lands in P3.
        self.cycles = self.cycles.wrapping_add(1);

        // RV-priv §3.1.11: `mcycle` / `minstret` tick when their
        // respective `mcountinhibit` bit is clear. Reset inhibits both
        // (mcountinhibit = 0b101), so firmware must opt in.
        if self.csrs.mcountinhibit & 0b001 == 0 {
            self.csrs.mcycle = self.csrs.mcycle.wrapping_add(1);
        }
        if self.csrs.mcountinhibit & 0b100 == 0 {
            self.csrs.minstret = self.csrs.minstret.wrapping_add(1);
        }
    }

    /// Per-core cycle count (scheduler view).
    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    /// True when the hart is halted or `wfi`-parked — either condition
    /// stops the quantum scheduler from dispatching this hart.
    ///
    /// `wfi_parked` is folded in so the scheduler skips parked harts
    /// cheaply. P4 may split this when `wfi` wake is wired to
    /// `(mip & mie) != 0` per HLD §4.6 — today the fold is safe because
    /// `Emulator::step` advances `clock` / `tick_peripherals` independently
    /// of core cycles.
    pub fn is_halted(&self) -> bool {
        self.halted || self.wfi_parked
    }

    /// Hard-wired `mhartid` (HLD §4.3). Exposed for P2 CSR dispatch.
    pub(crate) fn mhartid(&self) -> u32 {
        self.hart_id as u32
    }

    /// Hard-wired `misa` value — `0x4080_1105` (HLD §4.3).
    pub(crate) fn misa(&self) -> u32 {
        MISA_VALUE
    }

    /// Hard-wired `mvendorid` — 0 (Hazard3 upstream default).
    pub(crate) fn mvendorid(&self) -> u32 {
        0
    }

    /// Hard-wired `marchid` — 0 (Hazard3 upstream default).
    pub(crate) fn marchid(&self) -> u32 {
        0
    }

    /// Hard-wired `mimpid` — 0 (Hazard3 upstream default).
    pub(crate) fn mimpid(&self) -> u32 {
        0
    }

    /// Hard-wired `mconfigptr` (CSR 0xF15) — 0 (RV-priv 1.12 mandatory;
    /// Hazard3 csr.adoc :79).
    pub(crate) fn mconfigptr(&self) -> u32 {
        0
    }

    /// Read `mip`. Exposed for P4's `fan_out_riscv_irqs` (HLD §4.6).
    #[allow(dead_code)]
    pub(crate) fn mip(&self) -> u32 {
        self.csrs.mip
    }

    /// Write `mip`. Exposed for P4's `fan_out_riscv_irqs`, which drives
    /// bits 3 (MSIP) and 7 (MTIP) directly per RV-priv §3.1.9.
    #[allow(dead_code)]
    pub(crate) fn set_mip(&mut self, v: u32) {
        self.csrs.mip = v;
    }

    /// Read `mie`. Exposed for the `wfi` wake predicate
    /// `(mip & mie) != 0` (HLD §4.6).
    #[allow(dead_code)]
    pub(crate) fn mie(&self) -> u32 {
        self.csrs.mie
    }

    /// Compute the MEIP (`mip[11]`) source bit from the Hazard3 IRQ
    /// controller's view of `bus.irq_pending[hart] | meifa`, masked by
    /// `meiea`. HLD §4.6: MEIP = OR-reduce of `(meipa & meiea) | meifa`.
    pub(crate) fn compute_meip(&self, irq_pending: u64) -> bool {
        self.xh3irq.meip(irq_pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Arch, Config, Cores, EmulatorBuilder};

    #[test]
    fn reset_values_hart_0() {
        let c = Hazard3::new(0);

        assert_eq!(c.x, [0; 32]);
        assert_eq!(c.pc, 0x2000_0000);
        assert_eq!(c.cycles, 0);
        assert_eq!(c.mhartid(), 0);
        assert_eq!(c.misa(), 0x4080_1105);
        assert_eq!(c.mvendorid(), 0);
        assert_eq!(c.marchid(), 0);
        assert_eq!(c.mimpid(), 0);
        assert_eq!(c.mconfigptr(), 0);
        assert!(!c.halted);
        assert!(!c.wfi_parked);
        assert!(!c.is_halted());

        // §4.3 CSR resets.
        assert_eq!(c.csrs.mstatus, 0);
        assert_eq!(c.csrs.mie, 0);
        assert_eq!(c.csrs.mip, 0);
        assert_eq!(c.csrs.mtvec, 0x0000_1FFD);
        assert_eq!(c.csrs.mscratch, 0);
        assert_eq!(c.csrs.mepc, 0);
        assert_eq!(c.csrs.mcause, 0);
        assert_eq!(c.csrs.mtval, 0);
        assert_eq!(c.csrs.mcountinhibit, 0b101);
        assert_eq!(c.csrs.mcycle, 0);
        assert_eq!(c.csrs.minstret, 0);
    }

    #[test]
    fn reset_values_hart_1() {
        let c = Hazard3::new(1);
        assert_eq!(c.mhartid(), 1);
        // Everything else §4.3-identical to hart 0.
        assert_eq!(c.pc, 0x2000_0000);
        assert_eq!(c.csrs.mtvec, 0x0000_1FFD);
        assert_eq!(c.csrs.mcountinhibit, 0b101);
    }

    #[test]
    fn step_advances_pc_and_cycles() {
        let mut c = Hazard3::new(0);
        let mut bus = Bus::new();
        // Plant a NOP (ADDI x0, x0, 0) at the reset PC so the fetch
        // decodes cleanly — without this the zeroed SRAM decodes as an
        // illegal (low bits != 0b11) and the trap path overrides pc.
        bus.memory.sram_write32(0, 0x0000_0013);
        c.step(&mut bus);
        assert_eq!(c.pc, 0x2000_0004);
        assert_eq!(c.cycles(), 1);
    }

    #[test]
    fn emulator_reset_riscv_calls_hazard3_reset() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build();

        // Mutate both harts away from §4.3 defaults.
        {
            let Cores::RiscV(cs) = &mut emu.cores else {
                unreachable!("built with Arch::RiscV")
            };
            for c in cs.iter_mut() {
                c.pc = 0xDEAD_BEEF;
                c.cycles = 12345;
                c.csrs.mstatus = 0x1888;
                c.csrs.mie = 0x888;
                c.csrs.mtvec = 0xABCD_0000;
                c.csrs.mcountinhibit = 0;
                c.halted = true;
                c.wfi_parked = true;
                c.x[5] = 0x4242_4242;
                c.csrs.mtval = 0xFFFF_FFFF;
                c.x[0] = 0xFFFF_FFFF;
            }
        }

        emu.reset();

        let Cores::RiscV(cs) = &emu.cores else {
            unreachable!("built with Arch::RiscV")
        };
        for (i, c) in cs.iter().enumerate() {
            assert_eq!(c.mhartid(), i as u32, "hart id preserved");
            assert_eq!(c.pc, 0x2000_0000);
            assert_eq!(c.cycles, 0);
            assert_eq!(c.csrs.mstatus, 0);
            assert_eq!(c.csrs.mie, 0);
            assert_eq!(c.csrs.mtvec, 0x0000_1FFD);
            assert_eq!(c.csrs.mcountinhibit, 0b101);
            assert!(!c.halted);
            assert!(!c.wfi_parked);
            assert_eq!(c.x[5], 0);
            assert_eq!(c.csrs.mtval, 0);
            assert_eq!(c.x[0], 0);
        }
    }
}
