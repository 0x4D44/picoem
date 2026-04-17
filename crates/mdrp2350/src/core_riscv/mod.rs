// Hazard3 RISC-V core skeleton. P1b lands the struct + reset + CSR
// storage only; decode/execute/trap/IRQ/atomics live in P2..P4 per
// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`
// §4.4.

pub(crate) mod regs;

use crate::Bus;
use regs::CsrFile;

/// `misa` hardwired value — MXL=01 (bit 30) + X (bit 23) + I (bit 8) +
/// M (bit 12) + A (bit 0) + C (bit 2). No U/S/B. Per HLD §4.3.
const MISA_VALUE: u32 = 0x4080_1105;

/// Reset PC for V1 firmware loaded via `Emulator::load_image` into SRAM
/// (HLD §4.3 / §8 Q1).
const RESET_PC: u32 = 0x2000_0000;

/// Hazard3 core (single hart). Dual-core complex holds two of these.
pub struct Hazard3 {
    /// Integer register file. `x[0]` is architecturally wired to zero —
    /// the P2 executor is responsible for ignoring writes to index 0.
    /// P1b stores plain u32 without that guard.
    #[allow(dead_code)]
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
        }
    }

    /// Reset the hart to its §4.3 power-on state, preserving the hart
    /// ID.
    pub fn reset(&mut self) {
        *self = Self::new(self.hart_id);
    }

    /// Stub step. P2 replaces this with real fetch-decode-execute.
    /// Advances PC by 4 and the cycle counter by 1 so the quantum
    /// scheduler makes forward progress during bring-up.
    pub fn step(&mut self, _bus: &mut Bus) {
        self.pc = self.pc.wrapping_add(4);
        self.cycles = self.cycles.wrapping_add(1);
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
    #[allow(dead_code)]
    pub(crate) fn mhartid(&self) -> u32 {
        self.hart_id as u32
    }

    /// Hard-wired `misa` value — `0x4080_1105` (HLD §4.3).
    #[allow(dead_code)]
    pub(crate) fn misa(&self) -> u32 {
        MISA_VALUE
    }

    /// Hard-wired `mvendorid` — 0 (Hazard3 upstream default).
    #[allow(dead_code)]
    pub(crate) fn mvendorid(&self) -> u32 {
        0
    }

    /// Hard-wired `marchid` — 0 (Hazard3 upstream default).
    #[allow(dead_code)]
    pub(crate) fn marchid(&self) -> u32 {
        0
    }

    /// Hard-wired `mimpid` — 0 (Hazard3 upstream default).
    #[allow(dead_code)]
    pub(crate) fn mimpid(&self) -> u32 {
        0
    }

    /// Hard-wired `mconfigptr` (CSR 0xF15) — 0 (RV-priv 1.12 mandatory;
    /// Hazard3 csr.adoc :79).
    #[allow(dead_code)]
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
