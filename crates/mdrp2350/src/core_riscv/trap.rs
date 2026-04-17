// Trap entry + mret. HLD §4.5 / §4.6.
//
// mtval is hardwired 0 on Hazard3 (HLD §4.3 / §8 Q7 / RP2350 datasheet
// §3.8.4.1) — the trap-entry path accepts a tval argument for
// documentation only and never writes it to the CSR. Callers that compute
// a faulting address may still want to pass it for future expansion
// (when / if Hazard3 adds mtval storage in a later silicon rev).

use super::Hazard3;
use crate::Bus;

/// Trap causes used by the P2 executor. Keeps the call sites readable
/// and traceable to the HLD §4.5 table.
pub(crate) mod cause {
    pub(crate) const INSTR_ADDR_MISALIGNED: u32 = 0;
    pub(crate) const INSTR_ACCESS_FAULT:    u32 = 1;
    pub(crate) const ILLEGAL_INSTRUCTION:   u32 = 2;
    pub(crate) const BREAKPOINT:            u32 = 3;
    pub(crate) const LOAD_ADDR_MISALIGNED:  u32 = 4;
    pub(crate) const LOAD_ACCESS_FAULT:     u32 = 5;
    pub(crate) const STORE_ADDR_MISALIGNED: u32 = 6;
    pub(crate) const STORE_ACCESS_FAULT:    u32 = 7;
    pub(crate) const ECALL_FROM_M:          u32 = 11;
}

impl Hazard3 {
    /// Enter a synchronous trap. `epc` is the PC of the faulting
    /// instruction (not the next sequential PC). `cause` is the §4.5
    /// mcause value — exception causes only go through this path; P4
    /// will add an interrupt-delivery path that also lands here with the
    /// bit-31 interrupt flag set.
    ///
    /// `_tval` is documentation-only on Hazard3: mtval is hardwired 0
    /// (HLD §4.3). Passed in so call sites declare what they *would*
    /// have reported.
    ///
    /// `bus` is threaded through so the trap-entry path can clear this
    /// hart's LR/SC reservation (RISC-V A-extension §8.3 spec
    /// recommendation: "it is strongly recommended that the reservation
    /// set be invalidated on exceptions and context switches").
    pub(crate) fn enter_trap(&mut self, cause: u32, _tval: u32, epc: u32, bus: &mut Bus) {
        // §4.5: mepc captures the PC of the faulting instruction.
        self.csrs.mepc = epc;

        // mstatus.MPIE <- mstatus.MIE; mstatus.MIE <- 0; mstatus.MPP <- 0b11.
        let mie_bit = (self.csrs.mstatus >> 3) & 1;
        // Clear MIE, MPIE, MPP; write fresh values in.
        self.csrs.mstatus &= !((1 << 3) | (1 << 7) | (0b11 << 11));
        self.csrs.mstatus |= mie_bit << 7; // MPIE
        // MIE stays cleared.
        self.csrs.mstatus |= 0b11 << 11; // MPP = M

        // mcause: preserve the interrupt flag (bit 31) + the full cause
        // code. The trap-entry path only ever sees legal causes (the
        // executor's trap-entry call sites use the `cause::` constants),
        // so no WARL rounding is needed here — hardware-delivered traps
        // always have legal causes by construction. P4 will deliver
        // interrupts through this path with bit 31 set.
        self.csrs.mcause = cause;

        // mtval is hardwired 0 — do NOT store _tval.
        // self.csrs.mtval stays 0.

        // A-extension §8.3: clear this hart's LR/SC reservation on trap
        // entry. Any outstanding reservation would otherwise survive the
        // trap and potentially be consumed by a later sc.w in the handler
        // (a spec-recommended invalidation point).
        bus.reservation[self.hart_id as usize] = None;

        // Set PC: mtvec[0] = mode (0=direct, 1=vectored). Base is bits
        // [31:2] ANDed to word-align. Bit 1 is hardwired 0 (already
        // enforced by the mtvec write path).
        let base = self.csrs.mtvec & !0b11;
        let mode = self.csrs.mtvec & 0b1;
        let interrupt = (cause & 0x8000_0000) != 0;
        let code = cause & 0x7FFF_FFFF;
        self.pc = if mode == 1 && interrupt {
            // Vectored: only interrupts dispatch to per-cause slots;
            // exceptions still go to base.
            base.wrapping_add(4u32.wrapping_mul(code))
        } else {
            base
        };
    }

    /// `mret`: pop the trap. pc <- mepc; mstatus.MIE <- MPIE; MPIE <- 1;
    /// MPP <- 0b11 (WARL — only M-mode in V1).
    pub(crate) fn mret(&mut self) {
        self.pc = self.csrs.mepc;
        let mpie_bit = (self.csrs.mstatus >> 7) & 1;
        // Clear MIE + MPIE + MPP.
        self.csrs.mstatus &= !((1 << 3) | (1 << 7) | (0b11 << 11));
        // MIE <- old MPIE.
        self.csrs.mstatus |= mpie_bit << 3;
        // MPIE <- 1 (RV-priv §3.3.2).
        self.csrs.mstatus |= 1 << 7;
        // MPP <- M-mode. Spec says "set to the least-privileged mode";
        // with only M-mode implemented, the WARL result is 0b11.
        self.csrs.mstatus |= 0b11 << 11;
    }
}
