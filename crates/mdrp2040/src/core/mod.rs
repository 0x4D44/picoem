//! Cortex-M0+ CPU core (ARMv6-M).
//!
//! Phase 4.A: full Thumb-16 decode + execute for every encoding the
//! ARMv6-M ISA supports.
//!
//! Phase 4.B: adds the Thumb-32 subset (BL / MRS / MSR / DSB / DMB /
//! ISB), the exception model (stacking, EXC_RETURN, vector walk),
//! unaligned-access fault, and `Emulator::step` integration. Bus
//! contention + full address decode remain Phase 5.
//!
//! M0+ is a strict subset of the M33 register/decode path: no IT blocks,
//! no CBZ/CBNZ, no security state, no FP, no MPU, no wide-path handling
//! from inside Thumb-16.

pub mod registers;
pub(crate) mod decode;
mod execute;
mod execute_wide;
pub(crate) mod exceptions;
pub mod nvic;

use crate::bus::Bus;
pub use nvic::Nvic;
pub use registers::Registers;

/// Synchronous faults raised during instruction execution.
///
/// ARMv6-M has a single synchronous-fault vector (HardFault) plus the
/// SVC call (exception 11). Phase 4.B turns these variants into the
/// appropriate exception number via [`CortexM0Plus::deliver_fault`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub(crate) enum Fault {
    /// Undefined instruction — decoder rejected the encoding. Delivers
    /// as HardFault (exception #3).
    Undefined,
    /// Unaligned word / halfword access. Delivers as HardFault.
    Unaligned,
    /// BKPT without a debugger attached. Delivers as HardFault — M0+
    /// has no DebugMonitor exception.
    HardFault,
    /// SVC #imm8 — delivers as SVCall (exception #11).
    Svc,
    /// EXC_RETURN with invalid magic bits [3:0] — delivers as HardFault.
    InvalidExcReturn,
    /// Branch target with Thumb bit clear. Delivers as HardFault.
    InvalidEpsr,
}

/// Cortex-M0+ CPU core.
pub struct CortexM0Plus {
    pub regs: Registers,
    /// Monotonically increasing per-core cycle count. Updated by the
    /// `step` integration in Phase 4.B; the Phase 4.A `execute_one` test
    /// accessors do not touch this field.
    pub cycles: u64,
    core_id: u8,
    /// Address of the currently executing instruction. Used to compute
    /// the architectural "read PC = instr_addr + 4" value per the
    /// ARMv6-M definition.
    pub(crate) current_instr_addr: u32,
    /// Pending synchronous fault from the most recent instruction.
    /// Phase 4.B consumes this after instruction retire and drives
    /// HardFault entry.
    pub(crate) pending_fault: Option<Fault>,
    /// Core is halted — will not execute until explicitly woken.
    halted: bool,
}

impl CortexM0Plus {
    pub fn new() -> Self {
        Self::with_id(0)
    }

    pub fn with_id(core_id: u8) -> Self {
        Self {
            regs: Registers::new(),
            cycles: 0,
            core_id,
            current_instr_addr: 0,
            pending_fault: None,
            halted: false,
        }
    }

    /// Core ID (0 or 1).
    pub fn id(&self) -> u8 {
        self.core_id
    }

    /// Per-core cycle count.
    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    /// Whether the core is halted.
    pub fn is_halted(&self) -> bool {
        self.halted
    }

    /// Halt the core indefinitely.
    pub fn halt(&mut self) {
        self.halted = true;
        self.pending_fault = None;
    }

    /// Resume a halted core.
    pub fn wake(&mut self) {
        self.halted = false;
    }

    // --- Test / debug accessors ---

    pub fn reg(&self, n: usize) -> u32 {
        self.regs.r[n]
    }

    pub fn set_reg(&mut self, n: usize, val: u32) {
        self.regs.r[n] = val;
    }

    pub fn flag_n(&self) -> bool {
        self.regs.flag_n()
    }

    pub fn flag_z(&self) -> bool {
        self.regs.flag_z()
    }

    pub fn flag_c(&self) -> bool {
        self.regs.flag_c()
    }

    pub fn flag_v(&self) -> bool {
        self.regs.flag_v()
    }

    /// True if a synchronous fault is pending delivery. Phase 4.B will
    /// drive fault entry from this flag.
    pub fn has_pending_fault(&self) -> bool {
        self.pending_fault.is_some()
    }

    /// Execute a single 16-bit Thumb instruction directly (bypasses
    /// fetch / bus timing). Advances PC by 2 before execution — matching
    /// the ARM architectural definition of "read PC = instr_addr + 4".
    /// Uses a default [`Bus`] with zero-cycle memory.
    pub fn execute_one(&mut self, opcode: u16) -> u32 {
        let mut bus = Bus::default();
        self.execute_one_with_bus(opcode, &mut bus)
    }

    /// Execute a single 16-bit Thumb instruction against the supplied
    /// [`Bus`]. Used by load/store unit tests that need to observe
    /// memory side effects.
    pub fn execute_one_with_bus(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        self.pending_fault = None;
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(2));
        self.execute_thumb16(opcode, bus)
    }

    /// Execute a single 32-bit Thumb-2 instruction directly (bypasses
    /// fetch). Advances PC by 4 before execution. Uses a default
    /// [`Bus`] with zero-cycle memory.
    pub fn execute_one_wide(&mut self, hw0: u16, hw1: u16) -> u32 {
        let mut bus = Bus::default();
        self.execute_one_wide_with_bus(hw0, hw1, &mut bus)
    }

    /// Execute a single 32-bit Thumb-2 instruction against the supplied
    /// [`Bus`].
    pub fn execute_one_wide_with_bus(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        self.pending_fault = None;
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(4));
        self.execute_thumb32(hw0, hw1, bus)
    }

    /// Fetch-decode-execute one instruction. Integrates pending-fault
    /// delivery with the exception model — Phase 4.B wiring.
    ///
    /// Phase 1 Wave 2 additions (HLD V7 §5.2): before instruction fetch
    /// the step path polls the per-core NVIC for a pending-and-enabled
    /// IRQ whose priority can preempt the current execution priority.
    /// If one exists and isn't masked by PRIMASK, exception entry runs
    /// against vector `16 + irq` and the instruction fetch is deferred
    /// to the next call. Otherwise we fall through to the normal
    /// fetch-decode-execute path.
    ///
    /// Returns the cycle count consumed (instruction + any exception
    /// entry on fault delivery).
    pub fn step(&mut self, bus: &mut Bus) -> u32 {
        if self.halted {
            return 0;
        }

        // IRQ poll + dispatch (before instruction fetch). Returns the
        // cycle cost of exception entry if one was taken; `0` otherwise.
        let irq_cycles = self.maybe_dispatch_external_irq(bus);
        if irq_cycles != 0 {
            self.cycles = self.cycles.wrapping_add(irq_cycles as u64);
            return irq_cycles;
        }

        let mut cycles = self.decode_execute(bus);

        // Synchronous bus fault — unmapped loads/stores or XIP-before-
        // flash-loaded accesses set bus.bus_fault. On ARMv6-M (M0+) every
        // synchronous fault escalates to the single HardFault vector (#3),
        // so stage the HardFault and let deliver_fault drive entry. If the
        // instruction also raised a pending_fault, the bus fault takes
        // precedence (clearing the other keeps us from double-stacking).
        if bus.bus_fault() {
            bus.clear_bus_fault();
            self.pending_fault = Some(Fault::HardFault);
        }

        if let Some(fault) = self.pending_fault.take() {
            cycles = cycles.wrapping_add(self.deliver_fault(fault, bus));
        }

        self.cycles = self.cycles.wrapping_add(cycles as u64);
        cycles
    }

    /// Poll the per-core NVIC for a dispatchable external IRQ.
    ///
    /// Selection rule (HLD V7 §5.2, adapted for M0+'s 4-level priority):
    /// 1. Mask to pending AND enabled (`nvic.pending_and_enabled()`).
    /// 2. If PRIMASK is set, dispatch no external IRQ (PRIMASK raises
    ///    execution priority to 0, which ties all configurable IRQ
    ///    priorities).
    /// 3. Otherwise pick the lowest-numerical-priority IRQ (lower value
    ///    = architecturally higher priority); tie-break by lowest IRQ
    ///    number (ARMv6-M ARM §B1.5.10).
    /// 4. Require the candidate's priority to be strictly less (higher)
    ///    than the current execution priority. Current execution
    ///    priority is 0 when we're already in any handler (we simplify:
    ///    any in-progress exception has priority 0 on M0+, so external
    ///    IRQs with configurable priority never preempt — tail-chaining
    ///    still works because it runs from `exit_exception` not here).
    ///
    /// Returns the cycle count of exception entry (non-zero on dispatch,
    /// `0` otherwise).
    fn maybe_dispatch_external_irq(&mut self, bus: &mut Bus) -> u32 {
        // PRIMASK blocks everything below NMI/HardFault priority. Per
        // ARMv6-M M0+, configurable priorities are 0x00..0xC0; PRIMASK=1
        // effectively sets the "current execution priority floor" to 0,
        // masking all external IRQs.
        if self.regs.primask & 1 != 0 {
            return 0;
        }

        // If already in a handler, don't preempt for an external IRQ.
        // M0+ has coarse priority and our model collapses handler
        // priority to "higher than any configurable". Tail-chain flows
        // through `exit_exception`; this path only dispatches from
        // thread mode.
        if self.regs.in_handler_mode() {
            return 0;
        }

        let core_idx = self.core_id as usize;
        let candidates = bus.nvics[core_idx].pending_and_enabled();
        if candidates == 0 {
            return 0;
        }

        // Scan for lowest priority value, tie-break by lowest IRQ
        // number. Only look at implemented IRQs (0..32; RP2040 uses
        // 0..26 but the NVIC itself is 32 lines wide).
        let mut best_irq: Option<u8> = None;
        let mut best_prio: u8 = 0xFF;
        for irq in 0u8..32 {
            if candidates & (1u32 << irq) == 0 {
                continue;
            }
            let p = bus.nvics[core_idx].priority[irq as usize];
            if best_irq.is_none() || p < best_prio {
                best_irq = Some(irq);
                best_prio = p;
            }
        }
        let Some(irq) = best_irq else { return 0 };

        // Dispatch: clear pending, run exception entry against vector
        // 16 + irq. The NVIC pending bit stays clear until the source
        // re-asserts (level peripheral) or firmware writes NVIC_ISPR
        // (software-set).
        bus.nvics[core_idx].clear_pending(irq);
        self.enter_exception(16u16 + irq as u16, bus)
    }

    /// Test helper — direct exception entry without synthesising an
    /// instruction. Used by the exception-model unit tests.
    #[doc(hidden)]
    pub fn test_enter_exception(&mut self, exc_num: u16, bus: &mut Bus) -> u32 {
        self.enter_exception(exc_num, bus)
    }

    /// Test helper — direct exception return. Used by the
    /// exception-model unit tests.
    #[doc(hidden)]
    pub fn test_exit_exception(&mut self, exc_return: u32, bus: &mut Bus) -> u32 {
        self.exit_exception(exc_return, bus)
    }

    /// The ARM-defined "read PC" value during instruction execution:
    /// current instruction address + 4.
    #[inline(always)]
    pub(crate) fn read_pc(&self) -> u32 {
        self.current_instr_addr.wrapping_add(4)
    }
}

impl Default for CortexM0Plus {
    fn default() -> Self {
        Self::new()
    }
}
