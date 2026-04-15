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
    /// NVIC pending latch — peripheral-asserted external IRQs land
    /// here via [`crate::Emulator::drain_pending_irqs_to_cores`]. Full
    /// ISER / ICER / IPR decode lands in a later wave; Phase 1 only
    /// needs the pending bits.
    pub nvic: Nvic,
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
            nvic: Nvic::new(),
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
    /// Returns the cycle count consumed (instruction + any exception
    /// entry on fault delivery).
    pub fn step(&mut self, bus: &mut Bus) -> u32 {
        if self.halted {
            return 0;
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
