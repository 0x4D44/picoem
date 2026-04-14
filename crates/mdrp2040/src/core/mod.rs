//! Cortex-M0+ CPU core (ARMv6-M).
//!
//! Phase 4.A: full Thumb-16 decode + execute for every encoding the
//! ARMv6-M ISA supports. Thumb-32 subset (BL / MRS / MSR / DSB / DMB /
//! ISB), the exception model (stacking, EXC_RETURN, vector table),
//! unaligned-access fault, `Emulator::step` integration, and bus
//! contention land in Phase 4.B and Phase 5.
//!
//! M0+ is a strict subset of the M33 register/decode path: no IT blocks,
//! no CBZ/CBNZ, no security state, no FP, no MPU, no wide-path handling
//! from inside Thumb-16.

pub mod registers;
pub(crate) mod decode;
mod execute;

use crate::bus::Bus;
pub use registers::Registers;

/// Synchronous faults raised during instruction execution.
///
/// ARMv6-M has a single synchronous-fault vector (HardFault). Phase 4.B
/// turns each of these variants into HardFault (exception #3) via the
/// fault-delivery path. Keeping the variants distinct lets the fault
/// path record why the fault was taken even though they all share a
/// vector.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum Fault {
    /// Undefined instruction — decoder rejected the encoding.
    Undefined,
    /// Unaligned access — Phase 4.B will drive this from the bus path.
    #[allow(dead_code)]
    Unaligned,
    /// SVC / BKPT-initiated fault — Phase 4.B delivers as HardFault when
    /// the SVC handler path lands.
    #[allow(dead_code)]
    HardFault,
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
