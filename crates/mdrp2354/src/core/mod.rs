pub mod registers;
mod decode;
mod execute;
pub(crate) mod execute_thumb32;
mod execute_fpu;
pub(crate) mod exceptions;
pub(crate) mod coprocessor;

use crate::bus::Bus;
pub use registers::Registers;

/// Synchronous faults raised during instruction execution.
#[derive(Debug, Clone, Copy)]
pub(crate) enum Fault {
    UsageFault,
    // BusFault is delivered via bus.bus_fault() flag, not this enum
}

/// Cortex-M33 CPU core.
pub struct CortexM33 {
    pub regs: Registers,
    /// Cycles remaining on current multi-cycle operation.
    /// When > 0, step() decrements instead of fetching.
    stall_cycles: u32,
    /// Core ID (0 or 1).
    core_id: u8,
    /// Address of the currently executing instruction. Used to compute
    /// "read PC" value (instr_addr + 4) per ARM architecture definition.
    current_instr_addr: u32,
    /// IT block state. Format: cond[7:4]:mask[3:0]. mask=0 means not in IT block.
    it_state: u8,
    /// Pending synchronous fault from the most recent instruction.
    pub(crate) pending_fault: Option<Fault>,
    /// RCP (CP7) salt value.
    pub(crate) rcp_salt: u32,
    /// DCP (CP4/5) transfer registers.
    pub(crate) dcp_data: [u32; 2],
    /// ARM security state. `true` = Secure, `false` = Non-Secure.
    pub(crate) secure: bool,
}

impl CortexM33 {
    pub fn new() -> Self {
        Self::with_id(0)
    }

    pub fn with_id(core_id: u8) -> Self {
        Self {
            regs: Registers::new(),
            stall_cycles: 0,
            core_id,
            current_instr_addr: 0,
            it_state: 0,
            pending_fault: None,
            rcp_salt: 0,
            dcp_data: [0; 2],
            secure: true,
        }
    }

    /// Advance the core by one system clock cycle.
    pub fn step(&mut self, bus: &mut Bus) {
        if self.stall_cycles > 0 {
            self.stall_cycles -= 1;
            return;
        }
        let mut cycles = self.decode_execute(bus);

        // Synchronous bus fault
        let mut fault_handled = false;
        if bus.bus_fault() {
            fault_handled = true;
            let core = bus.active_core();
            let busfault_ena = bus.ppb[core].shcsr & (1 << 17) != 0;
            bus.ppb[core].cfsr |= (1 << 9) | (1 << 15); // PRECISERR + BFARVALID
            bus.ppb[core].bfar = bus.bus_fault_addr();
            bus.clear_bus_fault();
            if busfault_ena {
                cycles = self.enter_exception(5, bus);
            } else {
                bus.ppb[bus.active_core()].hfsr |= 1 << 30;
                cycles = self.enter_exception(3, bus);
            }
        }

        // Synchronous instruction fault (skip if bus fault already handled —
        // taking both would double-stack; Phase 3 takes only the first)
        if !fault_handled {
            if let Some(fault) = self.pending_fault.take() {
                cycles = self.deliver_fault(fault, bus);
            }
        } else {
            self.pending_fault = None;
        }

        self.stall_cycles = cycles.saturating_sub(1);
    }

    /// Returns the core ID (0 or 1).
    pub fn id(&self) -> u8 {
        self.core_id
    }

    /// Returns remaining stall cycles (for testing/debugging).
    pub fn stall_cycles(&self) -> u32 {
        self.stall_cycles
    }

    /// Swap all banked register pairs between Secure and Non-Secure.
    fn swap_security_banks(&mut self) {
        self.regs.sync_sp_to_banked();
        std::mem::swap(&mut self.regs.msp, &mut self.regs.msp_ns);
        std::mem::swap(&mut self.regs.psp, &mut self.regs.psp_ns);
        std::mem::swap(&mut self.regs.msplim, &mut self.regs.msplim_ns);
        std::mem::swap(&mut self.regs.psplim, &mut self.regs.psplim_ns);
        std::mem::swap(&mut self.regs.primask, &mut self.regs.primask_ns);
        std::mem::swap(&mut self.regs.basepri, &mut self.regs.basepri_ns);
        std::mem::swap(&mut self.regs.faultmask, &mut self.regs.faultmask_ns);
        std::mem::swap(&mut self.regs.control, &mut self.regs.control_ns);
        self.regs.sync_sp_from_banked();
    }

    /// Transition from Secure to Non-Secure state.
    /// Swaps all banked register pairs so the active set reflects NS state.
    pub(crate) fn transition_to_nonsecure(&mut self) {
        debug_assert!(self.secure);
        self.secure = false;
        self.swap_security_banks();
    }

    /// Transition from Non-Secure to Secure state (SG instruction).
    /// Swaps all banked register pairs so the active set reflects S state.
    pub(crate) fn transition_to_secure(&mut self) {
        debug_assert!(!self.secure);
        self.secure = true;
        self.swap_security_banks();
    }

    /// Halt the core indefinitely (stall for u32::MAX cycles).
    /// Used to hold Core 1 during reset.
    pub fn halt(&mut self) {
        self.stall_cycles = u32::MAX;
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

    /// Execute a single 16-bit Thumb instruction directly (bypasses fetch).
    /// Advances PC by 2 before execution, matching decode_execute behaviour.
    /// Returns cycle count.
    pub fn execute_one(&mut self, opcode: u16) -> u32 {
        self.pending_fault = None;
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(2));
        let mut bus = Bus::default();
        self.execute_thumb16(opcode, &mut bus)
    }

    /// Execute a single 16-bit instruction with a provided bus.
    pub fn execute_one_with_bus(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        self.pending_fault = None;
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(2));
        self.execute_thumb16(opcode, bus)
    }

    /// Execute a single 32-bit Thumb-2 instruction directly.
    /// Advances PC by 4 before execution.
    pub fn execute_one_wide(&mut self, hw0: u16, hw1: u16) -> u32 {
        self.pending_fault = None;
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(4));
        let mut bus = Bus::default();
        self.execute_thumb32(hw0, hw1, &mut bus)
    }

    /// Execute a single 32-bit Thumb-2 instruction with a provided bus.
    pub fn execute_one_wide_with_bus(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        self.pending_fault = None;
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(4));
        self.execute_thumb32(hw0, hw1, bus)
    }

    /// The ARM-defined "read PC" value during instruction execution:
    /// current instruction address + 4.
    #[inline(always)]
    fn read_pc(&self) -> u32 {
        self.current_instr_addr.wrapping_add(4)
    }

    /// Advance IT block state after executing one instruction inside an IT block.
    /// Shifts the mask left; clears it_state entirely when the last instruction completes.
    fn advance_it_state(&mut self) {
        if self.it_state & 0x7 == 0 {
            self.it_state = 0; // last instruction in block
        } else {
            self.it_state = (self.it_state & 0xE0) | ((self.it_state << 1) & 0x1F);
        }
    }

    /// Returns current IT block state (for testing).
    pub fn it_state(&self) -> u8 {
        self.it_state
    }
}

impl Default for CortexM33 {
    fn default() -> Self {
        Self::new()
    }
}
