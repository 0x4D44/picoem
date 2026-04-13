pub mod registers;
mod decode;
mod execute;

use crate::bus::Bus;
pub use registers::Registers;

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
        }
    }

    /// Advance the core by one system clock cycle.
    pub fn step(&mut self, bus: &mut Bus) {
        if self.stall_cycles > 0 {
            self.stall_cycles -= 1;
            return;
        }
        let cycles = self.decode_execute(bus);
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
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(2));
        let mut bus = Bus::default();
        self.execute_thumb16(opcode, &mut bus)
    }

    /// Execute a single 16-bit instruction with a provided bus.
    pub fn execute_one_with_bus(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(2));
        self.execute_thumb16(opcode, bus)
    }

    /// Execute a single 32-bit Thumb-2 instruction directly.
    /// Advances PC by 4 before execution.
    pub fn execute_one_wide(&mut self, hw0: u16, hw1: u16) -> u32 {
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        self.regs.set_pc(pc.wrapping_add(4));
        let mut bus = Bus::default();
        self.execute_thumb32(hw0, hw1, &mut bus)
    }

    /// The ARM-defined "read PC" value during instruction execution:
    /// current instruction address + 4.
    #[inline(always)]
    fn read_pc(&self) -> u32 {
        self.current_instr_addr.wrapping_add(4)
    }
}

impl Default for CortexM33 {
    fn default() -> Self {
        Self::new()
    }
}
