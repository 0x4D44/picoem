//! Cortex-M0+ skeleton. Phase 3 contains only enough structure for the
//! `Emulator::reset` path to compile and write SP/PC into the core's
//! registers. Decode, execute, exception model, IT blocks, etc. are
//! Phase 4.

/// Synchronous faults raised during instruction execution.
///
/// ARMv6-M only has a single fault vector (HardFault) — unlike ARMv7-M+,
/// there is no separate MemManage/BusFault/UsageFault. Phase 4 will add
/// further variants if the decode/execute path needs finer-grained
/// dispatch internally.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum Fault {
    HardFault,
}

/// Register file — bare minimum for Phase 3 reset wiring.
///
/// Phase 4 will expand this to mirror the ARMv6-M programmer's model
/// (CONTROL, PRIMASK, separate APSR/IPSR/EPSR views, etc.).
#[derive(Default)]
pub struct Registers {
    /// R0-R15. r[13] is SP, r[14] is LR, r[15] is PC.
    pub r: [u32; 16],
    /// Program status register.
    pub xpsr: u32,
    /// Main stack pointer (banked copy of r[13] when CONTROL.SPSEL=0).
    pub msp: u32,
    /// Process stack pointer (banked copy of r[13] when CONTROL.SPSEL=1).
    pub psp: u32,
}

impl Registers {
    /// Set PC (r[15]). Phase 4 will replace this with a proper
    /// branch-write helper that handles the T-bit.
    pub fn set_pc(&mut self, pc: u32) {
        self.r[15] = pc;
    }
}

/// Cortex-M0+ CPU core. Skeleton for Phase 3.
pub struct CortexM0Plus {
    pub regs: Registers,
    core_id: u8,
    /// Monotonically increasing per-core cycle count. Unused in Phase 3.
    pub cycles: u64,
    halted: bool,
    /// Address of the currently executing instruction. Used to compute
    /// the architectural "read PC = instr_addr + 4" value per the ARMv6-M
    /// definition. Phase 4 will wire this up in the decode/execute path.
    #[allow(dead_code)]
    pub(crate) current_instr_addr: u32,
    /// Pending synchronous fault from the most recent instruction.
    /// Phase 4 uses this to defer HardFault delivery until after the
    /// current instruction retires.
    #[allow(dead_code)]
    pub(crate) pending_fault: Option<Fault>,
}

impl CortexM0Plus {
    pub fn new() -> Self {
        Self::with_id(0)
    }

    pub fn with_id(core_id: u8) -> Self {
        Self {
            regs: Registers::default(),
            core_id,
            cycles: 0,
            halted: false,
            current_instr_addr: 0,
            pending_fault: None,
        }
    }

    /// Core ID (0 or 1).
    pub fn id(&self) -> u8 {
        self.core_id
    }

    /// Register accessor.
    pub fn reg(&self, n: usize) -> u32 {
        self.regs.r[n]
    }

    /// Register mutator.
    pub fn set_reg(&mut self, n: usize, val: u32) {
        self.regs.r[n] = val;
    }

    /// Whether the core is halted.
    pub fn is_halted(&self) -> bool {
        self.halted
    }
}

impl Default for CortexM0Plus {
    fn default() -> Self {
        Self::new()
    }
}
