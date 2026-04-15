pub mod registers;
pub(crate) mod decode;
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
    // Constructed by:
    //   * Phase 7 Stage B — lazy-FP flush fault when the FP frame's
    //     destination page is unmapped by the MPU.
    //   * Phase 7 Stage E — MPU TT path and any other MPU-sourced
    //     data-access fault.
    #[allow(dead_code)]
    MemManage,
    /// Raised by CP7 RCP assertion failure (Phase 7 Stage E) — delivered
    /// as exception #2 (NMI). Not masked by PRIMASK; FAULTMASK is honored
    /// by the upstream step() path (no delivery-site re-check).
    Nmi,
    // BusFault is delivered via bus.bus_fault() flag, not this enum
}

/// Cortex-M33 CPU core.
pub struct CortexM33 {
    pub regs: Registers,
    /// Monotonically increasing per-core cycle count.
    /// Each call to `step()` advances this by the executed instruction's
    /// cycle cost (including any exception-entry cost). Used by the
    /// quantum scheduler to decide when a core has caught up to the
    /// quantum's target cycle, and by DWT CYCCNT reads (Stage 2).
    pub(crate) cycles: u64,
    /// Core ID (0 or 1).
    core_id: u8,
    /// Address of the currently executing instruction. Used to compute
    /// "read PC" value (instr_addr + 4) per ARM architecture definition.
    current_instr_addr: u32,
    /// IT block state. Format: cond[7:4]:mask[3:0]. mask=0 means not in IT block.
    it_state: u8,
    /// Pending synchronous fault from the most recent instruction.
    pub(crate) pending_fault: Option<Fault>,
    /// DCP (CP4/5) half-word register file. Eight double-precision slots
    /// (indexed 0..7), each made of two 32-bit halves: half A (low) at
    /// index `d*2`, half B (high) at index `d*2 + 1`. Layout matches
    /// RP2350 datasheet §3.6.7 (double-precision coprocessor).
    pub(crate) dcp_halves: [u32; 16],
    /// DCP status register. After each arithmetic op, cleared and then:
    ///   bit 0 — result is zero
    ///   bit 1 — result is negative
    ///   bit 2 — result is infinity
    ///   bit 3 — result is NaN
    /// Compare ops set bit 0 on success, cleared on failure.
    pub(crate) dcp_status: u32,
    /// ARM security state. `true` = Secure, `false` = Non-Secure.
    pub(crate) secure: bool,
    /// Core is halted — will not execute until explicitly woken.
    halted: bool,
    /// Core is sleeping on WFE — will resume when event_flag is set.
    pub(crate) wfe_waiting: bool,
}

impl CortexM33 {
    pub fn new() -> Self {
        Self::with_id(0)
    }

    pub fn with_id(core_id: u8) -> Self {
        Self {
            regs: Registers::new(),
            cycles: 0,
            core_id,
            current_instr_addr: 0,
            it_state: 0,
            pending_fault: None,
            dcp_halves: [0; 16],
            dcp_status: 0,
            secure: true,
            halted: false,
            wfe_waiting: false,
        }
    }

    /// Execute one instruction atomically, advancing the core's own cycle
    /// count by the instruction's cycle cost (including any exception-entry
    /// cost if a synchronous fault is taken).
    pub fn step(&mut self, bus: &mut Bus) {
        if self.wfe_waiting {
            return;
        }
        if self.halted {
            return;
        }

        // ARMv8-M §B1.5.8 + §B3.7: take the highest-priority pending
        // exception at this instruction boundary before fetching the next
        // instruction. Unified arbitration over NMI + PendSV + SysTick +
        // external NVIC IRQs, so an external IRQ with a higher priority
        // than a pending PendSV/SysTick wins (and vice-versa). Covers
        // firmware pends via ICSR, peripheral asserts via `assert_irq_core`,
        // and tail-chain-as-re-entry after EXC_RETURN — the subsequent
        // step's top-of-loop check sees the still-pending exception.
        if let Some(cost) = self.try_take_any_pending_exception(bus) {
            self.cycles = self.cycles.wrapping_add(cost as u64);
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

        self.cycles = self.cycles.wrapping_add(cycles as u64);
    }

    /// Returns the core ID (0 or 1).
    pub fn id(&self) -> u8 {
        self.core_id
    }

    /// Returns the per-core cycle count. Monotonically increasing; used by
    /// the quantum scheduler and by DWT CYCCNT (Stage 2).
    pub fn cycles(&self) -> u64 {
        self.cycles
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

    /// Halt the core indefinitely — will not execute until explicitly woken.
    /// Used to hold Core 1 during reset.
    pub fn halt(&mut self) {
        self.halted = true;
        self.pending_fault = None;
    }

    /// Resume a halted core. The caller must set PC, SP, and xpsr before
    /// calling this — wake() only clears the halted flag.
    pub fn wake(&mut self) {
        self.halted = false;
    }

    /// Returns `true` if the core is halted.
    pub fn is_halted(&self) -> bool {
        self.halted
    }

    /// Returns `true` if the core is sleeping on WFE.
    pub fn is_wfe_waiting(&self) -> bool {
        self.wfe_waiting
    }

    /// Execute WFE hint. If event_flag is pending, consume it and continue.
    /// Otherwise, enter WFE sleep.
    pub(crate) fn wfe(&mut self, bus: &mut Bus) -> u32 {
        let core = bus.active_core();
        if bus.event_flag[core] {
            bus.event_flag[core] = false;
            1 // event was pending, consume it, no sleep
        } else {
            self.wfe_waiting = true;
            1
        }
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

    // --- DCP (CP4/5) test/harness accessors (Phase 7 Stage D) ---

    /// Read one 32-bit half of the DCP register file. `half_idx` is
    /// `d*2 + (0 for half A, 1 for half B)`.
    pub fn dcp_get_half(&self, half_idx: usize) -> u32 {
        self.dcp_halves[half_idx]
    }

    /// Write one 32-bit half of the DCP register file.
    pub fn dcp_set_half(&mut self, half_idx: usize, value: u32) {
        self.dcp_halves[half_idx] = value;
    }

    /// Read the DCP status register (four result-classification bits).
    pub fn dcp_get_status(&self) -> u32 {
        self.dcp_status
    }

    /// Read a DCP double-precision value (index 0..7).
    pub fn dcp_get_double(&self, idx: usize) -> f64 {
        let lo = self.dcp_halves[idx * 2] as u64;
        let hi = self.dcp_halves[idx * 2 + 1] as u64;
        f64::from_bits((hi << 32) | lo)
    }

    /// Write a DCP double-precision value (index 0..7).
    pub fn dcp_set_double(&mut self, idx: usize, v: f64) {
        let bits = v.to_bits();
        self.dcp_halves[idx * 2] = bits as u32;
        self.dcp_halves[idx * 2 + 1] = (bits >> 32) as u32;
    }

    // --- Phase 7 Stage B test/integration accessors ----------------------

    /// True if a synchronous fault is pending delivery on the next step().
    /// Used by integration tests to observe lazy-FP and stack-limit faults
    /// without needing to wire up a fault handler.
    #[doc(hidden)]
    pub fn has_pending_fault(&self) -> bool {
        self.pending_fault.is_some()
    }

    /// Direct exception entry — wraps the crate-internal `enter_exception`
    /// for integration tests that want to drive the FP-frame paths
    /// without synthesizing instructions.
    #[doc(hidden)]
    pub fn test_enter_exception(&mut self, exc_num: u16, bus: &mut Bus) -> u32 {
        self.enter_exception(exc_num, bus)
    }

    /// Direct exception return — wraps the crate-internal `exit_exception`.
    #[doc(hidden)]
    pub fn test_exit_exception(&mut self, exc_return: u32, bus: &mut Bus) -> u32 {
        self.exit_exception(exc_return, bus)
    }
}

impl Default for CortexM33 {
    fn default() -> Self {
        Self::new()
    }
}
