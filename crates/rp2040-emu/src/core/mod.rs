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

pub mod bus_trait;
pub(crate) mod decode;
pub(crate) mod exceptions;
mod execute;
mod execute_wide;
pub mod nvic;
pub mod registers;

use tracing::info;

use crate::bus::Bus;
pub use bus_trait::CoreBus;
pub use nvic::Nvic;
pub use registers::Registers;

/// Synchronous faults raised during instruction execution.
///
/// ARMv6-M has a single synchronous-fault vector (HardFault) plus the
/// SVC call (exception 11). Phase 4.B turns these variants into the
/// appropriate exception number via [`CortexM0Plus::deliver_fault`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    /// PC-keyed decoded-op cache. Direct-mapped,
    /// [`crate::bus::DECODE_CACHE_SIZE`] entries × 12 B = 96 KB per
    /// core. Populated lazily on fetch by
    /// [`Self::populate_decode_cache`]; invalidated by the driver after
    /// each `step()` by draining
    /// [`crate::bus::Bus::pending_cache_invalidations`] into
    /// [`Self::invalidate_decode_cache_entries`]. Bulk-invalidated by
    /// `load_bootrom` / `load_flash` / `load_image` / firmware ISB via
    /// [`Self::invalidate_decode_cache_regions`] (region-scoped) or
    /// [`Self::invalidate_decode_cache_all`] (everything). Modelled on
    /// the rp2350_emu per-core cache (commit `0c31479`).
    pub(crate) decode_cache: Box<[crate::bus::DecodedOp; crate::bus::DECODE_CACHE_SIZE]>,
}

impl CortexM0Plus {
    pub fn new() -> Self {
        Self::with_id(0)
    }

    pub fn with_id(core_id: u8) -> Self {
        use crate::bus::{DECODE_CACHE_SIZE, DecodedOp};
        // 96 KB heap allocation per core — can't live on the stack.
        // Every slot starts with `tag = u32::MAX` so lookups never
        // spuriously hit before the first populate.
        let decode_cache: Box<[DecodedOp; DECODE_CACHE_SIZE]> =
            vec![DecodedOp::empty(); DECODE_CACHE_SIZE]
                .into_boxed_slice()
                .try_into()
                .expect("length matches DECODE_CACHE_SIZE by construction");
        Self {
            regs: Registers::new(),
            cycles: 0,
            core_id,
            current_instr_addr: 0,
            pending_fault: None,
            halted: false,
            decode_cache,
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

    /// Execute a WFE hint. If the per-core event flag is latched,
    /// consume it (atomic swap-to-false) and fall through; otherwise
    /// park the core by setting `wfe_waiting`. Mirrors the rp2350_emu
    /// `CortexM33::wfe` helper but routes through the [`CoreBus`]
    /// trait because `CortexM0Plus` does not own an `Arc<CoreAtomics>`.
    /// See `wrk_docs/2026.04.26 - HLD - RP2040 WFE-SEV Wake Mechanics
    /// V1.md` §4.2.
    pub(crate) fn wfe<B: CoreBus>(&mut self, bus: &mut B) {
        let core = self.core_id as usize;
        if bus.consume_event_flag(core) {
            // Latched event consumed; do not park.
        } else {
            bus.set_wfe_waiting(core, true);
        }
    }

    /// Reset thread-mode architectural state before a multicore-launch
    /// wake. Mirrors `Emulator::reset`'s per-core init (`lib.rs:83-96`),
    /// but scoped to the fields that could leak across a halt/launch
    /// cycle (T5 "rehalt then relaunch" scenario):
    ///
    /// * `control = 0`    — SPSEL=0, so r13 aliases MSP after launch.
    /// * `psp     = 0`    — no stale process-stack pointer.
    /// * `xpsr    = 1<<24` — T bit set (ARMv6-M is Thumb-only), all
    ///   other xPSR bits (including the IPSR field at bits [8:0])
    ///   cleared. This puts the core in thread mode with NZCV=0.
    /// * `primask = 0`    — interrupts un-masked.
    ///
    /// Does NOT touch R0-R12, PC, MSP, or the halted/cycle counters —
    /// those are either set explicitly by the launch consumer (PC, MSP)
    /// or intentionally preserved (R0-R12 convey arguments; cycle
    /// counters are monotonic).
    pub fn reset_control_for_launch(&mut self) {
        self.regs.control = 0;
        self.regs.psp = 0;
        self.regs.xpsr = 1 << 24;
        self.regs.primask = 0;
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

    /// True iff the CPU is currently executing the HardFault handler,
    /// i.e. IPSR == 3. Used by harness integration tests to distinguish
    /// a misdispatch (HardFault) from a regular FAIL (counter mismatch).
    #[inline]
    pub fn is_in_hardfault(&self) -> bool {
        (self.regs.xpsr & 0x1FF) == 3
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
    pub fn step<B: CoreBus>(&mut self, bus: &mut B) -> u32 {
        if self.halted {
            return 0;
        }

        // Pre-fetch exception poll + dispatch (PendSV / SysTick / external
        // IRQ, all arbitrated in one pass). Returns the cycle cost of
        // exception entry if one was taken; `0` otherwise.
        let exc_cycles = self.try_take_any_pending_exception(bus);
        if exc_cycles != 0 {
            self.cycles = self.cycles.wrapping_add(exc_cycles as u64);
            return exc_cycles;
        }

        let mut cycles = self.decode_execute(bus);

        // Synchronous bus fault — unmapped loads/stores or XIP-before-
        // flash-loaded accesses set bus.bus_fault. On ARMv6-M (M0+) every
        // synchronous fault escalates to the single HardFault vector (#3),
        // so stage the HardFault and let deliver_fault drive entry. If the
        // instruction also raised a pending_fault, the bus fault takes
        // precedence (clearing the other keeps us from double-stacking).
        if bus.bus_fault() {
            info!(
                pc = format_args!("{:#010x}", self.current_instr_addr),
                addr = format_args!("{:#010x}", bus.bus_fault_addr()),
                "HardFault escalation from bus fault"
            );
            bus.clear_bus_fault();
            self.pending_fault = Some(Fault::HardFault);
        }

        if let Some(fault) = self.pending_fault.take() {
            cycles = cycles.wrapping_add(self.deliver_fault(fault, bus));
        }

        self.cycles = self.cycles.wrapping_add(cycles as u64);
        cycles
    }

    /// Poll all pending exception sources (PendSV via ICSR.PENDSVSET,
    /// SysTick via ICSR.PENDSTSET, and per-core NVIC IRQs) and dispatch
    /// the highest-priority candidate. HLD V5 §5.3.
    ///
    /// Selection rule:
    /// 1. PRIMASK=1 masks every configurable-priority exception, so we
    ///    return 0 immediately. NMI/HardFault are not driven through
    ///    this path — they enter via `deliver_fault`.
    /// 2. Candidates are PendSV (#14, priority via SHPR3 byte 2),
    ///    SysTick (#15, priority via SHPR3 byte 3), and the
    ///    highest-priority pending+enabled NVIC IRQ.
    /// 3. Lower numerical priority value wins; tie-break by lower
    ///    exception number (ARMv6-M ARM §B1.5.10 — system exceptions
    ///    sit below external IRQs in the tie-break order because their
    ///    exception numbers are lower).
    /// 4. `can_dispatch_now` gates the dispatch — V1 keeps the existing
    ///    "no preemption" rule (see doc comment).
    ///
    /// On dispatch we clear the latch for the chosen candidate
    /// (`ICSR.PENDSVSET`/`PENDSTSET` for system exceptions, NVIC pending
    /// bit for external IRQs) and run `enter_exception`. Returns the
    /// cycle count of exception entry (non-zero on dispatch, 0 otherwise).
    fn try_take_any_pending_exception<B: CoreBus>(&mut self, bus: &mut B) -> u32 {
        if self.regs.primask & 1 != 0 {
            return 0;
        }

        let core = self.core_id as usize;
        let icsr = bus.ppb(core).icsr;
        let pendsv = icsr & (1 << 28) != 0;
        let pendst = icsr & (1 << 26) != 0;

        // (priority, exception_number); lower priority value wins,
        // tie-break by lower exception number.
        let mut best: Option<(u8, u16)> = None;

        if pendsv {
            best = Some((bus.ppb(core).exception_priority(14) as u8, 14));
        }
        if pendst {
            let p = bus.ppb(core).exception_priority(15) as u8;
            best = match best {
                None => Some((p, 15)),
                Some((bp, be)) if p < bp || (p == bp && 15 < be) => Some((p, 15)),
                other => other,
            };
        }
        if let Some((irq, p)) = bus.nvic(core).highest_priority_pending() {
            let exc = 16u16 + irq as u16;
            best = match best {
                None => Some((p, exc)),
                Some((bp, be)) if p < bp || (p == bp && exc < be) => Some((p, exc)),
                other => other,
            };
        }

        let Some((_, candidate)) = best else { return 0 };
        if !self.can_dispatch_now(bus) {
            return 0;
        }

        match candidate {
            14 => bus.ppb_mut(core).icsr &= !(1 << 28),
            15 => bus.ppb_mut(core).icsr &= !(1 << 26),
            e => bus.nvic_mut(core).clear_pending((e - 16) as u8),
        }
        self.enter_exception(candidate, bus)
    }

    /// V1 dispatch gate: `true` iff no exception is currently active.
    ///
    /// This is **stricter** than ARMv6-M ARM §B1.5.4, which permits a
    /// higher-priority exception to preempt a lower-priority handler.
    /// V1 follows the existing `maybe_dispatch_external_irq` behaviour
    /// because the V1 oracle scenarios don't exercise preemption (all
    /// dispatches enter from thread mode). Tracked as deferred follow-up
    /// in HLD §9.2.
    fn can_dispatch_now<B: CoreBus>(&self, bus: &B) -> bool {
        !bus.ppb(self.core_id as usize).any_active()
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

    // --- Decode cache invalidation ---------------------------------------
    //
    // Modelled on the rp2350_emu helpers (commit 0c31479). Same shape, no
    // `flag_only` flag, no `fetch_wait` — the M0+ entry has only `tag`,
    // `hw0`, `hw1`, `flags`.

    /// Invalidate this core's decode-cache entries for the supplied
    /// addresses. Drained from `Bus::pending_cache_invalidations` after
    /// each `core.step()` by `Emulator::step_serial`.
    ///
    /// Clears the direct-mapped slot
    /// `((addr >> 1) & (DECODE_CACHE_SIZE - 1))` for each cacheable
    /// address, plus the preceding slot (so a wide instruction's `hw0`
    /// at `addr - 2` whose `hw1` is rewritten gets evicted too).
    /// Non-cacheable addresses are skipped.
    pub fn invalidate_decode_cache_entries(&mut self, addrs: &[u32]) {
        use crate::bus::{DECODE_CACHE_SIZE, DecodedOp, is_cacheable_pc};
        const MASK: u32 = (DECODE_CACHE_SIZE as u32) - 1;
        let empty = DecodedOp::empty();
        for &addr in addrs {
            let aligned = addr & !1;
            let prev = aligned.wrapping_sub(2);
            if is_cacheable_pc(prev) {
                let slot = ((prev >> 1) & MASK) as usize;
                self.decode_cache[slot] = empty;
            }
            if is_cacheable_pc(aligned) {
                let slot = ((aligned >> 1) & MASK) as usize;
                self.decode_cache[slot] = empty;
            }
        }
    }

    /// Invalidate decode-cache entries that back one or more regions,
    /// selected by `regions` (see
    /// [`crate::bus::invalidation_regions`]). Unaffected slots stay hot
    /// — `load_flash` no longer evicts SRAM-resident code, so firmware
    /// that reloads flash then runs SRAM code doesn't pay a cold tax.
    ///
    /// If `regions` has the [`crate::bus::invalidation_regions::BULK`]
    /// bit set, every slot is cleared regardless of tag — same as
    /// [`Self::invalidate_decode_cache_all`].
    ///
    /// If `regions == 0`, this is a no-op.
    pub fn invalidate_decode_cache_regions(&mut self, regions: u8) {
        use crate::bus::{DecodedOp, invalidation_regions::BULK};
        if regions == 0 {
            return;
        }
        let empty = DecodedOp::empty();
        if regions & BULK != 0 {
            for slot in self.decode_cache.iter_mut() {
                *slot = empty;
            }
            return;
        }
        // Region-scoped sweep: the region of a cached tag is
        // `(tag >> 28) as u8` (ROM = 0, XIP = 1, SRAM = 2). Bit `n` of
        // `regions` matches region `n`. Empty slots
        // (`tag == u32::MAX`, nibble = 0xF) never match a valid region
        // bit, so they're skipped without special-casing.
        for slot in self.decode_cache.iter_mut() {
            let nibble = (slot.tag >> 28) as u8;
            if nibble < 8 && regions & (1 << nibble) != 0 {
                *slot = empty;
            }
        }
    }

    /// Invalidate every decode-cache entry on this core. Used by `ISB`
    /// and any path that globally invalidates the instruction pipeline.
    pub fn invalidate_decode_cache_all(&mut self) {
        use crate::bus::DecodedOp;
        let empty = DecodedOp::empty();
        for slot in self.decode_cache.iter_mut() {
            *slot = empty;
        }
    }
}

impl Default for CortexM0Plus {
    fn default() -> Self {
        Self::new()
    }
}
