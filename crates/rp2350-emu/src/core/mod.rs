pub mod bus_trait;
pub(crate) mod coprocessor;
pub(crate) mod decode;
pub(crate) mod exceptions;
mod execute;
#[cfg(not(test))]
mod execute_fpu;
#[cfg(test)]
pub(crate) mod execute_fpu;
pub(crate) mod execute_thumb32;
pub mod registers;

use std::sync::Arc;

use picoem_common::Divider;

use tracing::info;

use crate::bus::Bus;
use crate::bus::ppb::Ppb;
use crate::threaded::CoreAtomics;
pub use bus_trait::CoreBus;
pub use registers::Registers;

// Per-core interpolator register file (INTERP0 or INTERP1). Phase 3
// Stage 3 (LLD V7 §6) moved register storage off `Sio` onto each core's
// `PerCoreSio`. Step 5 (Coverage Gap Fill, HLD V5 §5 Part B) replaced
// the passive `[u32; 16]` round-trip with a live arithmetic unit —
// shift + mask + signed + cross-input/result + add-raw + force-MSB +
// BLEND (INTERP0) + CLAMP (INTERP1) + sticky OVERF with W1C. The
// implementation lives in `sio::interp`; we re-export it here so
// `PerCoreSio::interp` uses the exact same type as any call site going
// through `sio::interp::Interp`.
pub use crate::sio::Interp;

/// Per-core SIO state that cannot be shared across cores — DIV and INTERP
/// register files. Lives on `CortexM33` so each core has its own copy;
/// the shared `Sio` owns only truly cross-core state (FIFO, spinlocks,
/// doorbells, MTIME, GPIO). Phase 3 Stage 3 (LLD V7 §6).
#[derive(Default)]
pub struct PerCoreSio {
    pub divider: Divider,
    pub interp: [Interp; 2],
}

impl PerCoreSio {
    /// Read a DIV/INTERP SIO register (offsets 0x060..=0x0FC).
    /// Caller must pre-mask the SIO offset to 12 bits.
    ///
    /// Matches the pre-Stage-3 semantics of `Sio::read32` for the
    /// DIV/INTERP arms. Non-DIV/INTERP offsets return 0 — the caller
    /// should have routed them to `Sio` / `Bus` before getting here.
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            // Integer divider (0x060–0x078)
            0x060 | 0x068 => self.divider.dividend,
            0x064 | 0x06C => self.divider.divisor,
            0x070 | 0x074 => self.divider_result_read(offset),
            0x078 => {
                let ready = 1u32;
                let dirty = if self.divider.dirty { 2 } else { 0 };
                ready | dirty
            }
            // Interpolators (0x080–0x0FC) — live arithmetic per §2.7.
            0x080..=0x0BC => {
                let off = offset - 0x080;
                self.interp[0].read(off, false)
            }
            0x0C0..=0x0FC => {
                let off = offset - 0x0C0;
                self.interp[1].read(off, true)
            }
            _ => 0,
        }
    }

    /// Write a DIV/INTERP SIO register.
    pub fn write32(&mut self, offset: u32, val: u32) {
        match offset {
            0x060..=0x078 => self.divider_write(offset, val),
            0x080..=0x0BC => {
                let off = offset - 0x080;
                // SIO lives in bus region 0xD with no APB alias encoding;
                // always pass alias = 0.
                self.interp[0].write(off, val, 0);
            }
            0x0C0..=0x0FC => {
                let off = offset - 0x0C0;
                self.interp[1].write(off, val, 0);
            }
            _ => {}
        }
    }

    /// True if `offset` (pre-masked to 12 bits) addresses DIV or INTERP.
    #[inline]
    pub fn owns_offset(offset: u32) -> bool {
        (0x060..=0x0FC).contains(&offset)
    }

    /// Read quotient or remainder, advancing the `reads_pending` counter.
    /// Clears DIRTY after both quotient and remainder have been read.
    fn divider_result_read(&mut self, offset: u32) -> u32 {
        let d = &mut self.divider;
        let val = match offset {
            0x070 => d.quotient,
            0x074 => d.remainder,
            _ => return 0,
        };
        if d.dirty {
            d.reads_pending += 1;
            if d.reads_pending >= 2 {
                d.dirty = false;
                d.reads_pending = 0;
            }
        }
        val
    }

    fn divider_write(&mut self, offset: u32, val: u32) {
        let d = &mut self.divider;
        match offset {
            0x060 => {
                // DIV_UDIVIDEND
                d.dividend = val;
                d.signed = false;
            }
            0x064 => {
                // DIV_UDIVISOR — triggers unsigned computation
                d.divisor = val;
                d.signed = false;
                Self::compute_division(d);
            }
            0x068 => {
                // DIV_SDIVIDEND
                d.dividend = val;
                d.signed = true;
            }
            0x06C => {
                // DIV_SDIVISOR — triggers signed computation
                d.divisor = val;
                d.signed = true;
                Self::compute_division(d);
            }
            0x070 => {
                // DIV_QUOTIENT (direct set)
                d.quotient = val;
                d.dirty = true;
                d.reads_pending = 0;
            }
            0x074 => {
                // DIV_REMAINDER (direct set)
                d.remainder = val;
                d.dirty = true;
                d.reads_pending = 0;
            }
            _ => {}
        }
    }

    fn compute_division(d: &mut Divider) {
        if d.divisor == 0 {
            // Division by zero (RP2350 behavior)
            if d.signed {
                let dividend_signed = d.dividend as i32;
                d.quotient = if dividend_signed < 0 {
                    1u32
                } else {
                    (-1i32) as u32
                };
            } else {
                d.quotient = 0xFFFF_FFFF;
            }
            d.remainder = d.dividend;
        } else if d.signed {
            let a = d.dividend as i32;
            let b = d.divisor as i32;
            d.quotient = a.wrapping_div(b) as u32;
            d.remainder = a.wrapping_rem(b) as u32;
        } else {
            d.quotient = d.dividend.wrapping_div(d.divisor);
            d.remainder = d.dividend.wrapping_rem(d.divisor);
        }
        d.dirty = true;
        d.reads_pending = 0;
    }
}

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

/// Per-core access counters for workload characterization (Phase 0a).
#[derive(Debug, Default, Clone)]
pub struct CoreCounters {
    pub decode_execute_cycles: u64,
    pub wfi_cycles: u64,
    pub wfe_cycles: u64,
    pub sram_reads: u64,
    pub sram_writes: u64,
    pub sio_accesses: u64,
    pub peripheral_accesses: u64,
    pub ppb_accesses: u64,
}

impl CoreCounters {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    fn classify_access(&mut self, addr: u32, is_write: bool) {
        let region = addr >> 28;
        match region {
            0x2 => {
                if is_write {
                    self.sram_writes += 1
                } else {
                    self.sram_reads += 1
                }
            }
            0xD => self.sio_accesses += 1,
            0xE => self.ppb_accesses += 1,
            _ => self.peripheral_accesses += 1,
        }
    }
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
    pub(crate) it_state: u8,
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
    /// Cross-core atomic state (halted / wfe_waiting / event_flag /
    /// irq_pending / RCP / bus_fault). Shared with `Bus.atomics` and,
    /// in threaded mode, `SharedState.atomics`. Phase 3 Stage 1
    /// (LLD V7 §2) — `halted`/`wfe_waiting` used to live here as
    /// `AtomicBool` fields; they are now accessed through
    /// `self.atomics.is_halted(core)` etc.
    pub(crate) atomics: Arc<CoreAtomics>,
    /// Per-core Private Peripheral Bus (NVIC, SCB, SysTick, FPCCR, MPU,
    /// SAU, DWT — all per-core M33 architectural state). Moved from
    /// `Bus.ppb: [Ppb; 2]` in Phase 0b.1 Commit B. See
    /// `wrk_docs/2026.04.16 - LLD - Threaded Dual-Core Phase 0 V4.md`.
    ///
    /// Public so integration tests and harness binaries (phase-7 lazy FP
    /// suite, softfloat_diff, isr_scenarios, probe/silicon oracles) can
    /// poke FPCCR/FPCAR/VTOR/CPACR/NVIC state without hand-rolling MMIO
    /// writes. The crate's public surface accepts this — the pre-Commit-B
    /// equivalent `Bus::ppb` was also public.
    pub ppb: Ppb,
    /// ARMv8-M local exclusive monitor: address tracked by the last LDREX
    /// on this core, or `None` when the monitor is open (no outstanding
    /// LDREX, or cleared by CLREX / peer write / successful STREX).
    /// Address-based per ARMv8-M §A3.4 — no value tracking. Invalidated
    /// by `Emulator::step` when the peer core performed any data-side
    /// write during its quantum (see `did_write_this_quantum`).
    /// Phase 0b.2 of the Threaded Dual-Core Emulation plan.
    pub(crate) exclusive_address: Option<u32>,
    /// Set by `bus_write{8,16,32}` on any data-side write by this core
    /// during the current quantum. Read and cleared by `Emulator::step`
    /// after the core's quantum slice completes — if the peer core has
    /// an outstanding LDREX, its monitor is invalidated. Same-core
    /// writes do NOT invalidate the local monitor (per ARM semantics).
    /// Phase 0b.2 of the Threaded Dual-Core Emulation plan.
    pub(crate) did_write_this_quantum: bool,
    /// Per-core SIO state — DIV and INTERP register files. Phase 3
    /// Stage 3 (LLD V7 §6): moved off `Sio` because each core sees
    /// its own divider and interpolator state. DIV/INTERP MMIO
    /// addresses (SIO 0x060..=0x0FC) are intercepted in the
    /// [`Self::bus_read32`] / [`Self::bus_write32`] wrappers and
    /// routed here rather than reaching `Bus::read32`/`write32`.
    pub sio_local: PerCoreSio,
    /// PC-keyed decoded-op cache. Direct-mapped,
    /// [`crate::bus::DECODE_CACHE_SIZE`] entries × 12 B = 192 KB per
    /// core. Populated lazily on fetch by
    /// [`Self::populate_decode_cache`]; invalidated by the driver after
    /// each `step()` by draining
    /// [`crate::bus::Bus::pending_cache_invalidations`] (or the
    /// threaded [`crate::threaded::bus::WorkerBus::pending_cache_invalidations`])
    /// into [`Self::invalidate_decode_cache_entries`]. Bulk-invalidated
    /// on `load_bootrom` / `load_flash` / firmware ISB via
    /// [`Self::invalidate_decode_cache_regions`] (region-scoped) or
    /// [`Self::invalidate_decode_cache_all`] (everything). Phase 3
    /// follow-up #10 — moved off `Bus` because each Cortex-M33 has its
    /// own pipeline and decoder.
    ///
    /// Per-core decode cache. ~192 KB × 2 cores = ~384 KB total in a
    /// dual-core `Emulator` (was 192 KB shared on `Bus` pre-migration).
    /// Architecturally correct — each M33 has its own pipeline +
    /// decoder — and enables the threaded runtime where cross-core
    /// cache coherence would require inter-thread synchronization
    /// otherwise.
    pub(crate) decode_cache: Box<[crate::bus::DecodedOp; crate::bus::DECODE_CACHE_SIZE]>,
    /// Per-core workload counters (Phase 0a instrumentation).
    pub counters: CoreCounters,
    /// Bootrom `reboot` hook PC in **Secure** state (HLD V5 §"Component
    /// 3 — Bootrom mask-ROM"). Populated by [`crate::Emulator::load_bootrom`]
    /// from [`crate::bootrom_hooks::resolve_bootrom_hooks`]; `None` when no
    /// bootrom has been loaded or the binary lacks the `RB` table entry.
    ///
    /// When the per-step PC equals this value the hook fires
    /// terminate-only — see [`Self::bootrom_hook_fired`].
    pub bootrom_reboot_hook_pc_s: Option<u32>,
    /// Bootrom `reboot` hook PC at the **Non-Secure** alias (`+0x8000`)
    /// per RP2350 IDAU. Same population path as `_pc_s`. NS firmware
    /// fetching the bootrom via the NS alias resolves to this PC and
    /// must also fire the hook.
    pub bootrom_reboot_hook_pc_ns: Option<u32>,
    /// Latched flag set by the per-step bootrom-hook check when
    /// `pc == bootrom_reboot_hook_pc_{s,ns}`. The outer scheduler
    /// (`Emulator::step_serial`, threaded core-worker) snapshots this
    /// flag each quantum and propagates it to
    /// [`crate::Emulator::shutdown_requested`]; once set, the core is
    /// halted via `atomics.set_halted(core)` so it never re-enters
    /// `decode_execute`. Terminate-only contract — never cleared by the
    /// core itself.
    pub bootrom_hook_fired: bool,
}

impl CortexM33 {
    /// Construct a core with the given core id and shared atomics.
    /// Phase 3 Stage 1: atomics are required at construction — no
    /// setter, no `Option`. Unit tests use [`Self::for_test`] to
    /// construct a solo core with its own atomics.
    pub fn new(core_id: u8, atomics: Arc<CoreAtomics>) -> Self {
        use crate::bus::{DECODE_CACHE_SIZE, DecodedOp};
        // 192 KB heap allocation per core — can't live on the stack.
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
            it_state: 0,
            pending_fault: None,
            dcp_halves: [0; 16],
            dcp_status: 0,
            secure: true,
            atomics,
            ppb: Ppb::default(),
            exclusive_address: None,
            did_write_this_quantum: false,
            sio_local: PerCoreSio::default(),
            decode_cache,
            counters: CoreCounters::default(),
            bootrom_reboot_hook_pc_s: None,
            bootrom_reboot_hook_pc_ns: None,
            bootrom_hook_fired: false,
        }
    }

    /// Construct a solo core for unit tests. Allocates its own
    /// `Arc<CoreAtomics>` — callers that also want a `Bus` in this
    /// core's atomics namespace should use `Bus::with_atomics` with
    /// `Arc::clone(&core.atomics)`.
    #[cfg(test)]
    pub fn for_test(core_id: u8) -> Self {
        Self::new(core_id, Arc::new(CoreAtomics::default()))
    }

    /// Execute one instruction atomically, advancing the core's own cycle
    /// count by the instruction's cycle cost (including any exception-entry
    /// cost if a synchronous fault is taken).
    ///
    /// Generic over the [`CoreBus`] surface (Phase 3 Stage 2, LLD V7 §1).
    /// In Stage 2 the only `impl CoreBus` is for `Bus`; Stage 5 adds
    /// `WorkerBus`. The Arc-sharing debug trip-wire is enforced here via
    /// the trait accessor `bus.atomics()` — all direct callers (tests,
    /// harness, `Emulator::step`) funnel through this method, so the
    /// invariant is caught regardless of whether the caller went through
    /// the single-threaded driver or constructed cores + bus by hand.
    pub fn step<B: CoreBus>(&mut self, bus: &mut B) {
        debug_assert!(
            Arc::ptr_eq(&self.atomics, bus.atomics()),
            "CortexM33 and its Bus hold disjoint Arc<CoreAtomics> — \
             signals won't route. Construct the Bus via Bus::with_atomics(\
             Arc::clone(&core.atomics)) or share the Arc explicitly."
        );
        let core = self.core_id as usize;
        if self.atomics.is_wfe_waiting(core) {
            self.counters.wfe_cycles += 1;
            return;
        }
        if self.atomics.is_halted(core) {
            self.counters.wfi_cycles += 1;
            return;
        }

        // Phase 3 Stage 1 (LLD V7 §2): peripheral-asserted IRQs live in
        // `CoreAtomics::irq_pending`. `take_irq_pending` swaps the mask
        // to zero — a non-zero return is the consume-and-merge trigger
        // that replaces the pre-stage-1 `irq_pending_dirty` flag.
        let pending = self.atomics.take_irq_pending(core);
        if pending != 0 {
            self.ppb.merge_irq_pending(pending);
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

        // Bootrom mask-ROM hook (HLD V5 §"Component 3"). Terminate-only
        // — when PC matches the resolved RB entry-point (Secure or
        // Non-Secure alias) we latch `bootrom_hook_fired`, halt the
        // core via `atomics.set_halted`, and return without dispatching
        // the instruction. The outer scheduler observes the halt at
        // the quantum boundary and propagates `shutdown_requested` on
        // the `Emulator`.
        let pc = self.regs.pc();
        if Some(pc) == self.bootrom_reboot_hook_pc_s || Some(pc) == self.bootrom_reboot_hook_pc_ns {
            self.bootrom_hook_fired = true;
            self.atomics.set_halted(core);
            return;
        }

        let mut cycles = self.decode_execute(bus);

        // Synchronous bus fault
        let mut fault_handled = false;
        if bus.bus_fault(self.core_id) {
            fault_handled = true;
            let busfault_ena = self.ppb.shcsr & (1 << 17) != 0;
            self.ppb.cfsr |= (1 << 9) | (1 << 15); // PRECISERR + BFARVALID
            self.ppb.bfar = bus.bus_fault_addr(self.core_id);
            bus.clear_bus_fault(self.core_id);
            if busfault_ena {
                cycles = self.enter_exception(5, bus);
            } else {
                info!(
                    pc = format_args!("{:#010x}", self.current_instr_addr),
                    "HardFault escalation from BusFault",
                );
                self.ppb.hfsr |= 1 << 30;
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

        self.counters.decode_execute_cycles += cycles as u64;
        self.cycles = self.cycles.wrapping_add(cycles as u64);
    }

    /// Debug step: clears halted/wfe_waiting before stepping.
    /// Used by QEMU diff harness so WFI doesn't stall the oracle.
    pub fn debug_step<B: CoreBus>(&mut self, bus: &mut B) {
        let core = self.core_id as usize;
        self.atomics.clear_halted(core);
        self.atomics.clear_wfe_waiting(core);
        self.step(bus);
    }

    /// Threaded-worker fast path: like [`Self::step`] but skips the
    /// per-step atomic loads (`is_wfe_waiting`, `is_halted`,
    /// `take_irq_pending`) that the caller has already handled at
    /// quantum boundaries. In the threaded runtime those atomics land
    /// on cache lines shared with the coordinator worker, and when the
    /// coordinator writes to `irq_pending` between quanta the core's
    /// per-step load turns into a ~30 ns cache-line bounce — tripling
    /// the cost of a single-cycle ALU instruction.
    ///
    /// Contract (upheld by `threaded::core_worker_body`):
    ///   - `!is_halted(core)` at the call site
    ///   - `!is_wfe_waiting(core)` at the call site (post-step WFE sets
    ///     are observed by the worker before the next call)
    ///   - Any pending IRQs were taken and merged into `ppb` at the
    ///     top of the quantum
    ///
    /// The arc-sharing trip-wire and the exception-entry + fault paths
    /// still run inside; only the cross-thread atomics on `CoreAtomics`
    /// are elided.
    pub fn step_no_atomics<B: CoreBus>(&mut self, bus: &mut B) {
        debug_assert!(
            Arc::ptr_eq(&self.atomics, bus.atomics()),
            "CortexM33 and its Bus hold disjoint Arc<CoreAtomics>"
        );
        debug_assert!(!self.atomics.is_halted(self.core_id as usize));
        debug_assert!(!self.atomics.is_wfe_waiting(self.core_id as usize));

        if let Some(cost) = self.try_take_any_pending_exception(bus) {
            self.cycles = self.cycles.wrapping_add(cost as u64);
            return;
        }

        // Bootrom mask-ROM hook — same shape as `Self::step`, see HLD
        // V5 §"Component 3 — Bootrom mask-ROM" / "Hook check placement".
        let pc = self.regs.pc();
        if Some(pc) == self.bootrom_reboot_hook_pc_s || Some(pc) == self.bootrom_reboot_hook_pc_ns {
            self.bootrom_hook_fired = true;
            self.atomics.set_halted(self.core_id as usize);
            return;
        }

        let mut cycles = self.decode_execute(bus);

        let mut fault_handled = false;
        if bus.bus_fault(self.core_id) {
            fault_handled = true;
            let busfault_ena = self.ppb.shcsr & (1 << 17) != 0;
            self.ppb.cfsr |= (1 << 9) | (1 << 15);
            self.ppb.bfar = bus.bus_fault_addr(self.core_id);
            bus.clear_bus_fault(self.core_id);
            if busfault_ena {
                cycles = self.enter_exception(5, bus);
            } else {
                info!(
                    pc = format_args!("{:#010x}", self.current_instr_addr),
                    "HardFault escalation from BusFault",
                );
                self.ppb.hfsr |= 1 << 30;
                cycles = self.enter_exception(3, bus);
            }
        }

        if !fault_handled {
            if let Some(fault) = self.pending_fault.take() {
                cycles = self.deliver_fault(fault, bus);
            }
        } else {
            self.pending_fault = None;
        }

        self.counters.decode_execute_cycles += cycles as u64;
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

    /// Invalidate this core's decode-cache entries for the supplied
    /// addresses.
    ///
    /// Phase 3 follow-up #10: the single-threaded and threaded drivers
    /// drain `pending_cache_invalidations` from `Bus` / `WorkerBus`
    /// after each `core.step()` and call this to evict stale entries
    /// whose backing halfwords were just rewritten. Mirrors
    /// [`Self::invalidate_decode_cache_all`] for bulk-load / ISB paths.
    ///
    /// Clears the direct-mapped slot `((addr >> 1) & (DECODE_CACHE_SIZE - 1))`
    /// for each cacheable address, plus the preceding slot (so a wide
    /// instruction's `hw0` at `addr - 2` whose `hw1` is rewritten gets
    /// evicted too). Non-cacheable addresses (anything outside ROM / XIP
    /// / SRAM per `is_cacheable_pc`) are skipped.
    pub fn invalidate_decode_cache_entries(&mut self, addrs: &[u32]) {
        use crate::bus::{DECODE_CACHE_SIZE, DecodedOp, is_cacheable_pc};
        const MASK: u32 = (DECODE_CACHE_SIZE as u32) - 1;
        let empty = DecodedOp::empty();
        for &addr in addrs {
            // Invalidate both the slot covering this halfword and the
            // preceding slot (so a wide-instruction hw0 at `addr - 2`
            // whose hw1 is rewritten at `addr` gets evicted).
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
    /// selected by the `regions` bitmask (see
    /// [`crate::bus::invalidation_regions`]). Unaffected slots stay hot
    /// — a `load_flash` no longer evicts SRAM-resident code, so
    /// firmware that reloads flash then runs SRAM code doesn't pay a
    /// cold-populate tax on every instruction of the next quantum.
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
        // `(tag >> 28) as u8` (ROM = 0, XIP = 1, SRAM = 2). We keep bit
        // `n` of the `regions` byte in sync with region `n`, so a test
        // of `regions & (1 << region_nibble)` picks out exactly the
        // slots this caller asked to drain. Empty slots (`tag ==
        // u32::MAX`, nibble = 0xF) never match any valid region bit, so
        // they're skipped without special-casing.
        for slot in self.decode_cache.iter_mut() {
            let nibble = (slot.tag >> 28) as u8;
            if nibble < 8 && regions & (1 << nibble) != 0 {
                *slot = empty;
            }
        }
    }

    /// Invalidate every decode-cache entry on this core. Used by `ISB`
    /// and any other path that globally invalidates the instruction
    /// pipeline. Equivalent to
    /// [`Self::invalidate_decode_cache_regions`] with
    /// [`crate::bus::invalidation_regions::BULK`]; retained as a
    /// convenience (one-liner at call sites that unconditionally wipe
    /// everything, such as the `ISB` handler).
    pub fn invalidate_decode_cache_all(&mut self) {
        use crate::bus::DecodedOp;
        let empty = DecodedOp::empty();
        for slot in self.decode_cache.iter_mut() {
            *slot = empty;
        }
    }

    /// Direct decode-cache lookup (by slot). Used by
    /// `CortexM33::decode_execute` / `populate_decode_cache`. The
    /// `DecodedOp` type is `pub(crate)`; external callers never need
    /// this accessor.
    #[inline(always)]
    pub(crate) fn decode_cache_get(&self, slot: usize) -> crate::bus::DecodedOp {
        self.decode_cache[slot]
    }

    /// Direct decode-cache store (by slot). Used by
    /// `CortexM33::populate_decode_cache`.
    #[inline(always)]
    pub(crate) fn decode_cache_set(&mut self, slot: usize, entry: crate::bus::DecodedOp) {
        self.decode_cache[slot] = entry;
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
        self.atomics.set_halted(self.core_id as usize);
        self.pending_fault = None;
    }

    /// Resume a halted core. The caller must set PC, SP, and xpsr before
    /// calling this — wake() only clears the halted flag.
    pub fn wake(&mut self) {
        self.atomics.clear_halted(self.core_id as usize);
    }

    /// Returns `true` if the core is halted.
    pub fn is_halted(&self) -> bool {
        self.atomics.is_halted(self.core_id as usize)
    }

    /// Returns `true` if the core is sleeping on WFE.
    pub fn is_wfe_waiting(&self) -> bool {
        self.atomics.is_wfe_waiting(self.core_id as usize)
    }

    /// Execute WFE hint. If event_flag is pending, consume it and continue.
    /// Otherwise, enter WFE sleep. Phase 3 Stage 1: the event_flag state
    /// lives on `CoreAtomics`; we consume with an AcqRel swap-to-false
    /// that pairs with `sev_both`'s Release stores.
    pub(crate) fn wfe<B: CoreBus>(&mut self, _bus: &mut B) -> u32 {
        let core = self.core_id as usize;
        if self.atomics.event_flag_consume(core) {
            1 // event was pending, consume it, no sleep
        } else {
            self.atomics.set_wfe_waiting(core);
            1
        }
    }

    // -------------------------------------------------------------------
    // PPB-intercept bus wrappers (Phase 0b.1 Commit B).
    //
    // Data-side bus accesses route through these: PPB addresses
    // (`0xE000_0000..=0xEFFF_FFFF`) resolve against `self.ppb` directly;
    // SIO DIV/INTERP addresses (`0xD000_0060..=0xD000_00FC`) resolve
    // against `self.sio_local` — see [`PerCoreSio`] (Phase 3 Stage 3,
    // LLD V7 §6). Everything else (including the boot-RAM carve-out at
    // `0xEFFF_F000..0xF000_0000`) falls through to `Bus::readN/writeN`.
    //
    // Instruction-fetch path in `decode.rs` bypasses these — opcodes are
    // never fetched from PPB or SIO, so the extra branches are pure
    // overhead there.
    // -------------------------------------------------------------------

    /// True when `addr` targets a DIV or INTERP register (SIO
    /// 0x060..=0x0FC, any 12-KB alias). Helper for the `bus_{read,write}*`
    /// intercepts.
    #[inline]
    fn is_sio_local(addr: u32) -> bool {
        addr >> 28 == 0xD && PerCoreSio::owns_offset(addr & 0xFFF)
    }

    pub(crate) fn bus_read32<B: CoreBus>(&mut self, addr: u32, bus: &mut B) -> u32 {
        self.counters.classify_access(addr, false);
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            let val = self.ppb.read32(addr);
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('R', 4, addr, val, self.core_id);
            }
            val
        } else if Self::is_sio_local(addr) {
            let val = self.sio_local.read32(addr & 0xFFF);
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('R', 4, addr, val, self.core_id);
            }
            val
        } else {
            bus.read32(addr, self.core_id)
        }
    }

    pub(crate) fn bus_write32<B: CoreBus>(&mut self, addr: u32, val: u32, bus: &mut B) {
        self.counters.classify_access(addr, true);
        // Phase 0b.2: any data-side write invalidates a peer core's
        // exclusive monitor. `Emulator::step` snoops this flag after the
        // core's quantum slice and clears the peer's `exclusive_address`.
        self.did_write_this_quantum = true;
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            self.ppb.write32(addr, val);
            self.sync_nvic_to_irq_pending(addr, bus);
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('W', 4, addr, val, self.core_id);
            }
        } else if Self::is_sio_local(addr) {
            self.sio_local.write32(addr & 0xFFF, val);
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('W', 4, addr, val, self.core_id);
            }
        } else {
            bus.write32(addr, val, self.core_id);
        }
    }

    pub(crate) fn bus_read16<B: CoreBus>(&mut self, addr: u32, bus: &mut B) -> u16 {
        self.counters.classify_access(addr, false);
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            // ARMv8-M: halfword PPB accesses are UNPREDICTABLE (word-only
            // registers). We defensively compose the result from the
            // containing 32-bit register rather than faulting, so rogue
            // firmware sees plausible data. Contrast bus_read8, which
            // returns 0 — byte access is more unusual and worth flagging
            // via a telltale zero.
            let word = self.ppb.read32(addr & !3);
            let val = if addr & 2 != 0 {
                (word >> 16) as u16
            } else {
                word as u16
            };
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('R', 2, addr, val as u32, self.core_id);
            }
            val
        } else if Self::is_sio_local(addr) {
            // Matches the pre-Stage-3 `Bus::read16` 0xD path: read the
            // containing 32-bit SIO register and slice the halfword.
            let word = self.sio_local.read32(addr & 0xFFF & !3);
            let val = if addr & 2 != 0 {
                (word >> 16) as u16
            } else {
                word as u16
            };
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('R', 2, addr, val as u32, self.core_id);
            }
            val
        } else {
            bus.read16(addr, self.core_id)
        }
    }

    pub(crate) fn bus_write16<B: CoreBus>(&mut self, addr: u32, val: u16, bus: &mut B) {
        self.counters.classify_access(addr, true);
        // Phase 0b.2: see `bus_write32` for the monitor-invalidation rationale.
        self.did_write_this_quantum = true;
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            // ARMv8-M: halfword PPB accesses are UNPREDICTABLE. We
            // defensively RMW the matching half of the containing 32-bit
            // register rather than faulting. Contrast bus_write8, which
            // drops the write — byte writes to PPB are more unusual.
            let old = self.ppb.read32(addr & !3);
            let new_val = if addr & 2 != 0 {
                (old & 0x0000_FFFF) | ((val as u32) << 16)
            } else {
                (old & 0xFFFF_0000) | val as u32
            };
            self.ppb.write32(addr & !3, new_val);
            self.sync_nvic_to_irq_pending(addr & !3, bus);
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('W', 2, addr, val as u32, self.core_id);
            }
        } else if Self::is_sio_local(addr) {
            // Pre-Stage-3 `Bus::write16` dropped SIO writes silently
            // (region 0xD had no write16 arm). Preserve that here: drop
            // the write, but still emit the trace line so observability
            // is unchanged.
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('W', 2, addr, val as u32, self.core_id);
            }
        } else {
            bus.write16(addr, val, self.core_id);
        }
    }

    pub(crate) fn bus_read8<B: CoreBus>(&mut self, addr: u32, bus: &mut B) -> u8 {
        self.counters.classify_access(addr, false);
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            // PPB registers are word-access-only; byte reads return 0.
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('R', 1, addr, 0, self.core_id);
            }
            0
        } else if Self::is_sio_local(addr) {
            // Matches the pre-Stage-3 `Bus::read8` 0xD path: read the
            // containing 32-bit SIO register and slice the byte.
            let word = self.sio_local.read32(addr & 0xFFF & !3);
            let byte_idx = (addr & 3) as usize;
            let val = word.to_le_bytes()[byte_idx];
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('R', 1, addr, val as u32, self.core_id);
            }
            val
        } else {
            bus.read8(addr, self.core_id)
        }
    }

    pub(crate) fn bus_write8<B: CoreBus>(&mut self, addr: u32, val: u8, bus: &mut B) {
        self.counters.classify_access(addr, true);
        // Phase 0b.2: see `bus_write32` for the monitor-invalidation rationale.
        self.did_write_this_quantum = true;
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            // PPB registers are word-access-only; byte writes drop.
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('W', 1, addr, val as u32, self.core_id);
            }
        } else if Self::is_sio_local(addr) {
            // Pre-Stage-3 `Bus::write8` dropped SIO writes silently
            // (region 0xD had no write8 arm). Preserve that; see the
            // matching note in `bus_write16`.
            if bus.mmio_trace_enabled() {
                bus.emit_mmio_trace('W', 1, addr, val as u32, self.core_id);
            }
        } else {
            bus.write8(addr, val, self.core_id);
        }
    }

    /// After a PPB write that may have touched NVIC_ISPR / NVIC_ICPR,
    /// reconstruct `bus.irq_pending[core]` from the post-write ISPR.
    /// Phase 0b.1 Commit B: replaces the mirror the old Bus-side PPB
    /// dispatch arm did inline (see `Bus::write32` 0xE branch before the
    /// PPB move).
    ///
    /// Firmware self-pends via ISPR and software-clears via ICPR — either
    /// way the architectural latch lives in `nvic_ispr`. `irq_pending`
    /// gates the step-path NVIC walk for cheap short-circuiting, so it
    /// must stay in sync with `nvic_ispr` after each write.
    fn sync_nvic_to_irq_pending<B: CoreBus>(&self, addr: u32, _bus: &mut B) {
        let low = addr & 0xFFFF;
        if matches!(low, 0xE200 | 0xE204 | 0xE280 | 0xE284) {
            let word = if low == 0xE200 || low == 0xE280 { 0 } else { 1 };
            let ispr = self.ppb.nvic_ispr[word].load(std::sync::atomic::Ordering::Relaxed);
            let mask64 = (ispr as u64) << (word * 32);
            let keep = if word == 0 {
                !0xFFFF_FFFFu64
            } else {
                0xFFFF_FFFFu64
            };
            let core = self.core_id as usize;
            // Phase 3 Stage 1: `irq_pending` migrated onto `CoreAtomics`.
            // Preserve the word that isn't being replaced; overwrite the
            // target word with the post-write ISPR bits.
            let prev = self.atomics.irq_pending_load(core);
            let new_val = (prev & keep) | mask64;
            // Swap to get a precise replacement rather than the ambiguous
            // union `fetch_or`. A single-threaded race-free model suffices
            // here (the core that just wrote NVIC_ISPR is the only writer
            // to its own slot in the single-threaded path).
            self.atomics.set_irq_pending(core, new_val);
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

    /// Enable a coprocessor in CPACR (full access = 0b11 for the slot).
    /// Convenience for unit tests and harnesses that need to flip
    /// coprocessor gates without threading MMIO writes through the bus.
    ///
    /// `coproc` is 0..=15; the bit positions are `[2*coproc+1:2*coproc]`.
    #[doc(hidden)]
    pub fn enable_coprocessor(&mut self, coproc: u8) {
        self.ppb.cpacr |= 0x3 << (coproc as u32 * 2);
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
        Self::new(0, Arc::new(CoreAtomics::default()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Integer divider tests (Phase 3 Stage 3 — migrated from
    // `sio::tests` when DIV moved to `PerCoreSio`). These exercise
    // the storage+compute semantics directly on `PerCoreSio`, matching
    // the pre-Stage-3 assertions on `Sio::read32/write32`. ----

    #[test]
    fn divider_unsigned_basic() {
        let mut s = PerCoreSio::default();
        // 100 / 7 = 14 remainder 2
        s.write32(0x060, 100); // DIV_UDIVIDEND
        s.write32(0x064, 7); // DIV_UDIVISOR
        assert_eq!(s.read32(0x070), 14);
        assert_eq!(s.read32(0x074), 2);
    }

    #[test]
    fn divider_signed_basic() {
        let mut s = PerCoreSio::default();
        // -100 / 7 = -14 remainder -2
        s.write32(0x068, (-100i32) as u32); // DIV_SDIVIDEND
        s.write32(0x06C, 7); // DIV_SDIVISOR
        assert_eq!(s.read32(0x070) as i32, -14);
        assert_eq!(s.read32(0x074) as i32, -2);
    }

    #[test]
    fn divider_signed_negative_divisor() {
        let mut s = PerCoreSio::default();
        // 100 / -7 = -14 remainder 2
        s.write32(0x068, 100);
        s.write32(0x06C, (-7i32) as u32);
        assert_eq!(s.read32(0x070) as i32, -14);
        assert_eq!(s.read32(0x074) as i32, 2);
    }

    #[test]
    fn divider_unsigned_div_by_zero() {
        let mut s = PerCoreSio::default();
        s.write32(0x060, 42); // dividend = 42
        s.write32(0x064, 0); // divisor = 0
        assert_eq!(s.read32(0x070), 0xFFFF_FFFF);
        assert_eq!(s.read32(0x074), 42);
    }

    #[test]
    fn divider_signed_div_by_zero_positive() {
        let mut s = PerCoreSio::default();
        s.write32(0x068, 42); // dividend = 42 (positive)
        s.write32(0x06C, 0); // divisor = 0
        // positive dividend / 0 → quotient = -1
        assert_eq!(s.read32(0x070) as i32, -1);
        assert_eq!(s.read32(0x074), 42);
    }

    #[test]
    fn divider_signed_div_by_zero_negative() {
        let mut s = PerCoreSio::default();
        s.write32(0x068, (-42i32) as u32); // dividend = -42
        s.write32(0x06C, 0); // divisor = 0
        // negative dividend / 0 → quotient = 1
        assert_eq!(s.read32(0x070), 1);
        assert_eq!(s.read32(0x074), (-42i32) as u32);
    }

    #[test]
    fn divider_dirty_flag_clear_after_both_reads() {
        let mut s = PerCoreSio::default();
        s.write32(0x060, 100);
        s.write32(0x064, 7);
        // CSR should show DIRTY (bit 1) and READY (bit 0)
        assert_eq!(s.read32(0x078) & 0x3, 0x3);
        // Read quotient — still dirty
        s.read32(0x070);
        assert_eq!(s.read32(0x078) & 0x2, 0x2);
        // Read remainder — dirty should clear
        s.read32(0x074);
        assert_eq!(s.read32(0x078) & 0x2, 0x0);
        // READY always 1
        assert_eq!(s.read32(0x078) & 0x1, 0x1);
    }

    #[test]
    fn divider_direct_write_quotient_remainder() {
        let mut s = PerCoreSio::default();
        s.write32(0x070, 0xDEAD);
        s.write32(0x074, 0xBEEF);
        assert_eq!(s.read32(0x070), 0xDEAD);
        assert_eq!(s.read32(0x074), 0xBEEF);
    }

    // ---- Interpolator tests (migrated from `sio::tests`) ----

    // Under live Interp semantics (Step 5), most INTERP register
    // offsets are NOT passive storage — POP/PEEK have side effects,
    // RESULT* are computed, CTRL_LANE1 strips OVERF bits on read,
    // BASE_1AND0 is write-only. Round-trip tests covering every
    // offset lived under passive storage and are obsolete here.
    // Arithmetic coverage lives in `sio::interp::tests`.

    #[test]
    fn interp_accum_round_trip_via_percoresio() {
        let mut s = PerCoreSio::default();
        // ACCUM0 on INTERP0 (offset 0x00 within INTERP0 bank at 0x080).
        s.write32(0x080, 0xCAFE_BABE);
        assert_eq!(s.read32(0x080), 0xCAFE_BABE);
        // ACCUM0 on INTERP1 (bank at 0x0C0).
        s.write32(0x0C0, 0xDEAD_BEEF);
        assert_eq!(s.read32(0x0C0), 0xDEAD_BEEF);
    }

    #[test]
    fn interp0_and_interp1_are_distinct_banks() {
        let mut s = PerCoreSio::default();
        // Same sub-offset 0 in INTERP0 (0x080) and INTERP1 (0x0C0)
        s.write32(0x080, 0x1111_1111);
        s.write32(0x0C0, 0x2222_2222);
        assert_eq!(s.read32(0x080), 0x1111_1111);
        assert_eq!(s.read32(0x0C0), 0x2222_2222);
    }

    // ---- DIV/INTERP per-core isolation on CortexM33 ----

    /// New Stage-3 regression test. Replaces the pre-Stage-3
    /// `Sio::divider_per_core_isolation` and `interp_per_core_isolation`
    /// assertions — now each core carries its own `PerCoreSio`, so the
    /// check moves from "one `Sio`, two cores" to "two cores, two
    /// `PerCoreSio`s, shared `CoreAtomics`".
    ///
    /// Drives the full MMIO intercept path (`bus_write32` / `bus_read32`
    /// at `0xD000_00XX`) rather than poking `sio_local` directly — the
    /// latter would also pass under aliased storage. This proves the
    /// intercept routes DIV/INTERP accesses to the per-core bank the
    /// emulator actually uses in production.
    #[test]
    fn per_core_sio_is_independent() {
        let atomics = Arc::new(CoreAtomics::default());
        let mut core0 = CortexM33::new(0, Arc::clone(&atomics));
        let mut core1 = CortexM33::new(1, Arc::clone(&atomics));
        let mut bus = Bus::with_atomics(Arc::clone(&atomics));

        // Core 0: 100 / 10 = 10 remainder 0 — all via MMIO.
        core0.bus_write32(0xD000_0060, 100, &mut bus); // DIV_UDIVIDEND
        core0.bus_write32(0xD000_0064, 10, &mut bus); // DIV_UDIVISOR (triggers compute)

        // Core 1 has not touched DIV yet — its quotient must still be
        // 0 (POR default). If storage were aliased, core 1 would see
        // core 0's quotient of 10 through the intercept.
        assert_eq!(
            core1.bus_read32(0xD000_0070, &mut bus),
            0,
            "core 1's quotient must be independent of core 0's divide"
        );

        // Core 0's MMIO-read quotient / remainder.
        assert_eq!(core0.bus_read32(0xD000_0070, &mut bus), 10);
        assert_eq!(core0.bus_read32(0xD000_0074, &mut bus), 0);

        // Core 1 does its own divide via MMIO: 99 / 9 = 11.
        core1.bus_write32(0xD000_0060, 99, &mut bus);
        core1.bus_write32(0xD000_0064, 9, &mut bus);
        assert_eq!(core1.bus_read32(0xD000_0070, &mut bus), 11);

        // Core 0's quotient is unchanged by core 1's concurrent divide.
        // (Reads of the DIV result are non-destructive on the value.)
        assert_eq!(core0.bus_read32(0xD000_0070, &mut bus), 10);

        // Same independence check for INTERP storage, again via MMIO.
        core0.bus_write32(0xD000_0080, 0xAAAA_AAAA, &mut bus); // INTERP0_ACCUM0 core 0
        core1.bus_write32(0xD000_0080, 0xBBBB_BBBB, &mut bus); // INTERP0_ACCUM0 core 1
        assert_eq!(core0.bus_read32(0xD000_0080, &mut bus), 0xAAAA_AAAA);
        assert_eq!(core1.bus_read32(0xD000_0080, &mut bus), 0xBBBB_BBBB);
    }

    /// Narrow (8/16-bit) intercept coverage. Proves `bus_read8`,
    /// `bus_read16`, `bus_write8`, `bus_write16` route DIV/INTERP
    /// addresses through the per-core intercept and preserve the
    /// pre-Stage-3 `Bus` semantics for narrow accesses:
    ///   - read16 / read8 synthesize bytes from the containing 32-bit
    ///     register (same as `Bus::read16/read8` 0xD arm used to do).
    ///   - write16 / write8 are silently dropped (matches the pre-Stage-3
    ///     `Bus` — no 0xD arm existed in `write8`, and `write16`'s 0xD
    ///     case fell through to `_ => {}`).
    #[test]
    fn per_core_sio_width_intercept() {
        let atomics = Arc::new(CoreAtomics::default());
        let mut core = CortexM33::new(0, Arc::clone(&atomics));
        let mut bus = Bus::with_atomics(Arc::clone(&atomics));

        // Seed a known word at DIV_QUOTIENT via the wide write (direct
        // port — see `PerCoreSio::divider_write` 0x070 arm). The low
        // halfword is 0xBEEF, the high halfword 0xDEAD.
        core.bus_write32(0xD000_0070, 0xDEAD_BEEF, &mut bus);

        // read16: low half at +0 returns 0xBEEF; high half at +2 returns 0xDEAD.
        assert_eq!(core.bus_read16(0xD000_0070, &mut bus), 0xBEEF);
        assert_eq!(core.bus_read16(0xD000_0072, &mut bus), 0xDEAD);

        // read8: byte 0..3 of the little-endian word.
        assert_eq!(core.bus_read8(0xD000_0070, &mut bus), 0xEF);
        assert_eq!(core.bus_read8(0xD000_0071, &mut bus), 0xBE);
        assert_eq!(core.bus_read8(0xD000_0072, &mut bus), 0xAD);
        assert_eq!(core.bus_read8(0xD000_0073, &mut bus), 0xDE);

        // write16: must be silently dropped — the word is unchanged.
        core.bus_write16(0xD000_0070, 0x1234, &mut bus);
        assert_eq!(
            core.bus_read32(0xD000_0070, &mut bus),
            0xDEAD_BEEF,
            "bus_write16 at a DIV offset must be silently dropped \
             (pre-Stage-3 Bus semantics)"
        );

        // write8: also silently dropped.
        core.bus_write8(0xD000_0070, 0x42, &mut bus);
        core.bus_write8(0xD000_0073, 0x99, &mut bus);
        assert_eq!(
            core.bus_read32(0xD000_0070, &mut bus),
            0xDEAD_BEEF,
            "bus_write8 at a DIV offset must be silently dropped \
             (pre-Stage-3 Bus semantics)"
        );

        // Same round-trip for an INTERP register, to confirm the
        // intercept covers both DIV (0x060..=0x07C) and INTERP
        // (0x080..=0x0FC) sub-ranges.
        core.bus_write32(0xD000_0080, 0xCAFE_F00D, &mut bus);
        assert_eq!(core.bus_read16(0xD000_0080, &mut bus), 0xF00D);
        assert_eq!(core.bus_read16(0xD000_0082, &mut bus), 0xCAFE);
        assert_eq!(core.bus_read8(0xD000_0083, &mut bus), 0xCA);
        core.bus_write16(0xD000_0080, 0xBEEF, &mut bus);
        core.bus_write8(0xD000_0080, 0x77, &mut bus);
        assert_eq!(core.bus_read32(0xD000_0080, &mut bus), 0xCAFE_F00D);
    }
}
