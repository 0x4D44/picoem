//! `CoreBus` trait — the bus-facing surface that `CortexM0Plus::step`
//! and its helpers use to talk to the bus fabric. Dual-execution HLD V1
//! Stage 3b.1.
//!
//! Modelled after `mdrp2350::core::bus_trait::CoreBus`, trimmed to the
//! ARMv6-M + RP2040 fabric shape:
//!
//! - Narrow memory access: `read{8,16,32}` / `write{8,16,32}` take
//!   **no** `core` argument. The RP2040 bus tracks the currently
//!   executing core via `Bus::set_active_core` / `Bus::active_core`
//!   instead of threading the identifier through every access. This
//!   matches the existing `Bus` inherent-method shape exactly, so
//!   monomorphized call-site codegen is unchanged.
//! - Single bus-fault slot: `bus_fault` takes no core argument — M0+
//!   has one synchronous-fault path and RP2040 uses a single sticky
//!   flag across both cores.
//! - `set_active_pc` — one PC per active core, stored in
//!   `Bus::active_pc[active_core]`. Matches the existing inherent
//!   method.
//! - Per-core PPB and NVIC accessors (`ppb_mut`, `nvic_mut`) — both
//!   live on `Bus` as two-element arrays today. Stage 3b.2 will add a
//!   `WorkerBus` impl that returns shared-state references with the
//!   same signature.
//! - GPIO and clock-tree / MMIO-trace accessors stay on the inherent
//!   `Bus` methods for now — Stage 3b.1 scope is only the step hot
//!   path, not the periphery. The trait can be extended in later
//!   sub-stages without breaking this one.
//!
//! Dropped vs RP2350's trait: FPU, secure world, `CoreAtomics`
//! namespace ptr-eq trip-wire, `set_burst_mode`, `add_extra_wait_states`
//! / `take_extra_wait_states`, `last_fetch_addr`, `emit_mmio_trace`.
//! None of these exist on M0+ / the RP2040 bus today. If later stages
//! need them (e.g. a per-core exclusive-monitor model on the threaded
//! path), they can be added then.

use crate::bus::ppb::Ppb;
use crate::core::Nvic;

pub trait CoreBus {
    // --- Memory access ------------------------------------------------
    //
    // Address-only (no `core` argument) — matches the existing RP2040
    // `Bus` inherent shape. Callers are expected to have called
    // `set_active_core(n)` before stepping so the bus knows which core
    // is issuing the access.

    /// 8-bit load.
    fn read8(&mut self, addr: u32) -> u8;
    /// 16-bit load. May raise an alignment fault on the bus.
    fn read16(&mut self, addr: u32) -> u16;
    /// 32-bit load. May raise an alignment fault on the bus.
    fn read32(&mut self, addr: u32) -> u32;

    /// 8-bit store.
    fn write8(&mut self, addr: u32, val: u8);
    /// 16-bit store. May raise an alignment fault on the bus.
    fn write16(&mut self, addr: u32, val: u16);
    /// 32-bit store. May raise an alignment fault on the bus.
    fn write32(&mut self, addr: u32, val: u32);

    // --- Instruction-boundary metadata --------------------------------

    /// Stash the instruction PC of the currently-executing instruction
    /// so the MMIO trace can tag subsequent accesses with it. Stored
    /// per-core; the active core is set via `Bus::set_active_core`.
    fn set_active_pc(&mut self, pc: u32);

    // --- Bus fault ----------------------------------------------------

    /// True if a synchronous bus fault is pending. Single sticky flag
    /// on M0+ — no per-core split.
    fn bus_fault(&self) -> bool;
    /// Address associated with the most recent bus fault.
    fn bus_fault_addr(&self) -> u32;
    /// Clear the sticky bus-fault flag.
    fn clear_bus_fault(&mut self);

    // --- Per-core PPB / NVIC ------------------------------------------
    //
    // Used by exception entry/return and the pre-fetch IRQ-dispatch
    // path. Indexed by core id (0 or 1). `Bus` forwards to
    // `self.ppb[core]` / `self.nvics[core]`; Stage 3b.2's `WorkerBus`
    // will return handle references into the shared-state bundle.

    /// Shared reference to the per-core PPB.
    fn ppb(&self, core: usize) -> &Ppb;
    /// Exclusive reference to the per-core PPB. Required for
    /// `mark_active` / `clear_active` on exception entry/return.
    fn ppb_mut(&mut self, core: usize) -> &mut Ppb;

    /// Shared reference to the per-core NVIC.
    fn nvic(&self, core: usize) -> &Nvic;
    /// Exclusive reference to the per-core NVIC. Required for
    /// `clear_pending` on exception dispatch.
    fn nvic_mut(&mut self, core: usize) -> &mut Nvic;

    // --- Scheduler plumbing -------------------------------------------

    /// Currently-active core (0 or 1). Same semantics as
    /// `Bus::active_core` — set by the dual-core scheduler before
    /// dispatching a step.
    fn active_core(&self) -> usize;
}
