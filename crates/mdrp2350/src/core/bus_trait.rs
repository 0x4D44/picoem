//! `CoreBus` trait — the MMIO + per-instruction-accounting surface that
//! `CortexM33::step` and its helpers use to talk to the bus. Phase 3
//! Stage 2 (LLD V7 §1).
//!
//! ## Intended canonical surface (LLD V7 §1, 13 methods)
//!
//! The final shape is:
//!
//! ```text
//! read{8,16,32}, write{8,16,32}, set_active_pc,
//! bus_fault, bus_fault_addr, clear_bus_fault,
//! set_burst_mode(on), add_extra_wait_states, take_extra_wait_states.
//! ```
//!
//! ## Stage 2 transitional extensions
//!
//! Several pieces of per-instruction state still live on `Bus` rather than
//! on `CortexM33` (trace sink, the `sio` sub-block, direct `gpio_in`, the
//! `atomics` Arc). Later stages of the Phase 3 roadmap move those onto
//! the core (Stage 3 for DIV/INTERP, Stage 5 for SIO/GPIO via
//! `SharedState`), at which point the trait surface shrinks back to the
//! 13-method canonical shape. The decode cache already lives on
//! `CortexM33` (Phase 3 follow-up #10) — writes into executable memory
//! push halfword addresses into `Bus::pending_cache_invalidations` /
//! `WorkerBus::pending_cache_invalidations`; the driver drains them into
//! the per-core cache.
//!
//! Until then, generic `<B: CoreBus>` helpers still need to reach those
//! fields, so this trait carries a handful of transient accessors. They are
//! clearly marked with `// TRANSIENT` comments. The extra cost is runtime
//! dispatch parity with the previous inherent-`Bus` calls, not a new
//! semantic contract — every transient method is a straight forwarder.
//!
//! Deviation from the pure 13-method spec is documented in the Stage 2
//! commit message and the Phase 3 journal.

use std::sync::Arc;

use crate::threaded::CoreAtomics;

pub trait CoreBus {
    // --- Canonical 13-method surface (LLD V7 §1) ----------------------

    fn read8(&mut self, addr: u32, core: u8) -> u8;
    fn read16(&mut self, addr: u32, core: u8) -> u16;
    fn read32(&mut self, addr: u32, core: u8) -> u32;

    fn write8(&mut self, addr: u32, val: u8, core: u8);
    fn write16(&mut self, addr: u32, val: u16, core: u8);
    fn write32(&mut self, addr: u32, val: u32, core: u8);

    fn set_active_pc(&mut self, pc: u32, core: u8);

    fn bus_fault(&self, core: u8) -> bool;
    fn bus_fault_addr(&self, core: u8) -> u32;
    fn clear_bus_fault(&mut self, core: u8);

    fn set_burst_mode(&mut self, on: bool);
    fn add_extra_wait_states(&mut self, n: u32);
    fn take_extra_wait_states(&mut self) -> u32;

    // --- TRANSIENT (Stage 2) ------------------------------------------
    //
    // These will be removed as state migrates off `Bus` in later stages
    // of Phase 3. Every method forwards straight to an existing `Bus`
    // field or inherent method.

    /// Shared atomics — required by the Arc-ptr-eq trip-wire in
    /// `CortexM33::step` to verify the core and bus share a
    /// `CoreAtomics` namespace. Callers that need SEV / RCP / IRQ
    /// pending state should reach them via `self.atomics` on
    /// `CortexM33` directly, not through this accessor.
    fn atomics(&self) -> &Arc<CoreAtomics>;

    // --- GPIO OUT / OE / IN (Phase 3 Stage 6a) -----------------------
    //
    // Typed accessors for bank-0 GPIO state (RP2354A only exposes bank 0
    // through SIO). Both `Bus` and `WorkerBus` implement these:
    //
    // - `Bus` forwards to `self.sio.gpio_out` / `gpio_oe` / `self.gpio_in`.
    // - `WorkerBus` forwards to `self.shared.gpio.*` — so CP0 GPIOC
    //   writes on the threaded path land on the live `AtomicGpio` and
    //   not on a dummy placeholder.
    //
    // These replaced the CP0 GPIOC call sites in `core/coprocessor.rs`
    // that previously reached through `bus.sio_mut()`, which was a
    // placeholder on `WorkerBus`.

    /// GPIO_OUT bulk read.
    fn gpio_read_out(&self) -> u32;
    /// GPIO_OUT bulk write.
    fn gpio_write_out(&mut self, val: u32);
    /// GPIO_OUT_SET: atomic OR.
    fn gpio_set_out(&mut self, mask: u32);
    /// GPIO_OUT_CLR: atomic AND-NOT.
    fn gpio_clear_out(&mut self, mask: u32);
    /// GPIO_OUT_XOR: atomic XOR.
    fn gpio_xor_out(&mut self, mask: u32);

    /// GPIO_OE bulk read.
    fn gpio_read_oe(&self) -> u32;
    /// GPIO_OE bulk write.
    fn gpio_write_oe(&mut self, val: u32);
    /// GPIO_OE_SET.
    fn gpio_set_oe(&mut self, mask: u32);
    /// GPIO_OE_CLR.
    fn gpio_clear_oe(&mut self, mask: u32);
    /// GPIO_OE_XOR.
    fn gpio_xor_oe(&mut self, mask: u32);

    /// GPIO_IN bulk read. Combined external-pin state as seen by CP0.
    fn gpio_read_in(&self) -> u32;

    /// Wait-state getter / reset. TRANSIENT: used by the decode-execute
    /// fast/slow path and the debug-assert purity check. The canonical
    /// `take_extra_wait_states` subsumes both once the caller is rewired
    /// to drain-on-read semantics.
    fn extra_wait_states(&self) -> u32;
    fn reset_extra_wait_states(&mut self);

    /// Address of the most recently fetched instruction. Used by
    /// `CortexM33::decode` to determine whether the current fetch is
    /// sequential (prefetch buffer absorbs bank 2/6 penalty) or
    /// non-sequential (penalty applies). Silicon fidelity fix from
    /// test_silicon baseline campaign (2026-04-16).
    fn last_fetch_addr(&self) -> u32;
    fn set_last_fetch_addr(&mut self, addr: u32);

    /// MMIO trace sink. TRANSIENT: used by `CortexM33`'s PPB-intercept
    /// read/write wrappers so PPB accesses land in the same wire-format
    /// stream as ordinary bus accesses.
    fn mmio_trace_enabled(&self) -> bool;
    fn emit_mmio_trace(&mut self, rw: char, size: u32, addr: u32, val: u32, core: u8);
}
