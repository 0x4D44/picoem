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
//! on `CortexM33` (decode cache, trace sink, the `sio` sub-block, direct
//! `gpio_in`, the `atomics` Arc). Later stages of the Phase 3 roadmap move
//! those onto the core (Stage 3 for DIV/INTERP, a later stage for
//! `decode_cache`, Stage 5 for SIO/GPIO via `SharedState`), at which point
//! the trait surface shrinks back to the 13-method canonical shape.
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

use crate::bus::DecodedOp;
use crate::sio::Sio;
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

    /// CP0 GPIO register-file access. TRANSIENT: moves to `SharedState`
    /// in Stage 5 along with the rest of SIO.
    fn sio(&self) -> &Sio;
    fn sio_mut(&mut self) -> &mut Sio;

    /// Current combined GPIO_IN value. TRANSIENT: moves to
    /// `AtomicGpio` in Stage 5.
    fn gpio_in(&self) -> u32;

    /// Decode cache get / set. TRANSIENT: `decode_cache` moves onto
    /// `CortexM33` in a later stage (V7 §12 final row).
    fn decode_cache_get(&self, slot: usize) -> DecodedOp;
    fn decode_cache_set(&mut self, slot: usize, entry: DecodedOp);

    /// Wait-state getter / reset. TRANSIENT: used by the decode-execute
    /// fast/slow path and the debug-assert purity check. The canonical
    /// `take_extra_wait_states` subsumes both once the caller is rewired
    /// to drain-on-read semantics.
    fn extra_wait_states(&self) -> u32;
    fn reset_extra_wait_states(&mut self);

    /// MMIO trace sink. TRANSIENT: used by `CortexM33`'s PPB-intercept
    /// read/write wrappers so PPB accesses land in the same wire-format
    /// stream as ordinary bus accesses.
    fn trace_enabled(&self) -> bool;
    fn emit_trace(&mut self, rw: char, size: u32, addr: u32, val: u32, core: u8);
}
