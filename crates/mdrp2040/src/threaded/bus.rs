//! `WorkerBus` — the per-CPU-thread `CoreBus` implementation for the
//! RP2040 threaded runtime.
//!
//! Stage 3b.2 (dual-execution HLD V1 §6.4): skeleton only.
//! `WorkerBus::new` constructs cleanly; `impl CoreBus for WorkerBus`
//! satisfies the full 17-method trait. The RAM / ROM path is wired
//! through [`crate::threaded::SharedMemory`] so a worker can already
//! round-trip SRAM. **MMIO peripheral routing is stubbed** — every
//! peripheral region returns 0 on read and no-ops on write with a
//! `TODO(stage-3b.3)` comment pointing at the follow-up.
//!
//! Stage 3b.3 will fan MMIO out through shared peripheral state inside
//! [`crate::threaded::SharedState`]. Stage 3b.4 will drop the
//! `WorkerBus` into `ThreadedEmulator::core_worker_body` where
//! `CortexM0Plus::step(&mut worker_bus)` drives the core.
//!
//! ## PPB and NVIC ownership
//!
//! Per-core PPB and NVIC live locally on the `WorkerBus` — each worker
//! owns the state its core mutates. The `CoreBus` trait takes a
//! `core: usize` index because the inherent `Bus` carries both cores'
//! state in a 2-element array; in the threaded path the caller always
//! passes the worker's own `core_id` (callers derive the index from
//! `bus.active_core()` which is wired to `self.core_id`). We still
//! hold a 2-element array so the trait signatures line up with the
//! serial `Bus` 1:1 — the non-local cell is an inert placeholder that
//! no real code path touches.

use std::sync::Arc;

use crate::bus::ppb::Ppb;
use crate::core::Nvic;
use crate::core::bus_trait::CoreBus;
use crate::threaded::SharedState;

/// Per-CPU-thread bus view. Carries a clone of [`SharedState`] plus the
/// per-instruction accounting fields that in the serial path live
/// directly on `Bus`.
pub struct WorkerBus {
    /// Core ID this worker drives (0 or 1).
    core_id: usize,
    /// Shared state bundle (cheap Arc clone per worker).
    shared: Arc<SharedState>,
    /// Per-core PPB. Only `[core_id]` is actually touched by this
    /// worker's core; the other slot is an inert placeholder so the
    /// trait's `&ppb[core]` indexing maps 1:1 with the serial `Bus`.
    ppb: [Ppb; 2],
    /// Per-core NVIC. Same layout + placeholder convention as `ppb`.
    nvic: [Nvic; 2],
    /// Sticky bus-fault flag (per-worker — M0+ has a single synchronous
    /// fault path and each worker has one core).
    bus_fault: bool,
    /// Address that raised the most recent bus fault.
    bus_fault_addr: u32,
    /// PC of the currently-executing instruction, stashed by the core
    /// decode path before a data-side access.
    active_pc: u32,
}

impl WorkerBus {
    /// Construct a new `WorkerBus` for `core_id` (0 or 1) with the given
    /// [`SharedState`] bundle.
    pub fn new(shared: Arc<SharedState>, core_id: usize) -> Self {
        debug_assert!(core_id < 2, "core_id must be 0 or 1");
        Self {
            core_id,
            shared,
            ppb: [Ppb::new(), Ppb::new()],
            nvic: [Nvic::new(), Nvic::new()],
            bus_fault: false,
            bus_fault_addr: 0,
            active_pc: 0,
        }
    }

    /// Access the shared state bundle (used by the worker loop in
    /// Stage 3b.4 and by diagnostics).
    #[allow(dead_code)]
    pub(crate) fn shared(&self) -> &Arc<SharedState> {
        &self.shared
    }
}

/// True when `addr` resolves to either SRAM (`0x2???_????`) or boot ROM
/// (`0x0000_0000..=0x0000_3FFF`). Everything else is MMIO in the RP2040
/// address space and will be routed through `SharedState` in Stage
/// 3b.3.
#[inline]
fn is_ram_or_rom_addr(addr: u32) -> bool {
    let region = addr >> 28;
    if region == 0x2 {
        // SRAM (or one of its aliases). SharedMemory caps at SRAM_SIZE
        // internally so an out-of-range offset still maps cleanly to
        // "unmapped" — we accept the whole 0x2 region here.
        return true;
    }
    // Boot ROM sits in region 0 at 0x0000_0000..=0x0000_3FFF (16 KB).
    addr < crate::threaded::memory::ROM_SIZE
}

impl CoreBus for WorkerBus {
    // --- Memory access ------------------------------------------------

    fn read8(&mut self, addr: u32) -> u8 {
        if is_ram_or_rom_addr(addr) {
            self.shared.memory.read8(addr)
        } else {
            // TODO(stage-3b.3): route MMIO through shared state.
            0
        }
    }

    fn read16(&mut self, addr: u32) -> u16 {
        if is_ram_or_rom_addr(addr) {
            self.shared.memory.read16(addr)
        } else {
            // TODO(stage-3b.3): route MMIO through shared state.
            0
        }
    }

    fn read32(&mut self, addr: u32) -> u32 {
        if is_ram_or_rom_addr(addr) {
            self.shared.memory.read32(addr)
        } else {
            // TODO(stage-3b.3): route MMIO through shared state.
            0
        }
    }

    fn write8(&mut self, addr: u32, val: u8) {
        if is_ram_or_rom_addr(addr) {
            self.shared.memory.write8(addr, val);
        } else {
            // TODO(stage-3b.3): route MMIO through shared state.
        }
    }

    fn write16(&mut self, addr: u32, val: u16) {
        if is_ram_or_rom_addr(addr) {
            self.shared.memory.write16(addr, val);
        } else {
            // TODO(stage-3b.3): route MMIO through shared state.
        }
    }

    fn write32(&mut self, addr: u32, val: u32) {
        if is_ram_or_rom_addr(addr) {
            self.shared.memory.write32(addr, val);
        } else {
            // TODO(stage-3b.3): route MMIO through shared state.
        }
    }

    // --- Instruction-boundary metadata --------------------------------

    fn set_active_pc(&mut self, pc: u32) {
        self.active_pc = pc;
    }

    // --- Bus fault ----------------------------------------------------

    fn bus_fault(&self) -> bool {
        self.bus_fault
    }

    fn bus_fault_addr(&self) -> u32 {
        self.bus_fault_addr
    }

    fn clear_bus_fault(&mut self) {
        self.bus_fault = false;
    }

    // --- Per-core PPB / NVIC ------------------------------------------

    fn ppb(&self, core: usize) -> &Ppb {
        &self.ppb[core]
    }

    fn ppb_mut(&mut self, core: usize) -> &mut Ppb {
        &mut self.ppb[core]
    }

    fn nvic(&self, core: usize) -> &Nvic {
        &self.nvic[core]
    }

    fn nvic_mut(&mut self, core: usize) -> &mut Nvic {
        &mut self.nvic[core]
    }

    // --- Scheduler plumbing -------------------------------------------

    fn active_core(&self) -> usize {
        self.core_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::threaded::memory::{SRAM_BASE, ROM_BASE};

    #[test]
    fn construct_and_basic_sram_roundtrip() {
        let shared = Arc::new(SharedState::new_default());
        let mut bus = WorkerBus::new(shared.clone(), 0);

        // Active core is whatever we constructed with.
        assert_eq!(bus.active_core(), 0);

        // SRAM write through the WorkerBus is observable from the
        // shared state, and readable back through the same bus.
        bus.write32(SRAM_BASE + 0x100, 0xDEAD_BEEF);
        assert_eq!(bus.read32(SRAM_BASE + 0x100), 0xDEAD_BEEF);
        // Cross-check via the underlying SharedMemory — round-trip is
        // genuinely going through the shared Arc, not a local cache.
        assert_eq!(shared.memory.read32(SRAM_BASE + 0x100), 0xDEAD_BEEF);

        // ROM reads back zero on a fresh memory (no bootrom loaded).
        // This also exercises the RAM/ROM classifier routing ROM through
        // SharedMemory rather than the MMIO stub path.
        assert_eq!(bus.read32(ROM_BASE), 0);

        // MMIO stub path: every peripheral read returns 0, writes drop.
        // TODO(stage-3b.3): replace with real routing tests.
        assert_eq!(bus.read32(0x4000_8000), 0); // CLOCKS base
        bus.write32(0x4000_8000, 0xDEAD_BEEF); // no panic, no effect
        assert_eq!(bus.read32(0x4000_8000), 0);

        // PPB / NVIC hand out live references indexed by core id.
        bus.ppb_mut(0).vtor = 0x1000_0100;
        assert_eq!(bus.ppb(0).vtor, 0x1000_0100);
        bus.nvic_mut(0).pending = 0xFF;
        assert_eq!(bus.nvic(0).pending, 0xFF);

        // Bus-fault slot round-trip.
        assert!(!bus.bus_fault());
        bus.clear_bus_fault();
        assert_eq!(bus.bus_fault_addr(), 0);

        // PC stash is a plain setter.
        bus.set_active_pc(0x1000_0200);
        // No reader — just prove the setter compiles and doesn't panic.
        let _ = bus.active_pc;
    }
}
