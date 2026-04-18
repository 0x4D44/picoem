use std::sync::Arc;

pub mod core;
pub mod core_riscv;
pub mod bus;
pub mod dma;
pub mod dreq;
pub mod irq;
pub mod memory;
pub mod peripherals;
pub mod sio;
pub mod pio;
pub mod threaded;

use tracing::info;

#[cfg(test)]
mod pio_tests;

#[cfg(test)]
mod tests_narrow;

pub use self::core::CortexM33;
pub use self::core::CoreCounters;
pub use self::core_riscv::Hazard3;
pub use self::bus::Bus;
pub use self::memory::Memory;
pub use self::sio::Sio;

pub use mdpicoem_common::{Clock, PacerSnapshot, PacerStats};
#[cfg(target_arch = "x86_64")]
pub use mdpicoem_common::Pacer;

/// Stop reason when running until a condition.
pub enum StopReason {
    CycleLimit,
    Breakpoint(u32),
    Wfi,
    Fault,
}

/// ROSC nominal frequency (~6.5 MHz). The RP2350 boots on ROSC;
/// PLL configuration (if any) happens later in firmware.
///
/// Re-exported from [`bus::clocks`] for backward compatibility.
pub use self::bus::clocks::ROSC_FREQ_HZ;

/// Emulator configuration.
pub struct Config {
    /// System clock frequency in Hz. Default: ROSC (~6.5 MHz).
    pub sys_clk_hz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sys_clk_hz: ROSC_FREQ_HZ,
        }
    }
}

/// Default quantum size in cycles. Each `Emulator::step()` advances the
/// system by exactly this many virtual cycles; both cores run atomically
/// (instruction-at-a-time) until their per-core cycle count catches up
/// with the target. 64 cycles @ 150 MHz is ~430 ns — well below any
/// firmware-observable timing the emulator currently models.
pub const DEFAULT_STEP_QUANTUM: u32 = 64;

/// Architecture selector. RP2350 ships both an Arm and a RISC-V
/// complex; OTP/POWMAN picks one at power-up. V1 only constructs the
/// Arm path with a real ISA — see
/// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`.
pub enum Arch {
    Arm,
    RiscV,
}

impl Default for Arch {
    fn default() -> Self {
        Arch::Arm
    }
}

/// Per-arch core pair. `expect_arm*` / `expect_riscv*` panic on the
/// wrong arm — documented programmer-error contract for call sites
/// that the shimmed `Emulator::core(id)` path can't cover.
pub enum Cores {
    Arm([CortexM33; 2]),
    RiscV([Hazard3; 2]),
}

impl Cores {
    pub fn expect_arm(&self) -> &[CortexM33; 2] {
        match self {
            Cores::Arm(cs) => cs,
            Cores::RiscV(_) => panic!("expect_arm called on RiscV emulator"),
        }
    }

    pub fn expect_arm_mut(&mut self) -> &mut [CortexM33; 2] {
        match self {
            Cores::Arm(cs) => cs,
            Cores::RiscV(_) => panic!("expect_arm_mut called on RiscV emulator"),
        }
    }

    pub fn expect_riscv(&self) -> &[Hazard3; 2] {
        match self {
            Cores::RiscV(cs) => cs,
            Cores::Arm(_) => panic!("expect_riscv called on Arm emulator"),
        }
    }

    pub fn expect_riscv_mut(&mut self) -> &mut [Hazard3; 2] {
        match self {
            Cores::RiscV(cs) => cs,
            Cores::Arm(_) => panic!("expect_riscv_mut called on Arm emulator"),
        }
    }

    pub fn is_arm(&self) -> bool {
        matches!(self, Cores::Arm(_))
    }

    pub fn is_riscv(&self) -> bool {
        matches!(self, Cores::RiscV(_))
    }
}

/// Top-level RP2350 emulator. Owns dual cores (Arm or RISC-V), bus
/// fabric, memory, and clock. SIO is owned by Bus. Peripherals and PIO
/// are injected via builder.
pub struct Emulator {
    pub cores: Cores,
    pub bus: Bus,
    pub clock: Clock,
    /// Cycles advanced per call to [`Self::step`]. See
    /// [`DEFAULT_STEP_QUANTUM`]. Distinct from `Pacer::quantum_cycles`
    /// which drives wall-clock pacing.
    pub step_quantum: u32,
}

impl Emulator {
    /// Create a new emulator with the given configuration.
    pub fn new(config: Config) -> Self {
        EmulatorBuilder::new(config).build()
    }

    /// Reset the emulator: load SP from ROM word 0, PC from ROM word 1.
    /// Both cores boot from the reset vector.
    pub fn reset(&mut self) {
        let initial_sp = self.bus.memory.rom_read32(0);
        let reset_vector = self.bus.memory.rom_read32(4);

        // Boot both cores from reset vector. Phase 3 Stage 1 (Arm arm):
        // cores share a single `CoreAtomics` with Bus. Rebuilding the
        // cores must reuse the existing Arc so post-reset asserts land
        // on the same state the Bus sees.
        let atomics = Arc::clone(&self.bus.atomics);
        match &mut self.cores {
            Cores::Arm(arm) => {
                for i in 0..2 {
                    arm[i] = CortexM33::new(i as u8, Arc::clone(&atomics));
                    arm[i].regs.msp = initial_sp;
                    arm[i].regs.r[13] = initial_sp;
                    arm[i].regs.set_pc(reset_vector & !1);
                    arm[i].regs.xpsr = 1 << 24; // Thumb bit (XPSR_T)
                }
            }
            Cores::RiscV(cs) => {
                // HLD §4.3: each hart resets to its §4.3 power-on state
                // (pc = 0x2000_0000, CSRs zeroed except mtvec / mcountinhibit,
                // hart_id preserved). Shared bus state resets below, identical
                // to the Arm arm.
                for i in 0..2 {
                    cs[i].reset();
                }
            }
        }

        // Clear the shared atomic state — halted / WFE / event_flag /
        // irq_pending / RCP / bus-fault. Replaces the per-core clears
        // that pre-Stage-1 touched the now-deleted Bus fields.
        atomics.reset();
        self.bus.clear_warned_addrs();
        self.bus.clear_watchdog_reset();
        // WATCHDOG SCRATCH0..7 survive reset by datasheet (§4.7); the
        // rest of the block (CTRL / TIME / LOAD / REASON) quiesces.
        self.bus.watchdog.post_reset();
        // SHA-256 accumulator quiesces on reset (HLD V5 §7.D.6). OTP
        // fuse state and TRNG counter intentionally persist across reset
        // — OTP is physical silicon state, and a persistent counter still
        // yields a unique sequence post-reset.
        self.bus.sha256.reset();
        // HLD V5 §5.7: post-bootrom RESETS state — peripherals
        // released by pico-sdk `runtime_init_bootrom_reset` start
        // deasserted. The emulator never runs the bootrom; we seed
        // the post-bootrom state directly.
        self.bus.resets_state = crate::bus::RESETS_POST_BOOTROM;
        self.bus.ticks.reset();
        self.bus.timer0.reset();
        self.bus.timer1.reset();
        self.bus.sio.reset();
        for pio in &mut self.bus.pio {
            pio.reset();
        }
        self.bus.gpio_in = 0;
        // External-input stimulus (harness-owned pin forcing) survives
        // reset only if the harness re-applies it post-reset. Clearing
        // here matches the real-silicon model: any host stimulus must
        // be re-asserted after a chip reset.
        self.bus.gpio_external_in = 0;
        self.bus.gpio_external_mask = 0;

        // Drop PLL lock-arm state so a post-reset power-up re-arms the
        // counter against the freshly-zeroed master cycle. Mirrors the
        // mdrp2040 reset path.
        self.bus.master_cycle = 0;
        self.bus.pll_sys_lock_at_cycle = None;
        self.bus.pll_usb_lock_at_cycle = None;

        // Fresh `CortexM33::new` above already produces empty decode
        // caches; clear any dirty-range state the old Bus was
        // carrying so it doesn't leak into the next step.
        self.bus.pending_cache_invalidations.clear();
        self.bus.pending_invalidation_regions = 0;

        // Reset clock. The authoritative sys_clk_hz lives on Bus's
        // clock tree (see bus/clocks.rs), so nothing to preserve here.
        self.clock = Clock { cycles: 0 };

        // HLD V5 §5.7: post-bootrom clock tree. firmware running via
        // `load_image` bypasses the bootrom, so scenarios see the
        // pico-sdk post-`runtime_init_clocks` state (clk_sys = 150 MHz,
        // clk_ref = 12 MHz, clk_peri = clk_sys).
        self.bus.seed_post_bootrom_clocks();
    }

    /// Load a raw binary at the given address.
    ///
    /// Supports the RP2350-native SRAM region (`0x2xxx_xxxx`) and the
    /// test-only oracle alias (`0x8xxx_xxxx`) added for the QEMU rv32
    /// differential oracle. See `Bus::canon_oracle_addr` for the
    /// rationale — QEMU virt rv32's only writable RAM lives at
    /// `0x8000_0000`, so the oracle lands code there on both sides.
    pub fn load_image(&mut self, addr: u32, data: &[u8]) {
        for (i, &byte) in data.iter().enumerate() {
            let a = addr.wrapping_add(i as u32);
            match a >> 28 {
                0x0 => {} // ROM is loaded via load_bootrom
                0x2 => self.bus.memory.sram_write8(a & 0x0FFF_FFFF, byte),
                0x8 => self.bus.memory.sram_write8(a & 0x0FFF_FFFF, byte),
                _ => {}
            }
        }
    }

    /// Load the bootrom (32 kB at address 0x00000000). Also invalidates
    /// the ROM-region decode-cache entries on both cores — the bytes
    /// have been replaced wholesale. SRAM / XIP slots are preserved.
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.bus.load_bootrom(data);
        // Bus set the ROM bit in `pending_invalidation_regions`; drain
        // it here so harness / app callers don't need to step before
        // observing the invalidation. Phase 3 follow-up #10 + Task #10
        // review fix — region-scoped to avoid cold-cache regressions.
        let regions = self.bus.pending_invalidation_regions;
        if let Cores::Arm(arm) = &mut self.cores {
            for core in arm.iter_mut() {
                core.invalidate_decode_cache_regions(regions);
            }
        }
        self.bus.pending_invalidation_regions = 0;
    }

    /// Load flash image (appears at XIP address 0x10000000). Invalidates
    /// only the XIP-region decode-cache entries on both cores — SRAM /
    /// ROM slots stay hot, so firmware that reloads flash then runs
    /// SRAM code doesn't pay a cold-cache repopulate tax on the next
    /// quantum.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.bus.load_flash(data);
        let regions = self.bus.pending_invalidation_regions;
        if let Cores::Arm(arm) = &mut self.cores {
            for core in arm.iter_mut() {
                core.invalidate_decode_cache_regions(regions);
            }
        }
        self.bus.pending_invalidation_regions = 0;
    }

    /// Advance the system by one quantum. Each core runs atomically —
    /// instruction-at-a-time — until its per-core cycle count catches up
    /// with the quantum's target. Peripherals tick the full quantum at
    /// the boundary. Returns the post-quantum master cycle count.
    ///
    /// **Overshoot:** a multi-cycle instruction straddling the boundary
    /// leaves `core.cycles > clock.cycles` by up to one instruction's
    /// worth. The next quantum's `while` predicate consumes that overshoot
    /// — the core executes proportionally fewer instructions until its
    /// `cycles` realigns with `clock.cycles`. Over many quanta the rate
    /// averages 1:1. A halted core never contributes `cycles`, so the
    /// `while` predicate never fires and the core is skipped cheaply.
    pub fn step(&mut self) -> u64 {
        debug_assert!(self.step_quantum > 0, "step_quantum must be >= 1");
        // Decode-cache invalidation strategy:
        //   (a) Emulator::load_bootrom/load_flash/reset drain regions
        //       proactively on both cores so pre-step tests see a clean
        //       slate.
        //   (b) Pre-step: drain Bus::pending_invalidation_regions into
        //       both cores. Covers any external `bus.load_*` /
        //       `bus.invalidate_all` pokes that happened between step()
        //       calls without going through Emulator.
        //   (c) Per-instruction: drain Bus::pending_cache_invalidations
        //       into the core that just ran. Covers in-step writes to
        //       executable memory.
        // Do not remove any layer — the test suite exercises all three
        // paths.
        //
        // Phase 3 Stage 2: the Arc-sharing trip-wire lives at the top of
        // `CortexM33::step` — every caller (tests, harness, this driver)
        // funnels through it via `bus.atomics()`. No need to duplicate
        // the check here.
        // Refresh the Bus's view of the master cycle count so any MMIO
        // reads / writes performed during this quantum (notably PLL CS
        // lock bit + lock-arm transitions — see
        // `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 P2)
        // observe a current cycle. Staleness is bounded by one quantum.
        self.bus.master_cycle = self.clock.cycles;
        let target = self.clock.cycles + self.step_quantum as u64;

        // (b) Pre-step region-scoped drain. Firmware-loading paths
        // (`load_bootrom`/`load_flash`) and `Bus::invalidate_all` set
        // bits in `pending_invalidation_regions` on the bus between
        // steps; drain them here (per-core, region-scoped) so stale
        // entries can't survive the reload while preserving any slots
        // outside the touched region. Phase 3 follow-up #10 + Task #10
        // review fix.
        if self.bus.pending_invalidation_regions != 0 {
            let regions = self.bus.pending_invalidation_regions;
            if let Cores::Arm(arm) = &mut self.cores {
                arm[0].invalidate_decode_cache_regions(regions);
                arm[1].invalidate_decode_cache_regions(regions);
            }
            self.bus.pending_invalidation_regions = 0;
        }

        // Compose external stimulus into `bus.gpio_in` before the cores
        // dispatch. `update_gpio` also runs at the end of the quantum
        // (inside `tick_peripherals`); the extra call here catches any
        // `gpio_external_in` / `gpio_external_mask` writes that landed
        // between `step()` invocations, so the cores' first MMIO read
        // of SIO_GPIO_IN in this quantum sees the freshly-composed view
        // instead of a one-quantum-stale value.
        self.update_gpio();

        match &mut self.cores {
            Cores::Arm(cs) => step_pair_arm(cs, &mut self.bus, target),
            Cores::RiscV(cs) => step_pair_riscv(cs, &mut self.bus, target),
        }

        self.clock.advance(self.step_quantum as u64);
        // S4: peripherals tick the full quantum, not `consumed` (bytes
        // the cores actually executed). V5 §5.5 prescribes an
        // unconditional per-cycle tick; batching by `step_quantum`
        // preserves the contract while saving dispatch cost. A halted
        // core skews `core.cycles` against `clock.cycles` by at most one
        // quantum, so the drift never exceeds `step_quantum` cycles — a
        // tolerance the HLD accepts (see V5 §5.5). mdrp2040's tick loop
        // uses `consumed` instead; mdrp2350 explicitly diverges because
        // the ARMv8-M dual-core contention model is disabled here
        // (CLAUDE.md "Bank contention model").
        self.tick_peripherals(self.step_quantum);
        // RISC-V has no SysTick — the SysTick block lives on the M33 PPB.
        if self.cores.is_arm() {
            self.tick_systick();
        }
        // P4: fan-out MTIP/MSIP/MEIP into per-hart `mip` before the wake
        // check. Order matters — `wake_checks` inspects `(mip & mie)` to
        // clear `wfi_parked`, so it needs a freshly-sourced `mip` first.
        // HLD §4.1 / §4.6.
        if self.cores.is_riscv() {
            self.fan_out_riscv_irqs();
        }
        self.wake_checks();
        self.clock.cycles
    }

    /// Drive Hazard3 `mip` bits 3 (MSIP), 7 (MTIP), and 11 (MEIP) from
    /// the per-hart hardware sources. MTIP is level-sensitive from SIO's
    /// `mtime_match_asserted`; MSIP is the per-hart bit of
    /// `SIO.RISCV_SOFTIRQ`; MEIP is computed by the Hazard3 IRQ
    /// controller from `(bus.irq_pending | meifa) & meiea`. HLD §4.6.
    ///
    /// Firmware CSR writes to MSIP/MTIP/MEIP (via `csrrw mip, ...`) are
    /// stomped here on the next quantum — the hardware source wins, per
    /// RV-priv §3.1.9 which classes these bits as hardware-owned.
    fn fan_out_riscv_irqs(&mut self) {
        let Cores::RiscV(cs) = &mut self.cores else { return; };
        let sio = &self.bus.sio;
        for c in 0..2 {
            let mut mip = cs[c].mip();
            // MTIP bit 7 — level-sensitive from SIO.
            if sio.mtime_match_asserted[c] {
                mip |= 1 << 7;
            } else {
                mip &= !(1 << 7);
            }
            // MSIP bit 3 — from RISCV_SOFTIRQ per-hart bits.
            let sw = (sio.riscv_softirq() >> c) & 1;
            if sw != 0 {
                mip |= 1 << 3;
            } else {
                mip &= !(1 << 3);
            }
            // MEIP bit 11 — from Hazard3 IRQ controller (P4).
            let meip = cs[c].compute_meip(self.bus.atomics.irq_pending_load(c));
            if meip {
                mip |= 1 << 11;
            } else {
                mip &= !(1 << 11);
            }
            cs[c].set_mip(mip);
        }
    }

    /// Run for at least `cycles` virtual cycles. Returns the final
    /// master cycle count. May overshoot by up to `step_quantum - 1`
    /// cycles (one quantum's worth), matching the documented overshoot
    /// behaviour of [`Self::step`].
    pub fn run(&mut self, cycles: u64) -> u64 {
        let target = self.clock.cycles + cycles;
        while self.clock.cycles < target {
            self.step();
        }
        self.clock.cycles
    }

    /// Advance peripherals by `cycles` virtual cycles. Called once at the
    /// end of each quantum.
    fn tick_peripherals(&mut self, cycles: u32) {
        let gpio_in = self.bus.gpio_in;
        let resets = self.bus.resets_state;
        // PIO0/1/2 are gated by their RESETS bits — real hardware holds
        // PIO inert while its reset line is asserted. RESET_PIO0..2 are
        // contiguous (11, 12, 13), so `RESET_PIO0 + i` gives the bit
        // for `pio[i]`.
        for (i, pio) in self.bus.pio.iter_mut().enumerate() {
            let bit = crate::bus::RESET_PIO0 + i as u8;
            if (resets & (1u32 << bit)) == 0 {
                pio.step_n(cycles, gpio_in);
            }
        }
        self.route_pio_irqs();
        self.update_gpio();
        // Bus peripherals (TICKS + TIMER0/1 + RISC-V MTIME).
        // HLD V5 §5.3 / §5.5: tick runs every quantum unconditionally,
        // no fast-path gate in V5. MTIME ticks are drained from
        // `TICKS.RISCV` inside `Bus::tick_peripherals` per residual A.2.1
        // (HLD `2026.04.17 - HLD - Residual A.2.1 MTIME WATCHDOG_TICK Fix.md`).
        // Drains alarm-match IRQs into both cores' NVIC pending masks
        // via `assert_irq_shared`.
        self.bus.tick_peripherals(cycles);
    }

    /// Route PIO IRQ flags to the NVIC via INT0_INTE / INT1_INTE masks.
    ///
    /// Each PIO block has two NVIC lines (IRQ_0 and IRQ_1). The 12-bit
    /// raw status (INTR) comprises `IRQ[3:0]` flags plus FIFO status
    /// (TXNFULL / RXNEMPTY). A flag reaches NVIC line N iff
    /// `(INTR & INTn_INTE) | INTn_INTF != 0`.
    ///
    /// PIO IRQs are shared (both cores see them). The IRQ numbers for
    /// each block are contiguous pairs starting at `IRQ_PIO0_IRQ_0`.
    fn route_pio_irqs(&mut self) {
        use crate::irq::IRQ_PIO0_IRQ_0;
        for i in 0..3 {
            // Capture INTS values before mutably borrowing `self.bus` for
            // `assert_irq_shared`.
            let ints0 = self.bus.pio[i].int0_ints_rp2350();
            let ints1 = self.bus.pio[i].int1_ints_rp2350();
            let irq0_line = IRQ_PIO0_IRQ_0 + (i as u32) * 2;
            let irq1_line = irq0_line + 1;
            if ints0 != 0 {
                self.bus.assert_irq_shared(irq0_line);
            }
            if ints1 != 0 {
                self.bus.assert_irq_shared(irq1_line);
            }
        }
    }

    /// Quantum-end SysTick advance. Each core's SysTick is ticked by the
    /// delta between its current `cycles` and the last `systick_advance`
    /// snapshot. The per-core CVR and COUNTFLAG state lives on
    /// `CortexM33::ppb` (Phase 0b.1 Commit B); pending exception delivery
    /// sets `ICSR.PENDSTSET` via `Ppb::pend_systick()` when TICKINT is
    /// enabled.
    fn tick_systick(&mut self) {
        let arm = self.cores.expect_arm_mut();
        for core_id in 0..2 {
            let cycles = arm[core_id].cycles();
            arm[core_id].ppb.systick_advance(cycles);
        }
    }

    /// WFE/SEV and WFI wake checks.
    /// - WFE: if event_flag is set, consume it and wake the core.
    /// - WFI: if an enabled pending IRQ exists, wake the core.
    pub(crate) fn wake_checks(&mut self) {
        match &mut self.cores {
            Cores::Arm(arm) => {
                for i in 0..2 {
                    // WFE wake: event flag clears WFE sleep. Consume
                    // (AcqRel swap to false) pairs with `sev_both`'s
                    // Release.
                    if self.bus.atomics.is_wfe_waiting(i)
                        && self.bus.atomics.event_flag_consume(i)
                    {
                        self.bus.atomics.clear_wfe_waiting(i);
                    }
                    // WFI wake: enabled pending IRQ clears WFI sleep.
                    // The peek is non-consuming; the next step() will
                    // merge via `take_irq_pending`.
                    if self.bus.atomics.is_halted(i) {
                        let pending = self.bus.atomics.irq_pending_load(i);
                        if pending != 0 && arm[i].ppb.any_pending_enabled(pending) {
                            self.bus.atomics.clear_halted(i);
                        }
                    }
                }
            }
            Cores::RiscV(cs) => {
                // HLD §4.6: `wfi` wakes when `(mip & mie) != 0`. The wake
                // decision ignores `mstatus.MIE` — MIE only gates trap
                // *delivery*. If MIE=0 the hart wakes and resumes the
                // next instruction; if MIE=1 the next step() will deliver
                // the trap at instruction boundary.
                for c in cs {
                    if c.wfi_parked && (c.mip() & c.mie()) != 0 {
                        c.wfi_parked = false;
                    }
                }
            }
        }
    }

    /// Merge SIO and PIO GPIO outputs into bus.gpio_in.
    /// PIO output-enable overrides SIO: if a PIO block drives a pin, its value wins.
    ///
    /// External-input stimulus (see [`Bus::gpio_external_mask`]) is overlaid
    /// last so the harness can force pins (CS, address bus, etc.) that would
    /// otherwise be recomputed every tick. Mask-clear bits reflect whatever
    /// SIO/PIO produced; mask-set bits reflect `gpio_external_in`.
    pub(crate) fn update_gpio(&mut self) {
        let mut out = self.bus.sio.gpio_out & self.bus.sio.gpio_oe;
        for pio in &self.bus.pio {
            let pio_mask = pio.pad_oe;
            out = (out & !pio_mask) | (pio.pad_out & pio_mask);
        }
        let ext_mask = self.bus.gpio_external_mask;
        let ext_val = self.bus.gpio_external_in;
        self.bus.gpio_in = (out & !ext_mask) | (ext_val & ext_mask);
    }

    /// Read a GPIO pin from the merged pin state.
    pub fn gpio_read(&self, pin: u8) -> bool {
        (self.bus.gpio_in >> pin) & 1 != 0
    }

    /// Write a GPIO pin (stub for Phase 1).
    pub fn gpio_write(&mut self, _pin: u8, _value: bool) {}

    /// Read all GPIO pins as a bitmask (lower 32 bits).
    pub fn gpio_read_all(&self) -> u64 {
        self.bus.gpio_in as u64
    }

    /// Access core state. **Panics on a RISC-V emulator** — this is a
    /// shim for Arm-only call sites (harness, tests). Cross-arch callers
    /// must dispatch on `cores.is_arm()` first.
    pub fn core(&self, id: usize) -> &CortexM33 {
        &self.cores.expect_arm()[id]
    }

    /// Mutable accessor; same panic contract as [`Self::core`].
    pub fn core_mut(&mut self, id: usize) -> &mut CortexM33 {
        &mut self.cores.expect_arm_mut()[id]
    }

    /// RISC-V counterpart to [`Self::core`]. **Panics on an Arm emulator.**
    pub fn core_riscv(&self, id: usize) -> &Hazard3 {
        &self.cores.expect_riscv()[id]
    }

    /// Mutable accessor; same panic contract as [`Self::core_riscv`].
    pub fn core_riscv_mut(&mut self, id: usize) -> &mut Hazard3 {
        &mut self.cores.expect_riscv_mut()[id]
    }

    /// Get a reference to a core's workload counters. Panics on RISC-V
    /// (Hazard3 has no workload-counters stash yet).
    pub fn core_counters(&self, core_id: usize) -> &CoreCounters {
        &self.cores.expect_arm()[core_id].counters
    }

    /// Reset all core counters. No-op on RISC-V.
    pub fn reset_counters(&mut self) {
        if let Cores::Arm(arm) = &mut self.cores {
            for core in arm.iter_mut() {
                core.counters.reset();
            }
        }
    }

    /// Direct memory read (bypasses bus timing).
    pub fn peek(&self, addr: u32) -> u32 {
        if Bus::is_boot_ram(addr) {
            self.bus.boot_ram_read32(addr)
        } else {
            self.bus.memory.peek32(addr)
        }
    }

    /// Direct memory write (bypasses bus timing).
    ///
    /// **Cache note:** this bypasses the `Bus::write32` path and does
    /// NOT invalidate the per-core decoded-op caches. Callers that poke
    /// into executable memory (ROM / XIP / SRAM) and then `step()` must
    /// call [`Bus::invalidate_all`] on `self.bus` between the poke and
    /// the next `step` to avoid executing stale decoded ops. The flag
    /// is consumed by the next `Emulator::step` pre-step phase, which
    /// invalidates both cores' caches. Pre-boot pokes (the common case
    /// for the harness) happen before any cache entries exist and are
    /// safe without an explicit invalidation.
    pub fn poke(&mut self, addr: u32, value: u32) {
        if Bus::is_boot_ram(addr) {
            self.bus.boot_ram_write32(addr, value);
        } else {
            self.bus.memory.poke32(addr, value);
        }
    }

    /// Current master cycle count.
    pub fn cycles(&self) -> u64 {
        self.clock.cycles
    }

    /// Write a 32-bit word to an MMIO address via the bus. Charges zero
    /// emulator cycles (intended for setup code running outside `run()`).
    ///
    /// Delegates to [`Bus::write32`], so alias bits (`(addr >> 12) & 3`)
    /// are honoured: base address = normal, XOR alias = `|0x1000`, SET
    /// alias = `|0x2000`, CLR alias = `|0x3000`. Useful for poking PIO
    /// INSTR_MEM, configuring SIO GPIO_OE/_OUT, releasing RESETS bits,
    /// etc., without hand-rolling the bus machinery.
    pub fn mmio_write32(&mut self, addr: u32, value: u32) {
        // Mirror the `step()` stash so PLL write-time lock-arm transitions
        // observe the current cycle count when the harness pokes MMIO
        // outside the step path. See HLD §6 P2.
        self.bus.master_cycle = self.clock.cycles;
        // Phase 0b.1 Commit B: PPB addresses live on core 0's per-core
        // PPB from the harness's perspective (same convention as before).
        // Route there directly; mirror any NVIC_ISPR/ICPR writes back to
        // `bus.irq_pending[0]` so the dispatch short-circuit stays in sync.
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            self.core_mut(0).ppb.write32(addr, value);
            let low = addr & 0xFFFF;
            if matches!(low, 0xE200 | 0xE204 | 0xE280 | 0xE284) {
                let word = if low == 0xE200 || low == 0xE280 { 0 } else { 1 };
                let ispr = self.core(0).ppb.nvic_ispr[word]
                    .load(std::sync::atomic::Ordering::Relaxed);
                let mask64 = (ispr as u64) << (word * 32);
                let keep = if word == 0 { !0xFFFF_FFFFu64 } else { 0xFFFF_FFFFu64 };
                let prev = self.bus.atomics.irq_pending_load(0);
                self.bus.atomics.set_irq_pending(0, (prev & keep) | mask64);
            }
        } else {
            self.bus.write32(addr, value, 0);
        }
    }

    /// Read a 32-bit word from an MMIO address via the bus. Charges zero
    /// emulator cycles (intended for setup code running outside `run()`).
    ///
    /// **Warning: reads may have side effects.** Several RP2350 MMIO
    /// registers mutate state on read — e.g. PIO `RXFn` pops the receive
    /// FIFO, SIO divider `QUOTIENT` / `REMAINDER` clear the CSR dirty
    /// bit, and a handful of W1C sticky flags are cleared by reads. Setup
    /// code should therefore be write-heavy; reads through this method
    /// are for confirmation only and should be chosen carefully to avoid
    /// disturbing the peripheral's state.
    pub fn mmio_read32(&mut self, addr: u32) -> u32 {
        // Mirror the `step()` stash so PLL CS reads observe the current
        // cycle count when the harness reads MMIO outside the step path.
        self.bus.master_cycle = self.clock.cycles;
        // Phase 0b.1 Commit B: PPB addresses route to core 0's PPB.
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            self.core_mut(0).ppb.read32(addr)
        } else {
            self.bus.read32(addr, 0)
        }
    }
}

/// Advance both Arm cores up to `target` cycles. Mirrors the original
/// serialised-interleave `step()` body: core 0 first, then core 1. Each
/// `CortexM33` owns its own PPB (Phase 0b.1 Commit B), so no active-core
/// indirection is needed. `update_latest_cycles` publishes the core's
/// cycle counter into its PPB so DWT_CYCCNT reads/writes land on a fresh
/// value — staleness is bounded by one instruction.
fn step_pair_arm(cs: &mut [CortexM33; 2], bus: &mut Bus, target: u64) {
    for core_id in 0..2 {
        // Quantum-boundary IRQ merge: peripherals in `tick_peripherals`
        // at the previous quantum raised IRQs via `assert_irq_*`.
        // Phase 3 Stage 1 (LLD V7 §2) — `take_irq_pending` swaps the
        // mask to zero; a non-zero return replaces the deleted
        // `irq_pending_dirty` flag as the consume-and-merge signal.
        let pending = bus.atomics.take_irq_pending(core_id);
        if pending != 0 {
            cs[core_id].ppb.merge_irq_pending(pending);
        }

        while !cs[core_id].is_halted()
            && !cs[core_id].is_wfe_waiting()
            && cs[core_id].cycles < target
        {
            // Publish the core's cycle count into its PPB before each
            // instruction so DWT_CYCCNT reads/writes land on a fresh
            // value. Staleness is bounded by one instruction.
            let cyc = cs[core_id].cycles;
            cs[core_id].ppb.update_latest_cycles(cyc);
            cs[core_id].step(bus);

            // (c) Drain per-instruction cache-invalidation queue into
            // the core that just ran. Phase 3 follow-up #10 — the
            // decode cache is per-core; writes during this step's bus
            // accesses recorded addresses in
            // `bus.pending_cache_invalidations`. Cross-core SMC still
            // requires firmware DSB+ISB per V7 spec.
            if !bus.pending_cache_invalidations.is_empty() {
                cs[core_id].invalidate_decode_cache_entries(
                    &bus.pending_cache_invalidations,
                );
                bus.pending_cache_invalidations.clear();
            }
            // Region-scoped invalidation triggered mid-step (via
            // `Bus::invalidate_all` or `load_bootrom`/`load_flash`
            // during a step — rare, but used by `Emulator::poke`
            // docs and tests). Drain both cores' caches for the
            // affected regions. Same-step signal so the peer core
            // sees it on its next turn.
            if bus.pending_invalidation_regions != 0 {
                let regions = bus.pending_invalidation_regions;
                cs[0].invalidate_decode_cache_regions(regions);
                cs[1].invalidate_decode_cache_regions(regions);
                bus.pending_invalidation_regions = 0;
            }
        }
        // Final refresh so any post-quantum inspection (e.g. tests
        // reading DWT_CYCCNT between steps) sees a current base.
        let cyc = cs[core_id].cycles;
        cs[core_id].ppb.update_latest_cycles(cyc);

        // Phase 0b.2: exclusive-monitor snoop. If the peer core has an
        // outstanding LDREX address and *this* core performed any
        // data-side write during its quantum slice, invalidate the
        // peer's monitor. Same-core writes do NOT invalidate the local
        // monitor (per ARMv8-M §A3.4). Clear the flag for the next
        // quantum. Correct under the serial-interleave scheduler
        // because cores run sequentially within a quantum; threaded
        // mode (Phase 1+) will require atomic CAS on SharedMemory.
        let peer = 1 - core_id;
        if cs[peer].exclusive_address.is_some() && cs[core_id].did_write_this_quantum {
            cs[peer].exclusive_address = None;
        }
        cs[core_id].did_write_this_quantum = false;
    }
}

/// Advance both RISC-V (Hazard3) cores up to `target` cycles. P1a stub:
/// no per-core PPB stash (RISC-V has no ARMv8-M system-control space),
/// no WFE (Hazard3 models `wfi` differently — see HLD §4.6, handled in
/// P4). Just drives the core's own `step` until it halts or hits the
/// target.
fn step_pair_riscv(cs: &mut [Hazard3; 2], bus: &mut Bus, target: u64) {
    for core_id in 0..2 {
        // Threading removed `bus.set_active_core`; each hart passes its
        // own `hart_id` into `bus.read*` / `write*` / `bus_fault(core)`
        // for MMIO-trace attribution and per-core bus-fault routing.
        while !cs[core_id].is_halted() && cs[core_id].cycles() < target {
            cs[core_id].step(bus);
        }
    }
}

/// Builder for assembling the emulator with optional peripherals.
pub struct EmulatorBuilder {
    config: Config,
    step_quantum: u32,
    arch: Arch,
}

impl EmulatorBuilder {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            step_quantum: DEFAULT_STEP_QUANTUM,
            arch: Arch::default(),
        }
    }

    /// Override the per-step quantum (default [`DEFAULT_STEP_QUANTUM`]).
    /// Useful for benches sweeping quantum size, or tests wanting tighter
    /// peripheral-latency observation.
    pub fn step_quantum(mut self, n: u32) -> Self {
        debug_assert!(n > 0, "step_quantum must be >= 1");
        self.step_quantum = n;
        self
    }

    /// Select the CPU architecture. Defaults to [`Arch::Arm`]; pass
    /// [`Arch::RiscV`] to construct the Hazard3 variant. V1 ships the
    /// placeholder Hazard3 — real ISA lands in P1b.
    pub fn arch(mut self, arch: Arch) -> Self {
        self.arch = arch;
        self
    }

    pub fn build(self) -> Emulator {
        // `Bus::new` already installs the HLD V5 §5.7 post-bootrom clock
        // table (`clk_sys = 150 MHz`, `clk_ref = 12 MHz`). Only override
        // it when the caller supplied a non-default `Config::sys_clk_hz`
        // — overwriting the post-bootrom seed with ROSC for default
        // callers would regress the invariant "Bus::new(), Emulator::new,
        // and Emulator::reset all yield the same clock state".
        //
        // Phase 3 Stage 1: construct a single `Arc<CoreAtomics>` and
        // hand it to Bus plus both cores so cross-core signalling
        // (SEV/event_flag, IRQ pending, bus-fault, RCP) lands on shared
        // state.
        let atomics = Arc::new(crate::threaded::CoreAtomics::default());
        let mut bus = Bus::with_atomics(Arc::clone(&atomics));
        if self.config.sys_clk_hz != Config::default().sys_clk_hz {
            bus.seed_sys_clk_hz(self.config.sys_clk_hz);
        }
        info!(
            rom_size = memory::ROM_SIZE,
            sram_size = memory::SRAM_SIZE,
            step_quantum = self.step_quantum,
            sys_clk_hz = bus.sys_clk_hz(),
            "emulator constructed",
        );
        let cores = match self.arch {
            Arch::Arm => Cores::Arm([
                CortexM33::new(0, Arc::clone(&atomics)),
                CortexM33::new(1, Arc::clone(&atomics)),
            ]),
            Arch::RiscV => Cores::RiscV([Hazard3::new(0), Hazard3::new(1)]),
        };
        // Silence unused-atomics warning on RiscV arm (no atomics wired yet).
        let _ = &atomics;
        Emulator {
            cores,
            bus,
            clock: Clock { cycles: 0 },
            step_quantum: self.step_quantum,
        }
    }
}

#[cfg(test)]
mod tests;
