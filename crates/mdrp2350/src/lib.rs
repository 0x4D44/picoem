pub mod core;
pub mod bus;
pub mod irq;
pub mod memory;
pub mod peripherals;
pub mod sio;
pub mod pio;

#[cfg(test)]
mod pio_tests;

pub use self::core::CortexM33;
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

/// Top-level RP2350 emulator. Owns dual Cortex-M33 cores, bus fabric,
/// memory, and clock. SIO is owned by Bus. Peripherals and PIO are
/// injected via builder.
pub struct Emulator {
    pub cores: [CortexM33; 2],
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

        // Boot both cores from reset vector. `with_id` already sets
        // cycles = 0, so cycle counters start fresh for a clean quantum
        // alignment with the reset clock.
        for i in 0..2 {
            self.cores[i] = CortexM33::with_id(i as u8);
            self.cores[i].regs.msp = initial_sp;
            self.cores[i].regs.r[13] = initial_sp;
            self.cores[i].regs.set_pc(reset_vector & !1);
            self.cores[i].regs.xpsr = 1 << 24; // Thumb bit (XPSR_T)
        }

        // Clear bus state
        self.bus.clear_bus_fault();
        self.bus.ppb = [Default::default(), Default::default()];
        // HLD V5 §5.7: post-bootrom RESETS state — peripherals
        // released by pico-sdk `runtime_init_bootrom_reset` start
        // deasserted. The emulator never runs the bootrom; we seed
        // the post-bootrom state directly.
        self.bus.resets_state = crate::bus::RESETS_POST_BOOTROM;
        self.bus.ticks.reset();
        self.bus.timer0.reset();
        self.bus.timer1.reset();
        self.bus.irq_pending = [0; 2];
        self.bus.event_flag = [false; 2];
        self.bus.rcp_salt = [0; 2];
        self.bus.rcp_salt_valid = [false; 2];
        self.bus.rcp_count = 0;
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
    pub fn load_image(&mut self, addr: u32, data: &[u8]) {
        for (i, &byte) in data.iter().enumerate() {
            let a = addr.wrapping_add(i as u32);
            match a >> 28 {
                0x0 => {} // ROM is loaded via load_bootrom
                0x2 => self.bus.memory.sram_write8(a & 0x0FFF_FFFF, byte),
                _ => {}
            }
        }
    }

    /// Load the bootrom (32 kB at address 0x00000000). Also invalidates
    /// any decoded-op cache entries that pointed into ROM — the bytes
    /// have been replaced wholesale.
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.bus.load_bootrom(data);
    }

    /// Load flash image (appears at XIP address 0x10000000).
    pub fn load_flash(&mut self, data: &[u8]) {
        self.bus.load_flash(data);
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
        // Refresh the Bus's view of the master cycle count so any MMIO
        // reads / writes performed during this quantum (notably PLL CS
        // lock bit + lock-arm transitions — see
        // `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 P2)
        // observe a current cycle. Staleness is bounded by one quantum.
        self.bus.master_cycle = self.clock.cycles;
        let target = self.clock.cycles + self.step_quantum as u64;

        // Core 0 first, then core 1. `bus.active_core` must be set so that
        // every `bus.ppb[bus.active_core()]` access (NVIC, SCB, SysTick,
        // FPCCR, MPU, fault state) lands on the right per-core PPB.
        for core_id in 0..2 {
            self.bus.set_active_core(core_id);
            while !self.cores[core_id].is_halted()
                && !self.cores[core_id].is_wfe_waiting()
                && self.cores[core_id].cycles < target
            {
                // Publish the core's cycle count into its PPB before each
                // instruction so DWT_CYCCNT reads/writes land on a fresh
                // value. Staleness is bounded by one instruction.
                self.bus.ppb[core_id].update_latest_cycles(self.cores[core_id].cycles);
                self.cores[core_id].step(&mut self.bus);
            }
            // Final refresh so any post-quantum inspection (e.g. tests
            // reading DWT_CYCCNT between steps) sees a current base.
            self.bus.ppb[core_id].update_latest_cycles(self.cores[core_id].cycles);
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
        self.tick_systick();
        self.wake_checks();
        self.clock.cycles
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
        for pio in &mut self.bus.pio {
            pio.step_n(cycles, gpio_in);
        }
        self.update_gpio();
        self.bus.sio.tick_mtime_n(cycles);
        // Bus peripherals (TICKS + TIMER0 + TIMER1 in V5 Phase 1).
        // HLD V5 §5.3 / §5.5: tick runs every quantum unconditionally,
        // no fast-path gate in V5. Drains alarm-match IRQs into both
        // cores' NVIC pending masks via `assert_irq_shared`.
        self.bus.tick_peripherals(cycles);
    }

    /// Quantum-end SysTick advance. Each core's SysTick is ticked by the
    /// delta between its current `cycles` and the last `systick_advance`
    /// snapshot. The per-core CVR and COUNTFLAG state live on
    /// `Bus::ppb[core_id]`; pending exception delivery sets
    /// `ICSR.PENDSTSET` via `Ppb::pend_systick()` when TICKINT is enabled.
    fn tick_systick(&mut self) {
        for core_id in 0..2 {
            self.bus.ppb[core_id].systick_advance(self.cores[core_id].cycles());
        }
    }

    /// WFE/SEV wake check. If a core is WFE-waiting and its event_flag
    /// is set, consume the event and wake the core.
    fn wake_checks(&mut self) {
        for i in 0..2 {
            if self.cores[i].wfe_waiting && self.bus.event_flag[i] {
                self.bus.event_flag[i] = false;
                self.cores[i].wfe_waiting = false;
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

    /// Access core state.
    pub fn core(&self, id: usize) -> &CortexM33 {
        &self.cores[id]
    }

    pub fn core_mut(&mut self, id: usize) -> &mut CortexM33 {
        &mut self.cores[id]
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
    /// NOT invalidate the decoded-op cache. Callers that poke into
    /// executable memory (ROM / XIP / SRAM) and then `step()` must call
    /// [`Bus::invalidate_all`] on `self.bus` between the poke and the
    /// next `step` to avoid executing stale decoded ops. Pre-boot
    /// pokes (the common case for the harness) happen before any cache
    /// entries exist and are safe without an explicit invalidation.
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
        self.bus.write32(addr, value);
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
        self.bus.read32(addr)
    }
}

/// Builder for assembling the emulator with optional peripherals.
pub struct EmulatorBuilder {
    config: Config,
    step_quantum: u32,
}

impl EmulatorBuilder {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            step_quantum: DEFAULT_STEP_QUANTUM,
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

    pub fn build(self) -> Emulator {
        // `Bus::new` already installs the HLD V5 §5.7 post-bootrom clock
        // table (`clk_sys = 150 MHz`, `clk_ref = 12 MHz`). Only override
        // it when the caller supplied a non-default `Config::sys_clk_hz`
        // — overwriting the post-bootrom seed with ROSC for default
        // callers would regress the invariant "Bus::new(), Emulator::new,
        // and Emulator::reset all yield the same clock state".
        let mut bus = Bus::new();
        if self.config.sys_clk_hz != Config::default().sys_clk_hz {
            bus.seed_sys_clk_hz(self.config.sys_clk_hz);
        }
        Emulator {
            cores: [CortexM33::with_id(0), CortexM33::with_id(1)],
            bus,
            clock: Clock { cycles: 0 },
            step_quantum: self.step_quantum,
        }
    }
}

#[cfg(test)]
mod tests;
