pub mod core;
pub mod bus;
pub mod memory;
pub mod sio;
pub mod clock;
pub mod pacer;
pub mod pio;

pub use self::core::CortexM33;
pub use self::bus::Bus;
pub use self::memory::Memory;
pub use self::clock::Clock;
pub use self::sio::Sio;
pub use self::pacer::{PacerStats, PacerSnapshot};
#[cfg(target_arch = "x86_64")]
pub use self::pacer::Pacer;

/// Trait for memory-mapped peripherals. Implemented by crates like
/// `mdrp2354-periph`. The core crate defines the interface only.
pub trait Peripheral {
    fn read32(&mut self, offset: u32) -> u32;
    fn write32(&mut self, offset: u32, value: u32);
    /// Called once per system clock. Return true if interrupt asserted.
    fn step(&mut self) -> bool;
}

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

/// Top-level RP2354 emulator. Owns dual Cortex-M33 cores, bus fabric,
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
        self.bus.resets_state = 0x1FFF_FFFF;
        self.bus.event_flag = [false; 2];
        self.bus.rcp_salt = [0; 2];
        self.bus.rcp_salt_valid = [false; 2];
        self.bus.rcp_count = 0;
        self.bus.sio.reset();
        for pio in &mut self.bus.pio {
            pio.reset();
        }
        self.bus.gpio_in = 0;

        // Reset clock. The authoritative sys_clk_hz lives on Bus's
        // clock tree (see bus/clocks.rs), so nothing to preserve here.
        self.clock = Clock { cycles: 0 };
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

    /// Load the bootrom (32 kB at address 0x00000000).
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.bus.memory.load_rom(data);
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
                self.cores[core_id].step(&mut self.bus);
            }
        }

        self.clock.advance(self.step_quantum as u64);
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
    }

    /// Quantum-end SysTick advance. Stage 2 wiring — no-op for now;
    /// DWT CYCCNT and SysTick CVR continue to return hardcoded 0
    /// from `Ppb::read32` until the SysTick/DWT work lands.
    fn tick_systick(&mut self) {
        // Intentionally empty — Stage 2.
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
    pub(crate) fn update_gpio(&mut self) {
        let mut out = self.bus.sio.gpio_out & self.bus.sio.gpio_oe;
        for pio in &self.bus.pio {
            let pio_mask = pio.pad_oe;
            out = (out & !pio_mask) | (pio.pad_out & pio_mask);
        }
        self.bus.gpio_in = out;
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
        self.step_quantum = n;
        self
    }

    pub fn build(self) -> Emulator {
        // Seed Bus's clock tree from Config::sys_clk_hz (vestigial
        // seed per LLD V2 §4.9). First write to any CLOCKS/PLL
        // register replaces the seed with the derived value.
        let mut bus = Bus::new();
        bus.seed_sys_clk_hz(self.config.sys_clk_hz);
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
