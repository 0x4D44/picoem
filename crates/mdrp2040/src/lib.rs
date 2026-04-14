//! RP2040 emulator library. Phase 3 skeleton: only `new`/`reset` and
//! direct memory peek/poke/cycles are implemented. All other methods
//! are `todo!()` placeholders tagged with the phase that will fill
//! them in.
//!
//! See `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

pub mod bus;
pub mod core;

#[cfg(test)]
mod tests;

pub use self::bus::Bus;
pub use self::core::CortexM0Plus;

pub use mdpicoem_common::{Clock, PacerSnapshot, PacerStats, Peripheral};
#[cfg(target_arch = "x86_64")]
pub use mdpicoem_common::Pacer;

/// ROSC nominal frequency (~6.5 MHz). RP2040 boots on ROSC at the same
/// nominal rate as RP2350; PLL configuration (if any) happens later in
/// firmware. Re-exported for callers that want the boot frequency
/// without touching the common crate directly.
pub use mdpicoem_common::ROSC_FREQ_HZ;

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

/// Default quantum size in cycles. Matches `mdrp2350`; Phase 4+ may
/// revisit once the M0+ step path is wired up.
pub const DEFAULT_STEP_QUANTUM: u32 = 64;

/// Top-level RP2040 emulator. Owns dual Cortex-M0+ cores, bus fabric,
/// memory, and clock.
///
/// Phase 3 is a skeleton: only `new`/`reset` do real work. `step`,
/// `run`, and anything touching peripherals/PIO/SIO panic with a
/// `todo!` that names the phase responsible.
pub struct Emulator {
    pub cores: [CortexM0Plus; 2],
    pub bus: Bus,
    pub clock: Clock,
    /// Cycles advanced per call to `Emulator::step()`. See
    /// [`DEFAULT_STEP_QUANTUM`].
    pub step_quantum: u32,
}

impl Emulator {
    /// Create a new emulator with the given configuration.
    pub fn new(config: Config) -> Self {
        EmulatorBuilder::new(config).build()
    }

    /// Reset the emulator: load SP from ROM offset 0, PC from ROM offset 4.
    /// Both cores boot from the reset vector.
    ///
    /// Phase 3 only sets registers; the Phase 4 M0+ exception model
    /// will handle reset-exception accounting, MSP/PSP banking, and
    /// NVIC state.
    pub fn reset(&mut self) {
        let initial_sp = self.bus.memory.rom_read32(0);
        let reset_vector = self.bus.memory.rom_read32(4);

        for i in 0..2 {
            self.cores[i] = CortexM0Plus::with_id(i as u8);
            self.cores[i].regs.msp = initial_sp;
            self.cores[i].regs.r[13] = initial_sp;
            self.cores[i].regs.set_pc(reset_vector & !1);
            self.cores[i].regs.xpsr = 1 << 24; // Thumb bit (XPSR_T)
        }

        self.bus.gpio_in = 0;
        self.clock = Clock { cycles: 0 };
    }

    /// Load a raw binary at the given address.
    pub fn load_image(&mut self, _addr: u32, _data: &[u8]) {
        todo!("RP2040 load_image — Phase 5 (address decode)")
    }

    /// Load the 16 KB RP2040 bootrom at address `0x0000_0000`.
    pub fn load_bootrom(&mut self, _data: &[u8]) {
        todo!("RP2040 load_bootrom — Phase 5 (BOOTROM mapping)")
    }

    /// Load an XIP flash image (appears at XIP address `0x1000_0000`).
    /// RP2040 has no onboard flash; the image is served from external
    /// QSPI via XIP_CTRL+SSI. Phase 3 delegates straight to the memory
    /// backing store; Phase 5 will add the XIP_CTRL+SSI wiring.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.bus.load_flash(data);
    }

    /// Advance the system by one quantum.
    ///
    /// Phase 4.B: drives core 0 only — `step` fetches / decodes /
    /// executes a single instruction and returns the cycle cost. Phase 5
    /// will extend this to dual-core scheduling with SIO + contention
    /// bookkeeping.
    pub fn step(&mut self) -> u64 {
        self.bus.set_active_core(0);
        let cycles = self.cores[0].step(&mut self.bus) as u64;
        self.clock.cycles = self.clock.cycles.wrapping_add(cycles);
        cycles
    }

    /// Run for at least `cycles` virtual cycles. Returns the number of
    /// cycles actually executed (which may exceed the target by at most
    /// one instruction's cost).
    pub fn run(&mut self, cycles: u64) -> u64 {
        let start = self.clock.cycles;
        while self.clock.cycles.wrapping_sub(start) < cycles {
            let consumed = self.step();
            if consumed == 0 {
                // Halted core — avoid spinning forever.
                break;
            }
        }
        self.clock.cycles.wrapping_sub(start)
    }

    /// Read a GPIO pin from the merged pin state.
    pub fn gpio_read(&self, _pin: u8) -> bool {
        todo!("RP2040 gpio_read — Phase 5 (IO_BANK0 / PADS_BANK0)")
    }

    /// Write a GPIO pin.
    pub fn gpio_write(&mut self, _pin: u8, _value: bool) {
        todo!("RP2040 gpio_write — Phase 5 (IO_BANK0 / PADS_BANK0)")
    }

    /// Read all GPIO pins as a bitmask.
    pub fn gpio_read_all(&self) -> u64 {
        todo!("RP2040 gpio_read_all — Phase 5 (IO_BANK0)")
    }

    /// Access core state.
    pub fn core(&self, id: usize) -> &CortexM0Plus {
        &self.cores[id]
    }

    pub fn core_mut(&mut self, id: usize) -> &mut CortexM0Plus {
        &mut self.cores[id]
    }

    /// Direct memory read (bypasses bus timing). Phase 3 delegates to
    /// the common `Memory` peek path; Phase 5 will add RP2040-specific
    /// regions (boot ROM alias, XIP, SRAM scratch banks) as needed.
    pub fn peek(&self, addr: u32) -> u32 {
        self.bus.peek32(addr)
    }

    /// Direct memory write (bypasses bus timing).
    pub fn poke(&mut self, addr: u32, value: u32) {
        self.bus.poke32(addr, value);
    }

    /// Current master cycle count.
    pub fn cycles(&self) -> u64 {
        self.clock.cycles
    }
}

/// Builder for assembling the emulator. Phase 3 has no optional
/// components to wire up; Phase 5 will add peripheral injection points
/// similar to `mdrp2350::EmulatorBuilder`.
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
    pub fn step_quantum(mut self, n: u32) -> Self {
        debug_assert!(n > 0, "step_quantum must be >= 1");
        self.step_quantum = n;
        self
    }

    pub fn build(self) -> Emulator {
        // Phase 5 will seed the bus clock tree from `config.sys_clk_hz`.
        // Phase 3 stores the value on `Config` but has no clock tree
        // to seed — referenced here to silence "unused field" warnings.
        let _ = self.config.sys_clk_hz;
        Emulator {
            cores: [CortexM0Plus::with_id(0), CortexM0Plus::with_id(1)],
            bus: Bus::new(),
            clock: Clock { cycles: 0 },
            step_quantum: self.step_quantum,
        }
    }
}
