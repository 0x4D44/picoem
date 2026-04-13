pub mod core;
pub mod bus;
pub mod memory;
pub mod sio;
pub mod clock;

pub use self::core::CortexM33;
pub use self::bus::Bus;
pub use self::memory::Memory;
pub use self::clock::Clock;
pub use self::sio::Sio;

/// Trait for memory-mapped peripherals. Implemented by crates like
/// `mdrp2354-periph`. The core crate defines the interface only.
pub trait Peripheral {
    fn read32(&mut self, offset: u32) -> u32;
    fn write32(&mut self, offset: u32, value: u32);
    /// Called once per system clock. Return true if interrupt asserted.
    fn step(&mut self) -> bool;
}

/// Trait for PIO blocks. Implemented by `mdrp2354-pio`.
pub trait PioInterface {
    /// Advance by one system clock. Returns GPIO output changes if any.
    fn step(&mut self, gpio_in: u64) -> Option<GpioChange>;
    fn read32(&mut self, offset: u32) -> u32;
    fn write32(&mut self, offset: u32, value: u32);
}

/// Describes a GPIO pin state change from a PIO block.
pub struct GpioChange {
    pub pin: u8,
    pub value: bool,
}

/// Stop reason when running until a condition.
pub enum StopReason {
    CycleLimit,
    Breakpoint(u32),
    Wfi,
    Fault,
}

/// Emulator configuration.
pub struct Config {
    /// System clock frequency in Hz. Default 150 MHz.
    pub sys_clk_hz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sys_clk_hz: 150_000_000,
        }
    }
}

/// Top-level RP2354 emulator. Owns dual Cortex-M33 cores, bus fabric,
/// memory, SIO, and clock. Peripherals and PIO are injected via builder.
pub struct Emulator {
    pub cores: [CortexM33; 2],
    pub bus: Bus,
    pub sio: Sio,
    pub clock: Clock,
}

impl Emulator {
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
        self.bus.memory.load_flash(data);
    }

    /// Step the entire system by one clock cycle.
    pub fn step(&mut self) -> u64 {
        self.clock.tick();
        self.cores[0].step(&mut self.bus);
        self.cores[1].step(&mut self.bus);
        self.clock.cycles
    }

    /// Run for N cycles. Returns actual cycles executed.
    pub fn run(&mut self, cycles: u64) -> u64 {
        for _ in 0..cycles {
            self.step();
        }
        self.clock.cycles
    }

    /// Read a GPIO pin (stub — always false for Phase 1).
    pub fn gpio_read(&self, _pin: u8) -> bool {
        false
    }

    /// Write a GPIO pin (stub for Phase 1).
    pub fn gpio_write(&mut self, _pin: u8, _value: bool) {}

    /// Read all 48 GPIO pins as a bitmask (stub).
    pub fn gpio_read_all(&self) -> u64 {
        0
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
        self.bus.read32(addr)
    }

    /// Direct memory write (bypasses bus timing).
    pub fn poke(&mut self, addr: u32, value: u32) {
        self.bus.write32(addr, value);
    }

    /// Current master cycle count.
    pub fn cycles(&self) -> u64 {
        self.clock.cycles
    }
}

/// Builder for assembling the emulator with optional peripherals.
pub struct EmulatorBuilder {
    config: Config,
}

impl EmulatorBuilder {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn build(self) -> Emulator {
        let clock = Clock {
            cycles: 0,
            sys_clk_hz: self.config.sys_clk_hz,
        };
        Emulator {
            cores: [CortexM33::with_id(0), CortexM33::with_id(1)],
            bus: Bus::new(),
            sio: Sio::new(),
            clock,
        }
    }
}

#[cfg(test)]
mod tests;
