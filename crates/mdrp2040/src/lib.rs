//! RP2040 emulator library.
//!
//! Phase 5.A fills in the bus fabric, CLOCKS/RESETS/PLL/XOSC/ROSC
//! register storage, full SIO (GPIO, CPUID, FIFO, spinlocks, divider,
//! interpolators — **no** doorbells / MTIME / coprocessor bridge),
//! IO_BANK0 / PADS_BANK0, XIP_CTRL / SSI stubs, and dual-core stepping
//! (core 0 runs; core 1 stays halted until woken via the SIO FIFO
//! protocol).
//!
//! Phase 5.B wires the two PIO blocks (`bus.pio[0]`, `bus.pio[1]`) into
//! AHB at `0x5020_0000` / `0x5030_0000`, steps them once per emulator
//! step, and merges their pad outputs into `bus.gpio_in` (PIO OE
//! overrides SIO on a per-pin basis, mirroring `mdrp2350::Emulator`).
//!
//! See `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

pub mod bus;
pub mod core;
pub mod memory;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod pio_tests;

pub use self::bus::Bus;
pub use self::core::CortexM0Plus;
pub use self::memory::{Memory, ROM_SIZE, SRAM_SIZE, bank_for_address};

pub use mdpicoem_common::{Clock, PacerSnapshot, PacerStats, Peripheral};
#[cfg(target_arch = "x86_64")]
pub use mdpicoem_common::Pacer;

/// ROSC nominal frequency (~6.5 MHz). RP2040 boots on ROSC at the same
/// nominal rate as RP2350; PLL configuration (if any) happens later in
/// firmware.
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

/// Default quantum size in cycles. Matches `mdrp2350`.
pub const DEFAULT_STEP_QUANTUM: u32 = 64;

/// Top-level RP2040 emulator. Owns dual Cortex-M0+ cores, bus fabric,
/// memory, and clock.
pub struct Emulator {
    pub cores: [CortexM0Plus; 2],
    pub bus: Bus,
    pub clock: Clock,
    /// Cycles advanced per call to [`Self::step`].
    pub step_quantum: u32,
}

impl Emulator {
    /// Create a new emulator with the given configuration.
    pub fn new(config: Config) -> Self {
        EmulatorBuilder::new(config).build()
    }

    /// Reset the emulator:
    /// * Load SP from ROM word 0, PC from ROM word 4 into both cores.
    /// * Core 0 is the bootstrapped core (runs from reset).
    /// * Core 1 is halted — the Pico SDK launches it by writing a
    ///   wake sequence through the SIO FIFO; `step` calls
    ///   [`Self::wake_checks`] each quantum to observe the handshake.
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
        // Core 1 stays halted — bootrom on real silicon parks core 1 in
        // a wait-for-event loop until core 0 sends the wake sequence.
        self.cores[1].halt();

        self.bus.sio.reset();
        self.bus.resets.reset();
        self.bus.clocks_regs.reset();
        self.bus.xosc_regs.reset();
        self.bus.rosc_regs.reset();
        self.bus.pll_sys_regs = bus::clocks::PLL_RESET;
        self.bus.pll_usb_regs = bus::clocks::PLL_RESET;
        self.bus.clock_tree = Default::default();
        self.bus.io_bank0.reset();
        self.bus.pads_bank0.reset();
        for pio in &mut self.bus.pio {
            pio.reset();
        }
        self.bus.clear_bus_fault();
        self.bus.ppb = [Default::default(), Default::default()];
        self.bus.event_flag = [false; 2];
        self.bus.gpio_in = 0;
        self.bus.end_core1_step();

        self.clock = Clock { cycles: 0 };
    }

    /// Load a raw binary at the given address. ROM writes are honoured
    /// (test seeding path); SRAM writes land in the SRAM backing store;
    /// XIP loads use [`Self::load_flash`].
    pub fn load_image(&mut self, addr: u32, data: &[u8]) {
        match addr >> 28 {
            0x0 => {
                // ROM: bootrom-style loads happen via `load_bootrom`.
                // Support ROM overlay here for tests that want to place
                // code at an arbitrary ROM offset without zero-padding.
                let offset = (addr & 0x0FFF_FFFF) as usize;
                let mut rom_buf = vec![0u8; ROM_SIZE];
                // Seed with current ROM content so a partial overlay
                // preserves whatever was already loaded.
                for i in 0..ROM_SIZE {
                    rom_buf[i] = self.bus.memory.rom_read8(i as u32);
                }
                let end = (offset + data.len()).min(ROM_SIZE);
                if offset < ROM_SIZE {
                    rom_buf[offset..end].copy_from_slice(&data[..end - offset]);
                    self.bus.memory.load_rom(&rom_buf);
                }
            }
            0x2 => {
                for (i, &byte) in data.iter().enumerate() {
                    let a = addr.wrapping_add(i as u32);
                    self.bus.memory.sram_write8(a & 0x00FF_FFFF, byte);
                }
            }
            _ => {}
        }
    }

    /// Load the 16 KB RP2040 bootrom at address `0x0000_0000`.
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.bus.load_bootrom(data);
    }

    /// Load an XIP flash image (appears at XIP address `0x1000_0000`).
    pub fn load_flash(&mut self, data: &[u8]) {
        self.bus.load_flash(data);
    }

    /// Advance the system by up to `step_quantum` master-clock cycles,
    /// then tick peripherals once. Returns the number of cycles actually
    /// consumed in this quantum (may be less than `step_quantum` if
    /// core 0 halts mid-quantum).
    ///
    /// Per-instruction interleaving of core 0 and core 1 is preserved so
    /// that bank contention timing on core 1 (`contention_check_active`)
    /// still accounts +1 cycle on same-port accesses. Per-instruction
    /// FIFO wake checks (`maybe_wake_core1`) also remain so a FIFO write
    /// from core 0 wakes core 1 within the same quantum.
    ///
    /// Dual-core schedule (per inner-loop iteration):
    /// 1. Step core 0 — fetch/decode/execute one instruction.
    /// 2. If core 1 is not halted, step it with `contention_check_active`
    ///    so same-bank SRAM accesses incur +1 cycle.
    ///
    /// Once `clock.cycles >= target` (or core 0 halts), advance both PIO
    /// blocks by the quantum's total consumed cycles, merge GPIO
    /// outputs, and run wake checks. Mirrors `mdrp2350::Emulator::step`'s
    /// quantum-end peripheral model; differs in per-iteration core
    /// interleaving, which is required here to preserve bank-contention
    /// timing on core 1.
    pub fn step(&mut self) -> u64 {
        debug_assert!(self.step_quantum > 0, "step_quantum must be >= 1");
        let start = self.clock.cycles;
        let target = start.wrapping_add(self.step_quantum as u64);

        while self.clock.cycles < target && !self.cores[0].is_halted() {
            self.bus.set_active_core(0);
            let c0 = self.cores[0].step(&mut self.bus) as u64;
            self.maybe_wake_core1(0);

            if !self.cores[1].is_halted() {
                self.bus.set_active_core(1);
                self.bus.begin_core1_step();
                let _ = self.cores[1].step(&mut self.bus);
                self.bus.end_core1_step();
                self.maybe_wake_core1(1);
            } else {
                // Still clear any leftover bank-tracking state so the
                // next iteration starts fresh.
                self.bus.end_core1_step();
            }

            self.clock.cycles = self.clock.cycles.wrapping_add(c0);
        }

        let consumed = self.clock.cycles.wrapping_sub(start);
        self.tick_pio(consumed as u32);
        self.update_gpio();
        self.wake_checks();
        consumed
    }

    /// Advance both PIO blocks by `cycles` system-clock cycles.
    ///
    /// PIO reads `bus.gpio_in` as its view of external pin state — feed it
    /// the pre-step merge so programs sampling GPIO (e.g. IN PINS) see the
    /// value SIO / the previous PIO step wrote last. The post-step
    /// `update_gpio()` then refreshes `bus.gpio_in` from `pad_out`/`pad_oe`.
    fn tick_pio(&mut self, cycles: u32) {
        if cycles == 0 {
            return;
        }
        let gpio_in = self.bus.gpio_in;
        for pio in &mut self.bus.pio {
            pio.step_n(cycles, gpio_in);
        }
    }

    /// Run for at least `cycles` virtual cycles. Returns the number of
    /// cycles actually executed. May overshoot by up to `step_quantum - 1`
    /// cycles (one quantum's worth), matching the documented overshoot
    /// behaviour of [`Self::step`].
    pub fn run(&mut self, cycles: u64) -> u64 {
        let start = self.clock.cycles;
        while self.clock.cycles.wrapping_sub(start) < cycles {
            let consumed = self.step();
            if consumed == 0 {
                break;
            }
        }
        self.clock.cycles.wrapping_sub(start)
    }

    /// Merge SIO and PIO GPIO outputs into `bus.gpio_in`.
    ///
    /// SIO `gpio_out & gpio_oe` is the base; each PIO block's
    /// `pad_out & pad_oe` overrides SIO on the pins it drives (PIO wins
    /// wherever `pad_oe` has a bit set — mirrors `mdrp2350::Emulator::
    /// update_gpio`). The result is masked to the RP2040 30-pin range
    /// (GPIO0..GPIO29).
    pub(crate) fn update_gpio(&mut self) {
        let mut out = self.bus.sio.gpio_out & self.bus.sio.gpio_oe;
        for pio in &self.bus.pio {
            let pio_mask = pio.pad_oe;
            out = (out & !pio_mask) | (pio.pad_out & pio_mask);
        }
        self.bus.gpio_in = out & 0x3FFF_FFFF;
    }

    /// WFE/SEV wake check. Phase 5.A doesn't yet model WFE on M0+;
    /// this is kept as a stub so the quantum-end plumbing lands where
    /// Phase 6 (QEMU-diff validation) can hook in. For now, halted
    /// core 1 is woken by `maybe_wake_core1` during FIFO traffic.
    fn wake_checks(&mut self) {
        // Consume any unhandled event flags so they don't latch forever.
        self.bus.event_flag[0] = false;
        // Leave core 1's flag alive — `maybe_wake_core1` observes it
        // once the FIFO push handshake completes.
    }

    /// Observe the Pico SDK multicore-wake handshake. When core 0
    /// writes a non-zero word through FIFO_WR while core 1 is halted,
    /// pop the word and wake core 1 with SP/PC from the next two
    /// handshake words (SDK convention).
    ///
    /// TODO(Phase 6): parse the full `multicore_launch_core1` handshake
    /// (six-word sequence: 0, 0, 1, VTOR, SP, entry) and wake core 1 at
    /// the supplied entry with the supplied SP/VTOR. Current placeholder
    /// just wakes core 1 at its reset PC on any non-zero FIFO push —
    /// enough to exercise the wake plumbing under unit tests, but any
    /// real SDK-based multicore firmware will land at the wrong entry.
    fn maybe_wake_core1(&mut self, writer_core: usize) {
        if writer_core != 0 {
            return;
        }
        if !self.cores[1].is_halted() {
            return;
        }
        if self.bus.event_flag[1] {
            // A FIFO write occurred — treat as a wake signal.
            self.bus.event_flag[1] = false;
            self.cores[1].wake();
        }
    }

    /// Read a GPIO pin from the merged pin state.
    pub fn gpio_read(&self, pin: u8) -> bool {
        if pin >= 30 {
            return false;
        }
        (self.bus.gpio_in >> pin) & 1 != 0
    }

    /// Write a GPIO pin. Sets the SIO GPIO_OUT bit and asserts output
    /// enable so the pin state becomes observable via [`Self::gpio_read`].
    /// Useful as a test-shim to inject a pin level without hand-rolling
    /// the SIO register poking.
    pub fn gpio_write(&mut self, pin: u8, value: bool) {
        if pin >= 30 {
            return;
        }
        let mask = 1u32 << pin;
        self.bus.sio.gpio_oe |= mask;
        if value {
            self.bus.sio.gpio_out |= mask;
        } else {
            self.bus.sio.gpio_out &= !mask;
        }
        self.update_gpio();
    }

    /// Read all GPIO pins as a bitmask.
    pub fn gpio_read_all(&self) -> u64 {
        self.bus.gpio_in as u64
    }

    /// Access core state.
    pub fn core(&self, id: usize) -> &CortexM0Plus {
        &self.cores[id]
    }

    pub fn core_mut(&mut self, id: usize) -> &mut CortexM0Plus {
        &mut self.cores[id]
    }

    /// Direct memory read (bypasses bus timing).
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

/// Builder for assembling the emulator. Seeds the Bus clock tree from
/// `Config::sys_clk_hz` — the first CLOCKS / PLL register write
/// replaces the seed with the derived value.
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
        let mut bus = Bus::new();
        bus.seed_sys_clk_hz(self.config.sys_clk_hz);
        let mut emu = Emulator {
            cores: [CortexM0Plus::with_id(0), CortexM0Plus::with_id(1)],
            bus,
            clock: Clock { cycles: 0 },
            step_quantum: self.step_quantum,
        };
        // Default: core 1 halted — Pico SDK wakes it via SIO FIFO.
        emu.cores[1].halt();
        emu
    }
}

