/// Master cycle counter. All timing in the emulator derives from this.
///
/// At 150 MHz, a u64 counter wraps after ~3,900 years.
///
/// The authoritative system-clock frequency lives in
/// [`crate::bus::Bus`]'s clock tree (see `bus/clocks.rs`). Callers who
/// need Hz should use `emu.bus.sys_clk_hz()`.
pub struct Clock {
    /// Monotonically increasing system clock cycle count.
    pub cycles: u64,
}

impl Clock {
    pub fn new() -> Self {
        Self { cycles: 0 }
    }

    #[inline(always)]
    pub fn tick(&mut self) {
        self.cycles += 1;
    }

    #[inline(always)]
    pub fn advance(&mut self, n: u64) {
        self.cycles += n;
    }
}

impl Default for Clock {
    fn default() -> Self {
        Self::new()
    }
}
