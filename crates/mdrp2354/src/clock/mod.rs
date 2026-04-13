/// Master cycle counter. All timing in the emulator derives from this.
///
/// At 150 MHz, a u64 counter wraps after ~3,900 years.
pub struct Clock {
    /// Monotonically increasing system clock cycle count.
    pub cycles: u64,
    /// System clock frequency in Hz. Default 150 MHz.
    pub sys_clk_hz: u32,
}

impl Clock {
    pub fn new() -> Self {
        Self {
            cycles: 0,
            sys_clk_hz: 6_500_000,
        }
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
