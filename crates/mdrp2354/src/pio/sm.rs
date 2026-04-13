use super::fifo::PioFifo;

/// One PIO state machine.
#[allow(dead_code)]  // Pin output fields used in Stage B+
pub struct StateMachine {
    // Program state
    pub(crate) pc: u8,
    pub(crate) x: u32,
    pub(crate) y: u32,
    pub(crate) isr: u32,
    pub(crate) osr: u32,
    pub(crate) isr_count: u8,
    pub(crate) osr_count: u8,

    // Execution state
    pub(crate) delay_count: u8,
    pub(crate) stalled: bool,
    pub(crate) enabled: bool,
    pub(crate) last_insn: u16,

    // Clock divider (16.8 fractional)
    pub(crate) clkdiv_int: u16,
    pub(crate) clkdiv_frac: u8,
    pub(crate) clkdiv_acc: u32,

    // Configuration registers
    pub(crate) execctrl: u32,
    pub(crate) shiftctrl: u32,
    pub(crate) pinctrl: u32,

    // FIFOs
    pub(crate) tx_fifo: PioFifo,
    pub(crate) rx_fifo: PioFifo,

    // Pin output (per-SM, merged into PioBlock.pad_out/pad_oe)
    pub(crate) out_pins: u32,
    pub(crate) out_pindirs: u32,
    pub(crate) set_pins: u32,
    pub(crate) set_pindirs: u32,
    pub(crate) sideset_pins: u32,
}

impl StateMachine {
    pub fn new() -> Self {
        Self {
            pc: 0,
            x: 0,
            y: 0,
            isr: 0,
            osr: 0,
            isr_count: 0,
            osr_count: 0,
            delay_count: 0,
            stalled: false,
            enabled: false,
            last_insn: 0,
            clkdiv_int: 1,
            clkdiv_frac: 0,
            clkdiv_acc: 0,
            execctrl: 0x0001_F000,
            shiftctrl: 0x000C_0000,
            pinctrl: 0x1400_0000,
            tx_fifo: PioFifo::new(4),
            rx_fifo: PioFifo::new(4),
            out_pins: 0,
            out_pindirs: 0,
            set_pins: 0,
            set_pindirs: 0,
            sideset_pins: 0,
        }
    }

    /// Reset to power-on defaults.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Read the CLKDIV register value (int[31:16], frac[15:8]).
    pub fn read_clkdiv(&self) -> u32 {
        ((self.clkdiv_int as u32) << 16) | ((self.clkdiv_frac as u32) << 8)
    }

    /// Write the CLKDIV register value.
    pub fn write_clkdiv(&mut self, val: u32) {
        self.clkdiv_int = (val >> 16) as u16;
        self.clkdiv_frac = (val >> 8) as u8;
    }
}
