//! apmemory APS6404L-3SQR-SN external SPI PSRAM model (8 MB).
//!
//! This is a PicoGUS-v2-specific integration — the pin assignment and the
//! command-byte subset are hardcoded to what the firmware actually uses.
//! It is NOT a generic SPI peripheral abstraction; if a second demo ever
//! needs a different external SPI device, extract a trait *then*.
//!
//! Pinout (fixed, matches PicoGUS v2 hardware):
//!
//! | GPIO | Role |
//! |------|------|
//! | 0    | MISO — PSRAM drives this when CS is low during reads |
//! | 1    | CS#  — active low (PSRAM listens when CS=0) |
//! | 2    | SCK  — bit clock, driven by master |
//! | 3    | MOSI — data from master |
//!
//! Command subset (single-SPI mode only — the firmware never sends the
//! QPI-enter opcode `0x38`, so QPI state is deliberately not modelled):
//!
//! | Opcode | Mnemonic | Frame |
//! |--------|----------|-------|
//! | `0x66` | Reset Enable | 1 cmd byte |
//! | `0x99` | Reset        | 1 cmd byte, must follow `0x66` |
//! | `0x02` | Write        | 1 cmd + 3 addr (BE) + N data |
//! | `0x0B` | Fast Read    | 1 cmd + 3 addr + 8 dummy cycles + N data out |
//!
//! Any other opcode is a silent NOP (buffer unchanged, no MISO drive) —
//! protocol errors should leave subsequent commands working.
//!
//! Real-chip wall-clock delays (50/100 μs reset waits, tRC, tCPH) are
//! NOT modelled — we honour the command sequence and nothing else.
//!
//! # Protocol framing
//!
//! * CS# falling edge starts a new frame: command byte shift register cleared.
//! * CS# rising edge ends the current frame: any partial byte is discarded;
//!   the buffer write done so far is preserved.
//! * Bits are clocked MSB-first on SCK rising edge (master-driven).
//! * The PSRAM drives MISO on SCK falling edge; we update the MISO latch
//!   on falling edges so it's stable for the master to sample on the next
//!   rising edge.

/// PSRAM size: 8 MiB, as on PicoGUS v2 hardware.
pub const PSRAM_SIZE: usize = 8 << 20;
/// Pin assignments (hardcoded).
pub const PIN_MISO: u8 = 0;
pub const PIN_CS: u8 = 1;
pub const PIN_SCK: u8 = 2;
pub const PIN_MOSI: u8 = 3;

const CMD_RESET_ENABLE: u8 = 0x66;
const CMD_RESET: u8 = 0x99;
const CMD_WRITE: u8 = 0x02;
const CMD_FAST_READ: u8 = 0x0B;

/// SPI frame-phase state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// CS is high — no frame in progress.
    Idle,
    /// CS is low; clocking in command byte bits.
    Cmd,
    /// Inside a write: clocking in three address bytes.
    WriteAddr,
    /// Inside a write: streaming data bytes to `buffer[addr..]`.
    WriteData,
    /// Inside a fast-read: clocking in three address bytes.
    ReadAddr,
    /// Inside a fast-read: 8 dummy cycles (1 byte) after address.
    ReadDummy,
    /// Inside a fast-read: clocking data bytes out on MISO.
    ReadData,
    /// Unrecognised command — silent NOP for the rest of the frame.
    SilentNop,
}

/// apmemory APS6404L 8 MB SPI PSRAM model.
pub struct Psram {
    /// Backing storage — fixed-size, zero-alloc hot path.
    pub buffer: Box<[u8; PSRAM_SIZE]>,

    phase: Phase,

    /// Shift register for bits clocked in on MOSI. MSB-first; bit count
    /// drops back to zero once a full byte is consumed.
    shift_in: u8,
    shift_in_bits: u8,

    /// Shift register for bits clocked out on MISO. MSB-first; top bit
    /// is the one the master will sample on the next rising edge.
    shift_out: u8,
    shift_out_bits: u8,

    /// Accumulator for the 3 big-endian address bytes at the start of a
    /// read/write frame.
    addr_bytes_seen: u8,
    addr: u32,

    /// True iff the last completed command was `0x66` (Reset Enable);
    /// enables the next `0x99` to actually reset.
    reset_armed: bool,

    /// Previous SCK / CS observations — edge detection lives here.
    prev_sck: bool,
    prev_cs: bool,
    /// Latched MOSI sample for the most recent SCK rising edge.
    latched_mosi: bool,
    /// Latest MISO bit we want to assert (only meaningful while driving).
    miso_bit: bool,
    /// True while we are actively driving MISO (i.e. inside ReadData /
    /// ReadDummy — MISO is don't-care during dummy cycles in the real
    /// chip's output, but we leave it at 0 so the pin is deterministic).
    driving_miso: bool,

    /// Byte counters — used by write buffer overflow detection. Not
    /// strictly required by the firmware but handy for debugging.
    bytes_written: u64,
    bytes_read: u64,
}

impl Default for Psram {
    fn default() -> Self {
        Self::new()
    }
}

impl Psram {
    pub fn new() -> Self {
        // Allocate the 8 MB buffer directly on the heap — `Box::new([0u8;
        // PSRAM_SIZE])` would materialise the 8 MB array on the stack
        // before moving into a Box, which blows the default 1 MB stack
        // on Windows debug builds. Go through a Vec to force heap alloc
        // and use into_boxed_slice + try_into for the sized-Box.
        let vec = vec![0u8; PSRAM_SIZE].into_boxed_slice();
        let buffer: Box<[u8; PSRAM_SIZE]> = vec
            .try_into()
            .expect("vec of exactly PSRAM_SIZE bytes fits a sized Box");
        Self {
            buffer,
            phase: Phase::Idle,
            shift_in: 0,
            shift_in_bits: 0,
            shift_out: 0,
            shift_out_bits: 0,
            addr_bytes_seen: 0,
            addr: 0,
            reset_armed: false,
            prev_sck: false,
            prev_cs: true,
            latched_mosi: false,
            miso_bit: false,
            driving_miso: false,
            bytes_written: 0,
            bytes_read: 0,
        }
    }

    /// Reset the protocol state machine (buffer preserved). Mirrors the
    /// behaviour of the 0x66+0x99 sequence on the real chip.
    pub fn reset_state(&mut self) {
        self.phase = Phase::Idle;
        self.shift_in = 0;
        self.shift_in_bits = 0;
        self.shift_out = 0;
        self.shift_out_bits = 0;
        self.addr_bytes_seen = 0;
        self.addr = 0;
        self.reset_armed = false;
        self.latched_mosi = false;
        self.miso_bit = false;
        self.driving_miso = false;
    }

    /// Observe the current GPIO pin state. Call on every emulator tick
    /// after the SIO + PIO merge has settled. `pins` is a bitmask where
    /// bit `n` is the level of GPIO`n`.
    ///
    /// Returns `Some(miso_bit)` if the PSRAM is driving MISO this tick,
    /// or `None` if MISO should keep whatever level the bus merge set.
    /// The caller is responsible for splicing the returned bit into
    /// `gpio_in` bit [`PIN_MISO`].
    pub fn tick(&mut self, pins: u32) -> Option<bool> {
        let cs = ((pins >> PIN_CS) & 1) != 0;
        let sck = ((pins >> PIN_SCK) & 1) != 0;
        let mosi = ((pins >> PIN_MOSI) & 1) != 0;

        // CS edge detection has to happen before clock-edge work so a
        // simultaneous CS-rise-and-clock (unusual on real hardware, but
        // possible in a single-tick emulator) ends the frame first.
        let cs_fell = !cs && self.prev_cs;
        let cs_rose = cs && !self.prev_cs;

        if cs_rose {
            self.end_frame();
        }
        if cs_fell {
            self.begin_frame();
        }

        if !cs {
            // Rising edge: master drives MOSI; we latch it.
            let rising = sck && !self.prev_sck;
            let falling = !sck && self.prev_sck;
            if rising {
                self.latched_mosi = mosi;
                self.on_sck_rising();
            } else if falling {
                self.on_sck_falling();
            }
        }

        self.prev_cs = cs;
        self.prev_sck = sck;

        if self.driving_miso {
            Some(self.miso_bit)
        } else {
            None
        }
    }

    // --- Frame-boundary handlers ---------------------------------------------

    fn begin_frame(&mut self) {
        // New frame — clear shift registers and drop to command phase.
        self.phase = Phase::Cmd;
        self.shift_in = 0;
        self.shift_in_bits = 0;
        self.shift_out = 0;
        self.shift_out_bits = 0;
        self.addr_bytes_seen = 0;
        self.addr = 0;
        self.driving_miso = false;
        self.miso_bit = false;
    }

    fn end_frame(&mut self) {
        // Frame ended — partial byte/in-progress command discarded. The
        // reset_armed flag survives so a `0x66` / CS-cycle / `0x99`
        // sequence still resets on the next frame. The buffer state and
        // any data written so far are preserved.
        self.phase = Phase::Idle;
        self.shift_in = 0;
        self.shift_in_bits = 0;
        self.shift_out = 0;
        self.shift_out_bits = 0;
        self.driving_miso = false;
        self.miso_bit = false;
    }

    // --- Clock-edge handlers -------------------------------------------------

    fn on_sck_rising(&mut self) {
        // Clock in one bit on rising edge.
        self.shift_in = (self.shift_in << 1) | (self.latched_mosi as u8);
        self.shift_in_bits += 1;
        if self.shift_in_bits == 8 {
            let byte = self.shift_in;
            self.shift_in = 0;
            self.shift_in_bits = 0;
            self.consume_byte(byte);
        }
    }

    fn on_sck_falling(&mut self) {
        // On falling edge, the PSRAM latches out the next MISO bit.
        // This happens *after* the master has sampled the previous bit
        // on the last rising edge.
        if self.shift_out_bits > 0 {
            self.miso_bit = (self.shift_out & 0x80) != 0;
            self.shift_out <<= 1;
            self.shift_out_bits -= 1;
            if self.shift_out_bits == 0 {
                // Byte fully shifted out — queue the next read byte.
                self.advance_read_byte();
            }
        }
    }

    // --- Per-byte state transitions ------------------------------------------

    fn consume_byte(&mut self, byte: u8) {
        match self.phase {
            Phase::Cmd => self.handle_command(byte),
            Phase::WriteAddr => self.handle_addr_byte(byte, /*is_read=*/ false),
            Phase::WriteData => {
                let off = (self.addr as usize) & (PSRAM_SIZE - 1);
                self.buffer[off] = byte;
                self.addr = self.addr.wrapping_add(1);
                self.bytes_written += 1;
                // Stay in WriteData — further bytes continue to flow.
            }
            Phase::ReadAddr => self.handle_addr_byte(byte, /*is_read=*/ true),
            Phase::ReadDummy => {
                // One byte of dummy cycles — accept and advance. We don't
                // care what the MOSI bits are.
                self.phase = Phase::ReadData;
                self.driving_miso = true;
                self.advance_read_byte();
            }
            Phase::ReadData => {
                // Master can keep clocking to read further bytes; the
                // input bits are don't-care. Nothing to do here — the
                // falling-edge handler drives MISO.
            }
            Phase::Idle | Phase::SilentNop => {
                // Silent — accept bits, produce nothing.
            }
        }
    }

    fn handle_command(&mut self, byte: u8) {
        match byte {
            CMD_RESET_ENABLE => {
                self.reset_armed = true;
                // Command complete; frame continues until CS rises. Any
                // further bytes inside this frame are ignored (treat as
                // silent nop), but CS-rise handling in end_frame() keeps
                // reset_armed so the next frame's 0x99 is effective.
                self.phase = Phase::SilentNop;
            }
            CMD_RESET => {
                if self.reset_armed {
                    // Reset the state machine — clears the in-progress
                    // phase but preserves buffer. `reset_state()` also
                    // clears `reset_armed`, which matches real chip
                    // semantics (reset is a one-shot).
                    self.reset_state();
                } else {
                    // 0x99 without prior 0x66 is a nop per the datasheet.
                    self.phase = Phase::SilentNop;
                }
            }
            CMD_WRITE => {
                self.reset_armed = false;
                self.phase = Phase::WriteAddr;
                self.addr_bytes_seen = 0;
                self.addr = 0;
            }
            CMD_FAST_READ => {
                self.reset_armed = false;
                self.phase = Phase::ReadAddr;
                self.addr_bytes_seen = 0;
                self.addr = 0;
            }
            _ => {
                // Unknown command — silent nop for the rest of the frame.
                self.reset_armed = false;
                self.phase = Phase::SilentNop;
            }
        }
    }

    fn handle_addr_byte(&mut self, byte: u8, is_read: bool) {
        self.addr = (self.addr << 8) | (byte as u32);
        self.addr_bytes_seen += 1;
        if self.addr_bytes_seen == 3 {
            // 24-bit address wraps at 8 MB (0x80_0000) — APS6404 wraps
            // addresses within the chip's address space naturally.
            self.addr &= (PSRAM_SIZE as u32) - 1;
            if is_read {
                self.phase = Phase::ReadDummy;
                // dummy phase consumes exactly one byte before data flows
            } else {
                self.phase = Phase::WriteData;
            }
        }
    }

    /// Load the next read byte into `shift_out` so the falling-edge
    /// handler can clock it out bit-by-bit.
    fn advance_read_byte(&mut self) {
        let off = (self.addr as usize) & (PSRAM_SIZE - 1);
        self.shift_out = self.buffer[off];
        self.shift_out_bits = 8;
        self.addr = self.addr.wrapping_add(1);
        self.bytes_read += 1;
    }

    // --- Test helpers --------------------------------------------------------

    #[cfg(test)]
    pub fn phase_is_idle(&self) -> bool {
        matches!(self.phase, Phase::Idle)
    }

    #[cfg(test)]
    pub fn reset_armed(&self) -> bool {
        self.reset_armed
    }

    #[cfg(test)]
    pub fn bytes_written(&self) -> u64 {
        self.bytes_written
    }
}

// =============================================================================
// Unit tests — PSRAM protocol state machine in isolation (no bus, no PIO).
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Clock one 8-bit byte out on MOSI with CS low. Returns the 8 MISO
    /// bits captured on each SCK rising edge (MSB first) — during the
    /// master's "read" phase the master samples on rising, so that's what
    /// we record for the test oracle.
    fn clock_byte(psram: &mut Psram, pins: &mut u32, byte: u8) -> u8 {
        let mut out: u8 = 0;
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1;
            // Set MOSI before the rising edge.
            *pins = (*pins & !(1 << PIN_MOSI)) | ((bit as u32) << PIN_MOSI);
            // Keep SCK low first — gives PSRAM a falling-edge slot to
            // load the next MISO bit (matches real chip: PSRAM drives on
            // falling edge, master samples on rising).
            *pins &= !(1 << PIN_SCK);
            let _ = psram.tick(*pins);
            // Rise SCK — master samples MISO, PSRAM latches MOSI.
            *pins |= 1 << PIN_SCK;
            let miso = psram.tick(*pins).unwrap_or(false);
            out = (out << 1) | (miso as u8);
        }
        // Drop SCK to leave the bus in a clean state.
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
        out
    }

    /// Drive CS low to open a frame.
    fn cs_fall(psram: &mut Psram, pins: &mut u32) {
        *pins &= !(1 << PIN_CS);
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
    }

    /// Drive CS high to close a frame.
    fn cs_rise(psram: &mut Psram, pins: &mut u32) {
        *pins |= 1 << PIN_CS;
        *pins &= !(1 << PIN_SCK);
        let _ = psram.tick(*pins);
    }

    fn fresh() -> (Psram, u32) {
        // Default idle: CS high, SCK low, MOSI low.
        let psram = Psram::new();
        let pins = 1u32 << PIN_CS;
        (psram, pins)
    }

    #[test]
    fn reset_enable_then_reset_clears_state() {
        let (mut psram, mut pins) = fresh();
        // Start a write but don't complete it, so we have in-progress state.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02); // WRITE
        clock_byte(&mut psram, &mut pins, 0x00); // addr byte 1
        cs_rise(&mut psram, &mut pins);
        // in-progress state was WriteAddr; CS rise drops us to Idle but
        // reset_armed is still false.
        assert!(!psram.reset_armed());

        // Frame 1: Reset Enable (0x66).
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x66);
        cs_rise(&mut psram, &mut pins);
        assert!(psram.reset_armed(), "0x66 must arm reset");

        // Frame 2: Reset (0x99).
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x99);
        cs_rise(&mut psram, &mut pins);
        assert!(!psram.reset_armed(), "0x99 after 0x66 must clear reset_armed");
        assert!(psram.phase_is_idle());
    }

    #[test]
    fn reset_alone_without_enable_is_nop() {
        let (mut psram, mut pins) = fresh();
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x99); // Reset without prior 0x66.
        cs_rise(&mut psram, &mut pins);
        assert!(!psram.reset_armed());
        assert!(psram.phase_is_idle());
    }

    #[test]
    fn write_round_trip() {
        let (mut psram, mut pins) = fresh();
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02); // WRITE
        clock_byte(&mut psram, &mut pins, 0x00); // addr[23:16]
        clock_byte(&mut psram, &mut pins, 0x00); // addr[15:8]
        clock_byte(&mut psram, &mut pins, 0x10); // addr[7:0]
        clock_byte(&mut psram, &mut pins, 0xDE);
        clock_byte(&mut psram, &mut pins, 0xAD);
        clock_byte(&mut psram, &mut pins, 0xBE);
        clock_byte(&mut psram, &mut pins, 0xEF);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(&psram.buffer[0x10..0x14], &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(psram.bytes_written(), 4);
    }

    #[test]
    fn fast_read_returns_written_bytes() {
        let (mut psram, mut pins) = fresh();
        // Prime the buffer.
        psram.buffer[0x10] = 0xDE;
        psram.buffer[0x11] = 0xAD;
        psram.buffer[0x12] = 0xBE;
        psram.buffer[0x13] = 0xEF;

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B); // Fast Read
        clock_byte(&mut psram, &mut pins, 0x00); // addr[23:16]
        clock_byte(&mut psram, &mut pins, 0x00); // addr[15:8]
        clock_byte(&mut psram, &mut pins, 0x10); // addr[7:0]
        clock_byte(&mut psram, &mut pins, 0x00); // 8 dummy cycles (one byte)
        let b0 = clock_byte(&mut psram, &mut pins, 0x00);
        let b1 = clock_byte(&mut psram, &mut pins, 0x00);
        let b2 = clock_byte(&mut psram, &mut pins, 0x00);
        let b3 = clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);

        assert_eq!([b0, b1, b2, b3], [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn fast_read_dummy_cycles_are_ignored() {
        let (mut psram, mut pins) = fresh();
        psram.buffer[0x00] = 0x5A;
        psram.buffer[0x01] = 0xA5;

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        // Send a non-zero dummy byte — output should be unaffected.
        clock_byte(&mut psram, &mut pins, 0xFF);
        let b0 = clock_byte(&mut psram, &mut pins, 0x12);
        let b1 = clock_byte(&mut psram, &mut pins, 0x34);
        cs_rise(&mut psram, &mut pins);

        assert_eq!([b0, b1], [0x5A, 0xA5]);
    }

    #[test]
    fn cs_rise_mid_command_discards_state() {
        let (mut psram, mut pins) = fresh();
        // Begin a write, send cmd + 2 (of 3) addr bytes, then yank CS up.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);
        assert!(psram.phase_is_idle());

        // Start a fresh write to a different address; expected to land
        // cleanly at the new address, unaffected by the aborted frame.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x20);
        clock_byte(&mut psram, &mut pins, 0x77);
        cs_rise(&mut psram, &mut pins);

        assert_eq!(psram.buffer[0x20], 0x77);
        // Nothing was written to the first few bytes of the buffer.
        assert_eq!(psram.buffer[0x00], 0);
        assert_eq!(psram.buffer[0x10], 0);
    }

    #[test]
    fn unknown_command_is_silent_nop() {
        let (mut psram, mut pins) = fresh();
        // 0x9F is READ-ID (per datasheet), which we don't model.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x9F);
        // Clock out a few bytes — we shouldn't be driving MISO.
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        cs_rise(&mut psram, &mut pins);

        // Buffer unchanged.
        assert!(psram.buffer[..].iter().all(|&b| b == 0));

        // Subsequent commands work normally.
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0xAB);
        cs_rise(&mut psram, &mut pins);
        assert_eq!(psram.buffer[0x00], 0xAB);
    }

    #[test]
    fn address_wraps_at_8mb() {
        // APS6404 wraps addresses inside the chip's address space. We
        // replicate this: a write to address 0x80_0001 lands at 0x01.
        let (mut psram, mut pins) = fresh();
        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, 0x80); // addr[23:16] = 0x80 → 8 MB
        clock_byte(&mut psram, &mut pins, 0x00);
        clock_byte(&mut psram, &mut pins, 0x01);
        clock_byte(&mut psram, &mut pins, 0xC3);
        cs_rise(&mut psram, &mut pins);

        assert_eq!(psram.buffer[0x01], 0xC3);
    }

    #[test]
    fn write_then_read_spanning_multiple_bytes() {
        // More thorough round-trip: 16 bytes, arbitrary address.
        let (mut psram, mut pins) = fresh();
        let base_addr: u32 = 0x12_3450;
        let data: [u8; 16] = [
            0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0,
            0xF0, 0x00,
        ];

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x02);
        clock_byte(&mut psram, &mut pins, (base_addr >> 16) as u8);
        clock_byte(&mut psram, &mut pins, (base_addr >> 8) as u8);
        clock_byte(&mut psram, &mut pins, base_addr as u8);
        for b in &data {
            clock_byte(&mut psram, &mut pins, *b);
        }
        cs_rise(&mut psram, &mut pins);

        cs_fall(&mut psram, &mut pins);
        clock_byte(&mut psram, &mut pins, 0x0B);
        clock_byte(&mut psram, &mut pins, (base_addr >> 16) as u8);
        clock_byte(&mut psram, &mut pins, (base_addr >> 8) as u8);
        clock_byte(&mut psram, &mut pins, base_addr as u8);
        clock_byte(&mut psram, &mut pins, 0x00); // dummy
        let mut got = [0u8; 16];
        for (i, slot) in got.iter_mut().enumerate() {
            *slot = clock_byte(&mut psram, &mut pins, i as u8);
        }
        cs_rise(&mut psram, &mut pins);

        assert_eq!(&got, &data);
    }

    #[test]
    fn tick_idle_without_cs_activity_stays_idle() {
        // Degenerate input: CS stays high, SCK and MOSI toggle randomly.
        // Must not affect state.
        let (mut psram, mut pins) = fresh();
        for _ in 0..16 {
            *(&mut pins) ^= 1 << PIN_SCK;
            *(&mut pins) ^= 1 << PIN_MOSI;
            let drive = psram.tick(pins);
            assert!(drive.is_none());
        }
        assert!(psram.phase_is_idle());
    }
}

// =============================================================================
// Bus-integration tests — drive PSRAM via the Emulator's GPIO state directly
// (no PIO program). Proves the update_gpio() hook actually calls psram.tick
// and splices MISO back into gpio_in.
// =============================================================================

#[cfg(test)]
mod bus_integration {
    use super::{PIN_CS, PIN_MISO, PIN_MOSI, PIN_SCK};
    use crate::{Config, Emulator};

    /// Drive the PSRAM's CS/SCK/MOSI pins by poking SIO directly, then
    /// call update_gpio() so the PSRAM observes the change.
    fn drive_pins(emu: &mut Emulator, cs: bool, sck: bool, mosi: bool) {
        // Use GPIO1/2/3 (CS/SCK/MOSI) on SIO with OE asserted.
        let mask = (1u32 << PIN_CS) | (1u32 << PIN_SCK) | (1u32 << PIN_MOSI);
        emu.bus.sio.gpio_oe |= mask;
        let mut out = emu.bus.sio.gpio_out & !mask;
        if cs {
            out |= 1 << PIN_CS;
        }
        if sck {
            out |= 1 << PIN_SCK;
        }
        if mosi {
            out |= 1 << PIN_MOSI;
        }
        emu.bus.sio.gpio_out = out;
        emu.update_gpio();
    }

    /// Clock a single byte out to the PSRAM with CS already low. Returns
    /// the 8 MISO bits sampled on rising edges (MSB first).
    fn clock_byte_via_bus(emu: &mut Emulator, byte: u8) -> u8 {
        let mut out: u8 = 0;
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1;
            drive_pins(emu, false, false, bit != 0);
            drive_pins(emu, false, true, bit != 0);
            // MISO appears as GPIO0 after update_gpio — read it back.
            let miso = ((emu.bus.gpio_in >> PIN_MISO) & 1) as u8;
            out = (out << 1) | miso;
        }
        drive_pins(emu, false, false, false);
        out
    }

    #[test]
    fn bus_hook_write_round_trip() {
        let mut emu = Emulator::new(Config::default());
        // Idle: CS high.
        drive_pins(&mut emu, true, false, false);

        drive_pins(&mut emu, false, false, false); // CS fall
        clock_byte_via_bus(&mut emu, 0x02);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x20);
        clock_byte_via_bus(&mut emu, 0xCA);
        clock_byte_via_bus(&mut emu, 0xFE);
        drive_pins(&mut emu, true, false, false); // CS rise

        assert_eq!(emu.bus.psram.buffer[0x20], 0xCA);
        assert_eq!(emu.bus.psram.buffer[0x21], 0xFE);
    }

    #[test]
    fn bus_hook_miso_drives_gpio_in_bit_zero() {
        let mut emu = Emulator::new(Config::default());
        // Seed the buffer so read returns a known non-zero byte.
        emu.bus.psram.buffer[0x00] = 0xFF; // all 1s — every MISO bit is 1

        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false); // CS fall
        clock_byte_via_bus(&mut emu, 0x0B);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        // Dummy byte.
        clock_byte_via_bus(&mut emu, 0x00);
        // Read one data byte — every bit must come back as 1.
        let got = clock_byte_via_bus(&mut emu, 0x00);
        drive_pins(&mut emu, true, false, false);

        assert_eq!(got, 0xFF,
            "PSRAM must drive GPIO0 (MISO) high for each '1' bit in the read byte");
    }

    #[test]
    fn bus_hook_miso_pio_merge_does_not_clobber_psram() {
        // If PIO1 is NOT asserting OE on GPIO0, the PSRAM's MISO drive
        // must land intact in gpio_in. (In real PicoGUS hardware, PIO1
        // configures GPIO0 as an input for its SPI SM; it doesn't drive
        // GPIO0.) This test just confirms the merge order: psram.tick
        // runs after the SIO+PIO merge, so no PIO OE on GPIO0 means MISO
        // wins.
        let mut emu = Emulator::new(Config::default());
        emu.bus.psram.buffer[0x00] = 0xAA;

        // PIO1 drives a different pin (not GPIO0) — ensure no collision.
        emu.bus.pio[1].pad_oe = 1 << 5;
        emu.bus.pio[1].pad_out = 0;

        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false);
        clock_byte_via_bus(&mut emu, 0x0B);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        let got = clock_byte_via_bus(&mut emu, 0x00);
        drive_pins(&mut emu, true, false, false);

        assert_eq!(got, 0xAA);
    }

    #[test]
    fn bus_hook_reset_clears_psram_state() {
        let mut emu = Emulator::new(Config::default());
        // Get the PSRAM into a non-idle state.
        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false);
        clock_byte_via_bus(&mut emu, 0x02); // WRITE cmd — partial frame
        // Leave the frame in-progress (no CS rise yet).

        // Seed a reset vector so reset() can run.
        emu.bus.memory.load_rom(&[
            0x00, 0x00, 0x03, 0x20,
            0x01, 0x00, 0x00, 0x20,
        ]);
        emu.reset();

        // After reset, PSRAM state machine must be idle again.
        assert!(emu.bus.psram.phase_is_idle(),
            "Emulator::reset() must propagate to psram.reset_state()");
    }
}

// =============================================================================
// PIO-driven integration tests — drive SCK from a PIO program and let
// Emulator::step()'s per-cycle PIO/PSRAM interleave deliver every edge.
//
// What these tests prove that the `bus_integration` tests above don't:
//
//   * Multiple SCK edges happen **inside a single `emu.step()` quantum**
//     (PIO toggles SCK every 2 sysclks; step_quantum=4, so one full SCK
//     period per step). The old pre-fix code — `tick_pio(consumed)`
//     followed by a single `update_gpio()` — would only surface the
//     quantum-end pin snapshot to the PSRAM, missing the SCK edges
//     between quantum start and end. The test therefore fails without
//     the interleave fix.
//
//   * No manual `emu.update_gpio()` call anywhere in the test body —
//     the step loop's per-cycle interleave is solely responsible for
//     feeding the PSRAM its pin view.
// =============================================================================

#[cfg(test)]
mod pio_integration {
    use super::{PIN_CS, PIN_MISO, PIN_MOSI, PIN_SCK};
    use crate::bus::{PIO1_BASE, SIO_BASE};
    use crate::{Config, Emulator, EmulatorBuilder};

    /// Step quantum for these tests. Chosen so one full SCK period
    /// (4 sysclks: rise, hold, fall, hold) exactly matches one quantum.
    /// Each `emu.step()` therefore presents the PSRAM with exactly one
    /// SCK rising edge and one falling edge — provided the interleave
    /// fix is in place.
    const STEP_QUANTUM: u32 = 4;

    /// Install a minimal SCK-generator program into PIO1 SM0 on pin
    /// [`PIN_SCK`], running at system clock (clkdiv = 1). The program:
    ///
    ///   addr 0: SET PINS, 1 [delay=1]   ; SCK rises, 2 cycles total
    ///   addr 1: SET PINS, 0 [delay=1]   ; SCK falls, 2 cycles total
    ///   (wrap addr 1 -> 0)
    ///
    /// Total period: 4 sysclks. One rising edge per 4-cycle quantum.
    fn install_sck_toggler(emu: &mut Emulator) {
        // Instruction encoding (no side-set): [SET 111][delay 00001][dst 000][data 00001]
        const SET_PINS_1_D1: u16 = 0xE101; // SET PINS, 1 with delay=1
        const SET_PINS_0_D1: u16 = 0xE100; // SET PINS, 0 with delay=1

        // INSTR_MEM0 / INSTR_MEM1.
        emu.bus.write32(PIO1_BASE + 0x048, SET_PINS_1_D1 as u32);
        emu.bus.write32(PIO1_BASE + 0x04C, SET_PINS_0_D1 as u32);

        // SM0_PINCTRL: SET count=1, SET base=PIN_SCK (=2).
        let pinctrl = (1u32 << 26) | ((PIN_SCK as u32) << 5);
        emu.bus.write32(PIO1_BASE + 0x0DC, pinctrl);

        // SM0_EXECCTRL: wrap_top=1, wrap_bottom=0.
        let execctrl = (1u32 << 12) | (0u32 << 7);
        emu.bus.write32(PIO1_BASE + 0x0CC, execctrl);

        // Force-execute SET PINDIRS, 1 to mark SCK as an output.
        emu.bus.write32(PIO1_BASE + 0x0D8, 0xE081);

        // NB: not enabled yet — caller enables after CS/MOSI are set up.
    }

    fn enable_sm0(emu: &mut Emulator) {
        // CTRL.SM_ENABLE bit 0 = SM0.
        emu.bus.write32(PIO1_BASE + 0x000, 0x1);
    }

    /// Park core 0 on a long chain of NOPs at 0x2000_0000 so each
    /// `emu.step()` quantum advances exactly `STEP_QUANTUM` sysclks on
    /// the PIO side (each M0+ NOP is a 1-cycle instruction — branches
    /// are 3, so no JMPs here).
    fn park_core0_on_nops(emu: &mut Emulator) {
        let prog = 0x2000_0000u32;
        for i in 0..256u32 {
            emu.bus.write16(prog + i * 2, 0xBF00); // NOP
        }
        emu.cores[0].regs.set_pc(prog);
        emu.cores[0].regs.msp = 0x2003_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    }

    /// Configure CS, MOSI, (and later we'll sample MISO at GPIO0) via
    /// SIO. Both pins start as outputs; CS starts high (idle), MOSI
    /// starts low.
    fn configure_sio_bits(emu: &mut Emulator) {
        let cs_mask = 1u32 << PIN_CS;
        let mosi_mask = 1u32 << PIN_MOSI;
        // GPIO_OE_SET — enable CS + MOSI outputs (leave MISO as input).
        emu.bus.write32(SIO_BASE + 0x024, cs_mask | mosi_mask);
        // GPIO_OUT_SET — CS high initially.
        emu.bus.write32(SIO_BASE + 0x014, cs_mask);
    }

    fn sio_set_mosi(emu: &mut Emulator, bit: bool) {
        let mask = 1u32 << PIN_MOSI;
        if bit {
            emu.bus.write32(SIO_BASE + 0x014, mask); // GPIO_OUT_SET
        } else {
            emu.bus.write32(SIO_BASE + 0x018, mask); // GPIO_OUT_CLR
        }
    }

    fn sio_set_cs(emu: &mut Emulator, high: bool) {
        let mask = 1u32 << PIN_CS;
        if high {
            emu.bus.write32(SIO_BASE + 0x014, mask); // GPIO_OUT_SET
        } else {
            emu.bus.write32(SIO_BASE + 0x018, mask); // GPIO_OUT_CLR
        }
    }

    /// Clock one MSB-first byte out: set MOSI per bit and call
    /// `emu.step()` once — exactly one SCK rising edge per step when
    /// `STEP_QUANTUM == 4` matches the PIO program's period.
    ///
    /// MISO is sampled **before** each step, matching real SPI timing:
    /// the PSRAM updates MISO on SCK falling edges, and the master
    /// samples it on the next rising edge (i.e., at the start of the
    /// next quantum). Sampling after the step would be one bit ahead.
    ///
    /// Returns the MISO byte assembled from bit 0 of `gpio_in`.
    fn clock_out_byte(emu: &mut Emulator, byte: u8) -> u8 {
        let mut miso_byte: u8 = 0;
        for i in 0..8 {
            let bit = ((byte >> (7 - i)) & 1) != 0;
            sio_set_mosi(emu, bit);
            // Sample MISO *before* stepping — this is the bit the PSRAM
            // loaded on the previous quantum's falling edge, which the
            // real master samples on the current rising edge.
            let miso = ((emu.bus.gpio_in >> PIN_MISO) & 1) as u8;
            miso_byte = (miso_byte << 1) | miso;
            emu.step();
        }
        miso_byte
    }

    fn fresh_emu() -> Emulator {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(STEP_QUANTUM)
            .build();
        configure_sio_bits(&mut emu);
        install_sck_toggler(&mut emu);
        park_core0_on_nops(&mut emu);
        // Let initial pin state propagate through one step before SM runs,
        // so the PSRAM's prev_cs latches to CS=high.
        emu.step();
        emu
    }

    #[test]
    fn pio_driven_write_then_read_round_trip() {
        let mut emu = fresh_emu();
        enable_sm0(&mut emu);

        // Write frame: drop CS, clock 0x02, 3 addr bytes (0x00,0x01,0x00
        // for address 0x100), 4 data bytes, raise CS.
        //
        // CS-fall and the first SCK rising edge land in the same quantum —
        // `psram::tick` handles that correctly (begin_frame runs before
        // the clock-edge work on the same tick), so the cmd byte's MSB
        // is the first bit captured.
        sio_set_cs(&mut emu, false);

        clock_out_byte(&mut emu, 0x02); // WRITE cmd
        clock_out_byte(&mut emu, 0x00); // addr [23:16]
        clock_out_byte(&mut emu, 0x01); // addr [15:8]
        clock_out_byte(&mut emu, 0x00); // addr [7:0]
        clock_out_byte(&mut emu, 0xDE);
        clock_out_byte(&mut emu, 0xAD);
        clock_out_byte(&mut emu, 0xBE);
        clock_out_byte(&mut emu, 0xEF);

        sio_set_cs(&mut emu, true);
        emu.step(); // propagate CS-rise to PSRAM.

        assert_eq!(
            &emu.bus.psram.buffer[0x100..0x104],
            &[0xDE, 0xAD, 0xBE, 0xEF],
            "PIO-driven SCK must deliver every rising edge to the PSRAM; \
             missing edges would leave buffer[0x100..0x104] at zero."
        );
    }

    #[test]
    fn pio_driven_fast_read_returns_written_bytes() {
        // Same plumbing, but exercises the fast-read path: 0x0B cmd + 3
        // addr bytes + 1 dummy byte + read MISO for N bytes.
        let mut emu = fresh_emu();
        enable_sm0(&mut emu);

        // Seed the buffer with a known pattern at address 0x200.
        emu.bus.psram.buffer[0x200] = 0x11;
        emu.bus.psram.buffer[0x201] = 0x22;
        emu.bus.psram.buffer[0x202] = 0x33;
        emu.bus.psram.buffer[0x203] = 0x44;

        sio_set_cs(&mut emu, false);

        clock_out_byte(&mut emu, 0x0B); // Fast Read cmd
        clock_out_byte(&mut emu, 0x00); // addr [23:16]
        clock_out_byte(&mut emu, 0x02); // addr [15:8]
        clock_out_byte(&mut emu, 0x00); // addr [7:0]
        clock_out_byte(&mut emu, 0x00); // 8 dummy cycles (one byte)
        let b0 = clock_out_byte(&mut emu, 0x00);
        let b1 = clock_out_byte(&mut emu, 0x00);
        let b2 = clock_out_byte(&mut emu, 0x00);
        let b3 = clock_out_byte(&mut emu, 0x00);

        sio_set_cs(&mut emu, true);
        emu.step();

        assert_eq!([b0, b1, b2, b3], [0x11, 0x22, 0x33, 0x44],
            "PIO-driven fast-read must return the seeded buffer bytes — \
             a single-edge-per-quantum interleave fix is required.");
    }
}
