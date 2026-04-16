//! LCD bit-bang decoder for the emulator showcase.
//!
//! The firmware drives a 20x2 character LCD over three GPIO pins:
//!
//! | GPIO | Role                                         |
//! |------|----------------------------------------------|
//! | SCLK | rising edge latches DATA                     |
//! | DATA | MSB-first, stable while SCLK is high         |
//! | CS   | active low; frames a transaction             |
//! |------|----------------------------------------------|
//!
//! Pin numbers are configurable via [`LcdDecoder::new`].
//!
//! Each frame is the sequence of bytes shifted in between a CS falling edge
//! and the next CS rising edge. The first byte of a frame is the opcode
//! (LLD §7.3):
//!
//! | Opcode | Name       | Args      | Effect                               |
//! |--------|------------|-----------|--------------------------------------|
//! | `0x01` | CLEAR      | —         | Fill rows with spaces, cursor=(0,0). |
//! | `0x02` | SET_CURSOR | col, row  | Clamp to [0,19]×[0,1].               |
//! | `0x03` | WRITE      | char+...  | Write each char at the cursor.       |
//!
//! WRITE advances the cursor after each character, wraps at column 20, and
//! scrolls at row 2 (row 1 copies down to row 0, row 1 cleared to spaces).
//! Any frame whose first byte is not one of these three opcodes (including
//! `0x00` and `0x04..=0xFF`) is silently dropped — no characters written,
//! no state change. A zero-byte frame is likewise a no-op.
//!
//! The decoder is fed one sample per sim-thread quantum via [`LcdDecoder::sample`].
//! Because it only sees the GPIO state at the end of each quantum, the
//! firmware must hold every signal level that the decoder needs to observe
//! for at least `2 * quantum_cycles` cycles (the contract in LLD §4.1).

// ---- observable state -------------------------------------------------------

const LCD_COLS: usize = 20;
const LCD_ROWS: usize = 2;

#[derive(Clone)]
pub struct LcdState {
    pub rows: [[u8; 20]; 2],
    pub cursor: (u8, u8),
}

impl Default for LcdState {
    fn default() -> Self {
        Self {
            rows: [[b' '; 20]; 2],
            cursor: (0, 0),
        }
    }
}

// ---- decoder ----------------------------------------------------------------

#[derive(Default)]
pub struct LcdDecoder {
    sclk_mask: u32,
    data_mask: u32,
    cs_mask: u32,
    state: LcdState,
    prev_gpio: u32,
    have_prev: bool,
    in_frame: bool,
    shift: u8,
    bit_count: u8,
    rx_buf: Vec<u8>,
}

impl LcdDecoder {
    pub fn new(sclk_pin: u8, data_pin: u8, cs_pin: u8) -> Self {
        Self {
            sclk_mask: 1u32 << sclk_pin,
            data_mask: 1u32 << data_pin,
            cs_mask: 1u32 << cs_pin,
            ..Self::default()
        }
    }

    /// Feed one post-quantum GPIO snapshot into the decoder.
    pub fn sample(&mut self, gpio_out: u32) {
        if !self.have_prev {
            self.prev_gpio = gpio_out;
            self.have_prev = true;
            return;
        }

        let cs_now = (gpio_out & self.cs_mask) == 0;
        let cs_prev = (self.prev_gpio & self.cs_mask) == 0;
        let sclk_now = (gpio_out & self.sclk_mask) != 0;
        let sclk_prev = (self.prev_gpio & self.sclk_mask) != 0;
        let data_now = (gpio_out & self.data_mask) != 0;

        let cs_falling = cs_now && !cs_prev;

        // CS falling edge: start a new frame.
        if cs_falling {
            self.in_frame = true;
            self.shift = 0;
            self.bit_count = 0;
            self.rx_buf.clear();
        }

        // SCLK rising edge inside a frame: shift in one bit (MSB first).
        // Note: the CS-falling sample is excluded — we wait until the next
        // sample (or later) to see a SCLK rising edge. Including the CS
        // sample itself would produce a phantom bit when the firmware's
        // CS-falling sample happens to have SCLK=H.
        if self.in_frame && !cs_falling && sclk_now && !sclk_prev {
            self.shift = (self.shift << 1) | u8::from(data_now);
            self.bit_count += 1;
            if self.bit_count == 8 {
                self.rx_buf.push(self.shift);
                self.shift = 0;
                self.bit_count = 0;
            }
        }

        // CS rising edge: frame is complete, interpret the buffered bytes.
        if !cs_now && cs_prev {
            self.in_frame = false;
            self.apply_frame();
        }

        self.prev_gpio = gpio_out;
        // Normalise prior SCLK state so the first in-frame SCLK=H sample is
        // seen as a rising edge even if SCLK was already HIGH at CS-fall
        // (e.g. leftover state from a prior firmware, or the very first
        // sample of a run). Without this, bit 7 would be silently lost.
        if cs_falling {
            self.prev_gpio &= !self.sclk_mask;
        }
    }

    pub fn state(&self) -> LcdState {
        self.state.clone()
    }

    fn apply_frame(&mut self) {
        let bytes = std::mem::take(&mut self.rx_buf);
        let Some(&first) = bytes.first() else {
            // Zero-byte frame: no opcode, nothing to do.
            return;
        };

        match first {
            0x01 => {
                self.state.rows = [[b' '; LCD_COLS]; LCD_ROWS];
                self.state.cursor = (0, 0);
            }
            0x02 => {
                let col = bytes.get(1).copied().unwrap_or(0);
                let row = bytes.get(2).copied().unwrap_or(0);
                let col = col.min((LCD_COLS - 1) as u8);
                let row = row.min((LCD_ROWS - 1) as u8);
                self.state.cursor = (col, row);
            }
            0x03 => {
                for &b in &bytes[1..] {
                    self.write_char(b);
                }
            }
            // Unknown opcode (0x00, 0x04..=0xFF): silently drop the frame.
            _ => {}
        }
    }

    fn write_char(&mut self, c: u8) {
        let (mut col, mut row) = self.state.cursor;
        // Lazy wrap from a prior write that pushed the cursor off-row:
        // kept as a safety net even though the post-write branch below
        // now wraps eagerly, so external readers never see col>=LCD_COLS.
        if col as usize >= LCD_COLS {
            col = 0;
            row = row.saturating_add(1);
        }
        if row as usize >= LCD_ROWS {
            self.scroll_up();
            row = (LCD_ROWS - 1) as u8;
        }
        self.state.rows[row as usize][col as usize] = c;
        col += 1;
        // Eager wrap: never leave the cursor at col==LCD_COLS. If the next
        // character would land off the current row, advance row now and
        // scroll if that falls off the bottom. This keeps `self.state.cursor`
        // in-range for external readers between writes.
        if col as usize >= LCD_COLS {
            col = 0;
            row = row.saturating_add(1);
            if row as usize >= LCD_ROWS {
                self.scroll_up();
                row = (LCD_ROWS - 1) as u8;
            }
        }
        self.state.cursor = (col, row);
    }

    fn scroll_up(&mut self) {
        for r in 1..LCD_ROWS {
            self.state.rows[r - 1] = self.state.rows[r];
        }
        self.state.rows[LCD_ROWS - 1] = [b' '; LCD_COLS];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_SCLK: u8 = 14;
    const TEST_DATA: u8 = 15;
    const TEST_CS: u8 = 16;

    /// Builds a GPIO word with the three LCD pins set to the requested state.
    fn make_gpio(cs_low: bool, sclk_high: bool, data_high: bool) -> u32 {
        let mut g: u32 = 0;
        if !cs_low {
            g |= 1u32 << TEST_CS;
        }
        if sclk_high {
            g |= 1u32 << TEST_SCLK;
        }
        if data_high {
            g |= 1u32 << TEST_DATA;
        }
        g
    }

    /// Pushes one byte worth of samples through the decoder, MSB-first.
    /// Precondition: caller has already driven CS low in a prior sample.
    fn push_byte(dec: &mut LcdDecoder, byte: u8) {
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1 != 0;
            // Hold DATA with SCLK low (idle).
            dec.sample(make_gpio(true, false, bit));
            // Rising SCLK edge — decoder shifts the bit in.
            dec.sample(make_gpio(true, true, bit));
            // Falling SCLK edge, keep CS low.
            dec.sample(make_gpio(true, false, bit));
        }
    }

    fn start_frame(dec: &mut LcdDecoder) {
        // CS high (idle) then CS low (frame start).
        dec.sample(make_gpio(false, false, false));
        dec.sample(make_gpio(true, false, false));
    }

    fn end_frame(dec: &mut LcdDecoder) {
        // CS rising edge — frame is applied.
        dec.sample(make_gpio(false, false, false));
    }

    /// Shortcut: push a whole frame's payload (opcode + args) through the
    /// decoder. The caller is responsible for the opcode byte — we just
    /// bracket the bytes with CS low / CS high.
    fn push_frame(dec: &mut LcdDecoder, payload: &[u8]) {
        start_frame(dec);
        for &b in payload {
            push_byte(dec, b);
        }
        end_frame(dec);
    }

    #[test]
    fn clear_set_cursor_and_write_hi() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);

        // Frame 1: CLEAR
        push_frame(&mut dec, &[0x01]);
        // Frame 2: SET_CURSOR 0, 0
        push_frame(&mut dec, &[0x02, 0, 0]);
        // Frame 3: WRITE "Hi"
        push_frame(&mut dec, &[0x03, b'H', b'i']);

        let state = dec.state();
        assert_eq!(state.rows[0][0], b'H');
        assert_eq!(state.rows[0][1], b'i');
        assert_eq!(state.rows[0][2], b' ');
        assert_eq!(state.cursor, (2, 0));
    }

    #[test]
    fn clear_fills_rows_with_spaces_and_homes_cursor() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);

        // Dirty the display first by writing an 'X' at (5, 1).
        push_frame(&mut dec, &[0x02, 5, 1]);
        push_frame(&mut dec, &[0x03, b'X']);

        assert_eq!(dec.state().rows[1][5], b'X');

        // CLEAR.
        push_frame(&mut dec, &[0x01]);

        let state = dec.state();
        assert!(state.rows.iter().all(|row| row.iter().all(|&b| b == b' ')));
        assert_eq!(state.cursor, (0, 0));
    }

    #[test]
    fn wrap_and_scroll_when_row1_overflows() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);

        // Clear, then position cursor at (19, 1).
        push_frame(&mut dec, &[0x01]);
        push_frame(&mut dec, &[0x02, 19, 1]);
        // Write two chars: 'A' at (19, 1), then 'B' should wrap and scroll.
        push_frame(&mut dec, &[0x03, b'A', b'B']);

        let state = dec.state();
        // After wrap+scroll: row 0 now shows what row 1 used to show, which
        // was spaces except for 'A' at column 19. 'B' landed at (0, 1).
        assert_eq!(state.rows[0][19], b'A');
        assert_eq!(state.rows[1][0], b'B');
    }

    // =========================================================================
    // Edge cases from Phase 2 review (fixes 5, 6, 7)
    // =========================================================================

    #[test]
    fn unknown_opcode_is_silently_dropped() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        // Prime with a known-good state.
        push_frame(&mut dec, &[0x03, b'X']);
        let before = dec.state();
        assert_eq!(before.rows[0][0], b'X');

        // Unknown opcode 0x00 followed by what would otherwise be "abc".
        push_frame(&mut dec, &[0x00, b'a', b'b', b'c']);
        // Unknown opcode 0x7F: random noise.
        push_frame(&mut dec, &[0x7F, b'!', b'!']);
        // Unknown opcode 0xFF: all-ones.
        push_frame(&mut dec, &[0xFF, b'?']);

        let after = dec.state();
        assert_eq!(after.rows, before.rows, "unknown opcode must not write");
        assert_eq!(after.cursor, before.cursor, "cursor must not move");
    }

    #[test]
    fn zero_byte_frame_is_a_noop() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        // Prime with a known-good state.
        push_frame(&mut dec, &[0x03, b'Y']);
        let before = dec.state();

        // CS low immediately followed by CS high — no bytes shifted in.
        start_frame(&mut dec);
        end_frame(&mut dec);

        let after = dec.state();
        assert_eq!(after.rows, before.rows, "zero-byte frame must be noop");
        assert_eq!(after.cursor, before.cursor);
    }

    #[test]
    fn set_cursor_missing_row_arg_defaults_to_zero() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        // SET_CURSOR with col=5 but no row byte.
        push_frame(&mut dec, &[0x02, 5]);
        assert_eq!(dec.state().cursor, (5, 0));
    }

    #[test]
    fn set_cursor_out_of_range_args_clamp() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        push_frame(&mut dec, &[0x02, 25, 5]);
        // col clamps to 19 (LCD_COLS-1), row clamps to 1 (LCD_ROWS-1).
        assert_eq!(dec.state().cursor, (19, 1));
    }

    #[test]
    fn col_wrap_without_scroll_keeps_cursor_in_range() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        // CLEAR, SET_CURSOR (19, 0), WRITE "XY".
        push_frame(&mut dec, &[0x01]);
        push_frame(&mut dec, &[0x02, 19, 0]);
        push_frame(&mut dec, &[0x03, b'X', b'Y']);

        let state = dec.state();
        assert_eq!(state.rows[0][19], b'X');
        assert_eq!(state.rows[1][0], b'Y');
        // Cursor must be in-range (col<20, row<2) — specifically (1, 1).
        assert_eq!(state.cursor, (1, 1));
        assert!((state.cursor.0 as usize) < LCD_COLS);
        assert!((state.cursor.1 as usize) < LCD_ROWS);
    }

    #[test]
    fn cursor_never_out_of_range_after_single_row0_write() {
        // Regression for the transient (20, 1) cursor: after one write at
        // col=19, the cursor should already have wrapped to col=0 row=1.
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        push_frame(&mut dec, &[0x01]);
        push_frame(&mut dec, &[0x02, 19, 0]);
        push_frame(&mut dec, &[0x03, b'Z']);

        let state = dec.state();
        assert_eq!(state.rows[0][19], b'Z');
        assert_eq!(state.cursor, (0, 1));
        assert!((state.cursor.0 as usize) < LCD_COLS);
        assert!((state.cursor.1 as usize) < LCD_ROWS);
    }

    #[test]
    fn mid_byte_frame_abort_drops_partial_byte() {
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);
        // Prime the display with a known 'Q'.
        push_frame(&mut dec, &[0x03, b'Q']);
        let before = dec.state();

        // Start a new frame, clock in 4 bits (not a full byte), end the frame.
        start_frame(&mut dec);
        for i in 0..4 {
            let bit = (0b1010u8 >> (3 - i)) & 1 != 0;
            dec.sample(make_gpio(true, false, bit));
            dec.sample(make_gpio(true, true, bit));
            dec.sample(make_gpio(true, false, bit));
        }
        end_frame(&mut dec);

        let after = dec.state();
        // Partial byte is silently dropped because rx_buf has zero bytes
        // — apply_frame sees an empty buffer and returns early.
        assert_eq!(after.rows, before.rows);
        assert_eq!(after.cursor, before.cursor);
    }

    #[test]
    fn sclk_high_before_cs_falls_still_captures_bit7() {
        // Regression for the lost-bit-7 bug. Pre-fix: if the prior quantum
        // had SCLK=HIGH, and the firmware drives SCLK=HIGH across the
        // CS-falling quantum AND the next quantum (i.e. the first in-frame
        // SCLK=H sample), the decoder misses that first rising edge because
        // `prev_gpio`'s SCLK bit is still H from before CS fell. Bit 7 is
        // silently dropped and every subsequent bit is off-by-one.
        //
        // Post-fix: the CS-falling handler clears `prev_gpio`'s SCLK bit
        // at end-of-sample so the next quantum with SCLK=H registers as
        // a rising edge cleanly.
        //
        // Payload: [0x81, 0xAA, 0x55, 0x00]
        //   Post-fix: rx_buf = [0x81, 0xAA, 0x55, 0x00]. first=0x81 is an
        //             unknown opcode -> frame dropped -> cursor stays (0,0),
        //             rows blank.
        //   Pre-fix:  bit 7 of byte 0 lost. Decoder shifts in 7 bits of
        //             0x81 (0b0000001) + bit 7 of 0xAA (1) = 0b00000011
        //             = 0x03 = WRITE opcode. Subsequent rx_buf bytes are
        //             off-by-one: [0x03, 0x54 ('T'), 0xAA]. apply_frame
        //             runs WRITE, writes 'T' at (0,0) and 0xAA at (1,0),
        //             advances cursor to (2, 0). Test fails on both the
        //             rows[0][0]=='T' and cursor==(2,0) assertions.
        let mut dec = LcdDecoder::new(TEST_SCLK, TEST_DATA, TEST_CS);

        // Idle state with SCLK HIGH — simulate the hazard.
        dec.sample(make_gpio(false, true, false));
        // CS falls, SCLK still HIGH. This is the CS-falling quantum.
        dec.sample(make_gpio(true, true, false));

        // Bit 7 of byte 0 (0x81): bit=1. SCLK stays HIGH on the next
        // quantum. Post-fix: prev_gpio's SCLK is L, so this sample is a
        // rising edge. Pre-fix: prev_gpio's SCLK is H, so it's missed.
        dec.sample(make_gpio(true, true, true));
        // SCLK drops to L.
        dec.sample(make_gpio(true, false, true));

        // Bits 6..0 of 0x81 = 0000001. Normal cadence.
        for i in 1..8 {
            let bit = (0x81u8 >> (7 - i)) & 1 != 0;
            dec.sample(make_gpio(true, false, bit));
            dec.sample(make_gpio(true, true, bit));
            dec.sample(make_gpio(true, false, bit));
        }
        // Remaining bytes shifted normally.
        push_byte(&mut dec, 0xAA);
        push_byte(&mut dec, 0x55);
        push_byte(&mut dec, 0x00);
        end_frame(&mut dec);

        let state = dec.state();
        assert_eq!(
            state.rows[0][0], b' ',
            "bit 7 of first byte was lost: rows[0][0] = {:#x}, expected space",
            state.rows[0][0]
        );
        assert_eq!(
            state.cursor,
            (0, 0),
            "bit 7 of first byte was lost: unknown opcode (0x81) was misread"
        );
    }
}
