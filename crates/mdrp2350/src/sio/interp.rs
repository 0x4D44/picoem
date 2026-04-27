//! SIO Interpolator (RP2350 §2.7).
//!
//! Live arithmetic model for the interpolator unit, replacing the prior
//! dead-storage `[[u32; 32]; 2]` backing. Each core owns two `Interp`
//! instances (INTERP0 and INTERP1) — see `sio::Sio` for the wiring.
//!
//! Register map (datasheet §2.7, offsets relative to the interp base —
//! `0x80` for INTERP0, `0xC0` for INTERP1, inside the 4 KB SIO window):
//!
//! | Offset | Register    | Notes                                                     |
//! |--------|-------------|-----------------------------------------------------------|
//! | 0x00   | ACCUM0      | RW                                                        |
//! | 0x04   | ACCUM1      | RW                                                        |
//! | 0x08   | BASE0       | RW                                                        |
//! | 0x0C   | BASE1       | RW                                                        |
//! | 0x10   | BASE2       | RW                                                        |
//! | 0x14   | POP_LANE0   | RO — reads compute result **and** update ACCUM0/1         |
//! | 0x18   | POP_LANE1   | RO — as above                                             |
//! | 0x1C   | POP_FULL    | RO — as above                                             |
//! | 0x20   | PEEK_LANE0  | RO — computes result, no side effects                     |
//! | 0x24   | PEEK_LANE1  | RO — ditto                                                |
//! | 0x28   | PEEK_FULL   | RO — ditto                                                |
//! | 0x2C   | CTRL_LANE0  | RW — read returns CTRL + OVERF bits                       |
//! | 0x30   | CTRL_LANE1  | RW                                                        |
//! | 0x34   | ACCUM0_ADD  | W  — `ACCUM0 += value & 0xFFFFFF`                         |
//! | 0x38   | ACCUM1_ADD  | W  — `ACCUM1 += value & 0xFFFFFF`                         |
//! | 0x3C   | BASE_1AND0  | W  — writes low/high 16-bit halves to BASE0 / BASE1       |
//!
//! See HLD `wrk_docs/2026.04.17 - HLD - RP2350 Coverage Gap Fill V5.md`
//! §5 (Part B) for the implementation scope.

use super::super::peripherals::apply_alias_rmw;

/// Mask of bits owned by `CTRL_LANE` (config + OVERF flags occupy
/// bits [25:0]; bits [31:26] are reserved and read as 0).
const CTRL_BITS_MASK: u32 = 0x03FF_FFFF;
/// OVERF flags occupy bits [25:23] in `CTRL_LANE0` only (readable there).
const CTRL_OVERF_MASK: u32 = 0x0380_0000;
/// OVERF0 = bit 23, OVERF1 = bit 24, OVERF = bit 25.
const CTRL_OVERF0: u32 = 1 << 23;
const CTRL_OVERF1: u32 = 1 << 24;
const CTRL_OVERF: u32 = 1 << 25;

/// Interpolator state.
///
/// Two lanes (0, 1) plus three BASE registers. CTRL_LANE0 stores both
/// the lane-0 control bits and the sticky OVERF flags in bits [25:23].
/// CTRL_LANE1 holds only lane-1 control bits; its bits [25:23] read 0.
pub struct Interp {
    /// ACCUM0, ACCUM1.
    pub accum: [u32; 2],
    /// BASE0, BASE1, BASE2.
    pub base: [u32; 3],
    /// CTRL_LANE0, CTRL_LANE1. OVERF flags are stored inside CTRL_LANE0.
    pub ctrl_lane: [u32; 2],
}

// --- CTRL bit encodings (per §2.7) ---

const CTRL_SHIFT_SHIFT: u32 = 0;
const CTRL_SHIFT_MASK: u32 = 0x1F;
const CTRL_MASK_LSB_SHIFT: u32 = 5;
const CTRL_MASK_LSB_MASK: u32 = 0x1F;
const CTRL_MASK_MSB_SHIFT: u32 = 10;
const CTRL_MASK_MSB_MASK: u32 = 0x1F;
const CTRL_SIGNED: u32 = 1 << 15;
const CTRL_CROSS_INPUT: u32 = 1 << 16;
const CTRL_CROSS_RESULT: u32 = 1 << 17;
const CTRL_ADD_RAW: u32 = 1 << 18;
const CTRL_FORCE_MSB_SHIFT: u32 = 19;
const CTRL_FORCE_MSB_MASK: u32 = 0x3;
const CTRL_BLEND: u32 = 1 << 21;
const CTRL_CLAMP: u32 = 1 << 22;

impl Interp {
    /// Zero-initialised — all registers clear, no CTRL bits set.
    pub const fn new() -> Self {
        Self {
            accum: [0; 2],
            base: [0; 3],
            ctrl_lane: [0; 2],
        }
    }

    /// 32-bit read at `offset` (0x00..=0x3C, stride 4) — per the register
    /// map. POP_LANE* / POP_FULL have side effects on ACCUM0/1; PEEK
    /// variants are pure.
    pub fn read(&mut self, offset: u32, is_interp1: bool) -> u32 {
        match offset & 0x3F {
            0x00 => self.accum[0],
            0x04 => self.accum[1],
            0x08 => self.base[0],
            0x0C => self.base[1],
            0x10 => self.base[2],
            0x14 => self.pop_lane(0, is_interp1),
            0x18 => self.pop_lane(1, is_interp1),
            0x1C => {
                // POP_FULL — compute both lanes' results with side effects.
                // In BLEND mode (INTERP0 only) POP_FULL returns the blend.
                let r0 = self.pop_lane(0, is_interp1);
                let r1 = self.pop_lane(1, is_interp1);
                if !is_interp1 && (self.ctrl_lane[1] & CTRL_BLEND) != 0 {
                    self.blend_result()
                } else {
                    r0 | r1
                }
            }
            0x20 => self.compute_lane(0, is_interp1),
            0x24 => self.compute_lane(1, is_interp1),
            0x28 => {
                if !is_interp1 && (self.ctrl_lane[1] & CTRL_BLEND) != 0 {
                    self.blend_result()
                } else {
                    let r0 = self.compute_lane(0, is_interp1);
                    let r1 = self.compute_lane(1, is_interp1);
                    r0 | r1
                }
            }
            0x2C => self.ctrl_lane[0] & CTRL_BITS_MASK,
            0x30 => (self.ctrl_lane[1] & CTRL_BITS_MASK) & !CTRL_OVERF_MASK,
            // ACCUM*_ADD and BASE_1AND0 are W-only; reads return 0.
            0x34 | 0x38 | 0x3C => 0,
            _ => 0,
        }
    }

    /// 32-bit write at `offset`. `alias` is one of 0..=3 (see
    /// [`super::super::peripherals::apply_alias_rmw`]). SIO is in the
    /// 0xD bus region with no APB alias, so callers pass `alias = 0`;
    /// the parameter is preserved for symmetry with other peripherals.
    pub fn write(&mut self, offset: u32, value: u32, alias: u32) {
        match offset & 0x3F {
            0x00 => apply_alias_rmw(&mut self.accum[0], value, alias),
            0x04 => apply_alias_rmw(&mut self.accum[1], value, alias),
            0x08 => apply_alias_rmw(&mut self.base[0], value, alias),
            0x0C => apply_alias_rmw(&mut self.base[1], value, alias),
            0x10 => apply_alias_rmw(&mut self.base[2], value, alias),
            // POP_LANE*, POP_FULL, PEEK_LANE*, PEEK_FULL: writes ignored.
            0x14 | 0x18 | 0x1C | 0x20 | 0x24 | 0x28 => {}
            0x2C => {
                // CTRL_LANE0: bits [22:0] are CTRL config; bits [25:23] are
                // W1C OVERF flags. Apply the alias RMW to the whole 26-bit
                // view, then treat the OVERF field specially: writes of 1
                // to an OVERF bit clear it (W1C).
                //
                // Natural interpretation: a plain write sets CTRL bits
                // verbatim and clears any OVERF bit the caller wrote as 1.
                // XOR/SET/CLR aliases are rare on SIO (no APB here), so we
                // go with the plain-write semantics the datasheet
                // documents. The alias machinery is preserved for parity.
                let cur_ctrl = self.ctrl_lane[0] & !CTRL_OVERF_MASK;
                let cur_overf = self.ctrl_lane[0] & CTRL_OVERF_MASK;
                let mut new_ctrl = cur_ctrl;
                apply_alias_rmw(&mut new_ctrl, value & !CTRL_OVERF_MASK, alias);
                // W1C the OVERF bits against the incoming value.
                let new_overf = cur_overf & !(value & CTRL_OVERF_MASK);
                self.ctrl_lane[0] = (new_ctrl & !CTRL_OVERF_MASK) | new_overf;
            }
            0x30 => {
                // CTRL_LANE1 has no OVERF field; plain aliased store, but
                // also mask the OVERF region to 0 (it reads as 0).
                let mut new_ctrl = self.ctrl_lane[1] & !CTRL_OVERF_MASK;
                apply_alias_rmw(&mut new_ctrl, value & !CTRL_OVERF_MASK, alias);
                self.ctrl_lane[1] = new_ctrl & !CTRL_OVERF_MASK;
            }
            0x34 => {
                // ACCUM0_ADD: ACCUM0 += value & 0xFFFFFF (24-bit add).
                let inc = value & 0x00FF_FFFF;
                self.accum[0] = self.accum[0].wrapping_add(inc);
            }
            0x38 => {
                // ACCUM1_ADD.
                let inc = value & 0x00FF_FFFF;
                self.accum[1] = self.accum[1].wrapping_add(inc);
            }
            0x3C => {
                // BASE_1AND0: low 16b -> BASE0, high 16b -> BASE1.
                self.base[0] = value & 0x0000_FFFF;
                self.base[1] = (value >> 16) & 0x0000_FFFF;
            }
            _ => {}
        }
    }

    // --- Internals ---

    /// POP_LANE<n>: return the computed result, then update ACCUM per
    /// ADD_RAW semantics and latch any sticky OVERF bits.
    fn pop_lane(&mut self, lane: usize, is_interp1: bool) -> u32 {
        let result = self.compute_lane(lane, is_interp1);
        // Update both accumulators every POP (hardware writes back
        // base + (raw or shifted+masked) into BOTH ACCUM0 and ACCUM1).
        //
        // The datasheet describes this as: "After a POP, each accumulator
        // is updated by the addition of (BASE + lane result)." We follow
        // the most natural interpretation: both lanes' accumulators are
        // updated, each using its own CTRL's ADD_RAW bit and its own
        // arithmetic pipeline. The §9 datasheet checklist will confirm.
        let new_accum0 = self.accum_update(0, is_interp1);
        let new_accum1 = self.accum_update(1, is_interp1);
        // Collect overflow flags from each lane's shift_and_mask before
        // we commit the new accumulators.
        let (_, of0) = {
            let raw = self.lane_raw_input(0);
            self.shift_and_mask(0, raw, is_interp1)
        };
        let (_, of1) = {
            let raw = self.lane_raw_input(1);
            self.shift_and_mask(1, raw, is_interp1)
        };
        if of0 {
            self.ctrl_lane[0] |= CTRL_OVERF0 | CTRL_OVERF;
        }
        if of1 {
            self.ctrl_lane[0] |= CTRL_OVERF1 | CTRL_OVERF;
        }
        self.accum[0] = new_accum0;
        self.accum[1] = new_accum1;
        result
    }

    /// Compute the ACCUM update value for lane `n`: `BASE<n> + contrib`
    /// where `contrib` is either the raw lane input (ADD_RAW=1) or the
    /// shifted+masked value (ADD_RAW=0).
    fn accum_update(&self, lane: usize, is_interp1: bool) -> u32 {
        let ctrl = self.ctrl_lane[lane];
        let raw = self.lane_raw_input(lane);
        let contrib = if ctrl & CTRL_ADD_RAW != 0 {
            raw
        } else {
            self.shift_and_mask(lane, raw, is_interp1).0
        };
        self.base[lane].wrapping_add(contrib)
    }

    /// Which value feeds into the shift/mask pipeline for `lane`.
    ///
    /// Default is `ACCUM<lane>`; `CROSS_INPUT=1` swaps to the other
    /// lane's accumulator.
    fn lane_raw_input(&self, lane: usize) -> u32 {
        let ctrl = self.ctrl_lane[lane];
        if ctrl & CTRL_CROSS_INPUT != 0 {
            self.accum[1 - lane]
        } else {
            self.accum[lane]
        }
    }

    /// Apply SHIFT -> MASK -> SIGNED sign-extend. Returns the resulting
    /// value and a flag indicating a signed-overflow condition for the
    /// sticky OVERF bits. Overflow is defined as: if SIGNED=1, the
    /// unmasked-but-shifted value has bits set beyond MASK_MSB that
    /// would not agree with the sign bit of the masked result.
    fn shift_and_mask(&self, lane: usize, raw: u32, _is_interp1: bool) -> (u32, bool) {
        let ctrl = self.ctrl_lane[lane];
        let shift = (ctrl >> CTRL_SHIFT_SHIFT) & CTRL_SHIFT_MASK;
        let mask_lsb = (ctrl >> CTRL_MASK_LSB_SHIFT) & CTRL_MASK_LSB_MASK;
        let mask_msb = (ctrl >> CTRL_MASK_MSB_SHIFT) & CTRL_MASK_MSB_MASK;
        let signed = ctrl & CTRL_SIGNED != 0;

        // Shift right by `shift`.
        let shifted = raw >> shift;

        // Build mask `[mask_lsb..=mask_msb]` (inclusive both ends).
        // When mask_msb < mask_lsb the field is empty — result is 0.
        let mask = if mask_msb < mask_lsb {
            0u32
        } else if mask_msb == 31 {
            // Avoid `1u32 << 32` UB.
            (!0u32) << mask_lsb
        } else {
            let width = mask_msb - mask_lsb + 1;
            ((1u32 << width) - 1) << mask_lsb
        };
        let masked = shifted & mask;

        // Signed sign-extension from MASK_MSB.
        let (value, overflow) = if signed && mask_msb < 31 {
            let sign_bit = 1u32 << mask_msb;
            if masked & sign_bit != 0 {
                // Sign-extend upwards.
                let ext = (!0u32) << mask_msb;
                (masked | ext, shifted & !mask != 0)
            } else {
                // Non-negative; overflow if any pre-mask bit above the
                // sign bit is set (would have been lost to the mask).
                let above = shifted & !mask;
                (masked, above != 0)
            }
        } else {
            (masked, false)
        };

        (value, overflow)
    }

    /// Force the top two bits of `v` to the 2-bit FORCE_MSB field.
    fn apply_force_msb(&self, lane: usize, v: u32) -> u32 {
        let ctrl = self.ctrl_lane[lane];
        let force = (ctrl >> CTRL_FORCE_MSB_SHIFT) & CTRL_FORCE_MSB_MASK;
        if force == 0 {
            v
        } else {
            (v & 0x3FFF_FFFF) | (force << 30)
        }
    }

    /// Compute the arithmetic result for a single lane, honouring
    /// CROSS_RESULT / FORCE_MSB / CLAMP (INTERP1 only). No side effects.
    ///
    /// `is_interp1` selects the CLAMP (INTERP1-only) and BLEND
    /// (INTERP0-only) branches.
    fn compute_lane(&self, lane: usize, is_interp1: bool) -> u32 {
        // BLEND on INTERP0 lane 0 returns the blended integer value.
        if !is_interp1 && lane == 0 && (self.ctrl_lane[1] & CTRL_BLEND) != 0 {
            return self.blend_result();
        }

        // CROSS_RESULT: this lane's result comes from the OTHER lane's
        // arithmetic (computed recursively, but cross-result is
        // applied once — the other lane's CROSS_RESULT bit is ignored
        // to avoid an infinite swap).
        let source_lane = if self.ctrl_lane[lane] & CTRL_CROSS_RESULT != 0 {
            1 - lane
        } else {
            lane
        };

        let raw = self.lane_raw_input(source_lane);
        let (sm, _overflow) = self.shift_and_mask(source_lane, raw, is_interp1);

        // CLAMP on INTERP1 lane 0: the lane-0 output is the shifted+masked
        // accumulator clamped to [BASE0, BASE1]. The BASE0 add is skipped
        // in this mode — BASE0 is repurposed as the clamp lower limit.
        //
        // Ambiguity resolved (HLD §5.B.3 is silent on the BASE0-add path
        // under CLAMP): the HLD §5.B.4 Test 7 expects ACCUM0=50 clamped
        // to 100 with BASE0=100. That is only consistent with CLAMP
        // bypassing the BASE0 add and using BASE0 purely as the lower
        // limit, which is also the de facto RP2040 firmware usage.
        // §9 datasheet check will pin the precise semantics.
        if is_interp1 && lane == 0 && (self.ctrl_lane[0] & CTRL_CLAMP) != 0 {
            let lo = self.base[0];
            let hi = self.base[1];
            let vi = sm as i32;
            let li = lo as i32;
            let hi_i = hi as i32;
            let clamped = if vi < li {
                lo
            } else if vi > hi_i {
                hi
            } else {
                sm
            };
            return self.apply_force_msb(source_lane, clamped);
        }

        let mut v = self.base[source_lane].wrapping_add(sm);
        v = self.apply_force_msb(source_lane, v);
        v
    }

    /// INTERP0 BLEND result: `BASE0 + ((BASE1 - BASE0) * alpha)`
    /// where alpha = ACCUM1 interpreted as Q0.32 (`0x8000_0000` = 0.5).
    /// Operates on signed 32-bit BASE0/BASE1.
    fn blend_result(&self) -> u32 {
        let base0 = self.base[0] as i32 as i64;
        let base1 = self.base[1] as i32 as i64;
        let alpha = self.accum[1] as u64; // Q0.32 unsigned
        let diff = base1 - base0;
        // diff * alpha >> 32. Use i128 to hold all cases.
        let scaled = ((diff as i128) * (alpha as i128)) >> 32;
        let result = base0 as i128 + scaled;
        (result as i32) as u32
    }
}

impl Default for Interp {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Reset-state + storage round-trip ---

    #[test]
    fn reset_state_is_zero() {
        let mut interp = Interp::new();
        assert_eq!(interp.accum, [0, 0]);
        assert_eq!(interp.base, [0, 0, 0]);
        assert_eq!(interp.ctrl_lane, [0, 0]);
        // PEEK reads on a freshly-constructed interp return 0.
        assert_eq!(interp.read(0x20, false), 0);
        assert_eq!(interp.read(0x24, false), 0);
        assert_eq!(interp.read(0x28, false), 0);
    }

    #[test]
    fn accum_base_ctrl_round_trip() {
        let mut interp = Interp::new();
        interp.write(0x00, 0xDEAD_BEEF, 0);
        interp.write(0x04, 0xCAFE_BABE, 0);
        interp.write(0x08, 0x1111_1111, 0);
        interp.write(0x0C, 0x2222_2222, 0);
        interp.write(0x10, 0x3333_3333, 0);
        interp.write(0x2C, 0x000F_FFFF, 0); // CTRL_LANE0 — CTRL bits only
        interp.write(0x30, 0x0000_FFFF, 0); // CTRL_LANE1
        assert_eq!(interp.read(0x00, false), 0xDEAD_BEEF);
        assert_eq!(interp.read(0x04, false), 0xCAFE_BABE);
        assert_eq!(interp.read(0x08, false), 0x1111_1111);
        assert_eq!(interp.read(0x0C, false), 0x2222_2222);
        assert_eq!(interp.read(0x10, false), 0x3333_3333);
        assert_eq!(interp.read(0x2C, false), 0x000F_FFFF);
        assert_eq!(interp.read(0x30, false), 0x0000_FFFF);
    }

    #[test]
    fn accum_add_24bit_increment() {
        let mut interp = Interp::new();
        interp.accum[0] = 0x0000_1000;
        interp.write(0x34, 0x00FF_FF01, 0); // low 24 bits only
        assert_eq!(interp.accum[0], 0x0100_0F01);
        // Write a value with bits above [23:0] — only low 24 bits add.
        interp.accum[1] = 0;
        interp.write(0x38, 0xFFFF_FF01, 0);
        assert_eq!(interp.accum[1], 0x00FF_FF01);
    }

    #[test]
    fn base_1and0_splits_halves() {
        let mut interp = Interp::new();
        interp.write(0x3C, 0xABCD_1234, 0);
        assert_eq!(interp.base[0], 0x0000_1234);
        assert_eq!(interp.base[1], 0x0000_ABCD);
    }

    // --- Eight named test vectors from HLD §5.B.4 ---

    /// Test 1: SHIFT only. ACCUM0 = 0xAAAA_AAAA, SHIFT=4, MASK=[0..=31]
    /// -> POP_LANE0 = 0x0AAA_AAAA.
    #[test]
    fn test1_shift_only() {
        let mut interp = Interp::new();
        interp.accum[0] = 0xAAAA_AAAA;
        // CTRL_LANE0: SHIFT=4, MASK_LSB=0, MASK_MSB=31
        let ctrl = 4u32 | (0 << 5) | (31 << 10);
        interp.write(0x2C, ctrl, 0);
        assert_eq!(interp.read(0x14, false), 0x0AAA_AAAA);
    }

    /// Test 2: SHIFT + MASK. ACCUM0 = 0xAAAA_AAAA, SHIFT=4, MASK=[0..=7]
    /// -> POP_LANE0 = 0xAA.
    #[test]
    fn test2_shift_and_mask() {
        let mut interp = Interp::new();
        interp.accum[0] = 0xAAAA_AAAA;
        let ctrl = 4u32 | (0 << 5) | (7 << 10);
        interp.write(0x2C, ctrl, 0);
        assert_eq!(interp.read(0x14, false), 0xAA);
    }

    /// Test 3: SIGNED sign-extension. ACCUM0 = 0x0000_0080, MASK_MSB=7,
    /// SIGNED=1 -> POP_LANE0 = 0xFFFF_FF80.
    #[test]
    fn test3_signed_sign_extension() {
        let mut interp = Interp::new();
        interp.accum[0] = 0x0000_0080;
        // SHIFT=0, MASK_LSB=0, MASK_MSB=7, SIGNED=1.
        let ctrl = 0u32 | (0 << 5) | (7 << 10) | (1 << 15);
        interp.write(0x2C, ctrl, 0);
        assert_eq!(interp.read(0x14, false), 0xFFFF_FF80);
    }

    /// Test 4: CROSS_INPUT. Lane 1 raw input = ACCUM0, not ACCUM1.
    #[test]
    fn test4_cross_input() {
        let mut interp = Interp::new();
        interp.accum[0] = 0x1234_5678;
        interp.accum[1] = 0xDEAD_BEEF; // should be ignored by lane 1
        // CTRL_LANE1: CROSS_INPUT=1, SHIFT=0, MASK_LSB=0, MASK_MSB=31
        let ctrl = 0u32 | (0 << 5) | (31 << 10) | (1 << 16);
        interp.write(0x30, ctrl, 0);
        // PEEK_LANE1 should read from ACCUM0.
        assert_eq!(interp.read(0x24, false), 0x1234_5678);
    }

    /// Test 5: ADD_RAW on POP. POP_LANE0 updates ACCUM0 with BASE0 + raw
    /// (not BASE0 + shifted+masked).
    #[test]
    fn test5_add_raw_on_pop() {
        let mut interp = Interp::new();
        interp.accum[0] = 0x0000_0100;
        interp.base[0] = 0x0000_0010;
        // SHIFT=4, MASK_LSB=0, MASK_MSB=31, ADD_RAW=1.
        let ctrl = 4u32 | (0 << 5) | (31 << 10) | (1 << 18);
        interp.write(0x2C, ctrl, 0);
        // Pre-POP: raw=0x100, shifted=0x10, shifted+masked=0x10.
        // POP's returned value: BASE0 + (shifted+masked) = 0x10 + 0x10 = 0x20.
        let popped = interp.read(0x14, false);
        assert_eq!(popped, 0x0000_0020);
        // Post-POP ACCUM0: BASE0 + raw = 0x10 + 0x100 = 0x110 (ADD_RAW).
        assert_eq!(interp.accum[0], 0x0000_0110);
    }

    /// Test 6: INTERP0 BLEND. BASE0=100, BASE1=200, ACCUM1=0x8000_0000
    /// -> PEEK_FULL = 150.
    #[test]
    fn test6_interp0_blend() {
        let mut interp = Interp::new();
        interp.base[0] = 100;
        interp.base[1] = 200;
        interp.accum[1] = 0x8000_0000; // Q0.32 alpha = 0.5
        // CTRL_LANE1: BLEND=1 (bit 21).
        interp.write(0x30, 1 << 21, 0);
        assert_eq!(interp.read(0x28, false), 150);
    }

    /// Test 7: INTERP1 CLAMP. BASE0=100, BASE1=200, ACCUM0=50 -> PEEK_LANE0
    /// clamped to 100 (lower limit); ACCUM0=250 -> clamped to 200 (upper);
    /// ACCUM0=150 passes through unchanged. CLAMP bypasses the BASE0 add
    /// on lane 0 — BASE0 is repurposed as the clamp lower limit (see the
    /// ambiguity note in `compute_lane`).
    #[test]
    fn test7_interp1_clamp() {
        let mut interp = Interp::new();
        interp.base[0] = 100;
        interp.base[1] = 200;
        // CTRL_LANE0: CLAMP=1 (bit 22), MASK_LSB=0, MASK_MSB=31.
        interp.write(0x2C, (31u32 << 10) | (1 << 22), 0);
        // ACCUM0 = 50 -> clamped up to 100.
        interp.accum[0] = 50;
        assert_eq!(interp.read(0x20, true), 100);
        // ACCUM0 = 250 -> clamped down to 200.
        interp.accum[0] = 250;
        assert_eq!(interp.read(0x20, true), 200);
        // ACCUM0 = 150 -> inside range, passes through.
        interp.accum[0] = 150;
        assert_eq!(interp.read(0x20, true), 150);
    }

    /// Test 8: CROSS_RESULT. Lane 0 CTRL has CROSS_RESULT=1; PEEK_LANE0
    /// returns lane 1's arithmetic.
    #[test]
    fn test8_cross_result() {
        let mut interp = Interp::new();
        // Lane 0 normally reads ACCUM0 = 0xAAAA.
        interp.accum[0] = 0xAAAA;
        interp.accum[1] = 0xBBBB;
        interp.base[0] = 0;
        interp.base[1] = 0;
        // CTRL_LANE0: CROSS_RESULT=1 (bit 17). Everything else minimal.
        interp.write(0x2C, (31u32 << 10) | (1 << 17), 0);
        // CTRL_LANE1: SHIFT=0, MASK=[0..=31].
        interp.write(0x30, 31u32 << 10, 0);
        // PEEK_LANE0 should return lane1 arithmetic = BASE1 + ACCUM1 = 0xBBBB.
        assert_eq!(interp.read(0x20, false), 0xBBBB);
    }

    // --- POP vs PEEK side-effect distinction ---

    #[test]
    fn peek_is_pure() {
        let mut interp = Interp::new();
        interp.accum[0] = 0x1234;
        interp.accum[1] = 0x5678;
        interp.base[0] = 1;
        interp.base[1] = 2;
        // Default CTRL (all zero) => SHIFT=0, MASK=[0..=0] -> masked to 1 bit.
        let a0_before = interp.accum[0];
        let a1_before = interp.accum[1];
        let _ = interp.read(0x20, false);
        let _ = interp.read(0x24, false);
        let _ = interp.read(0x28, false);
        assert_eq!(interp.accum[0], a0_before);
        assert_eq!(interp.accum[1], a1_before);
    }

    #[test]
    fn pop_updates_accum() {
        let mut interp = Interp::new();
        interp.accum[0] = 0x10;
        interp.accum[1] = 0x20;
        interp.base[0] = 1;
        interp.base[1] = 2;
        // CTRL with MASK=[0..=31] so arithmetic is predictable.
        interp.write(0x2C, 31u32 << 10, 0);
        interp.write(0x30, 31u32 << 10, 0);
        // POP_LANE0: new ACCUM0 = BASE0 + shifted+masked lane 0 input
        //          = 1 + 0x10 = 0x11; new ACCUM1 = 2 + 0x20 = 0x22.
        interp.read(0x14, false);
        assert_eq!(interp.accum[0], 0x11);
        assert_eq!(interp.accum[1], 0x22);
    }

    // --- CTRL_LANE OVERF bits ---

    #[test]
    fn ctrl_lane1_masks_overf_field_to_zero() {
        let mut interp = Interp::new();
        // Writing OVERF bits to CTRL_LANE1 should not stick.
        interp.write(0x30, 0x03FF_FFFF, 0);
        let read = interp.read(0x30, false);
        assert_eq!(
            read & 0x0380_0000,
            0,
            "CTRL_LANE1 OVERF region must read as 0"
        );
    }

    #[test]
    fn signed_overflow_latches_overf_flags() {
        let mut interp = Interp::new();
        // Lane 0: SHIFT=0, MASK_LSB=0, MASK_MSB=7, SIGNED=1.
        // ACCUM0 = 0x0000_FF00 — bit 7 clear, but bits above mask set.
        // After mask, result = 0x00; SIGNED sees "non-negative with bits
        // above mask" — that triggers overflow in our model.
        interp.write(0x2C, (7u32 << 10) | (1 << 15), 0);
        interp.accum[0] = 0x0000_FF00;
        // POP_LANE0 commits overflow flags.
        let _ = interp.read(0x14, false);
        let ctrl0 = interp.read(0x2C, false);
        assert_ne!(ctrl0 & CTRL_OVERF0, 0, "OVERF0 must be sticky-set");
        assert_ne!(ctrl0 & CTRL_OVERF, 0, "OVERF aggregate must be set");
    }

    #[test]
    fn ctrl_lane0_overf_w1c() {
        let mut interp = Interp::new();
        // Seed OVERF0 and OVERF1 directly.
        interp.ctrl_lane[0] |= CTRL_OVERF0 | CTRL_OVERF1;
        assert_eq!(
            interp.read(0x2C, false) & CTRL_OVERF_MASK,
            CTRL_OVERF0 | CTRL_OVERF1
        );
        // Write 1 to OVERF0 only — clears it, leaves OVERF1.
        interp.write(0x2C, CTRL_OVERF0, 0);
        assert_eq!(interp.read(0x2C, false) & CTRL_OVERF_MASK, CTRL_OVERF1);
    }

    // --- ACCUM0_ADD / ACCUM1_ADD / BASE_1AND0 are W-only ---

    #[test]
    fn wonly_regs_read_zero() {
        let mut interp = Interp::new();
        // Write, then read — reads must return 0, not the stored value.
        interp.write(0x34, 0xDEAD_BEEF, 0);
        interp.write(0x38, 0xDEAD_BEEF, 0);
        interp.write(0x3C, 0xDEAD_BEEF, 0);
        assert_eq!(interp.read(0x34, false), 0);
        assert_eq!(interp.read(0x38, false), 0);
        assert_eq!(interp.read(0x3C, false), 0);
    }
}
