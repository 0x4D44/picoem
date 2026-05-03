use super::{CoreBus, CortexM33, Fault};
use crate::sio::Sio;

impl CortexM33 {
    /// Top-level coprocessor dispatch.
    pub(crate) fn thumb32_coprocessor<B: CoreBus>(
        &mut self,
        hw0: u16,
        hw1: u16,
        bus: &mut B,
    ) -> u32 {
        let coproc = ((hw1 >> 8) & 0xF) as u8;

        // Check CPACR (2 bits per coprocessor). Phase 0b.1 Commit B:
        // per-core PPB (including CPACR) now lives on `self.ppb`.
        let cpacr = self.ppb.cpacr;
        let access = (cpacr >> (coproc as u32 * 2)) & 0x3;
        if access == 0 {
            self.pending_fault = Some(Fault::UsageFault);
            return 0;
        }

        match coproc {
            0 => self.cp0_gpioc(hw0, hw1, bus),
            4 | 5 => self.cp4_5_dcp(hw0, hw1),
            7 => self.cp7_rcp(hw0, hw1, bus),
            10 | 11 => self.fpu_execute(hw0, hw1, bus),
            _ => {
                self.pending_fault = Some(Fault::UsageFault);
                0
            }
        }
    }

    /// CP0 (GPIOC): GPIO coprocessor — SDK-emitted ops wired to SIO fast-path
    /// and `Bus.gpio_in`. See `cp0_mcr_mrc_family` for the encoding table.
    fn cp0_gpioc<B: CoreBus>(&mut self, hw0: u16, hw1: u16, bus: &mut B) -> u32 {
        let is_mrc_mcr = (hw0 >> 12) & 0xF == 0xE && hw1 & (1 << 4) != 0;
        if is_mrc_mcr {
            self.cp0_mcr_mrc_family(hw0, hw1, bus)
        } else {
            1 // CDP not used by CP0 — silent NOP, match existing stub style.
        }
    }

    /// CP0 MCR/MRC dispatch — matches HLD §C.1 / Pico SDK `hardware_gpio.h`.
    ///
    /// Thumb-32 MCR/MRC encoding:
    /// - hw0 = `1110 1110 opc1[3] L CRn[4]` (L=0 MCR, L=1 MRC)
    /// - hw1 = `Rt[4] coproc[4] opc2[3] 1 CRm[4]`
    ///
    /// The SDK uses `opc1` to select the bank (OUT/OE/IN × LO/HI) and then
    /// discriminates **bulk vs per-bit** by the `(CRn, CRm)` pair:
    ///   - `(CRn=0, CRm=0)` → bulk bank op; op2 selects get/put/set/clr/xor.
    ///   - otherwise → per-bit op on `pin = (CRn<<4)|CRm`; op2 selects the op.
    ///
    /// Bank mapping (RP2354A is 30-pin, HI bank is RAZ/WI):
    ///
    /// | opc1 | Bank              |
    /// |------|-------------------|
    /// |  0   | LO OUT (GPIO_OUT, pins 0..29)  |
    /// |  1   | LO OE  (GPIO_OE,  pins 0..29)  |
    /// |  2   | LO IN  (GPIO_IN,  pins 0..29)  |
    /// |  4   | HI OUT (pins 30..47 — RAZ/WI)  |
    /// |  5   | HI OE  (pins 30..47 — RAZ/WI)  |
    /// |  6   | HI IN  (pins 30..47 — RAZ)     |
    ///
    /// Per-bit op2 selection (when CRn or CRm is non-zero):
    /// `op2=0` → `_get` (MRC), `op2=4` → `_put` (MCR Rt[0]),
    /// `op2=5` → `_set`, `op2=6` → `_clr`, `op2=7` → `_xor`.
    ///
    /// Bulk op2 selection (when CRn=0 and CRm=0):
    /// MRC `op2=0` → `_get`. MCR `op2=0` → `_put`, `op2=1` → `_set`,
    /// `op2=2` → `_clr`, `op2=3` → `_xor`.
    ///
    /// Examples matching HLD §C.1:
    ///   `gpioc_lo_out_get()` = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
    ///   `gpioc_hi_out_get()` = MRC CP0, opc1=4, CRn=0, CRm=0, op2=0.
    ///   `gpioc_bit_out_get(pin)` = MRC CP0, opc1=0, CRn=pin_hi, CRm=pin_lo, op2=0.
    ///
    /// Note: pin 0 has (CRn=0, CRm=0), which collides with the bulk encoding.
    /// Per HLD, pin 0 per-bit ops are unreachable by this scheme; firmware
    /// uses the bulk mask path for pin 0. Matches Pico SDK behavior.
    ///
    /// Undefined op2 on MRC reads as 0; undefined op2 on MCR is silent NOP.
    /// Cycle cost: 1.
    fn cp0_mcr_mrc_family<B: CoreBus>(&mut self, hw0: u16, hw1: u16, bus: &mut B) -> u32 {
        let is_mrc = (hw0 >> 4) & 1 != 0; // L bit
        let opc1 = ((hw0 >> 5) & 0x7) as u8;
        let crn = (hw0 & 0xF) as u8;
        let crm = (hw1 & 0xF) as u8;
        let op2 = ((hw1 >> 5) & 0x7) as u8;
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let is_bulk = crn == 0 && crm == 0;

        match opc1 {
            // ---- LO banks (pins 0..29) ----
            0 => self.cp0_lo_out(bus, is_mrc, is_bulk, crn, crm, op2, rt),
            1 => self.cp0_lo_oe(bus, is_mrc, is_bulk, crn, crm, op2, rt),
            2 => self.cp0_lo_in(bus, is_mrc, is_bulk, crn, crm, rt),
            // ---- HI banks (pins 30..47) — RP2354A has no pins here.
            // Reads RAZ, writes WI. Preserve any Rt value on MRC by writing 0;
            // no SIO mutation on MCR. The is_mrc=false case (i.e. MCR) falls
            // through to the silent-NOP catch-all.
            4..=6 if is_mrc => {
                self.regs.r[rt] = 0;
            }
            _ => {} // unknown opc1 / MCR to HI bank -> silent NOP
        }
        1
    }

    /// LO OUT bank (opc1=0): bulk lo_out when (CRn=0,CRm=0), else per-bit on pin.
    fn cp0_lo_out<B: CoreBus>(
        &mut self,
        bus: &mut B,
        is_mrc: bool,
        is_bulk: bool,
        crn: u8,
        crm: u8,
        op2: u8,
        rt: usize,
    ) {
        if is_bulk {
            if is_mrc {
                // op2=0 is the documented get; other op2 treated as NOP read 0.
                self.regs.r[rt] = if op2 == 0 {
                    bus.gpio_read_out() & Sio::PIN_MASK
                } else {
                    0
                };
            } else {
                let v = self.regs.r[rt] & Sio::PIN_MASK;
                match op2 {
                    0 => bus.gpio_write_out(v),
                    1 => bus.gpio_set_out(v),
                    2 => bus.gpio_clear_out(v),
                    3 => bus.gpio_xor_out(v),
                    _ => {}
                }
            }
        } else {
            let pin = (crn << 4) | crm;
            if is_mrc {
                // Per-bit read: pin >= 30 masks to 0 (parity with
                // `Sio::gpio_bit_out_get`).
                let v = if op2 == 0 && pin < 30 {
                    (bus.gpio_read_out() >> pin) & 1 != 0
                } else {
                    false
                };
                self.regs.r[rt] = v as u32;
            } else if pin < 30 {
                let mask = 1u32 << pin;
                match op2 {
                    4 => {
                        if self.regs.r[rt] & 1 != 0 {
                            bus.gpio_set_out(mask);
                        } else {
                            bus.gpio_clear_out(mask);
                        }
                    }
                    5 => bus.gpio_set_out(mask),
                    6 => bus.gpio_clear_out(mask),
                    7 => bus.gpio_xor_out(mask),
                    _ => {}
                }
            }
        }
    }

    /// LO OE bank (opc1=1): bulk lo_oe when (CRn=0,CRm=0), else per-bit on pin.
    fn cp0_lo_oe<B: CoreBus>(
        &mut self,
        bus: &mut B,
        is_mrc: bool,
        is_bulk: bool,
        crn: u8,
        crm: u8,
        op2: u8,
        rt: usize,
    ) {
        if is_bulk {
            if is_mrc {
                self.regs.r[rt] = if op2 == 0 {
                    bus.gpio_read_oe() & Sio::PIN_MASK
                } else {
                    0
                };
            } else {
                let v = self.regs.r[rt] & Sio::PIN_MASK;
                match op2 {
                    0 => bus.gpio_write_oe(v),
                    1 => bus.gpio_set_oe(v),
                    2 => bus.gpio_clear_oe(v),
                    3 => bus.gpio_xor_oe(v),
                    _ => {}
                }
            }
        } else {
            let pin = (crn << 4) | crm;
            if is_mrc {
                let v = if op2 == 0 && pin < 30 {
                    (bus.gpio_read_oe() >> pin) & 1 != 0
                } else {
                    false
                };
                self.regs.r[rt] = v as u32;
            } else if pin < 30 {
                let mask = 1u32 << pin;
                match op2 {
                    4 => {
                        if self.regs.r[rt] & 1 != 0 {
                            bus.gpio_set_oe(mask);
                        } else {
                            bus.gpio_clear_oe(mask);
                        }
                    }
                    5 => bus.gpio_set_oe(mask),
                    6 => bus.gpio_clear_oe(mask),
                    7 => bus.gpio_xor_oe(mask),
                    _ => {}
                }
            }
        }
    }

    /// LO IN bank (opc1=2, read-only): bulk lo_in_get when (CRn=0,CRm=0),
    /// else per-bit in on pin. Source is `bus.gpio_read_in()`. MCR is a silent NOP.
    fn cp0_lo_in<B: CoreBus>(
        &mut self,
        bus: &mut B,
        is_mrc: bool,
        is_bulk: bool,
        crn: u8,
        crm: u8,
        rt: usize,
    ) {
        if !is_mrc {
            return; // writes to the input bank are undefined -> silent NOP.
        }
        if is_bulk {
            self.regs.r[rt] = bus.gpio_read_in() & Sio::PIN_MASK;
        } else {
            let pin = (crn << 4) | crm;
            self.regs.r[rt] = if pin < 30 {
                (bus.gpio_read_in() >> pin) & 1
            } else {
                0
            };
        }
    }

    /// CP4/5 (DCP): Double-precision coprocessor (Phase 7 Stage D).
    ///
    /// Dispatch splits into two families, paralleling the CP7 layout:
    ///   - `dcp_transfer_family`: MCR/MRC moving 32-bit halves between ARM
    ///     registers and the DCP half-register file (`wxma/wxmb/rfma/rfmb`).
    ///   - `dcp_cdp_family`: CDP/CDP2 for arithmetic, compare, convert,
    ///     and status-register access.
    ///
    /// The encoding table is intentionally internally consistent (not
    /// datasheet-verified — the datasheet §3.6.7 encodings are not
    /// enumerated in repo). See the test module's "DCP encoding lock-in"
    /// section for the full table.
    fn cp4_5_dcp(&mut self, hw0: u16, hw1: u16) -> u32 {
        let is_mrc_mcr = (hw0 >> 12) & 0xF == 0xE && hw1 & (1 << 4) != 0;
        if is_mrc_mcr {
            self.dcp_transfer_family(hw0, hw1)
        } else {
            self.dcp_cdp_family(hw0, hw1)
        }
    }

    /// DCP MCR/MRC — `wxma/wxmb` (ARM → half) and `rfma/rfmb` (half → ARM).
    ///
    /// Encoding:
    ///   hw0 = `1110 1110 opc1[3] L CRn[4]`, hw1 = `Rt[4] coproc[4] opc2[3] 1 CRm[4]`
    /// Fields:
    ///   L        = MCR/MRC discriminator (0 = MCR, 1 = MRC)
    ///   opc1     = 0 (transfer family; other opc1 values silent-NOP)
    ///   opc2     = 0 → half A (low word), 1 → half B (high word)
    ///   CRm[2:0] = double index (0..7); CRm[3] ignored
    ///   CRn      = unused (must be 0)
    ///   Rt       = ARM register
    ///
    /// Cycle cost: 1 (per HLD §12).
    fn dcp_transfer_family(&mut self, hw0: u16, hw1: u16) -> u32 {
        let is_mrc = (hw0 >> 4) & 1 != 0;
        let opc1 = ((hw0 >> 5) & 0x7) as u8;
        let opc2 = ((hw1 >> 5) & 0x7) as u8;
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let crm = (hw1 & 0xF) as u8;

        if opc1 != 0 {
            // Reserved opc1 for transfer family — silent NOP.
            return 1;
        }
        debug_assert!(
            crm & 0x8 == 0,
            "DCP transfer CRm[3] must be 0; got {:x}",
            crm
        );
        let double_idx = (crm as usize) & 0x7;
        let half_idx = double_idx * 2 + (opc2 as usize & 1);

        if is_mrc {
            // rfma / rfmb — DCP half → ARM register
            self.regs.r[rt] = self.dcp_halves[half_idx];
        } else {
            // wxma / wxmb — ARM register → DCP half
            self.dcp_halves[half_idx] = self.regs.r[rt];
        }
        1
    }

    /// DCP CDP/CDP2 — arithmetic, compare, convert, status access.
    ///
    /// Encoding:
    ///   hw0 = `1110 1110 opc1[4] CRn[4]`, hw1 = `CRd[4] coproc[4] opc2[3] 0 CRm[4]`
    ///   (CDP2 form uses 0xFE prefix; dispatch here treats both identically.)
    /// Fields:
    ///   opc1 = op class (see table below)
    ///   opc2 = op subcode / compare predicate
    ///   CRd  = destination double index (for arithmetic / convert)
    ///   CRn  = source double #1
    ///   CRm  = source double #2 (for binary ops) or source #1 for unary
    ///
    /// | opc1 | opc2 | Mnemonic   | Semantics                           | Cycles |
    /// |------|------|------------|-------------------------------------|--------|
    /// | 0    | 0    | dadd       | d[Rd] = d[Rn] + d[Rm], set status   | 4      |
    /// | 0    | 1    | dsub       | d[Rd] = d[Rn] - d[Rm], set status   | 4      |
    /// | 0    | 2    | dmul       | d[Rd] = d[Rn] * d[Rm], set status   | 5      |
    /// | 0    | 3    | ddiv       | d[Rd] = d[Rn] / d[Rm], set status   | 18     |
    /// | 0    | 4    | dsqrt      | d[Rd] = sqrt(d[Rn]), set status     | 28     |
    /// | 1    | 0..4 | dcmp_*     | status bit 0 = predicate(d[Rn],d[Rm])| 4     |
    /// | 2    | 0    | i2d        | d[Rd] = (f64) i32(half_a(d[Rn]))    | 4      |
    /// | 2    | 1    | u2d        | d[Rd] = (f64) u32(half_a(d[Rn]))    | 4      |
    /// | 2    | 2    | d2i        | half_a(d[Rd]) = d[Rn] as i32        | 4      |
    /// | 2    | 3    | d2u        | half_a(d[Rd]) = d[Rn] as u32        | 4      |
    /// | 2    | 4    | d2f        | half_a(d[Rd]) = d[Rn] as f32        | 4      |
    /// | 2    | 5    | f2d        | d[Rd] = (f64) f32(half_a(d[Rn]))    | 4      |
    /// | 3    | 0    | dcpstat_get| half_a(d[Rd]) = dcp_status          | 1      |
    /// | 3    | 1    | dcpstat_clr| dcp_status = 0                      | 1      |
    ///
    /// Compare predicates (opc1=1):
    ///   opc2=0 → dcmp_eq,  opc2=1 → dcmp_lt,  opc2=2 → dcmp_le,
    ///   opc2=3 → dcmp_gt,  opc2=4 → dcmp_ge.
    ///
    /// Unrecognized (opc1, opc2) combinations silent-NOP (cycle cost 1).
    fn dcp_cdp_family(&mut self, hw0: u16, hw1: u16) -> u32 {
        let opc1 = ((hw0 >> 4) & 0xF) as u8;
        let opc2 = ((hw1 >> 5) & 0x7) as u8;
        let crd = ((hw1 >> 12) & 0xF) as usize & 0x7;
        let crn = (hw0 & 0xF) as usize & 0x7;
        let crm = (hw1 & 0xF) as usize & 0x7;

        match opc1 {
            0 => match opc2 {
                0 => {
                    let a = self.dcp_read_double(crn);
                    let b = self.dcp_read_double(crm);
                    let r = a + b;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    4
                }
                1 => {
                    let a = self.dcp_read_double(crn);
                    let b = self.dcp_read_double(crm);
                    let r = a - b;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    4
                }
                2 => {
                    let a = self.dcp_read_double(crn);
                    let b = self.dcp_read_double(crm);
                    let r = a * b;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    5
                }
                3 => {
                    let a = self.dcp_read_double(crn);
                    let b = self.dcp_read_double(crm);
                    let r = a / b;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    18
                }
                4 => {
                    let a = self.dcp_read_double(crn);
                    let r = a.sqrt();
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    28
                }
                _ => 1, // Reserved opc2 under opc1=0 — silent NOP.
            },
            1 => {
                let a = self.dcp_read_double(crn);
                let b = self.dcp_read_double(crm);
                // Note: any compare with a NaN operand is false per IEEE-754,
                // EXCEPT dcmp_ne-style inequalities — we do not expose ne.
                // Rust's native f64 comparison operators already implement
                // the IEEE quiet-NaN-compares-false rule.
                let pass = match opc2 {
                    0 => a == b, // eq
                    1 => a < b,  // lt
                    2 => a <= b, // le
                    3 => a > b,  // gt
                    4 => a >= b, // ge
                    _ => {
                        return 1; // Unknown compare — silent NOP, no status write.
                    }
                };
                // Compares overwrite the entire status register with {1, 0},
                // not just bit 0. This differs from the arith path which sets
                // all 4 bits based on result — compares only care about the
                // predicate outcome.
                self.dcp_status = if pass { 1 } else { 0 };
                4
            }
            2 => match opc2 {
                0 => {
                    // i2d: half A of CRn carries an i32.
                    let i = self.dcp_halves[crn * 2] as i32;
                    let r = i as f64;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    4
                }
                1 => {
                    // u2d: half A of CRn carries a u32.
                    let u = self.dcp_halves[crn * 2];
                    let r = u as f64;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    4
                }
                2 => {
                    // d2i: saturating per `as i32` (Rust defines this since 1.45).
                    let d = self.dcp_read_double(crn);
                    self.dcp_halves[crd * 2] = d as i32 as u32;
                    self.dcp_halves[crd * 2 + 1] = 0;
                    // Status reflects the produced integer as if re-cast;
                    // datasheet §3.6.7 doesn't spec status here. We set
                    // based on the pre-cast f64 result to match arithmetic.
                    self.dcp_set_arith_status(d);
                    4
                }
                3 => {
                    // d2u: saturating.
                    let d = self.dcp_read_double(crn);
                    self.dcp_halves[crd * 2] = d as u32;
                    self.dcp_halves[crd * 2 + 1] = 0;
                    self.dcp_set_arith_status(d);
                    4
                }
                4 => {
                    // d2f: f64 → f32, stored in half A; half B cleared.
                    let d = self.dcp_read_double(crn);
                    let f = d as f32;
                    self.dcp_halves[crd * 2] = f.to_bits();
                    self.dcp_halves[crd * 2 + 1] = 0;
                    // Use the f32 as f64 to set flags on the visible result.
                    self.dcp_set_arith_status(f as f64);
                    4
                }
                5 => {
                    // f2d: f32 in half A of CRn → f64 in CRd.
                    let f = f32::from_bits(self.dcp_halves[crn * 2]);
                    let r = f as f64;
                    self.dcp_write_double(crd, r);
                    self.dcp_set_arith_status(r);
                    4
                }
                _ => 1,
            },
            3 => match opc2 {
                0 => {
                    // dcpstat_get: status → half A of CRd; half B cleared.
                    self.dcp_halves[crd * 2] = self.dcp_status;
                    self.dcp_halves[crd * 2 + 1] = 0;
                    1
                }
                1 => {
                    // dcpstat_clr: zero the status register.
                    self.dcp_status = 0;
                    1
                }
                _ => 1,
            },
            _ => 1, // Unrecognized opc1 — silent NOP.
        }
    }

    /// Pack two 32-bit halves back into an f64. Half A is the low 32 bits;
    /// half B is the high 32 bits (little-endian layout).
    #[inline]
    fn dcp_read_double(&self, idx: usize) -> f64 {
        let lo = self.dcp_halves[idx * 2] as u64;
        let hi = self.dcp_halves[idx * 2 + 1] as u64;
        f64::from_bits((hi << 32) | lo)
    }

    /// Split an f64 into two 32-bit halves and store at index `idx`.
    #[inline]
    fn dcp_write_double(&mut self, idx: usize, v: f64) {
        let bits = v.to_bits();
        self.dcp_halves[idx * 2] = bits as u32;
        self.dcp_halves[idx * 2 + 1] = (bits >> 32) as u32;
    }

    /// Set `dcp_status` flags from an arithmetic result.
    ///
    /// Bit 0 — zero    (including ±0)
    /// Bit 1 — negative (sign bit set; includes -0 and -NaN)
    /// Bit 2 — infinity
    /// Bit 3 — NaN
    ///
    /// All other bits are cleared; the status register is not sticky.
    #[inline]
    fn dcp_set_arith_status(&mut self, r: f64) {
        let mut s = 0u32;
        if r == 0.0 {
            s |= 1 << 0;
        }
        if r.is_sign_negative() {
            s |= 1 << 1;
        }
        if r.is_infinite() {
            s |= 1 << 2;
        }
        if r.is_nan() {
            s |= 1 << 3;
        }
        self.dcp_status = s;
    }

    /// CP7 (RCP): Redundancy coprocessor. Dispatches MCR/MRC (0xEE/0xFE)
    /// and MCRR/MRRC (0xEC/0xFC) encoding families.
    fn cp7_rcp<B: CoreBus>(&mut self, hw0: u16, hw1: u16, bus: &mut B) -> u32 {
        let hw0_high = (hw0 >> 8) & 0xFF;
        match hw0_high {
            0xEE | 0xFE => self.cp7_mcr_mrc_family(hw0, hw1, bus),
            0xEC | 0xFC => self.cp7_mcrr_mrrc_family(hw0, hw1, bus),
            _ => 1, // Not a recognized CP7 encoding
        }
    }

    /// CP7 MCR/MRC/CDP family dispatch (Phase 7 Stage E).
    ///
    /// Discriminates on bit 4 of hw1 (1 = MCR/MRC, 0 = CDP), L bit of hw0
    /// (0 = to-coproc / MCR, 1 = from-coproc / MRC), and then (opc1, opc2)
    /// to reach the specific RCP instruction. Encoding table at the test
    /// module's top — all patterns marked "bootrom" are verified against
    /// `roms/rp2350/arm-bootrom.dis`.
    ///
    /// On assertion failure: `self.pending_fault = Some(Fault::Nmi)`.
    /// The existing `step()`/`deliver_fault` path turns that into an NMI
    /// exception (`enter_exception(2, bus)`).
    fn cp7_mcr_mrc_family<B: CoreBus>(&mut self, hw0: u16, hw1: u16, _bus: &mut B) -> u32 {
        let is_cdp = hw1 & (1 << 4) == 0;
        if is_cdp {
            return self.cp7_cdp(hw0, hw1);
        }
        let is_mrc = (hw0 >> 4) & 1 != 0; // L bit
        let opc1 = ((hw0 >> 5) & 0x7) as u8;
        let crn = (hw0 & 0xF) as u8;
        let opc2 = ((hw1 >> 5) & 0x7) as u8;
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let crm = (hw1 & 0xF) as u8;
        let core = self.core_id as usize;

        if is_mrc {
            match (opc1, opc2) {
                (0, 1) => {
                    // rcp_canary_get Rt, imm — returns salt ^ 0xDEADBEEF.
                    // The immediate (CRn<<4)|CRm is a "tag" for SDK
                    // bookkeeping; we ignore it (bootrom pairs get/check
                    // with the same tag and relies only on consistency).
                    self.regs.r[rt] = self.atomics.rcp_salt_load(core) ^ 0xDEAD_BEEF;
                }
                (1, 0) if rt == 15 => {
                    // rcp_canary_status pc: write NZCV to APSR.
                    // N = salt_valid[core]; Z=0, C=0, V=0.
                    let n = if self.atomics.rcp_salt_is_valid(core) {
                        1u32 << 31
                    } else {
                        0
                    };
                    self.regs.xpsr = (self.regs.xpsr & 0x0FFF_FFFF) | n;
                }
                _ => {} // unrecognized MRC: silent NOP
            }
            return 1;
        }

        // MCR path — assertions (may raise Fault::Nmi).
        match (opc1, opc2) {
            (0, 1) => {
                // rcp_canary_check Rt, imm — assert Rt == salt ^ 0xDEADBEEF.
                //
                // Salt-invalid divergence from silicon (HLD §8.4 skip list):
                // when `rcp_salt_valid[core] == false`, both sides of the
                // comparison compute `0 ^ 0xDEADBEEF` and the check passes.
                // Real silicon raises NMI on any canary op while salt is
                // unseeded. We preserve the divergence so the bootrom can
                // execute its own salt-seeding path — which contains
                // canary_get/check pairs that would otherwise trip before
                // the salt is written — and continue to boot through.
                let expected = self.atomics.rcp_salt_load(core) ^ 0xDEAD_BEEF;
                if self.regs.r[rt] != expected {
                    self.pending_fault = Some(Fault::Nmi);
                }
            }
            (1, 0) => {
                // rcp_bvalid Rt — assert Rt ∈ {0, 1}.
                let v = self.regs.r[rt];
                if v > 1 {
                    self.pending_fault = Some(Fault::Nmi);
                }
            }
            // rcp_btrue Rt — assert Rt == 1.
            (2, 0) if self.regs.r[rt] != 1 => {
                self.pending_fault = Some(Fault::Nmi);
            }
            // rcp_bfalse Rt — assert Rt == 0.
            (3, 1) if self.regs.r[rt] != 0 => {
                self.pending_fault = Some(Fault::Nmi);
            }
            (4, 0) => {
                // rcp_count_init imm — set the redundancy counter to imm.
                self.atomics
                    .rcp_count_set(((crn as u32) << 4) | (crm as u32));
            }
            (5, 1) => {
                // rcp_count_check imm — assert counter == imm, then increment.
                let expected = ((crn as u32) << 4) | (crm as u32);
                if self.atomics.rcp_count_check(expected).is_err() {
                    self.pending_fault = Some(Fault::Nmi);
                }
            }
            _ => {
                // Unrecognized MCR encoding — silent NOP (HLD §8.4).
                // Notably `rcp_ifgte` (opc1=6, opc2=0) and `rcp_iflte`
                // (opc1=6, opc2=1) are *NOT implemented — silent NOP*; no
                // caller has materialized in the bootrom disassembly and we
                // refuse to speculate the encoding.
            }
        }
        1
    }

    /// CP7 CDP / CDP2 dispatch. One mnemonic currently handled:
    ///   - `rcp_panic` (opc1=0, opc2=1): unconditional NMI.
    ///
    /// Other CDP encodings — notably the speculative `rcp_switch`
    /// (opc1=0, opc2=2) — are *NOT implemented — silent NOP*. They do not
    /// appear in the bootrom disassembly; per HLD §8.4 we refuse to commit
    /// to an encoding until a real caller demands it.
    fn cp7_cdp(&mut self, hw0: u16, hw1: u16) -> u32 {
        let opc1 = ((hw0 >> 4) & 0xF) as u8;
        let opc2 = ((hw1 >> 5) & 0x7) as u8;
        // unrecognized CDP: silent NOP (HLD §8.4)
        if (opc1, opc2) == (0, 1) {
            // rcp_panic — unconditional NMI (bootrom encoding 0xEE00_0720).
            self.pending_fault = Some(Fault::Nmi);
        }
        1
    }

    fn cp7_mcrr_mrrc_family<B: CoreBus>(&mut self, hw0: u16, hw1: u16, _bus: &mut B) -> u32 {
        let l_bit = (hw0 >> 4) & 1;
        if l_bit != 0 {
            return 1; // MRRC2 from CP7: not used by bootrom
        }

        // MCRR2 dispatch: discriminator is opc1 in hw1[7:4]; CRm in hw1[3:0].
        // Rt in hw1[15:12], Rt2 in hw0[3:0].
        let opc1 = ((hw1 >> 4) & 0xF) as u8;
        let crm = (hw1 & 0xF) as u8;
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let rt2 = (hw0 & 0xF) as usize;

        match opc1 {
            // rcp_iequal Rt, Rt2 — assert Rt == Rt2 (bootrom 0xFC4x_x770).
            // Equal case (assertion holds) falls through to the catch-all.
            7 if self.regs.r[rt] != self.regs.r[rt2] => {
                self.pending_fault = Some(Fault::Nmi);
            }
            8 => {
                match crm {
                    0 => {
                        // rcp_salt_core0
                        self.atomics.rcp_salt_set(0, self.regs.r[rt]);
                    }
                    1 => {
                        // rcp_salt_core1
                        self.atomics.rcp_salt_set(1, self.regs.r[rt]);
                    }
                    _ => {} // unrecognized salt CRm: silent NOP
                }
            }
            _ => {
                // rcp_b2valid, rcp_bxortrue, rcp_bxorfalse, rcp_ivalid:
                // bootrom uses these sparingly; silent NOP matches existing
                // stub behavior (HLD §8.4 skip list).
            }
        }
        1
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::Ordering;

    use crate::bus::Bus;
    use crate::core::CortexM33;
    use crate::core::Fault;

    /// Encode MCR: ARM Rt -> CPn  (hw0[4]=0 means MCR)
    /// hw0 = `1110 1110 opc1[3] L CRn[4]`
    /// hw1 = `Rt[4] coproc[4] opc2[3] 1 CRm[4]`
    fn encode_mcr(coproc: u8, rt: u8, crm: u8) -> (u16, u16) {
        encode_mcr_full(coproc, 0, 0, rt, 0, crm)
    }

    /// Encode MRC: CPn -> ARM Rt  (hw0[4]=1 means MRC)
    fn encode_mrc(coproc: u8, rt: u8, crm: u8) -> (u16, u16) {
        encode_mrc_full(coproc, 0, 0, rt, 0, crm)
    }

    /// Full MCR encoder (opc1, CRn, Rt, op2, CRm).
    fn encode_mcr_full(coproc: u8, opc1: u8, crn: u8, rt: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE00
            | ((opc1 as u16 & 0x7) << 5)
            // L bit = 0 for MCR
            | (crn as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | 0x10
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// Full MRC encoder.
    fn encode_mrc_full(coproc: u8, opc1: u8, crn: u8, rt: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE00
            | ((opc1 as u16 & 0x7) << 5)
            | 0x10 // L bit = 1 for MRC
            | (crn as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | 0x10
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// Split a pin number into (CRn=pin_hi, CRm=pin_lo).
    fn pin_split(pin: u8) -> (u8, u8) {
        ((pin >> 4) & 0xF, pin & 0xF)
    }

    /// Set CPACR to enable a given coprocessor (full access = 0b11).
    /// Phase 0b.1 Commit B: per-core PPB (including CPACR) lives on
    /// `CortexM33.ppb`, so the helper targets a CPU rather than the Bus.
    fn enable_cp(cpu: &mut CortexM33, coproc: u8) {
        cpu.ppb.cpacr |= 0x3 << (coproc as u32 * 2);
    }

    #[test]
    fn test_cp7_rcp_salt_roundtrip() {
        // Phase 3 Stage 2: share atomics so rcp_canary_get reads the
        // bus-side salt.
        let atomics = std::sync::Arc::new(crate::threaded::CoreAtomics::default());
        let mut cpu = CortexM33::new(0, std::sync::Arc::clone(&atomics));
        let mut bus = Bus::with_atomics(atomics);
        enable_cp(&mut cpu, 7);

        // Poke salt directly via atomics (Phase 3 Stage 1 migration).
        bus.atomics.rcp_salt_set(0, 42);

        // rcp_canary_get Rt=1, imm=0 — Phase 7 Stage E encoding
        // (MRC2 cp7, opc1=0, opc2=1, CRn=0, CRm=0).
        let (hw0, hw1) = encode_mrc2_full(7, 0, 0, 1, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cpu.regs.r[1], 42 ^ 0xDEAD_BEEF);
    }

    #[test]
    fn test_cp4_5_dcp_transfer() {
        // Rewritten for Phase 7 Stage D (16-half register file).
        // wxma (MCR opc2=0, CRm=0) writes ARM Rt into half A of double 0;
        // rfma (MRC opc2=0, CRm=0) reads it back. Roundtrip locks in the
        // canonical transfer encoding.
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        let val: u32 = 0xCAFE_BABE;
        cpu.regs.r[2] = val;

        // wxma: R2 -> half A of double 0 (opc2=0, CRm=0).
        let (hw0, hw1) = encode_mcr_full(4, 0, 0, 2, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_halves[0], val);

        // rfma: half A of double 0 -> R3.
        let (hw0, hw1) = encode_mrc_full(4, 0, 0, 3, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[3], val);
    }

    #[test]
    fn test_cpacr_blocks_disabled_cp() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        // CPACR defaults to 0 — all coprocessors disabled

        let (hw0, hw1) = encode_mrc(7, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(matches!(cpu.pending_fault, Some(Fault::UsageFault)));
    }

    #[test]
    fn test_cpacr_allows_enabled_cp() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 7);

        // MCR with CP7 enabled — should not fault (MCR is a silent NOP now)
        cpu.regs.r[0] = 42;
        let (hw0, hw1) = encode_mcr(7, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(cpu.pending_fault.is_none());
    }

    // ---- CP0 GPIOC tests (Phase 7 Stage C) ----

    /// Baseline cycle-count assertion — replaces the old "returns 0" stub test
    /// per HLD §10 test impact table. New contract: CP0 MRC reads actual SIO
    /// state, and returns cycle count 1.
    #[test]
    fn test_cp0_gpioc_read_matches_sio() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        // Seed SIO gpio_out via direct field access, then read via CP0.
        bus.sio.gpio_out = 0x12345678 & 0x3FFF_FFFF;
        // HLD §C.1: gpioc_lo_out_get = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mrc_full(0, 0, 0, 3, 0, 0); // lo_out_get -> r3
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.regs.r[3], bus.sio.gpio_out);
        assert!(cpu.pending_fault.is_none());
    }

    // --- Per-bit GPIO_OUT ops ---

    #[test]
    fn test_cp0_bit_out_set() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        let pin = 5u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 1 << pin);
    }

    #[test]
    fn test_cp0_bit_out_clr() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);
        bus.sio.gpio_out = 0x0000_00FF;

        let pin = 3u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 6, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0xF7);
    }

    #[test]
    fn test_cp0_bit_out_xor() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);
        bus.sio.gpio_out = 0b1010;

        let pin = 1u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 7, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0b1000);
    }

    #[test]
    fn test_cp0_bit_out_put() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        let pin = 7u8;
        let (crn, crm) = pin_split(pin);

        // Put 1 into pin 7.
        cpu.regs.r[2] = 1;
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 2, 4, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 1 << pin);

        // Put 0 into pin 7.
        cpu.regs.r[2] = 0;
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 2, 4, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0);
    }

    #[test]
    fn test_cp0_bit_out_get() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        bus.sio.gpio_out = 1 << 9 | 1 << 15;

        // Read pin 9 -> 1.
        let (crn, crm) = pin_split(9);
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 4, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[4], 1);

        // Read pin 10 -> 0.
        let (crn, crm) = pin_split(10);
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 5, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[5], 0);
    }

    // --- Per-bit GPIO_OE ops ---

    #[test]
    fn test_cp0_bit_oe_set_clr_xor_put_get() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        let pin = 12u8;
        let (crn, crm) = pin_split(pin);

        // set
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 1 << pin);

        // get -> r6 = 1
        let (hw0, hw1) = encode_mrc_full(0, 1, crn, 6, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[6], 1);

        // xor -> clears
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 0, 7, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 0);

        // put 1
        cpu.regs.r[7] = 1;
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 7, 4, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 1 << pin);

        // clr
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 0, 6, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 0);
    }

    // --- Bulk GPIO_OUT ops ---

    #[test]
    fn test_cp0_lo_out_put_then_get() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        cpu.regs.r[1] = 0x1234_5678;
        // HLD §C.1: lo_out_put = MCR CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        // Value masked to 30 pins.
        assert_eq!(bus.sio.gpio_out, 0x1234_5678 & 0x3FFF_FFFF);

        // Read back: lo_out_get = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mrc_full(0, 0, 0, 2, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[2], 0x1234_5678 & 0x3FFF_FFFF);
    }

    #[test]
    fn test_cp0_lo_out_set_clr_xor() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        bus.sio.gpio_out = 0x0000_F000;

        // set 0x0F00: opc1=0, CRn=0, CRm=0, op2=1.
        cpu.regs.r[1] = 0x0F00;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0xFF00);

        // clr 0x00F0 (nothing to clear) — gpio_out unchanged. op2=2.
        cpu.regs.r[1] = 0x00F0;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 2, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0xFF00);

        // xor 0xF000 — flips high bits. op2=3.
        cpu.regs.r[1] = 0xF000;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 3, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0x0F00);
    }

    #[test]
    fn test_cp0_lo_oe_bulk() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        cpu.regs.r[0] = 0xDEAD_BEEF;
        // HLD §C.1: lo_oe_put = MCR CP0, opc1=1 (OE bank), CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mcr_full(0, 1, 0, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 0xDEAD_BEEF & 0x3FFF_FFFF);

        let (hw0, hw1) = encode_mrc_full(0, 1, 0, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[1], 0xDEAD_BEEF & 0x3FFF_FFFF);
    }

    // --- Input reads ---

    #[test]
    fn test_cp0_lo_in_get() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        bus.gpio_in.store(0xA5A5_A5A5, Ordering::Relaxed);
        // opc1=2 (IN bank), CRn=0, CRm=0 -> lo_in_get, MRC into r8.
        let (hw0, hw1) = encode_mrc_full(0, 2, 0, 8, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[8], 0xA5A5_A5A5 & 0x3FFF_FFFF);
    }

    #[test]
    fn test_cp0_bit_in_get() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        bus.gpio_in.store(0xA5A5_A5A5, Ordering::Relaxed);
        // 0xA5 = 1010 0101: bit 2 = 1, bit 3 = 0, bit 5 = 1.
        // Pin 0 is unreachable per-bit under the HLD encoding (CRn=0,CRm=0
        // is the bulk slot), so exercise pins 2 and 3 instead.
        let (crn, crm) = pin_split(2);
        let (hw0, hw1) = encode_mrc_full(0, 2, crn, 9, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[9], 1);

        let (crn, crm) = pin_split(3);
        let (hw0, hw1) = encode_mrc_full(0, 2, crn, 10, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[10], 0);
    }

    // --- Pin >= 30 masking ---

    #[test]
    fn test_cp0_bit_set_pin_30_masked() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        // Pin 30 is out of range on RP2354A (30 pins: 0..29). Write must be masked.
        let pin = 30u8;
        let (crn, crm) = pin_split(pin);
        let before = bus.sio.gpio_out;
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, before);

        // Read pin 30 -> returns 0 regardless of underlying bit.
        bus.sio.gpio_out = 0xFFFF_FFFF;
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 11, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[11], 0);
    }

    #[test]
    fn test_cp0_lo_out_put_masks_upper_bits() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        // Write a value with bits [31:30] set; expect them masked to 0.
        // HLD §C.1: lo_out_put = MCR CP0, opc1=0, CRn=0, CRm=0, op2=0.
        cpu.regs.r[0] = 0xFFFF_FFFF;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0x3FFF_FFFF);
    }

    // --- Discrimination: CP0 and SIO MMIO observe the same state ---

    #[test]
    fn test_cp0_write_observed_via_mmio() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        // Set pin 12 via CP0.
        let pin = 12u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        // Read GPIO_OUT through the SIO MMIO path at 0xD000_0010.
        let mmio_val = bus.read32(0xD000_0010, 0);
        assert_eq!(mmio_val, 1 << pin);

        // Conversely: write via MMIO GPIO_OUT_SET (0xD000_0018) and read via CP0.
        bus.write32(0xD000_0018, 1 << 20, 0);
        let (crn, crm) = pin_split(20);
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 0, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[0], 1);
    }

    // --- CPACR disabled -> UsageFault ---

    #[test]
    fn test_cp0_cpacr_disabled_faults() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        // CPACR defaults to 0 — CP0 disabled.

        // Encode `bit_out_set(pin=5)`: opc1=0, CRn=0, CRm=5, op2=5, MCR.
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 0, 5, 5);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(matches!(cpu.pending_fault, Some(Fault::UsageFault)));
        // Devil's-advocate follow-up: confirm the write was suppressed, not just
        // that a fault was raised. gpio_out must remain zero.
        assert_eq!(bus.sio.gpio_out, 0);
    }

    // --- HLD §C.1 compliance lock-in ---

    /// Regression guard for the encoding-bug fix (HLD §C.1 / SDK compliance).
    ///
    /// `gpioc_lo_out_get()` is `MRC CP0, opc1=0, CRn=0, CRm=0, op2=0` — the
    /// same opc1 as `gpioc_bit_out_get(pin)`. A prior implementation routed
    /// opc1=0 unconditionally through the per-bit path, which would make
    /// real-SDK firmware silently read only pin 0 of the bank. This test
    /// seeds a multi-bit pattern and asserts the read returns the full
    /// 30-bit bank, locking out the regression.
    #[test]
    fn test_cp0_hld_lo_out_get_reads_full_bank() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 0);

        // Set a pattern with bit 0 = 0 and other bits = 1 — catches a
        // per-bit-pin-0 regression (which would read 0, not 0x3FFF_FFFE).
        let pattern: u32 = 0x3FFF_FFFE;
        bus.sio.gpio_out = pattern;

        // HLD §C.1: gpioc_lo_out_get = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mrc_full(0, 0, 0, 7, 0, 0);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cycles, 1);
        assert_eq!(
            cpu.regs.r[7], pattern,
            "lo_out_get (opc1=0, CRn=0, CRm=0) must read the full 30-bit bank"
        );
        assert!(cpu.pending_fault.is_none());
    }

    // ============================================================
    // Phase 7 Stage E — CP7 RCP assertions (NMI on mismatch)
    // ============================================================
    //
    // Encoding lookup table — derived from the RP2350 ARM bootrom disassembly
    // (`roms/rp2350/arm-bootrom.dis`) wherever a real instance was observed, and
    // chosen for internal consistency for the few mnemonics the bootrom does
    // not exercise (rcp_ifgte, rcp_iflte, rcp_switch).
    //
    // | Mnemonic           | Form  | opc1 | opc2 | CRn          | CRm          | Notes |
    // |--------------------|-------|------|------|--------------|--------------|-------|
    // | rcp_canary_get     | MRC2  | 0    | 1    | imm[7:4]     | imm[3:0]     | bootrom |
    // | rcp_canary_check   | MCR2  | 0    | 1    | imm[7:4]     | imm[3:0]     | bootrom |
    // | rcp_canary_status  | MRC2  | 1    | 0    | 0            | 0            | bootrom (Rt=15) |
    // | rcp_bvalid         | MCR2  | 1    | 0    | 0            | 0            | bootrom |
    // | rcp_btrue          | MCR2  | 2    | 0    | 0            | 0            | bootrom |
    // | rcp_bfalse         | MCR2  | 3    | 1    | 0            | 0            | bootrom |
    // | rcp_count_init     | MCR2  | 4    | 0    | imm[7:4]     | imm[3:0]     | bootrom (`count_set`) |
    // | rcp_count_check    | MCR2  | 5    | 1    | imm[7:4]     | imm[3:0]     | bootrom |
    // | rcp_ifgte          | MCR2  | 6    | 0    | 0            | 0            | *NOT implemented — silent NOP* |
    // | rcp_iflte          | MCR2  | 6    | 1    | 0            | 0            | *NOT implemented — silent NOP* |
    // | rcp_panic          | CDP   | 0    | 1    | 0            | 0            | bootrom |
    // | rcp_switch         | CDP   | 0    | 2    | 0            | 0            | *NOT implemented — silent NOP* |
    // | rcp_iequal         | MCRR2 | 7    | —    | (Rt2 in hw0) | 0            | bootrom |
    // | rcp_salt_core0/1   | MCRR2 | 8    | —    | (Rt2 in hw0) | 0/1          | bootrom (existing) |

    /// MCR2 encoder (L=0). hw0 prefix 0xFE.. distinguishes from MCR (0xEE..).
    /// hw0 = `1111_1110_opc1[3]_L_CRn[4]`; hw1 = `Rt[4]_coproc[4]_opc2[3]_1_CRm[4]`.
    fn encode_mcr2_full(coproc: u8, opc1: u8, crn: u8, rt: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xFE00 | ((opc1 as u16 & 0x7) << 5) | (crn as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | 0x10
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// MRC2 encoder (L=1).
    fn encode_mrc2_full(coproc: u8, opc1: u8, crn: u8, rt: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xFE00 | ((opc1 as u16 & 0x7) << 5) | 0x10 | (crn as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | 0x10
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// MCRR2 encoder. hw0 = `1111_1100_0100_Rt2[4]`;
    /// hw1 = `Rt[4]_coproc[4]_opc1[4]_CRm[4]`.
    fn encode_mcrr2(coproc: u8, opc1: u8, rt2: u8, rt: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xFC40 | (rt2 as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((opc1 as u16 & 0xF) << 4)
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// CDP2 encoder. hw0 = `1111_1110_opc1[4]_CRn[4]`;
    /// hw1 = `CRd[4]_coproc[4]_opc2[3]_0_CRm[4]`.
    fn encode_cdp2(coproc: u8, opc1: u8, crn: u8, crd: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xFE00 | ((opc1 as u16 & 0xF) << 4) | (crn as u16 & 0xF);
        let hw1: u16 = ((crd as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | (crm as u16 & 0xF); // bit 4 = 0 → CDP
        (hw0, hw1)
    }

    /// CDP (non-2 variant). hw0 prefix 0xEE..
    fn encode_cdp(coproc: u8, opc1: u8, crn: u8, crd: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE00 | ((opc1 as u16 & 0xF) << 4) | (crn as u16 & 0xF);
        let hw1: u16 = ((crd as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    fn split_imm8(imm: u8) -> (u8, u8) {
        ((imm >> 4) & 0xF, imm & 0xF)
    }

    /// Convenience: prepare a CPU + Bus with CP7 enabled, salt set, salt valid.
    fn rcp_setup() -> (CortexM33, Bus) {
        // Phase 3 Stage 2: share atomics so coprocessor reads via
        // `self.atomics` see the test's bus-side salt/counter setup.
        let atomics = std::sync::Arc::new(crate::threaded::CoreAtomics::default());
        let mut cpu = CortexM33::new(0, std::sync::Arc::clone(&atomics));
        let bus = Bus::with_atomics(atomics);
        enable_cp(&mut cpu, 7);
        bus.atomics.rcp_salt_set(0, 0x1234_5678);
        (cpu, bus)
    }

    // ---------- rcp_canary_check (MCR2 form) ----------

    #[test]
    fn test_rcp_canary_check_pass() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[2] = 0x1234_5678 ^ 0xDEAD_BEEF;
        let (crn, crm) = split_imm8(0x6c);
        let (hw0, hw1) = encode_mcr2_full(7, 0, crn, 2, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(
            cpu.pending_fault.is_none(),
            "matching canary must not raise fault"
        );
    }

    #[test]
    fn test_rcp_canary_check_fail_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[2] = 0xBADD_CAFE; // not equal to salt^0xDEADBEEF
        let (crn, crm) = split_imm8(0x6c);
        let (hw0, hw1) = encode_mcr2_full(7, 0, crn, 2, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    // ---------- rcp_btrue / rcp_bfalse ----------

    #[test]
    fn test_rcp_btrue_pass() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[0] = 1;
        let (hw0, hw1) = encode_mcr2_full(7, 2, 0, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
    }

    #[test]
    fn test_rcp_btrue_fail_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[0] = 0;
        let (hw0, hw1) = encode_mcr2_full(7, 2, 0, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    #[test]
    fn test_rcp_bfalse_pass() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[3] = 0;
        let (hw0, hw1) = encode_mcr2_full(7, 3, 0, 3, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
    }

    #[test]
    fn test_rcp_bfalse_fail_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[3] = 1;
        let (hw0, hw1) = encode_mcr2_full(7, 3, 0, 3, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    // ---------- rcp_bvalid ----------

    #[test]
    fn test_rcp_bvalid_pass_zero() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[5] = 0;
        let (hw0, hw1) = encode_mcr2_full(7, 1, 0, 5, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
    }

    #[test]
    fn test_rcp_bvalid_pass_one() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[5] = 1;
        let (hw0, hw1) = encode_mcr2_full(7, 1, 0, 5, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
    }

    #[test]
    fn test_rcp_bvalid_fail_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[5] = 2;
        let (hw0, hw1) = encode_mcr2_full(7, 1, 0, 5, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    // ---------- rcp_count_init / rcp_count_check ----------

    #[test]
    fn test_rcp_count_init_then_check_pass() {
        let (mut cpu, mut bus) = rcp_setup();
        // count_init 0xc0
        let (crn, crm) = split_imm8(0xc0);
        let (hw0, hw1) = encode_mcr2_full(7, 4, crn, 0, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.atomics.rcp_count_load(), 0xc0);
        assert!(cpu.pending_fault.is_none());

        // count_check 0xc0 -> pass, increments to 0xc1
        let (hw0, hw1) = encode_mcr2_full(7, 5, crn, 0, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
        assert_eq!(bus.atomics.rcp_count_load(), 0xc1);

        // count_check 0xc1 -> pass, increments to 0xc2
        let (crn, crm) = split_imm8(0xc1);
        let (hw0, hw1) = encode_mcr2_full(7, 5, crn, 0, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
        assert_eq!(bus.atomics.rcp_count_load(), 0xc2);
    }

    #[test]
    fn test_rcp_count_check_fail_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        let (crn, crm) = split_imm8(0x40);
        let (hw0, hw1) = encode_mcr2_full(7, 4, crn, 0, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());

        // count_check 0x99 — wrong, must NMI
        let (crn, crm) = split_imm8(0x99);
        let (hw0, hw1) = encode_mcr2_full(7, 5, crn, 0, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    // ---------- rcp_panic (CDP form) ----------

    #[test]
    fn test_rcp_panic_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        // CDP cp7, opc1=0, opc2=1, all others zero — the bootrom encoding
        // (verified: 0xEE00_0720).
        let (hw0, hw1) = encode_cdp(7, 0, 0, 0, 1, 0);
        assert_eq!(
            (hw0, hw1),
            (0xEE00, 0x0720),
            "encoding must match bootrom rcp_panic"
        );
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    // ---------- rcp_iequal (MCRR2 form) ----------

    #[test]
    fn test_rcp_iequal_pass() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[2] = 0xCAFE_BABE;
        cpu.regs.r[3] = 0xCAFE_BABE;
        let (hw0, hw1) = encode_mcrr2(7, 7, 3, 2, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
    }

    #[test]
    fn test_rcp_iequal_fail_raises_nmi() {
        let (mut cpu, mut bus) = rcp_setup();
        cpu.regs.r[2] = 0xCAFE_BABE;
        cpu.regs.r[3] = 0xDEAD_BEEF;
        let (hw0, hw1) = encode_mcrr2(7, 7, 3, 2, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    // ---------- Unimplemented encodings: silent NOP (HLD §8.4) ----------

    /// Lock in the "no speculative encodings" policy (HLD §2, CLAUDE.md
    /// "don't predict the future"): `rcp_ifgte`, `rcp_iflte`, and
    /// `rcp_switch` do NOT appear in the bootrom disassembly and have no
    /// real callers. They must silent-NOP — no fault raised, no state
    /// change — so that if a real caller ever materializes the failure is
    /// loud and points at this test rather than at mysteriously-passing
    /// speculative semantics.
    #[test]
    fn test_rcp_unimplemented_ops_silent_nop() {
        // Canonical set of encodings we explicitly chose NOT to implement.
        // If a real caller ever shows up, delete the relevant entry here
        // and implement the op against a verified encoding.
        let cases: &[(&str, (u16, u16))] = &[
            // rcp_ifgte — previously opc1=6, opc2=0 MCR2.
            (
                "rcp_ifgte (MCR2 opc1=6 opc2=0)",
                encode_mcr2_full(7, 6, 0, 1, 0, 2),
            ),
            // rcp_iflte — previously opc1=6, opc2=1 MCR2.
            (
                "rcp_iflte (MCR2 opc1=6 opc2=1)",
                encode_mcr2_full(7, 6, 0, 1, 1, 2),
            ),
            // rcp_switch — previously opc1=0, opc2=2 CDP (and CDP2).
            (
                "rcp_switch (CDP  opc1=0 opc2=2)",
                encode_cdp(7, 0, 0, 0, 2, 0),
            ),
            (
                "rcp_switch (CDP2 opc1=0 opc2=2)",
                encode_cdp2(7, 0, 0, 0, 2, 0),
            ),
        ];

        for (label, (hw0, hw1)) in cases {
            let (mut cpu, mut bus) = rcp_setup();
            // Pre-load registers with values that the *old* speculative
            // semantics would treat as a FAIL (NMI) — so that if the code
            // ever regresses to the old behavior, this test flips loudly.
            //   ifgte: R1 < R2 would have been a FAIL.
            //   iflte: R1 > R2 would have been a FAIL.
            //   switch: R0 != R1 would have been a FAIL.
            cpu.regs.r[0] = 0x42;
            cpu.regs.r[1] = 10;
            cpu.regs.r[2] = 50;
            let r_before = cpu.regs.r;
            cpu.thumb32_coprocessor(*hw0, *hw1, &mut bus);
            assert!(
                cpu.pending_fault.is_none(),
                "{label}: must not raise any fault (silent NOP expected)"
            );
            assert_eq!(
                cpu.regs.r, r_before,
                "{label}: register file must not change"
            );
        }
    }

    // ---------- Sanity: existing canary_get / status / salt path still works ----------

    #[test]
    fn test_rcp_canary_status_n_flag_when_salt_valid() {
        let (mut cpu, mut bus) = rcp_setup();
        let (hw0, hw1) = encode_mrc2_full(7, 1, 0, 15, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.regs.flag_n(), "N flag must be set when salt is valid");
    }

    #[test]
    fn test_rcp_canary_status_n_flag_clear_when_salt_invalid() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 7);
        // salt_valid defaults false
        let (hw0, hw1) = encode_mrc2_full(7, 1, 0, 15, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(
            !cpu.regs.flag_n(),
            "N flag must be clear when salt is invalid"
        );
    }

    /// Encodings observed in the bootrom must round-trip through our encoder
    /// so future encoder changes can't silently lose dispatch.
    #[test]
    fn test_rcp_bootrom_encoding_lockin() {
        // Pairs derived from `roms/rp2350/arm-bootrom.dis`.
        // (encoded hw0, encoded hw1, expected fault outcome with appropriate setup)
        let cases: &[(u16, u16, &str)] = &[
            (0xFE16, 0x373C, "rcp_canary_get r3, 0x6C"),
            (0xFE06, 0x273C, "rcp_canary_check r2, 0x6C"),
            (0xFE30, 0xF710, "rcp_canary_status pc"),
            (0xFE40, 0x0710, "rcp_btrue r0"),
            (0xFE60, 0x4730, "rcp_bfalse r4"),
            (0xFE20, 0xC710, "rcp_bvalid r12"),
            (0xFE84, 0x0718, "rcp_count_init 0x48"),
            (0xFEA4, 0x0738, "rcp_count_check 0x48"),
            (0xEE00, 0x0720, "rcp_panic"),
            (0xFC43, 0x2770, "rcp_iequal r2, r3"),
        ];
        // Just assert the encoder helpers reproduce them.
        let (h0, h1) = encode_mrc2_full(7, 0, 6, 3, 1, 0xC);
        assert_eq!((h0, h1), (cases[0].0, cases[0].1), "{}", cases[0].2);
        let (h0, h1) = encode_mcr2_full(7, 0, 6, 2, 1, 0xC);
        assert_eq!((h0, h1), (cases[1].0, cases[1].1), "{}", cases[1].2);
        let (h0, h1) = encode_mrc2_full(7, 1, 0, 15, 0, 0);
        assert_eq!((h0, h1), (cases[2].0, cases[2].1), "{}", cases[2].2);
        let (h0, h1) = encode_mcr2_full(7, 2, 0, 0, 0, 0);
        assert_eq!((h0, h1), (cases[3].0, cases[3].1), "{}", cases[3].2);
        let (h0, h1) = encode_mcr2_full(7, 3, 0, 4, 1, 0);
        assert_eq!((h0, h1), (cases[4].0, cases[4].1), "{}", cases[4].2);
        let (h0, h1) = encode_mcr2_full(7, 1, 0, 12, 0, 0);
        assert_eq!((h0, h1), (cases[5].0, cases[5].1), "{}", cases[5].2);
        let (h0, h1) = encode_mcr2_full(7, 4, 4, 0, 0, 8);
        assert_eq!((h0, h1), (cases[6].0, cases[6].1), "{}", cases[6].2);
        let (h0, h1) = encode_mcr2_full(7, 5, 4, 0, 1, 8);
        assert_eq!((h0, h1), (cases[7].0, cases[7].1), "{}", cases[7].2);
        let (h0, h1) = encode_cdp(7, 0, 0, 0, 1, 0);
        assert_eq!((h0, h1), (cases[8].0, cases[8].1), "{}", cases[8].2);
        let (h0, h1) = encode_mcrr2(7, 7, 3, 2, 0);
        assert_eq!((h0, h1), (cases[9].0, cases[9].1), "{}", cases[9].2);
    }

    /// Bootrom-style flow: set salt, canary_get into a register, then later
    /// canary_check with that same register — must always pass. Locks in the
    /// "consistent get/check pair regardless of salt value" property the
    /// bootrom relies on.
    #[test]
    fn test_rcp_canary_get_check_roundtrip_with_zero_salt() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 7);
        // salt is 0, salt_valid is false — bootrom early state.

        // canary_get r3, 0x6C
        let (crn, crm) = split_imm8(0x6c);
        let (hw0, hw1) = encode_mrc2_full(7, 0, crn, 3, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        // Stash through r2 (mimic bootrom push/pop).
        cpu.regs.r[2] = cpu.regs.r[3];

        // canary_check r2, 0x6C
        let (hw0, hw1) = encode_mcr2_full(7, 0, crn, 2, 1, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(
            cpu.pending_fault.is_none(),
            "get/check pair must roundtrip even with zero salt"
        );
    }

    /// CDP2 `rcp_panic` via 0xFE00 prefix — same bit pattern but with the
    /// "2" prefix. Treated identically.
    #[test]
    fn test_rcp_panic_cdp2_form_also_nmis() {
        let (mut cpu, mut bus) = rcp_setup();
        let (hw0, hw1) = encode_cdp2(7, 0, 0, 0, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::Nmi)));
    }

    /// Unrecognized CP7 encodings remain silent NOPs (per HLD §8.4 — not all
    /// future encodings need to be enumerated; bootrom doesn't use them).
    #[test]
    fn test_rcp_unrecognized_mcr2_silent_nop() {
        let (mut cpu, mut bus) = rcp_setup();
        // opc1=7, opc2=7 — not assigned by us.
        let (hw0, hw1) = encode_mcr2_full(7, 7, 0, 0, 7, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(
            cpu.pending_fault.is_none(),
            "unknown CP7 encoding must be silent NOP"
        );
    }

    // ============================================================
    // Phase 7 Stage D — CP4/5 DCP (double-precision coprocessor)
    // ============================================================
    //
    // Encoding lock-in — see `dcp_transfer_family` and `dcp_cdp_family`
    // docstrings for the field meanings.
    //
    // | Mnemonic     | Form | opc1 | opc2 | CRn | CRm | Notes |
    // |--------------|------|------|------|-----|-----|-------|
    // | wxma         | MCR  | 0    | 0    | 0   | d[2:0] | ARM Rt -> half A of d |
    // | wxmb         | MCR  | 0    | 1    | 0   | d[2:0] | ARM Rt -> half B of d |
    // | rfma         | MRC  | 0    | 0    | 0   | d[2:0] | half A of d -> ARM Rt |
    // | rfmb         | MRC  | 0    | 1    | 0   | d[2:0] | half B of d -> ARM Rt |
    // | dadd         | CDP  | 0    | 0    | Rn  | Rm  | d[Rd] = d[Rn]+d[Rm]   |
    // | dsub         | CDP  | 0    | 1    | Rn  | Rm  | d[Rd] = d[Rn]-d[Rm]   |
    // | dmul         | CDP  | 0    | 2    | Rn  | Rm  | d[Rd] = d[Rn]*d[Rm]   |
    // | ddiv         | CDP  | 0    | 3    | Rn  | Rm  | d[Rd] = d[Rn]/d[Rm]   |
    // | dsqrt        | CDP  | 0    | 4    | Rn  | —   | d[Rd] = sqrt(d[Rn])   |
    // | dcmp_eq      | CDP  | 1    | 0    | Rn  | Rm  | status bit 0 = (==)   |
    // | dcmp_lt      | CDP  | 1    | 1    | Rn  | Rm  | status bit 0 = (<)    |
    // | dcmp_le      | CDP  | 1    | 2    | Rn  | Rm  | status bit 0 = (<=)   |
    // | dcmp_gt      | CDP  | 1    | 3    | Rn  | Rm  | status bit 0 = (>)    |
    // | dcmp_ge      | CDP  | 1    | 4    | Rn  | Rm  | status bit 0 = (>=)   |
    // | i2d          | CDP  | 2    | 0    | Rn  | —   | d[Rd] = (f64) i32(half_a(d[Rn])) |
    // | u2d          | CDP  | 2    | 1    | Rn  | —   | d[Rd] = (f64) u32(half_a(d[Rn])) |
    // | d2i          | CDP  | 2    | 2    | Rn  | —   | half_a(d[Rd]) = d[Rn] as i32     |
    // | d2u          | CDP  | 2    | 3    | Rn  | —   | half_a(d[Rd]) = d[Rn] as u32     |
    // | d2f          | CDP  | 2    | 4    | Rn  | —   | half_a(d[Rd]) = d[Rn] as f32     |
    // | f2d          | CDP  | 2    | 5    | Rn  | —   | d[Rd] = (f64) f32(half_a(d[Rn])) |
    // | dcpstat_get  | CDP  | 3    | 0    | —   | —   | half_a(d[Rd]) = dcp_status       |
    // | dcpstat_clr  | CDP  | 3    | 1    | —   | —   | dcp_status = 0                   |

    /// Write an f64 into double `d` of the DCP register file via two wxma
    /// (half A) / wxmb (half B) MCR ops. Test-only helper.
    fn dcp_load_double(cpu: &mut CortexM33, bus: &mut Bus, d: usize, v: f64) {
        let bits = v.to_bits();
        let lo = bits as u32;
        let hi = (bits >> 32) as u32;
        cpu.regs.r[0] = lo;
        let (hw0, hw1) = encode_mcr_full(4, 0, 0, 0, 0, d as u8);
        cpu.thumb32_coprocessor(hw0, hw1, bus);
        cpu.regs.r[0] = hi;
        let (hw0, hw1) = encode_mcr_full(4, 0, 0, 0, 1, d as u8);
        cpu.thumb32_coprocessor(hw0, hw1, bus);
    }

    /// Read an f64 from double `d` via two rfma/rfmb MRC ops. Test-only.
    fn dcp_read_double_via_mrc(cpu: &mut CortexM33, bus: &mut Bus, d: usize) -> f64 {
        let (hw0, hw1) = encode_mrc_full(4, 0, 0, 1, 0, d as u8);
        cpu.thumb32_coprocessor(hw0, hw1, bus);
        let lo = cpu.regs.r[1];
        let (hw0, hw1) = encode_mrc_full(4, 0, 0, 2, 1, d as u8);
        cpu.thumb32_coprocessor(hw0, hw1, bus);
        let hi = cpu.regs.r[2];
        f64::from_bits(((hi as u64) << 32) | (lo as u64))
    }

    /// Encode a CDP to CP4 (or CP5) with our DCP encoding layout.
    fn encode_cdp_dcp(opc1: u8, opc2: u8, crd: u8, crn: u8, crm: u8) -> (u16, u16) {
        encode_cdp(4, opc1, crn, crd, opc2, crm)
    }

    /// Helper: seed both halves of double `d` directly on the CPU state.
    /// Bypasses the MCR path — used by tests that want to test CDP in
    /// isolation from transfer-family correctness.
    fn dcp_set_double(cpu: &mut CortexM33, d: usize, v: f64) {
        let bits = v.to_bits();
        cpu.dcp_halves[d * 2] = bits as u32;
        cpu.dcp_halves[d * 2 + 1] = (bits >> 32) as u32;
    }

    // -------- Transfer roundtrip --------

    #[test]
    fn test_dcp_wxma_wxmb_rfma_rfmb_roundtrip() {
        // Half A and half B of a double roundtrip through the transfer
        // family independently. Confirms the opc2 bit cleanly discriminates
        // the two halves of a single double.
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        let v = -1.5_f64;
        dcp_load_double(&mut cpu, &mut bus, 2, v);
        let got = dcp_read_double_via_mrc(&mut cpu, &mut bus, 2);
        assert_eq!(got.to_bits(), v.to_bits());
    }

    #[test]
    fn test_dcp_transfer_cp5_mirrors_cp4() {
        // CP5 is a mirror of CP4 per the cp4_5_dcp dispatch. A write via
        // CP5 MCR must be observable via a CP4 MRC (and vice versa) —
        // they share the same register file on `CortexM33`.
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        enable_cp(&mut cpu, 5);

        cpu.regs.r[0] = 0xABCD_1234;
        // Write via CP5 — half A of double 3.
        let (hw0, hw1) = encode_mcr_full(5, 0, 0, 0, 0, 3);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        // Read back via CP4.
        let (hw0, hw1) = encode_mrc_full(4, 0, 0, 1, 0, 3);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[1], 0xABCD_1234);
    }

    // -------- Arithmetic: dadd / dsub / dmul / ddiv / dsqrt --------

    #[test]
    fn test_dcp_dadd_basic() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        dcp_set_double(&mut cpu, 0, 1.25);
        dcp_set_double(&mut cpu, 1, 2.75);
        let (hw0, hw1) = encode_cdp_dcp(0, 0, 2, 0, 1);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.dcp_read_double(2), 4.0);
        // Status: nonzero, positive, finite, not NaN — all bits clear.
        assert_eq!(cpu.dcp_status, 0);
    }

    #[test]
    fn test_dcp_dsub_negative_result_sets_neg_bit() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        dcp_set_double(&mut cpu, 0, 1.0);
        dcp_set_double(&mut cpu, 1, 5.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 1, 2, 0, 1);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.dcp_read_double(2), -4.0);
        assert_eq!(cpu.dcp_status & 0b1111, 0b0010, "N=1, others=0");
    }

    #[test]
    fn test_dcp_dmul_cycles_are_five() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 6.0);
        dcp_set_double(&mut cpu, 1, 7.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 2, 2, 0, 1);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cycles, 5);
        assert_eq!(cpu.dcp_read_double(2), 42.0);
    }

    #[test]
    fn test_dcp_ddiv_cycles_eighteen() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 10.0);
        dcp_set_double(&mut cpu, 1, 4.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cycles, 18);
        assert_eq!(cpu.dcp_read_double(2), 2.5);
        assert_eq!(cpu.dcp_status, 0);
    }

    #[test]
    fn test_dcp_dsqrt_cycles_twentyeight() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 16.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 4, 1, 0, 0);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cycles, 28);
        assert_eq!(cpu.dcp_read_double(1), 4.0);
    }

    // -------- Status register bits --------

    #[test]
    fn test_dcp_status_zero_bit() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 3.0);
        dcp_set_double(&mut cpu, 1, 3.0);
        // 3.0 - 3.0 = 0.0 -> status bit 0 set.
        let (hw0, hw1) = encode_cdp_dcp(0, 1, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status & 0b1111, 0b0001);
    }

    #[test]
    fn test_dcp_status_negative_zero_bit() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, -0.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        // -0.0 * 0.0 = -0.0 → Z=1 AND N=1.
        let (hw0, hw1) = encode_cdp_dcp(0, 2, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status & 0b1111, 0b0011, "Z=1, N=1");
    }

    #[test]
    fn test_dcp_status_infinity_bit() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 1.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        // 1 / 0 → +inf
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_read_double(2), f64::INFINITY);
        assert_eq!(cpu.dcp_status & 0b1111, 0b0100, "Inf=1, Z=0, N=0, NaN=0");
    }

    #[test]
    fn test_dcp_status_nan_bit() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 0.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        // 0 / 0 → NaN
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.dcp_read_double(2).is_nan());
        // NaN bit is mandatory. Sign bit on a 0/0 NaN is implementation
        // defined (x86 SSE yields a negative-signed NaN here); zero bit
        // must NOT be set; infinity bit must NOT be set.
        assert!(cpu.dcp_status & (1 << 3) != 0, "NaN bit must set");
        assert_eq!(cpu.dcp_status & (1 << 0), 0, "zero bit must clear");
        assert_eq!(cpu.dcp_status & (1 << 2), 0, "infinity bit must clear");
    }

    #[test]
    fn test_dcp_status_negative_infinity() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, -1.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_read_double(2), f64::NEG_INFINITY);
        assert_eq!(cpu.dcp_status & 0b1111, 0b0110, "Inf=1, N=1");
    }

    // -------- Compare operations --------

    #[test]
    fn test_dcp_dcmp_eq_true_and_false() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 3.5);
        dcp_set_double(&mut cpu, 1, 3.5);
        let (hw0, hw1) = encode_cdp_dcp(1, 0, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 1);

        dcp_set_double(&mut cpu, 1, 3.625);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 0);
    }

    #[test]
    fn test_dcp_dcmp_lt_le_gt_ge() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 1.0);
        dcp_set_double(&mut cpu, 1, 2.0);

        // lt: 1 < 2 true
        let (hw0, hw1) = encode_cdp_dcp(1, 1, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 1);

        // le: 1 <= 2 true
        let (hw0, hw1) = encode_cdp_dcp(1, 2, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 1);

        // gt: 1 > 2 false
        let (hw0, hw1) = encode_cdp_dcp(1, 3, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 0);

        // ge: 1 >= 2 false
        let (hw0, hw1) = encode_cdp_dcp(1, 4, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 0);

        // Equal case: le true, ge true, lt/gt false.
        dcp_set_double(&mut cpu, 1, 1.0);
        let (hw0, hw1) = encode_cdp_dcp(1, 2, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 1, "le on equals must be true");
        let (hw0, hw1) = encode_cdp_dcp(1, 4, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 1, "ge on equals must be true");
        let (hw0, hw1) = encode_cdp_dcp(1, 1, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 0, "lt on equals must be false");
    }

    #[test]
    fn test_dcp_dcmp_nan_all_false() {
        // IEEE-754: every compare involving a NaN (even eq) returns false.
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, f64::NAN);
        dcp_set_double(&mut cpu, 1, 1.0);
        for opc2 in 0..=4 {
            let (hw0, hw1) = encode_cdp_dcp(1, opc2, 0, 0, 1);
            cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
            assert_eq!(cpu.dcp_status, 0, "NaN compare opc2={opc2} must be false");
        }
    }

    // -------- Convert --------

    #[test]
    fn test_dcp_i2d_then_d2i_roundtrip() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        // Half A of double 0 carries i32 = -42.
        cpu.dcp_halves[0] = (-42_i32) as u32;
        let (hw0, hw1) = encode_cdp_dcp(2, 0, 1, 0, 0); // i2d: d[1] = (f64)(-42)
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_read_double(1), -42.0);

        // d2i: half_a(d[2]) = d[1] as i32
        let (hw0, hw1) = encode_cdp_dcp(2, 2, 2, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_halves[4] as i32, -42);
    }

    #[test]
    fn test_dcp_u2d_then_d2u_roundtrip() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        cpu.dcp_halves[0] = 0x8000_0001_u32;
        let (hw0, hw1) = encode_cdp_dcp(2, 1, 1, 0, 0); // u2d
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_read_double(1), 0x8000_0001_u32 as f64);

        // d2u: half_a(d[2]) = d[1] as u32.
        let (hw0, hw1) = encode_cdp_dcp(2, 3, 2, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_halves[4], 0x8000_0001);
    }

    #[test]
    fn test_dcp_d2i_rounds_toward_zero() {
        // Rust's `f64 as i32` truncates toward zero. We adopt this as the DCP
        // convention (cheapest, matches the Pico SDK documented semantics).
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 3.9);
        let (hw0, hw1) = encode_cdp_dcp(2, 2, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_halves[2] as i32, 3);

        dcp_set_double(&mut cpu, 0, -3.9);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_halves[2] as i32, -3);
    }

    #[test]
    fn test_dcp_f2d_then_d2f_roundtrip() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        let original: f32 = 2.625;
        cpu.dcp_halves[0] = original.to_bits();
        // f2d: d[1] = (f64)(f32)d_half_a(d[0])
        let (hw0, hw1) = encode_cdp_dcp(2, 5, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_read_double(1), original as f64);

        // d2f: half_a(d[2]) = f32(d[1])
        let (hw0, hw1) = encode_cdp_dcp(2, 4, 2, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(f32::from_bits(cpu.dcp_halves[4]), original);
    }

    // -------- Divide by zero / NaN edge cases --------

    #[test]
    fn test_dcp_ddiv_one_by_zero_plus_inf() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 1.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_read_double(2), f64::INFINITY);
        assert_eq!(cpu.dcp_status & 0b1111, 0b0100, "Inf bit only");
    }

    #[test]
    fn test_dcp_ddiv_zero_by_zero_nan() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);
        dcp_set_double(&mut cpu, 0, 0.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.dcp_read_double(2).is_nan());
        // NaN sign bit per IEEE is implementation-defined; on x86/ARM a
        // default qNaN has bit 63 clear, so N bit will be 0. NaN bit only.
        assert!(cpu.dcp_status & (1 << 3) != 0, "NaN bit must set");
    }

    // -------- Status register access via CDP --------

    #[test]
    fn test_dcp_status_get_and_clr() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        // Generate a NaN result so status gets the NaN bit.
        dcp_set_double(&mut cpu, 0, 0.0);
        dcp_set_double(&mut cpu, 1, 0.0);
        let (hw0, hw1) = encode_cdp_dcp(0, 3, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.dcp_status & (1 << 3) != 0);

        // dcpstat_get: half A of d[3] = status.
        let (hw0, hw1) = encode_cdp_dcp(3, 0, 3, 0, 0);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cycles, 1);
        assert_eq!(cpu.dcp_halves[6], cpu.dcp_status);

        // dcpstat_clr.
        let (hw0, hw1) = encode_cdp_dcp(3, 1, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.dcp_status, 0);
    }

    // -------- CPACR disabled --------

    #[test]
    fn test_dcp_cpacr_disabled_faults() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        // CPACR defaults to 0 — CP4/5 disabled.
        let (hw0, hw1) = encode_cdp_dcp(0, 0, 0, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(matches!(cpu.pending_fault, Some(Fault::UsageFault)));
    }

    // -------- Unrecognized CDP encoding — silent NOP --------

    #[test]
    fn test_dcp_unrecognized_cdp_silent_nop() {
        // opc1=7 is not assigned (we use 0..3). Reserved opc2 under a valid
        // opc1 must also silent-NOP and not corrupt doubles.
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        // Seed doubles to something recognizable.
        dcp_set_double(&mut cpu, 2, 1234.5);
        let halves_before = cpu.dcp_halves;

        // Try opc1=7, opc2=7 — fully unassigned.
        let (hw0, hw1) = encode_cdp_dcp(7, 7, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
        assert_eq!(cpu.dcp_halves, halves_before);

        // Try opc1=0, opc2=7 (arith class with unassigned opc2).
        let (hw0, hw1) = encode_cdp_dcp(0, 7, 2, 0, 1);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert!(cpu.pending_fault.is_none());
        assert_eq!(cpu.dcp_halves, halves_before);
    }

    // -------- Encoding lock-in — catch encoder-table drift --------

    /// These are the canonical (hw0, hw1) pairs the DCP handler must
    /// interpret as each listed op. If encoder helpers ever drift, this
    /// test fails loudly before downstream tests get confused.
    #[test]
    fn test_dcp_encoding_lockin() {
        // wxma Rt=1 -> half A of d[3]
        let e = encode_mcr_full(4, 0, 0, 1, 0, 3);
        assert_eq!(e.0 & 0xFFE0, 0xEE00, "wxma hw0 high bits");
        assert_eq!(e.0 & 0x10, 0x00, "wxma L=0");
        assert_eq!((e.1 >> 5) & 0x7, 0, "wxma opc2=0");
        assert_eq!(e.1 & 0xF, 3, "wxma CRm=3");
        assert_eq!(e.1 & 0x10, 0x10, "wxma bit4=1");

        // rfmb Rt=5 -> half B of d[2]
        let e = encode_mrc_full(4, 0, 0, 5, 1, 2);
        assert_eq!(e.0 & 0xF0, 0x10, "rfmb L=1");
        assert_eq!((e.1 >> 5) & 0x7, 1, "rfmb opc2=1");

        // dadd d[2] = d[0] + d[1]: encode_cdp_dcp(0, 0, 2, 0, 1)
        let e = encode_cdp_dcp(0, 0, 2, 0, 1);
        assert_eq!((e.0 >> 4) & 0xF, 0, "dadd opc1=0");
        assert_eq!((e.1 >> 5) & 0x7, 0, "dadd opc2=0");
        assert_eq!(e.1 & 0x10, 0, "CDP: bit 4 = 0");

        // dsqrt d[1] = sqrt(d[0])
        let e = encode_cdp_dcp(0, 4, 1, 0, 0);
        assert_eq!((e.0 >> 4) & 0xF, 0);
        assert_eq!((e.1 >> 5) & 0x7, 4);
    }

    // -------- All eight doubles independently addressable --------

    #[test]
    fn test_dcp_all_eight_doubles_distinct() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        for d in 0..8 {
            dcp_load_double(&mut cpu, &mut bus, d, (d as f64) + 0.5);
        }
        for d in 0..8 {
            let got = dcp_read_double_via_mrc(&mut cpu, &mut bus, d);
            assert_eq!(got, (d as f64) + 0.5, "d[{d}] mismatch");
        }
    }

    // -------- d2i/d2u NaN divergence lock-in --------

    /// Locks in the current d2i(NaN) behaviour: Rust's `NaN as i32` yields 0
    /// (since 1.45), so half A of the destination is 0; arith status then
    /// reflects the NaN input with the NaN bit set. This diverges from some
    /// FPUs that would signal IOC; we intentionally adopt the cheap Rust
    /// cast semantics (HLD §17.x accepted). Future "fixes" that silently
    /// change the produced integer or drop the NaN status bit must break
    /// this test.
    #[test]
    fn test_dcp_d2i_nan_produces_zero_with_nan_status() {
        let mut cpu = CortexM33::for_test(0);
        let mut bus = Bus::default();
        enable_cp(&mut cpu, 4);

        // Seed d[0] = NaN (quiet), then d2i: half_a(d[1]) = d[0] as i32.
        dcp_set_double(&mut cpu, 0, f64::NAN);
        let (hw0, hw1) = encode_cdp_dcp(2, 2, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        // Rust's `NaN as i32` saturates to 0.
        assert_eq!(
            cpu.dcp_halves[2] as i32, 0,
            "d2i(NaN) half A must be 0 (Rust `NaN as i32` semantics)"
        );
        // Half B is always cleared by the d2i path.
        assert_eq!(cpu.dcp_halves[3], 0, "d2i clears half B");
        // Arith status is set from the pre-cast NaN: NaN bit (1<<3) set.
        assert!(
            cpu.dcp_status & (1 << 3) != 0,
            "DCP_NAN status bit must be set; got {:#x}",
            cpu.dcp_status
        );
    }
}
