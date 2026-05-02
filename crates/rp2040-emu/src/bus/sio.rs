//! RP2040 Single-cycle IO block (base 0xD000_0000).
//!
//! Dataheet §2.3. Provides:
//!
//! * CPUID (0x000) — reads as the requesting core's id (0 or 1).
//! * GPIO_IN (0x004) — 30-bit input snapshot. Handled on Bus (merges SIO
//!   output + PIO outputs). This struct owns the rest.
//! * GPIO_OUT (0x010), GPIO_OUT_SET/CLR/XOR (0x014/0x018/0x01C).
//! * GPIO_OE (0x020), GPIO_OE_SET/CLR/XOR (0x024/0x028/0x02C).
//! * Inter-core FIFO (0x050-0x058).
//! * 32 spinlocks at 0x100-0x17C.
//! * Integer divider at 0x060-0x078.
//! * Interpolators 0/1 at 0x080-0x0FC.
//!
//! **Differs from RP2350 SIO** by:
//! * 30-bit GPIO mask (not 30 — same, but kept distinct for clarity).
//! * GPIO_OUT/OE offsets `0x010`/`0x014`/… (SET/CLR/XOR at 4-byte spacing),
//!   not the RP2350 8-byte spacing at `0x010`/`0x018`/….
//! * **No** DOORBELL block — RP2040 has no inter-core doorbells.
//! * **No** MTIME — RP2040 lacks the platform timer.
//! * **No** coprocessor bridge FIFO.

use tracing::trace;

use picoem_common::{Divider, Fifo};

/// RP2040 GPIO pin mask — 30 valid GPIOs (bits [29:0]).
pub(crate) const PIN_MASK: u32 = 0x3FFF_FFFF;

/// Launch token produced by the multicore handshake FSM on completion of
/// the 6-word sequence `{0, 0, 1, VTOR, SP, entry}`. Consumed by
/// `Emulator::step` via [`Sio::take_pending_launch`] to apply VTOR / MSP /
/// PC on core 1 and wake it.
#[derive(Debug, Clone, Copy)]
pub struct Core1Launch {
    pub vtor: u32,
    pub sp: u32,
    pub entry: u32,
}

/// Pico SDK / bootrom multicore-launch handshake mirror.
///
/// See `wrk_docs/2026.04.16 - HLD - RP2040 Core 1 Multicore Launch Handshake.md`.
/// The FSM is armed iff core 1 is halted (flag tracked eagerly by the
/// enclosing Emulator via `halt_core1` / `wake_core1`). While armed,
/// core-0 writes to `SIO_FIFO_WR` are consumed by the FSM — nothing is
/// pushed into `fifo_to_core1`; the expected response is echoed back into
/// `fifo_to_core0`.
struct MulticoreHandshake {
    /// True iff core 1 is halted (FSM only runs while armed).
    armed: bool,
    /// Current expected-word slot (0..=5). Resets to 0 on any mismatch.
    seq: u8,
    /// VTOR captured at seq=3.
    vtor: u32,
    /// SP captured at seq=4.
    sp: u32,
    /// Set at seq=5 on successful handshake completion. Consumed by
    /// `Emulator::step`.
    pending_launch: Option<Core1Launch>,
}

impl Default for MulticoreHandshake {
    fn default() -> Self {
        // Core 1 boots halted, so the FSM is armed from construction.
        Self {
            armed: true,
            seq: 0,
            vtor: 0,
            sp: 0,
            pending_launch: None,
        }
    }
}

/// Single-cycle IO block (RP2040).
pub struct Sio {
    /// GPIO_OUT register (offset 0x010).
    pub gpio_out: u32,
    /// GPIO_OE register (offset 0x020).
    pub gpio_oe: u32,
    /// Inter-processor FIFO: Core 0 writes → Core 1 reads.
    fifo_to_core1: Fifo,
    /// Inter-processor FIFO: Core 1 writes → Core 0 reads.
    fifo_to_core0: Fifo,
    /// Sticky write-overflow flag, per core.
    fifo_wof: [bool; 2],
    /// Sticky read-underflow flag, per core.
    fifo_roe: [bool; 2],
    /// 32 hardware spinlocks as a bitmask (bit N = SPINLOCK<N> claimed).
    spinlock_bits: u32,
    /// Set by FIFO_WR on successful push — Bus reads and clears this to
    /// set `event_flag[other_core]`. `Some(other_core_idx)` while pending.
    pub pending_fifo_event: Option<usize>,
    /// Per-core integer divider.
    divider: [Divider; 2],
    /// Per-core interpolator register backing store (INTERP0 at 0x080-0x0BC,
    /// INTERP1 at 0x0C0-0x0FC — 32 words per core).
    interp: [[u32; 32]; 2],
    /// Multicore-launch handshake FSM (per-HLD 2026.04.16).
    handshake: MulticoreHandshake,
}

impl Sio {
    pub fn new() -> Self {
        Self {
            gpio_out: 0,
            gpio_oe: 0,
            fifo_to_core1: Fifo::new(),
            fifo_to_core0: Fifo::new(),
            fifo_wof: [false; 2],
            fifo_roe: [false; 2],
            spinlock_bits: 0,
            pending_fifo_event: None,
            divider: [Divider::default(); 2],
            interp: [[0; 32]; 2],
            handshake: MulticoreHandshake::default(),
        }
    }

    /// Reset all SIO state. Called from `Emulator::reset()`.
    pub fn reset(&mut self) {
        self.gpio_out = 0;
        self.gpio_oe = 0;
        self.fifo_to_core1 = Fifo::new();
        self.fifo_to_core0 = Fifo::new();
        self.fifo_wof = [false; 2];
        self.fifo_roe = [false; 2];
        self.spinlock_bits = 0;
        self.pending_fifo_event = None;
        self.divider = [Divider::default(); 2];
        self.interp = [[0; 32]; 2];
        // Core 1 is re-halted by `Emulator::reset`, so the FSM re-arms.
        self.handshake = MulticoreHandshake::default();
    }

    // --- Multicore handshake plumbing (HLD 2026.04.16) ---------------------

    /// Sync the FSM `armed` flag with core 1's halt state. Called by
    /// `Emulator::halt_core1` / `Emulator::wake_core1` — the only
    /// sanctioned path for toggling core 1's halt in production code.
    /// Direct `cores[1].halt()` / `wake()` bypass this and will drift
    /// `armed` out of sync with reality (see §5 invariant in the HLD).
    #[inline]
    pub fn set_handshake_armed(&mut self, armed: bool) {
        self.handshake.armed = armed;
        if !armed {
            // Wake → FSM disabled. Clear any half-completed progress so
            // a future re-halt restarts from seq=0.
            self.handshake.seq = 0;
            self.handshake.vtor = 0;
            self.handshake.sp = 0;
            self.handshake.pending_launch = None;
        }
    }

    /// True iff the FSM is armed (core 1 currently halted). Exposed for
    /// tests (T6 in particular) and for future debuggers.
    #[inline]
    pub fn is_handshake_armed(&self) -> bool {
        self.handshake.armed
    }

    /// Current FSM slot (0..=5). Advances as valid words land; resets to
    /// 0 on any mismatch per §2.3. Exposed for tests.
    #[inline]
    pub fn handshake_seq(&self) -> u8 {
        self.handshake.seq
    }

    /// Consume any pending launch token produced by the FSM. Called by
    /// `Emulator::step` immediately after each core-0 step; on Some,
    /// the emulator applies VTOR / MSP / PC to core 1 and wakes it.
    #[inline]
    pub fn take_pending_launch(&mut self) -> Option<Core1Launch> {
        self.handshake.pending_launch.take()
    }

    /// Non-consuming snapshot of the core-0 → core-1 FIFO contents in
    /// head → tail order. Used by `threaded::ThreadedEmulator::
    /// from_emulator` to copy serial inter-core FIFO state into the
    /// threaded SPSC rings without mutating the source.
    pub fn fifo_0to1_snapshot(&self) -> Vec<u32> {
        self.fifo_to_core1.snapshot()
    }

    /// Non-consuming snapshot of the core-1 → core-0 FIFO contents in
    /// head → tail order. Counterpart to [`Self::fifo_0to1_snapshot`].
    pub fn fifo_1to0_snapshot(&self) -> Vec<u32> {
        self.fifo_to_core0.snapshot()
    }

    /// Read FIFO_ST.WOF (write-on-full sticky) for the given core.
    pub fn fifo_wof(&self, core: usize) -> bool {
        self.fifo_wof[core]
    }

    /// Read FIFO_ST.ROE (read-on-empty sticky) for the given core.
    pub fn fifo_roe(&self, core: usize) -> bool {
        self.fifo_roe[core]
    }

    /// Read the current 32-bit spinlock claim bitmask. Bit N = 1 iff
    /// SPINLOCK<N> is currently held.
    pub fn spinlock_bits(&self) -> u32 {
        self.spinlock_bits
    }

    /// 32-bit register read. `offset` is masked to 12 bits by Bus. GPIO_IN
    /// (0x004) is handled on Bus before this is called (merges SIO output
    /// with PIO output — Phase 5.B wires PIO in).
    pub fn read32(&mut self, offset: u32, core: usize) -> u32 {
        match offset {
            0x000 => core as u32, // CPUID
            0x010 => self.gpio_out,
            0x014 | 0x018 | 0x01C => self.gpio_out, // SET/CLR/XOR read as GPIO_OUT
            0x020 => self.gpio_oe,
            0x024 | 0x028 | 0x02C => self.gpio_oe,
            // FIFO block.
            0x050 => self.fifo_st_read(core),
            0x058 => self.fifo_rd(core),
            // Integer divider (0x060-0x078).
            0x060 | 0x068 => self.divider[core].dividend,
            0x064 | 0x06C => self.divider[core].divisor,
            0x070 | 0x074 => self.divider_result_read(offset, core),
            0x078 => {
                let ready = 1u32;
                let dirty = if self.divider[core].dirty { 2 } else { 0 };
                ready | dirty
            }
            // Interpolators (0x080-0x0FC). Each interpolator (INTERP0 at
            // 0x080, INTERP1 at 0x0C0) has its own register block. ACCUM
            // and BASE reads return the backing store; PEEK_LANE0/1,
            // PEEK_FULL, POP_LANE0/1, POP_FULL are computed on read from
            // CTRL/ACCUM/BASE per datasheet §2.3.1 (SHIFT, MASK, SIGNED,
            // CLAMP [INTERP1 L0], BLEND [INTERP0 L1]).
            0x080..=0x0FC => {
                let interp_idx = if offset < 0x0C0 { 0usize } else { 1usize };
                let reg_idx = ((offset - 0x080) >> 2) as usize;
                // Sub-offset within the INTERP block (0x00..=0x3C).
                let sub = (offset - 0x080) % 0x40;
                match sub {
                    // POP_LANE0/1/FULL, PEEK_LANE0/1/FULL → compute.
                    // NOTE: POP is the same as PEEK for now (we don't yet
                    // model the cross_input/cross_result accumulator
                    // side-effects — not needed for the PicoGUS use case,
                    // and the only POP consumers in currently-supported
                    // firmware use simple CLAMP/SHIFT lane outputs).
                    0x14 | 0x20 => self.interp_lane_peek(core, interp_idx, 0),
                    0x18 | 0x24 => self.interp_lane_peek(core, interp_idx, 1),
                    0x1C | 0x28 => self.interp_full_peek(core, interp_idx),
                    _ => self.interp[core][reg_idx],
                }
            }
            // Spinlock bank status at 0x05C (RP2040 specific).
            0x05C => self.spinlock_bits,
            // Spinlocks at 0x100-0x17F.
            0x100..=0x17F => self.spinlock_read(offset),
            _ => 0,
        }
    }

    /// 32-bit register write.
    pub fn write32(&mut self, offset: u32, val: u32, core: usize) {
        match offset {
            // GPIO_OUT block: 4-byte spacing (RP2040).
            0x010 => self.gpio_out = val & PIN_MASK,
            0x014 => self.gpio_out |= val & PIN_MASK, // SET
            0x018 => self.gpio_out &= !(val & PIN_MASK), // CLR
            0x01C => self.gpio_out ^= val & PIN_MASK, // XOR
            // GPIO_OE block.
            0x020 => self.gpio_oe = val & PIN_MASK,
            0x024 => self.gpio_oe |= val & PIN_MASK,
            0x028 => self.gpio_oe &= !(val & PIN_MASK),
            0x02C => self.gpio_oe ^= val & PIN_MASK,
            // FIFO block.
            0x050 => self.fifo_st_write(val, core),
            0x054 => self.fifo_wr(val, core),
            // Divider.
            0x060..=0x078 => self.divider_write(offset, val, core),
            // Interpolators.
            0x080..=0x0FC => {
                let idx = ((offset - 0x080) >> 2) as usize;
                if idx < 32 {
                    self.interp[core][idx] = val;
                }
            }
            // Spinlocks — any write releases.
            0x100..=0x17F => self.spinlock_write(offset),
            _ => {}
        }
    }

    // --- Interpolator PEEK/POP compute ------------------------------------
    //
    // Indices within `self.interp[core][...]` for INTERP{0,1}:
    //   0x080+offset/4 for INTERP0 (indices 0..15),
    //   0x0C0+offset/4 for INTERP1 (indices 16..31).
    //
    // Per-interp register layout (relative offsets, datasheet §2.3.1):
    //   +0x00 ACCUM0         +0x20 PEEK_LANE0
    //   +0x04 ACCUM1         +0x24 PEEK_LANE1
    //   +0x08 BASE0          +0x28 PEEK_FULL
    //   +0x0C BASE1          +0x2C CTRL_LANE0
    //   +0x10 BASE2          +0x30 CTRL_LANE1
    //   +0x14 POP_LANE0      +0x34 ACCUM0_ADD  (w: masked-add accum0)
    //   +0x18 POP_LANE1      +0x38 ACCUM1_ADD
    //   +0x1C POP_FULL       +0x3C BASE_1AND0  (w: base0=lo16, base1=hi16)
    //
    // CTRL bits relevant here (§2.3.1.7.6, INTERP0 and INTERP1 differ in a
    // few higher bits but share the core shape):
    //   [0:4]   SHIFT
    //   [5:9]   MASK_LSB
    //   [10:14] MASK_MSB
    //   [15]    SIGNED      (arithmetic shift + sign-extend from MASK_MSB)
    //   [16]    CROSS_INPUT (not modelled — not used by PicoGUS)
    //   [17]    CROSS_RESULT (not modelled — not used by PicoGUS)
    //   [18]    ADD_RAW
    //   [19:20] FORCE_MSB   (not modelled)
    //   [21]    BLEND       (INTERP0 lane 1 only — not modelled)
    //   [22]    CLAMP       (INTERP1 lane 0 only)
    //   [23:25] overflow sticky flags (RO; not modelled)
    //
    // Lane output = (accum[cross ? other : own] masked/shifted/sign-ext) + BASE[L]
    // except when CLAMP: BASE add is bypassed and the pre-base value is
    // clamped between signed(BASE0) and signed(BASE1).
    #[inline]
    fn interp_reg(&self, core: usize, which: usize, sub_word: usize) -> u32 {
        let base = if which == 0 { 0 } else { 16 };
        self.interp[core][base + sub_word]
    }

    fn interp_lane_peek(&self, core: usize, which: usize, lane: usize) -> u32 {
        debug_assert!(which < 2 && lane < 2);
        let accum0 = self.interp_reg(core, which, 0);
        let base0 = self.interp_reg(core, which, 2);
        let base1 = self.interp_reg(core, which, 3);
        let base_l = self.interp_reg(core, which, 2 + lane);
        let ctrl = self.interp_reg(core, which, 11 + lane);

        let shift = ctrl & 0x1F;
        let mask_lsb = (ctrl >> 5) & 0x1F;
        let mask_msb = (ctrl >> 10) & 0x1F;
        let signed = (ctrl >> 15) & 1 != 0;
        let add_raw = (ctrl >> 18) & 1 != 0;
        let clamp = which == 1 && lane == 0 && (ctrl >> 22) & 1 != 0;

        // Raw → shifted (arithmetic shift when SIGNED).
        let raw = accum0;
        let shifted: u32 = if shift >= 32 {
            if signed {
                ((raw as i32) >> 31) as u32
            } else {
                0
            }
        } else if signed {
            ((raw as i32) >> shift) as u32
        } else {
            raw >> shift
        };

        // Mask to [mask_lsb..=mask_msb]. When SIGNED, sign-extend from mask_msb.
        let mask: u64 = if mask_msb >= mask_lsb {
            let hi = if mask_msb >= 31 {
                0xFFFF_FFFFu64
            } else {
                (1u64 << (mask_msb + 1)) - 1
            };
            let lo = (1u64 << mask_lsb) - 1;
            hi & !lo
        } else {
            0
        };
        let masked = (shifted as u64) & mask;
        let mut value: u32 = if signed && mask_msb < 31 && mask_msb >= mask_lsb {
            let sign_bit = 1u32 << mask_msb;
            if (masked as u32) & sign_bit != 0 {
                (masked as u32) | (!(mask as u32))
            } else {
                masked as u32
            }
        } else {
            masked as u32
        };

        // Unused in simple lane path — ADD_RAW only affects POP's accumulator
        // update, not the lane output itself.
        let _ = add_raw;

        // Lane output: masked + base[L], unless CLAMP suppresses.
        if clamp {
            let v_signed = value as i32;
            let b0 = base0 as i32;
            let b1 = base1 as i32;
            value = if v_signed < b0 {
                base0
            } else if v_signed > b1 {
                base1
            } else {
                value
            };
        } else {
            value = value.wrapping_add(base_l);
        }
        value
    }

    fn interp_full_peek(&self, core: usize, which: usize) -> u32 {
        // Datasheet §2.3.1.2: FULL = lane0_result + lane1_result + BASE2.
        let lane0 = self.interp_lane_peek(core, which, 0);
        let lane1 = self.interp_lane_peek(core, which, 1);
        let base2 = self.interp_reg(core, which, 4);
        lane0.wrapping_add(lane1).wrapping_add(base2)
    }

    // --- GPIO bulk helpers (RP2040 has 30 valid GPIOs) --------------------

    #[inline]
    pub fn gpio_out_masked(&self) -> u32 {
        self.gpio_out & PIN_MASK
    }

    #[inline]
    pub fn gpio_oe_masked(&self) -> u32 {
        self.gpio_oe & PIN_MASK
    }

    // --- FIFO helpers ------------------------------------------------------

    fn fifo_st_read(&self, core: usize) -> u32 {
        // Bit 0: VLD — this core's RX queue has data.
        let rx_fifo = if core == 0 {
            &self.fifo_to_core0
        } else {
            &self.fifo_to_core1
        };
        let vld = !rx_fifo.is_empty();
        // Bit 1: RDY — other core's RX queue has space.
        let tx_fifo = if core == 0 {
            &self.fifo_to_core1
        } else {
            &self.fifo_to_core0
        };
        let rdy = !tx_fifo.is_full();
        let wof = self.fifo_wof[core];
        let roe = self.fifo_roe[core];
        (vld as u32) | ((rdy as u32) << 1) | ((wof as u32) << 2) | ((roe as u32) << 3)
    }

    fn fifo_st_write(&mut self, val: u32, core: usize) {
        if val & 0x4 != 0 {
            self.fifo_wof[core] = false;
        }
        if val & 0x8 != 0 {
            self.fifo_roe[core] = false;
        }
    }

    fn fifo_wr(&mut self, val: u32, core: usize) {
        // Armed path: core 0 pushing while core 1 halted. The FSM
        // consumes `val` — nothing lands in fifo_to_core1 — and the
        // required echo lands in fifo_to_core0. See §2.3 of the HLD.
        if core == 0 && self.handshake.armed {
            self.handshake_step(val);
            return;
        }

        // Unarmed path — existing behaviour. Raw IPC push.
        let other = 1 - core;
        let tx_fifo = if core == 0 {
            &mut self.fifo_to_core1
        } else {
            &mut self.fifo_to_core0
        };
        if tx_fifo.push(val) {
            self.pending_fifo_event = Some(other);
        } else {
            self.fifo_wof[core] = true;
        }
    }

    /// Drive the multicore-launch FSM for one incoming word from core 0.
    /// Echoes the protocol-defined response into `fifo_to_core0` and
    /// signals `pending_fifo_event = Some(0)` so Bus bubbles it up to
    /// `event_flag[0]` — identical to a user-code push from core 1.
    ///
    /// Transition table mirrors §2.3 of the HLD.
    fn handshake_step(&mut self, val: u32) {
        let seq = self.handshake.seq;
        debug_assert!(seq <= 5, "handshake.seq invariant: 0..=5");
        let (echo, next_seq) = match seq {
            0 => (0u32, if val == 0 { 1 } else { 0 }),
            1 => (0u32, if val == 0 { 2 } else { 0 }),
            2 => {
                if val == 1 {
                    (1u32, 3)
                } else {
                    (0u32, 0)
                }
            }
            3 => {
                if val == 0 {
                    (0u32, 0)
                } else {
                    self.handshake.vtor = val;
                    (val, 4)
                }
            }
            4 => {
                if val == 0 {
                    (0u32, 0)
                } else {
                    self.handshake.sp = val;
                    (val, 5)
                }
            }
            5 => {
                if val == 0 {
                    (0u32, 0)
                } else {
                    self.handshake.pending_launch = Some(Core1Launch {
                        vtor: self.handshake.vtor,
                        sp: self.handshake.sp,
                        entry: val,
                    });
                    (val, 0)
                }
            }
            _ => {
                debug_assert!(false, "handshake.seq out of range: {}", seq);
                (0u32, 0)
            }
        };
        self.handshake.seq = next_seq;

        // Echo into fifo_to_core0. Sender (core 0) pops this via FIFO_RD
        // as its `pop_blocking` response. `pending_fifo_event = Some(0)`
        // so Bus routes the usual FIFO-event bubble into event_flag[0].
        if self.fifo_to_core0.push(echo) {
            self.pending_fifo_event = Some(0);
        } else {
            // Unreachable on spec traffic: the sender `pop_blocking`s each
            // echo before the next push, and the FSM only echoes once per
            // incoming write, so `fifo_to_core0` holds at most one prior
            // unread echo well under the queue depth. Pin as a debug
            // invariant; silent drop in release rather than falsely
            // attributing a WOF to core 1 (the FSM is the sender here).
            debug_assert!(
                false,
                "handshake echo overflowed fifo_to_core0 — sender didn't drain per spec"
            );
        }
    }

    fn fifo_rd(&mut self, core: usize) -> u32 {
        let rx_fifo = if core == 0 {
            &mut self.fifo_to_core0
        } else {
            &mut self.fifo_to_core1
        };
        match rx_fifo.pop() {
            Some(v) => v,
            None => {
                self.fifo_roe[core] = true;
                0
            }
        }
    }

    // --- Spinlock helpers --------------------------------------------------

    fn spinlock_read(&mut self, offset: u32) -> u32 {
        let n = (offset - 0x100) >> 2;
        debug_assert!(n < 32);
        let mask = 1u32 << n;
        if self.spinlock_bits & mask == 0 {
            self.spinlock_bits |= mask;
            trace!(lock_id = n, "spinlock acquired");
            mask
        } else {
            0
        }
    }

    fn spinlock_write(&mut self, offset: u32) {
        let n = (offset - 0x100) >> 2;
        debug_assert!(n < 32);
        self.spinlock_bits &= !(1u32 << n);
        trace!(lock_id = n, "spinlock released");
    }

    // --- Divider helpers ---------------------------------------------------

    fn divider_result_read(&mut self, offset: u32, core: usize) -> u32 {
        let d = &mut self.divider[core];
        let val = match offset {
            0x070 => d.quotient,
            0x074 => d.remainder,
            _ => return 0,
        };
        if d.dirty {
            d.reads_pending += 1;
            if d.reads_pending >= 2 {
                d.dirty = false;
                d.reads_pending = 0;
            }
        }
        val
    }

    fn divider_write(&mut self, offset: u32, val: u32, core: usize) {
        let d = &mut self.divider[core];
        match offset {
            0x060 => {
                d.dividend = val;
                d.signed = false;
            }
            0x064 => {
                d.divisor = val;
                d.signed = false;
                Self::compute_division(d);
            }
            0x068 => {
                d.dividend = val;
                d.signed = true;
            }
            0x06C => {
                d.divisor = val;
                d.signed = true;
                Self::compute_division(d);
            }
            0x070 => {
                d.quotient = val;
                d.dirty = true;
                d.reads_pending = 0;
            }
            0x074 => {
                d.remainder = val;
                d.dirty = true;
                d.reads_pending = 0;
            }
            _ => {}
        }
    }

    fn compute_division(d: &mut Divider) {
        if d.divisor == 0 {
            if d.signed {
                let a = d.dividend as i32;
                d.quotient = if a < 0 { 1u32 } else { (-1i32) as u32 };
            } else {
                d.quotient = 0xFFFF_FFFF;
            }
            d.remainder = d.dividend;
        } else if d.signed {
            let a = d.dividend as i32;
            let b = d.divisor as i32;
            d.quotient = a.wrapping_div(b) as u32;
            d.remainder = a.wrapping_rem(b) as u32;
        } else {
            d.quotient = d.dividend.wrapping_div(d.divisor);
            d.remainder = d.dividend.wrapping_rem(d.divisor);
        }
        d.dirty = true;
        d.reads_pending = 0;
    }
}

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpuid_returns_requesting_core() {
        let mut sio = Sio::new();
        assert_eq!(sio.read32(0x000, 0), 0);
        assert_eq!(sio.read32(0x000, 1), 1);
    }

    #[test]
    fn gpio_out_write_and_read() {
        let mut sio = Sio::new();
        sio.write32(0x010, 0x3F, 0);
        assert_eq!(sio.read32(0x010, 0), 0x3F);
    }

    #[test]
    fn gpio_out_set_clr_xor() {
        let mut sio = Sio::new();
        sio.write32(0x010, 0x0F, 0);
        sio.write32(0x014, 0x10, 0); // SET
        assert_eq!(sio.gpio_out, 0x1F);
        sio.write32(0x018, 0x01, 0); // CLR
        assert_eq!(sio.gpio_out, 0x1E);
        sio.write32(0x01C, 0xFF, 0); // XOR
        assert_eq!(sio.gpio_out, 0xE1);
    }

    #[test]
    fn gpio_pin_mask_upper_bits() {
        let mut sio = Sio::new();
        sio.write32(0x010, 0xFFFF_FFFF, 0);
        assert_eq!(sio.gpio_out, PIN_MASK);
    }

    #[test]
    fn spinlock_claim_and_release() {
        let mut sio = Sio::new();
        // First claim returns the lock bit.
        let claim = sio.read32(0x100, 0);
        assert_eq!(claim, 1);
        // Second claim returns 0.
        let retry = sio.read32(0x100, 0);
        assert_eq!(retry, 0);
        // Release.
        sio.write32(0x100, 0, 0);
        let reclaim = sio.read32(0x100, 0);
        assert_eq!(reclaim, 1);
    }

    #[test]
    fn fifo_roundtrip() {
        let mut sio = Sio::new();
        // Default arm state is on (core 1 halted). Disarm so this test
        // exercises the raw-IPC pass-through path, not the handshake FSM.
        sio.set_handshake_armed(false);
        // Core 0 writes -> core 1 reads.
        sio.write32(0x054, 0xDEAD_BEEF, 0);
        assert_eq!(sio.pending_fifo_event, Some(1));
        let _ = sio.pending_fifo_event.take();
        assert_eq!(sio.read32(0x058, 1), 0xDEAD_BEEF);
    }

    #[test]
    fn handshake_fsm_armed_by_default() {
        let sio = Sio::new();
        assert!(sio.is_handshake_armed());
        assert_eq!(sio.handshake_seq(), 0);
    }

    #[test]
    fn handshake_seq_advances_on_valid_word() {
        let mut sio = Sio::new();
        sio.write32(0x054, 0, 0); // seq 0 -> 1
        assert_eq!(sio.handshake_seq(), 1);
        // Echo landed on fifo_to_core0.
        assert_eq!(sio.read32(0x058, 0), 0);
    }

    #[test]
    fn handshake_resets_on_mismatch() {
        let mut sio = Sio::new();
        sio.write32(0x054, 0x42, 0); // seq 0 mismatch -> 0
        assert_eq!(sio.handshake_seq(), 0);
        assert_eq!(sio.read32(0x058, 0), 0); // echoed 0
    }

    #[test]
    fn handshake_produces_launch_on_full_sequence() {
        let mut sio = Sio::new();
        let seq = [0u32, 0, 1, 0x2004_0000, 0x2001_0000, 0x2000_1001];
        for w in seq {
            sio.write32(0x054, w, 0);
            let _ = sio.read32(0x058, 0); // drain echo
        }
        let launch = sio.take_pending_launch().expect("launch token");
        assert_eq!(launch.vtor, 0x2004_0000);
        assert_eq!(launch.sp, 0x2001_0000);
        assert_eq!(launch.entry, 0x2000_1001);
    }

    #[test]
    fn handshake_unarmed_falls_through_to_raw_fifo() {
        let mut sio = Sio::new();
        sio.set_handshake_armed(false);
        sio.write32(0x054, 0x1234_5678, 0);
        // Raw push into fifo_to_core1; no echo on core-0 RX side.
        assert_eq!(sio.pending_fifo_event, Some(1));
        let _ = sio.pending_fifo_event.take();
        assert_eq!(sio.read32(0x058, 1), 0x1234_5678);
    }

    #[test]
    fn fifo_underflow_sets_roe() {
        let mut sio = Sio::new();
        // Read from empty RX fifo.
        let v = sio.read32(0x058, 0);
        assert_eq!(v, 0);
        assert_eq!(sio.read32(0x050, 0) & 0x8, 0x8); // ROE bit
    }

    #[test]
    fn divider_unsigned() {
        let mut sio = Sio::new();
        sio.write32(0x060, 100, 0);
        sio.write32(0x064, 7, 0);
        assert_eq!(sio.read32(0x070, 0), 14);
        assert_eq!(sio.read32(0x074, 0), 2);
    }

    #[test]
    fn divider_signed_divide_by_zero() {
        let mut sio = Sio::new();
        sio.write32(0x068, (-42i32) as u32, 0);
        sio.write32(0x06C, 0, 0);
        assert_eq!(sio.read32(0x070, 0), 1);
        assert_eq!(sio.read32(0x074, 0), (-42i32) as u32);
    }

    #[test]
    fn interp_roundtrip_per_core() {
        let mut sio = Sio::new();
        sio.write32(0x080, 0xAA, 0);
        sio.write32(0x080, 0xBB, 1);
        assert_eq!(sio.read32(0x080, 0), 0xAA);
        assert_eq!(sio.read32(0x080, 1), 0xBB);
    }

    /// INTERP1 configured as a signed 16-bit clamp (the PicoGUS audio
    /// mixer's use-case): SHIFT=14, SIGNED, CLAMP, BASE0=-32768, BASE1=+32767.
    /// PEEK_LANE0 should clamp (accum0 >> 14) to the [-32768, 32767] range.
    #[test]
    fn interp1_signed_clamp_lane0() {
        let mut sio = Sio::new();
        // CTRL_LANE0 @ 0x0EC = SHIFT=14 | MASK_LSB=0 | MASK_MSB=17 | SIGNED | CLAMP.
        sio.write32(0x0EC, 0x0040_C40E, 0);
        sio.write32(0x0C8, 0xFFFF_8000, 0); // BASE0 = -32768
        sio.write32(0x0CC, 0x0000_7FFF, 0); // BASE1 = +32767

        // Small positive: accum0 = 100_000. (100_000 >> 14) = 6. In range.
        sio.write32(0x0C0, 100_000, 0);
        assert_eq!(sio.read32(0x0E0, 0) as i32, 6);

        // Large positive: accum0 = 1_000_000_000. (>> 14) ≈ 61035. Clamp
        // to +32767.
        sio.write32(0x0C0, 1_000_000_000, 0);
        assert_eq!(sio.read32(0x0E0, 0), 0x0000_7FFF);

        // Large negative: accum0 = -1_000_000_000. Clamp to -32768.
        sio.write32(0x0C0, (-1_000_000_000i32) as u32, 0);
        assert_eq!(sio.read32(0x0E0, 0), 0xFFFF_8000);

        // Small negative: accum0 = -100_000. (>> 14) = -7 (arith shift).
        // In range; sign-extended to 32 bits.
        sio.write32(0x0C0, (-100_000i32) as u32, 0);
        assert_eq!(sio.read32(0x0E0, 0) as i32, -7);

        // Per-core isolation: core 1 should be independent.
        assert_eq!(sio.read32(0x0E0, 1), 0);
    }
}
