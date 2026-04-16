//! PIO state machine primitive.
//!
//! # Field-level invariants
//!
//! `StateMachine` fields are `pub(crate)` intentionally — several carry
//! invariants that must not be bypassed by external writes. Do **not**
//! promote these to `pub` without understanding each invariant:
//!
//! - `pc` is masked `& 0x1F` on every advance; external writes that skip
//!   the mask can read past `instr_mem[31]` and fetch garbage.
//! - `isr_count` / `osr_count` are clamped `.min(32)` after every IN/OUT;
//!   unclamped values desync autopush/autopull threshold checks.
//! - `stalled` and `stall_kind` are paired; clearing one without the other
//!   breaks `check_stall` re-evaluation.
//! - `pc`, `stalled`, `stall_kind`, `delay_count`, and `pending_exec` form
//!   the SM's control-flow state and must transition together (see
//!   `force_execute` for the guarded path).
//!
//! Expose chip-side read access via small accessor methods (e.g.
//! [`StateMachine::enabled`]). Writes from outside the crate are not
//! supported — reprogram via the PIO register bus instead.

use super::decode::{decode, DecodedInsn, PioOp};
use super::fifo::PioFifo;

/// One PIO state machine.
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
    pub(crate) pending_exec: Option<u16>,
    pub(crate) sm_id: u8,

    // Stall context (for re-evaluating stall conditions)
    pub(crate) stall_kind: StallKind,

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

    // Side-set is genuinely per-SM (each SM has its own sideset_base /
    // sideset_count and these latches carry across the delay cycles). The
    // non-side-set pad value / direction latches are shared at the block
    // level — see `PioBlock::shared_pin_values` / `shared_pin_dirs`.
    pub(crate) sideset_pins: u32,
    pub(crate) sideset_dirs: u32,

    /// Diagnostic counter — number of times this SM has successfully
    /// autopushed `isr` into `rx_fifo` (i.e. autopush enabled, threshold
    /// reached, and FIFO had room). Used by the PicoGUS bring-up harness
    /// to confirm the IN PINS → autopush → RX FIFO chain reaches the
    /// firmware. Pure observation — never read by execution logic.
    pub autopush_count: u64,

    /// Diagnostic mirror of the ISR word at the moment of the most
    /// recent successful autopush. Updated in the autopush branch only
    /// (paired with `autopush_count`); explicit `PUSH` instructions do
    /// not touch this field. Used by the PicoGUS capture-coverage
    /// diagnostic to decode the pushed `(addr, data)` against the fired
    /// event and attribute captured vs misattributed. Pure observation.
    pub last_autopush_word: u32,

    /// Diagnostic per-PC execution counters (one entry per 5-bit PC
    /// value). Incremented when an instruction fetched from
    /// `instr_mem[pc]` actually executes without stalling. Forced
    /// executions via `SMn_INSTR` (`pending_exec`) are excluded — they
    /// don't correspond to PC-driven control flow. Used by the PicoGUS
    /// PSRAM SPI debug to confirm the wrap loop visits each slot. Pure
    /// observation.
    pub pc_visits: [u64; 32],

    /// Diagnostic — count of PIO clock cycles where this SM was stalled
    /// (any `StallKind`), i.e. the `check_stall` → `still_stalled`
    /// early-return path in `execute_cycle`. Pure observation. Useful
    /// when paired with `pc_visits` to distinguish "SM made no progress
    /// because it was stalled" from "SM never reached that PC".
    pub stall_cycles: u64,

    /// Diagnostic — subset of `stall_cycles` where the stalled PC equals
    /// 0x19 (the PicoGUS PSRAM SPI `OUT PINS,1` slot). Pure observation.
    pub cycles_stalled_at_pc_0x19: u64,
}

/// Tracks what kind of stall we're in, so re-evaluation knows what to check.
pub(crate) enum StallKind {
    None,
    WaitGpio { polarity: bool, index: u8 },
    WaitPin { polarity: bool, index: u8 },
    WaitIrq { polarity: bool, index: u8 },
    Pull,
    Push,
    IrqWait { index: u8 },
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
            // OSR "empty" at reset = all 32 bits have been shifted out
            // (matches epio and real RP2350: autopull fires on the first
            // OUT so OSR gets a fresh value instead of outputting zeros).
            osr_count: 32,
            delay_count: 0,
            stalled: false,
            enabled: false,
            last_insn: 0,
            pending_exec: None,
            sm_id: 0,
            stall_kind: StallKind::None,
            clkdiv_int: 1,
            clkdiv_frac: 0,
            clkdiv_acc: 0,
            execctrl: 0x0001_F000,
            shiftctrl: 0x000C_0000,
            pinctrl: 0x1400_0000,
            tx_fifo: PioFifo::new(4),
            rx_fifo: PioFifo::new(4),
            // Sideset latch takes the pullup-reset convention (matches
            // epio and weakly-pulled-up RP2350 pad defaults): a side-set
            // pin whose value has never been written reads high.
            sideset_pins: u32::MAX,
            sideset_dirs: 0,
            autopush_count: 0,
            last_autopush_word: 0,
            pc_visits: [0; 32],
            stall_cycles: 0,
            cycles_stalled_at_pc_0x19: 0,
        }
    }

    /// Reset to power-on defaults.
    pub fn reset(&mut self) {
        let id = self.sm_id;
        *self = Self::new();
        self.sm_id = id;
    }

    /// Returns whether this SM is currently enabled (CTRL.SM_ENABLE bit).
    ///
    /// Chip-side code (bus tests, debug UIs) only needs a read view; writes
    /// happen through the PIO CTRL register. See module docs for the full
    /// invariant set.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// True iff this SM's TX FIFO is full (no room for another word).
    /// Used by `PioBlock::tx_dreq` to surface DMA DREQ status without
    /// exposing the FIFO itself.
    pub fn tx_fifo_full(&self) -> bool {
        self.tx_fifo.is_full()
    }

    /// Read-only view of the SM's program counter (5-bit, 0..=31).
    /// Diagnostic — chip-side observers (PicoGUS bring-up harness) need
    /// to track PC advances per system clock without going through MMIO.
    pub fn pc(&self) -> u8 {
        self.pc
    }

    /// Diagnostic: current ISR contents (32-bit). For tests that need
    /// to inspect the input shift register without popping it through
    /// the FIFO. Pure observation.
    pub fn isr_value(&self) -> u32 {
        self.isr
    }

    /// Diagnostic: current ISR shift count (0..=32).
    pub fn isr_shift_count(&self) -> u8 {
        self.isr_count
    }

    /// True iff this SM's RX FIFO is empty (nothing to drain). Used by
    /// `PioBlock::rx_dreq`.
    pub fn rx_fifo_empty(&self) -> bool {
        self.rx_fifo.is_empty()
    }

    /// Diagnostic: number of successful TX FIFO pushes (FIFO had room).
    /// Pure observation; never read by execution. Surfaced by the
    /// PicoGUS DMA-dispatch diagnostic to confirm DMA→TXF actually
    /// landed bytes vs hit a full FIFO.
    pub fn tx_push_success(&self) -> u64 {
        self.tx_fifo.push_success
    }

    /// Diagnostic: number of MMIO TXFn writes that hit a full TX FIFO
    /// and silently dropped the value (the only external path to
    /// `tx_fifo.push`). Non-zero means the SM didn't drain fast enough
    /// OR DREQ throttling failed upstream.
    pub fn tx_push_drop(&self) -> u64 {
        self.tx_fifo.push_drop
    }

    /// Diagnostic: number of successful RX FIFO pushes (autopush /
    /// explicit PUSH that found room). Mirrors `tx_push_success` for
    /// the RX direction. Catches autopush dropping on a full RX FIFO.
    pub fn rx_push_success(&self) -> u64 {
        self.rx_fifo.push_success
    }

    /// Diagnostic: number of explicit non-blocking `PUSH` instructions
    /// that hit a full RX FIFO and silently dropped the word. Autopush
    /// does NOT bump this counter — it gates on non-full and stalls
    /// otherwise, so use `autopush_count` staying low as the
    /// autopush-loss diagnostic instead.
    pub fn rx_push_drop(&self) -> u64 {
        self.rx_fifo.push_drop
    }

    /// Diagnostic: per-PC execution counter snapshot. Element `i` is the
    /// number of times a fetched-from-`instr_mem[i]` instruction has
    /// executed without stalling. Pure observation.
    pub fn pc_visits(&self) -> &[u64; 32] {
        &self.pc_visits
    }

    /// Diagnostic: total number of PIO clock cycles this SM spent
    /// stalled (any `StallKind`). Pure observation.
    pub fn stall_cycles(&self) -> u64 {
        self.stall_cycles
    }

    /// Diagnostic: number of stalled PIO clock cycles observed at
    /// PC=0x19 specifically. Pure observation.
    pub fn cycles_stalled_at_pc_0x19(&self) -> u64 {
        self.cycles_stalled_at_pc_0x19
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

    /// Returns true if this SM should execute a PIO cycle this system clock.
    pub fn clock_tick(&mut self) -> bool {
        if !self.enabled {
            return false;
        }

        let threshold = if self.clkdiv_int == 0 {
            256u32
        } else {
            (self.clkdiv_int as u32) * 256 + self.clkdiv_frac as u32
        };

        self.clkdiv_acc += 256;
        if self.clkdiv_acc >= threshold {
            self.clkdiv_acc -= threshold;
            true
        } else {
            false
        }
    }

    /// Execute one PIO cycle. Called when clock_tick() returns true.
    pub fn execute_cycle(
        &mut self,
        instr_mem: &[u16; 32],
        irq_flags: &mut u8,
        gpio_in: u32,
        shared_pin_values: &mut u32,
        shared_pin_dirs: &mut u32,
    ) {
        // Handle delay countdown
        if self.delay_count > 0 {
            self.delay_count -= 1;
            return;
        }

        // Re-evaluate stall condition
        if self.stalled {
            let still_stalled = self.check_stall(irq_flags, gpio_in);
            if still_stalled {
                // Diagnostic: bump stall-cycle counters. Pure observation.
                self.stall_cycles = self.stall_cycles.wrapping_add(1);
                if self.pc == 0x19 {
                    self.cycles_stalled_at_pc_0x19 =
                        self.cycles_stalled_at_pc_0x19.wrapping_add(1);
                }
                return;
            }
            self.stalled = false;
            self.stall_kind = StallKind::None;
            // Fall through to re-execute the stalled instruction.
            // PC hasn't advanced, so fetch from instr_mem[pc] gets the same
            // instruction — this time the stall condition is resolved (e.g. FIFO
            // now has data), so execution completes normally.
        }

        // Fetch instruction: pending_exec overrides normal fetch
        let (insn, is_forced) = if let Some(forced) = self.pending_exec.take() {
            (forced, true)
        } else {
            (instr_mem[self.pc as usize], false)
        };

        // Snapshot the fetched-from PC before execute_insn mutates
        // self.pc (JMP / OUT PC / MOV PC set it directly). Used by the
        // pc_visits diagnostic below.
        let fetched_pc = self.pc;

        // Decode
        let decoded = decode(insn, self.pinctrl, self.execctrl);

        // Apply side-set ALWAYS (even if instruction will stall)
        self.apply_sideset(&decoded);

        // Execute — returns true if instruction set PC directly (JMP, OUT PC, MOV PC)
        let pc_set = self.execute_insn(
            &decoded,
            irq_flags,
            gpio_in,
            shared_pin_values,
            shared_pin_dirs,
        );

        // If not stalled, set delay and advance PC
        if !self.stalled {
            self.delay_count = decoded.delay;
            if !is_forced && !pc_set {
                self.advance_pc();
            }
            // Diagnostic: this instruction actually ran to completion
            // at `fetched_pc`. Exclude forced execs (no PC fetch) and
            // cycles that stalled mid-execute (e.g. OUT stalling on a
            // just-emptied TX FIFO under autopull). Pure observation.
            if !is_forced {
                self.pc_visits[(fetched_pc as usize) & 0x1F] =
                    self.pc_visits[(fetched_pc as usize) & 0x1F].wrapping_add(1);
            }
        } else {
            // Instruction freshly stalled this cycle (e.g. WAIT with
            // condition unmet, PULL on empty TX FIFO). Count the cycle
            // against stall_cycles so the "cycles at PC=0x19 stalled"
            // counter includes both re-stalls and first-stalls.
            self.stall_cycles = self.stall_cycles.wrapping_add(1);
            if fetched_pc == 0x19 {
                self.cycles_stalled_at_pc_0x19 =
                    self.cycles_stalled_at_pc_0x19.wrapping_add(1);
            }
        }

        self.last_insn = insn;
    }

    /// Force-execute an instruction written to SMn_INSTR.
    pub fn force_execute(
        &mut self,
        insn: u16,
        instr_mem: &[u16; 32],
        irq_flags: &mut u8,
        gpio_in: u32,
        shared_pin_values: &mut u32,
        shared_pin_dirs: &mut u32,
    ) {
        // Clear any existing stall/delay
        self.stalled = false;
        self.stall_kind = StallKind::None;
        self.delay_count = 0;
        self.pending_exec = Some(insn);
        self.execute_cycle(
            instr_mem,
            irq_flags,
            gpio_in,
            shared_pin_values,
            shared_pin_dirs,
        );
    }

    /// Check if the current stall condition is still active.
    fn check_stall(&self, irq_flags: &mut u8, gpio_in: u32) -> bool {
        match self.stall_kind {
            StallKind::None => false,
            StallKind::Pull => self.tx_fifo.is_empty(),
            StallKind::Push => self.rx_fifo.is_full(),
            StallKind::WaitGpio { polarity, index } => {
                let pin_val = (gpio_in >> (index & 31)) & 1 != 0;
                pin_val != polarity
            }
            StallKind::WaitPin { polarity, index } => {
                let in_base = (self.pinctrl >> 15) & 0x1F;
                let pin = (in_base + index as u32) & 31;
                let pin_val = (gpio_in >> pin) & 1 != 0;
                pin_val != polarity
            }
            StallKind::WaitIrq { polarity, index } => {
                let flag_set = (*irq_flags >> (index & 7)) & 1 != 0;
                if flag_set == polarity {
                    // Match — auto-clear the flag
                    *irq_flags &= !(1 << (index & 7));
                    false
                } else {
                    true
                }
            }
            StallKind::IrqWait { index } => {
                // Wait until the flag we set is cleared by someone else
                let flag_set = (*irq_flags >> (index & 7)) & 1 != 0;
                flag_set
            }
        }
    }

    /// Advance PC with wrap check.
    fn advance_pc(&mut self) {
        let wrap_top = ((self.execctrl >> 12) & 0x1F) as u8;
        let wrap_bottom = ((self.execctrl >> 7) & 0x1F) as u8;
        if self.pc == wrap_top {
            self.pc = wrap_bottom;
        } else {
            self.pc = (self.pc + 1) & 0x1F;
        }
    }

    /// Apply side-set values to pins.
    fn apply_sideset(&mut self, decoded: &DecodedInsn) {
        if let Some(ss_val) = decoded.sideset {
            let sideset_count = ((self.pinctrl >> 29) & 7) as u8;
            let side_en = (self.execctrl >> 30) & 1 != 0;
            let actual_pins = if side_en {
                sideset_count.saturating_sub(1)
            } else {
                sideset_count
            };
            if actual_pins == 0 {
                return;
            }
            let sideset_base = ((self.pinctrl >> 10) & 0x1F) as u8;
            let side_pindir = (self.execctrl >> 29) & 1 != 0;
            if side_pindir {
                let mut pd = self.sideset_dirs;
                Self::write_pin_field(&mut pd, ss_val as u32, sideset_base, actual_pins);
                self.sideset_dirs = pd;
            } else {
                let mut sp = self.sideset_pins;
                Self::write_pin_field(&mut sp, ss_val as u32, sideset_base, actual_pins);
                self.sideset_pins = sp;
            }
        }
    }

    /// Write `count` bits of `value` to a pin field starting at `base`, wrapping mod 32.
    fn write_pin_field(field: &mut u32, value: u32, base: u8, count: u8) {
        if count == 0 {
            return;
        }
        let mask = if count >= 32 {
            u32::MAX
        } else {
            (1u32 << count) - 1
        };
        let positioned_val = (value & mask).rotate_left(base as u32);
        let positioned_mask = mask.rotate_left(base as u32);
        *field = (*field & !positioned_mask) | positioned_val;
    }

    /// Read input pins relative to IN_BASE.
    fn read_pins(&self, gpio_in: u32) -> u32 {
        let in_base = (self.pinctrl >> 15) & 0x1F;
        gpio_in.rotate_right(in_base)
    }

    /// Get the pull threshold from SHIFTCTRL. 0 means 32.
    fn pull_threshold(&self) -> u8 {
        let t = ((self.shiftctrl >> 25) & 0x1F) as u8;
        if t == 0 { 32 } else { t }
    }

    /// Check if autopull is enabled (SHIFTCTRL bit 17).
    fn is_autopull_enabled(&self) -> bool {
        (self.shiftctrl >> 17) & 1 != 0
    }

    /// Check if autopush is enabled (SHIFTCTRL bit 16).
    fn is_autopush_enabled(&self) -> bool {
        (self.shiftctrl >> 16) & 1 != 0
    }

    /// Get the push threshold from SHIFTCTRL. 0 means 32.
    fn push_threshold(&self) -> u8 {
        let t = ((self.shiftctrl >> 20) & 0x1F) as u8;
        if t == 0 { 32 } else { t }
    }

    /// Execute a single decoded instruction. Returns true if the instruction
    /// set the PC directly (JMP taken, OUT PC, MOV PC), meaning advance_pc
    /// should NOT be called.
    fn execute_insn(
        &mut self,
        decoded: &DecodedInsn,
        irq_flags: &mut u8,
        gpio_in: u32,
        shared_pin_values: &mut u32,
        shared_pin_dirs: &mut u32,
    ) -> bool {
        match &decoded.op {
            PioOp::Jmp { condition, address } => {
                self.exec_jmp(*condition, *address, gpio_in)
            }
            PioOp::Wait { polarity, source, index } => {
                self.exec_wait(*polarity, *source, *index, irq_flags, gpio_in);
                false
            }
            PioOp::In { source, bit_count } => {
                self.exec_in(*source, *bit_count, gpio_in);
                false
            }
            PioOp::Out { destination, bit_count } => {
                self.exec_out(*destination, *bit_count, shared_pin_values, shared_pin_dirs)
            }
            PioOp::Push { if_full, block } => {
                self.exec_push(*if_full, *block);
                false
            }
            PioOp::Pull { if_empty, block } => {
                self.exec_pull(*if_empty, *block);
                false
            }
            PioOp::Mov { destination, op, source } => {
                self.exec_mov(
                    *destination,
                    *op,
                    *source,
                    gpio_in,
                    shared_pin_values,
                    shared_pin_dirs,
                )
            }
            PioOp::Irq { clear, wait, index } => {
                self.exec_irq(*clear, *wait, *index, irq_flags);
                false
            }
            PioOp::Set { destination, data } => {
                self.exec_set(*destination, *data, shared_pin_values, shared_pin_dirs);
                false
            }
        }
    }

    /// JMP instruction. Returns true if the jump was taken (PC was set).
    fn exec_jmp(&mut self, condition: u8, address: u8, gpio_in: u32) -> bool {
        let take_jump = match condition {
            0 => true,                    // Always
            1 => self.x == 0,            // !X
            2 => {                        // X-- (jump+decrement iff X != 0)
                // Datasheet §3.4.4: if X is already 0, the branch is
                // not taken AND X remains 0. The decrement is part of
                // the taken-jump action, not a mandatory side effect —
                // otherwise X=0 wraps to 0xFFFFFFFF and the loop never
                // terminates (observed as the PicoGUS PSRAM slot 0x1A
                // running 1.1M iterations without reaching slot 0x1B).
                let was_nonzero = self.x != 0;
                if was_nonzero {
                    self.x = self.x.wrapping_sub(1);
                }
                was_nonzero
            }
            3 => self.y == 0,            // !Y
            4 => {                        // Y-- (jump+decrement iff Y != 0)
                let was_nonzero = self.y != 0;
                if was_nonzero {
                    self.y = self.y.wrapping_sub(1);
                }
                was_nonzero
            }
            5 => self.x != self.y,       // X!=Y
            6 => {                        // PIN (JMP_PIN from EXECCTRL[28:24])
                let jmp_pin = (self.execctrl >> 24) & 0x1F;
                (gpio_in >> jmp_pin) & 1 != 0
            }
            7 => {                        // !OSRE (osr_count < pull_threshold)
                self.osr_count < self.pull_threshold()
            }
            _ => false,
        };

        if take_jump {
            self.pc = address & 0x1F;
            true
        } else {
            false
        }
    }

    /// WAIT instruction.
    fn exec_wait(
        &mut self,
        polarity: bool,
        source: u8,
        index: u8,
        irq_flags: &mut u8,
        gpio_in: u32,
    ) {
        match source {
            // GPIO (absolute pin)
            0 => {
                let pin_val = (gpio_in >> (index & 31)) & 1 != 0;
                if pin_val != polarity {
                    self.stalled = true;
                    self.stall_kind = StallKind::WaitGpio { polarity, index };
                }
            }
            // PIN (in_base-relative)
            1 => {
                let in_base = (self.pinctrl >> 15) & 0x1F;
                let pin = (in_base + index as u32) & 31;
                let pin_val = (gpio_in >> pin) & 1 != 0;
                if pin_val != polarity {
                    self.stalled = true;
                    self.stall_kind = StallKind::WaitPin { polarity, index };
                }
            }
            // IRQ (auto-clear on match)
            2 => {
                let irq_idx = self.resolve_irq_index(index);
                let flag_set = (*irq_flags >> (irq_idx & 7)) & 1 != 0;
                if flag_set == polarity {
                    // Condition met — auto-clear the flag
                    *irq_flags &= !(1 << (irq_idx & 7));
                } else {
                    self.stalled = true;
                    self.stall_kind = StallKind::WaitIrq {
                        polarity,
                        index: irq_idx,
                    };
                }
            }
            // JMPPIN stub (RP2350 extension) — treat as NOP
            _ => {}
        }
    }

    /// IN instruction.
    fn exec_in(&mut self, source: u8, bit_count: u8, gpio_in: u32) {
        let in_shiftdir_right = (self.shiftctrl >> 18) & 1 != 0;

        let src_val = match source {
            0 => self.read_pins(gpio_in),  // PINS
            1 => self.x,                    // X
            2 => self.y,                    // Y
            3 => 0,                         // NULL
            6 => self.isr,                  // ISR
            7 => self.osr,                  // OSR
            _ => 0,                         // Reserved
        };

        let bc = bit_count as u32;
        let data = if bc >= 32 { src_val } else { src_val & ((1u32 << bc) - 1) };

        if in_shiftdir_right {
            // Shift right: new data goes into MSB side
            if bc >= 32 { self.isr = 0; } else { self.isr >>= bc; }
            if bc < 32 {
                self.isr |= data << (32 - bc);
            } else {
                self.isr = data;
            }
        } else {
            // Shift left: new data goes into LSB side
            if bc >= 32 { self.isr = 0; } else { self.isr <<= bc; }
            self.isr |= data;
        }

        self.isr_count = (self.isr_count + bit_count).min(32);

        // Autopush: push ISR to RX FIFO when threshold reached
        if self.is_autopush_enabled() {
            let threshold = self.push_threshold();
            if self.isr_count >= threshold {
                if !self.rx_fifo.is_full() {
                    self.rx_fifo.push(self.isr);
                    // Mirror the just-pushed word for downstream
                    // diagnostics. Paired with `autopush_count`; stays
                    // unchanged when the FIFO is full (no bump).
                    self.last_autopush_word = self.isr;
                    self.isr = 0;
                    self.isr_count = 0;
                    self.autopush_count = self.autopush_count.wrapping_add(1);
                }
                // If RX FIFO is full, ISR retains its value (no stall for autopush)
            }
        }
    }

    /// OUT instruction. Returns true if destination is PC (PC was set).
    fn exec_out(
        &mut self,
        destination: u8,
        bit_count: u8,
        shared_pin_values: &mut u32,
        shared_pin_dirs: &mut u32,
    ) -> bool {
        // Autopull: refill OSR from TX FIFO before OUT reads it
        if self.is_autopull_enabled() {
            let threshold = self.pull_threshold();
            if self.osr_count >= threshold {
                if let Some(val) = self.tx_fifo.pop() {
                    self.osr = val;
                    self.osr_count = 0;
                } else {
                    // TX FIFO empty — stall (same as blocking PULL)
                    self.stalled = true;
                    self.stall_kind = StallKind::Pull;
                    return false;
                }
            }
        }

        let out_shiftdir_right = (self.shiftctrl >> 19) & 1 != 0;
        let bc = bit_count as u32;

        // Extract data from OSR
        let data = if out_shiftdir_right {
            // Shift right: data comes from LSB side
            let d = if bc >= 32 { self.osr } else { self.osr & ((1u32 << bc) - 1) };
            if bc >= 32 { self.osr = 0; } else { self.osr >>= bc; }
            d
        } else {
            // Shift left: data comes from MSB side
            let d = if bc >= 32 { self.osr } else { self.osr >> (32 - bc) };
            if bc >= 32 { self.osr = 0; } else { self.osr <<= bc; }
            d
        };

        self.osr_count = (self.osr_count + bit_count).min(32);

        // Write data to destination
        let pc_set = destination == 5;
        match destination {
            0 => {
                // PINS (out_base-relative) — writes shared output latch
                let out_base = (self.pinctrl & 0x1F) as u8;
                let out_count = ((self.pinctrl >> 20) & 0x3F) as u8;
                let count = out_count.min(bit_count);
                Self::write_pin_field(shared_pin_values, data, out_base, count);
            }
            1 => self.x = data,           // X
            2 => self.y = data,           // Y
            3 => {}                        // NULL (discard)
            4 => {
                // PINDIRS — writes shared direction latch
                let out_base = (self.pinctrl & 0x1F) as u8;
                let out_count = ((self.pinctrl >> 20) & 0x3F) as u8;
                let count = out_count.min(bit_count);
                Self::write_pin_field(shared_pin_dirs, data, out_base, count);
            }
            5 => {
                // PC — set directly
                self.pc = (data & 0x1F) as u8;
            }
            6 => {
                // ISR
                self.isr = data;
            }
            7 => {
                // EXEC — store shifted value as instruction to execute next cycle
                self.pending_exec = Some(data as u16);
            }
            _ => {}
        }
        pc_set
    }

    /// PUSH instruction.
    fn exec_push(&mut self, if_full: bool, block: bool) {
        if self.rx_fifo.is_full() {
            if if_full {
                // If-full and FIFO is full: no-op
                return;
            }
            if block {
                // Block and FIFO is full: stall
                self.stalled = true;
                self.stall_kind = StallKind::Push;
                return;
            }
            // Non-blocking, non-if_full, full FIFO: push drops (FIFO handles)
        }
        self.rx_fifo.push(self.isr);
        self.isr = 0;
        self.isr_count = 0;
    }

    /// PULL instruction.
    fn exec_pull(&mut self, if_empty: bool, block: bool) {
        if self.tx_fifo.is_empty() {
            if if_empty {
                // If-empty and FIFO is empty: copy X to OSR
                self.osr = self.x;
                self.osr_count = 0;
                return;
            }
            if block {
                // Block and FIFO is empty: stall
                self.stalled = true;
                self.stall_kind = StallKind::Pull;
                return;
            }
            // Non-blocking, empty FIFO: copy X into OSR (RP2040 datasheet behaviour)
            self.osr = self.x;
            self.osr_count = 0;
            return;
        }
        self.osr = self.tx_fifo.pop().unwrap();
        self.osr_count = 0;
    }

    /// MOV instruction. Returns true if destination is PC (PC was set).
    fn exec_mov(
        &mut self,
        destination: u8,
        op: u8,
        source: u8,
        gpio_in: u32,
        shared_pin_values: &mut u32,
        shared_pin_dirs: &mut u32,
    ) -> bool {
        // Read source
        let mut val = match source {
            0 => {                          // PINS — RP2350 masks by IN_COUNT
                let raw = self.read_pins(gpio_in);
                let in_count = (self.shiftctrl & 0x1F) as u32;
                if in_count == 0 || in_count >= 32 {
                    raw
                } else {
                    raw & ((1u32 << in_count) - 1)
                }
            }
            1 => self.x,                    // X
            2 => self.y,                    // Y
            3 => 0,                         // NULL
            5 => {                          // STATUS
                let status_sel = (self.execctrl >> 4) & 1 != 0;
                let status_n = (self.execctrl & 0xF) as u8;
                let level = if status_sel {
                    self.rx_fifo.level()
                } else {
                    self.tx_fifo.level()
                };
                if level < status_n { u32::MAX } else { 0 }
            }
            6 => self.isr,                  // ISR
            7 => self.osr,                  // OSR
            _ => 0,                         // Reserved
        };

        // Apply operation
        val = match op {
            0 => val,                       // None
            1 => !val,                      // Invert
            2 => val.reverse_bits(),        // Bit-reverse
            _ => val,                       // Reserved
        };

        // Write destination
        let pc_set = destination == 5;
        match destination {
            0 => {
                // PINS (out_base-relative) — writes shared output latch
                let out_base = (self.pinctrl & 0x1F) as u8;
                let out_count = ((self.pinctrl >> 20) & 0x3F) as u8;
                Self::write_pin_field(shared_pin_values, val, out_base, out_count);
            }
            1 => self.x = val,
            2 => self.y = val,
            3 => {
                // PINDIRS (RP2350 extension) — OUT-pin-range direction latch.
                let out_base = (self.pinctrl & 0x1F) as u8;
                let out_count = ((self.pinctrl >> 20) & 0x3F) as u8;
                Self::write_pin_field(shared_pin_dirs, val, out_base, out_count);
            }
            4 => {
                // EXEC — execute val as instruction next cycle
                self.pending_exec = Some(val as u16);
            }
            5 => {
                // PC — set directly
                self.pc = (val & 0x1F) as u8;
            }
            6 => self.isr = val,
            7 => self.osr = val,
            _ => {}
        }
        pc_set
    }

    /// IRQ instruction.
    fn exec_irq(&mut self, clear: bool, wait: bool, index: u8, irq_flags: &mut u8) {
        let irq_num = self.resolve_irq_index(index);

        if clear {
            *irq_flags &= !(1 << (irq_num & 7));
        } else {
            *irq_flags |= 1 << (irq_num & 7);
            if wait {
                // Stall until the flag is cleared by someone else
                self.stalled = true;
                self.stall_kind = StallKind::IrqWait { index: irq_num };
            }
        }
    }

    /// SET instruction.
    fn exec_set(
        &mut self,
        destination: u8,
        data: u8,
        shared_pin_values: &mut u32,
        shared_pin_dirs: &mut u32,
    ) {
        match destination {
            0 => {
                // PINS (set_base-relative, up to SET_COUNT) — writes shared output latch
                let set_base = ((self.pinctrl >> 5) & 0x1F) as u8;
                let set_count = ((self.pinctrl >> 26) & 0x7) as u8;
                Self::write_pin_field(shared_pin_values, data as u32, set_base, set_count);
            }
            1 => self.x = data as u32,           // X (zero-extend)
            2 => self.y = data as u32,           // Y (zero-extend)
            4 => {
                // PINDIRS (set_base-relative, up to SET_COUNT) — writes shared direction latch
                let set_base = ((self.pinctrl >> 5) & 0x1F) as u8;
                let set_count = ((self.pinctrl >> 26) & 0x7) as u8;
                Self::write_pin_field(shared_pin_dirs, data as u32, set_base, set_count);
            }
            _ => {}
        }
    }

    /// Resolve IRQ index with relative flag.
    fn resolve_irq_index(&self, index: u8) -> u8 {
        if index & 0x10 != 0 {
            // Relative: offset lower 2 bits by SM id, preserve bit 2
            ((index & 3) + self.sm_id) % 4 | (index & 4)
        } else {
            index & 7
        }
    }
}

#[cfg(test)]
mod tests {
    //! Unit tests for `last_autopush_word` — the diagnostic mirror
    //! paired with `autopush_count`.

    use super::*;

    /// Build an SM with autopush enabled at an 8-bit threshold, shifting
    /// left (so an 8-bit IN of `value` lands as `value` in ISR bits 7..0).
    fn autopush_sm() -> StateMachine {
        let mut sm = StateMachine::new();
        // SHIFTCTRL: AUTOPUSH (bit 16) + PUSH_THRESH = 8 (bits 24..20)
        // + IN_SHIFTDIR = left (bit 18 = 0). Leave OUT_SHIFTDIR at its
        // reset value (bit 19 still cleared here — the tests don't OUT).
        sm.shiftctrl = (1u32 << 16) | (8u32 << 20);
        sm
    }

    #[test]
    fn last_autopush_word_tracks_last_autopushed_isr() {
        let mut sm = autopush_sm();
        sm.x = 0xAB;
        // IN X, 8 — shift 8 bits of X into ISR. With IN_SHIFTDIR=left,
        // ISR becomes 0xAB; isr_count reaches 8 == push threshold and
        // autopush fires into the empty RX FIFO.
        sm.exec_in(1, 8, 0);
        assert_eq!(sm.autopush_count, 1);
        assert_eq!(sm.last_autopush_word, 0xAB);
        // A second IN with a distinct value updates the mirror.
        sm.x = 0xCD;
        sm.exec_in(1, 8, 0);
        assert_eq!(sm.autopush_count, 2);
        assert_eq!(sm.last_autopush_word, 0xCD);
    }

    #[test]
    fn last_autopush_word_unchanged_when_fifo_full() {
        let mut sm = autopush_sm();
        // Fill the 4-entry RX FIFO directly so the next autopush finds
        // it full (matches the existing `autopush_count` semantics: no
        // bump when the FIFO has no room).
        for v in 0..4u32 {
            assert!(sm.rx_fifo.push(v));
        }
        assert!(sm.rx_fifo.is_full());
        let before_word = sm.last_autopush_word;
        let before_cnt = sm.autopush_count;
        sm.x = 0x5A;
        sm.exec_in(1, 8, 0);
        // autopush must not fire — ISR retains its value.
        assert_eq!(sm.autopush_count, before_cnt);
        assert_eq!(sm.last_autopush_word, before_word);
        assert_eq!(sm.isr, 0x5A);
        assert_eq!(sm.isr_count, 8);
    }

    /// pc_visits counter bumps once per completed non-stalling fetch at
    /// the corresponding slot. Program: slot 0 = `MOV Y, Y` (no-op that
    /// never stalls), slot 1 = `JMP 0` (unconditional, never stalls).
    /// Executing 10 cycles with a fresh SM should visit each slot five
    /// times (slot 0 → advance_pc → slot 1 → JMP 0 → slot 0 …).
    #[test]
    fn pc_visits_counter_splits_two_slot_loop() {
        let mut sm = StateMachine::new();
        sm.enabled = true;
        // Reset wrap top to 0x01 so advance_pc at slot 0 goes to slot 1
        // (default wrap_top=0x1F, wrap_bottom=0 would also work since
        // we JMP 0 from slot 1 — advance_pc after slot 0 just increments).
        // Leave execctrl at default (wrap_top=0x1F, wrap_bottom=0).
        let mut instr_mem = [0u16; 32];
        // MOV Y, Y: opcode 101_00000_010_00010 = 0xA042
        //   bits [15:13]=101 (MOV), [12:8]=00000 (delay/sideset=0),
        //   [7:5]=010 (dst=Y), [4:3]=00 (op=none), [2:0]=010 (src=Y).
        instr_mem[0] = 0xA042;
        // JMP 0 (unconditional): 000_00000_000_00000 = 0x0000
        instr_mem[1] = 0x0000;
        let mut irq = 0u8;
        let mut pins = 0u32;
        let mut dirs = 0u32;
        for _ in 0..10 {
            sm.execute_cycle(&instr_mem, &mut irq, 0, &mut pins, &mut dirs);
        }
        // Ten cycles split evenly across slots 0 and 1.
        assert_eq!(sm.pc_visits[0], 5, "slot 0 visited 5 times");
        assert_eq!(sm.pc_visits[1], 5, "slot 1 visited 5 times");
        // No other slots touched.
        let other_visits: u64 = (2..32).map(|i| sm.pc_visits[i]).sum();
        assert_eq!(other_visits, 0, "only slots 0 and 1 are visited");
        // Neither instruction stalls, so stall_cycles should be zero.
        assert_eq!(sm.stall_cycles, 0);
        assert_eq!(sm.cycles_stalled_at_pc_0x19, 0);
    }

    /// JMP X-- termination: with X=3 the loop body (slot 0) + back-edge
    /// (slot 1 = JMP X-- 0) must run exactly X+1 = 4 iterations, then
    /// fall through to the exit sentinel at slot 2. Datasheet §3.4.4:
    /// X-- evaluates `X != 0`; on true it jumps and decrements X, on
    /// false it falls through without decrement. Regression guard for
    /// the PicoGUS PSRAM SPI bring-up symptom where slot 0x19/0x1A
    /// looped 1,146,584 times and slot 0x1B never executed.
    #[test]
    fn jmp_x_minus_minus_exits_after_x_plus_one_iterations() {
        let mut sm = StateMachine::new();
        sm.enabled = true;
        sm.x = 3;
        let mut instr_mem = [0u16; 32];
        // Slot 0: MOV Y, Y (no-op body) = 0xA042.
        instr_mem[0] = 0xA042;
        // Slot 1: JMP X-- 0 = opcode 000, cond=010 (X--), addr=00000 = 0x0040.
        instr_mem[1] = 0x0040;
        // Slot 2: SET PINS, 0xF (exit sentinel) = 0xE00F.
        instr_mem[2] = 0xE00F;
        // Slot 3: WAIT 1 IRQ 0 = 0x20C0 — stalls forever so the counters
        // beyond the exit don't keep churning and perturb the asserts.
        instr_mem[3] = 0x20C0;
        let mut irq = 0u8;
        let mut pins = 0u32;
        let mut dirs = 0u32;
        for _ in 0..50 {
            sm.execute_cycle(&instr_mem, &mut irq, 0, &mut pins, &mut dirs);
        }
        assert_eq!(sm.x, 0, "X must be 0 after termination");
        assert_eq!(sm.pc_visits[0], 4, "slot 0 runs X+1 = 4 times");
        assert_eq!(sm.pc_visits[1], 4, "slot 1 runs X+1 = 4 times");
        assert!(
            sm.pc_visits[2] >= 1,
            "slot 2 (exit sentinel) must execute at least once, got {}",
            sm.pc_visits[2]
        );
    }

    /// JMP Y-- termination: analogous to the X-- test with Y=2 → 3
    /// iterations. Separate test because Y is a distinct condition
    /// code (4 vs 2) on a different register path in `exec_jmp`.
    #[test]
    fn jmp_y_minus_minus_exits_after_y_plus_one_iterations() {
        let mut sm = StateMachine::new();
        sm.enabled = true;
        sm.y = 2;
        let mut instr_mem = [0u16; 32];
        // Slot 0: MOV Y, Y (no-op body) = 0xA042.
        instr_mem[0] = 0xA042;
        // Slot 1: JMP Y-- 0 = opcode 000, cond=100 (Y--), addr=00000 = 0x0080.
        instr_mem[1] = 0x0080;
        // Slot 2: SET PINS, 0xF (exit sentinel) = 0xE00F.
        instr_mem[2] = 0xE00F;
        // Slot 3: WAIT 1 IRQ 0 = 0x20C0 (park after exit).
        instr_mem[3] = 0x20C0;
        let mut irq = 0u8;
        let mut pins = 0u32;
        let mut dirs = 0u32;
        for _ in 0..50 {
            sm.execute_cycle(&instr_mem, &mut irq, 0, &mut pins, &mut dirs);
        }
        assert_eq!(sm.y, 0, "Y must be 0 after termination");
        assert_eq!(sm.pc_visits[0], 3, "slot 0 runs Y+1 = 3 times");
        assert_eq!(sm.pc_visits[1], 3, "slot 1 runs Y+1 = 3 times");
        assert!(
            sm.pc_visits[2] >= 1,
            "slot 2 (exit sentinel) must execute at least once, got {}",
            sm.pc_visits[2]
        );
    }

    /// JMP X-- with X=0 falls through without decrementing. Datasheet
    /// §3.4.4: the condition is `X != 0`; a zero X must neither jump
    /// nor wrap around to 0xFFFFFFFF.
    #[test]
    fn jmp_x_minus_minus_with_x_zero_falls_through_without_decrement() {
        let mut sm = StateMachine::new();
        sm.enabled = true;
        // X is 0 from reset; spell it out for the reader.
        assert_eq!(sm.x, 0);
        let mut instr_mem = [0u16; 32];
        // Slot 0: JMP X-- 5 = opcode 000, cond=010, addr=00101 = 0x0045.
        instr_mem[0] = 0x0045;
        let mut irq = 0u8;
        let mut pins = 0u32;
        let mut dirs = 0u32;
        sm.execute_cycle(&instr_mem, &mut irq, 0, &mut pins, &mut dirs);
        assert_eq!(sm.x, 0, "X must stay 0 (no wrap to 0xFFFFFFFF)");
        assert_eq!(sm.pc, 1, "PC must advance past the JMP, not jump to 5");
        assert_eq!(sm.pc_visits[0], 1);
        assert_eq!(sm.pc_visits[5], 0, "target slot 5 must not have been visited");
    }
}
