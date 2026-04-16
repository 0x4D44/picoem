pub mod decode;
pub mod fifo;
pub mod sm;

use sm::{StallKind, StateMachine};

/// One PIO block (RP2350 has three: PIO0, PIO1, PIO2).
pub struct PioBlock {
    /// Per-SM state. `StateMachine` fields are `pub(crate)` — invariants
    /// live inside the SM (see `pio/sm.rs` module docs). Chip-side code
    /// reads SM state through accessors like [`StateMachine::enabled`].
    pub sm: [StateMachine; 4],
    pub(crate) instr_mem: [u16; 32],
    pub(crate) irq_flags: u8,
    input_sync_bypass: u32,
    fdebug: u32,
    /// Shared pad value latch — OUT/SET/MOV PINS from any SM writes here.
    /// Reset to `u32::MAX` (weak-pullup convention, matches epio).
    pub(crate) shared_pin_values: u32,
    /// Shared pad direction latch — OUT/SET/MOV PINDIRS from any SM writes
    /// here. Reset to 0 (all pins input). Side-set can overlay on top.
    pub(crate) shared_pin_dirs: u32,
    pub pad_out: u32,
    pub pad_oe: u32,
    /// Bit `i` is set iff `sm[i].enabled`. Deliberately redundant with
    /// `sm[i].enabled` (the SM field stays authoritative) — this cached
    /// mask exists solely for the single-load fast-path check at the top
    /// of [`Self::step`] / [`Self::step_n`]. Maintained via
    /// [`Self::set_sm_enabled`]; direct writes to `sm[i].enabled` must not
    /// be reintroduced on the production path.
    sm_enabled_mask: u8,
    /// IRQ0_INTE — 16-bit interrupt enable mask for NVIC line 0.
    /// Bits [15:8] = SM7..SM0 IRQ flags, [7:4] = SM3..SM0 TXNFULL,
    /// [3:0] = SM3..SM0 RXNEMPTY. RP2350 datasheet offset 0x170.
    pub int0_inte: u32,
    /// IRQ0_INTF — 16-bit interrupt force for NVIC line 0. Software
    /// can force individual interrupt sources. Offset 0x174.
    int0_intf: u32,
    /// IRQ1_INTE — 16-bit interrupt enable mask for NVIC line 1.
    /// Offset 0x17C.
    pub int1_inte: u32,
    /// IRQ1_INTF — 16-bit interrupt force for NVIC line 1. Offset 0x180.
    int1_intf: u32,

    /// Diagnostic — count of `pad_out` bit 1 transitions from 1 to 0.
    /// Tracks PSRAM SPI CS falling edges (PicoGUS pin assignment:
    /// CS=GPIO1). Observed by comparing `pad_out` before and after
    /// every [`Self::merge_pin_outputs`] within [`Self::step`]. Pure
    /// observation — never read by block execution. Independent of
    /// any downstream device model's own edge counters.
    pub pad_out_cs_falls: u64,
    /// Diagnostic — count of `pad_out` bit 1 transitions from 0 to 1.
    /// Paired with [`Self::pad_out_cs_falls`]; a healthy SPI program
    /// alternates falls and rises.
    pub pad_out_cs_rises: u64,
    /// Diagnostic — count of `pad_out` bit 2 toggles (either direction).
    /// Tracks PSRAM SPI SCK edges. Each SPI bit-clock period produces
    /// two toggles (rising + falling), so this counter divided by two
    /// estimates the number of SCK cycles that actually ran.
    pub pad_out_sck_toggles: u64,
    /// Diagnostic — count of cycles where `pad_out` bit 3 is high.
    /// MOSI is a level on a given PIO clock cycle; this count rises
    /// by one per cycle that drives it high. Pure observation.
    pub pad_out_mosi_writes_of_1: u64,
    /// Prior snapshot of `pad_out` used by the transition counters
    /// above. Seeded on construction/reset so the first step's
    /// comparison is against the reset value (0).
    pub(crate) prev_pad_out_diag: u32,
}

impl PioBlock {
    pub fn new() -> Self {
        let mut sm = [
            StateMachine::new(),
            StateMachine::new(),
            StateMachine::new(),
            StateMachine::new(),
        ];
        for (i, s) in sm.iter_mut().enumerate() {
            s.sm_id = i as u8;
        }
        Self {
            sm,
            instr_mem: [0; 32],
            irq_flags: 0,
            input_sync_bypass: 0,
            fdebug: 0,
            shared_pin_values: u32::MAX,
            shared_pin_dirs: 0,
            pad_out: 0,
            pad_oe: 0,
            sm_enabled_mask: 0,
            int0_inte: 0,
            int0_intf: 0,
            int1_inte: 0,
            int1_intf: 0,
            pad_out_cs_falls: 0,
            pad_out_cs_rises: 0,
            pad_out_sck_toggles: 0,
            pad_out_mosi_writes_of_1: 0,
            prev_pad_out_diag: 0,
        }
    }

    /// Reset to power-on defaults.
    pub fn reset(&mut self) {
        for sm in &mut self.sm {
            sm.reset();
        }
        self.instr_mem = [0; 32];
        self.irq_flags = 0;
        self.input_sync_bypass = 0;
        self.fdebug = 0;
        self.shared_pin_values = u32::MAX;
        self.shared_pin_dirs = 0;
        self.pad_out = 0;
        self.pad_oe = 0;
        self.sm_enabled_mask = 0;
        self.int0_inte = 0;
        self.int0_intf = 0;
        self.int1_inte = 0;
        self.int1_intf = 0;
        self.pad_out_cs_falls = 0;
        self.pad_out_cs_rises = 0;
        self.pad_out_sck_toggles = 0;
        self.pad_out_mosi_writes_of_1 = 0;
        self.prev_pad_out_diag = 0;
    }

    /// True iff at least one SM in the block is enabled. Chip-side
    /// fast-path uses this to decide whether a PIO step could move any
    /// pin (disabled blocks are semantic no-ops — see [`Self::step`]).
    pub fn any_sm_enabled(&self) -> bool {
        self.sm_enabled_mask != 0
    }

    /// 4-bit enable mask — bit `i` set iff SM `i` is enabled. Parity
    /// with the bit layout of the CTRL register's SM_ENABLE field.
    /// Used by the threaded runtime to republish the post-CTRL-write
    /// enable state onto `ThreadedPio::sm_enabled` so CPU workers see
    /// the correct mask without reaching into `pub(crate)` fields.
    #[inline]
    pub fn sm_enabled_mask(&self) -> u8 {
        self.sm_enabled_mask
    }

    /// The 8-bit PIO IRQ-flag register as a `u32` (upper bits zero).
    ///
    /// PIO maintains 8 internal IRQ flags (`IRQ[7:0]`); flags 0..3
    /// optionally route to the NVIC when the corresponding `IRQn_INTE`
    /// bit is set on the block's `IRQ0`/`IRQ1` interrupt controllers.
    /// The two RP2040 NVIC lines per block (`PIO0_IRQ_0`, `PIO0_IRQ_1`,
    /// `PIO1_IRQ_0`, `PIO1_IRQ_1`) carry only the low 2 of those 4
    /// routable flags each — the chip-side routing helper in
    /// `mdrp2040::Emulator::tick_pio_and_route_irqs_single` masks
    /// accordingly.
    ///
    /// This getter surfaces the flags as a `u32` so callers can shift
    /// / mask into the `Bus::irq_pending` wire without a cast at every
    /// site. It does not mutate state — firmware clears flags via the
    /// `IRQ` register W1C path already modelled in [`Self::write32`].
    /// Zero behaviour change; added for the Wave 1 IRQ-routing helper.
    #[inline]
    pub fn pending_irqs(&self) -> u32 {
        self.irq_flags as u32
    }

    /// Compute the 12-bit raw interrupt status (INTR register) using the
    /// RP2040 bit layout (RP2040 datasheet Table 358, INTR at offset 0x128):
    ///   bits  [3:0] = SM3..SM0 IRQ flags (from `irq_flags[3:0]`)
    ///   bits  [7:4] = SM3..SM0 RXNEMPTY (RX not empty → 1)
    ///   bits [11:8] = SM3..SM0 TXNFULL  (TX not full → 1)
    ///
    /// Only the low 4 IRQ flags are routable on RP2040; flags 4..7 are
    /// intra-PIO only and do not appear in INTR. Bits [31:12] are zero.
    #[inline]
    pub fn raw_intr_rp2040(&self) -> u32 {
        let mut v: u32 = (self.irq_flags as u32) & 0xF; // IRQ[3:0] → bits [3:0]
        for i in 0..4u32 {
            if !self.sm[i as usize].rx_fifo.is_empty() {
                v |= 1 << (4 + i); // RXNEMPTY → bits [7:4]
            }
            if !self.sm[i as usize].tx_fifo.is_full() {
                v |= 1 << (8 + i); // TXNFULL  → bits [11:8]
            }
        }
        v
    }

    /// Compute the 16-bit raw interrupt status (INTR register) using the
    /// RP2350 bit layout (RP2350 datasheet Table 1018, INTR at offset 0x16C):
    ///   bits  [3:0] = SM3..SM0 RXNEMPTY (RX not empty → 1)
    ///   bits  [7:4] = SM3..SM0 TXNFULL  (TX not full → 1)
    ///   bits [15:8] = SM7..SM0 IRQ flags (from `irq_flags`)
    ///
    /// All 8 IRQ flags appear; the NVIC-routable subset is determined
    /// by which bits the firmware sets in IRQ0_INTE / IRQ1_INTE.
    #[inline]
    pub fn raw_intr_rp2350(&self) -> u32 {
        let mut v: u32 = (self.irq_flags as u32) << 8; // IRQ[7:0] → bits [15:8]
        for i in 0..4u32 {
            if !self.sm[i as usize].rx_fifo.is_empty() {
                v |= 1 << i; // RXNEMPTY → bits [3:0]
            }
            if !self.sm[i as usize].tx_fifo.is_full() {
                v |= 1 << (4 + i); // TXNFULL → bits [7:4]
            }
        }
        v
    }

    /// Effective interrupt status for NVIC line 0 (RP2040 layout):
    /// `(INTR_rp2040 & INTE) | INTF`.
    #[inline]
    pub fn int0_ints_rp2040(&self) -> u32 {
        (self.raw_intr_rp2040() & self.int0_inte) | self.int0_intf
    }

    /// Effective interrupt status for NVIC line 1 (RP2040 layout):
    /// `(INTR_rp2040 & INTE) | INTF`.
    #[inline]
    pub fn int1_ints_rp2040(&self) -> u32 {
        (self.raw_intr_rp2040() & self.int1_inte) | self.int1_intf
    }

    /// Effective interrupt status for NVIC line 0 (RP2350 layout):
    /// `(INTR_rp2350 & INTE) | INTF`.
    #[inline]
    pub fn int0_ints_rp2350(&self) -> u32 {
        (self.raw_intr_rp2350() & self.int0_inte) | self.int0_intf
    }

    /// Effective interrupt status for NVIC line 1 (RP2350 layout):
    /// `(INTR_rp2350 & INTE) | INTF`.
    #[inline]
    pub fn int1_ints_rp2350(&self) -> u32 {
        (self.raw_intr_rp2350() & self.int1_inte) | self.int1_intf
    }

    /// DREQ (data-request) for SM `sm`'s TX FIFO: true when the FIFO
    /// has room for another word. Consumed by the RP2040 DMA matrix
    /// (Phase 4) for `DREQ_PIO{0,1}_TX{0..3}`. Out-of-range `sm` is
    /// treated as "not ready" so the caller doesn't need to bounds-check.
    #[inline]
    pub fn tx_dreq(&self, sm: usize) -> bool {
        if sm >= self.sm.len() {
            return false;
        }
        !self.sm[sm].tx_fifo_full()
    }

    /// DREQ for SM `sm`'s RX FIFO: true when the FIFO has data to drain.
    /// Consumed by the RP2040 DMA matrix for `DREQ_PIO{0,1}_RX{0..3}`.
    #[inline]
    pub fn rx_dreq(&self, sm: usize) -> bool {
        if sm >= self.sm.len() {
            return false;
        }
        !self.sm[sm].rx_fifo_empty()
    }

    /// Read-only view of the 32-entry instruction memory. RP2350
    /// `INSTR_MEM` is write-only via the register interface, so test
    /// harnesses use this accessor to verify programs were loaded.
    pub fn instr_mem(&self) -> &[u16; 32] {
        &self.instr_mem
    }

    /// Test-only: push a word directly into SM `sm`'s RX FIFO. Only
    /// available when the crate is built with `--features test-hooks`
    /// (or under `#[cfg(test)]`). Enables cross-crate tests that need
    /// to stage RX words without reaching into `pub(crate)` state.
    ///
    /// Returns `true` on success, `false` if the FIFO is full.
    #[cfg(any(test, feature = "test-hooks"))]
    pub fn push_rx(&mut self, sm: usize, word: u32) -> bool {
        self.sm[sm].rx_fifo.push(word)
    }

    /// Test-only: pop a word from SM `sm`'s TX FIFO. Symmetric with
    /// [`Self::push_rx`] — enables cross-crate tests that need to
    /// verify TXF push values without reaching into `pub(crate)` state.
    ///
    /// Returns `None` if the FIFO is empty.
    #[cfg(any(test, feature = "test-hooks"))]
    pub fn pop_tx(&mut self, sm: usize) -> Option<u32> {
        self.sm[sm].tx_fifo.pop()
    }

    /// Enable or disable state machine `i`, maintaining the cached
    /// `sm_enabled_mask` invariant. Every enable-state transition
    /// re-merges pin outputs so that a just-disabled SM's stuck pin
    /// bits are cleared on the same tick — this is what makes the
    /// fast-path skip in [`Self::step`] safe when the mask is zero.
    pub fn set_sm_enabled(&mut self, i: usize, enabled: bool) {
        let prev = self.sm[i].enabled;
        if prev == enabled {
            return;
        }
        self.sm[i].enabled = enabled;
        if enabled {
            self.sm_enabled_mask |= 1 << i;
        } else {
            self.sm_enabled_mask &= !(1 << i);
        }
        self.merge_pin_outputs();
    }

    /// Advance PIO block by one system clock.
    pub fn step(&mut self, gpio_in: u32) {
        if self.sm_enabled_mask == 0 {
            return;
        }
        for i in 0..4 {
            if self.sm[i].clock_tick() {
                self.sm[i].execute_cycle(
                    &self.instr_mem,
                    &mut self.irq_flags,
                    gpio_in,
                    &mut self.shared_pin_values,
                    &mut self.shared_pin_dirs,
                );
            }
        }
        self.merge_pin_outputs();
        self.bump_pad_out_diag();
    }

    /// Advance PIO block by `n` system clocks. Quantum-end variant of
    /// [`Self::step`]. Initial implementation is a naive loop — preserves all
    /// cross-cycle state (SM clock divider accumulators, FIFO pressure,
    /// pin-output merging). A bulk-advance optimisation is future work if
    /// PIO appears hot in a flamegraph.
    pub fn step_n(&mut self, n: u32, gpio_in: u32) {
        if self.sm_enabled_mask == 0 {
            return;
        }
        for _ in 0..n {
            self.step(gpio_in);
        }
    }

    /// Merge shared pad latches + per-SM side-set into pad_out/pad_oe.
    /// Non-sideset writes land in `shared_pin_values` / `shared_pin_dirs`
    /// directly — we just copy those. Side-set overlays on top.
    ///
    /// When every SM in the block is disabled, the PIO block isn't
    /// driving any pin (even if a prior program left pindir bits set);
    /// this is the property `disable_clears_pin_outputs` relies on.
    fn merge_pin_outputs(&mut self) {
        if self.sm_enabled_mask == 0 {
            self.pad_out = 0;
            self.pad_oe = 0;
            return;
        }
        let mut out: u32 = self.shared_pin_values;
        let mut oe: u32 = self.shared_pin_dirs;
        for sm in &self.sm {
            if !sm.enabled {
                continue;
            }

            // Side-set pins (separate base/count from PINCTRL)
            let ss_count = ((sm.pinctrl >> 29) & 7) as u8;
            let side_en = (sm.execctrl >> 30) & 1 != 0;
            let actual_ss_pins = if side_en {
                ss_count.saturating_sub(1)
            } else {
                ss_count
            };
            if actual_ss_pins > 0 {
                let ss_base = ((sm.pinctrl >> 10) & 0x1F) as u32;
                let ss_mask = if actual_ss_pins >= 32 {
                    u32::MAX
                } else {
                    (1u32 << actual_ss_pins) - 1
                };
                let positioned_mask = ss_mask.rotate_left(ss_base);

                let side_pindir = (sm.execctrl >> 29) & 1 != 0;
                if side_pindir {
                    // Side-set controls pin directions
                    oe = (oe & !positioned_mask) | (sm.sideset_dirs & positioned_mask);
                } else {
                    // Side-set controls pin values (normal mode). Per RP2350
                    // §11.3.2.3, value-drive side-set does NOT contribute to
                    // pad_oe; pin direction is owned by SET/OUT/MOV PINDIRS
                    // (already merged into `oe` via `shared_pin_dirs` above).
                    out = (out & !positioned_mask) | (sm.sideset_pins & positioned_mask);
                }
            }
        }
        self.pad_out = out;
        self.pad_oe = oe;
    }

    /// Diagnostic: compare the current `pad_out` against the prior
    /// snapshot and bump the PSRAM-SPI transition counters (bit 1=CS,
    /// bit 2=SCK, bit 3=MOSI). Called at the end of each [`Self::step`]
    /// after the pin outputs have been merged, so the counters track
    /// per-sysclock transitions as observed on the block's pad. Pure
    /// observation — never touches execution state. Kept independent
    /// of any downstream device model's own edge counts so the three
    /// numbers can be compared (SM PC visits ↔ pad_out transitions ↔
    /// PSRAM model edges) to localise gaps.
    #[inline]
    fn bump_pad_out_diag(&mut self) {
        let prev = self.prev_pad_out_diag;
        let cur = self.pad_out;
        if prev != cur {
            let prev_cs = (prev >> 1) & 1;
            let cur_cs = (cur >> 1) & 1;
            if prev_cs == 1 && cur_cs == 0 {
                self.pad_out_cs_falls = self.pad_out_cs_falls.wrapping_add(1);
            } else if prev_cs == 0 && cur_cs == 1 {
                self.pad_out_cs_rises = self.pad_out_cs_rises.wrapping_add(1);
            }
            let prev_sck = (prev >> 2) & 1;
            let cur_sck = (cur >> 2) & 1;
            if prev_sck != cur_sck {
                self.pad_out_sck_toggles = self.pad_out_sck_toggles.wrapping_add(1);
            }
        }
        if (cur >> 3) & 1 != 0 {
            self.pad_out_mosi_writes_of_1 =
                self.pad_out_mosi_writes_of_1.wrapping_add(1);
        }
        self.prev_pad_out_diag = cur;
    }

    /// Compute FSTAT register from current SM FIFO states.
    fn fstat(&self) -> u32 {
        let mut val = 0u32;
        for i in 0..4 {
            if self.sm[i].tx_fifo.is_empty() {
                val |= 1 << (24 + i); // TXEMPTY
            }
            if self.sm[i].tx_fifo.is_full() {
                val |= 1 << (16 + i); // TXFULL
            }
            if self.sm[i].rx_fifo.is_empty() {
                val |= 1 << (8 + i); // RXEMPTY
            }
            if self.sm[i].rx_fifo.is_full() {
                val |= 1 << i; // RXFULL
            }
        }
        val
    }

    /// Compute FLEVEL register from current SM FIFO levels.
    fn flevel(&self) -> u32 {
        let mut val = 0u32;
        for i in 0..4 {
            let tx = self.sm[i].tx_fifo.level() as u32;
            let rx = self.sm[i].rx_fifo.level() as u32;
            val |= (tx & 0xF) << (i * 8);
            val |= (rx & 0xF) << (i * 8 + 4);
        }
        val
    }

    /// Apply FIFO joining based on SHIFTCTRL bits for a given SM.
    fn apply_fifo_join(&mut self, sm_idx: usize) {
        let shiftctrl = self.sm[sm_idx].shiftctrl;
        let fjoin_tx = (shiftctrl >> 30) & 1 != 0;
        let fjoin_rx = (shiftctrl >> 31) & 1 != 0;

        if fjoin_tx {
            self.sm[sm_idx].tx_fifo.set_depth(8);
            self.sm[sm_idx].rx_fifo.set_depth(0);
        } else if fjoin_rx {
            self.sm[sm_idx].tx_fifo.set_depth(0);
            self.sm[sm_idx].rx_fifo.set_depth(8);
        } else {
            self.sm[sm_idx].tx_fifo.set_depth(4);
            self.sm[sm_idx].rx_fifo.set_depth(4);
        }
    }

    /// 32-bit register read. `offset` is masked to 12 bits by Bus.
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            // CTRL: only SM_ENABLE bits are readable (restart bits are self-clearing)
            0x000 => {
                let mut val = 0u32;
                for i in 0..4 {
                    if self.sm[i].enabled {
                        val |= 1 << i;
                    }
                }
                val
            }
            0x004 => self.fstat(),
            0x008 => self.fdebug,
            0x00C => self.flevel(),
            // TXF0-3: write-only, reads return 0
            0x010..=0x01C => 0,
            // RXF0-3: pop from SM's RX FIFO
            0x020 => self.sm[0].rx_fifo.pop().unwrap_or(0),
            0x024 => self.sm[1].rx_fifo.pop().unwrap_or(0),
            0x028 => self.sm[2].rx_fifo.pop().unwrap_or(0),
            0x02C => self.sm[3].rx_fifo.pop().unwrap_or(0),
            // IRQ
            0x030 => self.irq_flags as u32,
            // IRQ_FORCE: write-only
            0x034 => 0,
            // INPUT_SYNC_BYPASS
            0x038 => self.input_sync_bypass,
            // DBG_PADOUT
            0x03C => self.pad_out,
            // DBG_PADOE
            0x040 => self.pad_oe,
            // DBG_CFGINFO: 32 IMEM words, 4 SMs, 4 FIFO depth
            0x044 => 0x0020_0404,
            // INSTR_MEM0-31: write-only
            0x048..=0x0C4 => 0,
            // Per-SM registers (stride 0x18, SM0 at 0x0C8)
            0x0C8..=0x127 => self.read_sm_reg(offset),
            // RXFn_PUTGET0..3 (4 SMs × 4 entries, RP2350 offsets 0x128..0x164)
            // and GPIOBASE (0x168): unmodeled, return 0.
            0x128..=0x168 => 0,
            // INTR: raw interrupt status (read-only, 16 bits). RP2350 offset 0x16C.
            0x16C => self.raw_intr_rp2350(),
            // IRQ0_INTE. RP2350 offset 0x170.
            0x170 => self.int0_inte,
            // IRQ0_INTF. RP2350 offset 0x174.
            0x174 => self.int0_intf,
            // IRQ0_INTS: effective status = (INTR & INTE) | INTF. RP2350 offset 0x178.
            0x178 => self.int0_ints_rp2350(),
            // IRQ1_INTE. RP2350 offset 0x17C.
            0x17C => self.int1_inte,
            // IRQ1_INTF. RP2350 offset 0x180.
            0x180 => self.int1_intf,
            // IRQ1_INTS: effective status = (INTR & INTE) | INTF. RP2350 offset 0x184.
            0x184 => self.int1_ints_rp2350(),
            _ => 0,
        }
    }

    /// 32-bit register write. `offset` is masked to 12 bits by Bus.
    /// `alias`: 0=normal, 1=XOR, 2=SET (OR), 3=CLR (AND NOT).
    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        match offset {
            0x000 => self.write_ctrl(val, alias),
            // FSTAT: read-only
            0x004 => {}
            // FDEBUG: W1C (or alias)
            0x008 => {
                let mask = match alias {
                    0 | 3 => val,  // normal write and CLR both clear bits
                    1 => val,      // XOR
                    2 => val,      // SET
                    _ => return,
                };
                match alias {
                    0 => self.fdebug &= !mask,  // W1C: writing 1 clears
                    1 => self.fdebug ^= mask,
                    2 => self.fdebug |= mask,
                    3 => self.fdebug &= !mask,
                    _ => {}
                }
            }
            // FLEVEL: read-only
            0x00C => {}
            // TXF0-3: push to SM's TX FIFO
            0x010 => { self.sm[0].tx_fifo.push(val); }
            0x014 => { self.sm[1].tx_fifo.push(val); }
            0x018 => { self.sm[2].tx_fifo.push(val); }
            0x01C => { self.sm[3].tx_fifo.push(val); }
            // RXF0-3: read-only
            0x020..=0x02C => {}
            // IRQ: W1C (or alias)
            0x030 => {
                match alias {
                    0 => self.irq_flags &= !(val as u8),  // W1C
                    1 => self.irq_flags ^= val as u8,
                    2 => self.irq_flags |= val as u8,
                    3 => self.irq_flags &= !(val as u8),
                    _ => {}
                }
            }
            // IRQ_FORCE: set bits in irq_flags
            0x034 => {
                self.irq_flags |= val as u8;
            }
            // INPUT_SYNC_BYPASS
            0x038 => { self.input_sync_bypass = val; }
            // DBG_PADOUT, DBG_PADOE, DBG_CFGINFO: read-only
            0x03C..=0x044 => {}
            // INSTR_MEM0-31
            0x048..=0x0C4 => {
                let idx = ((offset - 0x048) >> 2) as usize;
                if idx < 32 {
                    self.instr_mem[idx] = val as u16;
                }
            }
            // Per-SM registers
            0x0C8..=0x127 => self.write_sm_reg(offset, val, alias),
            // RXFn_PUTGET0..3 and GPIOBASE (0x128..0x168): unmodeled, ignore writes.
            0x128..=0x168 => {}
            // INTR: read-only
            0x16C => {}
            // IRQ0_INTE (16-bit mask, alias-aware). RP2350 offset 0x170.
            0x170 => {
                let mask = val & 0xFFFF;
                match alias {
                    0 => self.int0_inte = mask,
                    1 => self.int0_inte ^= mask,
                    2 => self.int0_inte |= mask,
                    3 => self.int0_inte &= !mask,
                    _ => {}
                }
            }
            // IRQ0_INTF (16-bit force, alias-aware). RP2350 offset 0x174.
            0x174 => {
                let mask = val & 0xFFFF;
                match alias {
                    0 => self.int0_intf = mask,
                    1 => self.int0_intf ^= mask,
                    2 => self.int0_intf |= mask,
                    3 => self.int0_intf &= !mask,
                    _ => {}
                }
            }
            // IRQ0_INTS: read-only
            0x178 => {}
            // IRQ1_INTE (16-bit mask, alias-aware). RP2350 offset 0x17C.
            0x17C => {
                let mask = val & 0xFFFF;
                match alias {
                    0 => self.int1_inte = mask,
                    1 => self.int1_inte ^= mask,
                    2 => self.int1_inte |= mask,
                    3 => self.int1_inte &= !mask,
                    _ => {}
                }
            }
            // IRQ1_INTF (16-bit force, alias-aware). RP2350 offset 0x180.
            0x180 => {
                let mask = val & 0xFFFF;
                match alias {
                    0 => self.int1_intf = mask,
                    1 => self.int1_intf ^= mask,
                    2 => self.int1_intf |= mask,
                    3 => self.int1_intf &= !mask,
                    _ => {}
                }
            }
            // IRQ1_INTS: read-only
            0x184 => {}
            _ => {}
        }
    }

    /// Read per-SM register.
    fn read_sm_reg(&self, offset: u32) -> u32 {
        let sm_offset = offset - 0x0C8;
        let sm_idx = (sm_offset / 0x18) as usize;
        let reg = sm_offset % 0x18;
        if sm_idx >= 4 {
            return 0;
        }
        let sm = &self.sm[sm_idx];
        match reg {
            // SMn_CLKDIV
            0x00 => sm.read_clkdiv(),
            // SMn_EXECCTRL: bit 31 is EXEC_STALLED (read-only)
            0x04 => {
                let stalled = sm.stalled || sm.delay_count > 0;
                (sm.execctrl & 0x7FFF_FFFF) | ((stalled as u32) << 31)
            }
            // SMn_SHIFTCTRL
            0x08 => sm.shiftctrl,
            // SMn_ADDR: current PC
            0x0C => sm.pc as u32,
            // SMn_INSTR: last executed instruction
            0x10 => sm.last_insn as u32,
            // SMn_PINCTRL
            0x14 => sm.pinctrl,
            _ => 0,
        }
    }

    /// Write per-SM register, honouring the four-way APB alias dispatch
    /// (`alias=0`=plain, 1=XOR, 2=SET, 3=CLR). All five storage-backed
    /// per-SM registers (CLKDIV, EXECCTRL, SHIFTCTRL, INSTR, PINCTRL) are
    /// alias-aware. Read-only registers (ADDR) ignore writes regardless.
    ///
    /// PicoGUS firmware exercises XOR aliases on SHIFTCTRL to flip
    /// FJOIN_TX without disturbing the rest of the register; treating
    /// aliases as plain writes here silently corrupts AUTOPUSH/PUSH_THRESH.
    fn write_sm_reg(&mut self, offset: u32, val: u32, alias: u32) {
        let sm_offset = offset - 0x0C8;
        let sm_idx = (sm_offset / 0x18) as usize;
        let reg = sm_offset % 0x18;
        if sm_idx >= 4 {
            return;
        }
        match reg {
            // SMn_CLKDIV: pack/unpack through int/frac so alias-RMW
            // semantics apply to the canonical 32-bit register layout.
            0x00 => {
                let mut current = self.sm[sm_idx].read_clkdiv();
                Self::apply_alias_rmw(&mut current, val, alias);
                self.sm[sm_idx].write_clkdiv(current);
            }
            // SMn_EXECCTRL: bit 31 is read-only (EXEC_STALLED). Mask it
            // out of the alias operand so `SET`/`XOR` of bit 31 cannot
            // poison the stored value.
            0x04 => {
                let mut current = self.sm[sm_idx].execctrl;
                Self::apply_alias_rmw(&mut current, val & 0x7FFF_FFFF, alias);
                self.sm[sm_idx].execctrl = current & 0x7FFF_FFFF;
            }
            // SMn_SHIFTCTRL: reconfigure FIFO joining when the FJOIN bits
            // [31:30] change after the alias is applied.
            0x08 => {
                let old_join = self.sm[sm_idx].shiftctrl & 0xC000_0000;
                let mut current = self.sm[sm_idx].shiftctrl;
                Self::apply_alias_rmw(&mut current, val, alias);
                self.sm[sm_idx].shiftctrl = current;
                let new_join = current & 0xC000_0000;
                if old_join != new_join {
                    self.apply_fifo_join(sm_idx);
                }
            }
            // SMn_ADDR: read-only
            0x0C => {}
            // SMn_INSTR: force-execute. Apply the alias against the last
            // executed instruction (the register's only readable storage)
            // so an aliased write force-executes the RMW result.
            0x10 => {
                let mut current = self.sm[sm_idx].last_insn as u32;
                Self::apply_alias_rmw(&mut current, val, alias);
                let insn = current as u16;
                self.sm[sm_idx].force_execute(
                    insn,
                    &self.instr_mem,
                    &mut self.irq_flags,
                    0, // gpio_in not available in register write — use 0
                    &mut self.shared_pin_values,
                    &mut self.shared_pin_dirs,
                );
            }
            // SMn_PINCTRL
            0x14 => {
                let mut current = self.sm[sm_idx].pinctrl;
                Self::apply_alias_rmw(&mut current, val, alias);
                self.sm[sm_idx].pinctrl = current;
            }
            _ => {}
        }
    }

    /// Apply APB alias semantics to a stored register field.
    /// Mirrors `mdrp2040::peripherals::apply_alias_rmw`, inlined here
    /// because `mdpicoem-common` cannot depend on `mdrp2040`.
    #[inline]
    fn apply_alias_rmw(stored: &mut u32, value: u32, alias: u32) {
        match alias {
            0 => *stored = value,
            1 => *stored ^= value,
            2 => *stored |= value,
            3 => *stored &= !value,
            _ => {}
        }
    }

    /// Write CTRL register with alias support.
    fn write_ctrl(&mut self, val: u32, alias: u32) {
        let sm_enable_bits = val & 0xF;
        let sm_restart_bits = (val >> 4) & 0xF;
        let clkdiv_restart_bits = (val >> 8) & 0xF;

        // SM_ENABLE: apply alias logic
        match alias {
            0 => {
                // Normal write: set SM_ENABLE directly
                for i in 0..4 {
                    self.set_sm_enabled(i, (sm_enable_bits >> i) & 1 != 0);
                }
            }
            1 => {
                // XOR
                for i in 0..4 {
                    if (sm_enable_bits >> i) & 1 != 0 {
                        // Read current state before the call to sidestep
                        // the &mut self borrow inside set_sm_enabled.
                        let toggled = !self.sm[i].enabled;
                        self.set_sm_enabled(i, toggled);
                    }
                }
            }
            2 => {
                // SET (OR): enable indicated SMs
                for i in 0..4 {
                    if (sm_enable_bits >> i) & 1 != 0 {
                        self.set_sm_enabled(i, true);
                    }
                }
            }
            3 => {
                // CLR (AND NOT): disable indicated SMs
                for i in 0..4 {
                    if (sm_enable_bits >> i) & 1 != 0 {
                        self.set_sm_enabled(i, false);
                    }
                }
            }
            _ => {}
        }

        // SM_RESTART: self-clearing action (reset SM state)
        for i in 0..4 {
            if (sm_restart_bits >> i) & 1 != 0 {
                self.sm[i].pc = 0;
                self.sm[i].x = 0;
                self.sm[i].y = 0;
                self.sm[i].isr = 0;
                self.sm[i].osr = 0;
                self.sm[i].isr_count = 0;
                self.sm[i].osr_count = 0;
                self.sm[i].delay_count = 0;
                self.sm[i].stalled = false;
                self.sm[i].pending_exec = None;
                self.sm[i].stall_kind = StallKind::None;
            }
        }

        // CLKDIV_RESTART: self-clearing action (reset clock divider accumulator)
        for i in 0..4 {
            if (clkdiv_restart_bits >> i) & 1 != 0 {
                self.sm[i].clkdiv_acc = 0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_illegal_sideset_count_no_panic() {
        // PINCTRL with SIDESET_COUNT=7 (illegal) — should not panic
        let pinctrl = 0xE000_0000; // bits [31:29] = 111 = 7
        let insn = 0xE001; // SET PINS, 1
        let _decoded = crate::pio::decode::decode(insn, pinctrl, 0);
        // If we got here without panic, the test passes
    }

    #[test]
    fn test_sm_reset_values() {
        let sm = StateMachine::new();
        assert_eq!(sm.execctrl, 0x0001_F000, "EXECCTRL reset: wrap_top=31");
        assert_eq!(sm.shiftctrl, 0x000C_0000, "SHIFTCTRL reset: thresholds=0 (32)");
        assert_eq!(sm.pinctrl, 0x1400_0000, "PINCTRL reset: SET_COUNT=5");
        assert_eq!(sm.clkdiv_int, 1, "CLKDIV int reset: 1");
        assert_eq!(sm.clkdiv_frac, 0, "CLKDIV frac reset: 0");
        assert_eq!(sm.read_clkdiv(), 0x0001_0000, "CLKDIV register: 0x0001_0000");
    }

    #[test]
    fn test_register_roundtrip_clkdiv() {
        let mut pio = PioBlock::new();
        // Write CLKDIV for SM0: int=1302, frac=128
        let clkdiv_val = (1302u32 << 16) | (128u32 << 8);
        pio.write32(0x0C8, clkdiv_val, 0); // SM0 CLKDIV
        assert_eq!(pio.read32(0x0C8), clkdiv_val);
    }

    #[test]
    fn test_register_roundtrip_execctrl() {
        let mut pio = PioBlock::new();
        // Write EXECCTRL for SM0 with bit 31 set — bit 31 should be masked (read-only)
        pio.write32(0x0CC, 0xFFFF_FFFF, 0); // SM0 EXECCTRL
        let read_back = pio.read32(0x0CC);
        // Bit 31 is EXEC_STALLED (read-only), reflects sm.stalled || delay > 0
        // SM is not stalled and delay_count=0, so bit 31 should be 0
        assert_eq!(read_back & 0x8000_0000, 0, "bit 31 is read-only EXEC_STALLED");
        assert_eq!(read_back & 0x7FFF_FFFF, 0x7FFF_FFFF, "bits 30:0 roundtrip");
    }

    #[test]
    fn test_register_roundtrip_shiftctrl() {
        let mut pio = PioBlock::new();
        let val = 0xDEAD_BEEF;
        pio.write32(0x0D0, val, 0); // SM0 SHIFTCTRL
        assert_eq!(pio.read32(0x0D0), val);
    }

    /// Per-SM XOR alias must flip the targeted bits in SHIFTCTRL rather than
    /// silently clobbering the register with a plain write.
    ///
    /// PicoGUS firmware programs SHIFTCTRL=0x012b0000 then issues two
    /// `SHIFTCTRL_XOR` writes of 0x80000000 to toggle FJOIN_TX off and on
    /// (a no-op pair). If the alias parameter is dropped on the per-SM
    /// dispatch path, the second write leaves SHIFTCTRL at 0x80000000,
    /// which silently disables AUTOPUSH and PUSH_THRESH and breaks audio
    /// ingestion.
    #[test]
    fn per_sm_xor_alias_flips_bits_in_shiftctrl() {
        let mut pio = PioBlock::new();
        // 1. Plain write of the production SHIFTCTRL value.
        pio.write32(0x0D0, 0x012b_0000, 0);
        assert_eq!(pio.read32(0x0D0), 0x012b_0000, "plain write roundtrip");

        // 2. XOR-flip bit 31.
        pio.write32(0x0D0, 0x8000_0000, 1);
        assert_eq!(
            pio.read32(0x0D0),
            0x812b_0000,
            "XOR alias must flip bit 31, leaving the rest intact",
        );

        // 3. XOR again — bit 31 toggles back, the rest still untouched.
        pio.write32(0x0D0, 0x8000_0000, 1);
        assert_eq!(
            pio.read32(0x0D0),
            0x012b_0000,
            "second XOR alias write must restore the original value",
        );
    }

    #[test]
    fn test_register_roundtrip_pinctrl() {
        let mut pio = PioBlock::new();
        let val = 0xABCD_1234;
        pio.write32(0x0DC, val, 0); // SM0 PINCTRL
        assert_eq!(pio.read32(0x0DC), val);
    }

    #[test]
    fn test_ctrl_enable_disable() {
        let mut pio = PioBlock::new();
        // Enable SM0
        pio.write32(0x000, 0x1, 0);
        assert!(pio.sm[0].enabled);
        assert!(!pio.sm[1].enabled);
        // Read back CTRL: only SM_ENABLE bits
        assert_eq!(pio.read32(0x000), 0x1);

        // Disable SM0
        pio.write32(0x000, 0x0, 0);
        assert!(!pio.sm[0].enabled);
        assert_eq!(pio.read32(0x000), 0x0);

        // Enable SM0 and SM2
        pio.write32(0x000, 0x5, 0);
        assert!(pio.sm[0].enabled);
        assert!(!pio.sm[1].enabled);
        assert!(pio.sm[2].enabled);
        assert!(!pio.sm[3].enabled);
    }

    #[test]
    fn test_ctrl_restart_self_clearing() {
        let mut pio = PioBlock::new();
        // Enable SM0 and set some state
        pio.set_sm_enabled(0, true);
        pio.sm[0].pc = 15;
        pio.sm[0].x = 0x1234;

        // Write SM_RESTART for SM0 (bit 4) + keep SM0 enabled (bit 0)
        pio.write32(0x000, 0x11, 0);

        // SM0 should be enabled (bit 0 written)
        assert!(pio.sm[0].enabled);
        // SM0 state should be reset by restart
        assert_eq!(pio.sm[0].pc, 0);
        assert_eq!(pio.sm[0].x, 0);

        // Read back CTRL: restart bits are self-clearing, should read 0
        let ctrl = pio.read32(0x000);
        assert_eq!(ctrl & 0xF0, 0, "SM_RESTART bits read as 0");
        assert_eq!(ctrl & 0xF, 0x1, "SM_ENABLE bits persist");
    }

    #[test]
    fn test_instr_mem_write() {
        let mut pio = PioBlock::new();
        for i in 0..32u32 {
            pio.write32(0x048 + i * 4, 0xA000 + i, 0);
        }
        for i in 0..32 {
            assert_eq!(pio.instr_mem[i], 0xA000 + i as u16);
        }
    }

    #[test]
    fn test_fifo_push_pop() {
        let mut pio = PioBlock::new();
        // Push via TXF0
        pio.write32(0x010, 0xDEAD_BEEF, 0);

        // FSTAT: TX should not be empty for SM0
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & (1 << 24), 0, "SM0 TX not empty");

        // Pop from RXF0 — but wait, TXF pushes to TX FIFO, RXF pops from RX FIFO.
        // In the real PIO, data flows TX -> SM -> RX. For register-level testing,
        // push to TX and verify TX FIFO state, then manually push to RX and pop.
        // Let's verify TX state via FSTAT, then directly push to RX for pop test.
        pio.sm[0].rx_fifo.push(0xCAFE_BABE);
        let val = pio.read32(0x020);
        assert_eq!(val, 0xCAFE_BABE);
    }

    #[test]
    fn test_fifo_full_and_overflow() {
        let mut pio = PioBlock::new();
        // Push 4 values to SM0 TX FIFO
        for i in 0..4 {
            assert!(pio.sm[0].tx_fifo.push(i + 1));
        }
        assert!(pio.sm[0].tx_fifo.is_full());

        // 5th push should fail
        assert!(!pio.sm[0].tx_fifo.push(5));

        // FSTAT: TXFULL bit for SM0
        let fstat = pio.read32(0x004);
        assert_ne!(fstat & (1 << 16), 0, "SM0 TX full");
    }

    #[test]
    fn test_fifo_joining() {
        let mut pio = PioBlock::new();

        // Set FJOIN_TX in SHIFTCTRL for SM0 (bit 30)
        pio.write32(0x0D0, pio.sm[0].shiftctrl | (1 << 30), 0);

        // TX FIFO should now accept 8 values
        for i in 0..8 {
            assert!(pio.sm[0].tx_fifo.push(i + 1), "push {} should succeed", i);
        }
        assert!(pio.sm[0].tx_fifo.is_full(), "TX FIFO full at 8");
        assert!(!pio.sm[0].tx_fifo.push(9), "push 9 should fail");

        // RX FIFO should be depth 0 (unavailable): pop returns None
        assert_eq!(pio.sm[0].rx_fifo.pop(), None);
    }

    #[test]
    fn test_fstat_flags() {
        let mut pio = PioBlock::new();

        // Initially: TX empty, RX empty for all SMs
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & 0x0F00_0000, 0x0F00_0000, "all TX empty");
        assert_eq!(fstat & 0x0000_0F00, 0x0000_0F00, "all RX empty");
        assert_eq!(fstat & 0x000F_0000, 0, "no TX full");
        assert_eq!(fstat & 0x0000_000F, 0, "no RX full");

        // Push one value to SM0 TX
        pio.write32(0x010, 42, 0);
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & (1 << 24), 0, "SM0 TX not empty");
        assert_ne!(fstat & (1 << 25), 0, "SM1 TX still empty");

        // Fill SM1 TX FIFO
        for _ in 0..4 {
            pio.sm[1].tx_fifo.push(0);
        }
        let fstat = pio.read32(0x004);
        assert_ne!(fstat & (1 << 17), 0, "SM1 TX full");

        // Push to SM2 RX FIFO
        pio.sm[2].rx_fifo.push(0);
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & (1 << 10), 0, "SM2 RX not empty");
    }

    #[test]
    fn test_flevel() {
        let mut pio = PioBlock::new();

        // Push 2 to SM0 TX, 3 to SM1 RX
        pio.sm[0].tx_fifo.push(1);
        pio.sm[0].tx_fifo.push(2);
        pio.sm[1].rx_fifo.push(10);
        pio.sm[1].rx_fifo.push(20);
        pio.sm[1].rx_fifo.push(30);

        let flevel = pio.read32(0x00C);
        // SM0 TX level = 2 at bits [3:0]
        assert_eq!(flevel & 0xF, 2);
        // SM0 RX level = 0 at bits [7:4]
        assert_eq!((flevel >> 4) & 0xF, 0);
        // SM1 TX level = 0 at bits [11:8]
        assert_eq!((flevel >> 8) & 0xF, 0);
        // SM1 RX level = 3 at bits [15:12]
        assert_eq!((flevel >> 12) & 0xF, 3);
    }

    #[test]
    fn test_irq_force_and_w1c() {
        let mut pio = PioBlock::new();

        // Force IRQ bits 0, 2, 5
        pio.write32(0x034, 0x25, 0);
        assert_eq!(pio.irq_flags, 0x25);
        assert_eq!(pio.read32(0x030), 0x25);

        // W1C: clear bit 2 by writing 1 to bit 2
        pio.write32(0x030, 0x04, 0);
        assert_eq!(pio.irq_flags, 0x21);
        assert_eq!(pio.read32(0x030), 0x21);

        // Clear remaining
        pio.write32(0x030, 0x21, 0);
        assert_eq!(pio.irq_flags, 0);
    }

    #[test]
    fn test_dbg_cfginfo() {
        let mut pio = PioBlock::new();
        assert_eq!(pio.read32(0x044), 0x0020_0404);
    }

    // Bus-dispatch tests (PIO0/PIO1/PIO2 base addresses, CTRL alias
    // SET/CLR) are RP2350-specific and live in
    // `crates/mdrp2350/src/pio_tests.rs` because they exercise the chip
    // bus's address decode.

    #[test]
    fn test_pio_reset() {
        let mut pio = PioBlock::new();

        // Dirty up state
        pio.set_sm_enabled(0, true);
        pio.sm[0].pc = 10;
        pio.sm[0].x = 0xDEAD;
        pio.sm[1].tx_fifo.push(42);
        pio.instr_mem[5] = 0xFFFF;
        pio.irq_flags = 0xFF;
        pio.fdebug = 0x1234;
        pio.pad_out = 0xABCD;

        pio.reset();

        assert!(!pio.sm[0].enabled);
        assert_eq!(pio.sm[0].pc, 0);
        assert_eq!(pio.sm[0].x, 0);
        assert_eq!(pio.sm[0].execctrl, 0x0001_F000, "reset restores default EXECCTRL");
        assert!(pio.sm[1].tx_fifo.is_empty());
        assert_eq!(pio.instr_mem[5], 0);
        assert_eq!(pio.irq_flags, 0);
        assert_eq!(pio.fdebug, 0);
        assert_eq!(pio.pad_out, 0);
    }

    /// Load a tiny `SET PINS, data=val` / `SET PINS, 0` alternating
    /// program into SM0's instr_mem, enable SM0, and return the block
    /// so a test can drive transitions through the `step()` path. Uses
    /// default pinctrl (set_base=0, set_count=5) so SET writes land on
    /// pad bits [4:0] — letting the pad_out transition counters see
    /// bit 1 (CS) and bit 2 (SCK) directly.
    fn make_pio_for_set_pins_test(high_data: u8) -> PioBlock {
        let mut pio = PioBlock::new();
        // SET PINS, high_data (opcode=111, dst=0, data=high_data)
        pio.instr_mem[0] = 0xE000 | ((high_data as u16) & 0x1F);
        // SET PINS, 0 (opcode=111, dst=0, data=0)
        pio.instr_mem[1] = 0xE000;
        // Wrap: execctrl default has wrap_top=0x1F, wrap_bottom=0 —
        // after slot 1 advance_pc→2, but we only step twice so we never
        // reach slot 2. Default is fine.
        pio.set_sm_enabled(0, true);
        pio
    }

    #[test]
    fn pad_out_cs_fall_counter_bumps_on_1_to_0() {
        // First step: SET PINS, 2 → pad bit 1 goes 0 → 1 (a rise).
        // Second step: SET PINS, 0 → pad bit 1 goes 1 → 0 (a fall).
        let mut pio = make_pio_for_set_pins_test(0b00010);
        assert_eq!(pio.pad_out_cs_falls, 0);
        pio.step(0);
        // After one step: pad bit 1 is high. No fall yet.
        assert_eq!(pio.pad_out_cs_falls, 0);
        assert_eq!((pio.pad_out >> 1) & 1, 1, "pad bit 1 high after SET 2");
        pio.step(0);
        // After the second step: pad bit 1 goes low — one fall.
        assert_eq!((pio.pad_out >> 1) & 1, 0, "pad bit 1 low after SET 0");
        assert_eq!(pio.pad_out_cs_falls, 1, "one 1→0 transition on bit 1");
    }

    #[test]
    fn pad_out_cs_rise_counter_independent() {
        // Same program; after the first step we expect exactly one rise
        // and zero falls (asymmetric with the fall-only test above).
        let mut pio = make_pio_for_set_pins_test(0b00010);
        assert_eq!(pio.pad_out_cs_rises, 0);
        assert_eq!(pio.pad_out_cs_falls, 0);
        pio.step(0);
        assert_eq!(pio.pad_out_cs_rises, 1, "one 0→1 transition on bit 1");
        assert_eq!(pio.pad_out_cs_falls, 0, "no fall yet");
        // Bit 2 (SCK) never toggles under this program.
        assert_eq!(pio.pad_out_sck_toggles, 0);
        // Bit 3 (MOSI) stays low.
        assert_eq!(pio.pad_out_mosi_writes_of_1, 0);
    }

    // `test_gpio_in_moved_to_bus` lives in `crates/mdrp2350/src/pio_tests.rs`
    // — it exercises the chip `Bus`'s SIO GPIO_IN mirror.

    /// RP2350 silicon oracle regression: offset 0x134 is RXF0_PUTGET3
    /// (unmodeled FIFO debug access), NOT IRQ0_INTS.  IRQ0_INTS lives at
    /// 0x178 per the RP2350 datasheet (Table 1021).  Before this fix the
    /// emulator exposed IRQ0_INTS at 0x134, causing the
    /// `pio0_int_routing_split` silicon scenario to diverge (EMU=0x001,
    /// HW=0x987 at 0x50200134).
    ///
    /// Also validates the corrected bit layout of INTR / IRQ0_INTS:
    ///   bit 8 = SM0 IRQ flag 0 (RP2350 ds Table 1018 — "SM0" at bit 8)
    ///   bit 4 = SM0_TXNFULL
    ///   bit 0 = SM0_RXNEMPTY
    #[test]
    fn pio_int_registers_at_rp2350_offsets() {
        let mut pio = PioBlock::new();

        // 0x134 must NOT be IRQ0_INTS — it is RXF0_PUTGET3 (unmodeled → 0).
        assert_eq!(pio.read32(0x134), 0,
            "0x134 is RXF0_PUTGET3 on RP2350, must return 0");

        // Inject IRQ flag 0 via the IRQ_FORCE register (offset 0x034).
        pio.write32(0x034, 0x01, 0);
        assert_eq!(pio.irq_flags, 0x01, "irq_flags bit 0 set by IRQ_FORCE");

        // INTR at 0x16C must expose IRQ flag 0 at bit 8 (not bit 0).
        let intr = pio.read32(0x16C);
        assert_ne!(intr & (1 << 8), 0,
            "INTR at 0x16C: SM0 IRQ flag 0 must appear at bit 8");
        assert_eq!(intr & 0x1, 0,
            "INTR at 0x16C: bit 0 (SM0_RXNEMPTY) must be 0 when RX FIFO empty");

        // IRQ0_INTE at 0x170: write 0x100 (bit 8 = SM0/IRQ flag 0).
        pio.write32(0x170, 0x100, 0);
        assert_eq!(pio.int0_inte, 0x100, "int0_inte set via offset 0x170");

        // IRQ0_INTS at 0x178: (INTR & INTE) | INTF — must return 0x100.
        let ints = pio.read32(0x178);
        assert_ne!(ints & 0x100, 0,
            "IRQ0_INTS at 0x178 must show SM0 IRQ flag (bit 8) when INTE bit 8 set");

        // IRQ1_INTE at 0x17C: bit 9 (SM1/IRQ flag 1), SM0 never set it.
        pio.write32(0x17C, 0x200, 0);
        let ints1 = pio.read32(0x184);  // IRQ1_INTS
        assert_eq!(ints1 & 0x200, 0,
            "IRQ1_INTS must be 0: SM0 never set IRQ flag 1");
    }

    /// RP2040 INTR bit layout: IRQ flags at [3:0], RXNEMPTY at [7:4],
    /// TXNFULL at [11:8] — 12-bit register (RP2040 ds Table 358).
    /// IRQ flag 0 → bit 0. Asserts that `raw_intr_rp2040` returns 0x001.
    #[test]
    fn raw_intr_rp2040_layout() {
        let mut pio = PioBlock::new();
        pio.write32(0x034, 0x01, 0);
        assert_eq!(pio.irq_flags, 0x01, "irq_flags bit 0 must be set");
        let intr = pio.raw_intr_rp2040();
        assert_eq!(intr & 0x001, 0x001,
            "raw_intr_rp2040: IRQ flag 0 must appear at bit 0 (RP2040 ds Table 358)");
        assert_eq!(intr >> 12, 0,
            "raw_intr_rp2040: bits [31:12] must be zero (12-bit register)");
    }

    /// RP2350 INTR bit layout: IRQ flags at [15:8], TXNFULL at [7:4],
    /// RXNEMPTY at [3:0] — 16-bit register (RP2350 ds Table 1018).
    /// IRQ flag 0 → bit 8. Asserts that `raw_intr_rp2350` returns 0x100.
    #[test]
    fn raw_intr_rp2350_layout() {
        let mut pio = PioBlock::new();
        pio.write32(0x034, 0x01, 0);
        assert_eq!(pio.irq_flags, 0x01, "irq_flags bit 0 must be set");
        let intr = pio.raw_intr_rp2350();
        assert_ne!(intr & 0x100, 0,
            "raw_intr_rp2350: IRQ flag 0 must appear at bit 8 (RP2350 ds Table 1018)");
        assert_eq!(intr & 0x001, 0,
            "raw_intr_rp2350: bit 0 (SM0_RXNEMPTY) must be 0 when RX FIFO empty");
    }

    // ---- Stage B: Clock divider tests ----

    #[test]
    fn test_clock_div_1() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 0;
        // Should tick every cycle
        let mut ticks = 0;
        for _ in 0..1000 {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 1000);
    }

    #[test]
    fn test_clock_div_2() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 2;
        pio.sm[0].clkdiv_frac = 0;
        let mut ticks = 0;
        for _ in 0..1000 {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 500);
    }

    #[test]
    fn test_clock_div_1_frac_128() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 128;
        // Threshold = 256 + 128 = 384
        // Average period = 384/256 = 1.5 cycles
        // Over 3 cycles: 2 ticks (768/384 = 2)
        let mut ticks = 0;
        for _ in 0..3000 {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 2000, "1.5x divider: 2 ticks per 3 cycles");
    }

    #[test]
    fn test_clock_div_large() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1000;
        pio.sm[0].clkdiv_frac = 0;
        let mut ticks = 0;
        for _ in 0..10000 {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 10);
    }

    // ---- Stage B: Decoder tests ----

    #[test]
    fn test_decode_jmp() {
        use super::decode::{decode, PioOp};
        // JMP always to addr 5: opcode=000, delay/ss=00000, cond=000, addr=00101
        // insn = 0b000_00000_000_00101 = 0x0005
        let d = decode(0x0005, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::Jmp { condition, address } => {
                assert_eq!(condition, 0, "JMP always");
                assert_eq!(address, 5);
            }
            _ => panic!("expected JMP"),
        }
        assert_eq!(d.delay, 0);
        assert!(d.sideset.is_none());
    }

    #[test]
    fn test_decode_set() {
        use super::decode::{decode, PioOp};
        // SET PINS 0x1F: opcode=111, delay/ss=00000, dest=000, data=11111
        // insn = 0b111_00000_000_11111 = 0xE01F
        let d = decode(0xE01F, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::Set { destination, data } => {
                assert_eq!(destination, 0, "SET PINS");
                assert_eq!(data, 0x1F);
            }
            _ => panic!("expected SET"),
        }
        assert_eq!(d.delay, 0);
    }

    #[test]
    fn test_decode_sideset_delay_split() {
        use super::decode::{decode, PioOp};
        // PINCTRL with sideset_count=2 (bits[31:29]=010)
        let pinctrl = 0x1400_0000 | (2u32 << 29);
        // SET X, 5 with sideset_val=3, delay=6
        // sideset_count=2, delay_bits=3
        // delay/ss field: [ss1 ss0 d2 d1 d0] = [1 1 1 1 0] = 0b11_110 = 30
        // opcode=111, dest=001(X), data=00101(5)
        // insn = 0b111_11110_001_00101 = 0xFE25
        let d = decode(0xFE25, pinctrl, 0x0001_F000);
        match d.op {
            PioOp::Set { destination, data } => {
                assert_eq!(destination, 1, "SET X");
                assert_eq!(data, 5);
            }
            _ => panic!("expected SET"),
        }
        assert_eq!(d.delay, 6, "delay=bottom 3 bits of 0b11110 = 110 = 6");
        assert_eq!(d.sideset, Some(3), "sideset=top 2 bits of 0b11110 = 11 = 3");
    }

    // ---- Stage B: Instruction execution tests ----

    /// Helper: create a PIO block with a program loaded, SM0 enabled at div-1.
    fn make_pio_with_program(program: &[u16]) -> PioBlock {
        let mut pio = PioBlock::new();
        for (i, &insn) in program.iter().enumerate() {
            pio.instr_mem[i] = insn;
        }
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 0;
        pio
    }

    /// Step SM0 for N PIO ticks (at div-1, each system clock = 1 PIO tick).
    fn step_n(pio: &mut PioBlock, n: usize, gpio_in: u32) {
        for _ in 0..n {
            pio.step(gpio_in);
        }
    }

    #[test]
    fn test_set_x_y() {
        // SET X, 15; SET Y, 7
        // SET X = opcode 111, dest 001(X), data 01111 => 0b111_00000_001_01111 = 0xE02F
        // SET Y = opcode 111, dest 010(Y), data 00111 => 0b111_00000_010_00111 = 0xE047
        let mut pio = make_pio_with_program(&[0xE02F, 0xE047]);
        step_n(&mut pio, 1, 0); // SET X, 15
        assert_eq!(pio.sm[0].x, 15);
        step_n(&mut pio, 1, 0); // SET Y, 7
        assert_eq!(pio.sm[0].y, 7);
    }

    #[test]
    fn test_jmp_always() {
        // JMP 3: opcode=000, cond=000, addr=00011 => 0x0003
        let mut pio = make_pio_with_program(&[0x0003]);
        step_n(&mut pio, 1, 0);
        assert_eq!(pio.sm[0].pc, 3);
    }

    #[test]
    fn test_jmp_x_decrement() {
        // SET X, 2; JMP X-- 0
        // SET X, 2 = 0xE022 (dest=001, data=00010)
        // JMP X-- 0 = opcode 000, cond=010, addr=00000 => 0b000_00000_010_00000 = 0x0040
        let mut pio = make_pio_with_program(&[0xE022, 0x0040]);
        step_n(&mut pio, 1, 0); // SET X, 2 => x=2, pc -> 1
        assert_eq!(pio.sm[0].x, 2);
        step_n(&mut pio, 1, 0); // JMP X-- 0 => x was 2 (nonzero), dec to 1, jump to 0
        assert_eq!(pio.sm[0].x, 1);
        assert_eq!(pio.sm[0].pc, 0);
        step_n(&mut pio, 1, 0); // SET X, 2 again => x=2
        // skip to JMP again
        step_n(&mut pio, 1, 0); // JMP X-- 0 => x was 2, dec to 1, jump
        assert_eq!(pio.sm[0].x, 1);
        assert_eq!(pio.sm[0].pc, 0);
    }

    #[test]
    fn test_wrap() {
        // Set wrap_top=2, wrap_bottom=0: program wraps from addr 2 -> 0
        // EXECCTRL: wrap_top[16:12]=00010, wrap_bottom[11:7]=00000
        // wrap_top=2 => bits[16:12] = 0b00010 => 0x2000
        // wrap_bottom=0 => bits[11:7] = 0
        let execctrl = (2u32 << 12) | (0u32 << 7);
        // NOP-like instructions: SET X, 1; SET X, 2; SET X, 3
        let mut pio = make_pio_with_program(&[0xE021, 0xE022, 0xE023]);
        pio.sm[0].execctrl = execctrl;
        step_n(&mut pio, 1, 0); // addr 0: SET X, 1 -> pc=1
        assert_eq!(pio.sm[0].x, 1);
        assert_eq!(pio.sm[0].pc, 1);
        step_n(&mut pio, 1, 0); // addr 1: SET X, 2 -> pc=2
        assert_eq!(pio.sm[0].x, 2);
        assert_eq!(pio.sm[0].pc, 2);
        step_n(&mut pio, 1, 0); // addr 2: SET X, 3 -> pc wraps to 0
        assert_eq!(pio.sm[0].x, 3);
        assert_eq!(pio.sm[0].pc, 0);
    }

    #[test]
    fn test_mov_x_to_y() {
        // SET X, 31; MOV Y, X
        // SET X, 31 => 0b111_00000_001_11111 = 0xE03F (dest=001, data=31)
        // Actually SET only has 5-bit data so max is 31
        // MOV Y, X => opcode=101, dest=010(Y), op=00, src=001(X)
        //   => 0b101_00000_010_00_001 = 0xA041
        let mut pio = make_pio_with_program(&[0xE03F, 0xA041]);
        step_n(&mut pio, 1, 0); // SET X, 31
        assert_eq!(pio.sm[0].x, 31);
        step_n(&mut pio, 1, 0); // MOV Y, X
        assert_eq!(pio.sm[0].y, 31);
    }

    #[test]
    fn test_mov_invert() {
        // SET X, 0; MOV Y, ~X
        // SET X, 0 => 0xE020 (dest=001, data=0)
        // MOV Y, ~X => opcode=101, dest=010(Y), op=01(invert), src=001(X)
        //   => 0b101_00000_010_01_001 = 0xA049
        let mut pio = make_pio_with_program(&[0xE020, 0xA049]);
        step_n(&mut pio, 1, 0); // SET X, 0
        assert_eq!(pio.sm[0].x, 0);
        step_n(&mut pio, 1, 0); // MOV Y, ~X
        assert_eq!(pio.sm[0].y, 0xFFFF_FFFF);
    }

    #[test]
    fn test_mov_bit_reverse() {
        // SET X, 1; MOV Y, ::X (bit-reverse)
        // SET X, 1 => 0xE021
        // MOV Y, ::X => opcode=101, dest=010(Y), op=10(reverse), src=001(X)
        //   => 0b101_00000_010_10_001 = 0xA051
        let mut pio = make_pio_with_program(&[0xE021, 0xA051]);
        step_n(&mut pio, 1, 0);
        assert_eq!(pio.sm[0].x, 1);
        step_n(&mut pio, 1, 0);
        // bit-reverse of 0x0000_0001 = 0x8000_0000
        assert_eq!(pio.sm[0].y, 0x8000_0000);
    }

    #[test]
    fn test_pull_push() {
        // Push value to TX FIFO, PULL, verify OSR; then PUSH, verify RX FIFO
        // PULL block: opcode=100, dir=1, if_empty=0, block=1 => 0b100_00000_1_0_1_00000 = 0x80A0
        // PUSH block: opcode=100, dir=0, if_full=0, block=1 => 0b100_00000_0_0_1_00000 = 0x8020
        let mut pio = make_pio_with_program(&[0x80A0, 0x8020]);
        // Pre-load TX FIFO with a value
        pio.sm[0].tx_fifo.push(0xDEAD_BEEF);

        step_n(&mut pio, 1, 0); // PULL
        assert_eq!(pio.sm[0].osr, 0xDEAD_BEEF);
        assert_eq!(pio.sm[0].osr_count, 0);

        // Set ISR to a known value for PUSH
        pio.sm[0].isr = 0xCAFE_BABE;
        pio.sm[0].isr_count = 32;

        step_n(&mut pio, 1, 0); // PUSH
        assert_eq!(pio.sm[0].isr, 0, "ISR cleared after PUSH");
        assert_eq!(pio.sm[0].isr_count, 0);
        let popped = pio.sm[0].rx_fifo.pop().unwrap();
        assert_eq!(popped, 0xCAFE_BABE);
    }

    #[test]
    fn test_pull_blocking_stall() {
        // PULL block with empty FIFO: SM should stall
        // PULL block: 0x80A0
        // Next instruction: SET X, 5 => 0xE025
        let mut pio = make_pio_with_program(&[0x80A0, 0xE025]);

        step_n(&mut pio, 1, 0); // PULL with empty FIFO => stall
        assert!(pio.sm[0].stalled, "SM should stall on empty PULL");
        assert_eq!(pio.sm[0].pc, 0, "PC should not advance while stalled");

        step_n(&mut pio, 1, 0); // Still stalled
        assert!(pio.sm[0].stalled);

        // Push a value to TX FIFO — SM should unstall on next tick
        pio.sm[0].tx_fifo.push(42);
        step_n(&mut pio, 1, 0); // Re-evaluate: FIFO not empty => unstall, re-execute PULL
        assert!(!pio.sm[0].stalled);
        assert_eq!(pio.sm[0].osr, 42, "PULL transferred data from TX FIFO to OSR");
        assert_eq!(pio.sm[0].pc, 1, "PC advanced after unstall");

        step_n(&mut pio, 1, 0); // Execute SET X, 5 (at addr 1)
        assert_eq!(pio.sm[0].x, 5);
    }

    #[test]
    fn test_pull_noblock_empty_copies_x() {
        // PULL NOBLOCK with empty TX FIFO should copy X into OSR
        // PULL noblock: opcode=100, dir=1, if_empty=0, block=0
        // = 0b100_00000_1_0_0_00000 = 0x8080
        let mut pio = make_pio_with_program(&[0x8080]);
        pio.sm[0].x = 0xDEAD_BEEF;

        step_n(&mut pio, 1, 0); // PULL NOBLOCK with empty FIFO
        assert!(!pio.sm[0].stalled, "PULL NOBLOCK should not stall");
        assert_eq!(pio.sm[0].osr, 0xDEAD_BEEF, "OSR should be copied from X");
        assert_eq!(pio.sm[0].osr_count, 0);
    }

    #[test]
    fn test_out_pins() {
        // Load OSR with known value via PULL, then OUT PINS 4
        // PULL block: 0x80A0
        // OUT PINS, 4: opcode=011, dest=000, bit_count=00100 => 0b011_00000_000_00100 = 0x6004
        let mut pio = make_pio_with_program(&[0x80A0, 0x6004]);
        // Default SHIFTCTRL: OUT_SHIFTDIR=0 (left), so data comes from MSB
        // But default SHIFTCTRL is 0x000C_0000. Let's check:
        // bit 19 = OUT_SHIFTDIR. 0x000C_0000 = 0b0000_0000_0000_1100_0000_0000_0000_0000
        // bit 19 = 1. So shift right (data from LSB side).
        pio.sm[0].tx_fifo.push(0x0000_000F); // bottom 4 bits = 1111
        // Set OUT_COUNT to 4 and OUT_BASE to 0 in pinctrl
        let pinctrl = (4u32 << 20) | (0u32); // out_count=4, out_base=0
        pio.sm[0].pinctrl = pinctrl;
        step_n(&mut pio, 1, 0); // PULL => osr = 0x0000_000F
        step_n(&mut pio, 1, 0); // OUT PINS, 4 => shifts 4 LSBs out
        // With shift-right, bottom 4 bits of OSR = 0xF
        assert_eq!(pio.shared_pin_values & 0xF, 0xF, "bottom 4 pins set to 1");
    }

    #[test]
    fn test_in_shift_left() {
        // SET X, 0xAB (can't set >31 via SET, so use X=0x1F=31)
        // Actually SET only does 5-bit values. Let's use X=15 (0xF).
        // IN X, 8: shift 8 bits from X into ISR (left shift)
        // SET X, 15: 0xE02F
        // IN X, 8: opcode=010, src=001(X), bit_count=01000 => 0b010_00000_001_01000 = 0x4028
        let mut pio = make_pio_with_program(&[0xE02F, 0x4028]);
        // Force IN_SHIFTDIR=0 (left): bit 18 of shiftctrl = 0
        pio.sm[0].shiftctrl &= !(1 << 18);
        step_n(&mut pio, 1, 0); // SET X, 15
        step_n(&mut pio, 1, 0); // IN X, 8
        // Left shift: ISR = (0 << 8) | (15 & 0xFF) = 15
        assert_eq!(pio.sm[0].isr, 15);
        assert_eq!(pio.sm[0].isr_count, 8);
    }

    #[test]
    fn test_irq_set_clear() {
        // IRQ set 0: opcode=110, clear=0, wait=0, index=00000
        //   => 0b110_00000_0_0_0_00000 = 0xC000
        // IRQ clear 0: opcode=110, clear=1, wait=0, index=00000
        //   => 0b110_00000_0_1_0_00000 = 0xC040
        let mut pio = make_pio_with_program(&[0xC000, 0xC040]);
        assert_eq!(pio.irq_flags, 0);
        step_n(&mut pio, 1, 0); // IRQ set 0
        assert_eq!(pio.irq_flags & 1, 1, "IRQ flag 0 set");
        step_n(&mut pio, 1, 0); // IRQ clear 0
        assert_eq!(pio.irq_flags & 1, 0, "IRQ flag 0 cleared");
    }

    #[test]
    fn test_irq_relative() {
        // SM2 sets IRQ rel 0: index = 0x10 (relative flag)
        // IRQ set rel 0: opcode=110, clear=0, wait=0, index=10000
        //   => 0b110_00000_0_0_0_10000 = 0xC010
        let mut pio = make_pio_with_program(&[0xC010]);
        pio.set_sm_enabled(2, true);
        pio.sm[2].clkdiv_int = 1;
        pio.sm[2].clkdiv_frac = 0;
        // SM2 has sm_id=2, so relative IRQ 0 -> (0+2)%4 = 2
        step_n(&mut pio, 1, 0); // SM0 ticks (at addr 0 which is same insn)
        // But we want SM2 to execute. SM0 also ticks. Let's disable SM0.
        pio.set_sm_enabled(0, false);
        // Reset SM2 PC to start fresh
        pio.sm[2].pc = 0;
        pio.irq_flags = 0;
        step_n(&mut pio, 1, 0); // SM2 executes IRQ set rel 0
        assert_eq!(pio.irq_flags & (1 << 2), 1 << 2, "IRQ flag 2 set (rel 0 from SM2)");
    }

    #[test]
    fn test_wait_gpio_stall() {
        // WAIT 1 GPIO 5: polarity=1, source=00(GPIO), index=00101
        // operand = 0b1_00_00101 = 0x85
        // opcode=001, delay/ss=00000
        // insn = 0b001_00000_10000101 = 0x2085
        let mut pio = make_pio_with_program(&[0x2085, 0xE021]);
        step_n(&mut pio, 1, 0); // WAIT 1 GPIO 5 with pin 5 = 0 => stall
        assert!(pio.sm[0].stalled);

        step_n(&mut pio, 1, 0); // Still stalled (pin 5 still low)
        assert!(pio.sm[0].stalled);

        // Set pin 5 high
        step_n(&mut pio, 1, 1 << 5); // Pin 5 high => unstall
        assert!(!pio.sm[0].stalled);
    }

    #[test]
    fn test_delay() {
        // SET X, 1 with delay=3: takes 1+3=4 PIO cycles total
        // delay_bits=5 (no sideset), so delay field = 3 => insn[12:8]=00011
        // SET X, 1: opcode=111, dest=001, data=00001
        // insn = 0b111_00011_001_00001 = 0xE321
        let mut pio = make_pio_with_program(&[0xE321, 0xE022]);
        step_n(&mut pio, 1, 0); // Execute SET X, 1 (cycle 1), delay_count=3
        assert_eq!(pio.sm[0].x, 1);
        assert_eq!(pio.sm[0].delay_count, 3);
        assert_eq!(pio.sm[0].pc, 1); // PC already advanced

        step_n(&mut pio, 1, 0); // delay (cycle 2)
        assert_eq!(pio.sm[0].delay_count, 2);
        step_n(&mut pio, 1, 0); // delay (cycle 3)
        assert_eq!(pio.sm[0].delay_count, 1);
        step_n(&mut pio, 1, 0); // delay (cycle 4)
        assert_eq!(pio.sm[0].delay_count, 0);

        // Now next tick executes instruction at PC=1 (SET X, 2)
        step_n(&mut pio, 1, 0);
        assert_eq!(pio.sm[0].x, 2);
    }

    #[test]
    fn test_force_execute() {
        // Force-execute JMP 5 via SMn_INSTR write
        let mut pio = PioBlock::new();
        pio.sm[0].pc = 0;
        // JMP 5 = 0x0005
        // Write to SM0 INSTR register (offset 0x0C8 + 0x10 = 0x0D8)
        pio.write32(0x0D8, 0x0005, 0);
        assert_eq!(pio.sm[0].pc, 5, "force-execute JMP sets PC to 5");
        assert_eq!(pio.sm[0].last_insn, 0x0005);
    }

    #[test]
    fn test_force_execute_no_advance() {
        // Force-execute SET X, 7 — PC should NOT advance
        let mut pio = PioBlock::new();
        pio.sm[0].pc = 10;
        // SET X, 7 = 0xE027 (opcode=111, dest=001, data=00111)
        pio.write32(0x0D8, 0xE027, 0);
        assert_eq!(pio.sm[0].x, 7);
        assert_eq!(pio.sm[0].pc, 10, "PC should not advance for forced non-JMP");
    }

    #[test]
    fn test_sideset_on_stall() {
        // PULL block with side-set = verify side-set applied even though SM stalls
        // Use sideset_count=1, no SIDE_EN
        // PINCTRL: sideset_count=1 (bit[31:29]=001), sideset_base=3
        let pinctrl = (1u32 << 29) | (3u32 << 10);
        // PULL block with side-set=1:
        // delay_bits = 5-1 = 4, side-set occupies top 1 bit of [12:8]
        // delay/ss = [1_0000] = 0b10000 = 16
        // PULL block: opcode=100, dir=1, if_empty=0, block=1
        // operand = 0b1_0_1_00000 = 0xA0
        // insn = 0b100_10000_10100000 = 0x90A0
        let mut pio = make_pio_with_program(&[0x90A0]);
        pio.sm[0].pinctrl = pinctrl;
        // TX FIFO empty, so PULL will stall. But side-set should fire.
        step_n(&mut pio, 1, 0);
        assert!(pio.sm[0].stalled, "SM stalls on empty PULL");
        // Side-set=1 at sideset_base=3: pin 3 should be set
        assert_eq!(pio.sm[0].sideset_pins & (1 << 3), 1 << 3,
            "side-set applied even on stalling instruction");
    }

    // ---- Stage C: Autopush tests ----

    #[test]
    fn test_autopush_threshold_32() {
        // Enable autopush (bit 16), threshold=0 (meaning 32).
        // IN X, 8 four times => 32 bits shifted in => autopush fires.
        // SET X, 15: 0xE02F
        // IN X, 8: opcode=010, src=001(X), bit_count=01000 => 0x4028
        let mut pio = make_pio_with_program(&[0xE02F, 0x4028, 0x4028, 0x4028, 0x4028]);
        // Enable autopush (bit 16), thresholds stay at 0 (=32)
        pio.sm[0].shiftctrl |= 1 << 16;
        // Set IN_SHIFTDIR=0 (left) for simple accumulation
        pio.sm[0].shiftctrl &= !(1 << 18);

        step_n(&mut pio, 1, 0); // SET X, 15
        assert_eq!(pio.sm[0].x, 15);

        // Four IN X,8 => 32 bits total
        step_n(&mut pio, 1, 0); // IN X, 8 (8 bits) — no autopush yet
        assert_eq!(pio.sm[0].isr_count, 8);
        assert!(pio.sm[0].rx_fifo.is_empty(), "no push at 8 bits");

        step_n(&mut pio, 1, 0); // IN X, 8 (16 bits)
        assert_eq!(pio.sm[0].isr_count, 16);
        assert!(pio.sm[0].rx_fifo.is_empty(), "no push at 16 bits");

        step_n(&mut pio, 1, 0); // IN X, 8 (24 bits)
        assert_eq!(pio.sm[0].isr_count, 24);
        assert!(pio.sm[0].rx_fifo.is_empty(), "no push at 24 bits");

        step_n(&mut pio, 1, 0); // IN X, 8 (32 bits) — autopush fires!
        assert_eq!(pio.sm[0].isr_count, 0, "ISR count cleared by autopush");
        assert_eq!(pio.sm[0].isr, 0, "ISR cleared by autopush");
        assert!(!pio.sm[0].rx_fifo.is_empty(), "value pushed to RX FIFO");
        let val = pio.sm[0].rx_fifo.pop().unwrap();
        // ISR was shifted left: (((15 << 8 | 15) << 8 | 15) << 8 | 15) = 0x0F0F0F0F
        assert_eq!(val, 0x0F0F_0F0F);
    }

    #[test]
    fn test_autopush_threshold_16() {
        // Set push_threshold=16 (bits[24:20]=10000=16).
        // IN X, 8 twice => autopush at 16 bits.
        let mut pio = make_pio_with_program(&[0xE02F, 0x4028, 0x4028]);
        // Enable autopush (bit 16), set push threshold to 16 (bits [24:20])
        let shiftctrl = pio.sm[0].shiftctrl | (1 << 16); // autopush on
        let shiftctrl = (shiftctrl & !(0x1F << 20)) | (16u32 << 20); // push_threshold=16
        pio.sm[0].shiftctrl = shiftctrl;
        // Set IN_SHIFTDIR=0 (left)
        pio.sm[0].shiftctrl &= !(1 << 18);

        step_n(&mut pio, 1, 0); // SET X, 15
        step_n(&mut pio, 1, 0); // IN X, 8 (8 bits)
        assert_eq!(pio.sm[0].isr_count, 8);
        assert!(pio.sm[0].rx_fifo.is_empty(), "no push at 8 bits");

        step_n(&mut pio, 1, 0); // IN X, 8 (16 bits) — autopush fires!
        assert_eq!(pio.sm[0].isr_count, 0, "ISR cleared after autopush at 16");
        assert!(!pio.sm[0].rx_fifo.is_empty());
        let val = pio.sm[0].rx_fifo.pop().unwrap();
        // Left-shift: (15 << 8) | 15 = 0x0F0F
        assert_eq!(val, 0x0F0F);
    }

    #[test]
    fn test_autopush_default_shiftctrl() {
        // Default SHIFTCTRL = 0x000C_0000: autopush disabled.
        // Even after 32 bits shifted in, no auto-push.
        let mut pio = make_pio_with_program(&[0xE02F, 0x4028, 0x4028, 0x4028, 0x4028]);
        // Verify autopush is disabled by default
        assert_eq!(pio.sm[0].shiftctrl & (1 << 16), 0, "autopush disabled by default");
        // Set IN_SHIFTDIR=0 (left)
        pio.sm[0].shiftctrl &= !(1 << 18);

        step_n(&mut pio, 1, 0); // SET X, 15
        for _ in 0..4 {
            step_n(&mut pio, 1, 0); // IN X, 8
        }
        // isr_count saturates at 32
        assert_eq!(pio.sm[0].isr_count, 32);
        assert!(pio.sm[0].rx_fifo.is_empty(), "no autopush with default shiftctrl");
    }

    // ---- Stage C: Autopull tests ----

    #[test]
    fn test_autopull_basic() {
        // Enable autopull, threshold=32 (default). Push 0xABCD to TX FIFO.
        // Set osr_count=32 (exhausted). Execute OUT PINS,8.
        // Verify OSR was refilled from FIFO before the OUT shifted.
        // OUT PINS, 8: opcode=011, dest=000(PINS), bit_count=01000 => 0x6008
        let mut pio = make_pio_with_program(&[0x6008]);
        // Enable autopull (bit 17)
        pio.sm[0].shiftctrl |= 1 << 17;
        // Set out_count=8, out_base=0 in pinctrl
        pio.sm[0].pinctrl = 8u32 << 20; // out_count=8, out_base=0
        // Exhaust OSR
        pio.sm[0].osr_count = 32;
        // Push value to TX FIFO
        pio.sm[0].tx_fifo.push(0x0000_ABCD);

        step_n(&mut pio, 1, 0); // OUT PINS, 8 — autopull fires first, refills OSR
        assert!(!pio.sm[0].stalled, "should not stall — FIFO had data");
        // Autopull loaded 0x0000_ABCD into OSR, then OUT shifted 8 bits out.
        // Default shiftctrl bit 19 = 1 (shift right), so bottom 8 bits = 0xCD shifted out.
        assert_eq!(pio.sm[0].osr_count, 8, "8 bits shifted out after autopull refill");
        // The remaining OSR should be 0x0000_ABCD >> 8 = 0x0000_00AB
        assert_eq!(pio.sm[0].osr, 0x0000_00AB);
        // out_pins bottom 8 bits should be 0xCD
        assert_eq!(pio.shared_pin_values & 0xFF, 0xCD);
    }

    #[test]
    fn test_autopull_stall_on_empty() {
        // Enable autopull, osr_count=32, TX FIFO empty.
        // Execute OUT — SM should stall.
        // Push value, step again — SM should unstall and OUT completes.
        // OUT NULL, 8: opcode=011, dest=011(NULL), bit_count=01000 => 0x6068
        let mut pio = make_pio_with_program(&[0x6068, 0xE025]);
        // Enable autopull (bit 17)
        pio.sm[0].shiftctrl |= 1 << 17;
        // Exhaust OSR
        pio.sm[0].osr_count = 32;

        step_n(&mut pio, 1, 0); // OUT NULL, 8 — autopull fires, FIFO empty => stall
        assert!(pio.sm[0].stalled, "SM stalls when autopull finds empty FIFO");
        assert_eq!(pio.sm[0].pc, 0, "PC should not advance while stalled");

        step_n(&mut pio, 1, 0); // Still stalled
        assert!(pio.sm[0].stalled);

        // Push value to TX FIFO
        pio.sm[0].tx_fifo.push(0x1234_5678);
        step_n(&mut pio, 1, 0); // Re-evaluate: FIFO not empty => unstall, re-execute OUT
        assert!(!pio.sm[0].stalled, "SM unstalls when TX FIFO gets data");
        // The instruction at pc=0 (OUT NULL, 8) should have completed.
        // Autopull loaded 0x1234_5678, then OUT NULL shifted 8 bits (discarded).
        assert_eq!(pio.sm[0].osr_count, 8);
        assert_eq!(pio.sm[0].pc, 1, "PC advanced after unstall");

        step_n(&mut pio, 1, 0); // SET X, 5
        assert_eq!(pio.sm[0].x, 5);
    }

    // ---- Stage C: GPIO integration tests ----

    // GPIO-merge tests (PIO vs SIO arbitration via `Emulator::update_gpio`)
    // live in `crates/mdrp2350/src/pio_tests.rs` — they exercise the
    // chip `Emulator`.

    #[test]
    fn test_pin_mapping_out() {
        // Configure out_base=5, execute OUT PINS,4 with known value.
        // Verify out_pins has correct bits at positions [8:5].
        // PULL block: 0x80A0
        // OUT PINS, 4: 0x6004
        let mut pio = make_pio_with_program(&[0x80A0, 0x6004]);
        // out_base=5, out_count=4
        pio.sm[0].pinctrl = (4u32 << 20) | 5u32; // out_count=4, out_base=5
        pio.sm[0].tx_fifo.push(0x0000_000F); // bottom 4 bits = 1111
        // Block-shared pin_values resets to all-ones (pullup convention,
        // matches epio); pin this assumption down explicitly so the
        // OUT-only-touches-its-count assertion isolates OUT's behaviour.
        pio.shared_pin_values = 0;

        step_n(&mut pio, 1, 0); // PULL
        step_n(&mut pio, 1, 0); // OUT PINS, 4
        // Default shiftctrl: shift right, so bottom 4 bits (0xF) are shifted out.
        // out_base=5 means bits should appear at positions 5,6,7,8.
        let expected_mask = 0xF << 5;
        assert_eq!(pio.shared_pin_values & expected_mask, expected_mask,
            "OUT PINS with out_base=5 should set pins [8:5]");
        // Other pins should be 0
        assert_eq!(pio.shared_pin_values & !expected_mask, 0,
            "only pins [8:5] should be set");
    }

    #[test]
    fn test_pin_mapping_wrap() {
        // Configure out_base=30, execute OUT PINS,4. Verify wrap: bits at [31:30] and [1:0].
        // PULL block: 0x80A0
        // OUT PINS, 4: 0x6004
        let mut pio = make_pio_with_program(&[0x80A0, 0x6004]);
        // out_base=30, out_count=4
        pio.sm[0].pinctrl = (4u32 << 20) | 30u32; // out_count=4, out_base=30
        pio.sm[0].tx_fifo.push(0x0000_000F); // bottom 4 bits = 1111
        pio.shared_pin_values = 0; // see comment in test_pin_mapping_out

        step_n(&mut pio, 1, 0); // PULL
        step_n(&mut pio, 1, 0); // OUT PINS, 4
        // Pins should wrap: bits 30,31,0,1 all set
        let expected = (3u32 << 30) | 3u32; // bits 30,31 and bits 0,1
        assert_eq!(pio.shared_pin_values, expected,
            "OUT PINS with out_base=30 should wrap to bits [31:30] and [1:0]");
    }

    #[test]
    fn test_sideset_persists_during_delay() {
        // Side-set with delay=3: verify sideset_pins stays set across all delay cycles.
        // Use sideset_count=1, no SIDE_EN, sideset_base=7.
        // SET X, 1 with sideset=1, delay=3:
        // sideset_count=1 => delay_bits=4
        // delay/ss = [1_0011] = 0b10011 = 19 (ss=1, delay=3)
        // SET X, 1: opcode=111, dest=001, data=00001
        // insn = 0b111_10011_001_00001 = 0xF321
        let pinctrl = (1u32 << 29) | (7u32 << 10); // sideset_count=1, sideset_base=7
        let mut pio = make_pio_with_program(&[0xF321, 0xE022]);
        pio.sm[0].pinctrl = pinctrl;

        step_n(&mut pio, 1, 0); // Execute SET X, 1 [side 1] [delay 3]
        assert_eq!(pio.sm[0].x, 1);
        assert_eq!(pio.sm[0].sideset_pins & (1 << 7), 1 << 7,
            "sideset pin 7 set on execution");

        // Check through all 3 delay cycles
        for cycle in 0..3 {
            assert_eq!(pio.sm[0].sideset_pins & (1 << 7), 1 << 7,
                "sideset pin 7 persists during delay cycle {}", cycle);
            step_n(&mut pio, 1, 0);
        }
        // After delay completes, sideset_pins should still hold its value
        // (it's only overwritten by the next instruction's sideset)
        assert_eq!(pio.sm[0].sideset_pins & (1 << 7), 1 << 7,
            "sideset pin 7 still set after delay completes");
    }

    // ====================================================================
    // Stage D: Waveform integration tests
    // ====================================================================
    //
    // These tests drive PIO programs through the full `Emulator` and
    // therefore live in `crates/mdrp2350/src/pio_tests.rs`:
    // `test_pio_blinky_gpio25`, `test_pio_uart_tx_0x55`,
    // `test_pio_spi_clk_mosi` (plus helpers `pio_write`,
    // `pio_test_emulator`, `pio_load_program`).

    // ====================================================================
    // PIO Idle Skip — fast-path regression probes
    // ====================================================================

    #[test]
    fn idle_block_step_is_noop() {
        // Fresh block, no SMs enabled: step_n must be a semantic no-op.
        let mut pio = PioBlock::new();
        // Capture per-SM cross-cycle state that the fast path must not perturb.
        let pc_before: [u8; 4] = [pio.sm[0].pc, pio.sm[1].pc, pio.sm[2].pc, pio.sm[3].pc];
        let acc_before: [u32; 4] = [
            pio.sm[0].clkdiv_acc,
            pio.sm[1].clkdiv_acc,
            pio.sm[2].clkdiv_acc,
            pio.sm[3].clkdiv_acc,
        ];

        pio.step_n(1000, 0);

        assert_eq!(pio.pad_out, 0, "idle block must not drive pad_out");
        assert_eq!(pio.pad_oe, 0, "idle block must not drive pad_oe");
        for i in 0..4 {
            assert_eq!(pio.sm[i].pc, pc_before[i], "SM{i} pc must not advance");
            assert_eq!(
                pio.sm[i].clkdiv_acc, acc_before[i],
                "SM{i} clkdiv_acc must not advance"
            );
        }
    }

    #[test]
    fn disable_clears_pin_outputs() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        // Poke the block-shared pad latches directly (pub(crate)) to avoid
        // running a program — we only need the merge path to observe them.
        pio.shared_pin_values = 0x1;
        pio.shared_pin_dirs = 0x1;

        pio.step(0);

        assert!(
            pio.pad_out & pio.pad_oe != 0,
            "enabled SM's pin state should be merged into pad_out/pad_oe"
        );

        pio.set_sm_enabled(0, false);

        assert_eq!(
            pio.pad_out, 0,
            "disabling the last SM must clear pad_out on the same tick"
        );
        assert_eq!(
            pio.pad_oe, 0,
            "disabling the last SM must clear pad_oe on the same tick"
        );
    }

    // ====================================================================
    // Side-set pad_oe — RP2350 §11.3.2.3 compliance
    // ====================================================================

    #[test]
    fn side_set_value_drive_does_not_force_oe() {
        // With EXECCTRL.SIDE_PINDIR=0 (value-drive), side-set writes pin
        // values only; pad_oe must stay zero absent an explicit PINDIRS
        // programming. Reproduces the tech-debt scenario from
        // `silicon_periph_diff_rp2350::pio0_side_set_toggle`.
        let mut pio = PioBlock::new();
        // PINCTRL: SIDESET_COUNT=1, SIDESET_BASE=0
        pio.sm[0].pinctrl = 1u32 << 29;
        // EXECCTRL: SIDE_EN=0, SIDE_PINDIR=0 (value-drive)
        pio.sm[0].execctrl = 0;
        // JMP 0, side 1 (side-set bit in [12]=1, delay 0, JMP addr=0)
        pio.instr_mem[0] = 0x1000;
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 0;

        pio.step(0);

        // Side-set drove the VALUE on pin 0 …
        assert_ne!(
            pio.pad_out & 1,
            0,
            "side-set pin 0 value should be driven high"
        );
        // … but DID NOT force the direction.
        assert_eq!(
            pio.pad_oe & 1,
            0,
            "side-set value-drive must not set pad_oe without PINDIRS"
        );
    }

    #[test]
    fn side_set_direction_drive_still_sets_oe() {
        // Regression guard: EXECCTRL.SIDE_PINDIR=1 (direction-drive) must
        // continue to set pad_oe for the side-set pin — this path is
        // unchanged by the fix.
        let mut pio = PioBlock::new();
        pio.sm[0].pinctrl = 1u32 << 29; // SIDESET_COUNT=1
        pio.sm[0].execctrl = 1u32 << 29; // SIDE_PINDIR=1
        pio.instr_mem[0] = 0x1000; // JMP 0, side 1
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 0;

        pio.step(0);

        // Side-set wrote the DIRECTION bit via sideset_dirs;
        // merge_pin_outputs ORs sideset_dirs & positioned_mask into oe.
        assert_ne!(
            pio.pad_oe & 1,
            0,
            "SIDE_PINDIR=1 must still set pad_oe for the side-set pin"
        );
    }

    #[test]
    fn set_pindirs_drives_oe_without_side_set() {
        // Confirms the non-side-set PINDIRS path still works — the fix
        // leans on `shared_pin_dirs` (populated by SET/OUT/MOV PINDIRS)
        // being the sole source of pad_oe for side-set pins.
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        // PINCTRL: SET_COUNT=1, SET_BASE=0
        pio.sm[0].pinctrl = (1u32 << 26) | (0u32 << 5);
        // SET PINDIRS, 1  (opcode=111, dest=100, data=00001) = 0xE081
        pio.write32(0x0D8, 0xE081, 0);

        pio.step(0);

        assert_ne!(
            pio.pad_oe & 1,
            0,
            "SET PINDIRS, 1 with SET_BASE=0 must drive pad_oe bit 0"
        );
    }

    #[test]
    fn side_set_after_pindirs_keeps_oe() {
        // Composite: firmware uses SET PINDIRS to establish direction,
        // then side-set toggles values. The PINDIRS-established OE must
        // persist across the side-set (bidirectional-bus / open-drain
        // pattern).
        let mut pio = PioBlock::new();
        // PINCTRL: SIDESET_COUNT=1, SIDESET_BASE=0,
        //          SET_COUNT=1, SET_BASE=0
        pio.sm[0].pinctrl = (1u32 << 29) | (1u32 << 26);
        // EXECCTRL: SIDE_EN=0, SIDE_PINDIR=0, WRAP_TOP=31 (default-style
        // full-memory wrap so PC advances 0→1 between ticks instead of
        // wrapping back to 0).
        pio.sm[0].execctrl = 0x1Fu32 << 12;
        // Program:
        //   addr 0: SET PINDIRS, 1  (enable pin 0 as output)
        //   addr 1: NOP, side 1     (MOV Y, Y with side=1 — drives pin 0 high)
        pio.instr_mem[0] = 0xE081; // SET PINDIRS, 1
        pio.instr_mem[1] = 0xA042 | (1 << 12); // MOV Y,Y + side=1
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 0;

        pio.step(0); // SET PINDIRS, 1 → shared_pin_dirs bit 0 = 1
        assert_ne!(
            pio.pad_oe & 1,
            0,
            "OE established by SET PINDIRS on tick 1"
        );

        pio.step(0); // NOP side 1 → sideset_pins bit 0 = 1; oe stays set
        assert_ne!(
            pio.pad_oe & 1,
            0,
            "OE from PINDIRS must persist across side-set value write"
        );
        assert_ne!(
            pio.pad_out & 1,
            0,
            "side-set drove the value high on tick 2"
        );
    }
}
