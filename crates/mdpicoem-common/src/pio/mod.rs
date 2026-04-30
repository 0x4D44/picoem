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
    /// Cached: true iff at least one SM has SIDESET_COUNT > 0
    /// (PINCTRL bits [31:29]). When false,
    /// [`Self::merge_pin_outputs`] skips the per-SM side-set loop
    /// entirely — saves ~16% of `step_n` throughput on programs that
    /// don't use side-set (which is most). Recomputed by
    /// [`Self::recompute_any_sideset`] after every PINCTRL write.
    /// Placed next to `sm_enabled_mask` so both fast-path flags share
    /// the same cache line as `shared_pin_values`/`shared_pin_dirs`.
    pub(crate) any_sideset_programmed: bool,
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

impl Default for PioBlock {
    fn default() -> Self {
        Self::new()
    }
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
            any_sideset_programmed: false,
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
        // SM reset above sets pinctrl back to default (SIDESET_COUNT=0).
        self.any_sideset_programmed = false;
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
        #[cfg(feature = "pio-pad-diag")]
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
    #[inline]
    fn merge_pin_outputs(&mut self) {
        if self.sm_enabled_mask == 0 {
            self.pad_out = 0;
            self.pad_oe = 0;
            return;
        }
        let mut out: u32 = self.shared_pin_values;
        let mut oe: u32 = self.shared_pin_dirs;
        // Short-circuit: when no SM has SIDESET_COUNT > 0, the per-SM
        // loop produces no overlay — the shared latches ARE the merge
        // result. Saves ~16% of `step_n` cost on common (no-side-set)
        // programs. See `wrk_journals/2026.04.20 - JRN - PIO step_n
        // Profiling.md` §R2.
        if !self.any_sideset_programmed {
            // The `trace!` macro itself is elided in release builds
            // (`release_max_level_info`); the `!=` comparison that
            // guards it is what the optimiser has to fold away.
            if out != self.pad_out || oe != self.pad_oe {
                tracing::trace!(
                    target: "mdpicoem_common::pio",
                    old_out = format_args!("0x{:08x}", self.pad_out),
                    new_out = format_args!("0x{:08x}", out),
                    old_oe = format_args!("0x{:08x}", self.pad_oe),
                    new_oe = format_args!("0x{:08x}", oe),
                    "pad_change",
                );
            }
            self.pad_out = out;
            self.pad_oe = oe;
            return;
        }
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
                let ss_base = (sm.pinctrl >> 10) & 0x1F;
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
        // Diagnostic trace: if pad_out has changed since last merge,
        // emit a `trace!` with the new value AND the diff mask. Fires
        // at most once per `step()` (one merge per sysclk). Volume is
        // PIO-tick-bound (millions per second of sim time) so use
        // `trace!` to keep release builds silent — diagnostic builds
        // narrow the filter via `RUST_LOG`.
        if out != self.pad_out || oe != self.pad_oe {
            tracing::trace!(
                target: "mdpicoem_common::pio",
                old_out = format_args!("0x{:08x}", self.pad_out),
                new_out = format_args!("0x{:08x}", out),
                old_oe = format_args!("0x{:08x}", self.pad_oe),
                new_oe = format_args!("0x{:08x}", oe),
                "pad_change",
            );
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
    ///
    /// Gated behind the `pio-pad-diag` feature: the per-sysclk diff +
    /// counter bumps cost ~9% of `step_n` throughput on a 1-SM clkdiv=1
    /// program. Enable when running PicoGUS-style PSRAM-SPI diff work
    /// that needs the counters.
    #[cfg(feature = "pio-pad-diag")]
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
            self.pad_out_mosi_writes_of_1 = self.pad_out_mosi_writes_of_1.wrapping_add(1);
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
                    0 | 3 => val, // normal write and CLR both clear bits
                    1 => val,     // XOR
                    2 => val,     // SET
                    _ => return,
                };
                match alias {
                    0 => self.fdebug &= !mask, // W1C: writing 1 clears
                    1 => self.fdebug ^= mask,
                    2 => self.fdebug |= mask,
                    3 => self.fdebug &= !mask,
                    _ => {}
                }
            }
            // FLEVEL: read-only
            0x00C => {}
            // TXF0-3: push to SM's TX FIFO. Trace per push so the
            // PicoGUS silent-WAV class of bug (PWM-IRQ pushes the right
            // sample but the SM never shifts it out, or the SM shifts
            // out zeros because pushes never landed) can be told apart
            // by looking at the actual pushed `val` interleaved with
            // pad-out transitions and i2s_capture LRCLK edges.
            // `debug!` keeps this out of release builds. Volume: one
            // line per audio sample (~44 kHz) — fine for diag, never in
            // the hot path of release.
            0x010 => {
                let ok = self.sm[0].tx_fifo.push(val);
                tracing::debug!(
                    target: "mdpicoem_common::pio",
                    sm = 0u8,
                    val = format_args!("0x{:08x}", val),
                    push_ok = ok,
                    occupancy = self.sm[0].tx_fifo.level(),
                    "txf_write",
                );
            }
            0x014 => {
                let ok = self.sm[1].tx_fifo.push(val);
                tracing::debug!(
                    target: "mdpicoem_common::pio",
                    sm = 1u8,
                    val = format_args!("0x{:08x}", val),
                    push_ok = ok,
                    occupancy = self.sm[1].tx_fifo.level(),
                    "txf_write",
                );
            }
            0x018 => {
                let ok = self.sm[2].tx_fifo.push(val);
                tracing::debug!(
                    target: "mdpicoem_common::pio",
                    sm = 2u8,
                    val = format_args!("0x{:08x}", val),
                    push_ok = ok,
                    occupancy = self.sm[2].tx_fifo.level(),
                    "txf_write",
                );
            }
            0x01C => {
                let ok = self.sm[3].tx_fifo.push(val);
                tracing::debug!(
                    target: "mdpicoem_common::pio",
                    sm = 3u8,
                    val = format_args!("0x{:08x}", val),
                    push_ok = ok,
                    occupancy = self.sm[3].tx_fifo.level(),
                    "txf_write",
                );
                {
                    static COUNTER: std::sync::atomic::AtomicU64 =
                        std::sync::atomic::AtomicU64::new(0);
                    static MAX_VAL: std::sync::atomic::AtomicU32 =
                        std::sync::atomic::AtomicU32::new(0);
                    let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    MAX_VAL.fetch_max(val, std::sync::atomic::Ordering::Relaxed);
                    if val != 0 || n < 5 || n.is_multiple_of(10000) {
                        tracing::debug!(
                            target: "mdpicoem_common::pio::txf_sample",
                            sm = 3u8,
                            n,
                            push_ok = ok,
                            "txf_write sample val=0x{:08x} max_seen=0x{:08x}",
                            val,
                            MAX_VAL.load(std::sync::atomic::Ordering::Relaxed),
                        );
                    }
                }
            }
            // RXF0-3: read-only
            0x020..=0x02C => {}
            // IRQ: W1C (or alias)
            0x030 => {
                match alias {
                    0 => self.irq_flags &= !(val as u8), // W1C
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
            0x038 => {
                self.input_sync_bypass = val;
            }
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
                self.recompute_any_sideset();
            }
            _ => {}
        }
    }

    /// Refresh the `any_sideset_programmed` cache by scanning all 4
    /// SMs' PINCTRL.SIDESET_COUNT (bits [31:29]). Called after any
    /// PINCTRL write through [`Self::write32`]; tests that bypass
    /// `write32` to set `sm[i].pinctrl` directly must call this
    /// themselves before stepping if they expect side-set behaviour.
    pub fn recompute_any_sideset(&mut self) {
        self.any_sideset_programmed = self.sm.iter().any(|s| ((s.pinctrl >> 29) & 7) != 0);
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

        // SM_RESTART: self-clearing action (reset SM state).
        //
        // Shift counters use the same "empty" convention as `StateMachine::new`:
        //   - `isr_count = 0` → ISR empty, zero bits since last push.
        //   - `osr_count = 32` → OSR "empty" (fully consumed), autopull fires on
        //     the next OUT. Matches epio and real RP2040/RP2350: the SDK's
        //     `pio_sm_init` calls `pio_sm_restart` immediately before
        //     firmware pushes its first DMA byte, and the program assumes
        //     the first OUT reads byte 1 via autopull (not OSR=0). The
        //     rp2040-psram driver (`begin: out x, 8`) is the canonical
        //     case — with `osr_count=0`, the first reset command
        //     mis-aligns as `x=0`, `y=first_byte`, cascading across every
        //     subsequent command and breaking PSRAM entirely.
        for i in 0..4 {
            if (sm_restart_bits >> i) & 1 != 0 {
                self.sm[i].pc = 0;
                self.sm[i].x = 0;
                self.sm[i].y = 0;
                self.sm[i].isr = 0;
                self.sm[i].osr = 0;
                self.sm[i].isr_count = 0;
                self.sm[i].osr_count = 32;
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
        assert_eq!(
            sm.shiftctrl, 0x000C_0000,
            "SHIFTCTRL reset: thresholds=0 (32)"
        );
        assert_eq!(sm.pinctrl, 0x1400_0000, "PINCTRL reset: SET_COUNT=5");
        assert_eq!(sm.clkdiv_int, 1, "CLKDIV int reset: 1");
        assert_eq!(sm.clkdiv_frac, 0, "CLKDIV frac reset: 0");
        assert_eq!(
            sm.read_clkdiv(),
            0x0001_0000,
            "CLKDIV register: 0x0001_0000"
        );
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
        assert_eq!(
            read_back & 0x8000_0000,
            0,
            "bit 31 is read-only EXEC_STALLED"
        );
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
        assert_eq!(
            pio.sm[0].execctrl, 0x0001_F000,
            "reset restores default EXECCTRL"
        );
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

    #[cfg(feature = "pio-pad-diag")]
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

    #[cfg(feature = "pio-pad-diag")]
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
        assert_eq!(
            pio.read32(0x134),
            0,
            "0x134 is RXF0_PUTGET3 on RP2350, must return 0"
        );

        // Inject IRQ flag 0 via the IRQ_FORCE register (offset 0x034).
        pio.write32(0x034, 0x01, 0);
        assert_eq!(pio.irq_flags, 0x01, "irq_flags bit 0 set by IRQ_FORCE");

        // INTR at 0x16C must expose IRQ flag 0 at bit 8 (not bit 0).
        let intr = pio.read32(0x16C);
        assert_ne!(
            intr & (1 << 8),
            0,
            "INTR at 0x16C: SM0 IRQ flag 0 must appear at bit 8"
        );
        assert_eq!(
            intr & 0x1,
            0,
            "INTR at 0x16C: bit 0 (SM0_RXNEMPTY) must be 0 when RX FIFO empty"
        );

        // IRQ0_INTE at 0x170: write 0x100 (bit 8 = SM0/IRQ flag 0).
        pio.write32(0x170, 0x100, 0);
        assert_eq!(pio.int0_inte, 0x100, "int0_inte set via offset 0x170");

        // IRQ0_INTS at 0x178: (INTR & INTE) | INTF — must return 0x100.
        let ints = pio.read32(0x178);
        assert_ne!(
            ints & 0x100,
            0,
            "IRQ0_INTS at 0x178 must show SM0 IRQ flag (bit 8) when INTE bit 8 set"
        );

        // IRQ1_INTE at 0x17C: bit 9 (SM1/IRQ flag 1), SM0 never set it.
        pio.write32(0x17C, 0x200, 0);
        let ints1 = pio.read32(0x184); // IRQ1_INTS
        assert_eq!(
            ints1 & 0x200,
            0,
            "IRQ1_INTS must be 0: SM0 never set IRQ flag 1"
        );
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
        assert_eq!(
            intr & 0x001,
            0x001,
            "raw_intr_rp2040: IRQ flag 0 must appear at bit 0 (RP2040 ds Table 358)"
        );
        assert_eq!(
            intr >> 12,
            0,
            "raw_intr_rp2040: bits [31:12] must be zero (12-bit register)"
        );
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
        assert_ne!(
            intr & 0x100,
            0,
            "raw_intr_rp2350: IRQ flag 0 must appear at bit 8 (RP2350 ds Table 1018)"
        );
        assert_eq!(
            intr & 0x001,
            0,
            "raw_intr_rp2350: bit 0 (SM0_RXNEMPTY) must be 0 when RX FIFO empty"
        );
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
        use super::decode::{PioOp, decode};
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
        use super::decode::{PioOp, decode};
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
        use super::decode::{PioOp, decode};
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
        let execctrl = 2u32 << 12;
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
        assert_eq!(
            pio.sm[0].osr, 42,
            "PULL transferred data from TX FIFO to OSR"
        );
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
        let pinctrl = 4u32 << 20; // out_count=4, out_base=0
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
        assert_eq!(
            pio.irq_flags & (1 << 2),
            1 << 2,
            "IRQ flag 2 set (rel 0 from SM2)"
        );
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
        assert_eq!(
            pio.sm[0].sideset_pins & (1 << 3),
            1 << 3,
            "side-set applied even on stalling instruction"
        );
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
        assert_eq!(
            pio.sm[0].shiftctrl & (1 << 16),
            0,
            "autopush disabled by default"
        );
        // Set IN_SHIFTDIR=0 (left)
        pio.sm[0].shiftctrl &= !(1 << 18);

        step_n(&mut pio, 1, 0); // SET X, 15
        for _ in 0..4 {
            step_n(&mut pio, 1, 0); // IN X, 8
        }
        // isr_count saturates at 32
        assert_eq!(pio.sm[0].isr_count, 32);
        assert!(
            pio.sm[0].rx_fifo.is_empty(),
            "no autopush with default shiftctrl"
        );
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
        assert_eq!(
            pio.sm[0].osr_count, 8,
            "8 bits shifted out after autopull refill"
        );
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
        assert!(
            pio.sm[0].stalled,
            "SM stalls when autopull finds empty FIFO"
        );
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
        assert_eq!(
            pio.shared_pin_values & expected_mask,
            expected_mask,
            "OUT PINS with out_base=5 should set pins [8:5]"
        );
        // Other pins should be 0
        assert_eq!(
            pio.shared_pin_values & !expected_mask,
            0,
            "only pins [8:5] should be set"
        );
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
        assert_eq!(
            pio.shared_pin_values, expected,
            "OUT PINS with out_base=30 should wrap to bits [31:30] and [1:0]"
        );
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
        pio.recompute_any_sideset();

        step_n(&mut pio, 1, 0); // Execute SET X, 1 [side 1] [delay 3]
        assert_eq!(pio.sm[0].x, 1);
        assert_eq!(
            pio.sm[0].sideset_pins & (1 << 7),
            1 << 7,
            "sideset pin 7 set on execution"
        );

        // Check through all 3 delay cycles
        for cycle in 0..3 {
            assert_eq!(
                pio.sm[0].sideset_pins & (1 << 7),
                1 << 7,
                "sideset pin 7 persists during delay cycle {}",
                cycle
            );
            step_n(&mut pio, 1, 0);
        }
        // After delay completes, sideset_pins should still hold its value
        // (it's only overwritten by the next instruction's sideset)
        assert_eq!(
            pio.sm[0].sideset_pins & (1 << 7),
            1 << 7,
            "sideset pin 7 still set after delay completes"
        );
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
        pio.recompute_any_sideset();
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
        pio.recompute_any_sideset();
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
        pio.sm[0].pinctrl = 1u32 << 26;
        // SET PINDIRS, 1  (opcode=111, dest=100, data=00001) = 0xE081
        pio.write32(0x0D8, 0xE081, 0);

        pio.step(0);

        assert_ne!(
            pio.pad_oe & 1,
            0,
            "SET PINDIRS, 1 with SET_BASE=0 must drive pad_oe bit 0"
        );
    }

    // ====================================================================
    // Branch-coverage top-up for `pio/mod.rs` — see Stage 4 of
    // `wrk_docs/2026.04.23 - CC - Coverage Improvement Plan.md`.
    // ====================================================================

    /// DREQ helpers return false for out-of-range SM indices (line 258/268).
    #[test]
    fn dreq_helpers_reject_out_of_range_sm() {
        let pio = PioBlock::new();
        assert!(!pio.tx_dreq(4));
        assert!(!pio.tx_dreq(100));
        assert!(!pio.rx_dreq(4));
        assert!(!pio.rx_dreq(100));
        // In-range sanity: fresh SM has TX room and empty RX.
        assert!(pio.tx_dreq(0));
        assert!(!pio.rx_dreq(0)); // empty RX → no data to drain
    }

    /// `raw_intr_rp2040` / `raw_intr_rp2350`: exercise RXNEMPTY and TXNFULL
    /// on each SM index so every loop iteration's branches are visited
    /// (lines 192/195 and 214/217).
    #[test]
    fn raw_intr_covers_all_sm_fifo_states() {
        let mut pio = PioBlock::new();
        // SM0: RX has data. SM1: RX empty. SM2: TX full. SM3: TX empty.
        assert!(pio.sm[0].rx_fifo.push(0xAA));
        for _ in 0..4 {
            assert!(pio.sm[2].tx_fifo.push(0));
        }
        let intr40 = pio.raw_intr_rp2040();
        // RXNEMPTY at bits [7:4].
        assert_ne!(intr40 & (1 << 4), 0, "SM0 RXNEMPTY");
        assert_eq!(intr40 & (1 << 5), 0, "SM1 RX still empty");
        // TXNFULL at bits [11:8].
        assert_eq!(intr40 & (1 << 10), 0, "SM2 TX full → TXNFULL=0");
        assert_ne!(intr40 & (1 << 11), 0, "SM3 TX has room → TXNFULL=1");

        let intr35 = pio.raw_intr_rp2350();
        // RP2350: RXNEMPTY at [3:0], TXNFULL at [7:4].
        assert_ne!(intr35 & 0b0001, 0, "SM0 RXNEMPTY (RP2350)");
        assert_eq!(intr35 & 0b0010, 0, "SM1 RX empty (RP2350)");
        assert_eq!(intr35 & (1 << 6), 0, "SM2 TX full (RP2350)");
        assert_ne!(intr35 & (1 << 7), 0, "SM3 TX has room (RP2350)");
    }

    /// `fstat` covers every RX-full and TX-full arm (lines 497/503) by
    /// filling SM1's RX FIFO and SM3's TX FIFO.
    #[test]
    fn fstat_covers_rx_and_tx_full_arms() {
        let mut pio = PioBlock::new();
        for _ in 0..4 {
            assert!(pio.sm[1].rx_fifo.push(0xAA));
            assert!(pio.sm[3].tx_fifo.push(0xBB));
        }
        let fstat = pio.read32(0x004);
        // RXFULL at bits [3:0] → SM1 bit set.
        assert_ne!(fstat & (1 << 1), 0, "SM1 RXFULL");
        // TXFULL at bits [19:16] → SM3 bit set.
        assert_ne!(fstat & (1 << 19), 0, "SM3 TXFULL");
        // Inverse RXEMPTY/TXEMPTY confirms the full arms fired.
        assert_eq!(fstat & (1 << 9), 0, "SM1 RX not empty");
        assert_eq!(fstat & (1 << 27), 0, "SM3 TX not empty");
    }

    /// `apply_fifo_join` FJOIN_RX branch (line 531): RX grows to depth 8,
    /// TX shrinks to 0.
    #[test]
    fn apply_fifo_join_rx_grows_rx_depth() {
        let mut pio = PioBlock::new();
        // Set FJOIN_RX (bit 31) in SHIFTCTRL for SM0.
        pio.write32(0x0D0, 1 << 31, 0);
        // RX FIFO should now accept 8 values.
        for v in 0..8u32 {
            assert!(pio.sm[0].rx_fifo.push(v));
        }
        assert!(pio.sm[0].rx_fifo.is_full());
        // TX FIFO is depth 0 — push drops.
        assert!(!pio.sm[0].tx_fifo.push(0xDEAD));
    }

    /// `apply_fifo_join` default-arm (line 535): clearing FJOIN bits
    /// restores 4/4 depth even if we previously set FJOIN_TX.
    #[test]
    fn apply_fifo_join_default_restores_balanced_depth() {
        let mut pio = PioBlock::new();
        // First force FJOIN_TX.
        pio.write32(0x0D0, 1 << 30, 0);
        for v in 0..8u32 {
            assert!(pio.sm[0].tx_fifo.push(v));
        }
        // Now clear both FJOIN bits — apply_fifo_join runs the `else` arm.
        pio.write32(0x0D0, 0, 0);
        // Depth-4: two pushes OK, fifth drops.
        for v in 0..4u32 {
            assert!(pio.sm[0].tx_fifo.push(v));
        }
        assert!(!pio.sm[0].tx_fifo.push(99));
        // RX depth also restored to 4.
        for v in 0..4u32 {
            assert!(pio.sm[0].rx_fifo.push(v));
        }
        assert!(!pio.sm[0].rx_fifo.push(99));
    }

    /// SHIFTCTRL alias write that doesn't change FJOIN: covers the
    /// `old_join == new_join` false arm of line 846.
    #[test]
    fn shiftctrl_write_without_join_change_skips_apply_fifo_join() {
        let mut pio = PioBlock::new();
        // Set up FJOIN_TX initially.
        pio.write32(0x0D0, 1 << 30, 0);
        // Push 8 values (depth=8) and pop 3 so we can distinguish from
        // a "flush-to-4" reset.
        for v in 0..8u32 {
            assert!(pio.sm[0].tx_fifo.push(v));
        }
        assert_eq!(pio.sm[0].tx_fifo.pop(), Some(0));
        assert_eq!(pio.sm[0].tx_fifo.pop(), Some(1));
        assert_eq!(pio.sm[0].tx_fifo.pop(), Some(2));
        assert_eq!(pio.sm[0].tx_fifo.level(), 5);
        // Write SHIFTCTRL with FJOIN_TX still set — no join change.
        pio.write32(0x0D0, (1 << 30) | (1 << 17), 0);
        // apply_fifo_join must NOT have been called (fifo not flushed).
        assert_eq!(
            pio.sm[0].tx_fifo.level(),
            5,
            "join-preserving write must not flush"
        );
    }

    /// SMn_EXECCTRL read with SM stalled: bit 31 (EXEC_STALLED) must
    /// reflect `stalled || delay > 0` (line 792 true arm).
    #[test]
    fn execctrl_read_shows_stalled_bit_when_sm_stalled() {
        let mut pio = PioBlock::new();
        // Force SM0 into a stalled state.
        pio.sm[0].stalled = true;
        let v = pio.read32(0x0CC); // SM0 EXECCTRL
        assert_ne!(v & 0x8000_0000, 0, "EXEC_STALLED must appear when stalled");
        // Clear stall, set delay_count>0 — still reports stalled.
        pio.sm[0].stalled = false;
        pio.sm[0].delay_count = 5;
        let v = pio.read32(0x0CC);
        assert_ne!(v & 0x8000_0000, 0, "EXEC_STALLED must appear when delaying");
    }

    /// Read / write of the per-SM ADDR register. ADDR is read-only so
    /// the write path must not store (line 851 `{}` arm) and the read
    /// returns the current PC.
    #[test]
    fn per_sm_addr_register_is_read_only_and_reports_pc() {
        let mut pio = PioBlock::new();
        pio.sm[0].pc = 9;
        assert_eq!(pio.read32(0x0D4), 9, "SMn_ADDR reads current PC");
        // Write to ADDR is a no-op (read-only).
        pio.write32(0x0D4, 22, 0);
        assert_eq!(pio.sm[0].pc, 9, "write to ADDR must be ignored");
    }

    /// Read of per-SM reserved region (reg = 0x18 would be out of
    /// range; we use an offset whose `reg` lands inside 0x18 — but
    /// since 0x0C8..=0x127 covers exactly 4 × 0x18, any offset
    /// inside the range has `reg` in 0..0x18). For safety we exercise
    /// the unaligned access directly: write32 with sm_offset % 0x18
    /// landing on a reserved reg (the wildcard `_` arm) is not
    /// possible via the public range — however, read/write of the
    /// unmodeled 0x128..=0x168 range exercises the surrounding
    /// wildcards.
    #[test]
    fn unmodeled_intblock_range_reads_zero_and_ignores_writes() {
        let mut pio = PioBlock::new();
        // 0x134 is inside the unmodeled `0x128..=0x168` range.
        assert_eq!(pio.read32(0x134), 0);
        pio.write32(0x134, 0xDEAD_BEEF, 0);
        assert_eq!(pio.read32(0x134), 0);
        // Out-of-range upper offset hits the wildcard `_ => 0`.
        assert_eq!(pio.read32(0x200), 0);
        // Wildcard write is a no-op.
        pio.write32(0x200, 0xFFFF_FFFF, 0);
        assert_eq!(pio.read32(0x200), 0);
    }

    /// CTRL write alias=1 (XOR) toggles selected SMs.
    #[test]
    fn ctrl_xor_alias_toggles_enable_bits() {
        let mut pio = PioBlock::new();
        // Enable SM0 and SM2 via plain write.
        pio.write32(0x000, 0b0101, 0);
        assert!(pio.sm[0].enabled);
        assert!(!pio.sm[1].enabled);
        assert!(pio.sm[2].enabled);
        // XOR with 0b0011: flip SM0 and SM1.
        pio.write32(0x000, 0b0011, 1);
        assert!(!pio.sm[0].enabled, "XOR toggled SM0 off");
        assert!(pio.sm[1].enabled, "XOR toggled SM1 on");
        assert!(pio.sm[2].enabled, "SM2 untouched");
    }

    /// CTRL write alias=2 (SET/OR) enables selected SMs without
    /// disturbing others.
    #[test]
    fn ctrl_set_alias_enables_indicated_sms() {
        let mut pio = PioBlock::new();
        pio.write32(0x000, 0b0001, 0); // enable SM0
        pio.write32(0x000, 0b0100, 2); // SET alias: add SM2 to enabled
        assert!(pio.sm[0].enabled);
        assert!(pio.sm[2].enabled);
        assert!(!pio.sm[1].enabled);
    }

    /// CTRL write alias=3 (CLR) disables selected SMs only.
    #[test]
    fn ctrl_clr_alias_disables_indicated_sms() {
        let mut pio = PioBlock::new();
        pio.write32(0x000, 0b1111, 0); // all four enabled
        pio.write32(0x000, 0b1010, 3); // CLR SM1 and SM3
        assert!(pio.sm[0].enabled);
        assert!(!pio.sm[1].enabled);
        assert!(pio.sm[2].enabled);
        assert!(!pio.sm[3].enabled);
    }

    /// CTRL SM_RESTART sets per-SM restart on SMs 1..3 too (not just SM0).
    /// Visits line 963 for each of bits 4..7 set in sm_restart_bits.
    #[test]
    fn ctrl_restart_clears_state_for_all_sms() {
        let mut pio = PioBlock::new();
        for i in 0..4 {
            pio.sm[i].pc = 5;
            pio.sm[i].x = 0xABCD;
            pio.sm[i].y = 0xDEAD;
            pio.sm[i].osr_count = 0;
        }
        // SM_RESTART bits [7:4] = all set.
        pio.write32(0x000, 0xF0, 0);
        for i in 0..4 {
            assert_eq!(pio.sm[i].pc, 0);
            assert_eq!(pio.sm[i].x, 0);
            assert_eq!(pio.sm[i].y, 0);
            assert_eq!(pio.sm[i].osr_count, 32, "osr_count reset to 32");
        }
    }

    /// CTRL CLKDIV_RESTART bit: covers line 980 `clkdiv_restart_bits`
    /// arm for each SM.
    #[test]
    fn ctrl_clkdiv_restart_clears_accumulator() {
        let mut pio = PioBlock::new();
        // Stuff non-zero accumulators.
        for i in 0..4 {
            pio.sm[i].clkdiv_acc = 0x1000 + i as u32;
        }
        // CLKDIV_RESTART bits [11:8] = all set.
        pio.write32(0x000, 0xF00, 0);
        for i in 0..4 {
            assert_eq!(pio.sm[i].clkdiv_acc, 0, "SM{i} clkdiv_acc cleared");
        }
    }

    /// `merge_pin_outputs` side-set path with SIDE_EN=1 (line 403) and
    /// actual_ss_pins > 0 (line 408). Also exercises the side_pindir=1
    /// direction arm (line 418) and the pad-change trace (line 436).
    #[test]
    fn merge_pin_outputs_side_en_and_side_pindir_arms() {
        let mut pio = PioBlock::new();
        // PINCTRL SM0: SIDESET_COUNT=2 (bits[31:29]=010), SIDESET_BASE=3
        // (bits[14:10]=3).
        pio.sm[0].pinctrl = (2u32 << 29) | (3u32 << 10);
        pio.recompute_any_sideset();
        // EXECCTRL: SIDE_EN=1 (bit 30), SIDE_PINDIR=1 (bit 29).
        pio.sm[0].execctrl = (1u32 << 30) | (1u32 << 29);
        // Pre-set sideset_dirs so the merge sees a non-zero overlay.
        pio.sm[0].sideset_dirs = 0b11 << 3;
        pio.set_sm_enabled(0, true);

        pio.step(0);

        // actual_ss_pins = SIDESET_COUNT - 1 = 1 when SIDE_EN=1. One pin
        // at SIDESET_BASE=3 → bit 3 of pad_oe must be driven by
        // sideset_dirs.
        assert_ne!(
            pio.pad_oe & (1 << 3),
            0,
            "SIDE_PINDIR=1 drives oe via sideset_dirs"
        );
    }

    /// `merge_pin_outputs` SIDESET_COUNT=5 (max) side-set-value path hits
    /// the actual_ss_pins >= 32 check falsely (path to line 413 else arm).
    /// SIDESET_COUNT caps at 5, which never reaches 32, so the `>=32`
    /// mask-arm is unreachable from the public API — we instead exercise
    /// actual_ss_pins=3 side-value drive by encoding a MOV Y,Y with
    /// side-set field = 0b101 into [12:8], so the instruction itself
    /// puts 0b101 into sideset_pins[4:2] via apply_sideset and the
    /// subsequent merge overlays it into pad_out.
    #[test]
    fn merge_pin_outputs_value_drive_with_multi_bit_sideset() {
        let mut pio = PioBlock::new();
        pio.sm[0].pinctrl = (3u32 << 29) | (2u32 << 10); // count=3, base=2
        pio.recompute_any_sideset();
        pio.sm[0].execctrl = 0; // SIDE_EN=0, SIDE_PINDIR=0
        // MOV Y,Y (0xA042) with delay/sideset field [12:8] = 0b10100:
        // delay_bits = 5-3 = 2, top 3 bits are the side-set value = 0b101,
        // bottom 2 are the delay = 0b00 → field = 0b10100 = 0x14.
        // insn = 0xA042 | (0x14 << 8) = 0xB442.
        pio.instr_mem[0] = 0xB442;
        pio.set_sm_enabled(0, true);

        pio.step(0);

        // Value-drive overlays side-set=0b101 into pad_out bits [4:2].
        assert_eq!(
            pio.pad_out & (0b111 << 2),
            0b101 << 2,
            "side-set bits 0b101 land at pad_out[4:2]"
        );
        // pad_oe untouched by value-drive side-set.
        assert_eq!(pio.pad_oe, 0);
    }

    /// FDEBUG W1C / XOR / SET / CLR alias arms. Covers each alias match
    /// arm of the FDEBUG dispatcher.
    #[test]
    fn fdebug_alias_arms() {
        let mut pio = PioBlock::new();
        pio.fdebug = 0xFF;
        // alias=0 (W1C) → clear bits.
        pio.write32(0x008, 0x0F, 0);
        assert_eq!(pio.fdebug, 0xF0);
        // alias=1 (XOR).
        pio.write32(0x008, 0xAA, 1);
        assert_eq!(pio.fdebug, 0xF0 ^ 0xAA);
        // alias=2 (SET).
        pio.fdebug = 0x10;
        pio.write32(0x008, 0x0F, 2);
        assert_eq!(pio.fdebug, 0x1F);
        // alias=3 (CLR).
        pio.fdebug = 0xFF;
        pio.write32(0x008, 0x11, 3);
        assert_eq!(pio.fdebug, 0xEE);
    }

    /// IRQ W1C / XOR / SET / CLR alias dispatcher (line 697–703).
    #[test]
    fn irq_flags_alias_arms() {
        let mut pio = PioBlock::new();
        pio.irq_flags = 0xFF;
        pio.write32(0x030, 0x0F, 0); // W1C
        assert_eq!(pio.irq_flags, 0xF0);
        pio.write32(0x030, 0x33, 1); // XOR
        assert_eq!(pio.irq_flags, 0xF0 ^ 0x33);
        pio.irq_flags = 0;
        pio.write32(0x030, 0x05, 2); // SET (OR)
        assert_eq!(pio.irq_flags, 0x05);
        pio.write32(0x030, 0x01, 3); // CLR
        assert_eq!(pio.irq_flags, 0x04);
    }

    /// IRQ0_INTE / IRQ0_INTF / IRQ1_INTE / IRQ1_INTF alias dispatcher
    /// (lines 729/731/733, 740/743/745, 753/755/757, 765/767/769).
    #[test]
    fn int_inte_intf_alias_arms() {
        let mut pio = PioBlock::new();
        // int0_inte
        pio.write32(0x170, 0x00FF, 0); // plain
        assert_eq!(pio.int0_inte, 0x00FF);
        pio.write32(0x170, 0x00F0, 1); // XOR
        assert_eq!(pio.int0_inte, 0x00FF ^ 0x00F0);
        pio.write32(0x170, 0x0F00, 2); // SET
        assert_eq!(pio.int0_inte & 0x0F00, 0x0F00);
        pio.write32(0x170, 0x000F, 3); // CLR
        assert_eq!(pio.int0_inte & 0x000F, 0);

        // int0_intf through its own alias arms.
        pio.write32(0x174, 0x0011, 0);
        pio.write32(0x174, 0x0010, 1);
        pio.write32(0x174, 0x1000, 2);
        pio.write32(0x174, 0x0001, 3);

        // int1_inte / int1_intf XOR / SET / CLR — identical shape.
        pio.write32(0x17C, 0x1234, 0);
        pio.write32(0x17C, 0x000F, 1);
        pio.write32(0x17C, 0xF000, 2);
        pio.write32(0x17C, 0x0004, 3);

        pio.write32(0x180, 0x5678, 0);
        pio.write32(0x180, 0x00FF, 1);
        pio.write32(0x180, 0x0F00, 2);
        pio.write32(0x180, 0x0010, 3);
    }

    /// INPUT_SYNC_BYPASS register round-trip (covers its read/write arms).
    #[test]
    fn input_sync_bypass_roundtrip() {
        let mut pio = PioBlock::new();
        pio.write32(0x038, 0xDEAD_BEEF, 0);
        assert_eq!(pio.read32(0x038), 0xDEAD_BEEF);
    }

    /// DBG_PADOUT / DBG_PADOE read-only arms: read returns current
    /// pad state; writes ignored.
    #[test]
    fn dbg_padout_padoe_are_read_only() {
        let mut pio = PioBlock::new();
        pio.pad_out = 0x11;
        pio.pad_oe = 0x22;
        assert_eq!(pio.read32(0x03C), 0x11);
        assert_eq!(pio.read32(0x040), 0x22);
        pio.write32(0x03C, 0xFFFF_FFFF, 0);
        pio.write32(0x040, 0xFFFF_FFFF, 0);
        assert_eq!(pio.pad_out, 0x11, "DBG_PADOUT write ignored");
        assert_eq!(pio.pad_oe, 0x22, "DBG_PADOE write ignored");
    }

    /// INT0_INTS / INT1_INTS are read-only (read computes, write is a no-op).
    #[test]
    fn ints_registers_are_read_only() {
        let mut pio = PioBlock::new();
        pio.write32(0x170, 0xFFFF, 0); // enable everything
        pio.irq_flags = 0x01; // set IRQ flag 0
        // INTS computes (INTR & INTE) | INTF — with irq_flags bit 0 set and
        // the RP2350 layout mapping IRQ flag 0 to bit 8 …
        let ints0 = pio.read32(0x178);
        assert_ne!(ints0 & 0x100, 0);
        // Write ignored.
        pio.write32(0x178, 0xDEAD, 0);
        assert_eq!(pio.read32(0x178), ints0, "INTS read unchanged after write");
    }

    /// step_n runs multiple PIO cycles when SMs are enabled. Covers the
    /// loop body (line 352) — the fast-path `if sm_enabled_mask == 0`
    /// false arm. Slot 0 runs SET X, 1 and slot 1 is implicit JMP 0
    /// (instr_mem default zero = JMP always to 0), so 10 cycles alternate.
    #[test]
    fn step_n_with_enabled_sm_runs_all_cycles() {
        let mut pio = PioBlock::new();
        pio.instr_mem[0] = 0xE021; // SET X, 1
        pio.set_sm_enabled(0, true);
        pio.step_n(10, 0);
        assert_eq!(pio.sm[0].x, 1);
        // 10 alternating cycles: slot 0 and slot 1 each visited 5 times.
        assert_eq!(pio.sm[0].pc_visits[0], 5);
        assert_eq!(pio.sm[0].pc_visits[1], 5);
    }

    /// TXF write to SM1/SM2 paths (lines 645/656) — the existing
    /// `test_fifo_push_pop` only exercises SM0's 0x010.
    #[test]
    fn txf_push_through_every_sm_offset() {
        let mut pio = PioBlock::new();
        pio.write32(0x010, 0x11, 0);
        pio.write32(0x014, 0x22, 0);
        pio.write32(0x018, 0x33, 0);
        pio.write32(0x01C, 0x44, 0);
        assert_eq!(pio.sm[0].tx_fifo.level(), 1);
        assert_eq!(pio.sm[1].tx_fifo.level(), 1);
        assert_eq!(pio.sm[2].tx_fifo.level(), 1);
        assert_eq!(pio.sm[3].tx_fifo.level(), 1);
        // And RXF read through each SM's offset.
        pio.sm[1].rx_fifo.push(0xA1);
        pio.sm[2].rx_fifo.push(0xA2);
        pio.sm[3].rx_fifo.push(0xA3);
        assert_eq!(pio.read32(0x024), 0xA1);
        assert_eq!(pio.read32(0x028), 0xA2);
        assert_eq!(pio.read32(0x02C), 0xA3);
    }

    /// Per-SM CLKDIV alias write: covers the alias-RMW branch on the
    /// CLKDIV path.
    #[test]
    fn per_sm_clkdiv_alias_rmw() {
        let mut pio = PioBlock::new();
        pio.write32(0x0C8, 0x0001_0000, 0); // SM0 CLKDIV = 1.0 (default)
        pio.write32(0x0C8, 0x0002_0000, 1); // XOR int field → 1 ^ 2 = 3
        assert_eq!(pio.read32(0x0C8), 0x0003_0000);
    }

    /// Per-SM INSTR alias write: force-executes the aliased result
    /// (line 857 — write path covers `force_execute` from alias).
    #[test]
    fn per_sm_instr_alias_write_force_executes() {
        let mut pio = PioBlock::new();
        // Pre-condition last_insn so the XOR has something to RMW against.
        pio.sm[0].last_insn = 0xE025; // SET X, 5
        // XOR with 0 — insn stays 0xE025, force-executes SET X, 5.
        pio.write32(0x0D8, 0, 1);
        assert_eq!(pio.sm[0].x, 5);
    }

    /// `any_sm_enabled` returns false on a fresh block and true after
    /// enabling an SM.
    #[test]
    fn any_sm_enabled_tracks_mask() {
        let mut pio = PioBlock::new();
        assert!(!pio.any_sm_enabled());
        assert_eq!(pio.sm_enabled_mask(), 0);
        pio.set_sm_enabled(2, true);
        assert!(pio.any_sm_enabled());
        assert_eq!(pio.sm_enabled_mask(), 0b0100);
        pio.set_sm_enabled(2, false);
        assert!(!pio.any_sm_enabled());
    }

    /// `set_sm_enabled` with `prev == enabled` short-circuits (no mask
    /// change, no merge).
    #[test]
    fn set_sm_enabled_no_change_is_noop() {
        let mut pio = PioBlock::new();
        // Already disabled — setting disabled is a no-op.
        pio.set_sm_enabled(0, false);
        assert_eq!(pio.sm_enabled_mask(), 0);
        pio.set_sm_enabled(0, true);
        let mask_before = pio.sm_enabled_mask();
        // Setting enabled again — no change.
        pio.set_sm_enabled(0, true);
        assert_eq!(pio.sm_enabled_mask(), mask_before);
    }

    /// `instr_mem` accessor returns the backing array.
    #[test]
    fn instr_mem_accessor_exposes_backing_array() {
        let mut pio = PioBlock::new();
        pio.instr_mem[7] = 0xBEEF;
        assert_eq!(pio.instr_mem()[7], 0xBEEF);
    }

    /// Test-only helpers `push_rx` / `pop_tx` round-trip.
    #[test]
    fn push_rx_and_pop_tx_test_hooks() {
        let mut pio = PioBlock::new();
        assert!(pio.push_rx(1, 0xCAFE));
        assert_eq!(pio.sm[1].rx_fifo.pop(), Some(0xCAFE));
        pio.sm[2].tx_fifo.push(0xBABE);
        assert_eq!(pio.pop_tx(2), Some(0xBABE));
    }

    /// Exercise every iteration of CTRL's per-SM read loop with a
    /// mix of enabled states (covers each `self.sm[i].enabled` branch
    /// at line 547 for i in 0..4).
    #[test]
    fn ctrl_read_visits_each_sm_enable_bit() {
        let mut pio = PioBlock::new();
        // Enable alternating: SM0, SM2 on; SM1, SM3 off.
        pio.set_sm_enabled(0, true);
        pio.set_sm_enabled(2, true);
        assert_eq!(pio.read32(0x000) & 0xF, 0b0101);
        // Flip: SM1, SM3 on; SM0, SM2 off.
        pio.set_sm_enabled(0, false);
        pio.set_sm_enabled(2, false);
        pio.set_sm_enabled(1, true);
        pio.set_sm_enabled(3, true);
        assert_eq!(pio.read32(0x000) & 0xF, 0b1010);
        // All on.
        pio.set_sm_enabled(0, true);
        pio.set_sm_enabled(2, true);
        assert_eq!(pio.read32(0x000) & 0xF, 0b1111);
    }

    /// Read-only register reads that aren't exercised elsewhere:
    /// covers the bare `=> self.field` match arms at offsets 0x008
    /// (FDEBUG), 0x034 (IRQ_FORCE write-only → 0 read), 0x170/0x174/
    /// 0x17C/0x180 (INTE/INTF registers), and 0x010..=0x01C (TXF
    /// write-only → 0 read).
    #[test]
    fn read_only_and_write_only_offsets_round_trip() {
        let mut pio = PioBlock::new();
        pio.fdebug = 0x1234_5678;
        pio.int0_inte = 0x0000_BEEF;
        pio.int0_intf = 0x0000_F00D;
        pio.int1_inte = 0x0000_CAFE;
        pio.int1_intf = 0x0000_BABE;
        assert_eq!(pio.read32(0x008), 0x1234_5678, "FDEBUG read");
        assert_eq!(pio.read32(0x034), 0, "IRQ_FORCE reads as 0");
        assert_eq!(pio.read32(0x170), 0x0000_BEEF, "int0_inte read");
        assert_eq!(pio.read32(0x174), 0x0000_F00D, "int0_intf read");
        assert_eq!(pio.read32(0x17C), 0x0000_CAFE, "int1_inte read");
        assert_eq!(pio.read32(0x180), 0x0000_BABE, "int1_intf read");
        // TXF read-only range returns 0.
        for off in [0x010u32, 0x014, 0x018, 0x01C] {
            assert_eq!(pio.read32(off), 0);
        }
        // Write to read-only offsets is a no-op.
        let fstat_before = pio.read32(0x004);
        pio.write32(0x004, 0xFFFF_FFFF, 0);
        assert_eq!(pio.read32(0x004), fstat_before, "FSTAT is read-only");
        let flevel_before = pio.read32(0x00C);
        pio.write32(0x00C, 0xFFFF_FFFF, 0);
        assert_eq!(pio.read32(0x00C), flevel_before, "FLEVEL is read-only");
        // INTR / INTS are read-only.
        let intr_before = pio.read32(0x16C);
        pio.write32(0x16C, 0, 0);
        assert_eq!(pio.read32(0x16C), intr_before, "INTR is read-only");
        let ints_before = pio.read32(0x178);
        pio.write32(0x178, 0, 0);
        assert_eq!(pio.read32(0x178), ints_before, "INT0_INTS is read-only");
        let ints1_before = pio.read32(0x184);
        pio.write32(0x184, 0, 0);
        assert_eq!(pio.read32(0x184), ints1_before, "INT1_INTS is read-only");
    }

    /// RXF read drains per-SM RX FIFOs for each SM offset (0x020..=0x02C).
    /// Existing tests only exercise SM0's 0x020 and SM3's implicitly —
    /// this round-trips each of SM1 and SM2 via their offsets.
    #[test]
    fn rxf_read_drains_per_sm_fifo_offsets() {
        let mut pio = PioBlock::new();
        assert!(pio.sm[1].rx_fifo.push(0xB1));
        assert!(pio.sm[2].rx_fifo.push(0xB2));
        assert_eq!(pio.read32(0x024), 0xB1);
        assert_eq!(pio.read32(0x028), 0xB2);
        // Empty FIFO drains to 0.
        assert_eq!(pio.read32(0x024), 0);
        assert_eq!(pio.read32(0x028), 0);
    }

    /// `step` on an idle block (sm_enabled_mask==0) short-circuits (line
    /// 323 true arm). Complements the step_n variant which short-circuits
    /// at its own guard and never calls `step`.
    #[test]
    fn step_on_idle_block_short_circuits() {
        let mut pio = PioBlock::new();
        let pad_before = pio.pad_out;
        pio.step(0);
        assert_eq!(pio.pad_out, pad_before, "idle step is a no-op");
    }

    /// `merge_pin_outputs` with SIDE_EN=1 and SIDESET_COUNT=1: actual_ss_pins
    /// collapses to 0, so the inner write block (line 408 false arm) is
    /// skipped.
    #[test]
    fn merge_pin_outputs_side_en_collapses_to_zero_pins() {
        let mut pio = PioBlock::new();
        // PINCTRL: SIDESET_COUNT=1 (bits[31:29]=001).
        pio.sm[0].pinctrl = 1u32 << 29;
        pio.recompute_any_sideset();
        // EXECCTRL: SIDE_EN=1 (bit 30), SIDE_PINDIR=0.
        pio.sm[0].execctrl = 1u32 << 30;
        pio.set_sm_enabled(0, true);
        // Run one step — any_sideset_programmed takes us down the
        // per-SM loop; for SM0 actual_ss_pins = 1-1 = 0 → inner block
        // skipped. For SM1..3 ss_count==0 → also skipped. pad_out/pad_oe
        // fall back to shared latches (0).
        pio.step(0);
        // shared_pin_values resets to u32::MAX (weak-pullup); with no
        // side-set overlay, pad_out mirrors that default.
        assert_eq!(
            pio.pad_out,
            u32::MAX,
            "pad_out passes through shared_pin_values"
        );
        assert_eq!(pio.pad_oe, 0, "no PINDIRS ever set, oe stays clear");
    }

    /// int0_ints_rp2040 / int1_ints_rp2040 surface the (INTR & INTE) | INTF
    /// computation for the RP2040 layout.
    #[test]
    fn int_ints_rp2040_layout_computes_from_inte_intf() {
        let mut pio = PioBlock::new();
        // IRQ flag 0 set → RP2040 INTR bit 0.
        pio.write32(0x034, 0x01, 0); // IRQ_FORCE
        pio.int0_inte = 0x001;
        pio.int0_intf = 0x002;
        let v0 = pio.int0_ints_rp2040();
        assert_eq!(v0 & 0x003, 0x003, "bit 0 via INTE, bit 1 via INTF");
        pio.int1_inte = 0x004;
        pio.int1_intf = 0x008;
        let v1 = pio.int1_ints_rp2040();
        assert_eq!(v1 & 0x008, 0x008, "INTF sets bit 3 on IRQ1 line");
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
        pio.recompute_any_sideset();
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
        assert_ne!(pio.pad_oe & 1, 0, "OE established by SET PINDIRS on tick 1");

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

    // ====================================================================
    // Coverage top-up: PIO block IRQ flag/mask interactions, CLKDIV
    // fractional boundaries, SM enable/disable ripple, INSTR_MEM
    // boundary indices, and DBG_PADOUT/DBG_PADOE post-step reads.
    // ====================================================================

    /// IRQ-flag interactions through the INTE/INTF mask: a flag bit must
    /// only surface in the effective `*_INTS` register when the matching
    /// INTE bit is set, while INTF bits ride through unconditionally.
    /// Covers the `(INTR & INTE) | INTF` composition for both NVIC lines
    /// on the RP2350 layout.
    #[test]
    fn pio_int_flag_mask_interactions_rp2350() {
        let mut pio = PioBlock::new();
        // Set IRQ flags 0..3 via IRQ_FORCE.
        pio.write32(0x034, 0x0F, 0);
        // INTE0 enables only flag 0 (bit 8 in RP2350 layout).
        pio.write32(0x170, 1 << 8, 0);
        // INTF0 forces bit 9 unconditionally.
        pio.write32(0x174, 1 << 9, 0);
        let ints0 = pio.read32(0x178);
        assert_ne!(ints0 & (1 << 8), 0, "flag 0 surfaces via INTE bit 8");
        assert_eq!(
            ints0 & (1 << 10),
            0,
            "flag 2 (bit 10) suppressed by INTE=0"
        );
        assert_ne!(ints0 & (1 << 9), 0, "INTF bit 9 forces ints regardless");
        // Clear INTE — flags drop out of INTS but INTF stays.
        pio.write32(0x170, 0, 0);
        let ints0 = pio.read32(0x178);
        assert_eq!(
            ints0 & (1 << 8),
            0,
            "flag 0 hidden once INTE bit 8 cleared"
        );
        assert_ne!(ints0 & (1 << 9), 0, "INTF survives INTE clearing");

        // RP2040 layout: flag 0 surfaces at bit 0 of INTR; verify via
        // int0_ints_rp2040 / int1_ints_rp2040 helpers paired with INTE
        // and INTF.
        pio.int0_inte = 0x001;
        pio.int0_intf = 0x010;
        pio.int1_inte = 0x002;
        pio.int1_intf = 0;
        let l0 = pio.int0_ints_rp2040();
        let l1 = pio.int1_ints_rp2040();
        assert_ne!(l0 & 0x001, 0, "RP2040 line 0: flag 0 enabled by INTE");
        assert_ne!(l0 & 0x010, 0, "RP2040 line 0: INTF bit 4 forces");
        assert_eq!(l1 & 0x001, 0, "RP2040 line 1: flag 0 not enabled");
    }

    /// CLKDIV fractional boundary at frac=255 (just under int+1). Verifies
    /// the divider's averaged tick rate over many cycles. With int=1
    /// frac=255, threshold = 256 + 255 = 511; +256 per cycle yields 256
    /// ticks per 511 cycles (≈1.996x divisor).
    #[test]
    fn clkdiv_frac_boundary_at_255_averages_correctly() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 255;
        let mut ticks = 0;
        // 5110 cycles → expected ticks = 5110 * 256 / 511 = 2560.
        for _ in 0..5110 {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 2560, "frac=255 averages 256/511 ticks per cycle");
    }

    /// CLKDIV at the int=0 (treated as 256) boundary: every cycle ticks
    /// because acc += 256 and threshold == 256. Independently verified at
    /// the SM level by `clock_tick_treats_int_zero_as_256`; this variant
    /// drives the divider through `PioBlock` to exercise the full path.
    #[test]
    fn clkdiv_int_zero_through_block_ticks_every_cycle() {
        let mut pio = PioBlock::new();
        pio.write32(0x0C8, 0, 0); // SM0 CLKDIV: int=0, frac=0 → divisor 256
        pio.set_sm_enabled(0, true);
        let mut ticks = 0;
        for _ in 0..1000 {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 1000, "int=0 must mean divide-by-256 → 1.0 effective");
    }

    /// CLKDIV maximum integer divisor (int=0xFFFF, frac=0). Verify the
    /// threshold computation does not overflow and the divisor produces
    /// roughly 1 tick per 65535 cycles. Sample 4 ticks deterministically.
    #[test]
    fn clkdiv_int_max_divisor_does_not_overflow() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 0xFFFF;
        pio.sm[0].clkdiv_frac = 0;
        let mut ticks = 0;
        // 4 * 65535 = 262140 cycles → 4 ticks.
        for _ in 0..(4 * 65535) {
            if pio.sm[0].clock_tick() {
                ticks += 1;
            }
        }
        assert_eq!(ticks, 4);
    }

    /// SM enable/disable ripple: enabling SM0 alone must not perturb other
    /// SMs' enable mask bits, and disabling all SMs (last one) clears the
    /// pad latches via merge_pin_outputs.
    #[test]
    fn sm_enable_disable_ripple_is_isolated() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.set_sm_enabled(1, true);
        pio.set_sm_enabled(2, true);
        pio.set_sm_enabled(3, true);
        assert_eq!(pio.sm_enabled_mask(), 0b1111);
        // Disable SM2 — others untouched.
        pio.set_sm_enabled(2, false);
        assert_eq!(pio.sm_enabled_mask(), 0b1011);
        assert!(pio.sm[0].enabled);
        assert!(pio.sm[1].enabled);
        assert!(!pio.sm[2].enabled);
        assert!(pio.sm[3].enabled);
        // Disabling all clears pad latches.
        pio.shared_pin_values = 0xFFFF;
        pio.shared_pin_dirs = 0xFFFF;
        pio.set_sm_enabled(0, false);
        pio.set_sm_enabled(1, false);
        pio.set_sm_enabled(3, false);
        assert_eq!(pio.sm_enabled_mask(), 0);
        // Pad latches are zeroed by merge_pin_outputs once mask hits 0.
        assert_eq!(pio.pad_out, 0);
        assert_eq!(pio.pad_oe, 0);
    }

    /// INSTR_MEM accessor at boundary indices 0 and 31. Writes via the
    /// register interface and reads back through `instr_mem()` accessor.
    #[test]
    fn instr_mem_boundary_indices_round_trip() {
        let mut pio = PioBlock::new();
        // Slot 0 (offset 0x048).
        pio.write32(0x048, 0xC0DE, 0);
        // Slot 31 (offset 0x048 + 31*4 = 0x0C4).
        pio.write32(0x0C4, 0xBEEF, 0);
        // Bounds are tight: slot 32 doesn't exist; we use the public
        // accessor to verify boundary slots.
        let mem = pio.instr_mem();
        assert_eq!(mem[0], 0xC0DE);
        assert_eq!(mem[31], 0xBEEF);
        // Slots between boundaries untouched.
        for i in 1..31 {
            assert_eq!(mem[i], 0, "slot {i} untouched");
        }
        // Writes only land within 32 slots — offset just past the
        // INSTR_MEM range (0x0C8) goes to per-SM CLKDIV instead.
        pio.write32(0x0C4, 0x1111, 0); // slot 31 again
        assert_eq!(pio.instr_mem()[31], 0x1111);
    }

    /// DBG_PADOUT / DBG_PADOE reads after a step that drives the pads.
    /// Confirms the read path returns the post-merge state, not stale
    /// pre-step values.
    #[test]
    fn dbg_padout_padoe_reflect_post_step_state() {
        let mut pio = PioBlock::new();
        // SET PINDIRS,1 at slot 0 + SET PINS,1 at slot 1, with SET_BASE=4
        // and SET_COUNT=1 so we drive bit 4 of pad_out + pad_oe.
        pio.sm[0].pinctrl = (1u32 << 26) | (4u32 << 5);
        pio.instr_mem[0] = 0xE081; // SET PINDIRS, 1
        pio.instr_mem[1] = 0xE001; // SET PINS, 1
        // Wrap fully around 5-bit memory so PC advances 0→1→2…
        pio.sm[0].execctrl = 0x1Fu32 << 12;
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 1;
        pio.sm[0].clkdiv_frac = 0;
        pio.step(0);
        // After SET PINDIRS,1 → bit 4 of pad_oe set; pad_out unchanged.
        assert_ne!(pio.read32(0x040) & (1 << 4), 0, "DBG_PADOE bit 4 set");
        pio.step(0);
        // After SET PINS,1 → bit 4 of pad_out set, pad_oe still set.
        assert_ne!(pio.read32(0x03C) & (1 << 4), 0, "DBG_PADOUT bit 4 set");
        assert_ne!(pio.read32(0x040) & (1 << 4), 0, "DBG_PADOE bit 4 stays set");
    }

    /// FDEBUG alias arm with `alias >= 4`: the dispatcher returns early
    /// (line 618 `_ => return`). This should be a no-op (no panic, no
    /// state change).
    #[test]
    fn fdebug_alias_out_of_range_is_noop() {
        let mut pio = PioBlock::new();
        pio.fdebug = 0xCAFE;
        pio.write32(0x008, 0xFF, 99); // alias=99 → early return
        assert_eq!(pio.fdebug, 0xCAFE, "out-of-range alias must not modify FDEBUG");
    }

    /// CLKDIV write sequence: setting integer alone (frac=0) yields exact
    /// 1/N ticks; restart via CTRL.CLKDIV_RESTART zeroes the accumulator
    /// so the next tick fires on schedule.
    #[test]
    fn clkdiv_restart_resets_phase() {
        let mut pio = PioBlock::new();
        pio.set_sm_enabled(0, true);
        pio.sm[0].clkdiv_int = 4;
        pio.sm[0].clkdiv_frac = 0;
        // Run one tick, draining the accumulator.
        for _ in 0..4 {
            let _ = pio.sm[0].clock_tick();
        }
        // Acc lands on 0 after the threshold-cross subtraction; advance
        // by 1 to leave acc=256.
        let _ = pio.sm[0].clock_tick();
        assert_eq!(pio.sm[0].clkdiv_acc, 256);
        // CTRL CLKDIV_RESTART for SM0 (bit 8).
        pio.write32(0x000, 1 << 8, 0);
        assert_eq!(pio.sm[0].clkdiv_acc, 0, "CLKDIV_RESTART zeros acc");
    }
}
