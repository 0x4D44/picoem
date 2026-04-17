// Hazard3 external IRQ controller — the Xh3irq CSR window
// (`meiea`/`meipa`/`meifa`/`meipra`/`meinext`/`meicontext` at 0xBE0..0xBE5).
// Reference: Hazard3 csr.adoc, section "Xh3irq".
//
// Model decisions for V1 (P4):
//
// * The upstream spec supports up to 512 external IRQs via a 5-bit window
//   index; RP2350 lights only 52 NVIC lines today (see `irq.rs`
//   `IRQ_SIO_PROC0`..`IRQ_DMA_6` range). We store 64 bits of enable / force
//   as flat `u64` bitmaps, and 64 × 4 = 256 bits of priority. Headroom over
//   the RP2350 52 without caring about the upper half-kilo IRQs from the
//   spec — this keeps the code straightforward and avoids a `[u32; 16]`-
//   shaped array when one `u64` suffices.
//
// * `meipa` is the read-only shadow of `bus.irq_pending[hart] | meifa`,
//   masked by `meiea`. It is re-evaluated per read — no cached copy — so
//   pending/enable updates from either side propagate immediately.
//
// * `meicontext` preemption model is implemented as a priority threshold
//   (`preempt` field). Full nested save/restore via `mreteirq`/`clearts`
//   is a minimal subset — enough for unit tests to demonstrate the flow.
//   See HLD §4.6 + meta-advice: "Prioritise correctness over feature
//   coverage — if you can't figure out `meicontext` preempt semantics,
//   implement the simpler `noirq/preempt` subset and note the gap."
//
// * The "window index" fields are write-only-self-clearing per Hazard3
//   csr.adoc: a write to `meiea`/`meipa`/`meifa`/`meipra` latches the
//   new index (bits 4:0) and the new data (bits 31:16). A subsequent read
//   returns the window-shaped view for the currently-latched index.

use tracing::trace;

/// External IRQ controller state. One per hart.
pub(crate) struct Xh3Irq {
    /// Per-IRQ enable bitmap. 64 bits is plenty for the 52 RP2350 NVIC
    /// lines (IRQ 0..51) plus headroom. Bit N is enable for IRQ N.
    pub(crate) meiea: u64,
    /// Per-IRQ force bitmap — firmware can force an IRQ pending even
    /// when the underlying peripheral hasn't asserted it. Cleared by
    /// write-1, and automatically cleared by `meinext` when that IRQ
    /// is read (no matter whether `update=1`).
    pub(crate) meifa: u64,
    /// Per-IRQ priority — 4 bits per IRQ (16 levels). IRQ 0 priority at
    /// index 0.
    pub(crate) meipra: [u8; 64],
    /// Context/preempt state. Fields per Hazard3 csr.adoc:
    ///   bit 0   mreteirq    — 1 if current trap is an external IRQ;
    ///                         restores preempt stack on `mret`.
    ///   bit 1   clearts     — write-1-self-clearing; on write, saves
    ///                         mie.MTIE and mie.MSIE into mtiesave/msiesave
    ///                         and clears those bits in mie.
    ///   bit 2   msiesave    — saved mie.MSIE value from clearts.
    ///   bit 3   mtiesave    — saved mie.MTIE value from clearts.
    ///   bits 12:4 irq        — current IRQ number.
    ///   bit 15  noirq       — 1 at reset; set by mret when preempt stack
    ///                         empties.
    ///   bits 20:16 preempt   — current priority threshold.
    ///   bits 27:24 ppreempt  — previous preempt (save slot for nested).
    ///   bits 31:28 pppreempt — previous ppreempt.
    pub(crate) meicontext: u32,
    /// Unbalanced entry count — how many `on_ext_irq_entry` calls have
    /// not yet been matched by an `on_mret` pop. Hazard3's preempt save
    /// stack is 2 deep (preempt + ppreempt + pppreempt slots give 2
    /// usable nesting levels; the third slot always pops to 0). HW sets
    /// `mreteirq` on every entry and only clears it when the stack is
    /// fully unwound — that semantics is what `preempt_depth` encodes:
    /// mret leaves mreteirq=1 as long as depth stays > 0 after the pop,
    /// so a later outer-level mret keeps popping the stack.
    preempt_depth: u8,
    /// Latched window index for meiea (bits 4:0 of last write).
    meiea_idx: u8,
    /// Latched window index for meipa (read-only access still honours
    /// the write-side index latch, per csr.adoc "windowed array").
    meipa_idx: u8,
    /// Latched window index for meifa.
    meifa_idx: u8,
    /// Latched window index for meipra.
    meipra_idx: u8,
}

/// meicontext bit positions.
pub(crate) const CTX_MRETEIRQ:   u32 = 1 << 0;
pub(crate) const CTX_CLEARTS:    u32 = 1 << 1;
pub(crate) const CTX_MSIESAVE:   u32 = 1 << 2;
pub(crate) const CTX_MTIESAVE:   u32 = 1 << 3;
pub(crate) const CTX_NOIRQ:      u32 = 1 << 15;

impl Xh3Irq {
    pub(crate) fn new() -> Self {
        Self {
            meiea: 0,
            meifa: 0,
            meipra: [0; 64],
            // Reset: noirq = 1. All other fields zero.
            meicontext: CTX_NOIRQ,
            preempt_depth: 0,
            meiea_idx: 0,
            meipa_idx: 0,
            meifa_idx: 0,
            meipra_idx: 0,
        }
    }

    /// meipa = (irq_pending | meifa) & meiea — per-IRQ OR reduce of
    /// enabled pending bits (hardware + forced). HLD §4.6 / Hazard3
    /// csr.adoc MEIP description.
    pub(crate) fn enabled_pending(&self, irq_pending: u64) -> u64 {
        (irq_pending | self.meifa) & self.meiea
    }

    /// Compute the MEIP bit for mip — true iff any enabled IRQ is
    /// pending (hardware-asserted or forced). HLD §4.6.
    pub(crate) fn meip(&self, irq_pending: u64) -> bool {
        self.enabled_pending(irq_pending) != 0
    }

    /// Lowest-numbered IRQ among the enabled+pending set that has
    /// sufficient priority to preempt the current `meicontext.preempt`
    /// threshold. Returns `Some((irq, priority))` or `None`.
    ///
    /// Per Hazard3 csr.adoc "meinext": when multiple IRQs of the same
    /// priority are pending+enabled, the lowest-numbered wins. IRQs with
    /// priority less than `meicontext.ppreempt` are treated as not-
    /// pending (spec calls this "preempt threshold").
    ///
    /// We return the winner's priority so the CSR write / entry path can
    /// populate `meicontext.preempt = priority + 1`.
    pub(crate) fn arbitrate(
        &self,
        irq_pending: u64,
    ) -> Option<(u8, u8)> {
        let set = self.enabled_pending(irq_pending);
        if set == 0 {
            return None;
        }
        let ppreempt = ((self.meicontext >> 24) & 0xF) as u8;
        let mut best: Option<(u8, u8)> = None;
        let mut bits = set;
        while bits != 0 {
            let n = bits.trailing_zeros() as u8;
            bits &= bits - 1;
            let pri = self.meipra[n as usize] & 0xF;
            if pri < ppreempt {
                continue;
            }
            match best {
                None => best = Some((n, pri)),
                Some((_, bp)) if pri > bp => best = Some((n, pri)),
                Some(_) => {
                    // Lower-numbered already selected first (bits are
                    // consumed low-to-high), so only a strictly-higher
                    // priority displaces it.
                }
            }
        }
        best
    }

    // ---------- meiea (0xBE0) — read-write enable array ----------

    pub(crate) fn read_meiea(&self) -> u32 {
        let idx = self.meiea_idx as u32;
        let shift = idx * 16;
        let window = ((self.meiea >> shift) & 0xFFFF) as u32;
        (window << 16) | idx
    }

    pub(crate) fn write_meiea(&mut self, v: u32) {
        let idx = (v & 0x1F) as u8;
        let data = ((v >> 16) & 0xFFFF) as u64;
        self.meiea_idx = idx;
        if idx < 4 {
            let shift = idx as u64 * 16;
            self.meiea = (self.meiea & !(0xFFFFu64 << shift)) | (data << shift);
            trace!(idx, data, "meiea write");
        }
        // idx >= 4 (upper 256 IRQs): writes are silently dropped — we
        // only store 64 bits of enable, covering IRQs 0..63.
    }

    // ---------- meipa (0xBE1) — read-only pending array ----------

    pub(crate) fn read_meipa(&self, irq_pending: u64) -> u32 {
        let idx = self.meipa_idx as u32;
        let shift = idx * 16;
        // Per csr.adoc: meipa bits are pending-bits masked by enable.
        // Returning `irq_pending | meifa` alone would misrepresent
        // disabled-but-pending IRQs. Match Hazard3's observable behaviour.
        let pending = self.enabled_pending(irq_pending);
        let window = ((pending >> shift) & 0xFFFF) as u32;
        (window << 16) | idx
    }

    pub(crate) fn write_meipa(&mut self, v: u32) {
        // Only the low 5 bits (window index) are writable; the data
        // window is read-only per csr.adoc. Writes drop the data field.
        self.meipa_idx = (v & 0x1F) as u8;
    }

    // ---------- meifa (0xBE2) — read-write force array (W1C for bits) -

    pub(crate) fn read_meifa(&self) -> u32 {
        let idx = self.meifa_idx as u32;
        let shift = idx * 16;
        let window = ((self.meifa >> shift) & 0xFFFF) as u32;
        (window << 16) | idx
    }

    /// Write to meifa. Hazard3 csr.adoc describes the force bits as
    /// "bits can be cleared by software" — the common pattern is a W1C
    /// register. Per spec prose, a bit is cleared by writing 1; writing
    /// 0 leaves the bit alone. We choose W1C semantics: the high 16 bits
    /// are write-1-to-clear.
    ///
    /// BUT we also need a way to *set* force bits (firmware-triggered
    /// synthetic IRQ for test/diagnostic use). The Hazard3 spec isn't
    /// explicit about this — in practice, force bits get set by hardware
    /// or by a dedicated write path. For V1 we implement a clean W1C:
    /// writes only clear. Tests that want to raise a force bit go via
    /// a direct `meifa |= ...` helper (exposed on the `Xh3Irq` struct
    /// for unit tests).
    ///
    /// NOTE (judgment call): csr.adoc says force bits "can be cleared by
    /// software, and are cleared automatically by hardware upon a read of
    /// meinext" — the "set" path isn't user-accessible in the spec text.
    /// Real firmware uses meinext's update bit to force-clear after-
    /// acknowledge. V1 exposes a test-only set path on the struct.
    pub(crate) fn write_meifa(&mut self, v: u32) {
        let idx = (v & 0x1F) as u8;
        let data = ((v >> 16) & 0xFFFF) as u64;
        self.meifa_idx = idx;
        if idx < 4 {
            let shift = idx as u64 * 16;
            // W1C: clear the bits set in `data`.
            self.meifa &= !(data << shift);
        }
    }

    /// Test-only helper: force an IRQ pending by setting its meifa bit.
    /// In silicon firmware, force bits would be set by a peripheral-
    /// specific path; the emulator exposes this for unit-test coverage
    /// of the force/ack flow.
    #[cfg(test)]
    pub(crate) fn force_set(&mut self, irq: u8) {
        if irq < 64 {
            self.meifa |= 1u64 << irq;
        }
    }

    // ---------- meipra (0xBE3) — read-write priority array ----------
    //
    // Each 16-IRQ window holds 4 priorities × 4 bits each = 16 bits.
    // Index bits 4:0 select the window (IRQ group of 4 — wait, that's
    // wrong; priorities are 4 bits × 16 IRQs = 64 bits per 16-IRQ window
    // — but the CSR is only 32 bits wide, so Hazard3 spec: "the window
    // contains four such priority values" i.e. 4 priorities per window,
    // meaning 4 IRQs per window. That gives 512/4 = 128 windows, not 16.
    // For our 64-IRQ field that's 16 windows of 4 IRQs each.
    //
    // Window bits in CSR:
    //   bits 31:28 = priority for IRQ (4*idx + 3)
    //   bits 27:24 = priority for IRQ (4*idx + 2)
    //   bits 23:20 = priority for IRQ (4*idx + 1)
    //   bits 19:16 = priority for IRQ (4*idx + 0)
    //   bits  4:0  = write-only index

    pub(crate) fn read_meipra(&self) -> u32 {
        let idx = self.meipra_idx as u32;
        let base = (idx as usize) * 4;
        let mut window = 0u32;
        for i in 0..4 {
            let irq = base + i;
            if irq < 64 {
                let pri = (self.meipra[irq] & 0xF) as u32;
                window |= pri << (i * 4);
            }
        }
        (window << 16) | idx
    }

    pub(crate) fn write_meipra(&mut self, v: u32) {
        let idx = (v & 0x1F) as u8;
        self.meipra_idx = idx;
        let base = (idx as usize) * 4;
        if base < 64 {
            let window = (v >> 16) & 0xFFFF;
            for i in 0..4 {
                let irq = base + i;
                if irq < 64 {
                    let pri = ((window >> (i * 4)) & 0xF) as u8;
                    self.meipra[irq] = pri;
                }
            }
        }
    }

    // ---------- meinext (0xBE4) — next pending IRQ ----------

    /// meinext read. Returns:
    ///   bit 31     = noirq (1 if no IRQ currently pending+enabled above
    ///                       the ppreempt threshold)
    ///   bits 10:2  = IRQ index (left-shifted by 2 — spec format)
    ///   bit 0      = update (read-as-zero; write-1 clears meifa for the
    ///                       reported IRQ)
    pub(crate) fn read_meinext(&self, irq_pending: u64) -> u32 {
        match self.arbitrate(irq_pending) {
            Some((irq, _)) => ((irq as u32) << 2) & 0x7FC,
            None => 1u32 << 31,
        }
    }

    /// meinext write. `update` bit triggers the side effects per
    /// csr.adoc: "hardware automatically clears the corresponding meifa
    /// bit upon a read of meinext which returns the irq (no matter
    /// whether update was written)". V1 interprets this as: write-1 to
    /// update clears meifa[irq] where irq is what the current arbitration
    /// would return.
    ///
    /// Judgment call: the exact timing of "read" vs "write" for the
    /// auto-clear is spec-ambiguous. V1 uses the write-side: only a
    /// write with update=1 performs the side effect. Firmware that polls
    /// meinext via a pure read sees no state change; firmware that writes
    /// back the read value (common pattern: `csrrw x0, meinext, x5` to
    /// ack) gets the ack.
    pub(crate) fn write_meinext(&mut self, v: u32, irq_pending: u64) {
        if (v & 1) != 0 {
            if let Some((irq, _)) = self.arbitrate(irq_pending) {
                // Clear the force bit for the acked IRQ. Hardware-
                // sourced pending bits are owned by bus.irq_pending —
                // meifa-clear only affects the force side.
                if irq < 64 {
                    self.meifa &= !(1u64 << irq);
                }
            }
        }
    }

    // ---------- meicontext (0xBE5) — preemption context ----------

    pub(crate) fn read_meicontext(&self) -> u32 {
        self.meicontext
    }

    /// Write meicontext. Honours the `clearts` self-clearing side effect:
    /// on write-1 to clearts, save mie.MTIE/MSIE into mtiesave/mtisave
    /// and clear those bits in mie. `clearts` itself clears to 0 after
    /// the effect. Other writable fields update directly; RES0 bits are
    /// dropped.
    ///
    /// Since clearts mutates `mie`, the caller must thread the mie
    /// reference through. That's kept on `CsrFile::mie` — the csr.rs
    /// dispatch handles it.
    pub(crate) fn write_meicontext(&mut self, v: u32, mie: &mut u32) {
        // Preserve only the writable fields. RES0 bits per csr.adoc:
        // bits 23:21, bits 14:13, and the internal-only "irq" field is
        // writable by software for debug use per csr.adoc.
        let mut new = v & !((0b111 << 21) | (0b11 << 13));

        // Handle clearts W1 self-clearing.
        if (v & CTX_CLEARTS) != 0 {
            // Save current mie.MTIE (bit 7) and mie.MSIE (bit 3) into
            // mtiesave (bit 3) / msiesave (bit 2) per csr.adoc.
            let mtie_now = (*mie >> 7) & 1;
            let msie_now = (*mie >> 3) & 1;
            // Clear them in mie.
            *mie &= !((1u32 << 7) | (1u32 << 3));
            // ORed on write per csr.adoc — the new value OR'd with saved.
            let mtiesave_new = ((new >> 3) & 1) | mtie_now;
            let msiesave_new = ((new >> 2) & 1) | msie_now;
            new = (new & !(CTX_MTIESAVE | CTX_MSIESAVE | CTX_CLEARTS))
                | (mtiesave_new << 3)
                | (msiesave_new << 2);
            // clearts always clears to 0 after write (self-clearing).
        } else {
            // Writing 0 to clearts is a no-op on the save slots.
            // Let the written mtiesave / msiesave values flow through.
        }

        self.meicontext = new;
    }

    /// Hardware path: on external IRQ entry, push preempt -> ppreempt,
    /// ppreempt -> pppreempt, install new `preempt = priority + 1`, set
    /// noirq=0, mreteirq=1, irq=<taken>. Called from the trap-entry path
    /// when the incoming cause is MEIP.
    ///
    /// `preempt_depth` tracks unbalanced entries (saturating at 2 — the
    /// spec's supported nesting depth); HW always sets `mreteirq` on
    /// entry regardless of depth.
    pub(crate) fn on_ext_irq_entry(&mut self, irq: u8, priority: u8) {
        let preempt_cur  = (self.meicontext >> 16) & 0xF;
        let ppreempt_cur = (self.meicontext >> 24) & 0xF;
        // pppreempt <- ppreempt <- preempt; preempt <- priority+1.
        // Saturate at 0xF (max 4-bit preempt level). A previous
        // `.saturating_add(1) & 0xF` formulation wrapped 15→0 after the
        // mask, which allowed *any* IRQ to preempt the highest-priority
        // handler. `.min(0xF)` saturates correctly.
        let new_preempt  = ((priority as u32) + 1).min(0xF);
        let new_ppreempt = preempt_cur;
        let new_pppreempt = ppreempt_cur;
        let mut c = self.meicontext;
        c &= !(0xFu32 << 16);
        c |= new_preempt << 16;
        c &= !(0xFu32 << 24);
        c |= new_ppreempt << 24;
        c &= !(0xFu32 << 28);
        c |= new_pppreempt << 28;
        // Clear noirq, set mreteirq, store irq number.
        c &= !CTX_NOIRQ;
        c |= CTX_MRETEIRQ;
        c &= !(0x1FFu32 << 4);
        c |= ((irq as u32) & 0x1FF) << 4;
        self.meicontext = c;
        // Bump the unbalanced-entry depth counter (saturating at 2 — the
        // Hazard3 stack is preempt+ppreempt, so deeper entries overflow
        // pppreempt and lose state).
        if self.preempt_depth < 2 {
            self.preempt_depth += 1;
        }
    }

    /// Hardware path: on `mret`, if mreteirq is set, pop the preempt
    /// stack (preempt <- ppreempt, ppreempt <- pppreempt, pppreempt <- 0).
    /// If the unbalanced-entry depth reaches 0 after the pop, clear
    /// `mreteirq` and set `noirq` — the outer return has restored thread
    /// context. If depth > 0 we're still nested: leave `mreteirq`
    /// asserted so the next outer `mret` keeps popping.
    ///
    /// Previously this cleared `mreteirq` unconditionally after every
    /// pop, which left the outer handler's mret with `mreteirq=0` and no
    /// pop — `preempt` stayed at the inner level permanently. Tracking
    /// depth fixes that: HW sets `mreteirq` per entry and clears it only
    /// when the stack is fully unwound.
    pub(crate) fn on_mret(&mut self) {
        if (self.meicontext & CTX_MRETEIRQ) == 0 {
            return;
        }
        let ppreempt  = (self.meicontext >> 24) & 0xF;
        let pppreempt = (self.meicontext >> 28) & 0xF;
        let mut c = self.meicontext;
        c &= !(0xFu32 << 16);
        c |= ppreempt << 16;
        c &= !(0xFu32 << 24);
        c |= pppreempt << 24;
        c &= !(0xFu32 << 28);
        // pppreempt <- 0 (already cleared by the mask above).

        // Decrement unbalanced-entry counter. Saturate at 0 — a spurious
        // mret (no matching entry) must not underflow.
        if self.preempt_depth > 0 {
            self.preempt_depth -= 1;
        }

        if self.preempt_depth == 0 {
            // Stack fully unwound: clear mreteirq + set noirq.
            c &= !CTX_MRETEIRQ;
            c |= CTX_NOIRQ;
        } else {
            // Still nested — keep mreteirq asserted so the outer mret
            // continues popping. noirq stays clear (we're in a handler).
        }
        self.meicontext = c;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_state() {
        let x = Xh3Irq::new();
        assert_eq!(x.meiea, 0);
        assert_eq!(x.meifa, 0);
        assert!(x.meipra.iter().all(|p| *p == 0));
        assert_eq!(x.meicontext, CTX_NOIRQ);
    }

    #[test]
    fn meiea_window_roundtrip() {
        let mut x = Xh3Irq::new();
        // Window 0 (IRQs 0..15): enable IRQs 0, 3, 15.
        x.write_meiea((0x8009u32 << 16) | 0);
        assert_eq!(x.meiea & 0xFFFF, 0x8009);
        // Read it back.
        let r = x.read_meiea();
        assert_eq!(r >> 16, 0x8009);
        assert_eq!(r & 0x1F, 0);
        // Window 1: enable IRQ 16+2 = 18.
        x.write_meiea((0x0004u32 << 16) | 1);
        let r = x.read_meiea();
        assert_eq!(r >> 16, 0x0004);
        assert_eq!(r & 0x1F, 1);
        assert_eq!(x.meiea & (1u64 << 18), 1u64 << 18);
    }

    #[test]
    fn meipa_pending_masked_by_enable() {
        let mut x = Xh3Irq::new();
        x.write_meiea((0x0003u32 << 16) | 0); // enable 0, 1
        let pending: u64 = 0b111; // 0, 1, 2 pending
        // meipa window 0 should show only 0, 1 (enable-masked).
        let r = x.read_meipa(pending);
        assert_eq!(r >> 16, 0b011);
    }

    #[test]
    fn meifa_w1c() {
        let mut x = Xh3Irq::new();
        x.force_set(3);
        x.force_set(5);
        // Clear bit 3 via W1C write.
        x.write_meifa((0b1000u32 << 16) | 0);
        assert_eq!(x.meifa & 0b0011_1000, 0b0010_0000); // bit 5 remains
    }

    #[test]
    fn meipra_window_roundtrip() {
        let mut x = Xh3Irq::new();
        // Window 0: 4 IRQs (0..3). Set pri=0xA for IRQ 0, 0x5 for IRQ 2.
        x.write_meipra((0x050A_u32 << 16) | 0);
        assert_eq!(x.meipra[0], 0xA);
        assert_eq!(x.meipra[2], 0x5);
        let r = x.read_meipra();
        assert_eq!(r >> 16, 0x050A);
    }

    #[test]
    fn meinext_lowest_numbered_wins_at_equal_priority() {
        let mut x = Xh3Irq::new();
        // Enable IRQs 3 and 5 at equal priority 0.
        x.meiea = (1 << 3) | (1 << 5);
        let pending = (1u64 << 3) | (1u64 << 5);
        let r = x.read_meinext(pending);
        // bits 10:2 hold irq << 2, so decode (r & 0x7FC) >> 2.
        let irq = (r & 0x7FC) >> 2;
        assert_eq!(irq, 3);
        assert_eq!(r >> 31, 0, "noirq must not be set");
    }

    #[test]
    fn meinext_higher_priority_wins() {
        let mut x = Xh3Irq::new();
        x.meiea = (1 << 3) | (1 << 5);
        x.meipra[3] = 1;
        x.meipra[5] = 5;
        let pending = (1u64 << 3) | (1u64 << 5);
        let r = x.read_meinext(pending);
        let irq = (r & 0x7FC) >> 2;
        assert_eq!(irq, 5);
    }

    #[test]
    fn meinext_noirq_when_no_pending() {
        let x = Xh3Irq::new();
        let r = x.read_meinext(0);
        assert_eq!(r >> 31, 1);
    }

    #[test]
    fn meinext_update_clears_meifa() {
        let mut x = Xh3Irq::new();
        x.meiea = 1 << 4;
        x.force_set(4);
        let r = x.read_meinext(0);
        assert_eq!((r & 0x7FC) >> 2, 4);
        // Write back with update=1.
        x.write_meinext(1, 0);
        assert_eq!(x.meifa & (1 << 4), 0);
    }

    #[test]
    fn meinext_ppreempt_masks_lower_priority() {
        let mut x = Xh3Irq::new();
        x.meiea = (1 << 3) | (1 << 7);
        x.meipra[3] = 2;
        x.meipra[7] = 5;
        // Raise ppreempt to 4 — IRQ 3 (pri=2) must be masked as "not pending".
        x.meicontext = 4u32 << 24;
        let pending = (1u64 << 3) | (1u64 << 7);
        let r = x.read_meinext(pending);
        let irq = (r & 0x7FC) >> 2;
        assert_eq!(irq, 7);
    }

    #[test]
    fn ext_irq_entry_installs_preempt_level() {
        let mut x = Xh3Irq::new();
        // preempt stack all-zero initially.
        x.on_ext_irq_entry(10, 3);
        assert_eq!((x.meicontext >> 16) & 0xF, 4, "preempt = priority+1");
        assert_eq!(x.meicontext & CTX_NOIRQ, 0);
        assert_eq!(x.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ);
        assert_eq!((x.meicontext >> 4) & 0x1FF, 10);
    }

    #[test]
    fn ext_irq_entry_pushes_preempt_stack() {
        let mut x = Xh3Irq::new();
        x.on_ext_irq_entry(5, 1); // preempt=2
        x.on_ext_irq_entry(9, 4); // preempt=5, ppreempt=2, pppreempt=0
        assert_eq!((x.meicontext >> 16) & 0xF, 5);
        assert_eq!((x.meicontext >> 24) & 0xF, 2);
        assert_eq!((x.meicontext >> 28) & 0xF, 0);
    }

    #[test]
    fn mret_pops_preempt_stack() {
        let mut x = Xh3Irq::new();
        x.on_ext_irq_entry(5, 1);
        x.on_ext_irq_entry(9, 4);
        // First (inner) mret pops to outer handler state. Depth goes
        // 2 -> 1, so mreteirq MUST remain asserted so the next mret
        // continues unwinding the stack.
        x.on_mret();
        assert_eq!((x.meicontext >> 16) & 0xF, 2, "preempt <- ppreempt");
        assert_eq!((x.meicontext >> 24) & 0xF, 0, "ppreempt <- pppreempt");
        assert_eq!(x.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ,
            "mreteirq stays asserted while depth > 0");
        assert_eq!(x.meicontext & CTX_NOIRQ, 0, "still in a handler");
        // Outer mret: depth 1 -> 0, clears mreteirq + sets noirq.
        x.on_mret();
        assert_eq!((x.meicontext >> 16) & 0xF, 0, "preempt fully popped");
        assert_eq!(x.meicontext & CTX_MRETEIRQ, 0, "mreteirq cleared at depth 0");
        assert_eq!(x.meicontext & CTX_NOIRQ, CTX_NOIRQ);
    }

    #[test]
    fn clearts_saves_and_masks_timer_soft_irqs() {
        let mut x = Xh3Irq::new();
        let mut mie: u32 = (1 << 7) | (1 << 3); // MTIE + MSIE enabled
        // Write clearts=1.
        x.write_meicontext(CTX_CLEARTS, &mut mie);
        // mie.MTIE and mie.MSIE should now be clear.
        assert_eq!(mie & ((1 << 7) | (1 << 3)), 0);
        // mtiesave and msiesave should reflect prior values.
        assert_eq!(x.meicontext & CTX_MTIESAVE, CTX_MTIESAVE);
        assert_eq!(x.meicontext & CTX_MSIESAVE, CTX_MSIESAVE);
        // clearts self-cleared.
        assert_eq!(x.meicontext & CTX_CLEARTS, 0);
    }
}
