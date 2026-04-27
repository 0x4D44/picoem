// Hazard3 CSR file. P1b stores the known fixed set of M-mode CSRs the
// V1 executor will read/write. P2 wires real access + trap rules per
// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`
// §4.5.

/// M-mode CSR storage for one hart. Plain field storage; trap rules,
/// WARL rounding, and hardwired zero enforcement (`mtval`) live in the
/// P2 CSR dispatch (`csr.rs`). P2 accesses every field.
///
/// Explicitly not a `HashMap` — the HLD pins this to a known fixed set.
pub(crate) struct CsrFile {
    /// Machine status. Reset: 0.
    pub mstatus: u32,
    /// Machine interrupt enable. Reset: 0.
    pub mie: u32,
    /// Machine interrupt pending. Reset: 0. Bits 3 (MSIP) / 7 (MTIP) are
    /// driven by `Emulator::fan_out_riscv_irqs` (P4); bit 11 (MEIP) is
    /// driven by the Hazard3 IRQ controller (P4).
    pub mip: u32,
    /// Machine trap-vector base + mode. Reset: `0x0000_1FFD` —
    /// BASE=`0x1FFC` + MODE=Vectored (1); bit 1 hardwired 0 (HLD §4.3,
    /// RP2350 datasheet table 373).
    pub mtvec: u32,
    /// Machine scratch. Reset: 0.
    pub mscratch: u32,
    /// Machine exception PC. Reset: 0.
    pub mepc: u32,
    /// Machine trap cause. Reset: 0.
    pub mcause: u32,
    /// Machine trap value. Hardwired 0 on Hazard3 (HLD §4.3; RP2350
    /// datasheet §3.8.4.1). P2 enforces the write no-op.
    pub mtval: u32,
    /// Machine counter-inhibit. Reset: `0b101` — CY (bit 0) + IR (bit 2)
    /// inhibited; bit 1 reserved (HLD §4.3, Hazard3 csr.adoc :369).
    /// Firmware clears to start `mcycle`/`minstret`.
    pub mcountinhibit: u32,
    /// Machine cycle counter. Reset: 0. Storage is separate from the
    /// core's own `cycles` field — `mcycle` is CSR-writable and
    /// gated by `mcountinhibit.CY`, whereas the core cycle field is the
    /// authoritative scheduler counter.
    pub mcycle: u64,
    /// Machine retired-instruction counter. Reset: 0. Gated by
    /// `mcountinhibit.IR`.
    pub minstret: u64,
    /// Packed PMP cfg bytes — entry i config lives in byte (i%4) of
    /// pmpcfg[i/4]. Phase-2 pins `PMP_NUM_ENTRIES = 8` (matches RP2350
    /// datasheet §3.8 — 8 dynamically configurable entries; entries 8..15
    /// are hardwired and RAZ/WI at cold reset on both emulator and QEMU).
    /// L-bit (bit 7) write-protects the byte and its paired pmpaddr per
    /// RV-priv §3.7.1; `Hazard3::reset_pmp_csrs()` clears L between fuzz
    /// cases.
    pub pmpcfg: [u32; 4],
    /// Per-entry PMP address. `pmpaddr[i]` for `i >= PMP_NUM_ENTRIES` is
    /// RAZ/WI. Phase-2 models L-bit gating (own-L + entry i+1 TOR-lock).
    pub pmpaddr: [u32; 16],
}

impl CsrFile {
    /// Construct a CSR file with all HLD §4.3 reset values applied.
    pub(crate) fn new() -> Self {
        Self {
            mstatus: 0,
            mie: 0,
            mip: 0,
            // BASE = 0x1FFC, MODE = Vectored (bit 0); bit 1 hardwired 0.
            mtvec: 0x0000_1FFD,
            mscratch: 0,
            mepc: 0,
            mcause: 0,
            mtval: 0,
            // CY (bit 0) + IR (bit 2) inhibited at reset.
            mcountinhibit: 0b101,
            mcycle: 0,
            minstret: 0,
            pmpcfg: [0; 4],
            pmpaddr: [0; 16],
        }
    }
}
