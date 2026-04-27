// Zicsr CSR read/write dispatch. HLD §4.5 trap rules:
//
//   * Unimplemented CSR              -> mcause=2
//   * csrrw to read-only (bits[11:10]==0b11) even with rd=x0 -> mcause=2
//   * csrrs / csrrc with rs1 != x0 to read-only -> mcause=2
//   * csrrs / csrrc with rs1 == x0 to read-only -> read allowed, no trap
//   * mstatus.MPP WARL: writes that aren't 0b11 round to 0b11 (no U/S in V1)
//   * mcause WARL: store bit 31 + the low LSBs matching legal causes
//   * mtval: hardwired 0 (writes ignored)
//
// V1 supports the minimum M-mode CSR set the RV32I executor needs:
// mstatus / mie / mip / mtvec / mscratch / mepc / mcause / mtval /
// mcountinhibit / mcycle / minstret plus the read-only constants
// (mhartid / misa / mvendorid / marchid / mimpid / mconfigptr).

use super::Hazard3;
use super::decode::CsrKind;
use crate::Bus;

pub(crate) const CSR_MSTATUS: u16 = 0x300;
pub(crate) const CSR_MISA: u16 = 0x301;
pub(crate) const CSR_MIE: u16 = 0x304;
pub(crate) const CSR_MTVEC: u16 = 0x305;
pub(crate) const CSR_MCOUNTINHIBIT: u16 = 0x320;
pub(crate) const CSR_MSCRATCH: u16 = 0x340;
pub(crate) const CSR_MEPC: u16 = 0x341;
pub(crate) const CSR_MCAUSE: u16 = 0x342;
pub(crate) const CSR_MTVAL: u16 = 0x343;
pub(crate) const CSR_MIP: u16 = 0x344;
// Machine counters
pub(crate) const CSR_MCYCLE: u16 = 0xB00;
pub(crate) const CSR_MINSTRET: u16 = 0xB02;
pub(crate) const CSR_MCYCLEH: u16 = 0xB80;
pub(crate) const CSR_MINSTRETH: u16 = 0xB82;
// Read-only constants
pub(crate) const CSR_MVENDORID: u16 = 0xF11;
pub(crate) const CSR_MARCHID: u16 = 0xF12;
pub(crate) const CSR_MIMPID: u16 = 0xF13;
pub(crate) const CSR_MHARTID: u16 = 0xF14;
pub(crate) const CSR_MCONFIGPTR: u16 = 0xF15;
// Hazard3 Xh3irq external-IRQ controller (P4).
pub(crate) const CSR_MEIEA: u16 = 0xBE0;
pub(crate) const CSR_MEIPA: u16 = 0xBE1;
pub(crate) const CSR_MEIFA: u16 = 0xBE2;
pub(crate) const CSR_MEIPRA: u16 = 0xBE3;
pub(crate) const CSR_MEINEXT: u16 = 0xBE4;
pub(crate) const CSR_MEICONTEXT: u16 = 0xBE5;

// Physical Memory Protection (`pmpcfg0..3`, `pmpaddr0..15`) per RV-priv
// 1.12 §3.7. Phase-2 models the Hazard3 PMP as a CSR register bank only —
// writes and reads are WARL-modelled (including L-bit sticky-lock and
// TOR cross-entry lock) but no access-fault enforcement is wired to
// fetch/load/store (Hazard3 V1 is M-mode only, so PMP enforcement is
// architecturally a no-op even when set). See
// `wrk_docs/2026.04.18 - HLD - RISC-V PMP Coverage V1.md` (phase-1) and
// the V2 addendum that opens phase-2.
//
// `PMP_NUM_ENTRIES = 8` matches RP2350 datasheet §3.8 (Hazard3 synthesis
// parameter `PMP_REGIONS = 11`, of which entries 0..7 are dynamically
// configurable; entries 8..10 are "hardwired" — configured by the RP2350
// bootrom at boot, not silicon-const — and entries 11..15 are hardwired
// OFF). Phase-2 emulator models only the 8 dynamic entries; 8..15 are
// RAZ/WI, which matches QEMU at cold reset (no bootrom).
//
// Granule: G=0 in the emulator. Silicon ships G=3 (32-byte granule) per
// datasheet §3.8, but QEMU 10.2 rv32 defaults to G=0 and does NOT expose
// `pmp-grain` as a CPU property — so matching silicon here would diverge
// from QEMU on every pmpaddr readback. G=3 is deferred until a silicon
// RISC-V diff oracle exists (no such oracle today — `test_rp2350_probe_*`
// covers ARM only). See HLD V2 §A.5.
pub(crate) const CSR_PMPCFG0: u16 = 0x3A0;
pub(crate) const CSR_PMPCFG3: u16 = 0x3A3;
pub(crate) const CSR_PMPADDR0: u16 = 0x3B0;
pub(crate) const CSR_PMPADDR15: u16 = 0x3BF;
pub(crate) const PMP_NUM_ENTRIES: usize = 8;
/// pmpcfg reserved bits [6:5] per byte (Smepmp, not implemented by Hazard3)
/// — masked to zero on write.
const PMPCFG_RESERVED_BITS: u8 = 0b0110_0000;
/// pmpcfg L bit (bit 7). Once set, the byte itself and its paired pmpaddr
/// are locked until system reset (RV-priv §3.7.1). Phase-2 models this.
const PMPCFG_L_BIT: u8 = 0b1000_0000;
/// pmpcfg A field (bits [4:3]) mask.
const PMPCFG_A_MASK: u8 = 0b0001_1000;
/// A-field value for TOR (top-of-range) — entry i's range uses
/// `pmpaddr[i-1]` as its lower bound, so locking entry i with A=TOR also
/// locks entry i-1's pmpaddr.
const PMPCFG_A_TOR: u8 = 0b0000_1000;

/// mstatus writable mask. V1 supports MIE (bit 3), MPIE (bit 7),
/// MPP (bits [12:11], WARL to 0b11). All other bits (SIE/UIE/MPRV/
/// Secure-extension bits) read as 0 and ignore writes.
const MSTATUS_MIE: u32 = 1 << 3;
const MSTATUS_MPIE: u32 = 1 << 7;
const MSTATUS_MPP: u32 = 0b11 << 11;
pub(crate) const MSTATUS_WRITE_MASK: u32 = MSTATUS_MIE | MSTATUS_MPIE | MSTATUS_MPP;

/// mie writable mask — only MSIE (3), MTIE (7), MEIE (11). Bits outside
/// the standard M-mode triple are ignored (V1 is M-only).
const MIE_MASK: u32 = (1 << 3) | (1 << 7) | (1 << 11);
/// mip writable mask. On Hazard3 / RP2350 all three standard M-mode
/// pending bits are hardware-driven:
///   * MEIP (bit 11) — driven by the Xh3irq controller
///     (`compute_meip` / `fan_out_riscv_irqs`).
///   * MSIP (bit 3)  — driven by SIO's `RISCV_SOFTIRQ` register via
///     `fan_out_riscv_irqs`.
///   * MTIP (bit 7)  — driven by SIO's MTIME/MTIMECMP comparator via
///     `fan_out_riscv_irqs`.
/// None of these are firmware-writable: any bits a CSR write set would
/// be stomped within a single quantum by `fan_out_riscv_irqs`, so the
/// visible effect is zero. Worse, firmware setting MEIP with no matching
/// `meifa`/`meiea` winner would trap with no arbitration context — see
/// the MEIP arbitration gate in `mod.rs::step` for the fall-through.
///
/// Clean HW/firmware separation: firmware manipulates soft-IRQ via
/// `SIO.RISCV_SOFTIRQ` and forced ext-IRQs via `meifa`; `mip` itself is
/// observation-only from firmware's POV.
const MIP_MASK: u32 = 0;

/// Result of a CSR access. `Trap` indicates the executor must raise
/// mcause=2 (illegal instruction) at the current PC without updating rd.
pub(crate) enum CsrAccess {
    /// Old value to be written back into rd (x0 is filtered by caller).
    Ok(u32),
    /// Illegal instruction trap per §4.5 rules.
    Trap,
}

/// Return true if the CSR address is read-only per RV-priv (bits [11:10]==0b11).
#[inline]
fn is_read_only(csr: u16) -> bool {
    (csr >> 10) & 0b11 == 0b11
}

/// Dispatch a Zicsr instruction. `rs1_or_zimm` is the 5-bit source field
/// (register index for register forms, zero-extended immediate for `*i`
/// forms). `rs1_val` is the executor's resolved source register value
/// for register forms — ignored for immediate forms.
pub(crate) fn csr_access(
    hart: &mut Hazard3,
    bus: &Bus,
    kind: CsrKind,
    csr: u16,
    rs1_or_zimm: u8,
    rs1_val: u32,
) -> CsrAccess {
    let ro = is_read_only(csr);
    let is_imm = matches!(kind, CsrKind::Csrrwi | CsrKind::Csrrsi | CsrKind::Csrrci);
    let src = if is_imm { rs1_or_zimm as u32 } else { rs1_val };
    let is_write_like = match kind {
        CsrKind::Csrrw | CsrKind::Csrrwi => true, // always writes
        CsrKind::Csrrs | CsrKind::Csrrsi | CsrKind::Csrrc | CsrKind::Csrrci => rs1_or_zimm != 0,
    };

    // Trap gate per §4.5.
    if ro {
        match kind {
            // csrrw always traps on RO even with rd=x0 (write side is illegal).
            CsrKind::Csrrw | CsrKind::Csrrwi => return CsrAccess::Trap,
            CsrKind::Csrrs | CsrKind::Csrrsi | CsrKind::Csrrc | CsrKind::Csrrci => {
                if rs1_or_zimm != 0 {
                    return CsrAccess::Trap;
                }
                // rs1==x0 / zimm==0: RO read, no write side effect.
            }
        }
    }

    // Read old value. Unimplemented CSR -> trap.
    let irq_pending = bus.atomics.irq_pending_load(hart.hart_id as usize);
    let old = match read_csr(hart, csr, irq_pending) {
        Some(v) => v,
        None => return CsrAccess::Trap,
    };

    // Compute new value and write back for non-RO (or RO no-op).
    if is_write_like && !ro {
        let new = match kind {
            CsrKind::Csrrw | CsrKind::Csrrwi => src,
            CsrKind::Csrrs | CsrKind::Csrrsi => old | src,
            CsrKind::Csrrc | CsrKind::Csrrci => old & !src,
        };
        // write_csr is total over the supported set — unknown CSRs were
        // already caught by read_csr. WARL rounding lives inside the
        // per-CSR write path.
        write_csr(hart, csr, new, irq_pending);
    }

    CsrAccess::Ok(old)
}

/// Read a CSR. Returns `None` for unimplemented CSRs (executor turns
/// into mcause=2).
fn read_csr(hart: &Hazard3, csr: u16, irq_pending: u64) -> Option<u32> {
    Some(match csr {
        CSR_MSTATUS => hart.csrs.mstatus,
        CSR_MISA => hart.misa(),
        CSR_MIE => hart.csrs.mie,
        CSR_MTVEC => hart.csrs.mtvec,
        CSR_MCOUNTINHIBIT => hart.csrs.mcountinhibit,
        CSR_MSCRATCH => hart.csrs.mscratch,
        CSR_MEPC => hart.csrs.mepc,
        CSR_MCAUSE => hart.csrs.mcause,
        CSR_MTVAL => hart.csrs.mtval, // hardwired 0
        CSR_MIP => hart.csrs.mip,
        CSR_MCYCLE => hart.csrs.mcycle as u32,
        CSR_MINSTRET => hart.csrs.minstret as u32,
        CSR_MCYCLEH => (hart.csrs.mcycle >> 32) as u32,
        CSR_MINSTRETH => (hart.csrs.minstret >> 32) as u32,
        CSR_MVENDORID => hart.mvendorid(),
        CSR_MARCHID => hart.marchid(),
        CSR_MIMPID => hart.mimpid(),
        CSR_MHARTID => hart.mhartid(),
        CSR_MCONFIGPTR => hart.mconfigptr(),
        // Hazard3 Xh3irq CSRs. `meinext` / `meipa` read from
        // `bus.irq_pending | meifa` per HLD §4.6.
        CSR_MEIEA => hart.xh3irq.read_meiea(),
        CSR_MEIPA => hart.xh3irq.read_meipa(irq_pending),
        CSR_MEIFA => hart.xh3irq.read_meifa(),
        CSR_MEIPRA => hart.xh3irq.read_meipra(),
        CSR_MEINEXT => hart.xh3irq.read_meinext(irq_pending),
        CSR_MEICONTEXT => hart.xh3irq.read_meicontext(),
        // PMP register bank — phase-1 (NUM_ENTRIES=1). Unsynthesised
        // slots read as 0; synthesised slots return stored (already
        // WARL-masked) value. See `write_pmp_csr` for WARL rules.
        csr @ CSR_PMPCFG0..=CSR_PMPCFG3 => read_pmp_cfg(hart, csr),
        csr @ CSR_PMPADDR0..=CSR_PMPADDR15 => read_pmp_addr(hart, csr),
        _ => return None,
    })
}

/// Read a `pmpcfg*` CSR. For phase-1 (NUM_ENTRIES=1), only entry 0
/// (byte 0 of pmpcfg0) is live; all other bytes return 0.
fn read_pmp_cfg(hart: &Hazard3, csr: u16) -> u32 {
    let idx = (csr - CSR_PMPCFG0) as usize;
    hart.csrs.pmpcfg[idx]
}

/// Read a `pmpaddr*` CSR. For phase-1 (NUM_ENTRIES=1), only index 0 is
/// live; all others return 0.
fn read_pmp_addr(hart: &Hazard3, csr: u16) -> u32 {
    let idx = (csr - CSR_PMPADDR0) as usize;
    hart.csrs.pmpaddr[idx]
}

/// Write a CSR. Caller has already trap-gated read-only access; this
/// path only sees writable CSRs plus the no-op for hardwired-0 mtval.
fn write_csr(hart: &mut Hazard3, csr: u16, val: u32, irq_pending: u64) {
    match csr {
        CSR_MSTATUS => {
            // Apply writable mask; round MPP (bits [12:11]) to 0b11 per
            // WARL — no U/S mode in V1, so any non-0b11 write folds up.
            let masked = val & MSTATUS_WRITE_MASK;
            // If the incoming MPP field isn't 0b11, round up.
            let mpp_bits = (masked & MSTATUS_MPP) >> 11;
            let fixed_mpp = if mpp_bits == 0b11 {
                masked
            } else {
                (masked & !MSTATUS_MPP) | (0b11 << 11)
            };
            hart.csrs.mstatus = fixed_mpp;
        }
        CSR_MIE => hart.csrs.mie = val & MIE_MASK,
        CSR_MTVEC => {
            // Bit 1 hardwired 0 (HLD §4.3). Bit 0 is MODE (0=direct,
            // 1=vectored). Base field is word-aligned — bits [31:2]. We
            // store `(base & !0b11) | (mode & 0b1)`.
            hart.csrs.mtvec = (val & !0b11) | (val & 0b1);
        }
        CSR_MCOUNTINHIBIT => {
            // Bits CY (0) and IR (2) writable. Bit 1 reserved (hardwired 0).
            hart.csrs.mcountinhibit = val & 0b101;
        }
        CSR_MSCRATCH => hart.csrs.mscratch = val,
        CSR_MEPC => {
            // With C-extension, bit 1 of mepc is writable (any 2-byte
            // aligned address is a valid resumption point). Only bit 0 is
            // hardwired 0.
            hart.csrs.mepc = val & !0b1;
        }
        CSR_MCAUSE => {
            // WARL: keep bit 31 (interrupt flag) + full code field only
            // when the code is one of the implemented causes. Legal
            // exception causes for V1: 0,1,2,3,4,5,6,7,11. Legal
            // interrupt causes: 3 (MSI), 7 (MTI), 11 (MEI). Anything
            // else (e.g. 99, or a bit-pattern that happens to alias to
            // a legal low-nibble but has high bits set) rounds to 0.
            let interrupt = val & 0x8000_0000;
            let code = val & 0x7FFF_FFFF;
            let legal_code = if interrupt != 0 {
                // Interrupt — accept 3/7/11 exactly.
                if matches!(code, 3 | 7 | 11) { code } else { 0 }
            } else {
                // Exception — accept 0..=7, 11 exactly.
                if code <= 7 || code == 11 { code } else { 0 }
            };
            hart.csrs.mcause = interrupt | legal_code;
        }
        CSR_MTVAL => {
            // Hardwired 0 per HLD §4.3 — writes ignored.
            let _ = val;
        }
        CSR_MIP => {
            // Firmware can touch the software-interrupt side; the hardware
            // side (fan_out_riscv_irqs) will overwrite MSIP/MTIP on the
            // next quantum boundary. Still mask to the supported bits.
            hart.csrs.mip = (hart.csrs.mip & !MIP_MASK) | (val & MIP_MASK);
        }
        CSR_MCYCLE => hart.csrs.mcycle = (hart.csrs.mcycle & !0xFFFF_FFFF) | val as u64,
        CSR_MINSTRET => hart.csrs.minstret = (hart.csrs.minstret & !0xFFFF_FFFF) | val as u64,
        CSR_MCYCLEH => hart.csrs.mcycle = (hart.csrs.mcycle & 0xFFFF_FFFF) | ((val as u64) << 32),
        CSR_MINSTRETH => {
            hart.csrs.minstret = (hart.csrs.minstret & 0xFFFF_FFFF) | ((val as u64) << 32)
        }
        // Hazard3 Xh3irq CSRs.
        CSR_MEIEA => hart.xh3irq.write_meiea(val),
        CSR_MEIPA => hart.xh3irq.write_meipa(val),
        CSR_MEIFA => hart.xh3irq.write_meifa(val),
        CSR_MEIPRA => hart.xh3irq.write_meipra(val),
        CSR_MEINEXT => hart.xh3irq.write_meinext(val, irq_pending),
        CSR_MEICONTEXT => hart.xh3irq.write_meicontext(val, &mut hart.csrs.mie),
        // PMP register bank — phase-1 (NUM_ENTRIES=1). Writes to any
        // unsynthesised slot are silently dropped; writes to pmpcfg0
        // apply per-byte WARL rules (reserved [6:5] cleared, W=1/R=0
        // rounded to W=0/R=0). pmpaddr0 is fully writable (G=0).
        csr @ CSR_PMPCFG0..=CSR_PMPCFG3 => write_pmp_cfg(hart, csr, val),
        csr @ CSR_PMPADDR0..=CSR_PMPADDR15 => write_pmp_addr(hart, csr, val),
        // Read-only constants reached only via the RO-no-op read path;
        // write_csr is not called for them.
        _ => debug_assert!(false, "write_csr called for unsupported CSR {:#x}", csr),
    }
}

/// Write a `pmpcfg*` CSR. Per HLD V2 phase-2:
///   * entries >= PMP_NUM_ENTRIES — byte write silently dropped (stored
///     stays zero),
///   * if the currently-stored byte has L=1, the entire byte write is
///     dropped (sticky-lock; cleared only by `reset_pmp_csrs()` on
///     emulator, by system reset on silicon),
///   * bits [6:5] cleared (Smepmp reserved, unimplemented by Hazard3),
///   * W=1,R=0 rounded to W=0,R=0 per vendor spec and RV-priv §3.7.1 (a
///     write-without-read region is architecturally meaningless).
fn write_pmp_cfg(hart: &mut Hazard3, csr: u16, val: u32) {
    let cfg_idx = (csr - CSR_PMPCFG0) as usize;
    let cur = hart.csrs.pmpcfg[cfg_idx];
    let mut out: u32 = cur; // keep current; overwrite only unlocked bytes
    for byte in 0..4 {
        let entry = cfg_idx * 4 + byte;
        if entry >= PMP_NUM_ENTRIES {
            continue; // unsynthesised — byte stays 0
        }
        let cur_byte = ((cur >> (byte * 8)) & 0xFF) as u8;
        if (cur_byte & PMPCFG_L_BIT) != 0 {
            continue; // locked — drop this byte's write, keep cur_byte
        }
        let raw = ((val >> (byte * 8)) & 0xFF) as u8;
        let masked = warl_pmpcfg_byte(raw);
        out = (out & !(0xFFu32 << (byte * 8))) | ((masked as u32) << (byte * 8));
    }
    hart.csrs.pmpcfg[cfg_idx] = out;
}

/// WARL mask for one pmpcfg byte: clear reserved bits [6:5], then round
/// W=1,R=0 → W=0,R=0. L bit (7) is preserved in the write value — the
/// caller gates the entire byte on the *previously stored* L, so the
/// new L bit in the raw input is the "first-write" latch. Idempotent.
#[inline]
fn warl_pmpcfg_byte(b: u8) -> u8 {
    let v = b & !PMPCFG_RESERVED_BITS;
    // Bits [1:0] = R,W (R=bit0, W=bit1 per RV-priv §3.7.1). Illegal
    // combination W=1,R=0 → clear both.
    if (v & 0b11) == 0b10 { v & !0b11 } else { v }
}

/// Write a `pmpaddr*` CSR. Entries >= PMP_NUM_ENTRIES are silently
/// dropped. For synthesised entries, G=0 means every bit is writable,
/// modulo the L-bit gates per RV-priv §3.7.1:
///   * if entry i's own pmpcfg byte has L=1, pmpaddr[i] is locked,
///   * if entry i+1's pmpcfg byte has L=1 AND A=TOR, pmpaddr[i] is
///     locked (because entry i+1's TOR range uses pmpaddr[i] as its
///     lower bound).
fn write_pmp_addr(hart: &mut Hazard3, csr: u16, val: u32) {
    let idx = (csr - CSR_PMPADDR0) as usize;
    if idx >= PMP_NUM_ENTRIES {
        return; // unsynthesised — drop
    }
    if pmpaddr_is_locked(hart, idx) {
        return;
    }
    hart.csrs.pmpaddr[idx] = val;
}

/// Check both L-bit gates for `pmpaddr[idx]`.
fn pmpaddr_is_locked(hart: &Hazard3, idx: usize) -> bool {
    // Own-entry gate.
    if (pmpcfg_byte(hart, idx) & PMPCFG_L_BIT) != 0 {
        return true;
    }
    // TOR cross-entry gate: entry idx+1 with A=TOR locks pmpaddr[idx].
    let nxt = idx + 1;
    if nxt < PMP_NUM_ENTRIES {
        let nxt_byte = pmpcfg_byte(hart, nxt);
        if (nxt_byte & PMPCFG_L_BIT) != 0 && (nxt_byte & PMPCFG_A_MASK) == PMPCFG_A_TOR {
            return true;
        }
    }
    false
}

/// Extract pmpcfg byte for entry `idx`. `idx` must be < 16 (all CSR-addressable
/// positions); callers pre-gate on `PMP_NUM_ENTRIES` where relevant.
#[inline]
fn pmpcfg_byte(hart: &Hazard3, idx: usize) -> u8 {
    let cfg_word = hart.csrs.pmpcfg[idx / 4];
    ((cfg_word >> ((idx % 4) * 8)) & 0xFF) as u8
}
