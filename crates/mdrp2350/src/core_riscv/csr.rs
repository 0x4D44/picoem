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

pub(crate) const CSR_MSTATUS:      u16 = 0x300;
pub(crate) const CSR_MISA:         u16 = 0x301;
pub(crate) const CSR_MIE:          u16 = 0x304;
pub(crate) const CSR_MTVEC:        u16 = 0x305;
pub(crate) const CSR_MCOUNTINHIBIT:u16 = 0x320;
pub(crate) const CSR_MSCRATCH:     u16 = 0x340;
pub(crate) const CSR_MEPC:         u16 = 0x341;
pub(crate) const CSR_MCAUSE:       u16 = 0x342;
pub(crate) const CSR_MTVAL:        u16 = 0x343;
pub(crate) const CSR_MIP:          u16 = 0x344;
// Machine counters
pub(crate) const CSR_MCYCLE:       u16 = 0xB00;
pub(crate) const CSR_MINSTRET:     u16 = 0xB02;
pub(crate) const CSR_MCYCLEH:      u16 = 0xB80;
pub(crate) const CSR_MINSTRETH:    u16 = 0xB82;
// Read-only constants
pub(crate) const CSR_MVENDORID:    u16 = 0xF11;
pub(crate) const CSR_MARCHID:      u16 = 0xF12;
pub(crate) const CSR_MIMPID:       u16 = 0xF13;
pub(crate) const CSR_MHARTID:      u16 = 0xF14;
pub(crate) const CSR_MCONFIGPTR:   u16 = 0xF15;

/// mstatus writable mask. V1 supports MIE (bit 3), MPIE (bit 7),
/// MPP (bits [12:11], WARL to 0b11). All other bits (SIE/UIE/MPRV/
/// Secure-extension bits) read as 0 and ignore writes.
const MSTATUS_MIE:  u32 = 1 << 3;
const MSTATUS_MPIE: u32 = 1 << 7;
const MSTATUS_MPP:  u32 = 0b11 << 11;
pub(crate) const MSTATUS_WRITE_MASK: u32 = MSTATUS_MIE | MSTATUS_MPIE | MSTATUS_MPP;

/// mie writable mask — only MSIE (3), MTIE (7), MEIE (11). Bits outside
/// the standard M-mode triple are ignored (V1 is M-only).
const MIE_MASK: u32 = (1 << 3) | (1 << 7) | (1 << 11);
/// mip writable mask — firmware can set/clear software-interrupt
/// visibility and force bits. The hardware side (fan_out_riscv_irqs)
/// drives MSIP/MTIP directly (HLD §4.6); CSR writes to those bits just
/// get OR'd in with whatever hardware sourced.
const MIP_MASK: u32 = (1 << 3) | (1 << 7) | (1 << 11);

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
fn is_read_only(csr: u16) -> bool { (csr >> 10) & 0b11 == 0b11 }

/// Dispatch a Zicsr instruction. `rs1_or_zimm` is the 5-bit source field
/// (register index for register forms, zero-extended immediate for `*i`
/// forms). `rs1_val` is the executor's resolved source register value
/// for register forms — ignored for immediate forms.
pub(crate) fn csr_access(
    hart: &mut Hazard3,
    kind: CsrKind,
    csr: u16,
    rs1_or_zimm: u8,
    rs1_val: u32,
) -> CsrAccess {
    let ro = is_read_only(csr);
    let is_imm = matches!(kind, CsrKind::Csrrwi | CsrKind::Csrrsi | CsrKind::Csrrci);
    let src = if is_imm { rs1_or_zimm as u32 } else { rs1_val };
    let is_write_like = match kind {
        CsrKind::Csrrw | CsrKind::Csrrwi => true,            // always writes
        CsrKind::Csrrs | CsrKind::Csrrsi
        | CsrKind::Csrrc | CsrKind::Csrrci => rs1_or_zimm != 0,
    };

    // Trap gate per §4.5.
    if ro {
        match kind {
            // csrrw always traps on RO even with rd=x0 (write side is illegal).
            CsrKind::Csrrw | CsrKind::Csrrwi => return CsrAccess::Trap,
            CsrKind::Csrrs | CsrKind::Csrrsi
            | CsrKind::Csrrc | CsrKind::Csrrci => {
                if rs1_or_zimm != 0 {
                    return CsrAccess::Trap;
                }
                // rs1==x0 / zimm==0: RO read, no write side effect.
            }
        }
    }

    // Read old value. Unimplemented CSR -> trap.
    let old = match read_csr(hart, csr) {
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
        write_csr(hart, csr, new);
    }

    CsrAccess::Ok(old)
}

/// Read a CSR. Returns `None` for unimplemented CSRs (executor turns
/// into mcause=2).
fn read_csr(hart: &Hazard3, csr: u16) -> Option<u32> {
    Some(match csr {
        CSR_MSTATUS       => hart.csrs.mstatus,
        CSR_MISA          => hart.misa(),
        CSR_MIE           => hart.csrs.mie,
        CSR_MTVEC         => hart.csrs.mtvec,
        CSR_MCOUNTINHIBIT => hart.csrs.mcountinhibit,
        CSR_MSCRATCH      => hart.csrs.mscratch,
        CSR_MEPC          => hart.csrs.mepc,
        CSR_MCAUSE        => hart.csrs.mcause,
        CSR_MTVAL         => hart.csrs.mtval,         // hardwired 0
        CSR_MIP           => hart.csrs.mip,
        CSR_MCYCLE        => hart.csrs.mcycle as u32,
        CSR_MINSTRET      => hart.csrs.minstret as u32,
        CSR_MCYCLEH       => (hart.csrs.mcycle >> 32) as u32,
        CSR_MINSTRETH     => (hart.csrs.minstret >> 32) as u32,
        CSR_MVENDORID     => hart.mvendorid(),
        CSR_MARCHID       => hart.marchid(),
        CSR_MIMPID        => hart.mimpid(),
        CSR_MHARTID       => hart.mhartid(),
        CSR_MCONFIGPTR    => hart.mconfigptr(),
        _ => return None,
    })
}

/// Write a CSR. Caller has already trap-gated read-only access; this
/// path only sees writable CSRs plus the no-op for hardwired-0 mtval.
fn write_csr(hart: &mut Hazard3, csr: u16, val: u32) {
    match csr {
        CSR_MSTATUS => {
            // Apply writable mask; round MPP (bits [12:11]) to 0b11 per
            // WARL — no U/S mode in V1, so any non-0b11 write folds up.
            let masked = val & MSTATUS_WRITE_MASK;
            // If the incoming MPP field isn't 0b11, round up.
            let mpp_bits = (masked & MSTATUS_MPP) >> 11;
            let fixed_mpp = if mpp_bits == 0b11 { masked } else {
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
            // mepc low 2 bits hardwired 0 (no C-extension in P2). When C
            // lands in P3, bit 1 becomes writable and this mask relaxes to
            // `val & !0b1`.
            hart.csrs.mepc = val & !0b11;
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
        CSR_MCYCLE   => hart.csrs.mcycle   = (hart.csrs.mcycle   & !0xFFFF_FFFF) | val as u64,
        CSR_MINSTRET => hart.csrs.minstret = (hart.csrs.minstret & !0xFFFF_FFFF) | val as u64,
        CSR_MCYCLEH  => hart.csrs.mcycle   = (hart.csrs.mcycle   & 0xFFFF_FFFF) | ((val as u64) << 32),
        CSR_MINSTRETH=> hart.csrs.minstret = (hart.csrs.minstret & 0xFFFF_FFFF) | ((val as u64) << 32),
        // Read-only constants reached only via the RO-no-op read path;
        // write_csr is not called for them.
        _ => debug_assert!(false, "write_csr called for unsupported CSR {:#x}", csr),
    }
}
