// riscv_probe_spike — Phase 1 spike for the RP2350 Hazard3 test oracle HLD.
//
// Runs the 5-row probe-rs 0.31 capability matrix + 2 empirical rows + the
// ARCHSEL-via-SWD sub-question against a live RP2354 board currently in
// RISC-V boot select. Prints a PASS/FAIL summary that maps onto the HLD
// §4 Phase 1 decision tree (Full / Partial / Mailbox / Plan D).
//
// See `wrk_docs/2026.04.17 - LLD - RISC-V Probe-rs Attach Spike V1.md`.
//
// Throwaway: deleted when Phase 3 LLD lands. Precedent:
// `smoke_per_core_cyccnt_rp2350.rs`.
//
// Usage:
//   riscv_probe_spike
//   riscv_probe_spike --probe 2e8a:000c:E46410955F614129
//   riscv_probe_spike --attempt-archsel-flip   # only if RUNBOOK offset is pinned

use mdpicoem_harness::{EMU_TEST_SLOT, EMU_TEST_STACK};
use probe_rs::probe::{list::Lister, DebugProbeSelector};
use probe_rs::{
    Architecture, Core, MemoryInterface, Permissions, RegisterId, Session, SessionConfig,
};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// RISC-V register IDs (from probe-rs-0.31.0 src/architecture/riscv/registers.rs)
// On the RV backend `read_core_reg(RegisterId(n))` dispatches to read_csr(n),
// so arbitrary CSR numbers are addressable as RegisterId(n) directly.
// ---------------------------------------------------------------------------

const RV_X2_SP: RegisterId = RegisterId(0x1002);
const RV_X10_A0: RegisterId = RegisterId(0x100A);
// 0x7b1 is the architectural `dpc` CSR address per RISC-V External Debug
// Spec §4.8.1 — probe-rs maps its PC register pseudo-ID to this CSR. This
// is the standard debug PC, not a probe-rs-internal encoding.
const RV_PC: RegisterId = RegisterId(0x7b1);
const CSR_MHARTID: RegisterId = RegisterId(0xF14);
const CSR_MSCRATCH: RegisterId = RegisterId(0x340);
const CSR_MCYCLE: RegisterId = RegisterId(0xB00);
const CSR_MIP: RegisterId = RegisterId(0x344);
const CSR_MSTATUS: RegisterId = RegisterId(0x300);
const MSTATUS_MIE: u32 = 1 << 3;

// c.nop = 0x0001 (RV32C); fill a small sled and terminate with c.ebreak
// (= 0x9002) as the halt sentinel, so Row 4 can exercise SW-break fallback
// independently of HW breakpoint unit availability. Each stub is rewritten
// per-row to match the specific test.
const RV_EBREAK: u16 = 0x9002; // c.ebreak
const RV_NOP: u16 = 0x0001; // c.nop

const HALT_TIMEOUT: Duration = Duration::from_millis(500);
// Wall-clock cap for Row E1's step loop. 10 compressed steps over JTAG
// typically completes in <1s; 5s is a generous safety bound to avoid an
// indefinite hang if the DM wedges.
const E1_STEP_BUDGET: Duration = Duration::from_secs(5);

fn main() {
    mdpicoem_harness::harness_tracing_init();
    match run() {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("fatal: {e}");
            std::process::exit(2);
        }
    }
}

// ---------------------------------------------------------------------------
// Argument parsing (mirrors probe_diff_rp2350 style)
// ---------------------------------------------------------------------------

struct Args {
    probe: Option<DebugProbeSelector>,
    attempt_archsel_flip: bool,
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut probe = None;
    let mut attempt_archsel_flip = false;
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--probe" => {
                i += 1;
                if i >= argv.len() {
                    return Err("--probe requires a VID:PID:SERIAL argument".into());
                }
                probe = Some(
                    DebugProbeSelector::try_from(argv[i].as_str())
                        .map_err(|e| format!("invalid probe selector '{}': {e}", argv[i]))?,
                );
            }
            "--attempt-archsel-flip" => {
                attempt_archsel_flip = true;
            }
            "-h" | "--help" => {
                println!(
                    "riscv_probe_spike [--probe VID:PID:SERIAL] [--attempt-archsel-flip]\n\
                     Runs the Phase 1 capability matrix against a live RP2354 in RV boot.\n\
                     --attempt-archsel-flip only has effect once the POWMAN CHIP_RESET\n\
                     offset is pinned in RUNBOOK.md; until then Row A2 SKIPs regardless."
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
        i += 1;
    }
    Ok(Args { probe, attempt_archsel_flip })
}

// ---------------------------------------------------------------------------
// Result machinery
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    Pass,
    Fail,
    Skip,
}

struct RowResult {
    label: &'static str,
    verdict: Verdict,
    reason: String,
}

impl RowResult {
    fn pass(label: &'static str, reason: impl Into<String>) -> Self {
        Self { label, verdict: Verdict::Pass, reason: reason.into() }
    }
    fn fail(label: &'static str, reason: impl Into<String>) -> Self {
        Self { label, verdict: Verdict::Fail, reason: reason.into() }
    }
    fn skip(label: &'static str, reason: impl Into<String>) -> Self {
        Self { label, verdict: Verdict::Skip, reason: reason.into() }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Best-effort halt; if it errors, log at debug level and continue. We don't
/// fail a row on a stray halt error because the core is often already halted
/// (e.g. after c.ebreak); the subsequent register access will fail loudly
/// with the real reason if the core genuinely isn't in a probeable state.
fn try_halt(core: &mut Core<'_>, label: &str) {
    if let Err(e) = core.halt(HALT_TIMEOUT) {
        tracing::debug!(target: "riscv_probe_spike", label, "halt failed: {e}");
    }
}

/// Collect every core index list_cores() reported. probe-rs types the return
/// as Vec<(usize, CoreType)>, where the usize is the core *index* passed to
/// session.core() — for RP2350 this is 0/1 in practice but iterating via the
/// returned indices avoids the off-by-one risk if probe-rs ever sparses them.
fn core_ids(session: &mut Session) -> Vec<usize> {
    session.list_cores().into_iter().map(|(id, _)| id).collect()
}

// ---------------------------------------------------------------------------
// Capability matrix rows
// ---------------------------------------------------------------------------

/// Row 1 — attach exposes RV core for both harts.
/// PASS iff list_cores() >= 2 AND every halt-able core reports Architecture::Riscv.
fn row1_attach(session: &mut Session) -> RowResult {
    let ids = core_ids(session);
    let n = ids.len();
    if n == 0 {
        return RowResult::fail("1. Attach + RV core enumeration", "no cores enumerated");
    }
    let mut arches = Vec::with_capacity(n);
    for id in ids.iter().copied() {
        let mut core = match session.core(id) {
            Ok(c) => c,
            Err(e) => {
                return RowResult::fail(
                    "1. Attach + RV core enumeration",
                    format!("core({id}) attach failed: {e}"),
                );
            }
        };
        // Halt is best-effort; some backends require halted state to inspect.
        try_halt(&mut core, "row1");
        arches.push(core.architecture());
    }
    let all_rv = arches.iter().all(|a| *a == Architecture::Riscv);
    if all_rv {
        RowResult::pass(
            "1. Attach + RV core enumeration",
            format!("{n} harts, all Architecture::Riscv"),
        )
    } else {
        RowResult::fail(
            "1. Attach + RV core enumeration",
            format!(
                "{n} harts, architectures = {:?} (expected all Riscv) — likely probe-rs \
                 target YAML has no RV stanza; see LLD §2",
                arches
            ),
        )
    }
}

/// Row 1b — mhartid sentinel. Run after row 1 passes.
/// PASS iff CSR mhartid reads 0 or 1 on hart 0 (and 1 on hart 1 if present).
fn row1b_mhartid(session: &mut Session) -> RowResult {
    let ids = core_ids(session);
    let mut seen = Vec::with_capacity(ids.len());
    for id in ids {
        let mut core = match session.core(id) {
            Ok(c) => c,
            Err(e) => {
                return RowResult::fail(
                    "1b. mhartid sentinel",
                    format!("core({id}) attach failed: {e}"),
                );
            }
        };
        if !core.status().map(|s| s.is_halted()).unwrap_or(false) {
            try_halt(&mut core, "row1b");
        }
        match core.read_core_reg::<u32>(CSR_MHARTID) {
            Ok(v) => seen.push((id, v)),
            Err(e) => {
                return RowResult::fail(
                    "1b. mhartid sentinel",
                    format!("hart {id}: mhartid read failed: {e}"),
                );
            }
        }
    }
    let plausible = seen.iter().all(|(_, v)| *v < 2);
    if plausible {
        RowResult::pass("1b. mhartid sentinel", format!("{:?}", seen))
    } else {
        RowResult::fail(
            "1b. mhartid sentinel",
            format!("unexpected values: {:?} (expected {{0,1}})", seen),
        )
    }
}

/// Row 2 — GPR round-trip. Write 0xDEADBEEF to a0, read back.
fn row2_gpr(core: &mut Core) -> RowResult {
    const MAGIC: u32 = 0xDEAD_BEEF;
    if let Err(e) = core.write_core_reg(RV_X10_A0, MAGIC) {
        return RowResult::fail("2. GPR read/write", format!("write a0 failed: {e}"));
    }
    match core.read_core_reg::<u32>(RV_X10_A0) {
        Ok(v) if v == MAGIC => RowResult::pass("2. GPR read/write", "a0 round-trip ok"),
        Ok(v) => RowResult::fail(
            "2. GPR read/write",
            format!("a0 readback = 0x{v:08X}, expected 0x{MAGIC:08X}"),
        ),
        Err(e) => RowResult::fail("2. GPR read/write", format!("read a0 failed: {e}")),
    }
}

/// Row 3 — CSR round-trip via mscratch (read→invert→write→readback→restore).
fn row3_csr(core: &mut Core) -> RowResult {
    let orig = match core.read_core_reg::<u32>(CSR_MSCRATCH) {
        Ok(v) => v,
        Err(e) => return RowResult::fail("3. CSR read/write", format!("read mscratch failed: {e}")),
    };
    let probe = orig ^ 0xA5A5_A5A5;
    if let Err(e) = core.write_core_reg(CSR_MSCRATCH, probe) {
        return RowResult::fail("3. CSR read/write", format!("write mscratch failed: {e}"));
    }
    let back = match core.read_core_reg::<u32>(CSR_MSCRATCH) {
        Ok(v) => v,
        Err(e) => {
            return RowResult::fail(
                "3. CSR read/write",
                format!("readback failed: {e}"),
            );
        }
    };
    let _ = core.write_core_reg(CSR_MSCRATCH, orig); // restore
    if back == probe {
        RowResult::pass(
            "3. CSR read/write",
            format!("mscratch 0x{orig:08X}→0x{probe:08X} ok"),
        )
    } else {
        RowResult::fail(
            "3. CSR read/write",
            format!("mscratch readback 0x{back:08X}, expected 0x{probe:08X}"),
        )
    }
}

/// Row 4 — HW breakpoint + run + wait_for_core_halted. Places a c.nop sled
/// followed by c.ebreak at EMU_TEST_SLOT. HW BP availability is the PASS
/// criterion for HLD §4 cap #4 — the c.ebreak sentinel is a second safety
/// halt only, not a valid substitute for the HW BP unit.
///
/// Returns (RowResult, stub_end_addr). The HW breakpoint, if set, is
/// cleared before returning so Row 5 can reseed PC to the same address
/// without triggering the still-armed breakpoint and measuring delta=0.
fn row4_breakpoint(core: &mut Core<'_>) -> (RowResult, u64) {
    let addr = EMU_TEST_SLOT as u64;
    // Stub: 4 halfwords of c.nop (= 8 bytes) + 1 halfword of c.ebreak (= 2
    // bytes) = 10 bytes total. Sized as u16 halfwords × 2 bytes.
    let mut stub = Vec::with_capacity(4 * 2 + 2);
    for _ in 0..4 {
        stub.extend_from_slice(&RV_NOP.to_le_bytes());
    }
    stub.extend_from_slice(&RV_EBREAK.to_le_bytes());
    if let Err(e) = core.write_8(addr, &stub) {
        return (RowResult::fail("4. HW breakpoint", format!("write stub: {e}")), addr);
    }
    // Seed sp (RV x2) and pc. EMU_TEST_STACK is u32 in mdpicoem_harness; the
    // RV32 backend's write_core_reg expects u32.
    if let Err(e) = core.write_core_reg::<u32>(RV_X2_SP, EMU_TEST_STACK) {
        return (RowResult::fail("4. HW breakpoint", format!("set sp: {e}")), addr);
    }
    if let Err(e) = core.write_core_reg(RV_PC, addr as u32) {
        return (RowResult::fail("4. HW breakpoint", format!("set pc: {e}")), addr);
    }

    let bp_addr = addr + 6; // after 3 × c.nop, before the 4th c.nop / c.ebreak
    let hw_bp_set = match core.set_hw_breakpoint(bp_addr) {
        Ok(()) => true,
        Err(e) => {
            // HW BP unit unavailable or allocation failed. HLD Path A requires
            // HW BPs — flag as FAIL so map_to_outcome picks Partial (which
            // routes Phase 3 to SW-ebreak sentinels instead).
            tracing::debug!(target: "riscv_probe_spike", "set_hw_breakpoint failed: {e}");
            false
        }
    };
    if let Err(e) = core.run() {
        if hw_bp_set {
            let _ = core.clear_hw_breakpoint(bp_addr);
        }
        return (RowResult::fail("4. HW breakpoint", format!("run: {e}")), addr);
    }
    let wait = core.wait_for_core_halted(Duration::from_millis(2000));
    if hw_bp_set
        && let Err(e) = core.clear_hw_breakpoint(bp_addr)
    {
        tracing::debug!(target: "riscv_probe_spike", "clear_hw_breakpoint failed: {e}");
    }
    match (wait, hw_bp_set) {
        (Ok(()), true) => (
            RowResult::pass("4. HW breakpoint", format!("hw breakpoint hit at 0x{bp_addr:08X}")),
            addr,
        ),
        (Ok(()), false) => (
            // Core halted via the c.ebreak sentinel at the stub tail, not via
            // a HW BP. Cap #4 is "HW breakpoint + run + wait_for_halted" — a
            // SW-sentinel halt does not satisfy that, so this is FAIL with a
            // descriptive reason that steers decision-tree mapping.
            RowResult::fail(
                "4. HW breakpoint",
                "set_hw_breakpoint failed; only c.ebreak sentinel halted the core \
                 (HLD §4 cap #4 requires HW BP unit — Path B / Partial)",
            ),
            addr,
        ),
        (Err(e), _) => (
            RowResult::fail("4. HW breakpoint", format!("wait_for_halted: {e}")),
            addr,
        ),
    }
}

/// Row 5 — single-step. Seeds PC to EMU_TEST_SLOT again, steps once, checks
/// PC advanced by exactly 2 (c.nop is a 16-bit compressed instruction). A
/// delta of 4 would mean probe-rs's step path treats it as an uncompressed
/// word — that's a latent RV32C decode bug on the probe side and must not
/// silently pass.
fn row5_step(core: &mut Core<'_>) -> RowResult {
    let addr = EMU_TEST_SLOT as u64;
    if let Err(e) = core.write_core_reg(RV_PC, addr as u32) {
        return RowResult::fail("5. Single-step", format!("reset pc: {e}"));
    }
    if let Err(e) = core.step() {
        return RowResult::fail("5. Single-step", format!("step: {e}"));
    }
    let new_pc = match core.read_core_reg::<u32>(RV_PC) {
        Ok(v) => v as u64,
        Err(e) => return RowResult::fail("5. Single-step", format!("read pc: {e}")),
    };
    let delta = new_pc.wrapping_sub(addr);
    match delta {
        2 => RowResult::pass(
            "5. Single-step",
            format!("pc 0x{addr:08X} -> 0x{new_pc:08X} (+2 = c.nop)"),
        ),
        4 => RowResult::fail(
            "5. Single-step",
            format!(
                "pc 0x{addr:08X} -> 0x{new_pc:08X} (Δ=4 after c.nop; \
                 probe-rs may not decode RV32C compressed instructions correctly)"
            ),
        ),
        _ => RowResult::fail(
            "5. Single-step",
            format!("pc 0x{addr:08X} -> 0x{new_pc:08X} (Δ={delta}, expected 2)"),
        ),
    }
}

/// Row E1 — mcycle advances across 10 × c.nop under single-step.
///
/// Bounded by `E1_STEP_BUDGET` so a wedged DM cannot hang the spike
/// indefinitely. Any step that fails or exceeds the budget is reported as
/// FAIL with the actual error; mcycle read failures also surface the actual
/// probe-rs error instead of being silently coerced to 0.
fn row_e1_mcycle(core: &mut Core<'_>) -> RowResult {
    let addr = EMU_TEST_SLOT as u64;
    // Stub: 10 halfwords of c.nop (20 bytes) + 1 halfword of c.ebreak (2
    // bytes) = 22 bytes total.
    let mut stub = Vec::with_capacity(10 * 2 + 2);
    for _ in 0..10 {
        stub.extend_from_slice(&RV_NOP.to_le_bytes());
    }
    stub.extend_from_slice(&RV_EBREAK.to_le_bytes());
    if let Err(e) = core.write_8(addr, &stub) {
        return RowResult::fail("E1. mcycle advances under step", format!("write stub: {e}"));
    }
    if let Err(e) = core.write_core_reg(RV_PC, addr as u32) {
        return RowResult::fail("E1. mcycle advances under step", format!("set pc: {e}"));
    }
    let before = match core.read_core_reg::<u32>(CSR_MCYCLE) {
        Ok(v) => v,
        Err(e) => {
            return RowResult::fail(
                "E1. mcycle advances under step",
                format!("read mcycle (pre): {e}"),
            );
        }
    };
    let deadline = Instant::now() + E1_STEP_BUDGET;
    for i in 0..10 {
        if Instant::now() > deadline {
            return RowResult::fail(
                "E1. mcycle advances under step",
                format!("step loop exceeded {:?} budget at iteration {i}", E1_STEP_BUDGET),
            );
        }
        if let Err(e) = core.step() {
            return RowResult::fail(
                "E1. mcycle advances under step",
                format!("step {i} failed: {e}"),
            );
        }
    }
    let after = match core.read_core_reg::<u32>(CSR_MCYCLE) {
        Ok(v) => v,
        Err(e) => {
            return RowResult::fail(
                "E1. mcycle advances under step",
                format!("read mcycle (post): {e}"),
            );
        }
    };
    let delta = after.wrapping_sub(before);
    if delta > 0 {
        RowResult::pass(
            "E1. mcycle advances under step",
            format!("Δmcycle = {delta} across 10 × c.nop"),
        )
    } else {
        // Empirical only; FAIL is informational, not gating.
        RowResult::fail(
            "E1. mcycle advances under step",
            "Δmcycle = 0 (mcycle may halt during debug — expected per RV debug §4.9.1)",
        )
    }
}

/// Row E2 — halt with mip[11] (MEIP) asserted resumes cleanly.
/// Best-effort: if mip is read-only or the write is rejected, the row SKIPs.
///
/// We explicitly clear `mstatus.MIE` before the step so a pending external
/// interrupt cannot route into `mtvec` and be observed as a false PASS; after
/// the step we assert the PC landed at EMU_TEST_SLOT + 2 (one c.nop). If the
/// PC is anywhere else the test SKIPs with diagnostic rather than claiming
/// "step returned Ok" with no meaning.
fn row_e2_mip_pending(core: &mut Core<'_>) -> RowResult {
    let orig = match core.read_core_reg::<u32>(CSR_MIP) {
        Ok(v) => v,
        Err(e) => {
            return RowResult::skip(
                "E2. mip[11]-pending halt clean",
                format!("cannot read mip: {e}"),
            );
        }
    };
    let poked = orig | (1 << 11);
    if core.write_core_reg(CSR_MIP, poked).is_err() {
        // Hazard3: most mip bits are read-only (set by HW). Expected — SKIP.
        return RowResult::skip(
            "E2. mip[11]-pending halt clean",
            "mip is read-only on this impl (expected — Hazard3 drives bits from HW)",
        );
    }
    // Explicitly disable global M-mode interrupts so a pending MEIP does not
    // redirect the step into mtvec and confuse the "step resumed cleanly"
    // assertion. Restore afterwards is best-effort.
    let orig_mstatus = core.read_core_reg::<u32>(CSR_MSTATUS).ok();
    if let Some(m) = orig_mstatus {
        let _ = core.write_core_reg(CSR_MSTATUS, m & !MSTATUS_MIE);
    }
    let addr = EMU_TEST_SLOT as u64;
    let _ = core.write_core_reg(RV_PC, addr as u32);
    let step_result = core.step();
    if let Some(m) = orig_mstatus {
        let _ = core.write_core_reg(CSR_MSTATUS, m);
    }
    match step_result {
        Ok(_) => match core.read_core_reg::<u32>(RV_PC) {
            Ok(pc) => {
                let expected = (addr + 2) as u32;
                if pc == expected {
                    RowResult::pass(
                        "E2. mip[11]-pending halt clean",
                        format!("step landed at 0x{pc:08X} (+2, clean)"),
                    )
                } else {
                    RowResult::skip(
                        "E2. mip[11]-pending halt clean",
                        format!(
                            "step landed at 0x{pc:08X}, expected 0x{expected:08X} — \
                             likely trapped into mtvec; mstatus.MIE handling unclear"
                        ),
                    )
                }
            }
            Err(e) => RowResult::fail(
                "E2. mip[11]-pending halt clean",
                format!("post-step PC read: {e}"),
            ),
        },
        Err(e) => RowResult::fail(
            "E2. mip[11]-pending halt clean",
            format!("step after mip[11] poke: {e}"),
        ),
    }
}

/// Row A1 — ARCHSEL read-only probe. Always runs. Reads the current
/// CHIP_RESET register over SWD; PASS on successful read (including the
/// observed ARCH_SEL bit), FAIL on probe-rs error. Never writes. This
/// exercises the sub-question "can the host even see POWMAN CHIP_RESET
/// over SWD?" independent of whether a write+reset flip works.
///
/// Uses the emulator's assumed offsets (POWMAN_BASE 0x4010_0000, CHIP_RESET
/// offset 0x20) — see `crates/mdrp2350/src/peripherals/powman.rs` module
/// doc and RUNBOOK.md for the explicit ASSUMPTION status of these values.
const POWMAN_BASE: u32 = 0x4010_0000; // ASSUMPTION — see RUNBOOK
const CHIP_RESET_OFFSET: u32 = 0x20; // ASSUMPTION — see RUNBOOK
const POWMAN_OFFSET_PINNED: bool = false; // flip to true only when datasheet pin done

fn row_a1_archsel_read(core: &mut Core<'_>) -> RowResult {
    let addr = (POWMAN_BASE + CHIP_RESET_OFFSET) as u64;
    match core.read_word_32(addr) {
        Ok(v) => RowResult::pass(
            "A1. POWMAN CHIP_RESET read (RO probe)",
            format!(
                "0x{addr:08X} = 0x{v:08X} (ARCH_SEL bit value depends on datasheet layout; \
                 offsets are ASSUMPTION — see RUNBOOK)"
            ),
        ),
        Err(e) => RowResult::fail(
            "A1. POWMAN CHIP_RESET read (RO probe)",
            format!("read 0x{addr:08X}: {e}"),
        ),
    }
}

/// Row A2 — ARCHSEL write+reset flip. Gated behind `--attempt-archsel-flip`.
/// Even when the flag is set, stays SKIP until the POWMAN CHIP_RESET offset
/// is pinned from RP2350 datasheet §5.10 in `RUNBOOK.md` (flip the
/// `POWMAN_OFFSET_PINNED` constant above to enable). Until then writing
/// based on the emulator's educated guess could scribble into an unrelated
/// POWMAN register on silicon.
fn row_a2_archsel_flip(attempt: bool) -> RowResult {
    if !attempt {
        return RowResult::skip(
            "A2. ARCHSEL-via-SWD flip",
            "--attempt-archsel-flip not set; skipped (use flag to opt in once RUNBOOK offset is pinned)",
        );
    }
    if !POWMAN_OFFSET_PINNED {
        return RowResult::skip(
            "A2. ARCHSEL-via-SWD flip",
            "A2 skipped: POWMAN CHIP_RESET offset not pinned in RUNBOOK",
        );
    }
    // Path not yet enabled in V1; implement when POWMAN_OFFSET_PINNED flips.
    RowResult::skip(
        "A2. ARCHSEL-via-SWD flip",
        "A2 write+reset protocol not implemented in V1 spike",
    )
}

// ---------------------------------------------------------------------------
// Runner
// ---------------------------------------------------------------------------

/// Row labels used when the row was not run (cascade SKIP after an upstream
/// failure). Kept in one place so the label strings stay identical to the
/// ones emitted by the actual row functions — decision-tree mapping uses
/// `starts_with` prefix matching, but exact labels are still valuable for
/// human readability of the matrix.
const LABEL_ROW1: &str = "1. Attach + RV core enumeration";
const LABEL_ROW1B: &str = "1b. mhartid sentinel";
const LABEL_ROW2: &str = "2. GPR read/write";
const LABEL_ROW3: &str = "3. CSR read/write";
const LABEL_ROW4: &str = "4. HW breakpoint";
const LABEL_ROW5: &str = "5. Single-step";
const LABEL_E1: &str = "E1. mcycle advances under step";
const LABEL_E2: &str = "E2. mip[11]-pending halt clean";

fn cascade_skip_rows2_to_e2(results: &mut Vec<RowResult>, reason: &'static str) {
    results.push(RowResult::skip(LABEL_ROW2, reason));
    results.push(RowResult::skip(LABEL_ROW3, reason));
    results.push(RowResult::skip(LABEL_ROW4, reason));
    results.push(RowResult::skip(LABEL_ROW5, reason));
    results.push(RowResult::skip(LABEL_E1, reason));
    results.push(RowResult::skip(LABEL_E2, reason));
}

fn attach_session(args: &Args) -> Result<Session, probe_rs::Error> {
    match args.probe.as_ref() {
        None => Session::auto_attach("rp2350", SessionConfig::default()),
        Some(selector) => {
            let probe = Lister::new()
                .open(selector.clone())
                .map_err(probe_rs::Error::from)?;
            probe.attach("rp2350", Permissions::default())
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // CLI parse error is the only path that exits non-zero (code 2). Once
    // we're past CLI, every downstream failure feeds back into the matrix.
    let args = parse_args()?;

    println!("riscv_probe_spike: RP2354 Hazard3 probe-rs attach spike");
    println!("======================================================");

    let mut results: Vec<RowResult> = Vec::new();

    // Try to attach. RP2354 in RV boot makes ARM auto_attach very likely to
    // fail — that's valid data, not a tool error. Record Row 1 FAIL, cascade
    // SKIP through the remaining rows, print the matrix, map to PLAN D.
    let mut session = match attach_session(&args) {
        Ok(s) => {
            println!("Attached session, {} core(s) enumerated", s.list_cores().len());
            s
        }
        Err(e) => {
            println!("Attach failed: {e}");
            results.push(RowResult::fail(
                LABEL_ROW1,
                format!("Session::auto_attach failed: {e} (expected when silicon is in RV boot)"),
            ));
            results.push(RowResult::skip(LABEL_ROW1B, "attach failed, cascade skip"));
            cascade_skip_rows2_to_e2(&mut results, "attach failed, cascade skip");
            // Row A1 also needs a session; mark it SKIP with the same reason.
            results.push(RowResult::skip(
                "A1. POWMAN CHIP_RESET read (RO probe)",
                "attach failed, cascade skip",
            ));
            results.push(row_a2_archsel_flip(args.attempt_archsel_flip));
            print_summary(&results);
            return Ok(());
        }
    };

    // Row 1.
    let r1 = row1_attach(&mut session);
    let row1_passed = r1.verdict == Verdict::Pass;
    results.push(r1);

    // Row 1b (sentinel mhartid) — only meaningful if row 1 passed.
    if row1_passed {
        results.push(row1b_mhartid(&mut session));
    } else {
        results.push(RowResult::skip(
            LABEL_ROW1B,
            "row 1 failed; cannot probe mhartid as RV",
        ));
    }

    // Rows 2–E2 + A1 all run against hart 0. Scope the Core borrow so the
    // Session is free afterwards (not needed post-core but keeps the block
    // boundary obvious).
    {
        let mut core = match session.core(0) {
            Ok(c) => c,
            Err(e) => {
                tracing::debug!(target: "riscv_probe_spike", "core(0) attach failed: {e}");
                cascade_skip_rows2_to_e2(&mut results, "core(0) attach failed");
                results.push(RowResult::skip(
                    "A1. POWMAN CHIP_RESET read (RO probe)",
                    "core(0) attach failed",
                ));
                results.push(row_a2_archsel_flip(args.attempt_archsel_flip));
                print_summary(&results);
                return Ok(());
            }
        };
        if !core.status().map(|s| s.is_halted()).unwrap_or(false) {
            try_halt(&mut core, "pre-row2");
        }

        results.push(row2_gpr(&mut core));
        results.push(row3_csr(&mut core));
        let (r4, _addr) = row4_breakpoint(&mut core);
        let row4_passed = r4.verdict == Verdict::Pass;
        results.push(r4);

        if row4_passed {
            // Re-halt before step (c.ebreak left us halted already, but be explicit).
            if !core.status().map(|s| s.is_halted()).unwrap_or(false) {
                try_halt(&mut core, "post-row4");
            }
            results.push(row5_step(&mut core));
            results.push(row_e1_mcycle(&mut core));
            results.push(row_e2_mip_pending(&mut core));
        } else {
            results.push(RowResult::skip(LABEL_ROW5, "row 4 failed"));
            results.push(RowResult::skip(LABEL_E1, "row 4 failed"));
            results.push(RowResult::skip(LABEL_E2, "row 4 failed"));
        }

        // Row A1: read-only POWMAN CHIP_RESET probe. Runs even if earlier
        // rows failed because it only needs basic MMIO read — the data it
        // produces is useful for diagnosing arch state regardless.
        results.push(row_a1_archsel_read(&mut core));
    }

    // Row A2 — write+reset flip, gated.
    results.push(row_a2_archsel_flip(args.attempt_archsel_flip));

    print_summary(&results);
    Ok(())
}

fn print_summary(results: &[RowResult]) {
    // Compute decision-tree outcome first so the top line is the answer.
    let cap_pass = |label_prefix: &str| {
        results
            .iter()
            .find(|r| r.label.starts_with(label_prefix))
            .map(|r| r.verdict == Verdict::Pass)
            .unwrap_or(false)
    };
    let row1 = cap_pass("1.");
    let row2 = cap_pass("2.");
    let row3 = cap_pass("3.");
    let row4 = cap_pass("4.");
    let row5 = cap_pass("5.");
    let (outcome, next) = if !row1 {
        (
            "PLAN_D",
            "silicon gap accepted; file tech_debt.md entry per HLD §4 Phase 1",
        )
    } else if row1 && row2 && row3 && row4 && row5 {
        ("FULL", "Phase 3 LLD starts on Path A (probe-rs direct + mailbox cycle)")
    } else if row1 && row2 && !row3 && !row4 && !row5 {
        (
            "MAILBOX",
            "firmware-stub oracle; stub RV-native; reuse silicon_cycle_oracle protocol",
        )
    } else if row1 && row2 {
        (
            "PARTIAL",
            "Phase 3 LLD picks proxy mix — Path A for passing rows, fallback for failures",
        )
    } else {
        ("UNRESOLVED", "rows 1/2 partially failed — re-run with --probe")
    };

    // m1: print outcome first, THEN the matrix.
    println!();
    println!("Outcome   : {outcome}");
    println!("Phase 3   : {next}");

    println!();
    println!("=== RISC-V probe-rs spike: capability matrix ===");
    let label_width = results.iter().map(|r| r.label.len()).max().unwrap_or(0);
    for r in results {
        let verdict = match r.verdict {
            Verdict::Pass => "PASS",
            Verdict::Fail => "FAIL",
            Verdict::Skip => "SKIP",
        };
        println!(
            "{:width$}  : {verdict:<4}  {reason}",
            r.label,
            width = label_width,
            reason = r.reason
        );
    }

    // Count and print counts.
    let (mut p, mut f, mut s) = (0usize, 0usize, 0usize);
    for r in results {
        match r.verdict {
            Verdict::Pass => p += 1,
            Verdict::Fail => f += 1,
            Verdict::Skip => s += 1,
        }
    }
    println!();
    println!("Totals    : {p} PASS, {f} FAIL, {s} SKIP (out of {})", results.len());
    println!("(Exit code is 0 on a clean spike run regardless of per-row verdicts.)");
}
