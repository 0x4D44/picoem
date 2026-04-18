// QEMU differential test runner — RP2350 Hazard3 (RISC-V) oracle.
//
// Stage 5 of the Phase 2 plan; see
// `wrk_docs/2026.04.17 - LLD - QEMU Diff RISC-V V1.md` §10 (test loop),
// §3 (CSR-diff proxy), §4.1 (proxy self-check), §5 (WARL disposition).
//
// Orchestrates: spawn `qemu-system-riscv32 -machine none` holding an
// `ebreak` loader image at `0x2000_0000` (LLD §2), connect GDB on
// localhost:3335, splice the CSR-read prelude / test / CSR-capture
// epilogue / `ebreak` terminator at `0x2000_0100`, run both sides, diff
// GPRs + PC + scratchpad CSR snapshots.
//
// Usage:
//   test_qemu_diff_riscv32                          Targeted edge-case suite (default)
//   test_qemu_diff_riscv32 --fuzz N                 N fuzz tests (distributed by LLD §6 weights)
//   test_qemu_diff_riscv32 --fuzz N --seed S        Deterministic fuzz
//   test_qemu_diff_riscv32 --fuzz N --class <name>  Filter fuzz to one RiscvClass
//   test_qemu_diff_riscv32 --proxy-self-check-only  Run §4.1 proxy self-check and exit
//
// Class names for --class:
//   rv32i-alu rv32i-mem rv32i-misaligned rv32i-branch rv32i-upper
//   rv32m rv32a rv32c zicsr zifencei csr-sideeffect

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use rand::rngs::StdRng;
use rand::SeedableRng;

use mdpicoem_harness::gdb_client::{
    GdbClient, QemuProcess, QemuProfile, REG_RV_PC, REG_RV_X0,
};
use mdpicoem_harness::riscv_gen::{
    self, encode_csr, encode_i_type, encode_s_type, OPC_OP_IMM, OPC_SYSTEM, RiscvClass,
    RiscvTestCase, SCRATCH_BASE,
};

use mdrp2350::{Arch, Config, EmulatorBuilder};

// ============================================================================
// Address-map constants (LLD §3, §4.1, §10)
// ============================================================================

// Oracle address base. QEMU virt rv32 maps VIRT_FLASH at 0x2000_0000
// (read-only for CPU stores) and VIRT_DRAM at 0x8000_0000. The original
// LLD assumed 0x2000_0000 matched RP2350 SRAM on both sides, but CPU
// `sw` to virt flash silently no-ops — any test that spills to
// scratchpad fails trivially. All oracle addresses now live in virt
// DRAM; the mdrp2350 bus aliases `0x8xxx_xxxx` → SRAM via
// `canon_oracle_addr`, so both sides execute at the same absolute PC.

/// Boot image load address — virt DRAM base.
const LOADER_ADDR: u32 = 0x8000_0000;
/// Test instruction slot — CSR-read prelude starts here (LLD §3).
const TEST_SLOT: u32 = 0x8000_0100;
/// Trap-handler stub slot (Zicsr carve-out, LLD §4 mitigation (1)).
const TRAP_STUB: u32 = 0x8000_0200;
/// mcause slot where the trap handler spills its value (LLD §10).
const MCAUSE_SLOT: u32 = 0x8000_01F0;
/// CSR capture scratchpad — proxy's gp points here. 56 bytes total:
/// `+0..+0x1C` pre-snapshot, `+0x1C..+0x38` post-snapshot. Distinct from
/// `riscv_gen::SCRATCH_BASE` (the test-visible data region at
/// `0x8000_0400`), so test mem loads don't read captured CSR values.
const CSR_SCRATCH: u32 = 0x8000_0300;
/// Self-check stub slot (LLD §4.1). Moved past the test data region
/// (0x8000_0400..0x8000_04FF, 256 B reserved for tests).
const SELFCHECK_STUB: u32 = 0x8000_0500;
/// Self-check scratchpad base (pattern-A ×7 + pattern-B ×7 = 56 B, LLD §4.1).
const SELFCHECK_SCRATCH: u32 = 0x8000_0600;
/// Post-capture stub slot — runs AFTER the test stream halts, reads the
/// 7 diff-set CSRs, and spills them to the post-snapshot region of
/// `CSR_SCRATCH`. Separate from the test stream so control-flow tests
/// (branch/jump/c.j* etc.) can't redirect PC past the capture.
const POST_CAPTURE_STUB: u32 = 0x8000_0680;

// Proxy / handler registers — chosen to avoid collisions with generator
// cases (cases typically use x5..x15 plus x1 for branches). We keep the
// proxy's use of gp=x3 and t0=x5 visible (documented in LLD §3).
const REG_X0: u8 = 0;
const REG_GP: u8 = 3;
const REG_T0: u8 = 5;

// ============================================================================
// CSR diff set — must match LLD §3
// ============================================================================

/// Diff-set CSR addresses (order matters: snapshot layout follows this).
const CSR_DIFF_ADDRS: [u16; 7] = [
    0x300, // mstatus
    0x304, // mie
    0x305, // mtvec
    0x340, // mscratch
    0x341, // mepc
    0x342, // mcause
    0x344, // mip
];

/// Human-readable CSR names (for error reporting).
const CSR_DIFF_NAMES: [&str; 7] = ["mstatus", "mie", "mtvec", "mscratch", "mepc", "mcause", "mip"];

// CSR funct3 codes (per RISC-V spec §9):
const CSR_F3_RW: u32 = 0b001;
const CSR_F3_RS: u32 = 0b010;
// (CSRRC = 0b011; CSRRWI/SI/CI = 0b101/110/111 — unused here.)

/// `ebreak` instruction encoded as a 32-bit word (little-endian: 73 00 10 00).
const EBREAK_WORD: u32 = 0x0010_0073;

// ============================================================================
// Shutdown flag (Ctrl-C)
// ============================================================================

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn shutdown_requested() -> bool {
    SHUTDOWN.load(Ordering::SeqCst)
}

// ============================================================================
// main
// ============================================================================

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    if let Err(e) = ctrlc::set_handler(|| SHUTDOWN.store(true, Ordering::SeqCst)) {
        eprintln!("warning: failed to install Ctrl-C handler: {e}");
    }

    match run() {
        Ok(code) => code,
        Err(e) => {
            eprintln!("fatal: {e}");
            ExitCode::from(2)
        }
    }
}

// ============================================================================
// Argument parsing
// ============================================================================

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
    class_filter: Option<RiscvClass>,
    self_check_only: bool,
}

fn parse_class(s: &str) -> Result<RiscvClass, String> {
    match s {
        "rv32i-alu" => Ok(RiscvClass::Rv32iAlu),
        "rv32i-mem" => Ok(RiscvClass::Rv32iMem),
        "rv32i-misaligned" => Ok(RiscvClass::Rv32iMisalignedMem),
        "rv32i-branch" => Ok(RiscvClass::Rv32iBranch),
        "rv32i-upper" => Ok(RiscvClass::Rv32iUpper),
        "rv32m" => Ok(RiscvClass::Rv32m),
        "rv32a" => Ok(RiscvClass::Rv32aReservable),
        "rv32c" => Ok(RiscvClass::Rv32c),
        "zicsr" => Ok(RiscvClass::Zicsr),
        "zifencei" => Ok(RiscvClass::Zifencei),
        "csr-sideeffect" => Ok(RiscvClass::CsrSideEffect),
        other => Err(format!(
            "invalid --class value '{other}' (expected one of: \
             rv32i-alu | rv32i-mem | rv32i-misaligned | rv32i-branch | \
             rv32i-upper | rv32m | rv32a | rv32c | zicsr | zifencei | csr-sideeffect)"
        )),
    }
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut class_filter = None;
    let mut self_check_only = false;
    let mut i = 0;

    while i < args.len() {
        match args[i].as_str() {
            "--fuzz" => {
                i += 1;
                if i >= args.len() {
                    return Err("--fuzz requires a count argument".into());
                }
                fuzz_count = Some(
                    args[i]
                        .parse::<usize>()
                        .map_err(|e| format!("invalid fuzz count '{}': {e}", args[i]))?,
                );
            }
            "--seed" => {
                i += 1;
                if i >= args.len() {
                    return Err("--seed requires a value argument".into());
                }
                seed = Some(
                    args[i]
                        .parse::<u64>()
                        .map_err(|e| format!("invalid seed '{}': {e}", args[i]))?,
                );
            }
            "--class" => {
                i += 1;
                if i >= args.len() {
                    return Err("--class requires a class name".into());
                }
                class_filter = Some(parse_class(&args[i])?);
            }
            "--proxy-self-check-only" => {
                self_check_only = true;
            }
            "-h" | "--help" => {
                print_help();
                std::process::exit(0);
            }
            other => {
                return Err(format!(
                    "unknown argument '{other}' (use --help for usage)"
                ));
            }
        }
        i += 1;
    }

    if seed.is_some() && fuzz_count.is_none() {
        return Err("--seed requires --fuzz".into());
    }
    if class_filter.is_some() && fuzz_count.is_none() {
        return Err("--class requires --fuzz".into());
    }

    Ok(Args { fuzz_count, seed, class_filter, self_check_only })
}

fn print_help() {
    println!(
        "Usage:\n  \
         test_qemu_diff_riscv32                               Run targeted edge-case suite (default)\n  \
         test_qemu_diff_riscv32 --fuzz N                      Run N fuzz tests (distributed per class weights)\n  \
         test_qemu_diff_riscv32 --fuzz N --seed S             Deterministic fuzz\n  \
         test_qemu_diff_riscv32 --fuzz N --class <name>       Filter fuzz to one class\n  \
         test_qemu_diff_riscv32 --proxy-self-check-only       Run §4.1 proxy self-check and exit\n\n\
         Class names:\n  \
         rv32i-alu | rv32i-mem | rv32i-misaligned | rv32i-branch | rv32i-upper |\n  \
         rv32m | rv32a | rv32c | zicsr | zifencei | csr-sideeffect"
    );
}

// ============================================================================
// Main runner
// ============================================================================

fn run() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let args = parse_args()?;

    // 1. Write placeholder boot-stub (single ebreak) to a temp file.
    //    `_stub` is held across the whole run; its Drop unlinks the file
    //    even on panic / early-return paths.
    let _stub = write_boot_stub()?;

    // 2. Spawn QEMU with that loader image.
    let _qemu = QemuProcess::spawn_with_image(QemuProfile::RISCV32_RP2350, _stub.path())?;

    // 3. Connect GDB.
    let mut gdb = GdbClient::connect(
        &QemuProfile::RISCV32_RP2350.gdb_addr(),
        Duration::from_secs(5),
    )?;
    gdb.handshake()?;

    // 4. Sanity: PC should be at the loader address.
    gdb.verify_rv_loader_pc(LOADER_ADDR)?;

    // 5. Construct our emulator.
    //    `step_quantum(1)` makes each `Emulator::step` call retire exactly
    //    one Hazard3 instruction on core 0 (§4.3 flat 1-cycle/op), matching
    //    QEMU's single-step granularity. Core 1 is halted at startup so the
    //    quantum scheduler only dispatches core 0 — dual-core semantics are
    //    out of scope for this oracle.
    let mut emu = EmulatorBuilder::new(Config::default())
        .arch(Arch::RiscV)
        .step_quantum(1)
        .build();
    emu.core_riscv_mut(1).set_halted(true);

    // 6. §4.1 proxy self-check — runs on both sides before any test.
    run_proxy_self_check(&mut gdb, &mut emu)?;

    if args.self_check_only {
        println!("proxy self-check: OK");
        return Ok(ExitCode::SUCCESS);
    }

    // 7. Install the trap-handler stub (used by Zicsr class) on both sides.
    install_trap_handler(&mut gdb, &mut emu)?;

    // 7a. Seed mtvec = TRAP_STUB globally on both sides. Any trap from a
    //     test case (misaligned mem, illegal instruction, etc.) now
    //     deterministically lands in TRAP_STUB → writes mcause to
    //     MCAUSE_SLOT → `ebreak`, instead of jumping to mtvec=0 and
    //     hanging on unmapped memory.
    seed_global_mtvec(&mut gdb, &mut emu)?;

    // 8. Dispatch.
    match args.fuzz_count {
        None => run_targeted(&mut gdb, &mut emu),
        Some(count) => {
            let seed = args.seed.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64
            });
            run_fuzz(&mut gdb, &mut emu, count, seed, args.class_filter)
        }
    }
}

// ============================================================================
// Boot-stub management
// ============================================================================

/// RAII wrapper around the on-disk boot-stub: unlinks the backing file when
/// dropped, so a panic-on-error path doesn't leak scratch files in the
/// system temp dir. The happy-path unlink is implicit (drop at function
/// scope exit).
struct BootStub {
    path: PathBuf,
}

impl BootStub {
    fn path(&self) -> &std::path::Path {
        &self.path
    }
}

impl Drop for BootStub {
    fn drop(&mut self) {
        // Best-effort cleanup; ignore errors (file may already be gone,
        // drive may be read-only under test, etc.).
        let _ = std::fs::remove_file(&self.path);
    }
}

/// Write the 4-byte `ebreak` boot-stub to a deterministic temp file path and
/// return a [`BootStub`] RAII handle. The file is held open until the
/// handle drops — QEMU memory-maps it via `-device loader,file=...`, so we
/// keep it on disk for the lifetime of the harness run.
fn write_boot_stub() -> Result<BootStub, Box<dyn std::error::Error>> {
    let mut path = std::env::temp_dir();
    path.push(format!("mdpicoem-qemu-diff-riscv32-{}.bin", std::process::id()));
    std::fs::write(&path, EBREAK_WORD.to_le_bytes())?;
    Ok(BootStub { path })
}

// ============================================================================
// §4.1 Proxy self-check
// ============================================================================

/// Build the 15-instruction self-check stub: 7× (csrrs/sw) + 7× (csrrw/sw) + ebreak.
///
/// Pattern A uses `csrrs t0, csr, x0` (the proxy's canonical read). Pattern B
/// uses `csrrw t0, csr, x0` — per RV-priv §9.1.3 the `rs1=x0` form is a
/// read-with-no-op-write. Both patterns must yield identical values on each
/// side.
fn build_selfcheck_stub() -> Vec<u32> {
    let mut out = Vec::with_capacity(29);
    // Pattern A — csrrs t0, csr, x0; sw t0, off(gp).
    for (i, &csr) in CSR_DIFF_ADDRS.iter().enumerate() {
        out.push(encode_csr(csr, REG_X0, CSR_F3_RS, REG_T0));
        out.push(encode_s_type(
            (i as i32) * 4,
            REG_T0,
            REG_GP,
            0b010, // SW
            riscv_gen::OPC_STORE,
        ));
    }
    // Pattern B — csrrw t0, csr, x0; sw t0, off(gp).
    for (i, &csr) in CSR_DIFF_ADDRS.iter().enumerate() {
        out.push(encode_csr(csr, REG_X0, CSR_F3_RW, REG_T0));
        out.push(encode_s_type(
            (i as i32 + 7) * 4,
            REG_T0,
            REG_GP,
            0b010,
            riscv_gen::OPC_STORE,
        ));
    }
    out.push(EBREAK_WORD);
    out
}

/// Execute the §4.1 self-check on both sides; abort on any A vs B mismatch.
fn run_proxy_self_check(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
) -> Result<(), Box<dyn std::error::Error>> {
    let stub = build_selfcheck_stub();
    let stub_bytes = words_to_le_bytes(&stub);

    // QEMU: write stub + zero scratchpad, set PC/gp, run to ebreak.
    gdb.write_mem(SELFCHECK_STUB, &stub_bytes)?;
    gdb.write_mem(SELFCHECK_SCRATCH, &[0u8; 56])?;
    zero_gprs_qemu(gdb)?;
    gdb.write_reg(REG_GP, SELFCHECK_SCRATCH)?;
    gdb.write_reg(REG_RV_PC as u8, SELFCHECK_STUB)?;
    // HW breakpoint at the terminator ebreak (after 28 instrs × 4 bytes).
    let term = SELFCHECK_STUB + 28 * 4;
    gdb.set_hw_breakpoint(term, 2)?;
    gdb.continue_exec()?;
    gdb.remove_hw_breakpoint(term, 2)?;
    let qemu_snapshot = gdb.read_mem(SELFCHECK_SCRATCH, 56)?;

    // Emulator: load stub + zero scratchpad, set PC/gp, step to ebreak.
    emu.load_image(SELFCHECK_STUB, &stub_bytes);
    for i in 0..56u32 {
        emu.bus.memory.sram_write8((SELFCHECK_SCRATCH & 0x00FF_FFFF) + i, 0);
    }
    {
        let h = emu.core_riscv_mut(0);
        for r in 1u8..32 {
            h.set_gpr(r, 0);
        }
        h.set_gpr(REG_GP, SELFCHECK_SCRATCH);
        h.set_pc(SELFCHECK_STUB);
    }
    step_emu_until_pc(emu, term, 64)?;
    let emu_snapshot = read_emu_scratch(emu, SELFCHECK_SCRATCH, 56);

    // Compare pattern A vs pattern B on each side independently.
    check_patterns("QEMU", &qemu_snapshot)?;
    check_patterns("emu", &emu_snapshot)?;
    Ok(())
}

fn check_patterns(side: &str, snap: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
    if snap.len() != 56 {
        return Err(format!("{side}: self-check snapshot length {}, expected 56", snap.len()).into());
    }
    for i in 0..7 {
        let a = u32::from_le_bytes([snap[i * 4], snap[i * 4 + 1], snap[i * 4 + 2], snap[i * 4 + 3]]);
        let b_off = (i + 7) * 4;
        let b = u32::from_le_bytes([
            snap[b_off],
            snap[b_off + 1],
            snap[b_off + 2],
            snap[b_off + 3],
        ]);
        if a != b {
            return Err(format!(
                "proxy self-check failed on CSR {:#05x} ({}) via pattern {}: A={a:#010x} B={b:#010x} ({side})",
                CSR_DIFF_ADDRS[i],
                CSR_DIFF_NAMES[i],
                if a == 0 { "A" } else { "B" },
            )
            .into());
        }
    }
    Ok(())
}

// ============================================================================
// Trap-handler stub (Zicsr carve-out)
// ============================================================================

/// Build a minimal 3-instruction trap-handler stub:
///   csrrs t0, mcause, x0   ; read cause
///   sw t0, 0(x0)           ; (overridden below: actual imm = MCAUSE_SLOT low 12 bits)
///   ebreak
///
/// We cannot use a 12-bit signed immediate to reach `MCAUSE_SLOT` from x0
/// (its low 12 bits are 0x1F0 = 496, within range), so the sw uses x0 as
/// the base register and a positive 12-bit offset.
fn build_trap_handler_stub() -> Vec<u32> {
    let offset = MCAUSE_SLOT as i32; // 0x2000_01F0 — too large to fit imm12.
    // Instead of a direct-from-x0 store (out of range), load the address
    // via lui + addi then store. 4 instrs total is still tiny.
    //   lui   t0, %hi(MCAUSE_SLOT)     ; t0 = 0x2000_0000
    //   addi  t0, t0, %lo(MCAUSE_SLOT) ; t0 = 0x2000_01F0
    //   csrrs t1, mcause, x0
    //   sw    t1, 0(t0)
    //   ebreak
    let hi = MCAUSE_SLOT & 0xFFFF_F000;
    let lo = offset & 0xFFF; // sign-extended by addi; for 0x1F0 this is positive.

    let lui = riscv_gen::encode_u_type(hi, REG_T0, riscv_gen::OPC_LUI);
    let addi = encode_i_type(lo, REG_T0, 0b000, REG_T0, OPC_OP_IMM);
    let csrrs_mcause = encode_csr(0x342, REG_X0, CSR_F3_RS, 6); // rd=x6 (t1)
    let sw_mcause = encode_s_type(0, 6, REG_T0, 0b010, riscv_gen::OPC_STORE);
    vec![lui, addi, csrrs_mcause, sw_mcause, EBREAK_WORD]
}

/// Build + run the post-snapshot capture stub on both sides. Reads the
/// 7 diff-set CSRs and spills them to `CSR_SCRATCH + 0x1C..+0x38`.
///
/// Runs after the test halts so control-flow tests (branches/jumps)
/// can't redirect PC past the capture — the test's terminator ebreak
/// stops execution, then we re-seed PC/GPRs and run the stub fresh.
///
/// Clobbers gp (x3) and t6 (x31) on both sides. The caller already read
/// post-state GPRs before invoking this function.
fn capture_post_snapshot(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
) -> Result<(), Box<dyn std::error::Error>> {
    const SCRATCH_REG: u8 = 31; // t6
    // Build once per call — it's tiny and the full per-test cost is
    // dominated by the GDB round-trips anyway.
    let mut stream: Vec<u32> = Vec::with_capacity(15);
    for (i, &csr) in CSR_DIFF_ADDRS.iter().enumerate() {
        stream.push(encode_csr(csr, REG_X0, CSR_F3_RS, SCRATCH_REG));
        stream.push(encode_s_type(
            (i as i32 + 7) * 4, // post-snapshot offset
            SCRATCH_REG,
            REG_GP,
            0b010,
            riscv_gen::OPC_STORE,
        ));
    }
    stream.push(EBREAK_WORD);
    let bytes = words_to_le_bytes(&stream);
    let term = POST_CAPTURE_STUB + ((stream.len() - 1) as u32) * 4;

    // QEMU side.
    gdb.write_mem(POST_CAPTURE_STUB, &bytes)?;
    gdb.write_reg(REG_GP, CSR_SCRATCH)?;
    gdb.write_reg(REG_RV_PC as u8, POST_CAPTURE_STUB)?;
    gdb.set_hw_breakpoint(term, 2)?;
    gdb.continue_exec()?;
    let _ = gdb.remove_hw_breakpoint(term, 2);

    // Emulator side.
    emu.load_image(POST_CAPTURE_STUB, &bytes);
    {
        let h = emu.core_riscv_mut(0);
        h.set_gpr(REG_GP, CSR_SCRATCH);
        h.set_pc(POST_CAPTURE_STUB);
    }
    step_emu_until_pc(emu, term, 32)?;
    Ok(())
}

/// Install the Zicsr trap-handler stub at [`TRAP_STUB`] on both sides. The
/// stub only fires when a test in the Zicsr class installs `mtvec = TRAP_STUB`.
fn install_trap_handler(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
) -> Result<(), Box<dyn std::error::Error>> {
    let stub = build_trap_handler_stub();
    let bytes = words_to_le_bytes(&stub);
    gdb.write_mem(TRAP_STUB, &bytes)?;
    emu.load_image(TRAP_STUB, &bytes);
    Ok(())
}

/// One-shot startup routine: splice the 3-instruction mtvec-seed prelude
/// at a dedicated init slot, run to `ebreak`. Leaves `mtvec = TRAP_STUB`
/// on both sides. Non-Zicsr tests don't touch mtvec, so this one seed
/// persists for the entire run. Zicsr tests re-seed mtvec in their own
/// 3-instruction prelude (see `build_mtvec_seed_prelude`).
///
/// Readback asserts the seed actually landed on the QEMU side — a
/// regression safeguard. The combined stub also stashes the mtvec value
/// into `VERIFY_SCRATCH` so the verification is in-band (one vCont;c
/// pass, no stale-GDB-state hazards).
fn seed_global_mtvec(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
) -> Result<(), Box<dyn std::error::Error>> {
    const INIT_SLOT: u32 = 0x8000_0700;
    const VERIFY_SCRATCH: u32 = 0x8000_0740;

    // Combined seed + self-verify stub:
    //   lui   t0, %hi(TRAP_STUB); addi t0, t0, %lo(TRAP_STUB)
    //   csrrw x0, mtvec, t0        ; write mtvec
    //   csrrs t1, mtvec, x0        ; read mtvec back into t1
    //   lui   t2, %hi(VERIFY_SCRATCH); addi t2, t2, %lo(VERIFY_SCRATCH)
    //   sw    t1, 0(t2)            ; spill t1 to memory
    //   ebreak
    let seed = build_mtvec_seed_prelude();
    let hi_vs = VERIFY_SCRATCH & 0xFFFF_F000;
    let lo_vs = (VERIFY_SCRATCH as i32) & 0xFFF;
    let mut stream: Vec<u32> = seed.to_vec();
    stream.push(encode_csr(0x305, REG_X0, CSR_F3_RS, 6));
    stream.push(riscv_gen::encode_u_type(hi_vs, 7, riscv_gen::OPC_LUI));
    stream.push(encode_i_type(lo_vs, 7, 0b000, 7, OPC_OP_IMM));
    stream.push(encode_s_type(0, 6, 7, 0b010, riscv_gen::OPC_STORE));
    stream.push(EBREAK_WORD);
    let bytes = words_to_le_bytes(&stream);
    let term = INIT_SLOT + ((stream.len() as u32) - 1) * 4;

    // QEMU side.
    gdb.write_mem(INIT_SLOT, &bytes)?;
    gdb.write_mem(VERIFY_SCRATCH, &[0u8; 4])?;
    for r in 1u8..=31 {
        gdb.write_reg(r, 0)?;
    }
    gdb.write_reg(REG_RV_PC as u8, INIT_SLOT)?;
    gdb.set_hw_breakpoint(term, 2)?;
    gdb.continue_exec()?;
    let _ = gdb.remove_hw_breakpoint(term, 2);

    // Emulator side.
    emu.load_image(INIT_SLOT, &bytes);
    {
        let h = emu.core_riscv_mut(0);
        for r in 1u8..=31 {
            h.set_gpr(r, 0);
        }
        h.set_pc(INIT_SLOT);
    }
    step_emu_until_pc(emu, term, 32)?;

    // Verify QEMU side actually landed mtvec = TRAP_STUB. The emulator
    // side is trivially correct (we just stepped through the same
    // instructions in our own core).
    let qemu_mtvec_bytes = gdb.read_mem(VERIFY_SCRATCH, 4)?;
    let qemu_mtvec = u32::from_le_bytes([
        qemu_mtvec_bytes[0],
        qemu_mtvec_bytes[1],
        qemu_mtvec_bytes[2],
        qemu_mtvec_bytes[3],
    ]);
    if qemu_mtvec != TRAP_STUB {
        return Err(format!(
            "global mtvec seed failed on QEMU: mtvec={qemu_mtvec:#010x}, expected {TRAP_STUB:#010x}"
        ).into());
    }
    Ok(())
}

// ============================================================================
// Test loop — edge-case suite
// ============================================================================

fn run_targeted(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let tests = riscv_gen::generate_edge_cases();
    let total = tests.len();
    let mut pass = 0usize;
    let mut fail = 0usize;

    for tc in &tests {
        if shutdown_requested() {
            eprintln!("interrupted (Ctrl-C); exiting cleanly");
            return Ok(ExitCode::from(130));
        }
        match run_one_test(gdb, emu, tc, /*fuzz_mode=*/ false) {
            Ok(cycles) => {
                pass += 1;
                println!("OK {} ({} cycles)", tc.name, cycles);
            }
            Err(e) => {
                fail += 1;
                eprintln!("[FAIL] {}: {}", tc.name, e);
                eprintln!(
                    "  words: {}",
                    tc.words
                        .iter()
                        .map(|w| format!("{:#010x}", w))
                        .collect::<Vec<_>>()
                        .join(" ")
                );
            }
        }
    }

    println!("PASS {pass}/{total}");
    if fail > 0 {
        Ok(ExitCode::from(1))
    } else {
        Ok(ExitCode::SUCCESS)
    }
}

// ============================================================================
// Test loop — fuzz
// ============================================================================

fn run_fuzz(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
    count: usize,
    seed: u64,
    class_filter: Option<RiscvClass>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let class_str = class_filter
        .map(|c| class_to_str(c).to_string())
        .unwrap_or_else(|| "all".to_string());
    println!("Fuzz mode: {count} tests, seed={seed}, classes={class_str}");

    let mut rng = StdRng::seed_from_u64(seed);
    let raw = riscv_gen::generate_fuzz(&mut rng, count);
    let tests: Vec<RiscvTestCase> = match class_filter {
        Some(c) => raw.into_iter().filter(|tc| tc.class == c).collect(),
        None => raw,
    };
    let total = tests.len();
    println!("Generated {total} tests");

    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut per_class_fail = std::collections::BTreeMap::<&'static str, usize>::new();

    for (done, tc) in tests.iter().enumerate() {
        if done % 1000 == 0 && done > 0 {
            eprintln!("[{done}/{total}] {fail} failures...");
            if shutdown_requested() {
                eprintln!("interrupted (Ctrl-C); exiting cleanly");
                return Ok(ExitCode::from(130));
            }
        }
        if shutdown_requested() {
            eprintln!("interrupted (Ctrl-C); exiting cleanly");
            return Ok(ExitCode::from(130));
        }
        match run_one_test(gdb, emu, tc, /*fuzz_mode=*/ true) {
            Ok(_) => pass += 1,
            Err(e) => {
                fail += 1;
                *per_class_fail.entry(class_to_str(tc.class)).or_insert(0) += 1;
                eprintln!(
                    "[FAIL] {} (class={}, seed={seed}): {}\n  words: {}",
                    tc.name,
                    class_to_str(tc.class),
                    e,
                    tc.words
                        .iter()
                        .map(|w| format!("{:#010x}", w))
                        .collect::<Vec<_>>()
                        .join(" ")
                );
            }
        }
    }

    println!();
    println!("=== Fuzz summary ===");
    println!("Seed:    {seed}");
    println!("Classes: {class_str}");
    println!("Total:   {total}");
    println!("Passed:  {pass}");
    println!("Failed:  {fail}");
    if !per_class_fail.is_empty() {
        println!("Per-class failures:");
        for (k, v) in &per_class_fail {
            println!("  {k}: {v}");
        }
    }

    if fail > 0 {
        println!(
            "\nReproduce: test_qemu_diff_riscv32 --fuzz {count} --seed {seed} --class {class_str}"
        );
        Ok(ExitCode::from(1))
    } else {
        Ok(ExitCode::SUCCESS)
    }
}

fn class_to_str(c: RiscvClass) -> &'static str {
    match c {
        RiscvClass::Rv32iAlu => "rv32i-alu",
        RiscvClass::Rv32iMem => "rv32i-mem",
        RiscvClass::Rv32iMisalignedMem => "rv32i-misaligned",
        RiscvClass::Rv32iBranch => "rv32i-branch",
        RiscvClass::Rv32iUpper => "rv32i-upper",
        RiscvClass::Rv32m => "rv32m",
        RiscvClass::Rv32aReservable => "rv32a",
        RiscvClass::Rv32c => "rv32c",
        RiscvClass::Zicsr => "zicsr",
        RiscvClass::Zifencei => "zifencei",
        RiscvClass::CsrSideEffect => "csr-sideeffect",
    }
}

// ============================================================================
// Single-test execution
// ============================================================================

/// Layout of a non-Zicsr test slot:
///   [TEST_SLOT+0..+24]   6 instrs: CSR reset (csrrw x0, csr, x0 × 6)
///   [TEST_SLOT+24..+80]  14 instrs: prelude (CSR-read + sw) × 7
///   [TEST_SLOT+80..]     N instrs: the test case's `words`
///   [after test]         ebreak terminator (1 word)
///
/// Pre-snapshot lands at CSR_SCRATCH+0x00..+0x1C (28 bytes) during the
/// prelude. The POST-snapshot is captured by a separate stub at
/// `POST_CAPTURE_STUB`, invoked after the test halts — this way
/// branch/jump tests can't redirect PC past the post-capture path.
///
/// Zicsr class bypasses the CSR-read proxy entirely (per LLD §4
/// mitigation 1). Its layout is:
///   [TEST_SLOT+0..+12]   3 instrs: mtvec-seed prelude (lui/addi/csrw)
///   [TEST_SLOT+12..]     N instrs: the test case's `words`
///   [after test]         ebreak terminator (1 word)
///
/// The 3-word prelude writes `mtvec = TRAP_STUB` on both sides so a
/// trap-expected test case lands in a known handler slot — without this
/// seed, mtvec carries over from the previous test (leaked state) and
/// `tc.expect_trap` comparisons compare meaningless values.
fn run_one_test(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
    tc: &RiscvTestCase,
    fuzz_mode: bool,
) -> Result<u64, String> {
    let uses_proxy = tc.class != RiscvClass::Zicsr;

    // Build the spliced instruction stream + breakpoint target.
    let (stream, term_addr, test_start) = build_test_stream(tc, uses_proxy);
    let bytes = words_to_le_bytes(&stream);

    // Fresh SRAM windows. Zero the code region (up to TRAP_STUB — the
    // trap handler is installed once at startup and must survive), the
    // mcause spill slot (above TRAP_STUB), the CSR capture scratchpad,
    // and the test data scratchpad. Leaves the trap handler code at
    // TRAP_STUB..TRAP_STUB+0x14 untouched.
    zero_sram_region(gdb, emu, TEST_SLOT, TRAP_STUB - TEST_SLOT)?;
    zero_sram_region(gdb, emu, MCAUSE_SLOT, 0x10)?;
    zero_sram_region(gdb, emu, CSR_SCRATCH, 0x40)?;
    zero_sram_region(gdb, emu, SCRATCH_BASE, 0x100)?;

    // Write the code on both sides.
    gdb.write_mem(TEST_SLOT, &bytes).map_err(|e| format!("QEMU write_mem: {e}"))?;
    emu.load_image(TEST_SLOT, &bytes);

    // Reset the emulator's volatile diff-set CSRs (mcause, mstatus, mie,
    // mip, mscratch, mepc — but NOT mtvec, which stays seeded to
    // TRAP_STUB from the startup routine). Without this reset, `mcause`
    // set by a prior test's trap leaks into the current test's
    // pre-snapshot and immediately fails the CSR diff. QEMU-side CSRs
    // can't be poked over GDB (no flat-index CSR write on QEMU 10.2);
    // the harness compensates by skipping the pre-snapshot diff entirely
    // and only comparing the post-snapshot (see `diff_csr_snapshots`).
    emu.core_riscv_mut(0).reset_diff_csrs();

    // Seed pre-state on both sides.
    apply_reg_pre_qemu(gdb, tc)?;
    apply_reg_pre_emu(emu, tc);

    // PC = start of the stream; gp = scratchpad base (proxy needs it).
    gdb.write_reg(REG_RV_PC as u8, TEST_SLOT)
        .map_err(|e| format!("QEMU write PC: {e}"))?;
    emu.core_riscv_mut(0).set_pc(TEST_SLOT);

    // Drive both sides. Set TWO HW breakpoints:
    //   - `term_addr`: the test's terminator `ebreak` (normal fall-through).
    //   - `TRAP_STUB + 16`: the trap-handler's `ebreak`, fires on any
    //     unexpected-trap path (misaligned mem, illegal insn, etc.).
    // QEMU's rv32 GDB stub does NOT auto-halt on natural `ebreak`
    // instructions — it only stops on breakpoints. Without the second
    // bp, an ebreak executed via the trap handler delivers a
    // breakpoint exception, re-enters mtvec (TRAP_STUB), and loops.
    let trap_handler_ebreak = TRAP_STUB + 16;
    gdb.set_hw_breakpoint(term_addr, 2)
        .map_err(|e| format!("QEMU Z1 term: {e}"))?;
    gdb.set_hw_breakpoint(trap_handler_ebreak, 2)
        .map_err(|e| format!("QEMU Z1 trap: {e}"))?;
    gdb.continue_exec()
        .map_err(|e| format!("QEMU vCont;c: {e}"))?;
    // Best-effort breakpoint cleanup.
    let _ = gdb.remove_hw_breakpoint(term_addr, 2);
    let _ = gdb.remove_hw_breakpoint(trap_handler_ebreak, 2);

    let cycle_before = emu.core_riscv(0).cycles();
    let undef_before = emu.core_riscv(0).undef_count();
    step_emu_until_pc(emu, term_addr, 256)
        .map_err(|e| format!("emulator: {e}"))?;
    let cycle_after = emu.core_riscv(0).cycles();
    let undef_after = emu.core_riscv(0).undef_count();

    // LLD §10: fuzz-mode must escalate emulator unknown-opcode dispatches
    // to test failure. In targeted-edge-case mode we let these pass (they
    // may be intentional — e.g. checking the illegal-instruction trap
    // path); in fuzz the generator is probabilistic and an unknown
    // opcode almost always signals a decoder gap. The one documented
    // exception is a fuzz case whose generator set `expect_trap = Some(2)`
    // — that case is specifically probing the illegal-instruction path
    // (e.g. Zcmp Q2 collision tripwire), where an `Op::Illegal` dispatch
    // is the intended outcome and the `undef_count` tick is expected.
    let expecting_illegal = tc.expect_trap == Some(2);
    if fuzz_mode && !expecting_illegal && undef_after != undef_before {
        return Err(format!(
            "emulator emitted unknown-opcode warn (undef_count: {undef_before} → {undef_after})"
        ));
    }

    // Read post-state GPRs + PC before running the post-capture stub
    // (which uses gp and t6 as scratch and would otherwise overwrite
    // test outputs in those registers).
    let qemu_regs = read_gprs_qemu(gdb)?;
    let qemu_pc = gdb.read_reg(REG_RV_PC as u8).map_err(|e| format!("QEMU PC: {e}"))?;
    let emu_regs = read_gprs_emu(emu);
    let emu_pc = emu.core_riscv(0).pc();

    // Post-snapshot capture — run a separate stub that reads the 7
    // diff-set CSRs and spills them to the post region of CSR_SCRATCH.
    // Runs AFTER the test halts, so test branches/jumps can't have
    // redirected PC past an inline capture (which was the failure mode
    // of the original in-stream epilogue design).
    // Skip the misaligned-access class: the emulator traps (mcause=4/6
    // per Hazard3 datasheet §3.8.1) and PC is sitting inside the trap
    // handler, so the CSR-read prelude's scratchpad pointer (gp) is not
    // valid for a clean post-snapshot. The class has its own emu-only
    // mcause check below — no proxy snapshot is required.
    let qemu_oracle_valid = tc.class != RiscvClass::Rv32iMisalignedMem;
    if uses_proxy && qemu_oracle_valid {
        capture_post_snapshot(gdb, emu).map_err(|e| format!("post-capture: {e}"))?;
    }

    // Diff registers. Skips:
    //   - x0: architecturally wired to zero.
    //   - x31/t6 when the proxy ran: the CSR-read epilogue reads 7 CSRs
    //     into t6, so t6 always ends holding raw `mip`. That duplicates
    //     the CSR-level mip diff below and would otherwise leak virt-
    //     machine CLINT MTIP (bit 7) into a GPR diff the test case has
    //     no say in. t6 was picked as proxy scratch specifically because
    //     edge cases rarely use x31 as rs1 (mostly just as rd in shift
    //     cases, whose output the test still validates via CSR + memory
    //     checks).
    //   - x3/gp: proxy's scratchpad pointer; pre-seeded, never observed
    //     by the test case (generator debug_assert forbids it).
    //
    // The `qemu_oracle_valid` gate disables the QEMU cross-check for
    // classes where QEMU's virt rv32 platform behaves differently to
    // Hazard3 by design (e.g. misaligned accesses — QEMU virt rv32 has
    // no `misaligned-access=trap` CPU/machine flag in 10.2, so QEMU
    // completes the access while Hazard3 traps). For those classes we
    // fall back to verifying `tc.expect_trap` on the emu side below.
    // Work out whether the test's first word is a CSR-read instruction
    // that lands its result in a specific rd/csr pair. If so, the rd
    // value on that register must be compared through `warl_mask` on
    // both sides — Hazard3 forces `mstatus.MPP = 0b11` (M-mode only)
    // and the emu mirrors this, while QEMU virt rv32 is M+S+U and
    // accepts `MPP = 0b00`. The CLINT-wired `mip.MTIP` platform
    // artefact is the other major culprit. Both are already in
    // `warl_mask`; we just need to route the comparison through it.
    // Applies to both the Zicsr class (single CSR instr) and the
    // CsrSideEffect class (CSR instr followed by a branch).
    let csr_rd_hint: Option<(u8, u16)> = if matches!(
        tc.class,
        RiscvClass::Zicsr | RiscvClass::CsrSideEffect
    ) && !tc.words.is_empty()
    {
        let w = tc.words[0];
        let opcode = w & 0x7F;
        if opcode == 0x73 {
            let funct3 = (w >> 12) & 0b111;
            if funct3 != 0 && funct3 != 0b100 {
                let rd = ((w >> 7) & 0x1F) as u8;
                let csr = ((w >> 20) & 0xFFF) as u16;
                Some((rd, csr))
            } else { None }
        } else { None }
    } else { None };

    const PROXY_SCRATCH: u8 = 31;
    if qemu_oracle_valid {
        for r in 1u8..32 {
            if uses_proxy && (r == PROXY_SCRATCH || r == REG_GP) {
                continue;
            }
            let (q, e) = if let Some((rd, csr)) = csr_rd_hint {
                if rd == r {
                    (warl_mask(csr, qemu_regs[r as usize]),
                     warl_mask(csr, emu_regs[r as usize]))
                } else {
                    (qemu_regs[r as usize], emu_regs[r as usize])
                }
            } else {
                (qemu_regs[r as usize], emu_regs[r as usize])
            };
            if q != e {
                return Err(format!(
                    "x{r} diff: QEMU={q:#010x} emu={e:#010x}"
                ));
            }
        }
        if qemu_pc != emu_pc {
            return Err(format!("PC diff: QEMU={qemu_pc:#010x} emu={emu_pc:#010x}"));
        }
    }

    // Diff the CSR scratchpad (proxy path only, when QEMU is a valid
    // oracle for this class).
    if uses_proxy && qemu_oracle_valid {
        let qemu_snap = gdb
            .read_mem(CSR_SCRATCH, 56)
            .map_err(|e| format!("QEMU read scratch: {e}"))?;
        let emu_snap = read_emu_scratch(emu, CSR_SCRATCH, 56);
        diff_csr_snapshots(&qemu_snap, &emu_snap)?;
    } else if !qemu_oracle_valid {
        // Emulator-only verification: the generator declares the expected
        // trap cause, and the emu's `mcause` must match. This is the only
        // cross-check for classes where QEMU is not a valid oracle.
        if let Some(expected) = tc.expect_trap {
            let emu_mcause = emu.core_riscv(0).mcause();
            if emu_mcause != expected {
                return Err(format!(
                    "emu-only mcause diff: emu={emu_mcause:#010x} expected={expected:#010x}"
                ));
            }
        }
    }
    if !uses_proxy {
        // Zicsr path: optional trap check.
        if let Some(expected) = tc.expect_trap {
            let qemu_mcause = u32::from_le_bytes(
                gdb.read_mem(MCAUSE_SLOT, 4)
                    .map_err(|e| format!("QEMU read mcause slot: {e}"))?
                    .try_into()
                    .map_err(|_| "QEMU mcause slot length".to_string())?,
            );
            let emu_mcause = emu.core_riscv(0).mcause();
            if qemu_mcause != expected {
                return Err(format!(
                    "trap mcause diff: QEMU={qemu_mcause:#010x} expected={expected:#010x}"
                ));
            }
            if emu_mcause != expected {
                return Err(format!(
                    "trap mcause diff: emu={emu_mcause:#010x} expected={expected:#010x}"
                ));
            }
        }
    }

    let _ = test_start; // currently unused; reserved for future diagnostics.
    Ok(cycle_after - cycle_before)
}

/// Build the 3-instruction mtvec-seed prelude used by the Zicsr path
/// (and by [`seed_global_mtvec`] at startup):
///
///   lui   t0, %hi(TRAP_STUB)      ; t0 = 0x2000_0000
///   addi  t0, t0, %lo(TRAP_STUB)  ; t0 = 0x2000_0200
///   csrrw x0, mtvec, t0           ; mtvec = t0 (discard old value)
///
/// Runs on both sides at TEST_SLOT before the test `words`, so any trap
/// the test causes lands at the installed trap-handler stub instead of
/// whatever mtvec was left from the previous case. We reuse t0 (x5); the
/// CSR-read proxy is not in use on the Zicsr path so there's no
/// collision.
fn build_mtvec_seed_prelude() -> [u32; 3] {
    build_mtvec_seed_prelude_with(REG_T0)
}

/// Variant that names the scratch register explicitly. The proxy path
/// passes `x31` (`t6`) because it also clobbers it in the 14-instr CSR-
/// read snapshot below, so the two clobbers coalesce into one reserved
/// register. The Zicsr path and the one-shot [`seed_global_mtvec`]
/// routine both pass `x5` (`t0`) via the no-arg [`build_mtvec_seed_prelude`]
/// wrapper above — they run outside any proxy-prelude context and x5 is
/// not observed by the caller.
fn build_mtvec_seed_prelude_with(scratch: u8) -> [u32; 3] {
    let hi = TRAP_STUB & 0xFFFF_F000;
    let lo = (TRAP_STUB as i32) & 0xFFF;
    let lui = riscv_gen::encode_u_type(hi, scratch, riscv_gen::OPC_LUI);
    let addi = encode_i_type(lo, scratch, 0b000, scratch, OPC_OP_IMM);
    let csrw = encode_csr(0x305, scratch, CSR_F3_RW, REG_X0);
    [lui, addi, csrw]
}

/// Assemble the prelude + test + epilogue + terminator as a flat u32
/// stream. Returns `(stream, terminator_addr, test_start_addr)`.
///
/// For proxy (non-Zicsr) tests the prelude is the 14-instr CSR-read
/// snapshot; for Zicsr it's the 3-instr mtvec-seed prelude.
fn build_test_stream(tc: &RiscvTestCase, use_proxy: bool) -> (Vec<u32>, u32, u32) {
    let mut stream = Vec::with_capacity(32);
    let mut addr = TEST_SLOT;

    if use_proxy {
        // Per-test CSR reset: zero mstatus/mie/mip/mscratch/mepc/mcause
        // and re-seed mtvec = TRAP_STUB. QEMU's mtvec drifts to 0 across
        // the run even without any test writing it (we've not pinned
        // down the exact mechanism, but the "first vCont;c timeout" in
        // `branch_*_neg_off` and `upper_lui_zero` leaves QEMU in a state
        // where the next pre-snapshot reads mtvec as 0 regardless of
        // prior writes, cascading into tens of downstream "CSR diff pre
        // mtvec" failures). Re-seeding mtvec in every proxy prelude
        // keeps both sides aligned per test. Without this reset, cross-
        // test state — especially `mcause` from a prior trap — leaks
        // into the pre-snapshot. Uses `csrrw x0, csr, x0` — rs1=x0
        // writes 0, rd=x0 discards the old value. 6 CSR resets + 3-instr
        // mtvec seed = 9 instrs = 36 bytes.
        const CSR_RESET_LIST: &[u16] = &[0x300, 0x304, 0x344, 0x340, 0x341, 0x342];
        for &csr in CSR_RESET_LIST {
            stream.push(encode_csr(csr, REG_X0, CSR_F3_RW, REG_X0));
        }
        // Use x31 (t6) as the seed scratch so the 14-instr proxy prelude
        // below — which also clobbers x31 — folds the two clobbers into
        // one reserved register. Picking the default x5 would clobber
        // the ALU/mem edge catalogue's rs1 staging.
        for &w in &build_mtvec_seed_prelude_with(31) {
            stream.push(w);
        }
        addr += (CSR_RESET_LIST.len() as u32 + 3) * 4;

        // Proxy prelude: 14 instrs (7 × csrrs t6, csr, x0; sw t6, off(gp)).
        // Uses t6 (x31) as scratch rather than t0 (x5) because edge
        // cases frequently use x5 as rs1 — in particular all mem
        // load/store edge cases. After the prelude's final
        // `csrrs t6, mip, x0`, t6 holds raw mip (which on QEMU virt
        // carries CLINT MTIP in bit 7), so we skip x31 in the GPR diff
        // to keep that platform artefact out of the compare. x31
        // collides far less with the edge catalogue (only as rd in a
        // couple of shift cases, which the diff already ignores when
        // `uses_proxy` is true).
        const PROXY_SCRATCH: u8 = 31;
        for (i, &csr) in CSR_DIFF_ADDRS.iter().enumerate() {
            stream.push(encode_csr(csr, REG_X0, CSR_F3_RS, PROXY_SCRATCH));
            stream.push(encode_s_type(
                (i as i32) * 4,
                PROXY_SCRATCH,
                REG_GP,
                0b010,
                riscv_gen::OPC_STORE,
            ));
        }
        addr += (14 * 4) as u32;
    } else {
        // Zicsr prelude: 6-instr CSR reset (zero the volatile diff-set
        // CSRs so QEMU's drifted state doesn't bleed into the fuzz
        // rd result — the emu side does this via `reset_diff_csrs`
        // but GDB has no flat-index CSR write on QEMU 10.2) + 3-instr
        // mtvec seed = 9 instrs = 36 bytes.
        const CSR_RESET_LIST: &[u16] = &[0x300, 0x304, 0x344, 0x340, 0x341, 0x342];
        for &csr in CSR_RESET_LIST {
            stream.push(encode_csr(csr, REG_X0, CSR_F3_RW, REG_X0));
        }
        for &w in &build_mtvec_seed_prelude() {
            stream.push(w);
        }
        addr += (CSR_RESET_LIST.len() as u32 + 3) * 4;
    }

    let test_start = addr;

    // Test body — write each word per its native width.
    for &w in &tc.words {
        stream.push(w);
        addr += if riscv_gen::is_compressed(w) { 2 } else { 4 };
    }
    // Pad out to 4-byte alignment (epilogue / terminator are all 32-bit
    // instructions). The stream is u32-aligned so if the last test word
    // was 16-bit compressed we insert a c.nop (0x0001) half-word. We pack
    // two halfwords into one u32 in this case.
    if addr % 4 != 0 {
        // If the last pushed u32 held a single 16-bit compressed instr
        // in its low half, pack a c.nop into the high half.
        if let Some(last) = stream.last_mut() {
            if (*last & 0xFFFF_0000) == 0 {
                // Room in the high halfword — pack c.nop there.
                *last |= (0x0001u32) << 16;
                addr += 2;
            } else {
                // Last word is already occupied in both halves — push a
                // 32-bit NOP (addi x0, x0, 0 = 0x00000013).
                stream.push(0x0000_0013);
                addr += 4;
            }
        } else {
            stream.push(0x0000_0013);
            addr += 4;
        }
    }

    // Epilogue: 14 instrs at scratchpad offset +28 bytes (post-snapshot).
    // No epilogue in the test stream — the post-snapshot capture runs
    // separately after the test halts, so branch/jump tests can't
    // redirect PC past it. See `capture_post_snapshot`.

    // Terminator ebreak.
    stream.push(EBREAK_WORD);
    let term_addr = addr;

    (stream, term_addr, test_start)
}

// ============================================================================
// Snapshot diffing with WARL carve-outs (LLD §5)
// ============================================================================

/// Apply the LLD §5 WARL disposition to one CSR value, symmetrically.
/// Called on both QEMU and emulator readings before compare, so a value
/// outside the WARL range (e.g. Hazard3 returning an unexpected
/// `mstatus.MPP` mid-trap) still diffs cleanly against QEMU rather than
/// silently passing because only one side was normalized.
fn warl_mask(csr: u16, v: u32) -> u32 {
    match csr {
        // mstatus — Hazard3 V1 implements only MIE (bit 3), MPIE (bit 7),
        // MPP (bits [12:11], WARL to 0b11 for M-mode-only). Every other
        // mstatus bit is RAZ/WI on Hazard3, but QEMU's rv32 virt CPU has
        // the full M+S+U privilege set and holds live values in SIE (1),
        // SPIE (5), SPP (8), FS (14:13), MPRV (17), SUM (18), MXR (19),
        // TVM (20), TW (21), TSR (22), etc. Mask both sides down to the
        // Hazard3-visible bits and force MPP = 0b11.
        0x300 => {
            const HAZARD3_MSTATUS_VISIBLE: u32 = (1 << 3) | (1 << 7) | (0b11 << 11);
            (v & HAZARD3_MSTATUS_VISIBLE) | (0b11 << 11)
        }
        // mtvec.MODE — bit 0 is the mode select (direct vs vectored); bit
        // 1 is RAZ/WI on Hazard3 (no vectored dispatch). Keep bit 0,
        // clear bit 1, on both sides.
        0x305 => v & !0x2,
        // mie — Hazard3 implements MSIE (bit 3), MTIE (bit 7), MEIE (bit 11).
        0x304 => v & 0x888,
        // mip — MSIP (bit 3), MEIP (bit 11) are architecturally meaningful
        // on both sides. MTIP (bit 7) is NOT: QEMU virt wires mip.MTIP
        // from the CLINT timer compare (always set at reset because
        // mtimecmp=0 and mtime is running), while mdrp2350 has no CLINT
        // model so MTIP stays 0. That's a platform artefact, not a
        // semantics divergence — mask it out both sides.
        0x344 => v & 0x808,
        // mcause — Hazard3 WARL-rounds writes to the set of implemented
        // causes: exceptions {0..=7, 11} and interrupts {3, 7, 11}. Any
        // illegal pattern folds to 0 (preserving the interrupt bit is
        // irrelevant because the legal_code gate on each side of the
        // interrupt/exception split already zeros the code). QEMU accepts
        // arbitrary values on writes, so apply the Hazard3 rounding to
        // both sides before compare (LLD §5). Keeps zicsr / csr-sideeffect
        // fuzz from flagging every random write to mcause as a divergence.
        0x342 => {
            let interrupt = v & 0x8000_0000;
            let code = v & 0x7FFF_FFFF;
            let legal_code = if interrupt != 0 {
                if matches!(code, 3 | 7 | 11) { code } else { 0 }
            } else if code <= 7 || code == 11 { code } else { 0 };
            interrupt | legal_code
        }
        _ => v,
    }
}

/// Compare pre- and post-snapshots from both sides. WARL fields are
/// normalized through [`warl_mask`] on both the QEMU and emulator values
/// before compare (LLD §5 starter table):
///   - `mstatus.MPP` (bits 12:11) forced to `0b11` on both sides.
///   - `mtvec.MODE` (bits 1:0) cleared of bit 1 on both sides.
///   - `mie` / `mip` masked with `0x888` on both sides.
///
/// Layout: `snap[0..28]` = pre, `snap[28..56]` = post, 4 bytes/CSR in the
/// order of [`CSR_DIFF_ADDRS`].
fn diff_csr_snapshots(qemu: &[u8], emu: &[u8]) -> Result<(), String> {
    if qemu.len() != 56 || emu.len() != 56 {
        return Err(format!(
            "scratchpad length: qemu={} emu={} (expected 56)",
            qemu.len(),
            emu.len()
        ));
    }
    for phase in 0..2 {
        let off = phase * 28;
        for i in 0..7 {
            let qb: [u8; 4] = [qemu[off + i * 4], qemu[off + i * 4 + 1], qemu[off + i * 4 + 2], qemu[off + i * 4 + 3]];
            let eb: [u8; 4] = [emu[off + i * 4], emu[off + i * 4 + 1], emu[off + i * 4 + 2], emu[off + i * 4 + 3]];
            let csr = CSR_DIFF_ADDRS[i];
            let qv = warl_mask(csr, u32::from_le_bytes(qb));
            let ev = warl_mask(csr, u32::from_le_bytes(eb));
            if qv != ev {
                let phase_str = if phase == 0 { "pre" } else { "post" };
                return Err(format!(
                    "CSR diff {phase_str} {} ({:#05x}): QEMU={qv:#010x} emu={ev:#010x}",
                    CSR_DIFF_NAMES[i], csr
                ));
            }
        }
    }
    Ok(())
}

// ============================================================================
// Helpers — register / memory shim
// ============================================================================

fn zero_gprs_qemu(gdb: &mut GdbClient) -> Result<(), Box<dyn std::error::Error>> {
    for r in (REG_RV_X0 + 1)..=31 {
        gdb.write_reg(r as u8, 0)?;
    }
    Ok(())
}

fn read_gprs_qemu(gdb: &mut GdbClient) -> Result<[u32; 32], String> {
    let mut out = [0u32; 32];
    for r in 0u8..32 {
        out[r as usize] = gdb.read_reg(r).map_err(|e| format!("QEMU read x{r}: {e}"))?;
    }
    out[0] = 0;
    Ok(out)
}

fn read_gprs_emu(emu: &mdrp2350::Emulator) -> [u32; 32] {
    let mut out = [0u32; 32];
    let h = emu.core_riscv(0);
    for r in 0u8..32 {
        out[r as usize] = h.gpr(r);
    }
    out
}

fn apply_reg_pre_qemu(gdb: &mut GdbClient, tc: &RiscvTestCase) -> Result<(), String> {
    zero_gprs_qemu(gdb).map_err(|e| format!("{e}"))?;
    // gp always points at the CSR-capture scratchpad — the proxy
    // prelude/epilogue use it as the base for spill stores. Distinct
    // from the test data scratchpad at `SCRATCH_BASE` (see
    // `riscv_gen::SCRATCH_BASE` doc).
    gdb.write_reg(REG_GP, CSR_SCRATCH)
        .map_err(|e| format!("QEMU set gp: {e}"))?;
    // mtvec is seeded in-band by the 3-instruction prelude at TEST_SLOT
    // (Zicsr class) — see `build_mtvec_seed_prelude`. We can't write CSRs
    // over GDB (no flat-index support per LLD §3), so seeding in
    // instruction memory is the simplest portable mechanism.
    //
    // Per-test reg_pre from the generator. Skip x0 (wired to zero) and
    // x3/gp (owned by the CSR-proxy for scratchpad addressing — a test
    // case clobbering gp would redirect proxy stores into arbitrary SRAM
    // and mask every CSR comparison downstream). `debug_assert!` surfaces
    // any generator bug at test authoring time rather than silently
    // swallowing it.
    for &(reg, val) in &tc.reg_pre {
        if reg == 0 {
            continue;
        }
        debug_assert!(
            reg != REG_GP,
            "test case sets x3/gp; clobbers CSR-proxy scratchpad pointer"
        );
        if reg == REG_GP {
            continue;
        }
        gdb.write_reg(reg, val)
            .map_err(|e| format!("QEMU set x{reg}: {e}"))?;
    }
    // addr_regs — preload with scratchpad pointers.
    for &reg in &tc.addr_regs {
        if reg == 0 {
            continue;
        }
        gdb.write_reg(reg, SCRATCH_BASE)
            .map_err(|e| format!("QEMU set x{reg}=SCRATCH: {e}"))?;
    }
    Ok(())
}

fn apply_reg_pre_emu(emu: &mut mdrp2350::Emulator, tc: &RiscvTestCase) {
    let h = emu.core_riscv_mut(0);
    for r in 1u8..32 {
        h.set_gpr(r, 0);
    }
    h.set_gpr(REG_GP, CSR_SCRATCH);
    // Skip x3/gp — the CSR proxy owns it; see `apply_reg_pre_qemu` for
    // the full rationale.
    for &(reg, val) in &tc.reg_pre {
        debug_assert!(
            reg != REG_GP,
            "test case sets x3/gp; clobbers CSR-proxy scratchpad pointer"
        );
        if reg == REG_GP {
            continue;
        }
        h.set_gpr(reg, val);
    }
    for &reg in &tc.addr_regs {
        h.set_gpr(reg, SCRATCH_BASE);
    }
}

fn step_emu_until_pc(
    emu: &mut mdrp2350::Emulator,
    target_pc: u32,
    max_steps: u32,
) -> Result<(), String> {
    // `step_quantum=1` plus a halted core 1 means each `Emulator::step`
    // retires one Hazard3 instruction on core 0 and ticks the clock +
    // peripherals by a single sysclk — matching QEMU single-step
    // granularity while keeping `bus.master_cycle` primed via the normal
    // path in `lib.rs` (`bus.master_cycle = self.clock.cycles`). Previous
    // implementation hand-rolled the core dispatch and left
    // `bus.master_cycle` stale, breaking any code path that consults it
    // (PLL lock arming, MMIO trace, pacer).
    for _ in 0..max_steps {
        let pc = emu.core_riscv(0).pc();
        if pc == target_pc {
            return Ok(());
        }
        // Also stop if the instruction at PC is `ebreak` — QEMU's GDB
        // stub intercepts ebreak and halts without delivering the
        // breakpoint exception. Our emulator has no such interception,
        // so an ebreak inside e.g. the trap-handler stub would raise a
        // breakpoint trap, re-enter mtvec (TRAP_STUB), and loop
        // forever. Matching QEMU's stop-at-ebreak behaviour here keeps
        // the two sides synchronised at the same PC.
        //
        // Reading the instruction word through the bus is safe —
        // oracle-region addresses alias into SRAM and have no side
        // effects.
        let op = emu.bus.read32(pc, 0);
        if op == EBREAK_WORD {
            return Ok(());
        }
        // Compressed ebreak (`c.ebreak` = 0x9002) occupies the low
        // halfword. Check that form too — proxy stub terminators are
        // the 32-bit form but the trap handler is identical on both
        // sides; be robust.
        if (op & 0xFFFF) == 0x9002 {
            return Ok(());
        }
        emu.step();
    }
    Err(format!(
        "emulator did not reach terminator {target_pc:#010x} within {max_steps} steps (pc={:#010x})",
        emu.core_riscv(0).pc()
    ))
}

fn read_emu_scratch(emu: &mdrp2350::Emulator, addr: u32, len: usize) -> Vec<u8> {
    // Canonical SRAM alias mask from `mdrp2350::bus` decode — strips the
    // alias bits [27:24], matches what `sram_read8` expects as an offset.
    let off = addr & 0x00FF_FFFF;
    (0..len as u32)
        .map(|i| emu.bus.memory.sram_read8(off + i))
        .collect()
}

fn zero_sram_region(
    gdb: &mut GdbClient,
    emu: &mut mdrp2350::Emulator,
    addr: u32,
    len: u32,
) -> Result<(), String> {
    let zeros = vec![0u8; len as usize];
    gdb.write_mem(addr, &zeros)
        .map_err(|e| format!("QEMU zero {addr:#010x}: {e}"))?;
    // Canonical SRAM alias mask — see `read_emu_scratch`.
    let off = addr & 0x00FF_FFFF;
    for i in 0..len {
        emu.bus.memory.sram_write8(off + i, 0);
    }
    Ok(())
}

/// Concatenate u32 words into a little-endian byte stream. When a word
/// encodes two 16-bit compressed instructions packed into its halfwords
/// (low halfword nonzero, high halfword nonzero, both with `bits[1:0] != 0b11`)
/// we still emit all 4 bytes — the stream is word-granular storage only.
fn words_to_le_bytes(words: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(words.len() * 4);
    for &w in words {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

// Silence dead-code warnings for constants only referenced via the
// encoder helpers — keeps this binary readable without pulling in
// per-item `#[allow(dead_code)]`.
#[allow(dead_code)]
const _OPC_SYSTEM_USE: u32 = OPC_SYSTEM;
