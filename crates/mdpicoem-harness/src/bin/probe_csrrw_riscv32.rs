// Throwaway probe binary. Single question: does `csrrw` actually write
// CSRs on QEMU 10.2 rv32 + `-machine virt`?
//
// The Stage-6 smoke of the RISC-V diff oracle observed that
// `csrrw rd, mtvec, rs1` appeared to be a silent no-op. Before filtering
// entire test classes to route around it, pin the claim down with a
// minimal, isolated experiment — separate binary, clean GDB session,
// instruction-level logging of every read/write.
//
// Three back-to-back probes. Each one:
//   1. Writes a clearly-identifiable constant to a CSR via csrrw.
//   2. Reads the same CSR back via csrrs into a different GPR.
//   3. Spills both GPRs to a scratchpad for inspection.
//   4. Also spills mstatus + misa + mhartid so we know the CPU mode at
//      probe time (in particular mstatus.MPP tells us whether the
//      csrrw landed while we were in M-mode).
//
// Three CSRs exercised: mscratch (0x340, fully RW, no WARL), mtvec
// (0x305, WARL on MODE field), and mepc (0x341, low bit forced to 0
// with C extension). If all three reject csrrw writes identically, the
// issue is wholesale. If only mtvec rejects, WARL is the culprit. If
// mscratch writes take, csrrw works and something about the Stage-6
// probe's construction was wrong.

use std::process::ExitCode;
use std::time::Duration;

use mdpicoem_harness::gdb_client::{GdbClient, QemuProcess, QemuProfile, REG_RV_PC};
use mdpicoem_harness::riscv_gen::{
    OPC_OP_IMM, OPC_STORE, encode_csr, encode_i_type, encode_s_type, encode_u_type,
};

// virt machine's RAM base — 256 MiB at 0x8000_0000. Root cause of the
// earlier "csrrw silently no-ops" observation: virt maps VIRT_FLASH at
// 0x2000_0000 (64 MB), not RAM. CFI flash lets GDB writes land on the
// backing store (debugger bypasses the write-protect path) but quietly
// drops CPU `sw` instructions that lack the CFI unlock dance. So the
// CSR writes WERE taking effect — we just couldn't observe them
// because the `sw` that spilled the CSR readback to memory never
// actually stored anything. Re-running at 0x8000_0000 confirmed
// csrrw / csrrs / sw all work normally in virt DRAM.
const PROBE_SLOT: u32 = 0x8000_0100;
const SCRATCH: u32 = 0x8000_0200;

// GPR indices.
const X_ZERO: u8 = 0;
const X_T0: u8 = 5;
const X_T1: u8 = 6;
const X_T2: u8 = 7;
const X_T3: u8 = 28;
const X_A0: u8 = 10;

// CSR funct3 codes.
const F3_RW: u32 = 0b001;
const F3_RS: u32 = 0b010;

// Opcodes.
const OPC_LUI: u32 = 0b011_0111;

// CSR addresses tested.
const CSR_MSCRATCH: u16 = 0x340;
const CSR_MTVEC: u16 = 0x305;
const CSR_MEPC: u16 = 0x341;
const CSR_MSTATUS: u16 = 0x300;
const CSR_MISA: u16 = 0x301;
const CSR_MHARTID: u16 = 0xF14;

// Known-identifiable write values.
const WRITE_MSCRATCH: u32 = 0xABCD_0001;
const WRITE_MTVEC: u32 = 0x2000_0400; // 4-byte aligned, MODE=0
const WRITE_MEPC: u32 = 0x2000_0800;

const EBREAK: u32 = 0x0010_0073;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("fatal: {e}");
            ExitCode::from(2)
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // Write a 4-byte boot stub (ebreak) so QEMU has something to load.
    let stub_path = {
        let mut p = std::env::temp_dir();
        p.push(format!("csrrw-probe-rv32-{}.bin", std::process::id()));
        std::fs::write(&p, EBREAK.to_le_bytes())?;
        p
    };
    // Manual cleanup at end of run — best-effort.
    struct Guard(std::path::PathBuf);
    impl Drop for Guard {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    let _guard = Guard(stub_path.clone());

    // Spawn QEMU.
    let _qemu = QemuProcess::spawn_with_image(QemuProfile::RISCV32_RP2350, &stub_path)?;

    // Connect GDB (5s connect window, then 10s read timeout set inside
    // GdbClient).
    let mut gdb = GdbClient::connect(
        &QemuProfile::RISCV32_RP2350.gdb_addr(),
        Duration::from_secs(5),
    )?;
    gdb.handshake()?;

    let pc = gdb.read_reg(32)?; // PC regnum = 32 for RV
    println!("QEMU CPU attached; initial PC = {pc:#010x}");

    // Build the probe stub.
    //
    //   ; load scratchpad pointer into a3 via lui/addi
    //   lui    a0, %hi(SCRATCH)
    //   addi   a0, a0, %lo(SCRATCH)
    //   ; -- block 1: mscratch --
    //   lui    t0, %hi(WRITE_MSCRATCH)
    //   addi   t0, t0, %lo(WRITE_MSCRATCH)
    //   csrrs  t1, mscratch, x0      ; t1 = mscratch BEFORE
    //   csrrw  t3, mscratch, t0      ; t3 = swapped old; mscratch <- t0
    //   csrrs  t2, mscratch, x0      ; t2 = mscratch AFTER
    //   sw     t1, 0(a0)
    //   sw     t2, 4(a0)
    //   sw     t3, 8(a0)
    //   ; -- block 2: mtvec --
    //   lui    t0, %hi(WRITE_MTVEC)
    //   addi   t0, t0, %lo(WRITE_MTVEC)
    //   csrrs  t1, mtvec, x0
    //   csrrw  t3, mtvec, t0
    //   csrrs  t2, mtvec, x0
    //   sw     t1, 12(a0)
    //   sw     t2, 16(a0)
    //   sw     t3, 20(a0)
    //   ; -- block 3: mepc --
    //   lui    t0, %hi(WRITE_MEPC)
    //   addi   t0, t0, %lo(WRITE_MEPC)
    //   csrrs  t1, mepc, x0
    //   csrrw  t3, mepc, t0
    //   csrrs  t2, mepc, x0
    //   sw     t1, 24(a0)
    //   sw     t2, 28(a0)
    //   sw     t3, 32(a0)
    //   ; -- env probe: mstatus, misa, mhartid --
    //   csrrs  t1, mstatus, x0
    //   sw     t1, 36(a0)
    //   csrrs  t1, misa, x0
    //   sw     t1, 40(a0)
    //   csrrs  t1, mhartid, x0
    //   sw     t1, 44(a0)
    //   ebreak

    let mut prog: Vec<u32> = Vec::new();

    // a0 = SCRATCH
    prog.push(encode_u_type(SCRATCH & 0xFFFF_F000, X_A0, OPC_LUI));
    prog.push(encode_i_type(
        (SCRATCH as i32) & 0xFFF,
        X_A0,
        0b000,
        X_A0,
        OPC_OP_IMM,
    ));

    let csrs = [
        (CSR_MSCRATCH, WRITE_MSCRATCH, 0),
        (CSR_MTVEC, WRITE_MTVEC, 12),
        (CSR_MEPC, WRITE_MEPC, 24),
    ];
    for (csr, val, off) in csrs {
        // t0 = val
        prog.push(encode_u_type(val & 0xFFFF_F000, X_T0, OPC_LUI));
        prog.push(encode_i_type(
            (val as i32) & 0xFFF,
            X_T0,
            0b000,
            X_T0,
            OPC_OP_IMM,
        ));
        // t1 = csrrs csr, x0 (read)
        prog.push(encode_csr(csr, X_ZERO, F3_RS, X_T1));
        // t3 = csrrw csr, t0 (swap old with new; t3 = old CSR value after instruction)
        prog.push(encode_csr(csr, X_T0, F3_RW, X_T3));
        // t2 = csrrs csr, x0 (read after)
        prog.push(encode_csr(csr, X_ZERO, F3_RS, X_T2));
        prog.push(encode_s_type(off, X_T1, X_A0, 0b010, OPC_STORE));
        prog.push(encode_s_type(off + 4, X_T2, X_A0, 0b010, OPC_STORE));
        prog.push(encode_s_type(off + 8, X_T3, X_A0, 0b010, OPC_STORE));
    }

    // Environment probes.
    prog.push(encode_csr(CSR_MSTATUS, X_ZERO, F3_RS, X_T1));
    prog.push(encode_s_type(36, X_T1, X_A0, 0b010, OPC_STORE));
    prog.push(encode_csr(CSR_MISA, X_ZERO, F3_RS, X_T1));
    prog.push(encode_s_type(40, X_T1, X_A0, 0b010, OPC_STORE));
    prog.push(encode_csr(CSR_MHARTID, X_ZERO, F3_RS, X_T1));
    prog.push(encode_s_type(44, X_T1, X_A0, 0b010, OPC_STORE));
    prog.push(EBREAK);

    let mut bytes = Vec::with_capacity(prog.len() * 4);
    for w in &prog {
        bytes.extend_from_slice(&w.to_le_bytes());
    }

    let term = PROBE_SLOT + ((prog.len() - 1) as u32) * 4;

    // Write probe + zero scratchpad.
    gdb.write_mem(PROBE_SLOT, &bytes)?;
    gdb.write_mem(SCRATCH, &[0u8; 64])?;

    // Zero all GPRs.
    for r in 1u8..=31 {
        gdb.write_reg(r, 0)?;
    }

    // PC = probe start. Set a HW breakpoint at the ebreak and continue.
    gdb.write_reg(REG_RV_PC as u8, PROBE_SLOT)?;
    gdb.set_hw_breakpoint(term, 2)?;
    gdb.continue_exec()?;
    let _ = gdb.remove_hw_breakpoint(term, 2);

    // Read back PC to confirm we reached the terminator, and read the
    // scratchpad.
    let pc_final = gdb.read_reg(REG_RV_PC as u8)?;
    let scratch = gdb.read_mem(SCRATCH, 48)?;

    // Read back the probe bytes too, to rule out "QEMU silently dropped
    // our memory write" (unlikely given proxy self-check works — but
    // belt and braces for a definitive answer).
    let prog_readback = gdb.read_mem(PROBE_SLOT, (prog.len() * 4) as usize)?;

    println!();
    println!("=== csrrw probe result ===");
    println!(
        "Probe terminator PC expected: {term:#010x}, actual: {pc_final:#010x}  ({})",
        if pc_final == term {
            "hit bp"
        } else {
            "MISSED TERMINATOR"
        }
    );

    let read_u32 = |buf: &[u8], off: usize| -> u32 {
        u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
    };

    let cases: &[(&str, u32, u32)] = &[
        ("mscratch", WRITE_MSCRATCH, 0),
        ("mtvec", WRITE_MTVEC, 12),
        ("mepc", WRITE_MEPC, 24),
    ];
    for (name, write_val, off) in cases {
        let before = read_u32(&scratch, *off as usize);
        let after = read_u32(&scratch, (*off + 4) as usize);
        let swap_old = read_u32(&scratch, (*off + 8) as usize);
        println!();
        println!("{name}:");
        println!("  before csrrw (csrrs read):        {before:#010x}");
        println!("  wrote via csrrw:                  {write_val:#010x}");
        println!("  csrrw rd (old value swapped out): {swap_old:#010x}");
        println!("  after csrrw (csrrs read):         {after:#010x}");
        if after == *write_val {
            println!("  -> VERDICT: csrrw TOOK EFFECT on {name}");
        } else {
            println!(
                "  -> VERDICT: csrrw did NOT take effect on {name} (after == before = {})",
                before == after
            );
        }
    }

    let mstatus = read_u32(&scratch, 36);
    let misa = read_u32(&scratch, 40);
    let mhartid = read_u32(&scratch, 44);
    println!();
    println!("environment CSRs:");
    println!(
        "  mstatus = {mstatus:#010x}  (MPP[12:11] = 0b{:02b}, MIE[3] = {})",
        (mstatus >> 11) & 0x3,
        (mstatus >> 3) & 1
    );
    println!(
        "  misa    = {misa:#010x}  (MXL[31:30] = {}, priv-M-enabled bit not checked)",
        (misa >> 30) & 0x3
    );
    println!("  mhartid = {mhartid:#010x}");

    // Sanity: verify the probe bytes match what we wrote.
    let mut mismatches = 0;
    for (i, w) in prog.iter().enumerate() {
        let got = read_u32(&prog_readback, i * 4);
        if got != *w {
            if mismatches < 3 {
                println!(
                    "  mismatch at {:#010x}: wrote {:#010x}, read {:#010x}",
                    PROBE_SLOT + (i as u32) * 4,
                    w,
                    got
                );
            }
            mismatches += 1;
        }
    }
    if mismatches > 0 {
        println!("WARNING: {mismatches} probe words read back differently than written");
    } else {
        println!();
        println!("Probe bytes in memory match exactly — no memory-write issue.");
    }

    Ok(())
}
