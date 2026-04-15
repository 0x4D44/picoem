//! OneROM PIO differential — trace generator.
//!
//! Runs the OneROM 2364 scenario once through Piers Finlayson's `epio`,
//! writes a text trace to the path given as `argv[1]`. The trace has a
//! header block carrying the assembled PIO bytecode + per-SM register
//! state, followed by one tab-separated body row per cycle.
//!
//! Re-runs the scenario a second time, row-for-row, and aborts if the
//! two runs disagree (determinism check).
//!
//! See `wrk_docs/2026.04.14 - HLD - OneROM PIO Differential.md`.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::process::ExitCode;

use epio_sys::shim::{
    trace_gen_ctx, trace_gen_dump_instr_mem, trace_gen_dump_sm_reg, trace_gen_free,
    trace_gen_init, trace_gen_step,
};

const SCENARIO_NAME: &str = "onerom_2364_single_byte";
const TRACE_VERSION: u32 = 2;
const CYCLE_COUNT: u32 = 20;

// Stimulus: CS (pin 8) asserted low from cycle 0 onwards, matching
// the flow in epio/test/onerom.c::test_onerom_program.
const CS_PIN_MASK: u32 = 1 << 8;
const INPUT_DRIVE: u32 = CS_PIN_MASK;
const INPUT_LEVEL: u32 = 0;

/// One row of the body section — matches the text columns.
#[derive(Copy, Clone, PartialEq, Eq)]
struct BodyRow {
    cycle: u32,
    input_drive: u32,
    input_level: u32,
    out_drive: u32,
    out_level: u32,
}

/// Per-SM register snapshot. Mirrors `epio_sm_reg_t`.
#[derive(Copy, Clone, Default)]
struct SmReg {
    clkdiv: u32,
    execctrl: u32,
    shiftctrl: u32,
    pinctrl: u32,
}

/// Everything we read back from epio to stamp the header.
struct ProgramDump {
    block: u8,
    instr_count: u32,
    instrs: [u16; 32],
    sm_regs: [SmReg; 4],
}

/// Run the scenario once. Returns the program layout (constant across
/// runs by construction) plus the body rows.
fn run_scenario() -> (ProgramDump, Vec<BodyRow>) {
    unsafe {
        let ctx = trace_gen_init();
        assert!(!ctx.is_null(), "trace_gen_init returned NULL");

        let dump = dump_program(ctx, 0);

        let mut rows = Vec::with_capacity(CYCLE_COUNT as usize);
        for cycle in 0..CYCLE_COUNT {
            let mut out_drive: u32 = 0;
            let mut out_level: u32 = 0;
            trace_gen_step(
                ctx,
                INPUT_DRIVE,
                INPUT_LEVEL,
                &mut out_drive,
                &mut out_level,
            );
            rows.push(BodyRow {
                cycle,
                input_drive: INPUT_DRIVE,
                input_level: INPUT_LEVEL,
                out_drive,
                out_level,
            });
        }

        trace_gen_free(ctx);
        (dump, rows)
    }
}

// -- end of scenario runner ------------------------------------------------

unsafe fn dump_program(ctx: *mut trace_gen_ctx, block: u8) -> ProgramDump {
    let mut instrs = [0u16; 32];
    // SAFETY: `ctx` is a valid context created by `trace_gen_init`; `instrs`
    // is a 32-element u16 buffer with capacity matching the 32-limit in the
    // shim.
    let count = unsafe { trace_gen_dump_instr_mem(ctx, block, instrs.as_mut_ptr(), 32) };

    let mut sm_regs = [SmReg::default(); 4];
    for sm in 0..4u8 {
        let mut r = SmReg::default();
        // SAFETY: same as above; all out-pointers refer to locals.
        unsafe {
            trace_gen_dump_sm_reg(
                ctx,
                block,
                sm,
                &mut r.clkdiv,
                &mut r.execctrl,
                &mut r.shiftctrl,
                &mut r.pinctrl,
            );
        }
        sm_regs[sm as usize] = r;
    }

    ProgramDump {
        block,
        instr_count: count,
        instrs,
        sm_regs,
    }
}

fn write_trace(out_path: &PathBuf, dump: &ProgramDump, rows: &[BodyRow]) -> std::io::Result<()> {
    let file = File::create(out_path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "# onerom differential trace")?;
    writeln!(w, "# version: {}", TRACE_VERSION)?;
    writeln!(w, "# scenario: {}", SCENARIO_NAME)?;
    writeln!(w, "# cycles: {}", CYCLE_COUNT)?;
    writeln!(w, "# pin_width: 32")?;
    writeln!(w, "#")?;
    writeln!(w, "# Program section — one `instr` line per PIO block,")?;
    writeln!(w, "# and one `reg` line per SM (SM regs for unused SMs")?;
    writeln!(w, "# are emitted too; the diff side simply ignores SMs")?;
    writeln!(w, "# whose instr program isn't reached).")?;
    writeln!(w, "#")?;

    // Trim trailing zeros from instr dump — only write what was actually
    // loaded. Scan from the end for the last non-zero entry, default to
    // instr_count otherwise.
    let used_count = trimmed_instr_count(dump);
    write!(w, "instr {} {}", dump.block, used_count)?;
    for i in 0..used_count {
        write!(w, " 0x{:04X}", dump.instrs[i as usize])?;
    }
    writeln!(w)?;

    for sm in 0..4u8 {
        let r = &dump.sm_regs[sm as usize];
        writeln!(
            w,
            "reg {} {} 0x{:08X} 0x{:08X} 0x{:08X} 0x{:08X}",
            dump.block, sm, r.clkdiv, r.execctrl, r.shiftctrl, r.pinctrl
        )?;
    }

    writeln!(w, "#")?;
    writeln!(
        w,
        "# Body rows — columns: cycle, input_drive, input_level, out_drive, out_level"
    )?;
    writeln!(w, "#")?;

    for row in rows {
        writeln!(
            w,
            "{}\t0x{:08X}\t0x{:08X}\t0x{:08X}\t0x{:08X}",
            row.cycle, row.input_drive, row.input_level, row.out_drive, row.out_level
        )?;
    }

    w.flush()
}

/// Return the number of instructions to emit. Uses the dumped count from
/// epio as the upper bound; trims trailing zero halfwords (the post-load
/// scratch area is zero-initialised, so real programs never have a
/// trailing `0x0000` unless they end with `JMP 0` — rare and not our case
/// for OneROM).
fn trimmed_instr_count(dump: &ProgramDump) -> u32 {
    let mut n = dump.instr_count as usize;
    while n > 0 && dump.instrs[n - 1] == 0 {
        n -= 1;
    }
    n as u32
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: {} <output.trace>", args[0]);
        return ExitCode::from(2);
    }
    let out_path = PathBuf::from(&args[1]);

    // Determinism: run twice in-process, assert row-for-row equality.
    let (dump_a, rows_a) = run_scenario();
    let (_dump_b, rows_b) = run_scenario();

    if rows_a != rows_b {
        eprintln!("determinism check FAILED — trace rows differ between runs");
        for (i, (a, b)) in rows_a.iter().zip(rows_b.iter()).enumerate() {
            if a != b {
                eprintln!(
                    "  cycle {}: run_a out_drive=0x{:08X} level=0x{:08X}, \
                     run_b out_drive=0x{:08X} level=0x{:08X}",
                    i, a.out_drive, a.out_level, b.out_drive, b.out_level
                );
                break;
            }
        }
        return ExitCode::FAILURE;
    }

    match write_trace(&out_path, &dump_a, &rows_a) {
        Ok(()) => {
            println!("wrote {} rows to {}", rows_a.len(), out_path.display());
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("failed to write trace: {}", e);
            ExitCode::FAILURE
        }
    }
}

