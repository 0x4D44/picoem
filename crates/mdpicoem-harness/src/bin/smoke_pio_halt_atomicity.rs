// Stage 0 smoke: verify PIO freezes atomically when CTRL.SM_ENABLE=0 is
// written via probe-rs after BKPT halt. Validates HLD Assumption 1.
// Disposable — delete after Stage 0.

use probe_rs::{MemoryInterface, Session, SessionConfig};
use std::time::Duration;

const RESETS_BASE: u64 = 0x4002_0000;
const RESETS_RESET: u64 = RESETS_BASE + 0x00;
const RESETS_DONE: u64 = RESETS_BASE + 0x08;
const RESET_PIO0: u32 = 1 << 11;

const PIO0_BASE: u64 = 0x5020_0000;
const PIO_CTRL: u64 = PIO0_BASE + 0x000;
const PIO_INSTR_MEM0: u64 = PIO0_BASE + 0x048;
const PIO_INSTR_MEM1: u64 = PIO0_BASE + 0x04C;
const PIO_SM0_CLKDIV: u64 = PIO0_BASE + 0x0C8;
const PIO_SM0_ADDR: u64 = PIO0_BASE + 0x0D4;
const PIO_SM0_INSTR: u64 = PIO0_BASE + 0x0D8;

// Two-instr program so SM0_ADDR alternates 0 <-> 1 while running.
//   slot 0: MOV Y, Y (PIO nop) = 0xA042
//   slot 1: JMP 0            = 0x0000
const PIO_NOP: u32 = 0xA042;
const PIO_JMP_0: u32 = 0x0000;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("smoke_pio_halt_atomicity: verifying SM_ENABLE=0 gate after BKPT halt");

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;
    core.reset_and_halt(Duration::from_millis(500))?;

    // 1. Release PIO0 from reset and poll RESET_DONE.
    let reset_state: u32 = core.read_word_32(RESETS_RESET)?;
    core.write_word_32(RESETS_RESET, reset_state & !RESET_PIO0)?;
    for _ in 0..1000 {
        if core.read_word_32(RESETS_DONE)? & RESET_PIO0 != 0 {
            break;
        }
    }
    if core.read_word_32(RESETS_DONE)? & RESET_PIO0 == 0 {
        return Err("PIO0 never came out of reset".into());
    }

    // 2. Program INSTR_MEM slots 0 and 1.
    core.write_word_32(PIO_INSTR_MEM0, PIO_NOP)?;
    core.write_word_32(PIO_INSTR_MEM1, PIO_JMP_0)?;

    // 3. Configure SM0: CLKDIV int=1 frac=0 -> 0x0001_0000.
    core.write_word_32(PIO_SM0_CLKDIV, 0x0001_0000)?;
    // Force PC=0 via SM0_INSTR (JMP 0).
    core.write_word_32(PIO_SM0_INSTR, PIO_JMP_0)?;

    // 4. Start SM0 (SM_ENABLE bit 0).
    core.write_word_32(PIO_CTRL, 0x0000_0001)?;

    // 5. Sample ADDR twice to confirm PIO runs while core halted.
    let a_before = core.read_word_32(PIO_SM0_ADDR)?;
    std::thread::sleep(Duration::from_millis(1));
    let a_running = core.read_word_32(PIO_SM0_ADDR)?;

    // 6. Gate SM off, then sample twice to confirm deterministic freeze.
    core.write_word_32(PIO_CTRL, 0x0000_0000)?;
    let a_gated_1 = core.read_word_32(PIO_SM0_ADDR)?;
    std::thread::sleep(Duration::from_millis(10));
    let a_gated_2 = core.read_word_32(PIO_SM0_ADDR)?;

    println!("  A_before  = 0x{a_before:08X}");
    println!("  A_running = 0x{a_running:08X}");
    println!("  A_gated_1 = 0x{a_gated_1:08X}");
    println!("  A_gated_2 = 0x{a_gated_2:08X}");

    let motion = a_running != a_before;
    let frozen = a_gated_1 == a_gated_2;
    let pass = motion && frozen;

    println!();
    println!("  motion (A_running != A_before): {motion}");
    println!("  frozen (A_gated_1 == A_gated_2): {frozen}");
    println!();
    if pass {
        println!("PASS — SM_ENABLE=0 gate freezes PIO atomically");
        Ok(())
    } else {
        println!("FAIL — mitigation insufficient; fall back to self-terminating-only catalog");
        std::process::exit(1)
    }
}
