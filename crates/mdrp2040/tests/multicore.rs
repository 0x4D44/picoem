//! RP2040 multicore launch handshake integration tests.
//!
//! See `wrk_docs/2026.04.16 - HLD - RP2040 Core 1 Multicore Launch Handshake.md`
//! §3 for the test catalogue (T1..T9). Exercises the SDK-compatible 6-word
//! handshake (0, 0, 1, VTOR, SP, entry) and the pass-through / restart /
//! rehalt rules specified in §2.3.

use mdrp2040::{Config, EmulatorBuilder};

const SIO_BASE: u32 = 0xD000_0000;
const FIFO_ST: u32 = SIO_BASE + 0x050;
const FIFO_WR: u32 = SIO_BASE + 0x054;
const FIFO_RD: u32 = SIO_BASE + 0x058;
const VLD: u32 = 1 << 0;

/// Helper: push a single word through FIFO_WR as core 0 and return
/// whichever word came back through FIFO_RD (the echo for the armed
/// path). Reads FIFO_ST once before the read and asserts VLD=1 — pins
/// the synchronous-echo contract (§2.4).
fn armed_push_expect_echo(
    emu: &mut mdrp2040::Emulator,
    word: u32,
    expect: u32,
) {
    emu.bus.set_active_core(0);
    emu.bus.write32(FIFO_WR, word);
    let st = emu.bus.read32(FIFO_ST);
    assert!(
        st & VLD != 0,
        "VLD must be set synchronously after armed push (word=0x{:08x})",
        word
    );
    let rd = emu.bus.read32(FIFO_RD);
    assert_eq!(
        rd, expect,
        "echo mismatch: pushed 0x{:08x}, expected 0x{:08x}, got 0x{:08x}",
        word, expect, rd
    );
}

/// Valid six-word handshake producing a launch with
/// VTOR=0x2004_0000, SP=0x2001_0000, entry=0x2000_1001 (Thumb bit set).
const TEST_VTOR: u32 = 0x2004_0000;
const TEST_SP: u32 = 0x2001_0000;
const TEST_ENTRY: u32 = 0x2000_1001;

/// Seed core 0 with a NOP at a safe PC so `emu.step()` runs without
/// faulting on an empty reset vector.
fn seed_core0_nop(emu: &mut mdrp2040::Emulator) {
    let nop_addr = 0x2001_A000u32;
    emu.bus.write16(nop_addr, 0xBF00);
    emu.cores[0].regs.set_pc(nop_addr);
    emu.cores[0].regs.msp = 0x2002_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    emu.cores[0].regs.xpsr = 1 << 24;
}

/// Plant a `B .` self-loop at `entry & !1` so that, after
/// maybe_wake_core1 lands core 1 at the launch entry and the same
/// step quantum continues to run core 1, PC spins in place rather
/// than advancing into zero-initialised SRAM (which would give an
/// off-by-N PC value and churn the test).
fn plant_entry_self_loop(emu: &mut mdrp2040::Emulator, entry: u32) {
    emu.bus.write16(entry & !1, 0xE7FE); // B .
}

// ===========================================================================
// T1 — full handshake wakes core 1 at entry.
// ===========================================================================

#[test]
fn t1_handshake_wakes_core1_at_entry() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");
    assert!(
        emu.cores[1].is_halted(),
        "core 1 should be halted at boot"
    );

    let seq = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    for &w in &seq {
        armed_push_expect_echo(&mut emu, w, w);
    }

    seed_core0_nop(&mut emu);
    plant_entry_self_loop(&mut emu, TEST_ENTRY);

    emu.step().expect("Serial step is infallible");

    assert!(
        !emu.cores[1].is_halted(),
        "core 1 should be awake after handshake"
    );
    assert_eq!(
        emu.cores[1].regs.pc(),
        TEST_ENTRY & !1,
        "PC must be entry with Thumb bit stripped"
    );
    assert_eq!(emu.cores[1].regs.msp, TEST_SP, "MSP must be handshake SP");
    assert_eq!(
        emu.cores[1].regs.r[13],
        TEST_SP,
        "R13 must alias MSP after launch"
    );
    assert_eq!(
        emu.bus.ppb[1].vtor,
        TEST_VTOR,
        "ppb[1].vtor must be handshake VTOR"
    );
}

// ===========================================================================
// T2 — restart on mismatch at seq=2.
// ===========================================================================

#[test]
fn t2_restart_on_mismatch_at_seq2() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // Push {0, 0, 0x42}: third word triggers restart to seq=0.
    armed_push_expect_echo(&mut emu, 0, 0);
    armed_push_expect_echo(&mut emu, 0, 0);
    // At seq=2 the FSM expects 1; any other value echoes 0 and resets
    // seq → 0. Per §2.3.
    armed_push_expect_echo(&mut emu, 0x42, 0);

    // Now push the valid sequence from scratch.
    let seq = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    for &w in &seq {
        armed_push_expect_echo(&mut emu, w, w);
    }

    seed_core0_nop(&mut emu);
    plant_entry_self_loop(&mut emu, TEST_ENTRY);

    emu.step().expect("Serial step is infallible");

    assert!(!emu.cores[1].is_halted());
    assert_eq!(emu.cores[1].regs.pc(), TEST_ENTRY & !1);
    assert_eq!(emu.cores[1].regs.msp, TEST_SP);
    assert_eq!(emu.bus.ppb[1].vtor, TEST_VTOR);
}

// ===========================================================================
// T3 — restart on zero at seq=3 (VTOR slot).
// ===========================================================================

#[test]
fn t3_restart_on_zero_at_seq3() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // Push {0, 0, 1, 0}: at seq=3 a zero word resets to seq=0.
    armed_push_expect_echo(&mut emu, 0, 0);
    armed_push_expect_echo(&mut emu, 0, 0);
    armed_push_expect_echo(&mut emu, 1, 1);
    armed_push_expect_echo(&mut emu, 0, 0);

    seed_core0_nop(&mut emu);

    emu.step().expect("Serial step is infallible");

    assert!(
        emu.cores[1].is_halted(),
        "core 1 must stay halted — no pending_launch was emitted"
    );
}

// ===========================================================================
// T4 — VTOR captured independently of entry.
// ===========================================================================

#[test]
fn t4_vtor_captured_independently_of_entry() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // Pre-seed a distinctly wrong VTOR so we can prove the handshake
    // overwrites it.
    emu.bus.ppb[1].vtor = 0xDEAD_BEEF;

    let seq = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    for &w in &seq {
        armed_push_expect_echo(&mut emu, w, w);
    }

    seed_core0_nop(&mut emu);
    plant_entry_self_loop(&mut emu, TEST_ENTRY);

    emu.step().expect("Serial step is infallible");

    assert_eq!(
        emu.bus.ppb[1].vtor, TEST_VTOR,
        "VTOR must take the handshake value, not the pre-seeded default"
    );
    assert_ne!(emu.bus.ppb[1].vtor, TEST_ENTRY, "VTOR distinct from entry");
}

// ===========================================================================
// T5 — second launch after rehalt.
// ===========================================================================

#[test]
fn t5_second_launch_after_rehalt() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // First launch (same as T1).
    let seq1 = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    for &w in &seq1 {
        armed_push_expect_echo(&mut emu, w, w);
    }
    seed_core0_nop(&mut emu);
    plant_entry_self_loop(&mut emu, TEST_ENTRY);
    emu.step().expect("Serial step is infallible");
    assert!(!emu.cores[1].is_halted());

    // Rehalt core 1 via the wrapper — re-arms FSM.
    emu.halt_core1();
    assert!(emu.cores[1].is_halted());

    // Dirty the core-1 state that `reset_control_for_launch` must clear.
    emu.cores[1].regs.control = 0b10; // SPSEL=1 (PSP)
    emu.cores[1].regs.psp = 0xCAFEBABE;
    // IPSR is the low 9 bits of xpsr — encode an in-handler-mode
    // leftover plus junk flags so reset_control_for_launch has
    // something to wipe.
    emu.cores[1].regs.xpsr = 0x8000_0007; // N flag + IPSR=7, no T bit
    emu.cores[1].regs.primask = 1; // interrupts masked

    // Second handshake with distinct values.
    let vtor2 = 0x2005_0000u32;
    let sp2 = 0x2001_8000u32;
    let entry2 = 0x2000_2001u32;
    let seq2 = [0u32, 0, 1, vtor2, sp2, entry2];
    for &w in &seq2 {
        armed_push_expect_echo(&mut emu, w, w);
    }

    plant_entry_self_loop(&mut emu, entry2);
    emu.step().expect("Serial step is infallible");

    assert!(!emu.cores[1].is_halted(), "second launch must wake core 1");
    assert_eq!(emu.cores[1].regs.pc(), entry2 & !1);
    assert_eq!(emu.cores[1].regs.msp, sp2);
    assert_eq!(emu.cores[1].regs.r[13], sp2);
    assert_eq!(emu.bus.ppb[1].vtor, vtor2);

    // reset_control_for_launch invariants.
    assert_eq!(emu.cores[1].regs.control, 0, "CONTROL reset to 0");
    assert_eq!(emu.cores[1].regs.psp, 0, "PSP reset to 0");
    assert_eq!(
        emu.cores[1].regs.xpsr,
        1 << 24,
        "xPSR reset to T-only"
    );
    assert_eq!(
        emu.cores[1].regs.ipsr(),
        0,
        "IPSR reset (thread mode) — encoded in xpsr[8:0]"
    );
    assert_eq!(emu.cores[1].regs.primask, 0, "PRIMASK reset (un-masked)");
}

// ===========================================================================
// T6 — handshake while awake is pass-through.
// ===========================================================================

#[test]
fn t6_handshake_while_awake_is_pass_through() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // Wake core 1 via the wrapper — FSM disarms.
    emu.wake_core1();
    assert!(!emu.cores[1].is_halted());
    assert!(
        !emu.bus.sio.is_handshake_armed(),
        "FSM must be disarmed once core 1 is awake"
    );

    // Pre-capture core 1 PC so we can prove it's unchanged.
    let pc_before = emu.cores[1].regs.pc();

    // Push what *would* be a valid handshake, through the armed path.
    let seq = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    emu.bus.set_active_core(0);
    for &w in &seq {
        emu.bus.write32(FIFO_WR, w);
    }

    // Pass-through proofs.
    assert_eq!(
        emu.bus.sio.handshake_seq(),
        0,
        "FSM must not advance while unarmed"
    );
    assert!(
        !emu.bus.sio.is_handshake_armed(),
        "FSM must remain disarmed"
    );
    // No echoes on core-0's RX queue (VLD=0 when read as core 0).
    emu.bus.set_active_core(0);
    assert_eq!(
        emu.bus.read32(FIFO_ST) & VLD,
        0,
        "no echoes present when unarmed"
    );

    // All 6 words land in fifo_to_core1: read them from core 1's side.
    emu.bus.set_active_core(1);
    for &expected in &seq {
        let st = emu.bus.read32(FIFO_ST);
        assert!(st & VLD != 0, "core-1 RX queue must hold pushed word");
        let got = emu.bus.read32(FIFO_RD);
        assert_eq!(got, expected, "pass-through word mismatch");
    }

    // Core 1 PC unchanged — no launch was applied.
    assert_eq!(emu.cores[1].regs.pc(), pc_before);
}

// ===========================================================================
// T7 — echo visible on VLD before next instruction.
// ===========================================================================

#[test]
fn t7_echo_visible_on_vld_before_next_instruction() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    let seq = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    emu.bus.set_active_core(0);
    for &w in &seq {
        emu.bus.write32(FIFO_WR, w);
        let st = emu.bus.read32(FIFO_ST);
        assert!(
            st & VLD != 0,
            "VLD must be 1 immediately after write (no intervening step)"
        );
        // Pop so VLD is cleared before the next push — prevents RX
        // queue from backing up and confusing the per-word assertion.
        let _ = emu.bus.read32(FIFO_RD);
    }
}

// ===========================================================================
// T8 — FSM state resets on Emulator::reset.
// ===========================================================================

#[test]
fn t8_fsm_state_resets_on_emulator_reset() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // Advance FSM to seq=2.
    armed_push_expect_echo(&mut emu, 0, 0);
    armed_push_expect_echo(&mut emu, 0, 0);
    assert_eq!(emu.bus.sio.handshake_seq(), 2);

    emu.reset();
    assert_eq!(
        emu.bus.sio.handshake_seq(),
        0,
        "reset must wipe FSM seq"
    );
    assert!(
        emu.bus.sio.is_handshake_armed(),
        "reset re-arms FSM because core 1 is halted"
    );

    // Now push a fresh valid sequence — must wake.
    let seq = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    for &w in &seq {
        armed_push_expect_echo(&mut emu, w, w);
    }

    seed_core0_nop(&mut emu);
    plant_entry_self_loop(&mut emu, TEST_ENTRY);

    emu.step().expect("Serial step is infallible");
    assert!(!emu.cores[1].is_halted());
    assert_eq!(emu.cores[1].regs.pc(), TEST_ENTRY & !1);
}

// ===========================================================================
// T9 — scripted pure-Rust mirror of the SDK sender algorithm.
// ===========================================================================

#[test]
fn t9_scripted_sdk_sender_algorithm() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().expect("Serial build is infallible");

    // Seed core 0 with a NOP so future `emu.step()` inside the VLD
    // poll loop can execute without faulting. (Forward-compat for
    // future WFE-aware paths where the echo only surfaces after the
    // sender's own step — see HLD §2.4.)
    seed_core0_nop(&mut emu);
    // Self-loop at entry so core 1 spins in place post-launch.
    plant_entry_self_loop(&mut emu, TEST_ENTRY);

    let cmd_sequence = [0u32, 0, 1, TEST_VTOR, TEST_SP, TEST_ENTRY];
    let mut seq = 0usize;
    let mut guard = 0u32; // anti-infinite-loop guard
    emu.bus.set_active_core(0);
    while seq < 6 {
        assert!(
            guard < 10_000,
            "scripted sender overran guard — echo protocol broken"
        );
        guard += 1;

        let cmd = cmd_sequence[seq];
        if cmd == 0 {
            // drain fifo_to_core0 (multicore.c:192).
            while emu.bus.read32(FIFO_ST) & VLD != 0 {
                let _ = emu.bus.read32(FIFO_RD);
            }
        }
        emu.bus.write32(FIFO_WR, cmd);
        // pop_blocking: poll VLD then read. Keep emu.step() inside the
        // loop per §3.2 / HLD comment (forward-compat for WFE-aware).
        while emu.bus.read32(FIFO_ST) & VLD == 0 {
            emu.step().expect("Serial step is infallible");
            assert!(guard < 10_000, "VLD poll overran guard");
            guard += 1;
        }
        let response = emu.bus.read32(FIFO_RD);
        seq = if cmd == response { seq + 1 } else { 0 };
    }

    emu.step().expect("Serial step is infallible"); // consume pending_launch
    assert!(!emu.cores[1].is_halted());
    assert_eq!(emu.cores[1].regs.pc(), TEST_ENTRY & !1);
    assert_eq!(emu.cores[1].regs.msp, TEST_SP);
    assert_eq!(emu.bus.ppb[1].vtor, TEST_VTOR);
}
