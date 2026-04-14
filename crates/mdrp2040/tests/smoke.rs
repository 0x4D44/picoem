//! Phase 3 smoke tests for `mdrp2040`. Confirms the skeleton wires up:
//! construct, reset, peek, and basic config/cycle accessors work. The
//! full CPU / bus / peripheral paths arrive in Phase 4+.

use mdrp2040::{Config, Emulator, EmulatorBuilder};

#[test]
fn construct_and_reset() {
    let mut emu = Emulator::new(Config::default());
    emu.reset();
    // Cycles counter starts at 0; just confirm reset doesn't panic.
    assert_eq!(emu.cycles(), 0);
}

#[test]
fn peek_zero_rom() {
    let emu = Emulator::new(Config::default());
    // Reading from an unloaded ROM returns 0.
    assert_eq!(emu.peek(0), 0);
}

#[test]
fn reset_loads_sp_and_pc_from_rom() {
    let mut emu = Emulator::new(Config::default());
    // Write a reset vector into SRAM (peek/poke route only SRAM today;
    // Phase 5 adds ROM-write for test seeding). Instead, seed via
    // direct bus memory access — reset reads from ROM word 0 and 4.
    // With ROM all-zero, reset should set SP=0, PC=0, and both cores
    // should end up with xpsr=Thumb bit.
    emu.reset();
    for core in &emu.cores {
        assert_eq!(core.reg(13), 0);
        assert_eq!(core.regs.msp, 0);
        assert_eq!(core.regs.r[15], 0);
        assert_eq!(core.regs.xpsr & (1 << 24), 1 << 24);
    }
}

#[test]
fn builder_overrides_step_quantum() {
    let emu = EmulatorBuilder::new(Config::default())
        .step_quantum(32)
        .build();
    assert_eq!(emu.step_quantum, 32);
}

#[test]
fn core_ids_are_0_and_1() {
    let emu = Emulator::new(Config::default());
    assert_eq!(emu.core(0).id(), 0);
    assert_eq!(emu.core(1).id(), 1);
}
