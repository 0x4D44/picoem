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
        .build()
        .expect("Serial build is infallible");
    assert_eq!(emu.step_quantum, 32);
}

#[test]
fn core_ids_are_0_and_1() {
    let emu = Emulator::new(Config::default());
    assert_eq!(emu.core(0).id(), 0);
    assert_eq!(emu.core(1).id(), 1);
}

#[test]
fn direct_boot_from_flash_applies_vector_table() {
    // Simulate pico-sdk layout: boot2 at offset 0, vector table at 0x100
    // with SP at word 0 and reset handler (Thumb bit set) at word 1.
    let mut flash = vec![0u8; 0x200];
    // SP = 0x20042000
    flash[0x100..0x104].copy_from_slice(&0x2004_2000u32.to_le_bytes());
    // PC = 0x10000321 (thumb-tagged)
    flash[0x104..0x108].copy_from_slice(&0x1000_0321u32.to_le_bytes());

    let mut emu = EmulatorBuilder::new(Config::default())
        .flash(flash)
        .build()
        .expect("Serial build is infallible");
    emu.reset();
    emu.direct_boot_from_flash(0x100);

    // Both cores should now carry the vector-table values; core 1
    // remains halted (per reset() semantics).
    for core in &emu.cores {
        assert_eq!(core.regs.msp, 0x2004_2000);
        assert_eq!(core.regs.r[13], 0x2004_2000);
        // Thumb bit stripped from PC.
        assert_eq!(core.regs.r[15], 0x1000_0320);
    }
    // Both cores' VTOR must point at the flash vector table — the SDK's
    // runtime_init_install_ram_vector_table copies from `mem[VTOR + 4*i]`
    // and would otherwise read garbage out of the bootrom region.
    assert_eq!(emu.bus.ppb[0].vtor, 0x1000_0100);
    assert_eq!(emu.bus.ppb[1].vtor, 0x1000_0100);
    // Integration oracle: the SDK reads VTOR through the memory-mapped
    // 0xE000_ED08 path via get_vtable(). Phase 1 set the field correctly,
    // but the PPB match in read32 was masking 28 bits then comparing a
    // pattern-style constant, so every SCB read fell through to 0. This
    // assertion proves the end-to-end path is wired up.
    assert_eq!(emu.bus.read32(0xE000_ED08), 0x1000_0100);
    assert!(emu.cores[1].is_halted());
}
