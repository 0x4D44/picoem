//! RP2350-specific PIO tests.
//!
//! The PIO primitive (`PioBlock`, decode, state-machine) lives in
//! [`picoem_common::pio`]. These tests exercise PIO *through the
//! RP2350 `Bus` and `Emulator`*, which is chip-specific — register
//! address layout (PIO0=0x5020_0000, PIO1=0x5030_0000, PIO2=0x5040_0000),
//! number of PIO blocks (3), and the GPIO merge path all sit on
//! `rp2350_emu::Bus`.
//!
//! Tests that exercise only `PioBlock`/`StateMachine` internals stay
//! co-located with the primitive under `picoem-common/src/pio/mod.rs`.

use std::sync::atomic::Ordering;

use crate::bus::Bus;
use crate::{Config, Emulator, EmulatorBuilder};

#[test]
fn test_bus_dispatch_pio0() {
    let mut bus = Bus::new();

    // Write SM0 PINCTRL via PIO0 base address
    bus.write32(0x5020_00DC, 0x1234_5678, 0);

    // Read back
    let val = bus.read32(0x5020_00DC, 0);
    assert_eq!(val, 0x1234_5678);
}

#[test]
fn test_bus_dispatch_pio1_pio2() {
    let mut bus = Bus::new();

    // PIO1: write SM1 CLKDIV (SM1 offset = 0x0E0)
    let clkdiv = (500u32 << 16) | (64u32 << 8);
    bus.write32(0x5030_00E0, clkdiv, 0);
    assert_eq!(bus.read32(0x5030_00E0, 0), clkdiv);

    // PIO2: write CTRL to enable SM3
    bus.write32(0x5040_0000, 0x8, 0);
    assert_eq!(bus.read32(0x5040_0000, 0), 0x8);
    assert!(bus.pio[2].sm[3].enabled());
}

#[test]
fn test_ctrl_alias_set_clr() {
    let mut bus = Bus::new();

    // SET alias: addr + 0x2000 (alias=2)
    // Enable SM0 via SET alias
    bus.write32(0x5020_2000, 0x1, 0); // SET alias on CTRL
    assert!(bus.pio[0].sm[0].enabled());
    assert_eq!(bus.read32(0x5020_0000, 0), 0x1);

    // Enable SM2 via SET alias (SM0 should remain enabled)
    bus.write32(0x5020_2000, 0x4, 0);
    assert!(bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert_eq!(bus.read32(0x5020_0000, 0), 0x5);

    // CLR alias: addr + 0x3000 (alias=3)
    // Disable SM0 via CLR alias
    bus.write32(0x5020_3000, 0x1, 0);
    assert!(!bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert_eq!(bus.read32(0x5020_0000, 0), 0x4);
}

#[test]
fn test_ctrl_alias_xor() {
    // XOR alias: addr + 0x1000 (alias=1). SM_ENABLE bits with 1 toggle
    // the corresponding SM; bits with 0 leave it untouched.
    let mut bus = Bus::new();

    // Start: SM0 and SM2 enabled via normal write.
    bus.write32(0x5020_0000, 0x5, 0);
    assert!(bus.pio[0].sm[0].enabled());
    assert!(!bus.pio[0].sm[1].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert!(!bus.pio[0].sm[3].enabled());

    // XOR with 0x3: toggles SM0 (1->0) and SM1 (0->1); SM2/SM3 unchanged.
    bus.write32(0x5020_1000, 0x3, 0);
    assert!(!bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[1].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert!(!bus.pio[0].sm[3].enabled());
    assert_eq!(bus.read32(0x5020_0000, 0), 0x6);

    // XOR with 0x0: no-op.
    bus.write32(0x5020_1000, 0x0, 0);
    assert_eq!(bus.read32(0x5020_0000, 0), 0x6);

    // XOR with 0xF: toggles every SM.
    bus.write32(0x5020_1000, 0xF, 0);
    assert!(bus.pio[0].sm[0].enabled());
    assert!(!bus.pio[0].sm[1].enabled());
    assert!(!bus.pio[0].sm[2].enabled());
    assert!(bus.pio[0].sm[3].enabled());
    assert_eq!(bus.read32(0x5020_0000, 0), 0x9);
}

#[test]
fn test_gpio_in_moved_to_bus() {
    let mut bus = Bus::new();
    bus.gpio_in.store(0xFF, Ordering::Relaxed);

    // Read SIO GPIO_IN via bus at 0xD000_0004
    let val = bus.read32(0xD000_0004, 0);
    assert_eq!(val, 0xFF);
}

#[test]
fn test_gpio_merge_pio_overrides_sio() {
    // SIO drives pin 5 = 1. PIO0 drives pin 5 = 0 (with OE).
    // Verify bus.gpio_in bit 5 = 0 (PIO wins).
    let mut emu = Emulator::new(Config::default());
    // SIO: set pin 5 high with OE
    emu.bus.sio.gpio_out = 1 << 5;
    emu.bus.sio.gpio_oe = 1 << 5;
    // PIO0 pad_out: pin 5 = 0, pad_oe: pin 5 driven
    emu.bus.pio[0].pad_oe = 1 << 5;
    emu.bus.pio[0].pad_out = 0; // pin 5 = 0

    emu.update_gpio();
    assert_eq!(
        emu.bus.gpio_in.load(Ordering::Relaxed) & (1 << 5),
        0,
        "PIO overrides SIO on pin 5"
    );
}

#[test]
fn test_gpio_merge_independent_pins() {
    // PIO drives pin 5, SIO drives pin 10. Both should appear in gpio_in.
    let mut emu = Emulator::new(Config::default());
    // SIO drives pin 10
    emu.bus.sio.gpio_out = 1 << 10;
    emu.bus.sio.gpio_oe = 1 << 10;
    // PIO0 drives pin 5
    emu.bus.pio[0].pad_oe = 1 << 5;
    emu.bus.pio[0].pad_out = 1 << 5;

    emu.update_gpio();
    let gpio_in = emu.bus.gpio_in.load(Ordering::Relaxed);
    assert_ne!(gpio_in & (1 << 5), 0, "PIO pin 5 appears");
    assert_ne!(gpio_in & (1 << 10), 0, "SIO pin 10 appears");
}

// ====================================================================
// Stage D: Waveform integration tests
// ====================================================================
//
// These tests verify that PIO programs running through the full
// Emulator produce correct GPIO waveforms with cycle-accurate timing.

const PIO0_BASE: u32 = 0x5020_0000;
const PIO1_BASE: u32 = 0x5030_0000;

/// Write a PIO0 register through the emulator bus.
fn pio_write(emu: &mut Emulator, offset: u32, val: u32) {
    emu.bus.write32(PIO0_BASE + offset, val, 0);
}

fn pio1_write(emu: &mut Emulator, offset: u32, val: u32) {
    emu.bus.write32(PIO1_BASE + offset, val, 0);
}

/// Create an emulator configured for PIO integration tests.
///
/// Uses `step_quantum=1` so each `emu.step()` advances by exactly
/// one cycle — these tests read PIO pin state on a per-cycle basis,
/// which the quantum execution model would otherwise smear across up
/// to `DEFAULT_STEP_QUANTUM` cycles.
///
/// Releases PIO0 from RESETS so `tick_peripherals` actually ticks it.
/// PIO is held in reset post-bootrom (§7.5); the existing waveform
/// tests all target PIO0 and need it released before `step()`.
fn pio_test_emulator() -> Emulator {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    // De-assert PIO0's RESETS bit via the CLR alias.
    emu.bus.resets_state &= !(1u32 << crate::bus::RESET_PIO0);
    emu
}

fn release_pio(emu: &mut Emulator, block: u8) {
    emu.bus.resets_state &= !(1u32 << (crate::bus::RESET_PIO0 + block));
}

/// Load a PIO program into instruction memory via bus writes.
fn pio_load_program(emu: &mut Emulator, program: &[u16]) {
    for (i, &insn) in program.iter().enumerate() {
        pio_write(emu, 0x048 + (i as u32) * 4, insn as u32);
    }
}

fn pio1_load_program(emu: &mut Emulator, program: &[u16]) {
    for (i, &insn) in program.iter().enumerate() {
        pio1_write(emu, 0x048 + (i as u32) * 4, insn as u32);
    }
}

#[test]
fn pio_gpiobase_16_sees_gpio_external_in_hi() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    release_pio(&mut emu, 1);

    // IN PINS, 19. GPIOBASE=16 maps physical GPIO34 to local pin 18.
    pio1_load_program(&mut emu, &[0x4013]);
    pio1_write(&mut emu, 0x168, 16);
    let shiftctrl = emu.bus.pio[1].read32(0x0D0) & !(1 << 18);
    pio1_write(&mut emu, 0x0D0, shiftctrl);
    pio1_write(&mut emu, 0x000, 0x1);

    emu.bus.set_gpio_external_in_hi(1 << 2, 1 << 2);
    emu.run(1).unwrap();

    assert_eq!(emu.bus.pio[1].sm[0].isr_value(), 1 << 18);
    assert_eq!(emu.bus.pio[1].sm[0].isr_shift_count(), 19);
}

#[test]
fn update_gpio_maps_pio_gpiobase_16_outputs_to_gpio_in_hi() {
    let mut emu = Emulator::new(Config::default());

    emu.bus.pio[1].write32(0x168, 16, 0);
    emu.bus.pio[1].pad_out = (1 << 0) | (1 << 18) | (1 << 19) | (1 << 31);
    emu.bus.pio[1].pad_oe = (1 << 0) | (1 << 18) | (1 << 31);
    // PIO2 stays at base 0; this guards against accidentally making
    // GPIOBASE global instead of per block.
    emu.bus.pio[2].pad_out = 1 << 3;
    emu.bus.pio[2].pad_oe = 1 << 3;

    emu.update_gpio();

    let lo = emu.bus.gpio_in.load(Ordering::Relaxed);
    let hi = emu.bus.gpio_in_hi.load(Ordering::Relaxed);
    assert_eq!(lo & (1 << 16), 1 << 16, "PIO1 local pin 0 maps to GPIO16");
    assert_eq!(
        lo & (1 << 3),
        1 << 3,
        "PIO2 base 0 output stays in low bank"
    );
    assert_eq!(hi & (1 << 2), 1 << 2, "PIO1 local pin 18 maps to GPIO34");
    assert_eq!(hi & (1 << 15), 1 << 15, "PIO1 local pin 31 maps to GPIO47");
    assert_eq!(
        hi & (1 << 3),
        0,
        "pad_out without matching pad_oe must not drive high bank"
    );
}

#[test]
fn gpio_external_in_hi_preserved_in_gpio_in_hi_merge() {
    let mut emu = Emulator::new(Config::default());

    emu.bus.pio[1].write32(0x168, 16, 0);
    emu.bus.pio[1].pad_out = 0;
    emu.bus.pio[1].pad_oe = 1 << 18; // would drive GPIO34 low.
    emu.bus.set_gpio_external_in_hi(1 << 2, 1 << 2);

    emu.update_gpio();

    assert_eq!(
        emu.bus.gpio_in_hi.load(Ordering::Relaxed) & (1 << 2),
        1 << 2,
        "external high-bank stimulus must overlay PIO output in gpio_in_hi"
    );
    assert_eq!(emu.bus.read32(0xD000_0008, 0) & (1 << 2), 1 << 2);
}

#[test]
fn test_pio_blinky_gpio25() {
    // PIO program: toggle GPIO 25 every cycle, looping.
    //   addr 0: SET PINS, 1    (drive pin HIGH)
    //   addr 1: SET PINS, 0    (drive pin LOW)
    //   addr 2: JMP 0          (loop)
    //
    // With clkdiv=1, each instruction executes in 1 system clock.
    // Pattern repeats every 3 clocks: HIGH, LOW, LOW(jmp).

    let mut emu = pio_test_emulator();

    // Load program
    let set_pins_1: u16 = 0xE001; // SET PINS, 1
    let set_pins_0: u16 = 0xE000; // SET PINS, 0
    let jmp_0: u16 = 0x0000; // JMP 0
    pio_load_program(&mut emu, &[set_pins_1, set_pins_0, jmp_0]);

    // SM0_PINCTRL: set_base=25, set_count=1
    // set_count at bits[28:26], set_base at bits[9:5]
    let pinctrl = (1u32 << 26) | (25u32 << 5);
    pio_write(&mut emu, 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=2, wrap_bottom=0
    let execctrl = 2u32 << 12;
    pio_write(&mut emu, 0x0CC, execctrl);

    // Force-execute SET PINDIRS, 1 to enable output on pin 25.
    // SET PINDIRS, 1: opcode=111, dest=100(PINDIRS), data=00001
    // = 0b111_00000_100_00001 = 0xE081
    pio_write(&mut emu, 0x0D8, 0xE081);

    // Enable SM0: write 1 to CTRL
    pio_write(&mut emu, 0x000, 0x1);

    // Run 12 cycles (4 complete 3-cycle patterns).
    // Expected pin 25 after each step:
    //   Step 1: SET PINS,1 => HIGH
    //   Step 2: SET PINS,0 => LOW
    //   Step 3: JMP 0      => LOW (no pin change)
    //   Step 4: SET PINS,1 => HIGH
    //   ... repeats
    let expected = [
        true, false, false, // pattern 1
        true, false, false, // pattern 2
        true, false, false, // pattern 3
        true, false, false, // pattern 4
    ];

    let mut actual = Vec::new();
    for _ in 0..12 {
        emu.step().unwrap();
        actual.push(emu.gpio_read(25));
    }

    assert_eq!(
        actual, expected,
        "GPIO 25 waveform mismatch over 12 cycles\n  actual:   {:?}\n  expected: {:?}",
        actual, expected
    );
}

#[test]
fn test_pio_uart_tx_0x55() {
    // PIO program: shift out 8 data bits LSB-first from OSR.
    //   addr 0: PULL BLOCK       (wait for TX FIFO data)
    //   addr 1: SET X, 7         (bit counter = 8-1)
    //   addr 2: OUT PINS, 1      (shift 1 data bit to pin)
    //   addr 3: JMP X-- 2        (loop 8 times)
    //   addr 4: JMP 0            (next byte)
    //
    // With clkdiv=1, each instruction = 1 system clock.
    // Data bits appear on OUT steps; JMP steps leave the pin unchanged.
    // Each data bit occupies 2 system clocks (OUT + JMP), except
    // the last bit (X was 0, JMP falls through to addr 4).

    let mut emu = pio_test_emulator();

    let pull_block: u16 = 0x80A0;
    let set_x_7: u16 = 0xE027;
    let out_pins_1: u16 = 0x6001;
    let jmp_xdec_2: u16 = 0x0042;
    let jmp_0: u16 = 0x0000;
    pio_load_program(
        &mut emu,
        &[pull_block, set_x_7, out_pins_1, jmp_xdec_2, jmp_0],
    );

    // SM0_PINCTRL: out_base=0, out_count=1, set_count=1, set_base=0
    let pinctrl = (1u32 << 26) | (1u32 << 20);
    pio_write(&mut emu, 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=4, wrap_bottom=0
    let execctrl = 4u32 << 12;
    pio_write(&mut emu, 0x0CC, execctrl);

    // SM0_SHIFTCTRL: OUT_SHIFTDIR=1 (shift right, LSB first).
    // Default shiftctrl = 0x000C_0000 which has bit 19 set already.
    // Keep defaults.

    // Force-execute SET PINDIRS, 1 to enable output on pin 0.
    pio_write(&mut emu, 0x0D8, 0xE081);

    // Push 0x55 (0b01010101) to TX FIFO
    pio_write(&mut emu, 0x010, 0x55);

    // Enable SM0
    pio_write(&mut emu, 0x000, 0x1);

    // Timeline (clkdiv=1):
    //   Step 1: PULL BLOCK => OSR = 0x55
    //   Step 2: SET X, 7
    //   Step 3: OUT PINS, 1 => pin = bit0 of 0x55 = 1 (HIGH)
    //   Step 4: JMP X-- 2  (X was 7 -> 6, jump taken) => pin unchanged
    //   Step 5: OUT PINS, 1 => pin = bit1 = 0 (LOW)
    //   Step 6: JMP X-- 2  (X: 6->5, taken)
    //   Step 7: OUT PINS, 1 => pin = bit2 = 1 (HIGH)
    //   Step 8: JMP X-- 2  (X: 5->4, taken)
    //   Step 9: OUT PINS, 1 => pin = bit3 = 0 (LOW)
    //   Step 10: JMP X-- 2 (X: 4->3, taken)
    //   Step 11: OUT PINS, 1 => pin = bit4 = 1 (HIGH)
    //   Step 12: JMP X-- 2 (X: 3->2, taken)
    //   Step 13: OUT PINS, 1 => pin = bit5 = 0 (LOW)
    //   Step 14: JMP X-- 2 (X: 2->1, taken)
    //   Step 15: OUT PINS, 1 => pin = bit6 = 1 (HIGH)
    //   Step 16: JMP X-- 2 (X: 1->0, taken — X was nonzero)
    //   Step 17: OUT PINS, 1 => pin = bit7 = 0 (LOW)
    //   Step 18: JMP X-- 2 (X was 0, not taken => falls to addr 4)
    //   Step 19: JMP 0

    // Data bits of 0x55 = 0b01010101, LSB first: 1,0,1,0,1,0,1,0
    // Each bit appears on the OUT step.

    // Collect pin 0 state at each step
    let total_steps = 19;
    let mut pin_trace = Vec::new();
    for _ in 0..total_steps {
        emu.step().unwrap();
        pin_trace.push(emu.gpio_read(0));
    }

    // Extract the 8 data bits from the OUT-instruction steps.
    // OUT executes at steps: 3, 5, 7, 9, 11, 13, 15, 17 (1-indexed)
    let out_steps: Vec<usize> = vec![2, 4, 6, 8, 10, 12, 14, 16]; // 0-indexed
    let mut received_bits: Vec<bool> = Vec::new();
    for &i in &out_steps {
        received_bits.push(pin_trace[i]);
    }

    // Expected: 0x55 LSB-first = 1,0,1,0,1,0,1,0
    let expected_bits: Vec<bool> = vec![true, false, true, false, true, false, true, false];
    assert_eq!(
        received_bits, expected_bits,
        "UART TX 0x55 data bits mismatch (LSB first)\n  received: {:?}\n  expected: {:?}",
        received_bits, expected_bits
    );

    // Reconstruct the byte from received bits
    let mut byte: u8 = 0;
    for (i, &bit) in received_bits.iter().enumerate() {
        if bit {
            byte |= 1 << i;
        }
    }
    assert_eq!(
        byte, 0x55,
        "reconstructed byte should be 0x55, got {:#04x}",
        byte
    );
}

#[test]
fn test_pio_spi_clk_mosi() {
    // PIO SPI program: clock out 8 bits with CLK on side-set (pin 1)
    // and MOSI on OUT (pin 0).
    //
    //   addr 0: PULL BLOCK  side 0  (get data, CLK LOW)
    //   addr 1: SET X, 7    side 0  (8 bits, CLK LOW)
    //   addr 2: OUT PINS, 1 side 1  (MOSI = data bit, CLK HIGH)
    //   addr 3: JMP X-- 2   side 0  (CLK LOW, loop)
    //   addr 4: JMP 0       side 0  (done, CLK LOW)
    //
    // With sideset_count=1, SIDE_EN=0:
    //   bit[12] = side-set value, bits[11:8] = delay

    let mut emu = pio_test_emulator();

    // Encode instructions (sideset_count=1, bit 12 = sideset)
    let pull_block_s0: u16 = 0x80A0; // 100_0_0000_10100000
    let set_x7_s0: u16 = 0xE027; // 111_0_0000_001_00111
    let out_pins1_s1: u16 = 0x7001; // 011_1_0000_000_00001
    let jmp_xdec2_s0: u16 = 0x0042; // 000_0_0000_010_00010
    let jmp_0_s0: u16 = 0x0000; // 000_0_0000_000_00000
    pio_load_program(
        &mut emu,
        &[
            pull_block_s0,
            set_x7_s0,
            out_pins1_s1,
            jmp_xdec2_s0,
            jmp_0_s0,
        ],
    );

    // SM0_PINCTRL:
    //   out_base=0 (MOSI on pin 0), out_count=1
    //   set_base=0, set_count=2 (covers both MOSI at pin 0 and CLK at pin 1)
    //   sideset_base=1 (CLK on pin 1), sideset_count=1
    let pinctrl = (1u32 << 29)   // sideset_count=1
                | (2u32 << 26)   // set_count=2
                | (1u32 << 20)   // out_count=1
                | (1u32 << 10); // out_base=0
    pio_write(&mut emu, 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=4, wrap_bottom=0, SIDE_EN=0
    let execctrl = 4u32 << 12;
    pio_write(&mut emu, 0x0CC, execctrl);

    // Force-execute SET PINDIRS, 3 — bits[1:0] drive the direction
    // latch for the two SET pins starting at SET_BASE=0: pin 0 (MOSI)
    // and pin 1 (CLK). Silicon requires explicit PINDIRS programming
    // for side-set pins to drive; side-set with SIDE_PINDIR=0 writes
    // pin values only, not directions (RP2350 §11.3.2.3).
    pio_write(&mut emu, 0x0D8, 0xE083); // SET PINDIRS, 3

    // Push 0x55 to TX FIFO
    pio_write(&mut emu, 0x010, 0x55);

    // Enable SM0
    pio_write(&mut emu, 0x000, 0x1);

    // Timeline (same structure as UART TX, but with CLK side-set):
    //   Step 1: PULL BLOCK  side 0 => CLK=0, OSR=0x55
    //   Step 2: SET X, 7    side 0 => CLK=0
    //   Step 3: OUT PINS, 1 side 1 => CLK=1, MOSI=bit0=1
    //   Step 4: JMP X-- 2   side 0 => CLK=0 (falling edge)
    //   Step 5: OUT PINS, 1 side 1 => CLK=1, MOSI=bit1=0
    //   Step 6: JMP X-- 2   side 0 => CLK=0
    //   ...
    //   Step 17: OUT PINS, 1 side 1 => CLK=1, MOSI=bit7=0
    //   Step 18: JMP X-- 2  side 0 => CLK=0 (X was 0, falls through)
    //   Step 19: JMP 0      side 0 => CLK=0

    let total_steps = 19;
    let mut clk_trace = Vec::new();
    let mut mosi_trace = Vec::new();
    for _ in 0..total_steps {
        emu.step().unwrap();
        clk_trace.push(emu.gpio_read(1));
        mosi_trace.push(emu.gpio_read(0));
    }

    // CLK should be HIGH only on OUT steps (side 1): steps 3,5,7,...,17
    // (0-indexed: 2,4,6,8,10,12,14,16)
    let expected_clk: Vec<bool> = (0..total_steps)
        .map(|i| {
            // OUT steps at 0-indexed: 2, 4, 6, 8, 10, 12, 14, 16
            (2..=16).contains(&i) && i % 2 == 0
        })
        .collect();

    assert_eq!(
        clk_trace, expected_clk,
        "SPI CLK waveform mismatch\n  actual:   {:?}\n  expected: {:?}",
        clk_trace, expected_clk
    );

    // MOSI data bits (sampled on CLK rising edges = OUT steps)
    let out_steps: Vec<usize> = vec![2, 4, 6, 8, 10, 12, 14, 16];
    let mut mosi_bits: Vec<bool> = Vec::new();
    for &i in &out_steps {
        mosi_bits.push(mosi_trace[i]);
    }

    // Expected: 0x55 LSB-first = 1,0,1,0,1,0,1,0
    let expected_mosi: Vec<bool> = vec![true, false, true, false, true, false, true, false];
    assert_eq!(
        mosi_bits, expected_mosi,
        "SPI MOSI data mismatch (LSB first)\n  actual:   {:?}\n  expected: {:?}",
        mosi_bits, expected_mosi
    );

    // Verify CLK and MOSI timing relationship: MOSI transitions
    // should be captured on the CLK rising edge (OUT instruction).
    // On CLK falling edges (JMP instruction), MOSI holds its value.
    for &i in &out_steps {
        assert!(
            clk_trace[i],
            "CLK must be HIGH when MOSI data bit is presented (step {})",
            i
        );
    }
}

/// Reproduce the OneROM PIO2 data-writer shape in isolation: SM1 runs
/// `OUT PINS, 8` at a wrap_top=wrap_bottom=6 self-loop, with autopull
/// threshold 8 and shift-right. A byte pushed to SM1's TX FIFO should
/// autopull into the OSR and drive `pad_out` bits 16..23 within a few
/// cycles. This guards the PIO primitive against regressions of the
/// exact shape OneROM's data-writer SM uses — see
/// `wrk_journals/2026.04.15 - JRN - PIO2 pad_out Propagation Fix.md`.
///
/// Pushes go to SM1's TX FIFO (TXF1 at offset `0x014`), not SM0's (TXF0
/// at `0x010`). Routing pushes to the wrong FIFO is exactly the
/// harness-side bug the accompanying glue-DMA fix corrects; this test
/// proves the PIO engine itself correctly handles the shape.
#[test]
fn test_pio_onerom_sm1_out_pins_8_autopull_drives_pad_out() {
    let mut emu = pio_test_emulator();

    // OneROM's PIO2 data-writer is a one-instruction loop at PC 6.
    // The instructions before PC 6 belong to SM0 (CS handler) in the
    // real firmware; we leave them zero here — SM1 never reaches them
    // because its wrap_top = wrap_bottom = 6.
    let out_pins_8: u16 = 0x6008; // OUT PINS, 8
    pio_load_program(&mut emu, &[0, 0, 0, 0, 0, 0, out_pins_8, 0]);

    // SM1 base = 0x0E0 (SM0=0x0C8, stride 0x18).
    const SM1_EXECCTRL: u32 = 0x0E0 + 0x04;
    const SM1_SHIFTCTRL: u32 = 0x0E0 + 0x08;
    const SM1_PINCTRL: u32 = 0x0E0 + 0x14;
    const SM1_INSTR: u32 = 0x0E0 + 0x10;

    // SM1 EXECCTRL: wrap_top = 6, wrap_bottom = 6 (self-loop on PC 6).
    // Other fields zero — no side-set, no JMP_PIN, etc.
    let execctrl: u32 = (6u32 << 12) | (6u32 << 7);
    pio_write(&mut emu, SM1_EXECCTRL, execctrl);

    // SM1 SHIFTCTRL: OUT_SHIFTDIR = 1 (shift right), AUTOPULL = 1,
    // PULL_THRESH = 8, IN_COUNT = 0. Bit 19 = OUT_SHIFTDIR,
    // bit 17 = AUTOPULL, bits [29:25] = PULL_THRESH.
    let shiftctrl: u32 = (1u32 << 19) | (1u32 << 17) | (8u32 << 25);
    pio_write(&mut emu, SM1_SHIFTCTRL, shiftctrl);

    // SM1 PINCTRL: OUT_BASE = 16, OUT_COUNT = 8 (D0..D7 on GPIO 16..23).
    // bits [25:20] = OUT_COUNT, bits [4:0] = OUT_BASE.
    let pinctrl: u32 = (8u32 << 20) | 16u32;
    pio_write(&mut emu, SM1_PINCTRL, pinctrl);

    // Force SM1's PC to 6 via force-execute of `JMP 6` (opcode 0x0006).
    pio_write(&mut emu, SM1_INSTR, 0x0006);

    // Enable output direction for pins 16..23 by force-executing a
    // MOV PINDIRS, !NULL via SM1 (PINCTRL OUT_BASE/OUT_COUNT picks the
    // window). Encoding: MOV dest=PINDIRS(3), op=invert(1), source=NULL(3)
    // = 1010_0000_011_01_011 = 0xA06B.
    pio_write(&mut emu, SM1_INSTR, 0xA06B);

    // Enable SM1.
    pio_write(&mut emu, 0x000, 0b0010);

    // Seed SM1's TX FIFO with a fresh byte every cycle so the autopull
    // pipeline always has data. Byte is `0xAA` — nonzero and
    // distinguishable from `0xFF` (reset) and `0x00`.
    const PAYLOAD: u8 = 0xAA;
    let word: u32 = (PAYLOAD as u32)
        | ((PAYLOAD as u32) << 8)
        | ((PAYLOAD as u32) << 16)
        | ((PAYLOAD as u32) << 24);

    // Push once and step enough cycles for autopull → OSR → OUT PINS →
    // shared_pin_values → pad_out propagation. With autopull refilling
    // the OSR, OUT PINS, 8 drops one byte per step, so the new value is
    // latched by `merge_pin_outputs` on the very first step.
    pio_write(&mut emu, 0x014, word); // TXF1 (SM1's TX FIFO)
    emu.step().unwrap();
    emu.step().unwrap();

    // Helpers default to PIO0; read the same block here.
    let pad_out = emu.bus.pio[0].pad_out;
    let data_slice = ((pad_out >> 16) & 0xFF) as u8;
    assert_eq!(
        data_slice, PAYLOAD,
        "pad_out bits 16..23 should reflect 0x{:02X} after SM1 autopull+OUT PINS; \
         got pad_out=0x{:08X}",
        PAYLOAD, pad_out
    );

    // Sanity: pad_oe should also have the D0..D7 region driven, proving
    // MOV PINDIRS ran through the shared direction latch. This mirrors
    // the OneROM sync-time state (pad_oe = 0xFF across D0..D7).
    let pad_oe_slice = ((emu.bus.pio[0].pad_oe >> 16) & 0xFF) as u8;
    assert_eq!(
        pad_oe_slice, 0xFF,
        "pad_oe bits 16..23 should be driven after MOV PINDIRS, !NULL; \
         got pad_oe=0x{:08X}",
        emu.bus.pio[0].pad_oe
    );
}

// ====================================================================
// Phase 4.3: PIO RESETS gating
// ====================================================================
//
// Real hardware holds PIO inert while its RESETS bit is asserted.
// The tick path in `Emulator::tick_peripherals` must skip a PIO block
// whose RESETS bit is still set. This test verifies that contract.

#[test]
fn test_pio_resets_gating() {
    // Build an emulator with step_quantum=1 but do NOT release PIO0
    // from reset — the default `RESETS_POST_BOOTROM` holds PIO0/1/2.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();

    // Confirm PIO0 is held in reset.
    assert_ne!(
        emu.bus.resets_state & (1u32 << crate::bus::RESET_PIO0),
        0,
        "PIO0 should be held in reset post-bootrom"
    );

    // Program a trivial PIO0 SM0 program: NOP loop.
    //   addr 0: MOV Y, Y  (NOP — any non-branch that advances PC)
    //   addr 1: JMP 0     (loop back)
    //
    // MOV Y, Y = 101_00000_010_00_010 = 0xA042
    let nop: u16 = 0xA042;
    let jmp_0: u16 = 0x0000;
    // Write to instruction memory (register writes pass through even
    // while PIO is held in reset — matching real hardware).
    emu.bus.write32(PIO0_BASE + 0x048, nop as u32, 0);
    emu.bus.write32(PIO0_BASE + 0x04C, jmp_0 as u32, 0);

    // SM0_EXECCTRL: wrap_top=1, wrap_bottom=0.
    emu.bus.write32(PIO0_BASE + 0x0CC, 1u32 << 12, 0);
    // SM0_CLKDIV: integer=1, frac=0 (one instruction per system clock).
    emu.bus.write32(PIO0_BASE + 0x0C8, 1u32 << 16, 0);

    // Enable SM0 via CTRL.
    emu.bus.write32(PIO0_BASE, 0x1, 0);

    // Read SM0_ADDR (PC) via the bus register path at PIO offset
    // 0x0C8 (SM0 base) + 0x0C (ADDR within SM) = 0x0D4.
    let pc_before = emu.bus.read32(PIO0_BASE + 0x0D4, 0);

    // Tick several cycles — PIO0 is held in reset, so SM0's PC must
    // not advance.
    for _ in 0..10 {
        emu.step().unwrap();
    }

    assert_eq!(
        emu.bus.read32(PIO0_BASE + 0x0D4, 0),
        pc_before,
        "SM0 PC must not advance while PIO0 is held in reset"
    );

    // De-assert PIO0's RESETS bit (clear bit 11).
    emu.bus.resets_state &= !(1u32 << crate::bus::RESET_PIO0);
    assert_eq!(
        emu.bus.resets_state & (1u32 << crate::bus::RESET_PIO0),
        0,
        "PIO0 should be released from reset"
    );

    // Now tick — SM0 should execute and its PC should advance.
    let pc_after_release = emu.bus.read32(PIO0_BASE + 0x0D4, 0);
    for _ in 0..5 {
        emu.step().unwrap();
    }

    assert_ne!(
        emu.bus.read32(PIO0_BASE + 0x0D4, 0),
        pc_after_release,
        "SM0 PC must advance after PIO0 is released from reset \
         (ran 5 cycles with a 2-instruction NOP loop)"
    );
}

/// Narrow (byte / halfword) writes to PIO TX FIFO registers must widen
/// to 32 bit and land as a single FIFO entry. On real RP2350 silicon
/// the AHB widens any narrow write to a 32-bit transaction before it
/// reaches the PIO peripheral; a byte write at byte-lane N produces a
/// `(val as u32) << (N*8)` FIFO entry. DMA byte-mode transfers from
/// SRAM to a PIO TX FIFO (the OneROM CH1 pattern) rely on this.
///
/// Regression: pre-0.2.4 the `Bus::write8` / `Bus::write16` PIO arms
/// were silent no-ops, so DMA byte transfers vanished even though
/// `TRANS_COUNT` decremented and `INTR` latched.
#[test]
fn pio_narrow_writes_widen_to_32_bit() {
    let mut bus = Bus::new();

    // PIO0 TXF1 (offset 0x014) — byte / halfword / word at byte lane 0.
    bus.write8(0x5020_0014, 0xCD, 0);
    assert_eq!(bus.pio[0].pop_tx(1), Some(0x0000_00CD));
    bus.write16(0x5020_0014, 0xABCD, 0);
    assert_eq!(bus.pio[0].pop_tx(1), Some(0x0000_ABCD));
    bus.write32(0x5020_0014, 0xCAFE_F00D, 0);
    assert_eq!(bus.pio[0].pop_tx(1), Some(0xCAFE_F00D));

    // PIO1 TXF2 (offset 0x018) — confirm the dispatch arm hits the
    // right block and SM, not just PIO0.
    bus.write8(0x5030_0018, 0x42, 0);
    assert_eq!(bus.pio[1].pop_tx(2), Some(0x0000_0042));
    bus.write16(0x5030_0018, 0xBEEF, 0);
    assert_eq!(bus.pio[1].pop_tx(2), Some(0x0000_BEEF));

    // PIO2 TXF1 (offset 0x014) — the exact register the OneROM CH1
    // DMA byte stream targets in the SDRR firmware.
    bus.write8(0x5040_0014, 0xCD, 0);
    assert_eq!(bus.pio[2].pop_tx(1), Some(0x0000_00CD));
    bus.write16(0x5040_0014, 0xABCD, 0);
    assert_eq!(bus.pio[2].pop_tx(1), Some(0x0000_ABCD));
    bus.write32(0x5040_0014, 0x1234_5678, 0);
    assert_eq!(bus.pio[2].pop_tx(1), Some(0x1234_5678));

    // Byte lane within the TXF word: a byte write to TXF0+1 goes to
    // byte lane 1, producing `0x0000_CD00`. (DMA byte-mode in practice
    // hits lane 0 because the FIFO is a permanent address; the
    // higher-lane behaviour is what the AHB widening spec mandates and
    // what differentiates "drop" from "wrong byte lane".)
    bus.write8(0x5040_0011, 0xCD, 0);
    assert_eq!(bus.pio[2].pop_tx(0), Some(0x0000_CD00));

    // Lane 2 byte write — `STRB Rn, [Rm, #2]` against TXF0 base+2
    // should land at bits [23:16]. Confirms the `(val as u32) <<
    // (byte_idx * 8)` shift covers the full byte-lane range, not just
    // lanes 0/1. (TXF0 = offset 0x010; the 0x012 address hits TXF0
    // word-aligned at 0x010, byte_idx = 2.)
    bus.write8(0x5040_0012, 0xCD, 0);
    assert_eq!(bus.pio[2].pop_tx(0), Some(0x00CD_0000));

    // Lane 3 byte write — top byte lane (TXF0, byte_idx = 3).
    bus.write8(0x5040_0013, 0xCD, 0);
    assert_eq!(bus.pio[2].pop_tx(0), Some(0xCD00_0000));

    // Halfword lane 1 — `STRH Rn, [Rm, #2]`. The half_idx*16 shift maps
    // base+2 to bits [31:16]. (TXF0 word-aligned, half_idx = 1.)
    bus.write16(0x5040_0012, 0xABCD, 0);
    assert_eq!(bus.pio[2].pop_tx(0), Some(0xABCD_0000));
}

/// Narrow writes to PIO RXF (offsets 0x020..=0x02C) MUST NOT pop the RX
/// FIFO. The standard subword-alias RMW used for the rest of the PIO
/// register space would issue `read32(reg_offset)` to fetch the lanes
/// outside the byte being written, but `read32` for RXF is destructive —
/// it pops the head of the SM's RX FIFO (`picoem_common::pio::mod.rs`
/// 0x020..=0x02C dispatch). On real silicon RXF is read-only on write,
/// so the AHB-widened narrow write is silently dropped at the peripheral.
///
/// Regression: pre-fix the new 0.2.4 PIO arm RMW'd RXF too, so a `STRB`
/// to the RXF aperture would spurious-drain RX from underneath the
/// firmware that was about to read it.
#[test]
fn pio_narrow_writes_to_rxf_dont_pop_fifo() {
    let mut bus = Bus::new();

    // Stage a known word at the head of PIO0 SM0 RX FIFO via the
    // test-only push_rx hook (`cfg(any(test, feature = "test-hooks"))`).
    bus.pio[0].push_rx(0, 0xCAFE_F00D);

    // A narrow write to PIO0 RXF0 (offset 0x020) — byte lane 0, plain
    // alias — MUST NOT pop the FIFO.
    bus.write8(0x5020_0020, 0xFF, 0);
    // Halfword lane 0, plain alias — also MUST NOT pop.
    bus.write16(0x5020_0020, 0xFFFF, 0);
    // Byte lane 1 (a different byte lane to prove the guard isn't only
    // checking lane 0).
    bus.write8(0x5020_0021, 0xAA, 0);
    // SET-alias byte at +0x2000 — proves the guard is unconditional on
    // alias too, not just plain (alias=0) writes.
    bus.write8(0x5020_2020, 0x55, 0);

    // The original word should still be at the head of RXF0; read32 pops it.
    assert_eq!(bus.read32(0x5020_0020, 0), 0xCAFE_F00D);
    // FIFO is now empty — read32 returns 0.
    assert_eq!(bus.read32(0x5020_0020, 0), 0);

    // Cross-block coverage: stage a word in PIO2 SM3 RXF (offset 0x02C)
    // and confirm the carve-out fires for the whole RXF range, not just
    // RXF0 of PIO0.
    bus.pio[2].push_rx(3, 0x1234_5678);
    bus.write8(0x5040_002C, 0xFF, 0);
    bus.write16(0x5040_002C, 0xFFFF, 0);
    bus.write8(0x5040_202C, 0x55, 0); // SET-alias byte
    assert_eq!(bus.read32(0x5040_002C, 0), 0x1234_5678);
    assert_eq!(bus.read32(0x5040_002C, 0), 0);
}

/// Narrow writes to FDEBUG (offset `0x008`) MUST NOT corrupt unchanged
/// byte lanes. FDEBUG is W1C with a *live* read — unlike UART/SPI/I2C
/// W1C registers (which read as 0 and so degenerate cleanly under
/// subword RMW), FDEBUG reads its current state. The standard
/// subword-alias RMW would read the live state, splice the changed
/// byte, and write it back — but the W1C semantics turn that into
/// "every set bit in the unchanged lanes is cleared". With the
/// TXF-only-widen policy, narrow writes outside `0x010..=0x01C` are
/// dropped at the bus arm, leaving FDEBUG untouched (matching silicon
/// AHB5 byte-strobe behaviour for a register that doesn't decode
/// narrow strobes).
#[test]
fn pio_narrow_writes_to_fdebug_dont_corrupt() {
    let mut bus = Bus::new();

    // Seed FDEBUG with a known non-zero value via the SET-alias word
    // write (alias 2, offset +0x2000). FDEBUG starts at 0; we can't
    // write 1's via the plain alias because that's W1C clear.
    bus.write32(0x5020_2008, 0x1234_5678, 0);
    let pre = bus.read32(0x5020_0008, 0);
    assert_eq!(
        pre, 0x1234_5678,
        "seed via SET alias must land at FDEBUG"
    );

    // Narrow byte-0 write to FDEBUG. With the broken RMW path this
    // would (a) read 0x1234_5678, (b) splice byte 0 to 0xFF, (c) write
    // 0x1234_56FF back, and the W1C arm would clear every 1-bit in
    // 0x1234_56FF — destroying the FDEBUG state. With TXF-only-widen
    // the write is dropped; FDEBUG stays put.
    bus.write8(0x5020_0008, 0xFF, 0);
    assert_eq!(
        bus.read32(0x5020_0008, 0),
        0x1234_5678,
        "narrow byte-0 write must not corrupt FDEBUG"
    );

    // Narrow halfword-1 write — same reasoning, different lane.
    bus.write16(0x5020_000A, 0xFFFF, 0);
    assert_eq!(
        bus.read32(0x5020_0008, 0),
        0x1234_5678,
        "narrow halfword-1 write must not corrupt FDEBUG"
    );
}

/// Narrow byte-1 write to CTRL (`0x000`) MUST NOT trigger SM_RESTART.
/// CTRL byte 1 holds the `SM_RESTART[3:0]` self-clearing bits at
/// `[7:4]`. The standard subword-alias RMW would read CTRL (which
/// reads back 0 in those bits because they're self-clearing — fine)
/// and splice the byte. But because the byte we're writing now has
/// `[7:4]` set, the write triggers a per-SM state reset (PC←0, X/Y/
/// ISR/OSR cleared). Real silicon decodes the AHB5 byte strobes and
/// only touches the byte that was strobed — but on RP2350 PIO CTRL
/// requires a 32-bit access; narrow writes are dropped at the
/// peripheral. The TXF-only-widen policy implements that drop.
#[test]
fn pio_narrow_writes_to_ctrl_dont_trigger_sm_restart() {
    let mut bus = Bus::new();

    // Force SM0 to a non-zero PC so SM_RESTART would be detectable as
    // PC←0. Easiest way: force-execute a `JMP 5` via a full-width
    // SMn_INSTR write. The force_execute path on JMP sets PC directly.
    bus.write32(0x5020_00D8, 0x0005, 0);
    assert_eq!(
        bus.pio[0].sm[0].pc(),
        5,
        "precondition: SM0 PC=5 via force-executed JMP"
    );

    // Issue a narrow byte-1 write to CTRL with byte = 0xF0 (SM_RESTART
    // bits [7:4] all set). The TXF-only-widen drop means this byte
    // never reaches the PIO peripheral — SM_RESTART must NOT fire and
    // PC must stay at 5.
    bus.write8(0x5020_0001, 0xF0, 0);
    assert_eq!(
        bus.pio[0].sm[0].pc(),
        5,
        "narrow byte-1 write must NOT trigger SM_RESTART (PC stays at 5)"
    );

    // Comparison case: a full-width 32-bit write of `0x0000_00F0`
    // DOES trigger SM_RESTART (PC←0). This proves byte-1 narrow drop
    // is genuinely a drop rather than a coincidence (e.g. that the
    // SM_RESTART path itself isn't broken).
    bus.write32(0x5020_0000, 0x0000_00F0, 0);
    assert_eq!(
        bus.pio[0].sm[0].pc(),
        0,
        "full-width SM_RESTART write must reset PC to 0 (control case)"
    );
}

/// Narrow byte-0 write to SMn_INSTR (`0x0D8`) MUST NOT force-execute.
/// SMn_INSTR is write-execute on every 32-bit write; the standard
/// subword-alias RMW path computes `(last_insn[31:8] | val)` and
/// passes the result to `force_execute`. Real silicon for a
/// narrow-write to SMn_INSTR either drops the write or executes
/// `(0x0000 | val)` — in either case it does NOT mix in the previous
/// `last_insn`. The TXF-only-widen drop matches the safer
/// silicon-friendly behaviour: no spurious force-execute at all.
#[test]
fn pio_narrow_writes_to_sm_instr_dont_force_execute() {
    let mut bus = Bus::new();

    // Seed `last_insn` with a recognisable opcode by full-width
    // writing `JMP 5` (0x0005). force_execute updates last_insn and
    // sets PC.
    bus.write32(0x5020_00D8, 0x0005, 0);
    assert_eq!(bus.pio[0].sm[0].pc(), 5, "precondition: PC=5 after JMP 5");
    assert_eq!(
        bus.read32(0x5020_00D8, 0),
        0x0005,
        "precondition: last_insn = 0x0005"
    );

    // Issue narrow byte-0 write of `0x07` to SMn_INSTR. Under the
    // broken RMW path this would force-execute `(0x0005 & !0xFF) |
    // 0x07 = 0x0007` (JMP 7), advancing PC to 7. Under TXF-only-widen
    // the write is dropped — PC stays at 5.
    bus.write8(0x5020_00D8, 0x07, 0);
    assert_eq!(
        bus.pio[0].sm[0].pc(),
        5,
        "narrow byte-0 write to SMn_INSTR must NOT force-execute"
    );
    assert_eq!(
        bus.read32(0x5020_00D8, 0),
        0x0005,
        "last_insn must be unchanged (no force-execute happened)"
    );
}
