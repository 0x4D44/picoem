//! RP2350-specific PIO tests.
//!
//! The PIO primitive (`PioBlock`, decode, state-machine) lives in
//! [`mdpicoem_common::pio`]. These tests exercise PIO *through the
//! RP2350 `Bus` and `Emulator`*, which is chip-specific — register
//! address layout (PIO0=0x5020_0000, PIO1=0x5030_0000, PIO2=0x5040_0000),
//! number of PIO blocks (3), and the GPIO merge path all sit on
//! `mdrp2350::Bus`.
//!
//! Tests that exercise only `PioBlock`/`StateMachine` internals stay
//! co-located with the primitive under `mdpicoem-common/src/pio/mod.rs`.

use crate::bus::Bus;
use crate::{Config, Emulator, EmulatorBuilder};

#[test]
fn test_bus_dispatch_pio0() {
    let mut bus = Bus::new();

    // Write SM0 PINCTRL via PIO0 base address
    bus.write32(0x5020_00DC, 0x1234_5678);

    // Read back
    let val = bus.read32(0x5020_00DC);
    assert_eq!(val, 0x1234_5678);
}

#[test]
fn test_bus_dispatch_pio1_pio2() {
    let mut bus = Bus::new();

    // PIO1: write SM1 CLKDIV (SM1 offset = 0x0E0)
    let clkdiv = (500u32 << 16) | (64u32 << 8);
    bus.write32(0x5030_00E0, clkdiv);
    assert_eq!(bus.read32(0x5030_00E0), clkdiv);

    // PIO2: write CTRL to enable SM3
    bus.write32(0x5040_0000, 0x8);
    assert_eq!(bus.read32(0x5040_0000), 0x8);
    assert!(bus.pio[2].sm[3].enabled());
}

#[test]
fn test_ctrl_alias_set_clr() {
    let mut bus = Bus::new();

    // SET alias: addr + 0x2000 (alias=2)
    // Enable SM0 via SET alias
    bus.write32(0x5020_2000, 0x1); // SET alias on CTRL
    assert!(bus.pio[0].sm[0].enabled());
    assert_eq!(bus.read32(0x5020_0000), 0x1);

    // Enable SM2 via SET alias (SM0 should remain enabled)
    bus.write32(0x5020_2000, 0x4);
    assert!(bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert_eq!(bus.read32(0x5020_0000), 0x5);

    // CLR alias: addr + 0x3000 (alias=3)
    // Disable SM0 via CLR alias
    bus.write32(0x5020_3000, 0x1);
    assert!(!bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert_eq!(bus.read32(0x5020_0000), 0x4);
}

#[test]
fn test_ctrl_alias_xor() {
    // XOR alias: addr + 0x1000 (alias=1). SM_ENABLE bits with 1 toggle
    // the corresponding SM; bits with 0 leave it untouched.
    let mut bus = Bus::new();

    // Start: SM0 and SM2 enabled via normal write.
    bus.write32(0x5020_0000, 0x5);
    assert!(bus.pio[0].sm[0].enabled());
    assert!(!bus.pio[0].sm[1].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert!(!bus.pio[0].sm[3].enabled());

    // XOR with 0x3: toggles SM0 (1->0) and SM1 (0->1); SM2/SM3 unchanged.
    bus.write32(0x5020_1000, 0x3);
    assert!(!bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[1].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert!(!bus.pio[0].sm[3].enabled());
    assert_eq!(bus.read32(0x5020_0000), 0x6);

    // XOR with 0x0: no-op.
    bus.write32(0x5020_1000, 0x0);
    assert_eq!(bus.read32(0x5020_0000), 0x6);

    // XOR with 0xF: toggles every SM.
    bus.write32(0x5020_1000, 0xF);
    assert!(bus.pio[0].sm[0].enabled());
    assert!(!bus.pio[0].sm[1].enabled());
    assert!(!bus.pio[0].sm[2].enabled());
    assert!(bus.pio[0].sm[3].enabled());
    assert_eq!(bus.read32(0x5020_0000), 0x9);
}

#[test]
fn test_gpio_in_moved_to_bus() {
    let mut bus = Bus::new();
    bus.gpio_in = 0xFF;

    // Read SIO GPIO_IN via bus at 0xD000_0004
    let val = bus.read32(0xD000_0004);
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
    assert_eq!(emu.bus.gpio_in & (1 << 5), 0, "PIO overrides SIO on pin 5");
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
    assert_ne!(emu.bus.gpio_in & (1 << 5), 0, "PIO pin 5 appears");
    assert_ne!(emu.bus.gpio_in & (1 << 10), 0, "SIO pin 10 appears");
}

// ====================================================================
// Stage D: Waveform integration tests
// ====================================================================
//
// These tests verify that PIO programs running through the full
// Emulator produce correct GPIO waveforms with cycle-accurate timing.

const PIO0_BASE: u32 = 0x5020_0000;

/// Write a PIO0 register through the emulator bus.
fn pio_write(emu: &mut Emulator, offset: u32, val: u32) {
    emu.bus.write32(PIO0_BASE + offset, val);
}

/// Create an emulator configured for PIO integration tests.
///
/// Uses `step_quantum=1` so each `emu.step()` advances by exactly
/// one cycle — these tests read PIO pin state on a per-cycle basis,
/// which the quantum execution model would otherwise smear across up
/// to `DEFAULT_STEP_QUANTUM` cycles.
fn pio_test_emulator() -> Emulator {
    EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
}

/// Load a PIO program into instruction memory via bus writes.
fn pio_load_program(emu: &mut Emulator, program: &[u16]) {
    for (i, &insn) in program.iter().enumerate() {
        pio_write(emu, 0x048 + (i as u32) * 4, insn as u32);
    }
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
    let jmp_0: u16 = 0x0000;      // JMP 0
    pio_load_program(&mut emu, &[set_pins_1, set_pins_0, jmp_0]);

    // SM0_PINCTRL: set_base=25, set_count=1
    // set_count at bits[28:26], set_base at bits[9:5]
    let pinctrl = (1u32 << 26) | (25u32 << 5);
    pio_write(&mut emu, 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=2, wrap_bottom=0
    let execctrl = (2u32 << 12) | (0u32 << 7);
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
        true, false, false,  // pattern 1
        true, false, false,  // pattern 2
        true, false, false,  // pattern 3
        true, false, false,  // pattern 4
    ];

    let mut actual = Vec::new();
    for _ in 0..12 {
        emu.step();
        actual.push(emu.gpio_read(25));
    }

    assert_eq!(actual, expected,
        "GPIO 25 waveform mismatch over 12 cycles\n  actual:   {:?}\n  expected: {:?}",
        actual, expected);
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
    pio_load_program(&mut emu, &[pull_block, set_x_7, out_pins_1, jmp_xdec_2, jmp_0]);

    // SM0_PINCTRL: out_base=0, out_count=1, set_count=1, set_base=0
    let pinctrl = (1u32 << 26) | (1u32 << 20);
    pio_write(&mut emu, 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=4, wrap_bottom=0
    let execctrl = (4u32 << 12) | (0u32 << 7);
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
        emu.step();
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
    assert_eq!(received_bits, expected_bits,
        "UART TX 0x55 data bits mismatch (LSB first)\n  received: {:?}\n  expected: {:?}",
        received_bits, expected_bits);

    // Reconstruct the byte from received bits
    let mut byte: u8 = 0;
    for (i, &bit) in received_bits.iter().enumerate() {
        if bit {
            byte |= 1 << i;
        }
    }
    assert_eq!(byte, 0x55, "reconstructed byte should be 0x55, got {:#04x}", byte);
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
    let set_x7_s0: u16     = 0xE027; // 111_0_0000_001_00111
    let out_pins1_s1: u16  = 0x7001; // 011_1_0000_000_00001
    let jmp_xdec2_s0: u16  = 0x0042; // 000_0_0000_010_00010
    let jmp_0_s0: u16      = 0x0000; // 000_0_0000_000_00000
    pio_load_program(&mut emu, &[
        pull_block_s0, set_x7_s0, out_pins1_s1, jmp_xdec2_s0, jmp_0_s0,
    ]);

    // SM0_PINCTRL:
    //   out_base=0 (MOSI on pin 0), out_count=1
    //   set_base=0, set_count=2 (covers both MOSI at pin 0 and CLK at pin 1)
    //   sideset_base=1 (CLK on pin 1), sideset_count=1
    let pinctrl = (1u32 << 29)   // sideset_count=1
                | (2u32 << 26)   // set_count=2
                | (1u32 << 20)   // out_count=1
                | (1u32 << 10)   // sideset_base=1
                | (0u32 << 5)    // set_base=0
                | (0u32);        // out_base=0
    pio_write(&mut emu, 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=4, wrap_bottom=0, SIDE_EN=0
    let execctrl = (4u32 << 12) | (0u32 << 7);
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
        emu.step();
        clk_trace.push(emu.gpio_read(1));
        mosi_trace.push(emu.gpio_read(0));
    }

    // CLK should be HIGH only on OUT steps (side 1): steps 3,5,7,...,17
    // (0-indexed: 2,4,6,8,10,12,14,16)
    let expected_clk: Vec<bool> = (0..total_steps).map(|i| {
        // OUT steps at 0-indexed: 2, 4, 6, 8, 10, 12, 14, 16
        i >= 2 && i <= 16 && i % 2 == 0
    }).collect();

    assert_eq!(clk_trace, expected_clk,
        "SPI CLK waveform mismatch\n  actual:   {:?}\n  expected: {:?}",
        clk_trace, expected_clk);

    // MOSI data bits (sampled on CLK rising edges = OUT steps)
    let out_steps: Vec<usize> = vec![2, 4, 6, 8, 10, 12, 14, 16];
    let mut mosi_bits: Vec<bool> = Vec::new();
    for &i in &out_steps {
        mosi_bits.push(mosi_trace[i]);
    }

    // Expected: 0x55 LSB-first = 1,0,1,0,1,0,1,0
    let expected_mosi: Vec<bool> = vec![true, false, true, false, true, false, true, false];
    assert_eq!(mosi_bits, expected_mosi,
        "SPI MOSI data mismatch (LSB first)\n  actual:   {:?}\n  expected: {:?}",
        mosi_bits, expected_mosi);

    // Verify CLK and MOSI timing relationship: MOSI transitions
    // should be captured on the CLK rising edge (OUT instruction).
    // On CLK falling edges (JMP instruction), MOSI holds its value.
    for &i in &out_steps {
        assert!(clk_trace[i], "CLK must be HIGH when MOSI data bit is presented (step {})", i);
    }
}
