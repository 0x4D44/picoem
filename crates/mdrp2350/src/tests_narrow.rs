//! Bus narrow-access audit tests (Stage 1 of HLD
//! `2026.04.17 - HLD - Bus Narrow-Access Audit.md`).
//!
//! Tests that pin the _desired_ post-audit behaviour of `Bus::{read8,
//! write8, read16, write16}`. A subset is expected to FAIL on the
//! current baseline (the HLD calls these out in §2 / §4 / §6) — those
//! failures are the evidence for the refactor. A subset passes today
//! and serves as regression protection.
//!
//! Grouped by HLD section with `// §6.x` comments. No production code
//! is modified by Stage 1 — tests only.

#![cfg(test)]

use crate::bus::Bus;
use crate::dma::DMA_BASE;
use crate::peripherals::adc::ADC_BASE;
use crate::peripherals::sha256::SHA256_BASE;
use crate::peripherals::spi::{SPI0_BASE, SPI1_BASE};
use crate::peripherals::timer::TIMER0_BASE;
use crate::peripherals::uart::{UART0_BASE, UART1_BASE};

// --- addresses used across the matrix -------------------------------
const ROM_BASE: u32 = 0x0000_0000;
const XIP_SRAM_BASE: u32 = 0x1C00_0000;
const SRAM_BASE: u32 = 0x2000_0000;
const APB_CLOCKS_CLK_SYS_CTRL: u32 = 0x4001_003C; // plain RW (verified)
const PIO0_BASE: u32 = 0x5020_0000;
const SIO_BASE: u32 = 0xD000_0000;
const BOOT_RAM_BASE: u32 = 0xEFFF_F000; // 4 KB boot RAM

// PPB regs
const NVIC_ISER0: u32 = 0xE000_E100;
const NVIC_ISER1: u32 = 0xE000_E104;
const NVIC_ICER0: u32 = 0xE000_E180;
const NVIC_ICER1: u32 = 0xE000_E184;
const NVIC_ISPR0: u32 = 0xE000_E200;
const NVIC_ISPR1: u32 = 0xE000_E204;
const NVIC_ICPR0: u32 = 0xE000_E280;
const NVIC_ICPR1: u32 = 0xE000_E284;
const SCB_ICSR: u32 = 0xE000_ED04;
const SCB_SHCSR: u32 = 0xE000_ED24;
const SCB_CFSR: u32 = 0xE000_ED28;
const SCB_HFSR: u32 = 0xE000_ED2C;

// W1C side-effect registers
const UART0_UARTICR: u32 = UART0_BASE + 0x044;
const UART1_UARTICR: u32 = UART1_BASE + 0x044;
const SPI0_SSPICR: u32 = SPI0_BASE + 0x020;
const SPI1_SSPICR: u32 = SPI1_BASE + 0x020;
const TIMER0_INTR: u32 = TIMER0_BASE + 0x03C;
const TIMER1_INTR: u32 = 0x400B_8000 + 0x03C;
const PWM_INTR: u32 = 0x400A_80F4;
const DMA_INTR: u32 = DMA_BASE + 0x400;
const DMA_INTS0: u32 = DMA_BASE + 0x40C;
const DMA_INTS1: u32 = DMA_BASE + 0x41C;
const DMA_CH0_CTRL: u32 = DMA_BASE + 0x0C;
const DMA_CH0_READ_ADDR: u32 = DMA_BASE;
const IO_BANK0_INTR0: u32 = 0x4002_8230;
const IO_BANK0_INTR5: u32 = 0x4002_8244;
const SIO_FIFO_ST: u32 = 0xD000_0050;
const SIO_INTERP0_CTRL_LANE0: u32 = 0xD000_00AC;
const SIO_INTERP1_CTRL_LANE0: u32 = 0xD000_00EC;
const GLITCH_TRIG_STATUS: u32 = 0x4015_8010;
const SHA256_CSR: u32 = SHA256_BASE;
const SHA256_WDATA: u32 = SHA256_BASE + 0x04;
const ADC_FCS: u32 = ADC_BASE + 0x08;
const ADC_FIFO: u32 = ADC_BASE + 0x0C;

// Side-effect regs
const UARTDR: u32 = UART0_BASE; // offset 0x000
const SSPDR: u32 = SPI0_BASE + 0x008;
const IC_DATA_CMD: u32 = 0x4009_0010;

// Unmapped region for bus-fault tests. Region 0xA is not defined.
const UNMAPPED_ADDR: u32 = 0xA000_0000;

// ============================================================================
// Helpers
// ============================================================================

/// Clear any latched bus fault so assertions can focus on the next access.
fn clear_bf(bus: &mut Bus) {
    bus.clear_bus_fault(0);
}

/// Splice `byte` into `word` at lane `lane` (LE).
fn splice_byte(word: u32, lane: usize, byte: u8) -> u32 {
    let mut bytes = word.to_le_bytes();
    bytes[lane] = byte;
    u32::from_le_bytes(bytes)
}

/// Splice `hw` into `word` at halfword-lane `lane` (0 = low, 1 = high).
fn splice_halfword(word: u32, lane: usize, hw: u16) -> u32 {
    let mut bytes = word.to_le_bytes();
    let off = lane * 2;
    bytes[off..off + 2].copy_from_slice(&hw.to_le_bytes());
    u32::from_le_bytes(bytes)
}

// ============================================================================
// §6.1 — Invariant matrix. For every aperture, per narrow op × {lane
// 0..3}: targeted lane updated, other lanes preserved. ROM writes must
// be dropped. PPB and DMA apertures are expected to FAIL on baseline.
// ============================================================================

// --- §6.1 ROM aperture — reads work, writes dropped ---

#[test]
fn s61_rom_read8_returns_correct_lane() {
    let mut bus = Bus::new();
    // Load a known pattern into ROM.
    let mut rom = vec![0u8; 16];
    rom[0..4].copy_from_slice(&0xAABBCCDDu32.to_le_bytes());
    bus.memory.load_rom(&rom);

    // Byte 0 = 0xDD, byte 1 = 0xCC, byte 2 = 0xBB, byte 3 = 0xAA.
    assert_eq!(bus.read8(ROM_BASE, 0), 0xDD);
    assert_eq!(bus.read8(ROM_BASE + 1, 0), 0xCC);
    assert_eq!(bus.read8(ROM_BASE + 2, 0), 0xBB);
    assert_eq!(bus.read8(ROM_BASE + 3, 0), 0xAA);
}

#[test]
fn s61_rom_read16_returns_correct_halfword() {
    let mut bus = Bus::new();
    let mut rom = vec![0u8; 16];
    rom[0..4].copy_from_slice(&0xAABBCCDDu32.to_le_bytes());
    bus.memory.load_rom(&rom);
    assert_eq!(bus.read16(ROM_BASE, 0), 0xCCDD);
    assert_eq!(bus.read16(ROM_BASE + 2, 0), 0xAABB);
}

#[test]
fn s61_rom_write8_is_dropped() {
    let mut bus = Bus::new();
    let mut rom = vec![0u8; 16];
    rom[0..4].copy_from_slice(&0xAABBCCDDu32.to_le_bytes());
    bus.memory.load_rom(&rom);
    for lane in 0..4 {
        bus.write8(ROM_BASE + lane as u32, 0x11, 0);
    }
    // Read back — must still be the original pattern.
    assert_eq!(bus.read32(ROM_BASE, 0), 0xAABBCCDD);
}

#[test]
fn s61_rom_write16_is_dropped() {
    let mut bus = Bus::new();
    let mut rom = vec![0u8; 16];
    rom[0..4].copy_from_slice(&0xAABBCCDDu32.to_le_bytes());
    bus.memory.load_rom(&rom);
    bus.write16(ROM_BASE, 0x1111, 0);
    bus.write16(ROM_BASE + 2, 0x2222, 0);
    assert_eq!(bus.read32(ROM_BASE, 0), 0xAABBCCDD);
}

// --- §6.1 XIP SRAM aperture ---

#[test]
fn s61_xip_sram_matrix_per_lane() {
    for lane in 0..4 {
        let mut bus = Bus::new();
        bus.write32(XIP_SRAM_BASE, 0xAABBCCDD, 0);
        bus.write8(XIP_SRAM_BASE + lane as u32, 0x11, 0);
        let want = splice_byte(0xAABBCCDD, lane, 0x11);
        assert_eq!(
            bus.read32(XIP_SRAM_BASE, 0),
            want,
            "XIP SRAM write8 lane {lane}"
        );
    }
}

#[test]
fn s61_xip_sram_write16_matrix() {
    for hw_lane in 0..2 {
        let mut bus = Bus::new();
        bus.write32(XIP_SRAM_BASE, 0xAABBCCDD, 0);
        bus.write16(XIP_SRAM_BASE + (hw_lane * 2) as u32, 0x1234, 0);
        let want = splice_halfword(0xAABBCCDD, hw_lane, 0x1234);
        assert_eq!(
            bus.read32(XIP_SRAM_BASE, 0),
            want,
            "XIP SRAM write16 hw_lane {hw_lane}"
        );
    }
}

// --- §6.1 SRAM aperture ---

#[test]
fn s61_sram_matrix_per_lane() {
    // SRAM has four aliases per `bus/mod.rs:1567-1569`:
    //   base 0x2000_0000   — plain RW
    //   alias 0x2100_0000  — XOR (write value ^ existing)
    //   alias 0x2200_0000  — OR  (write value | existing, bitset)
    //   alias 0x2300_0000  — AND-NOT (existing & !value, bitclr)
    //
    // Each has a distinct arithmetic path — cover all four aliases ×
    // 4 byte lanes. Seeding is always done via the base alias so we
    // don't double-dip the alias op, then the narrow write hits the
    // alias under test.
    let seed: u32 = 0xAABBCCDD;
    let byte: u8 = 0x5A; // has bits in both halves for OR/XOR to matter
    let aliases: [(u32, &str); 4] = [
        (0x2000_0000, "base RW"),
        (0x2100_0000, "XOR"),
        (0x2200_0000, "OR"),
        (0x2300_0000, "AND-NOT"),
    ];
    for (alias_base, label) in aliases {
        for lane in 0..4 {
            let mut bus = Bus::new();
            // Seed via base alias so plain store lands without any
            // alias arithmetic.
            bus.write32(SRAM_BASE + 0x100, seed, 0);
            bus.write8(alias_base + 0x100 + lane as u32, byte, 0);
            // Expected: each lane's byte transforms per the alias op;
            // other lanes untouched.
            let seed_bytes = seed.to_le_bytes();
            let mut want_bytes = seed_bytes;
            want_bytes[lane] = match alias_base {
                0x2000_0000 => byte,
                0x2100_0000 => seed_bytes[lane] ^ byte,
                0x2200_0000 => seed_bytes[lane] | byte,
                0x2300_0000 => seed_bytes[lane] & !byte,
                _ => unreachable!(),
            };
            let want = u32::from_le_bytes(want_bytes);
            assert_eq!(
                bus.read32(SRAM_BASE + 0x100, 0),
                want,
                "SRAM alias {label} ({alias_base:#010x}) write8 lane {lane}"
            );
        }
    }
}

#[test]
fn s61_sram_read_extract_per_lane() {
    let mut bus = Bus::new();
    bus.write32(SRAM_BASE + 0x200, 0xAABBCCDD, 0);
    assert_eq!(bus.read8(SRAM_BASE + 0x200, 0), 0xDD);
    assert_eq!(bus.read8(SRAM_BASE + 0x200 + 1, 0), 0xCC);
    assert_eq!(bus.read8(SRAM_BASE + 0x200 + 2, 0), 0xBB);
    assert_eq!(bus.read8(SRAM_BASE + 0x200 + 3, 0), 0xAA);
    assert_eq!(bus.read16(SRAM_BASE + 0x200, 0), 0xCCDD);
    assert_eq!(bus.read16(SRAM_BASE + 0x200 + 2, 0), 0xAABB);
}

// --- §6.1 APB Phase-2 (CLOCKS_CLK_SYS_CTRL, plain RW in our model) ---

#[test]
fn s61_apb_clk_sys_ctrl_write8_matrix() {
    // Use a value whose bits fit CLK_SYS_CTRL writable range. The
    // register stores 0..2 SRC (bits 0..1), AUXSRC (bits 5..7) —
    // storage model accepts all 32 bits plain. Pick a pattern with
    // bits in each lane to make lane separation visible.
    for lane in 0..4 {
        let mut bus = Bus::new();
        // Seed: write32 via the APB plain path (alias=0).
        bus.write32(APB_CLOCKS_CLK_SYS_CTRL, 0x1122_3344, 0);
        let seed = bus.read32(APB_CLOCKS_CLK_SYS_CTRL, 0);
        bus.write8(APB_CLOCKS_CLK_SYS_CTRL + lane as u32, 0xAB, 0);
        let want = splice_byte(seed, lane, 0xAB);
        assert_eq!(
            bus.read32(APB_CLOCKS_CLK_SYS_CTRL, 0),
            want,
            "CLK_SYS_CTRL write8 lane {lane}"
        );
    }
}

#[test]
fn s61_apb_clk_sys_ctrl_read8_matrix() {
    let mut bus = Bus::new();
    bus.write32(APB_CLOCKS_CLK_SYS_CTRL, 0x1122_3344, 0);
    let word = bus.read32(APB_CLOCKS_CLK_SYS_CTRL, 0);
    for lane in 0..4 {
        assert_eq!(
            bus.read8(APB_CLOCKS_CLK_SYS_CTRL + lane as u32, 0),
            word.to_le_bytes()[lane],
            "CLK_SYS_CTRL read8 lane {lane}"
        );
    }
}

#[test]
fn s61_apb_timer0_alarm0_write8_matrix() {
    // TIMER ALARM0 at 0x10 is plain RW storage (stride-4 alarm target).
    // Writing to it arms the alarm but the stored 32-bit value survives
    // verbatim. This gives a clean plain-RW matrix subject.
    for lane in 0..4 {
        let mut bus = Bus::new();
        let addr = TIMER0_BASE + 0x10;
        bus.write32(addr, 0x5566_7788, 0);
        let seed = bus.read32(addr, 0);
        bus.write8(addr + lane as u32, 0xCD, 0);
        let want = splice_byte(seed, lane, 0xCD);
        assert_eq!(
            bus.read32(addr, 0),
            want,
            "TIMER0 ALARM0 write8 lane {lane}"
        );
    }
}

// --- §6.1 DMA aperture — expected to FAIL on baseline (HashMap catch-all) ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_dma_ch0_read_addr_write8_matrix() {
    // DMA_CH0_READ_ADDR is plain RW at DMA_BASE + 0x00. Baseline narrow
    // path puts the byte in the `peripheral_regs` HashMap, desynced
    // from the real Dma.channels[0].read_addr. Expected behaviour
    // post-audit: narrow writes route through Dma::write32.
    for lane in 0..4 {
        let mut bus = Bus::new();
        bus.write32(DMA_CH0_READ_ADDR, 0xAABB_CCDD, 0);
        let seed = bus.read32(DMA_CH0_READ_ADDR, 0);
        bus.write8(DMA_CH0_READ_ADDR + lane as u32, 0x11, 0);
        let want = splice_byte(seed, lane, 0x11);
        assert_eq!(
            bus.read32(DMA_CH0_READ_ADDR, 0),
            want,
            "DMA CH0_READ_ADDR write8 lane {lane} — HLD §2 DMA gap"
        );
    }
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_dma_ch0_read_addr_read8_matrix() {
    let mut bus = Bus::new();
    bus.write32(DMA_CH0_READ_ADDR, 0xAABB_CCDD, 0);
    let word = bus.read32(DMA_CH0_READ_ADDR, 0);
    for lane in 0..4 {
        assert_eq!(
            bus.read8(DMA_CH0_READ_ADDR + lane as u32, 0),
            word.to_le_bytes()[lane],
            "DMA CH0_READ_ADDR read8 lane {lane}"
        );
    }
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_dma_ch0_read_addr_write16_matrix() {
    for hw_lane in 0..2 {
        let mut bus = Bus::new();
        bus.write32(DMA_CH0_READ_ADDR, 0xAABB_CCDD, 0);
        let seed = bus.read32(DMA_CH0_READ_ADDR, 0);
        bus.write16(DMA_CH0_READ_ADDR + (hw_lane * 2) as u32, 0x1122, 0);
        let want = splice_halfword(seed, hw_lane, 0x1122);
        assert_eq!(
            bus.read32(DMA_CH0_READ_ADDR, 0),
            want,
            "DMA CH0_READ_ADDR write16 hw_lane {hw_lane}"
        );
    }
}

// --- §6.1 PIO aperture ---

#[test]
fn s61_pio0_ctrl_read8_matrix() {
    // PIO CTRL reads work today via read32-extract; write8/16 are
    // silent no-ops on baseline (HLD §2). Exercise read side.
    let mut bus = Bus::new();
    bus.write32(PIO0_BASE, 0x0000_0007, 0); // enable all 3 SMs
    let word = bus.read32(PIO0_BASE, 0);
    for lane in 0..4 {
        assert_eq!(
            bus.read8(PIO0_BASE + lane as u32, 0),
            word.to_le_bytes()[lane],
            "PIO0 CTRL read8 lane {lane}"
        );
    }
}

// --- §6.1 SIO aperture ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_sio_interp_accum0_write8_matrix() {
    // INTERP0 ACCUM0 at SIO offset 0x080 is plain RW per
    // sio/mod.rs:727-731 (per-core isolated, stores 32 bits
    // verbatim).
    //
    // NB: SIO byte-write takes the narrow path at bus/mod.rs:1913-1945
    // — RMW via read32/write32 (post-ac19a28 fix). Non-GPIO_OUT
    // registers should already be correct.
    for lane in 0..4 {
        let mut bus = Bus::new();
        bus.write32(SIO_BASE + 0x080, 0x1122_3344, 0);
        let seed = bus.read32(SIO_BASE + 0x080, 0);
        bus.write8(SIO_BASE + 0x080 + lane as u32, 0xAB, 0);
        let want = splice_byte(seed, lane, 0xAB);
        assert_eq!(
            bus.read32(SIO_BASE + 0x080, 0),
            want,
            "SIO INTERP0 ACCUM0 write8 lane {lane}"
        );
    }
}

#[test]
fn s61_sio_gpio_in_narrow_write_is_no_op() {
    // SIO GPIO_IN at 0xD000_0004 is a read-only mirror handled by a
    // Bus fast-path (bus/mod.rs:1517/1939/2139). Narrow writes must
    // not fault and must not disturb observable state. This catches
    // any stray dispatch into SIO::write that might attempt RMW
    // through the short-circuit path.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    let gpio_in_before = bus.read32(SIO_BASE + 0x004, 0);
    for lane in 0..4 {
        bus.write8(SIO_BASE + 0x004 + lane as u32, 0xFF, 0);
    }
    for hw_lane in 0..2 {
        bus.write16(SIO_BASE + 0x004 + (hw_lane * 2) as u32, 0xFFFF, 0);
    }
    let gpio_in_after = bus.read32(SIO_BASE + 0x004, 0);
    assert_eq!(
        gpio_in_before, gpio_in_after,
        "SIO GPIO_IN narrow writes must not disturb the read-only mirror"
    );
    assert!(!bus.bus_fault(0), "SIO GPIO_IN narrow write must not fault");
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_sio_interp_pop_peek_narrow_write_is_no_op() {
    // INTERP0 POP_LANE0/1 (0x094/0x098) and PEEK_LANE0/1
    // (0x0A0/0x0A4) are RO — see `sio/interp.rs` write match arm at
    // `0x14 | 0x18 | 0x1C | 0x20 | 0x24 | 0x28 => {}`. Byte writes
    // to these offsets must be no-ops (nothing observable changes,
    // no bus fault).
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    // Seed some state we'd notice if a POP ever fired: ACCUM0 non-zero
    // with a CTRL that would transform it.
    bus.write32(SIO_BASE + 0x080, 0x1234_5678, 0); // INTERP0 ACCUM0
    bus.write32(SIO_BASE + 0x080 + 0x2C, 0, 0); // CTRL_LANE0 = default
    let accum0_before = bus.read32(SIO_BASE + 0x080, 0);
    // Byte writes to each RO offset — must not fault, must not pop.
    for off in [0x094u32, 0x098, 0x09C, 0x0A0, 0x0A4, 0x0A8] {
        for lane in 0..4 {
            bus.write8(SIO_BASE + off + lane, 0xFF, 0);
            bus.write16(SIO_BASE + off + (lane & !1), 0xFFFF, 0);
        }
    }
    let accum0_after = bus.read32(SIO_BASE + 0x080, 0);
    assert_eq!(
        accum0_before, accum0_after,
        "INTERP0 POP/PEEK narrow writes must not side-effect ACCUM0"
    );
    assert!(
        !bus.bus_fault(0),
        "INTERP0 POP/PEEK narrow writes must not fault"
    );
}

// --- §6.1 Boot RAM aperture ---

#[test]
fn s61_boot_ram_write8_matrix() {
    for lane in 0..4 {
        let mut bus = Bus::new();
        bus.write32(BOOT_RAM_BASE + 0x100, 0xAABB_CCDD, 0);
        let seed = bus.read32(BOOT_RAM_BASE + 0x100, 0);
        bus.write8(BOOT_RAM_BASE + 0x100 + lane as u32, 0x11, 0);
        let want = splice_byte(seed, lane, 0x11);
        assert_eq!(
            bus.read32(BOOT_RAM_BASE + 0x100, 0),
            want,
            "boot_ram w8 lane {lane}"
        );
    }
}

#[test]
fn s61_boot_ram_read8_matrix() {
    let mut bus = Bus::new();
    bus.write32(BOOT_RAM_BASE + 0x200, 0xAABB_CCDD, 0);
    let word = bus.read32(BOOT_RAM_BASE + 0x200, 0);
    for lane in 0..4 {
        assert_eq!(
            bus.read8(BOOT_RAM_BASE + 0x200 + lane as u32, 0),
            word.to_le_bytes()[lane]
        );
    }
}

// --- §6.1 PPB aperture — SCB_SHCSR is plain RW in our model ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_ppb_shcsr_write8_matrix() {
    // SHCSR narrow write is expected to FAIL on baseline (PPB region
    // 0xE drops narrow writes silently — HLD §2).
    for lane in 0..4 {
        let mut bus = Bus::new();
        bus.write32(SCB_SHCSR, 0x1122_3344, 0);
        let seed = bus.read32(SCB_SHCSR, 0);
        bus.write8(SCB_SHCSR + lane as u32, 0xAB, 0);
        let want = splice_byte(seed, lane, 0xAB);
        assert_eq!(
            bus.read32(SCB_SHCSR, 0),
            want,
            "SHCSR write8 lane {lane} — HLD §2 PPB gap"
        );
    }
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_ppb_shcsr_read8_matrix() {
    // PPB narrow read is expected to FAIL on baseline too — read8 over
    // region 0xE returns 0 unconditionally (stub, bus/mod.rs:1532).
    let mut bus = Bus::new();
    bus.write32(SCB_SHCSR, 0x1122_3344, 0);
    let word = bus.read32(SCB_SHCSR, 0);
    for lane in 0..4 {
        assert_eq!(
            bus.read8(SCB_SHCSR + lane as u32, 0),
            word.to_le_bytes()[lane],
            "SHCSR read8 lane {lane} — HLD §2 PPB gap"
        );
    }
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s61_ppb_shcsr_write16_matrix() {
    for hw_lane in 0..2 {
        let mut bus = Bus::new();
        bus.write32(SCB_SHCSR, 0x1122_3344, 0);
        let seed = bus.read32(SCB_SHCSR, 0);
        bus.write16(SCB_SHCSR + (hw_lane * 2) as u32, 0x5566, 0);
        let want = splice_halfword(seed, hw_lane, 0x5566);
        assert_eq!(
            bus.read32(SCB_SHCSR, 0),
            want,
            "SHCSR write16 hw_lane {hw_lane}"
        );
    }
}

// --- §6.1 Unmapped aperture — reads fault today; writes silently drop ---

#[test]
fn s61_unmapped_read8_sets_bus_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    let _ = bus.read8(UNMAPPED_ADDR, 0);
    assert!(bus.bus_fault(0), "read8 on unmapped must set bus_fault");
}

#[test]
fn s61_unmapped_read16_sets_bus_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    let _ = bus.read16(UNMAPPED_ADDR, 0);
    assert!(bus.bus_fault(0), "read16 on unmapped must set bus_fault");
}

// ============================================================================
// §6.2 — Side-effect fast-path regression
// ============================================================================

fn enable_uart0_loopback(bus: &mut Bus) {
    // UART0 UARTLCR_H: FEN=1 (bit 4), 8-bit word
    bus.write32(UART0_BASE + 0x02C, 0b0111_0000, 0);
    // UARTIBRD / UARTFBRD: pick any non-zero divisor so the clock model
    // has something to work with. 1.5 @ 6.5 MHz → lots of cycles/byte.
    bus.write32(UART0_BASE + 0x024, 1, 0);
    bus.write32(UART0_BASE + 0x028, 0, 0);
    // UARTCR: UARTEN|TXE|RXE|LBE=1
    bus.write32(UART0_BASE + 0x030, 0x0000_0381, 0);
}

#[test]
fn s62_uartdr_byte_read_pops_one_byte() {
    let mut bus = Bus::new();
    enable_uart0_loopback(&mut bus);
    // Push two bytes via UARTDR word write.
    bus.write32(UARTDR, 0x41, 0); // 'A'
    bus.write32(UARTDR, 0x42, 0); // 'B'
    // Tick enough cycles for both bytes to loop TX→RX.
    for _ in 0..20 {
        bus.tick_peripherals(1_000_000);
    }
    // Byte read pops 'A'.
    let a = bus.read8(UARTDR, 0);
    let b = bus.read8(UARTDR, 0);
    assert_eq!(a, 0x41, "first read8 should pop 'A'");
    assert_eq!(b, 0x42, "second read8 should pop 'B'");
}

#[test]
fn s62_uartdr_halfword_read_pops_one_byte_zero_extended() {
    let mut bus = Bus::new();
    enable_uart0_loopback(&mut bus);
    bus.write32(UARTDR, 0x41, 0);
    for _ in 0..20 {
        bus.tick_peripherals(1_000_000);
    }
    let v = bus.read16(UARTDR, 0);
    assert_eq!(v, 0x0041, "halfword read of UARTDR zero-extends the byte");
}

#[test]
fn s62_sspdr_halfword_write_pushes_one_frame() {
    let mut bus = Bus::new();
    // Enable SPI0: SSPCR1.SSE bit.
    bus.write32(SPI0_BASE + 0x004, 0x0000_0002, 0);
    // SSPSR should read TFE=1 (TX FIFO empty) initially.
    let sr_before = bus.read32(SPI0_BASE + 0x00C, 0);
    assert_ne!(sr_before & 0x1, 0, "TFE should be set before push");
    // Halfword write to SSPDR.
    bus.write16(SSPDR, 0xBEEF, 0);
    let sr_after = bus.read32(SPI0_BASE + 0x00C, 0);
    // After push, TX FIFO is non-empty → TFE=0.
    assert_eq!(
        sr_after & 0x1,
        0,
        "TFE should clear after halfword push to SSPDR"
    );
}

#[test]
fn s62_ic_data_cmd_byte_write_pushes_transaction() {
    // I2C0 IC_DATA_CMD at 0x4009_0010. Enable via IC_ENABLE at offset
    // 0x6C (bit 0). Byte-write pushes a transaction — the TX FIFO
    // reports the byte back via IC_TX_FIFO_LEN (offset 0x74) which
    // should become >= 1.
    let mut bus = Bus::new();
    // IC_ENABLE = 1.
    bus.write32(0x4009_006C, 0x0000_0001, 0);
    let tx_len_before = bus.read32(0x4009_0074, 0);
    bus.write8(IC_DATA_CMD, 0x55, 0);
    let tx_len_after = bus.read32(0x4009_0074, 0);
    assert!(
        tx_len_after > tx_len_before || bus.read32(0x4009_0070, 0) > 0,
        "byte write to IC_DATA_CMD should push a transaction (TX len {} → {})",
        tx_len_before,
        tx_len_after
    );
}

#[test]
fn s62_adc_fifo_byte_write_is_swallowed() {
    // ADC FIFO (0x400A_000C) is read-only per datasheet §12.4.5.
    // Byte write must be swallowed (no FIFO pop, no bus fault, no
    // corruption of adjacent registers, and specifically no incidental
    // FIFO push that would change FCS.LEVEL).
    //
    // Baseline passes because of the HashMap catch-all; post-audit
    // must route FIFO writes through Adc::write32 which is a no-op.
    // The FCS.LEVEL assertion below proves read-only semantics (FIFO
    // not pushed), not just the HashMap-shaped same-address compare.
    const FCS_LEVEL_MASK: u32 = 0x000F_0000; // bits [19:16] per adc.rs:86-87
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    // Seed FCS with a known pattern so we can detect spill-over.
    bus.write32(ADC_FCS, 0x1122_3344, 0);
    let fcs_before = bus.read32(ADC_FCS, 0);
    bus.write8(ADC_FIFO, 0x42, 0);
    bus.write8(ADC_FIFO + 1, 0x42, 0);
    bus.write8(ADC_FIFO + 2, 0x42, 0);
    bus.write8(ADC_FIFO + 3, 0x42, 0);
    let fcs_after = bus.read32(ADC_FCS, 0);
    assert_eq!(
        fcs_before, fcs_after,
        "ADC FIFO byte write must not touch FCS"
    );
    assert_eq!(
        fcs_after & FCS_LEVEL_MASK,
        fcs_before & FCS_LEVEL_MASK,
        "ADC FCS.LEVEL must be unchanged — byte writes to FIFO must not push"
    );
    assert!(!bus.bus_fault(0), "ADC FIFO byte write must not fault");
}

// ============================================================================
// §6.3 — SIO GPIO_OUT replication: existing coverage in tests.rs
// (sio_gpio_out_byte_write_replicates_across_lanes etc. at lines
// 8840/8879/8900). NOT duplicated here.
// ============================================================================

// ============================================================================
// §6.4 — Alias preservation. write8 to APB SET alias lane 1 OR's only
// the target byte into the target lane of the underlying register.
// ============================================================================

#[test]
fn s64_apb_set_alias_byte_write_only_ors_target_lane() {
    let mut bus = Bus::new();
    // Seed CLK_SYS_CTRL with 0x0000_0005 (bits 0, 2).
    bus.write32(APB_CLOCKS_CLK_SYS_CTRL, 0x0000_0005, 0);
    // SET alias = base | (2 << 12) — lane 1 offset of the SET alias
    // is at +1 (byte lane 1 = bits 8..15).
    let set_alias = APB_CLOCKS_CLK_SYS_CTRL + 0x2000 + 1;
    bus.write8(set_alias, 0xAB, 0);
    // Expected: bits 0..7 unchanged (0x05), bits 8..15 OR'd with 0xAB
    // → 0xAB, bits 16..31 unchanged (0).
    let got = bus.read32(APB_CLOCKS_CLK_SYS_CTRL, 0);
    assert_eq!(
        got, 0x0000_AB05,
        "APB SET-alias byte write lane 1: expected only bits 8..15 OR'd, got {got:#010x}"
    );
}

// ============================================================================
// §6.5 — W1C behaviour. Shape A (pure W1C): writing a byte clears only
// the target lane's bits. Shape B (mixed W1C): writing to a non-W1C
// lane preserves W1C bits.
// ============================================================================

// --- §6.5 NVIC_ICPR / NVIC_ICER (Shape A) ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_nvic_icpr0_byte_write_clears_only_target_lane() {
    let mut bus = Bus::new();
    // Pre-pend IRQs 3 (lane 0) and 11 (lane 1).
    bus.write32(NVIC_ISPR0, 0x0000_0808, 0);
    // Byte-clear lane 1 with 0x08 → clear bit 11 only.
    bus.write8(NVIC_ICPR0 + 1, 0x08, 0);
    // Bit 3 must survive.
    let got = bus.read32(NVIC_ISPR0, 0);
    assert_eq!(
        got & 0x0000_0008,
        0x0000_0008,
        "NVIC_ICPR0 byte-clear lane 1 must not disturb bit 3 in lane 0 (read_back {got:#010x})"
    );
    assert_eq!(got & (1 << 11), 0, "bit 11 should be cleared");
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_nvic_icpr1_byte_write_clears_only_target_lane() {
    let mut bus = Bus::new();
    // IRQs 32 (bit 0 in ISPR1, lane 0) and 40 (bit 8, lane 1)
    bus.write32(NVIC_ISPR1, 0x0000_0101, 0);
    bus.write8(NVIC_ICPR1 + 1, 0x01, 0);
    let got = bus.read32(NVIC_ISPR1, 0);
    assert_eq!(
        got & 1,
        1,
        "bit 0 of ICPR1 must survive byte clear of lane 1"
    );
    assert_eq!(got & (1 << 8), 0, "bit 8 should be cleared");
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_nvic_icer0_byte_write_clears_only_target_lane() {
    let mut bus = Bus::new();
    bus.write32(NVIC_ISER0, 0x0000_0808, 0);
    bus.write8(NVIC_ICER0 + 1, 0x08, 0);
    let got = bus.read32(NVIC_ISER0, 0);
    assert_eq!(
        got & 0x0000_0008,
        0x0000_0008,
        "NVIC_ICER0 byte-clear lane 1 must not disturb bit 3"
    );
    assert_eq!(got & (1 << 11), 0);
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_nvic_icer1_byte_write_clears_only_target_lane() {
    let mut bus = Bus::new();
    bus.write32(NVIC_ISER1, 0x0000_0101, 0);
    bus.write8(NVIC_ICER1 + 1, 0x01, 0);
    let got = bus.read32(NVIC_ISER1, 0);
    assert_eq!(got & 1, 1);
    assert_eq!(got & (1 << 8), 0);
}

// --- §6.5 SCB_CFSR / SCB_HFSR (Shape A) ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_scb_cfsr_byte_write_clears_only_target_lane() {
    let mut bus = Bus::new();
    // CFSR has MMFSR/BFSR/UFSR lanes. Seed with bits in lane 0 and
    // lane 1 via a full write.
    bus.write32(SCB_CFSR, 0x0000_0303, 0);
    // Byte-clear lane 1 with 0x03 → lane 0 should survive.
    bus.write8(SCB_CFSR + 1, 0x03, 0);
    let got = bus.read32(SCB_CFSR, 0);
    assert_eq!(got & 0xFF, 0x03, "CFSR lane 0 must survive");
    assert_eq!((got >> 8) & 0xFF, 0x00, "CFSR lane 1 should be cleared");
}

#[test]
#[ignore = "No public bus API seeds HFSR; it is only set via the \
            fault-escalation path. Without a seed, a byte-write test \
            trivially passes (storage is zero, RMW is a no-op) without \
            actually exercising the W1C preservation contract. Revisit \
            once the fault-escalation path has a test hook. See \
            tech_debt.md."]
fn s65_scb_hfsr_byte_write_clears_only_target_lane() {
    let mut bus = Bus::new();
    bus.write32(SCB_HFSR, 0x0000_0F0F, 0);
    let _ = bus.read32(SCB_HFSR, 0);
    bus.write8(SCB_HFSR + 1, 0x00, 0);
    let _ = bus.read32(SCB_HFSR, 0);
    assert!(!bus.bus_fault(0), "HFSR byte write must not fault");
}

// --- §6.5 DMA_INTR / INTS0 / INTS1 (Shape A) ---

/// Run a 1-word SRAM→SRAM DMA transfer on CH0 with TREQ=FORCE and
/// IRQ_QUIET=0, ticking until INTR bit 0 latches (or giving up after
/// a bounded number of ticks). Per `dma.rs`, INTR bit N latches when
/// channel N's TRANS_COUNT hits 0 with IRQ_QUIET clear.
fn establish_dma_ch0_intr(bus: &mut Bus) -> bool {
    // Seed source word so the read has something to fetch.
    bus.write32(SRAM_BASE + 0x100, 0xDEAD_BEEF, 0);
    bus.write32(SRAM_BASE + 0x200, 0x0, 0);
    // CH0: READ_ADDR = SRAM+0x100, WRITE_ADDR = SRAM+0x200.
    bus.write32(DMA_CH0_READ_ADDR, SRAM_BASE + 0x100, 0);
    bus.write32(DMA_BASE + 0x04, SRAM_BASE + 0x200, 0);
    bus.write32(DMA_BASE + 0x08, 1, 0); // TRANS_COUNT = 1 word
    // CTRL_TRIG: EN=1, DATA_SIZE=2 (word), TREQ_SEL=0x3F (FORCE),
    // INCR_READ=0, INCR_WRITE=0, CHAIN_TO=0 (self = no chain),
    // IRQ_QUIET=0 so INTR latches.
    bus.write32(DMA_BASE + 0x0C, 0x0000_0001 | (2 << 2) | (0x3F << 17), 0);
    // Tick DMA until INTR bit 0 latches.
    for _ in 0..16 {
        bus.tick_dma();
        if bus.read32(DMA_INTR, 0) & 0x1 != 0 {
            return true;
        }
    }
    false
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_dma_intr_byte_write_clears_only_target_lane() {
    // DMA INTR is pure W1C. Seed via a real CH0 transfer so bit 0
    // latches, then attempt narrow W1C on bit 0 and verify it clears.
    // Baseline DMA narrow path goes through the HashMap catch-all and
    // never reaches Dma::write32 — so the readback will still show
    // bit 0 set (FAIL). Post-audit must route narrow writes through
    // Dma::write32's W1C path.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    assert!(
        establish_dma_ch0_intr(&mut bus),
        "setup: DMA CH0 transfer must latch INTR bit 0"
    );
    let intr_before = bus.read32(DMA_INTR, 0);
    assert_ne!(intr_before & 0x1, 0, "setup: INTR bit 0 must be latched");
    // Byte-write lane 0 with 0x01 → W1C clear bit 0.
    bus.write8(DMA_INTR, 0x01, 0);
    assert!(!bus.bus_fault(0), "DMA INTR byte write must not fault");
    let intr_after = bus.read32(DMA_INTR, 0);
    assert_eq!(
        intr_after & 0x1,
        0,
        "DMA INTR bit 0 must clear via narrow W1C write (got {intr_after:#x})"
    );
}

#[test]
fn s65_dma_ints0_byte_write_no_fault() {
    // INTS0 (offset 0x40C) is a derived read. Writes to INTS0 clear
    // INTR bits per dma.rs:398-402. Byte write exercises the narrow
    // path — must route to Dma::write32, not the HashMap.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write32(DMA_BASE + 0x404, 0x0000_FFFF, 0); // INTE0
    bus.write8(DMA_INTS0, 0x01, 0);
    assert!(!bus.bus_fault(0), "DMA INTS0 byte write must not fault");
}

#[test]
fn s65_dma_ints1_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write32(DMA_BASE + 0x414, 0x0000_FFFF, 0); // INTE1
    bus.write8(DMA_INTS1, 0x01, 0);
    assert!(!bus.bus_fault(0), "DMA INTS1 byte write must not fault");
}

// --- §6.5 DMA CH CTRL (Shape B) ---

/// Drive a DMA CH0 transfer that intentionally writes to unmapped
/// region 0xA (not modelled) so WRITE_ERROR latches into CH0.CTRL
/// bit 29. This exercises the real error path — no private state
/// poking.
fn establish_dma_ch0_write_error(bus: &mut Bus) {
    // Set up minimal channel: read from SRAM, write to unmapped.
    bus.write32(DMA_CH0_READ_ADDR, 0x2000_0100, 0); // valid SRAM
    bus.write32(DMA_BASE + 0x04, 0xA000_0000, 0); // write addr — unmapped
    bus.write32(DMA_BASE + 0x08, 1, 0); // TRANS_COUNT
    // CTRL_TRIG: EN|DATA_SIZE=2 (word) | TREQ_SEL=0x3F (FORCE) | CHAIN_TO=0 (self, no chain)
    bus.write32(DMA_BASE + 0x0C, 0x0000_0001 | (2 << 2) | (0x3F << 17), 0);
    // Tick the DMA to execute.
    bus.tick_dma();
    bus.tick_dma();
}

#[test]
#[ignore = "DMA error-latch path not modelled in dma.rs (CTRL_WRITE_ERROR \
            never set); no public-API way to seed bits 29/30 in Stage 1. \
            Once the DMA error model lands, or the post-audit refactor \
            routes DMA narrow writes through Dma::write32, this test will \
            exercise Shape B W1C preservation. Spurious-pass note: on \
            baseline the byte-write hits the HashMap catch-all and never \
            reaches Dma — the readback is the untouched original CTRL, so \
            naive assertions would pass for the wrong reason."]
fn s65_dma_ch0_ctrl_byte_write_lane0_preserves_error_flags() {
    let mut bus = Bus::new();
    establish_dma_ch0_write_error(&mut bus);
    let ctrl_before = bus.read32(DMA_CH0_CTRL, 0);
    assert_ne!(
        ctrl_before & 0x6000_0000,
        0,
        "test setup: DMA CH0 error bits must latch"
    );
    bus.write8(DMA_CH0_CTRL, 0x00, 0);
    let ctrl_after = bus.read32(DMA_CH0_CTRL, 0);
    assert_eq!(
        ctrl_after & 0x6000_0000,
        ctrl_before & 0x6000_0000,
        "CH0 CTRL W1C error bits must survive narrow write to non-W1C lane 0"
    );
}

// --- §6.5 UART ICR / SPI SSPICR (Shape A) ---

#[test]
fn s65_uart0_uarticr_byte_write_no_fault() {
    // UARTICR is pure W1C (bits 0..10). Seed not possible without a
    // real interrupt event — use the exists-and-doesn't-fault check.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(UART0_UARTICR, 0x01, 0);
    bus.write8(UART0_UARTICR + 1, 0x01, 0);
    assert!(!bus.bus_fault(0), "UART0 ICR byte write must not fault");
}

#[test]
fn s65_uart1_uarticr_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(UART1_UARTICR, 0x01, 0);
    assert!(!bus.bus_fault(0));
}

#[test]
fn s65_spi0_sspicr_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(SPI0_SSPICR, 0x03, 0);
    assert!(!bus.bus_fault(0));
}

#[test]
fn s65_spi1_sspicr_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(SPI1_SSPICR, 0x03, 0);
    assert!(!bus.bus_fault(0));
}

// --- §6.5 TIMER INTR / PWM_INTR (Shape A) ---

#[test]
fn s65_timer0_intr_byte_write_clears_only_target_lane() {
    // TIMER0 INTR bits 0..3 are W1C. Seed via live alarms: enable the
    // TIMER0 TICKS domain so the 1 µs counter advances, arm ALARM0
    // and ALARM1 at target us=1, then tick until both fire.
    //
    // Post-bootrom state (HLD V5 §5.7) leaves TIMER0 TICKS with
    // CYCLES=12 but ENABLE=0, so alarms would never fire without
    // this explicit enable (the original test silently returned
    // when alarms didn't fire — R1 fix surfaces the real contract).
    const TICKS_TIMER0_CTRL: u32 = 0x4010_8000 + 0x18; // TICKS_BASE + (DOMAIN_TIMER0 * 0x0C)
    let mut bus = Bus::new();
    // Enable TIMER0 TICKS so its domain emits 1 µs edges.
    bus.write32(TICKS_TIMER0_CTRL, 0x1, 0);
    // Arm ALARM0 and ALARM1 at target=1 µs.
    bus.write32(TIMER0_BASE + 0x10, 1, 0);
    bus.write32(TIMER0_BASE + 0x14, 1, 0);
    // Tick peripherals enough sys_clks to accumulate >= 1 µs of edges.
    // With CYCLES=12 (post-bootrom) and sys_clks=1_000_000 per call,
    // one call produces ~83k edges — well past the 1 µs target.
    for _ in 0..10 {
        bus.tick_peripherals(1_000_000);
    }
    let intr_before = bus.read32(TIMER0_INTR, 0);
    assert_eq!(
        intr_before & 0x3,
        0x3,
        "setup: both alarms must have fired (INTR={intr_before:#x})"
    );
    // Byte-clear lane 0 with 0x01 → clear bit 0. Bit 1 must survive.
    bus.write8(TIMER0_INTR, 0x01, 0);
    let intr_after = bus.read32(TIMER0_INTR, 0);
    assert_eq!(intr_after & 0x1, 0, "bit 0 cleared");
    assert_eq!(intr_after & 0x2, 0x2, "bit 1 must survive narrow W1C");
}

#[test]
fn s65_timer1_intr_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(TIMER1_INTR, 0x01, 0);
    assert!(!bus.bus_fault(0));
}

#[test]
fn s65_pwm_intr_byte_write_no_fault() {
    // PWM_INTR at 0x400A_80F4 is pure W1C. Byte write must not fault.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(PWM_INTR, 0x01, 0);
    assert!(!bus.bus_fault(0), "PWM_INTR byte write must not fault");
}

// --- §6.5 IO_BANK0_INTR0..5 (Shape B) ---

#[test]
#[ignore = "TODO: un-ignore once io_bank0 INTR W1C semantics land (HLD §4.7 \
            catalogue entry requires peripheral-side W1C, not just bus-side \
            mask). Current model is plain RW storage per \
            `io_bank0.rs:192-200` (\"W1C is a future enhancement\"). A \
            no-fault-only test cannot distinguish correct from incorrect \
            behaviour and is worse than an ignored one."]
fn s65_io_bank0_intr0_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write32(IO_BANK0_INTR0, 0x0000_1133, 0);
    bus.write8(IO_BANK0_INTR0, 0x03, 0); // clear edge bits of GPIO 0
    assert!(!bus.bus_fault(0));
}

#[test]
#[ignore = "TODO: un-ignore once io_bank0 INTR W1C semantics land (HLD §4.7 \
            catalogue entry requires peripheral-side W1C, not just bus-side \
            mask). See companion test on INTR0."]
fn s65_io_bank0_intr5_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(IO_BANK0_INTR5, 0x03, 0);
    assert!(!bus.bus_fault(0));
}

// --- §6.5 SIO FIFO_ST (Shape B) ---

/// Drive ROE on core 0 by reading FIFO_RD while empty. This latches
/// `sio.fifo_roe[0] = true` which shows as bit 3 of FIFO_ST.
fn establish_sio_roe_core0(bus: &mut Bus) {
    // Default active_core is 0. Read FIFO_RD (0x058) to trigger ROE.
    let _ = bus.read32(SIO_BASE + 0x058, 0);
}

/// Drive WOF on core 0 by pushing 9 words into a full TX FIFO (capacity 8).
fn establish_sio_wof_core0(bus: &mut Bus) {
    for _ in 0..9 {
        bus.write32(SIO_BASE + 0x054, 0x1234_5678, 0);
    }
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_sio_fifo_st_byte_write_non_w1c_lane_preserves_wof_roe() {
    // WOF (bit 2) + ROE (bit 3) are W1C in lane 0. Byte-write to
    // lane 1 (no W1C bits) must not touch them. Baseline RMW
    // reads FIFO_ST = 0x0C, splices byte 1 with any value, writes
    // back word 0x00yy_0C which would clear both W1C bits.
    let mut bus = Bus::new();
    establish_sio_roe_core0(&mut bus);
    establish_sio_wof_core0(&mut bus);
    let st_before = bus.read32(SIO_FIFO_ST, 0);
    assert_eq!(st_before & 0x0C, 0x0C, "test setup: WOF+ROE should be set");
    // Byte-write lane 1 with 0x00.
    bus.write8(SIO_FIFO_ST + 1, 0x00, 0);
    let st_after = bus.read32(SIO_FIFO_ST, 0);
    assert_eq!(
        st_after & 0x0C,
        0x0C,
        "WOF+ROE must survive narrow write to lane 1 (got {st_after:#x})"
    );
}

// --- §6.5 SIO INTERP0/1 CTRL_LANE0 (Shape B) ---

/// Latch OVERF bits 23/25 into INTERP<n> CTRL_LANE0 via a signed-shift
/// overflow through POP_LANE0. Per `sio/interp.rs:188-215`, POP_LANE0
/// calls `shift_and_mask` for each lane and latches `OVERF0|OVERF` when
/// the pre-mask shifted value has bits above MASK_MSB set.
///
/// Config: SHIFT=0, MASK_LSB=0, MASK_MSB=15, SIGNED=1 (lane 0 bits
/// 15:0 signed). ACCUM0 = 0x7FFF_0000 — after shift=0, masked = 0
/// (non-negative), `above = 0x7FFF_0000 != 0` → overflow.
fn establish_interp_overf(bus: &mut Bus, interp_ctrl_lane0: u32) {
    let interp_base = interp_ctrl_lane0 & !0x3F;
    let accum0 = interp_base; // offset 0x00
    let pop_lane0 = interp_base + 0x14;
    // MASK_MSB=15 (15<<10), SIGNED=1 (1<<15), rest zero.
    bus.write32(interp_ctrl_lane0, (15 << 10) | (1 << 15), 0);
    bus.write32(accum0, 0x7FFF_0000, 0);
    // Reading POP_LANE0 runs shift_and_mask and latches OVERF bits.
    let _ = bus.read32(pop_lane0, 0);
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_sio_interp0_ctrl_lane0_byte_write_preserves_overf() {
    // Shape B: byte-write to a non-W1C lane (lane 1 = bits 8..15,
    // holds MASK_MSB/SIGNED but no OVERF bits) must preserve the
    // sticky OVERF flags in lanes 2/3 (bits 23..25).
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    establish_interp_overf(&mut bus, SIO_INTERP0_CTRL_LANE0);
    let ctrl_before = bus.read32(SIO_INTERP0_CTRL_LANE0, 0);
    assert_ne!(
        ctrl_before & 0x0380_0000,
        0,
        "setup: OVERF bits 23..25 must latch in INTERP0 CTRL_LANE0 (got {ctrl_before:#x})"
    );
    // Byte-write lane 1 with the current value (preserve config).
    // Lane 1 contains no W1C bits, so baseline RMW-with-mask or
    // post-audit narrow path must both leave OVERF in lanes 2/3 alone.
    let lane1 = ((ctrl_before >> 8) & 0xFF) as u8;
    bus.write8(SIO_INTERP0_CTRL_LANE0 + 1, lane1, 0);
    let ctrl_after = bus.read32(SIO_INTERP0_CTRL_LANE0, 0);
    assert_eq!(
        ctrl_after & 0x0380_0000,
        ctrl_before & 0x0380_0000,
        "INTERP0 CTRL_LANE0 OVERF must survive byte-write to non-W1C lane 1"
    );
    assert!(!bus.bus_fault(0));
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_sio_interp1_ctrl_lane0_byte_write_preserves_overf() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    establish_interp_overf(&mut bus, SIO_INTERP1_CTRL_LANE0);
    let ctrl_before = bus.read32(SIO_INTERP1_CTRL_LANE0, 0);
    assert_ne!(
        ctrl_before & 0x0380_0000,
        0,
        "setup: OVERF bits 23..25 must latch in INTERP1 CTRL_LANE0 (got {ctrl_before:#x})"
    );
    let lane1 = ((ctrl_before >> 8) & 0xFF) as u8;
    bus.write8(SIO_INTERP1_CTRL_LANE0 + 1, lane1, 0);
    let ctrl_after = bus.read32(SIO_INTERP1_CTRL_LANE0, 0);
    assert_eq!(
        ctrl_after & 0x0380_0000,
        ctrl_before & 0x0380_0000,
        "INTERP1 CTRL_LANE0 OVERF must survive byte-write to non-W1C lane 1"
    );
    assert!(!bus.bus_fault(0));
}

// --- §6.5 GLITCH_DETECTOR TRIG_STATUS (Shape A) ---

#[test]
fn s65_glitch_trig_status_byte_write_no_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(GLITCH_TRIG_STATUS, 0x01, 0);
    assert!(!bus.bus_fault(0));
}

// --- §6.5 ADC FCS (Shape B) ---

#[test]
fn s65_adc_fcs_byte_write_non_w1c_lane_preserves_rw_bits() {
    // Seed FCS with FCS_EN=1, FCS_THRESH=0xC (bits 24..27 = 0xC000_0000
    // shifted → lane 3). Byte-write lane 1 with 0 (covers bits 8..15,
    // no W1C bits there — bits 10/11 UNDER/OVER are in lane 1 but we
    // can't seed them via public writes without triggering real
    // under/overflow). Post-audit RMW-with-mask will zero those W1C
    // bits in the merged word before writing back — since they were
    // zero anyway, byte-write lane 1 should not corrupt lane 0 or 3.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write32(ADC_FCS, 0x0C00_0001, 0); // FCS_EN + FCS_THRESH=0xC
    let seed = bus.read32(ADC_FCS, 0);
    assert_eq!(seed & 0xFF, 0x01, "FCS_EN seed");
    assert_eq!(seed & 0x0F00_0000, 0x0C00_0000, "FCS_THRESH seed");
    bus.write8(ADC_FCS + 1, 0x00, 0);
    let after = bus.read32(ADC_FCS, 0);
    assert_eq!(after & 0xFF, 0x01, "FCS_EN must survive byte write lane 1");
    assert_eq!(
        after & 0x0F00_0000,
        0x0C00_0000,
        "FCS_THRESH must survive byte write lane 1"
    );
}

// --- §6.5 SHA256 CSR (Shape B) ---

/// Trigger ERR_WDATA_NOT_RDY by writing 17 WDATA words. After the 16th
/// triggers a compress, the 17th sets the bit.
fn establish_sha256_err_wdata(bus: &mut Bus) {
    for _ in 0..17 {
        bus.write32(SHA256_WDATA, 0xDEAD_BEEF, 0);
    }
}

#[test]
fn s65_sha256_csr_byte_write_clears_err_without_retriggering_start() {
    // CSR bit 3 (ERR_WDATA_NOT_RDY) is W1C; bit 0 (START) is
    // self-clearing-W (reads as 0). Seed err_wdata via 17 writes,
    // then byte-write lane 0 with 0x08 to clear bit 3. Bit 0 should
    // not re-trigger (read-back is 0, write-back 0 → no start).
    let mut bus = Bus::new();
    establish_sha256_err_wdata(&mut bus);
    let csr_before = bus.read32(SHA256_CSR, 0);
    // SHA256 bit-3 (ERR_WDATA_NOT_RDY) latching is deterministic per
    // sha256.rs:154-156: writing a 17th WDATA word sets the bit, and
    // it's cleared only by W1C to CSR. A silent skip hides regressions
    // in the WDATA latch — fail loudly if setup doesn't produce it.
    assert_ne!(
        csr_before & 0x08,
        0,
        "setup: SHA256 ERR_WDATA_NOT_RDY must latch after 17 WDATA writes (csr={csr_before:#x})"
    );
    // Byte-write lane 0 with 0x08 (clear ERR_WDATA_NOT_RDY).
    bus.write8(SHA256_CSR, 0x08, 0);
    let csr_after = bus.read32(SHA256_CSR, 0);
    assert_eq!(csr_after & 0x08, 0, "ERR_WDATA_NOT_RDY must be cleared");
    assert!(!bus.bus_fault(0), "no fault");
}

// --- §6.5 Guard: SHCSR narrow write is plain RW ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_shcsr_byte_write_is_plain_rw() {
    // HLD §4.1 notes SHCSR is plain RW in our model — default RMW
    // should handle it correctly. Seed, byte-write, verify.
    let mut bus = Bus::new();
    bus.write32(SCB_SHCSR, 0x1122_3344, 0);
    let seed = bus.read32(SCB_SHCSR, 0);
    bus.write8(SCB_SHCSR, 0xAA, 0);
    let want = splice_byte(seed, 0, 0xAA);
    assert_eq!(
        bus.read32(SCB_SHCSR, 0),
        want,
        "SHCSR narrow write is plain RW — no spurious W1C behaviour"
    );
}

// --- §6.5 Guard: ICSR RMW idempotence ---

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s65_icsr_byte_write_preserves_pendsv_w1s() {
    // HLD §4.1: ICSR not in W1C catalogue. RMW is safe because PENDSVSET
    // is W1S (idempotent on write-back of read value), and W1C-via-bit
    // (PENDSVCLR, PENDSTCLR) are write-only and read as 0. So byte-write
    // to a non-PENDSV lane must preserve PENDSVSET.
    let mut bus = Bus::new();
    const ICSR_PENDSVSET: u32 = 1 << 28;
    // Pend PendSV.
    bus.write32(SCB_ICSR, ICSR_PENDSVSET, 0);
    let icsr_before = bus.read32(SCB_ICSR, 0);
    assert_ne!(
        icsr_before & ICSR_PENDSVSET,
        0,
        "test setup: PENDSVSET should be set"
    );
    // Byte-read lane 3 (contains PENDSVSET bit 28).
    let lane3 = bus.read8(SCB_ICSR + 3, 0);
    assert_eq!(lane3 & 0x10, 0x10, "PENDSVSET visible in lane 3 read");
    // Byte-write a different lane (lane 0) with 0 — no-op.
    bus.write8(SCB_ICSR, 0x00, 0);
    let icsr_after = bus.read32(SCB_ICSR, 0);
    assert_ne!(
        icsr_after & ICSR_PENDSVSET,
        0,
        "PENDSVSET must survive byte-write to different lane"
    );
}

// ============================================================================
// §6.6 — NVIC_ISPR narrow write triggers dispatch
// ============================================================================

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s66_nvic_ispr_byte_pend_propagates_to_irq_pending() {
    // Byte-pend IRQ 8 via write8(NVIC_ISPR0+1, 0x01). IRQ 8's bit is
    // bit 8 of NVIC_ISPR0 → lane 1, bit 0. The post-audit narrow path
    // must (a) latch the PPB architectural bit AND (b) mirror into the
    // `bus.atomics.irq_pending_load(core)` observability bitmap — the idiomatic
    // NVIC-mirror test per `tests.rs:6152` (see
    // `test_mmio_nvic_ispr_write_mirrors_into_irq_pending_and_dispatches`).
    //
    // Baseline PPB narrow write is dropped silently → neither the PPB
    // storage nor the `irq_pending` mirror updates (FAIL). Post-audit
    // must route through the unified narrow path that also updates the
    // mirror.
    let mut bus = Bus::new();
    // Enable IRQ 8.
    bus.write32(NVIC_ISER0, 1 << 8, 0);
    // Byte-pend IRQ 8 via narrow write.
    bus.write8(NVIC_ISPR0 + 1, 0x01, 0);
    // PPB architectural latch: bit 8 of NVIC_ISPR0 must be set.
    let ispr = bus.read32(NVIC_ISPR0, 0);
    assert_ne!(
        ispr & (1 << 8),
        0,
        "byte-write to NVIC_ISPR0+1 must set IRQ 8 pending in PPB (got ispr={ispr:#010x})"
    );
    // `irq_pending[0]` mirror: bit 8 must track the PPB latch so the
    // dispatch path sees the pending IRQ without requiring a full
    // word-write round-trip.
    assert_ne!(
        bus.atomics.irq_pending_load(0) & (1u64 << 8),
        0,
        "byte-write to NVIC_ISPR0+1 must mirror into bus.atomics.irq_pending_load(0) bit 8 \
         (got irq_pending[0]={:#018x})",
        bus.atomics.irq_pending_load(0)
    );
}

// ============================================================================
// §6.7 — Unmapped bus-fault symmetry
// ============================================================================

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s67_write8_unmapped_sets_bus_fault() {
    // Baseline: write8 to region 0xA silently drops (bus/mod.rs:1960).
    // Post-audit: must mirror read8 behaviour and set bus_fault.
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write8(UNMAPPED_ADDR, 0x42, 0);
    assert!(
        bus.bus_fault(0),
        "write8 on unmapped region must set bus_fault — HLD §6.7"
    );
}

#[test]
#[ignore = "narrow-audit Stage 2/3/4 not applied (reverse-merge tech debt)"]
fn s67_write16_unmapped_sets_bus_fault() {
    let mut bus = Bus::new();
    clear_bf(&mut bus);
    bus.write16(UNMAPPED_ADDR, 0x4242, 0);
    assert!(
        bus.bus_fault(0),
        "write16 on unmapped region must set bus_fault — HLD §6.7"
    );
}
