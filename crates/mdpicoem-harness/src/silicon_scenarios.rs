// Peripheral oracle catalog — scenarios diffed by
// `silicon_periph_diff_rp2350` against real RP2354 silicon. Each
// scenario is absolute-MMIO writes + a sysclk bound + readbacks; the
// runner CYCCNT-measures actual cycles and diffs emulator vs HW. See
// `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md`
// §Oracle 1.

// ---------------------------------------------------------------------------
// Absolute MMIO bases and bit constants (RP2350)
// ---------------------------------------------------------------------------

pub const PIO0_BASE: u32 = 0x5020_0000;
pub const PIO1_BASE: u32 = 0x5030_0000;

pub const PLL_SYS_BASE: u32 = 0x4005_0000;

pub const RESETS_BASE: u32 = 0x4002_0000;
pub const RESETS_RESET: u32 = RESETS_BASE + 0x00;

/// APB alias offsets. `+0x2000` = SET (OR), `+0x3000` = CLR (AND NOT).
pub const ALIAS_SET: u32 = 0x2000;
pub const ALIAS_CLR: u32 = 0x3000;

/// RESETS bits used by v1 scenarios.
pub const RESET_IO_BANK0: u32 = 1 << 6;
pub const RESET_PADS_BANK0: u32 = 1 << 9;
pub const RESET_PIO0: u32 = 1 << 11;
pub const RESET_PIO1: u32 = 1 << 12;
pub const RESET_PLL_SYS: u32 = 1 << 14;

// PIO register offsets (identical for all three PIO blocks).
pub const PIO_CTRL_OFF: u32 = 0x000;
pub const PIO_FDEBUG_OFF: u32 = 0x008;
pub const PIO_DBG_PADOE_OFF: u32 = 0x040;
pub const PIO_INSTR_MEM_OFF: u32 = 0x048;
pub const PIO_SM_STRIDE: u32 = 0x18;
pub const PIO_SM0_BASE_OFF: u32 = 0x0C8;
pub const PIO_SM_CLKDIV_OFF: u32 = 0x00;
pub const PIO_SM_EXECCTRL_OFF: u32 = 0x04;
pub const PIO_SM_ADDR_OFF: u32 = 0x0C;
pub const PIO_SM_PINCTRL_OFF: u32 = 0x14;

/// Compute the absolute address of `SMx_<field>` for a given PIO base.
pub const fn pio_sm_addr(base: u32, sm: u32, field_off: u32) -> u32 {
    base + PIO_SM0_BASE_OFF + sm * PIO_SM_STRIDE + field_off
}

/// Compute the absolute address of `INSTR_MEM[slot]` for a given PIO base.
pub const fn pio_instr_mem_addr(base: u32, slot: u32) -> u32 {
    base + PIO_INSTR_MEM_OFF + slot * 4
}

// SIO / IO_BANK0 / PADS_BANK0 addresses used below.
pub const SIO_GPIO_IN: u32 = 0xD000_0004;
pub const SIO_GPIO_OE: u32 = 0xD000_0030;
pub const IO_BANK0_GPIO0_CTRL: u32 = 0x4002_8000 + 0x04;
pub const PADS_BANK0_GPIO0: u32 = 0x4003_8000 + 0x04;

// PLL_SYS register offsets + CS.LOCK bit.
pub const PLL_CS_OFF: u32 = 0x000;
pub const PLL_PWR_OFF: u32 = 0x004;
pub const PLL_FBDIV_INT_OFF: u32 = 0x008;
pub const PLL_PRIM_OFF: u32 = 0x00C;
pub const PLL_CS_LOCK_BIT: u32 = 1 << 31;

// ---------------------------------------------------------------------------
// Scenario type
// ---------------------------------------------------------------------------

/// A single peripheral oracle scenario. Runner applies `setup` in order
/// (probe-rs on HW, `bus.write32` on EMU), runs at most `max_sysclks`
/// cycles, then reads `observe` (MMIO) + `observe_pins` (GPIO). First
/// masked-bit divergence wins.
pub struct PeriphScenario {
    pub name: &'static str,
    /// `(absolute_addr, value)` — must be in APB/AHB (0x4000_0000..
    /// 0x5FFF_FFFF) or SIO (0xD000_0000); enforced by unit tests.
    pub setup: &'static [(u32, u32)],
    /// Upper bound on sysclks; CYCCNT-measured actual is handed to
    /// `Emulator::run` so both sides advance identically.
    pub max_sysclks: u32,
    /// `(absolute_addr, mask)` — `0xFFFF_FFFF` = full word.
    pub observe: &'static [(u32, u32)],
    /// GPIO pins to sample drive+level. 0 = skip pins.
    pub observe_pins: u32,
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

// S1: PIO0 SM0 runs `JMP 0` in a one-instruction loop. Positive
// control — ADDR never advances past 0, HW and EMU MUST agree.
const S_PIO0_NOP_LOOP: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0x0000), // JMP 0
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_NOP_LOOP: &[(u32, u32)] = &[
    // SM_ADDR is a 5-bit register.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_ADDR_OFF), 0x1F),
];

// S2: `SET X, 31 / JMP X-- [1] / JMP 2 (stall)`. After countdown, ADDR
// settles at slot 2 (the stall). EXECCTRL is programmed with a
// non-default WRAP so divergence in wrap-bit storage shows up.
const S_PIO0_FIXED_CYCLES: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0xE03F), // SET X, 31
    (pio_instr_mem_addr(PIO0_BASE, 1), 0x0041), // JMP X-- 1
    (pio_instr_mem_addr(PIO0_BASE, 2), 0x0002), // JMP 2 (stall)
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    // EXECCTRL: WRAP_TOP=2 (bits 16:12), WRAP_BOTTOM=0 (bits 11:7).
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 2u32 << 12),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_FIXED_CYCLES: &[(u32, u32)] = &[
    // ADDR is a 5-bit field.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_ADDR_OFF), 0x1F),
    // EXECCTRL WRAP_TOP/BOTTOM bits [16:7] — the fields we programmed.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 0x0001_FF80),
];

// S3: GPIO0 side-set toggle. SM0 runs `JMP 0, side 1` — one
// instruction with side=1 on GPIO0 every cycle. IO_BANK0 / PADS_BANK0
// configured to route GPIO0 through PIO0.
const S_PIO0_SIDE_SET_TOGGLE: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0 | RESET_IO_BANK0 | RESET_PADS_BANK0),
    // PADS_BANK0 GPIO0: IE=1, drive=4 mA (value matches paced_bench_rp2350).
    (PADS_BANK0_GPIO0, 0x0000_0056),
    // IO_BANK0 GPIO0_CTRL: FUNCSEL=6 (PIO0).
    (IO_BANK0_GPIO0_CTRL, 0x0000_0006),
    // INSTR_MEM[0] = JMP 0, side-set 1. With SIDESET_COUNT=1, side value
    // 1 lives in delay/sideset field bit 4 → opcode 0x1000.
    (pio_instr_mem_addr(PIO0_BASE, 0), 0x1000),
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    // PINCTRL: SIDESET_COUNT=1 (bits 31:29), SIDESET_BASE=0.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_PINCTRL_OFF), 1 << 29),
    // EXECCTRL: SIDE_PINDIR=0 (default — side-set writes to OUT).
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 0),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_SIDE_SET_TOGGLE: &[(u32, u32)] = &[
    // PRIMARY SIGNAL. `DBG_PADOE` is the PIO-side output-enable mirror
    // (PIO0 + 0x040). With EXECCTRL.SIDE_PINDIR=0 and no explicit
    // `SET PINDIRS`, RP2350 §11.3.2.3 says side-set drives pin values
    // only — direction stays zero. Emulator's `PioBlock::update_pads`
    // currently ORs the positioned side-set mask into `pad_oe`
    // unconditionally (see `mdpicoem-common/src/pio/mod.rs:196`), so
    // HW=0 / EMU=1 here is the load-bearing divergence.
    (PIO0_BASE + PIO_DBG_PADOE_OFF, 0xFFFF_FFFF),
    // Corroboration: FDEBUG TXSTALL/TXOVER bands [27:24] + [19:16] — a
    // healthy side-set loop keeps both zero.
    (PIO0_BASE + PIO_FDEBUG_OFF, 0x0F0F_0000),
    // Corroboration only — `observe_pins` below samples the SIO-side
    // OE/IN latches through `SIO_GPIO_OE` / `SIO_GPIO_IN`. These mirror
    // bus state and trivially agree regardless of which peripheral is
    // driving, so they corroborate but do not themselves expose the
    // bug. The primary signal is `DBG_PADOE` above.
];

// S4: PIO0 RESETS gating — 2-instruction program (NOP + JMP 0) so
// ADDR alternates 0↔1 while running. Re-asserting PIO0 reset *after*
// starting SM0 should freeze ADDR on silicon (RP2350 holds PIO inert
// while RESETS bit is set). The emulator currently ticks PIO
// regardless of RESETS (tech-debt "PIO not gated on RESETS bit") so a
// broken emu keeps alternating during `emu.run(actual_sysclks)` and
// terminates at `actual_sysclks % 2`; a correctly-gated emu stays at
// its post-setup ADDR (0). Silicon's ADDR at gate time is a function
// of probe-rs setup latency (HW runs real sysclks between each MMIO
// write), so HW lands at either 0 or 1.
//
// This is an imperfect oracle: if EMU (broken) and HW happen to land
// on the same parity, the bug hides for that run. Acceptable as a
// best-effort signal — the ~50% exposure rate is strictly better than
// the prior 1-instruction design's 0% exposure.
const S_PIO0_RESET_GATING: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0xA042), // NOP (MOV Y, Y)
    (pio_instr_mem_addr(PIO0_BASE, 1), 0x0000), // JMP 0
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
    // Slam PIO0 back into reset after SM is running.
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
];
const O_PIO0_RESET_GATING: &[(u32, u32)] = &[
    // SM_ADDR (5-bit). Divergence here = gating broken.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_ADDR_OFF), 0x1F),
];

// S5: PLL_SYS lock. Configure FBDIV=100, REFDIV=1, POSTDIV=2/2, power
// up, spin 1500 sysclks, read CS.LOCK. Emulator's `pll_read_from`
// forces LOCK=1 unconditionally (tech-debt "PLL LOCK always 1",
// originally logged against mdrp2040 — same pattern in mdrp2350).
const S_PLL_SYS_LOCK_TIMING: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PLL_SYS),
    (PLL_SYS_BASE + PLL_CS_OFF, 1),        // REFDIV=1
    (PLL_SYS_BASE + PLL_FBDIV_INT_OFF, 100),
    (PLL_SYS_BASE + PLL_PRIM_OFF, (2u32 << 16) | (2u32 << 12)),
    (PLL_SYS_BASE + PLL_PWR_OFF, 0),       // all powered up
];
const O_PLL_SYS_LOCK_TIMING: &[(u32, u32)] = &[
    // CS.LOCK bit only — the narrower the mask, the clearer the failure.
    (PLL_SYS_BASE + PLL_CS_OFF, PLL_CS_LOCK_BIT),
];

/// Initial catalog. New scenarios append to the end so filter-by-substring
/// output stays ordered as cases are added.
pub const SCENARIOS: &[PeriphScenario] = &[
    PeriphScenario {
        name: "pio0_nop_loop",
        setup: S_PIO0_NOP_LOOP,
        max_sysclks: 100,
        observe: O_PIO0_NOP_LOOP,
        observe_pins: 0,
    },
    PeriphScenario {
        name: "pio0_fixed_cycles",
        setup: S_PIO0_FIXED_CYCLES,
        max_sysclks: 200,
        observe: O_PIO0_FIXED_CYCLES,
        observe_pins: 0,
    },
    PeriphScenario {
        name: "pio0_side_set_toggle",
        setup: S_PIO0_SIDE_SET_TOGGLE,
        max_sysclks: 100,
        observe: O_PIO0_SIDE_SET_TOGGLE,
        observe_pins: 0x0000_0001,
    },
    PeriphScenario {
        name: "pio0_reset_gating",
        setup: S_PIO0_RESET_GATING,
        max_sysclks: 200,
        observe: O_PIO0_RESET_GATING,
        observe_pins: 0,
    },
    PeriphScenario {
        name: "pll_sys_lock_timing",
        setup: S_PLL_SYS_LOCK_TIMING,
        max_sysclks: 1500,
        observe: O_PLL_SYS_LOCK_TIMING,
        observe_pins: 0,
    },
];

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn is_mmio(addr: u32) -> bool {
        (0x4000_0000..0x6000_0000).contains(&addr)
            || (0xD000_0000..0xE000_0000).contains(&addr)
    }

    /// Catalog must ship the five v1 scenarios the HLD enumerates.
    #[test]
    fn test_scenarios_catalog_nonempty() {
        assert!(SCENARIOS.len() >= 5, "at least 5 scenarios, got {}", SCENARIOS.len());
    }

    /// Every setup / observe address must target MMIO — catches a
    /// relative-address regression (e.g. 0x0C8 instead of 0x5020_00C8).
    #[test]
    fn test_setup_addresses_all_absolute() {
        for sc in SCENARIOS {
            for (i, (a, _)) in sc.setup.iter().enumerate() {
                assert!(is_mmio(*a), "{} setup[{}] 0x{:08X}", sc.name, i, a);
            }
            for (i, (a, _)) in sc.observe.iter().enumerate() {
                assert!(is_mmio(*a), "{} observe[{}] 0x{:08X}", sc.name, i, a);
            }
        }
    }

    /// Observing nothing = always PASS = bug.
    #[test]
    fn test_observe_masks_are_nonzero() {
        for sc in SCENARIOS {
            let any = sc.observe.iter().any(|(_, m)| *m != 0) || sc.observe_pins != 0;
            assert!(any, "scenario '{}' observes nothing", sc.name);
        }
    }

    /// `max_sysclks=0` would never execute the sled.
    #[test]
    fn test_max_sysclks_positive() {
        for sc in SCENARIOS {
            assert!(sc.max_sysclks > 0, "'{}' has max_sysclks=0", sc.name);
        }
    }

    /// Duplicate names would confuse `--filter` and summary output.
    #[test]
    fn test_no_duplicate_scenario_names() {
        let mut seen: HashSet<&str> = HashSet::new();
        for sc in SCENARIOS {
            assert!(seen.insert(sc.name), "duplicate name '{}'", sc.name);
        }
    }

    /// Reading `TXFx` / `RXFx` pops a FIFO entry — an observable that
    /// silently mutates state is a footgun waiting for a future catalog
    /// author. Neither block allows this under any circumstances.
    ///
    /// TXF range per PIO block: `[base + 0x10, base + 0x20)` (4 SMs × 4
    /// bytes). RXF range: `[base + 0x20, base + 0x30)`.
    #[test]
    fn test_no_fifo_pop_on_read_observables() {
        let mut violations = 0usize;
        for sc in SCENARIOS {
            for &(addr, _mask) in sc.observe {
                for base in [PIO0_BASE, PIO1_BASE] {
                    let txf_lo = base + 0x10;
                    let txf_hi = base + 0x20;
                    let rxf_lo = base + 0x20;
                    let rxf_hi = base + 0x30;
                    if (txf_lo..txf_hi).contains(&addr) || (rxf_lo..rxf_hi).contains(&addr)
                    {
                        eprintln!(
                            "scenario '{}' observes FIFO 0x{:08X} (pops on read)",
                            sc.name, addr,
                        );
                        violations += 1;
                    }
                }
            }
        }
        assert_eq!(violations, 0, "FIFO-popping observables present");
    }
}
