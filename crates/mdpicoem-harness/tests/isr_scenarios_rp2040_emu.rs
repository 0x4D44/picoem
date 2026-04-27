//! EMU-side integration test for the V1 oracle scenarios
//! (`silicon_isr_diff_rp2040`). The silicon side requires a Pico
//! debug probe attached to an RP2040 board and is therefore not in CI;
//! this offline test pins the EMU side and is the day-to-day gate
//! that V5 IRQ plumbing still satisfies the oracle's done-definition.
//!
//! Validates HLD V5 §6.2 / §10: both V1 scenarios PASS on the EMU
//! side under `step_quantum=1` serial mode. HardFault and Timeout
//! outcomes produce distinct, diagnostic error messages so a
//! regression points directly at the misdispatch / hang.

use mdpicoem_harness::isr_scenarios_rp2040::{
    self, EmuOutcome, IsrScenario, run_emu_scenario, setup_emulator_image,
};
use mdrp2040::{Config, EmulatorBuilder};

/// Build a `step_quantum=1` serial `Emulator`, halt core 1 (M0+ V1
/// oracle is single-core), set the active core to 0, then run the
/// shared setup + run loop for `sc`.
fn run_scenario(sc: &IsrScenario) -> EmuOutcome {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .expect("Serial build is infallible");
    emu.core_mut(1).halt();
    emu.bus.set_active_core(0);
    setup_emulator_image(&mut emu, sc);
    run_emu_scenario(&mut emu, sc)
}

fn find_scenario(name: &str) -> &'static IsrScenario {
    isr_scenarios_rp2040::SCENARIOS
        .iter()
        .find(|s| s.name == name)
        .unwrap_or_else(|| panic!("scenario '{}' missing from catalogue", name))
}

#[test]
fn isr_m0_timer_cold_passes_on_emu() {
    let sc = find_scenario("isr_m0_timer_cold");
    match run_scenario(sc) {
        EmuOutcome::Completed(obs) => {
            // Per HLD V5 §6.2 step 3 / OBS_TIMER_COLD ordering:
            //   obs[0] = ctr_timer (full word)
            //   obs[1] = TIMER.INTR (mask 0x1)
            assert_eq!(
                obs[0], 1,
                "ctr_timer should be 1 after one ISR fire, got {}",
                obs[0],
            );
            assert_eq!(
                obs[1] & 1,
                0,
                "TIMER.INTR low bit should be cleared by handler W1C, got 0x{:08X}",
                obs[1],
            );
        }
        EmuOutcome::HardFault { pc, ipsr } => {
            panic!(
                "isr_m0_timer_cold: EMU hardfault at pc=0x{:08X} ipsr={}",
                pc, ipsr,
            );
        }
        EmuOutcome::Timeout => {
            panic!("isr_m0_timer_cold: cycle budget exhausted before ctr_timer reached 1");
        }
    }
}

#[test]
fn isr_m0_tail_chain_passes_on_emu() {
    let sc = find_scenario("isr_m0_tail_chain_pendsv_systick");
    match run_scenario(sc) {
        EmuOutcome::Completed(obs) => {
            // Per HLD V5 §6.2 + §11 / OBS_TAIL_CHAIN ordering:
            //   obs[0] = ctr_pendsv
            //   obs[1] = ctr_systick
            // Both should be exactly 1 — SysTick disable inside the
            // handler (V5 §11) is what makes ctr_systick == 1
            // satisfiable in the first place.
            assert_eq!(obs[0], 1, "ctr_pendsv should be 1, got {}", obs[0]);
            assert_eq!(obs[1], 1, "ctr_systick should be 1, got {}", obs[1]);
        }
        EmuOutcome::HardFault { pc, ipsr } => {
            panic!(
                "isr_m0_tail_chain: EMU hardfault at pc=0x{:08X} ipsr={}",
                pc, ipsr,
            );
        }
        EmuOutcome::Timeout => {
            panic!("isr_m0_tail_chain: cycle budget exhausted before ctr_pendsv reached 1",);
        }
    }
}
