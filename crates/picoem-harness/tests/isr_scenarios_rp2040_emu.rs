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

use picoem_harness::isr_scenarios_rp2040::{
    self, EmuOutcome, IsrScenario, run_emu_scenario, setup_emulator_image,
};
use rp2040_emu::{Config, EmulatorBuilder};

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

/// V2 §3.4 — NVIC ISER/ICER/ISPR/ICPR high-bits RAZ/WI scenario.
///
/// Pure register-shape; no handler dispatch. The main body writes
/// `0xFFFF_FFFF` to each of the four NVIC bitmaps, reads each back, and
/// stores the readback into a SRAM cell. Pre-seeds before ICER/ICPR use
/// values within the implemented mask (`0x0000_FFFF`) since firmware
/// can't bypass the mask on real silicon.
///
/// Expected (per HLD §3.4):
///   - iser_readback == 0x03FF_FFFF — high bits RAZ
///   - ispr_readback == 0x03FF_FFFF — same
///   - icer_readback == 0           — masked clear covers full pre-seed
///   - icpr_readback == 0           — same
#[test]
fn isr_m0_nvic_high_bits_razwi_passes_on_emu() {
    let sc = find_scenario("isr_m0_nvic_high_bits_razwi");
    match run_scenario(sc) {
        EmuOutcome::Completed(obs) => {
            // Per OBS_NVIC_RAZWI ordering:
            //   obs[0] = iser_readback (primary)
            //   obs[1] = ispr_readback
            //   obs[2] = icer_readback
            //   obs[3] = icpr_readback
            assert_eq!(
                obs[0], 0x03FF_FFFF,
                "iser_readback should be 0x03FF_FFFF (high bits RAZ), got 0x{:08X}",
                obs[0],
            );
            assert_eq!(
                obs[1], 0x03FF_FFFF,
                "ispr_readback should be 0x03FF_FFFF (high bits RAZ), got 0x{:08X}",
                obs[1],
            );
            assert_eq!(
                obs[2], 0,
                "icer_readback should be 0 after masked-clear of full pre-seed, got 0x{:08X}",
                obs[2],
            );
            assert_eq!(
                obs[3], 0,
                "icpr_readback should be 0 after masked-clear of full pre-seed, got 0x{:08X}",
                obs[3],
            );
        }
        EmuOutcome::HardFault { pc, ipsr } => {
            panic!(
                "isr_m0_nvic_high_bits_razwi: EMU hardfault at pc=0x{:08X} ipsr={}",
                pc, ipsr,
            );
        }
        EmuOutcome::Timeout => {
            panic!(
                "isr_m0_nvic_high_bits_razwi: cycle budget exhausted before iser_readback advanced",
            );
        }
    }
}

/// V2 §3.3 — WFI wake on TIMER alarm. Validates the both-cores-park
/// wake path now that tech_debt §1649 is closed for RP2040
/// (`step_serial` advances the master clock to the soonest scheduled
/// alarm when both cores are blocked).
#[test]
fn isr_m0_wfi_wake_passes_on_emu() {
    let sc = find_scenario("isr_m0_wfi_wake");
    match run_scenario(sc) {
        EmuOutcome::Completed(obs) => {
            // Per OBS_WFI ordering:
            //   obs[0] = phase_at_entry (primary, load-bearing)
            //   obs[1] = ctr_timer
            //   obs[2] = phase
            assert_eq!(
                obs[0], 1,
                "phase_at_entry should be 1 (handler ran during WFI window, before main resumed), got {}",
                obs[0],
            );
            assert_eq!(
                obs[1], 1,
                "ctr_timer should be 1 after one TIMER ISR fire, got {}",
                obs[1],
            );
            assert_eq!(
                obs[2], 2,
                "phase should be 2 after main resumed past wfi, got {}",
                obs[2],
            );
        }
        EmuOutcome::HardFault { pc, ipsr } => {
            panic!(
                "isr_m0_wfi_wake: EMU hardfault at pc=0x{:08X} ipsr={}",
                pc, ipsr,
            );
        }
        EmuOutcome::Timeout => {
            panic!("isr_m0_wfi_wake: cycle budget exhausted before phase_at_entry advanced",);
        }
    }
}

/// V2 §3.1 — priority preemption / tail-chain.
///
/// Two TIMER alarms armed at the same TIMERAWL deadline, both INTE bits
/// set, with `NVIC_IPR0 = (0xC0 << 0) | (0x40 << 8)`. M0+ implements
/// priority bits [7:6]: IRQ #0 = 3 (lower priority), IRQ #1 = 1 (higher
/// priority). When both fire on the same `tick_peripherals`, both NVIC
/// pending bits set in lock-step; `try_take_any_pending_exception` picks
/// IRQ #1 first; on its return the tail-chain poll picks IRQ #0.
///
/// Each handler writes a distinct non-zero sentinel to `order_first_irq`
/// — but only if the cell is still zero. Whichever ran first wins. PASS
/// is `order_first_irq == 0xA1` (IRQ_1 sentinel).
#[test]
fn isr_m0_priority_preempt_passes_on_emu() {
    let sc = find_scenario("isr_m0_priority_preempt");
    match run_scenario(sc) {
        EmuOutcome::Completed(obs) => {
            // Per OBS_PRIORITY_PREEMPT ordering:
            //   obs[0] = order_first_irq (primary, load-bearing)
            //   obs[1] = ctr_irq_0
            //   obs[2] = ctr_irq_1
            assert_eq!(
                obs[0], 0xA1,
                "order_first_irq should be 0xA1 (IRQ_1 ran first because of higher priority), got 0x{:08X}",
                obs[0],
            );
            assert_eq!(
                obs[1], 1,
                "ctr_irq_0 should be 1 (IRQ_0 ran via tail-chain after IRQ_1), got {}",
                obs[1],
            );
            assert_eq!(
                obs[2], 1,
                "ctr_irq_1 should be 1 (IRQ_1 ran first), got {}",
                obs[2],
            );
        }
        EmuOutcome::HardFault { pc, ipsr } => {
            panic!(
                "isr_m0_priority_preempt: EMU hardfault at pc=0x{:08X} ipsr={}",
                pc, ipsr,
            );
        }
        EmuOutcome::Timeout => {
            panic!(
                "isr_m0_priority_preempt: cycle budget exhausted before order_first_irq advanced",
            );
        }
    }
}

/// V2 §3.2 — PRIMASK-gated pend then `cpsie i` unmask.
///
/// Verifies the PRIMASK gate inside `try_take_any_pending_exception`
/// (`rp2040_emu/src/core/mod.rs:331`) and that dispatch happens at the
/// `cpsie i` boundary, not later. Main pends + enables TIMER_IRQ_0 with
/// PRIMASK=1, stores `gate=1`, executes `cpsie i`, then attempts to
/// store `gate=2`. The handler reads `gate` into `gate_at_entry`. On a
/// correctly-implemented core the handler runs between the two main
/// stores, so `gate_at_entry == 1` and `gate == 2` after main resumes.
#[test]
fn isr_m0_masked_pending_unmask_passes_on_emu() {
    let sc = find_scenario("isr_m0_masked_pending_unmask");
    match run_scenario(sc) {
        EmuOutcome::Completed(obs) => {
            // Per OBS_MASKED ordering:
            //   obs[0] = gate_at_entry (primary, load-bearing)
            //   obs[1] = ctr_timer
            //   obs[2] = gate
            assert_eq!(
                obs[0], 1,
                "gate_at_entry should be 1 (handler ran between gate=1 and gate=2 stores), got {}",
                obs[0],
            );
            assert_eq!(
                obs[1], 1,
                "ctr_timer should be 1 after one TIMER ISR fire, got {}",
                obs[1],
            );
            assert_eq!(
                obs[2], 2,
                "gate should be 2 after main resumed past cpsie i, got {}",
                obs[2],
            );
        }
        EmuOutcome::HardFault { pc, ipsr } => {
            panic!(
                "isr_m0_masked_pending_unmask: EMU hardfault at pc=0x{:08X} ipsr={}",
                pc, ipsr,
            );
        }
        EmuOutcome::Timeout => {
            panic!(
                "isr_m0_masked_pending_unmask: cycle budget exhausted before gate_at_entry advanced",
            );
        }
    }
}
