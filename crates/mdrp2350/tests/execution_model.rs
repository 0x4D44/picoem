//! Dual-execution HLD V1 Stage 1b — TDD tests for the runtime
//! `ExecutionModel` selector and panic-containment wiring.
//!
//! Run with `cargo test -p mdrp2350 --features testing` — the
//! `testing` feature activates the panic-injection hooks used by
//! `worker_panic_surfaces_as_error` and
//! `threaded_placeholder_fields_panic_in_debug`. Without it the
//! test binary is skipped via `required-features` in
//! `crates/mdrp2350/Cargo.toml`.
//!
//! Seven behaviours under test:
//!   1. `Serial` builds succeed on every host.
//!   2. `Threaded` builds succeed on x86_64 Windows with the
//!      `threading` feature on.
//!   3. `Threaded` builds return `ConfigError::ThreadingUnavailable`
//!      when the `threading` feature is off.
//!   4. A worker panic in Threaded mode surfaces as
//!      `EmulatorError::WorkerPanicked` and further calls on the
//!      `Emulator` are one-shot — they return the same error without
//!      re-attempting workers.
//!   5. `run_quantum` on Serial returns the same cycle total as
//!      `run(step_quantum)` — locks the HLD §5.4 parity row for
//!      `run_quantum`.
//!   6. `step()` on a Threaded emulator returns
//!      `Err(EmulatorError::NotSupportedInThreadedMode)` — locks the
//!      HLD §5.4 row 1.
//!   7. After promotion to Threaded the guarded accessors
//!      (`core_mut`, …) fire a `debug_assert!` naming the
//!      Serial-only contract — locks the Stage 1b review REQUIRED #1
//!      fix.

use mdrp2350::{Config, EmulatorBuilder, ExecutionModel};

#[test]
fn build_with_serial_succeeds() {
    let result = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Serial)
        .build();
    assert!(
        result.is_ok(),
        "Serial build must succeed: {:?}",
        result.err()
    );
}

#[cfg(all(target_arch = "x86_64", target_os = "windows", feature = "threading"))]
#[test]
fn build_with_threaded_succeeds_on_supported_platform() {
    let result = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build();
    assert!(
        result.is_ok(),
        "Threaded build must succeed on x86_64 Windows with `threading` feature: {:?}",
        result.err()
    );
}

#[cfg(not(feature = "threading"))]
#[test]
fn build_with_threaded_returns_err_when_feature_off() {
    use mdrp2350::ConfigError;
    let result = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build();
    match result {
        Err(ConfigError::ThreadingUnavailable) => {}
        other => panic!(
            "expected Err(ConfigError::ThreadingUnavailable), got {:?}",
            other.map(|_| "<Emulator>")
        ),
    }
}

#[cfg(all(target_arch = "x86_64", target_os = "windows", feature = "threading"))]
#[test]
fn worker_panic_surfaces_as_error() {
    use mdrp2350::{EmulatorError, threaded::WorkerName};

    let mut emu = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build()
        .expect("Threaded build should succeed");

    // Arm the test-only panic injector: the next quantum's PIO0 worker
    // will fire `PioCommand::TestPanic` and panic with `pio0` in the
    // message. Both CPU cores are halted so core workers complete
    // cleanly; only the PIO worker panics.
    emu.inject_panic_for_testing(WorkerName::Pio0);

    let first = emu.run_quantum();
    match first {
        Err(EmulatorError::WorkerPanicked {
            ref which,
            ref message,
        }) => {
            assert_eq!(*which, WorkerName::Pio0, "panic must be attributed to pio0");
            assert!(
                message.contains("pio0"),
                "panic message must name the worker: got {message:?}"
            );
        }
        other => panic!("expected Err(EmulatorError::WorkerPanicked), got {other:?}"),
    }

    // One-shot guarantee: the next call must return the SAME error
    // without re-attempting workers. (The previous `run_quantum` call
    // consumed the core state, so re-entry into a live worker would
    // panic on `Option::take`; a correctly one-shot implementation
    // short-circuits before that.)
    let second = emu.run_quantum();
    match second {
        Err(EmulatorError::WorkerPanicked {
            ref which,
            ref message,
        }) => {
            assert_eq!(*which, WorkerName::Pio0);
            assert!(message.contains("pio0"));
        }
        other => panic!("one-shot: second call must also return WorkerPanicked, got {other:?}"),
    }
}

/// Stage 1b review REQUIRED #3 test (5): `run_quantum()` on Serial
/// must consume the same cycle budget as `run(step_quantum)` so
/// callers that standardise on the §5.4 entry point (Threaded's
/// primary) get the same Serial result as the old `run(cycles)` path.
/// Locks the HLD V1 §5.4 parity row for `run_quantum`.
#[test]
fn serial_step_quantum_matches_run_step_quantum() {
    let mut a = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Serial)
        .build()
        .expect("Serial build is infallible");
    let mut b = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Serial)
        .build()
        .expect("Serial build is infallible");

    // The two emulators are constructed identically — same step_quantum,
    // same Config, fresh clocks at 0. A single `run_quantum()` call
    // must land `a` on the same master cycle that `run(step_quantum)`
    // lands `b` on.
    let q = b.step_quantum as u64;
    let a_cycles = a.run_quantum().expect("Serial run_quantum is infallible");
    let b_cycles = b.run(q).expect("Serial run is infallible");
    assert_eq!(
        a_cycles, b_cycles,
        "HLD §5.4 parity: run_quantum() must equal run(step_quantum) on Serial",
    );
}

/// Stage 1b review REQUIRED #3 test (6): `step()` is a Serial-only
/// entry point. On a Threaded emulator it must return
/// `Err(EmulatorError::NotSupportedInThreadedMode)`. Locks HLD §5.4
/// row 1.
#[cfg(all(target_arch = "x86_64", target_os = "windows", feature = "threading"))]
#[test]
fn threaded_step_returns_not_supported() {
    use mdrp2350::EmulatorError;

    let mut emu = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build()
        .expect("Threaded build should succeed");

    match emu.step() {
        Err(EmulatorError::NotSupportedInThreadedMode) => {}
        other => panic!("Threaded step() must return NotSupportedInThreadedMode, got {other:?}"),
    }
}

/// Stage 1b review REQUIRED #1 contract: after `promote_to_threaded`
/// fires (lazily on first `run_quantum`), the top-level `cores` /
/// `bus` / `clock` fields hold zero-cost placeholders. Typed
/// accessors (`core_mut`, `cycles`, `peek`, …) carry a
/// `debug_assert!` that fires when called in this state — in debug
/// builds only. Release builds elide the assertion entirely, which is
/// intentional (`tech_debt.md` entry describes the full contract).
///
/// This test therefore only runs under `debug_assertions`; on
/// `cargo test --release` it is compiled out. The expected-substring
/// matches the shared `Self::PLACEHOLDER_GUARD_MSG` prefix in
/// `crates/mdrp2350/src/lib.rs`.
#[cfg(all(
    debug_assertions,
    target_arch = "x86_64",
    target_os = "windows",
    feature = "threading"
))]
#[test]
#[should_panic(expected = "Serial-only")]
fn threaded_placeholder_fields_panic_in_debug() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build()
        .expect("Threaded build should succeed");

    // Drive one quantum so `promote_to_threaded` runs and the flat
    // `cores` / `bus` / `clock` fields become placeholders. (A fresh
    // Threaded emulator leaves them as authoritative pre-promotion
    // state to support pre-run harness setup — see HLD §5.4 Notes for
    // the `cores().read_register(r)` row.)
    let _ = emu
        .run_quantum()
        .expect("initial run_quantum should succeed");

    // Now a guarded accessor must fire the debug-assert. `core_mut`
    // is the simplest — it returns a direct reference into the
    // placeholder `cores` array.
    let _ = emu.core_mut(0);
}
