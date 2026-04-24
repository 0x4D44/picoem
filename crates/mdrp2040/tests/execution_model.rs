//! Dual-execution HLD V1 Stage 3b.4 — TDD tests for the runtime
//! `ExecutionModel` selector and panic-containment wiring.
//!
//! Mirrors `mdrp2350/tests/execution_model.rs` but simpler: RP2040 has
//! only three worker variants (`Core0`, `Core1`, `Coord`), no PIO
//! worker (PIO runs on the coordinator in Stage 3b.4).
//!
//! Run with `cargo test -p mdrp2040 --features testing` — the
//! `testing` feature activates the panic-injection hooks used by
//! `worker_panic_surfaces_as_error` and
//! `threaded_placeholder_fields_panic_in_debug`.

#![cfg(all(feature = "threading", feature = "testing"))]

use mdrp2040::{Config, EmulatorBuilder, ExecutionModel};

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
    use mdrp2040::ConfigError;
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

/// Panic-injection contract: a worker panic in Threaded mode surfaces
/// as `EmulatorError::WorkerPanicked`, the panic message carries the
/// worker identifier, and subsequent calls are one-shot (return the
/// cached error without re-entering worker threads).
#[cfg(all(target_arch = "x86_64", target_os = "windows", feature = "threading"))]
#[test]
fn worker_panic_surfaces_as_error() {
    use mdrp2040::{EmulatorError, WorkerName};

    let mut emu = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build()
        .expect("Threaded build should succeed");

    emu.inject_panic_for_testing(WorkerName::Core0);

    let first = emu.run_quantum();
    match first {
        Err(EmulatorError::WorkerPanicked {
            ref which,
            ref message,
        }) => {
            assert_eq!(*which, WorkerName::Core0, "panic must be attributed to core0");
            assert!(
                message.contains("core0"),
                "panic message must name the worker: got {message:?}"
            );
        }
        other => panic!(
            "expected Err(EmulatorError::WorkerPanicked), got {other:?}"
        ),
    }

    // One-shot guarantee: the next call must return the SAME error
    // without re-attempting workers.
    let second = emu.run_quantum();
    match second {
        Err(EmulatorError::WorkerPanicked {
            ref which,
            ref message,
        }) => {
            assert_eq!(*which, WorkerName::Core0);
            assert!(message.contains("core0"));
        }
        other => panic!(
            "one-shot: second call must also return WorkerPanicked, got {other:?}"
        ),
    }
}

/// Serial parity for `run_quantum` — a single `run_quantum()` must
/// consume the same cycle budget as `run(step_quantum)` on Serial.
/// Locks the HLD V1 §5.4 parity row.
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

    let q = b.step_quantum as u64;
    let a_cycles = a.run_quantum().expect("Serial run_quantum is infallible");
    let b_cycles = b.run(q).expect("Serial run is infallible");
    assert_eq!(
        a_cycles, b_cycles,
        "HLD §5.4 parity: run_quantum() must equal run(step_quantum) on Serial",
    );
}

/// `step()` is a Serial-only entry point; on a Threaded emulator it
/// must return `Err(EmulatorError::NotSupportedInThreadedMode)`.
#[cfg(all(target_arch = "x86_64", target_os = "windows", feature = "threading"))]
#[test]
fn threaded_step_returns_not_supported() {
    use mdrp2040::EmulatorError;

    let mut emu = EmulatorBuilder::new(Config::default())
        .execution(ExecutionModel::Threaded)
        .build()
        .expect("Threaded build should succeed");

    match emu.step() {
        Err(EmulatorError::NotSupportedInThreadedMode) => {}
        other => panic!(
            "Threaded step() must return NotSupportedInThreadedMode, got {other:?}"
        ),
    }
}

/// Placeholder-guard contract: after `promote_to_threaded` fires
/// lazily on the first `run_quantum`, the top-level `cores` / `bus` /
/// `clock` fields hold zero-cost placeholders. Typed accessors carry
/// a `debug_assert!` that fires in this state — debug builds only.
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
    // fields become placeholders.
    let _ = emu
        .run_quantum()
        .expect("initial run_quantum should succeed");

    // Now a guarded accessor must fire the debug-assert.
    let _ = emu.core_mut(0);
}
