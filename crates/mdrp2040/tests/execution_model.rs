//! Placeholder integration-test file for the dual-execution HLD V1.
//!
//! Stage 3b.1 lands the public types + CoreBus trait refactor; the
//! actual contract tests (mirror of `mdrp2350/tests/execution_model.rs`
//! — build_with_serial_succeeds, build_with_threaded_*, panic
//! containment) arrive with Stage 3b.4 once the builder wiring is in
//! place. This file exists so the `[[test]] name = "execution_model"`
//! entry in `Cargo.toml` resolves; gated by `threading + testing`.

#![cfg(all(feature = "threading", feature = "testing"))]
