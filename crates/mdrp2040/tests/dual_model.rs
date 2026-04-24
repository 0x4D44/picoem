//! Placeholder integration-test file for the dual-execution HLD V1.
//!
//! Stage 3b.1 lands the public types + CoreBus trait refactor; the
//! curated ~20-30 smoke tests that run identical scenarios on Serial
//! and Threaded and assert end-state equality (mirror of
//! `mdrp2350/tests/dual_model.rs`) arrive with Stage 4.1. This file
//! exists so the `[[test]] name = "dual_model"` entry in `Cargo.toml`
//! resolves; gated by `threading`.

#![cfg(feature = "threading")]
