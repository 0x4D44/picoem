use std::time::Duration;

use mdrp2354::Emulator;

use crate::snapshot::BenchmarkReport;

#[derive(Default)]
pub struct BenchmarkPoller {}

impl BenchmarkPoller {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn poll(&mut self, _emu: &Emulator, _wall: Duration) {
        // Phase 3 will implement the sentinel poller per LLD §8.2.
    }

    pub fn report(&self) -> Option<BenchmarkReport> {
        None
    }
}
