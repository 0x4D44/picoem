use crate::snapshot::LcdState;

#[derive(Default)]
pub struct LcdDecoder {
    state: LcdState,
}

impl LcdDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn sample(&mut self, _gpio_out: u32) {
        // Phase 2 will implement the bit-bang decoder per LLD §7.2.
    }

    pub fn state(&self) -> LcdState {
        self.state.clone()
    }
}
