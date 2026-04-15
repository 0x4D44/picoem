//! RP2040 DMA controller — Phase 1 stub.
//!
//! Phase 4 of the RP2040 peripheral coverage plan (HLD V7 §5.6) lands
//! the real 12-channel DMA with ring / chain / DREQ support. Phase 1
//! carries this trivial stub so the fast-path gate in
//! [`crate::Emulator::step`] can consult [`Dma::is_idle`] without
//! forward-referencing a module that doesn't exist yet.
//!
//! The stub is always idle. When Phase 4 replaces this with real state,
//! [`Dma::is_idle`] grows into "no channel has `BUSY == 1`" and the
//! slow-path will wake up whenever a DMA channel is live.

/// DMA controller state — Phase 1 empty stub. Phase 4 replaces this
/// with the full 12-channel model.
#[doc = "Phase 4 will implement the real 12-channel DMA. \
         Today this is an always-idle placeholder."]
#[derive(Default)]
pub struct Dma;

impl Dma {
    /// Construct an idle DMA controller.
    pub fn new() -> Self {
        Self
    }

    /// True iff no channel has `BUSY == 1`. Phase 1: always true
    /// (no channels exist yet).
    #[inline]
    pub fn is_idle(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_dma_is_always_idle() {
        assert!(Dma::new().is_idle());
        assert!(Dma::default().is_idle());
    }
}
