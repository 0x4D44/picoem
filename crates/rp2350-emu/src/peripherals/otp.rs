//! RP2350 OTP — one-time-programmable fuse array, HLD V5 §7.D.4.
//!
//! Models the 16 KB `OTP_DATA` aperture as a flat 4096-word backing
//! store with OR-only write semantics (fuse bits can only burn from
//! 0 → 1, never cleared). Reads return the stored word directly.
//!
//! # Intentional simplifications (HLD V5 §7.D.4)
//!
//! - **No SBPI sequencing.** Firmware that drives the real SBPI
//!   controller at `OTP_BASE = 0x4012_0000` lands in the Bus
//!   HashMap fallthrough — step-1 warn-once catches it.
//! - **No row-lockout / ECC / guarded rows.** Every word is writable.
//! - **No READY bit / STATUS register.** The aperture is always
//!   implicitly ready; writes complete synchronously.
//! - **No fuse-load builder.** Initial state is all zeros (blank
//!   OTP). V3 explicitly cut the `with_fuses(...)` constructor.
//! - **Survives reset.** OTP fuses are physically persistent silicon
//!   state; [`Emulator::reset`](crate::Emulator::reset) does not
//!   re-zero this struct.
//!
//! # Base address
//!
//! `OTP_DATA_BASE = 0x4013_0000` — RP2350 datasheet §11 (OTP). This
//! is the **data aperture**, distinct from the SBPI control block at
//! `OTP_BASE = 0x4012_0000` (confirmed against `one-rom` `reg-rp235x.h`).
//! The aperture is 16 KB (4096 × u32). If silicon uses a different
//! aperture base, the Bus dispatch warn-once will flag firmware hits.

use tracing::debug;

/// OTP data aperture base. Datasheet §11. See module doc.
pub const OTP_DATA_BASE: u32 = 0x4013_0000;
/// Size of the OTP data aperture in bytes (16 KB).
pub const OTP_DATA_SIZE: u32 = 16 * 1024;
/// Number of u32 words in the OTP data aperture.
pub const OTP_WORD_COUNT: usize = (OTP_DATA_SIZE / 4) as usize;

/// OTP fuse array — 16 KB of OR-only backing storage.
pub struct Otp {
    /// 4096 words. Boxed to avoid bloating the `Bus` struct.
    storage: Box<[u32; OTP_WORD_COUNT]>,
}

impl Otp {
    pub fn new() -> Self {
        Self {
            storage: Box::new([0u32; OTP_WORD_COUNT]),
        }
    }

    /// Read a word. `offset` is bytes from [`OTP_DATA_BASE`].
    /// Out-of-range reads return 0.
    pub fn read32(&self, offset: u32) -> u32 {
        let idx = (offset >> 2) as usize;
        if idx < OTP_WORD_COUNT {
            self.storage[idx]
        } else {
            0
        }
    }

    /// Write a word with OR-only fuse semantics: `storage[i] |= value`.
    /// A bit that is already set (`1`) cannot be cleared by this write.
    /// Ignores APB alias bits — fuse semantics override any notion of
    /// XOR / CLR; only the data word is relevant.
    pub fn write32(&mut self, offset: u32, value: u32) {
        let idx = (offset >> 2) as usize;
        if idx < OTP_WORD_COUNT {
            let before = self.storage[idx];
            self.storage[idx] |= value;
            if self.storage[idx] != before {
                debug!(
                    target: "rp2350_emu::otp",
                    offset = format_args!("{:#X}", offset),
                    before = format_args!("{:#010X}", before),
                    after = format_args!("{:#010X}", self.storage[idx]),
                    "OTP fuse bits burned",
                );
            }
        }
    }
}

impl Default for Otp {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_state_is_zero() {
        let otp = Otp::new();
        for off in (0..OTP_DATA_SIZE).step_by(4) {
            assert_eq!(
                otp.read32(off),
                0,
                "OTP word at {:#X} must be 0 at reset",
                off
            );
        }
    }

    #[test]
    fn or_write_accumulates_bits() {
        let mut otp = Otp::new();
        otp.write32(0x20, 0x0000_000F);
        assert_eq!(otp.read32(0x20), 0x0000_000F);
        otp.write32(0x20, 0x0000_00F0);
        assert_eq!(otp.read32(0x20), 0x0000_00FF);
        otp.write32(0x20, 0xF000_0000);
        assert_eq!(otp.read32(0x20), 0xF000_00FF);
    }

    #[test]
    fn rewrite_cannot_clear_bit() {
        let mut otp = Otp::new();
        otp.write32(0x40, 0xFFFF_FFFF);
        assert_eq!(otp.read32(0x40), 0xFFFF_FFFF);
        // Attempt to clear by writing all zeros — OR-semantics is a no-op.
        otp.write32(0x40, 0x0000_0000);
        assert_eq!(otp.read32(0x40), 0xFFFF_FFFF);
        // Attempt to clear specific bits by writing their inverse — still no-op.
        otp.write32(0x40, 0x0000_0000);
        assert_eq!(otp.read32(0x40), 0xFFFF_FFFF);
    }
}
