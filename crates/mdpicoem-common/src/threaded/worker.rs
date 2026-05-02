//! Worker-thread spawn / pin / panic-payload helpers shared by both
//! chip emulators' threaded runtimes.
//!
//! Promoted from `mdrp2350::threaded::emulator` and
//! `mdrp2040::threaded::emulator` per the 2026-04-30 Threaded Helpers
//! Pull-Up HLD V1 — three byte-identical helpers (`panic_message`,
//! `spawn_worker`, `pin_to_host_core`) collapsed into one copy. Function
//! bodies are character-preserving moves; the richer RP2350 doc
//! comments were adopted as the canonical version.
//!
//! Per-OS gating mirrors the chip-side `threaded::emulator` modules:
//! `pin_to_host_core` requires Windows (`SetThreadAffinityMask`) or
//! Linux (`pthread_setaffinity_np`). Other UNIX hosts (macOS, FreeBSD,
//! …) stay on the serial `Emulator::run` path until `pin_to_host_core`
//! grows a port.

use std::any::Any;
use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::thread::{self, JoinHandle};

use super::SpinBarrier;

/// Extract a human-readable message from a `JoinHandle::join()` Err
/// payload. Falls back to a fixed string if the payload is neither a
/// `String` nor a `&'static str` — matches the downcast pattern used in
/// the in-crate panic-assertion tests.
pub fn panic_message(err: Option<&Box<dyn Any + Send>>) -> String {
    match err {
        Some(payload) => payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| {
                payload
                    .downcast_ref::<&'static str>()
                    .map(|s| s.to_string())
            })
            .unwrap_or_else(|| "<non-string panic payload>".to_string()),
        None => String::new(),
    }
}

/// Spawn a worker thread pinned to `host_core` running `body`. Catches
/// panics from `body` and poisons the shared barrier before re-raising
/// the panic so the remaining workers drop out of their spin loops.
///
/// Generic over the body's return type so the three different body
/// signatures (`CortexM33` / `PioBlock` / `()`) share the same spawn
/// path without a trait object.
pub fn spawn_worker<F, R>(host_core: usize, barrier: Arc<SpinBarrier>, body: F) -> JoinHandle<R>
where
    F: FnOnce(Arc<SpinBarrier>) -> R + Send + 'static,
    R: Send + 'static,
{
    thread::spawn(move || {
        pin_to_host_core(host_core);
        let b_for_body = barrier.clone();
        match std::panic::catch_unwind(AssertUnwindSafe(move || body(b_for_body))) {
            Ok(r) => r,
            Err(payload) => {
                barrier.poison();
                std::panic::resume_unwind(payload);
            }
        }
    })
}

/// Pin the current thread to the supplied host logical-CPU id.
///
/// Per-OS backends:
/// - Windows: `SetThreadAffinityMask` (single-bit mask).
/// - Linux: `pthread_setaffinity_np` with a one-bit `cpu_set_t`.
///
/// The whole module is gated to those two operating systems, so the
/// `cfg` arms below are exhaustive on supported builds.
pub fn pin_to_host_core(host_core: usize) {
    assert!(
        host_core < usize::BITS as usize,
        "host_core {host_core} exceeds processor-mask bit width"
    );
    #[cfg(target_os = "windows")]
    {
        use winapi::um::processthreadsapi::GetCurrentThread;
        use winapi::um::winbase::SetThreadAffinityMask;
        // SAFETY: `GetCurrentThread` is a pure FFI call that returns a
        // pseudo-handle to the calling thread. It takes no arguments,
        // never fails, and the handle does not need to be closed.
        let h = unsafe { GetCurrentThread() };
        let mask = 1usize << host_core;
        // SAFETY: `h` is the valid pseudo-handle just returned by
        // `GetCurrentThread`. `mask` is non-zero because the
        // `host_core < usize::BITS` precondition asserted above bounds
        // the shift, so `1 << host_core` cannot wrap to 0 (an all-zero
        // mask is the only invalid value `SetThreadAffinityMask`
        // rejects). FFI signature otherwise has no further preconditions.
        let prev = unsafe { SetThreadAffinityMask(h, mask) };
        assert!(
            prev != 0,
            "SetThreadAffinityMask failed for host core {host_core}"
        );
    }
    #[cfg(target_os = "linux")]
    {
        // SAFETY: `cpu_set_t` is a fixed-size POD bitset (a `[u64; 16]`
        // wrapper on glibc x86_64) for which the all-zero bit pattern
        // is valid and equivalent to `CPU_ZERO`.
        let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
        // SAFETY: `&mut set` is a valid, aligned, exclusive pointer to
        // a fully-initialized `cpu_set_t`. `host_core` is bounded by
        // the `host_core < usize::BITS` precondition (≤ 63 on 64-bit),
        // well below `CPU_SETSIZE` (1024 on glibc), so `CPU_SET` will
        // not write out of bounds.
        unsafe {
            libc::CPU_ZERO(&mut set);
            libc::CPU_SET(host_core, &mut set);
        }
        // SAFETY: `pthread_self()` always returns the calling thread's
        // valid pthread handle. `cpusetsize` matches the layout of the
        // `cpu_set_t` we just initialized, and `&set` is a valid
        // pointer to that initialized value for the duration of the
        // call. The kernel only reads through the pointer.
        let rc = unsafe {
            libc::pthread_setaffinity_np(
                libc::pthread_self(),
                std::mem::size_of::<libc::cpu_set_t>(),
                &set,
            )
        };
        assert!(
            rc == 0,
            "pthread_setaffinity_np failed for host core {host_core}: errno={rc}"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `panic_message` extracts the payload from `JoinHandle::join`'s
    /// `Err` arm. Cover all three arms: `String`, `&'static str`, and
    /// the `<non-string panic payload>` fallback. Also covers `None`
    /// (Ok join — empty string).
    #[test]
    fn panic_message_extracts_all_payload_kinds() {
        // Build payloads that mimic what `JoinHandle::join` returns.
        let s_payload: Box<dyn std::any::Any + Send> = Box::new(String::from("string panic"));
        assert_eq!(panic_message(Some(&s_payload)), "string panic");

        let static_payload: Box<dyn std::any::Any + Send> = Box::new("static-str panic");
        assert_eq!(panic_message(Some(&static_payload)), "static-str panic");

        let other_payload: Box<dyn std::any::Any + Send> = Box::new(42u64);
        assert_eq!(
            panic_message(Some(&other_payload)),
            "<non-string panic payload>"
        );

        assert_eq!(panic_message(None), "");
    }
}
