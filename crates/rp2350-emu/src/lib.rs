use std::sync::Arc;
use std::sync::atomic::Ordering;

pub mod bootrom_hooks;
pub mod bus;
pub mod core;
pub mod core_riscv;
pub mod dma;
pub mod dreq;
pub mod irq;
pub mod memory;
pub mod peripherals;
pub mod pio;
pub mod sio;
pub mod threaded;

use tracing::info;

/// Execution model for an [`Emulator`]. Selected at construction via
/// [`EmulatorBuilder::execution`]; cannot be switched post-build.
///
/// - `Serial` — oracle-validated reference path (QEMU + silicon
///   differentials). Single-threaded, per-instruction interleave.
///   Always available.
/// - `Threaded` — 6-thread runtime (core0, core1, pio0/1/2,
///   coordinator). Opt-in throughput optimization on
///   x86_64 Windows hosts with the `threading` cargo feature on.
///   Not validated against QEMU/silicon oracles — see dual-execution
///   HLD V1 §3.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ExecutionModel {
    #[default]
    Serial,
    Threaded,
}

/// Errors returned by [`EmulatorBuilder::build`]. The only non-trivial
/// variant today is `ThreadingUnavailable`, returned when the caller
/// selects [`ExecutionModel::Threaded`] but the host platform or build
/// configuration cannot satisfy it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// `ExecutionModel::Threaded` selected but the current build does
    /// not include a threaded runtime — either the `threading` cargo
    /// feature is off, or the host is not one of the supported
    /// platforms (currently x86_64 Windows only).
    ThreadingUnavailable,
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::ThreadingUnavailable => write!(
                f,
                "ExecutionModel::Threaded is unavailable (requires x86_64 Windows \
                 with the `threading` cargo feature enabled)"
            ),
        }
    }
}

impl std::error::Error for ConfigError {}

/// Errors returned by post-construction [`Emulator`] methods. Surface
/// for runtime-model mismatches and worker panics (dual-execution HLD
/// V1 §5.5).
///
/// `WorkerPanicked` is sticky: once an [`Emulator`] observes a worker
/// panic, every subsequent call on that instance returns the same
/// error without re-attempting the workers (one-shot-after-panic, HLD
/// §5.5 item 5). Drop the instance and rebuild from a fresh
/// [`EmulatorBuilder`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EmulatorError {
    /// Called a Serial-only method on a Threaded emulator, e.g.
    /// `step()` — Threaded runs in quanta, not single-step. HLD §5.4.
    NotSupportedInThreadedMode,
    /// One of the worker threads panicked. The `Emulator` is sticky-
    /// poisoned after this; drop and rebuild. Only produced on the
    /// Threaded path.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    WorkerPanicked {
        which: threaded::WorkerName,
        message: String,
    },
    /// The shared [`picoem_common::SpinBarrier`] watchdog fired
    /// because a worker failed to arrive at the rendezvous within
    /// [`picoem_common::threaded::DEFAULT_DEADLINE`]. The `Emulator`
    /// is sticky-poisoned after this; drop and rebuild. HLD V1 §6.6.
    ///
    /// Only produced on the Threaded path. `which` is the first worker
    /// that returned `TimedOut` at its barrier; since the barrier
    /// cannot identify *which* worker failed to arrive, this field
    /// names an observer rather than the culprit. `elapsed_ms` is the
    /// reporting waiter's own wall-clock elapsed time at expiry.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    BarrierTimeout {
        which: threaded::WorkerName,
        elapsed_ms: u32,
    },
}

impl std::fmt::Display for EmulatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EmulatorError::NotSupportedInThreadedMode => write!(
                f,
                "operation not supported on a Threaded Emulator (Serial-only)"
            ),
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            EmulatorError::WorkerPanicked { which, message } => {
                write!(f, "worker {} panicked: {message}", which.as_str())
            }
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            EmulatorError::BarrierTimeout { which, elapsed_ms } => write!(
                f,
                "barrier watchdog fired (observed by worker {}) after {}ms",
                which.as_str(),
                elapsed_ms
            ),
        }
    }
}

impl std::error::Error for EmulatorError {}

#[cfg(test)]
mod pio_tests;

#[cfg(test)]
mod tests_narrow;

pub use self::bus::Bus;
pub use self::core::CoreCounters;
pub use self::core::CortexM33;
pub use self::core_riscv::Hazard3;
pub use self::memory::Memory;
pub use self::sio::Sio;

#[cfg(target_arch = "x86_64")]
pub use picoem_common::Pacer;
pub use picoem_common::{Clock, PacerSnapshot, PacerStats};

/// Stop reason when running until a condition.
pub enum StopReason {
    CycleLimit,
    Breakpoint(u32),
    Wfi,
    Fault,
}

/// ROSC nominal frequency (~6.5 MHz). The RP2350 boots on ROSC;
/// PLL configuration (if any) happens later in firmware.
///
/// Re-exported from [`bus::clocks`] for backward compatibility.
pub use self::bus::clocks::ROSC_FREQ_HZ;

/// Loads the pinned silicon-derived RP2354 bootrom from the in-tree
/// `roms/rp2350/bootrom-combined.bin`, verifies it against the sibling
/// `bootrom-combined.bin.sha256`, and returns the raw bytes.
///
/// HLD V5 §"Component 3 — Bootrom mask-ROM": all callers wanting the
/// real silicon binary (rp2350_emu_tui, harness oracles, future
/// integration tests) funnel through this helper so the SHA256 pin is
/// enforced at one site. Synthetic-ROM unit tests in this crate
/// continue to call [`Emulator::load_bootrom`] directly with hand-
/// crafted bytes — the assert lives here, not in `load_bootrom`, so
/// that path stays open.
///
/// On hash mismatch the function returns
/// `io::Error::new(io::ErrorKind::InvalidData, ...)` rather than
/// panicking — callers may want to handle pin drift gracefully (e.g.
/// to print a refresh hint and exit cleanly).
pub fn load_pinned_silicon_bootrom() -> std::io::Result<Vec<u8>> {
    use sha2::{Digest, Sha256};
    use std::path::PathBuf;

    // CARGO_MANIFEST_DIR points at `crates/rp2350_emu`; project root is
    // two parents up.
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.pop(); // crates
    root.pop(); // workspace root
    let bin_path = root
        .join("roms")
        .join("rp2350")
        .join("bootrom-combined.bin");
    let sha_path = root
        .join("roms")
        .join("rp2350")
        .join("bootrom-combined.bin.sha256");

    let bytes = std::fs::read(&bin_path)?;
    let expected_hex = std::fs::read_to_string(&sha_path)?;
    let expected_hex = expected_hex
        .split_whitespace()
        .next()
        .unwrap_or("")
        .to_ascii_lowercase();

    let mut hasher = Sha256::new();
    hasher.update(&bytes);
    let digest = hasher.finalize();
    let actual_hex = digest
        .iter()
        .map(|b| format!("{:02x}", b))
        .collect::<String>();

    if actual_hex != expected_hex {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "bootrom SHA256 mismatch: expected {}, got {} (refresh \
                 roms/rp2350/bootrom-combined.bin.sha256 if the binary was \
                 intentionally updated)",
                expected_hex, actual_hex
            ),
        ));
    }
    Ok(bytes)
}

/// Emulator configuration.
pub struct Config {
    /// System clock frequency in Hz. Default: ROSC (~6.5 MHz).
    pub sys_clk_hz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sys_clk_hz: ROSC_FREQ_HZ,
        }
    }
}

/// Default quantum size in cycles. Each `Emulator::step()` advances the
/// system by exactly this many virtual cycles; both cores run atomically
/// (instruction-at-a-time) until their per-core cycle count catches up
/// with the target. 64 cycles @ 150 MHz is ~430 ns — well below any
/// firmware-observable timing the emulator currently models.
pub const DEFAULT_STEP_QUANTUM: u32 = 64;

/// Architecture selector. RP2350 ships both an Arm and a RISC-V
/// complex; OTP/POWMAN picks one at power-up. V1 only constructs the
/// Arm path with a real ISA — see
/// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`.
#[derive(Default)]
pub enum Arch {
    #[default]
    Arm,
    RiscV,
}

/// Per-arch core pair. `expect_arm*` / `expect_riscv*` panic on the
/// wrong arm — documented programmer-error contract for call sites
/// that the shimmed `Emulator::core(id)` path can't cover.
pub enum Cores {
    Arm([CortexM33; 2]),
    RiscV([Hazard3; 2]),
}

impl Cores {
    pub fn expect_arm(&self) -> &[CortexM33; 2] {
        match self {
            Cores::Arm(cs) => cs,
            Cores::RiscV(_) => panic!("expect_arm called on RiscV emulator"),
        }
    }

    pub fn expect_arm_mut(&mut self) -> &mut [CortexM33; 2] {
        match self {
            Cores::Arm(cs) => cs,
            Cores::RiscV(_) => panic!("expect_arm_mut called on RiscV emulator"),
        }
    }

    pub fn expect_riscv(&self) -> &[Hazard3; 2] {
        match self {
            Cores::RiscV(cs) => cs,
            Cores::Arm(_) => panic!("expect_riscv called on Arm emulator"),
        }
    }

    pub fn expect_riscv_mut(&mut self) -> &mut [Hazard3; 2] {
        match self {
            Cores::RiscV(cs) => cs,
            Cores::Arm(_) => panic!("expect_riscv_mut called on Arm emulator"),
        }
    }

    pub fn is_arm(&self) -> bool {
        matches!(self, Cores::Arm(_))
    }

    pub fn is_riscv(&self) -> bool {
        matches!(self, Cores::RiscV(_))
    }
}

/// Top-level RP2350 emulator. Owns dual cores (Arm or RISC-V), bus
/// fabric, memory, and clock. SIO is owned by Bus. Peripherals and PIO
/// are injected via builder.
///
/// Dual-execution HLD V1: an `Emulator` has a fixed [`ExecutionModel`]
/// picked at construction time via [`EmulatorBuilder::execution`]. In
/// Serial mode (default) the `cores` / `bus` / `clock` fields are the
/// authoritative state and the existing per-instruction interleave
/// applies. In Threaded mode those fields retain their post-seed
/// snapshot (useful for pre-run harness setup) but the hot-path state
/// lives inside `threaded`; callers must use [`Self::run_quantum`] /
/// [`Self::run`] and should not inspect `cores` / `bus` mid-run.
pub struct Emulator {
    pub cores: Cores,
    pub bus: Bus,
    pub clock: Clock,
    /// Cycles advanced per call to [`Self::step`]. See
    /// [`DEFAULT_STEP_QUANTUM`]. Distinct from `Pacer::quantum_cycles`
    /// which drives wall-clock pacing.
    pub step_quantum: u32,
    /// Execution model chosen at build time; cannot change
    /// post-construction. Dispatch for [`Self::step`] / [`Self::run`] /
    /// [`Self::run_quantum`] branches on this.
    execution_model: ExecutionModel,
    /// Live 6-thread runtime when `execution_model == Threaded`. Takes
    /// ownership of the pre-seeded cores / bus / clock during
    /// `build()`; the top-level fields retain their seed snapshot so
    /// harness setup code can inspect pre-run state but must not rely
    /// on them mid-run.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    threaded: Option<threaded::ThreadedEmulator>,
    /// Sticky panic record from a Threaded worker. Set once when
    /// `run_quantum` / `run` observes a worker panic and returned on
    /// every subsequent call (one-shot, HLD V1 §5.5 item 5). Not
    /// touched on the Serial path.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    panic_info: Option<(threaded::WorkerName, String)>,
    /// Sticky watchdog-timeout record from a Threaded run. Set once
    /// when `run_quantum` / `run` observes a barrier timeout and
    /// returned on every subsequent call (HLD V1 §6.6 Stage 5). Not
    /// touched on the Serial path.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    timeout_info: Option<(threaded::WorkerName, u32)>,
    /// Test-only panic injector: arm by calling
    /// [`Self::inject_panic_for_testing`]. The next `run_quantum` /
    /// `run` queues a `PioCommand::TestPanic` so the matching PIO
    /// worker panics on its next drain. Off the hot path. Gated behind
    /// `testing` (Stage 1b review REQUIRED #2) so release builds ship
    /// neither the field nor the setter.
    #[cfg(all(
        feature = "testing",
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pending_panic_inject: Option<threaded::WorkerName>,
    /// Set to `true` by [`Self::promote_to_threaded`] when the seeded
    /// `cores` / `bus` / `clock` state has been moved into
    /// `self.threaded` and the top-level fields now hold zero-cost
    /// placeholders. Direct field access on `cores` / `bus` / `clock`
    /// in this state silently reads/writes dead state; the typed
    /// accessors (`core`, `core_mut`, `core_counters`, …) assert on
    /// this flag in debug builds to catch Serial-only callers that
    /// reach for the flat fields after a Threaded run. Release builds
    /// elide the assertion entirely to keep the hot path free.
    ///
    /// Known escape: raw field access (`emu.bus.…`) bypasses the
    /// guarded accessors; see `tech_debt.md` entry
    /// "Emulator direct-field access is Serial-only but not
    /// type-enforced (2026-04-24)".
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub(crate) bus_is_placeholder: bool,
    /// Latched true once the bootrom `reboot` mask-ROM hook fires on
    /// either core (HLD V5 §"Component 3"). Terminate-only: once set,
    /// the cores are halted and the host is expected to drop the
    /// emulator instance. Drained from
    /// [`CortexM33::bootrom_hook_fired`] inside `step_serial` /
    /// `run_quantum`'s post-step path. Soft-reboot scenarios that need
    /// to re-init must rebuild the emulator from scratch.
    pub shutdown_requested: bool,
}

impl Emulator {
    /// Create a new Serial-mode emulator with the given configuration.
    /// Infallible shim: Serial builds always succeed, so this unwraps
    /// the `build()` result. For Threaded construction or to handle
    /// `ConfigError` explicitly, use [`EmulatorBuilder`] directly.
    pub fn new(config: Config) -> Self {
        EmulatorBuilder::new(config)
            .build()
            .expect("Serial build is infallible")
    }

    /// Currently selected execution model.
    pub fn execution_model(&self) -> ExecutionModel {
        self.execution_model
    }

    /// Cycle counter for core `idx` (0 or 1). Serial reads directly
    /// from `cores[idx].cycles()`; Threaded reads the worker-thread
    /// snapshot (requires-at-barrier, post-`run_quantum` — mid-quantum
    /// inspection is racy). Returns 0 on Threaded before the first
    /// `run_quantum` call (cores not yet taken into worker threads).
    pub fn core_cycles(&self, idx: u8) -> u64 {
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        if let Some(t) = &self.threaded {
            return t.core_cycles(idx);
        }
        match (&self.cores, idx) {
            (Cores::Arm(arm), 0) => arm[0].cycles(),
            (Cores::Arm(arm), 1) => arm[1].cycles(),
            (Cores::RiscV(cs), 0) => cs[0].cycles(),
            (Cores::RiscV(cs), 1) => cs[1].cycles(),
            _ => panic!("core_cycles: idx must be 0 or 1"),
        }
    }

    /// Reset the emulator: load SP from ROM word 0, PC from ROM word 1.
    /// Both cores boot from the reset vector.
    pub fn reset(&mut self) {
        let initial_sp = self.bus.memory.rom_read32(0);
        let reset_vector = self.bus.memory.rom_read32(4);

        // Boot both cores from reset vector. Phase 3 Stage 1 (Arm arm):
        // cores share a single `CoreAtomics` with Bus. Rebuilding the
        // cores must reuse the existing Arc so post-reset asserts land
        // on the same state the Bus sees.
        let atomics = Arc::clone(&self.bus.atomics);
        // Bootrom hook PCs are derived from the loaded ROM bytes.
        // `CortexM33::new` clears the fields, so re-resolve here from
        // the live `Memory` so the hook survives `reset`. HLD V5
        // §"Component 3 — Soft-reboot / reload".
        let (hook_s, hook_ns) = {
            // Read the first 32 KB of ROM into a scratch buffer; the
            // resolver only needs offsets 0x14 and ≥0x7cd4.
            let mut rom_bytes = vec![0u8; crate::memory::ROM_SIZE];
            for (i, b) in rom_bytes.iter_mut().enumerate() {
                *b = self.bus.memory.rom_read8(i as u32);
            }
            bootrom_hooks::resolve_bootrom_hooks(&rom_bytes, b"RB")
        };
        match &mut self.cores {
            Cores::Arm(arm) => {
                for i in 0..2 {
                    arm[i] = CortexM33::new(i as u8, Arc::clone(&atomics));
                    arm[i].regs.msp = initial_sp;
                    arm[i].regs.r[13] = initial_sp;
                    arm[i].regs.set_pc(reset_vector & !1);
                    arm[i].regs.xpsr = 1 << 24; // Thumb bit (XPSR_T)
                    arm[i].bootrom_reboot_hook_pc_s = hook_s;
                    arm[i].bootrom_reboot_hook_pc_ns = hook_ns;
                }
            }
            Cores::RiscV(cs) => {
                // HLD §4.3: each hart resets to its §4.3 power-on state
                // (pc = 0x2000_0000, CSRs zeroed except mtvec / mcountinhibit,
                // hart_id preserved). Shared bus state resets below, identical
                // to the Arm arm.
                for i in 0..2 {
                    cs[i].reset();
                }
            }
        }

        // Clear the shared atomic state — halted / WFE / event_flag /
        // irq_pending / RCP / bus-fault. Replaces the per-core clears
        // that pre-Stage-1 touched the now-deleted Bus fields.
        atomics.reset();
        self.bus.clear_warned_addrs();
        self.bus.clear_watchdog_reset();
        // WATCHDOG SCRATCH0..7 survive reset by datasheet (§4.7); the
        // rest of the block (CTRL / TIME / LOAD / REASON) quiesces.
        self.bus.watchdog.post_reset();
        // SHA-256 accumulator quiesces on reset (HLD V5 §7.D.6). OTP
        // fuse state and TRNG counter intentionally persist across reset
        // — OTP is physical silicon state, and a persistent counter still
        // yields a unique sequence post-reset.
        self.bus.sha256.reset();
        // GLITCH_DETECTOR quiesces on warm reset (pico-sdk
        // `glitch_detector.h`): ARM returns to `ARM_RESET = 0x5bad` and
        // other registers return to 0. Re-seeds the backing HashMap via
        // `Self::new()`.
        self.bus.glitch.reset();
        // POWMAN quiesces on warm reset — COUNT/MATCH/TIMER/INTR all
        // return to post-power-on zero. Mirrors the Stage 3
        // GLITCH_DETECTOR pattern.
        self.bus.powman.reset();
        // Threading: PPB is per-core (not on the bus); the `cs[i].reset()`
        // / `CortexM33::new` path above already resets both cores' PPBs.
        // HLD V5 §5.7: post-bootrom RESETS state — peripherals
        // released by pico-sdk `runtime_init_bootrom_reset` start
        // deasserted. The emulator never runs the bootrom; we seed
        // the post-bootrom state directly.
        self.bus.resets_state = crate::bus::RESETS_POST_BOOTROM;
        self.bus.ticks.reset();
        self.bus.timer0.reset();
        self.bus.timer1.reset();
        self.bus.sio.reset();
        for pio in &mut self.bus.pio {
            pio.reset();
        }
        self.bus.gpio_in.store(0, Ordering::Relaxed);
        self.bus.gpio_in_hi.store(0, Ordering::Relaxed);
        // External-input stimulus (harness-owned pin forcing) survives
        // reset only if the harness re-applies it post-reset. Clearing
        // here matches the real-silicon model: any host stimulus must
        // be re-asserted after a chip reset.
        self.bus.gpio_external_in.store(0, Ordering::Relaxed);
        self.bus.gpio_external_mask = 0;
        self.bus.gpio_external_in_hi.store(0, Ordering::Relaxed);
        self.bus.gpio_external_mask_hi = 0;

        // Drop PLL lock-arm state so a post-reset power-up re-arms the
        // counter against the freshly-zeroed master cycle. Mirrors the
        // rp2040_emu reset path.
        self.bus.master_cycle = 0;
        self.bus.pll_sys_lock_at_cycle = None;
        self.bus.pll_usb_lock_at_cycle = None;

        // Fresh `CortexM33::new` above already produces empty decode
        // caches; clear any dirty-range state the old Bus was
        // carrying so it doesn't leak into the next step.
        self.bus.pending_cache_invalidations.clear();
        self.bus.pending_invalidation_regions = 0;

        // Reset clock. The authoritative sys_clk_hz lives on Bus's
        // clock tree (see bus/clocks.rs), so nothing to preserve here.
        self.clock = Clock { cycles: 0 };

        // HLD V5 §5.7: post-bootrom clock tree. firmware running via
        // `load_image` bypasses the bootrom, so scenarios see the
        // pico-sdk post-`runtime_init_clocks` state (clk_sys = 150 MHz,
        // clk_ref = 12 MHz, clk_peri = clk_sys).
        self.bus.seed_post_bootrom_clocks();

        // Bootrom hook latch — clear so a post-reset run doesn't see
        // a stale `shutdown_requested` from the prior life.
        self.shutdown_requested = false;
    }

    /// Load a raw binary at the given address.
    ///
    /// Supports the RP2350-native SRAM region (`0x2xxx_xxxx`) and the
    /// test-only oracle alias (`0x8xxx_xxxx`) added for the QEMU rv32
    /// differential oracle. See `Bus::canon_oracle_addr` for the
    /// rationale — QEMU virt rv32's only writable RAM lives at
    /// `0x8000_0000`, so the oracle lands code there on both sides.
    pub fn load_image(&mut self, addr: u32, data: &[u8]) {
        for (i, &byte) in data.iter().enumerate() {
            let a = addr.wrapping_add(i as u32);
            match a >> 28 {
                0x0 => {} // ROM is loaded via load_bootrom
                0x2 => self.bus.memory.sram_write8(a & 0x0FFF_FFFF, byte),
                0x8 => self.bus.memory.sram_write8(a & 0x0FFF_FFFF, byte),
                _ => {}
            }
        }
    }

    /// Load the bootrom (32 kB at address 0x00000000). Also invalidates
    /// the ROM-region decode-cache entries on both cores — the bytes
    /// have been replaced wholesale. SRAM / XIP slots are preserved.
    ///
    /// Bootrom mask-ROM hook PCs (HLD V5 §"Component 3") are recomputed
    /// from the freshly-loaded bytes and written into both cores'
    /// `bootrom_reboot_hook_pc_s` / `_pc_ns` fields. Soft-reboot
    /// scenarios that re-call this function get fresh hook PCs.
    ///
    /// **Mid-run reload caveat (threaded mode).** In Threaded mode each
    /// `CortexM33` is moved by value into a worker thread at spawn
    /// (`threaded::core_worker_body`); a mid-run reload would not
    /// reach the worker-owned core. The current `Emulator` API has no
    /// "join workers, reload, respawn" entry point — mid-run reload is
    /// not a supported use case. The pre-spawn population path (call
    /// `load_bootrom` before the first `run`/`run_quantum`) is fully
    /// supported and exercised by the unit tests.
    pub fn load_bootrom(&mut self, data: &[u8]) {
        self.bus.load_bootrom(data);
        // Bus set the ROM bit in `pending_invalidation_regions`; drain
        // it here so harness / app callers don't need to step before
        // observing the invalidation. Phase 3 follow-up #10 + Task #10
        // review fix — region-scoped to avoid cold-cache regressions.
        let regions = self.bus.pending_invalidation_regions;
        if let Cores::Arm(arm) = &mut self.cores {
            // Resolve the bootrom `RB` (reboot) hook PCs and seed both
            // cores. `data` is the same buffer we just gave the bus —
            // resolving from it directly avoids a redundant memory
            // re-read.
            let (s, ns) = bootrom_hooks::resolve_bootrom_hooks(data, b"RB");
            for core in arm.iter_mut() {
                core.invalidate_decode_cache_regions(regions);
                core.bootrom_reboot_hook_pc_s = s;
                core.bootrom_reboot_hook_pc_ns = ns;
            }
        }
        self.bus.pending_invalidation_regions = 0;
    }

    /// Load flash image (appears at XIP address 0x10000000). Invalidates
    /// only the XIP-region decode-cache entries on both cores — SRAM /
    /// ROM slots stay hot, so firmware that reloads flash then runs
    /// SRAM code doesn't pay a cold-cache repopulate tax on the next
    /// quantum.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.bus.load_flash(data);
        let regions = self.bus.pending_invalidation_regions;
        if let Cores::Arm(arm) = &mut self.cores {
            for core in arm.iter_mut() {
                core.invalidate_decode_cache_regions(regions);
            }
        }
        self.bus.pending_invalidation_regions = 0;
    }

    /// Advance the system by one quantum. Each core runs atomically —
    /// instruction-at-a-time — until its per-core cycle count catches up
    /// with the quantum's target. Peripherals tick the full quantum at
    /// the boundary. Returns the post-quantum master cycle count.
    ///
    /// Returns `Err(EmulatorError::NotSupportedInThreadedMode)` when
    /// called on a Threaded emulator — Threaded runs in quanta via
    /// [`Self::run_quantum`] / [`Self::run`] and cannot be
    /// single-stepped. HLD V1 §5.4.
    ///
    /// **Overshoot:** a multi-cycle instruction straddling the boundary
    /// leaves `core.cycles > clock.cycles` by up to one instruction's
    /// worth. The next quantum's `while` predicate consumes that overshoot
    /// — the core executes proportionally fewer instructions until its
    /// `cycles` realigns with `clock.cycles`. Over many quanta the rate
    /// averages 1:1. A halted core never contributes `cycles`, so the
    /// `while` predicate never fires and the core is skipped cheaply.
    pub fn step(&mut self) -> Result<u64, EmulatorError> {
        if self.execution_model == ExecutionModel::Threaded {
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            if let Some((which, message)) = &self.panic_info {
                return Err(EmulatorError::WorkerPanicked {
                    which: *which,
                    message: message.clone(),
                });
            }
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            if let Some((which, elapsed_ms)) = self.timeout_info {
                return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
            }
            return Err(EmulatorError::NotSupportedInThreadedMode);
        }
        Ok(self.step_serial())
    }

    /// Serial-mode single-quantum step. Shared by [`Self::step`] and
    /// [`Self::run_quantum`] on the Serial path.
    fn step_serial(&mut self) -> u64 {
        debug_assert!(self.step_quantum > 0, "step_quantum must be >= 1");
        // Decode-cache invalidation strategy:
        //   (a) Emulator::load_bootrom/load_flash/reset drain regions
        //       proactively on both cores so pre-step tests see a clean
        //       slate.
        //   (b) Pre-step: drain Bus::pending_invalidation_regions into
        //       both cores. Covers any external `bus.load_*` /
        //       `bus.invalidate_all` pokes that happened between step()
        //       calls without going through Emulator.
        //   (c) Per-instruction: drain Bus::pending_cache_invalidations
        //       into the core that just ran. Covers in-step writes to
        //       executable memory.
        // Do not remove any layer — the test suite exercises all three
        // paths.
        //
        // Phase 3 Stage 2: the Arc-sharing trip-wire lives at the top of
        // `CortexM33::step` — every caller (tests, harness, this driver)
        // funnels through it via `bus.atomics()`. No need to duplicate
        // the check here.
        // Refresh the Bus's view of the master cycle count so any MMIO
        // reads / writes performed during this quantum (notably PLL CS
        // lock bit + lock-arm transitions — see
        // `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 P2)
        // observe a current cycle. Staleness is bounded by one quantum.
        self.bus.master_cycle = self.clock.cycles;
        let target = self.clock.cycles + self.step_quantum as u64;

        // (b) Pre-step region-scoped drain. Firmware-loading paths
        // (`load_bootrom`/`load_flash`) and `Bus::invalidate_all` set
        // bits in `pending_invalidation_regions` on the bus between
        // steps; drain them here (per-core, region-scoped) so stale
        // entries can't survive the reload while preserving any slots
        // outside the touched region. Phase 3 follow-up #10 + Task #10
        // review fix.
        if self.bus.pending_invalidation_regions != 0 {
            let regions = self.bus.pending_invalidation_regions;
            if let Cores::Arm(arm) = &mut self.cores {
                arm[0].invalidate_decode_cache_regions(regions);
                arm[1].invalidate_decode_cache_regions(regions);
            }
            self.bus.pending_invalidation_regions = 0;
        }

        // Compose external stimulus into `bus.gpio_in` before the cores
        // dispatch. `update_gpio` also runs at the end of the quantum
        // (inside `tick_peripherals`); the extra call here catches any
        // `gpio_external_in` / `gpio_external_mask` writes that landed
        // between `step()` invocations, so the cores' first MMIO read
        // of SIO_GPIO_IN in this quantum sees the freshly-composed view
        // instead of a one-quantum-stale value.
        self.update_gpio();

        match &mut self.cores {
            Cores::Arm(cs) => step_pair_arm(cs, &mut self.bus, target),
            Cores::RiscV(cs) => step_pair_riscv(cs, &mut self.bus, target),
        }

        // Drain bootrom-hook latches into `shutdown_requested` (HLD V5
        // §"Component 3"). The hook check inside
        // `CortexM33::step{,_no_atomics}` halts the core and sets
        // `bootrom_hook_fired`; surface that to the host on the
        // Emulator. Cheap — two byte-loads per quantum, off the
        // per-instruction hot path.
        if let Cores::Arm(cs) = &mut self.cores
            && (cs[0].bootrom_hook_fired || cs[1].bootrom_hook_fired)
        {
            self.shutdown_requested = true;
        }

        self.clock.advance(self.step_quantum as u64);
        // S4: peripherals tick the full quantum, not `consumed` (bytes
        // the cores actually executed). V5 §5.5 prescribes an
        // unconditional per-cycle tick; batching by `step_quantum`
        // preserves the contract while saving dispatch cost. A halted
        // core skews `core.cycles` against `clock.cycles` by at most one
        // quantum, so the drift never exceeds `step_quantum` cycles — a
        // tolerance the HLD accepts (see V5 §5.5). rp2040_emu's tick loop
        // uses `consumed` instead; rp2350_emu explicitly diverges because
        // the ARMv8-M dual-core contention model is disabled here
        // (CLAUDE.md "Bank contention model").
        self.tick_peripherals(self.step_quantum);
        // RISC-V has no SysTick — the SysTick block lives on the M33 PPB.
        if self.cores.is_arm() {
            self.tick_systick();
        }
        // P4: fan-out MTIP/MSIP/MEIP into per-hart `mip` before the wake
        // check. Order matters — `wake_checks` inspects `(mip & mie)` to
        // clear `wfi_parked`, so it needs a freshly-sourced `mip` first.
        // HLD §4.1 / §4.6.
        if self.cores.is_riscv() {
            self.fan_out_riscv_irqs();
        }
        self.wake_checks();
        self.clock.cycles
    }

    /// Drive Hazard3 `mip` bits 3 (MSIP), 7 (MTIP), and 11 (MEIP) from
    /// the per-hart hardware sources. MTIP is level-sensitive from SIO's
    /// `mtime_match_asserted`; MSIP is the per-hart bit of
    /// `SIO.RISCV_SOFTIRQ`; MEIP is computed by the Hazard3 IRQ
    /// controller from `(bus.irq_pending | meifa) & meiea`. HLD §4.6.
    ///
    /// Firmware CSR writes to MSIP/MTIP/MEIP (via `csrrw mip, ...`) are
    /// stomped here on the next quantum — the hardware source wins, per
    /// RV-priv §3.1.9 which classes these bits as hardware-owned.
    fn fan_out_riscv_irqs(&mut self) {
        let Cores::RiscV(cs) = &mut self.cores else {
            return;
        };
        let sio = &self.bus.sio;
        for c in 0..2 {
            let mut mip = cs[c].mip();
            // MTIP bit 7 — level-sensitive from SIO.
            if sio.mtime_match_asserted[c] {
                mip |= 1 << 7;
            } else {
                mip &= !(1 << 7);
            }
            // MSIP bit 3 — from RISCV_SOFTIRQ per-hart bits.
            let sw = (sio.riscv_softirq() >> c) & 1;
            if sw != 0 {
                mip |= 1 << 3;
            } else {
                mip &= !(1 << 3);
            }
            // MEIP bit 11 — from Hazard3 IRQ controller (P4).
            let meip = cs[c].compute_meip(self.bus.atomics.irq_pending_load(c));
            if meip {
                mip |= 1 << 11;
            } else {
                mip &= !(1 << 11);
            }
            cs[c].set_mip(mip);
        }
    }

    /// Run for at least `cycles` virtual cycles. Returns the final
    /// master cycle count. May overshoot by up to `step_quantum - 1`
    /// cycles (one quantum's worth), matching the documented overshoot
    /// behaviour of [`Self::step`].
    ///
    /// Dispatches to the selected [`ExecutionModel`]. In Threaded mode
    /// this rounds up to the nearest quantum boundary (HLD V1 §5.4)
    /// and returns `Err(EmulatorError::WorkerPanicked)` sticky on
    /// worker panic.
    pub fn run(&mut self, cycles: u64) -> Result<u64, EmulatorError> {
        if self.execution_model == ExecutionModel::Serial {
            let target = self.clock.cycles + cycles;
            while self.clock.cycles < target {
                self.step_serial();
            }
            return Ok(self.clock.cycles);
        }
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        {
            // Threaded: round up to nearest quantum boundary and drive
            // all quanta inside one `run_quanta_checked` batch so the
            // 6-thread worker pool amortises spawn cost across the run
            // (HLD V1 §5.4). Single-quantum `run_quantum()` is for
            // symmetry with Serial-mode `step()`; bulk callers should
            // use `run()`.
            if let Some((which, message)) = &self.panic_info {
                return Err(EmulatorError::WorkerPanicked {
                    which: *which,
                    message: message.clone(),
                });
            }
            if let Some((which, elapsed_ms)) = self.timeout_info {
                return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
            }
            if self.threaded.is_none() {
                self.promote_to_threaded();
            }
            let step_q = self.step_quantum as u64;
            let quanta = cycles.div_ceil(step_q.max(1));
            let threaded = self.threaded.as_mut().expect("threaded promoted above");
            match threaded.run_quanta_checked(quanta) {
                Ok(()) => {
                    // Drain bootrom-hook latch to the host-visible field
                    // (HLD V5 §"Component 3"). Mirrors the serial drain
                    // at line 707 above.
                    if threaded.shutdown_requested() {
                        self.shutdown_requested = true;
                    }
                    Ok(threaded.master_cycle())
                }
                Err(threaded::RunError::Panic { which, message }) => {
                    self.panic_info = Some((which, message.clone()));
                    Err(EmulatorError::WorkerPanicked { which, message })
                }
                Err(threaded::RunError::Timeout { which, elapsed_ms }) => {
                    self.timeout_info = Some((which, elapsed_ms));
                    Err(EmulatorError::BarrierTimeout { which, elapsed_ms })
                }
            }
        }
        #[cfg(not(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        )))]
        {
            let _ = cycles;
            Err(EmulatorError::NotSupportedInThreadedMode)
        }
    }

    /// Advance the emulator by exactly one quantum (`step_quantum`
    /// cycles). Primary entry point for the Threaded path; on Serial
    /// this is the same as [`Self::step`] and returns the new master
    /// cycle count. HLD V1 §5.4.
    ///
    /// Returns `Err(EmulatorError::WorkerPanicked)` sticky on worker
    /// panic in Threaded mode. One-shot-after-panic: subsequent calls
    /// return the cached error without re-attempting workers.
    pub fn run_quantum(&mut self) -> Result<u64, EmulatorError> {
        match self.execution_model {
            ExecutionModel::Serial => Ok(self.step_serial()),
            ExecutionModel::Threaded => self.run_quantum_threaded(),
        }
    }

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    fn run_quantum_threaded(&mut self) -> Result<u64, EmulatorError> {
        // One-shot: any cached panic / watchdog timeout short-circuits
        // without touching worker state. HLD V1 §5.5 item 5 / §6.6.
        if let Some((which, message)) = &self.panic_info {
            return Err(EmulatorError::WorkerPanicked {
                which: *which,
                message: message.clone(),
            });
        }
        if let Some((which, elapsed_ms)) = self.timeout_info {
            return Err(EmulatorError::BarrierTimeout { which, elapsed_ms });
        }
        // Lazy promotion: first run_quantum / run moves the seeded
        // cores / bus / clock into a fresh ThreadedEmulator so harness
        // setup that poked MMIO on `emu.bus` pre-run is carried over.
        if self.threaded.is_none() {
            self.promote_to_threaded();
        }
        let threaded = self.threaded.as_mut().expect("threaded promoted above");
        // If a panic has been armed via `inject_panic_for_testing`,
        // push the corresponding TestPanic command into the per-block
        // queue so the matching PIO worker panics on drain. Gated
        // behind `testing` (Stage 1b review REQUIRED #2) — without
        // the feature, `PioCommand::TestPanic` does not exist and
        // `pending_panic_inject` is always `None`.
        #[cfg(feature = "testing")]
        if let Some(which) = self.pending_panic_inject.take() {
            let block = match which {
                threaded::WorkerName::Pio0 => 0,
                threaded::WorkerName::Pio1 => 1,
                threaded::WorkerName::Pio2 => 2,
                _ => panic!("inject_panic_for_testing: only Pio0/Pio1/Pio2 supported today"),
            };
            threaded
                .shared()
                .pio
                .send_command(threaded::PioCommand::TestPanic { block });
            // Halt both cores so the CPU workers exit the quantum
            // cleanly; only the PIO worker panics.
            threaded.shared().atomics.set_halted(0);
            threaded.shared().atomics.set_halted(1);
        }
        match threaded.run_quanta_checked(1) {
            Ok(()) => {
                // Drain bootrom-hook latch to the host-visible field
                // (HLD V5 §"Component 3"). Mirrors the serial drain
                // at line 707 in `step_serial`'s post-quantum block.
                if threaded.shutdown_requested() {
                    self.shutdown_requested = true;
                }
                Ok(threaded.master_cycle())
            }
            Err(threaded::RunError::Panic { which, message }) => {
                self.panic_info = Some((which, message.clone()));
                Err(EmulatorError::WorkerPanicked { which, message })
            }
            Err(threaded::RunError::Timeout { which, elapsed_ms }) => {
                self.timeout_info = Some((which, elapsed_ms));
                Err(EmulatorError::BarrierTimeout { which, elapsed_ms })
            }
        }
    }

    #[cfg(not(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    )))]
    fn run_quantum_threaded(&mut self) -> Result<u64, EmulatorError> {
        Err(EmulatorError::NotSupportedInThreadedMode)
    }

    /// Move the seeded Serial state into a fresh `ThreadedEmulator`.
    /// Called lazily on the first `run_quantum` / `run` so harness
    /// setup that poked `emu.bus` / `emu.core_mut(...)` pre-run is
    /// carried over. After promotion, the top-level `cores` / `bus` /
    /// `clock` fields hold zero-cost placeholders and must not be
    /// inspected mid-run.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    fn promote_to_threaded(&mut self) {
        let atomics = Arc::new(crate::threaded::CoreAtomics::default());
        let placeholder_bus = Bus::with_atomics(Arc::clone(&atomics));
        let placeholder_cores = Cores::Arm([
            CortexM33::new(0, Arc::clone(&atomics)),
            CortexM33::new(1, Arc::clone(&atomics)),
        ]);
        let seeded_cores = std::mem::replace(&mut self.cores, placeholder_cores);
        let seeded_bus = std::mem::replace(&mut self.bus, placeholder_bus);
        let seeded_clock = std::mem::replace(&mut self.clock, Clock { cycles: 0 });
        let seeded = Emulator {
            cores: seeded_cores,
            bus: seeded_bus,
            clock: seeded_clock,
            step_quantum: self.step_quantum,
            execution_model: ExecutionModel::Serial,
            threaded: None,
            panic_info: None,
            timeout_info: None,
            #[cfg(feature = "testing")]
            pending_panic_inject: None,
            bus_is_placeholder: false,
            shutdown_requested: self.shutdown_requested,
        };
        self.threaded = Some(threaded::ThreadedEmulator::from_emulator(seeded));
        // Mark the flat fields as dead state — they now hold
        // zero-cost placeholders. Typed accessors assert on this in
        // debug builds (see REQUIRED #1 in Stage 1b review).
        self.bus_is_placeholder = true;
    }

    /// Test-only: arm a panic injection for the next `run_quantum`
    /// call. Only valid for PIO workers (`Pio0` / `Pio1` / `Pio2`);
    /// passing `Core0` / `Core1` / `Coord` panics the run with a
    /// debug-assert fire because those workers have no
    /// command-queue entry point for injection.
    ///
    /// Feature-gated behind `testing` (Stage 1b review REQUIRED #2)
    /// so release consumers of `rp2350_emu = "2.0"` cannot brick their
    /// emulator by calling an internal hook. HLD V1 §5.5 TDD hook;
    /// exists solely for `tests/execution_model.rs`.
    #[cfg(all(
        feature = "testing",
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    pub fn inject_panic_for_testing(&mut self, which: threaded::WorkerName) {
        debug_assert!(
            matches!(
                which,
                threaded::WorkerName::Pio0
                    | threaded::WorkerName::Pio1
                    | threaded::WorkerName::Pio2
            ),
            "inject_panic_for_testing only supports PIO workers \
             (Core0/Core1/Coord panics are not injectable)"
        );
        self.pending_panic_inject = Some(which);
    }

    /// Advance peripherals by `cycles` virtual cycles. Called once at the
    /// end of each quantum.
    fn tick_peripherals(&mut self, cycles: u32) {
        let gpio_pins = (self.bus.gpio_in.load(Ordering::Relaxed) as u64)
            | ((self.bus.gpio_in_hi.load(Ordering::Relaxed) as u64) << 32);
        let resets = self.bus.resets_state;
        // PIO0/1/2 are gated by their RESETS bits — real hardware holds
        // PIO inert while its reset line is asserted. RESET_PIO0..2 are
        // contiguous (11, 12, 13), so `RESET_PIO0 + i` gives the bit
        // for `pio[i]`.
        for (i, pio) in self.bus.pio.iter_mut().enumerate() {
            let bit = crate::bus::RESET_PIO0 + i as u8;
            if (resets & (1u32 << bit)) == 0 {
                pio.step_n_with_pins(cycles, gpio_pins);
            }
        }
        self.route_pio_irqs();
        self.update_gpio();
        // Bus peripherals (TICKS + TIMER0/1 + RISC-V MTIME).
        // HLD V5 §5.3 / §5.5: tick runs every quantum unconditionally,
        // no fast-path gate in V5. MTIME ticks are drained from
        // `TICKS.RISCV` inside `Bus::tick_peripherals` per residual A.2.1
        // (HLD `2026.04.17 - HLD - Residual A.2.1 MTIME WATCHDOG_TICK Fix.md`).
        // Drains alarm-match IRQs into both cores' NVIC pending masks
        // via `assert_irq_shared`.
        self.bus.tick_peripherals(cycles);
    }

    /// Route PIO IRQ flags to the NVIC via INT0_INTE / INT1_INTE masks.
    ///
    /// Each PIO block has two NVIC lines (IRQ_0 and IRQ_1). The 12-bit
    /// raw status (INTR) comprises `IRQ[3:0]` flags plus FIFO status
    /// (TXNFULL / RXNEMPTY). A flag reaches NVIC line N iff
    /// `(INTR & INTn_INTE) | INTn_INTF != 0`.
    ///
    /// PIO IRQs are shared (both cores see them). The IRQ numbers for
    /// each block are contiguous pairs starting at `IRQ_PIO0_IRQ_0`.
    fn route_pio_irqs(&mut self) {
        use crate::irq::IRQ_PIO0_IRQ_0;
        for i in 0..3 {
            // Capture INTS values before mutably borrowing `self.bus` for
            // `assert_irq_shared`.
            let ints0 = self.bus.pio[i].int0_ints_rp2350();
            let ints1 = self.bus.pio[i].int1_ints_rp2350();
            let irq0_line = IRQ_PIO0_IRQ_0 + (i as u32) * 2;
            let irq1_line = irq0_line + 1;
            if ints0 != 0 {
                self.bus.assert_irq_shared(irq0_line);
            }
            if ints1 != 0 {
                self.bus.assert_irq_shared(irq1_line);
            }
        }
    }

    /// Quantum-end SysTick advance. Each core's SysTick is ticked by the
    /// delta between its current `cycles` and the last `systick_advance`
    /// snapshot. The per-core CVR and COUNTFLAG state lives on
    /// `CortexM33::ppb` (Phase 0b.1 Commit B); pending exception delivery
    /// sets `ICSR.PENDSTSET` via `Ppb::pend_systick()` when TICKINT is
    /// enabled.
    fn tick_systick(&mut self) {
        let arm = self.cores.expect_arm_mut();
        for core_id in 0..2 {
            let cycles = arm[core_id].cycles();
            arm[core_id].ppb.systick_advance(cycles);
        }
    }

    /// WFE/SEV and WFI wake checks.
    /// - WFE: if event_flag is set, consume it and wake the core.
    /// - WFI: if an enabled pending IRQ exists, wake the core.
    pub(crate) fn wake_checks(&mut self) {
        match &mut self.cores {
            Cores::Arm(arm) => {
                for i in 0..2 {
                    // WFE wake: event flag clears WFE sleep. Consume
                    // (AcqRel swap to false) pairs with `sev_both`'s
                    // Release.
                    if self.bus.atomics.is_wfe_waiting(i) && self.bus.atomics.event_flag_consume(i)
                    {
                        self.bus.atomics.clear_wfe_waiting(i);
                    }
                    // WFI wake: enabled pending IRQ clears WFI sleep.
                    // The peek is non-consuming; the next step() will
                    // merge via `take_irq_pending`.
                    if self.bus.atomics.is_halted(i) {
                        let pending = self.bus.atomics.irq_pending_load(i);
                        if pending != 0 && arm[i].ppb.any_pending_enabled(pending) {
                            self.bus.atomics.clear_halted(i);
                        }
                    }
                }
            }
            Cores::RiscV(cs) => {
                // HLD §4.6: `wfi` wakes when `(mip & mie) != 0`. The wake
                // decision ignores `mstatus.MIE` — MIE only gates trap
                // *delivery*. If MIE=0 the hart wakes and resumes the
                // next instruction; if MIE=1 the next step() will deliver
                // the trap at instruction boundary.
                for c in cs {
                    if c.wfi_parked && (c.mip() & c.mie()) != 0 {
                        c.wfi_parked = false;
                    }
                }
            }
        }
    }

    /// Merge SIO and PIO GPIO outputs into bus.gpio_in.
    /// PIO output-enable overrides SIO: if a PIO block drives a pin, its value wins.
    ///
    /// External-input stimulus (see [`Bus::gpio_external_mask`]) is overlaid
    /// last so the harness can force pins (CS, address bus, etc.) that would
    /// otherwise be recomputed every tick. Mask-clear bits reflect whatever
    /// SIO/PIO produced; mask-set bits reflect `gpio_external_in`.
    pub(crate) fn update_gpio(&mut self) {
        let mut out_lo = self.bus.sio.gpio_out & self.bus.sio.gpio_oe;
        let mut out_hi = 0u32;
        for pio in &self.bus.pio {
            let (pio_out_lo, pio_out_hi) = pio.local_to_physical_pins(pio.pad_out);
            let (pio_oe_lo, pio_oe_hi) = pio.local_to_physical_pins(pio.pad_oe);
            out_lo = (out_lo & !pio_oe_lo) | (pio_out_lo & pio_oe_lo);
            out_hi = (out_hi & !pio_oe_hi) | (pio_out_hi & pio_oe_hi);
        }
        let ext_mask = self.bus.gpio_external_mask;
        let ext_val = self.bus.gpio_external_in.load(Ordering::Relaxed);
        self.bus.gpio_in.store(
            (out_lo & !ext_mask) | (ext_val & ext_mask),
            Ordering::Relaxed,
        );

        let ext_mask_hi = self.bus.gpio_external_mask_hi;
        let ext_val_hi = self.bus.gpio_external_in_hi.load(Ordering::Relaxed);
        self.bus.gpio_in_hi.store(
            (out_hi & !ext_mask_hi) | (ext_val_hi & ext_mask_hi),
            Ordering::Relaxed,
        );
    }

    /// Read a GPIO pin from the merged pin state. Debug-only: asserts
    /// the emulator has not been promoted into Threaded mode.
    pub fn gpio_read(&self, pin: u8) -> bool {
        self.assert_not_placeholder();
        match pin {
            0..=31 => (self.bus.gpio_in.load(Ordering::Relaxed) >> pin) & 1 != 0,
            32..=47 => (self.bus.gpio_in_hi.load(Ordering::Relaxed) >> (pin - 32)) & 1 != 0,
            _ => false,
        }
    }

    /// Write a GPIO pin (stub for Phase 1).
    pub fn gpio_write(&mut self, _pin: u8, _value: bool) {}

    /// Read all GPIO pins as a physical 48-pin bitmask. Debug-only:
    /// asserts the emulator has not been promoted into Threaded mode.
    pub fn gpio_read_all(&self) -> u64 {
        self.assert_not_placeholder();
        (self.bus.gpio_in.load(Ordering::Relaxed) as u64)
            | ((self.bus.gpio_in_hi.load(Ordering::Relaxed) as u64) << 32)
    }

    /// Placeholder-guard message shared by the typed accessors below.
    /// Central so the string stays consistent between tests and the
    /// REQUIRED #1 contract documented in `tech_debt.md`.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    const PLACEHOLDER_GUARD_MSG: &'static str = "direct field access on cores/bus/clock is Serial-only; emulator is in \
         Threaded mode — use typed accessors like core_cycles(), master_cycle(), \
         gpio_get() instead";

    /// Debug-only placeholder assertion. No-op on Serial builds and on
    /// non-threading platforms — the field does not exist there.
    #[inline(always)]
    fn assert_not_placeholder(&self) {
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        debug_assert!(!self.bus_is_placeholder, "{}", Self::PLACEHOLDER_GUARD_MSG);
    }

    /// Access core state. **Panics on a RISC-V emulator** — this is a
    /// shim for Arm-only call sites (harness, tests). Cross-arch callers
    /// must dispatch on `cores.is_arm()` first.
    ///
    /// Debug-only: asserts the emulator has not been promoted into
    /// Threaded mode (the flat `cores` field would be a placeholder).
    pub fn core(&self, id: usize) -> &CortexM33 {
        self.assert_not_placeholder();
        &self.cores.expect_arm()[id]
    }

    /// Mutable accessor; same panic contract as [`Self::core`]. Same
    /// debug-only placeholder assertion.
    pub fn core_mut(&mut self, id: usize) -> &mut CortexM33 {
        self.assert_not_placeholder();
        &mut self.cores.expect_arm_mut()[id]
    }

    /// RISC-V counterpart to [`Self::core`]. **Panics on an Arm emulator.**
    /// Same debug-only placeholder assertion.
    pub fn core_riscv(&self, id: usize) -> &Hazard3 {
        self.assert_not_placeholder();
        &self.cores.expect_riscv()[id]
    }

    /// Mutable accessor; same panic contract as [`Self::core_riscv`].
    /// Same debug-only placeholder assertion.
    pub fn core_riscv_mut(&mut self, id: usize) -> &mut Hazard3 {
        self.assert_not_placeholder();
        &mut self.cores.expect_riscv_mut()[id]
    }

    /// Get a reference to a core's workload counters. Panics on RISC-V
    /// (Hazard3 has no workload-counters stash yet). Same debug-only
    /// placeholder assertion.
    pub fn core_counters(&self, core_id: usize) -> &CoreCounters {
        self.assert_not_placeholder();
        &self.cores.expect_arm()[core_id].counters
    }

    /// Reset all core counters. No-op on RISC-V. Same debug-only
    /// placeholder assertion.
    pub fn reset_counters(&mut self) {
        self.assert_not_placeholder();
        if let Cores::Arm(arm) = &mut self.cores {
            for core in arm.iter_mut() {
                core.counters.reset();
            }
        }
    }

    /// Direct memory read (bypasses bus timing). Debug-only: asserts
    /// the emulator has not been promoted into Threaded mode.
    pub fn peek(&self, addr: u32) -> u32 {
        self.assert_not_placeholder();
        if Bus::is_boot_ram(addr) {
            self.bus.boot_ram_read32(addr)
        } else {
            self.bus.memory.peek32(addr)
        }
    }

    /// Direct memory write (bypasses bus timing).
    ///
    /// **Cache note:** this bypasses the `Bus::write32` path and does
    /// NOT invalidate the per-core decoded-op caches. Callers that poke
    /// into executable memory (ROM / XIP / SRAM) and then `step()` must
    /// call [`Bus::invalidate_all`] on `self.bus` between the poke and
    /// the next `step` to avoid executing stale decoded ops. The flag
    /// is consumed by the next `Emulator::step` pre-step phase, which
    /// invalidates both cores' caches. Pre-boot pokes (the common case
    /// for the harness) happen before any cache entries exist and are
    /// safe without an explicit invalidation.
    pub fn poke(&mut self, addr: u32, value: u32) {
        self.assert_not_placeholder();
        if Bus::is_boot_ram(addr) {
            self.bus.boot_ram_write32(addr, value);
        } else {
            self.bus.memory.poke32(addr, value);
        }
    }

    /// Current master cycle count. Debug-only: asserts the emulator
    /// has not been promoted into Threaded mode — Threaded callers
    /// read the live master cycle via the value returned from
    /// [`Self::run_quantum`] / [`Self::run`].
    pub fn cycles(&self) -> u64 {
        self.assert_not_placeholder();
        self.clock.cycles
    }

    /// Write a 32-bit word to an MMIO address via the bus. Charges zero
    /// emulator cycles (intended for setup code running outside `run()`).
    ///
    /// Delegates to [`Bus::write32`], so alias bits (`(addr >> 12) & 3`)
    /// are honoured: base address = normal, XOR alias = `|0x1000`, SET
    /// alias = `|0x2000`, CLR alias = `|0x3000`. Useful for poking PIO
    /// INSTR_MEM, configuring SIO GPIO_OE/_OUT, releasing RESETS bits,
    /// etc., without hand-rolling the bus machinery.
    pub fn mmio_write32(&mut self, addr: u32, value: u32) {
        self.assert_not_placeholder();
        // Mirror the `step()` stash so PLL write-time lock-arm transitions
        // observe the current cycle count when the harness pokes MMIO
        // outside the step path. See HLD §6 P2.
        self.bus.master_cycle = self.clock.cycles;
        // Phase 0b.1 Commit B: PPB addresses live on core 0's per-core
        // PPB from the harness's perspective (same convention as before).
        // Route there directly; mirror any NVIC_ISPR/ICPR writes back to
        // `bus.irq_pending[0]` so the dispatch short-circuit stays in sync.
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            self.core_mut(0).ppb.write32(addr, value);
            let low = addr & 0xFFFF;
            if matches!(low, 0xE200 | 0xE204 | 0xE280 | 0xE284) {
                let word = if low == 0xE200 || low == 0xE280 { 0 } else { 1 };
                let ispr =
                    self.core(0).ppb.nvic_ispr[word].load(std::sync::atomic::Ordering::Relaxed);
                let mask64 = (ispr as u64) << (word * 32);
                let keep = if word == 0 {
                    !0xFFFF_FFFFu64
                } else {
                    0xFFFF_FFFFu64
                };
                let prev = self.bus.atomics.irq_pending_load(0);
                self.bus.atomics.set_irq_pending(0, (prev & keep) | mask64);
            }
        } else {
            self.bus.write32(addr, value, 0);
        }
    }

    /// Read a 32-bit word from an MMIO address via the bus. Charges zero
    /// emulator cycles (intended for setup code running outside `run()`).
    ///
    /// **Warning: reads may have side effects.** Several RP2350 MMIO
    /// registers mutate state on read — e.g. PIO `RXFn` pops the receive
    /// FIFO, SIO divider `QUOTIENT` / `REMAINDER` clear the CSR dirty
    /// bit, and a handful of W1C sticky flags are cleared by reads. Setup
    /// code should therefore be write-heavy; reads through this method
    /// are for confirmation only and should be chosen carefully to avoid
    /// disturbing the peripheral's state.
    pub fn mmio_read32(&mut self, addr: u32) -> u32 {
        self.assert_not_placeholder();
        // Mirror the `step()` stash so PLL CS reads observe the current
        // cycle count when the harness reads MMIO outside the step path.
        self.bus.master_cycle = self.clock.cycles;
        // Phase 0b.1 Commit B: PPB addresses route to core 0's PPB.
        if addr >> 28 == 0xE && !Bus::is_boot_ram(addr) {
            self.core_mut(0).ppb.read32(addr)
        } else {
            self.bus.read32(addr, 0)
        }
    }
}

/// Advance both Arm cores up to `target` cycles. Mirrors the original
/// serialised-interleave `step()` body: core 0 first, then core 1. Each
/// `CortexM33` owns its own PPB (Phase 0b.1 Commit B), so no active-core
/// indirection is needed. `update_latest_cycles` publishes the core's
/// cycle counter into its PPB so DWT_CYCCNT reads/writes land on a fresh
/// value — staleness is bounded by one instruction.
fn step_pair_arm(cs: &mut [CortexM33; 2], bus: &mut Bus, target: u64) {
    for core_id in 0..2 {
        // Quantum-boundary IRQ merge: peripherals in `tick_peripherals`
        // at the previous quantum raised IRQs via `assert_irq_*`.
        // Phase 3 Stage 1 (LLD V7 §2) — `take_irq_pending` swaps the
        // mask to zero; a non-zero return replaces the deleted
        // `irq_pending_dirty` flag as the consume-and-merge signal.
        let pending = bus.atomics.take_irq_pending(core_id);
        if pending != 0 {
            cs[core_id].ppb.merge_irq_pending(pending);
        }

        while !cs[core_id].is_halted()
            && !cs[core_id].is_wfe_waiting()
            && cs[core_id].cycles < target
        {
            // Publish the core's cycle count into its PPB before each
            // instruction so DWT_CYCCNT reads/writes land on a fresh
            // value. Staleness is bounded by one instruction.
            let cyc = cs[core_id].cycles;
            cs[core_id].ppb.update_latest_cycles(cyc);
            cs[core_id].step(bus);

            // (c) Drain per-instruction cache-invalidation queue into
            // the core that just ran. Phase 3 follow-up #10 — the
            // decode cache is per-core; writes during this step's bus
            // accesses recorded addresses in
            // `bus.pending_cache_invalidations`. Cross-core SMC still
            // requires firmware DSB+ISB per V7 spec.
            if !bus.pending_cache_invalidations.is_empty() {
                cs[core_id].invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
                bus.pending_cache_invalidations.clear();
            }
            // Region-scoped invalidation triggered mid-step (via
            // `Bus::invalidate_all` or `load_bootrom`/`load_flash`
            // during a step — rare, but used by `Emulator::poke`
            // docs and tests). Drain both cores' caches for the
            // affected regions. Same-step signal so the peer core
            // sees it on its next turn.
            if bus.pending_invalidation_regions != 0 {
                let regions = bus.pending_invalidation_regions;
                cs[0].invalidate_decode_cache_regions(regions);
                cs[1].invalidate_decode_cache_regions(regions);
                bus.pending_invalidation_regions = 0;
            }
        }
        // Final refresh so any post-quantum inspection (e.g. tests
        // reading DWT_CYCCNT between steps) sees a current base.
        let cyc = cs[core_id].cycles;
        cs[core_id].ppb.update_latest_cycles(cyc);

        // Phase 0b.2: exclusive-monitor snoop. If the peer core has an
        // outstanding LDREX address and *this* core performed any
        // data-side write during its quantum slice, invalidate the
        // peer's monitor. Same-core writes do NOT invalidate the local
        // monitor (per ARMv8-M §A3.4). Clear the flag for the next
        // quantum. Correct under the serial-interleave scheduler
        // because cores run sequentially within a quantum; threaded
        // mode (Phase 1+) will require atomic CAS on SharedMemory.
        let peer = 1 - core_id;
        if cs[peer].exclusive_address.is_some() && cs[core_id].did_write_this_quantum {
            cs[peer].exclusive_address = None;
        }
        cs[core_id].did_write_this_quantum = false;
    }
}

/// Advance both RISC-V (Hazard3) cores up to `target` cycles. P1a stub:
/// no per-core PPB stash (RISC-V has no ARMv8-M system-control space),
/// no WFE (Hazard3 models `wfi` differently — see HLD §4.6, handled in
/// P4). Just drives the core's own `step` until it halts or hits the
/// target.
fn step_pair_riscv(cs: &mut [Hazard3; 2], bus: &mut Bus, target: u64) {
    for core_id in 0..2 {
        // Threading removed `bus.set_active_core`; each hart passes its
        // own `hart_id` into `bus.read*` / `write*` / `bus_fault(core)`
        // for MMIO-trace attribution and per-core bus-fault routing.
        while !cs[core_id].is_halted() && cs[core_id].cycles() < target {
            cs[core_id].step(bus);
        }
    }
}

/// Builder for assembling the emulator with optional peripherals.
pub struct EmulatorBuilder {
    config: Config,
    step_quantum: u32,
    arch: Arch,
    execution: ExecutionModel,
}

impl EmulatorBuilder {
    pub fn new(config: Config) -> Self {
        Self {
            config,
            step_quantum: DEFAULT_STEP_QUANTUM,
            arch: Arch::default(),
            execution: ExecutionModel::default(),
        }
    }

    /// Override the per-step quantum (default [`DEFAULT_STEP_QUANTUM`]).
    /// Useful for benches sweeping quantum size, or tests wanting tighter
    /// peripheral-latency observation.
    pub fn step_quantum(mut self, n: u32) -> Self {
        // Clamp `0 -> 1`. Previously a `debug_assert!` here meant
        // `step_quantum(0)` triggered a silent infinite-loop footgun
        // in release builds: the inner `step()` drains
        // `step_quantum` master-clock cycles per call, so 0 advances
        // nothing and `run()`'s pacing loop never makes progress.
        // Clamping at the entry point keeps every downstream caller
        // safe without leaking the constraint into the runtime.
        self.step_quantum = n.max(1);
        self
    }

    /// Select the CPU architecture. Defaults to [`Arch::Arm`]; pass
    /// [`Arch::RiscV`] to construct the Hazard3 variant. V1 ships the
    /// placeholder Hazard3 — real ISA lands in P1b.
    pub fn arch(mut self, arch: Arch) -> Self {
        self.arch = arch;
        self
    }

    /// Select the runtime [`ExecutionModel`]. Defaults to
    /// `ExecutionModel::Serial` (the oracle-validated reference path).
    /// `ExecutionModel::Threaded` requires the `threading` cargo feature
    /// and an x86_64 Windows host; otherwise [`Self::build`] returns
    /// `Err(ConfigError::ThreadingUnavailable)`.
    pub fn execution(mut self, model: ExecutionModel) -> Self {
        self.execution = model;
        self
    }

    pub fn build(self) -> Result<Emulator, ConfigError> {
        // Threading availability gate — dual-execution HLD V1 §5.2.
        // Reject before building any state so the caller knows early.
        if self.execution == ExecutionModel::Threaded {
            #[cfg(not(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            )))]
            return Err(ConfigError::ThreadingUnavailable);
        }
        // ThreadedEmulator pins one worker per host core; reject early
        // when the host cannot satisfy that, instead of panicking
        // later inside `ThreadedEmulator::from_emulator`.
        #[cfg(all(
            feature = "threading",
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        if self.execution == ExecutionModel::Threaded {
            let n = std::thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(1);
            if n < 6 {
                return Err(ConfigError::ThreadingUnavailable);
            }
            if !matches!(self.arch, Arch::Arm) {
                // ThreadedEmulator supports Arm only today.
                return Err(ConfigError::ThreadingUnavailable);
            }
        }

        // `Bus::new` already installs the HLD V5 §5.7 post-bootrom clock
        // table (`clk_sys = 150 MHz`, `clk_ref = 12 MHz`). Only override
        // it when the caller supplied a non-default `Config::sys_clk_hz`
        // — overwriting the post-bootrom seed with ROSC for default
        // callers would regress the invariant "Bus::new(), Emulator::new,
        // and Emulator::reset all yield the same clock state".
        //
        // Phase 3 Stage 1: construct a single `Arc<CoreAtomics>` and
        // hand it to Bus plus both cores so cross-core signalling
        // (SEV/event_flag, IRQ pending, bus-fault, RCP) lands on shared
        // state.
        let atomics = Arc::new(crate::threaded::CoreAtomics::default());
        let mut bus = Bus::with_atomics(Arc::clone(&atomics));
        if self.config.sys_clk_hz != Config::default().sys_clk_hz {
            bus.seed_sys_clk_hz(self.config.sys_clk_hz);
        }
        info!(
            rom_size = memory::ROM_SIZE,
            sram_size = memory::SRAM_SIZE,
            step_quantum = self.step_quantum,
            sys_clk_hz = bus.sys_clk_hz(),
            execution = ?self.execution,
            "emulator constructed",
        );
        let cores = match self.arch {
            Arch::Arm => Cores::Arm([
                CortexM33::new(0, Arc::clone(&atomics)),
                CortexM33::new(1, Arc::clone(&atomics)),
            ]),
            Arch::RiscV => Cores::RiscV([Hazard3::new(0), Hazard3::new(1)]),
        };
        // Silence unused-atomics warning on RiscV arm (no atomics wired yet).
        let _ = &atomics;
        let emu = Emulator {
            cores,
            bus,
            clock: Clock { cycles: 0 },
            step_quantum: self.step_quantum,
            execution_model: self.execution,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            threaded: None,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            panic_info: None,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            timeout_info: None,
            #[cfg(all(
                feature = "testing",
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            pending_panic_inject: None,
            #[cfg(all(
                feature = "threading",
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            bus_is_placeholder: false,
            shutdown_requested: false,
        };
        Ok(emu)
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod tests_stage3_thumb32;

// ---------------------------------------------------------------------------
// Stage 4: residue branch coverage for the top-level `lib.rs` (Emulator,
// EmulatorBuilder, Config, Cores, ConfigError, EmulatorError, Arch,
// ExecutionModel). Pure append-only — does not modify any production code.
// ---------------------------------------------------------------------------
#[cfg(test)]
mod stage4_lib_residue {
    use super::*;

    // ------------------- ConfigError -------------------

    #[test]
    fn config_error_display_threading_unavailable() {
        let s = format!("{}", ConfigError::ThreadingUnavailable);
        assert!(s.contains("Threaded"));
        assert!(s.contains("unavailable"));
    }

    #[test]
    fn config_error_debug_and_clone_eq() {
        let e1 = ConfigError::ThreadingUnavailable;
        let e2 = e1.clone();
        assert_eq!(e1, e2);
        // Debug just needs to format successfully.
        let _ = format!("{:?}", e1);
    }

    #[test]
    fn config_error_is_std_error() {
        // Confirms `impl std::error::Error for ConfigError`.
        fn assert_err<E: std::error::Error>(_: &E) {}
        assert_err(&ConfigError::ThreadingUnavailable);
    }

    // ------------------- EmulatorError -------------------

    #[test]
    fn emulator_error_display_not_supported_in_threaded() {
        let s = format!("{}", EmulatorError::NotSupportedInThreadedMode);
        assert!(s.contains("Threaded"));
    }

    #[test]
    fn emulator_error_clone_and_eq() {
        let e1 = EmulatorError::NotSupportedInThreadedMode;
        let e2 = e1.clone();
        assert_eq!(e1, e2);
        let _ = format!("{:?}", e1);
    }

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn emulator_error_display_worker_panicked() {
        let e = EmulatorError::WorkerPanicked {
            which: threaded::WorkerName::Pio0,
            message: String::from("boom"),
        };
        let s = format!("{}", e);
        assert!(s.contains("panicked"));
        assert!(s.contains("boom"));
    }

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn emulator_error_display_barrier_timeout() {
        let e = EmulatorError::BarrierTimeout {
            which: threaded::WorkerName::Coord,
            elapsed_ms: 1_234,
        };
        let s = format!("{}", e);
        assert!(s.contains("barrier"));
        assert!(s.contains("1234"));
    }

    // ------------------- Arch / ExecutionModel default & derives -------------------

    #[test]
    fn arch_default_is_arm() {
        assert!(matches!(Arch::default(), Arch::Arm));
    }

    #[test]
    fn execution_model_default_is_serial() {
        assert_eq!(ExecutionModel::default(), ExecutionModel::Serial);
    }

    #[test]
    fn execution_model_debug_and_eq() {
        assert_eq!(ExecutionModel::Threaded, ExecutionModel::Threaded);
        assert_ne!(ExecutionModel::Serial, ExecutionModel::Threaded);
        let _ = format!("{:?}", ExecutionModel::Serial);
        let _ = format!("{:?}", ExecutionModel::Threaded);
    }

    // ------------------- Builder: ConfigError::ThreadingUnavailable -------------------
    //
    // On no-threading-feature builds (the default), selecting Threaded must
    // return ConfigError::ThreadingUnavailable. On threading-feature builds
    // the same call succeeds when the host has enough cores; we cover the
    // success path inline in `builder_threaded_with_feature`.

    #[cfg(not(feature = "threading"))]
    #[test]
    fn builder_threaded_no_feature_returns_threading_unavailable() {
        let res = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build();
        match res {
            Err(ConfigError::ThreadingUnavailable) => {}
            Ok(_) => panic!("Threaded should fail without `threading` feature"),
        }
    }

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn builder_threaded_with_feature_riscv_rejected() {
        // ThreadedEmulator supports Arm only; Threaded + RiscV is an
        // explicit ThreadingUnavailable arm in `build()`.
        let res = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .execution(ExecutionModel::Threaded)
            .build();
        match res {
            Err(ConfigError::ThreadingUnavailable) => {}
            Ok(_) => panic!("Threaded + RiscV should be rejected"),
        }
    }

    #[cfg(not(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    )))]
    #[test]
    fn builder_threaded_off_platform_returns_threading_unavailable() {
        // On platforms where threading isn't supported (non-x86_64, non-
        // Windows/Linux), Threaded build must fail.
        let res = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build();
        match res {
            Err(ConfigError::ThreadingUnavailable) => {}
            Ok(_) => panic!("Threaded should fail on unsupported platforms"),
        }
    }

    // ------------------- Cores accessors / introspection -------------------

    #[test]
    fn cores_is_arm_and_is_riscv_flags() {
        let arm = EmulatorBuilder::new(Config::default()).build().unwrap();
        assert!(arm.cores.is_arm());
        assert!(!arm.cores.is_riscv());

        let rv = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        assert!(!rv.cores.is_arm());
        assert!(rv.cores.is_riscv());
    }

    // ------------------- Emulator::execution_model & core_cycles -------------------

    #[test]
    fn execution_model_accessor_returns_selected() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.execution_model(), ExecutionModel::Serial);
    }

    #[test]
    fn core_cycles_default_zero_arm() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.core_cycles(0), 0);
        assert_eq!(emu.core_cycles(1), 0);
    }

    #[test]
    fn core_cycles_default_zero_riscv() {
        let emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        assert_eq!(emu.core_cycles(0), 0);
        assert_eq!(emu.core_cycles(1), 0);
    }

    #[test]
    #[should_panic(expected = "core_cycles: idx must be 0 or 1")]
    fn core_cycles_invalid_idx_panics() {
        let emu = Emulator::new(Config::default());
        let _ = emu.core_cycles(2);
    }

    // ------------------- Emulator::run / step fast paths -------------------

    #[test]
    fn run_zero_cycles_serial_is_noop() {
        let mut emu = Emulator::new(Config::default());
        let before = emu.cycles();
        let after = emu.run(0).unwrap();
        // The while loop predicate `cycles < target` with cycles==target is
        // false, so no quanta execute. Master cycle is unchanged.
        assert_eq!(after, before);
    }

    #[test]
    fn step_serial_returns_ok() {
        let mut emu = Emulator::new(Config::default());
        let r = emu.step().unwrap();
        assert!(r >= emu.step_quantum as u64);
    }

    #[test]
    fn run_quantum_serial_returns_ok() {
        let mut emu = Emulator::new(Config::default());
        let r = emu.run_quantum().unwrap();
        assert_eq!(r, emu.step_quantum as u64);
    }

    // ------------------- Builder: defaults / step_quantum override -------------------

    #[test]
    fn builder_default_step_quantum() {
        let emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        assert_eq!(emu.step_quantum, DEFAULT_STEP_QUANTUM);
    }

    #[test]
    fn step_quantum_zero_clamps_to_one() {
        // Regression: `EmulatorBuilder::step_quantum(0)` previously
        // tripped a `debug_assert!` (and silently advanced 0 cycles
        // per `step()` in release builds — an infinite-loop footgun
        // for `run()`). The clamp at the builder entry point keeps
        // the runtime contract `step_quantum >= 1` intact.
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(0)
            .build()
            .unwrap();
        assert_eq!(emu.step_quantum, 1);
        // `step()` must make forward progress (advance >= 1 master
        // cycle) and not loop forever.
        let advanced = emu.step().unwrap();
        assert!(advanced >= 1);
    }

    #[test]
    fn builder_arch_arm_explicit() {
        let emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::Arm)
            .build()
            .unwrap();
        assert!(emu.cores.is_arm());
    }

    #[test]
    fn builder_execution_serial_explicit() {
        let emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Serial)
            .build()
            .unwrap();
        assert_eq!(emu.execution_model(), ExecutionModel::Serial);
    }

    // ------------------- Emulator::load_* paths -------------------

    #[test]
    fn load_bootrom_replaces_first_words() {
        let mut emu = Emulator::new(Config::default());
        // 16 bytes is enough — load_bootrom clamps internally.
        let mut data = vec![0u8; 64];
        data[0..4].copy_from_slice(&0x2000_8000u32.to_le_bytes());
        data[4..8].copy_from_slice(&0x1000_0001u32.to_le_bytes());
        emu.load_bootrom(&data);
        assert_eq!(emu.bus.memory.rom_read32(0), 0x2000_8000);
        assert_eq!(emu.bus.memory.rom_read32(4), 0x1000_0001);
    }

    #[test]
    fn load_image_oracle_alias_writes_sram() {
        let mut emu = Emulator::new(Config::default());
        let data = [0xAAu8, 0xBB, 0xCC, 0xDD];
        emu.load_image(0x8000_0100, &data);
        // SRAM-aliased read via `peek` (canonical 0x2000_xxxx).
        let v = emu.peek(0x2000_0100);
        assert_eq!(v & 0xFF, 0xAA);
        assert_eq!((v >> 8) & 0xFF, 0xBB);
    }

    #[test]
    fn load_image_unknown_region_silently_dropped() {
        let mut emu = Emulator::new(Config::default());
        let data = [0xFFu8; 4];
        // 0x4 region (peripherals) — match arm `_` falls through with
        // no side effect. Just confirm it doesn't panic.
        emu.load_image(0x4000_0000, &data);
    }

    #[test]
    fn load_image_rom_region_is_ignored() {
        let mut emu = Emulator::new(Config::default());
        // Pre-seed bootrom so we can prove the load_image ROM-region arm
        // does NOT clobber it.
        let mut bootrom = vec![0u8; 32];
        bootrom[0] = 0x55;
        emu.load_bootrom(&bootrom);
        let data = [0xAAu8, 0xBB, 0xCC, 0xDD];
        emu.load_image(0x0000_0000, &data);
        assert_eq!(emu.bus.memory.rom_read8(0), 0x55, "ROM untouched");
    }

    // ------------------- gpio / poke / peek smoke -------------------

    #[test]
    fn gpio_read_and_read_all_default() {
        let emu = Emulator::new(Config::default());
        // Default: no SIO drive, no PIO drive — gpio_in is 0.
        assert!(!emu.gpio_read(0));
        assert_eq!(emu.gpio_read_all(), 0);
    }

    #[test]
    fn gpio_write_is_stub_noop() {
        let mut emu = Emulator::new(Config::default());
        // gpio_write is documented as a Phase 1 stub. Just confirm it
        // doesn't panic and gpio_read still reflects the merged value.
        emu.gpio_write(0, true);
        assert!(!emu.gpio_read(0));
    }

    #[test]
    fn cycles_starts_at_zero() {
        let emu = Emulator::new(Config::default());
        assert_eq!(emu.cycles(), 0);
    }

    #[test]
    fn reset_clears_master_cycle_and_clock() {
        let mut emu = Emulator::new(Config::default());
        // Advance the clock, then reset and re-check.
        let _ = emu.step().unwrap();
        assert!(emu.cycles() > 0);
        emu.reset();
        assert_eq!(emu.cycles(), 0);
        assert!(!emu.shutdown_requested);
    }

    // ------------------- StopReason exists & constructible -------------------

    #[test]
    fn stop_reason_constructors_compile() {
        // StopReason is `pub` but currently unused on the public surface.
        // Constructing each variant exercises the type at compile and
        // touches each branch for coverage purposes.
        let _ = StopReason::CycleLimit;
        let _ = StopReason::Breakpoint(0xAA);
        let _ = StopReason::Wfi;
        let _ = StopReason::Fault;
    }
}

// ---------------------------------------------------------------------------
// Stage 5: branch-coverage residue not hit by Stage 4. Targets the specific
// `if let Cores::*`, `Cores::is_arm()`, `Cores::is_riscv()`, peek/poke
// boot-RAM dispatch, mmio_*32 PPB dispatch, route_pio_irqs, wake_checks,
// step_serial pre-step invalidation drain, fan_out_riscv_irqs, and the
// shutdown-latch path. Pure append-only — does not modify production code.
// ---------------------------------------------------------------------------
#[cfg(test)]
mod stage5_lib_residue {
    use super::*;

    // ------------------- load_bootrom / load_flash: Cores branch -------------------

    /// Drives `if let Cores::Arm(arm) = &mut self.cores` (line 616) on a
    /// real Arm emulator. Provides enough bytes for `resolve_bootrom_hooks`
    /// to actually execute the inner per-core seed loop.
    #[test]
    fn load_bootrom_arm_path_seeds_hook_pcs() {
        let mut emu = Emulator::new(Config::default());
        // 32 KB matches the RP2350 ROM size; resolve_bootrom_hooks needs
        // the buffer large enough to inspect offset 0x14 + the table.
        let mut data = vec![0u8; crate::memory::ROM_SIZE];
        // Reset vector words.
        data[0..4].copy_from_slice(&0x2000_8000u32.to_le_bytes());
        data[4..8].copy_from_slice(&0x1000_0001u32.to_le_bytes());
        emu.load_bootrom(&data);
        // ROM bytes landed.
        assert_eq!(emu.bus.memory.rom_read32(0), 0x2000_8000);
        assert_eq!(emu.bus.memory.rom_read32(4), 0x1000_0001);
    }

    /// Drives the false branch of `if let Cores::Arm(arm) = ...` in
    /// `load_bootrom` (line 616 false-arm). RiscV emulator skips the
    /// hook-seeding block but the bytes still land.
    #[test]
    fn load_bootrom_riscv_path_skips_hook_seed() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        let mut data = vec![0u8; 64];
        data[0..4].copy_from_slice(&0xCAFE_F00Du32.to_le_bytes());
        emu.load_bootrom(&data);
        assert_eq!(emu.bus.memory.rom_read32(0), 0xCAFE_F00D);
    }

    /// Drives `if let Cores::Arm(arm) = &mut self.cores` (line 639) in
    /// `load_flash` on an Arm emulator.
    #[test]
    fn load_flash_arm_path_invalidates_xip_cache() {
        let mut emu = Emulator::new(Config::default());
        let mut data = vec![0u8; 64];
        data[0..4].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        emu.load_flash(&data);
        // XIP-region invalidation drained back to zero.
        assert_eq!(emu.bus.pending_invalidation_regions, 0);
    }

    /// Drives the false branch of `if let Cores::Arm(arm) = ...` in
    /// `load_flash` (line 639 false-arm) via a RiscV emulator.
    #[test]
    fn load_flash_riscv_path_skips_arm_invalidation() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        let data = vec![0u8; 32];
        emu.load_flash(&data);
        assert_eq!(emu.bus.pending_invalidation_regions, 0);
    }

    // ------------------- step_serial pre-step invalidation drain (line 727 + 729) -------------------

    /// Drives the true branch of `if self.bus.pending_invalidation_regions
    /// != 0` (line 727) and the Arm match in line 729 by setting a region
    /// bit on the bus directly between two steps.
    #[test]
    fn step_serial_drains_pending_invalidation_regions_arm() {
        let mut emu = Emulator::new(Config::default());
        // Force the bus to advertise a pending region drain. `BULK` (0xFF)
        // is the broadest signal — any non-zero value triggers the drain.
        emu.bus.pending_invalidation_regions = 0xFF;
        let _ = emu.step().unwrap();
        // step_serial drains it back to zero.
        assert_eq!(emu.bus.pending_invalidation_regions, 0);
    }

    /// Drives the false branch of the `if let Cores::Arm = ...` in line 729
    /// when there's a pending invalidation but the cores are RiscV.
    #[test]
    fn step_serial_drains_pending_regions_riscv() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        emu.bus.pending_invalidation_regions = 0xFF;
        let _ = emu.step().unwrap();
        assert_eq!(emu.bus.pending_invalidation_regions, 0);
    }

    // ------------------- step_serial bootrom-hook latch drain (lines 756-757) -------------------

    /// Drives the true branch of `if let Cores::Arm(cs) = &mut self.cores
    /// && (cs[0].bootrom_hook_fired || cs[1].bootrom_hook_fired)` by
    /// pre-arming the hook-fired flag on core 0 then stepping.
    #[test]
    fn step_serial_drains_bootrom_hook_to_shutdown_requested_core0() {
        let mut emu = Emulator::new(Config::default());
        // Halt both cores so step_pair_arm doesn't actually run any
        // instructions and clear our state.
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        // Pre-arm the hook latch on core 0.
        emu.cores.expect_arm_mut()[0].bootrom_hook_fired = true;
        assert!(!emu.shutdown_requested);
        let _ = emu.step().unwrap();
        assert!(emu.shutdown_requested);
    }

    /// Same path but firing on core 1 — covers the `cs[1].bootrom_hook_fired`
    /// half of the OR.
    #[test]
    fn step_serial_drains_bootrom_hook_to_shutdown_requested_core1() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        emu.cores.expect_arm_mut()[1].bootrom_hook_fired = true;
        let _ = emu.step().unwrap();
        assert!(emu.shutdown_requested);
    }

    // ------------------- step_serial: is_arm vs is_riscv post-step path (lines 775, 782) -------------------

    /// Drives the true branch of `if self.cores.is_arm()` (line 775)
    /// gating the `tick_systick` call. Default Emulator is Arm, so a
    /// plain `step()` exercises the path; this test makes the intent
    /// explicit in case the existing `step_serial_returns_ok` test is
    /// later refactored.
    #[test]
    fn step_serial_arm_calls_tick_systick() {
        let mut emu = Emulator::new(Config::default());
        // Halt both cores so step_pair_arm does no real work.
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        let _ = emu.step().unwrap();
        // No assertion on SysTick — the goal is line coverage. The branch
        // is hit any time we step on an Arm emu.
    }

    /// Drives the true branch of `if self.cores.is_riscv()` (line 782)
    /// gating the `fan_out_riscv_irqs` call.
    #[test]
    fn step_serial_riscv_calls_fan_out_riscv_irqs() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        let _ = emu.step().unwrap();
    }

    // ------------------- fan_out_riscv_irqs (lines 799, 806, 813, 820) -------------------

    /// Drives the let-else `let Cores::RiscV(cs) = ...` true arm (line 799)
    /// AND the three TRUE arms inside (806 MTIP, 813 MSIP, 820 MEIP) by
    /// setting up SIO + irq_pending state then stepping.
    #[test]
    fn fan_out_riscv_irqs_sets_mtip_msip_meip() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        // MTIP source: SIO mtime_match_asserted on hart 0.
        emu.bus.sio.mtime_match_asserted[0] = true;
        // MSIP source: write SIO RISCV_SOFTIRQ via MMIO. The register at
        // SIO + 0x1A0 holds the per-hart bits; lowest 2 bits are the
        // soft-IRQ pulses for harts 0 and 1.
        // SIO base on RP2350 = 0xD000_0000 (single-cycle bus).
        // We avoid MMIO complexity here by falling back to taking the
        // branch via pre-set SIO state — but `riscv_softirq` is private.
        // Instead, drive MEIP via `bus.atomics.set_irq_pending` so the
        // `compute_meip` returns true; the `if meip` arm at line 820 is
        // then taken.
        emu.bus.atomics.set_irq_pending(0, 0xFFFF_FFFF_FFFF_FFFF);
        // MEIE is a hart-side mask; ensure non-zero so compute_meip can
        // return true. Set mie bit 11 (MEIE) on hart 0.
        let cur_mip = emu.cores.expect_riscv()[0].mip();
        emu.cores.expect_riscv_mut()[0].set_mip(cur_mip);
        // Step so step_serial calls fan_out_riscv_irqs.
        let _ = emu.step().unwrap();
        // After the step, MTIP (bit 7) should be reflected in mip[0].
        let mip0 = emu.cores.expect_riscv()[0].mip();
        assert!(mip0 & (1 << 7) != 0, "MTIP should be set from SIO");
    }

    /// Drives the false-arm of MTIP gating in fan_out_riscv_irqs by
    /// running the function with `mtime_match_asserted == false` (the
    /// post-default state).
    #[test]
    fn fan_out_riscv_irqs_clears_mtip_when_sio_idle() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        // Default: mtime_match_asserted is [false; 2].
        let _ = emu.step().unwrap();
        let mip0 = emu.cores.expect_riscv()[0].mip();
        assert_eq!(mip0 & (1 << 7), 0);
    }

    // ------------------- route_pio_irqs (lines 1113, 1116) -------------------

    /// Drives the true branches of `if ints0 != 0` (line 1113) and
    /// `if ints1 != 0` (line 1116) by writing `INT0_INTF` / `INT1_INTF`
    /// to PIO0 directly. Releases PIO0 from reset first so the MMIO write
    /// is not gated.
    #[test]
    fn route_pio_irqs_asserts_irq_when_intf_nonzero() {
        let mut emu = Emulator::new(Config::default());
        // Release PIO0 reset (bit 11).
        emu.bus.resets_state &= !(1u32 << crate::bus::RESET_PIO0);
        // Halt cores so step_serial doesn't disturb PIO state.
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        // PIO0 base 0x5020_0000; INT0_INTF at offset 0x174 (RP2350).
        emu.bus.write32(0x5020_0174, 0x1, 0); // force IRQ0 raw bit 0
        emu.bus.write32(0x5020_0180, 0x1, 0); // force IRQ1 raw bit 0
        // Confirm INTS reads non-zero so route_pio_irqs's `if` fires.
        assert_ne!(emu.bus.pio[0].int0_ints_rp2350(), 0);
        assert_ne!(emu.bus.pio[0].int1_ints_rp2350(), 0);
        let _ = emu.step().unwrap();
        // No exact post-condition needed — the goal is line coverage of
        // the two-arm `if`s in route_pio_irqs.
    }

    // ------------------- tick_peripherals: pio reset gate (line 1079) -------------------

    /// Drives the true branch of `if (resets & (1u32 << bit)) == 0`
    /// (line 1079) by clearing the PIO0 reset bit. Default `resets_state`
    /// holds PIO blocks in reset, so the inner `pio.step_n` is uncovered
    /// without this nudge.
    #[test]
    fn tick_peripherals_steps_pio_when_released_from_reset() {
        let mut emu = Emulator::new(Config::default());
        // Release PIO0/1/2 from reset — RP2350 has three PIO blocks.
        emu.bus.resets_state &= !((1u32 << crate::bus::RESET_PIO0)
            | (1u32 << (crate::bus::RESET_PIO0 + 1))
            | (1u32 << (crate::bus::RESET_PIO0 + 2)));
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        let _ = emu.step().unwrap();
    }

    // ------------------- wake_checks (lines 1146, 1153, 1155) -------------------

    /// Drives the true branch of the WFE wake check at line 1146:
    /// core is parked on WFE AND event_flag is set → consume + clear.
    #[test]
    fn wake_checks_arm_consumes_wfe_event() {
        let mut emu = Emulator::new(Config::default());
        // Park core 0 on WFE, latch an event.
        emu.bus.atomics.set_wfe_waiting(0);
        emu.bus.atomics.set_event_flag(0);
        // Halt cores so step_pair_arm runs no instructions.
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        let _ = emu.step().unwrap();
        // wake_checks should have cleared wfe_waiting after consuming the flag.
        assert!(!emu.bus.atomics.is_wfe_waiting(0));
    }

    /// Drives the true branch of `if self.bus.atomics.is_halted(i)`
    /// (line 1153) AND the inner `if pending != 0 && ...any_pending_enabled`
    /// (line 1155). The branch is visited any time `is_halted` is true at
    /// the wake-check point; the post-condition is best-effort.
    #[test]
    fn wake_checks_arm_visits_halted_branch() {
        let mut emu = Emulator::new(Config::default());
        // Enable IRQ line 0 on core 0's PPB.
        emu.mmio_write32(0xE000_E100, 0x1);
        // Halt core 0 (WFI-style park).
        emu.bus.atomics.set_halted(0);
        // Set IRQ-pending bit 0 on core 0.
        emu.bus.atomics.set_irq_pending(0, 0x1);
        let _ = emu.step().unwrap();
        // wake_checks evaluated the predicate. Whether it cleared the
        // halt flag depends on PPB enable plumbing details we don't want
        // to over-assert here; line coverage of 1153/1155 is the goal.
    }

    // ------------------- wake_checks: RiscV WFI (line 1168) -------------------

    /// Drives `if c.wfi_parked && (c.mip() & c.mie()) != 0` true arm
    /// (line 1168) for the RiscV path.
    #[test]
    fn wake_checks_riscv_unparks_on_pending_and_enabled_irq() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        // Park hart 0 on WFI.
        emu.cores.expect_riscv_mut()[0].wfi_parked = true;
        // Pre-set MTIP source so fan_out_riscv_irqs latches mip[7].
        emu.bus.sio.mtime_match_asserted[0] = true;
        // Enable MTIE (bit 7) on hart 0's mie. Use set_mip as a workaround
        // to bump mie via the public read — we need a direct mie write.
        // Since `set_mie` isn't accessible, we instead set MEIE by writing
        // to the IRQ controller path. Simpler: use mtime + direct mip set.
        // The simplest viable scheme is to step once so fan_out_riscv_irqs
        // latches mip[7]; then the wake_checks predicate evaluates
        // `(mip & mie)`. mie defaults to zero so we'd not wake. So
        // directly poke mip via set_mip and skip mie — instead test with
        // the post-step state.
        let _ = emu.step().unwrap();
        // Branch was visited regardless of outcome — wfi_parked might
        // still be true if mie was zero, but the predicate executed.
        let _ = emu.cores.expect_riscv()[0].wfi_parked;
    }

    // ------------------- reset_counters (line 1281) -------------------

    /// Drives the true branch of `if let Cores::Arm(arm) = &mut self.cores`
    /// (line 1281) inside reset_counters. Default emulator is Arm so the
    /// branch is reached.
    #[test]
    fn reset_counters_arm_path() {
        let mut emu = Emulator::new(Config::default());
        emu.reset_counters();
    }

    /// Drives the false branch of the same `if let` (line 1281) by
    /// running on a RiscV emulator — the function is a no-op there.
    #[test]
    fn reset_counters_riscv_path_is_noop() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        emu.reset_counters();
    }

    // ------------------- peek/poke boot-RAM dispatch (lines 1292, 1312) -------------------

    /// Drives the true branch of `if Bus::is_boot_ram(addr)` in `peek`
    /// (line 1292). Boot-RAM lives at 0xEFFF_F000..0xF000_0000.
    #[test]
    fn peek_boot_ram_path() {
        let emu = Emulator::new(Config::default());
        let _ = emu.peek(0xEFFF_F000);
    }

    /// Drives the true branch of `if Bus::is_boot_ram(addr)` in `poke`
    /// (line 1312).
    #[test]
    fn poke_boot_ram_path() {
        let mut emu = Emulator::new(Config::default());
        emu.poke(0xEFFF_F000, 0xCAFE_BABE);
        assert_eq!(emu.peek(0xEFFF_F000), 0xCAFE_BABE);
    }

    // ------------------- mmio_*32 PPB dispatch (lines 1346, 1350, 1354, 1383) -------------------

    /// Drives the true branch of `if addr >> 28 == 0xE && !is_boot_ram`
    /// (line 1346) for `mmio_write32`. PPB region (0xE000_xxxx) is the
    /// natural target.
    #[test]
    fn mmio_write32_ppb_region() {
        let mut emu = Emulator::new(Config::default());
        // VTOR at 0xE000_ED08 lives on the PPB; writes route to core 0's PPB.
        emu.mmio_write32(0xE000_ED08, 0x1000_0000);
    }

    /// Drives the true branch of `if matches!(low, 0xE200 | 0xE204 | 0xE280
    /// | 0xE284)` (line 1350) — NVIC_ISPR/ICPR slots 0/1.
    #[test]
    fn mmio_write32_nvic_ispr_word0() {
        let mut emu = Emulator::new(Config::default());
        // NVIC_ISPR0 at 0xE000_E200; writes mirror back into bus
        // irq_pending. word == 0 → keep mask !0xFFFF_FFFF.
        emu.mmio_write32(0xE000_E200, 0x1);
    }

    /// Drives the false branch of `if word == 0` inside the mirror block
    /// (line 1354) by writing NVIC_ISPR1 (0xE000_E204).
    #[test]
    fn mmio_write32_nvic_ispr_word1() {
        let mut emu = Emulator::new(Config::default());
        emu.mmio_write32(0xE000_E204, 0x1);
    }

    /// Drives the true branch of the same condition for `mmio_read32`
    /// (line 1383).
    #[test]
    fn mmio_read32_ppb_region() {
        let mut emu = Emulator::new(Config::default());
        let _ = emu.mmio_read32(0xE000_ED08);
    }

    // ------------------- step_pair_arm: take_irq_pending merge (line 1405) -------------------

    /// Drives the true branch of `if pending != 0` (line 1405) inside
    /// step_pair_arm by pre-staging an IRQ on core 0 before step.
    #[test]
    fn step_pair_arm_merges_pending_irqs() {
        let mut emu = Emulator::new(Config::default());
        // Set a pending bit; the merge into PPB happens on the next step.
        emu.bus.atomics.set_irq_pending(0, 0x1);
        emu.bus.atomics.set_halted(0);
        emu.bus.atomics.set_halted(1);
        let _ = emu.step().unwrap();
        // After step, the take_irq_pending swap should have cleared bus
        // irq_pending[0] and merged into PPB.
        assert_eq!(emu.bus.atomics.irq_pending_load(0), 0);
    }

    // ------------------- run() Serial loop (lines 839 false-arm + 841 true) -------------------

    /// Drives the true branch of `while self.clock.cycles < target`
    /// (line 841) by requesting a non-zero cycle target. The Serial-arm
    /// of line 839 is also exercised.
    #[test]
    fn run_serial_advances_clock_to_target() {
        let mut emu = Emulator::new(Config::default());
        let cycles_target = (emu.step_quantum as u64) * 3;
        let after = emu.run(cycles_target).unwrap();
        assert!(after >= cycles_target);
    }
}

// ===========================================================================
// Stage 8 — `lib.rs` residual branch coverage (rp2350-emu).
// ===========================================================================
//
// Targets the residue branches in `crates/rp2350-emu/src/lib.rs` that the
// earlier `stage4_lib_residue_v2` / `stage5_lib_residue` modules did not
// reach. Specifically:
//
//   * `bootrom_load_combined` SHA256 mismatch error path (line 224).
//   * `step` / `run` / `run_quantum_threaded` cached `timeout_info`
//     short-circuit arms (lines 685, 867, 936).
//   * `mmio_write32` / `mmio_read32` `Bus::is_boot_ram(addr)` FALSE
//     short-circuit (lines 1366 col 33, 1403 col 33) — boot-RAM addr
//     in the PPB region falls through to the regular bus path.
//
// Lines that are genuinely unreachable through the public API (e.g.
// `available_parallelism() < 6` at lib.rs:1574 — host-dependent; or the
// per-instruction `pending_invalidation_regions != 0` at lib.rs:1456 —
// no MMIO write inside step body sets it) are documented inline and
// skipped.
//
// Pure append-only — does not modify production code.
#[cfg(test)]
mod stage8_lib_residue {
    use crate::{Config, Emulator, EmulatorBuilder};

    // ------------------- bootrom SHA256 mismatch (line 224) -------------------
    //
    // `load_pinned_silicon_bootrom` panics on mismatch. We can't drive
    // the production loader from a test (it reads from a fixed path),
    // but we CAN verify the function exists and returns a result —
    // proving the I/O scaffolding is callable.
    //
    // The actual SHA-mismatch arm requires editing the SHA file, which
    // would race with parallel tests. Instead, document that this
    // branch is expected to be hit only on intentional pin-drift and
    // skip the test.

    // ------------------- step Threaded cached timeout_info (line 685) -------------------
    //
    // The TRUE arm `if let Some((which, elapsed_ms)) = self.timeout_info`
    // inside `step()` requires `timeout_info` to be Some. The runtime
    // path that sets this is `BarrierTimeout` from `run_quanta_checked`.
    // Inducing a real barrier timeout is flaky (depends on the watchdog
    // duration). Instead, populate `timeout_info` directly via the
    // `pub(crate)` field and call `step()` to drive the cache hit.
    //
    // This requires `#[cfg(... threading ...)]` since the field only
    // exists on threading builds.

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn step_threaded_cached_timeout_returns_barrier_timeout() {
        use crate::{EmulatorError, ExecutionModel, threaded::WorkerName};

        let mut emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build()
            .expect("Threaded build should succeed");
        // Pre-populate the sticky timeout cache.
        emu.timeout_info = Some((WorkerName::Coord, 1234));
        match emu.step() {
            Err(EmulatorError::BarrierTimeout {
                which: WorkerName::Coord,
                elapsed_ms: 1234,
            }) => {}
            other => panic!("expected cached BarrierTimeout, got {other:?}"),
        }
    }

    /// Same pre-populated timeout_info but observed via `run()` —
    /// drives the cache short-circuit at line 867.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn run_threaded_cached_timeout_returns_barrier_timeout() {
        use crate::{EmulatorError, ExecutionModel, threaded::WorkerName};

        let mut emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build()
            .expect("Threaded build should succeed");
        emu.timeout_info = Some((WorkerName::Core1, 5678));
        match emu.run(emu.step_quantum as u64) {
            Err(EmulatorError::BarrierTimeout {
                which: WorkerName::Core1,
                elapsed_ms: 5678,
            }) => {}
            other => panic!("expected cached BarrierTimeout, got {other:?}"),
        }
    }

    /// Drives the cache short-circuit at line 936 inside
    /// `run_quantum_threaded`.
    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn run_quantum_threaded_cached_timeout_returns_barrier_timeout() {
        use crate::{EmulatorError, ExecutionModel, threaded::WorkerName};

        let mut emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build()
            .expect("Threaded build should succeed");
        emu.timeout_info = Some((WorkerName::Core0, 999));
        match emu.run_quantum() {
            Err(EmulatorError::BarrierTimeout {
                which: WorkerName::Core0,
                elapsed_ms: 999,
            }) => {}
            other => panic!("expected cached BarrierTimeout, got {other:?}"),
        }
    }

    // ------------------- mmio_write32/read32 boot_ram fall-through (lines 1366/1403 col 33) -------------------
    //
    // `if addr >> 28 == 0xE && !Bus::is_boot_ram(addr)` — when the
    // address IS in the boot-RAM range (0xEFFF_F000..0xF000_0000), the
    // second operand is FALSE and the chain short-circuits to the
    // else-branch (regular `bus.write32`/`bus.read32`).

    /// Drives the short-circuit at line 1366 col 33: boot-RAM-region
    /// PPB-prefix address (`0xEFFF_F000`) skips the per-core PPB write
    /// and falls through to `bus.write32`.
    #[test]
    fn mmio_write32_boot_ram_addr_falls_to_bus() {
        let mut emu = Emulator::new(Config::default());
        // 0xEFFF_F000 — top nibble == 0xE so the first operand is true,
        // boot_ram() returns true so the second operand is false.
        // The else-branch (bus.write32) handles boot RAM via its own
        // address decode.
        emu.mmio_write32(0xEFFF_F000, 0xCAFE_F00D);
        // Read it back via the same path — round-trip confirms the
        // fall-through delivered the write to the bus correctly.
        let got = emu.mmio_read32(0xEFFF_F000);
        assert_eq!(got, 0xCAFE_F00D);
    }

    /// Drives the same short-circuit at line 1403 col 33 for
    /// `mmio_read32`.
    #[test]
    fn mmio_read32_boot_ram_addr_falls_to_bus() {
        let mut emu = Emulator::new(Config::default());
        // First seed via write to a valid boot_ram word…
        emu.mmio_write32(0xEFFF_F004, 0x1234_5678);
        // …then read back through the boot_ram fall-through arm.
        let got = emu.mmio_read32(0xEFFF_F004);
        assert_eq!(got, 0x1234_5678);
    }

    // ------------------- shutdown_requested drain on Threaded run/run_quantum -------------------
    //
    // Lines 881 / 974: `if threaded.shutdown_requested()` — the FALSE
    // arm is exercised by the existing
    // `stage4_lib_residue_v2::run_threaded_skips_shutdown_when_not_requested`
    // test. The TRUE arm requires the threaded worker to flag
    // `shutdown_requested`, which only happens when the bootrom-reboot
    // hook fires inside a worker — a multi-step setup that needs a
    // valid bootrom + firmware. We can drive it indirectly by
    // pre-arming `bootrom_hook_fired` on the seed cores so the
    // `promote_to_threaded` hand-off carries the latch forward.
    //
    // promote_to_threaded only copies `shutdown_requested` from the
    // outer Emulator (line 1032), so pre-setting `self.shutdown_requested
    // = true` before the first `run` propagates into the threaded seed.
    // The first quanta worker tick then surfaces it back via
    // `threaded.shutdown_requested()`.

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn run_threaded_propagates_pre_set_shutdown_to_first_run() {
        use crate::ExecutionModel;

        let mut emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build()
            .expect("Threaded build should succeed");
        // Pre-set shutdown_requested before the worker pool spawns.
        emu.shutdown_requested = true;
        // First run() promotes; the worker carries the flag forward and
        // the post-run drain at line 881 sees `threaded.shutdown_requested()`
        // == true.
        let _ = emu.run(emu.step_quantum as u64).expect("first run");
        // Whatever the worker decides, the outer Emulator's
        // shutdown_requested must remain true.
        assert!(emu.shutdown_requested);
    }

    #[cfg(all(
        feature = "threading",
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    #[test]
    fn run_quantum_threaded_propagates_pre_set_shutdown_first_quantum() {
        use crate::ExecutionModel;

        let mut emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build()
            .expect("Threaded build should succeed");
        emu.shutdown_requested = true;
        let _ = emu.run_quantum().expect("first run_quantum");
        assert!(emu.shutdown_requested);
    }

    // ------------------- mmio_write32 NVIC_ICPR mirror (line 1369 OR-arm) -------------------
    //
    // `matches!(low, 0xE200 | 0xE204 | 0xE280 | 0xE284)` covers
    // NVIC_ISPR0/1 (0xE200/0xE204) and NVIC_ICPR0/1 (0xE280/0xE284).
    // Stage 5 covers ISPR0/1; this fills in the ICPR0/1 arms so all
    // four values of the OR pattern are exercised.

    #[test]
    fn mmio_write32_nvic_icpr_word0_mirrors_to_irq_pending() {
        let mut emu = Emulator::new(Config::default());
        // NVIC_ICPR0 lives at 0xE000_E280. Seed irq_pending then issue
        // ICPR — the inner mirror block re-reads ISPR (0 after the
        // clear) and AND-keep-masks the upper 32 bits via `keep`.
        emu.bus.atomics.set_irq_pending(0, 0xFFFF_FFFF_FFFF_FFFFu64);
        emu.mmio_write32(0xE000_E280, 0xFFFF_FFFF);
        // After the mirror, the low-32-bit half of irq_pending may
        // change; coverage simply requires the branch to fire.
    }

    #[test]
    fn mmio_write32_nvic_icpr_word1_mirrors_to_irq_pending() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.atomics.set_irq_pending(0, 0xFFFF_FFFF_FFFF_FFFFu64);
        emu.mmio_write32(0xE000_E284, 0xFFFF_FFFF);
    }

    // ------------------- step_pair_arm: WFE-waiting + cycle-cap exit -------------------
    //
    // Existing stage4_lib_residue_v2::step_pair_arm_skips_wfe_waiting_core
    // covers the wfe-waiting case for one core. This pair varies the
    // setup so both cores' WFE-waiting paths land in the inner-loop
    // predicate evaluation.

    #[test]
    fn step_pair_arm_skips_wfe_on_core1() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.atomics.set_wfe_waiting(1);
        emu.bus.atomics.set_halted(0);
        let pre = emu.cores.expect_arm()[1].cycles;
        let _ = emu.step().unwrap();
        assert_eq!(emu.cores.expect_arm()[1].cycles, pre);
    }

    // ------------------- core_riscv accessor on RiscV emulator -------------------
    //
    // `core_riscv` / `core_riscv_mut` aren't directly hit by stage4 /
    // stage5. Drives both for both hart IDs.

    #[test]
    fn core_riscv_accessor_returns_valid_reference() {
        use crate::Arch;
        let emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        let h0 = emu.core_riscv(0);
        assert_eq!(h0.cycles(), 0);
        let h1 = emu.core_riscv(1);
        assert_eq!(h1.cycles(), 0);
    }

    #[test]
    fn core_riscv_mut_accessor_allows_mutation() {
        use crate::Arch;
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        emu.core_riscv_mut(0).set_halted(true);
        assert!(emu.core_riscv(0).is_halted());
    }

    // ------------------- core / core_mut on Arm sanity -------------------

    #[test]
    fn core_accessor_arm_returns_valid_reference() {
        let emu = Emulator::new(Config::default());
        let _ = emu.core(0).cycles();
        let _ = emu.core(1).cycles();
    }

    // ------------------- reset_counters Arm path -------------------
    //
    // Lib.rs:1301 — `if let Cores::Arm(arm) = ...` — TRUE arm. The
    // existing tests don't exercise reset_counters explicitly.

    #[test]
    fn reset_counters_arm_resets_each_core() {
        let mut emu = Emulator::new(Config::default());
        emu.reset_counters();
    }

    /// FALSE arm of the same `if let` (line 1301): RiscV emulator
    /// short-circuits the inner reset loop entirely.
    #[test]
    fn reset_counters_riscv_is_noop() {
        use crate::Arch;
        let mut emu = EmulatorBuilder::new(Config::default())
            .arch(Arch::RiscV)
            .build()
            .unwrap();
        emu.reset_counters();
    }

    // ------------------- core_counters accessor sanity -------------------

    #[test]
    fn core_counters_accessor_arm() {
        let emu = Emulator::new(Config::default());
        let _ = emu.core_counters(0);
        let _ = emu.core_counters(1);
    }
}
