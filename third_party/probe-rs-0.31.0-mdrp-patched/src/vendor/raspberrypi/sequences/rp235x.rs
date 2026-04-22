//! Implement chip-specific dual-core reset for RP235x

use crate::MemoryMappedRegister;
use crate::architecture::arm::armv6m::{Aircr, Demcr};
use crate::architecture::arm::communication_interface::DapProbe;
use crate::architecture::arm::dp::{Ctrl, DpAddress, DpRegister};
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{
    ArmDebugSequence, ArmDebugSequenceError, alert_sequence, cortex_m_wait_for_reset,
    swd_line_reset,
};
use crate::architecture::arm::{AdiVersion, ApV2Address, ArmError, FullyQualifiedApAddress};
use crate::probe::WireProtocol;
use std::sync::Arc;
use std::time::{Duration, Instant};

const SIO_CPUID_OFFSET: u64 = 0xd000_0000;
const RP_AP: FullyQualifiedApAddress =
    FullyQualifiedApAddress::v2_with_dp(DpAddress::Default, ApV2Address(Some(0x80000)));

/// An address in RAM, used for validating that the core is working.
const RAM_ADDRESS: u64 = 0x2000_0000;

/// Resetting the core can sometimes take multiple attempts. Abandon reset
/// if it takes longer than this duration. During testing of over 5000 reset-
/// program cycles, the longest observed reset cycle was 465 ms after 114
/// reset attempts. It may be that this is attempting to disable XIP on the SPI
/// flash, but it is ultimately unclear what's causing the delay.
const RESET_TIMEOUT: Duration = Duration::from_secs(1);

/// Debug implementation for RP235x
#[derive(Debug)]
pub struct Rp235x {}

impl Rp235x {
    /// Create a debug sequencer for a Raspberry Pi RP235x
    pub fn create() -> Arc<Self> {
        tracing::info!(
            "RP235x debug sequence constructed; declaring ADIv6 for V2-AP addressing \
             (matches OpenOCD `dap create ... -adiv6`, see HLD 2026.04.22 Track A.2c)"
        );
        Arc::new(Rp235x {})
    }

    fn perform_reset(
        &self,
        core: &mut dyn ArmMemoryInterface,
        core_type: crate::CoreType,
        debug_base: Option<u64>,
        should_catch_reset: bool,
    ) -> Result<(), ArmError> {
        let ap = core.fully_qualified_address();
        let arm_interface = core.get_arm_debug_interface()?;

        // Take note of the existing values for CTRL. We will need to restore these after entering
        // rescue mode.
        let existing_core_0 = arm_interface.read_raw_dp_register(ap.dp(), Ctrl::ADDRESS)?;
        tracing::trace!("Core 0 DP_CTRL: {existing_core_0:08x}");

        // Put the SoC in rescue reset as the datasheet.
        //
        // This process consists of setting a flag in the CTRL register of
        // the RP-AP, which causes a flag to be set in POWMAN CHIP_RESET
        // (RESCUE) and the SoC to reset. The BootROM checks for the RESCUE
        // flag, and if set, halts the system. We then attach as usual.
        //
        // See: 'RP2350 Datasheet', sections:
        //   3.5.8: Rescue Reset
        //   3.5.10.1: RP-AP list of registers

        // CTRL register offset within RP-AP.
        const CTRL: u64 = 0;
        const RESCUE_RESTART: u32 = 0x8000_0000;

        // Poke 1 to CTRL.RESCUE_RESTART.
        let ctrl = arm_interface.read_raw_ap_register(&RP_AP, CTRL)?;
        arm_interface.write_raw_ap_register(&RP_AP, CTRL, ctrl | RESCUE_RESTART)?;
        // Poke 0 to CTRL.RESCUE_RESTART.
        let ctrl = arm_interface.read_raw_ap_register(&RP_AP, CTRL)?;
        arm_interface.write_raw_ap_register(&RP_AP, CTRL, ctrl & !RESCUE_RESTART)?;

        let new_core_0 = arm_interface.read_raw_dp_register(ap.dp(), Ctrl::ADDRESS)?;
        tracing::trace!("new core 0 CTRL: {new_core_0:08x}");

        // Start the debug core back up which brings it out of Rescue Mode
        self.debug_core_start(arm_interface, &ap, core_type, debug_base, None)?;

        // Restore the ctrl values
        tracing::trace!("Restoring ctrl values");
        arm_interface.write_raw_dp_register(ap.dp(), Ctrl::ADDRESS, existing_core_0)?;

        // If we were set to catch the reset before, set it up again
        if should_catch_reset {
            tracing::trace!("Re-setting reset catch");
            self.reset_catch_set(core, core_type, debug_base)?
        }

        // Perform a reset to get out of Rescue Mode by poking AIRCR.
        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);
        core.write_word_32(Aircr::get_mmio_address(), aircr.into())?;

        // Wait for the core to finish resetting
        cortex_m_wait_for_reset(core)?;

        // As a final check, make sure we can read from RAM.
        core.read_word_32(RAM_ADDRESS).map(|_| ())
    }
}

impl ArmDebugSequence for Rp235x {
    /// RP235x uses ADIv6 with V2-shaped APs at 32-bit base addresses
    /// (core 0 at 0x2000, core 1 at 0x4000, RP-AP at 0x80000).
    /// Declared up-front as a target property regardless of DPIDR.version,
    /// matching OpenOCD's `dap create ... -adiv6` Tcl model. Without this
    /// override, probe-rs would misroute V2-AP accesses on silicon that
    /// reports DPIDR = 0x10212927 (see HLD 2026.04.22 Track A.2c §2.5).
    fn adi_version(&self) -> AdiVersion {
        AdiVersion::V6
    }

    /// RP235x ADIv6 dormant-wake override.
    ///
    /// The stock `ArmDebugSequence::debug_port_setup` only sends the
    /// dormant-to-SWD wake sequence after the first `debug_port_connect`
    /// attempt *fails*. On RP2354 silicon the initial JTAG-to-SWD
    /// (`0xE79E`) attempt succeeds in reading DPIDR, but returns
    /// `0x10212927` — the DP's power-on/legacy identity, not its
    /// ADIv6/DPv3 identity. The retry-gated dormant fallback therefore
    /// never fires, and subsequent V2-AP reads return 0 because the DP
    /// is routed to the wrong internal register.
    ///
    /// OpenOCD's `dap create ... -adiv6` branch sends the 128-bit JEDEC
    /// selection alert + `0x1A` SWD activation on every attach, which
    /// transitions the DP into its ADIv6 identity (DPIDR `0x4c013477`,
    /// V2 MEM-AP at `0x2000` reads `0x34770008`).
    ///
    /// This override mirrors the OpenOCD behaviour: unconditionally send
    /// SWD→Dormant + alert + activation on every retry. RP235x does not
    /// support JTAG attach in this fork; attempts with the wrong active
    /// protocol return an error immediately.
    ///
    /// See `wrk_docs/2026.04.22 - HLD - Track A.2d Dormant-wake gap.md`
    /// §2 and §4.1 for wire-level rationale.
    fn debug_port_setup(
        &self,
        interface: &mut dyn DapProbe,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        tracing::info!(
            "Rp235x::debug_port_setup: ADIv6 dormant-wake path (matches OpenOCD -adiv6); dp={dp:x?}"
        );

        if interface.active_protocol() != Some(WireProtocol::Swd) {
            return Err(ArmDebugSequenceError::SequenceSpecific(
                "Rp235x requires SWD (JTAG attach not supported by this vendor sequence)".into(),
            )
            .into());
        }

        // Retry count matches the upstream default (`NUM_RETRIES = 5`).
        const NUM_RETRIES: usize = 5;
        let mut result = Ok(());
        for _ in 0..NUM_RETRIES {
            // Ensure current debug interface is in reset state.
            swd_line_reset(interface, 0)?;

            // SWD → Dormant (31-bit `0x33BBBBBA`). Safe from any prior
            // state per the ARM IHI 0074C commentary; matches the
            // upstream dormant branch at sequences.rs §debug_port_setup.
            tracing::debug!("RP235x: SWD → Dormant (0x33BBBBBA, 31 bits)");
            interface.swj_sequence(31, 0x33BBBBBA)?;

            // 128-bit JEDEC selection alert (two 64-bit halves + 8-bit
            // guard prefix). The shared helper was promoted to
            // pub(crate) in Commit 1 for vendor reuse.
            alert_sequence(interface)?;

            // 4 cycles SWDIO low + 8-bit SWD activation code `0x1A`.
            interface.swj_sequence(12, 0x1A0)?;

            // Attempt DPIDR read (this does its own swd_line_reset + read).
            result = self.debug_port_connect(interface, dp);
            if result.is_ok() {
                break;
            }
        }

        result
    }

    fn reset_system(
        &self,
        core: &mut dyn ArmMemoryInterface,
        core_type: crate::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        tracing::trace!("reset_system(interface, {core_type:?}, {debug_base:x?})");
        // Only perform a system reset from core 0
        let core_id = core.read_word_32(SIO_CPUID_OFFSET)?;
        if core_id != 0 {
            tracing::error!("Skipping reset of core {core_id}");
            return Ok(());
        }

        // Since we're resetting the core, the catch_reset flag will get lost.
        // Note whether we should re-set it after entering Rescue Mode.
        let should_catch_reset =
            Demcr(core.read_word_32(Demcr::get_mmio_address())?).vc_corereset();

        // Reset seems to get stuck in a state where RAM is inaccessible. A second reset
        // fixes this about 20% of the time. Perform multiple resets to try and get the
        // board into a state where it's functioning. Note that a full reset is required
        // in this case -- simply resetting the core does not get it into a functioning
        // state.
        let start = Instant::now();
        let mut attempt = 0;
        loop {
            attempt += 1;
            tracing::debug!("Performing reset (attempt {attempt})...");
            let Err(e) = self.perform_reset(core, core_type, debug_base, should_catch_reset) else {
                tracing::info!(
                    "Finished RP235x reset sequence after {attempt} attempts and {} ms",
                    start.elapsed().as_millis()
                );
                return Ok(());
            };

            if start.elapsed() > RESET_TIMEOUT {
                tracing::error!(
                    "Reset failed after {attempt} attempts and {} ms: {e}",
                    start.elapsed().as_millis()
                );
                return Err(e);
            }
        }
    }
}
