// Hardware differential test runner — mdrp2040 (Cortex-M0+) vs real
// RP2040 silicon via SWD.
//
// **Stub.** We don't currently have a Pico H debug probe wired to an
// RP2040 board, and the lab rig runs against an RP2354 instead. When
// hardware is available, mirror `probe_diff_rp2350` with the following
// adjustments:
//
//   * Session target: "RP2040" (probe-rs chip pack) instead of "RP235x".
//   * Emulator: `mdrp2040::Emulator` / `CortexM0Plus` / `Bus`.
//   * Register IDs: same as M33 (probe-rs uses the ARM AADR numbering).
//   * DWT_CYCCNT: M0+ does not implement DWT on the RP2040 reference
//     design — drop the `--cycles` mode, or validate cycle counts via
//     the in-emulator cycle counter only.
//   * TestCase filter: reuse `is_m0plus_safe` from `qemu_diff_m0plus`
//     once that helper is extracted to the harness lib.
//
// See `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md`
// Phase 6 section and `tech_debt.md` (RP2040 probe_diff item).
//
// Running this binary prints a rationale and exits with status 2 so
// CI / smoke scripts fail loudly if they accidentally invoke it.

fn main() {
    eprintln!("probe_diff_rp2040: NOT IMPLEMENTED");
    eprintln!();
    eprintln!(
        "This binary is a placeholder. The RP2040 probe-rs oracle has not\n\
         been implemented because the lab rig only carries an RP2354 board\n\
         at present. See wrk_docs/2026.04.14 - HLD - mdpicoem Workspace\n\
         Restructure.md Phase 6 for the implementation plan when hardware\n\
         is available.\n\
         \n\
         For the software oracle, use:\n  \
         cargo run -p mdpicoem-harness --release --bin qemu_diff_m0plus\n"
    );
    std::process::exit(2);
}
