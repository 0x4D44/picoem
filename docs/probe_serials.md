# Probe serial → DUT mapping

This document records the hard-wired mapping between the Pico debug
probes attached to the development host and the silicon they connect
to. The harness binaries that talk to real silicon take a
`--probe <VID:PID:SERIAL>` argument to disambiguate when more than one
probe is attached; the table below is the canonical answer to "which
serial goes where" on this rig.

## Mapping

| Probe serial | `--probe` argument | DUT |
|---|---|---|
| `E46410955F614129` | `2e8a:000c:E46410955F614129` | RP2354 (Pico 2) |
| `E46410955F3C5C27` | `2e8a:000c:E46410955F3C5C27` | RP2040 (Pico V1) |

`2e8a:000c` is the USB VID:PID for the Raspberry Pi debug probe
(`2e8a` = Raspberry Pi Foundation, `000c` = debug probe). The serial
suffix is the per-device unique ID burnt into the probe's RP2040.

`probe-rs list` shows the serials of all probes currently attached;
match against the table above to identify which probe is which.

## Why explicit `--probe` is required on this host

`probe-rs auto_attach` picks the first enumerated probe regardless of
target type. On a host with both an RP2354 probe and an RP2040 probe
attached, that succeeds approximately half the time and fails the
other half — the wrong probe attaches to the wrong target and the
session aborts. Passing the full `VID:PID:SERIAL` triplet makes the
selection deterministic.

## Affected harness binaries

All silicon-touching harness binaries accept `--probe`:

- `probe_diff_rp2350`
- `probe_diff_rp2040`
- `probe_verify_rp2350`
- `bank_conflict_test_rp2350`
- `silicon_cycle_oracle_rp2350`
- `silicon_periph_diff_rp2350`
- `silicon_dualcore_diff_rp2350`
- `silicon_isr_diff_rp2350`
- `silicon_periph_diff_rp2040`
- `silicon_isr_diff_rp2040`
- `test_silicon` (orchestrator)
- The `picogus_probe_pc` live-silicon variant
- The OneROM rig oracles (`onerom_*`)

## If the probes are reassigned

If a probe is moved to a different DUT or replaced, update this file
in place. The mapping is referenced from `CLAUDE.md` and is the
single source of truth for the development rig.
