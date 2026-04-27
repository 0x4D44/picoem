# picogus-v4.0.0 — `test_psram` fast-path build variant

This file documents the local modification that produces
`picogus-v4.0.0-patched.bin` from the upstream
`picogus-v4.0.0.bin`. It is provided as the corresponding source for
the modified binary, as required by GPL-2.0-or-later §3.

## Summary

A four-byte change to the `test_psram` self-diagnostic that causes it
to return success immediately, rather than walking the entire PSRAM
address space.

## Why we keep this variant

The stock `test_psram` is correct: it walks the full PSRAM address
space writing-then-reading patterns to validate the SPI link. On real
PicoGUS hardware running at 370 MHz this completes in milliseconds. On
the `mdrp2040` emulator at typical sim rates it takes approximately
one hour, blocking firmware boot before any audio output is produced.

`test_psram` is a self-diagnostic only — its result is not used by any
downstream init or audio code path. Short-circuiting it has no
functional effect on emulated audio output beyond removing the
boot-time wait.

This variant is consumed only by the `picogus_diff_rp2040` and
`picogus_probe_pc` harnesses. Anyone running the firmware on real
PicoGUS hardware should use the unmodified `picogus-v4.0.0.bin`.

## The four-byte change

Two consecutive 16-bit Thumb instructions at flash offset `0x0b198`
are rewritten. Three of the four bytes differ; the byte at offset
`0x0b198` is identical in both files.

### Byte table

| Flash offset | Stock | Variant | Note |
|---|---|---|---|
| `0x0b198` | `0x00` | `0x00` | identical |
| `0x0b199` | `0xd0` | `0x20` | high byte of first halfword |
| `0x0b19a` | `0xb7` | `0x70` | low byte of second halfword |
| `0x0b19b` | `0xe3` | `0x47` | high byte of second halfword |

### Instruction-level

Stock — branches into the regular code path that performs the full
PSRAM sweep:

```
0x0b198:  d0 00            BEQ   0x0b19c
0x0b19a:  e3 b7            B     0x0b90c
```

Variant — returns `r0 = 0` to the caller without entering either
branch target:

```
0x0b198:  20 00            MOVS  r0, #0
0x0b19a:  47 70            BX    lr
```

Note: after the firmware's `.data` copy these instructions also live
in SRAM at address `0x20012FA4`. Earlier in the project the same
change was applied as a one-time SRAM write to that address from the
harness; the variant binary makes the change persistent so the harness
no longer needs the runtime workaround.

## How to re-create the variant from the upstream binary

The change is a four-byte rewrite at flash offset `0x0b198` of
`picogus-v4.0.0.bin`. Any hex editor (e.g. HxD on Windows, `bvi` on
Unix) can apply it: navigate to offset `0x0b198`, replace the four
bytes there with `00 20 70 47` in that order, and save.

The resulting file should be 906752 bytes — identical in length to
the input — and should differ from it at exactly bytes `0x0b199`,
`0x0b19a`, `0x0b19b`.

## Provenance

The change was first developed in
`wrk_journals/2026.04.16 - JRN - PicoGUS End-to-End Bring-up.md`
(Phase E) as a runtime SRAM write at address `0x20012FA4`. The
permanent on-disk variant was created at the same time. The four-byte
delta was independently re-confirmed in
`wrk_journals/2026.04.24 - JRN - Single-voice dump and FM trace.md`
(Session 8) by binary-diffing the variant and stock files.
