# picoem-harness fixtures

Test fixtures consumed by the harness binaries. The binary fixtures here
are either functional captures (the `*.trace` files), locally-built RP2350
firmware images (the `onerom-*.bin` files), or open-source firmware that we
redistribute under its license (currently SeaBIOS, see below).

## Trace files (`*.trace`)

CSV-format port-write traces in our `picogus-tap v1` schema:

```
# picogus-tap v1
ns,port,value,kind
<timestamp_ns>,<port_hex>,<value_hex>,write8|read8|write16|read16
```

A trace records the sequence of x86 ISA-bus port accesses (writes and
reads) that a host program issued, with nanosecond-resolution
timestamps. Traces contain only the externally-observable bus traffic
— no copyrighted game code, audio samples, or ROM contents are
captured or redistributed.

| File | Source | Duration | Use |
|---|---|---|---|
| `sample_gus.trace` | hand-authored test case | ~75 µs | Smoke-test for `picogus_diff_rp2040`'s GUS path. |
| `monkey_island_theme.trace` | captured under our patched DOSBox-X | ~30 s of game audio | Replay-to-WAV through `picogus_diff_rp2040`'s GUS engine. ~524k events. |
| `monkey_island_adlib.trace` | captured under our patched DOSBox-X | ~3 s of game audio | Replay through the OPL3/Adlib path of `picogus_diff_rp2040`. ~29k events. |

The Monkey Island traces were captured by running an unmodified
retail copy of *The Secret of Monkey Island* under DOSBox-X with our
locally-applied port-tap patch (see
`third_party/dosbox-x-picogus-tap.patch`). The patch logs every read
and write to the GUS / OPL3 / Adlib port ranges to a CSV file; the
result is a stream of `(timestamp, port, value, kind)` tuples that
documents what the game's audio driver issued at the bus interface.

The capture method is analogous to recording the MIDI commands a
musical performance generates rather than its acoustic output — the
file records bus-level activity, not the game's content. Game data,
ROM data, audio samples, and game source code are all upstream of
this interface and are not present in any trace file.

## OneROM firmware images (`onerom-*.bin`)

`onerom-fire-24-a-rp2350-*.bin` are OneROM firmware images built
locally for the harness's `onerom_*` oracles (CPU, PIO, full-system,
serving, stress, and speed-grade variants). They are RP2350 firmware
images, not redistributed third-party content; rebuilds of these
binaries are produced from the OneROM source tree as the project
evolves. See the OneROM HLD documents under `wrk_docs/` for build
provenance.

`onerom-fire-32-a-rp2350-seabios.bin` is the fire-32-a RP2350 PIO-serve
SeaBIOS fixture used by `seabios32_fixture_byte_correct`. It was generated
locally from the OneROM `fire-32-a` RP2350 pipeline with the JSON config
`sources/seabios-32-27c020.json` and the source BIOS image
`sources/seabios-256k.bin`; see
`wrk_journals/2026.05.04 - JRN - Fire-32-a SeaBIOS Firmware Build.md` and
`wrk_docs/2026.05.04 - HLD - OneROM Serving Oracle Fixture Generalization.md`
for the reproduction recipe and pin-map notes.

| File | Role | SHA-256 |
|---|---|---|
| `onerom-fire-32-a-rp2350-seabios.bin` | Generated fire-32-a PIO-serve 27C020 fixture | `3fb7cb6f85ad371a483a4bbaa6597f29a36c58de6fdac928bf4843789e266c00` |
| `sources/seabios-32-27c020.json` | OneROM source config used to generate the fixture | `7c00bc8b559024779e6f18140cadfaa692449fd7f8c58642ef2151eefd3e3ccf` |
| `sources/seabios-256k.bin` | Source SeaBIOS image referenced by the JSON config | `ae6f6aa973aaccc143f57aa960fb035fd9de4daee4ad0cd713322f8c259e7650` |

## SeaBIOS image (`sources/seabios-256k.bin`)

`sources/seabios-256k.bin` is a 256 KiB SeaBIOS x86 BIOS binary used by
`build_seabios_fixture` to author `onerom-fire-24-a-rp2350-seabios-cpu.bin`
and by the fire-32-a 27C020 config above.
SeaBIOS is open-source firmware (LGPLv3) maintained at
https://github.com/coreboot/seabios. The byte-identical copy lives in
mddosem at `assets/roms/bios-256k.bin`; see the journal
`wrk_journals/2026.05.03 - JRN - SDRR SeaBIOS fixture.md` for SHA-256 +
provenance.

The derived fixture `onerom-fire-24-a-rp2350-seabios-cpu.bin` embeds the
SeaBIOS bytes inside SDRR firmware envelope; the fixture inherits SeaBIOS's
LGPLv3.

## Notes for downstream users

These fixtures are consumed by harness binaries with hard-coded paths
relative to this directory; do not rename or move them without
updating the corresponding `--trace` / `--firmware` defaults in
`crates/picoem-harness/src/bin/`.

The Monkey Island traces are large (the GUS theme trace is ~30 MB
uncompressed). They are committed directly rather than fetched at run
time so the harness oracles work out of a fresh clone with no extra
download steps.
