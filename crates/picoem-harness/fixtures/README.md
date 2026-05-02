# picoem-harness fixtures

Test fixtures consumed by the harness binaries. None of these files
contain redistributed third-party game data, ROM data, or source code.
They are functional captures and locally-built firmware images.

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

## Notes for downstream users

These fixtures are consumed by harness binaries with hard-coded paths
relative to this directory; do not rename or move them without
updating the corresponding `--trace` / `--firmware` defaults in
`crates/picoem-harness/src/bin/`.

The Monkey Island traces are large (the GUS theme trace is ~30 MB
uncompressed). They are committed directly rather than fetched at run
time so the harness oracles work out of a fresh clone with no extra
download steps.
