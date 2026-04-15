# third_party/

Vendored patches and external artefacts consumed by the mdpicoem
workspace. Nothing in here is compiled by `cargo build`.

## `dosbox-x-picogus-tap.patch`

### Purpose

Adds a small, env-gated CSV logger to DOSBox-X's GUS (Gravis
Ultrasound) I/O handlers. Every port access to `0x2xx` / `0x3xx`
decoded by `read_gus` / `write_gus` is written to a flat text trace
file. The trace is consumed by the `picogus_diff_rp2040` harness
binary (Stage 4 of the PicoGUS Integration HLD) to drive PicoGUS
firmware inside `mdrp2040`.

The tap is **off by default**. No runtime cost when unused. When
enabled it writes a line per I/O and `fflush()`es immediately, so a
partial trace from a crashed DOSBox-X session is still usable.

### DOSBox-X source version

Written against `joncampbell123/dosbox-x` on `master`, commit
`f43ce61d8863439b4c4bedf1344d626b38b2cd75` (2026-04-14). Any
reasonably recent DOSBox-X master should apply with `-F5` fuzz —
the patch hooks file-scope static state and two well-anchored
function entry points (`read_gus`, `write_gus` in
`src/hardware/gus.cpp`) which rarely change.

### Upstream drift

If `read_gus` / `write_gus` no longer exist in upstream (renamed,
deleted, or replaced by a different dispatch path), regenerate the
patch against the new function names and bump the commit SHA at the
top of this README to match the version you rebased against.

### Clone the reference version

```sh
git clone https://github.com/joncampbell123/dosbox-x.git
cd dosbox-x
git checkout f43ce61d8863439b4c4bedf1344d626b38b2cd75
```

### Apply the patch

From the DOSBox-X working tree:

```sh
# Preferred: git's 3-way apply tolerates context drift.
git apply --3way /path/to/mdrp2354/third_party/dosbox-x-picogus-tap.patch

# Fallback if 3-way refuses: GNU patch with fuzzy matching.
patch -p1 -F5 --no-backup-if-mismatch \
    < /path/to/mdrp2354/third_party/dosbox-x-picogus-tap.patch
```

Only `src/hardware/gus.cpp` is touched. If a hunk fails to locate,
open the `.rej` / conflict markers and re-anchor manually — the
patch is small enough (four hunks, ~103 lines added, 1 line
removed) to transplant by hand in a few minutes.

### Build

Follow DOSBox-X's own build instructions — see its top-level
`README.md` / `BUILD.md`. Nothing in this patch changes the build
system; only `src/hardware/gus.cpp` is modified.

On Linux this usually means:

```sh
./build  # or: ./autogen.sh && ./configure && make -j
```

On Windows, DOSBox-X's MSVC solution builds `src/hardware/gus.cpp`
like any other translation unit — no extra steps.

### Usage

Point the env var at a writable file, then launch DOSBox-X with
GUS enabled and a MIDI player configured to use GUS:

```sh
export PICOGUS_TAP_FILE=/tmp/doom_e1m1.trace
./dosbox-x -conf gus.conf               # boots DOS, loads MIDI player
# ... play a MIDI inside DOS, e.g. via MPXPLAY / JMPLAY in GUS mode ...
# exit DOSBox-X cleanly (don't kill -9)
```

After exit, `/tmp/doom_e1m1.trace` contains the recording. On
Windows `cmd.exe`:

```bat
set PICOGUS_TAP_FILE=C:\temp\doom_e1m1.trace
dosbox-x.exe -conf gus.conf
```

A log line `PicoGUS tap: logging GUS I/O to "<path>"` appears in
DOSBox-X's normal log output when the tap is armed.

### Trace format

Plain-text CSV, UTF-8 safe, LF line endings. Example:

```
# picogus-tap v1
ns,port,value,kind
0,0x247,0x00,write8
50000,0x24b,0x4c,write8
125000,0x247,0x01,write8
675000,0x242,0x0400,write16
1500000,0x246,0x20,read8
```

Columns:

| Column  | Format                 | Meaning                                                             |
|---------|------------------------|---------------------------------------------------------------------|
| `ns`    | unsigned decimal       | Nanoseconds since DOSBox-X start (monotonic; derived from `PIC_FullIndex`). |
| `port`  | `0x%03x`               | Fully decoded 12-bit ISA port (GUS base `0x240`..`0x24F`, plus aliases). |
| `value` | `0x%02x` or `0x%04x`   | Payload value. Width is `0x%02x` for `write8`/`read8`, `0x%04x` for `write16`/`read16`. Reads log `0x00` (replayer ignores reads). |
| `kind`  | string                 | One of `write8`, `write16`, `read8`, `read16`.                      |

Events appear in issue order, timestamps are non-decreasing. The
first line (`# picogus-tap v1`) is a magic/version comment; tooling
must check it.

A canonical hand-crafted fixture of ~20 events lives at
`crates/mdpicoem-harness/fixtures/sample_gus.trace` — useful for
unit-testing the Stage 4 replayer without a DOSBox-X build.

### Known limitations

- **Single writer.** One file per DOSBox-X process; path comes from
  `PICOGUS_TAP_FILE`. No rotation, no concurrent writers.
- **Blocking I/O.** Every logged access does a `fprintf` + `fflush`
  on DOSBox-X's CPU thread. Fine for interactive MIDI playback,
  measurable if you're running CPU-bound benchmarks in parallel.
- **OFF by default.** If the env var is unset, no file is opened,
  the GUS code path is unchanged. There's no in-DOSBox config
  setting — this is a debug-only hook.
- **Uses `PICOGUS_TAP_FILE` env var to enable.** This is deliberate:
  DOSBox-X lacks a cleaner config mechanism, and the tap is a
  debug-only hook in third-party code. Our first-party Rust code
  (Stages 1-2, 4-6) remains env-var-free per HLD.
- **Reads log `value=0`.** The tap fires at `read_gus` entry, before
  dispatch returns the actual byte. The replayer in Stage 4
  discards reads anyway (see HLD Risks section on read-driven
  control flow).
- **Timestamp resolution.** `PIC_FullIndex()` is millisecond-scale
  with sub-millisecond fractional cycles. Truncating to integer ns
  is lossless for that resolution; don't expect true ns-granular
  ordering between two I/Os inside the same 386 instruction.

### Specification

See `wrk_docs/2026.04.14 - HLD - PicoGUS Integration.md`, Stage 3
("DOSBox-X tap + trace format") for the full design rationale,
alternatives considered, and integration into Stages 4-6.
