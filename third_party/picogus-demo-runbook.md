# PicoGUS end-to-end MIDI demo runbook

Stage 6 of the PicoGUS Integration HLD
(`wrk_docs/2026.04.14 - HLD - PicoGUS Integration.md`). Walks from
zero (nothing built, nothing captured) to `output.wav` — with honest
call-outs where steps land on Arthur externally.

## Prerequisites checklist

1. **Patched DOSBox-X binary** (Arthur, external).
   Apply `third_party/dosbox-x-picogus-tap.patch` to DOSBox-X at the
   commit SHA pinned in `third_party/README.md`, then build. The
   patch only touches `src/hardware/gus.cpp`; no build-system
   changes. On Linux: `./build`. On Windows: MSVC solution, no extra
   steps.
2. **DOS MIDI player executable** (Arthur, external).
   You need a DOS `.exe` inside the DOSBox guest that can play a
   `.mid` through the GUS hardware port range. Candidates:
   - `MIDPLAY.EXE` (Prestige Technology) — simple, widely compatible.
   - `CLM.EXE` — classical MIDI player; supports GUS.
   - `JMPLAY.EXE` (JukeBox) — another solid GUS-capable option.
   - A DOS game that uses GUS for MIDI (e.g. Doom, Duke3D).
   None are shipped in this repo — drop one into
   `d:\language\mdsc55\tests\` or wherever your DOS mount lives.
3. **GUS-configured DOSBox-X `gus.conf`** (Arthur, external).
   A conf file that sets `machine=pcjr`/`svga_s3`, enables
   `[gus] gus=true`, picks an IRQ/DMA/base-port (defaults:
   `gusbase=240`, `gusirq=5`, `gusdma=3`), and autoexecs into your
   DOS mount. Minimal starter (tweak as needed):
   ```
   [sdl]
   autolock=false
   [cpu]
   core=dynamic
   cputype=pentium_slow
   cycles=max
   [mixer]
   rate=44100
   [gus]
   gus=true
   gusrate=44100
   gusbase=240
   gusirq=5
   gusdma=3
   ultradir=C:\ULTRASND
   [autoexec]
   mount c d:\language\mdsc55\tests
   c:
   ```
4. **Target MIDI file** (Arthur-provided, verified).
   `d:\language\mdsc55\tests\doom2_map01_running_from_evil.mid` — 30 KB,
   exists on disk.
5. **PicoGUS firmware** (automated fetch).
   Run `bash scripts/picogus_demo.sh --prepare` once. It downloads
   `picogus-v4.0.0.zip` from the pinned upstream release, verifies
   SHA256, extracts, and converts `picogus.uf2` to raw bin. Details
   in `third_party/picogus/README.md`.
6. **Built harness binary** (automated).
   `cargo build --release -p mdpicoem-harness --bin picogus_diff_rp2040`.

## Step 1 — Build patched DOSBox-X

Follow `third_party/README.md` (the DOSBox-X section). One-liner
summary:

```sh
git clone https://github.com/joncampbell123/dosbox-x.git
cd dosbox-x
git checkout f43ce61d8863439b4c4bedf1344d626b38b2cd75
git apply --3way /path/to/mdrp2354/third_party/dosbox-x-picogus-tap.patch
# Linux:
./build
# Windows: open the MSVC solution, build Debug or Release.
```

Result: a `dosbox-x.exe` (Windows) or `dosbox-x` (Linux) binary that
will log GUS I/O when the env var `PICOGUS_TAP_FILE` is set.

## Step 2 — Stage the PicoGUS firmware

```sh
bash scripts/picogus_demo.sh --prepare
```

After this completes you should see in `third_party/picogus/`:

- `picogus-v4.0.0.zip`     (fetched)
- `picogus.uf2`            (extracted)
- `picogus-v4.0.0.bin`     (derived, ~906 KB)
- `pg-ne2k.uf2`, `pgusinit.exe`, `UPSTREAM_README.md` (unused but
  kept for reference)

## Step 3 — Capture the ISA trace

Launch your patched DOSBox-X with the tap env var set:

```sh
# Linux / bash:
export PICOGUS_TAP_FILE=/tmp/doom2_e1m1.trace
./dosbox-x -conf gus.conf \
    -c "mount c d:\language\mdsc55\tests" \
    -c "c:" \
    -c "midplay doom2_map01_running_from_evil.mid"
# exit DOSBox-X cleanly (don't kill -9 — buffered lines would be lost)
```

On Windows `cmd.exe`:

```bat
set PICOGUS_TAP_FILE=C:\temp\doom2_e1m1.trace
dosbox-x.exe -conf gus.conf ^
    -c "mount c d:\language\mdsc55\tests" ^
    -c "c:" ^
    -c "midplay doom2_map01_running_from_evil.mid"
```

Substitute whichever MIDI player `.exe` you have. The capture ends
when you exit DOSBox-X; the tap writes lines in append mode with
fflush after each write so partial traces are usable if DOSBox-X
crashes.

**Sanity-check the trace.** Should be >1000 lines for a 10-second
MIDI. First line is `# picogus-tap v1`, second line is
`ns,port,value,kind`. Timestamps should be monotonically
non-decreasing and span roughly the duration of the MIDI playback.

## Step 4 — Run the harness end-to-end

```sh
cargo run -p mdpicoem-harness --release --bin picogus_diff_rp2040 -- \
    --flash third_party/picogus/picogus-v4.0.0.bin \
    --trace /tmp/doom2_e1m1.trace \
    --out   crates/mdpicoem-harness/oracles/picogus_doom2_e1m1.wav \
    --post-roll 1.0
```

`--post-roll 1.0` steps the emulator for an extra simulated second
after the last trace event, giving the firmware's I2S DMA chain time
to flush the last audio buffer.

Expected timings: roughly 15–30 seconds of wall-clock per 10 seconds
of simulated audio on recent desktop hardware (HLD Risks section).
If your MIDI is 90 seconds long, budget 3–5 minutes.

## Step 5 — Capture DOSBox-X's own reference audio

DOSBox-X has a built-in WAV recorder:

- Inside DOSBox-X, press `Ctrl+F6` to start / stop WAV capture.
- By default the WAV drops into `capture/` in DOSBox-X's working
  directory (or wherever the `captures=` setting in the conf
  points).

Run the MIDI again (you can reuse the same trace-capture invocation
from Step 3, hit Ctrl+F6 once playback starts, hit it again once
playback ends). You now have `capture/dosbox-x_<timestamp>.wav` as
the ground-truth reference.

## Step 6 — Compare

The HLD's acceptance is **subjective ear-test match on a 10-second
opening segment**.

1. **Listen in parallel.** Open `output.wav` and
   `dosbox-x_<timestamp>.wav` side-by-side. `aplay` / `afplay` /
   Windows Media Player all work.
2. **Spectrogram compare.** Audacity's built-in spectrogram view is
   adequate; sox can do batch STFT. You're looking for the same
   dominant frequency bands at roughly the same times. Minor phase
   and mixing differences are fine — the two pipelines share the
   GUS model pedigree but aren't sample-accurate clones.
3. **Recognisability.** Does the melody sound like the same piece of
   music played on the same instruments? If yes, Stage 6 passes.

## Known failure modes and what they mean

- **`output.wav` is 44 bytes and `Frames: 0` in the summary.**
  Firmware didn't drive the I2S pins. Almost certainly because the
  firmware failed to boot past boot2. See
  `third_party/picogus/README.md` ("What `picogus_diff_rp2040`
  actually boots") — we don't yet ship the real RP2040 bootrom; the
  synthetic stub in `roms/rp2040/bootrom.bin` only handles vector-
  table mapping for flat SRAM-linked images like `blinky.bin`. Fixing
  this is Phase 8 work, tracked in `tech_debt.md`.
- **`output.wav` is populated but sounds nothing like the MIDI.**
  Either the trace is wrong (DOSBox-X wasn't actually playing GUS
  — check `gus.conf`), the ISA pin injection got the address/data
  muxing wrong (see the write16-splitting note in `tech_debt.md`), or
  the firmware branched on a status read we didn't replay. HLD Risks
  section covers this — try another MIDI first before diving.
- **The harness reports tens of thousands of stall events.**
  Same root cause as "44 bytes": firmware never got running, so the
  cores halt into a fault loop, the cycle counter stops advancing,
  and every subsequent trace event fires at the same stuck cycle.
- **`sha256 mismatch` when fetching the firmware.**
  Upstream may have re-published the asset. Check the release page
  and update `third_party/picogus/VERSION` with the new checksum.

## What Arthur needs to figure out externally

- **Which DOS MIDI player.** We don't pin one; any GUS-aware player
  works. I'd start with `CLM.EXE` because it's small and widely
  tested against DOSBox GUS.
- **Exact `gus.conf` tuning.** The starter above works against
  stock DOSBox-X's GUS. If the firmware decodes a different base
  port / IRQ / DMA combo, adjust to match the firmware's defaults
  (see `picogus/sw/src/isa_io.pio` + the `pgusinit` output for what
  the firmware expects at `0x240`/`0x340`).
- **The boot2 bootrom gap.** This is the single biggest external
  unknown for Stage 6 acceptance. Options Arthur can pursue:
  1. Vendor the real Raspberry Pi RP2040 bootrom (open, permissive
     licence) into `roms/rp2040/` and teach `picogus_diff_rp2040`
     to load it alongside `--flash`.
  2. Write a minimal Rust-side boot2 shim that does the
     flash-to-SRAM copy the SDK bootrom would do (requires reading
     the linker script to find `__StackTop` and
     `__vector_table_in_ram`).
  3. Pre-process the firmware: extract the post-boot2 payload, place
     it straight into SRAM via `--image` (the mdrp2040 SRAM-loading
     path that already works for blinky). Requires an LLDB-style
     parse of the firmware's vector table + BSS / data sections.
  Option 1 is the cleanest. The other two are only useful if the
  bootrom licence turns out to be a blocker, which there's no
  evidence of.

## Quick reference — files you'll end up creating

| File | Where | What |
|---|---|---|
| `dosbox-x-patched` | your dev tree | Patched DOSBox-X binary |
| `gus.conf` | your dev tree | DOSBox-X conf with GUS enabled |
| `/tmp/doom2_e1m1.trace` | whatever `$PICOGUS_TAP_FILE` points to | ISA port-write trace |
| `third_party/picogus/picogus-v4.0.0.bin` | this repo | Raw flash image |
| `crates/mdpicoem-harness/oracles/picogus_doom2_e1m1.wav` | this repo | Harness audio output |
| `capture/dosbox-x_*.wav` | your dev tree | DOSBox-X reference audio |
