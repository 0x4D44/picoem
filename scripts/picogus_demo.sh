#!/usr/bin/env bash
# picogus_demo.sh — end-to-end PicoGUS demo driver.
#
# Stages of the PicoGUS Integration HLD
# (`wrk_docs/2026.04.14 - HLD - PicoGUS Integration.md`).
#
# Two modes:
#   --prepare        Stage the PicoGUS firmware under third_party/picogus/
#                    (download, sha256-check, UF2 → bin convert). Safe to
#                    re-run; idempotent.
#   (default)        Run the end-to-end demo pipeline: check prereqs,
#                    invoke picogus_diff_rp2040, emit output.wav. Assumes
#                    the ISA trace already exists (produced externally by
#                    a patched DOSBox-X — see runbook).
#
# Exit codes:
#   0 — success
#   1 — user-visible error (missing prereq, bad args)
#   2 — fatal internal error (curl / sha256 / harness crashed)

set -euo pipefail

# ----------------------------------------------------------------------------
# Paths — resolved relative to the workspace root, not the caller's cwd.
# ----------------------------------------------------------------------------

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

PICOGUS_DIR="$REPO_ROOT/third_party/picogus"
VERSION_FILE="$PICOGUS_DIR/VERSION"
RUNBOOK="$REPO_ROOT/third_party/picogus-demo-runbook.md"

# Pinned upstream artefacts (also recorded in $VERSION_FILE).
RELEASE_TAG="v4.0.0"
RELEASE_ZIP="picogus-${RELEASE_TAG}.zip"
RELEASE_URL="https://github.com/polpo/picogus/releases/download/${RELEASE_TAG}/${RELEASE_ZIP}"
EXPECTED_SHA256="27f34281b9a620ae12d9e704d810d198a12e3d655172cd0d8144b4382bb5b38a"
FIRMWARE_UF2="picogus.uf2"
FIRMWARE_BIN="picogus-${RELEASE_TAG}.bin"

# Demo defaults (overridden by env vars of the same name if set).
TRACE_FILE="${TRACE_FILE:-}"
MIDI_FILE="${MIDI_FILE:-d:/language/mdsc55/tests/doom2_map01_running_from_evil.mid}"
OUT_WAV="${OUT_WAV:-$REPO_ROOT/crates/mdpicoem-harness/oracles/picogus_doom2_e1m1.wav}"
POST_ROLL="${POST_ROLL:-1.0}"

# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

log()   { printf '[picogus_demo] %s\n' "$*" >&2; }
die()   { printf '[picogus_demo] ERROR: %s\n' "$*" >&2; exit 1; }
fatal() { printf '[picogus_demo] FATAL: %s\n' "$*" >&2; exit 2; }

usage() {
    cat <<EOF
Usage: $0 [--prepare] [--trace <path>] [--out <path>] [--post-roll <secs>]

Modes:
  --prepare            Stage the PicoGUS firmware (download + verify +
                       UF2→bin convert). Run once before the demo.
  (default)            Run the demo pipeline end-to-end.

Options (demo mode):
  --trace <path>       ISA trace produced by patched DOSBox-X.
                       Required in demo mode.
  --out <path>         Output WAV path.
                       Default: crates/mdpicoem-harness/oracles/picogus_doom2_e1m1.wav
  --post-roll <secs>   Extra sim-time after last trace event for DMA drain.
                       Default: 1.0

Full walkthrough: $RUNBOOK
EOF
}

sha256_file() {
    # Portable sha256 helper — Linux sha256sum, macOS shasum -a 256, Windows
    # Git Bash ships sha256sum. Return lowercase hex only.
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | awk '{print $1}'
    elif command -v shasum >/dev/null 2>&1; then
        shasum -a 256 "$1" | awk '{print $1}'
    else
        fatal "no sha256 tool found (tried sha256sum / shasum)"
    fi
}

# ----------------------------------------------------------------------------
# Stage: prepare firmware
# ----------------------------------------------------------------------------

prepare_firmware() {
    log "preparing PicoGUS firmware ${RELEASE_TAG} in $PICOGUS_DIR"
    mkdir -p "$PICOGUS_DIR"

    local zip_path="$PICOGUS_DIR/$RELEASE_ZIP"
    local uf2_path="$PICOGUS_DIR/$FIRMWARE_UF2"
    local bin_path="$PICOGUS_DIR/$FIRMWARE_BIN"

    # Step 1: fetch zip if missing, verify checksum.
    if [[ ! -f "$zip_path" ]]; then
        log "downloading $RELEASE_URL"
        command -v curl >/dev/null 2>&1 || fatal "curl not found"
        if ! curl -sSL --fail --max-time 120 -o "$zip_path" "$RELEASE_URL"; then
            fatal "download failed — check connectivity / upstream URL"
        fi
    else
        log "using cached $zip_path"
    fi

    local actual
    actual="$(sha256_file "$zip_path")"
    if [[ "$actual" != "$EXPECTED_SHA256" ]]; then
        fatal "sha256 mismatch on $zip_path
            expected: $EXPECTED_SHA256
            actual:   $actual
            (delete the file and re-run to force redownload)"
    fi
    log "sha256 OK"

    # Step 2: extract if uf2 missing.
    if [[ ! -f "$uf2_path" ]]; then
        command -v unzip >/dev/null 2>&1 || fatal "unzip not found"
        log "extracting zip"
        # -o overwrite, -q quiet. We rename the upstream README to
        # UPSTREAM_README.md after extract to avoid colliding with ours.
        ( cd "$PICOGUS_DIR" && unzip -oq "$RELEASE_ZIP" )
        if [[ -f "$PICOGUS_DIR/README.md.upstream" ]]; then
            : # already renamed on a previous run
        elif [[ -f "$PICOGUS_DIR/README.md" && ! -f "$PICOGUS_DIR/UPSTREAM_README.md" ]]; then
            # Guard: don't overwrite our own README.md.
            # After `unzip -o` the upstream README.md clobbered ours;
            # detect that by looking for our signature string and, if
            # missing, assume we just overwrote with upstream.
            if ! grep -q "PicoGUS firmware (vendored)" "$PICOGUS_DIR/README.md"; then
                mv "$PICOGUS_DIR/README.md" "$PICOGUS_DIR/UPSTREAM_README.md"
                log "renamed upstream README.md -> UPSTREAM_README.md"
                log "re-run 'git checkout -- third_party/picogus/README.md' if our README was clobbered"
            fi
        fi
    else
        log "using cached $uf2_path"
    fi

    # Step 3: UF2 → bin convert (inline Python fallback).
    if [[ ! -f "$bin_path" || "$uf2_path" -nt "$bin_path" ]]; then
        command -v python3 >/dev/null 2>&1 || command -v python >/dev/null 2>&1 \
            || fatal "python3 / python not found (needed for UF2→bin)"
        local py
        if command -v python3 >/dev/null 2>&1; then py=python3; else py=python; fi
        log "converting $FIRMWARE_UF2 -> $FIRMWARE_BIN"
        "$py" - "$uf2_path" "$bin_path" <<'PY'
import struct, sys
src, dst = sys.argv[1], sys.argv[2]
with open(src, 'rb') as f:
    data = f.read()
out = bytearray()
base = None
for i in range(len(data) // 512):
    b = data[i*512:(i+1)*512]
    if struct.unpack_from('<I', b, 0)[0] != 0x0A324655 \
       or struct.unpack_from('<I', b, 4)[0] != 0x9E5D5157:
        sys.exit(f'bad UF2 magic at block {i}')
    addr = struct.unpack_from('<I', b, 12)[0]
    psz  = struct.unpack_from('<I', b, 16)[0]
    if base is None:
        base = addr
    rel = addr - base
    payload = b[32:32+psz]
    if len(out) < rel + psz:
        out.extend(b'\xff' * (rel + psz - len(out)))
    out[rel:rel+psz] = payload
with open(dst, 'wb') as f:
    f.write(out)
print(f'wrote {dst}: {len(out)} bytes, base=0x{base:08x}')
PY
    else
        log "using cached $bin_path"
    fi

    log "firmware ready: $bin_path ($(stat -c %s "$bin_path" 2>/dev/null || stat -f %z "$bin_path") bytes)"
    log "done. run '$0 --trace <path>' to replay a trace."
}

# ----------------------------------------------------------------------------
# Stage: run demo
# ----------------------------------------------------------------------------

run_demo() {
    local missing=0

    # Prereq 1: firmware staged.
    local bin_path="$PICOGUS_DIR/$FIRMWARE_BIN"
    if [[ ! -f "$bin_path" ]]; then
        log "missing firmware: $bin_path"
        log "  run: $0 --prepare"
        missing=1
    fi

    # Prereq 2: MIDI file exists (informational — the trace capture
    # happens externally in DOSBox-X; we just sanity-check the file
    # Arthur is meant to be playing).
    if [[ ! -f "$MIDI_FILE" ]]; then
        log "MIDI file not found at expected path: $MIDI_FILE"
        log "  (override with MIDI_FILE=<path> env var if relocated)"
        missing=1
    fi

    # Prereq 3: trace file.
    if [[ -z "$TRACE_FILE" ]]; then
        log "--trace <path> is required in demo mode"
        log "  capture one via patched DOSBox-X — see:"
        log "  $RUNBOOK"
        missing=1
    elif [[ ! -f "$TRACE_FILE" ]]; then
        log "trace file not found: $TRACE_FILE"
        missing=1
    fi

    if (( missing )); then
        log ""
        log "not all prerequisites are in place. see $RUNBOOK."
        exit 1
    fi

    # Ensure the output directory exists (the harness creates parent dirs
    # too, but being explicit helps when $OUT_WAV is a relative path).
    mkdir -p "$(dirname -- "$OUT_WAV")"

    log "running harness..."
    log "  flash:     $bin_path"
    log "  trace:     $TRACE_FILE"
    log "  out wav:   $OUT_WAV"
    log "  post-roll: ${POST_ROLL}s"
    log ""

    ( cd "$REPO_ROOT" && \
      cargo run -p mdpicoem-harness --release --bin picogus_diff_rp2040 -- \
        --flash "$bin_path" \
        --trace "$TRACE_FILE" \
        --out   "$OUT_WAV" \
        --post-roll "$POST_ROLL" )
    local rc=$?

    if (( rc != 0 )); then
        fatal "picogus_diff_rp2040 exited $rc"
    fi

    log ""
    log "done. output WAV: $OUT_WAV"
    log "compare vs DOSBox-X's reference capture (Ctrl+F6 inside DOSBox-X)."
    log "expected: recognisably the same music, minor phase / mixing deltas ok."
}

# ----------------------------------------------------------------------------
# Dispatch
# ----------------------------------------------------------------------------

PREPARE=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --prepare)    PREPARE=1; shift ;;
        --trace)      TRACE_FILE="$2"; shift 2 ;;
        --out)        OUT_WAV="$2"; shift 2 ;;
        --post-roll)  POST_ROLL="$2"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        *)            die "unknown argument: $1 (try --help)" ;;
    esac
done

if (( PREPARE )); then
    prepare_firmware
else
    run_demo
fi
