# mdrp2354 - RP2354 Emulator

## Build & Test

```bash
# Build everything
cargo build --release

# Run unit tests
cargo test

# Run code coverage
cargo llvm-cov
```

## Differential Fuzz Testing (QEMU harness)

The QEMU differential test harness compares our emulator's instruction execution against QEMU.

```bash
# Run N random fuzz tests per instruction class
cargo run -p mdrp2354-test-harness --release --bin qemu_diff -- --fuzz <N>

# Reproducible run with a specific seed
cargo run -p mdrp2354-test-harness --release --bin qemu_diff -- --fuzz <N> --seed <S>

# Run targeted edge-case tests only (default, no args)
cargo run -p mdrp2354-test-harness --release --bin qemu_diff
```

### Typical fuzz sessions

| Goal | Command |
|---|---|
| Quick smoke test | `--fuzz 1000` |
| Standard session | `--fuzz 100000` |
| Extended soak | `--fuzz 1000000` (or more, time permitting) |

When asked to "fuzz test" or "do some fuzzing", run with `--fuzz 100000` as the default unless a different count or duration is specified. For time-based requests ("fuzz for 2 hours"), estimate iterations based on prior run throughput and adjust accordingly.

### Handling failures

When the harness reports a mismatch:
1. Note the seed and instruction class from the failure output
2. Reproduce with `--seed <S>` to get a deterministic repro
3. Investigate the specific instruction's decode/execute path in our emulator
4. Fix and re-run the same seed to confirm the fix
