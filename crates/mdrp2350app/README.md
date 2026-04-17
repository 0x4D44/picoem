# mdrp2350app

Interactive TUI (ratatui/crossterm) for the `mdrp2350` RP2350 / RP2354
emulator: register / memory / trace inspection and firmware loading for
the dual-core Cortex-M33 complex.

## RISC-V (Hazard3) — library only in P5

As of Phase 5 of the RISC-V Hazard3 HLD
(`wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`),
the `mdrp2350` library can construct a RISC-V emulator via

```rust
let mut emu = EmulatorBuilder::new(Config::default())
    .arch(Arch::RiscV)
    .build();
```

and run bare-asm `rv32i` firmware (e.g. `roms/rp2350/blinky-riscv.bin`
from `gen_blinky_riscv.py`). The integration smoke test lives at
`crates/mdrp2350/tests/hello_riscv_blinky.rs`.

**This TUI is Arm-specific.** Its panels render Cortex-M33 state —
FPU, RCP, DCP, banked SP, secure / non-secure context, xPSR, ARMv8-M
exception entry diagnostics — that have no Hazard3 analogue. Wiring up
an `--arch riscv` flag here would need a parallel panel set for Hazard3
state (x0..x31, `mstatus`, `mcause`, `mepc`, the `xh3irq` CSR window,
`mcycle`/`minstret`, `wfi` park flag) as well as dispatch-by-`Arch` in
every existing panel.

P5 deliberately ships the library and smoke firmware only. The TUI
integration is **a follow-up phase**; until it lands, use the library
API directly from a test or a small custom binary if you need to drive a
Hazard3 emulator interactively.
