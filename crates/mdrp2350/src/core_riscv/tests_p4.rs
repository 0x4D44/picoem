// P4 scenario tests — Hazard3 external IRQ controller, MTIP/MSIP wiring,
// trap delivery at instruction boundary, and `wfi` wake. HLD §4.6, §7.
//
// Aim is practical coverage of the 11 categories in the HLD §7 taxonomy,
// not exhaustive (the HLD marks P4 as "Unit-tested only"; Verified requires
// P6 silicon). Test style: construct a Hazard3 + Bus, drive state directly
// via CSR ops and the xh3irq struct, dispatch via `execute` for CSR
// round-trips, and `step` for trap-delivery checks.
//
// A full-emulator test (via `Emulator::step`) is used where `fan_out_riscv_irqs`
// behaviour matters — the `fan_out_riscv_irqs` hook only fires on the
// emulator-level step path.

use super::csr::{
    CSR_MEICONTEXT, CSR_MEIEA, CSR_MEIFA, CSR_MEINEXT,
    CSR_MEIPA, CSR_MEIPRA, CSR_MEPC,
};
use super::decode::{CsrKind, Op};
use super::irq::{CTX_CLEARTS, CTX_MRETEIRQ, CTX_NOIRQ};
use super::Hazard3;
use crate::{Arch, Bus, Config, Cores, EmulatorBuilder};

// ---------- helpers ----------

fn fresh() -> (Hazard3, Bus) {
    (Hazard3::new(0), Bus::new())
}

/// CSR-write a CSR via the `Op::Csr` path (csrrw; rs1 register set to rv).
fn csr_write(c: &mut Hazard3, bus: &mut Bus, csr: u16, val: u32) {
    c.x[5] = val;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 6, rs1_or_zimm: 5, csr },
        bus,
        0x2000_0000,
    );
}

/// CSR-read a CSR via the `Op::Csr` path (csrrs rs1=x0 to read without
/// side effect). Returns the observed value.
fn csr_read(c: &mut Hazard3, bus: &mut Bus, csr: u16) -> u32 {
    c.execute(
        Op::Csr { kind: CsrKind::Csrrs, rd: 7, rs1_or_zimm: 0, csr },
        bus,
        0x2000_0000,
    );
    c.x[7]
}

// ---------- Per-CSR-window read/write round-trip (6 tests) ----------

#[test]
fn p4_csr_meiea_roundtrip() {
    let (mut c, mut bus) = fresh();
    // Window 0, bits 0 + 3 + 15 enabled.
    csr_write(&mut c, &mut bus, CSR_MEIEA, (0x8009u32 << 16) | 0);
    let got = csr_read(&mut c, &mut bus, CSR_MEIEA);
    assert_eq!(got >> 16, 0x8009);
    assert_eq!(got & 0x1F, 0);
}

#[test]
fn p4_csr_meipa_shows_enabled_pending() {
    let (mut c, mut bus) = fresh();
    csr_write(&mut c, &mut bus, CSR_MEIEA, (0x0003u32 << 16) | 0);
    bus.atomics.set_irq_pending(0, 0b111);
    let got = csr_read(&mut c, &mut bus, CSR_MEIPA);
    // Only bits 0, 1 visible after enable mask.
    assert_eq!(got >> 16, 0b011);
}

#[test]
fn p4_csr_meifa_w1c() {
    let (mut c, mut bus) = fresh();
    c.xh3irq.force_set(3);
    c.xh3irq.force_set(5);
    // Write-1 to bit 3 clears only bit 3.
    csr_write(&mut c, &mut bus, CSR_MEIFA, (0b1000u32 << 16) | 0);
    let got = csr_read(&mut c, &mut bus, CSR_MEIFA);
    assert_eq!(got >> 16, 0b0010_0000);
}

#[test]
fn p4_csr_meipra_roundtrip() {
    let (mut c, mut bus) = fresh();
    csr_write(&mut c, &mut bus, CSR_MEIPRA, (0x050Au32 << 16) | 0);
    let got = csr_read(&mut c, &mut bus, CSR_MEIPRA);
    assert_eq!(got >> 16, 0x050A);
    assert_eq!(c.xh3irq.meipra[0], 0xA);
    assert_eq!(c.xh3irq.meipra[2], 0x5);
}

#[test]
fn p4_csr_meinext_noirq_when_no_pending() {
    let (mut c, mut bus) = fresh();
    let got = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!(got >> 31, 1);
}

#[test]
fn p4_csr_meicontext_reset_noirq() {
    let (mut c, mut bus) = fresh();
    let got = csr_read(&mut c, &mut bus, CSR_MEICONTEXT);
    assert_eq!(got & CTX_NOIRQ, CTX_NOIRQ);
}

// ---------- meinext priority arbitration (3 tests) ----------

#[test]
fn p4_meinext_lowest_numbered_wins() {
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = (1 << 3) | (1 << 5);
    bus.atomics.set_irq_pending(0, (1 << 3) | (1 << 5));
    let got = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!((got & 0x7FC) >> 2, 3);
}

#[test]
fn p4_meinext_higher_priority_wins() {
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = (1 << 3) | (1 << 5);
    c.xh3irq.meipra[3] = 1;
    c.xh3irq.meipra[5] = 10;
    bus.atomics.set_irq_pending(0, (1 << 3) | (1 << 5));
    let got = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!((got & 0x7FC) >> 2, 5);
}

#[test]
fn p4_meinext_ppreempt_masks_low_priority() {
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = (1 << 3) | (1 << 7);
    c.xh3irq.meipra[3] = 2;
    c.xh3irq.meipra[7] = 5;
    c.xh3irq.meicontext = 4u32 << 24;
    bus.atomics.set_irq_pending(0, (1 << 3) | (1 << 7));
    let got = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!((got & 0x7FC) >> 2, 7);
}

// ---------- meicontext save/restore (2 tests) ----------

#[test]
fn p4_meicontext_entry_push() {
    let (mut c, _bus) = fresh();
    c.xh3irq.on_ext_irq_entry(5, 3);
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 4);
    assert_eq!(c.xh3irq.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ);
    assert_eq!(c.xh3irq.meicontext & CTX_NOIRQ, 0);
}

#[test]
fn p4_meicontext_mret_pop() {
    let (mut c, mut bus) = fresh();
    // Enter IRQ, then mret.
    c.xh3irq.on_ext_irq_entry(5, 3);
    csr_write(&mut c, &mut bus, CSR_MEPC, 0x2000_2000);
    c.mret();
    // After single-level mret, preempt popped to 0, noirq set.
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 0);
    assert_eq!(c.xh3irq.meicontext & CTX_NOIRQ, CTX_NOIRQ);
    assert_eq!(c.xh3irq.meicontext & CTX_MRETEIRQ, 0);
}

// ---------- meifa force-bit (2 tests) ----------

#[test]
fn p4_meifa_force_raises_meip() {
    let (c, bus) = {
        let (mut c, bus) = fresh();
        c.xh3irq.meiea = 1 << 4;
        c.xh3irq.force_set(4);
        (c, bus)
    };
    assert!(c.compute_meip(bus.atomics.irq_pending_load(0)));
}

#[test]
fn p4_meifa_ack_via_meinext_update() {
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = 1 << 4;
    c.xh3irq.force_set(4);
    // Write meinext with update=1 — acks the pending IRQ by clearing meifa.
    csr_write(&mut c, &mut bus, CSR_MEINEXT, 1);
    assert_eq!(c.xh3irq.meifa & (1 << 4), 0);
}

// ---------- meipra priority changes (2 tests) ----------

#[test]
fn p4_meipra_change_at_rest() {
    let (mut c, mut bus) = fresh();
    csr_write(&mut c, &mut bus, CSR_MEIPRA, (0x000A_u32 << 16) | 0);
    assert_eq!(c.xh3irq.meipra[0], 0xA);
    // Overwrite IRQ 0 priority.
    csr_write(&mut c, &mut bus, CSR_MEIPRA, (0x0005_u32 << 16) | 0);
    assert_eq!(c.xh3irq.meipra[0], 0x5);
    // IRQ 1..3 default still 0.
    assert_eq!(c.xh3irq.meipra[1], 0);
}

#[test]
fn p4_meipra_affects_arbitration() {
    let (mut c, bus) = fresh();
    c.xh3irq.meiea = (1 << 3) | (1 << 5);
    c.xh3irq.meipra[3] = 2;
    c.xh3irq.meipra[5] = 2;
    // At equal priority: lower-numbered wins.
    let r = c.xh3irq.read_meinext((1u64 << 3) | (1u64 << 5));
    assert_eq!((r & 0x7FC) >> 2, 3);
    // Raise IRQ 5's priority — it wins.
    c.xh3irq.meipra[5] = 5;
    let r = c.xh3irq.read_meinext((1u64 << 3) | (1u64 << 5));
    assert_eq!((r & 0x7FC) >> 2, 5);
    let _ = bus;
}

// ---------- MEIP↔meinext interlock (2 tests) ----------

#[test]
fn p4_meip_raised_by_enable_bit() {
    let (mut c, mut bus) = fresh();
    bus.atomics.set_irq_pending(0, 1 << 4);
    assert!(!c.compute_meip(bus.atomics.irq_pending_load(0)), "disabled -> no MEIP");
    // Enable IRQ 4 via window 0.
    csr_write(&mut c, &mut bus, CSR_MEIEA, (0x0010_u32 << 16) | 0);
    assert!(c.compute_meip(bus.atomics.irq_pending_load(0)));
}

#[test]
fn p4_meip_cleared_when_pending_clears() {
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = 1 << 4;
    bus.atomics.set_irq_pending(0, 1 << 4);
    assert!(c.compute_meip(bus.atomics.irq_pending_load(0)));
    bus.atomics.set_irq_pending(0, 0);
    assert!(!c.compute_meip(bus.atomics.irq_pending_load(0)));
}

// ---------- wfi wake (5 tests total) ----------

/// Helper: build an Emulator with a RISC-V hart in `wfi_parked` state and
/// the specified `mie`/`mstatus` init.
fn build_emu_parked(mie: u32, mie_global: bool) -> crate::Emulator {
    let mut emu = EmulatorBuilder::new(Config::default())
        .arch(Arch::RiscV)
        .build().unwrap();
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].wfi_parked = true;
        cs[0].csrs.mie = mie;
        if mie_global {
            cs[0].csrs.mstatus |= 1 << 3;
        }
    }
    emu
}

/// Plant a `wfi` (0x10500073) 32-bit instruction at `addr` so a trap
/// handler landing there self-parks on the next step. Keeps the test
/// focused on the *single* trap-entry event rather than runaway
/// handler-fetch-illegal loops.
fn plant_wfi_handler(bus: &mut Bus, addr: u32) {
    let offset = addr & 0x0FFF_FFFF;
    bus.memory.sram_write32(offset, 0x1050_0073);
}

/// Also prevent the SIO MTIME tick from auto-re-asserting MTIP during
/// the quantum. Setting `mtime_ctrl` bit 0 = 0 halts the counter;
/// setting `mtimecmp` = u64::MAX also works.
fn freeze_mtime(bus: &mut Bus) {
    bus.sio.mtime_ctrl = 0;
    bus.sio.mtimecmp = [u64::MAX; 2];
    bus.sio.mtime_match_asserted = [false; 2];
}

#[test]
fn p4_wfi_wake_from_mtip_mie1_delivers_trap() {
    let mut emu = build_emu_parked(1 << 7, true);
    freeze_mtime(&mut emu.bus);
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].csrs.mtvec = 0x2000_1000;
    }
    plant_wfi_handler(&mut emu.bus, 0x2000_1000);
    emu.bus.sio.mtime_match_asserted[0] = true;
    emu.step().unwrap();
    assert!(!emu.cores.expect_riscv()[0].wfi_parked, "wake cleared wfi_parked");
    // Step again — trap should deliver (MIE=1, mip.MTIP=1, mie.MTIE=1).
    // Handler is `wfi` so the hart re-parks after trap entry.
    emu.step().unwrap();
    let c = &emu.cores.expect_riscv()[0];
    assert_eq!(c.csrs.mcause, 0x8000_0007, "mcause = interrupt-bit + MTIP");
    // pc is the handler's next-sequential after the `wfi` that self-parked
    // the hart (mtvec=0x2000_1000 + 4-byte wfi). Trap entry did set pc to
    // 0x2000_1000 which the executor then advanced past the wfi.
    assert_eq!(c.pc, 0x2000_1004, "pc advanced past wfi in handler");
}

#[test]
fn p4_wfi_wake_from_msip_mie1_delivers_trap() {
    let mut emu = build_emu_parked(1 << 3, true);
    freeze_mtime(&mut emu.bus);
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].csrs.mtvec = 0x2000_2000;
    }
    plant_wfi_handler(&mut emu.bus, 0x2000_2000);
    emu.bus.sio.write32(0x1A0, 0x1, 0);
    emu.step().unwrap();
    assert!(!emu.cores.expect_riscv()[0].wfi_parked);
    emu.step().unwrap();
    assert_eq!(emu.cores.expect_riscv()[0].csrs.mcause, 0x8000_0003);
}

#[test]
fn p4_wfi_wake_from_meip_mie1_delivers_trap() {
    let mut emu = build_emu_parked(1 << 11, true);
    freeze_mtime(&mut emu.bus);
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].csrs.mtvec = 0x2000_3000;
        cs[0].xh3irq.meiea = 1 << 5;
    }
    plant_wfi_handler(&mut emu.bus, 0x2000_3000);
    emu.bus.atomics.set_irq_pending(0, 1 << 5);
    emu.step().unwrap();
    assert!(!emu.cores.expect_riscv()[0].wfi_parked);
    emu.step().unwrap();
    assert_eq!(emu.cores.expect_riscv()[0].csrs.mcause, 0x8000_000B);
    let ctx = emu.cores.expect_riscv()[0].xh3irq.meicontext;
    assert_eq!((ctx >> 16) & 0xF, 1, "priority 0 + 1 => preempt=1");
    assert_eq!(ctx & CTX_MRETEIRQ, CTX_MRETEIRQ);
}

#[test]
fn p4_wfi_wake_mie_global_zero_no_trap() {
    // MIE=1 at the bit level but mstatus.MIE=0 globally: wake, but no trap
    // delivery. HLD §4.6 / RV-priv §3.3.2.
    let mut emu = build_emu_parked(1 << 7, false);
    freeze_mtime(&mut emu.bus);
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].csrs.mtvec = 0x2000_1000;
        // Plant a wfi at pc so the hart re-parks instead of faulting on
        // zeroed SRAM — that way mcause stays at its reset value (0).
        cs[0].pc = 0x2000_4000;
    }
    plant_wfi_handler(&mut emu.bus, 0x2000_4000);
    emu.bus.sio.mtime_match_asserted[0] = true;
    emu.step().unwrap();
    assert!(!emu.cores.expect_riscv()[0].wfi_parked, "wfi wakes regardless of MIE");
    emu.step().unwrap();
    // With MIE=0 global, the hart executes the `wfi` at pc=0x2000_4000
    // and parks again. mcause stays at reset value 0 — no trap.
    let mcause = emu.cores.expect_riscv()[0].csrs.mcause;
    assert_eq!(mcause & 0x8000_0000, 0,
        "MIE=0 global blocks interrupt delivery (mcause={:#x})", mcause);
    // mepc was never written (no trap entry), so it stays at reset value
    // 0. Confirms the trap never happened.
    assert_eq!(emu.cores.expect_riscv()[0].csrs.mepc, 0,
        "no trap entry occurred while MIE=0");
}

#[test]
fn p4_wfi_wake_mie_bit_clear_no_wake() {
    // mie bit clear — wake predicate `(mip & mie) != 0` fails even
    // though mip.MTIP is set. Hart stays parked.
    let mut emu = build_emu_parked(0, true);
    freeze_mtime(&mut emu.bus);
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].csrs.mtvec = 0x2000_1000;
    }
    emu.bus.sio.mtime_match_asserted[0] = true;
    emu.step().unwrap();
    assert!(emu.cores.expect_riscv()[0].wfi_parked,
        "mie.MTIE=0 -> predicate zero -> no wake");
}

// ---------- mie per-bit masking (3 tests) ----------

#[test]
fn p4_mie_msie_independent_of_mtie() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 3;   // MIE global
    c.csrs.mie = 1 << 3;       // only MSIE
    c.csrs.mip = (1 << 3) | (1 << 7); // both MSIP + MTIP asserted
    c.csrs.mtvec = 0x2000_1000;
    // Plant a nop at pc so fetch succeeds if no trap.
    bus.memory.sram_write32(0x2000, 0x0000_0013);
    c.pc = 0x2000_2000;
    c.step(&mut bus);
    // MSIP cause = 3.
    assert_eq!(c.csrs.mcause, 0x8000_0003);
}

#[test]
fn p4_mie_mtie_independent_of_msie() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 3;
    c.csrs.mie = 1 << 7;
    c.csrs.mip = (1 << 3) | (1 << 7);
    c.csrs.mtvec = 0x2000_1000;
    c.pc = 0x2000_2000;
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0x8000_0007);
}

#[test]
fn p4_mie_meie_independent() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 3;
    c.csrs.mie = 1 << 11;
    c.csrs.mip = 1 << 11;
    c.csrs.mtvec = 0x2000_1000;
    c.xh3irq.meiea = 1 << 8;
    bus.atomics.set_irq_pending(0, 1 << 8);
    c.pc = 0x2000_2000;
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0x8000_000B);
}

// ---------- mstatus.MIE global gate (2 tests) ----------

#[test]
fn p4_mstatus_mie_shuffle_on_trap_entry() {
    let (mut c, mut bus) = fresh();
    // MIE=1 initially.
    c.csrs.mstatus = 1 << 3;
    c.csrs.mie = 1 << 7;
    c.csrs.mip = 1 << 7;
    c.csrs.mtvec = 0x2000_1000;
    c.pc = 0x2000_2000;
    c.step(&mut bus);
    // On entry: MIE <- 0, MPIE <- old MIE (1).
    assert_eq!(c.csrs.mstatus & (1 << 3), 0, "MIE cleared");
    assert_eq!(c.csrs.mstatus & (1 << 7), 1 << 7, "MPIE = old MIE");
}

#[test]
fn p4_mstatus_mie_shuffle_on_mret() {
    let (mut c, _bus) = fresh();
    // Simulate in-handler state: MIE=0, MPIE=1.
    c.csrs.mstatus = 1 << 7;
    c.csrs.mepc = 0x2000_4000;
    c.mret();
    // After mret: MIE <- old MPIE (1), MPIE <- 1.
    assert_eq!(c.csrs.mstatus & (1 << 3), 1 << 3);
    assert_eq!(c.csrs.mstatus & (1 << 7), 1 << 7);
    assert_eq!(c.pc, 0x2000_4000);
}

// ---------- Nested preempt (1 test) ----------

#[test]
fn p4_nested_preempt_stack_push_then_pop() {
    // Simulate nested entry at two different priority levels, then
    // observe preempt stack pops correctly on two mrets.
    //
    // HW semantics: `mreteirq` stays asserted while the unbalanced-entry
    // depth is > 0, so a second mret from the outer handler continues
    // unwinding the stack without firmware needing to re-arm the bit.
    let (mut c, _bus) = fresh();
    c.xh3irq.on_ext_irq_entry(5, 1); // preempt=2, ppreempt=0
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 2);
    c.xh3irq.on_ext_irq_entry(9, 4); // preempt=5, ppreempt=2, pppreempt=0
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 5);
    assert_eq!((c.xh3irq.meicontext >> 24) & 0xF, 2);
    c.mret();
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 2, "pop to outer preempt");
    assert_eq!(c.xh3irq.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ,
        "mreteirq re-armed by HW while still nested");
}

#[test]
fn p4_nested_preempt_two_levels_unwinds_correctly() {
    // Regression for the nested-mret-popping bug: two levels of external
    // IRQ entry followed by two mrets must land back in thread context
    // with preempt=0 + noirq=1 without any firmware-level manual fixup.
    let (mut c, _bus) = fresh();
    c.xh3irq.on_ext_irq_entry(5, 1); // depth=1; preempt=2
    c.xh3irq.on_ext_irq_entry(9, 4); // depth=2; preempt=5, ppreempt=2
    // Inner mret: depth 2 -> 1, preempt pops to 2.
    c.mret();
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 2, "inner pop to outer level");
    assert_eq!(c.xh3irq.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ,
        "mreteirq stays asserted for the outer mret");
    assert_eq!(c.xh3irq.meicontext & CTX_NOIRQ, 0, "still in a handler");
    // Outer mret: depth 1 -> 0, preempt pops to 0 + noirq/mreteirq flip.
    c.mret();
    assert_eq!((c.xh3irq.meicontext >> 16) & 0xF, 0, "thread context");
    assert_eq!(c.xh3irq.meicontext & CTX_MRETEIRQ, 0,
        "mreteirq cleared when stack fully unwound");
    assert_eq!(c.xh3irq.meicontext & CTX_NOIRQ, CTX_NOIRQ);
}

// ---------- fan_out_riscv_irqs full-emulator wiring (1 test) ----------

#[test]
fn p4_fan_out_drives_mip_from_hw_sources() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .arch(Arch::RiscV)
        .build().unwrap();
    // Default mtimecmp=0 means the MTIP match-asserts as soon as the
    // mtime tick fires; freeze MTIME to assert per-core manually below.
    emu.bus.sio.mtime_ctrl = 0;
    emu.bus.sio.mtimecmp = [u64::MAX; 2];
    emu.bus.sio.mtime_match_asserted = [true, false];
    // Bit 0 of RISCV_SOFTIRQ = core 0 SW IRQ.
    emu.bus.sio.write32(0x1A0, 0x1, 0);
    // MEIP source for core 0: enable IRQ 4, set pending.
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].xh3irq.meiea = 1 << 4;
        cs[1].xh3irq.meiea = 0;
        // Park both harts so step_pair doesn't execute anything and
        // stomp on the test's mip expectations via trap delivery.
        cs[0].wfi_parked = true;
        cs[1].wfi_parked = true;
    }
    emu.bus.atomics.set_irq_pending(0, 1 << 4);
    emu.bus.atomics.set_irq_pending(1, 0);

    // Step — fan_out_riscv_irqs runs at quantum end.
    emu.step().unwrap();

    let c0_mip = emu.cores.expect_riscv()[0].mip();
    assert_eq!(c0_mip & (1 << 7), 1 << 7, "MTIP set");
    assert_eq!(c0_mip & (1 << 3), 1 << 3, "MSIP set");
    assert_eq!(c0_mip & (1 << 11), 1 << 11, "MEIP set");
    let c1_mip = emu.cores.expect_riscv()[1].mip();
    assert_eq!(c1_mip & ((1 << 7) | (1 << 3) | (1 << 11)), 0,
        "core 1 has no hw sources asserted");
}

// ---------- clearts (meicontext side effect) test ----------

#[test]
fn p4_meicontext_clearts_masks_timer_soft() {
    let (mut c, mut bus) = fresh();
    c.csrs.mie = (1 << 7) | (1 << 3) | (1 << 11);
    // Write clearts via CSR path.
    csr_write(&mut c, &mut bus, CSR_MEICONTEXT, CTX_CLEARTS);
    // mie.MTIE and mie.MSIE cleared; MEIE survives.
    assert_eq!(c.csrs.mie & (1 << 7), 0);
    assert_eq!(c.csrs.mie & (1 << 3), 0);
    assert_eq!(c.csrs.mie & (1 << 11), 1 << 11);
}

// ---------- mret_to_wfi cascade (2 tests) ----------
//
// A common low-power pattern: the handler's final instruction is `mret`,
// and the mepc points to a `wfi`. After `mret`, PC is at the wfi; stepping
// parks the hart; a new pending IRQ must wake it.

/// Plant a 32-bit RV32 instruction at the given SRAM address.
fn plant_insn(bus: &mut Bus, addr: u32, insn: u32) {
    let offset = addr & 0x0FFF_FFFF;
    bus.memory.sram_write32(offset, insn);
}

#[test]
fn p4_mret_to_wfi_parks_then_new_irq_wakes() {
    let mut emu = EmulatorBuilder::new(Config::default())
        .arch(Arch::RiscV)
        .build().unwrap();
    freeze_mtime(&mut emu.bus);
    let wfi_pc = 0x2000_5000u32;
    plant_insn(&mut emu.bus, wfi_pc, 0x1050_0073); // wfi

    if let Cores::RiscV(cs) = &mut emu.cores {
        // Mid-handler state: MPIE=1 (so MIE restores to 1 on mret); mepc
        // points at the wfi. The second hart parks so it doesn't race.
        cs[0].csrs.mstatus = 1 << 7;
        cs[0].csrs.mepc = wfi_pc;
        cs[0].csrs.mie = 1 << 7;   // MTIE
        cs[0].csrs.mtvec = 0x2000_6000;
        // Simulate that we were inside an external-IRQ handler by driving
        // xh3irq through one entry; the mret must pop cleanly.
        cs[0].xh3irq.on_ext_irq_entry(4, 2);
        cs[1].wfi_parked = true;
    }
    // Handler fires mret — PC jumps to wfi_pc, xh3irq pops.
    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].mret();
        assert_eq!(cs[0].pc, wfi_pc);
        assert_eq!(cs[0].csrs.mstatus & (1 << 3), 1 << 3, "MIE restored");
        assert_eq!(cs[0].xh3irq.meicontext & CTX_NOIRQ, CTX_NOIRQ,
            "preempt stack unwound on mret");
    }

    // Step once — executes wfi and parks.
    emu.step().unwrap();
    assert!(emu.cores.expect_riscv()[0].wfi_parked,
        "wfi parked the hart after mret landed on it");

    // Fire a new MTIP source. fan_out_riscv_irqs lifts mip[7]; wake_checks
    // clears wfi_parked.
    emu.bus.sio.mtime_match_asserted[0] = true;
    emu.step().unwrap();
    assert!(!emu.cores.expect_riscv()[0].wfi_parked,
        "new pending IRQ wakes the hart parked at wfi");
}

#[test]
fn p4_mret_to_wfi_immediately_woken_if_irq_already_pending() {
    // Variant: the IRQ source is already asserted at the moment of mret.
    // After mret the hart is at the wfi; the first emu.step() quantum
    // runs the wfi (parks), then fan_out + wake_checks un-parks because
    // (mip & mie) != 0. On the next emu.step() quantum the trap delivers
    // MTIP and the handler (a self-loop so the quantum's remaining
    // iterations don't drift into zeroed SRAM) runs.
    let mut emu = EmulatorBuilder::new(Config::default())
        .arch(Arch::RiscV)
        .build().unwrap();
    freeze_mtime(&mut emu.bus);
    let wfi_pc = 0x2000_5100u32;
    let handler_pc = 0x2000_6100u32;
    plant_insn(&mut emu.bus, wfi_pc, 0x1050_0073); // wfi
    // Handler: self-loop (`jal x0, 0`) — keeps the rest of the 64-cycle
    // quantum inside fetchable memory so we don't observe a subsequent
    // illegal-instruction trap overwriting mcause.
    plant_insn(&mut emu.bus, handler_pc, 0x0000_006F); // jal x0, 0

    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].csrs.mstatus = 1 << 7; // MPIE=1
        cs[0].csrs.mepc = wfi_pc;
        cs[0].csrs.mie = 1 << 7;
        cs[0].csrs.mtvec = handler_pc;
        cs[0].xh3irq.on_ext_irq_entry(4, 2);
        cs[1].wfi_parked = true;
    }
    // Assert the IRQ source BEFORE mret so it's latent through the cascade.
    emu.bus.sio.mtime_match_asserted[0] = true;

    if let Cores::RiscV(cs) = &mut emu.cores {
        cs[0].mret();
    }

    // One step: executes wfi (parks), then wake_checks un-parks because
    // (mip & mie) != 0.
    emu.step().unwrap();
    assert!(!emu.cores.expect_riscv()[0].wfi_parked,
        "pre-pending IRQ wakes immediately");
    // Next step delivers the MTIP trap; the self-loop keeps the hart at
    // handler_pc without further traps.
    emu.step().unwrap();
    let c = &emu.cores.expect_riscv()[0];
    assert_eq!(c.csrs.mcause, 0x8000_0007,
        "MTIP trap delivered after wake");
    assert_eq!(c.pc, handler_pc,
        "handler self-loop holds PC at the vector");
}

// ---------- IRQ coincident with mret (2 tests) ----------
//
// An IRQ arrives asynchronously while a handler is mret-ing. The mret
// completes normally, restoring MIE=1; on the next instruction boundary
// the pending IRQ immediately re-traps.

#[test]
fn p4_mret_with_pending_meip_retraps_on_next_step() {
    let (mut c, mut bus) = fresh();
    // In-handler state: MPIE=1 so mret restores MIE. mepc to a planted nop.
    c.csrs.mstatus = 1 << 7;
    c.csrs.mepc = 0x2000_7000;
    c.csrs.mie = 1 << 11;   // MEIE
    c.csrs.mtvec = 0x2000_8000;
    plant_insn(&mut bus, 0x2000_7000, 0x0000_0013); // nop after mret
    plant_insn(&mut bus, 0x2000_8000, 0x0000_0013); // handler nop
    // Pending IRQ that will re-fire after mret. Arbitration must succeed
    // (IRQ is enabled, ppreempt=0 in fresh state).
    c.xh3irq.meiea = 1 << 9;
    bus.atomics.set_irq_pending(0, 1 << 9);
    c.csrs.mip = 1 << 11;

    // Emulate an mret that races with the pending IRQ.
    c.mret();
    assert_eq!(c.pc, 0x2000_7000, "mret returns to mepc");
    assert_eq!(c.csrs.mstatus & (1 << 3), 1 << 3, "MIE restored");

    // Next step sees MEIP still asserted; trap fires before the nop.
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0x8000_000B, "re-trap as MEI");
    assert_eq!(c.csrs.mepc, 0x2000_7000,
        "mepc = PC of the un-executed instruction after the earlier mret");
    assert_eq!((c.xh3irq.meicontext >> 4) & 0x1FF, 9,
        "xh3irq latched the winning IRQ");
}

#[test]
fn p4_mret_with_pending_mtip_retraps_on_next_step() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 7;
    c.csrs.mepc = 0x2000_7100;
    c.csrs.mie = 1 << 7;
    c.csrs.mtvec = 0x2000_8100;
    plant_insn(&mut bus, 0x2000_7100, 0x0000_0013);
    plant_insn(&mut bus, 0x2000_8100, 0x0000_0013);
    c.csrs.mip = 1 << 7;  // MTIP still asserted at mret boundary

    c.mret();
    // Step: trap delivery fires immediately.
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0x8000_0007);
    assert_eq!(c.pc, 0x2000_8100, "direct-mode tvec — PC at base");
}

// ---------- meifa force while real pending (1 test) ----------

#[test]
fn p4_meifa_force_coexists_with_real_pending() {
    // Both `bus.atomics.irq_pending_load(0)` bit 5 AND meifa bit 5 set — meipa shows
    // the IRQ as pending. `meinext.update=1` clears meifa but the HW
    // pending bit stays, so a subsequent meinext read still reports IRQ 5.
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = 1 << 5;
    c.xh3irq.force_set(5);
    bus.atomics.set_irq_pending(0, 1 << 5);

    // meinext sees IRQ 5.
    let r = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!((r & 0x7FC) >> 2, 5);
    assert_eq!(r >> 31, 0);

    // Ack via write-1 to update — clears meifa.
    csr_write(&mut c, &mut bus, CSR_MEINEXT, 1);
    assert_eq!(c.xh3irq.meifa & (1 << 5), 0, "meifa bit 5 cleared");
    // But bus.irq_pending still carries the HW bit.
    assert_eq!(bus.atomics.irq_pending_load(0) & (1 << 5), 1 << 5,
        "HW pending is not touched by meinext.update");

    // Next read still shows IRQ 5 pending — the HW source is independent.
    let r2 = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!((r2 & 0x7FC) >> 2, 5);
    assert_eq!(r2 >> 31, 0, "IRQ still visible through HW pending");
}

// ---------- meipra priority change mid-handler (1 test) ----------

#[test]
fn p4_meipra_change_mid_handler_affects_meinext() {
    // Inside a nested handler, firmware re-prioritises a pending IRQ;
    // meinext must reflect the new priority arbitration immediately.
    //
    // Arbitration uses `ppreempt` (the slot holding the previous preempt
    // level) as the "treat-as-not-pending" threshold — see `arbitrate()`.
    // To make IRQ 7 visible-then-invisible we need a nested entry so
    // ppreempt holds a non-zero threshold.
    let (mut c, mut bus) = fresh();
    c.xh3irq.meiea = (1 << 3) | (1 << 7);
    c.xh3irq.meipra[3] = 5;
    c.xh3irq.meipra[7] = 2;
    bus.atomics.set_irq_pending(0, (1 << 3) | (1 << 7));
    // Outer entry at priority 5 — preempt=6, ppreempt=0.
    c.xh3irq.on_ext_irq_entry(3, 5);
    // Nested entry at priority 3 — preempt=4, ppreempt=6, pppreempt=0.
    // Now arbitrate's threshold is ppreempt=6; IRQ 3 (pri=5) and IRQ 7
    // (pri=2) are both below → meinext reports noirq.
    c.xh3irq.on_ext_irq_entry(5, 3);
    let r = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    assert_eq!(r >> 31, 1, "no IRQ above ppreempt threshold");

    // Firmware raises IRQ 7's priority to 10 — now above ppreempt=6.
    // Poke the storage directly (CSR windowed write would require
    // preserving the other priorities in the same window).
    c.xh3irq.meipra[7] = 10;
    let r2 = csr_read(&mut c, &mut bus, CSR_MEINEXT);
    let irq = (r2 & 0x7FC) >> 2;
    assert_eq!(r2 >> 31, 0, "IRQ 7 now above ppreempt");
    assert_eq!(irq, 7, "re-prioritisation makes IRQ 7 the arbitration winner");
}
