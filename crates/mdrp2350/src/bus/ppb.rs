// FPCCR bit positions (DDI0553 §D1.2.32). Public so other crate modules
// (exceptions.rs, execute_fpu.rs) can reference them by name.
pub const FPCCR_LSPACT:    u32 = 1 << 0;
pub const FPCCR_MMRDY:     u32 = 1 << 5;
pub const FPCCR_BFRDY:     u32 = 1 << 6;
pub const FPCCR_SPLIMVIOL: u32 = 1 << 9;
pub const FPCCR_LSPEN:     u32 = 1 << 30;
pub const FPCCR_ASPEN:     u32 = 1 << 31;

// DWT CTRL / DEMCR bit positions (DDI0553 §D1.2.1, §D1.2.22).
const DWT_CTRL_CYCCNTENA: u32 = 1 << 0;
const DEMCR_TRCENA:       u32 = 1 << 24;

// SysTick CSR bit positions (ARMv8-M §B11.1).
const SYST_CSR_ENABLE:    u32 = 1 << 0;
const SYST_CSR_TICKINT:   u32 = 1 << 1;
// CLKSOURCE is used for register round-trip only; scaling is deferred.
#[allow(dead_code)]
const SYST_CSR_CLKSOURCE: u32 = 1 << 2;
const SYST_CSR_COUNTFLAG: u32 = 1 << 16;

/// SysTick CVR is a 24-bit counter.
const SYST_24BIT_MASK: u32 = 0x00FF_FFFF;

// ICSR pending bits (ARMv8-M §B3.2.4). SET bits are W1S (write 1 sets,
// write 0 ignored). CLR bits are W1C for the corresponding SET bit
// (write 1 clears the SET bit, write 0 ignored). Other ICSR bits are
// read-only status; writes are preserved only in storage for round-trip.
pub(crate) const ICSR_NMIPENDSET: u32 = 1 << 31;
pub(crate) const ICSR_PENDSVSET:  u32 = 1 << 28;
const ICSR_PENDSVCLR:             u32 = 1 << 27;
pub(crate) const ICSR_PENDSTSET:  u32 = 1 << 26;
const ICSR_PENDSTCLR:             u32 = 1 << 25;

/// Per-core Private Peripheral Bus state (NVIC, SCB, SysTick stubs).
/// Phase 3: slim — only what the bootrom needs.
pub struct Ppb {
    // SCB registers
    pub vtor: u32,      // Vector Table Offset (0xE000ED08, reset: 0)
    pub aircr: u32,     // App Interrupt/Reset Control (0xE000ED0C)
    pub scr: u32,       // System Control (0xE000ED10)
    pub ccr: u32,       // Configuration Control (0xE000ED14, reset: 0x200)
    pub shpr: [u8; 12], // System Handler Priority, exceptions 4-15 (0xE000ED18-ED20)
    pub shcsr: u32,     // System Handler Control/Status (0xE000ED24)
    pub cfsr: u32,      // Configurable Fault Status (0xE000ED28)
    pub hfsr: u32,      // Hard Fault Status (0xE000ED2C)
    pub mmfar: u32,     // MemManage Fault Address (0xE000ED34)
    pub bfar: u32,      // Bus Fault Address (0xE000ED38)
    pub cpacr: u32,     // Coprocessor Access Control (0xE000ED88)
    pub icsr: u32,      // Interrupt Control/State (0xE000ED04)

    // FP extension registers (Phase 7 Stage B — DDI0553 §D1.2.32-34)
    //
    // Invariants enforced by the emulator:
    //   1. CONTROL.FPCA=1 ⇒ S0-S31 + FPSCR are live thread-mode state.
    //   2. FPCCR.LSPACT=1 ⇒ FPCAR points at a reserved FP frame; S0-S15
    //      and FPSCR are still the pre-exception values, not yet written.
    //   3. EXC_RETURN[4]=0 ⇒ exception entry reserved 18 words above the
    //      basic frame.
    //   4. Only fpu_execute writes FPCA=1; only enter_exception/
    //      exit_exception write FPCA=0 / restore it.
    //
    /// FP Context Control Register. Reset 0xC000_0000 (ASPEN=1, LSPEN=1).
    /// Bit layout per DDI0553 §D1.2.32:
    ///   [0] LSPACT   [1] USER     [2] S        [3] THREAD
    ///   [4] HFRDY    [5] MMRDY    [6] BFRDY    [7] SFRDY
    ///   [8] MONRDY   [9] SPLIMVIOL [10] UFRDY  (11-25 reserved)
    ///   [26] TS      [27] CLRONRETS [28] CLRONRET
    ///   [29] LSPENS  [30] LSPEN   [31] ASPEN
    /// Emulator actively models: ASPEN, LSPEN, LSPACT, SPLIMVIOL,
    /// MMRDY, BFRDY. Others are RW storage but inert.
    pub fpccr: u32,
    /// FP Context Address Register. Writes mask bits [2:0] to 0
    /// (8-byte alignment).
    pub fpcar: u32,
    /// FP Default Status Control. Template for FPSCR at exception entry;
    /// active bits are AHP (26), DN (25), FZ (24), RMODE (23:22).
    pub fpdscr: u32,

    // MPU (0xE000ED94-0xE000EDA0)
    pub mpu_ctrl: u32,                 // MPU Control (0xE000ED94)
    pub mpu_rnr: u32,                  // MPU Region Number (0xE000ED98)
    pub mpu_regions: [(u32, u32); 16], // 16 regions: (RBAR, RLAR) pairs

    // SAU (0xE000EDD0-0xE000EDE0)
    pub sau_ctrl: u32,                // SAU Control (bit 0 = enable, bit 1 = ALLNS)
    pub sau_rnr: u32,                 // Region Number Register (selects active region)
    pub sau_regions: [(u32, u32); 8], // 8 regions: (RBAR, RLAR) pairs

    // DWT (Quantum Execution Model Stage 2)
    //
    // Backing for CYCCNT is the `dwt_cyccnt_base` offset: firmware-visible
    // CYCCNT = (core.cycles as u32).wrapping_add(base) when DWT is enabled
    // (DEMCR.TRCENA AND DWT_CTRL.CYCCNTENA), else just `base`. Write to
    // CYCCNT stores `written - core.cycles` so the next live read returns
    // `written + cycles_elapsed_since_write`.
    /// DWT_CTRL at 0xE000_1000 — bit 0 is CYCCNTENA.
    pub dwt_ctrl: u32,
    /// Offset applied to live core cycles to produce CYCCNT reads.
    pub dwt_cyccnt_base: u32,
    /// DEMCR at 0xE000_EDFC — bit 24 is TRCENA (gates DWT entirely).
    pub demcr: u32,
    /// Latest published per-core cycle count. Refreshed by the scheduler
    /// before each core runs so CYCCNT read/write paths can compute against
    /// a recent snapshot without threading `core.cycles` through every bus
    /// access. Staleness is bounded by a single instruction.
    pub(crate) latest_cycles: u64,

    // SysTick (Quantum Execution Model Stage 2) — per-core, 24-bit down-counter.
    /// SYST_CSR at 0xE000_E010.
    pub syst_csr: u32,
    /// SYST_RVR at 0xE000_E014 — reload value (24-bit).
    pub syst_rvr: u32,
    /// SYST_CVR at 0xE000_E018 — current value (24-bit).
    pub syst_cvr: u32,
    /// Snapshot of the owning core's `cycles` at the last `systick_advance`.
    /// Delta since the previous tick is computed as
    /// `core.cycles - last_systick_cycles` and then subtracted from CVR.
    pub last_systick_cycles: u64,
}

impl Default for Ppb {
    fn default() -> Self {
        Self {
            vtor: 0,
            aircr: 0,
            scr: 0,
            ccr: 0x0000_0200, // STKALIGN=1
            shpr: [0; 12],
            shcsr: 0,
            cfsr: 0,
            hfsr: 0,
            mmfar: 0,
            bfar: 0,
            cpacr: 0x00F0_0000, // CP10/11 (FPU) full access
            icsr: 0,
            // ASPEN=1 (auto FP context save), LSPEN=1 (lazy enabled).
            fpccr: 0xC000_0000,
            fpcar: 0,
            fpdscr: 0,
            mpu_ctrl: 0,
            mpu_rnr: 0,
            mpu_regions: [(0, 0); 16],
            sau_ctrl: 0,
            sau_rnr: 0,
            sau_regions: [(0, 0); 8],
            dwt_ctrl: 0,
            dwt_cyccnt_base: 0,
            demcr: 0,
            latest_cycles: 0,
            syst_csr: 0,
            syst_rvr: 0,
            syst_cvr: 0,
            last_systick_cycles: 0,
        }
    }
}

impl Ppb {
    /// Pack 4 consecutive SHPR bytes into a u32 (little-endian).
    fn pack_shpr(&self, start: usize) -> u32 {
        u32::from_le_bytes([
            self.shpr[start],
            self.shpr[start + 1],
            self.shpr[start + 2],
            self.shpr[start + 3],
        ])
    }

    /// Unpack a u32 into 4 consecutive SHPR bytes.
    /// Only bits [7:5] per byte are implemented on Cortex-M33.
    fn unpack_shpr(&mut self, start: usize, val: u32) {
        let bytes = val.to_le_bytes();
        for i in 0..4 {
            self.shpr[start + i] = bytes[i] & 0xE0;
        }
    }

    pub fn read32(&mut self, addr: u32) -> u32 {
        match addr & 0xFFFF {
            // ICTR — Interrupt Controller Type: 64 external IRQ lines
            0xE004 => 1,

            // SYST_CSR — return current CSR value and clear COUNTFLAG as a
            // side effect (ARMv8-M spec: COUNTFLAG reads as 1 since the last
            // time it was read). Taking `&mut self` lets us implement this
            // without a shadow-read path.
            0xE010 => {
                let out = self.syst_csr;
                self.syst_csr &= !SYST_CSR_COUNTFLAG;
                out
            }
            // SYST_RVR — 24-bit reload value.
            0xE014 => self.syst_rvr & SYST_24BIT_MASK,
            // SYST_CVR — 24-bit current value.
            0xE018 => self.syst_cvr & SYST_24BIT_MASK,
            // SYST_CALIB — RP2350 doesn't expose calibration; return 0.
            0xE01C => 0,

            // NVIC (stub)
            0xE100..=0xE4FF => 0,

            // DWT_CTRL — CYCCNTENA + reserved bits.
            0x1000 => self.dwt_ctrl,
            // DWT_CYCCNT — gated by DEMCR.TRCENA AND DWT_CTRL.CYCCNTENA.
            0x1004 => self.read_cyccnt(self.latest_cycles),

            // CPUID
            0xED00 => 0x411F_D210,

            // ICSR
            0xED04 => self.icsr,

            // VTOR
            0xED08 => self.vtor,

            // AIRCR
            0xED0C => self.aircr,

            // SCR
            0xED10 => self.scr,

            // CCR
            0xED14 => self.ccr,

            // SHPR1 (exceptions 4-7)
            0xED18 => self.pack_shpr(0),

            // SHPR2 (exceptions 8-11)
            0xED1C => self.pack_shpr(4),

            // SHPR3 (exceptions 12-15)
            0xED20 => self.pack_shpr(8),

            // SHCSR
            0xED24 => self.shcsr,

            // CFSR
            0xED28 => self.cfsr,

            // HFSR
            0xED2C => self.hfsr,

            // MMFAR
            0xED34 => self.mmfar,

            // BFAR
            0xED38 => self.bfar,

            // CPACR
            0xED88 => self.cpacr,

            // FPCCR / FPCAR / FPDSCR (Phase 7 Stage B)
            0xEF34 => self.fpccr,
            0xEF38 => self.fpcar,
            0xEF3C => self.fpdscr,

            // MPU_TYPE: 16 regions on RP2350 Cortex-M33
            0xED90 => 0x0000_1000, // DREGION=16, IREGION=0, SEPARATE=0
            // MPU_CTRL
            0xED94 => self.mpu_ctrl,
            // MPU_RNR
            0xED98 => self.mpu_rnr,
            // MPU_RBAR
            0xED9C => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].0
            }
            // MPU_RLAR
            0xEDA0 => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].1
            }
            // MPU_RBAR_A1 / RLAR_A1 / ... A3 (ARMv8-M §B11.2.5-8):
            // alias registers access region `(RNR & !3) | n` for n ∈ {1,2,3}.
            // Surfaced by the bootrom's MPU readback self-test which writes
            // all four (base, alias1, alias2, alias3) pairs in a single stmia.
            0xEDA4 | 0xEDAC | 0xEDB4 => {
                let n = ((addr as usize) - 0xEDA4) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].0
            }
            0xEDA8 | 0xEDB0 | 0xEDB8 => {
                let n = ((addr as usize) - 0xEDA8) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].1
            }

            // SAU_CTRL
            0xEDD0 => self.sau_ctrl,
            // SAU_TYPE: 8 regions (RP2350 has 8)
            0xEDD4 => 8,
            // SAU_RNR
            0xEDD8 => self.sau_rnr,
            // SAU_RBAR: bits [4:0] are RES0
            0xEDDC => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].0 & !0x1F
            }
            // SAU_RLAR
            0xEDE0 => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].1
            }

            // DEMCR — Debug Exception and Monitor Control, TRCENA at bit 24.
            0xEDFC => self.demcr,

            // Unknown PPB register
            _ => 0,
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        match addr & 0xFFFF {
            // SYST_CSR — preserve COUNTFLAG (read-clears only), accept the
            // other configuration bits. TODO: CLKSOURCE=0 ref-clock scaling.
            0xE010 => {
                let preserved = self.syst_csr & SYST_CSR_COUNTFLAG;
                self.syst_csr = (val & !SYST_CSR_COUNTFLAG) | preserved;
            }
            // SYST_RVR — 24-bit reload value.
            0xE014 => self.syst_rvr = val & SYST_24BIT_MASK,
            // SYST_CVR — any write clears both CVR and COUNTFLAG (ARMv8-M spec).
            0xE018 => {
                self.syst_cvr = 0;
                self.syst_csr &= !SYST_CSR_COUNTFLAG;
            }
            // SYST_CALIB — read-only, writes ignored.
            0xE01C => {}

            // NVIC (stub — accept and ignore)
            0xE100..=0xE4FF => {}

            // DWT_CTRL — only CYCCNTENA (bit 0) is modelled; other bits
            // are stored for firmware round-trip.
            0x1000 => self.dwt_ctrl = val,
            // DWT_CYCCNT — compute new base so live reads yield
            // `written + cycles_elapsed_since_write`.
            0x1004 => self.write_cyccnt(val, self.latest_cycles),

            // CPUID — read-only, ignore writes
            0xED00 => {}

            // ICSR — ARMv8-M §B3.2.4: pend bits (PENDSVSET, PENDSTSET,
            // NMIPENDSET) are W1S; clear bits (PENDSVCLR, PENDSTCLR) are
            // W1C for the corresponding SET bit. Writing 0 to any of these
            // is ignored. If a SET and its CLR are written in the same
            // store, CLR wins (apply CLR after SET so the net effect is
            // "not pended"). Other ICSR bits are read-only status.
            0xED04 => {
                if val & ICSR_NMIPENDSET != 0 { self.icsr |= ICSR_NMIPENDSET; }
                if val & ICSR_PENDSVSET  != 0 { self.icsr |= ICSR_PENDSVSET;  }
                if val & ICSR_PENDSTSET  != 0 { self.icsr |= ICSR_PENDSTSET;  }
                if val & ICSR_PENDSVCLR  != 0 { self.icsr &= !ICSR_PENDSVSET; }
                if val & ICSR_PENDSTCLR  != 0 { self.icsr &= !ICSR_PENDSTSET; }
            }

            // VTOR — 128-byte aligned
            0xED08 => self.vtor = val & !0x7F,

            // AIRCR
            0xED0C => self.aircr = val,

            // SCR
            0xED10 => self.scr = val,

            // CCR
            0xED14 => self.ccr = val,

            // SHPR1 (exceptions 4-7)
            0xED18 => self.unpack_shpr(0, val),

            // SHPR2 (exceptions 8-11)
            0xED1C => self.unpack_shpr(4, val),

            // SHPR3 (exceptions 12-15)
            0xED20 => self.unpack_shpr(8, val),

            // SHCSR
            0xED24 => self.shcsr = val,

            // CFSR — write-1-to-clear
            0xED28 => self.cfsr &= !val,

            // HFSR — write-1-to-clear
            0xED2C => self.hfsr &= !val,

            // MMFAR
            0xED34 => self.mmfar = val,

            // BFAR
            0xED38 => self.bfar = val,

            // CPACR
            0xED88 => self.cpacr = val,

            // FPCCR / FPCAR / FPDSCR (Phase 7 Stage B). FPCAR is force-aligned
            // to 8 bytes (DDI0553 §D1.2.33). FPCCR has reserved bits but no
            // mask is applied — software is allowed to write the full word.
            0xEF34 => self.fpccr = val,
            0xEF38 => self.fpcar = val & !0x7,
            0xEF3C => self.fpdscr = val,

            // MPU_TYPE: read-only
            0xED90 => {}
            // MPU_CTRL
            0xED94 => self.mpu_ctrl = val,
            // MPU_RNR
            0xED98 => self.mpu_rnr = val & 0xF,
            // MPU_RBAR (ARMv8-M §B11.2.5): [31:5] BASE, [4:3] SH,
            // [2:1] AP, [0] XN — all bits carry meaning.
            0xED9C => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].0 = val;
            }
            // MPU_RLAR (ARMv8-M §B11.2.8): [31:5] LIMIT, [4] RES0,
            // [3:1] AttrIndx, [0] EN. Mask bit [4] so it reads back as 0
            // (the bootrom's readback self-test depends on this).
            0xEDA0 => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].1 = val & !0x10;
            }
            // MPU_RBAR_An / RLAR_An aliases — see read path for definition.
            0xEDA4 | 0xEDAC | 0xEDB4 => {
                let n = ((addr as usize) - 0xEDA4) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].0 = val;
            }
            0xEDA8 | 0xEDB0 | 0xEDB8 => {
                let n = ((addr as usize) - 0xEDA8) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].1 = val & !0x10;
            }

            // SAU_CTRL
            0xEDD0 => self.sau_ctrl = val,
            // SAU_TYPE: read-only, ignore writes
            0xEDD4 => {}
            // SAU_RNR
            0xEDD8 => self.sau_rnr = val & 0x7,
            // SAU_RBAR
            0xEDDC => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].0 = val;
            }
            // SAU_RLAR
            0xEDE0 => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].1 = val;
            }

            // DEMCR — only TRCENA (bit 24) is modelled; other bits are
            // stored for firmware round-trip.
            0xEDFC => self.demcr = val,

            // Unknown PPB register — ignore
            _ => {}
        }
    }

    /// Get the priority of a system exception (4-15) from SHPR.
    /// Returns i16: HardFault=-1, others from shpr[]. Only bits [7:5] used.
    pub fn exception_priority(&self, exc_num: u16) -> i16 {
        match exc_num {
            1 => -3,  // Reset
            2 => -2,  // NMI
            3 => -1,  // HardFault (fixed)
            4..=15 => (self.shpr[(exc_num - 4) as usize] & 0xE0) as i16,
            _ => 0,   // External IRQs default to 0 (Phase 5 will add NVIC_IPR)
        }
    }

    /// Clear the active bit for an exception. Phase 3 stub: just clear IPSR-related state in ICSR.
    pub fn clear_active(&mut self, _exc_num: u16) {
        // Phase 3: no NVIC active tracking. ICSR.VECTACTIVE handled by core IPSR.
    }

    // ----------------------------------------------------------------
    // DWT CYCCNT + SysTick — Quantum Execution Model Stage 2
    // ----------------------------------------------------------------

    /// Publish a per-core cycle count snapshot. Called by the scheduler
    /// before the owning core runs so DWT_CYCCNT reads/writes compute
    /// against a recent value without threading `core.cycles` through
    /// every bus access. Staleness is bounded by one instruction.
    pub fn update_latest_cycles(&mut self, cycles: u64) {
        self.latest_cycles = cycles;
    }

    /// Read DWT_CYCCNT computed against the supplied live cycle count.
    /// When DWT is enabled (DEMCR.TRCENA AND DWT_CTRL.CYCCNTENA), returns
    /// `(cycles as u32) + dwt_cyccnt_base`; otherwise returns the stored
    /// base (whatever was last written).
    pub fn read_cyccnt(&self, core_cycles: u64) -> u32 {
        if (self.demcr & DEMCR_TRCENA) != 0
            && (self.dwt_ctrl & DWT_CTRL_CYCCNTENA) != 0
        {
            (core_cycles as u32).wrapping_add(self.dwt_cyccnt_base)
        } else {
            self.dwt_cyccnt_base
        }
    }

    /// Write DWT_CYCCNT. Stores `written - core_cycles` so the next live
    /// read returns `written + cycles_elapsed_since_write`.
    pub fn write_cyccnt(&mut self, written: u32, core_cycles: u64) {
        // Wrapping is safe: core_cycles is a monotonic u64, the u32 truncation
        // and wrapping subtract reproduce 32-bit modular arithmetic so later
        // `read_cyccnt` returns `(written + elapsed) mod 2^32` — the exact
        // architectural behaviour of the 32-bit CYCCNT register.
        self.dwt_cyccnt_base = written.wrapping_sub(core_cycles as u32);
    }

    /// Advance SysTick by `core_cycles - last_systick_cycles` cycles. Called
    /// once per quantum end for each core. Multi-reload within a single tick
    /// is handled via the `loop`.
    ///
    /// TODO: CLKSOURCE=0 ref-clock scaling — currently all cycles tick the
    /// counter regardless of CLKSOURCE.
    pub fn systick_advance(&mut self, core_cycles: u64) {
        let delta = core_cycles.wrapping_sub(self.last_systick_cycles);
        self.last_systick_cycles = core_cycles;

        if self.syst_csr & SYST_CSR_ENABLE == 0 {
            return;
        }

        // Saturating downcast: a delta larger than u32 would imply a quantum
        // of 4-billion cycles which we don't support. Anything that big is
        // treated as u32::MAX and will still correctly trigger reloads.
        let mut rem: u32 = if delta > u32::MAX as u64 {
            u32::MAX
        } else {
            delta as u32
        };

        loop {
            if rem <= self.syst_cvr {
                self.syst_cvr -= rem;
                break;
            }
            // Underflow: consume `cvr + 1` cycles to reach 0 and wrap.
            rem -= self.syst_cvr + 1;
            self.syst_cvr = self.syst_rvr & SYST_24BIT_MASK;
            self.syst_csr |= SYST_CSR_COUNTFLAG;
            if self.syst_csr & SYST_CSR_TICKINT != 0 {
                self.pend_systick();
            }
            // When RVR=0 we fire the single underflow for this advance but do
            // NOT latch a stopped state. Firmware that leaves RVR=0 with
            // ENABLE=1 will see one COUNTFLAG+PENDSTSET per quantum. Real
            // Armv8-M §B11.2.1 stops the counter; follow-up if firmware relies
            // on that. The `break` is required here to prevent an infinite
            // loop within this call when `rem > 0` and CVR has just reloaded
            // to 0.
            if (self.syst_rvr & SYST_24BIT_MASK) == 0 {
                break;
            }
        }
    }

    /// Set ICSR.PENDSTSET (bit 26) — SysTick exception (#15) pending.
    /// ARMv8-M §B3.2.4. This is the architectural mechanism by which
    /// firmware (and the exception-dispatch infrastructure) observes a
    /// SysTick pending state.
    pub fn pend_systick(&mut self) {
        self.icsr |= ICSR_PENDSTSET;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpuid_read() {
        let mut ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_ED00), 0x411F_D210);
    }

    #[test]
    fn test_vtor_roundtrip() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED08, 0x200);
        assert_eq!(ppb.read32(0xE000_ED08), 0x200);
    }

    #[test]
    fn test_vtor_alignment() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED08, 0x201);
        assert_eq!(ppb.read32(0xE000_ED08), 0x200);
    }

    #[test]
    fn test_shpr_roundtrip() {
        let mut ppb = Ppb::default();
        // Write SHPR1 with packed bytes: priorities 0x20, 0x40, 0x60, 0xE0
        let val = u32::from_le_bytes([0x20, 0x40, 0x60, 0xE0]);
        ppb.write32(0xE000_ED18, val);
        assert_eq!(ppb.read32(0xE000_ED18), val);

        // Verify individual bytes (only bits [7:5] survive)
        assert_eq!(ppb.shpr[0], 0x20);
        assert_eq!(ppb.shpr[1], 0x40);
        assert_eq!(ppb.shpr[2], 0x60);
        assert_eq!(ppb.shpr[3], 0xE0);
    }

    #[test]
    fn test_cfsr_write_one_to_clear() {
        let mut ppb = Ppb::default();
        ppb.cfsr = 0xFF;
        ppb.write32(0xE000_ED28, 0x0F);
        assert_eq!(ppb.read32(0xE000_ED28), 0xF0);
    }

    // --- ICSR W1S/W1C semantics (ARMv8-M §B3.2.4) ---

    #[test]
    fn test_icsr_pendsv_set_w1s() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED04, ICSR_PENDSVSET);
        assert_ne!(ppb.read32(0xE000_ED04) & ICSR_PENDSVSET, 0);
    }

    #[test]
    fn test_icsr_write_zero_preserves_set_bit() {
        let mut ppb = Ppb::default();
        ppb.icsr = ICSR_PENDSVSET;
        // Writing 0 to PENDSVSET must NOT clear it (W1S — write 0 ignored).
        ppb.write32(0xE000_ED04, 0);
        assert_ne!(ppb.read32(0xE000_ED04) & ICSR_PENDSVSET, 0);
    }

    #[test]
    fn test_icsr_pendsv_clr_clears_set() {
        let mut ppb = Ppb::default();
        ppb.icsr = ICSR_PENDSVSET;
        ppb.write32(0xE000_ED04, ICSR_PENDSVCLR);
        assert_eq!(ppb.read32(0xE000_ED04) & ICSR_PENDSVSET, 0);
    }

    #[test]
    fn test_icsr_pendst_set_w1s() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED04, ICSR_PENDSTSET);
        assert_ne!(ppb.read32(0xE000_ED04) & ICSR_PENDSTSET, 0);
    }

    #[test]
    fn test_icsr_pendst_clr_clears_set() {
        let mut ppb = Ppb::default();
        ppb.icsr = ICSR_PENDSTSET;
        ppb.write32(0xE000_ED04, ICSR_PENDSTCLR);
        assert_eq!(ppb.read32(0xE000_ED04) & ICSR_PENDSTSET, 0);
    }

    #[test]
    fn test_icsr_set_and_clr_simultaneous_clr_wins() {
        // ARMv8-M §B3.2.4: if both SET and CLR bits are written as 1 in the
        // same store, the CLR takes effect and the exception is NOT pended.
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED04, ICSR_PENDSVSET | ICSR_PENDSVCLR);
        assert_eq!(ppb.read32(0xE000_ED04) & ICSR_PENDSVSET, 0);
    }

    #[test]
    fn test_icsr_nmipendset_w1s() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED04, ICSR_NMIPENDSET);
        assert_ne!(ppb.read32(0xE000_ED04) & ICSR_NMIPENDSET, 0);
    }

    #[test]
    fn test_exception_priority() {
        let mut ppb = Ppb::default();
        // HardFault is fixed at -1
        assert_eq!(ppb.exception_priority(3), -1);

        // Set exception 4 (MemManage) priority to 0xA0 via SHPR1
        ppb.write32(0xE000_ED18, u32::from_le_bytes([0xA0, 0, 0, 0]));
        assert_eq!(ppb.exception_priority(4), 0xA0_u8 as i16);
    }

    #[test]
    fn test_nvic_stub_returns_zero() {
        let mut ppb = Ppb::default();
        // NVIC_ISER0 at 0xE000E100
        assert_eq!(ppb.read32(0xE000_E100), 0);
    }

    #[test]
    fn test_systick_stub_returns_zero() {
        let mut ppb = Ppb::default();
        // SYST_CSR at 0xE000E010
        assert_eq!(ppb.read32(0xE000_E010), 0);
    }

    #[test]
    fn test_sau_type_returns_8() {
        let mut ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_EDD4), 8);
    }

    #[test]
    fn test_sau_ctrl_roundtrip() {
        let mut ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_EDD0), 0);
        ppb.write32(0xE000_EDD0, 1);
        assert_eq!(ppb.read32(0xE000_EDD0), 1);
    }

    #[test]
    fn test_sau_rnr_masks_to_3_bits() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EDD8, 0xFF);
        assert_eq!(ppb.read32(0xE000_EDD8), 7);
    }

    #[test]
    fn test_sau_region_roundtrip() {
        let mut ppb = Ppb::default();
        // Select region 3
        ppb.write32(0xE000_EDD8, 3);
        // Write RBAR and RLAR
        ppb.write32(0xE000_EDDC, 0x1000_4787);
        ppb.write32(0xE000_EDE0, 0x0000_7FE1);
        // Read back: RBAR has low 5 bits masked
        assert_eq!(ppb.read32(0xE000_EDDC), 0x1000_4780);
        assert_eq!(ppb.read32(0xE000_EDE0), 0x0000_7FE1);
        // Other regions remain zero
        ppb.write32(0xE000_EDD8, 0);
        assert_eq!(ppb.read32(0xE000_EDDC), 0);
        assert_eq!(ppb.read32(0xE000_EDE0), 0);
    }

    #[test]
    fn test_sau_type_write_ignored() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EDD4, 0xDEAD);
        assert_eq!(ppb.read32(0xE000_EDD4), 8);
    }

    // ----------------------------------------------------------------
    // FP extension registers (Phase 7 Stage B)
    // ----------------------------------------------------------------

    #[test]
    fn test_fpccr_reset_value() {
        let mut ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_EF34), 0xC000_0000);
        assert_eq!(ppb.fpccr & FPCCR_ASPEN, FPCCR_ASPEN);
        assert_eq!(ppb.fpccr & FPCCR_LSPEN, FPCCR_LSPEN);
        assert_eq!(ppb.fpccr & FPCCR_LSPACT, 0);
    }

    #[test]
    fn test_fpccr_roundtrip() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EF34, 0xDEAD_BEEF);
        assert_eq!(ppb.read32(0xE000_EF34), 0xDEAD_BEEF);
    }

    #[test]
    fn test_fpcar_alignment_mask() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EF38, 0x2000_1007);
        // Bits [2:0] are forced to 0.
        assert_eq!(ppb.read32(0xE000_EF38), 0x2000_1000);
    }

    #[test]
    fn test_fpdscr_roundtrip() {
        let mut ppb = Ppb::default();
        // Set AHP=1, DN=1, FZ=1, RMODE=10 (round toward -inf).
        ppb.write32(0xE000_EF3C, (1 << 26) | (1 << 25) | (1 << 24) | (0b10 << 22));
        assert_eq!(ppb.read32(0xE000_EF3C),
            (1 << 26) | (1 << 25) | (1 << 24) | (0b10 << 22));
    }

    #[test]
    fn test_sau_bootrom_region7_setup() {
        // Reproduces the bootrom's SAU setup: region 7 with
        // RBAR=0x4787, RLAR=0x7FE1 (Secure, enabled)
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EDD0, 1);  // SAU_CTRL = enable
        ppb.write32(0xE000_EDD8, 7);  // SAU_RNR = region 7
        ppb.write32(0xE000_EDDC, 0x4787); // SAU_RBAR
        ppb.write32(0xE000_EDE0, 0x7FE1); // SAU_RLAR
        // Verify readback
        assert_eq!(ppb.read32(0xE000_EDDC), 0x4780); // RBAR low 5 bits masked
        assert_eq!(ppb.read32(0xE000_EDE0), 0x7FE1);
    }

    // ----------------------------------------------------------------
    // DWT CYCCNT + DEMCR (Quantum Execution Model Stage 2)
    // ----------------------------------------------------------------

    #[test]
    fn test_dwt_ctrl_roundtrip_cyccntena() {
        let mut ppb = Ppb::default();
        // Bit 0 = CYCCNTENA. Reset is 0.
        assert_eq!(ppb.read32(0xE000_1000), 0);
        ppb.write32(0xE000_1000, 1);
        assert_eq!(ppb.read32(0xE000_1000), 1);
    }

    #[test]
    fn test_demcr_roundtrip_trcena() {
        let mut ppb = Ppb::default();
        // Bit 24 = TRCENA. Reset is 0.
        assert_eq!(ppb.read32(0xE000_EDFC), 0);
        ppb.write32(0xE000_EDFC, 1 << 24);
        assert_eq!(ppb.read32(0xE000_EDFC), 1 << 24);
    }

    #[test]
    fn test_cyccnt_read_after_write_tracks_elapsed_cycles() {
        let mut ppb = Ppb::default();
        // Enable DWT: TRCENA + CYCCNTENA
        ppb.write32(0xE000_EDFC, 1 << 24);
        ppb.write32(0xE000_1000, 1);

        // Publish core cycle count, then write CYCCNT = 1000.
        ppb.update_latest_cycles(500);
        ppb.write32(0xE000_1004, 1000);

        // Read immediately: elapsed = 0 → returns 1000.
        assert_eq!(ppb.read_cyccnt(500), 1000);

        // Advance 250 cycles and read: returns 1250.
        assert_eq!(ppb.read_cyccnt(750), 1250);
    }

    #[test]
    fn test_cyccnt_disabled_returns_stored_base() {
        let mut ppb = Ppb::default();
        // TRCENA on, CYCCNTENA off.
        ppb.write32(0xE000_EDFC, 1 << 24);
        ppb.update_latest_cycles(0);
        ppb.write32(0xE000_1004, 1234);
        // CYCCNTENA=0: read returns stored base (no live cycle contribution).
        assert_eq!(ppb.read_cyccnt(999), 1234);
    }

    #[test]
    fn test_cyccnt_trcena_gates_dwt() {
        let mut ppb = Ppb::default();
        // CYCCNTENA on but TRCENA off — DWT is off entirely.
        ppb.write32(0xE000_1000, 1);
        ppb.update_latest_cycles(0);
        ppb.write32(0xE000_1004, 42);
        assert_eq!(ppb.read_cyccnt(999), 42,
            "TRCENA=0 must gate CYCCNT reads to the stored base");
    }

    // ----------------------------------------------------------------
    // SysTick (Quantum Execution Model Stage 2)
    // ----------------------------------------------------------------

    #[test]
    fn test_systick_single_underflow() {
        let mut ppb = Ppb::default();
        // Enable, CLKSOURCE=processor, TICKINT=0. Write via CSR to exercise
        // the register path; CVR must be set via the field (a write to CVR
        // always clears it, per ARMv8-M).
        ppb.write32(0xE000_E010, 1 | (1 << 2));
        ppb.write32(0xE000_E014, 100); // RVR = 100
        ppb.syst_cvr = 50;
        ppb.last_systick_cycles = 0;

        ppb.systick_advance(51); // one underflow
        // COUNTFLAG set
        assert_ne!(ppb.syst_csr & (1 << 16), 0, "COUNTFLAG must be set on underflow");
        // After 51 decrements from 50: decrement 51 steps. The pseudocode:
        // rem=51, cvr=50: rem > cvr (51 > 50) → rem -= cvr+1 (51-51=0), cvr = RVR (100).
        // Loop: rem <= cvr (0 <= 100) → cvr -= 0 → cvr = 100.
        assert_eq!(ppb.syst_cvr, 100, "CVR reloads to RVR after exactly one underflow");
        // SysTick pending bit set? TICKINT=0, so ICSR.PENDSTSET must NOT be set.
        assert_eq!(ppb.icsr & (1 << 26), 0,
            "TICKINT=0: ICSR.PENDSTSET must remain clear");
    }

    #[test]
    fn test_systick_multi_reload() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_E010, 1 | (1 << 2)); // ENABLE + CLKSOURCE
        ppb.write32(0xE000_E014, 10); // RVR = 10
        ppb.syst_cvr = 5;
        ppb.last_systick_cycles = 0;

        // 50 cycles: rem=50, cvr=5. First pass: 50 > 5 → rem=50-6=44, cvr=10.
        // Next: 44 > 10 → rem=33, cvr=10. 33>10 → rem=22, cvr=10. 22>10 → rem=11, cvr=10.
        // 11>10 → rem=0, cvr=10. 0 <= 10 → cvr -= 0 → cvr=10. Five reloads total.
        ppb.systick_advance(50);
        assert_ne!(ppb.syst_csr & (1 << 16), 0, "COUNTFLAG must be set");
        assert_eq!(ppb.syst_cvr, 10, "CVR should be RVR after multi-reload");
    }

    #[test]
    fn test_systick_disabled_does_not_tick() {
        let mut ppb = Ppb::default();
        // ENABLE=0
        ppb.write32(0xE000_E010, 1 << 2); // CLKSOURCE only; ENABLE=0
        ppb.write32(0xE000_E014, 100);
        ppb.syst_cvr = 50;
        ppb.last_systick_cycles = 0;

        ppb.systick_advance(200); // would underflow twice if enabled
        assert_eq!(ppb.syst_cvr, 50, "CVR must not change when ENABLE=0");
        assert_eq!(ppb.syst_csr & (1 << 16), 0,
            "COUNTFLAG must not be set when ENABLE=0");
    }

    #[test]
    fn test_systick_countflag_read_clears() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_E010, 1 | (1 << 2));
        ppb.write32(0xE000_E014, 100);
        ppb.syst_cvr = 50;
        ppb.last_systick_cycles = 0;

        ppb.systick_advance(60); // underflow
        // First read of CSR: COUNTFLAG=1
        let first = ppb.read32(0xE000_E010);
        assert_ne!(first & (1 << 16), 0, "First CSR read must show COUNTFLAG=1");

        // Second read: COUNTFLAG should be cleared
        let second = ppb.read32(0xE000_E010);
        assert_eq!(second & (1 << 16), 0, "Second CSR read must show COUNTFLAG=0");
        // ENABLE/CLKSOURCE bits must still be readable
        assert_ne!(second & 1, 0, "ENABLE must remain set");
    }

    #[test]
    fn test_systick_tickint_pends_exception() {
        let mut ppb = Ppb::default();
        // ENABLE + TICKINT + CLKSOURCE
        ppb.write32(0xE000_E010, 1 | (1 << 1) | (1 << 2));
        ppb.write32(0xE000_E014, 100);
        ppb.syst_cvr = 50;
        ppb.last_systick_cycles = 0;

        ppb.systick_advance(60); // underflow
        // ICSR.PENDSTSET (bit 26) must be set by pend_systick().
        assert_ne!(ppb.icsr & (1 << 26), 0,
            "TICKINT=1 + underflow must set ICSR.PENDSTSET");
    }

    #[test]
    fn test_systick_cvr_write_clears_cvr_and_countflag() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_E010, 1 | (1 << 2));
        ppb.write32(0xE000_E014, 100);
        ppb.syst_cvr = 50;
        ppb.last_systick_cycles = 0;

        ppb.systick_advance(60); // underflow; COUNTFLAG=1
        assert_ne!(ppb.syst_csr & (1 << 16), 0);

        // Write CVR: hardware spec clears CVR and COUNTFLAG (any value).
        ppb.write32(0xE000_E018, 0x1234_5678);
        assert_eq!(ppb.syst_cvr, 0, "CVR write clears CVR");
        assert_eq!(ppb.syst_csr & (1 << 16), 0, "CVR write clears COUNTFLAG");
    }

    #[test]
    fn test_systick_cvr_masks_to_24_bits() {
        let mut ppb = Ppb::default();
        // CVR is 24-bit. A write of 0xFF_FFFF stores 0xFF_FFFF;
        // but since writes-to-CVR always clear the register per the spec,
        // the value stored is 0, not the input. Instead, check RVR.
        ppb.write32(0xE000_E014, 0xFFFF_FFFF); // RVR
        assert_eq!(ppb.read32(0xE000_E014), 0x00FF_FFFF,
            "RVR read must be masked to 24 bits");
    }
}
