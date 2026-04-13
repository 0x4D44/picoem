pub mod decode;
pub mod fifo;
pub mod sm;

use sm::StateMachine;

/// One PIO block (RP2350 has three: PIO0, PIO1, PIO2).
pub struct PioBlock {
    pub(crate) sm: [StateMachine; 4],
    pub(crate) instr_mem: [u16; 32],
    pub(crate) irq_flags: u8,
    input_sync_bypass: u32,
    fdebug: u32,
    pub pad_out: u32,
    pub pad_oe: u32,
}

impl PioBlock {
    pub fn new() -> Self {
        Self {
            sm: [
                StateMachine::new(),
                StateMachine::new(),
                StateMachine::new(),
                StateMachine::new(),
            ],
            instr_mem: [0; 32],
            irq_flags: 0,
            input_sync_bypass: 0,
            fdebug: 0,
            pad_out: 0,
            pad_oe: 0,
        }
    }

    /// Reset to power-on defaults.
    pub fn reset(&mut self) {
        for sm in &mut self.sm {
            sm.reset();
        }
        self.instr_mem = [0; 32];
        self.irq_flags = 0;
        self.input_sync_bypass = 0;
        self.fdebug = 0;
        self.pad_out = 0;
        self.pad_oe = 0;
    }

    /// Placeholder: advance PIO block by one system clock.
    pub fn step(&mut self, _gpio_in: u32) {
        // Stage B will implement clock divider ticking + instruction execution.
    }

    /// Compute FSTAT register from current SM FIFO states.
    fn fstat(&self) -> u32 {
        let mut val = 0u32;
        for i in 0..4 {
            if self.sm[i].tx_fifo.is_empty() {
                val |= 1 << (24 + i); // TXEMPTY
            }
            if self.sm[i].tx_fifo.is_full() {
                val |= 1 << (16 + i); // TXFULL
            }
            if self.sm[i].rx_fifo.is_empty() {
                val |= 1 << (8 + i); // RXEMPTY
            }
            if self.sm[i].rx_fifo.is_full() {
                val |= 1 << i; // RXFULL
            }
        }
        val
    }

    /// Compute FLEVEL register from current SM FIFO levels.
    fn flevel(&self) -> u32 {
        let mut val = 0u32;
        for i in 0..4 {
            let tx = self.sm[i].tx_fifo.level() as u32;
            let rx = self.sm[i].rx_fifo.level() as u32;
            val |= (tx & 0xF) << (i * 8);
            val |= (rx & 0xF) << (i * 8 + 4);
        }
        val
    }

    /// Apply FIFO joining based on SHIFTCTRL bits for a given SM.
    fn apply_fifo_join(&mut self, sm_idx: usize) {
        let shiftctrl = self.sm[sm_idx].shiftctrl;
        let fjoin_tx = (shiftctrl >> 30) & 1 != 0;
        let fjoin_rx = (shiftctrl >> 31) & 1 != 0;

        if fjoin_tx {
            self.sm[sm_idx].tx_fifo.set_depth(8);
            self.sm[sm_idx].rx_fifo.set_depth(0);
        } else if fjoin_rx {
            self.sm[sm_idx].tx_fifo.set_depth(0);
            self.sm[sm_idx].rx_fifo.set_depth(8);
        } else {
            self.sm[sm_idx].tx_fifo.set_depth(4);
            self.sm[sm_idx].rx_fifo.set_depth(4);
        }
    }

    /// 32-bit register read. `offset` is masked to 12 bits by Bus.
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            // CTRL: only SM_ENABLE bits are readable (restart bits are self-clearing)
            0x000 => {
                let mut val = 0u32;
                for i in 0..4 {
                    if self.sm[i].enabled {
                        val |= 1 << i;
                    }
                }
                val
            }
            0x004 => self.fstat(),
            0x008 => self.fdebug,
            0x00C => self.flevel(),
            // TXF0-3: write-only, reads return 0
            0x010..=0x01C => 0,
            // RXF0-3: pop from SM's RX FIFO
            0x020 => self.sm[0].rx_fifo.pop().unwrap_or(0),
            0x024 => self.sm[1].rx_fifo.pop().unwrap_or(0),
            0x028 => self.sm[2].rx_fifo.pop().unwrap_or(0),
            0x02C => self.sm[3].rx_fifo.pop().unwrap_or(0),
            // IRQ
            0x030 => self.irq_flags as u32,
            // IRQ_FORCE: write-only
            0x034 => 0,
            // INPUT_SYNC_BYPASS
            0x038 => self.input_sync_bypass,
            // DBG_PADOUT
            0x03C => self.pad_out,
            // DBG_PADOE
            0x040 => self.pad_oe,
            // DBG_CFGINFO: 32 IMEM words, 4 SMs, 4 FIFO depth
            0x044 => 0x0020_0404,
            // INSTR_MEM0-31: write-only
            0x048..=0x0C4 => 0,
            // Per-SM registers (stride 0x18, SM0 at 0x0C8)
            0x0C8..=0x127 => self.read_sm_reg(offset),
            _ => 0,
        }
    }

    /// 32-bit register write. `offset` is masked to 12 bits by Bus.
    /// `alias`: 0=normal, 1=XOR, 2=SET (OR), 3=CLR (AND NOT).
    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        match offset {
            0x000 => self.write_ctrl(val, alias),
            // FSTAT: read-only
            0x004 => {}
            // FDEBUG: W1C (or alias)
            0x008 => {
                let mask = match alias {
                    0 | 3 => val,  // normal write and CLR both clear bits
                    1 => val,      // XOR
                    2 => val,      // SET
                    _ => return,
                };
                match alias {
                    0 => self.fdebug &= !mask,  // W1C: writing 1 clears
                    1 => self.fdebug ^= mask,
                    2 => self.fdebug |= mask,
                    3 => self.fdebug &= !mask,
                    _ => {}
                }
            }
            // FLEVEL: read-only
            0x00C => {}
            // TXF0-3: push to SM's TX FIFO
            0x010 => { self.sm[0].tx_fifo.push(val); }
            0x014 => { self.sm[1].tx_fifo.push(val); }
            0x018 => { self.sm[2].tx_fifo.push(val); }
            0x01C => { self.sm[3].tx_fifo.push(val); }
            // RXF0-3: read-only
            0x020..=0x02C => {}
            // IRQ: W1C (or alias)
            0x030 => {
                match alias {
                    0 => self.irq_flags &= !(val as u8),  // W1C
                    1 => self.irq_flags ^= val as u8,
                    2 => self.irq_flags |= val as u8,
                    3 => self.irq_flags &= !(val as u8),
                    _ => {}
                }
            }
            // IRQ_FORCE: set bits in irq_flags
            0x034 => {
                self.irq_flags |= val as u8;
            }
            // INPUT_SYNC_BYPASS
            0x038 => { self.input_sync_bypass = val; }
            // DBG_PADOUT, DBG_PADOE, DBG_CFGINFO: read-only
            0x03C..=0x044 => {}
            // INSTR_MEM0-31
            0x048..=0x0C4 => {
                let idx = ((offset - 0x048) >> 2) as usize;
                if idx < 32 {
                    self.instr_mem[idx] = val as u16;
                }
            }
            // Per-SM registers
            0x0C8..=0x127 => self.write_sm_reg(offset, val),
            _ => {}
        }
    }

    /// Read per-SM register.
    fn read_sm_reg(&self, offset: u32) -> u32 {
        let sm_offset = offset - 0x0C8;
        let sm_idx = (sm_offset / 0x18) as usize;
        let reg = sm_offset % 0x18;
        if sm_idx >= 4 {
            return 0;
        }
        let sm = &self.sm[sm_idx];
        match reg {
            // SMn_CLKDIV
            0x00 => sm.read_clkdiv(),
            // SMn_EXECCTRL: bit 31 is EXEC_STALLED (read-only)
            0x04 => {
                let stalled = sm.stalled || sm.delay_count > 0;
                (sm.execctrl & 0x7FFF_FFFF) | ((stalled as u32) << 31)
            }
            // SMn_SHIFTCTRL
            0x08 => sm.shiftctrl,
            // SMn_ADDR: current PC
            0x0C => sm.pc as u32,
            // SMn_INSTR: last executed instruction
            0x10 => sm.last_insn as u32,
            // SMn_PINCTRL
            0x14 => sm.pinctrl,
            _ => 0,
        }
    }

    /// Write per-SM register.
    fn write_sm_reg(&mut self, offset: u32, val: u32) {
        let sm_offset = offset - 0x0C8;
        let sm_idx = (sm_offset / 0x18) as usize;
        let reg = sm_offset % 0x18;
        if sm_idx >= 4 {
            return;
        }
        match reg {
            // SMn_CLKDIV
            0x00 => self.sm[sm_idx].write_clkdiv(val),
            // SMn_EXECCTRL: bit 31 is read-only (EXEC_STALLED)
            0x04 => self.sm[sm_idx].execctrl = val & 0x7FFF_FFFF,
            // SMn_SHIFTCTRL: reconfigure FIFO joining when changed
            0x08 => {
                let old_join = self.sm[sm_idx].shiftctrl & 0xC000_0000;
                self.sm[sm_idx].shiftctrl = val;
                let new_join = val & 0xC000_0000;
                if old_join != new_join {
                    self.apply_fifo_join(sm_idx);
                }
            }
            // SMn_ADDR: read-only
            0x0C => {}
            // SMn_INSTR: force-execute (placeholder — Stage B)
            0x10 => {
                self.sm[sm_idx].last_insn = val as u16;
            }
            // SMn_PINCTRL
            0x14 => self.sm[sm_idx].pinctrl = val,
            _ => {}
        }
    }

    /// Write CTRL register with alias support.
    fn write_ctrl(&mut self, val: u32, alias: u32) {
        let sm_enable_bits = val & 0xF;
        let sm_restart_bits = (val >> 4) & 0xF;
        let clkdiv_restart_bits = (val >> 8) & 0xF;

        // SM_ENABLE: apply alias logic
        match alias {
            0 => {
                // Normal write: set SM_ENABLE directly
                for i in 0..4 {
                    self.sm[i].enabled = (sm_enable_bits >> i) & 1 != 0;
                }
            }
            1 => {
                // XOR
                for i in 0..4 {
                    if (sm_enable_bits >> i) & 1 != 0 {
                        self.sm[i].enabled = !self.sm[i].enabled;
                    }
                }
            }
            2 => {
                // SET (OR): enable indicated SMs
                for i in 0..4 {
                    if (sm_enable_bits >> i) & 1 != 0 {
                        self.sm[i].enabled = true;
                    }
                }
            }
            3 => {
                // CLR (AND NOT): disable indicated SMs
                for i in 0..4 {
                    if (sm_enable_bits >> i) & 1 != 0 {
                        self.sm[i].enabled = false;
                    }
                }
            }
            _ => {}
        }

        // SM_RESTART: self-clearing action (reset SM state)
        for i in 0..4 {
            if (sm_restart_bits >> i) & 1 != 0 {
                self.sm[i].pc = 0;
                self.sm[i].x = 0;
                self.sm[i].y = 0;
                self.sm[i].isr = 0;
                self.sm[i].osr = 0;
                self.sm[i].isr_count = 0;
                self.sm[i].osr_count = 0;
                self.sm[i].delay_count = 0;
                self.sm[i].stalled = false;
            }
        }

        // CLKDIV_RESTART: self-clearing action (reset clock divider accumulator)
        for i in 0..4 {
            if (clkdiv_restart_bits >> i) & 1 != 0 {
                self.sm[i].clkdiv_acc = 0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sm_reset_values() {
        let sm = StateMachine::new();
        assert_eq!(sm.execctrl, 0x0001_F000, "EXECCTRL reset: wrap_top=31");
        assert_eq!(sm.shiftctrl, 0x000C_0000, "SHIFTCTRL reset: thresholds=0 (32)");
        assert_eq!(sm.pinctrl, 0x1400_0000, "PINCTRL reset: SET_COUNT=5");
        assert_eq!(sm.clkdiv_int, 1, "CLKDIV int reset: 1");
        assert_eq!(sm.clkdiv_frac, 0, "CLKDIV frac reset: 0");
        assert_eq!(sm.read_clkdiv(), 0x0001_0000, "CLKDIV register: 0x0001_0000");
    }

    #[test]
    fn test_register_roundtrip_clkdiv() {
        let mut pio = PioBlock::new();
        // Write CLKDIV for SM0: int=1302, frac=128
        let clkdiv_val = (1302u32 << 16) | (128u32 << 8);
        pio.write32(0x0C8, clkdiv_val, 0); // SM0 CLKDIV
        assert_eq!(pio.read32(0x0C8), clkdiv_val);
    }

    #[test]
    fn test_register_roundtrip_execctrl() {
        let mut pio = PioBlock::new();
        // Write EXECCTRL for SM0 with bit 31 set — bit 31 should be masked (read-only)
        pio.write32(0x0CC, 0xFFFF_FFFF, 0); // SM0 EXECCTRL
        let read_back = pio.read32(0x0CC);
        // Bit 31 is EXEC_STALLED (read-only), reflects sm.stalled || delay > 0
        // SM is not stalled and delay_count=0, so bit 31 should be 0
        assert_eq!(read_back & 0x8000_0000, 0, "bit 31 is read-only EXEC_STALLED");
        assert_eq!(read_back & 0x7FFF_FFFF, 0x7FFF_FFFF, "bits 30:0 roundtrip");
    }

    #[test]
    fn test_register_roundtrip_shiftctrl() {
        let mut pio = PioBlock::new();
        let val = 0xDEAD_BEEF;
        pio.write32(0x0D0, val, 0); // SM0 SHIFTCTRL
        assert_eq!(pio.read32(0x0D0), val);
    }

    #[test]
    fn test_register_roundtrip_pinctrl() {
        let mut pio = PioBlock::new();
        let val = 0xABCD_1234;
        pio.write32(0x0DC, val, 0); // SM0 PINCTRL
        assert_eq!(pio.read32(0x0DC), val);
    }

    #[test]
    fn test_ctrl_enable_disable() {
        let mut pio = PioBlock::new();
        // Enable SM0
        pio.write32(0x000, 0x1, 0);
        assert!(pio.sm[0].enabled);
        assert!(!pio.sm[1].enabled);
        // Read back CTRL: only SM_ENABLE bits
        assert_eq!(pio.read32(0x000), 0x1);

        // Disable SM0
        pio.write32(0x000, 0x0, 0);
        assert!(!pio.sm[0].enabled);
        assert_eq!(pio.read32(0x000), 0x0);

        // Enable SM0 and SM2
        pio.write32(0x000, 0x5, 0);
        assert!(pio.sm[0].enabled);
        assert!(!pio.sm[1].enabled);
        assert!(pio.sm[2].enabled);
        assert!(!pio.sm[3].enabled);
    }

    #[test]
    fn test_ctrl_restart_self_clearing() {
        let mut pio = PioBlock::new();
        // Enable SM0 and set some state
        pio.sm[0].enabled = true;
        pio.sm[0].pc = 15;
        pio.sm[0].x = 0x1234;

        // Write SM_RESTART for SM0 (bit 4) + keep SM0 enabled (bit 0)
        pio.write32(0x000, 0x11, 0);

        // SM0 should be enabled (bit 0 written)
        assert!(pio.sm[0].enabled);
        // SM0 state should be reset by restart
        assert_eq!(pio.sm[0].pc, 0);
        assert_eq!(pio.sm[0].x, 0);

        // Read back CTRL: restart bits are self-clearing, should read 0
        let ctrl = pio.read32(0x000);
        assert_eq!(ctrl & 0xF0, 0, "SM_RESTART bits read as 0");
        assert_eq!(ctrl & 0xF, 0x1, "SM_ENABLE bits persist");
    }

    #[test]
    fn test_instr_mem_write() {
        let mut pio = PioBlock::new();
        for i in 0..32u32 {
            pio.write32(0x048 + i * 4, 0xA000 + i, 0);
        }
        for i in 0..32 {
            assert_eq!(pio.instr_mem[i], 0xA000 + i as u16);
        }
    }

    #[test]
    fn test_fifo_push_pop() {
        let mut pio = PioBlock::new();
        // Push via TXF0
        pio.write32(0x010, 0xDEAD_BEEF, 0);

        // FSTAT: TX should not be empty for SM0
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & (1 << 24), 0, "SM0 TX not empty");

        // Pop from RXF0 — but wait, TXF pushes to TX FIFO, RXF pops from RX FIFO.
        // In the real PIO, data flows TX -> SM -> RX. For register-level testing,
        // push to TX and verify TX FIFO state, then manually push to RX and pop.
        // Let's verify TX state via FSTAT, then directly push to RX for pop test.
        pio.sm[0].rx_fifo.push(0xCAFE_BABE);
        let val = pio.read32(0x020);
        assert_eq!(val, 0xCAFE_BABE);
    }

    #[test]
    fn test_fifo_full_and_overflow() {
        let mut pio = PioBlock::new();
        // Push 4 values to SM0 TX FIFO
        for i in 0..4 {
            assert!(pio.sm[0].tx_fifo.push(i + 1));
        }
        assert!(pio.sm[0].tx_fifo.is_full());

        // 5th push should fail
        assert!(!pio.sm[0].tx_fifo.push(5));

        // FSTAT: TXFULL bit for SM0
        let fstat = pio.read32(0x004);
        assert_ne!(fstat & (1 << 16), 0, "SM0 TX full");
    }

    #[test]
    fn test_fifo_joining() {
        let mut pio = PioBlock::new();

        // Set FJOIN_TX in SHIFTCTRL for SM0 (bit 30)
        pio.write32(0x0D0, pio.sm[0].shiftctrl | (1 << 30), 0);

        // TX FIFO should now accept 8 values
        for i in 0..8 {
            assert!(pio.sm[0].tx_fifo.push(i + 1), "push {} should succeed", i);
        }
        assert!(pio.sm[0].tx_fifo.is_full(), "TX FIFO full at 8");
        assert!(!pio.sm[0].tx_fifo.push(9), "push 9 should fail");

        // RX FIFO should be depth 0 (unavailable): pop returns None
        assert_eq!(pio.sm[0].rx_fifo.pop(), None);
    }

    #[test]
    fn test_fstat_flags() {
        let mut pio = PioBlock::new();

        // Initially: TX empty, RX empty for all SMs
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & 0x0F00_0000, 0x0F00_0000, "all TX empty");
        assert_eq!(fstat & 0x0000_0F00, 0x0000_0F00, "all RX empty");
        assert_eq!(fstat & 0x000F_0000, 0, "no TX full");
        assert_eq!(fstat & 0x0000_000F, 0, "no RX full");

        // Push one value to SM0 TX
        pio.write32(0x010, 42, 0);
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & (1 << 24), 0, "SM0 TX not empty");
        assert_ne!(fstat & (1 << 25), 0, "SM1 TX still empty");

        // Fill SM1 TX FIFO
        for _ in 0..4 {
            pio.sm[1].tx_fifo.push(0);
        }
        let fstat = pio.read32(0x004);
        assert_ne!(fstat & (1 << 17), 0, "SM1 TX full");

        // Push to SM2 RX FIFO
        pio.sm[2].rx_fifo.push(0);
        let fstat = pio.read32(0x004);
        assert_eq!(fstat & (1 << 10), 0, "SM2 RX not empty");
    }

    #[test]
    fn test_flevel() {
        let mut pio = PioBlock::new();

        // Push 2 to SM0 TX, 3 to SM1 RX
        pio.sm[0].tx_fifo.push(1);
        pio.sm[0].tx_fifo.push(2);
        pio.sm[1].rx_fifo.push(10);
        pio.sm[1].rx_fifo.push(20);
        pio.sm[1].rx_fifo.push(30);

        let flevel = pio.read32(0x00C);
        // SM0 TX level = 2 at bits [3:0]
        assert_eq!(flevel & 0xF, 2);
        // SM0 RX level = 0 at bits [7:4]
        assert_eq!((flevel >> 4) & 0xF, 0);
        // SM1 TX level = 0 at bits [11:8]
        assert_eq!((flevel >> 8) & 0xF, 0);
        // SM1 RX level = 3 at bits [15:12]
        assert_eq!((flevel >> 12) & 0xF, 3);
    }

    #[test]
    fn test_irq_force_and_w1c() {
        let mut pio = PioBlock::new();

        // Force IRQ bits 0, 2, 5
        pio.write32(0x034, 0x25, 0);
        assert_eq!(pio.irq_flags, 0x25);
        assert_eq!(pio.read32(0x030), 0x25);

        // W1C: clear bit 2 by writing 1 to bit 2
        pio.write32(0x030, 0x04, 0);
        assert_eq!(pio.irq_flags, 0x21);
        assert_eq!(pio.read32(0x030), 0x21);

        // Clear remaining
        pio.write32(0x030, 0x21, 0);
        assert_eq!(pio.irq_flags, 0);
    }

    #[test]
    fn test_dbg_cfginfo() {
        let mut pio = PioBlock::new();
        assert_eq!(pio.read32(0x044), 0x0020_0404);
    }

    #[test]
    fn test_bus_dispatch_pio0() {
        use crate::bus::Bus;

        let mut bus = Bus::new();

        // Write SM0 PINCTRL via PIO0 base address
        bus.write32(0x5020_00DC, 0x1234_5678);

        // Read back
        let val = bus.read32(0x5020_00DC);
        assert_eq!(val, 0x1234_5678);
    }

    #[test]
    fn test_bus_dispatch_pio1_pio2() {
        use crate::bus::Bus;

        let mut bus = Bus::new();

        // PIO1: write SM1 CLKDIV (SM1 offset = 0x0E0)
        let clkdiv = (500u32 << 16) | (64u32 << 8);
        bus.write32(0x5030_00E0, clkdiv);
        assert_eq!(bus.read32(0x5030_00E0), clkdiv);

        // PIO2: write CTRL to enable SM3
        bus.write32(0x5040_0000, 0x8);
        assert_eq!(bus.read32(0x5040_0000), 0x8);
        assert!(bus.pio[2].sm[3].enabled);
    }

    #[test]
    fn test_ctrl_alias_set_clr() {
        use crate::bus::Bus;

        let mut bus = Bus::new();

        // SET alias: addr + 0x2000 (alias=2)
        // Enable SM0 via SET alias
        bus.write32(0x5020_2000, 0x1); // SET alias on CTRL
        assert!(bus.pio[0].sm[0].enabled);
        assert_eq!(bus.read32(0x5020_0000), 0x1);

        // Enable SM2 via SET alias (SM0 should remain enabled)
        bus.write32(0x5020_2000, 0x4);
        assert!(bus.pio[0].sm[0].enabled);
        assert!(bus.pio[0].sm[2].enabled);
        assert_eq!(bus.read32(0x5020_0000), 0x5);

        // CLR alias: addr + 0x3000 (alias=3)
        // Disable SM0 via CLR alias
        bus.write32(0x5020_3000, 0x1);
        assert!(!bus.pio[0].sm[0].enabled);
        assert!(bus.pio[0].sm[2].enabled);
        assert_eq!(bus.read32(0x5020_0000), 0x4);
    }

    #[test]
    fn test_pio_reset() {
        let mut pio = PioBlock::new();

        // Dirty up state
        pio.sm[0].enabled = true;
        pio.sm[0].pc = 10;
        pio.sm[0].x = 0xDEAD;
        pio.sm[1].tx_fifo.push(42);
        pio.instr_mem[5] = 0xFFFF;
        pio.irq_flags = 0xFF;
        pio.fdebug = 0x1234;
        pio.pad_out = 0xABCD;

        pio.reset();

        assert!(!pio.sm[0].enabled);
        assert_eq!(pio.sm[0].pc, 0);
        assert_eq!(pio.sm[0].x, 0);
        assert_eq!(pio.sm[0].execctrl, 0x0001_F000, "reset restores default EXECCTRL");
        assert!(pio.sm[1].tx_fifo.is_empty());
        assert_eq!(pio.instr_mem[5], 0);
        assert_eq!(pio.irq_flags, 0);
        assert_eq!(pio.fdebug, 0);
        assert_eq!(pio.pad_out, 0);
    }

    #[test]
    fn test_gpio_in_moved_to_bus() {
        use crate::bus::Bus;

        let mut bus = Bus::new();
        bus.gpio_in = 0xFF;

        // Read SIO GPIO_IN via bus at 0xD000_0004
        let val = bus.read32(0xD000_0004);
        assert_eq!(val, 0xFF);
    }
}
