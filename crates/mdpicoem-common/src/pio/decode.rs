/// Decoded PIO instruction.
pub struct DecodedInsn {
    pub op: PioOp,
    pub delay: u8,
    pub sideset: Option<u8>,
}

/// PIO instruction opcodes.
pub enum PioOp {
    Jmp { condition: u8, address: u8 },
    Wait { polarity: bool, source: u8, index: u8 },
    In { source: u8, bit_count: u8 },
    Out { destination: u8, bit_count: u8 },
    Push { if_full: bool, block: bool },
    Pull { if_empty: bool, block: bool },
    Mov { destination: u8, op: u8, source: u8 },
    Irq { clear: bool, wait: bool, index: u8 },
    Set { destination: u8, data: u8 },
}

/// Decode a 16-bit PIO instruction.
///
/// `pinctrl` and `execctrl` are needed to determine the side-set/delay split
/// and SIDE_EN behavior.
pub fn decode(insn: u16, pinctrl: u32, execctrl: u32) -> DecodedInsn {
    let opcode = (insn >> 13) & 0x7;
    let delay_sideset = ((insn >> 8) & 0x1F) as u8;
    let operand = (insn & 0xFF) as u8;

    // Side-set / delay split
    let sideset_count = (((pinctrl >> 29) & 7) as u8).min(5);
    let delay_bits = 5 - sideset_count;
    let side_en = (execctrl >> 30) & 1 != 0;

    let (sideset, delay) = if sideset_count == 0 {
        // No side-set, all 5 bits are delay
        (None, delay_sideset)
    } else {
        // Side-set occupies the TOP bits, delay the BOTTOM
        let delay_mask = (1u8 << delay_bits) - 1;
        let delay = delay_sideset & delay_mask;
        let ss_raw = delay_sideset >> delay_bits;

        let sideset = if side_en {
            // MSB of side-set field is enable bit
            let enable = (ss_raw >> (sideset_count - 1)) & 1 != 0;
            if enable {
                // Actual side-set value is remaining bits below enable
                let ss_val_bits = sideset_count - 1;
                let ss_val = ss_raw & ((1u8 << ss_val_bits) - 1);
                Some(ss_val)
            } else {
                None
            }
        } else {
            Some(ss_raw)
        };

        (sideset, delay)
    };

    let op = match opcode {
        // 000: JMP
        0 => PioOp::Jmp {
            condition: (operand >> 5) & 0x7,
            address: operand & 0x1F,
        },
        // 001: WAIT
        1 => PioOp::Wait {
            polarity: (operand >> 7) & 1 != 0,
            source: (operand >> 5) & 0x3,
            index: operand & 0x1F,
        },
        // 010: IN
        2 => {
            let bit_count = operand & 0x1F;
            PioOp::In {
                source: (operand >> 5) & 0x7,
                bit_count: if bit_count == 0 { 32 } else { bit_count },
            }
        }
        // 011: OUT
        3 => {
            let bit_count = operand & 0x1F;
            PioOp::Out {
                destination: (operand >> 5) & 0x7,
                bit_count: if bit_count == 0 { 32 } else { bit_count },
            }
        }
        // 100: PUSH/PULL — direction=bit7
        4 => {
            let direction = (operand >> 7) & 1 != 0;
            let if_x = (operand >> 6) & 1 != 0;
            let block = (operand >> 5) & 1 != 0;
            if direction {
                PioOp::Pull { if_empty: if_x, block }
            } else {
                PioOp::Push { if_full: if_x, block }
            }
        }
        // 101: MOV
        5 => PioOp::Mov {
            destination: (operand >> 5) & 0x7,
            op: (operand >> 3) & 0x3,
            source: operand & 0x7,
        },
        // 110: IRQ
        6 => PioOp::Irq {
            clear: (operand >> 6) & 1 != 0,
            wait: (operand >> 5) & 1 != 0,
            index: operand & 0x1F,
        },
        // 111: SET
        _ => PioOp::Set {
            destination: (operand >> 5) & 0x7,
            data: operand & 0x1F,
        },
    };

    DecodedInsn { op, delay, sideset }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SIDE_EN=1 with the enable bit set selects side-set (covers line 44
    /// `side_en=true` arm and line 47 `enable=true` arm).
    #[test]
    fn side_en_enable_bit_set_yields_sideset() {
        // PINCTRL: SIDESET_COUNT=2 (bits[31:29]=010).
        let pinctrl = 2u32 << 29;
        // EXECCTRL: SIDE_EN=1 (bit 30).
        let execctrl = 1u32 << 30;
        // SIDESET_COUNT=2, SIDE_EN=1 → top bit of the 2-bit side-set
        // field is the enable; with enable=1 there's 1 value bit below.
        // delay_bits = 5-2 = 3. Field layout: [enable(1) ss(1) d(3)].
        // Pick enable=1, ss=1, delay=5 → [1 1 1 0 1] = 0b11101 = 29.
        // SET X, 3 (opcode=111, dest=001, data=00011).
        // insn = 0b111_11101_001_00011 = 0xFD23.
        let d = decode(0xFD23, pinctrl, execctrl);
        assert_eq!(d.delay, 5, "delay is the bottom 3 bits");
        assert_eq!(d.sideset, Some(1), "side-set value bit = 1");
    }

    /// SIDE_EN=1 with the enable bit cleared yields `None` (covers the
    /// `enable=false` arm following line 47).
    #[test]
    fn side_en_enable_bit_clear_yields_none() {
        let pinctrl = 2u32 << 29;
        let execctrl = 1u32 << 30; // SIDE_EN=1
        // Field: [enable(1) ss(1) d(3)] = [0 1 0 1 0] = 0b01010 = 10 → delay=2, ss_raw=01.
        // SET X, 0.
        // insn = 0b111_01010_001_00000 = 0xEA20.
        let d = decode(0xEA20, pinctrl, execctrl);
        assert_eq!(d.delay, 2);
        assert!(d.sideset.is_none(), "enable bit clear must yield None");
    }

    /// IN with bit_count field = 0 decodes as bit_count=32 (covers
    /// line 79 `bit_count == 0` branch).
    #[test]
    fn in_with_bc_zero_decodes_as_32() {
        // IN X, 32: opcode=010, src=001(X), bit_count=00000 → 0x4020.
        let d = decode(0x4020, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::In { source, bit_count } => {
                assert_eq!(source, 1);
                assert_eq!(bit_count, 32, "bc field=0 decodes as 32");
            }
            _ => panic!("expected IN"),
        }
    }

    /// OUT with bit_count field = 0 decodes as bit_count=32 (covers
    /// line 87 `bit_count == 0` branch).
    #[test]
    fn out_with_bc_zero_decodes_as_32() {
        // OUT PINS, 32: opcode=011, dest=000(PINS), bit_count=00000 → 0x6000.
        let d = decode(0x6000, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::Out { destination, bit_count } => {
                assert_eq!(destination, 0);
                assert_eq!(bit_count, 32);
            }
            _ => panic!("expected OUT"),
        }
    }

    /// PUSH/PULL opcode (100) with direction=1 decodes as PULL (covers
    /// line 95 `direction=true` arm).
    #[test]
    fn pull_decodes_from_direction_bit() {
        // PULL block: opcode=100, dir=1, if_empty=0, block=1 → 0x80A0.
        let d = decode(0x80A0, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::Pull { if_empty, block } => {
                assert!(!if_empty);
                assert!(block);
            }
            _ => panic!("expected PULL"),
        }
    }

    /// MOV and IRQ opcodes round-trip through decode (covers the MOV and
    /// IRQ arms of the opcode match — belt-and-braces for line 108
    /// style branches not explicitly listed but adjacent).
    #[test]
    fn mov_and_irq_opcodes_decode() {
        // MOV Y, ~X: opcode=101, dst=010, op=01, src=001 → 0xA049.
        let d = decode(0xA049, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::Mov { destination, op, source } => {
                assert_eq!(destination, 2);
                assert_eq!(op, 1);
                assert_eq!(source, 1);
            }
            _ => panic!("expected MOV"),
        }
        // IRQ clear 5 wait: opcode=110, clear=1, wait=1, index=00101 → 0xC065.
        let d = decode(0xC065, 0x1400_0000, 0x0001_F000);
        match d.op {
            PioOp::Irq { clear, wait, index } => {
                assert!(clear);
                assert!(wait);
                assert_eq!(index, 5);
            }
            _ => panic!("expected IRQ"),
        }
    }
}
