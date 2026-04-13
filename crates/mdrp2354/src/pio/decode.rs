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
