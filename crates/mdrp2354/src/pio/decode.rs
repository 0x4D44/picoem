/// Decoded PIO instruction.
#[allow(dead_code)]  // Stage B: decoder implementation
pub struct DecodedInsn {
    pub op: PioOp,
    pub delay: u8,
    pub sideset: Option<u8>,
}

/// PIO instruction opcodes.
#[allow(dead_code)]  // Stage B: decoder implementation
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
