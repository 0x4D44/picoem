use mdpicoem_devices::LcdState;

#[derive(Clone, Default)]
pub struct Snapshot {
    pub cycles: u64,
    pub wall_ms: u64,
    pub effective_mhz: f64,
    pub pc: u32,

    pub gpio_out: u32,
    pub gpio_oe: u32,

    pub lcd: LcdState,
    pub benchmark: Option<BenchmarkReport>,
}

#[derive(Clone)]
pub struct BenchmarkReport {
    pub sections: Vec<BenchmarkSection>,
    pub complete: bool,
    pub stall: Option<u32>,
}

#[derive(Clone)]
pub struct BenchmarkSection {
    pub name: &'static str,
    pub emu_cycles: u64,
    pub ref_cycles: u64,
    pub iterations: u32,
}
