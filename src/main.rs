use mdrp2350::{Config, EmulatorBuilder};

fn main() {
    let emu = EmulatorBuilder::new(Config::default()).build();
    println!(
        "mdrp2350: RP2350 emulator initialised — {} Hz, {} cores",
        emu.bus.sys_clk_hz(),
        emu.cores.len()
    );
}
