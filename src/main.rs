use mdrp2354::{Config, EmulatorBuilder};

fn main() {
    let emu = EmulatorBuilder::new(Config::default()).build();
    println!(
        "mdrp2354: RP2354 emulator initialised — {} Hz, {} cores",
        emu.clock.sys_clk_hz,
        emu.cores.len()
    );
}
