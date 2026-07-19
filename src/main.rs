#![allow(warnings)]
#![no_std]
#![no_main]

use crate::{
    arch::trap::{self, InterruptId},
    drivers::ws2812::RgbLed,
    hal::{gpio::GpioPin, watchdog::feed_watchdog},
};
use core::{arch::global_asm, panic::PanicInfo};

mod arch;
mod drivers;
mod hal;

global_asm!(include_str!("boot.s"));

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    error!("PANIC: {}", _info);
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    loop {
        feed_watchdog();
        led.refresh((5, 0, 0)); // RED = Panic
    }
}

fn on_machine_timer() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((0, 5, 0)); // GREEN
    let raw = hal::tsens::read_raw();
    info!("[tick] watchdog fed, tsens raw = {}", raw);
}

fn on_other_interrupt() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 5, 0)); // YELLOW
    warn!("[irq] unexpected local interrupt fired");
}

fn on_exception(mcause: usize) {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 0, 5)); // PURPLE
    error!("[exception] mcause = 0x{:08x}", mcause);
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    hal::watchdog::disable_lp_watchdog();
    // hal::watchdog::disable_hp_watchdog();
    hal::timer::systimer_enable();

    hal::uart::init(115200);
    info!("--- project-trobbio booting ---");

    hal::tsens::init();

    trap::register(InterruptId::MachineTimer, on_machine_timer);
    trap::register(InterruptId::UserSoftware, on_other_interrupt);
    trap::register(InterruptId::MachineSoftware, on_other_interrupt);
    trap::register(InterruptId::UserTimer, on_other_interrupt);
    trap::set_exception_handler(on_exception);

    // CLINT runs at 16 MHz, so 16_000_000 ticks == 1 second between fires.
    trap::init_periodic_timer(16_000_000);

    info!("boot complete, entering idle loop");

    // UART1 on GPIO10 (TX) / GPIO11 (RX)
    // Also hardware-tested with an external TX->RX jumper, clean and repeatable.
    hal::uart1::init(10, 11, 115200);
    hal::uart1::set_loopback(true); // if disabled, the 2 physical pins must be connected via
    // jumpers

    loop {
        hal::uart1::write_bytes(b"test").ok();
        hal::timer::delay_ms(100);
        while let Some(b) = hal::uart1::read_byte() {
            info!("Loopback success: {}", b as char);
        }
        hal::timer::delay_ms(1000);
    }
}
