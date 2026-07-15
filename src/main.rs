#![allow(warnings)]
#![no_std]
#![no_main]

use crate::{
    arch::trap::{self, InterruptId},
    drivers::ws2812::RgbLed,
    hal::{gpio::GpioPin, timer, uart, watchdog::feed_watchdog},
};
use core::fmt::Write as _;
use core::{arch::global_asm, panic::PanicInfo};

mod arch;
mod drivers;
mod hal;

global_asm!(include_str!("boot.s"));

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    writeln!(uart::Uart0, "PANIC: {}", _info).ok();
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    loop {
        feed_watchdog();
        led.refresh((5, 0, 0)); // RED = Panic
    }
}

/// Every machine-timer tick, right after the kernel's own (unconditional)
/// watchdog feed. Flips the LED green and prints, so both are visible proof
/// the tick fired.
fn on_machine_timer() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((0, 5, 0)); // GREEN
    writeln!(uart::Uart0, "[tick] machine timer fired, watchdog fed").ok();
}

fn on_other_interrupt() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 5, 0)); // YELLOW
    writeln!(uart::Uart0, "[irq] unexpected local interrupt fired").ok();
}

fn on_exception(mcause: usize) {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 0, 5)); // PURPLE
    writeln!(uart::Uart0, "[exception] mcause = 0x{:08x}", mcause).ok();
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    hal::watchdog::disable_lp_watchdog();
    // hal::watchdog::disable_hp_watchdog();
    hal::timer::systimer_enable();

    uart::init(115200);
    writeln!(uart::Uart0, "\r\n--- project-trobbio booting ---").ok();

    trap::register(InterruptId::MachineTimer, on_machine_timer);
    trap::register(InterruptId::UserSoftware, on_other_interrupt);
    trap::register(InterruptId::MachineSoftware, on_other_interrupt);
    trap::register(InterruptId::UserTimer, on_other_interrupt);
    trap::set_exception_handler(on_exception);

    // CLINT runs at 16 MHz, so 16_000_000 ticks == 1 second between fires.
    trap::init_periodic_timer(16_000_000);

    writeln!(uart::Uart0, "boot complete, entering idle loop").ok();

    loop {
        hal::timer::delay_ms(50);
    }
}
