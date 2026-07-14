#![allow(warnings)]
#![no_std]
#![no_main]

use crate::{
    arch::trap::{self, InterruptId},
    drivers::ws2812::RgbLed,
    hal::{gpio::GpioPin, timer, watchdog::feed_watchdog},
};
use core::{arch::global_asm, panic::PanicInfo};

mod arch;
mod drivers;
mod hal;

global_asm!(".global _vector_table", include_str!("boot.s"));

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    loop {
        feed_watchdog();
        led.refresh((5, 0, 0)); // RED = Panic
    }
}

/// Runs from trap context on every machine-timer tick, right after the
/// kernel's own (unconditional) watchdog feed. This is optional — it's
/// just here as visible proof the tick is actually firing.
fn on_machine_timer() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((0, 5, 0)); // GREEN
}

/// Runs from trap context for any other (non-timer) local interrupt.
fn on_other_interrupt() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 5, 0)); // YELLOW
}

/// Runs from trap context for a genuine exception (mcause interrupt bit clear).
fn on_exception(_mcause: usize) {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 0, 5)); // PURPLE
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    hal::watchdog::disable_lp_watchdog();
    // hal::watchdog::disable_hp_watchdog();
    hal::timer::systimer_enable();

    trap::register(InterruptId::MachineTimer, on_machine_timer);
    trap::register(InterruptId::UserSoftware, on_other_interrupt);
    trap::register(InterruptId::MachineSoftware, on_other_interrupt);
    trap::register(InterruptId::UserTimer, on_other_interrupt);
    trap::set_exception_handler(on_exception);

    // CLINT runs at 16 MHz, so 16_000_000 ticks == 1 second between fires.
    trap::init_periodic_timer(16_000_000);

    // No manual feed_watchdog() here anymore: MTIME is reserved by the
    // kernel (see arch::trap) to do that on every tick, unconditionally.
    // The idle loop just... idles.
    loop {
        hal::timer::delay_ms(50);
    }
}
