#![no_std]
#![no_main]

use project_trobbio::{
    arch::sched,
    drivers::ws2812::RgbLed,
    hal::{self, gpio::GpioPin},
};

fn on_machine_timer() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((0, 5, 0)); // GREEN
    let raw = hal::tsens::read_raw();
    project_trobbio::info!("[tick] watchdog fed, tsens raw = {}", raw);
}

fn on_other_interrupt() {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 5, 0)); // YELLOW
    project_trobbio::warning!("[irq] unexpected local interrupt fired");
}

fn on_exception(mcause: usize) {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 0, 5)); // PURPLE
    project_trobbio::error!("[exception] mcause = 0x{:08x}", mcause);
}

fn task1() {
    loop {
        project_trobbio::debug!("Task 1 running");
        hal::timer::delay_ms(500);
    }
}

fn task2() {
    // used to test stack canary
    // let _ = core::hint::black_box(consume_stack(u32::MAX));
    loop {
        project_trobbio::debug!("Task 2 running");
        hal::timer::delay_ms(500);
    }
}

#[allow(dead_code)]
#[inline(never)]
/// Overflows the stack
///
/// This function is present only for testing purposes,
/// specifically, to test the stack canary
fn consume_stack(n: u32) -> u32 {
    let mut buf = [0u8; 64];
    core::hint::black_box(&mut buf); // forces a real stack frame per call
    if n == 0 {
        0
    } else {
        1 + consume_stack(n - 1) // the `1 +` makes this NOT a tail call
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let _kernel = project_trobbio::boot_scheduled!(
        timer: on_machine_timer,
        other: on_other_interrupt,
        exception: on_exception,
        tasks: [task1 => sched::Priority::Low, task2]
    );

    project_trobbio::info!("boot complete, entering idle loop");

    // UART1 on GPIO10 (TX) / GPIO11 (RX)
    // Also hardware-tested with an external TX->RX jumper, clean and repeatable.
    hal::uart1::init(10, 11, 115200);
    hal::uart1::set_loopback(true); // if disabled, the 2 physical pins must be connected via
    // jumpers

    loop {
        hal::uart1::write_bytes(b"test").ok();
        hal::timer::delay_ms(100);
        while let Some(b) = hal::uart1::read_byte() {
            project_trobbio::info!("Loopback success: {}", b as char);
        }
        hal::timer::delay_ms(1000);
    }
}
