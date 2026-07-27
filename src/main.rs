#![allow(warnings)]
#![no_std]
#![no_main]

use crate::{
    arch::{
        sched,
        trap::{self, InterruptId},
    },
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
    extern "C" fn log_panic(info_ptr: usize) {
        // Safety: report_fatal_and_halt only ever calls this with the
        // address of the PanicInfo passed in below, still alive for the
        // duration of the call.
        let info = unsafe { &*(info_ptr as *const PanicInfo) };
        error!("PANIC: {}", info);
    }
    arch::trap::report_fatal_and_halt(log_panic, _info as *const PanicInfo as usize)
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
    warning!("[irq] unexpected local interrupt fired");
}

fn on_exception(mcause: usize) {
    let mut led = RgbLed::new(GpioPin::new(8, hal::gpio::GpioFunction::Gpio));
    led.refresh((5, 0, 5)); // PURPLE
    error!("[exception] mcause = 0x{:08x}", mcause);
}

fn task1() {
    loop {
        debug!("Task 1 running");
        hal::timer::delay_ms(500);
    }
}

fn task2() {
    // used to test stack canary
    let _ = core::hint::black_box(consume_stack(u32::MAX));
    loop {
        debug!("Task 2 running");
        hal::timer::delay_ms(500);
    }
}

#[inline(never)]
fn consume_stack(n: u32) -> u32 {
    let mut buf = [0u8; 64];
    core::hint::black_box(&mut buf); // forces a real stack frame per call
    if n == 0 {
        0
    } else {
        1 + consume_stack(n - 1) // the `1 +` makes this NOT a tail call
    }
}

// TODO:
// 1) make the exception code simpler
// 2) add an API for exception hooks
// 3) package the kernel inside a single API and call it done

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
    sched::init();

    info!("boot complete, entering idle loop");

    // Task builder test here
    sched::TaskBuilder::new(task1)
        .priority(sched::Priority::Low)
        .spawn()
        .expect("Failed to spawn task1");
    sched::spawn(task2).expect("Failed to spawn task2");
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
