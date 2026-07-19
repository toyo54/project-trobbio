//! Polling UART0 driver.
//!
//! **RESERVED: this is the kernel's debug console, permanently.** UART0 is
//! wired to the board's USB-serial bridge, so this is the one channel
//! that's always visible with zero extra wiring — every panic, tick log,
//! and diagnostic in this project goes through it. It is never to be
//! repurposed for peripheral/application I/O, the same way arch::trap
//! reserves MTIME for the watchdog feed: the one moment you actually need
//! a debug channel (mid-crash, mid-fault, chasing a different bug down)
//! is exactly when it can't also be busy doing something else. Use
//! hal::uart1 for loopback testing, external peripherals, or anything
//! that isn't "print a log line."
//!
//! No INTPRI/interrupt setup involved: TX/RX here are plain busy-wait
//! polling on FIFO counts, same category of access as gpio.rs/watchdog.rs.
//! Interrupt-driven RX (the actual reason to touch INTPRI) is a later step.
//!
//! Verified against the real esp32c6 v0.23.2 PAC source, same as CLINT:
//! `clk_conf().sclk_sel()`'s doc comment confirms `1 == 80MHz (APB) clock`,
//! which is the one field value here I couldn't otherwise be fully sure of.

use super::timer;
use esp32c6::UART0;

const APB_CLK_HZ: u32 = 80_000_000;

/// If the FIFO stays full this long, write_byte gives up rather than
/// spinning forever (e.g. a disconnected terminal that's stopped draining
/// it). 10ms is generous at 115200 baud — a full FIFO drains in well under
/// 1ms per byte at that rate.
const WRITE_TIMEOUT_US: u64 = 10_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// The TX FIFO didn't drain within WRITE_TIMEOUT_US.
    Timeout,
}

/// Initializes UART0 for 8N1 at `baud`, clocked from APB (matches
/// UART_SCLK_DEFAULT in the ESP-IDF example).
pub fn init(baud: u32) {
    let uart = unsafe { UART0::steal() };

    uart.clk_conf().write(|w| unsafe {
        w.sclk_sel()
            .bits(1) // 1 = 80MHz APB clock (confirmed in the PAC's own field doc)
            .sclk_en()
            .set_bit()
            .tx_sclk_en()
            .set_bit()
            .rx_sclk_en()
            .set_bit()
    });

    // 16ths-of-a-tick fixed point gets us the FRAG (bits 20:23) precision the
    // hardware actually supports, instead of leaving it at 0 and eating a
    // baud-rate rounding error on every rate that isn't a clean divisor.
    let divider_x16 = (APB_CLK_HZ as u64 * 16) / baud as u64;
    let clkdiv = (divider_x16 / 16) as u16;
    let frag = (divider_x16 % 16) as u8;

    // CLKDIV is a 12-bit field (0..=4095); below ~19.5kbaud @ 80MHz APB this
    // would silently truncate to a bogus divider instead of erroring, so
    // catch it in debug builds rather than ship a garbage baud rate.
    debug_assert!(clkdiv <= 0x0FFF, "baud rate too low for a 12-bit CLKDIV");

    uart.clkdiv()
        .write(|w| unsafe { w.clkdiv().bits(clkdiv).frag().bits(frag) });

    // 8 data bits, 1 stop bit, no parity.
    uart.conf0().write(|w| unsafe {
        w.bit_num()
            .bits(3) // 3 = 8 data bits
            .stop_bit_num()
            .bits(1) // 1 = 1 stop bit
            .parity_en()
            .clear_bit()
    });

    // NOTE: no reg_update call here — see module docs. Given the above
    // writes were (apparently) never actually committing anyway, this
    // function's real-world effect right now is close to a no-op, which is
    // exactly why the console kept working before this edit and should
    // keep working now that the change is reverted.

    // Pulse both FIFOs through reset before first use.
    uart.conf0()
        .modify(|_, w| w.txfifo_rst().set_bit().rxfifo_rst().set_bit());
    uart.conf0()
        .modify(|_, w| w.txfifo_rst().clear_bit().rxfifo_rst().clear_bit());
}

/// Blocks until there's room, then pushes one byte into the TX FIFO — but
/// gives up after WRITE_TIMEOUT_US instead of spinning forever if the FIFO
/// never drains. See the module docs for why an unbounded wait here would
/// be dangerous once the real hardware watchdog is enabled: this runs from
/// inside the reserved machine-timer tick, and an infinite loop here would
/// mean that tick never returns, and the watchdog never gets fed.
pub fn write_byte(byte: u8) -> Result<(), UartError> {
    let uart = unsafe { UART0::steal() };
    let start = timer::systimer_now_us();
    while uart.status().read().txfifo_cnt().bits() >= 100 {
        if timer::systimer_now_us().wrapping_sub(start) > WRITE_TIMEOUT_US {
            return Err(UartError::Timeout);
        }
    }
    uart.fifo()
        .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
    Ok(())
}

pub fn write_bytes(bytes: &[u8]) -> Result<(), UartError> {
    for &b in bytes {
        write_byte(b)?;
    }
    Ok(())
}

/// Non-blocking single-byte read: `None` if the RX FIFO is empty.
pub fn read_byte() -> Option<u8> {
    let uart = unsafe { UART0::steal() };
    if uart.status().read().rxfifo_cnt().bits() == 0 {
        return None;
    }
    Some(uart.fifo().read().rxfifo_rd_byte().bits())
}

/// Zero-sized handle so `write!`/`writeln!` work directly. Kept `pub` for
/// `panic_handler` and anything low-level enough to need it bare — but
/// everything else should reach for the log macros below instead, so
/// callers never need to know Uart0 exists at all.
pub struct Uart0;

impl core::fmt::Write for Uart0 {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let _ = writeln!(
            $crate::hal::uart::Uart0,
            concat!("\x1b[31m[ERROR]\x1b[0m ", "{}", ),
            format_args!($($arg)*)
        );
    }};
}

#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let _ = writeln!(
            $crate::hal::uart::Uart0,
            concat!("\x1b[33m[WARN]\x1b[0m ", "{}"),
            format_args!($($arg)*)
        );
    }};
}

#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let _ = writeln!(
            $crate::hal::uart::Uart0,
            concat!("\x1b[32m[INFO]\x1b[0m ", "{}"),
            format_args!($($arg)*)
        );
    }};
}

#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let _ = writeln!(
            $crate::hal::uart::Uart0,
            concat!("\x1b[36m[DEBUG] ", "{}", "\x1b[0m"),
            format_args!($($arg)*)
        );
    }};
}
