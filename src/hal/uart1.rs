//! Polling UART1 driver, routed through the GPIO Matrix to caller-chosen
//! pins — NOT INTMTX (that's interrupt routing, unrelated; this is data
//! signal routing). UART0 stays untouched on its default console pins;
//! this is a second, independent, pin-flexible UART.
//!
//! Verified against real sources, not assumed:
//! - `U1TXD_OUT_IDX`/`U1RXD_IN_IDX` = 9 (both), from ESP-IDF's
//!   esp32c6/include/soc/gpio_sig_map.h.
//! - UART1 shares the exact same register layout as UART0 in the PAC
//!   (`esp32c6::UART1 = Periph<uart0::RegisterBlock, 0x6000_1000>`), so
//!   the clock/baud/FIFO logic below mirrors hal::uart.rs exactly.
//!
//! NOT independently verified (best-effort, needs testing on real
//! hardware): the OEN_SEL reasoning for the TX pin below — leaving it at
//! its reset value (0 = peripheral-controlled output-enable) should mean
use esp32c6::{GPIO, IO_MUX, UART1};

const APB_CLK_HZ: u32 = 80_000_000;
const U1TXD_OUT_IDX: u8 = 9;
const U1RXD_IN_IDX: u8 = 9;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    Timeout,
}

const WRITE_TIMEOUT_US: u64 = 10_000;

/// Routes `tx_pin`/`rx_pin` through the GPIO Matrix to UART1, then
/// initializes it for 8N1 at `baud`, clocked from APB.
pub fn init(tx_pin: u8, rx_pin: u8, baud: u32) {
    let io_mux = unsafe { IO_MUX::steal() };
    let gpio = unsafe { GPIO::steal() };

    io_mux
        .gpio(tx_pin as usize)
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
    io_mux
        .gpio(rx_pin as usize)
        .modify(|_, w| unsafe { w.mcu_sel().bits(1) });

    gpio.func_out_sel_cfg(tx_pin as usize)
        .write(|w| unsafe { w.out_sel().bits(U1TXD_OUT_IDX) });

    gpio.func_in_sel_cfg(U1RXD_IN_IDX as usize)
        .write(|w| unsafe { w.in_sel().bits(rx_pin).sel().set_bit() });

    let uart = unsafe { UART1::steal() };

    uart.clk_conf().write(|w| unsafe {
        w.sclk_sel()
            .bits(1)
            .sclk_en()
            .set_bit()
            .tx_sclk_en()
            .set_bit()
            .rx_sclk_en()
            .set_bit()
    });

    let divider_x16 = (APB_CLK_HZ as u64 * 16) / baud as u64;
    let clkdiv = (divider_x16 / 16) as u16;
    let frag = (divider_x16 % 16) as u8;
    debug_assert!(clkdiv <= 0x0FFF, "baud rate too low for a 12-bit CLKDIV");

    uart.clkdiv()
        .write(|w| unsafe { w.clkdiv().bits(clkdiv).frag().bits(frag) });

    uart.conf0().modify(|_, w| unsafe {
        w.bit_num()
            .bits(3)
            .stop_bit_num()
            .bits(1)
            .parity_en()
            .clear_bit()
    });

    uart.reg_update().modify(|_, w| w.reg_update().set_bit());
    while uart.reg_update().read().reg_update().bit_is_set() {
        core::hint::spin_loop();
    }

    uart.conf0()
        .modify(|_, w| w.txfifo_rst().set_bit().rxfifo_rst().set_bit());
    uart.conf0()
        .modify(|_, w| w.txfifo_rst().clear_bit().rxfifo_rst().clear_bit());
}

pub fn write_byte(byte: u8) -> Result<(), UartError> {
    let uart = unsafe { UART1::steal() };
    let start = super::timer::systimer_now_us();
    while uart.status().read().txfifo_cnt().bits() >= 100 {
        if super::timer::systimer_now_us().wrapping_sub(start) > WRITE_TIMEOUT_US {
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

pub fn read_byte() -> Option<u8> {
    let uart = unsafe { UART1::steal() };
    if uart.status().read().rxfifo_cnt().bits() == 0 {
        return None;
    }
    Some(uart.fifo().read().rxfifo_rd_byte().bits())
}

/// Toggles UART_LOOPBACK (CONF0 bit 12): connects TX straight to RX
/// *inside the peripheral*, before either signal reaches the GPIO Matrix
/// or a pin. Useful for testing with zero external hardware — but it only
/// exercises the clock/baud/FIFO/framing logic, since it bypasses the
/// pins entirely. It does NOT validate func_out_sel_cfg/func_in_sel_cfg
/// or the OEN_SEL behavior documented above; that still needs an actual
/// external TX->RX wire (or at minimum a scope/meter on the pin).
pub fn set_loopback(enable: bool) {
    let uart = unsafe { UART1::steal() };
    uart.conf0().modify(|_, w| w.loopback().bit(enable));

    uart.reg_update().modify(|_, w| w.reg_update().set_bit());
    while uart.reg_update().read().reg_update().bit_is_set() {
        core::hint::spin_loop();
    }
}

pub struct Uart1;

impl core::fmt::Write for Uart1 {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}
