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
//! - FUNC_OUT_SEL_CFG.OEN_SEL is left at its reset value (0 =
//!   peripheral-controlled output-enable). GPIO_ENABLE is still set by
//!   hand below — that part's required either way, it's just OEN_SEL
//!   that should stay off for a routed UART TX signal.
//!
//! Hardware-tested on an ESP32-C6-WROOM devkit, TX/RX looped through an
//! external jumper (GPIO10 -> GPIO11), 115200 8N1. Confirmed clean,
//! repeated byte-for-byte reception.

use esp32c6::{GPIO, IO_MUX, PCR, UART1};

const APB_CLK_HZ: u32 = 80_000_000;
const U1TXD_OUT_IDX: u8 = 9;
const U1RXD_IN_IDX: u8 = 9;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    Timeout,
}

const WRITE_TIMEOUT_US: u64 = 10_000;

pub fn init(tx_pin: u8, rx_pin: u8, baud: u32) {
    let pcr = unsafe { PCR::steal() };
    let io_mux = unsafe { IO_MUX::steal() };
    let gpio = unsafe { GPIO::steal() };
    let uart = unsafe { UART1::steal() };

    io_mux
        .gpio(tx_pin as usize)
        .write(|w| unsafe { w.mcu_sel().bits(1).fun_ie().set_bit().fun_drv().bits(2) });
    io_mux.gpio(rx_pin as usize).write(|w| unsafe {
        w.mcu_sel()
            .bits(1)
            .fun_ie()
            .set_bit()
            .fun_wpu()
            .set_bit()
            .fun_drv()
            .bits(2)
    });

    pcr.uart(1)
        .conf()
        .modify(|_, w| w.clk_en().set_bit().rst_en().clear_bit());
    pcr.uart(1).clk_conf().modify(|_, w| w.sclk_en().set_bit());

    io_mux
        .gpio(tx_pin as usize)
        .modify(|_, w| unsafe { w.mcu_sel().bits(1).fun_ie().set_bit() });

    io_mux
        .gpio(rx_pin as usize)
        .modify(|_, w| unsafe { w.mcu_sel().bits(1).fun_ie().set_bit().fun_wpu().set_bit() });

    // OEN_SEL left clear (peripheral controls TX output-enable). GPIO_ENABLE
    // is still asserted below via enable_w1ts, which is needed regardless.
    gpio.func_out_sel_cfg(tx_pin as usize)
        .write(|w| unsafe { w.out_sel().bits(U1TXD_OUT_IDX) });

    gpio.func_in_sel_cfg(U1RXD_IN_IDX as usize)
        .write(|w| unsafe { w.in_sel().bits(rx_pin).sel().set_bit() });

    gpio.enable_w1ts()
        .write(|w| unsafe { w.enable_w1ts().bits(1 << tx_pin) });

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
    uart.reg_update().modify(|_, w| w.reg_update().set_bit());
    while uart.reg_update().read().reg_update().bit_is_set() {
        core::hint::spin_loop();
    }

    uart.conf0()
        .modify(|_, w| w.txfifo_rst().clear_bit().rxfifo_rst().clear_bit());
    uart.reg_update().modify(|_, w| w.reg_update().set_bit());
    while uart.reg_update().read().reg_update().bit_is_set() {
        core::hint::spin_loop();
    }
}

pub fn write_byte(byte: u8) -> Result<(), UartError> {
    let uart = unsafe { UART1::steal() };
    let start = super::timer::systimer_now_us();
    while uart.status().read().txfifo_cnt().bits() >= 100 {
        if super::timer::systimer_now_us().wrapping_sub(start) > WRITE_TIMEOUT_US {
            return Err(UartError::Timeout);
        }
    }
    // Field is named rxfifo_rd_byte in the PAC even for TX pushes -- it's
    // the same physical FIFO register on both sides, SVD just kept one name.
    uart.fifo()
        .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
    Ok(())
}

pub fn read_byte() -> Option<u8> {
    let uart = unsafe { UART1::steal() };
    if uart.status().read().rxfifo_cnt().bits() == 0 {
        return None;
    }
    Some(uart.fifo().read().rxfifo_rd_byte().bits())
}

pub fn write_bytes(bytes: &[u8]) -> Result<(), UartError> {
    for &b in bytes {
        write_byte(b)?;
    }
    Ok(())
}

/// Toggles UART_LOOPBACK (CONF0 bit 12): connects TX straight to RX
/// *inside the peripheral*, before either signal reaches the GPIO Matrix
/// or a pin. Useful for testing with zero external hardware — but it only
/// exercises the clock/baud/FIFO/framing logic, since it bypasses the
/// pins entirely. It does NOT validate func_out_sel_cfg/func_in_sel_cfg
/// or pin-level wiring; that still needs an actual external TX->RX wire.
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

/// No-tools-needed hardware diagnostics -- if UART1 ever goes
/// quiet again on a different board or different pins,
/// run these before assuming it's a code bug.
pub mod diag {
    use super::*;

    #[derive(Debug, Clone, Copy)]
    pub struct RegSnapshot {
        pub func_out_sel: u32,
        pub oen_sel: bool,
        pub func_in_sel: u32,
        pub sig_in_sel: bool,
        pub gpio_enable: bool,
        pub tx_iomux_mcu_sel: u8,
        pub rx_iomux_mcu_sel: u8,
    }

    /// Dumps the exact registers `init` touches, read back from hardware.
    pub fn snapshot(tx_pin: u8, rx_pin: u8) -> RegSnapshot {
        let io_mux = unsafe { IO_MUX::steal() };
        let gpio = unsafe { GPIO::steal() };

        let out_cfg = gpio.func_out_sel_cfg(tx_pin as usize).read();
        let in_cfg = gpio.func_in_sel_cfg(U1RXD_IN_IDX as usize).read();
        let enable_reg = gpio.enable().read().bits();

        RegSnapshot {
            func_out_sel: out_cfg.out_sel().bits() as u32,
            oen_sel: out_cfg.oen_sel().bit_is_set(),
            func_in_sel: in_cfg.in_sel().bits() as u32,
            sig_in_sel: in_cfg.sel().bit_is_set(),
            gpio_enable: (enable_reg & (1 << tx_pin)) != 0,
            tx_iomux_mcu_sel: io_mux.gpio(tx_pin as usize).read().mcu_sel().bits(),
            rx_iomux_mcu_sel: io_mux.gpio(rx_pin as usize).read().mcu_sel().bits(),
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct SelfTestResult {
        pub internal_loopback_ok: bool,
        pub external_wire_ok: bool,
    }

    /// Internal loopback first (rules clock/baud/FIFO in or out), then
    /// the real pin path. Splits "is the peripheral alive" from "is the
    /// wiring good" in one call.
    pub fn self_test(tx_pin: u8, rx_pin: u8, baud: u32) -> SelfTestResult {
        init(tx_pin, rx_pin, baud);

        set_loopback(true);
        let _ = write_byte(0xA5);
        super::super::timer::delay_ms(5);
        let internal_loopback_ok = read_byte() == Some(0xA5);
        set_loopback(false);

        while read_byte().is_some() {}

        let _ = write_byte(0x5A);
        super::super::timer::delay_ms(5);
        let external_wire_ok = read_byte() == Some(0x5A);

        SelfTestResult {
            internal_loopback_ok,
            external_wire_ok,
        }
    }

    /// Bypasses the UART peripheral and the signal matrix entirely --
    /// plain GPIO drive on tx_pin, plain GPIO read on rx_pin.
    pub fn raw_gpio_loopback_test(tx_pin: u8, rx_pin: u8) -> (bool, bool) {
        const SIG_GPIO_OUT_IDX: u8 = 128;

        let io_mux = unsafe { IO_MUX::steal() };
        let gpio = unsafe { GPIO::steal() };

        io_mux
            .gpio(tx_pin as usize)
            .write(|w| unsafe { w.mcu_sel().bits(1).fun_drv().bits(2) });
        io_mux
            .gpio(rx_pin as usize)
            .write(|w| unsafe { w.mcu_sel().bits(1).fun_ie().set_bit().fun_wpd().set_bit() });

        gpio.func_out_sel_cfg(tx_pin as usize)
            .write(|w| unsafe { w.out_sel().bits(SIG_GPIO_OUT_IDX).oen_sel().set_bit() });
        gpio.enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << tx_pin) });

        gpio.out_w1ts()
            .write(|w| unsafe { w.out_w1ts().bits(1 << tx_pin) });
        super::super::timer::delay_us(50);
        let high_seen = (gpio.in_().read().bits() & (1 << rx_pin)) != 0;

        gpio.out_w1tc()
            .write(|w| unsafe { w.out_w1tc().bits(1 << tx_pin) });
        super::super::timer::delay_us(50);
        let low_seen = (gpio.in_().read().bits() & (1 << rx_pin)) == 0;

        (high_seen, low_seen)
    }

    /// Drives `pin` and reads it back on itself -- no second pin, no
    /// wire. Use `raw_gpio_loopback_test` for a real pad-to-pad check.
    pub fn self_output_test(pin: u8) -> bool {
        const SIG_GPIO_OUT_IDX: u8 = 128;
        let io_mux = unsafe { IO_MUX::steal() };
        let gpio = unsafe { GPIO::steal() };

        io_mux
            .gpio(pin as usize)
            .write(|w| unsafe { w.mcu_sel().bits(1).fun_ie().set_bit().fun_drv().bits(2) });
        gpio.func_out_sel_cfg(pin as usize)
            .write(|w| unsafe { w.out_sel().bits(SIG_GPIO_OUT_IDX).oen_sel().set_bit() });
        gpio.enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << pin) });

        gpio.out_w1ts()
            .write(|w| unsafe { w.out_w1ts().bits(1 << pin) });
        super::super::timer::delay_us(50);
        let high_seen = (gpio.in_().read().bits() & (1 << pin)) != 0;

        gpio.out_w1tc()
            .write(|w| unsafe { w.out_w1tc().bits(1 << pin) });
        super::super::timer::delay_us(50);
        let low_seen = (gpio.in_().read().bits() & (1 << pin)) == 0;

        high_seen && low_seen
    }

    /// GPIO9 is the BOOT button on this devkit, active-low, no wiring
    /// needed. Good known-good reference for "is gpio.in_() reading
    /// pads correctly at all" when everything else is in doubt.
    pub fn read_boot_button() -> bool {
        let io_mux = unsafe { IO_MUX::steal() };
        let gpio = unsafe { GPIO::steal() };

        io_mux
            .gpio(9)
            .write(|w| unsafe { w.mcu_sel().bits(1).fun_ie().set_bit() });

        (gpio.in_().read().bits() & (1 << 9)) != 0
    }

    /// Polls BOOT for `duration_ms` so press/release doesn't have to be
    /// timed against a slow log by eye. Returns (saw_released, saw_pressed).
    pub fn watch_boot_button(duration_ms: u32) -> (bool, bool) {
        let mut saw_true = false;
        let mut saw_false = false;
        let iterations = duration_ms / 20;
        for _ in 0..iterations {
            if read_boot_button() {
                saw_true = true;
            } else {
                saw_false = true;
            }
            super::super::timer::delay_ms(20);
        }
        (saw_true, saw_false)
    }
}
