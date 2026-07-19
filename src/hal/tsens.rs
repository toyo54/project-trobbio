//! Digital temperature sensor (TSENS) — minimal driver.
//!
//! IMPORTANT: this reads the sensor's RAW output code, not calibrated
//! degrees Celsius. Real ESP-IDF conversion to °C depends on per-chip
//! eFuse calibration constants and the selected clk_div measurement
//! range that I have not verified — presenting an uncalibrated formula as
//! "temperature in °C" would just be a  guess, so this doesn't do that.
//! The raw code IS monotonic with temperature for a fixed clk_div.
//!
//! Verified against ESP-IDF's actual temperature_sensor_ll.h /
//! temperature_sensor.c (esp32c6):
//! - Reset is a PULSE (`tsens_rst_en = 1` then `= 0`), not a held level.
//! - The real driver waits ~300us after enabling before the reading
//!   settles ("output value gradually approaches the true temperature
//!   value as measurement time increases").
//!
//! Still NOT implementable from the PAC alone, and not attempted here:
//! calibrating the sensor's measurement range/DAC bias
//! (`temperature_sensor_ll_set_range`) requires writing through REGI2C —
//! an internal bit-banged I2C-style config bus for analog blocks that
//! isn't a memory-mapped peripheral at all, so it's invisible to the PAC.
//! Real ESP-IDF needs three more pieces to do this: enabling MODEM_LPCON's
//! analog-I2C-master clock, a specific PMU register reset/enable pulse
//! sequence, and the actual bit-banged read/write transaction over a
//! dedicated internal bus. That's a real, separate subsystem — not
//! something to bolt on quickly. Without it, this sensor may still not
//! read a properly-calibrated value even after the fixes below; it's an
//! honest open gap, not something papered over here.

use esp32c6::{APB_SARADC, PCR};

pub fn init() {
    let pcr = unsafe { PCR::steal() };

    pcr.tsens_clk_conf().modify(|_, w| {
        w.tsens_clk_sel()
            .clear_bit() // 0 = internal FOSC, no XTAL dependency
            .tsens_clk_en()
            .set_bit()
    });

    // Reset is a PULSE, not a held level — matches
    // temperature_sensor_ll_reset_module() exactly.
    pcr.tsens_clk_conf()
        .modify(|_, w| w.tsens_rst_en().set_bit());
    pcr.tsens_clk_conf()
        .modify(|_, w| w.tsens_rst_en().clear_bit());

    let saradc = unsafe { APB_SARADC::steal() };
    // clk_div deliberately untouched: its hardware reset value is 6
    // (decoded from the PAC's own reset value 0x0001_8080 for this
    // register), which matches ESP-IDF's own comment on this field
    // ("suggest just keep it as default value 6... only used in legacy
    // driver") — the current, non-legacy driver never sets it at all.
    saradc.tsens_ctrl().modify(|_, w| w.pu().set_bit());

    // Separate from `pu` — this is the actual "start sampling" trigger.
    saradc.tsens_sample().modify(|_, w| w.en().set_bit());

    // The real driver's own comment: output "gradually approaches the
    // true temperature value as measurement time increases" — ~300us
    // before the first reading is meaningful.     super::timer::delay_us(300);
}

/// Raw 8-bit sensor code. Monotonic with die temperature for a fixed
/// clk_div — NOT a calibrated °C value. See module docs. Safe to call
/// immediately after `init()`; the settle delay is already handled there.
pub fn read_raw() -> u8 {
    let saradc = unsafe { APB_SARADC::steal() };
    saradc.tsens_ctrl().read().out().bits()
}
