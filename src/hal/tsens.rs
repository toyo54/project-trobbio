//! Digital temperature sensor (TSENS) — minimal driver.
//!
//! IMPORTANT: this reads the sensor's RAW output code, not calibrated
//! degrees Celsius. TRM 39.3 gives the conversion formula:
//!
//!   T(°C) = 0.4386 * VALUE - 27.88 * offset - 20.52
//!
//! `offset` is a fixed constant per measurement range (TRM Table
//! 39.3-1), NOT eFuse data. The real blocker is that this driver never
//! selects a range in the first place — that requires writing the DAC
//! bias via temperature_sensor_ll_set_range() in REGI2C, an internal
//! bit-banged config bus for analog blocks that isn't memory-mapped, so
//! it's invisible to the PAC (needs MODEM_LPCON's analog-I2C-master
//! clock, a PMU reset/enable sequence, and the bit-banged transaction
//! itself — a separate subsystem, not attempted here). Without a known
//! range, applying the formula with a guessed offset would just be
//! presenting an assumption as a calibrated reading, so this doesn't do
//! that. The raw code IS monotonic with temperature for a fixed clk_div,
//! which is enough for scheduler thresholding.
//!
//! Verified against ESP-IDF's actual temperature_sensor_ll.h /
//! temperature_sensor.c (esp32c6):
//! - Reset is a PULSE (`tsens_rst_en = 1` then `= 0`), not a held level.
//! - The real driver waits ~300us after enabling before the reading
//!   settles ("output value gradually approaches the true temperature
//!   value as measurement time increases").

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

    saradc.tsens_ctrl().modify(|_, w| w.pu().set_bit());

    // Separate from `pu` — this is the actual "start sampling" trigger.
    saradc.tsens_sample().modify(|_, w| w.en().set_bit());

    // The real driver's own comment: output "gradually approaches the
    // true temperature value as measurement time increases" — ~300us
    // before the first reading is meaningful.
    super::timer::delay_us(300);
}

/// Raw 8-bit sensor code. Monotonic with die temperature for a fixed
/// clk_div — NOT a calibrated °C value. See module docs. Safe to call
/// immediately after `init()`; the settle delay is already handled there.
pub fn read_raw() -> u8 {
    let saradc = unsafe { APB_SARADC::steal() };
    saradc.tsens_ctrl().read().out().bits()
}
