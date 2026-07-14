//! # WS2812 Driver
//!
//! The ws2812 protocol is simple, each color is represented through a
//! a stream of bytes, the protocol accepts RGB in this order GRB, so
//! if you want the color red on the LED you send:
//! ( G: 0b0000_0000; R: 0b1111_1111; B: 0b0000_0000 )
//!
//!  -  Note that not every bit of each color byte must be 1, the lower the byte value
//!     the less intense is the brightness of that color channel.
//!
//! ## Color encoding
//! So, how does the RGB led understand when to be ON and when to be OFF?
//! - The answer is: timing. If the pin is HIGH for ~700 nanoseconds and low
//!   LOW for ~600 ns, then HIGH dominates. Otherwise if the pin is HIGH for
//!   ~350 ns and LOW for ~850 ns, then LOW dominates.
//!
//! ## Details about hardware
//! Now, the circuit is implemented with a single data wire so the hardware
//! interprets each color value bit by bit, this results in the hardware
//! effectively deciphering the byte value bit after bit.
//!
//! Keep in mind that in the end, the RGB LED hardware expects nothing but a
//! stream of bits on GPIO8, the catch is in how it interprets the bits (timing is critical)
//!
//! ## Reset pulse
//! After the 24 bits are sent, the line must be held low for at least 50 microseconds.
//! Without the reset pulse will wait for more data instead of displaying the color.
//!
//! ## Why did the hardware manufacturers pass the timing responsibility on us?
//! Because of a design choice in mind: simplicity. RGB LEDs are meant to be
//! chainable and to keep things simple they can work by only sharing a single
//! wire in succession. The trade-off is that there isn't any clock wire
//! that tells the hardware: "ok now you can read the data wire" so this
//! means that the timing responsibility is up to the developer, not the hardware.

use crate::hal::gpio::GpioPin;
use crate::hal::timer::{delay_ns, delay_us};

pub struct RgbLed {
    pin: GpioPin,
    rgb: (u8, u8, u8),
}

impl RgbLed {
    pub fn new(pin: GpioPin) -> Self {
        let led = Self {
            pin,
            rgb: (0, 0, 0),
        };
        led.set_color();
        led
    }

    /// Updates the currently stored color and sends it to the hardware
    ///
    /// Takes a tuple of (red ,green, blue) each 0 - 255
    #[inline(always)]
    pub fn refresh(&mut self, rgb: (u8, u8, u8)) {
        self.rgb = rgb;
        self.set_color();
    }

    /// Sends a single byte to the LED one bit at a time, MSB first.
    ///
    /// The loop iters from bit 7 (most signficant) down to bit 0 (least signficant)
    /// because the WS2812 expects MSB first
    ///
    /// Timing is handled by delay_ns which uses CPU cycle counting
    /// via inline assembly. See `hal/timer.rs` for details on why
    /// SYSTIMER is not used here
    fn send_byte(&self, byte: u8) {
        for i in (0..=7).rev() {
            if byte & (1 << i) != 0 {
                self.pin.set_high();
                delay_ns(500);
                self.pin.set_low();
                delay_ns(450);
            } else {
                self.pin.set_high();
                delay_ns(100);
                self.pin.set_low();
                delay_ns(700);
            }
        }
    }

    /// Sends the currently stored color to the hardware
    ///
    /// Useful for re-latching the color after a power glitch
    /// or if another device corrupted the LED state.
    /// Normally use `refresh()` instead.
    ///
    /// ```text
    /// # Transmission sequence:
    /// PIN: LOW 50 micros -----> bit stream -----> idle LOW
    /// ```
    pub fn set_color(&self) {
        let (r, g, b) = self.rgb;

        // start low to prevent invalid states
        self.pin.set_low();
        delay_us(50);

        // sends 24 bits in GRB order MSB first
        self.send_byte(g);
        self.send_byte(r);
        self.send_byte(b);

        // Reset pulse
        delay_us(50);
    }
}
