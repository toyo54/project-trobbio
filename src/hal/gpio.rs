use esp32c6::{GPIO, IO_MUX};

pub const GPIO_COUNT: u8 = 31;

/// Selects wich internal function is routed to a GPIO pin via IO_MUX.
/// This maps to the `mcu_sel` field in the IO_MUX register
#[derive(Clone, Copy)]
pub enum GpioFunction {
    Gpio,
    Peripheral(u8),
}

impl GpioFunction {
    fn bits(self) -> u8 {
        match self {
            Self::Gpio => 1,
            GpioFunction::Peripheral(f) => f,
        }
    }
}

/// A configured GPIO output pin
#[derive(Clone, Copy)]
pub struct GpioPin {
    pin: u8,
}

impl GpioPin {
    /// Creates and initializes a new GPIO pin
    ///
    /// Combines construction and initialization in one step
    /// to avoid invalid state and prevent accidental usage
    /// before initialization
    pub fn new(pin: u8, function: GpioFunction) -> Self {
        assert!(pin < GPIO_COUNT);
        let pin = Self { pin };
        pin.init(function);
        pin
    }

    /// Configures the IO_MUX and GPIO matrix for this pin
    fn init(&self, function: GpioFunction) {
        let io_mux = unsafe { IO_MUX::steal() };
        let gpio = unsafe { GPIO::steal() };

        // selects function on the IO_MUX matrix
        io_mux
            .gpio(self.pin as usize)
            .modify(|_, w| unsafe { w.mcu_sel().bits(function.bits()) });

        //  ensure the pin start low before enabling it as an output
        gpio.out_w1tc()
            .write(|w| unsafe { w.out_w1tc().bits(1 << self.pin) });

        // enables the gpio
        gpio.enable_w1ts()
            .write(|w| unsafe { w.enable_w1ts().bits(1 << self.pin) });
    }

    /// Drives the pin HIGH
    ///
    /// Uses W1TS register - atomic operation
    #[inline(always)]
    pub fn set_high(&self) {
        let gpio = unsafe { GPIO::steal() };
        gpio.out_w1ts()
            .write(|w| unsafe { w.out_w1ts().bits(1 << self.pin) });
    }

    /// Drives the pin LOW
    ///
    /// Uses W1TC register - atomic operation
    #[inline(always)]
    pub fn set_low(&self) {
        let gpio = unsafe { GPIO::steal() };
        gpio.out_w1tc()
            .write(|w| unsafe { w.out_w1tc().bits(1 << self.pin) });
    }
}
