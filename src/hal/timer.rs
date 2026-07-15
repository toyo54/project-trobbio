//! # SYSTIMER - Implementation
//!
//! This implementation uses a dual-unit architecture to bypass hardware snapshot
//! collisions. Unit 1 is dedicated entirely to OS delays, while Unit 0 is
//! dedicated exclusively to driving the periodic scheduler/watchdog interrupt.

use esp32c6::{Interrupt as HwInterrupt, PCR, SYSTIMER};

// SYSTIMER runs at 16 MHz regardless of the CPU clock
pub const SYSTIMER_CLK_HZ: u64 = 16_000_000;

// This MUST be 11 to match `set_mext()` in main.rs and `match code` in trap.rs
pub const SYSTIMER_CPU_LINE: usize = 17;

pub fn systimer_enable() {
    let systimer = unsafe { SYSTIMER::steal() };
    let pcr = unsafe { PCR::steal() };

    pcr.systimer_conf()
        .modify(|_, w| w.systimer_clk_en().set_bit().systimer_rst_en().clear_bit());

    pcr.systimer_func_clk_conf()
        .modify(|_, w| w.systimer_func_clk_en().set_bit());

    systimer.conf().modify(|_, w| {
        w.clk_en()
            .set_bit()
            .timer_unit0_work_en()
            .set_bit()
            .timer_unit1_work_en()
            .set_bit()
    });
}

/* ======================= OS Delays (UNIT 1) ======================= */

/// Reads the raw 52-bit counter value from SYSTIMER Unit 1 safely.
/// This is physically isolated from the interrupt handler.
fn systimer_now_ticks_unit1() -> u64 {
    let systimer = unsafe { SYSTIMER::steal() };

    systimer.unit1_op().write(|w| w.update().set_bit());
    while systimer.unit1_op().read().value_valid().bit_is_clear() {}

    let hi = systimer.unit1_value().hi().read().bits() as u64;
    let lo = systimer.unit1_value().lo().read().bits() as u64;
    (hi << 32) | lo
}

pub fn systimer_now_us() -> u64 {
    systimer_now_ticks_unit1() / 16
}
pub fn systimer_now_ms() -> u64 {
    systimer_now_ticks_unit1() / 16_000
}

/// delay milliseconds
pub fn delay_ms(ms: u64) {
    delay_us(ms * 1_000);
}

/// delay microseconds
pub fn delay_us(us: u64) {
    let start = systimer_now_us();
    while systimer_now_us().wrapping_sub(start) < us {}
}

/// delaay nanoseconds
#[inline(always)]
pub fn delay_ns(nanos: u32) {
    let mut iterations = (nanos * 80) / 3000;
    if iterations > 0 {
        unsafe {
            core::arch::asm!(
                "1:", "addi {0}, {0}, -1", "bnez {0}, 1b",
                inout(reg) iterations, options(nostack)
            );
        }
    }
}

/* ======================= Watchdog / Scheduler (UNIT 0) ======================= */

/// Reads the raw 52-bit counter value from SYSTIMER Unit 0 safely.
/// Dedicated exclusively to the interrupt handler to prevent race conditions.
fn systimer_now_ticks_unit0() -> u64 {
    let systimer = unsafe { SYSTIMER::steal() };

    systimer.unit0_op().write(|w| w.update().set_bit());
    while systimer.unit0_op().read().value_valid().bit_is_clear() {}

    let hi = systimer.unit0_value().hi().read().bits() as u64;
    let lo = systimer.unit0_value().lo().read().bits() as u64;
    (hi << 32) | lo
}
