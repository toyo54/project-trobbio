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

pub fn systimer_enable_interrupt() {
    let systimer = unsafe { esp32c6::SYSTIMER::steal() };
    let period_ticks = 1_600_000; // 100ms

    // 1. Disable comparator before configuring
    systimer
        .conf()
        .modify(|_, w| w.target1_work_en().clear_bit());

    // 2. Disable broken Period Mode & bind Target 1 to Unit 0
    systimer
        .target1_conf()
        .modify(|_, w| w.period_mode().clear_bit().timer_unit_sel().clear_bit());

    // 3. Calculate initial absolute target based on Unit 0's actual time
    let target = systimer_now_ticks_unit0() + period_ticks;

    // 4. Set the target registers
    systimer
        .trgt(1)
        .hi()
        .write(|w| unsafe { w.bits((target >> 32) as u32) });
    systimer
        .trgt(1)
        .lo()
        .write(|w| unsafe { w.bits(target as u32) });
    systimer.comp1_load().write(|w| w.load().set_bit());

    // 5. Clear garbage state and enable interrupt
    systimer.int_clr().write(|w| w.target1().bit(true));
    systimer.int_ena().modify(|_, w| w.target1().set_bit());

    // 6. Turn comparator back on
    systimer.conf().modify(|_, w| w.target1_work_en().set_bit());

    // 7. Route to CPU
    // crate::arch::trap::route_interrupt(HwInterrupt::SYSTIMER_TARGET1, SYSTIMER_CPU_LINE, 1);
}

pub fn systimer_clear_interrupt() {
    let systimer = unsafe { esp32c6::SYSTIMER::steal() };
    let intpri = unsafe { esp32c6::INTPRI::steal() };

    // 1. Clear peripheral
    systimer.int_clr().write(|w| w.target1().bit(true));

    // 2. CRITICAL: Clear the interrupt controller line 11 (or 16!)
    // If you don't do this, the CPU thinks the interrupt is still active
    intpri
        .cpu_int_clear()
        .write(|w| unsafe { w.bits(1 << SYSTIMER_CPU_LINE) });

    // 3. Re-arm the target
    let next_target = systimer_now_ticks_unit0() + 1_600_000;
    systimer
        .trgt(1)
        .hi()
        .write(|w| unsafe { w.bits((next_target >> 32) as u32) });
    systimer
        .trgt(1)
        .lo()
        .write(|w| unsafe { w.bits(next_target as u32) });
    systimer.comp1_load().write(|w| w.load().set_bit());
}
