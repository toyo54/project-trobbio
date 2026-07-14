use esp32c6::{LP_WDT, TIMG0};

// The magic key to unlock ESP32 watchdogs
const WDT_WKEY: u32 = 0x50D8_3AA1;

/// This function disables the watchdogs
/// Must be used only for for debugging or absolutely critical purposes
pub fn disable_lp_watchdog() {
    let lp_wdt = unsafe { LP_WDT::steal() };

    // 1. Unlock and Disable LP_WDT (Low Power Watchdog)
    lp_wdt
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    lp_wdt.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
    lp_wdt
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) }); // Relock
}

pub fn disable_hp_watchdog() {
    let wdt = unsafe { TIMG0::steal() };

    wdt.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    wdt.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
    wdt.wdtwprotect().write(|w| unsafe { w.wdt_wkey().bits(0) }); // Relock
}

/// This function enables the TIMG0 watchdog
///
/// When time is over a reset happens
pub fn enable_timg0() {
    let timg0 = unsafe { TIMG0::steal() };

    // unlock
    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });

    // 100 ms feed interval
    timg0.wdtconfig1().write(|w| unsafe { w.bits(160_000_000) });

    // configure it to reset the system
    timg0.wdtconfig0().modify(|_, w| unsafe {
        w.wdt_stg0()
            .bits(3)
            .wdt_en()
            .set_bit()
            .wdt_flashboot_mod_en()
            .clear_bit()
    });

    /* immediate feeding */
    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) }); // unlock
    timg0.wdtfeed().write(|w| unsafe { w.wdt_feed().bits(1) }); // feed
    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) }); // relock
}

/// Feeds the watchdogs
///
/// This function is meant to be called only by the systimer interrupt routine
pub fn feed_watchdog() {
    let timg0 = unsafe { TIMG0::steal() };

    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) }); // unlock

    // Explicitly write true to the feed bit
    timg0.wdtfeed().write(|w| unsafe { w.wdt_feed().bits(1) });

    timg0
        .wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) }); // relock
}
