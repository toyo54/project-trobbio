use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::hal;
use crate::hal::uart::Uart0;

// ---------- task progress counters -------------------------------------

pub static CNT_LOW: AtomicU32 = AtomicU32::new(0);
pub static CNT_NORMAL: AtomicU32 = AtomicU32::new(0);
pub static CNT_HIGH: AtomicU32 = AtomicU32::new(0);

/// The one and only workload body. `#[inline(never)]` guarantees the
/// compiler emits exactly ONE copy of this hot loop in the binary, so all
/// three tasks execute the identical instructions at the identical flash
/// address. That removes the per-function code-placement / cache-line
/// effect that otherwise lets three "identical" loops run at slightly
/// different speeds — which is what made the Low task look 25% faster in
/// the first baseline. With a single shared body, any difference in the
/// counters reflects only how the scheduler shared CPU time, which is the
/// whole point of the fairness measurement.
///
/// Takes the accumulator by reference and black_box'es it so LLVM can't
/// hoist, fold, or specialise the loop per call site.
#[inline(never)]
fn burn(acc: &mut u32) {
    for _ in 0..1_000 {
        *acc = acc.wrapping_mul(2_654_435_761);
        core::hint::black_box(&mut *acc);
    }
}

/// Body for the "Low"-priority task. Identical to the Normal and High
/// bodies except for which counter it bumps: all three call the same
/// `burn()`, so the per-iteration cost is identical across them.
pub fn task_low() {
    let mut acc: u32 = 0x1234_5678;
    loop {
        burn(&mut acc);
        CNT_LOW.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn task_normal() {
    let mut acc: u32 = 0x1234_5678;
    loop {
        burn(&mut acc);
        CNT_NORMAL.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn task_high() {
    let mut acc: u32 = 0x1234_5678;
    loop {
        burn(&mut acc);
        CNT_HIGH.fetch_add(1, Ordering::Relaxed);
    }
}

// ---------- tick-jitter tracker (u32 only) -----------------------------

static LAST_TICK_US: AtomicU32 = AtomicU32::new(0);
static LAST_DELTA_US: AtomicU32 = AtomicU32::new(0);
static MIN_DELTA_US: AtomicU32 = AtomicU32::new(u32::MAX);
static MAX_DELTA_US: AtomicU32 = AtomicU32::new(0);
static TICK_COUNT: AtomicU32 = AtomicU32::new(0);

/// Machine-timer callback. Register via
/// `arch::trap::register(InterruptId::MachineTimer, on_tick)`.
/// Runs from inside the trap handler: no allocation, no formatting, no
/// UART I/O — only atomic updates.
pub fn on_tick() {
    // Truncate the 64-bit systimer to 32 bits. Wraps every ~71 min, which
    // is far above any single run, and delta computation uses wrapping_sub
    // so a wrap between two consecutive samples still yields the correct
    // short delta.
    let now = hal::timer::systimer_now_us() as u32;
    let prev = LAST_TICK_US.swap(now, Ordering::Relaxed);
    let count = TICK_COUNT.fetch_add(1, Ordering::Relaxed);

    // Skip the very first tick — no previous timestamp to subtract from.
    if count == 0 || prev == 0 {
        return;
    }

    let delta = now.wrapping_sub(prev);
    LAST_DELTA_US.store(delta, Ordering::Relaxed);

    // CAS loop for min.
    let mut cur = MIN_DELTA_US.load(Ordering::Relaxed);
    while delta < cur {
        match MIN_DELTA_US.compare_exchange_weak(cur, delta, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(observed) => cur = observed,
        }
    }

    // CAS loop for max.
    let mut cur = MAX_DELTA_US.load(Ordering::Relaxed);
    while delta > cur {
        match MAX_DELTA_US.compare_exchange_weak(cur, delta, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(observed) => cur = observed,
        }
    }
}

// ---------- CSV logger -------------------------------------------------

/// Prints the CSV header exactly once, right after boot.
pub fn print_header() {
    // No ANSI prefix — this must stay parseable as pure CSV.
    let _ = writeln!(
        Uart0,
        "t_ms,tsens_raw,cnt_low,cnt_normal,cnt_high,\
         tick_last_us,tick_min_us,tick_max_us"
    );
}

/// One CSV sample line. Call from a task context (safe to writeln! and to
/// read the sensor), not from the trap handler.
pub fn print_sample() {
    let t_ms = hal::timer::systimer_now_ms();
    let tsens = read_tsens();
    let cl = CNT_LOW.load(Ordering::Relaxed);
    let cn = CNT_NORMAL.load(Ordering::Relaxed);
    let ch = CNT_HIGH.load(Ordering::Relaxed);
    let last = LAST_DELTA_US.load(Ordering::Relaxed);
    let mn = MIN_DELTA_US.load(Ordering::Relaxed);
    let mx = MAX_DELTA_US.load(Ordering::Relaxed);

    let _ = writeln!(
        Uart0,
        "{},{},{},{},{},{},{},{}",
        t_ms, tsens, cl, cn, ch, last, mn, mx
    );
}

#[cfg(feature = "tsens")]
fn read_tsens() -> u16 {
    hal::tsens::read_raw() as u16
}

#[cfg(not(feature = "tsens"))]
fn read_tsens() -> u16 {
    // No sensor compiled in: still emit a column so the CSV shape stays
    // identical across baseline and eco runs. The plot script treats a
    // constant zero column as "no thermal data" and skips the overlay.
    0
}
