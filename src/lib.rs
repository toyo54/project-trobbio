#![no_std]
#![allow(warnings)]

use core::arch::global_asm;
use core::panic::PanicInfo;

pub mod arch;
pub mod drivers;
pub mod hal;
pub mod instrumentation;

global_asm!(include_str!("boot.s"));

#[cfg(feature = "default-panic-handler")]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    extern "C" fn log_panic(info_ptr: usize) {
        // Safety: report_fatal_and_halt only ever calls this with the
        // address of the PanicInfo passed in below, still alive for the
        // duration of the call.
        let info = unsafe { &*(info_ptr as *const PanicInfo) };
        crate::error!("PANIC: {}", info);
    }
    arch::trap::report_fatal_and_halt(log_panic, _info as *const PanicInfo as usize)
}

/// Peripheral bring-up, in dependency order. Deliberately does NOT touch
/// interrupts — `enable_interrupts()` is a separate, explicit step because
/// it must run *after* the caller has registered handlers via
/// `trap::register`/`trap::set_exception_handler`. Folding it into
/// `build()` would make that ordering invisible and easy to get wrong.
pub struct Kernel {
    periodic_timer_ticks: Option<u64>,
}

impl Kernel {
    pub fn builder() -> KernelBuilder {
        KernelBuilder::default()
    }

    /// Arms the periodic tick (if configured via `.periodic_timer(...)`)
    /// and globally enables interrupts. Call this AFTER
    /// `trap::register`/`trap::set_exception_handler`/task spawning —
    /// anything registered later won't be live for ticks that already fired.
    pub fn enable_interrupts(&self) {
        if let Some(ticks) = self.periodic_timer_ticks {
            arch::trap::init_periodic_timer(ticks);
        }
    }
}

#[derive(Default)]
pub struct KernelBuilder {
    watchdog_disabled: bool,
    uart_baud: Option<u32>,
    periodic_timer_ticks: Option<u64>,
    scheduler: bool,
    #[cfg(feature = "tsens")]
    tsens: bool,
}

/// Minimum sane period, in CLINT ticks (16 MHz). Not a hard hardware
/// limit — it's a guard against unit confusion (raw ticks vs. seconds),
/// which is the actual failure mode this catches: a period this short
/// guarantees the ISR can't return before the next tick is already due,
/// permanently starving every task of CPU time. 1600 ticks = 100µs is
/// still aggressive but leaves room for a trap round-trip plus a short
/// handler; tune down only if you've profiled your specific handlers.
const MIN_PERIOD_TICKS: u64 = 1600;

impl KernelBuilder {
    pub fn watchdog_disabled(mut self) -> Self {
        self.watchdog_disabled = true;
        self
    }

    pub fn uart(mut self, baud: u32) -> Self {
        self.uart_baud = Some(baud);
        self
    }

    /// Opts into the preemptive round-robin scheduler (`arch::sched`).
    /// Runtime toggle, not a compile-time feature: `arch::sched` is cheap
    /// enough to always compile in, and gating a single dispatch branch
    /// behind `cfg` bought nothing but an extra build axis.
    pub fn scheduler(mut self) -> Self {
        self.scheduler = true;
        self
    }

    #[cfg(feature = "tsens")]
    pub fn tsens(mut self) -> Self {
        self.tsens = true;
        self
    }

    /// sets the periodic timer in tick (each tick lasts 6.25 * 10^-8 seconds)
    pub fn periodic_timer(mut self, ticks: u64) -> Self {
        assert!(
            ticks >= MIN_PERIOD_TICKS,
            "periodic_timer({ticks}) is too short (min {MIN_PERIOD_TICKS} ticks == {}µs at 16MHz). \
         Did you mean {ticks}_000_000 for {ticks} seconds?",
            ticks
        );
        self.periodic_timer_ticks = Some(ticks);
        self
    }

    /// Convenience wrapper: period expressed in milliseconds instead of raw
    /// CLINT ticks, removing the unit-confusion failure mode entirely for
    /// the common case. `1000` here means 1 second, unambiguously.
    pub fn periodic_timer_ms(self, ms: u64) -> Self {
        self.periodic_timer(ms * (crate::arch::trap::CLINT_HZ / 1000))
    }
    /// Applies bring-up effects in a fixed order and returns the running
    /// `Kernel`. Does NOT touch interrupts — see `Kernel::enable_interrupts`.
    pub fn build(self) -> Kernel {
        if self.watchdog_disabled {
            hal::watchdog::disable_lp_watchdog();
        }
        hal::timer::systimer_enable();
        if let Some(baud) = self.uart_baud {
            hal::uart::init(baud);
        }
        #[cfg(feature = "tsens")]
        if self.tsens {
            hal::tsens::init();
        }

        if self.scheduler {
            arch::sched::init();
        }

        arch::trap::set_sched_enabled(self.scheduler);

        Kernel {
            periodic_timer_ticks: self.periodic_timer_ticks,
        }
    }
}

/// Minimal boot: watchdog off, UART console up, nothing else.
/// No scheduler, no periodic tick beyond whatever's already reserved.
/// Good starting point for plain GPIO + console experiments.
#[macro_export]
macro_rules! boot_basic {
    () => {
        $crate::Kernel::builder()
            .watchdog_disabled()
            .uart(115200)
            .build()
    };
}

/// Full preemptive-scheduling boot: tsens + eco-scheduler + periodic tick,
/// registers the given handlers, spawns the given tasks, then goes live.
///
/// Usage:
/// ```ignore
/// let kernel = boot_scheduled!(
///     timer: on_machine_timer,
///     other: on_other_interrupt,
///     exception: on_exception,
///     tasks: [task1 => sched::Priority::Low, task2]
/// );
/// ```
#[macro_export]
macro_rules! boot_scheduled {
    (
        timer: $timer:expr,
        other: $other:expr,
        exception: $exc:expr,
        tasks: [$($task:expr $(=> $prio:expr)?),* $(,)?]
    ) => {{
        use $crate::arch::trap::{self, InterruptId};

        let kernel = $crate::Kernel::builder()
            .watchdog_disabled()
            .uart(115200)
            .tsens()
            .scheduler()
            .periodic_timer(16_000_000)
            .build();


        trap::register(InterruptId::MachineTimer, $timer);
        trap::register(InterruptId::UserSoftware, $other);
        trap::register(InterruptId::MachineSoftware, $other);
        trap::register(InterruptId::UserTimer, $other);
        trap::set_exception_handler($exc);

        $(
            let mut builder = $crate::arch::sched::TaskBuilder::new($task);
            $( builder = builder.priority($prio); )?
            builder.spawn().unwrap();
        )*

        kernel.enable_interrupts();
        kernel
    }};
}
