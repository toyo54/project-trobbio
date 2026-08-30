#![no_std]
#![no_main]

use project_trobbio::{
    Kernel,
    arch::{
        sched::{self, Priority, TaskBuilder},
        trap::{self, InterruptId},
    },
    instrumentation,
};

// Raw TSENS thresholds for the eco run. Defaults in sched.rs are 110 /
// 140; lowered here so the passive integer workload can actually reach
// them within a ~5-minute run. Tune after the baseline run tells us what
// tsens_raw peaks at on this specific chip.
const EXPERIMENT_WARM: u8 = 95;
const EXPERIMENT_HOT: u8 = 115;

fn on_other_interrupt() {
    project_trobbio::warning!("[irq] unexpected local interrupt fired");
}

fn on_exception(mcause: usize) {
    project_trobbio::error!("[exception] mcause = 0x{:08x}", mcause);
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    // Build the kernel manually rather than through boot_scheduled!,
    // because that macro hard-codes .tsens() and would break the baseline
    // build (--no-default-features).
    let builder = Kernel::builder()
        .watchdog_disabled()
        .uart(115200)
        .scheduler()
        .periodic_timer_ms(1);

    #[cfg(feature = "tsens")]
    let builder = builder.tsens();

    let kernel = builder.build();

    // Runtime threshold tuning. Callable regardless of the tsens feature;
    // has no effect on baseline runs (thermal_state() returns Cool).
    sched::set_warm_threshold(EXPERIMENT_WARM);
    sched::set_hot_threshold(EXPERIMENT_HOT);

    // Timer callback: track jitter, nothing else. The unconditional
    // watchdog feed happens inside trap::handle_machine_timer before this
    // registered callback runs, so overriding it here is safe.
    trap::register(InterruptId::MachineTimer, instrumentation::on_tick);
    trap::register(InterruptId::UserSoftware, on_other_interrupt);
    trap::register(InterruptId::MachineSoftware, on_other_interrupt);
    trap::register(InterruptId::UserTimer, on_other_interrupt);
    trap::set_exception_handler(on_exception);

    // Three workload tasks. Same body, differing only by which counter
    // they bump. Their relative growth rates are the whole experiment.
    TaskBuilder::new(instrumentation::task_low)
        .priority(Priority::Low)
        .spawn()
        .expect("spawn low");
    TaskBuilder::new(instrumentation::task_normal)
        .priority(Priority::Normal)
        .spawn()
        .expect("spawn normal");
    TaskBuilder::new(instrumentation::task_high)
        .priority(Priority::High)
        .spawn()
        .expect("spawn high");

    kernel.enable_interrupts();

    // Deliberately skip the boot log — it would land above the CSV header
    // and confuse the plot script. The header itself is the first line.
    instrumentation::print_header();

    // Main is task 0 (always eligible per sched::eligible) so this loop
    // keeps printing even under Hot, when only the High workload also
    // gets CPU. Every 500 ms we emit one CSV row.
    loop {
        instrumentation::print_sample();
        project_trobbio::hal::timer::delay_ms(500);
    }
}
