# project-trobbio

A small, from-scratch kernel for the **ESP32-C6** (RISC-V, `riscv32imac`), written as a bachelor's thesis project. It boots bare-metal (no ESP-IDF, no `esp-hal`), drives its own trap/interrupt handling in hand-written RISC-V assembly, and implements a preemptive round-robin scheduler with a novel twist: **eco-scheduling** — task eligibility is gated by live on-die temperature (TSENS), so lower-priority tasks are throttled under thermal load instead of running the chip flat-out regardless of heat.

This is a didactic project, not a general-purpose embedded OS. It targets exactly one chip on purpose (see [Scope](#scope)).

## Contents

- [Scope](#scope)
- [Project structure](#project-structure)
- [Getting started](#getting-started)
- [Feature flags](#feature-flags)
- [Two ways to boot: `boot_basic!` vs `boot_scheduled!`](#two-ways-to-boot)
- [Tuning the eco-scheduler thresholds](#tuning-the-eco-scheduler-thresholds)
- [Using this as a dependency](#using-this-as-a-dependency)
- [Architecture, briefly](#architecture-briefly)
- [Stability note](#stability-note)
- [License](#license)

## Scope

- **ESP32-C6 only.** No board abstraction, no other chip support — this is intentional, not a limitation to be fixed later.
- **Single-hart.** The C6 has one RISC-V hart and this kernel assumes exactly that (`riscv` crate's `critical-section-single-hart` feature).
- **Demonstrative.** The goal is to make the scheduling mechanism (and the thermal-aware eco-scheduling idea in particular) legible and reproducible, not to compete with production RTOSes.

## Project structure

```

project-trobbio/
├── Cargo.toml
├── Cargo.lock
├── LICENSE
├── build.rs              # embeds src/link.x, exposes it via a link-search path
├── rust-analyzer.toml
└── src/
    ├── lib.rs             # Kernel/KernelBuilder, boot_basic!/boot_scheduled! macros
    ├── main.rs            # demo entry point — clone-and-edit starting point
    ├── boot.s             # _start, vector table, trap entry/exit (asm)
    ├── link.x             # linker script (memory layout, sections)
    ├── arch/
    │   ├── mod.rs
    │   ├── trap.rs        # CLINT/interrupt/exception dispatch
    │   └── sched.rs        # preemptive scheduler + eco-scheduling thresholds
    ├── drivers/
    │   ├── mod.rs
    │   └── ws2812.rs       # onboard RGB LED, bit-banged
    └── hal/
        ├── mod.rs
        ├── gpio.rs
        ├── timer.rs         # SYSTIMER-based delays
        ├── tsens.rs          # on-die temperature sensor
        ├── uart.rs            # UART0, reserved as the debug console
        ├── uart1.rs            # UART1, general-purpose
        └── watchdog.rs

```

## Getting started

### Prerequisites

```

rustup target add riscv32imac-unknown-none-elf
cargo install espflash

```

### Build

```

git clone <this repo>
cd project-trobbio
cargo build --release

```

`main.rs` uses the LED, TSENS, and UART1 drivers directly, so a plain `cargo build` (default features) is what you want for the demo to compile out of the box — see [Feature flags](#feature-flags) for trimming it down.

### Flash

```

espflash flash --release target/riscv32imac-unknown-none-elf/release/project-trobbio
espflash monitor

```

You should see colored `[INFO]`/`[DEBUG]`/`[WARN]`/`[ERROR]` log lines over UART0 (115200 8N1) as the kernel boots, spawns tasks, and runs the UART1 loopback demo.

## Feature flags

| Feature                 | Default | Effect                                                              |
|--------------------------|:-------:|----------------------------------------------------------------------|
| `default-panic-handler`  | on      | Registers this crate's `#[panic_handler]` (reports over UART0, halts with red LED). Disable if you want to supply your own. |
| `tsens`                  | on      | Compiles the TSENS driver. Required for real eco-scheduling throttling — without it, `sched` still runs but never throttles. |
| `led`                    | on      | Compiles the WS2812 driver (`drivers::ws2812`).                     |
| `uart1`                  | on      | Compiles the second UART (`hal::uart1`).                            |

All four are **on by default** so a fresh clone builds `main.rs` as-is. Trim with `--no-default-features --features <...>` if you're writing your own minimal `main.rs` (plain GPIO + console, no LED/TSENS/UART1). Note the scheduler itself (`arch::sched`) is **always compiled** — whether it actually runs is a runtime choice, not a feature (see below), so there's no `sched` feature flag.

## Two ways to boot

`lib.rs` provides two macros covering the common configurations; both wrap `Kernel::builder()` under the hood and you can always drop to the builder directly for anything in between.

**`boot_basic!()`** — watchdog off, UART0 console up, nothing else. No scheduler, no periodic tick. Good starting point for plain GPIO + console experiments:

```rust
let _kernel = project_trobbio::boot_basic!();
```

**`boot_scheduled!(...)`** — watchdog off, UART0 up, TSENS up, scheduler on, periodic tick armed, handlers registered, tasks spawned, interrupts enabled — the full eco-scheduling demo in one call:

```rust
let _kernel = project_trobbio::boot_scheduled!(
    timer: on_machine_timer,
    other: on_other_interrupt,
    exception: on_exception,
    tasks: [task1 => sched::Priority::Low, task2]
);
```

Handler registration always happens *before* `kernel.enable_interrupts()` is called internally — this ordering is load-bearing (a handler registered after interrupts are live may miss the first tick) and the macro enforces it so you don't have to think about it.

## Tuning the eco-scheduler thresholds

`sched::warm_threshold()` / `sched::hot_threshold()` are raw TSENS-code cutoffs (not calibrated °C — see `hal/tsens.rs` for why) that gate task eligibility. They're runtime-adjustable via atomics, no `unsafe` required:

```rust
sched::set_warm_threshold(120);
sched::set_hot_threshold(150);
```

Call this before or after `boot_scheduled!()` — thresholds take effect on the next tick regardless.

## Using this as a dependency

You can either clone this repo directly (above), or depend on it as a library crate:

```toml
[dependencies]
project-trobbio = "0.1"
```

**One manual step is required** in your own project's `build.rs`:

```rust
// build.rs
fn main() {
    println!("cargo:rustc-link-arg=-Tlink.x");
}
```

This is unavoidable: Cargo does not propagate a dependency's `cargo:rustc-link-arg` output to the dependent's own link step. You do **not** need to copy `link.x` itself — this crate's own `build.rs` embeds it and exposes it via a propagating `cargo:rustc-link-search`, so the linker finds it by name automatically.

Your `main.rs` then looks like:

```rust
#![no_std]
#![no_main]

use project_trobbio::{boot_basic, Kernel};

#[unsafe(no_mangle)]
pub extern "C" fn main() -> ! {
    let _kernel = boot_basic!();
    loop { /* your app */ }
}
```

## Architecture, briefly

- **Boot** (`boot.s`): sets up the stack, `mtvec`, `gp`, zeroes `.bss`, copies `.data` from flash, jumps to `main`.
- **Trap handling** (`arch/trap.rs`): the *only* module touching `mcause`/`mie`/`mstatus`/`mtvec`/CLINT directly. Everything else goes through `trap::register()`. MTIME (the periodic tick) is permanently reserved to feed the watchdog, independent of whatever the app registers on it.
- **Scheduling** (`arch/sched.rs`): per-task stacks with a stack-canary overflow check, priority levels (`Low`/`Normal`/`High`), and eco-scheduling — task eligibility is filtered by `ThermalState` (`Cool`/`Warm`/`Hot`) derived from live TSENS readings, so thermal pressure degrades gracefully by priority instead of throttling uniformly.
- **Fault handling** (`arch/trap.rs`, bottom half): a dedicated emergency stack and a one-shot reentrancy guard so a fault *while reporting a fault* doesn't loop forever — reports over UART0, halts with the status LED red.

## Stability note

This is a thesis artifact, not a stability-guaranteed library. The public API (`Kernel`, `KernelBuilder`, `sched`, `trap`, driver modules) may change between `0.x` versions without a major-version-style guarantee. Pin an exact version if you depend on it for something that needs to keep working.

## License

BSD 2-Clause — see [`LICENSE`](./LICENSE).
