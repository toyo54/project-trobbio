//! M-mode trap / CLINT abstraction.
//!
//! This is the *only* place that touches mcause/mie/mstatus/mtvec directly.
//! Everything above this module (drivers, `main`, whatever else the "OS"
//! grows into) only ever calls `register()` and `init_periodic_timer()` —
//! no CSRs, no addresses, no knowledge that a CLINT even exists.
//!
//! MTIME (the machine-timer interrupt, ID 7) is reserved by the kernel for
//! feeding the watchdog — see the `feed_watchdog()` call inside
//! `handle_machine_timer`. That feed is unconditional and independent of
//! whatever app code registers on `InterruptId::MachineTimer`; the whole
//! point of reserving this tick is that the watchdog can't be starved just
//! because some app-level handler forgot to feed it, panicked, or was never
//! wired up.
//!
//! Whether this tick ALSO drives scheduling is a runtime decision, made via
//! `KernelBuilder::scheduler()` -> `set_sched_enabled()`. When disabled, the
//! tick still fires, still feeds the watchdog, still runs whatever's
//! registered on `MachineTimer` — it just returns the same `sp` it was
//! given (a null switch), same as any other interrupt.
//!
//! ## Exceptions: fatal vs. voluntary yield
//! Almost every exception on this chip is treated as fatal and routed to
//! `report_fatal_and_halt` — illegal instructions, misaligned/faulting
//! accesses, all of it. The single deliberate exception to that rule is
//! `ECALL_FROM_M` (M-mode `ecall`), which `yield_now()` triggers on purpose
//! to force an immediate, synchronous reschedule instead of waiting for the
//! next timer tick. This is what makes task exit (`sched::task_exited`)
//! real round-robin instead of a task idling away the rest of its slice in
//! `wfi`. Every other exception code still halts exactly as before — this
//! is a narrow, explicit carve-out, not a general loosening of the fault
//! path.
//!
//! ## PAC vs. raw addresses
//! CLINT peripheral access goes through `esp32c6::CLINT` (confirmed against
//! esp32c6 v0.23.2: `CLINT::PTR == 0x2000_0000`, matching TRM Table 1.4-1's
//! "CPU Sub-system region", with `MTIMECTL`/`MTIME`/`MTIMECMP` at the same
//! 0x1804/0x1808/0x1810 offsets TRM 1.7.5/1.7.6 gives). `MTIMECTL` uses the
//! PAC's normal field accessors.
//!
//! `MTIME`/`MTIMECMP` are the exception: the PAC models both as single
//! 64-bit registers, but RV32 has no atomic 64-bit load/store, so a `u64`
//! volatile access there quietly becomes two separate 32-bit accesses with
//! no ordering guarantee — a read can tear if the low word wraps mid-read
//! (~every 268s at 16MHz), and a naive write can briefly present a
//! mismatched (old-hi, new-lo) pair, risking a spurious early compare
//! match. Both registers are still accessed at 32-bit granularity here
//! (hi/lo, with the retry-on-read and write-hi-as-guard-then-lo-then-hi
//! patterns), just anchored to `CLINT::PTR` instead of a hardcoded literal.
//!
//! ## riscv crate vs. raw asm
//! `mcause` decoding goes through `riscv::register::mcause` — unlike CLINT,
//! `mcause`'s layout (interrupt bit + exception code) is defined by the
//! standard RISC-V privileged ISA, not vendor-specific, so there's no
//! address-verification risk the way there was for CLINT; using the crate
//! here is a pure readability win, confirmed against the exact crate
//! version pinned in Cargo.lock. `mtvec` and the exception path's MPIE
//! clear stay as raw `csrw`/`csrc` asm: `mtvec` needs a raw address
//! computed from a linker symbol, and this version of the `riscv` crate
//! only generates a *setter* for MPIE (`set_mpie`), not a clearer — there's
//! no equivalent convenience function to swap in for that one line.

use crate::drivers::ws2812::RgbLed;
use crate::hal::gpio::{GpioFunction, GpioPin};
use crate::hal::uart;
use crate::hal::watchdog::feed_watchdog;
use core::arch::asm;
use esp32c6::CLINT;

const MTIME_LO_OFFSET: usize = 0x1808;
const MTIME_HI_OFFSET: usize = 0x180C;
const MTIMECMP_LO_OFFSET: usize = 0x1810;
const MTIMECMP_HI_OFFSET: usize = 0x1814;

// NOTE: On ESP32-C6, the CPU Timer (MTIME) runs at the CPU core frequency,
// not the 16MHz SYSTIMER frequency. This is typically 40MHz at boot, or 160MHz if PLL is enabled.
pub const CLINT_HZ: u64 = 40_000_000; // Updated to reflect typical boot speed

#[inline(always)]
fn clint_reg(offset: usize) -> *mut u32 {
    (CLINT::PTR as *mut u8).wrapping_add(offset) as *mut u32
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum InterruptId {
    UserSoftware = 0,
    MachineSoftware = 3,
    UserTimer = 4,
    MachineTimer = 7,
}

const NUM_IDS: usize = 32;
pub type Handler = fn();
pub type ExceptionHandler = fn(mcause: usize);

static mut HANDLERS: [Option<Handler>; NUM_IDS] = [None; NUM_IDS];
static mut EXCEPTION_HANDLER: Option<ExceptionHandler> = None;

// Not an Atomic: AtomicU64 isn't available on riscv32imac (the RV32 'A'
// extension only has 32-bit atomics). Safe as a plain static mut because
// it's written once in init_periodic_timer() before interrupts are ever
// enabled, and only read from trap context afterward, which can't itself
// be preempted (mstatus.MIE is hardware-cleared for the trap's duration).
static mut PERIOD_TICKS: u64 = 0;

/// M-mode ECALL exception code (RISC-V privileged spec, mcause value when
/// mcause.is_interrupt() is false). This is the exception `yield_now()`
/// deliberately triggers to force an immediate, synchronous reschedule —
/// the difference between "real" round-robin and one that wastes the rest
/// of a dead task's time slice sitting in `wfi`.
const ECALL_FROM_M: usize = 11;

/// Whether `handle_machine_timer` should hand off to `arch::sched::next_sp`.
/// Set once by `KernelBuilder::build()` via `set_sched_enabled()`, based on
/// whether `.scheduler()` was called — a plain runtime flag, not a `cfg`,
/// since `arch::sched` is always compiled in regardless.
static mut SCHED_ENABLED: bool = false;

/// Called by `Kernel::builder()...build()`. Not meant to be called directly
/// by application code.
pub(crate) fn set_sched_enabled(enabled: bool) {
    unsafe {
        SCHED_ENABLED = enabled;
    }
}

pub fn critical_section<R>(f: impl FnOnce() -> R) -> R {
    let saved_mstatus: usize;
    unsafe {
        asm!("csrr {0}, mstatus", out(reg) saved_mstatus);
        asm!("csrc mstatus, {0}", in(reg) 0x8_usize); // MIE = 0
    }

    let result = f();

    unsafe {
        asm!("fence");
        if saved_mstatus & 0x8 != 0 {
            asm!("csrs mstatus, {0}", in(reg) 0x8_usize); // restore MIE
        }
    }

    result
}

/// Registers `handler` to run whenever local interrupt `id` fires. Runs in
/// trap context with mstatus.MIE hardware-cleared (no other interrupt can
/// preempt it), so keep it short.
pub fn register(id: InterruptId, handler: Handler) {
    critical_section(|| unsafe {
        HANDLERS[id as usize] = Some(handler);
    });
}

/// Registers a handler for anything that traps as an exception
/// (`mcause` with the interrupt bit clear). Defaults to clearing MPIE if
/// nothing is registered.
pub fn set_exception_handler(handler: ExceptionHandler) {
    critical_section(|| unsafe {
        EXCEPTION_HANDLER = Some(handler);
    });
}

/// Reads the free-running 64-bit CLINT timer (16 MHz).
pub fn now() -> u64 {
    unsafe {
        loop {
            let hi1 = core::ptr::read_volatile(clint_reg(MTIME_HI_OFFSET));
            let lo = core::ptr::read_volatile(clint_reg(MTIME_LO_OFFSET));
            let hi2 = core::ptr::read_volatile(clint_reg(MTIME_HI_OFFSET));
            if hi1 == hi2 {
                return ((hi1 as u64) << 32) | (lo as u64);
            }
        }
    }
}

fn set_next_tick_at(target: u64) {
    unsafe {
        core::ptr::write_volatile(clint_reg(MTIMECMP_HI_OFFSET), 0xFFFF_FFFF);
        core::ptr::write_volatile(clint_reg(MTIMECMP_LO_OFFSET), target as u32);
        core::ptr::write_volatile(clint_reg(MTIMECMP_HI_OFFSET), (target >> 32) as u32);
        asm!("fence");
    }
}

/// Arms a periodic machine-timer tick every `period_ticks` CLINT ticks
/// (16 MHz clock, so e.g. 16_000_000 == 1 second) and globally enables
/// interrupts. Call once from `main`.
///
/// This tick doubles as the watchdog-feed cadence (see the module docs) —
/// if you later enable the real hardware watchdog via
/// `hal::watchdog::enable_timg0()`, make sure `period_ticks` here is
/// comfortably shorter than whatever timeout you configure there.
///
/// The tick is auto re-armed on every fire — a registered `MachineTimer`
/// handler doesn't need to touch mtimecmp itself, and doesn't need to feed
/// the watchdog either; that part is guaranteed by the kernel already.
pub fn init_periodic_timer(period_ticks: u64) {
    unsafe {
        PERIOD_TICKS = period_ticks;
    }
    set_next_tick_at(now() + period_ticks);

    let clint = unsafe { CLINT::steal() };
    clint
        .mtimectl()
        .write(|w| w.mtce().set_bit().mtie().set_bit());

    unsafe {
        asm!("fence");
        asm!("csrs mie, {0}", in(reg) 0x80_usize);
        asm!("csrs mstatus, {0}", in(reg) 0x8_usize);
    }
}

/// Called from the assembly trap entry (`boot.s`) on every trap. `sp` is
/// the just-saved current task's stack pointer; the return value is what
/// `boot.s` loads back into `sp` before restoring registers and `mret`ing
/// — returning the same value it was given is a null switch (Stage 1);
/// returning something else is a real context switch (Stage 2).
#[unsafe(no_mangle)]
pub extern "C" fn rust_trap_handler(sp: usize) -> usize {
    let mcause = riscv::register::mcause::read();
    match mcause.is_interrupt() {
        true => dispatch_interrupt(mcause.code(), sp),
        false => dispatch_exception(mcause.bits(), sp),
    }
}

/// Routes a local interrupt ID to its handler. `MachineTimer` gets the
/// kernel's own reserved handling first; everything else goes straight to
/// whatever's registered (and is a null switch — only the reserved tick
/// drives scheduling right now).
fn dispatch_interrupt(id: usize, sp: usize) -> usize {
    match id {
        id if id == InterruptId::MachineTimer as usize => handle_machine_timer(sp),
        id => {
            call_registered(id);
            sp
        }
    }
}

/// The reserved machine-timer tick: rearm the next fire, feed the watchdog
/// unconditionally, let app code piggyback via its own registered handler,
/// then — only if `.scheduler()` was requested at boot — ask the scheduler
/// what should run next. Otherwise this is a null switch, same as any
/// other interrupt.
fn handle_machine_timer(sp: usize) -> usize {
    let period = unsafe { PERIOD_TICKS };
    if period != 0 {
        set_next_tick_at(now() + period);
    }
    feed_watchdog();
    call_registered(InterruptId::MachineTimer as usize);

    if unsafe { SCHED_ENABLED } {
        crate::arch::sched::next_sp(sp)
    } else {
        sp
    }
}

/// Looks up and runs whatever's registered for `id`, if anything.
fn call_registered(id: usize) {
    if id > NUM_IDS {
        return;
    }
    if let Some(handler) = unsafe { HANDLERS[id] } {
        handler();
    }
}

extern "C" fn run_registered_exception_handler(cause: usize) {
    match unsafe { EXCEPTION_HANDLER } {
        Some(handler) => handler(cause),
        None => unsafe {
            asm!("csrc mstatus, {0}", in(reg) 0x80_usize);
        },
    }
}

/// Routes a caught exception. `ECALL_FROM_M` is the one deliberate,
/// non-fatal case — see the module docs — and is handled by advancing
/// `mepc` past the `ecall` and immediately asking the scheduler for the
/// next task. Every other exception code is fatal.
fn dispatch_exception(cause: usize, sp: usize) -> usize {
    if cause == ECALL_FROM_M {
        return handle_yield(sp);
    }
    report_fatal_and_halt(run_registered_exception_handler, cause)
}

/// Handles a voluntary `ecall` yield: advances the saved `mepc` past the
/// `ecall` instruction (which, unlike a normal instruction, doesn't
/// auto-advance on trap entry — skipping this would re-execute the same
/// `ecall` in an infinite loop next time this frame is resumed), then
/// hands off to the scheduler exactly like a preempting timer tick would.
fn handle_yield(sp: usize) -> usize {
    unsafe {
        // TrapFrame.mepc lives at byte offset 112 in the saved frame
        // (see the layout comment in boot.s).
        let mepc_ptr = (sp + 112) as *mut usize;
        *mepc_ptr += 4;
    }
    if unsafe { SCHED_ENABLED } {
        crate::arch::sched::next_sp(sp)
    } else {
        sp
    }
}

/// Voluntarily gives up the rest of the current time slice, immediately
/// switching to the next eligible task instead of waiting for the next
/// timer tick. Traps via `ecall`, so it goes through the exact same
/// save/restore path as a preempting interrupt — no separate context-
/// switch machinery to keep in sync. Used by `sched::task_exited` so a
/// task that finishes early hands off the CPU immediately, not after
/// idling out the rest of its slice.
pub fn yield_now() {
    unsafe { core::arch::asm!("ecall") };
}

/* ===================== Fatal-fault safety net ===================== */

const EXCEPTION_STACK_SIZE: usize = 512;

#[repr(align(16))]
struct EmergencyStack([u8; EXCEPTION_STACK_SIZE]);
static mut EMERGENCY_STACK: EmergencyStack = EmergencyStack([0; EXCEPTION_STACK_SIZE]);

static mut FAULT_IN_PROGRESS: bool = false;

const STATUS_LED_PIN: u8 = 8;

fn emergency_stack_top() -> usize {
    unsafe { (core::ptr::addr_of_mut!(EMERGENCY_STACK) as usize + EXCEPTION_STACK_SIZE) & !0xF }
}

/// Calls `f(arg)` with `sp` pointed at the emergency stack, then restores
/// the original `sp` before returning.
///
/// Deliberately a plain function pointer + `usize` argument rather than
///  a generic closure, so the sp swap has no captured environment and nothing for
///  the compiler to spill onto whichever stack happens to be live at the wrong moment.
#[inline(never)]
fn on_emergency_stack(f: extern "C" fn(usize), arg: usize) {
    let stack_top = emergency_stack_top();
    let old_sp: usize;
    unsafe {
        asm!("mv {0}, sp", out(reg) old_sp);
        asm!("mv sp, {0}", in(reg) stack_top);
    }
    f(arg);
    unsafe {
        asm!("mv sp, {0}", in(reg) old_sp);
    }
}

/// Common endpoint for both a fatal exception and a Rust panic: try once
/// (and only once) to report `f(arg)` on the emergency stack, then halt
/// forever with the status LED red and the watchdog fed. Never returns —
/// resuming the original context isn't safe to assume once we're here.
pub fn report_fatal_and_halt(f: extern "C" fn(usize), arg: usize) -> ! {
    let already_faulting = unsafe {
        let prev = FAULT_IN_PROGRESS;
        FAULT_IN_PROGRESS = true;
        prev
    };

    if !already_faulting {
        on_emergency_stack(f, arg);
    } else {
        // Reporting itself just faulted a second time — don't try
        // again, don't touch core::fmt or go through the normal driver
        // call chain. A bare fixed message is the only thing worth trying.
        let _ = uart::write_bytes(b"\r\n[FATAL] double fault while reporting, halting\r\n");
    }

    halt_forever()
}

fn halt_forever() -> ! {
    // Force sp onto the emergency stack unconditionally, even if we're
    // already on it — this must not depend on the incoming sp being
    // anything in particular.
    unsafe {
        asm!("mv sp, {0}", in(reg) emergency_stack_top());
    }

    let mut led = RgbLed::new(GpioPin::new(STATUS_LED_PIN, GpioFunction::Gpio));
    loop {
        feed_watchdog();
        led.refresh((5, 0, 0));
    }
}
