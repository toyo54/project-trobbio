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

use crate::hal::watchdog::feed_watchdog;
use core::arch::asm;
use esp32c6::CLINT;

// MTIME/MTIMECMP need 32-bit-granularity access (see module docs) that the
// PAC's u64-typed accessors don't give us. We'd like these as `const`
// pointers anchored to CLINT::PTR, but rustc's const evaluator flatly
// refuses pointer-to-integer casts at compile time ("pointers cannot be
// cast to integers during const eval") — even though CLINT::PTR's value is
// in fact fixed, the const evaluator doesn't reason about that. So the
// *offsets* are consts (plain integers, no pointer arithmetic involved),
// and the pointer itself is computed at runtime by `clint_reg()` below.
const MTIME_LO_OFFSET: usize = 0x1808;
const MTIME_HI_OFFSET: usize = 0x180C;
const MTIMECMP_LO_OFFSET: usize = 0x1810;
const MTIMECMP_HI_OFFSET: usize = 0x1814;

/// Computes a pointer to a byte offset inside the CLINT block, anchored to
/// the PAC's own `CLINT::PTR` — the address itself is still single-sourced
/// from the PAC, just resolved at runtime instead of compile time.
#[inline(always)]
fn clint_reg(offset: usize) -> *mut u32 {
    (CLINT::PTR as *mut u8).wrapping_add(offset) as *mut u32
}

/// Local CLINT interrupt IDs (TRM Table 1.7-1). These 4 are fixed-priority
/// and always enabled at the INTC level; everything else (external
/// peripheral interrupts, IDs 1-2, 5-6, 8-31) would need INTPRI setup too,
/// which isn't wired up here yet.
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

/// Runs `f` with `mstatus.MIE` forced off, restoring it to whatever it was
/// before on the way out. This is the software workaround TRM 1.6.3.1
/// recommends whenever touching Interrupt Controller (INTC/INTPRI)
/// registers specifically — they take up to 4 cycles to settle into
/// steady state, and interrupt ordering isn't guaranteed during that
/// transient window — but it's equally the right tool any time code needs
/// to touch shared state that a trap handler might also touch (like the
/// handler tables below).
///
/// Nesting-safe: only re-enables MIE on exit if it was actually on before
/// entry, so a critical_section called from inside another one won't
/// prematurely flip interrupts back on.
///
/// Exposed publicly so driver code (future INTPRI/UART setup included)
/// doesn't have to hand-roll this sequence itself.
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

/// Registers a handler for anything that traps as a genuine exception
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
/// Doesn't touch `mtvec`: `boot.s`'s `_start` already points it at
/// `_vector_table` (vectored, same computation this function used to
/// redo) before `main()` is ever reached, so setting it again here was
/// dead weight — this is now the single place that does it.
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

    // MTIMECTL is a genuinely 32-bit register with real bitfields (MTCE,
    // MTIE, ...), no torn-access risk — use the PAC normally here.
    let clint = unsafe { CLINT::steal() };
    clint
        .mtimectl()
        .write(|w| w.mtce().set_bit().mtie().set_bit());

    unsafe {
        asm!("fence");

        // mie bit 7 = MTIE, unmasks the machine timer interrupt at core level.
        asm!("csrs mie, {0}", in(reg) 0x80_usize);
        // mstatus bit 3 = MIE, the global machine-interrupt enable.
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
        false => {
            dispatch_exception(mcause.bits());
            sp
        }
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
/// unconditionally (see the module docs — this can't be skipped or
/// forgotten), let app code piggyback via its own registered handler, and
/// only then ask the scheduler what should run next.
fn handle_machine_timer(sp: usize) -> usize {
    let period = unsafe { PERIOD_TICKS };
    if period != 0 {
        set_next_tick_at(now() + period);
    }
    feed_watchdog();
    call_registered(InterruptId::MachineTimer as usize);
    crate::arch::sched::next_sp(sp)
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

/// Routes a genuine exception (mcause interrupt bit clear) to whatever's
/// registered, falling back to clearing MPIE if nothing is.
fn dispatch_exception(cause: usize) {
    match unsafe { EXCEPTION_HANDLER } {
        Some(handler) => handler(cause),
        None => unsafe {
            asm!("csrc mstatus, {0}", in(reg) 0x80_usize);
        },
    }
}
