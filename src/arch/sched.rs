//! Preemptive scheduler
//!
//! Implements per-task stack allocation, trap frame initialization,
//! round-robin scheduling gated by priority + tsens-driven throttling,
//! graceful task exit with slot reuse, and an optional builder for
//! setting priority / on-exit callbacks at spawn time.
//!
//! Always compiled in — opting into it is a runtime decision made via
//! `KernelBuilder::scheduler()`, not a Cargo feature. See `arch::trap`'s
//! `SCHED_ENABLED` flag for where that decision takes effect.

use core::sync::atomic::{AtomicU8, AtomicUsize, Ordering};

#[repr(C)]
struct TrapFrame {
    ra: usize,
    t0: usize,
    t1: usize,
    t2: usize,
    a0: usize,
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    a5: usize,
    a6: usize,
    a7: usize,
    t3: usize,
    t4: usize,
    t5: usize,
    t6: usize,
    s0: usize,
    s1: usize,
    s2: usize,
    s3: usize,
    s4: usize,
    s5: usize,
    s6: usize,
    s7: usize,
    s8: usize,
    s9: usize,
    s10: usize,
    s11: usize,
    mepc: usize,
    _pad: [usize; 3],
}

const _: () = assert!(core::mem::size_of::<TrapFrame>() == 128);

const STACK_CANARY: usize = 0xC0FFEE54;

/// Writes the canary word to the bottom of task `id`'s stack. Called once at spawn time.
fn write_canary(id: usize) {
    unsafe {
        let bottom = STACKS[id - 1].as_mut_ptr() as *mut usize;
        core::ptr::write_volatile(bottom, STACK_CANARY);
    }
}

/// Checks whether task `id`'s stack-bottom canary is still intact. `false` means overflow.
fn check_canary(id: usize) -> bool {
    unsafe {
        let bottom = STACKS[id - 1].as_ptr() as *const usize;
        core::ptr::read_volatile(bottom) == STACK_CANARY
    }
}

const STACK_SIZE: usize = 2048;
const MAX_TASKS: usize = 17; // 16 spawnable tasks + main (task 0)
// (MAX_TASKS - 1) * STACK_SIZE = 16 * 2048 = 32 Kb
//             ^^^ exclude main

static mut STACKS: [[u8; STACK_SIZE]; MAX_TASKS - 1] = [[0; STACK_SIZE]; MAX_TASKS - 1];
static mut SAVED_SP: [usize; MAX_TASKS] = [0; MAX_TASKS];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Priority {
    Low,
    Normal,
    High,
}

static mut PRIORITIES: [Priority; MAX_TASKS] = [Priority::Normal; MAX_TASKS];

/// Whether each slot currently holds a running task. `false` means the
/// slot is either never-used or belongs to a task that has exited —
/// `eligible()` skips these, and `spawn()` prefers reusing them over
/// growing `NUM_TASKS`.
static mut TASK_ALIVE: [bool; MAX_TASKS] = [true; MAX_TASKS];

/// Per-task exit callback, invoked exactly once by `task_exited` right
/// before that slot yields for the last time. Zero-capture `fn()` rather
/// than a boxed `dyn FnOnce` — this crate has no allocator, so a closure
/// with captured state can't be stored without either boxing
/// (unavailable) or a fixed-capacity capture scheme (not worth the
/// complexity here). Cleared back to `None` immediately after firing, so
/// the "once" guarantee holds even though the slot itself lives on for
/// reuse.
static mut ON_EXIT: [Option<fn()>; MAX_TASKS] = [None; MAX_TASKS];

// Task bookkeeping. Atomics here aren't about thread-safety (single-hart,
// and this all runs inside an ISR where hardware already blocks preemption)
// — they force the compiler to treat reads/writes as real memory accesses
// instead of caching/reordering them across calls, which a plain `static
// mut` legally permits and `codegen-units = 1` makes far more likely to
// actually bite.
static NUM_TASKS: AtomicUsize = AtomicUsize::new(1);
static CURRENT_TASK: AtomicUsize = AtomicUsize::new(0);

/// Resets scheduler state to "just task 0, no spawns yet". Must run before any `spawn()`.
/// Called automatically by `KernelBuilder::build()` when `.scheduler()` was requested.
pub fn init() {
    NUM_TASKS.store(1, Ordering::Relaxed);
    CURRENT_TASK.store(0, Ordering::Relaxed);
    // Defensive re-assert: these two are *supposed* to already be
    // DEFAULT_WARM_THRESHOLD/DEFAULT_HOT_THRESHOLD via the `AtomicU8::new`
    // initializer living in .data, but a 0/0 readback here is silently
    // catastrophic — see `thermal_state()`: `raw >= hot_threshold()` is
    // trivially true for every u8 when hot_threshold() is 0, which pins
    // the eco-scheduler in `ThermalState::Hot` forever. `init()` runs from
    // Rust (after boot.s has already done its job, correctly or not), so
    // reasserting here means boot correctness doesn't silently depend on
    // nothing having gone wrong upstream of `main`.
    WARM_THRESHOLD.store(DEFAULT_WARM_THRESHOLD, Ordering::Relaxed);
    HOT_THRESHOLD.store(DEFAULT_HOT_THRESHOLD, Ordering::Relaxed);
    unsafe {
        PRIORITIES = [Priority::Normal; MAX_TASKS];
        TASK_ALIVE = [true; MAX_TASKS];
        ON_EXIT = [None; MAX_TASKS];
    }
}

/// Shorthand for `TaskBuilder::new(entry).spawn()` — spawns at default (`Normal`) priority.
pub fn spawn(entry: fn()) -> Result<usize, ()> {
    TaskBuilder::new(entry).spawn()
}

/// Reports whether task `id` is still running. `false` for an id that
/// was never spawned, or whose task has since exited.
pub fn is_alive(id: usize) -> bool {
    if id >= MAX_TASKS {
        return false;
    }
    unsafe { TASK_ALIVE[id] }
}

pub struct TaskBuilder {
    entry: fn(),
    priority: Priority,
    on_exit: Option<fn()>,
}

impl TaskBuilder {
    /// Starts a task builder for `entry`, defaulting to `Priority::Normal`, no exit callback.
    pub fn new(entry: fn()) -> Self {
        Self {
            entry,
            priority: Priority::Normal,
            on_exit: None,
        }
    }

    /// Overrides the task's priority before spawning.
    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Registers a callback fired exactly once when this task returns
    /// normally (i.e. finishes instead of looping forever). Runs on the
    /// exiting task's own stack, right before it yields for the last time.
    pub fn on_exit(mut self, callback: fn()) -> Self {
        self.on_exit = Some(callback);
        self
    }

    /// Allocates a stack slot (reusing a dead task's slot if one exists,
    /// otherwise growing `NUM_TASKS`), seeds a trap frame pointing at
    /// `entry` with `ra` set to the exit trampoline, and registers the
    /// task. Fails once `MAX_TASKS` is reached with no reusable slot.
    ///
    /// Runs inside a critical section: this mutates the same state
    /// `next_sp()` reads from inside the trap handler, so it must be
    /// atomic with respect to a timer tick firing mid-spawn.
    pub fn spawn(self) -> Result<usize, ()> {
        crate::arch::trap::critical_section(|| unsafe {
            let num_tasks = NUM_TASKS.load(Ordering::Relaxed);
            let current = CURRENT_TASK.load(Ordering::Relaxed);

            // Prefer reusing a dead task's slot over growing NUM_TASKS --
            // but NEVER the slot that is *currently executing this call*.
            // A task can mark itself dead and then, from its own on_exit
            // hook, call spawn() before it has actually switched away --
            // at that point CURRENT_TASK is still that task's id and its
            // stack is genuinely in use (this very call is running on
            // it). Reusing it would mean writing a fresh TrapFrame into
            // the top of the same buffer this call chain's own return
            // addresses live in, corrupting them out from under itself.
            // Excluding `current` just defers that slot's reuse to the
            // next spawn() that happens after this task has truly
            // finished (CURRENT_TASK moved on) -- see task_exited().
            let id = (1..num_tasks)
                .find(|&i| !TASK_ALIVE[i] && i != current)
                .unwrap_or(num_tasks);

            if id >= MAX_TASKS {
                return Err(());
            }

            let stack_top = STACKS[id - 1].as_ptr() as usize + STACK_SIZE;
            let stack_top = stack_top & !0xF;

            let frame_base = stack_top - core::mem::size_of::<TrapFrame>();
            let frame = frame_base as *mut TrapFrame;

            core::ptr::write_bytes(frame, 0, 1);
            (*frame).ra = task_exited as usize;
            (*frame).mepc = self.entry as usize;

            SAVED_SP[id] = frame_base;
            PRIORITIES[id] = self.priority;
            ON_EXIT[id] = self.on_exit;
            TASK_ALIVE[id] = true;
            write_canary(id);

            if id == num_tasks {
                NUM_TASKS.store(id + 1, Ordering::Relaxed);
            }

            Ok(id)
        })
    }
}

/// Landing pad for a task function that returns normally instead of
/// looping forever. Fires that task's `on_exit` callback (if any) exactly
/// once, marks the slot dead so `eligible()` stops picking it and
/// `spawn()` can reuse it, then voluntarily yields (`trap::yield_now()`)
/// so the CPU hands off to the next eligible task immediately — instead
/// of idling in `wfi` and wasting the rest of this slice.
extern "C" fn task_exited() -> ! {
    let id = CURRENT_TASK.load(Ordering::Relaxed);
    crate::arch::trap::critical_section(|| unsafe {
        // Mark dead *before* running the callback: this is what lets an
        // on_exit hook see (via is_alive()/eligible()) that this task is
        // really gone, and lets a *different* in-flight spawn() elsewhere
        // reuse this slot immediately. It deliberately does NOT let this
        // callback reuse its own slot for a replacement task -- spawn()
        // refuses to hand out the currently-executing task's own id (see
        // the comment there) because that stack is still live: we're
        // running ON it right now. A same-slot respawn from on_exit will
        // land in a new slot instead, and this one becomes reusable by
        // the next unrelated spawn() once CURRENT_TASK has moved on.
        // Wrapped in critical_section so a timer tick can't preempt
        // mid-callback and leave the exit sequence half-done.
        TASK_ALIVE[id] = false;
        if let Some(callback) = ON_EXIT[id].take() {
            callback();
        }
    });
    loop {
        crate::arch::trap::yield_now();
    }
}

const DEFAULT_WARM_THRESHOLD: u8 = 110;
const DEFAULT_HOT_THRESHOLD: u8 = 140;

// Runtime-tunable TSENS cutoffs. Relaxed ordering is sufficient: this
// crate is single-hart, and each value is independent with nothing else
// that needs to stay ordered against it.
static WARM_THRESHOLD: AtomicU8 = AtomicU8::new(DEFAULT_WARM_THRESHOLD);
static HOT_THRESHOLD: AtomicU8 = AtomicU8::new(DEFAULT_HOT_THRESHOLD);

/// Reads the current "warm" cutoff (raw TSENS code, not calibrated °C).
pub fn warm_threshold() -> u8 {
    WARM_THRESHOLD.load(Ordering::Relaxed)
}

/// Reads the current "hot" cutoff (raw TSENS code).
pub fn hot_threshold() -> u8 {
    HOT_THRESHOLD.load(Ordering::Relaxed)
}

/// Sets the "warm" cutoff. Takes effect on the next tick; callable anytime,
/// no `unsafe` required.
pub fn set_warm_threshold(value: u8) {
    WARM_THRESHOLD.store(value, Ordering::Relaxed);
}

/// Sets the "hot" cutoff. Takes effect on the next tick; callable anytime,
/// no `unsafe` required.
pub fn set_hot_threshold(value: u8) {
    HOT_THRESHOLD.store(value, Ordering::Relaxed);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ThermalState {
    Cool,
    Warm,
    Hot,
}

/// Classifies live die temperature into Cool/Warm/Hot using the current
/// (possibly runtime-adjusted) thresholds. Compiled when `tsens` is enabled.
#[cfg(feature = "tsens")]
fn thermal_state() -> ThermalState {
    let raw = crate::hal::tsens::read_raw();
    if raw >= hot_threshold() {
        ThermalState::Hot
    } else if raw >= warm_threshold() {
        ThermalState::Warm
    } else {
        ThermalState::Cool
    }
}

/// Fallback when `tsens` isn't compiled in: always Cool, so `eligible()`
/// degenerates to plain round-robin with no thermal gating.
#[cfg(not(feature = "tsens"))]
fn thermal_state() -> ThermalState {
    ThermalState::Cool
}

/// Decides whether task `id` may run under the given thermal `state`.
/// A dead task (exited normally, slot pending reuse) is never eligible,
/// regardless of thermal state. Otherwise: Cool runs everyone, Warm
/// throttles Low, Hot allows only High.
///
/// Task 0 (`main`, the idle/background task) is always eligible,
/// independent of priority or thermal state. Without this floor, a
/// thermal state of `Hot` with no `Priority::High` task defined leaves
/// *nothing* eligible — `next_sp()`'s search wraps around and silently
/// keeps re-selecting whatever was already running without that pick
/// ever satisfying `eligible()`. That's a full scheduler stall, not
/// "graceful degradation by priority": task0 as an always-eligible floor
/// guarantees plain round-robin fairness never fully collapses, while
/// task1/task2 still get throttled by priority exactly as designed.
fn eligible(id: usize, priority: Priority, state: ThermalState) -> bool {
    if !unsafe { TASK_ALIVE[id] } {
        return false;
    }
    if id == 0 {
        return true;
    }
    match (state, priority) {
        (ThermalState::Cool, _) => true,
        (ThermalState::Warm, Priority::Low) => false,
        (ThermalState::Warm, _) => true,
        (ThermalState::Hot, Priority::High) => true,
        (ThermalState::Hot, _) => false,
    }
}

/// Called from the trap handler on every scheduling tick or voluntary
/// yield. Saves the current task's `sp`, checks all live task canaries,
/// picks the next eligible task in round-robin order under the current
/// thermal state, and returns its `sp`.
pub fn next_sp(current_sp: usize) -> usize {
    unsafe {
        let current = CURRENT_TASK.load(Ordering::Relaxed);
        let num_tasks = NUM_TASKS.load(Ordering::Relaxed);

        SAVED_SP[current] = current_sp;

        for id in 1..num_tasks {
            if TASK_ALIVE[id] && !check_canary(id) {
                panic!("stack overflow detected in task {}", id);
            }
        }

        let state = thermal_state();
        let mut next = current;
        for _ in 0..num_tasks {
            next = (next + 1) % num_tasks;
            if eligible(next, PRIORITIES[next], state) {
                break;
            }
        }
        CURRENT_TASK.store(next, Ordering::Relaxed);

        SAVED_SP[next]
    }
}
