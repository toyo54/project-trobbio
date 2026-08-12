//! Preemptive scheduler
//!
//! Implements per-task stack allocation, trap frame initialization,
//! round-robin scheduling gated by priority + tsens-driven throttling,
//! and an optional builder for setting priority at spawn time.
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

const MAX_TASKS: usize = 4;
const STACK_SIZE: usize = 2048;

static mut STACKS: [[u8; STACK_SIZE]; MAX_TASKS - 1] = [[0; STACK_SIZE]; MAX_TASKS - 1];
static mut SAVED_SP: [usize; MAX_TASKS] = [0; MAX_TASKS];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Priority {
    Low,
    Normal,
    High,
}

static mut PRIORITIES: [Priority; MAX_TASKS] = [Priority::Normal; MAX_TASKS];

// Task bookkeeping. Atomics here aren't about thread-safety (single-hart,
// and this all runs inside an ISR where hardware already blocks preemption)
// — they force the compiler to treat reads/writes as real memory accesses
// instead of caching/reordering them across calls, which a plain `static
// mut` legally permits and `codegen-units = 1` makes far more likely to
// actually bite.
static NUM_TASKS: AtomicUsize = AtomicUsize::new(1);
static CURRENT_TASK: AtomicUsize = AtomicUsize::new(0);

/// Resets scheduler state to "just task 0, no spawns yet" and reasserts
/// the default eco-scheduling thresholds. Must run before any `spawn()`.
/// Called automatically by `KernelBuilder::build()` when `.scheduler()`
/// was requested.
///
/// Explicitly re-stores WARM_THRESHOLD/HOT_THRESHOLD here rather than
/// relying solely on their static initializers surviving the .data
/// flash->RAM copy in boot.s — belt-and-suspenders against exactly the
/// kind of silent zero-init this caught (thresholds read as 0, putting
/// the scheduler permanently in ThermalState::Hot and starving every
/// non-High task).
pub fn init() {
    NUM_TASKS.store(1, Ordering::Relaxed);
    CURRENT_TASK.store(0, Ordering::Relaxed);
    WARM_THRESHOLD.store(DEFAULT_WARM_THRESHOLD, Ordering::Relaxed);
    HOT_THRESHOLD.store(DEFAULT_HOT_THRESHOLD, Ordering::Relaxed);
    unsafe {
        PRIORITIES = [Priority::Normal; MAX_TASKS];
    }
}

/// Shorthand for `TaskBuilder::new(entry).spawn()` — spawns at default (`Normal`) priority.
pub fn spawn(entry: fn()) -> Result<usize, ()> {
    TaskBuilder::new(entry).spawn()
}

pub struct TaskBuilder {
    entry: fn(),
    priority: Priority,
}

impl TaskBuilder {
    /// Starts a task builder for `entry`, defaulting to `Priority::Normal`.
    pub fn new(entry: fn()) -> Self {
        Self {
            entry,
            priority: Priority::Normal,
        }
    }

    /// Overrides the task's priority before spawning.
    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Allocates a stack slot, seeds a trap frame pointing at `entry`, and registers the task.
    /// Fails once `MAX_TASKS` is reached.
    pub fn spawn(self) -> Result<usize, ()> {
        unsafe {
            let id = NUM_TASKS.load(Ordering::Relaxed);
            if id >= MAX_TASKS {
                return Err(());
            }

            let stack_top = STACKS[id - 1].as_ptr() as usize + STACK_SIZE;
            let stack_top = stack_top & !0xF;

            let frame_base = stack_top - core::mem::size_of::<TrapFrame>();
            let frame = frame_base as *mut TrapFrame;

            core::ptr::write_bytes(frame, 0, 1);
            (*frame).mepc = self.entry as usize;

            SAVED_SP[id] = frame_base;
            PRIORITIES[id] = self.priority;
            write_canary(id);
            NUM_TASKS.store(id + 1, Ordering::Relaxed);

            Ok(id)
        }
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

/// Decides whether a task of the given `priority` may run under the given
/// thermal `state`. Cool: everyone runs. Warm: Low is throttled. Hot: only High runs.
fn eligible(priority: Priority, state: ThermalState) -> bool {
    match (state, priority) {
        (ThermalState::Cool, _) => true,
        (ThermalState::Warm, Priority::Low) => false,
        (ThermalState::Warm, _) => true,
        (ThermalState::Hot, Priority::High) => true,
        (ThermalState::Hot, _) => false,
    }
}

/// Called from the trap handler on every scheduling tick. Saves the current
/// task's `sp`, checks all stack canaries, picks the next eligible task in
/// round-robin order under the current thermal state, and returns its `sp`.
pub fn next_sp(current_sp: usize) -> usize {
    unsafe {
        let current = CURRENT_TASK.load(Ordering::Relaxed);
        let num_tasks = NUM_TASKS.load(Ordering::Relaxed);

        SAVED_SP[current] = current_sp;

        for id in 1..num_tasks {
            if !check_canary(id) {
                panic!("stack overflow detected in task {}", id);
            }
        }

        let state = thermal_state();
        let mut next = current;
        for _ in 0..num_tasks {
            next = (next + 1) % num_tasks;
            if eligible(PRIORITIES[next], state) {
                break;
            }
        }
        CURRENT_TASK.store(next, Ordering::Relaxed);

        SAVED_SP[next]
    }
}
