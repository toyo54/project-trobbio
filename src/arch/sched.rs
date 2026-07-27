//! Preemptive scheduler — STAGE 2
//!
//! Implements per-task stack allocation, trap frame initialization,
//! round-robin scheduling gated by priority + tsens-driven throttling,
//! and an optional builder for setting priority at spawn time.

/// Mirrors the exact 128-byte frame layout `boot.s`'s trap handler
/// saves/restores (see the offset comment there: ra/t0-t2/a0-a7/t3-t6/
/// s0-s11/mepc, then 3 padding words to round out to 128 bytes / 16-byte
/// alignment). `#[repr(C)]` guarantees field order == memory order with
/// no inserted padding between same-sized fields, so this struct's layout
/// is byte-for-byte what the assembly expects -- it's not just
/// documentation, `spawn()` below actually writes through it.
///
/// The const assertion is the actual payoff: if `boot.s`'s frame size
/// ever changes and this struct doesn't get updated to match (or vice
/// versa), the build fails with a clear message instead of silently
/// corrupting every task's saved state.
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

/// Written at the LOWEST address of each spawned task's stack region at
/// spawn time — the far end from the trap frame, since the stack grows
/// downward from `frame_base` toward here as the task actually runs. If a
/// task's real stack usage ever grows enough to reach this word, that's
/// unambiguous proof of overflow: it's about to (or already did) corrupt
/// whatever sits below it in memory — the next task's stack, or the
/// `STACKS` array's own bounds. Checked every tick in `next_sp()`, so an
/// overflow shows up as an immediate, attributed panic naming the task
/// id, instead of silent corruption surfacing later as a mystery bug in
/// whatever happens to sit next to it.
///
/// Known gap: this only covers spawned tasks. Task 0 (`main`) runs on the
/// boot stack set up directly in `boot.s`, which isn't part of `STACKS`
/// and has no canary here — extending this to cover it would mean adding
/// a guard symbol in `link.x`, not just this file.
///
/// Also worth being precise about what this does and doesn't catch: it
/// only detects usage reaching all the way down to this exact word. A
/// smaller out-of-bounds write deep in a nested call that never reaches
/// the bottom of the stack won't trip it — same fundamental limitation
/// any single-canary scheme has, not something specific to this one.
const STACK_CANARY: usize = 0xC0FFEE42;

fn write_canary(id: usize) {
    unsafe {
        let bottom = STACKS[id - 1].as_mut_ptr() as *mut usize;
        core::ptr::write_volatile(bottom, STACK_CANARY);
    }
}

fn check_canary(id: usize) -> bool {
    unsafe {
        let bottom = STACKS[id - 1].as_ptr() as *const usize;
        core::ptr::read_volatile(bottom) == STACK_CANARY
    }
}

const MAX_TASKS: usize = 4;
const STACK_SIZE: usize = 2048;

// Physical memory for spawned tasks. Task 0 (main) uses the boot stack.
static mut STACKS: [[u8; STACK_SIZE]; MAX_TASKS - 1] = [[0; STACK_SIZE]; MAX_TASKS - 1];

static mut SAVED_SP: [usize; MAX_TASKS] = [0; MAX_TASKS];
static mut CURRENT_TASK: usize = 0;
static mut NUM_TASKS: usize = 1; // Task 0 is pre-registered as `main`

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Priority {
    Low,
    Normal,
    High,
}

static mut PRIORITIES: [Priority; MAX_TASKS] = [Priority::Normal; MAX_TASKS];

/// Must be called before any spawn
pub fn init() {
    unsafe {
        NUM_TASKS = 1;
        CURRENT_TASK = 0;
        PRIORITIES = [Priority::Normal; MAX_TASKS];
    }
}

/// Spawns a new task at Normal priority. Sugar over
/// `TaskBuilder::new(entry).spawn()` for the common case where you don't
/// care about priority at all.
pub fn spawn(entry: fn()) -> Result<usize, ()> {
    TaskBuilder::new(entry).spawn()
}

/// Optional explicit control over a task's priority at spawn time.
/// `sched::spawn(f)` still works unchanged for the common case (defaults
/// to Normal); reach for this when a task is more or less
/// important than the rest -- that's a fact only you, the task's author,
/// can supply. It's not something the scheduler can infer on its own:
/// no honest way to derive "how important is this" from the code alone.
pub struct TaskBuilder {
    entry: fn(),
    priority: Priority,
}

impl TaskBuilder {
    pub fn new(entry: fn()) -> Self {
        Self {
            entry,
            priority: Priority::Normal,
        }
    }

    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Sets up the task's initial trap frame and registers it.
    pub fn spawn(self) -> Result<usize, ()> {
        unsafe {
            let id = NUM_TASKS;
            if id >= MAX_TASKS {
                return Err(());
            }

            let stack_top = STACKS[id - 1].as_ptr() as usize + STACK_SIZE;
            let stack_top = stack_top & !0xF; // 16-byte align, ABI requirement

            let frame_base = stack_top - core::mem::size_of::<TrapFrame>();
            let frame = frame_base as *mut TrapFrame;

            // Zeroed, not garbage: nothing reads these before the task's
            // own first instructions overwrite them.
            core::ptr::write_bytes(frame, 0, 1);
            // mepc is what `mret` jumps to -- pointing it at `entry`
            // means this task's first-ever resume looks identical, to
            // boot.s, to resuming any other already-running task.
            (*frame).mepc = self.entry as usize;

            SAVED_SP[id] = frame_base;
            PRIORITIES[id] = self.priority;
            write_canary(id);
            NUM_TASKS += 1;

            Ok(id)
        }
    }
}

/// Against `hal::tsens::read_raw()`. Grounded in real observed data:
/// ~99-100 at ambient room temperature (not actively cooled), ~70 under
/// active cooling (chip in front of an air conditioner). Raised from an
/// earlier 100/130 guess after confirming 100 sat right at/below ambient
/// baseline -- that was throttling Low-priority tasks constantly at rest,
/// not just under real thermal stress. HOT (140) is still an unverified
/// guess -- no real hot data point exists yet, tune it when you have one.
const WARM_THRESHOLD: u8 = 110;
const HOT_THRESHOLD: u8 = 140;

#[derive(Clone, Copy, PartialEq, Eq)]
enum ThermalState {
    Cool,
    Warm,
    Hot,
}

fn thermal_state() -> ThermalState {
    let raw = crate::hal::tsens::read_raw();
    if raw >= HOT_THRESHOLD {
        ThermalState::Hot
    } else if raw >= WARM_THRESHOLD {
        ThermalState::Warm
    } else {
        ThermalState::Cool
    }
}

fn eligible(priority: Priority, state: ThermalState) -> bool {
    match (state, priority) {
        (ThermalState::Cool, _) => true,
        (ThermalState::Warm, Priority::Low) => false,
        (ThermalState::Warm, _) => true,
        (ThermalState::Hot, Priority::High) => true,
        (ThermalState::Hot, _) => false,
    }
}

/// Called from the reserved machine-timer trap on every tick.
///
/// Fallback if nothing is eligible (e.g. every task is Low/Normal and
/// it's Hot): the search wraps all the way back to `CURRENT_TASK` and
/// just resumes whoever was already running, rather than switching to
/// something that shouldn't run. Deliberate, not an oversight.
pub fn next_sp(current_sp: usize) -> usize {
    unsafe {
        SAVED_SP[CURRENT_TASK] = current_sp;

        for id in 1..NUM_TASKS {
            if !check_canary(id) {
                panic!("stack overflow detected in task {}", id);
            }
        }

        let state = thermal_state();
        let mut next = CURRENT_TASK;
        for _ in 0..NUM_TASKS {
            next = (next + 1) % NUM_TASKS;
            if eligible(PRIORITIES[next], state) {
                break;
            }
        }
        CURRENT_TASK = next;

        SAVED_SP[CURRENT_TASK]
    }
}
