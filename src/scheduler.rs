//! Kernel thread scheduler (preemptive round-robin, single core).
//!
//! Threads switch voluntarily (`yield_now`, `sleep_ms`, `exit`) and are
//! also preempted by the timer: `tick()` runs from the PIT interrupt,
//! decides whether the running thread's time slice is over (or a sleeper
//! is due), and raises `NEED_RESCHED`. `idt::irq_dispatch` calls
//! `preempt_if_needed()` right after acknowledging the interrupt, which
//! switches threads *inside* the interrupt handler. The interrupted
//! thread's registers and interrupt frame simply stay on its own kernel
//! stack until it is scheduled again and returns through `iretq`.
//!
//! Everything shared sits behind an `IrqMutex` (interrupts are off while
//! a lock is held, so the tick can never land in the middle of a
//! scheduler critical section), and the context switch itself always
//! runs with interrupts off.
//!
//! Design:
//! - Every thread has its own kernel stack in a dedicated virtual region.
//!   Each stack slot has an unmapped guard page at its bottom, so an
//!   overflow is a clean page fault instead of silent corruption.
//! - The boot thread (Limine's stack) becomes thread 0. A dedicated idle
//!   thread runs `hlt` whenever nothing else is runnable.
//! - Round-robin ready queue; sleeping threads are woken by comparing
//!   against `pit::uptime_ms()` whenever the scheduler runs.
//! - A thread that exits can't free the stack it is still running on, so
//!   it parks itself on a zombie list and whichever thread runs next
//!   reaps it (`finish_switch`).

#![allow(dead_code)]

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

use crate::pmm::{self, PAGE_SIZE};
use crate::sync::{self, IrqMutex};
use crate::{idt, pit, vmm};
use crate::{log_fail, log_ok};

/// Virtual region for kernel stacks. Sits between the heap
/// (0xffffffff90000000 + 64 MiB) and the VMM self-test page
/// (0xffffffffc0000000).
const STACK_REGION: u64 = 0xffff_ffff_a000_0000;
/// Virtual size of one slot: 1 guard page + `STACK_PAGES` mapped pages
/// (the rest of the slot is simply never mapped).
const SLOT_SIZE: u64 = 0x2_0000; // 128 KiB
const MAX_SLOTS: usize = 512;
const STACK_PAGES: u64 = 16; // 64 KiB of usable stack per thread

extern "C" {
    /// Save callee-saved registers on the current stack, store the
    /// resulting RSP through `old_rsp`, load `new_rsp` and restore the
    /// registers saved there.
    fn switch_context(old_rsp: *mut u64, new_rsp: u64);
    /// First code a new thread executes; see `create_thread`.
    fn thread_trampoline();
}

core::arch::global_asm!(
    r#"
.section .text

.global switch_context
switch_context:
    push rbp
    push rbx
    push r12
    push r13
    push r14
    push r15
    mov [rdi], rsp
    mov rsp, rsi
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
    ret

.global thread_trampoline
thread_trampoline:
    mov rdi, r12
    call thread_start
    ud2
"#
);

struct Thread {
    id: u64,
    name: &'static str,
    /// Saved kernel RSP while the thread is not running.
    rsp: u64,
    /// Index into the stack region; `None` for the boot thread, which
    /// runs on the bootloader's stack.
    stack_slot: Option<usize>,
    /// Uptime (ms) at which a sleeping thread becomes runnable again.
    wake_at: u64,
    is_idle: bool,
}

struct Scheduler {
    current: Option<Box<Thread>>,
    ready: VecDeque<Box<Thread>>,
    sleeping: Vec<Box<Thread>>,
    idle: Option<Box<Thread>>,
    zombies: Vec<Box<Thread>>,
    free_slots: Vec<usize>,
    next_slot: usize,
    next_id: u64,
}

impl Scheduler {
    const fn new() -> Self {
        Scheduler {
            current: None,
            ready: VecDeque::new(),
            sleeping: Vec::new(),
            idle: None,
            zombies: Vec::new(),
            free_slots: Vec::new(),
            next_slot: 0,
            next_id: 1, // 0 is the boot thread
        }
    }
}

static SCHED: IrqMutex<Scheduler> = IrqMutex::new(Scheduler::new());
static READY: AtomicBool = AtomicBool::new(false);

/// Length of a thread's time slice.
const TIME_SLICE_MS: u64 = 10;

/// Timer ticks the current thread has used up of its slice.
static SLICE_TICKS: AtomicU64 = AtomicU64::new(0);
/// Set by `tick()`, consumed by `preempt_if_needed()`.
static NEED_RESCHED: AtomicBool = AtomicBool::new(false);
/// How many times a thread was preempted by the timer.
static PREEMPTIONS: AtomicU64 = AtomicU64::new(0);

enum Next<'a> {
    Yield,
    Sleep(u64),
    Exit,
    /// Park the current thread on an external queue instead of `ready`
    /// or `sleeping`. Used by `WaitQueue::wait_until`.
    Block(&'a IrqMutex<VecDeque<Box<Thread>>>),
}

fn slot_base(slot: usize) -> u64 {
    STACK_REGION + slot as u64 * SLOT_SIZE
}

/// Unmap the first `pages` stack pages of the slot at `base` (page 0 of
/// the slot is the guard and is never mapped) and return their frames.
fn unmap_stack(base: u64, pages: u64) {
    for i in 0..pages {
        let virt = base + (1 + i) * PAGE_SIZE;
        if let Some(phys) = vmm::translate(virt) {
            vmm::unmap_page(virt);
            pmm::free_frame(phys);
        }
    }
}

/// Build a thread that will start in `entry`. It is not queued anywhere.
fn create_thread(name: &'static str, entry: fn(), is_idle: bool) -> Option<Box<Thread>> {
    let (slot, id) = {
        let mut guard = SCHED.lock();
        let s = &mut *guard;
        let slot = if let Some(i) = s.free_slots.pop() {
            i
        } else if s.next_slot < MAX_SLOTS {
            let i = s.next_slot;
            s.next_slot += 1;
            i
        } else {
            return None;
        };
        let id = s.next_id;
        s.next_id += 1;
        (slot, id)
    };

    let base = slot_base(slot);
    for i in 0..STACK_PAGES {
        let Some(phys) = pmm::alloc_frame() else {
            unmap_stack(base, i);
            SCHED.lock().free_slots.push(slot);
            return None;
        };
        vmm::map_page(base + (1 + i) * PAGE_SIZE, phys, vmm::KERNEL_RW);
    }

    // Initial frame, lowest address first, exactly what `switch_context`
    // pops: r15 r14 r13 r12 rbx rbp, then the return address. After the
    // `ret` into the trampoline RSP equals `stack_top` (16-byte aligned),
    // so the trampoline's `call` gives the callee a correctly aligned
    // stack.
    let stack_top = base + (1 + STACK_PAGES) * PAGE_SIZE;
    let rsp = stack_top - 7 * 8;
    unsafe {
        let frame = rsp as *mut u64;
        for k in 0..7 {
            frame.add(k).write(0);
        }
        frame.add(3).write(entry as usize as u64); // r12
        frame.add(6).write(thread_trampoline as usize as u64); // return address
    }

    Some(Box::new(Thread {
        id,
        name,
        rsp,
        stack_slot: Some(slot),
        wake_at: 0,
        is_idle,
    }))
}

/// Release a dead thread's stack and bookkeeping.
fn free_thread(t: Box<Thread>) {
    if let Some(slot) = t.stack_slot {
        unmap_stack(slot_base(slot), STACK_PAGES);
        SCHED.lock().free_slots.push(slot);
    }
    drop(t);
}

/// Runs right after every context switch, on the new thread's stack:
/// frees any thread that exited just before the switch.
fn finish_switch() {
    let dead = {
        let mut guard = SCHED.lock();
        core::mem::take(&mut guard.zombies)
    };
    for t in dead {
        free_thread(t);
    }
}

fn wake_sleepers(s: &mut Scheduler) {
    if s.sleeping.is_empty() {
        return;
    }
    let now = pit::uptime_ms();
    let mut i = 0;
    while i < s.sleeping.len() {
        if s.sleeping[i].wake_at <= now {
            let t = s.sleeping.swap_remove(i);
            s.ready.push_back(t);
        } else {
            i += 1;
        }
    }
}

/// Take the current thread off the CPU (yielding, sleeping or exiting)
/// and run the next one. Returns when this thread is scheduled again;
/// for `Next::Exit` it never does.
fn reschedule(how: Next<'_>) {
    let saved = sync::irq_save();

    // Whoever runs next gets a fresh time slice.
    SLICE_TICKS.store(0, Ordering::Relaxed);
    NEED_RESCHED.store(false, Ordering::Relaxed);

    let switch = {
        let mut guard = SCHED.lock();
        let s = &mut *guard;
        wake_sleepers(s);

        let mut cur = s.current.take().expect("scheduler: no current thread");
        let cur_ptr: *mut Thread = &mut *cur;
        let cur_is_idle = cur.is_idle;

        match how {
            Next::Yield => {
                if cur_is_idle {
                    s.idle = Some(cur);
                } else {
                    s.ready.push_back(cur);
                }
            }
            Next::Sleep(wake_at) => {
                cur.wake_at = wake_at;
                s.sleeping.push(cur);
            }
            Next::Exit => s.zombies.push(cur),
            Next::Block(queue) => {
                // Locking `queue` while `guard` (SCHED) is held is safe:
                // this is a single-core scheduler and both locks disable
                // interrupts, so there is no real concurrency to race
                // against, only nesting, which `IrqMutex` handles fine.
                queue.lock().push_back(cur);
            }
        }

        let mut next = match s.ready.pop_front() {
            Some(t) => t,
            None => s.idle.take().expect("scheduler: no idle thread"),
        };
        let next_ptr: *mut Thread = &mut *next;
        let next_rsp = next.rsp;
        s.current = Some(next);

        if cur_ptr == next_ptr {
            // Nobody else wants the CPU: keep running.
            None
        } else {
            Some((unsafe { core::ptr::addr_of_mut!((*cur_ptr).rsp) }, next_rsp))
        }
    };

    if let Some((old_rsp, new_rsp)) = switch {
        unsafe { switch_context(old_rsp, new_rsp) };
        // We are running again, possibly much later.
        finish_switch();
    }

    sync::irq_restore(saved);
}

/// Entry point of every new thread (called from `thread_trampoline`).
#[no_mangle]
extern "C" fn thread_start(entry: usize) -> ! {
    finish_switch();
    // The switch that got us here ran with interrupts off.
    idt::enable_interrupts();
    let f: fn() = unsafe { core::mem::transmute(entry) };
    f();
    exit()
}

fn idle_main() {
    loop {
        // `sti; hlt` back to back: the CPU can't take an interrupt
        // between them, so a wakeup can't slip past the hlt.
        unsafe { core::arch::asm!("sti", "hlt", options(nomem, nostack)) };
        yield_now();
    }
}

/// Turn the currently running code into thread 0, create the idle
/// thread, and enable the scheduler. Call after the heap, VMM and PIT
/// are up and interrupts are enabled.
pub fn init() {
    let Some(idle) = create_thread("idle", idle_main, true) else {
        log_fail!("Scheduler", "Init", "Could not create the idle thread");
        return;
    };
    let boot = Box::new(Thread {
        id: 0,
        name: "boot",
        rsp: 0,
        stack_slot: None,
        wake_at: 0,
        is_idle: false,
    });
    {
        let mut guard = SCHED.lock();
        guard.current = Some(boot);
        guard.idle = Some(idle);
    }
    READY.store(true, Ordering::SeqCst);

    log_ok!(
        "Scheduler",
        "Init",
        "Preemptive round-robin scheduler online ({} ms time slice, {} KiB stacks with guard pages, idle thread ready)",
        TIME_SLICE_MS,
        STACK_PAGES * PAGE_SIZE / 1024
    );
}

/// Start a new kernel thread running `entry`. Returns its id.
pub fn spawn(name: &'static str, entry: fn()) -> Option<u64> {
    let t = create_thread(name, entry, false)?;
    let id = t.id;
    SCHED.lock().ready.push_back(t);
    Some(id)
}

/// Give up the CPU; the thread stays runnable.
pub fn yield_now() {
    if READY.load(Ordering::SeqCst) {
        reschedule(Next::Yield);
    }
}

/// Sleep for at least `ms` milliseconds (needs the PIT tick running).
pub fn sleep_ms(ms: u64) {
    if !READY.load(Ordering::SeqCst) {
        return;
    }
    reschedule(Next::Sleep(pit::uptime_ms() + ms));
}

/// End the calling thread.
pub fn exit() -> ! {
    if READY.load(Ordering::SeqCst) {
        reschedule(Next::Exit);
    }
    // Only reachable if the scheduler was never initialised.
    loop {
        unsafe { core::arch::asm!("cli; hlt", options(nomem, nostack)) };
    }
}

/// Called from the timer interrupt on every tick (interrupts are off).
/// Decides whether the running thread should be preempted: its slice is
/// used up and someone else is waiting, or a sleeper's deadline passed.
pub fn tick() {
    if !READY.load(Ordering::Relaxed) {
        return;
    }
    let used = SLICE_TICKS.fetch_add(1, Ordering::Relaxed) + 1;
    let slice_ticks = (pit::frequency_hz() * TIME_SLICE_MS / 1000).max(1);

    // Interrupts were off while the interrupted code ran any scheduler
    // critical section, so this can't normally fail; if it does, just
    // try again on the next tick rather than risk spinning here.
    let Some(s) = SCHED.try_lock() else { return };
    let now = pit::uptime_ms();
    let sleeper_due = s.sleeping.iter().any(|t| t.wake_at <= now);
    let slice_over = used >= slice_ticks && !s.ready.is_empty();
    drop(s);

    if sleeper_due || slice_over {
        NEED_RESCHED.store(true, Ordering::Relaxed);
    }
}

/// Called by the IRQ dispatcher after the interrupt has been
/// acknowledged. Switches to another thread if `tick()` asked for it.
pub fn preempt_if_needed() {
    if NEED_RESCHED.swap(false, Ordering::Relaxed) {
        PREEMPTIONS.fetch_add(1, Ordering::Relaxed);
        reschedule(Next::Yield);
    }
}

pub fn preemption_count() -> u64 {
    PREEMPTIONS.load(Ordering::Relaxed)
}

pub fn current_id() -> u64 {
    SCHED.lock().current.as_ref().map(|t| t.id).unwrap_or(0)
}

pub fn current_name() -> &'static str {
    SCHED.lock().current.as_ref().map(|t| t.name).unwrap_or("?")
}

/// Stack slots currently in use (live threads other than the boot thread).
pub fn stack_slots_in_use() -> usize {
    let s = SCHED.lock();
    s.next_slot - s.free_slots.len()
}

// ---------------------------------------------------------------------
// Blocking primitives
// ---------------------------------------------------------------------

/// A queue of threads parked waiting for some condition, plus the
/// primitive other synchronization types (blocking `Mutex`, `Semaphore`,
/// ...) are built on. Unlike `sleep_ms`, threads on a `WaitQueue` are
/// not touched by the timer at all: they only become ready again
/// through `wake_one`/`wake_all`, so this costs nothing while blocked
/// and wakes are exact instead of poll-driven.
///
/// `wait_until` is the primitive to reach for: it evaluates `cond`
/// and, if the thread needs to block, parks it on the queue in the
/// same interrupts-off critical section, so a wakeup that happens
/// between the check and the park can never be lost. Plain `wait()` has
/// no such guard and is only safe when the caller has independently
/// ruled out a racing wakeup (e.g. nothing else can run yet).
pub struct WaitQueue {
    waiters: IrqMutex<VecDeque<Box<Thread>>>,
}

impl WaitQueue {
    pub const fn new() -> Self {
        WaitQueue {
            waiters: IrqMutex::new(VecDeque::new()),
        }
    }

    /// Block the current thread until `cond` returns `true`.
    ///
    /// `cond` is called with interrupts disabled; if it returns `false`
    /// the thread is parked on this queue and switched away *before*
    /// interrupts come back on, so nothing can slip a wakeup in between
    /// the check and the park. `cond` may be called more than once
    /// (once per wakeup, spuriously or not) and must be cheap and
    /// side-effect-safe to repeat -- exactly like a condition variable
    /// predicate.
    pub fn wait_until<F: FnMut() -> bool>(&self, mut cond: F) {
        loop {
            let saved = sync::irq_save();
            if cond() {
                sync::irq_restore(saved);
                return;
            }
            if !READY.load(Ordering::SeqCst) {
                // Scheduler isn't up yet; nobody could ever wake us.
                sync::irq_restore(saved);
                return;
            }
            // `reschedule` manages its own (nested) interrupt state and
            // only returns here once we've been woken and rescheduled.
            reschedule(Next::Block(&self.waiters));
            sync::irq_restore(saved);
        }
    }

    /// Unconditionally park the current thread here. Only safe when the
    /// caller can guarantee nothing wakes this queue before the thread
    /// is actually parked; prefer `wait_until` otherwise.
    pub fn wait(&self) {
        if !READY.load(Ordering::SeqCst) {
            return;
        }
        reschedule(Next::Block(&self.waiters));
    }

    /// Move one waiting thread (if any) back onto the ready queue.
    /// Returns whether a thread was woken.
    pub fn wake_one(&self) -> bool {
        let woken = self.waiters.lock().pop_front();
        match woken {
            Some(t) => {
                SCHED.lock().ready.push_back(t);
                true
            }
            None => false,
        }
    }

    /// Move every waiting thread back onto the ready queue.
    pub fn wake_all(&self) {
        let all: Vec<Box<Thread>> = self.waiters.lock().drain(..).collect();
        if all.is_empty() {
            return;
        }
        let mut s = SCHED.lock();
        for t in all {
            s.ready.push_back(t);
        }
    }

    /// Number of threads currently parked here. For diagnostics/tests.
    pub fn len(&self) -> usize {
        self.waiters.lock().len()
    }
}

// ---------------------------------------------------------------------
// Self-test
// ---------------------------------------------------------------------

static DONE: AtomicUsize = AtomicUsize::new(0);
static ORDER_LEN: AtomicUsize = AtomicUsize::new(0);
static ORDER: [AtomicU64; 16] = [const { AtomicU64::new(0) }; 16];
static SLEEP_ELAPSED: AtomicU64 = AtomicU64::new(0);

static STOP: AtomicBool = AtomicBool::new(false);
static SPIN_A: AtomicU64 = AtomicU64::new(0);
static SPIN_B: AtomicU64 = AtomicU64::new(0);
static SPIN_DONE: AtomicUsize = AtomicUsize::new(0);

const WORKERS: usize = 3;
const ROUNDS: usize = 4;

fn worker() {
    let id = current_id();
    for _ in 0..ROUNDS {
        let i = ORDER_LEN.fetch_add(1, Ordering::SeqCst);
        if i < ORDER.len() {
            ORDER[i].store(id, Ordering::SeqCst);
        }
        yield_now();
    }
    DONE.fetch_add(1, Ordering::SeqCst);
}

fn sleeper() {
    let start = pit::uptime_ms();
    sleep_ms(50);
    SLEEP_ELAPSED.store(pit::uptime_ms() - start, Ordering::SeqCst);
    DONE.fetch_add(1, Ordering::SeqCst);
}

/// Busy threads that never yield: only the timer can take the CPU away.
fn spinner_a() {
    while !STOP.load(Ordering::Relaxed) {
        SPIN_A.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    }
    SPIN_DONE.fetch_add(1, Ordering::SeqCst);
}

fn spinner_b() {
    while !STOP.load(Ordering::Relaxed) {
        SPIN_B.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    }
    SPIN_DONE.fetch_add(1, Ordering::SeqCst);
}

/// Yield until `DONE` reaches `target`, or give up after two seconds.
fn wait_for(counter: &AtomicUsize, target: usize) -> bool {
    let deadline = pit::uptime_ms() + 2000;
    while counter.load(Ordering::SeqCst) < target {
        if pit::uptime_ms() > deadline {
            return false;
        }
        yield_now();
    }
    true
}

pub fn self_test() {
    if !READY.load(Ordering::SeqCst) {
        log_fail!("Scheduler", "SelfTest", "Scheduler not initialized");
        return;
    }

    DONE.store(0, Ordering::SeqCst);
    ORDER_LEN.store(0, Ordering::SeqCst);
    let slots_before = stack_slots_in_use();

    // --- Part 1: cooperative switching, sleeping, exit/reap -----------
    for _ in 0..WORKERS {
        if spawn("worker", worker).is_none() {
            log_fail!("Scheduler", "SelfTest", "Could not spawn a worker thread");
            return;
        }
    }
    if spawn("sleeper", sleeper).is_none() {
        log_fail!("Scheduler", "SelfTest", "Could not spawn the sleeper thread");
        return;
    }

    if !wait_for(&DONE, WORKERS + 1) {
        log_fail!(
            "Scheduler",
            "SelfTest",
            "Timed out: only {} of {} threads finished",
            DONE.load(Ordering::SeqCst),
            WORKERS + 1
        );
        return;
    }

    // Let the last exited thread be reaped (this also exercises sleeping
    // the boot thread while the idle thread runs).
    sleep_ms(20);

    let total = ORDER_LEN.load(Ordering::SeqCst);
    if total != WORKERS * ROUNDS {
        log_fail!(
            "Scheduler",
            "SelfTest",
            "Workers made {} steps, expected {}",
            total,
            WORKERS * ROUNDS
        );
        return;
    }

    // With the timer able to preempt a worker between two of its steps,
    // the exact order is no longer guaranteed, but each worker must
    // still have taken exactly ROUNDS steps and the first steps must not
    // all belong to one thread.
    let order: [u64; 16] = core::array::from_fn(|i| ORDER[i].load(Ordering::SeqCst));
    for i in 0..total {
        let n = order[..total].iter().filter(|&&x| x == order[i]).count();
        if n != ROUNDS {
            log_fail!(
                "Scheduler",
                "SelfTest",
                "Thread {} took {} steps, expected {}",
                order[i],
                n,
                ROUNDS
            );
            return;
        }
    }
    if order[0] == order[1] && order[1] == order[2] {
        log_fail!("Scheduler", "SelfTest", "Threads did not interleave");
        return;
    }

    let slept = SLEEP_ELAPSED.load(Ordering::SeqCst);
    if slept < 50 || slept > 500 {
        log_fail!("Scheduler", "SelfTest", "sleep_ms(50) took {} ms", slept);
        return;
    }

    // --- Part 2: preemption --------------------------------------------
    // Two threads that never yield. If the timer could not preempt them,
    // the boot thread's sleep below would never end.
    STOP.store(false, Ordering::SeqCst);
    SPIN_A.store(0, Ordering::SeqCst);
    SPIN_B.store(0, Ordering::SeqCst);
    SPIN_DONE.store(0, Ordering::SeqCst);
    let preempt_before = preemption_count();

    if spawn("spin-a", spinner_a).is_none() || spawn("spin-b", spinner_b).is_none() {
        log_fail!("Scheduler", "SelfTest", "Could not spawn the busy threads");
        return;
    }

    let start = pit::uptime_ms();
    sleep_ms(100);
    let woke_after = pit::uptime_ms() - start;

    let a = SPIN_A.load(Ordering::Relaxed);
    let b = SPIN_B.load(Ordering::Relaxed);
    STOP.store(true, Ordering::SeqCst);

    if !wait_for(&SPIN_DONE, 2) {
        log_fail!("Scheduler", "SelfTest", "Busy threads did not stop");
        return;
    }
    sleep_ms(20); // reap them

    let preemptions = preemption_count() - preempt_before;
    if a == 0 || b == 0 {
        log_fail!(
            "Scheduler",
            "SelfTest",
            "Busy threads did not share the CPU (a={}, b={})",
            a,
            b
        );
        return;
    }
    if preemptions < 2 {
        log_fail!(
            "Scheduler",
            "SelfTest",
            "Only {} preemptions during a 100 ms busy period",
            preemptions
        );
        return;
    }
    if woke_after > 100 + 100 {
        log_fail!(
            "Scheduler",
            "SelfTest",
            "Sleeping thread woke {} ms late behind busy threads",
            woke_after - 100
        );
        return;
    }

    let slots_after = stack_slots_in_use();
    if slots_after != slots_before {
        log_fail!(
            "Scheduler",
            "SelfTest",
            "Stack slots leaked: {} in use before, {} after",
            slots_before,
            slots_after
        );
        return;
    }

    log_ok!(
        "Scheduler",
        "SelfTest",
        "cooperative + preemptive verified: sleep_ms(50) took {} ms, 2 busy threads shared the CPU ({} preemptions in 100 ms, boot woke after {} ms), stacks reclaimed",
        slept,
        preemptions,
        woke_after
    );
}