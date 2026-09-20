//! Blocking synchronization primitives.
//!
//! `sync::IrqMutex` is a spinlock: a thread that can't get it burns CPU
//! (with interrupts off, no less) until it can. That's the right choice
//! for the short, non-preemptible critical sections inside the
//! scheduler itself, but wrong for anything a thread might hold for a
//! while -- the waiter should give up the CPU instead of spinning.
//!
//! `Mutex` and `Semaphore` here do that: a thread that can't proceed
//! parks itself on a `scheduler::WaitQueue` and is woken explicitly by
//! whoever releases the resource, rather than being polled by the timer
//! (as `sleep_ms` is) or spinning (as `IrqMutex` does).

#![allow(dead_code)]

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, AtomicI64, Ordering};

use crate::scheduler::WaitQueue;
use crate::sync::IrqMutex;

/// A mutex that blocks (rather than spins) when contended.
pub struct Mutex<T> {
    locked: AtomicBool,
    queue: WaitQueue,
    data: UnsafeCell<T>,
}

unsafe impl<T: Send> Send for Mutex<T> {}
unsafe impl<T: Send> Sync for Mutex<T> {}

pub struct MutexGuard<'a, T> {
    mutex: &'a Mutex<T>,
}

impl<T> Mutex<T> {
    pub const fn new(value: T) -> Self {
        Mutex {
            locked: AtomicBool::new(false),
            queue: WaitQueue::new(),
            data: UnsafeCell::new(value),
        }
    }

    /// Block until the lock is held.
    pub fn lock(&self) -> MutexGuard<'_, T> {
        // The swap is the actual acquire attempt; `wait_until` runs it
        // with interrupts off and, if it fails, parks us on `queue`
        // before interrupts come back -- so a concurrent `unlock` can
        // never wake the queue in the gap between "we failed to take
        // the lock" and "we're on the queue".
        self.queue
            .wait_until(|| !self.locked.swap(true, Ordering::Acquire));
        MutexGuard { mutex: self }
    }

    /// Take the lock only if it's free right now.
    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        if !self.locked.swap(true, Ordering::Acquire) {
            Some(MutexGuard { mutex: self })
        } else {
            None
        }
    }
}

impl<T> Deref for MutexGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe { &*self.mutex.data.get() }
    }
}

impl<T> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.mutex.data.get() }
    }
}

impl<T> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        self.mutex.locked.store(false, Ordering::Release);
        // At most one waiter can actually take the lock next, so waking
        // one is enough; the rest stay parked and get re-checked (via
        // `wait_until`'s predicate) whenever they're next woken.
        self.mutex.queue.wake_one();
    }
}

/// A counting semaphore. `acquire` blocks while the count is zero;
/// `release` increments it and wakes one waiter.
pub struct Semaphore {
    count: AtomicI64,
    queue: WaitQueue,
}

impl Semaphore {
    pub const fn new(initial: i64) -> Self {
        Semaphore {
            count: AtomicI64::new(initial),
            queue: WaitQueue::new(),
        }
    }

    pub fn acquire(&self) {
        self.queue.wait_until(|| {
            // Fetch-then-restore-if-we-shouldn't-have-taken-it would
            // race, so use a CAS loop against the current value instead
            // of a blind fetch_sub.
            let mut cur = self.count.load(Ordering::Relaxed);
            loop {
                if cur <= 0 {
                    return false;
                }
                match self.count.compare_exchange_weak(
                    cur,
                    cur - 1,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => return true,
                    Err(actual) => cur = actual,
                }
            }
        });
    }

    /// Try to acquire without blocking.
    pub fn try_acquire(&self) -> bool {
        let mut cur = self.count.load(Ordering::Relaxed);
        while cur > 0 {
            match self.count.compare_exchange_weak(
                cur,
                cur - 1,
                Ordering::Acquire,
                Ordering::Relaxed,
            ) {
                Ok(_) => return true,
                Err(actual) => cur = actual,
            }
        }
        false
    }

    pub fn release(&self) {
        self.count.fetch_add(1, Ordering::Release);
        self.queue.wake_one();
    }

    pub fn available(&self) -> i64 {
        self.count.load(Ordering::Relaxed)
    }
}

// ---------------------------------------------------------------------
// Self-test
// ---------------------------------------------------------------------

use core::sync::atomic::AtomicU64;

use crate::scheduler;
use crate::{log_fail, log_ok};

static TEST_MUTEX: Mutex<u64> = Mutex::new(0);
static MUTEX_WITNESS: AtomicU64 = AtomicU64::new(0);
static MUTEX_DONE: IrqMutex<u64> = IrqMutex::new(0);

const MUTEX_WORKERS: u64 = 4;
const MUTEX_ITERS: u64 = 200;

fn mutex_worker() {
    for _ in 0..MUTEX_ITERS {
        let mut guard = TEST_MUTEX.lock();
        // If locking weren't exclusive, two threads could both observe
        // the same value here and the total below would come up short.
        let before = *guard;
        scheduler::yield_now(); // widen the window for a real race
        *guard = before + 1;
        drop(guard);
    }
    *MUTEX_DONE.lock() += 1;
}

fn wait_for(counter: impl Fn() -> u64, target: u64) -> bool {
    let deadline = crate::pit::uptime_ms() + 2000;
    while counter() < target {
        if crate::pit::uptime_ms() > deadline {
            return false;
        }
        scheduler::yield_now();
    }
    true
}

static TEST_SEM: Semaphore = Semaphore::new(2);
static SEM_INSIDE: AtomicU64 = AtomicU64::new(0);
static SEM_MAX_INSIDE: AtomicU64 = AtomicU64::new(0);
static SEM_DONE: IrqMutex<u64> = IrqMutex::new(0);

const SEM_WORKERS: u64 = 5;

fn sem_worker() {
    TEST_SEM.acquire();
    let now = SEM_INSIDE.fetch_add(1, Ordering::SeqCst) + 1;
    SEM_MAX_INSIDE.fetch_max(now, Ordering::SeqCst);
    for _ in 0..5 {
        scheduler::yield_now();
    }
    SEM_INSIDE.fetch_sub(1, Ordering::SeqCst);
    TEST_SEM.release();
    *SEM_DONE.lock() += 1;
}

pub fn self_test() {
    // --- Mutex: mutual exclusion under contention ---------------------
    *MUTEX_DONE.lock() = 0;
    for _ in 0..MUTEX_WORKERS {
        if scheduler::spawn("mutex-worker", mutex_worker).is_none() {
            log_fail!("Blocking", "SelfTest", "Could not spawn a mutex worker");
            return;
        }
    }
    if !wait_for(|| *MUTEX_DONE.lock(), MUTEX_WORKERS) {
        log_fail!("Blocking", "SelfTest", "Mutex workers timed out");
        return;
    }
    let total = *TEST_MUTEX.lock();
    let expected = MUTEX_WORKERS * MUTEX_ITERS;
    if total != expected {
        log_fail!(
            "Blocking",
            "SelfTest",
            "Mutex allowed a lost update: counter is {}, expected {}",
            total,
            expected
        );
        return;
    }
    if TEST_MUTEX.queue.len() != 0 {
        log_fail!("Blocking", "SelfTest", "Mutex wait queue not empty after test");
        return;
    }

    // --- Semaphore: bounds concurrency, never over-admits --------------
    *SEM_DONE.lock() = 0;
    SEM_INSIDE.store(0, Ordering::SeqCst);
    SEM_MAX_INSIDE.store(0, Ordering::SeqCst);
    for _ in 0..SEM_WORKERS {
        if scheduler::spawn("sem-worker", sem_worker).is_none() {
            log_fail!("Blocking", "SelfTest", "Could not spawn a semaphore worker");
            return;
        }
    }
    if !wait_for(|| *SEM_DONE.lock(), SEM_WORKERS) {
        log_fail!("Blocking", "SelfTest", "Semaphore workers timed out");
        return;
    }
    let peak = SEM_MAX_INSIDE.load(Ordering::SeqCst);
    if peak > 2 {
        log_fail!(
            "Blocking",
            "SelfTest",
            "Semaphore admitted {} threads at once, limit was 2",
            peak
        );
        return;
    }
    if peak < 2 {
        log_fail!(
            "Blocking",
            "SelfTest",
            "Semaphore never let two threads run concurrently (peak {})",
            peak
        );
        return;
    }
    if TEST_SEM.available() != 2 {
        log_fail!(
            "Blocking",
            "SelfTest",
            "Semaphore count is {} after test, expected 2",
            TEST_SEM.available()
        );
        return;
    }

    log_ok!(
        "Blocking",
        "SelfTest",
        "blocking Mutex ({} workers x {} increments, no lost updates) and Semaphore \
         (peak concurrency {}/2) verified, no busy-waiting",
        MUTEX_WORKERS,
        MUTEX_ITERS,
        peak
    );
}