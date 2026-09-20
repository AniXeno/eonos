//! Interrupt-safe spinlock.
//!
//! A plain spinlock deadlocks on a single core if an interrupt handler
//! tries to take a lock the interrupted code already holds. `IrqMutex`
//! avoids that by disabling interrupts for as long as the lock is held,
//! and restoring the previous interrupt state (not blindly re-enabling)
//! when the guard is dropped.
//!
//! Notes:
//! - Guards should be dropped in reverse order of acquisition (normal
//!   scoping does this automatically).
//! - `try_lock` and `force_unlock` exist for crash paths, where waiting
//!   on a lock the faulting code may hold would hang the machine.

#![allow(dead_code)]

use core::arch::asm;
use core::mem::ManuallyDrop;
use core::ops::{Deref, DerefMut};

use spin::{Mutex, MutexGuard};

/// Disable interrupts and return whether they were enabled before.
/// Pair with `irq_restore` (the scheduler uses these directly).
#[inline]
pub fn irq_save() -> bool {
    let flags: u64;
    unsafe {
        // Deliberately no `nomem`/`preserves_flags`: this must also act
        // as a compiler barrier so accesses can't move across the `cli`.
        asm!("pushfq", "pop {0}", "cli", out(reg) flags);
    }
    flags & (1 << 9) != 0
}

/// Re-enable interrupts only if they were enabled when `irq_save` ran.
#[inline]
pub fn irq_restore(was_enabled: bool) {
    if was_enabled {
        unsafe { asm!("sti") };
    }
}

pub struct IrqMutex<T> {
    inner: Mutex<T>,
}

pub struct IrqMutexGuard<'a, T> {
    guard: ManuallyDrop<MutexGuard<'a, T>>,
    irq_was_enabled: bool,
}

impl<T> IrqMutex<T> {
    pub const fn new(value: T) -> Self {
        IrqMutex {
            inner: Mutex::new(value),
        }
    }

    pub fn lock(&self) -> IrqMutexGuard<'_, T> {
        let irq_was_enabled = irq_save();
        IrqMutexGuard {
            guard: ManuallyDrop::new(self.inner.lock()),
            irq_was_enabled,
        }
    }

    /// Take the lock if it is free, otherwise return `None` immediately
    /// (with the interrupt state left exactly as it was).
    pub fn try_lock(&self) -> Option<IrqMutexGuard<'_, T>> {
        let irq_was_enabled = irq_save();
        match self.inner.try_lock() {
            Some(g) => Some(IrqMutexGuard {
                guard: ManuallyDrop::new(g),
                irq_was_enabled,
            }),
            None => {
                irq_restore(irq_was_enabled);
                None
            }
        }
    }

    /// Release the lock without a guard.
    ///
    /// # Safety
    /// Only for crash paths where the holder will never run again.
    pub unsafe fn force_unlock(&self) {
        self.inner.force_unlock();
    }
}

impl<T> Deref for IrqMutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.guard
    }
}

impl<T> DerefMut for IrqMutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut self.guard
    }
}

impl<T> Drop for IrqMutexGuard<'_, T> {
    fn drop(&mut self) {
        // Release the lock first, then restore interrupts.
        unsafe { ManuallyDrop::drop(&mut self.guard) };
        irq_restore(self.irq_was_enabled);
    }
}