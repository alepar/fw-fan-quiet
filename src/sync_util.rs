//! One poison-tolerant `Mutex` lock helper, shared by every cross-thread
//! mutex in this binary.
//!
//! Rationale (roast PR-1 finding 3): a `.lock().expect(...)` fail-stops the
//! *reader* when some other thread panicked while holding the lock. On the
//! sensor path that would kill the 1 Hz sample stream while the controller
//! keeps its applied caps and the thermal watchdog stops observing — strictly
//! worse than continuing with whatever the poisoned value holds, which is
//! always a fully-initialised, self-consistent snapshot here (every writer
//! publishes with a single assignment). `telemetry::lock` has used exactly
//! this rule since before the fw-fanctrl loop landed ("keep logging instead of
//! cascading the panic"); this is that rule, generic, in one place.
//!
//! Callers keep lock scopes tiny and must never hold one across I/O.

use std::sync::{Mutex, MutexGuard, PoisonError};

/// Locks `m`, recovering the guard instead of panicking if the mutex is
/// poisoned.
pub fn lock<T: ?Sized>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn lock_recovers_a_poisoned_mutex_instead_of_panicking() {
        let m = Arc::new(Mutex::new(7u32));
        let m2 = Arc::clone(&m);
        // Poison it: panic while holding the guard.
        let _ = std::thread::spawn(move || {
            let _guard = m2.lock().unwrap();
            panic!("deliberate poisoning panic");
        })
        .join();
        assert!(m.is_poisoned(), "setup: the mutex must actually be poisoned");
        // `.lock().unwrap()`/`.expect()` would panic here; this must not.
        assert_eq!(*lock(&m), 7);
    }
}
