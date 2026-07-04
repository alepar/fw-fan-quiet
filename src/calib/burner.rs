//! In-process CPU burner: pins N threads in spin loops to generate full CPU
//! load without external tools (`stress-ng` etc.). Used by the selftest and
//! by calibration sweeps to make power limits observable.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;

/// A running set of spin-loop threads. Create with [`Burner::start`], tear
/// down with [`Burner::stop`] (consumes self: a stopped burner cannot be
/// reused, start a new one).
pub struct Burner {
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl Burner {
    /// Spawns `n_threads` spin loops (`black_box`ed counter increments so the
    /// optimizer cannot delete the work). `start(0)` is a valid no-op burner.
    pub fn start(n_threads: usize) -> Burner {
        let stop = Arc::new(AtomicBool::new(false));
        let threads = (0..n_threads)
            .map(|i| {
                let stop = Arc::clone(&stop);
                std::thread::Builder::new()
                    .name(format!("burner-{i}"))
                    .spawn(move || {
                        let mut counter: u64 = 0;
                        while !stop.load(Ordering::Relaxed) {
                            counter = std::hint::black_box(counter.wrapping_add(1));
                        }
                        std::hint::black_box(counter);
                    })
                    .expect("failed to spawn burner thread")
            })
            .collect();
        Burner { stop, threads }
    }

    /// Flips the stop flag and joins every thread. Prompt by construction:
    /// each loop iteration checks the flag.
    pub fn stop(self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads {
            if t.join().is_err() {
                tracing::error!("burner thread panicked");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn stop_returns_promptly() {
        let burner = Burner::start(2);
        // Let the spin loops actually run a little before stopping.
        std::thread::sleep(Duration::from_millis(50));
        let t = Instant::now();
        burner.stop();
        assert!(
            t.elapsed() < Duration::from_secs(1),
            "stop() took {:?}, expected < 1 s",
            t.elapsed()
        );
    }

    #[test]
    fn zero_threads_is_noop() {
        Burner::start(0).stop(); // must not panic or hang
    }
}
