//! Command runner seam: actuator logic calls external programs (modprobe,
//! ryzenadj) through this trait so it is testable without hardware or root.

use std::io;
use std::process::Output;

pub trait Runner {
    fn run(&self, program: &str, args: &[&str]) -> io::Result<Output>;
}

/// A shared reference to a Runner is itself a Runner, so several components
/// (e.g. RestoreGuard + a borrowed actuator in tests) can share one instance.
impl<R: Runner + ?Sized> Runner for &R {
    fn run(&self, program: &str, args: &[&str]) -> io::Result<Output> {
        (**self).run(program, args)
    }
}

/// Runs commands for real via `std::process::Command`.
pub struct RealRunner;

impl Runner for RealRunner {
    fn run(&self, program: &str, args: &[&str]) -> io::Result<Output> {
        std::process::Command::new(program).args(args).output()
    }
}

/// Shared test double for all actuator modules' tests (cfg(test) makes it
/// visible crate-wide in the unit-test build only, so no production dead code).
#[cfg(test)]
pub mod test_support {
    use super::Runner;
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::io;
    use std::process::Output;

    /// Records invocations and returns scripted results. When the script
    /// queue is empty, returns success with empty stdout (exit code 0) --
    /// EXCEPT for `ryzenadj --info` immediately after a `ryzenadj
    /// --stapm-limit=...` write, which instead synthesizes a read-back
    /// table that agrees with what was just written (see
    /// `synthesize_ryzenadj_info`). Read-back verification (design §2.9)
    /// means every `CpuActuator::set_sustained_mw` call now makes TWO
    /// runner calls; without this default, every pre-existing test built
    /// against the single-call write (the whole suite predates read-back)
    /// would need to start scripting a matching `--info` reply by hand. A
    /// test that needs `Mismatch`/`Unreadable` still scripts the queue
    /// explicitly -- that takes priority (FIFO) over this default.
    #[derive(Default)]
    pub struct FakeRunner {
        calls: RefCell<Vec<(String, Vec<String>)>>,
        results: RefCell<VecDeque<io::Result<Output>>>,
        last_ryzenadj_write: RefCell<Option<Vec<String>>>,
    }

    impl FakeRunner {
        pub fn new() -> Self {
            Self::default()
        }

        /// Queue the result for the next `run` call (FIFO).
        pub fn push_result(&self, result: io::Result<Output>) {
            self.results.borrow_mut().push_back(result);
        }

        /// Every invocation so far, as (program, args).
        pub fn calls(&self) -> Vec<(String, Vec<String>)> {
            self.calls.borrow().clone()
        }
    }

    impl Runner for FakeRunner {
        fn run(&self, program: &str, args: &[&str]) -> io::Result<Output> {
            let args_owned: Vec<String> = args.iter().map(|a| a.to_string()).collect();
            self.calls
                .borrow_mut()
                .push((program.to_string(), args_owned.clone()));
            if let Some(result) = self.results.borrow_mut().pop_front() {
                return result;
            }
            if program == "ryzenadj" {
                if args.len() == 1 && args[0] == "--info" {
                    if let Some(last_write) = self.last_ryzenadj_write.borrow().as_deref() {
                        return Ok(synthesize_ryzenadj_info(last_write));
                    }
                } else {
                    *self.last_ryzenadj_write.borrow_mut() = Some(args_owned);
                }
            }
            Ok(output_with_code(0))
        }
    }

    /// Build a `ryzenadj --info`-shaped `Output` reporting STAPM/slow/fast
    /// exactly as commanded by `write_args` (a `ryzenadj --stapm-limit=<mw>
    /// --slow-limit=<mw> --fast-limit=<mw>` argument list) -- the read-back
    /// agrees with the write by construction, so it verifies. Any flag not
    /// present in `write_args` reports 0 W.
    fn synthesize_ryzenadj_info(write_args: &[String]) -> Output {
        let watts_for = |flag: &str| -> f64 {
            write_args
                .iter()
                .find_map(|a| a.strip_prefix(flag))
                .and_then(|v| v.parse::<u32>().ok())
                .map(|mw| f64::from(mw) / 1000.0)
                .unwrap_or(0.0)
        };
        let stapm_w = watts_for("--stapm-limit=");
        let slow_w = watts_for("--slow-limit=");
        let fast_w = watts_for("--fast-limit=");
        output_with_stdout(&ryzenadj_info_table(slow_w, fast_w, stapm_w))
    }

    /// An `Output` with the given exit code and empty stdout/stderr.
    pub fn output_with_code(code: i32) -> Output {
        use std::os::unix::process::ExitStatusExt;
        Output {
            // Unix wait status: exit code lives in bits 8..16.
            status: std::process::ExitStatus::from_raw(code << 8),
            stdout: Vec::new(),
            stderr: Vec::new(),
        }
    }

    /// A successful (`output_with_code(0)`) `Output` whose stdout is `stdout`.
    pub fn output_with_stdout(stdout: &str) -> Output {
        let mut out = output_with_code(0);
        out.stdout = stdout.as_bytes().to_vec();
        out
    }

    /// A `ryzenadj --info`-shaped `| Name | Value | Parameter |` read-back
    /// table reporting the given slow/fast/STAPM watts, in the exact shape
    /// `ryzenadj --info` prints (see `tests/fixtures/ryzenadj_info.txt` and
    /// `actuators::cpu`'s own parser). Shared here (not private to
    /// `actuators::cpu`'s test module) so any actuator-module OR
    /// controller-level test can script a specific `WriteVerdict` — in
    /// particular a genuine `Mismatch` (a table that disagrees with what
    /// was actually commanded), not just the auto-agreeing default this
    /// `FakeRunner` otherwise synthesizes — through the real
    /// `CpuActuator::set_sustained_mw` write path.
    pub fn ryzenadj_info_table(slow_w: f64, fast_w: f64, stapm_w: f64) -> String {
        format!(
            "|        Name         |   Value   |     Parameter      |\n\
             |---------------------|-----------|--------------------|\n\
             | STAPM LIMIT         |{stapm_w:>11.3}| stapm-limit        |\n\
             | PPT LIMIT FAST      |{fast_w:>11.3}| fast-limit         |\n\
             | PPT LIMIT SLOW      |{slow_w:>11.3}| slow-limit         |\n"
        )
    }

    /// Queues one full write + read-back cycle — exactly the two `Runner`
    /// calls one `CpuActuator::set_sustained_mw` call makes (the write,
    /// then `ryzenadj --info`) — reporting the given table on the
    /// read-back regardless of what gets commanded. A table that disagrees
    /// with the actual command (e.g. an arbitrary slow-limit watts value
    /// outside the real operating range) yields a genuine
    /// `WriteVerdict::Mismatch` from the real write path; a table that
    /// agrees yields `Verified`. Each `set_sustained_mw` call needing a
    /// specific scripted read-back needs its own call to this (the
    /// re-read `CpuActuator::set_sustained_mw` makes when the first result
    /// comes back `Mismatch` is a second, independent call).
    pub fn queue_ryzenadj_readback(runner: &FakeRunner, slow_w: f64, fast_w: f64, stapm_w: f64) {
        runner.push_result(Ok(output_with_code(0))); // the write
        runner.push_result(Ok(output_with_stdout(&ryzenadj_info_table(
            slow_w, fast_w, stapm_w,
        ))));
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn fake_runner_records_calls_and_scripts_results() {
            let runner = FakeRunner::new();
            runner.push_result(Ok(output_with_code(1)));

            // Scripted result comes back first.
            let out = runner.run("modprobe", &["-r", "ryzen_smu"]).unwrap();
            assert!(!out.status.success());
            assert_eq!(out.status.code(), Some(1));

            // Queue drained: default is success with empty stdout.
            let out = runner.run("true", &[]).unwrap();
            assert!(out.status.success());
            assert!(out.stdout.is_empty());

            assert_eq!(
                runner.calls(),
                vec![
                    (
                        "modprobe".to_string(),
                        vec!["-r".to_string(), "ryzen_smu".to_string()]
                    ),
                    ("true".to_string(), vec![]),
                ]
            );
        }

        #[test]
        fn fake_runner_scripts_io_errors() {
            let runner = FakeRunner::new();
            runner.push_result(Err(io::Error::other("scripted failure")));
            assert!(runner.run("modprobe", &["ryzen_smu"]).is_err());
        }
    }
}
