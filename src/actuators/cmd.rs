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
    /// queue is empty, returns success with empty stdout (exit code 0).
    #[derive(Default)]
    pub struct FakeRunner {
        calls: RefCell<Vec<(String, Vec<String>)>>,
        results: RefCell<VecDeque<io::Result<Output>>>,
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
            self.calls.borrow_mut().push((
                program.to_string(),
                args.iter().map(|a| a.to_string()).collect(),
            ));
            self.results
                .borrow_mut()
                .pop_front()
                .unwrap_or_else(|| Ok(output_with_code(0)))
        }
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
