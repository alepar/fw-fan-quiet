//! CPU sustained-power actuator (design doc §3): clamps and applies the
//! sustained package power via ryzenadj (STAPM + slow limit), while leaving
//! the stock fast (burst) limit untouched so short spikes stay responsive.
//! Stock restore works by toggling the ACPI platform profile: firmware
//! reasserts its own limits on a profile change (verified on this machine).

use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use super::WriteVerdict;
use super::cmd::Runner;

/// Production platform-profile sysfs node (main and FinalRestore build
/// their actuators against it).
pub const PLATFORM_PROFILE_PATH: &str = "/sys/firmware/acpi/platform_profile";

/// Absolute safety floor (design §5): below ~10 W risks UI stalls and
/// resume instability.
const MIN_SUSTAINED_MW: u32 = 10_000;
/// HX 370 cTDP ceiling.
const MAX_SUSTAINED_MW: u32 = 54_000;
/// Stock burst ceiling (verified on-machine invocation shape).
const DEFAULT_FAST_LIMIT_MW: u32 = 53_000;
/// Read-back agreement tolerance (design §2.9): slow and fast PPT limits
/// must land within this many watts of what was commanded to score
/// `Verified`. STAPM is excluded from this check entirely — written but not
/// required to verify (reported to fail silently on this SoC).
const MISMATCH_TOLERANCE_W: f64 = 0.5;
/// Dwell for the SECOND profile toggle when the first one's read-back shows
/// the commanded cap still in force (field 2026-09-10: a 200 ms toggle
/// issued within a second of a cap write left a 17 W slow limit behind; a
/// manual toggle with a 1 s dwell restored stock).
const RESTORE_RETRY_DWELL: Duration = Duration::from_secs(1);

/// The three `ryzenadj --info` table rows this actuator verifies against
/// (design §2.9), parsed as watts. `None` when a row is missing from the
/// table or its value is `nan` (unsupported on this PM-table version).
#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct InfoTable {
    ppt_limit_slow_w: Option<f64>,
    ppt_limit_fast_w: Option<f64>,
    stapm_limit_w: Option<f64>,
}

/// Parse `ryzenadj --info`'s `| Name | Value | Parameter |` table, picking
/// out `PPT LIMIT SLOW`, `PPT LIMIT FAST` and `STAPM LIMIT`. Every other row
/// (TDC/EDC/skin-temp/etc, `nan` on this SoC) is ignored. Lines that aren't
/// three-plus-column pipe rows (the preamble, the header, the `---`
/// separator) fail to match a tracked name and are skipped harmlessly.
fn parse_info_table(text: &str) -> InfoTable {
    let mut table = InfoTable::default();
    for line in text.lines() {
        let mut cols = line.split('|').map(str::trim);
        let Some(_before_leading_pipe) = cols.next() else {
            continue;
        };
        let Some(name) = cols.next() else { continue };
        let Some(value) = cols.next() else { continue };
        let value = value.parse::<f64>().ok().filter(|v| !v.is_nan());
        match name {
            "PPT LIMIT SLOW" => table.ppt_limit_slow_w = value,
            "PPT LIMIT FAST" => table.ppt_limit_fast_w = value,
            "STAPM LIMIT" => table.stapm_limit_w = value,
            _ => {}
        }
    }
    table
}

pub struct CpuActuator<R: Runner> {
    runner: R,
    /// Stock burst ceiling left untouched so short spikes stay fast
    /// (design §3). Pub so config (Task 21) can set it.
    pub fast_limit_mw: u32,
    /// Sustained operating ceiling (mW) commanded power is clamped to, from
    /// `Config::cpu_max_w`. Set via `set_sustained_max_mw` (which re-clamps to
    /// the hardware `MAX_SUSTAINED_MW`); defaults to that hardware ceiling.
    max_sustained_mw: u32,
    /// Production: /sys/firmware/acpi/platform_profile.
    profile_path: PathBuf,
    /// Pause between the profile toggle writes so firmware registers both.
    /// pub(crate) so guard tests can shorten it.
    pub(crate) toggle_delay: Duration,
    /// The last sustained limit this actuator successfully wrote (mW), `0`
    /// when none / already restored. `restore_stock` reads the hardware
    /// back against it: a restore is only believed once the slow limit has
    /// LEFT that value.
    last_commanded_mw: AtomicU32,
}

impl<R: Runner> CpuActuator<R> {
    pub fn new(runner: R, profile_path: PathBuf) -> Self {
        Self {
            runner,
            fast_limit_mw: DEFAULT_FAST_LIMIT_MW,
            max_sustained_mw: MAX_SUSTAINED_MW,
            profile_path,
            toggle_delay: Duration::from_millis(200),
            last_commanded_mw: AtomicU32::new(0),
        }
    }

    /// Set the sustained operating ceiling (from `Config::cpu_max_w`),
    /// re-clamped to the hardware envelope `[MIN_SUSTAINED_MW, MAX_SUSTAINED_MW]`
    /// so a bad config can never raise the cap above the silicon's cTDP.
    pub fn set_sustained_max_mw(&mut self, mw: u32) {
        self.max_sustained_mw = mw.clamp(MIN_SUSTAINED_MW, MAX_SUSTAINED_MW);
    }

    /// Clamp `mw` to `[MIN_SUSTAINED_MW, max_sustained_mw]`, command it as
    /// the sustained limit
    /// (`ryzenadj --stapm-limit=<mw> --slow-limit=<mw> --fast-limit=<fast>`),
    /// then run `ryzenadj --info` and score the read-back against what was
    /// just commanded (design §2.9). A failed write (spawn error or nonzero
    /// exit) is `Unreadable` too — there is nothing to have verified.
    pub fn set_sustained_mw(&self, mw: u32) -> WriteVerdict {
        let mw = mw.clamp(MIN_SUSTAINED_MW, self.max_sustained_mw);
        let args = [
            format!("--stapm-limit={mw}"),
            format!("--slow-limit={mw}"),
            format!("--fast-limit={}", self.fast_limit_mw),
        ];
        let args: Vec<&str> = args.iter().map(String::as_str).collect();
        match self.runner.run("ryzenadj", &args) {
            Ok(output) if output.status.success() => {}
            Ok(output) => {
                tracing::warn!(
                    "ryzenadj write failed ({}): {}",
                    output.status,
                    String::from_utf8_lossy(&output.stderr).trim()
                );
                return WriteVerdict::Unreadable;
            }
            Err(e) => {
                tracing::warn!("ryzenadj could not run: {e}");
                return WriteVerdict::Unreadable;
            }
        }
        self.last_commanded_mw.store(mw, Ordering::Relaxed);
        self.verify_write(mw)
    }

    /// Run `ryzenadj --info` and parse its table; `None` when the call fails
    /// or cannot be spawned (the `ryzen_smu`-loaded precondition) — the
    /// shared "Unreadable" case.
    fn read_info_table(&self) -> Option<InfoTable> {
        let output = match self.runner.run("ryzenadj", &["--info"]) {
            Ok(output) if output.status.success() => output,
            Ok(output) => {
                tracing::warn!(
                    "ryzenadj --info failed ({}): {}",
                    output.status,
                    String::from_utf8_lossy(&output.stderr).trim()
                );
                return None;
            }
            Err(e) => {
                tracing::warn!("ryzenadj --info could not run: {e}");
                return None;
            }
        };
        Some(parse_info_table(&String::from_utf8_lossy(&output.stdout)))
    }

    /// Run `ryzenadj --info` and score its read-back against the
    /// just-commanded slow/fast limits (design §2.9). A failed `--info`
    /// invocation (the documented `ryzen_smu`-loaded precondition failure)
    /// is `Unreadable`, never `Mismatch` — that distinction is what stops a
    /// module-load precondition from being read as a hardware fault. STAPM
    /// is parsed but never checked: written, not required to verify.
    fn verify_write(&self, commanded_mw: u32) -> WriteVerdict {
        let Some(table) = self.read_info_table() else {
            return WriteVerdict::Unreadable;
        };
        let (Some(slow_w), Some(fast_w)) = (table.ppt_limit_slow_w, table.ppt_limit_fast_w) else {
            return WriteVerdict::Unreadable;
        };
        let commanded_slow_w = f64::from(commanded_mw) / 1000.0;
        if (slow_w - commanded_slow_w).abs() > MISMATCH_TOLERANCE_W {
            return WriteVerdict::Mismatch {
                field: "PPT LIMIT SLOW",
                commanded: commanded_slow_w,
                read: slow_w,
            };
        }
        let commanded_fast_w = f64::from(self.fast_limit_mw) / 1000.0;
        if (fast_w - commanded_fast_w).abs() > MISMATCH_TOLERANCE_W {
            return WriteVerdict::Mismatch {
                field: "PPT LIMIT FAST",
                commanded: commanded_fast_w,
                read: fast_w,
            };
        }
        WriteVerdict::Verified(commanded_slow_w)
    }

    /// Restore stock CPU limits by toggling the platform profile: firmware
    /// reasserts its own limits on a profile change. Reads the current
    /// profile, writes a different one, waits, writes the original back.
    ///
    /// Then READS BACK (field 2026-09-10): a toggle issued within a second
    /// of a cap write left the cap in force while this logged success. If
    /// a cap was ever written and the slow limit still reads that value
    /// after the toggle, toggle again with [`RESTORE_RETRY_DWELL`]; if it
    /// STILL reads the cap, return an error naming the value so every
    /// caller logs it — a cap that survives a restore is the one outcome
    /// the design promises never to leave behind. A blind read-back
    /// (`--info` unavailable) is not a failure: nothing can be verified.
    /// (A commanded cap that happens to equal stock costs one spurious
    /// retry and error line; it never leaves a cap behind.)
    pub fn restore_stock(&self) -> io::Result<()> {
        self.toggle_profile(self.toggle_delay)?;
        let commanded_mw = self.last_commanded_mw.load(Ordering::Relaxed);
        if commanded_mw == 0 {
            return Ok(());
        }
        let commanded_w = f64::from(commanded_mw) / 1000.0;
        let still_capped = |slow_w: f64| (slow_w - commanded_w).abs() <= MISMATCH_TOLERANCE_W;
        match self.read_info_table().and_then(|t| t.ppt_limit_slow_w) {
            None => {
                tracing::debug!("stock restore read-back blind (ryzenadj --info unavailable)");
                self.last_commanded_mw.store(0, Ordering::Relaxed);
                Ok(())
            }
            Some(slow_w) if !still_capped(slow_w) => {
                tracing::info!("stock restore verified: slow limit now {slow_w} W");
                self.last_commanded_mw.store(0, Ordering::Relaxed);
                Ok(())
            }
            Some(slow_w) => {
                tracing::warn!(
                    "stock restore did not take: slow limit still {slow_w} W (commanded \
                     {commanded_w} W); toggling again with a {RESTORE_RETRY_DWELL:?} dwell"
                );
                self.toggle_profile(RESTORE_RETRY_DWELL)?;
                match self.read_info_table().and_then(|t| t.ppt_limit_slow_w) {
                    Some(slow_w) if still_capped(slow_w) => Err(io::Error::other(format!(
                        "stock CPU limits did not restore: slow limit still {slow_w} W after two \
                         platform-profile toggles (commanded {commanded_w} W)"
                    ))),
                    _ => {
                        self.last_commanded_mw.store(0, Ordering::Relaxed);
                        Ok(())
                    }
                }
            }
        }
    }

    /// One profile round trip: read the current profile, write a different
    /// one, wait `dwell`, write the original back.
    fn toggle_profile(&self, dwell: Duration) -> io::Result<()> {
        let original = std::fs::read_to_string(&self.profile_path)?;
        let original = original.trim();
        // A same-value write would be a no-op the firmware may ignore, so
        // toggle through a *different* profile.
        let intermediate = if original == "low-power" {
            "balanced"
        } else {
            "low-power"
        };
        // Crash window between the two writes is benign: firmware reasserts
        // stock limits on *any* profile change, so dying here leaves stock
        // power limits with only the user's profile preference lost.
        std::fs::write(&self.profile_path, intermediate)?;
        std::thread::sleep(dwell);
        std::fs::write(&self.profile_path, original)?;
        tracing::info!("restored stock CPU limits via platform profile toggle ({original})");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuators::cmd::test_support::{FakeRunner, output_with_code};
    use std::fs;
    use std::path::PathBuf;
    use std::process::Output;

    fn actuator(runner: FakeRunner) -> CpuActuator<FakeRunner> {
        CpuActuator::new(runner, PathBuf::from("/nonexistent/platform_profile"))
    }

    fn expected_write_args(mw: u32) -> Vec<String> {
        vec![
            format!("--stapm-limit={mw}"),
            format!("--slow-limit={mw}"),
            "--fast-limit=53000".to_string(),
        ]
    }

    /// A successful `Output` with the given stdout text. Thin local alias
    /// for the shared `cmd::test_support` builder (kept so this module's
    /// many existing call sites don't need a rename) — see that module's
    /// doc comment for why it lives there now, not here: `controller.rs`'s
    /// tests need the same table shape to script a real `WriteVerdict`
    /// through this actuator's write path.
    fn info_output(stdout: &str) -> Output {
        crate::actuators::cmd::test_support::output_with_stdout(stdout)
    }

    /// A `| Name | Value | Parameter |` table with the given slow/fast/stapm
    /// watt values, in the shape `ryzenadj --info` actually prints (see
    /// `tests/fixtures/ryzenadj_info.txt`). Thin local alias — see
    /// `info_output` above.
    fn info_table_text(slow_w: f64, fast_w: f64, stapm_w: f64) -> String {
        crate::actuators::cmd::test_support::ryzenadj_info_table(slow_w, fast_w, stapm_w)
    }

    #[test]
    fn set_clamps_low() {
        let cpu = actuator(FakeRunner::new());
        // No `--info` scripted: FakeRunner's default (empty stdout, exit 0)
        // parses to no rows found, so this only pins down the WRITE args —
        // the resulting verdict is exercised by the dedicated tests below.
        cpu.set_sustained_mw(5_000);
        assert_eq!(
            cpu.runner.calls()[0],
            ("ryzenadj".to_string(), expected_write_args(10_000))
        );
    }

    #[test]
    fn set_clamps_high() {
        let cpu = actuator(FakeRunner::new());
        cpu.set_sustained_mw(90_000);
        assert_eq!(
            cpu.runner.calls()[0],
            ("ryzenadj".to_string(), expected_write_args(54_000))
        );
    }

    #[test]
    fn set_passes_through_in_range() {
        let cpu = actuator(FakeRunner::new());
        cpu.set_sustained_mw(20_000);
        assert_eq!(
            cpu.runner.calls()[0],
            ("ryzenadj".to_string(), expected_write_args(20_000))
        );
    }

    #[test]
    fn write_failure_is_unreadable_and_skips_the_info_call() {
        let runner = FakeRunner::new();
        let mut failed = output_with_code(1);
        failed.stderr = b"Unable to get os_access Obj\n".to_vec();
        runner.push_result(Ok(failed));
        let cpu = actuator(runner);

        assert_eq!(cpu.set_sustained_mw(20_000), WriteVerdict::Unreadable);
        // Only the write was attempted -- a failed write never reaches for
        // a read-back that couldn't possibly confirm anything.
        assert_eq!(cpu.runner.calls().len(), 1);

        // Spawn failure on the write is Unreadable too.
        let runner = FakeRunner::new();
        runner.push_result(Err(io::Error::other("no such binary")));
        let cpu = actuator(runner);
        assert_eq!(cpu.set_sustained_mw(20_000), WriteVerdict::Unreadable);
    }

    /// Step 1 (TDD): the `ryzenadj --info` parser on the checked-in fixture
    /// extracts the three named rows as watts.
    #[test]
    fn parses_the_ryzenadj_info_fixture() {
        let text =
            fs::read_to_string(crate::test_support::fixtures::path("ryzenadj_info.txt")).unwrap();
        let table = parse_info_table(&text);
        assert_eq!(table.ppt_limit_slow_w, Some(48.000));
        assert_eq!(table.ppt_limit_fast_w, Some(65.000));
        assert_eq!(table.stapm_limit_w, Some(40.000));
    }

    #[test]
    fn parser_ignores_nan_rows() {
        let text = "| StapmTimeConst      |       nan | stapm-time         |\n";
        let table = parse_info_table(text);
        assert_eq!(table.stapm_limit_w, None);
        assert_eq!(table.ppt_limit_slow_w, None);
        assert_eq!(table.ppt_limit_fast_w, None);
    }

    /// Step 2 (TDD): commanding a value the fixture table agrees with
    /// (within 0.5 W on slow and fast) yields `Verified`; a disagreeing
    /// STAPM does not break verification.
    #[test]
    fn agreeing_read_back_yields_verified_even_with_stapm_disagreeing() {
        let runner = FakeRunner::new();
        // Write succeeds (FakeRunner default), then --info reports a table
        // that agrees with the commanded slow/fast but NOT with STAPM
        // (commanded 48 W; table says STAPM is stuck at 40 W).
        runner.push_result(Ok(output_with_code(0))); // the write
        runner.push_result(Ok(info_output(&info_table_text(48.000, 65.000, 40.000))));
        let mut cpu = actuator(runner);
        cpu.fast_limit_mw = 65_000; // matches the fixture's PPT LIMIT FAST

        let verdict = cpu.set_sustained_mw(48_000);

        assert_eq!(verdict, WriteVerdict::Verified(48.0), "got {verdict:?}");
    }

    /// Within-tolerance disagreement (< 0.5 W) still verifies.
    #[test]
    fn read_back_within_tolerance_still_verifies() {
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0)));
        runner.push_result(Ok(info_output(&info_table_text(48.400, 65.400, 40.000))));
        let mut cpu = actuator(runner);
        cpu.fast_limit_mw = 65_000;

        assert_eq!(cpu.set_sustained_mw(48_000), WriteVerdict::Verified(48.0));
    }

    /// Step 3 (TDD): a fake `Runner` returning a stale table (read-back
    /// still shows the OLD limit) yields `Mismatch` naming the offending
    /// field.
    #[test]
    fn stale_read_back_yields_mismatch_naming_the_field() {
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0))); // the write "succeeds"
        // But --info still reports the previous (stale) limit: 40 W, not
        // the freshly-commanded 48 W.
        runner.push_result(Ok(info_output(&info_table_text(40.000, 65.000, 40.000))));
        let mut cpu = actuator(runner);
        cpu.fast_limit_mw = 65_000;

        let verdict = cpu.set_sustained_mw(48_000);

        assert_eq!(
            verdict,
            WriteVerdict::Mismatch {
                field: "PPT LIMIT SLOW",
                commanded: 48.0,
                read: 40.0,
            }
        );
    }

    #[test]
    fn stale_fast_read_back_names_the_fast_field() {
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0)));
        // Slow agrees; fast is stuck at the stock 53 W default.
        runner.push_result(Ok(info_output(&info_table_text(48.000, 53.000, 40.000))));
        let mut cpu = actuator(runner);
        cpu.fast_limit_mw = 65_000;

        let verdict = cpu.set_sustained_mw(48_000);

        assert_eq!(
            verdict,
            WriteVerdict::Mismatch {
                field: "PPT LIMIT FAST",
                commanded: 65.0,
                read: 53.0,
            }
        );
    }

    /// Step 4 (TDD): a fake `Runner` whose `--info` invocation fails yields
    /// `Unreadable` -- explicitly not `Mismatch` (design §2.9: a module-load
    /// precondition failure must never be read as a hardware fault).
    #[test]
    fn failed_info_call_is_unreadable_not_mismatch() {
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0))); // the write succeeds
        // `ryzenadj --info` fails: the ryzen_smu-loaded precondition.
        let mut failed_info = output_with_code(1);
        failed_info.stderr = b"no compatible ryzen_smu kernel module found\n".to_vec();
        runner.push_result(Ok(failed_info));
        let cpu = actuator(runner);

        let verdict = cpu.set_sustained_mw(48_000);

        // A wrong implementation that fell through to reading whatever
        // partial/garbage table a nonzero-exit `--info` still printed would
        // fail this: it would score a Mismatch (or even a spurious
        // Verified) instead of recognising the exit code and stopping.
        assert_eq!(
            verdict,
            WriteVerdict::Unreadable,
            "a failed --info must be Unreadable, never scored as Mismatch"
        );
    }

    #[test]
    fn info_spawn_failure_is_unreadable() {
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0))); // the write succeeds
        runner.push_result(Err(io::Error::other("no such binary")));
        let cpu = actuator(runner);

        assert_eq!(cpu.set_sustained_mw(48_000), WriteVerdict::Unreadable);
    }

    /// A wedged `ryzenadj` now surfaces as `io::ErrorKind::TimedOut` from
    /// the bounded `Runner` (see `actuators::cmd::run_with_timeout`) instead
    /// of hanging the controller thread. Both legs of §2.9 must score that
    /// as `Unreadable` — read-back blind — never as a hardware `Mismatch`.
    #[test]
    fn a_timed_out_ryzenadj_is_read_back_blind_not_a_mismatch() {
        let timed_out = || io::Error::new(io::ErrorKind::TimedOut, "`ryzenadj` did not exit");

        // The write itself wedged: nothing was commanded, nothing to verify.
        let runner = FakeRunner::new();
        runner.push_result(Err(timed_out()));
        assert_eq!(
            actuator(runner).set_sustained_mw(48_000),
            WriteVerdict::Unreadable
        );

        // The `--info` read-back wedged: the write landed, the verdict is blind.
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0)));
        runner.push_result(Err(timed_out()));
        assert_eq!(
            actuator(runner).set_sustained_mw(48_000),
            WriteVerdict::Unreadable
        );
    }

    #[test]
    fn missing_rows_in_the_read_back_table_are_unreadable() {
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0)));
        // A table missing the SLOW row entirely (e.g. a truncated read).
        runner.push_result(Ok(info_output(
            "| STAPM LIMIT         |    40.000 | stapm-limit        |\n\
             | PPT LIMIT FAST      |    65.000 | fast-limit         |\n",
        )));
        let cpu = actuator(runner);

        assert_eq!(cpu.set_sustained_mw(48_000), WriteVerdict::Unreadable);
    }

    /// Unique-per-test profile file fixture; caller removes the dir when done.
    fn profile_fixture(name: &str, content: &str) -> (PathBuf, PathBuf) {
        let dir =
            std::env::temp_dir().join(format!("bazerame-cpu-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("platform_profile");
        fs::write(&path, content).unwrap();
        (dir, path)
    }

    fn fast_actuator(profile_path: PathBuf) -> CpuActuator<FakeRunner> {
        let mut cpu = CpuActuator::new(FakeRunner::new(), profile_path);
        cpu.toggle_delay = Duration::from_millis(1); // keep tests fast
        cpu
    }

    #[test]
    fn restore_toggles_profile() {
        let (dir, path) = profile_fixture("balanced", "balanced\n");
        let cpu = fast_actuator(path.clone());

        cpu.restore_stock().unwrap();

        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn restore_from_low_power_toggles_via_balanced() {
        let (dir, path) = profile_fixture("low-power", "low-power\n");
        let cpu = fast_actuator(path.clone());

        cpu.restore_stock().unwrap();

        assert_eq!(fs::read_to_string(&path).unwrap(), "low-power");
        fs::remove_dir_all(&dir).unwrap();
    }

    /// Field 2026-09-10: the first toggle's read-back still shows the cap;
    /// a second toggle (longer dwell) clears it. Ok, and every `--info`
    /// read happened (verify, read-back, read-back).
    #[test]
    fn restore_reads_back_and_retries_once_when_the_cap_survives() {
        let (dir, path) = profile_fixture("restore-retry", "balanced\n");
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0))); // the 17 W write
        runner.push_result(Ok(info_output(&info_table_text(17.0, 53.0, 17.0)))); // verify
        runner.push_result(Ok(info_output(&info_table_text(17.0, 53.0, 17.0)))); // still capped
        runner.push_result(Ok(info_output(&info_table_text(45.0, 65.0, 45.3)))); // restored
        let mut cpu = CpuActuator::new(runner, path.clone());
        cpu.toggle_delay = Duration::from_millis(1);
        assert_eq!(cpu.set_sustained_mw(17_000), WriteVerdict::Verified(17.0));

        cpu.restore_stock().unwrap();

        let infos = cpu
            .runner
            .calls()
            .iter()
            .filter(|(_, a)| a == &vec!["--info".to_string()])
            .count();
        assert_eq!(infos, 3, "verify + two restore read-backs");
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        // A restored actuator does not read back again on the next restore.
        cpu.restore_stock().unwrap();
        assert_eq!(cpu.runner.calls().len(), 4, "no further --info without a new write");
        fs::remove_dir_all(&dir).unwrap();
    }

    /// The cap survives BOTH toggles: the error names the surviving value so
    /// every caller's `warn!` carries it.
    #[test]
    fn restore_reports_a_cap_that_survives_two_toggles() {
        let (dir, path) = profile_fixture("restore-stuck", "balanced\n");
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(0)));
        runner.push_result(Ok(info_output(&info_table_text(17.0, 53.0, 17.0))));
        runner.push_result(Ok(info_output(&info_table_text(17.0, 53.0, 17.0))));
        runner.push_result(Ok(info_output(&info_table_text(17.0, 53.0, 17.0))));
        let mut cpu = CpuActuator::new(runner, path.clone());
        cpu.toggle_delay = Duration::from_millis(1);
        cpu.set_sustained_mw(17_000);

        let err = cpu.restore_stock().unwrap_err();

        assert!(err.to_string().contains("17 W"), "{err}");
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        fs::remove_dir_all(&dir).unwrap();
    }

    /// No cap was ever written: nothing to verify against, no `--info`.
    #[test]
    fn restore_without_a_prior_write_does_not_read_back() {
        let (dir, path) = profile_fixture("restore-blind", "balanced\n");
        let cpu = fast_actuator(path);
        cpu.restore_stock().unwrap();
        assert!(cpu.runner.calls().is_empty());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn restore_missing_file_is_err() {
        let cpu = fast_actuator(PathBuf::from("/nonexistent/platform_profile"));
        assert!(cpu.restore_stock().is_err());
    }
}
