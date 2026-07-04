//! Hardware selftest (`bazerame-fans selftest`): exercises the Milestone 2
//! actuator + sensor stack end to end and prints one plain
//! `[ OK ]/[FAIL]/[SKIP] step: detail` line per step. No TUI, no file
//! logging — this runs from a terminal as root and its stdout IS the report.
//!
//! Safety: the working actuators live in a [`RestoreGuard`] on this (single)
//! thread's stack, so an unexpected panic mid-test still unwinds into
//! `restore_all`. The happy path restores each actuator explicitly so every
//! restore step gets its own OK/FAIL line.

use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::actuators::cmd::RealRunner;
use crate::actuators::cpu::{CpuActuator, PLATFORM_PROFILE_PATH};
use crate::actuators::gpu::GpuActuator;
use crate::actuators::guard::RestoreGuard;
use crate::actuators::smu_module::SmuModule;
use crate::calib::burner::Burner;
use crate::sensors::rapl::RaplReader;

/// RAPL package zone; `energy_uj` inside it is root-readable only, which
/// doubles as the root check.
const RAPL_ZONE: &str = "/sys/class/powercap/intel-rapl:0";

/// Sustained CPU limit commanded for the test.
const CPU_LIMIT_MW: u32 = 20_000;
/// Measured package watts under full burn must come in at or below this.
/// 2 W of slack over the 20 W command covers RAPL-vs-STAPM domain skew.
const CPU_WATTS_THRESHOLD: f64 = 22.0;
/// Full-machine burn (24 hardware threads on this box).
const BURN_THREADS: usize = 24;
/// GPU max-clock lock commanded for the test.
const GPU_LOCK_MHZ: u32 = 1200;

const BASELINE_SECS: u64 = 3;
/// STAPM/slow limits take a few seconds to bite after ryzenadj applies them;
/// wait this long under load before opening the measurement window.
const LIMIT_SETTLE_SECS: u64 = 10;
const MEASURE_SECS: u64 = 5;

/// One `[ OK ]` / `[FAIL]` report line.
fn format_step(desc: &str, result: &Result<String, String>) -> String {
    match result {
        Ok(msg) => format!("[ OK ] {desc}: {msg}"),
        Err(msg) => format!("[FAIL] {desc}: {msg}"),
    }
}

/// One `[SKIP]` report line (does not affect the exit code).
fn format_skip(desc: &str, msg: &str) -> String {
    format!("[SKIP] {desc}: {msg}")
}

/// Runs the full selftest sequence. Returns the process exit code:
/// 0 iff every non-skipped step passed.
pub fn run() -> i32 {
    let mut all_ok = true;
    let mut check = |desc: &str, result: Result<String, String>| -> bool {
        println!("{}", format_step(desc, &result));
        let ok = result.is_ok();
        all_ok &= ok;
        ok
    };

    // 1. Root check: without RAPL read access nothing below can work.
    let root = std::fs::File::open(Path::new(RAPL_ZONE).join("energy_uj"))
        .map(|_| "RAPL energy counter readable (running as root)".to_string())
        .map_err(|e| format!("cannot open {RAPL_ZONE}/energy_uj: {e} — run with sudo"));
    if !check("root check", root) {
        return 1;
    }

    // 2. ryzen_smu workaround (blocks ryzenadj while loaded without pm_table).
    let smu = match SmuModule::ensure_unloaded(&RealRunner, Path::new("/sys/kernel")) {
        Ok(smu) => {
            let msg = if smu.unloaded_by_us() {
                "ryzen_smu unloaded (will reload at the end)"
            } else {
                "ryzen_smu not loaded or already usable; nothing to do"
            };
            check("smu module", Ok(msg.to_string()));
            Some(smu)
        }
        Err(e) => {
            check("smu module", Err(e.to_string()));
            None
        }
    };

    // 3. Actuators + stack guard (panic safety) + startup reset.
    let cpu = CpuActuator::new(RealRunner, PathBuf::from(PLATFORM_PROFILE_PATH));
    let gpu_result = GpuActuator::new();
    let gpu_msg = match &gpu_result {
        Ok(_) => Ok("cpu + gpu constructed (NVML device 0 present)".to_string()),
        Err(e) => Err(format!("GPU actuator init failed: {e}")),
    };
    let mut guard = RestoreGuard::new(RealRunner, Some(cpu), gpu_result.ok(), smu);
    guard.startup_reset();
    check(
        "actuators",
        gpu_msg.map(|m| format!("{m}; startup reset done")),
    );

    // 4. Baseline package power.
    let mut rapl = RaplReader::new(Path::new(RAPL_ZONE));
    let baseline = match rapl.as_mut() {
        None => Err(format!("RAPL zone {RAPL_ZONE} unreadable")),
        Some(r) => {
            r.read_watts(); // arm the counter window
            std::thread::sleep(Duration::from_secs(BASELINE_SECS));
            r.read_watts()
                .map(|w| format!("{w:.1} W avg over {BASELINE_SECS} s (pre-limit)"))
                .ok_or_else(|| "RAPL read failed".to_string())
        }
    };
    check("baseline power", baseline);

    // 5. CPU limit under full burn: the core Milestone 2 assertion.
    check("cpu limit", cpu_limit_step(&guard, rapl.as_mut()));

    // 6. GPU max-clock lock. Without GPU load the current clock idles low, so
    // the honest observable is that the lock COMMAND succeeded.
    match guard.gpu.as_mut() {
        None => println!(
            "{}",
            format_skip("gpu clock lock", "gpu actuator unavailable")
        ),
        Some(g) => {
            let result = g
                .set_max_clock(GPU_LOCK_MHZ)
                .map(|()| {
                    format!(
                        "gpu clock lock commanded (applied={} MHz)",
                        g.applied().unwrap_or(0)
                    )
                })
                .map_err(|e| format!("set_max_clock({GPU_LOCK_MHZ}) failed: {e}"));
            check("gpu clock lock", result);
        }
    }

    // 7. Restore everything, one report line per actuator. Items are take()n
    // out of the guard so its Drop (the panic net) becomes a no-op afterwards.
    match guard.gpu.take() {
        None => println!("{}", format_skip("gpu release", "no gpu actuator")),
        Some(mut g) => {
            let result = g
                .release()
                .map(|()| "clock locks reset to default".to_string())
                .map_err(|e| e.to_string());
            check("gpu release", result);
        }
    }
    if let Some(cpu) = guard.cpu.take() {
        let result = cpu
            .restore_stock()
            .map(|()| "stock limits reasserted (platform profile toggle)".to_string())
            .map_err(|e| e.to_string());
        check("cpu restore", result);
    }
    match guard.smu.take() {
        None => println!(
            "{}",
            format_skip("smu restore", "no smu handle (unload failed earlier)")
        ),
        Some(mut smu) => {
            let msg = if smu.unloaded_by_us() {
                smu.restore(&RealRunner); // logs its own outcome; lsmod verifies
                "ryzen_smu reload commanded"
            } else {
                "ryzen_smu untouched (we did not unload it)"
            };
            check("smu restore", Ok(msg.to_string()));
        }
    }

    if all_ok {
        println!("selftest: PASS");
        0
    } else {
        println!("selftest: FAIL");
        1
    }
}

/// Command 20 W sustained, load all threads, wait for the limit to bite,
/// then measure a 5-s RAPL average and compare against the threshold.
fn cpu_limit_step(
    guard: &RestoreGuard<RealRunner>,
    rapl: Option<&mut RaplReader>,
) -> Result<String, String> {
    let cpu = guard.cpu.as_ref().ok_or("no CPU actuator")?;
    let rapl = rapl.ok_or("RAPL reader unavailable")?;
    let mw = cpu
        .set_sustained_mw(CPU_LIMIT_MW)
        .map_err(|e| format!("ryzenadj: {e}"))?;

    let burner = Burner::start(BURN_THREADS);
    std::thread::sleep(Duration::from_secs(LIMIT_SETTLE_SECS));
    rapl.read_watts(); // discard the ramp window; opens the measurement window
    std::thread::sleep(Duration::from_secs(MEASURE_SECS));
    let watts = rapl.read_watts();
    burner.stop();

    let watts = watts.ok_or("RAPL read failed during burn")?;
    let detail = format!(
        "measured {watts:.1} W avg over {MEASURE_SECS} s under {BURN_THREADS}-thread burn \
         (commanded {mw} mW, threshold {CPU_WATTS_THRESHOLD} W)"
    );
    if watts <= CPU_WATTS_THRESHOLD {
        Ok(detail)
    } else {
        Err(detail)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_step_ok() {
        assert_eq!(
            format_step("cpu limit", &Ok("measured 19.8 W".to_string())),
            "[ OK ] cpu limit: measured 19.8 W"
        );
    }

    #[test]
    fn format_step_fail() {
        assert_eq!(
            format_step("cpu limit", &Err("measured 35.2 W".to_string())),
            "[FAIL] cpu limit: measured 35.2 W"
        );
    }

    #[test]
    fn format_skip_line() {
        assert_eq!(
            format_skip("gpu clock lock", "gpu actuator unavailable"),
            "[SKIP] gpu clock lock: gpu actuator unavailable"
        );
    }
}
