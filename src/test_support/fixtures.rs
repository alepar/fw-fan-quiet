//! Resolves paths into the checked-in `tests/fixtures/` corpus (Task 2 /
//! fwloop.20). Always anchored at `CARGO_MANIFEST_DIR` (a compile-time env
//! var), never at the process's current directory — `cargo test` runs test
//! binaries from varying working directories, so a relative path would
//! resolve differently depending on how the suite is invoked.

use std::path::PathBuf;

/// Resolve `tests/fixtures/<rel>` to an absolute path, anchored at
/// `CARGO_MANIFEST_DIR`. Panics if the resulting path does not exist:
/// fixtures are a checked-in corpus, so a missing one is a bug in the
/// caller or the corpus, not a runtime condition worth an `Option`/`Result`.
pub fn path(rel: &str) -> PathBuf {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(rel);
    assert!(p.exists(), "fixture not found: {}", p.display());
    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::Path;

    #[test]
    fn resolves_an_existing_fixture_to_an_absolute_path() {
        let p = path("fanctrl/print_all_quiet16.json");
        assert!(p.is_absolute(), "{} is not absolute", p.display());
        assert!(p.is_file(), "{} does not exist", p.display());
    }

    #[test]
    #[should_panic(expected = "fixture not found")]
    fn panics_on_a_fixture_that_does_not_exist() {
        path("fanctrl/does_not_exist.json");
    }

    /// The full set of files this task's bead requires to exist, relative
    /// to `tests/fixtures/`. A cros_ec hwmon tree contributes `name` plus
    /// eight `tempN_label` files and seven `tempN_input` files (temp8 —
    /// `gpu_temp@40` — has no `_input` file: the ENODATA convention, see
    /// `cros_ec_idle_gpu_temp_40_has_no_input_file` below).
    fn required_fixture_files() -> Vec<String> {
        let mut rels = vec![
            "fanctrl/print_all_quiet16.json".to_string(),
            "fanctrl/print_all_cool16.json".to_string(),
            "fanctrl/print_all_load.json".to_string(),
            "fanctrl/print_speed.json".to_string(),
            "hwmon/nvme/name".to_string(),
            "hwmon/nvme/temp1_label".to_string(),
            "hwmon/nvme/temp1_input".to_string(),
            "power_supply/ACAD/online".to_string(),
            "ryzenadj_info.txt".to_string(),
            "state_v1.json".to_string(),
        ];
        for tree in ["cros_ec_idle", "cros_ec_load", "cros_ec_dgpu_on"] {
            rels.push(format!("hwmon/{tree}/name"));
            for n in 1..=8 {
                rels.push(format!("hwmon/{tree}/temp{n}_label"));
            }
            for n in 1..=7 {
                rels.push(format!("hwmon/{tree}/temp{n}_input"));
            }
        }
        rels
    }

    #[test]
    fn every_required_fixture_file_exists_and_resolves() {
        for rel in required_fixture_files() {
            // path() itself panics (and so fails this test) if `rel` is
            // missing; the is_file check catches the file-vs-directory
            // mixup path() alone would not.
            let p = path(&rel);
            assert!(p.is_file(), "{rel} exists but is not a regular file");
        }
    }

    /// Every `tempN_label` file present under a cros_ec fixture tree,
    /// paired with the millidegree value of its `tempN_input` sibling (or
    /// `None` when that sibling is absent — the ENODATA convention).
    fn read_cros_ec_labelled(dir: &Path) -> Vec<(String, Option<i64>)> {
        let mut out = Vec::new();
        for n in 1..=8 {
            let label_path = dir.join(format!("temp{n}_label"));
            if !label_path.exists() {
                continue;
            }
            let label = fs::read_to_string(&label_path).unwrap().trim().to_string();
            let input_path = dir.join(format!("temp{n}_input"));
            let input = if input_path.exists() {
                Some(
                    fs::read_to_string(&input_path)
                        .unwrap()
                        .trim()
                        .parse::<i64>()
                        .unwrap_or_else(|e| panic!("{}: {e}", input_path.display())),
                )
            } else {
                None
            };
            out.push((label, input));
        }
        out
    }

    #[test]
    fn cros_ec_idle_gpu_temp_40_has_no_input_file() {
        // The ENODATA convention this task owns: a labelled sensor with no
        // `_input` sibling means "the kernel driver returned ENODATA", not
        // "the sensor reads zero". Task 8 keys its `EcReading` parser off
        // this exact absence.
        let dir = path("hwmon/cros_ec_idle");
        let entries = read_cros_ec_labelled(&dir);
        let gpu_temp_40 = entries
            .iter()
            .find(|(label, _)| label == "gpu_temp@40")
            .unwrap_or_else(|| panic!("gpu_temp@40 label file missing from {}", dir.display()));
        assert_eq!(
            gpu_temp_40.1, None,
            "gpu_temp@40 has an _input file — the ENODATA convention requires none"
        );
    }

    #[test]
    fn cros_ec_dgpu_on_has_no_positive_gpu_reading() {
        // §Facts, 2026-09-08: with the dGPU powered (NVML 18.9 W, P0), the
        // cros_ec gpu_amb/gpu_vr/gpu_vram sensors still read -150 and
        // gpu_temp@40 still returns ENODATA — they never report on this
        // machine. A positive gpu_* reading here would mean the fixture
        // silently reverted to the round-2 "they come alive" assumption
        // this task replaces.
        let dir = path("hwmon/cros_ec_dgpu_on");
        for (label, input) in read_cros_ec_labelled(&dir) {
            if !label.starts_with("gpu_") {
                continue;
            }
            if let Some(v) = input {
                assert!(v <= 0, "{label} reported a positive value: {v}");
            }
        }
    }

    /// Max over the positive `tempN_input` readings in a cros_ec fixture
    /// tree, in millidegrees — the replica rule §Facts records ("max over
    /// positive readings, rounded to integer °C").
    fn cros_ec_positive_max_millideg(dir: &Path) -> i64 {
        read_cros_ec_labelled(dir)
            .into_iter()
            .filter_map(|(_, v)| v)
            .filter(|&v| v > 0)
            .max()
            .expect("no positive reading in tree")
    }

    #[test]
    fn cros_ec_load_max_rounds_to_the_paired_print_all_temperature() {
        let dir = path("hwmon/cros_ec_load");
        let max_c = cros_ec_positive_max_millideg(&dir) as f64 / 1000.0;
        let rounded = max_c.round();
        assert_eq!(
            rounded, 75.0,
            "cros_ec_load max {max_c} does not round to 75"
        );

        let load_json = fs::read_to_string(path("fanctrl/print_all_load.json")).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&load_json).unwrap();
        let temperature = parsed["temperature"].as_f64().unwrap();
        assert_eq!(
            temperature, rounded,
            "print_all_load.json temperature does not match the load tree's rounded max"
        );
    }

    /// `(temp, duty)` points parsed straight out of the `cool16` strategy
    /// embedded in `print_all_cool16.json` — no dependency on `Curve`
    /// (Task 1 may not be on this branch yet).
    fn cool16_points() -> Vec<(f64, u8)> {
        let text = fs::read_to_string(path("fanctrl/print_all_cool16.json")).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&text).unwrap();
        let curve = parsed["configuration"]["data"]["strategies"]["cool16"]["speedCurve"]
            .as_array()
            .unwrap();
        curve
            .iter()
            .map(|p| {
                (
                    p["temp"].as_f64().unwrap(),
                    p["speed"].as_u64().unwrap() as u8,
                )
            })
            .collect()
    }

    /// Linear interpolation + truncation toward zero — the fixture-shape
    /// equivalent of `Curve::duty_at` (Task 1), used only to check the
    /// captured cool16 points reproduce the measured case, not as a
    /// production implementation.
    fn duty_at_truncated(points: &[(f64, u8)], t: f64) -> i64 {
        for w in points.windows(2) {
            let (t0, d0) = w[0];
            let (t1, d1) = w[1];
            if t >= t0 && t <= t1 {
                let frac = (t - t0) / (t1 - t0);
                let duty = d0 as f64 + frac * (d1 as f64 - d0 as f64);
                return duty as i64; // as-cast on f64->i64 truncates toward zero
            }
        }
        panic!("{t} is outside the curve's domain");
    }

    #[test]
    fn cool16_reproduces_the_measured_truncation_case() {
        // §Facts: "Verified truncation case on cool16: T_eff 51.8 -> duty 21."
        let points = cool16_points();
        assert_eq!(duty_at_truncated(&points, 51.8), 21);
    }

    #[test]
    fn cool16_segment_above_70c_has_slope_over_2_pct_per_c() {
        let points = cool16_points();
        let (t0, d0) = points[3];
        let (t1, d1) = points[4];
        assert_eq!((t0, d0), (70.0, 42), "cool16 point 3 is not (70, 42)");
        assert_eq!((t1, d1), (85.0, 100), "cool16 point 4 is not (85, 100)");
        let slope = (d1 as f64 - d0 as f64) / (t1 - t0);
        assert!(slope > 2.0, "cool16's >70C slope {slope} is not > 2.0 %/C");
    }
}
