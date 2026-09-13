# Task 16 implementation report

**Status:** DONE

## Result

Removed the retired scalar controller and its GPU watts conversion path. Runtime control now has only the revision-4 shared T* source, independent CPU/GPU `DeviceLoop`s, per-device hot-guard ratchets, paired verification, qualified seeds, and native per-device calibration.

## Deletion manifest

Deleted:

- `src/control/budget.rs`
- `src/control/allocator.rs`
- `src/control/spike_antiwindup.rs`
- `src/control/mode.rs`
- `src/control/lut.rs`
- `src/control/gpu_pid.rs`
- `src/calib/lut_sweep.rs`

Removed their module declarations plus the old controller state, scalar warm-start helpers, calibration runner/effects, scalar fit wrapper, config/state fields, telemetry columns, fixtures, and UI/status compatibility surface. `Config::gpu_max_w`, `PersistedState::{lut,loop_gains}`, `LoopMode`, `AutoAllocated`, and the deprecated decision columns no longer exist as runtime fields or types.

Surviving semantics were moved or retained before deletion:

- CPU hardware bounds remain in `config.rs`.
- Paired warm-start keys use `state::warm_start_key(strategy, duty, on_ac)`.
- Calibration uses the per-device runner and scores EC reconciliation with the same fresh-view slope/age/mismatch/reseed gates as Auto.
- GPU watt chart scaling is presentation-only (`GPU_POWER_SCALE_W`), independent of controller configuration.
- `NOT CALIBRATED` is keyed to missing fitted CPU/GPU gains for the active strategy/interval.
- Raw GPU watt sensing and the physical simulation power model remain because they are measurements, not a watts-to-clock control loop.

## Safety coverage preserved

`control::sim_tests` remains a nonzero suite with revision-4 tests for actuator-mismatch hold/recovery, hot-guard ratchet floor/recovery, and three-strike EC mismatch/three-match recovery. The eleven `sim::scenarios` acceptance tests remain and cover bumpless entry/recovery, absent draw, Curve/Held transitions and restart seeds, device isolation, exhaustive states/flags, CPU/GPU guard episodes, implausible sensors, resume/verifier behavior, and watchdog/reconciliation paths. The integration flow now asserts `Curve -> Held -> Curve` through the T* state rather than the deleted mode aliases.

The live scenarios formerly embedded in the legacy `sim_tests` harness map to revision-4 controller tests as follows: absent socket/invalid fan release and recovery maps to `fan_invalid_with_no_fanctrl_view_releases_to_stock` plus `review_released_clears_engagement_and_releases_only_on_transition`; a non-sticking GPU lock maps to `review_gpu_mismatch_rewrites_then_release_does_not_relock_same_sample`; idle-dGPU unverifiable readback maps to `verdictstate_unreadable_and_unverifiable_are_non_events`; three confirmed CPU mismatches and subsequent recovery map to `cpu_mismatch_freezes_flags_reasserts_and_releases_to_stock_then_a_later_verified_recovers`; AC-edge suppression maps to `verdictstate_suppressed_mismatch_near_an_on_ac_edge_is_never_scored` plus `review_ac_suppressed_gpu_mismatch_does_not_latch_device_loop`; A-to-B-to-A EC reconciliation maps to `reconciliation_is_scored_on_the_1hz_sample_carrying_the_view_not_the_5s_tick` plus `review_raw_reconciliation_and_implausible_diagnostics_remain_distinct`; and resume with an absent GPU maps to `review_resume_discards_mismatch_hold_and_quit_clears_decisions`, `review_resume_records_every_completed_reassert_without_scoring_its_sample`, and the paired-warm-start entry tests. GPU/NVMe guards, emergency release, shutdown fencing, and watchdog sensor loss remain under their corresponding named controller tests. The three focused `control::sim_tests` cases retain lower-level assertions for mismatch hold/recovery, ratchet recovery, and EC strike clearing.

The round-one review found that the initial deletion had also removed live unit coverage. The final tree restores that coverage and classifies every removed base test:

- `control::controller::tests`: 114 of 139 base tests were restored or adapted to the revision-4 API. These cover shutdown/emergency release, sensor loss and resume, actuator verification/mismatch recovery, per-device ratchets, paired warm starts, EC reconciliation, calibration ownership/abort/freeze, bumpless transitions, status emission, persistence, and production `apply_effects` telemetry. The telemetry test runs a real Controller Auto sample through `apply_effects` and JSON, requires populated T*/CPU/GPU fields and applied caps, and requires every retired decision key to be absent.
- Round two restored the live portions of five initially misclassified tests. Strategy, AC-source, and snapped-duty changes prove that the steady window re-keys without consuming a hostile paired warm start, device candidates follow the ordinary step, and the settled pair records under the new key. Released re-entry first engages without the distinctive record, verifies Released reset both device engagements, inserts the record, and proves fresh recovery consumes both caps. Achieved-duty mismatch first accumulates exactly 30 qualified samples, proves one mismatched tread resets the window and the next match restarts at one, and proves neither a warm record nor table refinement lands.
- The 25 controller tests removed as obsolete asserted only deleted scalar/LUT mechanics: `control_status_carries_the_new_loop_fields`, `control_status_default_has_no_loop_state_yet`, `auto_allocated_carries_the_new_arbiter_fields`, `persisted_state_seeds_controller_lut`, `raising_the_gpu_floor_against_a_soft_capped_gpu_max_w_does_not_panic`, `a_lut_landing_at_save_state_re_clamps_an_unaffordable_gpu_floor`, `set_floors_shifts_the_next_allocation`, `gpu_pi_seeded_from_applied_lock_at_entry`, `pi_rate_reference_tracks_hardware_on_failed_gpu_set`, `the_controller_sample_period_is_the_samplers_own_cadence`, `temploop_entry_takes_the_spec_15_seconds_at_the_1_hz_sample_cadence`, `temploop_tick_computes_t_star_minus_ma_and_moves_the_budget`, `sign3_is_neutral_on_zero_and_nan_unlike_signum`, `an_exactly_zero_loop_error_does_not_arm_the_demand_limited_halt`, `an_exactly_zero_loop_error_does_not_arm_the_high_unreachable_rule`, `calib_budget_w_stays_frozen_through_per_device_calibration`, `mode_transitions_emit_noted`, `resumed_sample_clears_the_ec_boxcar`, `demand_limited_anti_windup_eventually_holds_an_idle_budget_off_the_ceiling`, `gpu_hot_episode_ratchets_the_cap_without_spuriously_triggering_demand_limited`, `steady_window_on_the_smoothed_series_records_warm_start_and_refines_the_table`, `steady_window_rejects_smoothed_rpm_outside_target_tolerance`, `auto_entry_seeds_u_from_a_matching_warm_start_key`, `reengaging_from_released_reseeds_the_budget_without_a_step`, and `fan_dropout_clears_fan_valid_and_drops_rpmloop_within_one_sample`.
- `calib::step`: all 18 native per-device tests were restored. The 14 removed tests (`enter_starts_the_burner_and_holds_the_floor_before_any_gate_is_checked`, `burner_stops_only_after_the_step_concludes`, `idle_uncontrollable_argmax_does_not_self_skip_once_loaded_argmax_is_controllable`, `settle_requires_both_ec_flat_and_rpm_steady`, `settle_gives_up_at_the_five_minute_cap_and_keeps_defaults`, `step_requests_floor_plus_thirty_watts`, `gpu_below_pin_threshold_nags_needs_load_every_ten_samples`, `gain_is_derived_from_the_measured_delta_not_the_nominal_thirty_watts`, `successful_fit_stamps_fitted_at_from_the_sample_clock`, `unloaded_step_skips_with_noted_reason_and_keeps_defaults`, `min_step_delta_boundary_is_inclusive`, `ec_over_95c_aborts_mid_step_and_restores_the_floor`, `argmax_label_change_mid_step_skips_and_keeps_defaults`, and `a_rejected_fit_skips_with_noted_reason_and_keeps_defaults`) exercised the deleted scalar step harness. Their live timing, cross-term, verified-pair, guard, key-drift, fit, and restore assertions are covered by the 18 per-device tests.
- `calib::fopdt`: 8 live fit/native-gain tests were restored. Four scalar/RPM-only tests were removed: `derive_gains_uses_fitted_theta_as_is_not_plus_ma_interval_half`, `derive_gains_sets_ti_equal_to_tau_per_signal`, `fit_fopdt_rejects_fan_response_under_150_rpm`, and `duty_pinned_step_passes_magnitude_but_derive_gains_rejects_the_ratio_band`.
- Restored current non-controller coverage totals are config 21, state 17, telemetry 13, and EC/Boxcar/replica 27 tests. Removed config/state cases were limited to deleted Rust LUT/scalar fields; raw migration-key tests remain. The GPU chart has a regression test pinning the fixed presentation-scale wording.

## Documentation

Rewrote `README.md` around the implemented revision-4 behavior and schema. Marked the 2026-09-07 spec, 2026-09-11 seed, and research 03/05 documents historical/superseded with links to revision 4. Updated the specs index to identify revision 4 as current and implemented. Immutable run/review/plan evidence was preserved.

## Deletion audit

The final basename command:

```sh
find . -type f \( -name budget.rs -o -name allocator.rs -o -name spike_antiwindup.rs -o -name mode.rs -o -name lut.rs -o -name gpu_pid.rs -o -name lut_sweep.rs \) -print
```

returned zero paths.

The final runtime-symbol search across `src`, `README.md`, and `TODO.md` for `ClockWattsLut`, `GpuPid`, `gpu_share_override`, `shadow_band`, `AutoAllocated`, `TempLoop`, `RpmLoop`, `WarmStart::`, `LoopGains`, `Freeze`, `Budget`, `split_budget`, pinned margins, demand margins, and deprecated telemetry keys returned only explicit negative serialization/UI assertions, the current `PerDeviceStepTest` type, and generic English uses: LED shutdown “freeze” and external-command wall-clock “budget”. The negative assertions prove `demand_cpu`, `demand_gpu`, `alloc_cpu_w`, `alloc_gpu_w`, `pi_target_w`, `budget_w`, and `freeze` do not serialize.

The current-document false-zero search across `AGENTS.md`, `CLAUDE.md`, `README.md`, `TODO.md`, and `docs/superpowers/specs/INDEX.md` for clock/watts tables, LUT, scalar/power budget, split, allocator, Mode A/B, TempLoop/RpmLoop, pinned, and shadow-band terms returned zero hits.

Allowed migration hits are exactly:

- `src/config.rs`: removes/warns/tests the raw TOML key `gpu_max_w` while retaining `gpu_max_mhz`.
- `src/state.rs`: removes/warns/tests the raw JSON keys `lut` and `loop_gains` while retaining current siblings.
- `tests/fixtures/state_v1.json`: historical v1 migration input containing `lut`.

Generic non-legacy words classified separately are LED shutdown “freeze” and the external-command wall-clock “budget”. Historical specs, runs, reviews, plans, and research evidence retain their old vocabulary under explicit historical markers or immutable-artifact policy.

## Validation

- `cargo check --all-targets` — passed without warnings.
- `cargo test control::sim_tests -- --nocapture` — 3 passed, nonzero.
- `cargo test calib::runner::per_device_runner_tests -- --format=terse` — 3 passed.
- `cargo test every_telemetry_field_is_populated -- --nocapture` — 1 passed.
- `cargo test engage_walk_calibrate_and_restart_migrates_legacy_state_safely -- --nocapture` — 1 passed.
- `cargo test calib::step::per_device_behavior_tests -- --format=terse` — 18 passed.
- `cargo test calib::fopdt::tests -- --format=terse` — 8 passed.
- `cargo test config::tests -- --format=terse` — 21 passed.
- `cargo test state::tests -- --format=terse` — 17 passed.
- `cargo test telemetry::tests -- --format=terse` — 13 passed.
- `cargo test sensors::ec::tests -- --format=terse` — 27 passed.
- `cargo test control::controller::tests -- --format=terse` — 118 passed. This includes three live re-key cases, released re-entry, achieved-duty gating, exhaustive calibration context provenance, and calibration reconciliation skip/latch/reseed coverage.
- `cargo test released_reentry_consumes_the_matching_paired_warm_start -- --nocapture` — 1 passed.
- `cargo test mismatched_achieved_duty_restarts_the_steady_window -- --nocapture` — 1 passed.
- `cargo test integration_tests::wiring_sweep::every_telemetry_field_is_populated -- --nocapture` — 1 passed.
- `cargo test ui::view::tests::gpu_watts_chart_names_its_fixed_presentation_scale -- --format=terse` — 1 passed.
- `cargo test --no-fail-fast -- --format=terse` — 691 passed, 0 failed, 2 ignored; 162.23 s on the final tree.
- `cargo clippy --all-targets -- -D warnings` — passed.
- `git diff --check` — passed.

`cargo fmt -- --check` still reports broad pre-existing formatting drift, including untouched files and large files inherited from earlier tasks. No repository-wide formatter rewrite was applied, per the task constraint against collateral formatting changes.

## Concerns

No hardware was accessed. The two ignored tests remain the existing explicit hardware tests.
