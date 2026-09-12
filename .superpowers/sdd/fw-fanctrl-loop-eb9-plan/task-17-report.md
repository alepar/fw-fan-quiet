# Task 17 implementation report

**Status:** DONE

## Result

Completed the revision-4 final integration sweep. The sweep found and fixed two small live controller defects: raw sample gaps were clipped before `TStarSource` could recognize an unmarked wall gap, and the Auto-only `NotCalibrated` flag survived into Monitor. It also replaced vacuous integration assertions with causal restart, resume, state-transition and group-recovery coverage.

## Coverage and fixes

- `src/control/controller.rs` now passes raw finite positive sample elapsed time to `TStarSource`; `DeviceLoop` retains its own internal two-second control-time clamp. `TStarSource` resets and freezes its dwell/debounce clocks and quarantine-recovery streaks before a resumed or unmarked gap tick. The controller regression uses a controllable curve, jumps 7,200 seconds without `resumed`, and proves the first fresh sample remains Held rather than spending wall time to enter Curve. Two source regressions prove a 29-match quarantine-recovery streak cannot cross either gap form.
- Auto exit clears `NotCalibrated` together with the other flags owned by the active Auto session. The lifecycle test first proves the flag is active, exits Auto, then proves Monitor is clean.
- `src/integration_tests.rs` now proves actual CPU/GPU calibration fits land under `quiet16:60`, the qualified seed and timestamp survive reload, 45 qualified live samples persist a non-bound warm pair, and a new controller consumes that exact pair and the qualified target. An identical controller without persisted fits diverges after a PI update, causally proving the fitted gains are installed. Its separate plant resume test holds and reasserts both caps over a 7,200-second gap.
- The exhaustive Config registry checks every top-level and LED field against a non-comment production consumer. The exhaustive status registry checks a production raise expression per flag, executes the actual Decision serialization mapper, serializes the result, and executes the TUI renderer.
- Sim 8 assertions are scoped to the fault windows, proving unknown-to-Held/Regulate, known-uncontrollable-to-Bypass, GroupUnavailable-to-GroupLost, and returned-group recovery.
- Corrected live comments for GPU guard hysteresis and structured-flag clearing semantics.

## Reused acceptance evidence

The existing `.11` and `.14` scenario suites remain the authoritative acceptance matrix. The full gate exercised nominal CPU/GPU/both cases, CPU robustness, all 108 GPU tau/K/theta cells, cold/warm/disabled/missing-draw knee timing, square-wave and draw-dip recovery, restart-seed variants, both guard matrices, verifier behavior, raw reconciliation, quarantine and implausible/uncontrollable sensor legs. No threshold was weakened and no scenario was duplicated merely to increase test count.

Enum-derived registries and exhaustive matches cover all Config fields, `StatusFlag`s, `TStarState`s, `Selected` values, `Hold` values, telemetry record fields, structured v3 flags and UI rendering. The production `apply_effects` telemetry test proves live Auto values reach JSON and retired decision keys do not.

## Validation

- `cargo test integration_tests -- --nocapture` — 9 passed.
- `cargo test control::sim_tests -- --nocapture` — 3 passed.
- `cargo test every_telemetry_field_is_populated -- --nocapture` — 1 passed.
- `cargo test control::controller::tests -- --format=terse` — 119 passed.
- Elevated `cargo test --no-fail-fast -- --format=terse` — 695 passed, 0 failed, 2 ignored; 162.77 s.
- `cargo clippy --all-targets -- -D warnings` — passed.
- `git diff --check` — passed.
- Targeted `rustfmt --check` reports broad inherited formatting drift in the large pre-existing files; no repository-wide formatter rewrite was applied.

## Deletion and hardware audit

No retired module basename remains. Retired decision-key strings in current source are negative JSON assertions, and `gpu_max_w` is confined to explicit migration handling/tests. Hardware was not accessed. The two hardware checks remain ignored and parked for the user.

No blocker or open offline acceptance item was found.
