# Coverage ledger — per-device-temperature-loops (root epic fw-fanctrl-loop-eb9)

One line per disposed finding: `id · type · disposition · one-line description`. Dispositions
are automatic (autonomous run); every applied fix names the bead(s) it changed.

## Round 1 (2026-09-11, 3 reviewers, 40 raw → 16 deduped)

- c1-01 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.7 seeds shadows but had no path to eb9.4 → edge eb9.7←eb9.4 + reason line.
- c1-02 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.13 consumes the v3 ControlStatus fields owned by eb9.9 → edge eb9.13←eb9.9.
- c1-03 · UNOWNED-SEAM · applied · decision-line emission call site: eb9.7 now owns it; edge eb9.7←eb9.9 (schema first).
- c1-04 · UNOWNED-SEAM · applied · DeviceLoop seed/resync API: eb9.3 owns seed()/resync_error() with no-kick acceptance; eb9.5/eb9.7 consume.
- c1-05 · UNOWNED-SEAM · applied · t_star_last_good load/persist wiring: eb9.7 owns it; edge eb9.5←eb9.6 for the field shape.
- c1-06 · GAP · applied · no fault injection for GroupUnavailable/Released: eb9.10 owns a fault-injection API (dGPU off, fan outage, EC invalid/stale view); eb9.14 cites it (needs: eb9.10).
- c1-07 · UNEXERCISED-CONFIGURATION · applied · Selected::Floor/Max never exercised: eb9.14's smoke now covers every Selected value (GPU-HOT-to-floor leg, unreachable at max).
- c1-08 · UNOWNED-SEAM · applied · sim_tests.rs edited by eb9.11 and eb9.12 unordered: eb9.11 owns removing the ClockWattsLut sims; eb9.12's list amended; edge eb9.12←eb9.11.
- c1-09 · UNSATISFIABLE-ACCEPTANCE (unwired) · applied · eb9.12's green gate did not wait for the acceptance sims → edges eb9.12←eb9.11, eb9.12←eb9.14.
- c1-10 · NARRATIVE-EDGE (unstated) · rejected (already applied before dispatch) · eb9.12←eb9.13 reason line was added between assembly and dispatch; the dump the reviewers saw predates it.
- c1-11 · UNOWNED-SEAM / GAP · applied · calibration freeze of both loops: eb9.7 owns freeze/unfreeze + reseed; edge eb9.8←eb9.7.
- c1-12 · UNSATISFIABLE-ACCEPTANCE (prose) · applied · eb9.3's "first-order plant" restated as a local test stub in device_loop.rs (no dependency on eb9.10).
- c1-13 · GAP · applied · per-device draw inputs plumbing: eb9.7 owns passing cpu_pkg_w / gpu_sm_mhz + gpu_util_pct into each tick.
- c1-14 · GAP · applied · duty↔RPM table refinement at warm start: eb9.7 owns it, unchanged from today.
- c1-15 · GAP · applied · WarmStart::key(strategy, duty, on_ac) form: eb9.6 owns it with a round-trip test.
- c1-16 · GAP · applied · telemetry schema docs at v3: eb9.9 owns the README/docs telemetry section.
- c1-17 · flag-sweep eb9.7 · applied (split) · the GPU HOT max ratchet rule is its own leaf eb9.15 (blocked by eb9.3, blocking eb9.7); the NOT CALIBRATED gate removal stays in eb9.7.
- c1-18 · flag-sweep eb9.5 · applied (split) · the Held driver (RPM PI), feasibility/steepness flags and DeviceUnreachable are their own leaf eb9.16 (blocked by eb9.5 and eb9.3); eb9.14's Held sim consumes it.
- c1-19 · flag-sweep eb9.8 · accepted · reason holds: one-to-one with §2.6, single file set.
- c1-20 · flag-sweep eb9.9 · accepted · reason holds: split to eb9.13 is visible; the two seams it left (c1-02, c1-03) are applied.
- c1-21 · flag-sweep eb9.11 · accepted · reason holds: split to eb9.14 is visible and contracted.
- R-new (accepted into the canonical list as R17, R18): R17 the calibration path freezes both loops at their applied caps for the run and reseeds them on exit with no cap step; R18 DeviceLoop exposes a seed/resync API the controller and TStarSource drive.
