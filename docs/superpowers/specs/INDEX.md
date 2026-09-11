# Design specs index

One row per spec: date · title · link · one-line summary · status · tags. Earlier designs live in
`docs/plans/` and are not re-indexed here.

| Date | Title | Link | Summary | Status | Tags |
|---|---|---|---|---|---|
| 2026-09-07 | Closing the loop on fw-fanctrl | [2026-09-07-fw-fanctrl-loop-design.md](2026-09-07-fw-fanctrl-loop-design.md) | Replace the learned power→RPM model with one PI on fw-fanctrl's input temperature (RPM fallback), delete Kalman/thermal-model/trust/cooldown and the calibration matrix | implemented | fw-fanctrl-loop, control, root |
| 2026-09-11 | Per-device temperature loops | [2026-09-11-per-device-temperature-loops-design.md](../runs/2026-09-11-per-device-temperature-loops/2026-09-11-per-device-temperature-loops-design.md) | Replace the scalar budget + CPU/GPU split with two per-device temperature PIs on the EC sensor groups, a shadow cap with override control, a T* source replacing the Mode A/B arbiter, GPU clock driven directly (LUT deleted), per-device step test | draft | per-device-temperature-loops, control, root |
