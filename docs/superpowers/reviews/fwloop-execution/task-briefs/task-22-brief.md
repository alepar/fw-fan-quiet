## Task 22: Closed-loop acceptance + configuration smoke

**Bead:** `fw-fanctrl-loop-cm7`

**filesTouched:** `src/control/sim_tests.rs`, `src/control/mod.rs`

`src/control/mod.rs` — add exactly `#[cfg(test)] mod sim_tests;`.

### Global constraints

All of "Global Constraints" above applies. Normative: **§5** (the testing section, including the
period-agnostic relay rule). Every run is controller-level, on `ChainedPlant` (Task 17), through
the **real** `on_sample`, with Task 20's hooks active. The load step is a **utilisation + watts**
step, not a watts-only one. Seeded RNG throughout — every run must be deterministic.

### The two grading rules that are easy to get wrong

- **Relay detection is period-agnostic (§5):** no 3 or more consecutive sign-alternating band
  excursions **at any period**. Report the count and the dominant period for each run. Do not
  substitute a fixed-period oscillation check.
- **The calibration run's fit is graded against the boxcar-filtered plant it actually sees, not
  the raw tau 35 / theta 20 constants.** The criterion is that the derived `Kc` lands within
  25 % of the IMC value **for that filtered plant** and that the closed loop passes — **never**
  that tau/theta match the raw plant.

### The run list

**Baseline (4 runs):** `quiet16` and `cool16` x `TempLoop` and `RpmLoop` — load step then
30 min; at least 90 % of samples inside +/-150 RPM; no relay under the rule above.

**Robustness:** the same 4 runs with plant K/tau/theta perturbed +/-50 %.

**Refinement:** plant table biased -8 %; refinement brings RPM inside +/-150 within 20 min and
T\* follows the re-snapped duty.

**Demand-starved:** a long idle with the plant drawing far below the cap, then a load onset —
`u` never reaches the upper bound and the onset overshoot stays inside +/-150.

**Calibration:** a StepTest on the plant, then the `quiet16`/`TempLoop` acceptance repeated with
the fitted gains meeting the same bar, graded per the filtered-plant rule above.

**Transients:** a load release at t=1200 back inside +/-150 within 90 s; a dGPU-powered-and-hot
30 min run that stays in `TempLoop` with no `EC MISMATCH`; a dGPU-unpowered run (no `GPU HOT`,
floor honoured, `verify_lock` `Unverifiable`); a sub-floor target run raising
`TARGET UNREACHABLE (low)` and holding the floor without relay.

**Bumpless:** socket death at t=600 (A to B); `active: false` at t=700 with a fresh socket
(A to B); an in-place same-name curve edit at t=900. Each leaves |delta u| at most one increment
and the caps continuous.

**`active: false` authority run** (plant in EC-autofan mode, §Facts staircase): with a target
below the EC's flat band the loop parks `u` at the floor, raises `TARGET UNREACHABLE (low)`
naming the achievable RPM within 60 s, and the integrator does not wind; with the EC on its
steep segment below 64 °C the loop does not hunt. **The same run repeated with the socket
`Absent`** (the killed-daemon regime, §2.5) behaves identically and additionally raises
`FANCTRL LOST`.

**Demand-limited:** a duty-cycled load (5 min on, 2 min off, x3) must not let `u` decay toward
the lull draw and must return inside +/-150 RPM within 90 s of each onset; a CPU-only run with
the dGPU unpowered must leave `u` off its lower bound and `cpu_w` above `cpu_floor_w` after
10 min.

**Rejected curve:** a non-monotone curve keeps the loop in `RpmLoop` at the `0.25x` gain clamp
with `CURVE INVALID` raised, and **never** `SteepCurve`.

**`Released`:** socket absent **and** an invalid fan reading gives stock caps within one
hysteresis window, `FANCTRL LOST` + `SENSOR LOST` set; then sensor recovery re-engages `RpmLoop`
from the warm-start **without a cap step**.

**Faults:** feasibility (T\* below ambient + 5); the `high` unreachable case; the steep-curve
flag; a single read-back mismatch freeze; a mismatch suppressed across an `on_ac` edge; a
three-strike release followed by a `Verified` re-engagement; a 5 min `GPU HOT` episode at the
90 °C threshold with no post-episode overshoot above 150 RPM; an `NVME HOT` episode that raises
the flag **while the RPM trace is indistinguishable from the same run without it**;
reconciliation A to B to A with reseed; a scored view skipped because the replica was slewing;
and a `resumed` edge mid-run that clears the windows and writes **no** warm-start or refinement
across the gap.

**Global assertions over every run:** only `Speed`/`All` commands were ever recorded by the
fake; `ec_ma_c` tracks the emulator's `ma_temperature` within 1 °C in steady state; at least one
steady window is detected per converged run.

### Acceptance criteria (verbatim from the bead)

> all listed runs pass deterministically (seeded RNG); each spec-enumerated configuration (2
> strategies x 3 modes, dGPU on/off, default vs fitted gains) is exercised end to end (needs:
> fwloop.12, needs: fwloop.19).

### Implementation steps

1. Build the harness first: a run descriptor (strategy, mode, plant parameters, script,
   duration, seed) and a grader that returns the band-residency percentage, the relay count and
   the dominant period. Write the **grader's own unit tests** before any acceptance run — a
   broken relay detector silently passes everything.
2. Implement the **period-agnostic** relay rule and test it on synthetic traces: a clean
   converged trace (0 relays), a 3-alternation trace at one period, and a 3-alternation trace at
   a very different period (both must be caught).
3. Add the four baseline runs, then the +/-50 % perturbation set.
4. Add the refinement, demand-starved and calibration runs. For the calibration run, compute the
   IMC value **for the boxcar-filtered plant** and assert the derived `Kc` within 25 % of it —
   write that computation explicitly so nobody later "fixes" it to the raw constants.
5. Add the transient, bumpless, `active: false` authority (both socket-alive and `Absent`),
   demand-limited, rejected-curve and `Released` runs.
6. Add the fault matrix.
7. Add the three global assertions as a shared post-run check applied to **every** run.
8. Confirm the configuration coverage: 2 strategies x 3 modes, dGPU on/off, default vs fitted
   gains — write it as an explicit checklist test so a missing combination fails.
9. Run the test suite and the linter; both clean. Report each run's band residency, relay count
   and dominant period.

### Deliverable

A deterministic acceptance suite that grades the finished loop against §5, with the relay
detector itself unit-tested.

---

