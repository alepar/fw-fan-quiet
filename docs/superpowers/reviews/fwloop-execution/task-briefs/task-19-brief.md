## Task 19: Controller loop integration

**Bead:** `fw-fanctrl-loop-j6s`

**filesTouched:** `src/control/controller.rs`

### EPIC-SPECIFIC CONSTRAINT — the anti-windup rule is NOT yours to invent

> bead `fw-fanctrl-loop-9it` ("Spike: settle the anti-windup rule") is a SPIKE whose deliverable
> is a DECISION written into §2.4 of
> docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md, reached by measurement on its own
> throwaway harness. §2.4 deliberately fixes only the INVARIANTS — anti-windup is directional
> and may halt only the deepening direction, never pull u toward the measured draw; it judges
> per axis; any hold is visible — and deliberately leaves the predicate, the margins, the
> hysteresis and the GPU-HOT interaction UNSPECIFIED. Two earlier prose attempts (a
> back-calculation toward measured draw, which is a tracker rather than anti-windup; and a
> direction-blind freeze, which self-latches) were each independently confirmed Blocking.
> Therefore: (a) task `fw-fanctrl-loop-9it`'s plan section must state that the spike DECIDES
> those unspecified items by measurement and REWRITES §2.4 with the result, and must not present
> §2.4's current text as an implementable rule; (b) the plan section for `fw-fanctrl-loop-j6s`
> ("Controller loop integration"), which is blocked by 9it, must state explicitly that its
> implementer reads the REWRITTEN §2.4 from the integration branch and MUST NOT invent, infer,
> or reconstruct the anti-windup predicate, margins, hysteresis, or GPU-HOT interaction from
> prose — if §2.4 still reads as open when that task runs, that is a BLOCKED condition, not a
> licence to improvise.

**Concretely, before you write the anti-windup wiring:**

1. Open `docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` §2.4 **on the integration
   branch** (Task 11 rewrote it there).
2. Confirm it states, normatively: the predicate; `DEMAND_MARGIN_W` per axis; the
   hysteresis/dwell/debounce; whether leaving calls `resync_error`; whether the per-axis
   comparison uses the **pre-** or **post-**guard-override cap; and what a `GPU HOT` episode
   does.
3. If **any** of those still reads as open, undecided, or "for the spike to decide": **STOP and
   report BLOCKED.** Name which item is open. Do **not** invent, infer, or reconstruct it from
   the surrounding prose, from the invariants, or from the placeholder Task 3 left in
   `Budget::set_demand_state`. That placeholder is a seam, not a rule.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.4** (as rewritten), **§2.5**, **§2.6**,
**§2.9**, **§3.2**. This task builds on the tier-free `controller.rs` from Task 12.

### The data flow this task owns

`Budget::new(persisted loop_gains or default)` on auto entry, then `on_auto_sample`:

1. **Window pushes** — the existing fan window; `rpm_smoothed` = `FAN_SMOOTH_N` tail-mean;
   `fan_valid` from the sample.
2. **`EcAverage` push** — the controller owns the live instance: `set_interval` on view change,
   `reseed` on `Decision.reseed_ma`.
3. **Guards** (`Option` inputs) — `nvme_hot` raises its flag **only**.
4. **Every 5 s:**
   - budget bounds: `lo = cpu_floor_w + lut.watts_at(gpu_floor_mhz)`,
     `hi = cpu_max_w + gpu_max_w`, then `Budget::set_bounds`
   - `target_duty` from the user's target via `DutyRpmTable::duty_for_rpm` +
     `Curve::nearest_tread`
   - arbiter (`ArbiterInput` incl. `view_changed`, `at_lower_bound_for`, `at_upper_bound_for`,
     `error_sign`)
   - `resync_error` when `t_star_changed`
   - `LoopError` — Mode B's error is against `rpm_for_duty(target_duty)`, with the gain scaled by
     `scale_rpm_gain(slope)`
   - **the anti-windup halt exactly as the rewritten §2.4 states it** —
     `Budget::set_demand_state` with the per-axis smoothed draws, their caps and the error sign
   - `split_budget` (+ guard overrides + slew clamp)
   - CPU write + read-back verdict
   - GPU PI target at 1 Hz + `verify_lock` verdict

**The shared verdict rule for both actuators:** a candidate `Mismatch` is **re-read once** and
**suppressed for 3 ticks after an `on_ac` edge**; then `Mismatch` gives `LimitNotSticking` +
`Freeze::ActuatorMismatch` + an **immediate reassert on the same tick**; **three consecutive**
gives release to stock **with the flag held while the write + read-back keeps running every
reassert period, so a later `Verified` is producible** and clears the strikes.
`Unreadable` / `Unverifiable` are non-events; **six consecutive `Unreadable`** raises
`ReadbackBlind` until the next `Verified`.

`EcAverage` is seeded from `view.ma_temperature` on auto entry, on re-engagement, and at
calibration exit; it plus the fan and steady windows are **cleared on a `resumed` sample**.

A `Released` decision gives `release_to_stock` + `Freeze::Released` + flags; a later usable
decision re-engages — **seeding from the floors here**; the warm-start seed arrives in Task 20.

Auto entry requires `lut` **only**. Transitions emit `Noted { mode: ... }`. Status fields are
mirrored every tick. **The RAPL stickiness watchdog in `on_sample` is retained untouched.**

**Reconciliation scoring is at 1 Hz** — on the sample carrying the view, **not** on the 5 s
arbiter tick. The arbiter consumes the counters; this task owns when they are scored.

### Acceptance criteria (verbatim from the bead)

> controller unit tests on the fake seams cover: auto entry with LUT only; `Some(gains)` is
> loaded into `Budget`, `None` uses defaults; the integrator floor tracks a LUT change; an
> `NVME HOT` tick raises the flag and leaves `target_duty`, T* and the budget **unchanged**; a
> tick whose snapped duty the curve skips uses `nearest_tread`; a TempLoop tick computes
> `T* - MA` and moves the budget; an RpmLoop tick on socket Absent drives u from
> `rpm_for_duty(target_duty) - rpm_smoothed`; a same-name curve edit calls `resync_error` and
> produces no kick; a fan dropout clears `fan_valid` within one window; **the anti-windup
> scenarios `fwloop.24` decided, replayed at controller level against the rule it chose** — at
> minimum: an idle tick far below the cap neither winds to the ceiling nor decays toward the
> draw; a seeded-high budget with a lighter load still integrates **down**; a CPU-only tick with
> the dGPU unpowered keeps `u` off its lower bound; and a `GPU HOT` episode behaves as the spike
> specified; a scored view is evaluated on the 1 Hz sample carrying it, not on the 5 s arbiter
> tick; a rejected curve raises `CurveInvalid` and RpmLoop runs at the `0.25x` clamp; a CPU and a
> GPU `Mismatch` each freeze, flag and reassert on the same tick, and a mismatch within 3 ticks
> of an `on_ac` edge is suppressed; three consecutive mismatches release to stock with the flag
> held, the read-back keeps running, and a later `Verified` re-engages without a step; six
> `Unreadable` raise `ReadbackBlind` with no freeze; a `resumed` sample clears the boxcar and the
> steady window; a `Released` decision releases the caps, freezes, and a later usable decision
> re-engages without a step in u; a view change updates the boxcar interval without discontinuity
> in `ec_ma_c` and a `reseed_ma` decision re-seeds it; `Noted` transitions appear; the RAPL
> watchdog test still passes.

### Implementation steps (TDD)

1. **First, before any code:** perform the §2.4 check described in the EPIC-SPECIFIC CONSTRAINT
   above. Quote the decided rule and its constants into your report. If it is still open, stop
   and report BLOCKED.
2. **Test first:** auto entry with `lut` only succeeds; `Some(gains)` loads into `Budget` and
   `None` uses `LoopGains::default()`. Then implement entry.
3. **Test first:** the integrator floor tracks a LUT change (`lo = cpu_floor_w +
   lut.watts_at(gpu_floor_mhz)`). Then implement the bounds derivation.
4. **Test first:** `target_duty` snapping via `duty_for_rpm`, and a tick whose snapped duty the
   curve skips falls back to `nearest_tread`. Then implement.
5. **Test first:** a TempLoop tick computes `T* - MA` and moves the budget; an RpmLoop tick with
   the socket `Absent` drives `u` from `rpm_for_duty(target_duty) - rpm_smoothed`. Then implement
   the `LoopError` construction and the `scale_rpm_gain(slope)` application.
6. **Test first:** a same-name curve edit sets `t_star_changed`, calls `resync_error`, and
   produces no kick. Then wire it.
7. **Test first:** an `NVME HOT` tick raises the flag and leaves `target_duty`, T\* **and the
   budget unchanged**. Then wire the guards.
8. **Test first:** a fan dropout clears `fan_valid` within one window. Then wire the windows and
   `rpm_smoothed`.
9. **Test first — the anti-windup replay, against the decided rule only:** an idle tick far below
   the cap neither winds to the ceiling nor decays toward the draw; a seeded-high budget with a
   lighter load still integrates **down**; a CPU-only tick with the dGPU unpowered keeps `u` off
   its lower bound; a `GPU HOT` episode behaves **as §2.4 now specifies**. Then wire
   `set_demand_state` with the per-axis smoothed draws, their caps (pre- or post-guard-override
   **as §2.4 states**) and the error sign.
10. **Test first:** a scored view is evaluated on the 1 Hz sample carrying it, **not** on the 5 s
    arbiter tick. Then implement the §2.6 scoring cadence.
11. **Test first:** a rejected curve raises `CurveInvalid` and RpmLoop runs at the `0.25x` clamp.
12. **Test first, the verdict rule:** a CPU `Mismatch` and a GPU `Mismatch` each freeze, flag and
    reassert on the same tick; a mismatch within 3 ticks of an `on_ac` edge is suppressed; three
    consecutive release to stock **with the read-back still running**, and a later `Verified`
    re-engages **without a step**; six `Unreadable` raise `ReadbackBlind` **with no freeze**.
    Then implement the shared rule once, used by both actuators.
13. **Test first:** a `resumed` sample clears the boxcar and the steady window; a view change
    updates the boxcar interval **without a discontinuity in `ec_ma_c`**; a `reseed_ma` decision
    re-seeds it. Then wire the live `EcAverage`.
14. **Test first:** a `Released` decision releases the caps and freezes; a later usable decision
    re-engages **without a step in `u`** (seeded from the floors here). Then implement.
15. **Test first:** `Noted { mode }` transitions appear on each mode change; status fields are
    mirrored every tick.
16. Confirm the RAPL stickiness watchdog and its tests are untouched and still pass.
17. Run the test suite and the linter; both clean.

### Deliverable

A fully wired auto loop on the fake seams, whose anti-windup behaviour is exactly the measured
rule §2.4 states — quoted in your report — and never a reconstruction of it.

---

