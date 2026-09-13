# Task 16 report: Mode arbiter, reconciliation, feasibility (`fw-fanctrl-loop-iym`)

## What I implemented

`src/control/mode.rs` (new, 1088 lines including tests), and one `pub mod mode;` line in
`src/control/mod.rs`.

- `Arbiter`: a stateful, pure decision engine. Owns the §2.5 mode-table evaluation, the §2.6
  reconciliation counters (`EC MISMATCH` latch, the MA-check reseed streak), the §2.7
  feasibility latch, the argmax-controllable debounce, and the point-keyed `Curve` cache /
  all T* derivation, exactly as the brief's "what this task owns" list specifies.
- `Arbiter::decide(&mut self, input: &ArbiterInput) -> Decision` — the single entry point, one
  call per allocator tick (5 s). `ArbiterInput` carries every field the brief names: `fanctrl:
  Option<&FanctrlView>`, `freshness: Freshness`, `view_changed: bool`, `ec: Option<&EcReading>`,
  `ec_ma: Option<f64>`, `fan_valid: bool`, `target_duty: u8`, `at_lower_bound_for`/
  `at_upper_bound_for: Duration`, `error_sign: f64`, `curve_valid: bool`, plus two fields the
  brief's prose implies but doesn't name a home for — `replica_slope_5s_c_per_s` and
  `view_to_sample_gap_s`, the skip rule's two raw ingredients (see design decision #1 below).
- `Decision { mode, t_star, slope, reasons: Vec<String>, flags: Vec<StatusFlag>, reseed_ma:
  Option<f64>, ec_mismatch: bool, t_star_changed: bool }` — matches the brief's struct shape
  literally.

### §2.5 row table

Rows evaluated in order via a `core_ok = hard_ok && argmax_ok && feasible_ok` gate (freshness,
`active`, EC validity, curve validity, feasibility, the debounced argmax check), plus a
**separate** `reconciliation_ok = reconciled && !ec_mismatch` gate. `core_ok` must hold for
`ENTRY_HYSTERESIS_TICKS` (3) *consecutive* `decide` calls before TempLoop is reachable; exit is
immediate (streak resets to 0 the instant `core_ok` goes false, dropping the mode the same tick).
`RpmLoop` = `fan_valid`; `Released` otherwise. See design decision #2 below for why
reconciliation is deliberately *not* folded into the same streak as `core_ok`.

### §2.6 reconciliation

Scored only when `input.view_changed` **and** the skip rule passes (replica slope < 0.5 °C/s,
gap < 2 s). Three consecutive scored mismatches (`|max_c - temperature| > 1`) latch
`EC MISMATCH`; three consecutive matches clear it and set `reseed_ma = Some(ma_temperature)` on
that tick. A separate streak drives the MA check (`|ec_ma - ma_temperature| > 2`): three
consecutive failures request a reseed (same `reseed_ma` output) **without** touching the
mismatch latch. `reconciled` is a one-way latch, set true on the first scored view (match or
mismatch) — `unreconciled` and `ec_mismatch` are distinct reasons, per acceptance.

### §2.7 feasibility / steepness / low / high

- Feasibility (`T* >= max(uncontrollable readings) + 5`) is the only one of the three
  "unreachable" rules that needs its own sticky state (`infeasible_latched` +
  `feasible_streak`, cleared after `FEASIBLE_CLEAR_TICKS` = 12 consecutive feasible ticks =
  60 s). `low`/`high` don't need an equivalent: they read `Budget`'s own
  `at_lower_bound_for`/`at_upper_bound_for` `Duration`s directly, which already reset themselves
  the instant the bound is left — so those two rules are purely live, stateless checks.
- `low` fires on either `curve.nearest_tread(target_duty).is_none()` (sub-floor duty — reusing
  Task 1's own `nearest_tread`, whose documented `None` case *is* the sub-floor condition) or
  `at_lower_bound_for >= 60s && error_sign < 0`.
- `high` fires on `at_upper_bound_for >= 60s && error_sign > 0`.
- `SteepCurve` (`slope_at(T*) > 2`) is evaluated whenever a T* resolves, independent of mode —
  matches "informational only; the loop runs".
- `curve_valid: false` short-circuits the whole curve/T*/feasibility/steepness path: `t_star` and
  `slope` are forced `None`, `CurveInvalid` is the only flag raised, `SteepCurve` is
  unreachable — verified by its own test.

### T* derivation / point-keyed cache

Cache keyed on `Vec<(f64, u8)>` (the raw points), separate from `target_duty`. Re-derives
(`Curve::from_points` + `t_star_changed = true`) whenever either differs from the cache, or when
`curve_valid` just flipped from true to false (the T*→None transition is also a real change the
controller should `resync_error` on).

## Design decisions worth a second look

1. **Two fields not named in the brief's `ArbiterInput` list:** `replica_slope_5s_c_per_s: f64`
   and `view_to_sample_gap_s: f64`. §2.6's skip rule needs both, and the brief's own bead text
   says this task "own[s]... the skip rule" — but the field list only says "and the
   reconciliation counters the controller scores at 1 Hz" without naming these two explicitly. I
   read that phrase as referring to these two raw numbers (computed by the controller, task 19,
   from its own sample history) rather than pre-tallied match/mismatch counts, since the task
   also explicitly owns "EC MISMATCH counters" as *internal* state (confirmed by "What this task
   owns": "the mismatch and feasibility counters" are Arbiter's own fields, not something handed
   in already-tallied). If task 19 disagrees with this shape, it's a one-line field rename, not
   a redesign — the counting logic itself doesn't move.

2. **Entry hysteresis explicitly excludes `reconciliation_ok` from the streak.** A first reading
   of §2.5's "3 consecutive ticks (15 s)" bullet suggests one monolithic streak over every row-1
   condition. But acceptance criterion 3 ("three matches -> TempLoop with `reseed_ma`") requires
   the mode to land on TempLoop *the same tick* the third match clears `EC MISMATCH` — a
   monolithic streak would need 3 more ticks after that (6 total), contradicting the criterion
   as stated. Splitting the gate — `core_ok` streak (unaffected by reconciliation, so it keeps
   climbing straight through a mismatch episode) vs. `reconciliation_ok` (checked fresh, no
   streak of its own) — satisfies both the generic "15 s for a clean cold engagement" case and
   the "three matches, same tick" case at once. This is documented at length in the module's own
   doc comment (the "Entry hysteresis is layered, not monolithic" section) since it's the least
   obvious call in the file. Worth a second pair of eyes.

3. **No new `StatusFlag` variants.** `ec.is_none()` (EC replica invalid) and
   `argmax_uncontrollable` have no dedicated flag in the design's enumerated list (only
   `FanctrlLost` for socket staleness and `SensorLost` for the *fan* reading are named for the
   `Released`/`RpmLoop` fallback) — I surface both as `reasons` text only, no flag, rather than
   inventing a variant in `controller.rs` (which is outside this task's `filesTouched` and the
   Global Constraints' "don't widen scope" rule).

4. **`SensorLost` is only raised in `Released`, matching the design bullet's literal wording**
   ("Released... flag... SENSOR LOST when the fan reading is invalid"), even though the row
   table doesn't list `fan_valid` as a TempLoop precondition (so `TempLoop` with `!fan_valid` is
   representable but untested — not exercised by any acceptance criterion, and the design gives
   no guidance for that combination).

## Tests

Table-driven per §2.5 row, plus one test per remaining acceptance-criteria bullet — 14 tests
total, all in `src/control/mode.rs`'s own `#[cfg(test)] mod tests`, no fixtures directory
dependency (matches Task 1's precedent of using the §Facts point lists inline). `EcReading`/
`EcLabel` have no public constructor outside `sensors::ec` (by design, `EcLabel::new` is
module-private) — `ec_reading()` builds one through the module's own public `read()` entry point
against a synthetic temp hwmon-shaped fixture dir, mirroring `ec.rs`'s own test helpers exactly
(cannot import theirs directly: they're private to that module's `#[cfg(test)]`).

```
cargo test --bin fw-fan-quiet control::mode::
-> test result: ok. 14 passed; 0 failed; 0 ignored

cargo test --bin fw-fan-quiet
-> test result: ok. 567 passed; 0 failed; 2 ignored
```

### Assertion discipline note

Every assertion in the 14 tests names a concrete alternate value the code path could produce
that would fail it (e.g. `d1.ec_mismatch` false-vs-true after exactly 1/2/3 scored mismatches;
`t_star_changed` true-vs-false on a repeat call with identical points; the `>1 °C` argmax lead
boundary tested both just-under and clearly-over). None are decoration — I checked each one
against "what value would this code actually produce if the logic under test were wrong" before
keeping it. One thing to flag: `table_driven_row_selection`'s `warmed_up()` helper itself
contains an assertion (`d.reasons.contains(&"unreconciled"...) || d.mode != TempLoop`) that is
a precondition sanity check, not a criterion under test — it exists to catch a broken fixture
early with a clear message rather than a confusing downstream failure, and I did not count it as
covering any acceptance bullet.

## Files changed

- `src/control/mode.rs` (new)
- `src/control/mod.rs` (`pub mod mode;`, one line, per the Hot Files rule)

## Quality gate

- `cargo test --bin fw-fan-quiet`: **567 passed, 0 failed, 2 ignored** (the 2 ignored are
  pre-existing NVML hardware smoke tests, unrelated to this task).
- `cargo clippy --all-targets -- -D warnings`: **not clean — dead_code only, precedented.**
  Confirmed via `git stash` on the base commit: baseline is 63 dead_code errors (structural to
  this mid-epic binary crate — many already-landed modules, e.g. `EcReading`, `FanctrlView`,
  `Curve`, `Budget`, have no production call site yet, same pattern Tasks 1/3/7/10 each
  documented). My change adds exactly 16 more, all `dead_code`, all in `mode.rs`
  (`ArbiterInput`, `Decision`, `Arbiter`, its `new`/`decide`, and the 12 module constants) — I
  grepped the full clippy output for any non-`dead_code` error line and found none. This is the
  direct, structural consequence of the deliverable as specified ("tested standalone... no
  controller wiring in this task"): nothing outside `#[cfg(test)]` calls `Arbiter::decide` yet,
  by design, until Task 19 wires it in. I did **not** add `#[allow(dead_code)]` (explicitly
  forbidden by the Global Constraints) and did **not** wire `mode.rs` into `controller.rs`
  myself (explicitly out of this task's scope — `filesTouched` is `mode.rs` + one line of
  `mod.rs`).
- `cargo fmt --check`: clean for `mode.rs` and `mod.rs` (the two files this task touches).
  `budget.rs` has 2 pre-existing formatting diffs from before this task started — confirmed via
  `git diff` that my working tree never touched that file; left alone as out of scope.

## Concerns

1. The clippy dead-code gate, expected/precedented per the note above — flagging per the Global
   Constraints' literal "clean for every task" wording, though I believe this is the same
   structural, non-negotiable consequence Tasks 1/3/7/10 each hit and reported the same way.
2. Design decisions #1 and #2 above (the two `ArbiterInput` fields the brief's prose didn't
   explicitly enumerate, and the layered-not-monolithic entry hysteresis) are the two places I'd
   most want a second pair of eyes against the original design author's intent — neither, I
   believe, changes correctness against the stated acceptance criteria, but both involved
   real judgment calls the brief left implicit.

Status: **DONE_WITH_CONCERNS** (clippy dead-code gate, expected/precedented; two design
judgment calls worth double-checking — neither blocks the task or, I believe, is incorrect).
