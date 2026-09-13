# Task 1 report: Curve model + DutyRpmTable (fw-fanctrl-loop-9dv)

## What I implemented

`src/fanctrl/mod.rs` (new, `pub mod curve; pub mod table;`), `src/fanctrl/curve.rs` (new),
`src/fanctrl/table.rs` (new), and `src/main.rs` (`mod fanctrl;`, one line, alphabetically placed
between `mod event;` and `mod led;` — nothing else touched).

Full API surface per the brief:

- `Curve::from_points(Vec<(f64, u8)>) -> Result<Curve, CurveError>`, `duty_at`, `tread`,
  `t_star`, `slope_at`, `nearest_tread`, `min_tread_duty`. `CurveError { Empty,
  DescendingSegment { at_index } }` — a plain domain error (`Display` + `std::error::Error`, no
  flag, no panic), matching the existing `FitError` pattern in `control/thermal_model.rs`.
- `DutyRpmTable` (serde `Serialize`/`Deserialize`, `Default` = the ten seeded points),
  `rpm_for_duty`, `duty_for_rpm`, `refine`.

## Design decisions the brief left open (and why)

The brief and design doc §2.1/§2.3 specify behaviour precisely everywhere I could find ground
truth, but three points needed a judgment call. Flagging all three explicitly since they're the
kind of thing a reviewer or a later task might read differently:

1. **`tread()`'s bounds at the curve's own floor/ceiling duty are `NEG_INFINITY`/`INFINITY`, not
   an arbitrary finite cutoff.** §2.1 defines `tread(d)` as "the maximal temperature interval
   where `duty_at(t) == d`". The flat clamp below the first point / above the last is genuinely
   unbounded — fw-fanctrl would report that duty at any arbitrarily low/high temperature — so
   returning anything finite there would not be "maximal". I implemented it literally: `tread`
   on the curve's lowest duty returns `(NEG_INFINITY, finite)`, on the highest `(finite,
   INFINITY)`. Consequence: `t_star` on those two duties is itself infinite. I considered
   bounding both ends to the curve's own point range instead (which would keep `t_star` always
   finite and directly usable as a PI setpoint), but rejected it: nothing in §2.1 hints at that
   restriction, and this task explicitly does not own the loop that consumes `t_star` — "this
   task only owns the curve model, not the loop that consumes it" is in the module doc comment.
   Whichever downstream task computes a setpoint from `t_star` (the arbiter/budget wiring) will
   need to special-case an infinite result for a target that snaps to the curve's floor or
   ceiling duty. I'm fairly confident this is the literal reading of the spec, but flag it as the
   single highest-value thing for the reviewer to double check against the design doc author's
   intent.
2. **`slope_at` at a temperature shared by two segments (an interior curve breakpoint) resolves
   to the segment *starting* there** (the later of the two), not the one ending there. §2.1 only
   specifies the clamp value (`0` on the flat clamps); it doesn't say which side wins at an
   interior tie. Documented in the function's doc comment and exercised directly in
   `slope_at_inside_and_at_segment_boundaries` (asserts `slope_at(75.0)` on quiet16 returns the
   75→82 segment's slope, not 65→75's).
3. **`nearest_tread`'s "skipped integer" case cannot be reproduced from `quiet16`/`cool16`.** I
   verified this by construction, not just by testing: `duty_at` is `trunc(continuous_value(t))`
   where `continuous_value` is continuous and non-decreasing (guaranteed by `from_points`
   rejecting descending segments) over a curve whose points all have distinct temperatures. By
   the intermediate value theorem such a function attains *every* integer between its floor and
   ceiling duty at some real `t` — so neither reference curve ever actually skips an integer; I
   confirmed this by hand-computing several interior treads (e.g. quiet16's duty 25, between the
   65→75 segment's 21 and 31) and finding all nonzero-width. The only way to construct a genuine
   skip is a curve with two points at the *same* temperature and different duty (a legal,
   non-descending, but degenerate vertical jump) — `from_points` doesn't reject this, since
   nothing in the brief asks it to. `nearest_tread_skipped_integer_resolves_to_nearest_lower`
   therefore builds its own tiny synthetic curve (`(0,10)(50,10)(50,20)(100,100)`, skipping
   11..=19) rather than reusing the two reference curves, and the doc comment on the test
   explains why. `continuous_duty_at`/`first_t_reaching` both handle the same-temperature case
   explicitly (resolve to the arriving/higher duty) so this doesn't panic or divide by zero.

## A bug the property test caught (and the fix)

Step 9's "100 noisy refinements stay strictly increasing" property test genuinely earned its
keep. My first `DutyRpmTable::refine` clamped the blended value against each neighbour
independently — `blended.max(lower + MARGIN)` then `blended.min(upper - MARGIN)`, `MARGIN` a
fixed `1e-6`. That's correct as long as the two neighbours are farther apart than `2*MARGIN`, but
repeated refinements at the flat-clamped extremes (every duty above the table's highest seeded
key, 85, shares the *same* interpolated prior when newly created) squeeze existing entries to
within a few `MARGIN` of each other; a later refinement landing between two such entries could
then get an upper-clamped and a lower-clamped bound that round to the *same* f64, silently
producing a tie instead of a strict inequality.

RED: with the original two-independent-clamps implementation, `stays_strictly_increasing_after_
100_noisy_refinements` (seed `0xC0FF_EE01`, the Global-Constraints-mandated hand-rolled
xorshift32, 100 iterations, `duty` sampled `0..=255` at the time) failed on iteration 4:
```
thread '...' panicked at src/fanctrl/table.rs:315:13:
table not strictly increasing after refining duty 150 toward 6376.925626145483:
{... 94: 5920.000001, 150: 5920.000001, 245: 5920.000002000001 ...}
```
(`94` and `150` landed on the exact same `f64`.) I traced it by adding a temporary `eprintln!`
per iteration and re-running with `--nocapture` — the root cause was legible directly from the
dump: `150`'s two neighbours (`94` and `245`) were only `~1.6e-6` apart, less than
`2*MARGIN_RPM`.

GREEN, fix: scale the margin to a quarter of the *actual* gap between neighbours when both exist
(`margin = ((hi - lo) / 4.0).min(MONOTONE_MARGIN_RPM)`), then `blended.clamp(lo + margin, hi -
margin)` in one call instead of two independent ones. `hi > lo` is the invariant this method
itself maintains, so `margin <= gap/4` guarantees `lo + margin < hi - margin` algebraically
regardless of how narrow the gap has become, only falling back to the fixed `1e-6` constant when
there's room to spare. Re-ran the same test: green. I also stress-tested well past the
brief's own bar (20,000 iterations instead of 100, several different seeds) with `duty` sampled
over its realistic domain — the same range I ended up committing (see below) — with no failures.
At the *original* 0..=255 domain and 5000 iterations the same numerical-precision wall
reappears (all 256 possible key slots end up needing distinct floats within a ~4700 RPM span);
I judged that out of scope rather than fixing further, see "Domain of `duty`" below.

## What I tested (framed as RED before GREEN per step, `cargo test --bin fw-fan-quiet fanctrl`)

I wrote the full test file per module (not literally one test-then-implement cycle per function
in separate commits — the curve inversion math needed to be worked out as a whole to get
`tread`'s bounds right), then verified genuine RED/GREEN coverage by mutation after the fact,
one deliberate bug at a time, confirming each specific assertion actually catches the failure it
claims to:

- `duty_at`: changed truncation to `.round()` → `cool16_truncation_case_from_the_facts` failed
  (`left: 22, right: 21`) as expected; reverted, green.
- `from_points`: disabled the descending-segment check → both
  `descending_duty_segment_is_rejected` and `descending_segment_error_is_a_plain_domain_error`
  failed (`unwrap_err()` on `Ok`); reverted, green.
- `nearest_tread`: swapped the tie-break order to check the upper duty before the lower →
  `nearest_tread_skipped_integer_resolves_to_nearest_lower` failed (`left: Some(20), right:
  Some(10)`); reverted, green.
- `DutyRpmTable::refine`'s clamp: this is the bug above — the property test caught a real defect
  on the first real run, not a seeded mutation. I count that as stronger evidence than a mutation
  pass, since it wasn't manufactured.

Per **assertion discipline**: every numeric assertion in both files states a value the
implementation could plausibly have produced wrong (a truncated vs. rounded duty, an off-by-one
segment boundary, a wrong tie-break direction, an unclamped monotonicity violation) — none of
them are checking a value the type or a fixture already guarantees. I did not find any assertion
in either file that's decoration once I re-read the file looking for that specifically.

Final run, both modules:
```
cargo test --bin fw-fan-quiet fanctrl
running 30 tests (17 curve + 13 table)
test result: ok. 30 passed; 0 failed; 0 ignored; 0 measured; 428 filtered out
```
Full suite (confirms nothing outside `fanctrl/` regressed):
```
cargo test --bin fw-fan-quiet
test result: ok. 456 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out
```
(426 pre-existing + 30 new = 456; the 2 ignored are pre-existing and untouched by this task.)

`cargo fmt --check`: clean (ran `cargo fmt` once on both new files, both round-trip clean now).

## Quality gate: `cargo clippy --all-targets -- -D warnings` — not clean, and I believe correctly so

9 `dead_code` errors, one per new public item (`CurveError`, `Curve` and all 12 of its methods
grouped as "multiple associated items", the 4 `table.rs` module consts, `DutyRpmTable`, and its 3
methods grouped similarly). I grepped the full clippy output for any other lint category and
found none — every single error is `dead_code`. This reproduces identically under plain `cargo
build`/`cargo clippy` (no `--all-targets` needed): confirmed by running `git stash` and clippy on
the unmodified branch first (clean), then re-running on my changes alone.

This is a direct, structural consequence of the deliverable as written: "compile and are
unit-tested standalone. **No other module imports them yet.**" Nothing outside `#[cfg(test)]`
calls into either module, so rustc's `dead_code` lint fires on the whole public surface in the
plain `bin` build. I did **not** add `#[allow(dead_code)]` (explicitly forbidden by the Global
Constraints) and did **not** wire `fanctrl` into any consumer myself (out of this task's scope —
the brief names Task 7 as the socket client that will own `FanctrlView`, and Task 16 as the
arbiter that consumes `curve_valid`/`CURVE INVALID`).

I found this is an established, already-accepted pattern within this exact epic run: Task 3's
report (`task-3-report.md`, `src/control/budget.rs`, landed on this same integration branch
before I started) hit the identical situation for the identical reason and reached the identical
conclusion — 12 `dead_code` errors, no `#[allow]` added, no premature wiring, flagged as a
Concern. I'm confident this is the expected transient state a later wiring task resolves as a
side effect, not a defect in this task's code, but flagging it per the brief's "for every task"
wording, same as Task 3 did.

## Files changed

- `src/fanctrl/mod.rs` (new, 9 lines)
- `src/fanctrl/curve.rs` (new, ~500 lines incl. tests and doc comments)
- `src/fanctrl/table.rs` (new, ~365 lines incl. tests and doc comments)
- `src/main.rs` (+1 line: `mod fanctrl;`)

## Self-review findings

- Re-read both files end to end after the mutation pass above; no leftover debug output, no
  stray `#[allow]`, no TODOs.
- `DutyRpmTable::is_strictly_increasing` is `#[cfg(test)]`-gated (a private test helper, not part
  of the owned API) — its doc comment says "left `pub(crate)` for a future consumer" but the
  actual visibility is private-to-module; I noticed this drift while writing this report and it's
  harmless (nothing outside the test module calls it, and making it `pub(crate)` for real would
  just add another dead-code surface before Task 16 or the controller wants it) but the comment
  should say "private" not "`pub(crate)`" — small enough that I fixed it directly rather than
  leaving it as a concern: see the corrected doc comment in the committed file.
- **Domain of `duty: u8` in `DutyRpmTable`.** The type is `u8` (0-255) but the physical quantity
  is a percentage (0-100). Nothing in `refine`/`rpm_for_duty`/`duty_for_rpm` rejects an
  out-of-range value — they all degrade gracefully (flat-clamp or the same blend/clamp math) —
  but the *property test*'s original random-duty sampling covered the full `u8` range, which is
  what surfaced the numerical-precision wall noted above at 5000+ iterations (not the 100 the
  brief asks for). I narrowed the property test to sample `duty` over `0..=100`, its real
  operating domain, with a comment explaining why. I did not add explicit range validation to the
  public methods themselves — nothing in the brief asks for it, and doing so would be
  scope-widening a "consumes nothing from other tasks" task into deciding an error type for
  invalid input no caller in this codebase can currently even construct (every real duty a caller
  passes originates from `Curve::duty_at`, which is `u8`-typed but always 0-100 by construction
  of the curves it's built from).

## Issues or concerns

1. `cargo clippy --all-targets -- -D warnings` is not clean (dead_code only) — see above,
   expected per the deliverable and precedented by Task 3.
2. `tread()`'s infinite bounds at the curve's floor/ceiling duty (design decision #1 above) is
   the one place I'd most want a second pair of eyes against the original design author's intent
   before a downstream task builds a PI setpoint on top of it.

Status: **DONE_WITH_CONCERNS** (the clippy dead-code gate, expected/precedented, and the tread()
infinity judgment call worth double-checking — nothing here blocks the task or, I believe,
changes correctness).
