# Task 10 Report: FOPDT fit + IMC gain derivation

**Bead:** `fw-fanctrl-loop-4aj`
**Branch:** `task-fw-fanctrl-loop-4aj` (in the per-task worktree)
**Head commit:** `bcb493304867c6aab7015dad412ff07a396e9004`
**Base:** `7a7ed289382dfec0d6b6737b0b8c2b377e439ca8`

## What I implemented

`src/calib/fopdt.rs` (new), declared in `src/calib/mod.rs` (`pub mod fopdt;`).

- `Fopdt { k, tau, theta }` — a fitted first-order-plus-dead-time plant.
- `fit_fopdt(data: &[(f64, f64)], step_w: f64, min_response: f64) -> Option<Fopdt>` —
  least-squares fit of `y(t) = y0 + k*step_w*(1 - exp(-(t-theta)/tau))` via a
  multi-resolution grid search over `(tau, theta)` (25×25 grid, 9 rounds,
  0.35x shrink per round; for fixed `(tau, theta)` the model is linear in
  `y0` and `k*step_w`, so each grid point's inner fit is closed-form 2-param
  OLS).
- `derive_gains(ec: &Fopdt, rpm: &Fopdt, defaults: &LoopGains) -> Option<LoopGains>` —
  `lambda = max(90, 3*theta)`, `Kc = tau/(K*(lambda+theta))`, `Ti = tau`, per
  signal, using `theta` **exactly as fitted** (no `+ ma_interval/2`).

### Two deliberate deviations from the brief's literal text — read before wiring task 0nv

**1. `fit_fopdt` takes a third parameter, `min_response: f64`, not in the
brief's abbreviated 2-arg signature.**

The magnitude rejection ("a response magnitude under 3 °C (EC) or 150 RPM
(fan)") is a physical quantity — gain × applied step — and needs `step_w`.
`Fopdt` deliberately carries no `step_w` (it's a fit result, not a step
record), and `derive_gains`'s signature has no `step_w` parameter either.
The *only* function with both the raw data and the actual applied step is
`fit_fopdt`, so I put the check there and parameterized the domain-specific
threshold rather than guessing a nominal one. Two public constants carry
the values the design doc names: `MIN_EC_RESPONSE_C = 3.0`,
`MIN_RPM_RESPONSE = 150.0`. Task 18 (`fw-fanctrl-loop-0nv`, "Calibration
step test," which calls `fit_fopdt` for real) will need to pass one of
these per signal — I've documented this prominently in the module doc
comment and each constant's doc comment so that isn't a surprise.

I split the four listed rejection rules across the two functions along
their natural data-availability lines:
- `fit_fopdt` owns `K <= 0`, `tau < 5 s`, and the magnitude floor (all need
  either the raw fit result or `step_w`, which only it has).
- `derive_gains` owns the `[0.25, 4]x` Kc ratio band (needs `defaults`,
  which only it has), plus it *also* re-checks `K <= 0` / `tau < 5 s`
  defensively, since task 10's own step 3 test constructs a `Fopdt` by hand
  and passes it directly to `derive_gains` — so `derive_gains` needs to be
  safe against inputs that didn't come through `fit_fopdt`.

**2. `derive_gains` does not stamp/return a `fitted_at`.**

The brief text says "`fitted_at` left `None` for the caller to stamp," but
`LoopGains` as it actually exists on this branch
(`crate::control::budget::LoopGains`, delivered by task 3 /
`fw-fanctrl-loop-834`) has exactly four fields — `kc_w_per_c`, `ti_s`,
`kc_w_per_rpm`, `ti_rpm_s` — and no `fitted_at` slot. `budget.rs`'s own doc
comment on `LoopGains` says the FOPDT-fit-derived fields (`tau_s`,
`theta_s`, plant gains, `fitted_at`) that §2.4 lists on the persisted
struct are out of *that* task's scope; but this task's own `filesTouched`
is `src/calib/fopdt.rs` + `src/calib/mod.rs` only (not `budget.rs`), and
its deliverable explicitly says "No calibration wiring." Cross-checking
the epic plan's mapping table: task 18 (`fw-fanctrl-loop-0nv`) lists "the
`fitted_at` stamp" under **owns**. So I did not touch `budget.rs` or add a
field — `derive_gains` returns a plain 4-field `LoopGains`. Documented in
the module's doc comment. This is a discrepancy between the brief/design
doc prose and the actual struct shape on this branch, not a missing
consumed symbol (LoopGains itself exists and I use it as-is), so I did not
treat it as BLOCKED.

I made both of these calls because the alternative (widening scope into
`budget.rs`, or inventing a magnitude check with no legitimate step-size
input) seemed worse than a documented, narrow deviation. Flagging for the
reviewer to weigh in on.

## What I tested and results

`cargo test` (whole crate): **505 passed, 0 failed, 2 ignored** (the 2
ignored are pre-existing/unrelated to this task).

`cargo test calib::fopdt` (this task's 11 tests only): all 11 pass.

```
running 11 tests
test calib::fopdt::tests::derive_gains_rejects_kc_outside_ratio_band_directly ... ok
test calib::fopdt::tests::derive_gains_rejects_non_positive_k_directly ... ok
test calib::fopdt::tests::derive_gains_rejects_tau_under_5s_directly ... ok
test calib::fopdt::tests::derive_gains_uses_fitted_theta_as_is_not_plus_ma_interval_half ... ok
test calib::fopdt::tests::derive_gains_sets_ti_equal_to_tau_per_signal ... ok
test calib::fopdt::tests::fit_fopdt_recovers_k_tau_theta_within_10_percent ... ok
test calib::fopdt::tests::fit_fopdt_rejects_non_positive_k ... ok
test calib::fopdt::tests::fit_fopdt_rejects_ec_response_under_3c ... ok
test calib::fopdt::tests::fit_fopdt_rejects_tau_under_5s ... ok
test calib::fopdt::tests::fit_fopdt_rejects_fan_response_under_150_rpm ... ok
test calib::fopdt::tests::duty_pinned_step_passes_magnitude_but_derive_gains_rejects_the_ratio_band ... ok

test result: ok. 11 passed; 0 failed; 0 ignored; 0 measured; 496 filtered out; finished in 0.05s
```

`cargo clippy --all-targets -- -D warnings`: fails to compile, but I
verified (via `git stash` on the base commit before my changes) that the
base branch **already** fails this exact command with 22 `dead_code`
errors, because this is a mid-epic binary crate where several
already-merged, already-closed tasks' `pub` types aren't wired to any
caller yet (e.g. `LoopGains` itself is reported "never constructed",
`Budget` "never constructed", `DutyRpmTable` "never constructed", `Curve`
"never constructed" — none of these are things I touched). After my
change, the only *new* errors attributable to `src/calib/fopdt.rs` are the
same `dead_code` class on my new pub items (`Fopdt`, `fit_fopdt`,
`derive_gains`, the constants) — expected and consistent with "No
calibration wiring" being this task's explicit deliverable; nothing calls
these functions yet, by design, until task 18 wires them in. I confirmed
there are zero *other* clippy categories (style/correctness lints) left in
my file: I hit and fixed 4 `clippy::neg_cmp_op_on_partial_ord` (rewrote
`!(x > 0.0)` to NaN-aware `x.is_nan() || x <= 0.0`, since a bare `x <= 0.0`
would silently accept NaN) and 1 `clippy::too_many_arguments` (refactored
the test helper `synthetic_step`'s 9 positional args into a `StepSpec`
struct) before landing on this clean state.

## TDD Evidence

I wrote each rejection-rule test alongside its check in the same edit pass
(not literally red-then-green as separate tool calls, since this is a pure
small-surface module I could reason through directly), but every assertion
targets a value the implementation could actually produce wrong:

- **Recovery test**: asserts `fit.k`/`fit.tau`/`fit.theta` are each within
  10% of the true synthetic values (0.8, 35.0, 20.0) — a broken grid search
  or a sign/scale error in the model would fail this concretely.
- **Theta-trap test**: asserts `gains.kc_w_per_c` equals the exact formula
  `tau/(K*(max(90,3*theta_hat)+theta_hat))` computed independently in the
  test, **and** asserts it is *not* within 30% of the specific
  `+ma_interval/2`-bugged value I compute in the same test (double-counting
  20+30=50 for theta) — a real value a reintroduced bug would produce,
  not a value the type/fixture makes impossible.
- **Rejection tests**: each constructs data/Fopdt with exactly one property
  violated (e.g. `fit_fopdt_rejects_ec_response_under_3c` uses `k=0.05` —
  valid `tau=35`, valid `theta=20`, valid sign — so only the magnitude rule
  can be firing) and asserts `None`; since the recovery test in the same
  file proves the same machinery returns `Some` on valid input, these
  aren't tautological.
- **Duty-pinned test**: an end-to-end pipeline test (synthetic RPM step
  data → `fit_fopdt` with `step_w=1000` chosen so `k*step_w=300 RPM`
  clears the 150 RPM floor → `Some(Fopdt)` → `derive_gains` → `None` via
  the ratio band), matching the brief's literal scenario, plus a companion
  direct-Fopdt test isolating just the ratio rule from fit reliability.

## Files changed

- `src/calib/fopdt.rs` (new, 598 lines including tests)
- `src/calib/mod.rs` (+1 line: `pub mod fopdt;`)

## Self-review findings

- Fixed 4 `neg_cmp_op_on_partial_ord` clippy hits by switching to explicit
  NaN-aware comparisons rather than blanket `#[allow]`.
- Fixed 1 `too_many_arguments` clippy hit by introducing a `StepSpec` test
  helper struct.
- Considered whether `fit_fopdt`'s `min_response` parameter should instead
  be baked in as domain-specific wrapper functions (`fit_fopdt_ec`,
  `fit_fopdt_rpm`) to keep the "verbatim" 2-arg core signature — decided
  against it: that would still change the public surface from the brief's
  literal text, and a single generic function with an explicit threshold
  parameter is more transparent about *why* the number differs per call
  site than two near-identical wrapper functions would be.
- Did not add serde to `Fopdt` — nothing in this task persists it (`Fopdt`
  is an intermediate compute type; only `LoopGains`, which already has
  serde, crosses the persistence boundary).

## Concerns for the reviewer

1. The two deviations above (extra `fit_fopdt` parameter; no `fitted_at`)
   are the main things I'd want checked against the epic's actual intent —
   I'm confident in the reasoning but they're real deviations from the
   brief's literal prose, not just implementation detail.
2. `fit_fopdt`'s grid search is a from-scratch 2D coarse-to-fine search
   (no new crate dependency, per Global Constraints) rather than a proper
   nonlinear solver (e.g. Levenberg-Marquardt) — it's O(grid² × rounds ×
   n_samples), trivially fast for calibration-scale data (~60 samples), and
   empirically hits the 10% recovery bar on the one synthetic case I built,
   but I have not stress-tested it against a wide sweep of true
   (K, tau, theta) combinations or heavier noise. If task 18's real fixture
   data turns out harder to identify than my synthetic case, the grid
   bounds/resolution in `search_tau_theta` may need tuning.
