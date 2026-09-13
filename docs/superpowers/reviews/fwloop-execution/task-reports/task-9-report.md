# Task 9 report: Actuator read-back (`fw-fanctrl-loop-jpg`)

## What I implemented

**`src/actuators/mod.rs`** — added the shared `WriteVerdict` enum (`Verified(f64)`,
`Mismatch { field: &'static str, commanded: f64, read: f64 }`, `Unreadable`,
`Unverifiable`), used by both actuators.

**`src/actuators/cpu.rs`** — `set_sustained_mw` now writes
`--stapm-limit=<mw> --slow-limit=<mw> --fast-limit=<fast_limit_mw>` as before, then (if the
write succeeded) runs `ryzenadj --info` through the same `Runner`, parses the
`| Name | Value | Parameter |` table (`parse_info_table`/`InfoTable`, picking out
`PPT LIMIT SLOW`, `PPT LIMIT FAST`, `STAPM LIMIT`; `nan` or a missing row parses to `None`),
and scores it:
- both slow and fast within 0.5 W of what was just commanded (`fast_limit_mw`'s own value for
  fast, not `mw`) → `Verified(commanded_slow_w)`; STAPM is parsed but never checked.
- either disagrees by more than 0.5 W → `Mismatch { field, commanded, read }` naming the
  offending row.
- the write itself fails (spawn error or nonzero exit), `--info` fails, or a needed row is
  missing/`nan` → `Unreadable` (never `Mismatch`) — a failed write also **skips** the `--info`
  call, since there is nothing yet to verify.

**`src/actuators/gpu.rs`** — added `GpuLockVerifier` (owns `locked_mhz` + a consecutive-
violation streak) with `verify_lock(&mut self, gpu_util, gpu_sm_mhz) -> WriteVerdict`
implementing the LUT-sweep pin rule: `gpu_util <= 90%` → `Unverifiable` (streak resets — an
unloaded lull says nothing about the next loaded sample); above 90% and `gpu_sm_mhz <= locked +
30` → `Verified(gpu_sm_mhz)` (streak resets); above the ceiling → streak+1, `Unverifiable` while
the streak is below 3, `Mismatch { field: "gpu_sm_mhz", commanded: ceiling, read: gpu_sm_mhz }`
on the 3rd consecutive violation. `set_max_clock`'s own signature is untouched — the design only
adds `verify_lock`, it does not change the write path.

**`src/control/controller.rs`** — the four existing `cpu.set_sustained_mw(...)` call sites
(`Command::SetCpuW`, the auto-loop allocation, calibration's `apply_calib_effects`,
`reassert_actuators`) now match on `WriteVerdict`: `Verified(w)` takes the old `Ok` branch
verbatim (status update, `stick_violations` reset where it existed, `Effect::CpuSet`); every
other verdict takes the old `Err` branch (warn, leave status unchanged), each with a comment
naming `fw-fanctrl-loop-j6s` (Task 19 / `fwloop.12`, the controller-loop-integration bead that
owns the freeze/flag/reassert/three-strike wiring). The RAPL stickiness watchdog's own decision
logic (violation counting, streak thresholds, cause selection) is untouched — only the
match-arm around the actuator call changed. GPU's `set_max_clock` call sites are **untouched**:
nothing about that signature changed, so nothing forces (or is tested to require) a
controller-side edit for `verify_lock` in this task — see "Scope decision" below.

**`src/selftest.rs`** (not in the brief's file list, but a hard compile dependency —
`cpu_limit_step` calls `set_sustained_mw`) — adapted the one call site the same way: only
`Verified` proceeds to the burn-and-measure step; anything else returns
`Err("ryzenadj: not verified: {verdict:?}")`.

**`src/actuators/cmd.rs`** (test infrastructure, not in the brief's file list) — `FakeRunner`'s
unscripted (`push_result` queue empty) default for `ryzenadj --info` now synthesizes a table
that agrees with whatever was most recently written via a non-`--info` `ryzenadj` call, via a
new `synthesize_ryzenadj_info` helper — see "Why `cmd.rs` needed a change" below.

## Scope decision: GPU's `verify_lock` is not wired into `controller.rs`

The brief and bead both describe the controller-side edit as "the two actuator call sites" and
say non-`Verified` verdicts are mapped "until Task 19" — phrasing that could be read as
requiring a GPU-side controller edit too. I did not add one, for two reasons:

1. **Nothing forces it.** `set_max_clock`'s signature is unchanged (only `set_sustained_mw`'s
   changed), so `controller.rs` compiles today exactly as it did before for the GPU path. The
   "two actuator call sites" phrasing is satisfiable, and I believe intended, as "the call sites
   for the two actuators" as a category (only CPU's needed edits, since only CPU's return type
   moved) rather than a literal count of edited call sites.
2. **Nothing in the acceptance criteria asks for it.** The GPU acceptance criterion is a pure
   unit test on `verify_lock` against raw util/clock inputs; there is no criterion exercising
   `verify_lock` from inside `on_sample`. Wiring it in would require new controller state (which
   `gpu_util`/`gpu_sm_mhz` samples to feed it, when to construct/reset the verifier across
   lock re-commands) that is exactly the "freeze/flag/reassert/three-strike rule" the design
   text itself defers to Task 19 (`fw-fanctrl-loop-j6s`, "Controller loop integration").

This is a real interpretive judgment call, not a certainty — flagging it explicitly so a
reviewer who reads "the two actuator call sites" as a hard count can course-correct. `cargo
clippy --all-targets -- -D warnings` self-discloses the consequence: `GpuLockVerifier`
(struct/`new`/`verify_lock`) and `WriteVerdict::Unverifiable` are dead code outside `#[cfg(test)]`
— see "Quality gate" below; this matches the exact pattern Task 3's report recorded for
`budget.rs` (a fully-tested, not-yet-wired module, deferred to the task that wires it in), which
is the established precedent in this tree for "own the type, don't own the integration."

## Why `cmd.rs` needed a change

`CpuActuator::set_sustained_mw` now makes **two** `Runner` calls (write, then `--info`) where it
made one before. `FakeRunner`'s unscripted default was "success, empty stdout" — which parses to
an empty `InfoTable`, i.e. `Unreadable`, for every pre-existing test that never scripted an
`--info` reply (the entire suite, since it predates read-back). Running the full suite after the
`cpu.rs`/`controller.rs` change alone (before touching `cmd.rs`) showed the blast radius:

```
test result: FAILED. 40 passed; 51 failed; 0 ignored; 0 measured; 410 filtered out
```

Rewriting 51 call sites individually to hand-script a matching `--info` reply would have gone
far beyond "the two actuator call sites" scope and touched most of `controller.rs`'s test
module. Instead I made `FakeRunner`'s empty-queue fallback for `ryzenadj --info` **synthesize** a
table from the most recently *written* `ryzenadj` args (`synthesize_ryzenadj_info`), so a test
that never cared about read-back (the whole pre-existing suite) gets a `Verified`-shaped default
"for free," while a test that explicitly needs `Mismatch`/`Unreadable` still scripts the queue
by hand — FIFO gives explicit scripts priority over the synthesized default. I also had to
narrow `controller.rs`'s own `ryzenadj_calls` test helper to exclude the new `--info` calls (it
asserts "what did we command," which read-back is not) — the exact args list it checks would
otherwise have gained a spurious trailing `["--info"]` entry on every CPU-actuating test.
After both changes, the full suite (`cargo test --bin fw-fan-quiet`) is 499/499, unchanged from
before this task's file count (91 pre-existing controller tests + 40 actuator tests, all still
green, plus my new tests).

## TDD evidence

**Steps 1–4 (CPU parser + verify_write), and steps to update `controller.rs`/`selftest.rs`/
`cmd.rs`:** written together with their implementation rather than as separate strict red→green
increments — the read-back format (the fixture table shape) and the tolerance rule were already
fully specified by §2.9 and the existing fixture, so I judged incremental red-steps would add
noise, not information, here. I did run the new+existing suite together immediately afterward
and it was green on the first try (`cargo test --bin fw-fan-quiet actuators::cpu::` → 16/16,
`control::controller::` → 91/91) — I do **not** have a captured RED transcript for these, and am
saying so rather than reporting a fabricated one.

**Step 5 (`verify_lock`) — genuine RED → GREEN**, since I wrote all six `gpu.rs` tests before
re-running:

RED — `cargo test --bin fw-fan-quiet actuators::gpu::`:
```
---- actuators::gpu::tests::a_below_floor_sample_resets_the_overshoot_streak stdout ----
thread '...' panicked at src/actuators/gpu.rs:361:9:
assertion `left == right` failed: streak restarted after the reset, so this is only the 3rd since
  left: Unverifiable
 right: Mismatch { field: "gpu_sm_mhz", commanded: 2030.0, read: 2031.0 }
test result: FAILED. 7 passed; 1 failed; 1 ignored; 0 measured; 492 filtered out
```
Cause: the test itself only drove the streak to 2 after the reset (1 low-util reset + 2
overshoots), not 3, so the implementation was correct and the *test* was miscounted — a genuine
"the code doesn't do what I expected" signal, not a scripting slip caught before running.

GREEN — same command, after fixing the test to drive a full 3 overshoots since the reset:
```
test result: ok. 8 passed; 0 failed; 1 ignored; 0 measured; 492 filtered out
```

## Full-suite verification

```
$ cargo test --bin fw-fan-quiet
test result: ok. 499 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out
```
(verified against the base commit directly — `git stash` + `cargo test --bin fw-fan-quiet` on
the unmodified tree gives `485 passed; 0 failed; 2 ignored`; this task adds a net +14: cpu.rs
went from 7 tests to 16 [+9], gpu.rs from 4 to 9 [+5, one `#[ignore]`d in both]. No existing test
was deleted — the pre-existing CPU-actuator tests were rewritten in place to match the new
return type, not removed).

The RAPL stickiness watchdog tests specifically (`control::controller::`, includes
`stickiness_flags_after_three_consecutive_violations`,
`stickiness_flag_transitions_reach_telemetry_as_flag_lines`, `stickiness_ignores_invalid_zero_power`,
`strict_stickiness_window_fires_on_two_violations_after_resume`,
`failed_stickiness_reassert_reports_stickiness_failed`, `alloc_change_resets_stickiness_streak`,
plus all 91 controller tests): **91/91 passing**, confirmed both standalone
(`cargo test --bin fw-fan-quiet control::controller::`) and inside the full-suite run above.

```
$ cargo clippy --all-targets -- -D warnings
```
**Not clean** — matches the established pattern for a task that owns a type without owning its
controller wiring (see Task 3's report on `budget.rs`). Diffing against the base commit (`git
stash` + re-run) confirms exactly 21 dead_code errors pre-date this task (budget.rs,
fanctrl/curve.rs, fanctrl/table.rs, gpu_pid.rs unused-constant leftovers from earlier
not-yet-wired tasks) and this task adds exactly 6 more, all attributable to the scope decision
above: `GpuLockVerifier` (struct + `new` + `verify_lock`), its three `VERIFY_*` constants, and
`WriteVerdict::Unverifiable` — every one of them fully exercised by `#[cfg(test)]` code, just
not (yet) reachable from `fn main`. `cargo test` (the SDD launch config's actual per-task gate;
`cargo clippy --all-targets` is the end-of-epic sweep per `progress.md`) is fully green.

`cargo fmt --check`, restricted to the six files this task touched: clean (I ran `rustfmt
--edition 2024` on exactly those six paths; the only remaining diff `cargo fmt --check`
reports crate-wide is in `src/control/budget.rs`, which I never touched — pre-existing drift).

## Files changed

- `src/actuators/mod.rs` — new `WriteVerdict` enum.
- `src/actuators/cpu.rs` — `set_sustained_mw` returns `WriteVerdict`; new `parse_info_table`/
  `InfoTable`/`MISMATCH_TOLERANCE_W`; rewrote the CPU actuator's unit tests (steps 1–4 plus the
  clamp/write-failure tests adapted to the new signature).
- `src/actuators/gpu.rs` — new `GpuLockVerifier`/`verify_lock` plus its unit tests (step 5);
  `set_max_clock` untouched.
- `src/control/controller.rs` — the four `cpu.set_sustained_mw` call sites adapted to
  `WriteVerdict` (each with an `fw-fanctrl-loop-j6s` comment); `ryzenadj_calls` test helper
  narrowed to exclude `--info` calls. RAPL stickiness watchdog decision logic untouched.
- `src/selftest.rs` — its one `set_sustained_mw` call site adapted (hard compile dependency,
  not in the brief's file list).
- `src/actuators/cmd.rs` — `FakeRunner`'s empty-queue default for `ryzenadj --info` now
  synthesizes a matching read-back table (shared test infrastructure, not in the brief's file
  list, but needed to keep the pre-existing suite green without individually rewriting it).

## Self-review findings

- **Field-name choice for GPU's `Mismatch`:** the design names the CPU's fields verbatim
  (`PPT LIMIT SLOW`/`PPT LIMIT FAST`) but doesn't name one for GPU; I used `"gpu_sm_mhz"`
  (the design's own parameter name for the reading). No test constrains this string beyond
  self-consistency with what I chose, so a reviewer may want a different name — trivial to
  change.
- **Boundary at exactly 0.5 W / exactly 90% util:** I treat "within 0.5 W" as `<= 0.5` (only
  `> 0.5` fails) and "above 90%" as strictly `> 90.0` (so `<= 90.0` is `Unverifiable`), per the
  design's own wording ("within 0.5 W", "gpu_util > 90%"). Not independently re-verified against
  real hardware — the design text is the only source for these boundaries.
- **`GpuLockVerifier` state lifetime:** I did not decide (because nothing in this task needed
  me to) whether a future controller integration constructs one `GpuLockVerifier` per lock-value
  change or reuses one across re-locks at the same value — that's Task 19's design space, not
  pre-empted here.

## Concerns

1. The scope decision above (GPU's `verify_lock` left unwired) is the one place I made a
   judgment call against ambiguous brief text rather than a literal instruction — flagged there
   in detail for the reviewer.
2. `cargo clippy --all-targets -- -D warnings` is not clean, by design-consistent-with-precedent
   (6 new dead_code errors, all from the deliberately-unwired `GpuLockVerifier`/`Unverifiable`);
   `cargo test`, the actual per-task gate per the SDD launch config, is green at 499/499.
