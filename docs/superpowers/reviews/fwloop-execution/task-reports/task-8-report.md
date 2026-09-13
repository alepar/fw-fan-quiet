# Task 8 Report: EC replica + NVMe + AC sensors (fw-fanctrl-loop-52c)

## What I implemented

`src/sensors/ec.rs` (new), plus additions to `src/sensors/hwmon.rs` and a
`pub mod ec;` line in `src/sensors/mod.rs`.

### `EcLabel`

Thin wrapper around a trimmed `tempN_label` string. `is_controllable()`
classifies by prefix: `apu`/`cpu`/`gpu_*` -> controllable; everything else
(including `ambient`/`charger`) -> uncontrollable, per §2.2 of the design
doc. The "everything else" default is deliberate — a label neither known
prefix set matches falls safely to uncontrollable rather than being assumed
steerable.

### `EcReading`

`EcReading::read(dir: &Path) -> Option<Self>` scans `temp1_label`..`temp32_
label` under `dir` (a resolved `cros_ec` chip directory — same shape the
checked-in fixtures use, e.g. `tests/fixtures/hwmon/cros_ec_idle/`), pairing
each label with its `tempN_input` sibling. A missing `_label` file just
means "no sensor at this index" (`continue`, scan keeps going past gaps).
A missing/unreadable/unparsable `_input`, or a value `<= 0`, is dropped
without invalidating the rest of the reading — this is what makes the -150
sentinels and the input-less `gpu_temp@40` sensor (the ENODATA convention)
transparent instead of poisoning the whole read. The max is picked with a
strict `>` comparison over the surviving positive readings in ascending
`tempN` order, so a tie resolves to the lower `N` (sysfs order) by
construction — no separate tie-break step needed. `max_c` is the rounded
integer of that max; `all` keeps every surviving `(EcLabel, f64)` pair
unrounded, in sysfs order. `read()` returns `None` only when no positive
reading survives at all (chip present but every sensor dead, or the
directory doesn't exist) — the hook `Sample.ec_valid` (a later task) will
key off that.

I deliberately did **not** add a `cros_ec` chip-discovery layer on top of
`EcReading::read` (mirroring `Hwmon::discover`'s own scan-for-a-named-chip
logic) — this task's file list only touches `ec.rs`/`hwmon.rs`/`mod.rs`, and
`Hwmon`'s `chips: HashMap` is private with no accessor, so `ec.rs` doing its
own duplicate root scan would either need a new public accessor on `Hwmon`
(out of scope per the brief) or hand-rolled duplicate discovery I couldn't
verify against anything. Wiring `EcReading::read` to a resolved chip
directory is the sampler-integration task's job; I built and tested the
part this task owns.

### `EcAverage` — the boxcar + its off-by-one

`push(&mut self, sample_c: f64) -> Option<f64>` is the one method that
matters here: it returns the moving average **as of before** `sample_c` is
folded in (samples `n-N..n-1`), then appends `sample_c` (dropped if `<= 0`,
same rule as an EC reading) for the *next* call to see. This fuses "read
the old mean" and "append the new sample" into one atomic call so the
off-by-one (`FanController.py`'s `adapt_speed` reads `mean(buffer)` before
`buffer.append(current_sample)`, per `docs/research/05-fw-fanctrl-loop.md`
line 25-26) is a structural property of the type, not something a caller
has to get the call order right for. The doc comment on `push` explicitly
warns against "fixing" it to include `sample_c` in its own return, and the
test (`push_returns_the_pre_push_mean_literal_values`) pins five
consecutive calls to literal expected means.

`set_interval(n)` clamps to `[1, MAX_INTERVAL=100]` and never clears —
shrinking pops the oldest samples down to the new cap, growing just raises
the cap (existing samples untouched; the buffer fills toward the wider
window through later `push` calls). `reseed(value)` clears the buffer to a
single sample and is the only operation that does. `is_seeded()` is a
sticky flag: false until either `reseed` is called or the buffer naturally
reaches a full `interval` samples; nothing un-sets it once true (there's no
"un-seed" operation described in the design doc, only `reseed`, which
itself always seeds). `sample_count()` reports the retained count.

### `sensors/hwmon.rs` additions

- `Hwmon::nvme_composite_c(&self) -> Option<f64>` — follows the exact same
  pattern as the existing `cpu_temp_c`/`igpu_w` (`read_chip_value("nvme",
  "temp1_input")` / 1000.0). `None` when the `nvme` chip is absent.
- `pub fn on_ac(power_supply_root: &Path) -> Option<bool>` — a free
  function, not a `Hwmon` method, because `/sys/class/power_supply` is a
  different sysfs subtree than the `hwmon` root `Hwmon::discover` scans;
  putting it on `Hwmon` would imply a root it doesn't have. Reads
  `<root>/ACAD/online`; `"1"` -> `Some(true)`, `"0"` -> `Some(false)`,
  anything else (missing file, unreadable, garbage) -> `None`.

## Testing

**Honest process note, not the idealized version:** I designed the module
(types + all tests) from the brief and the design doc's §2.2/§Facts/§3.4
together, then implemented it, then ran the suite. That is not strict
single-behavior red/green — I did not commit a series of individually
failing tests before each increment. One real failure surfaced on the
first full run (`set_interval_grows_and_shrinks_...`, see Self-review
below): my test's own expected count was wrong (my design reasoning about
"grow after shrink" miscounted when the cap was still enforced), not the
implementation. I fixed the test, re-ran, green.

Because that means I don't have genuine per-behavior RED transcripts to
report, I went back afterward and **actually ran** the one experiment
whose result the report leans on hardest — whether the off-by-one test
would catch the "obvious wrong fix" (returning the *post*-push mean
instead of the pre-push one). This was a real edit-run-revert done in this
session, not a claim:

```
$ cargo test sensors::ec::tests::push_returns_the_pre_push_mean_literal_values
# (with push() edited to call self.mean() *after* appending sample_c)
thread '...' panicked at src/sensors/ec.rs:372:9:
assertion `left == right` failed: nothing retained before the first push
  left: Some(10.0)
 right: None
test result: FAILED. 0 passed; 1 failed
```
Reverted immediately after (`cp` from a saved-good copy), then confirmed
green again:
```
$ cargo test sensors::
test result: ok. 50 passed; 0 failed; 1 ignored
```

### Full suite

```
$ cargo test
test result: ok. 473 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out; finished in 2.00s
```
(50 of those 473 are in `sensors::` — the 21 new tests this task added, plus
the pre-existing 29.) Output pristine, no warnings printed by the test
binary itself.

```
$ cargo clippy --all-targets
# 22 warnings, 0 errors. All 22 are pre-existing-pattern "never
# constructed / never used" dead_code warnings on this branch's not-yet-
# wired modules: 12 pre-existed on budget.rs (from the already-merged
# fw-fanctrl-loop-834 task, unrelated to this one) and 10 are on ec.rs's
# public API, expected because nothing wires EcReading/EcAverage into the
# sampler yet -- that's a later task, same as budget.rs's Budget/LoopGains/
# Freeze/LoopError are unwired pending the controller-integration task.
# No clippy correctness/style lint fired on any file this task touched.
```

```
$ rustfmt --check --edition 2024 src/sensors/ec.rs src/sensors/hwmon.rs src/sensors/mod.rs
# clean, no diff
```
(Note: `cargo fmt --check -- <files>` does **not** actually scope to the
files passed after `--` in this cargo/rustfmt version — it still walks and
reports on the whole crate, which is how I discovered and then reverted an
accidental whole-crate reformat that had touched `src/control/budget.rs`,
a file outside this task's scope. `rustfmt --check` invoked directly, as
above, does scope correctly and confirms the three files this task owns
are clean.)

## Assertion discipline

Every assertion in the new tests names a concrete value the code under test
could actually produce that would fail it:
- `push_returns_the_pre_push_mean_literal_values`: each of the 5 literal
  expected means would be wrong under the "obvious" post-push
  implementation (verified during RED, see above) or under any other
  off-by-one direction (e.g. skipping two samples instead of one).
- `ties_in_the_max_are_broken_by_sysfs_order`: fails if the tie-break used
  `>=` instead of `>` (would pick `cpu@4c`, the later index, instead of
  `apu_f75303@4d`).
- `cros_ec_idle_max_is_ambient_47_85_rounds_to_48` / `..._load_...75...`:
  fail under wrong rounding (truncation would give 47/74), wrong drop rule
  (a -150 or the input-less sensor surviving would change the max or the
  survivor count), or a discovery bug that missed `ambient`/`cpu`.
- `set_interval_grows_and_shrinks_without_clearing_and_caps_at_100`: the
  cap-at-100 half genuinely exercises the boundary — 150 pushes into an
  unbounded interval, asserting exactly 100 retained; not decoration, since
  a missing `.clamp` would produce 150 instead.
- `on_ac_none_on_an_unexpected_value`: a fixture with `"banana"` — fails if
  the parser fell back to a truthy/falsy string check instead of exact
  `"1"`/`"0"` matching.

No assertion in this file is sequenced after a failing one in the same
test body, and none of them are checks a type already guarantees (e.g. no
"length <= 32" check on something capped at compile time).

## Files changed

- `src/sensors/ec.rs` (new) — `EcLabel`, `EcReading`, `EcAverage`, 21 unit
  tests.
- `src/sensors/hwmon.rs` — `Hwmon::nvme_composite_c`, free fn `on_ac`, 5
  new unit tests appended to the existing `mod tests`.
- `src/sensors/mod.rs` — `pub mod ec;`.

## Self-review findings

- Initial draft of `set_interval_grows_and_shrinks_...` had a wrong
  expected count after growing the interval past a shrink (asserted 4,
  should have been 3, since `push` still enforces the *old* cap during the
  call that happens before `set_interval` grows it). Caught by RED/GREEN
  running for real, not just written and assumed — fixed the test, not the
  implementation, since the implementation's behavior was correct and
  matches the design doc's "no discontinuity" requirement.
- One stray non-ASCII character (a leftover Cyrillic word) in an assertion
  message from an autocomplete artifact — caught on re-read, fixed before
  committing.
- Accidentally reformatted `src/control/budget.rs` (outside this task's
  scope) via `cargo fmt -- <file list>` not actually scoping as documented
  above — reverted with `git checkout -- src/control/budget.rs` before
  committing, confirmed via `git status --short` that only this task's
  three files are touched.

## Concerns / open items for later tasks

- `EcReading::read` takes an already-resolved chip directory rather than
  performing its own `cros_ec`-by-name discovery from a hwmon root. The
  sampler-integration task (not this one) will need to resolve that
  directory — either by adding a small accessor to `Hwmon` or by giving
  `ec.rs` its own root-scanning constructor. I did not build that here
  because it's outside this task's file list and the brief's acceptance
  criteria are all phrased in terms of reading a resolved fixture
  directory directly.
- `EcAverage` is not yet wired into anything (`Sample.ec`, the controller's
  live instance, etc.) — per the design doc and the `fwloop.12` task text,
  that wiring is explicitly a later task's job ("blocked-by fwloop.3:
  consumes `EcAverage::{push, set_interval, reseed}`"). The dead_code
  warnings on this module are therefore expected, matching the same
  pattern already established by the merged budget integrator task.
