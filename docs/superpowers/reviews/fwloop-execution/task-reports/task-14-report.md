# Task 14 report: Sample plumbing + socket poller (fw-fanctrl-loop-sov)

## Status: IMPLEMENTED

`bd comments fw-fanctrl-loop-sov` returned no comments — the brief itself, plus the design doc
(§3.4, §2.1, §2.2) and the already-landed `fanctrl/client.rs` (fwloop.2 / task 7) and
`sensors/ec.rs` + `sensors/hwmon.rs` (fwloop.3 / task 3), were the full input.

## What I implemented

### `src/types.rs`

`Sample` gains the seven new fields: `ec: Option<EcReading>`, `ec_valid: bool`,
`nvme_temp_c: Option<f64>`, `fanctrl: Option<FanctrlView>`, `fanctrl_freshness: Freshness`,
`fanctrl_view_changed: bool`, `on_ac: bool`. Two structural consequences, both required and both
worked through:

- **`Sample` is no longer `Copy`** — `EcReading` and `FanctrlView` each own a `Vec`/`String`. The
  only call site that relied on the old implicit bitwise copy was `Sampler::spawn`'s
  multi-subscriber fan-out loop (`for tx in &txs { tx.send(Event::Sample(sample)) }`), fixed to
  `sample.clone()` per subscriber. I grepped the whole crate for every other `Sample { ... }`
  construction and `Event::Sample(...)` match site (controller.rs, allocator.rs, watchdog.rs,
  calib/runner.rs, calib/lut_sweep.rs, model.rs, ui/view.rs, telemetry.rs) — every one either
  borrows (`&Sample`) or moves a freshly-constructed/matched value once; none depended on `Copy`.
- **`Default` can no longer be derived** — `Freshness` (owned by `fanctrl/client.rs`, outside
  this task's file scope) has no `Default` of its own. Wrote a manual `impl Default for Sample`
  instead of widening `Freshness`'s own file; the new fields default to "nothing observed yet"
  (`None`/`false`), and `fanctrl_freshness` defaults to `Freshness::Stale` — the same verdict
  `compute_freshness` itself returns for `(last_absent: false, view: None)`, so a bare
  `Sample::default()` and a freshly-constructed real source in the same "never polled" state agree.
  `Default` is load-bearing: every existing `Sample { field: v, ..Sample::default() }` test
  fixture across the tree (controller.rs, allocator.rs, watchdog.rs, calib/*, model.rs, ui/view.rs)
  depends on it and none of them name every field, so I verified no fully-exhaustive `Sample {}`
  literal exists anywhere before relying on that.

`ec`, `fanctrl`, and `fanctrl_freshness` carry `#[serde(skip)]`: `EcReading`/`Freshness` don't
derive `Serialize` (extending them is outside this task's files) and `FanctrlView` carries
`Instant` stamps, which serde cannot serialize at all. The flattened telemetry columns
(`ec_max`, `fanctrl_speed`, `strategy`, ...) are explicitly fwloop.15's job per the design doc's
§3.5 telemetry-field list, not this task's. `ec_valid`, `nvme_temp_c`, `fanctrl_view_changed`,
`on_ac` are plain and stay on the wire.

### `src/sensors/poller.rs` (new file)

Owns both background pollers, kept separate from `sampler.rs` per the task's `filesTouched`
including `sensors/mod.rs` (only needed if a submodule is added):

- **`FanctrlPoller`**: `tick(&mut self, now: Instant)` is the testable core — sends whichever of
  `Speed`(5 s)/`All`(30 s) is due via a fixed `next_due += PERIOD` schedule (no drift), against a
  `SharedFanctrl = Arc<Mutex<Box<dyn FanctrlSource + Send>>>`. `spawn` wraps it in a real thread
  looping `tick(Instant::now())` + a 250 ms-sliced shutdown-aware sleep (reusing the sampler's own
  `sleep_unless_shutdown`, now `pub(crate)`). The sampler tick only ever calls `view()`/
  `freshness()` through the same handle — never `poll` — so a hung socket round trip on this
  thread cannot reach a `Sample`.
- **`spawn_nvme_poller`**: runs a caller-supplied `FnMut() -> Option<f64>` at 30 s on its own
  thread, publishing `(temp, Instant::now())` into a `SharedNvme = Arc<Mutex<Option<(f64,
  Instant)>>>` on success. `read_nvme(cache, now)` is the sampler tick's side: `None` if never
  populated or older than `NVME_STALE_AFTER` (90 s — 3x its own 30 s cadence, the same ratio
  `fanctrl::client::ALL_STALE_AFTER` uses over `print all`'s 30 s poll; the design doc specifies
  the *shape* of this rule, not a number, so I picked the one already established in the file this
  rule mirrors and documented why). A still-in-flight `read` only delays that thread's own next
  iteration — it shares no lock with the sampler or `FanctrlPoller`.

### `src/sensors/sampler.rs`

`Sampler` gains `power_supply_root`, `cros_ec_dir: Option<PathBuf>` (resolved once at
construction via a small local `find_chip_dir` scan — `Hwmon` only exposes named per-sensor
getters, not a raw chip directory, and widening that API is `sensors/hwmon.rs`, outside this
task), `fanctrl: SharedFanctrl`, `nvme_cache: SharedNvme`, and `prev_all_observed_at: Option<Instant>`.
`sample_at(t_mono)` now also: reads `EcReading::read(cros_ec_dir)` and `hwmon::on_ac(...)`
directly (cheap sysfs, same cost class as the existing fan/temp reads); calls
`poller::read_nvme` and a new `merge_fanctrl` helper (locks `fanctrl` just long enough to clone
`view()` and call `freshness(now)`) — never polls either. `fanctrl_view_changed` compares this
tick's view's `all_observed_at` against the value stashed from the *previous* tick, so it flips
exactly once per new `All` view and never on a `Speed`-only refresh (which never touches
`all_observed_at`).

One deliberate single-source-of-truth choice: `sample_at`'s `now: Instant` used for
`freshness`/`read_nvme` is derived as `self.epoch + Duration::from_secs_f64(t_mono)`, not a fresh
`Instant::now()` — the same injected `t_mono` tests already use for resume-gap detection now also
drives the fanctrl/NVMe time base deterministically, and matches the brief's "all
freshness/cadence decisions are computed from the Sample/FanctrlView monotonic timestamps"
requirement directly (no second, independent clock in this path at all).

`Sampler::with_paths`/`new_system` signatures grew (`power_supply_root`, `fanctrl`, `nvme_cache`
params) — the only two call sites are this file's own tests and `main.rs`, both updated.

### `src/main.rs`

Exactly what the brief scoped: poller *construction* and *shutdown join*, nothing else.
Constructs the real `UnixFanctrlClient` from `config.fanctrl_socket.clone()` (grabbed before
`config` moves into `Controller::new` later in the function), boxes it as
`Box<dyn FanctrlSource + Send>` behind the shared `Arc<Mutex<_>>`, spawns `FanctrlPoller`; builds
a second `Hwmon::discover("/sys/class/hwmon")` instance (a plain owned value, safely moved into
the NVMe thread's closure — no shared state with the sampler's own `Hwmon`) and spawns
`spawn_nvme_poller` with it. `Sampler::new_system` now takes the fanctrl/NVMe handles. Both
threads are joined right after the existing `sampler.join()` call, using the same pattern (log
and continue on panic, don't propagate).

## Acceptance criteria — evidence

> over a 60 s scripted run the poller issues `Speed` at 5 s ±1 tick and `All` exactly twice

`sensors::poller::tests::speed_every_5s_and_all_every_30s_over_a_60s_scripted_run` — 60
synthetic 1-second ticks (no real sleeping) against a command-recording fake source; asserts
`Speed` count == 12, `All` count == 2, and that both fire in the right order (`All` before
`Speed`) on the ticks where both are due. The fixed `next_due += PERIOD` schedule never drifts
when driven at 1-tick granularity, so the "±1 tick" tolerance is trivially satisfied — no jitter
to tolerate in this design. `cargo test --bin bazerame-fans sensors::poller`: **5/5 passing**.

> `Stale` after 90 s of `All` failures and after 15 s of `Speed` failures

Two tests, each isolating one window from the other by keeping the *other* poll type
continuously succeeding (or, for the Speed case, simply never letting `All` come due again in the
short test window): `freshness_stale_after_90s_of_all_failures_with_speed_kept_fresh` and
`freshness_stale_after_15s_of_speed_failures_with_all_kept_fresh_by_the_seed`. Both assert the
exact boundary (`Fresh` one second before the window elapses, `Stale` exactly at it), driven
through the real `FanctrlPoller::tick` + `FakeFanctrl`, not a direct call into
`compute_freshness` (which is already exhaustively covered in `fanctrl/client.rs`'s own tests) —
this is testing the poller's cadence integration, not restating that coverage.

> `fanctrl_view_changed` is true exactly once per new `All` view and never on a `Speed`-only
> refresh

`sensors::sampler::tests::fanctrl_view_changed_true_once_per_new_all_view_only` — drives a
scripted `FakeFanctrl` through All → Speed → Speed → All → (no poll), interleaved with
`sampler.sample_at`, asserting the flag on every one of the five resulting samples: false, true,
false, false, true, false.

> a scripted NVMe read that blocks for 60 s delays no `Sample`, leaves `Freshness` `Fresh` and the
> `print speed` cadence intact, and surfaces only as `nvme_temp_c: None`

`sensors::sampler::tests::nvme_blocking_read_does_not_stall_sampler_or_fanctrl_poller`. I did
**not** use a literal 60 s sleep (would make the suite slow without adding any signal beyond "a
mpsc channel can outlast a short one") — instead the "NVMe read" closure blocks on
`mpsc::Receiver::recv()` until the test explicitly releases it, modelling an *arbitrarily* long
stall deterministically: correctness here is structural (the read runs on its own thread behind
its own mutex, sharing nothing with the sampler or `FanctrlPoller`), not time-magnitude-dependent,
so a literal 60 s wait would prove nothing a channel-blocked wait doesn't already prove, at 240x
the cost. The test is genuinely load-bearing: if the NVMe read were ever reachable from the
sampler tick (the bug this whole three-thread design exists to prevent), the `sampler.sample_at`
calls inside the test would themselves block on the still-held channel and the test would hang
against its own bounded setup-wait, not silently pass. Sequence: spawn the NVMe thread (blocks
immediately) and a `FanctrlPoller` thread (always-succeeding fake) → bounded (2 s) wait for both
"nvme thread is inside the blocking read" and "fanctrl poller's first view landed" → three
`sample_at` calls, each asserting `nvme_temp_c == None` and `fanctrl_freshness == Fresh` → release
→ join both threads. Runs in ~0.25 s wall clock as part of the full suite.

> the real client is constructed from the configured path and both threads join on shutdown

This is `main.rs` wiring, not a unit-testable function (`main` isn't a library entry point here —
this crate has no lib target, only a bin). Verified by code reading: `fanctrl_source` is built
from `UnixFanctrlClient::new(config.fanctrl_socket.clone())` (the real client, the configured
path) before `config` moves into `Controller::new`; `fanctrl_poller_thread`/`nvme_poller_thread`
are joined immediately after the existing `sampler.join()`, using `shutdown` (already flipped
`true` earlier in the same shutdown sequence) — the same pattern the sampler/LED threads already
use. **I'm flagging this explicitly as unmeasured, not green**: the design doc's own task table
assigns exactly this kind of check — "config → poller construction" — to fwloop.23 (task 23, the
integration sweep), whose own acceptance criteria list it verbatim as one of the integration
tests "no per-task test covers." I did not fabricate a `main()`-level test to paper over that; it
belongs there, with real end-to-end wiring across every task's pieces, not faked here.

> no wall-clock (`SystemTime::now`) appears in any freshness or cadence decision on this path

`grep -n "SystemTime" src/types.rs src/sensors/sampler.rs src/sensors/poller.rs
src/sensors/mod.rs src/main.rs` — zero matches across all five touched files.

## Full test run

`cargo test --bin bazerame-fans`: **562 passed, 0 failed, 2 ignored** (the 2 ignored predate this
task, unrelated to this path). Focused runs before the full one: `sensors::poller` (5/5),
`sensors::sampler` (8/8, all passing including the two pre-existing tests I strengthened with new
field assertions), `types::` (1/1).

`cargo clippy --all-targets`: 34 warnings, all pre-existing `dead_code` from not-yet-integrated
code (`Curve`/`DutyRpmTable`/`EcAverage`/etc. from earlier tasks, plus this task's own `ec`,
`fanctrl`, `fanctrl_freshness` — read only by later tasks per the design doc's own consumer
boundaries). I compared against the pre-task baseline (`git stash` + same command): **63
warnings before, 34 after** — this task's own consumption of previously-dead `on_ac`,
`EcReading::read`, `nvme_composite_c`, and every `fanctrl::client` export actually *reduces* the
count; it adds exactly 3 new dead fields, all documented as intentional (later-task consumers).
`cargo clippy --all-targets -- -D warnings` fails both before (62 errors) and after (33 errors)
this task's changes for the same pre-existing reason — not a regression this task introduced, and
not something in scope to fix (it would mean either wiring in unrelated future tasks' consumers or
deleting their not-yet-used code, both outside `filesTouched`).

`cargo fmt --check`: clean on every file this task touches. (One unrelated pre-existing
`fmt --check` diff on `src/control/budget.rs`, outside this task's files — I ran `cargo fmt --
<my 5 files>` first, which unexpectedly reformatted `budget.rs` too; caught it via
`git status --short` before committing and reverted that file with `git checkout --
src/control/budget.rs`. Worth a note for whoever runs `cargo fmt` on this tree next: pass exact
file paths with care, or expect it to touch files outside your own task.)

## Files changed

- `src/types.rs` — `Sample`'s seven new fields, manual `Default`, field-presence test.
- `src/sensors/poller.rs` (new) — `FanctrlPoller`, `spawn_nvme_poller`/`read_nvme`, their tests.
- `src/sensors/sampler.rs` — EC/AC/NVMe/fanctrl merge in `sample_at`, `Sampler` field/constructor
  changes, `sleep_unless_shutdown` made `pub(crate)`, fan-out `.clone()` fix, new tests.
- `src/sensors/mod.rs` — `pub mod poller;`.
- `src/main.rs` — poller construction (real `UnixFanctrlClient`, real NVMe `Hwmon`) + shutdown
  join, `Sampler::new_system` call site updated.

## Self-review

- Removed an initially-added `handle()` accessor on `FanctrlPoller` once `cargo clippy` flagged it
  as dead code — nothing in this task's own wiring needed it (both `main.rs` and the tests already
  hold their own `Arc` clone from before constructing the poller), so it was unbuilt scope, not a
  gap.
- Removed an initially-drafted `sample_is_no_longer_copy_but_is_still_clone` test in `types.rs`:
  it was a `fn assert_clone<T: Clone>() {} assert_clone::<Sample>();` compile-time tautology that
  can never fail once the crate compiles — decoration per the assertion-discipline standard for
  this task, not a real test, so I cut it rather than report it as coverage.
- No `TODO`/stray debug output left in the new code; `#[allow(clippy::too_many_arguments)]` on
  `Sampler::with_paths` is the one lint suppression I added, matching the existing pattern
  elsewhere in this codebase (`Sampler::with_paths` already had 5 params before this task; adding
  the 3 required ones pushed it over clippy's default threshold — I didn't try to hide that behind
  a params struct, since every other constructor in this file already uses a flat positional list
  and introducing a struct here alone would be inconsistent, not simpler).
