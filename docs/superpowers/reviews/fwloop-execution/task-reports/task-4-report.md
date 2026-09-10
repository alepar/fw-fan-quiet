# Task 4 report: Controller status surface

## What I implemented

Reshaped the type surface in `src/control/controller.rs` per the brief, and
updated `src/ui/view.rs` and `src/telemetry.rs`-adjacent call sites (telemetry
itself needed no changes; `src/model.rs` needed a small compile-only fix not
listed in the brief's `filesTouched` — see Ruling 1) minimally to compile.

- Added `LoopMode { TempLoop, RpmLoop, #[default] Released }` — a new,
  dependency-free enum (no `Budget`/`Curve`/`Arbiter` import).
- Added `Severity { Critical, Warning, Info }` and a `pub fn
  flag_severity(StatusFlag) -> Severity` with a hand-maintained
  `const ALL_STATUS_FLAGS` and its own tests (brief step 2). `flag_severity`
  lives in `controller.rs` (the module that owns `StatusFlag`), not
  `view.rs` — see Ruling 3.
- `StatusFlag`: dropped `ModelDistrust`; added `FanctrlLost`, `EcMismatch`,
  `SteepCurve` (info), `CurveInvalid` (warning), `GpuHot`, `NvmeHot`,
  `ReadbackBlind`, each with an `as_str()` arm and a `flag_severity` arm.
  The 7 new variants carry `#[allow(dead_code)]` (no call site raises them
  yet — the guards/arbiter that would are later tasks) matching this
  codebase's existing convention (`ring.rs`, `lut.rs`, `trust.rs`, etc.).
- `ControlStatus`: dropped `trim_rpm`, `gain`; added `loop_mode: LoopMode`,
  `t_star_c: Option<f64>`, `ec_ma_c: Option<f64>`, `ec_argmax: Option<String>`,
  `duty_cmd: Option<u8>`, `snapped_rpm: f64`, `strategy: Option<String>`,
  `budget_w: f64` — see Ruling 2 on the `loop_mode` naming.
- `Effect::ModelSnapshot` removed. `Effect::AutoAllocated` gained `mode:
  LoopMode`, `error: f64`, `budget_w: f64`, `freeze: Option<&'static str>`.
- Updated every production call site that populated the removed fields or
  matched the removed variant, all with Task-4-scoped defaults
  (`LoopMode::default()`, `0.0`, `None`) — see the diff for the full list;
  the notable ones are documented as Rulings below.
- `src/ui/view.rs`: dropped the trim/gain header rendering block (the
  fields it read no longer exist); extended `flag_span`'s exhaustive match
  with plain placeholder spans for the 7 new flags (Task 15 designs the
  real header); restored a **separate** `render_priority` (renamed from the
  old `flag_severity`) as the UI's own full, strict per-flag sort order —
  see Ruling 4.
- `src/model.rs`: removed two now-nonexistent `trim_rpm: 0.0,` field
  initializers in test-only `ControlStatus` literals (compile-only fix; see
  Ruling 1).

## TDD evidence

Followed the brief's four "test first" steps in order (RED confirmed by the
compiler rejecting the field/variant before I added it, since these are
type-level changes rather than behavioral ones):

1. `control_status_carries_the_new_loop_fields` /
   `control_status_default_has_no_loop_state_yet` — construct/read every new
   `ControlStatus` field; RED was "no field `loop_mode`/`t_star_c`/..." until
   the struct was extended.
2. `flag_severity_covers_every_flag` (see the "assertion discipline" note
   below) and `curve_invalid_is_warning_steep_curve_is_info_and_they_differ`
   — RED was "cannot find function/type `flag_severity`/`Severity`" until
   both were added; the CurveInvalid/SteepCurve assertions are real (they
   fail if I'd assigned the same tier to both, which I initially had to get
   right deliberately).
3. `auto_allocated_carries_the_new_arbiter_fields` — destructures
   `Effect::AutoAllocated` by name (`let-else`, no `..`), so a missing/typo'd
   field fails to compile; RED was "no field `mode`/`error`/`budget_w`/
   `freeze`" until `Effect` was extended.
4/5/6. Ran `cargo build`, `cargo check --tests`, `cargo test`, `cargo clippy
   --all-targets`, `cargo fmt --check` repeatedly while fixing the resulting
   ~120 compile errors (enumerated below) until all were green; the
   adaptation tier and its full test suite are unmodified in *behavior*,
   only in *how they read state that moved off `ControlStatus`* (see
   Ruling 5).

**Commands run, final state:**
```
cargo test            -> test result: ok. 426 passed; 0 failed; 2 ignored
cargo clippy --all-targets -> Finished ... (0 warnings)
cargo fmt --check     -> (no diff)
```
The 2 ignored tests (`nvml_lock_smoke`, `nvml_smoke_reads_real_gpu`) are
pre-existing hardware-only tests, unaffected by this task (confirmed via
`git stash` against the base commit).
`cargo test` also passed at exactly 426/426 on the base commit
(`15b6dbf`) — I removed 5 tests whose entire premise (trim/gain header
rendering, ModelDistrust rendering) no longer exists and added 5 new ones
for the Task 4 type surface, net zero, which is a coincidence I cross-
checked rather than relied on.

### Assertion discipline note

`flag_severity_covers_every_flag` cannot actually fail: `flag_severity`'s
match has no wildcard arm, so the real "every variant is classified"
guarantee is enforced by the **compiler** (a variant added to `StatusFlag`
without a matching arm in `flag_severity` fails the build), not by this
test. I kept the test (and its `const ALL_STATUS_FLAGS`) as a named,
documented anchor for the brief's "hand-maintained const ALL" ask, but it is
decoration relative to the compile-time guarantee — recording that here per
the assertion-discipline instruction rather than presenting it as load-
bearing. The two assertions inside
`curve_invalid_is_warning_steep_curve_is_info_and_they_differ` (and
everything in `auto_allocated_carries_the_new_arbiter_fields` and the two
`ControlStatus` field tests) are real: each names a concrete `Severity` or
field value the code could have produced (e.g. `Severity::Info` for
`CurveInvalid`, or a stale field from `..ControlStatus::default()` clobbering
an explicit one) that would fail the assertion.

## Files changed

- `src/control/controller.rs` (types, call sites, ~10 new/rewritten tests,
  ~90 test call-site fixes for the removed `trim_rpm`/`gain`/`ModelDistrust`)
- `src/ui/view.rs` (header rendering, `flag_span`, `render_priority`,
  5 tests deleted, 2 tests trimmed)
- `src/model.rs` (2-line compile-only fix)

## Self-review findings

- Caught and fixed during self-review (before this report): the initial
  3-tier `Severity` alone broke `ui::view::tests::
  emergency_stays_visible_at_120_cols_with_many_flags` because
  `ThermalEmergency`/`SensorLost`/`TargetUnreachable` all became tied at
  `Severity::Critical`, and a stable sort no longer forced
  `ThermalEmergency` first. Fixed via Ruling 4 (separate `render_priority`).
- Caught and fixed: a blind pattern-substitution turned four post-Auto-exit
  assertions (`ctl.status().trim_rpm` after `SetAuto(false)`/`ReleaseAll`)
  into `ctl.auto.as_ref().unwrap()...` panics, since `AutoState` (including
  the KF) drops whole on exit. Fixed by asserting `ctl.auto.is_none()`
  instead, which is the more direct expression of what those tests were
  actually checking ("the visible state resets on exit").
- Caught and fixed: `drive_to_distrust`'s inner assertion
  (`has_flagged(&effects, "model_distrust", true)`) tested telemetry
  behavior that no longer exists once `ModelDistrust` has no `StatusFlag`
  to transition. Removed that specific assertion (documented in the
  function's doc comment) while keeping the surrounding test intact, since
  it still exercises the real adaptation-tier behavior (the trust verdict
  itself).

## Concerns

None outstanding. The `#[allow(dead_code)]` attributes on the 7 new
`StatusFlag` variants, `LoopMode::{TempLoop,RpmLoop}`, `Severity`, and
`flag_severity` are expected and scope-bounded: nothing in *production*
code constructs/calls them yet (only their own tests do) because wiring
them up is Task 12 (adaptation-tier removal) and Task 15 (header design),
not this task.

---

## Rulings I made

**Ruling 1 — `src/model.rs` is in scope for a compile-only fix despite not
being in the brief's `filesTouched`.**
`model.rs` (line ~552, `status_syncs_local_setpoints`) constructs a
`ControlStatus { trim_rpm: 0.0, ... }` literal in test code. The brief's
acceptance criterion ("crate compiles and tests pass") is the overriding
authority; `filesTouched` is the brief's own prediction of scope, and this
is a 2-line, zero-behavior-change consequence of the mandated field
removal, not a design decision. Cost if wrong: essentially none — it is a
deletion of a now-nonexistent field name from a test fixture.

**Ruling 2 — the new `LoopMode` field is named `loop_mode`, not `mode`,
despite the spec text literally saying "gains `mode: LoopMode`".**
`ControlStatus` already has a field named `mode: Mode` (the controller's
own Monitor/Manual/Calibrating/Auto state), which `src/model.rs` uses
extensively for UI state gating (`self.status.mode == Mode::Calibrating`,
etc.) — `model.rs` is *not* in the brief's `filesTouched`, so nothing there
may break. A field cannot be named `mode` twice, and retyping the existing
`mode` field to `LoopMode` would break every one of those `model.rs` call
sites (and is a real design change beyond "types only"). I read the spec's
"gains `mode: LoopMode`" as shorthand/imprecision, not a literal field
rename, and named the new field `loop_mode` instead — consistent with the
doc comment I added distinguishing "which top-level state the controller is
in" (`Mode`) from "which loop is regulating within Auto" (`LoopMode`). Cost
if wrong: a mechanical rename in Task 15 (or wherever the real header design
lands) if the intended name really was `mode`; nothing downstream in this
tree depends on the exact name yet.

**Ruling 3 — `flag_severity`/`Severity` live in `controller.rs`, not
`view.rs`, and `view.rs` keeps a separate, unrelated `render_priority`
function for its own sort order.**
The brief's TDD steps 1–3 (which include the `flag_severity`/`const ALL`
step) are all framed as `controller.rs`'s own type-surface work, and the
acceptance criterion says "`flag_severity` covers every new flag" without
tying it to `view.rs`'s pre-existing private helper of the same name. I
treated the two as different concerns: `Severity` is a `StatusFlag`
*classification* (Critical/Warning/Info) that this task's type surface
owns; the UI's render order (which flag physically appears first in the
header, needed to survive terminal-width clipping) is a full strict
per-flag ranking Task 15 explicitly still owns ("Do not design the header
segment here"). Using the 3-tier `Severity` alone as the UI sort key
broke `emergency_stays_visible_at_120_cols_with_many_flags` (three flags
tied at `Critical`, so a stable sort stopped guaranteeing
`ThermalEmergency` renders first) — confirming these needed to stay
separate. Cost if wrong: `view.rs` carries two flag-ordering functions
instead of one until Task 15 reconciles them; no test or behavior is
incorrect either way.

**Ruling 4 — `render_priority` (view.rs) is a full strict ranking, not
derived from `Severity`.**
Direct consequence of Ruling 3's fix. It reuses the *values* the old
`flag_severity(u8)` assigned to the 6 pre-existing flags (so no rendering
behavior changes for them) and appends the 7 new flags afterward in an
arbitrary-but-documented order, since none of them render with real
styling yet (`flag_span`'s new arms are plain `Span::raw` placeholders).
Cost if wrong: Task 15 reorders/restyles this table anyway as its own
chartered work.

**Ruling 5 — the adaptation tier's ~90 test call sites that read
`ctl.status().trim_rpm` / `.gain` / `StatusFlag::ModelDistrust` now read
`ctl.auto.as_ref().unwrap().kf.bias()` / `.kf.gain()` / `.distrusted`
directly, since `ControlStatus` no longer carries that state.**
This resolves what would otherwise be a direct contradiction in the brief:
it mandates dropping `trim_rpm`/`gain` from `ControlStatus` *and* mandates
that "the adaptation tier and its tests are still present and still
passing." The tests already had precedent for this exact pattern before my
change (e.g. `assert_eq!(auto.kf.bias(), -120.0)` a few lines from tests
that also used `ctl.status().trim_rpm` for the same value) — `auto:
Option<AutoState>` is a private field on `Controller` and the test module
is a child module, so it has always had direct access. I applied this
mechanically (`perl` regex substitution, then manually fixed the ~6 spots
where the substitution produced a panic because `ctl.auto` is `None`
post-Auto-exit — those became `assert!(ctl.auto.is_none())`, which is a
more direct statement of what those specific tests were actually checking).
The same substitution replaced `ctl.status().flags.contains(&StatusFlag::
ModelDistrust)` with `ctl.auto.as_ref().unwrap().distrusted`. This also
required two production-code changes to keep telemetry behavior identical
(not just test-compiling): `apply_effects`'s `Record::Decision` closure now
reads `controller.auto.as_ref().map(|a| a.kf.bias()/.kf.gain())` instead of
`status.trim_rpm`/`status.gain` (exactly equivalent — `auto` is documented
`Some` exactly while `Mode::Auto`, the same gating the old field-based
`.then_some()` provided), and `on_sample`'s tail gained an `else if let
Some(cause) = cause { effects.push(Effect::Noted { cause }) }` fallback
(mirroring the pattern `on_calib_sample` already used) — without it, a
KF-only sample with no other status change would silently emit **no**
telemetry Decision line at all, since `self.status != before` used to be
made true by the trim/gain mirror and no longer is. This is a direct,
minimal, unavoidable consequence of the type change, not a repurposed
`cause` for some other reason — I verified it via the pre-existing
`kf_decisions_reach_telemetry_with_offset` test, which asserts a specific
`"auto:kf"` telemetry line lands at a specific sample and would otherwise
have silently vanished. Cost if wrong: none observed — the full existing
adaptation-tier suite (including this exact telemetry test) passes
unmodified in its assertions, only in how it reads the now-relocated state.
