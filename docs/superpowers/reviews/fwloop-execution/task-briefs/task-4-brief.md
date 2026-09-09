## Task 4: Controller status surface

**Bead:** `fw-fanctrl-loop-fo1`

**filesTouched:** `src/control/controller.rs`, `src/ui/view.rs`, `src/telemetry.rs`

`src/ui/view.rs` and `src/telemetry.rs` edits are **compile-only** here: update call sites just
enough that the crate builds. The real rendering and serialisation are Task 15.

### Global constraints

All of "Global Constraints" above applies. Normative: §2.5 (mode names), §2.7, §2.8, §2.9,
§3.2, §3.5.

**Scope fence, stated in the bead and repeated here because it is the likeliest overreach:**

> The repo-wide symbol sweep is **not** this task's — `controller.rs` still holds the adaptation
> tier and its tests at this point, and removing them is fwloop.22's chartered work; this task
> only reshapes the type surface and updates call sites enough to compile.

`fwloop.22` is Task 12. Leave the adaptation tier alone. This task is **types only**.

This task is deliberately **dependency-free**: every new field is a plain type (`f64`, `u8`,
`String`, `Option<...>`, `&'static str`). Do **not** import `Budget`, `Curve`, `Arbiter` or any
other epic type into these definitions.

### The type surface this task owns

- `LoopMode { TempLoop, RpmLoop, Released }`
- `ControlStatus`: **drops** `trim_rpm`, `gain`; **adds** `mode`, `t_star_c: Option<f64>`,
  `ec_ma_c: Option<f64>`, `ec_argmax: Option<String>`, `duty_cmd: Option<u8>`,
  `snapped_rpm: f64`, `strategy: Option<String>`, `budget_w: f64`.
- `CalibProgressLite.phase` **stays a plain `String`**.
- `StatusFlag`: **drops** `ModelDistrust`; **adds** `FanctrlLost`, `EcMismatch`,
  `SteepCurve` (info), `CurveInvalid` (**warning** — a permanent loss of Mode A must not share
  the informational `SteepCurve` severity), `GpuHot`, `NvmeHot`, `ReadbackBlind`, each with a
  severity.
- `Effect::ModelSnapshot` **removed**.
- `Effect::AutoAllocated` **gains** `mode`, `error: f64`, `budget_w`,
  `freeze: Option<&'static str>`.

Existing code populates the new fields with defaults so the crate compiles.

### Acceptance criteria (verbatim from the bead)

> crate compiles and tests pass with the new type surface; `flag_severity` covers every new
> flag; the new `ControlStatus`/`Effect` definitions carry no `trim_rpm`/`gain`/`ModelSnapshot`
> field or variant.

### Implementation steps (TDD)

1. **Test first:** a test that constructs `ControlStatus` with the new field set. The old
   `trim_rpm`/`gain` call sites failing to compile is the signal that the fields are gone. Add
   `LoopMode` and the new fields.
2. **Test first:** `flag_severity` returns a severity for **every** `StatusFlag` variant — write
   it against a hand-maintained `const ALL: [StatusFlag; N]` and an exhaustive match, so adding
   a variant later fails to compile rather than silently defaulting. Assert specifically that
   `CurveInvalid` is **warning** and `SteepCurve` is **info**, and that the two differ. Then add
   the flags and their severities and delete `ModelDistrust`.
3. **Test first:** an `Effect::AutoAllocated` value carries `mode`, `error`, `budget_w` and
   `freeze`; `Effect` has no `ModelSnapshot` variant. Then change `Effect`.
4. Update `src/ui/view.rs` and `src/telemetry.rs` **minimally** — enough to compile. Where a
   removed field was rendered or serialised, drop that item; where a new field is needed for the
   match to be exhaustive, render/serialise it in the plainest possible way. Do not design the
   header segment here (Task 15).
5. Populate the new `ControlStatus` fields at existing controller call sites with defaults
   (`None` / `0.0` / `LoopMode::Released` as appropriate) so the build passes.
6. Run the test suite and the linter; both clean. Confirm the adaptation tier and its tests are
   **still present and still passing** — that is the evidence you stayed in scope.

### Deliverable

The crate compiles and the whole existing suite passes on the new type surface, with the
adaptation tier untouched.

---

