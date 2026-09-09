## Task 5: Guards (dGPU, NVMe) + config keys

**Bead:** `fw-fanctrl-loop-mm2`

**filesTouched:** `src/control/guards.rs`, `src/config.rs`, `src/control/mod.rs`

`src/control/mod.rs` is a barrel: add exactly `pub mod guards;`.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.8**, plus §Facts's two measured
paragraphs (the card's 87 °C target specification; the NVMe airflow probe).

### The load-bearing negative result

**There is no `effective_target`.** The NVMe guard is **reporting-only** (§2.8, measured): the
airflow probe showed near-maximum airflow did not hold the drive while the SoC cooled, so
raising the fan target for a hot SSD buys nothing. Therefore **nothing modifies the user's RPM
target**, and `nvme_hot` only drives a flag. If you find yourself adding a function that returns
an adjusted target, stop — that is the deleted design.

`gpu_hot_c` defaults to **90** (exit 85), derived from the card's own 87 °C target
specification: any threshold below 87 fires during normal gaming.

### API this task owns

`Guards::step(gpu_temp_c: Option<f64>, nvme_temp_c: Option<f64>) -> GuardState { gpu_hot,
nvme_hot }` with enter thresholds, **exit = enter - 5**, and **`None` means that guard is
inactive (and it exits any hot state)**; `gpu_share_override(current_gpu_w, gpu_floor_w)`.
Config keys `gpu_hot_c` (90) and `nvme_hot_c` (80). The `online_rls` legacy note/test in
`config.rs` is generalised to "**unknown keys are ignored**".

### Acceptance criteria (verbatim from the bead)

> hysteresis enters at threshold, exits 5 below; a `None` input deactivates the guard and clears
> a hot state; the GPU share override is computed per spec §2.8; a hot NVMe sets `nvme_hot` and
> changes nothing else (no target, no budget); config round-trips with defaults; a config
> containing `online_rls`, a stale `nvme_boost_rpm` and an arbitrary unknown key still loads.

### Implementation steps (TDD)

1. Create `src/control/guards.rs`, add `pub mod guards;` to `src/control/mod.rs`.
2. **Test first:** `Guards::step(Some(90.0), None)` enters `gpu_hot`; it stays hot at 86.0 and
   clears at 85.0 (exit = enter - 5); symmetric table for `nvme_hot` at 80/75. Then implement
   the hysteresis.
3. **Test first:** a `None` GPU reading deactivates the guard **and clears an already-hot
   state**; same for NVMe. Then implement.
4. **Test first:** `gpu_share_override(current_gpu_w, gpu_floor_w)` returns the §2.8 value
   across the cases §2.8 enumerates. Then implement.
5. **Test first — the negative assertion:** a step whose only hot guard is `nvme_hot` returns a
   `GuardState` that carries **no** target or budget adjustment, and the module exposes no
   `effective_target` symbol. Assert `GuardState`'s field set explicitly.
6. **Test first:** `Config` round-trips with `gpu_hot_c` defaulting to 90 and `nvme_hot_c` to
   80. Then add the keys.
7. **Test first:** a config containing `online_rls`, a stale `nvme_boost_rpm` and an arbitrary
   unknown key loads successfully. Generalise the existing `online_rls` test and its comment to
   the "unknown keys are ignored" rule. Then implement (any `deny_unknown_fields` on this struct
   must be absent).
8. Run the test suite and the linter; both clean.

### Deliverable

`src/control/guards.rs` unit-tested standalone, plus the two config keys and the unknown-key
tolerance rule. No controller wiring.

---

