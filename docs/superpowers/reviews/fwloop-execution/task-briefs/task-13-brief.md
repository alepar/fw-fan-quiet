## Task 13: Persisted state migration

**Bead:** `fw-fanctrl-loop-dsh`

**filesTouched:** `src/state.rs`, `src/control/controller.rs`

`src/control/controller.rs` — **the persist call sites only** (`save_persisted_state`,
`exit_auto_and_persist`, `apply_calib_effects`). Nothing else.

### Global constraints

All of "Global Constraints" above applies. Normative: §3.2, §4, §2.3 (the table's serde
default), §2.4 (`LoopGains`).

### The schema this task owns

    PersistedState {
        lut,
        calibrated_at,
        loop_gains: Option<LoopGains>,
        duty_rpm_table: DutyRpmTable,
        warm_start: BTreeMap<String, f64>,
    }

`model`, `adapt_bias` and `adapt_gain` are **removed**, along with the `state.rs` thermal-model
fit tests. **Old files must still load**: unknown keys ignored, missing new keys defaulted.
`DutyRpmTable`'s serde default (Task 1) is what makes a legacy file come back with the ten
seeded points rather than an empty table.

### Acceptance criteria (verbatim from the bead)

> the `state_v1.json` fixture loads with `lut` intact, `duty_rpm_table` equal to the ten seeded
> points, `warm_start` empty and `loop_gains` `None`; round-trip of the new schema; no
> `thermal_model` import remains in `state.rs`.

### Implementation steps (TDD)

1. **Test first:** loading `tests/fixtures/state_v1.json` (which contains `model` / `adapt_*`)
   succeeds, with `lut` intact, `duty_rpm_table` equal to the ten seeded points, `warm_start`
   empty and `loop_gains` `None`. Then reshape `PersistedState` and confirm the unknown-key
   tolerance.
2. **Test first:** a full round-trip of the new schema — write, read back, compare — including a
   populated `warm_start` and a `Some(LoopGains)`.
3. Delete `model`, `adapt_bias`, `adapt_gain` and the `state.rs` thermal-model fit tests.
4. Update the three persist call sites in `controller.rs` to the new shape. Nothing else in that
   file.
5. **Verify:** search `src/state.rs` for `thermal_model` — zero hits. Record it in your report.
6. Run the test suite and the linter; both clean.

### Deliverable

A migrated `state.json` schema that loads today's file without loss and round-trips the new
fields.

---

