## Task 15: TUI + telemetry surface

**Bead:** `fw-fanctrl-loop-mjv`

**filesTouched:** `src/ui/view.rs`, `src/telemetry.rs`, `src/model.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.5**.

### What this task owns

`src/ui/view.rs`: the header segment `mode A|B|rel · T* · ma · duty -> rpm · budget`; rendering
and **ranking** of the new flags including `READBACK BLIND` (info); calibration progress renders
`CalibProgressLite.phase` as the plain string it already is (no enum, no mapping table).

`src/telemetry.rs`: sample fields `ec_max`, `ec_argmax`, `ec_ma`, `nvme_c`, `fanctrl_speed`,
`fanctrl_active`, `strategy`; decision fields `mode`, `t_star`, `budget_w`, `freeze`; **remove**
`trim_rpm`, `gain`, `model_*`.

### Acceptance criteria (verbatim from the bead)

> view snapshot tests for each mode and each new flag; a telemetry line serialises the new fields
> and omits the removed ones.

### Implementation steps (TDD)

1. **Test first:** a view snapshot for each of `TempLoop`, `RpmLoop` and `Released`, showing the
   header segment in the specified order. Then implement the header.
2. **Test first:** a view snapshot per new `StatusFlag` (`FanctrlLost`, `EcMismatch`,
   `SteepCurve`, `CurveInvalid`, `GpuHot`, `NvmeHot`, `ReadbackBlind`), and a ranking test that
   a warning-severity flag outranks an info one when both are present — specifically that
   `CurveInvalid` outranks `SteepCurve`. Then implement rendering and ranking.
3. **Test first:** calibration progress renders `phase` verbatim as the string it is, for both
   `"lut"` and `"step"`.
4. **Test first:** a serialised telemetry line contains every listed sample and decision field
   and contains **none** of `trim_rpm`, `gain`, `model_*`. Assert the absence explicitly, by
   substring, not only by struct shape. Then implement.
5. Update `src/model.rs` for whatever the view now needs.
6. Run the test suite and the linter; both clean.

### Deliverable

A header segment and flag surface with snapshot coverage for every mode and flag, and a
telemetry line matching §3.5 exactly.

---

