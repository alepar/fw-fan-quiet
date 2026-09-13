## Task 23: README + docs

**Bead:** `fw-fanctrl-loop-7ij`

**filesTouched:** `README.md`, `docs/research/03-control.md`,
`docs/superpowers/specs/INDEX.md`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.5**, §2.8, and the config keys as
they actually exist in `src/config.rs` at the time you run.

**Read the code, not this plan, for the names.** Config key names and defaults come from
`src/config.rs`; the flags table comes from the `StatusFlag` enum; the calibration flow
(durations, skip reasons) comes from `src/calib/step.rs`. If any of those disagrees with what
this section says, the code wins and you note it.

### What to write

- **Calibration walkthrough** — rewritten for the LUT sweep + step test, the burner, and
  `NeedsLoad`.
- **Auto mode** — the cascade, the modes, and a flags table.
- **Safety model** — fw-fanctrl owns the fans; the socket is read-only **by construction**;
  guards; read-back including `READBACK BLIND`; and a line saying **plainly that the NVMe
  reading is reported and not acted on**, with the measurement behind that (§2.8: near-maximum
  airflow did not hold the drive while the SoC cooled).
- **Configuration table** — drop `online_rls`; add `fanctrl_socket`, `gpu_hot_c`, `nvme_hot_c`.
- `docs/research/03-control.md` — a pointer note.
- `docs/superpowers/specs/INDEX.md` — status to implemented.

### Acceptance criteria (verbatim from the bead)

> every config key in `config.rs` appears in the README table and vice versa; every `StatusFlag`
> variant appears in the flags table; no README mention of matrix/model/trim/RLS remains.

### Implementation steps

1. Enumerate the actual `Config` keys from `src/config.rs` and the actual `StatusFlag` variants
   from `src/control/controller.rs`. Put both lists in your report.
2. Rewrite the Calibration walkthrough, Auto mode, Safety model and Configuration sections.
3. **Verify both directions of the config table:** every key in `config.rs` appears in the
   README, and every row in the README exists in `config.rs`. Do the same for the flags table.
   Record both checks in your report.
4. Search `README.md` for `matrix`, `model`, `trim`, `RLS` — zero mentions of the deleted
   subsystems remain (Task 21 deliberately left the README to you). Record the output.
5. Update `docs/research/03-control.md` and `docs/superpowers/specs/INDEX.md`.

### Deliverable

A README whose config and flags tables are provably in sync with the code, and no residue of the
deleted subsystems.

---

