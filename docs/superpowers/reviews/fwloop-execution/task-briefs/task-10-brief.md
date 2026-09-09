## Task 10: FOPDT fit + IMC gain derivation

**Bead:** `fw-fanctrl-loop-4aj`

**filesTouched:** `src/calib/fopdt.rs`, `src/calib/mod.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§3.3** and **§2.4**.

### The theta trap — the single most important thing in this task

**Theta is taken as fitted.** The §3.3 fit runs on the **already-filtered** EC average, so its
`theta_hat` already contains the boxcar; adding `ma_interval/2` again would count the filter
twice and roughly **halve** `Kc`. The `+ ma_interval/2` substitution belongs **only** to the
raw-domain defaults of §2.4. If you write `theta_hat + ma_interval/2` anywhere in this file, the
task is wrong.

### What this task owns

`Fopdt { k, tau, theta }`, `fit_fopdt(&[(t, y)], step_w) -> Option<Fopdt>` by least squares on a
step response, and
`derive_gains(ec: &Fopdt, rpm: &Fopdt, defaults: &LoopGains) -> Option<LoopGains>` with
`lambda = max(90, 3*theta_hat)`, `Kc = tau/(K*(lambda + theta_hat))`, `Ti = tau` per signal, and
`fitted_at` left `None` for the caller to stamp.

**Rejection rules, all yielding `None`:** `K` at or below 0; `tau < 5 s`; a response magnitude
under 3 °C (EC) or 150 RPM (fan); a derived `Kc` outside `[0.25, 4]x` the corresponding default.
The magnitude and ratio bounds are what stop `Kc = tau/(K*(lambda+theta))` running away as `K`
approaches zero from above.

**Pure functions, no I/O.** No file reads, no clock reads.

### Acceptance criteria (verbatim from the bead)

> recovers K/tau/theta within 10 % on a synthetic noisy FOPDT step; derived gains match the IMC
> formulas **with the fitted `theta_hat` used as-is, no `+ ma_interval/2`** (the boxcar is
> already in `theta_hat`; adding it again halves `Kc`); each rejection rule fires on its own
> crafted input — `K <= 0`, `tau < 5 s`, a sub-threshold response, and a duty-pinned step whose
> tiny positive `K_rpm` would otherwise derive a `Kc` orders of magnitude above the default.

### Implementation steps (TDD)

1. Create `src/calib/fopdt.rs` and declare it in `src/calib/mod.rs`.
2. **Test first:** generate a synthetic FOPDT step (known K, tau, theta) with seeded noise;
   `fit_fopdt` recovers all three within 10 %. Then implement the least-squares fit.
3. **Test first — the theta assertion, written explicitly:** given a `Fopdt` with a known
   `theta_hat`, `derive_gains` produces `Kc = tau/(K*(max(90, 3*theta_hat) + theta_hat))`. Add a
   second assertion that the value is **not** what the `+ ma_interval/2` variant would give (it
   would be roughly half). Then implement `derive_gains`.
4. **Test first:** `Ti = tau` per signal, and `fitted_at` comes back `None`.
5. **Test first, one per rejection rule:** a non-positive `K`; `tau < 5 s`; an EC response under
   3 °C; a fan response under 150 RPM; and a **duty-pinned step whose tiny positive `K_rpm`**
   would otherwise derive a `Kc` orders of magnitude above the default, caught by the
   `[0.25, 4]x` band. Each returns `None`. Then implement each rule.
6. Run the test suite and the linter; both clean.

### Deliverable

Two pure functions with a complete rejection-rule test matrix. No calibration wiring.

---

