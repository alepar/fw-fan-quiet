## Task 8: EC replica + NVMe + AC sensors

**Bead:** `fw-fanctrl-loop-52c`

**filesTouched:** `src/sensors/ec.rs`, `src/sensors/hwmon.rs`, `src/sensors/mod.rs`

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.2**, §3.4, §Facts (the idle/load/
dGPU-on measurements).

### The measured fact this task must not contradict

With the dGPU powered at 18.9 W the cros_ec `gpu_amb`, `gpu_vr` and `gpu_vram` sensors still
read -150 and `gpu_temp@40` still returns ENODATA. They **never** report on this machine. The
replica's rule is nonetheless "every positive reading joins the max" — it matches fw-fanctrl's
own regex and costs nothing if a future firmware makes them live. Implement the general rule;
assert the measured fact.

### What this task owns

`EcReading` / `EcLabel` (controllable: `apu`, `cpu`, `gpu_*` if one ever reports; uncontrollable:
`ambient`, `charger`); every positive reading takes part in the max; drop readings at or below
zero, unreadable or unparsable `_input`; round to integer; max + argmax with ties broken by
**sysfs order**. `EcAverage`: boxcar of N non-zero samples **with the off-by-one**,
`set_interval(n)` capped at 100 and **retaining** existing samples, `reseed(value)` as the
**only** clearing operation, plus `is_seeded()` / `sample_count()` so the controller can refuse
to use an underfilled mean as a full one. `sensors/hwmon.rs` gains
`nvme_composite_c() -> Option<f64>` and `on_ac()`.

### Acceptance criteria (verbatim from the bead)

> on `cros_ec_idle` 47.85 -> 48, argmax `ambient`, the -150 and the input-less sensor dropped
> without invalidating the reading; on `cros_ec_load` 74.85 -> 75 with argmax `cpu@4c`, matching
> that fixture's paired socket `temperature` of 75.0; on `cros_ec_dgpu_on` the `gpu_*` sensors
> are still -150/absent and are dropped, so the max comes from `cpu`/`ambient` (measured: they
> never report even with the dGPU powered); a synthetic positive `gpu_*` reading would join the
> max and classify controllable; boxcar returns mean of n-N..n-1; `set_interval` grows/shrinks
> without clearing, `reseed` replaces the contents; `is_seeded` is false until seeded or N
> samples deep; nvme/ac readers on fixtures, nvme `None` when the chip is absent.

### Implementation steps (TDD)

1. Create `src/sensors/ec.rs` and declare it in `src/sensors/mod.rs`.
2. **Test first:** reading `hwmon/cros_ec_idle` yields max 48 (47.85 rounded) with argmax
   `ambient`; the three -150 sensors and the label-without-`_input` sensor are dropped and the
   reading stays valid. Then implement `EcReading` parsing.
3. **Test first:** reading `hwmon/cros_ec_load` yields 75 with argmax `cpu@4c`, and that equals
   the paired `print_all_load.json` `temperature` of 75.0. Then confirm the rounding rule.
4. **Test first:** reading `hwmon/cros_ec_dgpu_on` still drops every `gpu_*` sensor, so the max
   comes from `cpu`/`ambient`.
5. **Test first:** a **synthetic** positive `gpu_*` reading joins the max and classifies as
   **controllable**; `ambient` and `charger` classify uncontrollable. Then implement `EcLabel`
   and its controllability rule.
6. **Test first:** ties in the max are broken by sysfs order. Then implement.
7. **Test first:** `EcAverage` returns the mean of samples `n-N..n-1` — reproduce the off-by-one
   exactly, and write the test as a literal expected value so a later "fix" cannot silently
   change it. Then implement the boxcar over non-zero samples.
8. **Test first:** `set_interval` grows and shrinks **without clearing** retained samples and
   caps at 100; `reseed(v)` replaces the contents and is the only clearing operation. Then
   implement.
9. **Test first:** `is_seeded()` is false until `reseed` is called or N samples have arrived;
   `sample_count()` reports the retained count. Then implement.
10. **Test first:** `nvme_composite_c()` reads the `hwmon/nvme` fixture and returns `None` when
    the chip is absent; `on_ac()` reads `power_supply/ACAD/online`. Then implement in
    `src/sensors/hwmon.rs`.
11. Run the test suite and the linter; both clean.

### Deliverable

An EC replica whose max/argmax matches the socket on both captured fixtures, plus the boxcar
with its retain-on-resize contract, all unit-tested.

---

