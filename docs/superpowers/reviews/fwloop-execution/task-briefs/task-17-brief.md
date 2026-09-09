## Task 17: fw-fanctrl emulator + chained plant

**Bead:** `fw-fanctrl-loop-51b`

**filesTouched:** `src/test_support/plant.rs`

This replaces the empty `cfg(test)` stub Task 2 created. It is one file, and it is the whole
task.

### Global constraints

All of "Global Constraints" above applies. Normative: **§2.5**, §5, and **§Facts** — especially
the EC-autofan staircase and its stated limitation.

**No new crate dependency.** The RNG is a hand-rolled seeded xorshift.

### The measured EC staircase, and the limit on what it may be used for

Measured medians: **4096 RPM at 61-62 °C, 4520 at 63 °C, 4658 at 64 °C, then flat at ~4748 RPM
across 67-73 °C.** The **67-73 plateau is well sampled (n = 102 at 71 °C) and load-bearing** for
the no-authority result. The **steep 61-64 segment is thin (n = 4 at 64 °C)** and its rising
branch and hysteresis width are **not** established (§Facts limitation). Model the plateau as
measured; treat the steep segment as **approximate**, and do not derive any claim about the EC
from it.

### The two upstream quirks that must be reproduced

These are what make the replica's average diverge from the socket's in ways the instantaneous
value hides — they are the whole reason `EC MISMATCH` exists:

1. **No history append while paused.**
2. **A hardcoded 50 °C injected on a scripted sensor-read failure.**

### The socket-death regime

The `Freshness` injection hook for socket death **also switches the plant into EC-autofan
mode** — a stopped fw-fanctrl leaves the fans on the EC curve via its unit's
`ExecStopPost --autofanctrl`, so `absent` and `active: false` are **one plant regime** (§2.5).

### What this task owns

`FanctrlEmulator` (1 s tick; boxcar N non-zero **with the off-by-one**; `eff = min(MA, cur)`;
`Curve`; `int()` truncation; switchable `active`; `edit_curve_in_place(points)` under the
unchanged strategy name; `view(now) -> FanctrlView` with curve, `ma_temperature`, `ma_interval`,
`active`, **both stamps**, plus the `Freshness` injection hook above).

`ThermalPlant` (watts -> controllable EC °C, tau 35, theta 20, K 0.8, plus separately labelled
`ambient` / `charger` channels and scriptable `gpu_*` channels emitted as an `EcReading`).

`FanPlant` (duty -> RPM via **its own table**, seeded from the same points with a **configurable
per-duty offset**; a one-sided momentum kick on positive slew; noise of +/-90 RPM from the seeded
xorshift; **plus an EC-autofan mode** used whenever the emulator is `active: false`, driving RPM
straight from EC temperature on the measured staircase).

Scriptable `gpu_temp_c` / `nvme_temp_c` (both `Option`), `cpu_util` / `gpu_util` / `gpu_sm_mhz`
tracks, **a demand model that decides how much of the commanded cap is actually drawn** (so
`cpu_pkg_w` / `gpu_w` on the emitted `Sample` are a **measured draw that can sit well below the
cap** — without this the plant cannot exercise the demand-starved wind-up at all), a scriptable
`resumed` edge, and `ChainedPlant` composing them into a full `Sample` per tick with
`fanctrl_view_changed` set **once per new `print all` view**.

### Acceptance criteria (verbatim from the bead)

> emulator reproduces the verified truncation case (T_eff 51.8 -> 21 on cool16) and the
> off-by-one; a scripted temperature drop drives `eff` from the `current` branch; a scripted
> socket death yields `Absent` **and puts the fan plant on the measured EC staircase**; an
> in-place curve edit yields new points under the same name in the emitted view;
> `fanctrl_view_changed` is set exactly once per new `print all` view; an open-loop step on the
> chained plant shows a 26-30 s watts->RPM lag; the plant table offset shifts steady RPM by the
> configured amount; scripted CPU-heavy vs GPU-heavy utilisation shifts the demand split; a
> low-demand script emits a measured draw well below the commanded cap; a paused emulator stops
> appending history, a scripted read failure injects 50 °C, and an `active: false` emulator
> drives RPM from the measured EC staircase instead of the commanded duty.

### Implementation steps (TDD)

1. **Test first:** the emulator reproduces `T_eff 51.8 -> 21` on cool16 (truncation) and the
   boxcar off-by-one, asserted as literal expected values. Then implement `FanctrlEmulator`'s
   tick, boxcar and `eff = min(MA, cur)`.
2. **Test first:** a scripted temperature **drop** drives `eff` from the `current` branch (not
   the MA branch). Then implement.
3. **Test first:** a paused emulator **stops appending history**; a scripted sensor-read failure
   injects **50 °C**. Then implement the two quirks.
4. **Test first:** `edit_curve_in_place(points)` yields the new points under the **same strategy
   name** in the emitted view, and bumps `all_observed_at`. Then implement.
5. **Test first:** `fanctrl_view_changed` is set exactly once per new `print all` view. Then
   implement the view emission.
6. **Test first:** an open-loop watts step on `ChainedPlant` shows a **26-30 s** watts-to-RPM
   lag. Then implement `ThermalPlant` (tau 35, theta 20, K 0.8) and the chaining.
7. **Test first:** the plant table's configurable per-duty offset shifts steady RPM by exactly
   that amount; the positive-slew momentum kick is one-sided; the noise is +/-90 RPM and
   reproducible from the seed. Then implement `FanPlant`.
8. **Test first:** a scripted socket death yields `Absent` **and** puts the fan plant on the
   measured EC staircase; an `active: false` emulator does the same. Assert the plateau values
   (4748 across 67-73 °C) exactly, and the steep segment only loosely. Then implement the
   EC-autofan mode.
9. **Test first:** scripted CPU-heavy versus GPU-heavy utilisation shifts the demand split; a
   low-demand script emits a **measured draw well below the commanded cap**. Then implement the
   demand model.
10. **Test first:** a scripted `resumed` edge appears on the emitted `Sample`.
11. Run the test suite and the linter; both clean.

### Deliverable

A deterministic, seeded `ChainedPlant` that emits a full `Sample` per tick and can reproduce
every regime Task 22's acceptance runs need — including the socket-dead / `active: false`
EC-autofan regime and demand starvation.

---

