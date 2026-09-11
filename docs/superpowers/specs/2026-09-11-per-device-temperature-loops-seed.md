# Seed: replace the budget split with per-device temperature loops

Status: brainstorming seed for a `super-auto` run (2026-09-11). Base branch:
`epic-fw-fanctrl-loop-6ma-integration` (tip `a8ae29b`). Not a spec — the run's design phase
produces that.

## Goal

The fan target is met by regulating each device's own EC temperature toward one shared
setpoint, with no scalar power budget and no CPU/GPU split. Engaging Auto under a running game is
bumpless; a device the load does not stress runs uncapped; a device pinned at its cap is never
held back by the other device's unused allocation.

## Why (field evidence, 2026-09-10/11)

`run-1789067819` and `run-1789139478`: the scalar budget + demand split + any-axis demand-limited
halt (§2.4/§2.5 of `2026-09-07-fw-fanctrl-loop-design.md`) failed three ways in two sessions:

- Auto entry split the seeded budget by CPU **utilisation** (7 % on a few-thread game drawing
  34 W) and cut the CPU in half in one second; GPU utilisation fell with it; fans dropped
  3160 → 2400 RPM below a 3500 target that had been nearly met with no caps at all.
- The GPU "floor watts" is the LUT clamped at its lowest swept clock (49.3 W at 1200 MHz under
  full load). A lightly loaded card draws 38 W at any clock, so the floor exceeded the card's
  whole appetite and 11–16 W of budget was permanently reserved for a hunger that did not exist.
- The halt is `any(axis demand-limited)` on the **total**: the overfed GPU under-drew its share,
  the halt vetoed every upward PI step, and the pinned CPU (demand 1.0) could never receive
  watts the PI wanted to give. The PI was alive and blocked for 90 s.

Two earlier defects on the same branch (budget deadlocked at its lower bound via the LUT-clamped
floor; GPU PI integrator frozen at the floor) were the same abstraction failing from other
angles. The split is an open-loop guess between two signals that are directly measurable.

## The design, settled so far

**Measured fact the design rests on.** The EC fan curve (and fw-fanctrl's) is driven by the
**max** over the readings `framework_tool --thermal` prints, which are the eight `cros_ec` hwmon
sensors the daemon already reads:

| framework_tool | hwmon label | group |
|---|---|---|
| F75303_Local | `ambient_f75303@4d` | uncontrollable |
| F75303_DDR | `charger_f75303@4d` | uncontrollable |
| F75303_CPU | `cpu@4c` | CPU |
| APU | `apu_f75303@4d` | CPU |
| dGPU VR | `gpu_vr_f75303@4d` | GPU |
| dGPU VRAM | `gpu_vram_f75303@4d` | GPU |
| dGPU AMB | `gpu_amb_f75303@4d` | GPU |
| dGPU temp | `gpu_temp@40` | GPU |

Spike (one load test): the VR/VRAM values looked swapped between the two tools at idle
(42/46 vs 44.85/40.85); pin the label-to-label mapping under load before the group sets are
frozen.

1. **One shared setpoint T\*.** As today (§2.6): the fan target RPM → duty (duty↔RPM table) →
   the temperature on the live fw-fanctrl curve's tread that yields that duty. Both device loops
   regulate toward this same T\*. Because the fan follows the max, the device that is not the
   hottest may run all the way up to T\* at no acoustic cost — this is the point of the design.
2. **CPU loop.** PI on `max(cpu@4c, apu)` (the EC replica's boxcar of it, as §2.2) → sustained
   CPU watts cap via `ryzenadj`, clamped to `[cpu_floor_w, cpu_max_w]`. Reuses the existing CPU
   actuator, read-back verification (§2.9), stickiness watchdog, stock restore.
3. **GPU loop.** PI on `max(gpu_vr, gpu_vram, gpu_amb, gpu_temp)` → **GPU max-clock lock
   directly**, clamped to `[gpu_floor_mhz, 3090]`, rate-limited as the current clock loop is.
   The clock→watts LUT, its calibration sweep, and the watts inner loop are **deleted**.
   `verify_lock`, the GPU hot guard (88 / exit 86), and `GPU_TRIP_C` stay.
4. **Anti-windup per loop, "halt = hold".** Standard directional conditional integration:
   when the device is drawing more than its margin below its cap (CPU: watts vs cap; GPU:
   clock vs lock, using the existing pinned test) AND the error asks for more, the integrator
   holds its last value (never zeroed, never reset). Output clamps bound the rest. This replaces
   §2.4's demand-limited rule, the spike's harness and `DEMAND_MARGIN_W_*` semantics.
5. **Shadow cap + override control (no device ever runs "uncapped").** Per device, two
   candidate caps go through a **min selector**:
   - the **thermal cap** from the PI above (setpoint T\*);
   - a **shadow cap** = `draw + headroom` (CPU: watts, ~10 W; GPU: clock, a few hundred MHz —
     tunables). It **rises one headroom step per sample** while the device is pinned against
     it and its group temperature is below T\* minus a band, so a load jump becomes a ramp of
     seconds (38 → 100 W in ~6 s) instead of a one-sample step the fan overshoots on; it
     **falls slowly** when draw falls, so a scene dip does not pull the cap under the next
     burst. Rise time is the key tunable: ~6 s reads as smoothness, >15 s is the onset
     starvation §2.4's "run uncapped through the dead time" rule existed to prevent.
   - **Tracking:** whichever cap is NOT selected has its state set to the selected value every
     tick (standard override control), so the thermal PI never winds to its max clamp while
     the shadow binds and takes over from exactly the applied cap when the device reaches T\* —
     no descent-from-max overshoot through the 60 s EC average. The thermal integrator moves
     only while the thermal cap is the selected one; that IS the hold rule for this device.
   - Why the shadow cap earns its keep beyond ramp shaping: a GPU clock lock is a **frequency
     ceiling**, so a card at a 2 GHz lock runs ~80 % duty at a lower V/F point instead of
     boosting to 3 GHz at ~56 % — same work, fewer watt-hours; the GPU shadow binds
     continuously. On the CPU `ryzenadj` caps average power, so a cap just above draw only
     clips boost peaks — a smaller win.
   A device whose load cannot reach T\* therefore sits at its shadow cap, near saturation.
   Re-derive the §2.7 "target unreachable" rules and the fan-band acceptance metric per
   device; the fan target is met when the **argmax** device sits at T\*.
6. **Outer RPM trim (Mode B and drift).** A third, slow PI on the fan-RPM error that nudges the
   shared T\* up or down (bounded authority, same hold-on-halt rule). In Mode B (fw-fanctrl
   absent / curve invalid, no T\* derivable) it is the only source of T\*, seeded from the last
   good curve; in Mode A it runs as a slow correction for curve/table drift. One mechanism, two
   gains — decide whether the Mode A/B arbiter survives or collapses into "T\* source".
7. **Gains: per-device step test.** Calibration steps the CPU watts and fits the CPU group's
   response, then steps the GPU clock and fits the GPU group's response (IMC gains per fit, as
   §3.3 does for one plant). Replaces the single step test and the LUT sweep. The settle gate
   must be honest about needing a steady load on the device being stepped (the 2026-09-10
   calibration never settled: gpu-burn ended mid-settle; fan noise 100–200 RPM per 30 s). The
   two plants have different dead times (`gpu_vr` trails the die by 40 s+ and the EC averages
   60 s on top).
8. **Coupling.** CPU heat raises the GPU group a little and vice versa; both loops target the
   same T\* and share one fan, so they settle rather than fight, but a step test on one device
   sees a small response on the other's sensors — the fit must ignore the cross term.
9. **Carries over unchanged.** EC replica + reconciliation (§2.2/§2.6), duty↔RPM table and its
   refinement, thermal guards and watchdog, warm start (now per device cap, keyed as today),
   floors as output clamps, the fanctrl socket poller, telemetry v2 (columns change), TUI.
10. **Deleted.** `Budget`, `split_budget`, `allocator::demand`, `DEMAND_MARGIN_W_*`, the budget
    bounds and `with_lut_floor_clamp`, `ClockWattsLut` + sweep, the watts inner loop, the
    `spike_antiwindup` harness, §2.4/§2.5 as written.

## Acceptance (sketch for the design phase to sharpen)

- Engaging Auto under a steady game load changes neither device's draw by more than the margin
  in the first 10 s (bumpless).
- Plant-closed sim: CPU-heavy load with a light GPU → CPU regulated to T\*, GPU uncapped at
  3090; GPU-heavy load with a light CPU → the mirror; both heavy → both at T\*, fans within
  ±150 RPM of target ≥ 90 % of a 30-min converged window (as today's bar).
- A device that cannot reach T\* never holds the other device's cap.
- Hardware: the same 30-min gaming check that graded every previous change, on this machine.

## Open for the design phase

- Exact hold rule for the GPU loop (clock pinned vs lock, with what utilisation floor).
- Whether the outer trim replaces the Mode A/B arbiter or sits beside it.
- Telemetry columns for two loops (per-device error, cap, hold state — and the `decision`
  line's `freeze` column must be live this time).
- Migration of `state.json` (LUT removed, per-device gains added) and of the warm-start keys.
