# GPU-group time-constant evidence for roast d3-15

## Disposition

The September captures do **not** support calling any single GPU-group time constant a clean
measurement.  The best direct replay of the implementation's FOPDT method gives 11.3 s and
13.9 s for the raw `gpu_group_c = max(gpu_amb, gpu_vr, gpu_vram, gpu_temp)` rising response.
Those fits are repeatable, but the experiment changed fan speed at the same time as GPU power
and the group's dominant sensor changes from the fast die proxy to the slower VR sensor.  A
single-input, single-pole model therefore folds changing cooling and two thermal masses into
one number.

Use **`tau_gpu_s = 15 s` as an explicitly provisional default**, not as a measured hardware
constant.  This is a conservative rounding above both full-rise group fits.  With the design's
other default assumptions (`K = 0.02 C/MHz`, `theta_eff = 90 s`, `lambda = 270 s`), re-derive:

```text
Ti = tau = 15 s
Kc = tau / (K * (lambda + theta_eff))
   = 15 / (0.02 * (270 + 90))
   = 2.083 MHz/C          # use about 2.1 MHz/C
```

The existing 35 s / 4.9 MHz/C pair should not remain merely because 35 s was already the CPU
default.  Fifteen seconds is outside roast d3-15's old-default sensitivity band of
17.5--52.5 s, so the corresponding default `Kc` must change with it.

For offline closed-loop validation, cover the deliberately broad raw-plant range
**`tau_gpu_s in [8, 50] s`**, including at least 8, 15, 25, and 50 s.  Do not substitute a
`+/-50%` sweep around 15 s; that would miss the slow VR/VRAM branch.  Cross this range with the
already intended uncertainty in **`K in [0.01, 0.03] C/MHz`** and
**`theta_eff in [45, 135] s`**.  The controller defaults remain the provisional nominal values
above while the simulated plant takes these perturbations.

## Exact data sources

Both purpose-built captures are tracked in this repository (introduced by commit
`cb4a2ab0d74608c2f3e0e0e317826fb1a65854c5`):

| Capture | Experiment | SHA-256 |
|---|---|---|
| `docs/research/2026-09-09-gpu-burn-fanctrl-quiet16.csv` | Approximately 100 W `gpu-burn`, fw-fanctrl `quiet16`; burn rises between t=11 and 13 s and ends at t=258 s | `a2fdd5ed82ca1c4b7b98b8be9bc1ec68ee93e1c5e5440d27c56543ba6b5adfbe` |
| `docs/research/2026-09-09-gpu-burn-ec-native.csv` | Approximately 100 W `gpu-burn`, EC-native fan control; burn rises between t=12 and 14 s and ends at t=258 s | `76d1606245a508ad4d2cdda689d4cdb8e5a22e7b1e37b9acf78a31e9f190e2e1` |

The longer September telemetry files were also inspected:

```text
/var/lib/bazerame-fans/telemetry/run-1789067819.jsonl
/var/lib/bazerame-fans/telemetry/run-1789139478.jsonl
```

They record only the global `ec_max` and `ec_argmax`, not the four raw GPU-group members.  At
the start of GPU activity the global maximum is often a CPU/ambient label, and the workload
power is not a controlled square step.  They cannot reconstruct a GPU-group baseline and are
therefore unsuitable for this fit.  They were not used numerically.

## Fit method and results

For each CSV row, reconstruct the raw group exactly as proposed by design section 2.1:

```text
gpu_group_c = max(gpu_amb_f75303@4d,
                  gpu_vr_f75303@4d,
                  gpu_vram_f75303@4d,
                  gpu_temp@40)
```

Take the power-step time as the midpoint between the last sample below 90 W and the first
sample at or above 90 W.  Fit every sample from the beginning of the file through the last
loaded sample before the falling edge.  The model and coarse-to-fine grid are the same as
`src/calib/fopdt.rs::fit_fopdt`:

```text
y(t) = y0 + delta_y * (1 - exp(-(t - theta) / tau)), t > theta
y(t) = y0,                                           t <= theta
```

For each `(tau, theta)`, solve `y0` and `delta_y` by ordinary least squares, then minimize SSE
over the same 25 by 25 grid, 9 refinement rounds, 0.35 shrink factor, initial
`tau=[0.1, 3*span]` and `theta=[0, 0.5*span]`.  This fit uses the **raw group**.  Applying the
60 s boxcar first would be wrong for the design's default calculation because its
`theta_eff` already adds `ma_interval/2`; filtering here would count that lag twice.

| Capture | Samples | tau | theta | fitted y0 | fitted delta | R2 | RMSE |
|---|---:|---:|---:|---:|---:|---:|---:|
| EC native | 114 | 11.26 s | 0.00 s | 56.38 C | 19.56 C | 0.9455 | 1.21 C |
| quiet16 | 114 | 13.89 s | 0.00 s | 54.93 C | 28.45 C | 0.9764 | 1.16 C |

The fit is reasonably stable once the quiet16 capture contains 90 s of loaded data.  Changing
the post-step horizon exposes the between-experiment uncertainty:

| Loaded horizon | EC-native tau | quiet16 tau |
|---:|---:|---:|
| 60 s | 8.36 s | 16.99 s |
| 90 s | 8.64 s | 15.23 s |
| 120 s | 9.05 s | 13.92 s |
| 180 s | 10.10 s | 13.80 s |
| Full rise (about 245 s) | 11.26 s | 13.89 s |

Fitting individual group members over the full rise demonstrates why one group pole is only
an approximation:

| Signal | EC-native tau | quiet16 tau |
|---|---:|---:|
| `gpu_temp@40` (die proxy; dominates the early group max) | 5.6 s | 10.2 s |
| `gpu_vr_f75303@4d` | 43.4 s | 43.1 s |
| `gpu_vram_f75303@4d` | 36.6 s | 34.6 s |
| `gpu_amb_f75303@4d` | 39.2 s | 39.7 s |

The falling edge cannot repair the identification.  Only 30--31 s of cooldown was captured;
the fans fall during that window and the group never returns to baseline.  An unconstrained
FOPDT extrapolates 24.4 s (EC native) and 37.8 s (quiet16), but its fitted final temperatures
are 62.8 C and 60.9 C versus pre-burn group means of 55.7 C and 53.7 C.  Those are truncated
tail estimates, not independent tau measurements.

## Evidence limitations

- Fan speed is not held fixed.  Across the loaded interval, it rises from roughly 1.5k to
  3.4k RPM under quiet16 and from roughly 2.1k to 4.0k RPM under EC-native control.  Cooling is
  an omitted, time-varying input to the one-input FOPDT fit.
- `max(...)` is nonlinear.  `gpu_temp@40` dominates the fast rise; `gpu_vr` dominates the late
  plateau and cooldown.  The controller sees this composite, but it is not a physical
  single-pole sensor.
- Temperatures are integer degrees and samples are approximately 2--3 s apart.  There are only
  six pre-step samples, and the exact burn start lies between samples.
- There are only two runs, both at essentially the same approximately 100 W step.  They do not
  test clock-dependent tau, intermediate loads, repeatability, or a constant-RPM condition.
- The 15 s disposition therefore represents the observed rising group response and errs
  slightly above both full-rise fits.  The 8--50 s validation range carries the unresolved
  fast-die/slow-VR ambiguity into simulation instead of hiding it in false precision.

A future value can be called measured only after a capture holds fan RPM fixed (or identifies
fan RPM as an independently excited input), logs all four GPU labels at 1 Hz, records at least
30 s of pre-step baseline and at least `3 * tau` after both rising and falling clock/power
steps, and repeats at two or more power levels.

## Reproduction

Run from the repository/worktree root.  The following is self-contained and uses only the
Python standard library.  It prints the full-rise table and the horizon sensitivity table
above.

```bash
python3 - <<'PY'
import csv, glob, math

GPU = [
    'gpu_amb_f75303@4d', 'gpu_vr_f75303@4d',
    'gpu_vram_f75303@4d', 'gpu_temp@40',
]

def linear_fit(data, tau, theta):
    g = [0.0 if t <= theta else 1.0 - math.exp(-(t-theta)/tau)
         for t, _ in data]
    n = len(data)
    gb = sum(g) / n
    yb = sum(y for _, y in data) / n
    den = sum((x-gb)**2 for x in g)
    if den < 1e-9:
        return None
    b = sum((x-gb)*(y-yb) for x, (_, y) in zip(g, data)) / den
    a = yb - b*gb
    sse = sum((y-a-b*x)**2 for x, (_, y) in zip(g, data))
    return a, b, sse

def fit(data):
    span = max(t for t, _ in data) - min(t for t, _ in data)
    tau_lo, tau_hi = 0.1, 3.0*span
    theta_lo, theta_hi = 0.0, 0.5*span
    for _ in range(9):
        best = None
        for i in range(25):
            tau = tau_lo + (tau_hi-tau_lo)*i/24
            for j in range(25):
                theta = theta_lo + (theta_hi-theta_lo)*j/24
                f = linear_fit(data, tau, theta)
                if f and (best is None or f[2] < best[2]):
                    best = (tau, theta, f[2], f[0], f[1])
        tau, theta = best[:2]
        tau_span = (tau_hi-tau_lo)*0.35
        theta_span = (theta_hi-theta_lo)*0.35
        tau_lo, tau_hi = max(0.01, tau-tau_span/2), tau+tau_span/2
        theta_lo, theta_hi = max(0.0, theta-theta_span/2), theta+theta_span/2
    return best

for path in glob.glob('docs/research/2026-09-09-gpu-burn-*.csv'):
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        for k in ['t', 'gpu_w', *GPU]:
            r[k] = float(r[k])
        r['group'] = max(r[k] for k in GPU)
    first = next(i for i, r in enumerate(rows) if r['gpu_w'] >= 90)
    stop = next(i for i, r in enumerate(rows[first+1:], first+1)
                if r['gpu_w'] < 50)
    step_t = (rows[first-1]['t'] + rows[first]['t']) / 2
    print('\n' + path, 'step midpoint', step_t, 'stop', rows[stop]['t'])
    for horizon in [60, 90, 120, 180, float('inf')]:
        data = [(r['t']-step_t, r['group']) for r in rows[:stop]
                if r['t']-step_t <= horizon]
        tau, theta, sse, y0, delta = fit(data)
        mean = sum(y for _, y in data) / len(data)
        sst = sum((y-mean)**2 for _, y in data)
        print(f'h={horizon:>4}: n={len(data):3d} tau={tau:5.2f} '
              f'theta={theta:4.2f} y0={y0:5.2f} delta={delta:5.2f} '
              f'R2={1-sse/sst:.4f} RMSE={math.sqrt(sse/len(data)):.3f}')
    print('individual members, full rise:')
    for key in GPU:
        data = [(r['t']-step_t, r[key]) for r in rows[:stop]]
        tau, theta, sse, y0, delta = fit(data)
        print(f'  {key:24s} tau={tau:5.2f} theta={theta:4.2f}')
    cool_t = rows[stop]['t']
    data = [(r['t']-cool_t, r['group']) for r in rows
            if r['t'] >= cool_t-30]
    tau, theta, sse, y0, delta = fit(data)
    pre_mean = sum(r['group'] for r in rows[:first]) / first
    print(f'cooldown: tau={tau:5.2f} theta={theta:4.2f} '
          f'fitted_final={y0+delta:5.2f} pre_mean={pre_mean:5.2f}')
PY

sha256sum docs/research/2026-09-09-gpu-burn-*.csv
wc -l docs/research/2026-09-09-gpu-burn-*.csv
head -2 /var/lib/bazerame-fans/telemetry/run-1789067819.jsonl
head -2 /var/lib/bazerame-fans/telemetry/run-1789139478.jsonl
```
