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
