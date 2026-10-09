#!/usr/bin/env python3
"""Fit release_curve.lisp output: per current, Lm^2/Lr from the voltage at release and tau_r from the decay.

Stator voltage after release: V(t) = w_r * (Lm^2/Lr) * I * exp(-t/tau_r), riding on a fixed offset vector o.
Averaged over a whole electrical cycle, |V + o|^2 = V^2 + |o|^2, so cycle-mean(|v|^2) - REST = V(t)^2.
"""
import math, re, sys
PP = 4
rest, dt, samples, pts = None, None, [], []
for line in open(sys.argv[1]):
    m = re.search(r'"(REST|DT|S|PT)\b ?([^"]*)"', line)
    if not m: continue
    tag, rest_of = m.group(1), m.group(2).split()
    if tag == "REST": rest = float(rest_of[0])
    elif tag == "DT": dt = float(rest_of[0]) / 1000.0; samples = []
    elif tag == "S": samples += [float(x) for x in rest_of]
    elif tag == "PT":
        I, duty, idd, iq, rpm, vin = map(float, rest_of)
        pts.append(dict(I=I, duty=duty, id=idd, iq=iq, rpm=rpm, vin=vin, dt=dt, s=samples))
print(f"offset at rest: {math.sqrt(rest):.3f} V rms\n")
print(f"{'I set':>5} {'I meas':>6} {'rotor':>6} {'slip':>5} {'V0':>6} {'Lm2/Lr':>7} {'psi_r':>6} {'tau_r':>6} {'fit pts':>7}")
print(f"{'A':>5} {'A':>6} {'rpm':>6} {'%':>5} {'V':>6} {'uH':>7} {'mWb':>6} {'ms':>6} {'':>7}")
for p in pts:
    w_r = 2 * math.pi * p["rpm"] / 60 * PP              # rotor electrical rad/s (coasting, ~constant)
    f_e = w_r / (2 * math.pi)
    win = max(3, round(1.0 / f_e / p["dt"]))            # samples per electrical cycle
    s = p["s"]
    t, y = [], []
    for k in range(0, len(s) - win, max(1, win // 2)):  # half-cycle steps, full-cycle windows
        v2 = sum(s[k:k + win]) / win - rest
        t.append((k + win / 2) * p["dt"]); y.append(v2)
    # fit ln(V^2) vs t over the part well above the noise floor
    noise = max(0.02 * rest, 1e-4)
    use = [(ti, math.log(yi)) for ti, yi in zip(t, y) if yi > 10 * noise]
    use = use[: next((i for i in range(1, len(use)) if use[i][1] > use[i-1][1] + 0.5), len(use))]
    i_meas = math.hypot(p["id"], p["iq"])
    if len(use) >= 3:
        n = len(use); mt = sum(a for a, _ in use) / n; my = sum(b for _, b in use) / n
        slope = sum((a - mt) * (b - my) for a, b in use) / sum((a - mt) ** 2 for a, _ in use)
        icpt = my - slope * mt
        tau = -2.0 / slope                               # V^2 decays at 2/tau
        v0 = math.sqrt(math.exp(icpt))
        lm2lr = v0 / (w_r * i_meas)
        slip = 100 * (1 - p["rpm"] * PP / 6000.0)
        print(f"{p['I']:5.0f} {i_meas:6.1f} {p['rpm']:6.0f} {slip:5.1f} {v0:6.3f} {lm2lr*1e6:7.1f} {v0/w_r*1e3:6.3f} {tau*1e3:6.1f} {n:7d}")
    else:
        print(f"{p['I']:5.0f} {i_meas:6.1f} {p['rpm']:6.0f}   decay not resolved ({len(use)} usable points)")
    if "-v" in sys.argv:
        print("   t(ms)  V(V):", " ".join(f"{ti*1e3:.0f}:{math.sqrt(max(yi,0)):.2f}" for ti, yi in zip(t, y)))
