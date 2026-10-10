#!/usr/bin/env python3
"""Run measure_res at several currents and fit V = R*I + V0 (V0 = residual dead-time error)."""
import re, subprocess, sys, os
d = os.path.dirname(os.path.abspath(__file__))
pts = []
for i in [5, 10, 20, 30]:
    out = subprocess.run([sys.executable, os.path.join(d, "vesc_term.py"), f"measure_res {i}", "--wait", "8"], capture_output=True, text=True).stdout
    m = re.search(r"Resistance: ([\d.]+)", out)
    if not m: print(f"{i} A: no result\n{out}"); continue
    r = float(m.group(1)); pts.append((i, r * i))
    print(f"{i:3d} A  R {r*1000:7.2f} mOhm  V {r*i:.3f} V")
n = len(pts); mi = sum(p[0] for p in pts) / n; mv = sum(p[1] for p in pts) / n
slope = sum((a - mi) * (b - mv) for a, b in pts) / sum((a - mi) ** 2 for a, _ in pts)
print(f"fit: Rs = {slope*1000:.2f} mOhm, residual dead-time voltage = {mv - slope*mi:.3f} V")
