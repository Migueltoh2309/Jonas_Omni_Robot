#!/usr/bin/env python3
"""Analiza escalones de /jonas/wheel_state (CSV de `ros2 topic echo --csv`), por rueda.

Columnas del CSV: data_offset, objetivo x3, medida x3, duty x3.
"""
import sys

RPM = 60.0 / (2 * 3.141592653589793)
DT = 0.02  # wheel_state a 50 Hz

rows = []
for line in open(sys.argv[1]):
    f = line.strip().split(',')
    if len(f) < 10:
        continue
    rows.append([float(x) for x in f[1:10]])

print(f"{'rueda':>5} {'ref RPM':>8} {'t90 s':>6} {'sobrepico %':>11} {'error ss RPM':>12} "
      f"{'ruido RPM':>9} {'duty ss':>8} {'duty max':>8}")
for w in range(3):
    segs, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or abs(rows[i][w] - rows[start][w]) > 1e-6:
            segs.append((start, i))
            start = i
    for a, b in segs:
        ref, n = rows[a][w], b - a
        if abs(ref) < 1e-6 or n < 20:
            continue
        meas = [r[3 + w] for r in rows[a:b]]
        duty = [r[6 + w] for r in rows[a:b]]
        t90 = next((k * DT for k, m in enumerate(meas) if m / ref >= 0.9), float('nan'))
        peak = max(meas) if ref > 0 else min(meas)
        over = max(0.0, (peak - ref) / ref * 100)
        tail = meas[int(n * 0.6):]
        mt = sum(tail) / len(tail)
        noise = (sum((m - mt) ** 2 for m in tail) / len(tail)) ** 0.5
        dt_ = duty[int(n * 0.6):]
        print(f"{w + 1:>5} {ref * RPM:8.1f} {t90:6.2f} {over:11.1f} {(ref - mt) * RPM:12.2f} "
              f"{noise * RPM:9.2f} {sum(dt_) / len(dt_):8.3f} {max(abs(d) for d in duty):8.3f}")
