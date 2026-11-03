"""Compare two trajectory CSVs from run_*_sequence.py. Usage: compare.py seq.json a.csv b.csv"""
import json
import sys

import numpy as np

seq = json.load(open(sys.argv[1]))
A = np.loadtxt(sys.argv[2], delimiter=",", skiprows=1)
B = np.loadtxt(sys.argv[3], delimiter=",", skiprows=1)
names = [f"joint{i}" for i in range(1, 9)]
dt = seq["dt"]


def settle(q, tgt, band):
    out = np.abs(q - tgt) > band
    if out[-1]:
        return float("nan")
    bad = np.nonzero(out)[0]
    return 0.0 if len(bad) == 0 else (bad[-1] + 1) * dt


start = 0
print(f"{'seg':>3} {'joint':>6} {'target':>8} | {'A final':>9} {'A err':>9} {'A settle':>8} | {'B final':>9} {'B err':>9} {'B settle':>8}")
for s, seg in enumerate(seq["segments"]):
    n = round(seg["dur"] / dt)
    for j, name in enumerate(names):
        tgt = seg["target"][j] if j < 7 else -seg["target"][6]
        band = 0.01 if j < 6 else 0.001
        qa, qb = A[start:start + n, j + 1], B[start:start + n, j + 1]
        print(f"{s:>3} {name:>6} {tgt:>8.4f} | {qa[-1]:>9.4f} {qa[-1]-tgt:>+9.4f} {settle(qa, tgt, band):>8.3f} | "
              f"{qb[-1]:>9.4f} {qb[-1]-tgt:>+9.4f} {settle(qb, tgt, band):>8.3f}")
    start += n
d = A[:, 1:] - B[:, 1:]
print("RMS diff A-B per joint:", " ".join(f"{n}={v:.4f}" for n, v in zip(names, np.sqrt((d ** 2).mean(0)))))
print("MAX diff A-B per joint:", " ".join(f"{n}={v:.4f}" for n, v in zip(names, np.abs(d).max(0))))
