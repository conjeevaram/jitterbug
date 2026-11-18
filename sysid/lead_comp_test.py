# pyright: basic

"""
compare lead compensation settings on teleop runs

lag ms is how far behind the leader the follower is in time: for steady motion the
error is lag * leader speed, so lag = std(error) / rms(speed)

the prediction from the fitted model is that lag SHOULD fall from about 100 ms with no lead
to about 30 ms at 115 ms, e2e rms falls about 65%, 75 and 160 ms land in between (115 ms is the theoretical optimum)
and goal activity rises to about 1.4 - this is important. The goal noise is amplified, which means there is a tradeoff.
"""

import csv
import sys
from pathlib import Path

import numpy as np

DT = 0.005
SKIP = 400  # samples, the first 2 s while the follower catches up to the leader


def load(path, joints):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {f"{p}{j}": np.array([int(r[f"{p}{j}"]) for r in rows], float)[SKIP:] for p in "lgm" for j in joints}


def metrics(leader, goal, meas):
    err = leader - meas
    speed = (leader[10:] - leader[:-10]) / (10 * DT)  # ticks/s, over 10 samples to get past quantization
    return {
        "lag": 1e3 * err.std() / np.sqrt(np.mean(speed**2)),
        "e2e": np.sqrt(np.mean(err**2)),
        "activity": np.abs(np.diff(goal)).mean() / np.abs(np.diff(leader)).mean(),  # 1 = goal moves like the leader
    }


if __name__ == "__main__":
    joints = [int(j) for j in (sys.argv[2] if len(sys.argv) > 2 else "2,3").split(",")]
    runs = {}
    for path in sorted(Path(sys.argv[1]).glob("lead*_*.csv")):
        d = load(path, joints)
        ms = int(path.stem[4:].split("_")[0])
        runs.setdefault(ms, []).append({j: metrics(d[f"l{j}"], d[f"g{j}"], d[f"m{j}"]) for j in joints})
    for j in joints:
        print(f"joint {j}\nlead ms  runs  lag ms (min..max)  e2e rms  vs no lead  goal activity")
        base = np.mean([r[j]["e2e"] for r in runs[0]])
        for ms, rs in sorted(runs.items()):
            lag = [r[j]["lag"] for r in rs]
            e2e = np.mean([r[j]["e2e"] for r in rs])
            act = np.mean([r[j]["activity"] for r in rs])
            print(f"{ms:7} {len(rs):5}  {np.mean(lag):5.0f} ({min(lag):.0f}..{max(lag):.0f})  {e2e:8.1f}  {100 * (e2e / base - 1):+9.0f}%  {act:13.2f}")
