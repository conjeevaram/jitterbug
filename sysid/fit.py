# pyright: basic

"""
fit a model of one joint, goal -> measured follower position

    goal -> deadband -> delay -> lag -> measured
"""

import argparse
import csv
import sys
from typing import NamedTuple

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

DT = 0.005  # s / sample (200hz)
DEADBANDS = range(13)  # in ticks
DELAYS = range(1, 16)  # in samples, min 1 because follower is read before writing goal


class Model(NamedTuple):
    deadband: int  # the goal needs to move this far before servo reacts
    delay: int  # samples between the goal moving and the servo reacting
    tau: float  # time constant of the lag (s)
    gain: float  # settled measurement = gain * goal + offset
    offset: float


def load(path, joint):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if any(int(r["missed"]) for r in rows):
        sys.exit(f"{path}: missed ticks, sampling is not uniform")
    goal = np.array([int(r[f"g{joint}"]) for r in rows], float)
    measured = np.array([int(r[f"m{joint}"]) for r in rows], float)
    if (measured == 65535).any():
        sys.exit(f"{path}: follower read failed somewhere in the log")
    return goal, measured


def apply_deadband(goal, deadband):
    """goal the servo reacts to. it stays put until the real goal is farther than the
    deadband, then it follows, always deadband behind"""
    held = goal.copy()
    for k in range(1, len(goal)):
        held[k] = min(max(held[k - 1], goal[k] - deadband), goal[k] + deadband)
    return held


def simulate_servo(held, start, delay, tau, gain, offset):
    """we predict follower position (forward propogation), for a goal that made it past the deadband"""
    a = np.exp(-DT / tau)                                       # servo chases a target closing some fraction (1-a)*gap every sample
    target = np.full(len(held), start)                          # servo starts at rest at start pose
    target[delay:] = gain * held[: len(held) - delay] + offset
    # same as x[k] = x[k-1] + (1 - a) * (target[k] - x[k-1]), starting at x = start
    return lfilter([1 - a], [1, -a], target, zi=[a * start])[0]


def predict(model, goal, measured):
    held = apply_deadband(goal, model.deadband)
    return simulate_servo(held, measured[0], model.delay, model.tau, model.gain, model.offset)


def percent(predicted, measured):
    return 100 * (1 - np.linalg.norm(predicted - measured) / np.linalg.norm(measured - measured.mean()))


def score(model, clips):
    return [percent(predict(model, g, m), m) for g, m in clips]


def fit_lag(held, measured, delay):
    """tau, gain and offset that produce the least error between simulation model and measurements.
    the error of each clip is divided by its spread, so that squaring doesnt affect weights"""

    def residual(p):
        tau, gain, offset = p
        return np.concatenate([
            (simulate_servo(h, m[0], delay, tau, gain, offset) - m) / np.linalg.norm(m - m.mean())
            for h, m in zip(held, measured)
        ])

    bounds = ([0.01, -np.inf, -np.inf], [1.0, np.inf, np.inf])  # tau must stay positive
    return least_squares(residual, [0.1, 1.0, 0.0], bounds=bounds).x


def fit(train):
    """try every deadband and delay, fit the rest for each using fit_lag()"""
    measured = [m for _, m in train]
    tried = []
    for deadband in DEADBANDS:
        held = [apply_deadband(g, deadband) for g, _ in train]
        for delay in DELAYS:
            tau, gain, offset = fit_lag(held, measured, delay)
            model = Model(deadband, delay, tau, gain, offset)
            predicted = [simulate_servo(h, m[0], delay, tau, gain, offset) for h, m in zip(held, measured)]
            tried.append((np.mean([percent(p, m) for p, m in zip(predicted, measured)]), model))
    best = max(s for s, _ in tried)
    return max(tried, key=lambda t: t[0])[1], [m for s, m in tried if s >= best - 1]


def cross_validate(clips):
    """fit on all clips but one, score on the one left out"""
    return [score(fit(clips[:i] + clips[i + 1 :])[0], [clips[i]])[0] for i in range(len(clips))]


def show(model, ties, names, train, val):
    """print the model info"""
    ms = DT * 1e3
    goal = np.mean(np.concatenate([g for g, _ in train]))
    rest = model.gain * goal + model.offset - goal  # where it settles relative to the goal
    print(f"deadband {model.deadband} ticks, delay {model.delay * ms:.0f} ms, time constant {model.tau * 1e3:.0f} ms")
    print(f"gain {model.gain:.2f}, settles {rest:+.0f} ticks from the goal at the mean goal {goal:.0f}")
    print(f"within 1 point of best: deadband {min(t.deadband for t in ties)} to {max(t.deadband for t in ties)}"
          f", delay {min(t.delay for t in ties) * ms:.0f} to {max(t.delay for t in ties) * ms:.0f} ms")
    for name, s in zip(names[: len(train)], score(model, train)):
        print(f"train {s:6.1f}%  {name}")
    for name, s in zip(names[len(train) :], score(model, val)):
        print(f"val   {s:6.1f}%  {name}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, required=True)
    ap.add_argument("--cv", action="store_true", help="also hold each training clip out in turn")
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--val", nargs="+", default=[])
    a = ap.parse_args()
    train = [load(p, a.joint) for p in a.train]
    val = [load(p, a.joint) for p in a.val]
    model, ties = fit(train)
    show(model, ties, a.train + a.val, train, val)
    if a.cv:
        cv = cross_validate(train)
        print(f"held out, one clip at a time: mean {np.mean(cv):.0f}%, median {np.median(cv):.0f}%")
        for name, s in zip(a.train, cv):
            print(f"held out {s:6.1f}%  {name}")
