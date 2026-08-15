#!/usr/bin/env python3
"""
test_rotor_failure.py — the rotor-failure injection, checked without Isaac.

RotorFailureBackend is duck-typed rather than a Backend subclass precisely so
this can run under plain python3. What it verifies:

  1  no failure before the settle window has elapsed
  2  auto-fail fires once the vehicle holds a position
  3  a vehicle that keeps moving never auto-fails
  4  the stopped rotor reads exactly 0.0, and only that one
  5  the failure is one-shot: it never moves to another rotor
  6  trigger() fires immediately, before any settle
  7  a forced rotor index is honoured
  8  unknown attributes fall through to the wrapped controller

Run:  python3 examples/controller_tests/evaluations/test_rotor_failure.py
"""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import rotor_failure


class FakeState:
    def __init__(self, p, v, w=(0.0, 0.0, 0.0)):
        self.position = np.array(p, dtype=float)
        self.linear_velocity = np.array(v, dtype=float)
        self.angular_velocity = np.array(w, dtype=float)


class FakeController:
    """Minimal stand-in: always commands the same four rotor velocities."""
    def __init__(self, cmd=(600.0, 610.0, 620.0, 630.0)):
        self.cmd = list(cmd)
        self.hover_target = np.array([0.0, 0.0, 1.5])
        self.marker = "i-am-the-inner-controller"
        self.started = False
    def initialize(self, vehicle): pass
    def start(self): self.started = True
    def stop(self): pass
    def reset(self): pass
    def update_sensor(self, *a): pass
    def update_graphical_sensor(self, *a): pass
    def update_state(self, state): pass
    def update(self, dt): pass
    def input_reference(self): return list(self.cmd)


class FakeMonitor:
    def __init__(self, airborne=True):
        self.airborne = airborne
        self.failure = ""
    def reset(self): pass
    def update(self, t, p, bounds): return False


DT = 1.0 / 250.0


def build(**kw):
    inner = FakeController()
    kw.setdefault("monitor", FakeMonitor())
    kw.setdefault("rng", np.random.default_rng(0))
    b = rotor_failure.RotorFailureBackend(inner, **kw)
    b.start()
    return inner, b


def fly(b, seconds, p=(0.0, 0.0, 1.5), v=(0.0, 0.0, 0.0), drift=0.0):
    """Step the backend, optionally drifting so it never settles."""
    n = int(seconds / DT)
    pos = np.array(p, dtype=float)
    for _ in range(n):
        pos = pos + np.array([drift * DT, 0.0, 0.0])
        b.update_state(FakeState(pos, v))
        b.update(DT)


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def main() -> int:
    ok = True
    print("rotor-failure injection")

    # 1 / 2 — auto-fail waits for the full hold window, then fires
    _, b = build(auto_fail=True, settle_hold=1.5)
    fly(b, 1.0)
    ok &= check("no failure before the hold window elapses", not b.has_failed)
    fly(b, 1.0)
    ok &= check("auto-fail fires once settled", b.has_failed)
    ok &= check("failure time is recorded", b.failure_time is not None)

    # 3 — a moving vehicle never settles
    _, b = build(auto_fail=True, settle_hold=1.5)
    fly(b, 6.0, v=(1.0, 0.0, 0.0), drift=1.0)
    ok &= check("a drifting vehicle never auto-fails", not b.has_failed)

    # 4 — exactly the stopped rotor reads zero
    _, b = build(auto_fail=True, settle_hold=1.0, rotor=2)
    fly(b, 2.0)
    ref = b.input_reference()
    ok &= check("stopped rotor is exactly 0.0", ref[2] == 0.0)
    ok &= check("other rotors untouched",
                [ref[i] for i in (0, 1, 3)] == [600.0, 610.0, 630.0])

    # 5 — one-shot: a second trigger cannot move the failure
    first = b.failed_rotor
    b.trigger(rotor=0)
    fly(b, 0.5)
    ok &= check("failure is one-shot (rotor unchanged)", b.failed_rotor == first)
    ok &= check("still only one rotor stopped",
                sum(1 for x in b.input_reference() if x == 0.0) == 1)

    # 6 — manual trigger fires with no settle at all
    _, b = build(auto_fail=False, settle_hold=99.0)
    fly(b, 0.5, v=(5.0, 0.0, 0.0), drift=5.0)
    ok &= check("manual mode does not auto-fail", not b.has_failed)
    b.trigger()
    fly(b, 2 * DT)
    ok &= check("trigger() fires immediately", b.has_failed)

    # 7 — forced index honoured
    _, b = build(auto_fail=True, settle_hold=0.5, rotor=3)
    fly(b, 1.0)
    ok &= check("forced rotor index honoured", b.failed_rotor == 3)

    # 8 — attribute passthrough to the wrapped controller
    inner, b = build()
    ok &= check("unknown attributes reach the inner controller",
                b.marker == inner.marker)
    ok &= check("start() propagated to the inner controller", inner.started)
    ok &= check("hover_target reachable through the wrapper",
                np.allclose(b.hover_target, [0.0, 0.0, 1.5]))

    # Guard the gate itself: not airborne means never settled.
    _, b = build(auto_fail=True, settle_hold=0.5, monitor=FakeMonitor(airborne=False))
    fly(b, 3.0)
    ok &= check("not airborne cannot settle or fail", not b.has_failed)

    print("\n" + ("all checks passed" if ok else "FAILURES ABOVE"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
