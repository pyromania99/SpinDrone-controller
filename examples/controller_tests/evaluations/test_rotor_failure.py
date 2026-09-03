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

# desaturation.py lives in the extension but imports nothing from Pegasus or
# Isaac, so it is loaded by file path — importing it as a package member would
# pull in the whole extension and require the simulator.
import importlib.util as _ilu
_DESAT = (Path(__file__).resolve().parents[3] / "extensions" / "pegasus.simulator"
          / "pegasus" / "simulator" / "logic" / "vehicles" / "desaturation.py")
_spec = _ilu.spec_from_file_location("desaturation", _DESAT)
desaturation = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(desaturation)
desaturate = desaturation.desaturate


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

    ok &= desaturation_checks()

    print("\n" + ("all checks passed" if ok else "FAILURES ABOVE"))
    return 0 if ok else 1


def desaturation_checks() -> bool:
    """The mixer's saturation policy — see extensions/.../desaturation.py."""
    print("\ndesaturation policy")
    ok = True
    LIM = 1100.0**2                      # max_rotor_velocity ** 2
    kT = 8.54858e-6

    # Below saturation the two policies must be indistinguishable, or enabling
    # "collective" would change normal flight as well as the failure case.
    s = np.array([0.3, 0.5, 0.4, 0.45]) * LIM
    u = desaturate(s, LIM, "uniform")
    c = desaturate(s, LIM, "collective")
    ok &= check("below saturation: unchanged", np.allclose(u, s) and np.allclose(c, s))
    ok &= check("below saturation: policies agree", np.allclose(u, c))

    # At saturation: uniform loses collective, collective keeps it.
    s = np.array([0.53, 2.55, 0.39, 2.69]) * LIM      # the measured failure case
    u = desaturate(s, LIM, "uniform")
    c = desaturate(s, LIM, "collective")
    ok &= check("uniform loses collective", u.mean() < 0.75 * min(s.mean(), LIM))
    ok &= check("collective preserves more thrust", c.sum() > u.sum())
    ok &= check("collective beats uniform on the 3 live rotors",
                c[[0, 1, 2]].sum() > u[[0, 1, 2]].sum())

    # Bounds must hold for both, on adversarial input.
    rng = np.random.default_rng(0)
    bounds_ok = True
    for _ in range(500):
        s = rng.uniform(0, 6, 4) * LIM
        for pol in ("uniform", "collective"):
            out = desaturate(s, LIM, pol)
            if out.min() < -1e-9 or out.max() > LIM * (1 + 1e-9):
                bounds_ok = False
    ok &= check("never negative, never above the limit (500 random cases)", bounds_ok)

    # A collective demand beyond the airframe total cannot be redistributed —
    # it must saturate to an even split rather than raise or overshoot.
    s = np.full(4, 3.0 * LIM)
    c = desaturate(s, LIM, "collective")
    ok &= check("over-ceiling demand -> all rotors at max",
                np.allclose(c, LIM))

    # Graceful degradation: a demand that only just saturates should keep some
    # spread (i.e. some moment), not collapse straight to an even split.
    s = np.array([0.55, 0.95, 0.50, 1.05]) * LIM
    c = desaturate(s, LIM, "collective")
    ok &= check("partial saturation retains some moment", c.std() > 1e-6)
    ok &= check("partial saturation keeps the mean",
                abs(c.mean() - s.mean()) < 1e-6 * LIM)

    # The headline number from the plan, in newtons.
    s = np.array([4.52, 21.77, 3.32, 22.97]) / kT / 1.0
    s = np.minimum(s, 1e12)
    u = desaturate(s, LIM, "uniform") * kT
    c = desaturate(s, LIM, "collective") * kT
    W = 1.618615 * 9.81
    ok &= check(f"live-rotor lift: uniform {u[[0,1,2]].sum():.2f} N < weight {W:.2f} N",
                u[[0, 1, 2]].sum() < W)
    ok &= check(f"live-rotor lift: collective {c[[0,1,2]].sum():.2f} N > weight {W:.2f} N",
                c[[0, 1, 2]].sum() > W)
    return ok


if __name__ == "__main__":
    raise SystemExit(main())
