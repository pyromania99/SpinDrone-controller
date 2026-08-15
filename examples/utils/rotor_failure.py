#!/usr/bin/env python
"""
| File: rotor_failure.py
| Description: Stop one rotor mid-flight, on a trigger or once the vehicle settles.

Where the failure is injected
─────────────────────────────
``Multirotor.update`` reads its rotor commands from
``self._backends[0].input_reference()`` and hands them straight to
``QuadraticThrustCurve``. Zeroing one element of that vector is exactly a
stopped motor: the thrust curve computes F = k * w^2 = 0 for it, and it
contributes nothing to the rolling moment either. Nothing else has to be
touched, and the controller is not told — it keeps commanding all four and
sees the consequences only through the state, which is the point.

Why the settle test does not use a position target
──────────────────────────────────────────────────
Three of the four controllers here carry ``hover_target``; NonlinearController
does not, because it flies a trajectory. A criterion built on distance-to-target
would therefore work for some controllers and raise for others.

"Settled" is instead defined as the vehicle *holding a position*: airborne, at a
speed below ``settle_speed``, and within ``settle_radius`` of where it first
stopped, continuously for ``settle_hold`` seconds. That is what "stabilised in
its final position" means operationally, it needs no reference, and it is the
same test for every controller.

Both halves are load-bearing. Speed alone would accept a steady slow drift as a
settle, so the position anchor pins it to one place; the anchor alone would
accept a vehicle oscillating through a point, so the speed bound excludes that.
Any violation drops the anchor and the hold restarts from zero.

A fixed delay was rejected: it fires mid-transient on a slow run and long after
settling on a fast one, and those produce completely different failure
responses.
"""
from __future__ import annotations

import os
import numpy as np

# Deliberately NOT a Backend subclass, following RobustnessBackend: the base
# class pulls in the Pegasus extension, which makes the class unconstructible
# outside Isaac and so untestable. The vehicle assigns backends duck-typed, so
# the interface is what matters, not the ancestry. See test_rotor_failure.py.

# Defaults, shared so the standalone script and the interactive app agree.
SETTLE_RADIUS: float = 0.15     # m of positional spread across the window
SETTLE_SPEED: float = 0.20      # m/s
SETTLE_HOLD: float = 1.5        # s the vehicle must hold before auto-firing


class RotorFailureBackend:
    """Wraps a controller and stops one rotor, manually or on settle.

    Two ways to fire:

    ``trigger()``            immediately, e.g. from a UI button.
    ``auto_fail=True``       the first time the settle test passes.

    Both are one-shot: once a rotor is stopped it stays stopped, and a second
    call is ignored rather than moving the failure to another rotor mid-run.

    Attribute lookups that are not part of the backend interface fall through
    to the inner controller, so existing code reading e.g. ``backend.w`` or
    ``backend.hover_target`` keeps working when a controller is wrapped.
    """

    def __init__(self, inner, results_file: str = None, rotor: int = None,
                 rng=None, auto_fail: bool = True,
                 settle_radius: float = SETTLE_RADIUS,
                 settle_speed: float = SETTLE_SPEED,
                 settle_hold: float = SETTLE_HOLD,
                 monitor=None, num_rotors: int = 4, log=None):
        self._vehicle = None
        self.inner = inner
        self.results_file = results_file
        self.rng = rng if rng is not None else np.random.default_rng()
        self.num_rotors = int(num_rotors)

        if rotor is not None and not 0 <= int(rotor) < self.num_rotors:
            raise ValueError(f"rotor must be in [0, {self.num_rotors}), got {rotor}")
        self._forced_rotor = None if rotor is None else int(rotor)

        self.auto_fail = bool(auto_fail)
        self.settle_radius = float(settle_radius)
        self.settle_speed = float(settle_speed)
        self.settle_hold = float(settle_hold)

        self.monitor = monitor
        self._log = log

        self.failed_rotor = None
        self.failure_time = None
        self.settle_time = None
        self.total_time = 0.0
        self._stopped = False
        self._pending_trigger = False

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.omega = np.zeros(3)

        # Anchor for the settle test: the time and place the vehicle started
        # holding still. Settled once it has stayed within settle_radius of
        # _anchor_p, at low speed, for settle_hold seconds.
        #
        # This replaces a trailing-window implementation that could never fire:
        # it pruned the window to points newer than settle_hold and then asked
        # whether the window SPANNED settle_hold, which is false by
        # construction (the span is at most settle_hold, and in general
        # settle_hold - dt). Caught by test_rotor_failure.py.
        self._anchor_t = None
        self._anchor_p = None
        self._settled = False

        self.t_hist, self.pos_hist = [], []
        self.vel_hist, self.w_hist, self.cmd_hist = [], [], []

    # Anything not defined here belongs to the controller being wrapped.
    def __getattr__(self, name):
        # Only reached when normal lookup fails. Guard against recursion during
        # __init__, before self.inner exists.
        if name == "inner":
            raise AttributeError(name)
        return getattr(self.__dict__["inner"], name)

    @property
    def is_settled(self) -> bool:
        """True once the vehicle has held a position for settle_hold seconds."""
        return self._settled

    def _update_settle(self):
        """Advance the hold anchor. Sets _settled and, on the first pass,
        settle_time."""
        airborne = (self.monitor is None
                    or getattr(self.monitor, "airborne", True))
        holding = (airborne
                   and float(np.linalg.norm(self.velocity)) <= self.settle_speed)
        if holding and self._anchor_p is not None:
            # Must stay near where it first stopped, not merely be slow —
            # a steady slow drift is not a settle.
            holding = bool(np.linalg.norm(self.position - self._anchor_p)
                           <= self.settle_radius)

        if not holding:
            self._anchor_t = None
            self._anchor_p = None
            self._settled = False
            return

        if self._anchor_t is None:
            self._anchor_t = self.total_time
            self._anchor_p = self.position.copy()

        self._settled = (self.total_time - self._anchor_t) >= self.settle_hold
        if self._settled and self.settle_time is None:
            self.settle_time = self._anchor_t

    @property
    def has_failed(self) -> bool:
        return self.failed_rotor is not None

    def trigger(self, rotor: int = None) -> int:
        """Stop a rotor now. Returns the rotor index, or the existing one.

        Safe to call before the first physics step: the request is latched and
        applied on the next update, so a button press while paused is not lost.
        """
        if self.has_failed:
            return self.failed_rotor
        if rotor is not None:
            self._forced_rotor = int(rotor)
        self._pending_trigger = True
        return (self._forced_rotor if self._forced_rotor is not None else -1)

    def _fire(self):
        self.failed_rotor = (
            self._forced_rotor if self._forced_rotor is not None
            else int(self.rng.integers(self.num_rotors)))
        self.failure_time = self.total_time
        self._pending_trigger = False
        if self._log is not None:
            self._log(f"[rotor-failure] *** ROTOR {self.failed_rotor} STOPPED at "
                      f"t = {self.failure_time:.3f} s ***  alt = "
                      f"{self.position[2]:.3f} m, |v| = "
                      f"{np.linalg.norm(self.velocity):.3f} m/s")

    # ── Backend interface (duck-typed; see the note at the imports) ─────────
    @property
    def vehicle(self):
        return self._vehicle

    def initialize(self, vehicle):
        self._vehicle = vehicle
        self.inner.initialize(vehicle)

    def start(self):
        self.total_time = 0.0
        self._stopped = False
        self.failed_rotor = None
        self.failure_time = None
        self.settle_time = None
        self._pending_trigger = False
        self._anchor_t = None
        self._anchor_p = None
        self._settled = False
        for h in (self.t_hist, self.pos_hist, self.vel_hist,
                  self.w_hist, self.cmd_hist):
            h.clear()
        if self.monitor is not None:
            self.monitor.reset()
        self.inner.start()

    def reset(self):
        self.inner.reset()

    def update_sensor(self, sensor_type, data):
        self.inner.update_sensor(sensor_type, data)

    def update_graphical_sensor(self, sensor_type, data):
        self.inner.update_graphical_sensor(sensor_type, data)

    def update_state(self, state):
        self.position = np.asarray(state.position, dtype=float).copy()
        self.velocity = np.asarray(state.linear_velocity, dtype=float).copy()
        self.omega = np.asarray(state.angular_velocity, dtype=float).copy()
        self.inner.update_state(state)

    def input_reference(self):
        ref = list(self.inner.input_reference())
        if self.failed_rotor is not None:
            ref[self.failed_rotor] = 0.0
        return ref

    def update(self, dt: float):
        self.total_time += dt
        self.inner.update(dt)

        self._update_settle()

        if not self.has_failed:
            if self._pending_trigger or (self.auto_fail and self._settled):
                self._fire()

        self.t_hist.append(self.total_time)
        self.pos_hist.append(self.position.copy())
        self.vel_hist.append(self.velocity.copy())
        self.w_hist.append(self.omega.copy())
        self.cmd_hist.append(np.asarray(self.input_reference(), dtype=float))

    def check_oob(self, bounds) -> bool:
        if self.monitor is None:
            return False
        return self.monitor.update(self.total_time, self.position, bounds)

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        self.inner.stop()
        if self.results_file is None:
            return

        os.makedirs(os.path.dirname(self.results_file), exist_ok=True)
        data = {}
        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file, allow_pickle=True))
            except Exception:
                data = {}

        def stack(seq, width):
            return np.vstack(seq) if seq else np.zeros((0, width))

        # Truth, recorded here rather than taken from the controller's own log:
        # the controllers log the rotor command they ISSUED, which after the cut
        # is not what the vehicle applied.
        data.update({
            "t_true": np.array(self.t_hist),
            "p_true": stack(self.pos_hist, 3),
            "v_true": stack(self.vel_hist, 3),
            "w_true": stack(self.w_hist, 3),
            "rotor_cmd_applied": stack(self.cmd_hist, self.num_rotors),
            "failed_rotor": np.array(-1 if self.failed_rotor is None
                                     else self.failed_rotor),
            "failure_time": np.array(np.nan if self.failure_time is None
                                     else self.failure_time),
            "settle_time": np.array(np.nan if self.settle_time is None
                                    else self.settle_time),
        })
        if self.monitor is not None:
            data["failure_mode"] = np.array(getattr(self.monitor, "failure", "") or "")
        np.savez(self.results_file, **data)
        if self._log is not None:
            self._log(f"[rotor-failure] telemetry → {self.results_file}")
