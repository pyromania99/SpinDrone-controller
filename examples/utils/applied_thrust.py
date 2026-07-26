#!/usr/bin/env python
"""
| File: applied_thrust.py
| Description: Per-rotor thrust the simulator actually applies, for telemetry.

Why this exists
───────────────
Both controllers used to log a *reconstruction* of per-rotor thrust, rebuilt
from the collective and the torque command::

    u1 = base_thrust - tau_x + tau_y      # etc.

That reconstruction is not what the vehicle does, for three separate reasons:

1. **Saturation is a normalization, not a clip.** When any rotor exceeds
   ``max_rotor_velocity``, ``Multirotor.force_and_torques_to_velocities``
   divides *every* squared velocity by the same factor, preserving the
   thrust/torque ratio and lowering total thrust. Per-motor clipping —
   which is what the reconstructions modelled — changes the ratio instead,
   so it produces a different attitude response as well as a different
   magnitude.

2. **The bounds were wrong.** EigenController clipped each rotor at
   ``max_total_thrust`` (80 N, the *total*), roughly 8x the real per-rotor
   limit. CascadedPDController applied no upper bound at all, so its log
   contained values as large as 3.6e5 N. Comparing an effort metric built
   from an unbounded quantity against one built from an 80 N-bounded
   quantity biased every head-to-head between the two.

3. **The allocation is not the naive mixer.** The real allocation matrix is
   built from measured rotor positions and per-rotor constants and inverted
   with a pseudo-inverse; ``±tau_x ±tau_y`` is only an idealisation of it.

The applied force is instead recovered exactly, from the same rotor
velocities the controller has just handed to the vehicle::

    v_applied = clip(ang_vel, min_rotor_velocity, max_rotor_velocity)
    F_i       = rotor_constant_i * v_applied_i^2

which is line-for-line what ``QuadraticThrustCurve.update`` computes. Doing
it from the returned command keeps it synchronous — reading the thrust
curve's ``force`` property instead would return the *previous* physics
step's value, since the backend update runs before the thrusters do.
"""
from __future__ import annotations

import numpy as np


def applied_rotor_forces(vehicle, rotor_velocities) -> np.ndarray:
    """Per-rotor thrust [N] the simulator will apply for `rotor_velocities`.

    Mirrors QuadraticThrustCurve.update exactly, reading the coefficients off
    the vehicle's own thruster model so the two cannot drift apart.

    Returns zeros when there is no vehicle or no thruster model — with nothing
    attached, no force is applied, so zero is the accurate reading rather than
    a fallback standing in for a real one.
    """
    velocities = np.asarray(rotor_velocities, dtype=float).ravel()

    thrusters = getattr(vehicle, "_thrusters", None) if vehicle is not None else None
    if thrusters is None:
        return np.zeros_like(velocities)

    try:
        rotor_constant = np.asarray(thrusters._rotor_constant, dtype=float)
        v_min = np.asarray(thrusters.min_rotor_velocity, dtype=float)
        v_max = np.asarray(thrusters.max_rotor_velocity, dtype=float)
    except AttributeError:
        return np.zeros_like(velocities)

    n = min(len(velocities), len(rotor_constant))
    v_applied = np.clip(velocities[:n], v_min[:n], v_max[:n])
    return rotor_constant[:n] * np.square(v_applied)


def max_rotor_force(thrusters=None) -> float:
    """Largest thrust one rotor can produce [N] — rotor_constant * v_max^2.

    With the Iris defaults (rotor_constant 8.54858e-6, max_rotor_velocity
    1100 rad/s) this is 10.34 N, giving a thrust-to-weight of about 2.4 for
    the ~1.75 kg airframe. Passing None returns that default, which is what
    the sweep metrics use to size their saturation threshold.
    """
    if thrusters is not None:
        try:
            return float(np.max(np.asarray(thrusters._rotor_constant, dtype=float)
                                * np.square(np.asarray(thrusters.max_rotor_velocity,
                                                       dtype=float))))
        except AttributeError:
            pass
    # QuadraticThrustCurve defaults.
    return 8.54858e-6 * 1100.0 ** 2
