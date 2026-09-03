"""
| File: desaturation.py
| Description: What to do when the thrust allocation asks for more than a rotor can give.

Deliberately pure numpy, with no Pegasus or Isaac imports, so it can be loaded
by file path and unit-tested outside the simulator. See
examples/tools/evaluations/test_rotor_failure.py.

The two policies
────────────────
The allocator solves for squared rotor velocities and routinely produces a
solution some rotor cannot reach. The four commands carry two quantities at
once: their MEAN is the collective thrust, their SPREAD is the moments. A
desaturation policy is a choice about which of the two to give up.

``uniform``     Divide every squared velocity by a common factor until the peak
                fits. This preserves the thrust/torque ratio — the right choice
                in normal flight, and the historical behaviour of this
                simulator. Its cost is that ONE saturated rotor drags down
                rotors that had headroom to spare, so the collective collapses
                as collateral damage.

``collective``  Hold the mean and shrink only the deviations from it, i.e. give
                up moment before lift.

Why ``collective`` exists
─────────────────────────
Measured on a stopped-rotor experiment (examples/utils/rotor_failure.py) with
the Iris at km = 6e-7. The vehicle fell out of the sky in 0.77 s even though the
three surviving rotors can produce 31.0 N against a 15.9 N weight. At the moment
it was falling, the demand was 52.83 N collective with tau_y = 4.5 N*m:

    pinv wants, per rotor [N]:   4.52  21.77   3.32  22.97   sum 52.57
    uniform scale-down:          2.04   9.80   1.49  10.34   sum 23.67
      -> rotor 3 dead, its share discarded: 13.33 N = 0.84x weight, FALLS

    collective:                 10.34  10.34  10.34  10.34   sum 41.38
      -> rotor 3 dead, its share discarded: 31.03 N = 1.95x weight, CLIMBS

Rotor 0 had asked for 4.52 N against a 10.34 N ceiling — less than half its
limit — and ``uniform`` still cut it to 2.04 N.

Note what this is NOT: nothing here detects a failure, estimates actuator
effectiveness, or rebuilds the allocation matrix. It is a fixed rule applied on
every step of every flight, and the dead rotor is still allocated its share and
still wastes it. The survivors simply are no longer dragged down with it. That
buys altitude, not attitude — a fault-blind allocator cannot hold roll and pitch
on three rotors, so the vehicle climbs while tumbling.

The policy degrades gracefully rather than being all-or-nothing: it shrinks the
spread only as far as saturation forces. At a 34 N demand it yields
[7.02, 9.98, 6.75, 10.26], which still carries moment.
"""
from __future__ import annotations

import numpy as np


def desaturate(squared_ang_vel, max_squared: float, policy: str = "uniform"):
    """Bring squared rotor velocities within ``max_squared``.

    Args:
        squared_ang_vel: per-rotor squared angular velocity, already clamped
            non-negative by the caller.
        max_squared: ``max_rotor_velocity ** 2``.
        policy: ``"uniform"`` or ``"collective"``.

    Returns:
        The adjusted array. Never negative, never above ``max_squared``.
    """
    s = np.asarray(squared_ang_vel, dtype=float)
    if max_squared <= 0.0:
        return np.zeros_like(s)

    max_val = np.max(s) if s.size else 0.0
    if max_val < max_squared:
        # Nothing saturates: both policies leave the command untouched, so a
        # run that never saturates is identical under either.
        return s

    if policy == "uniform":
        normalize = np.maximum(max_val / max_squared, 1.0)
        return s / normalize

    if policy != "collective":
        raise ValueError(f"unknown desaturation policy {policy!r}")

    # Cap the mean first: a collective demand beyond the airframe's total
    # capability cannot be met by any redistribution, only by scaling down.
    mean = float(np.mean(s))
    if mean >= max_squared:
        return np.full_like(s, max_squared)

    # Largest uniform shrink of the deviations that brings the peak inside the
    # limit, in closed form — no iteration in the physics loop.
    #
    #   s' = mean + alpha * (s - mean),  alpha in [0, 1]
    #   peak fits  <=>  mean + alpha * max(s - mean) <= max_squared
    dev = s - mean
    peak_dev = float(np.max(dev))
    if peak_dev <= 0.0:
        return np.clip(s, 0.0, max_squared)

    alpha = float(np.clip((max_squared - mean) / peak_dev, 0.0, 1.0))
    # The lower clip can only bite when a deviation is negative enough to push a
    # rotor below zero; that costs collective, but a negative rotor speed is not
    # available to spend.
    return np.clip(mean + alpha * dev, 0.0, max_squared)
