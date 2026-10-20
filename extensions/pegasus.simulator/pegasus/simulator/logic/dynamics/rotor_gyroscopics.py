"""
| File: rotor_gyroscopics.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Gyroscopic reaction of spinning propellers on the airframe.

WHY THIS EXISTS FOR THIS AIRFRAME AND NOT FOR A NORMAL QUADROTOR
----------------------------------------------------------------
A conventional quadrotor sets ``rot_dir = [-1, -1, +1, +1]``. The four propeller angular
momenta then cancel to essentially zero and neglecting them is harmless, which is why
upstream Pegasus neglects them.

This airframe spins *because* every motor turns the same way,
``rot_dir = [-1, -1, -1, -1]``, so the four momenta **add**. At hover they total about
0.209 kg*m^2/s -- 16 percent of the whole vehicle's angular momentum -- and the resulting
gyroscopic moment exceeds the rotating wing's entire aerodynamic moment above roughly
1.5 rad/s of body rate.

SIGN CONVENTION
---------------
Pegasus applies ``sum_i c_i omega_i^2 rot_dir_i`` to the body as a yaw torque. A rotor
whose contribution is negative is therefore reacting against a propeller spinning in the
POSITIVE sense: the motor drives the propeller one way and the airframe feels the opposite.
Hence the propeller momentum carries the opposite sign to ``rot_dir``.

This module deliberately imports nothing but NumPy, so the physics test suite can exercise
it without Isaac Sim.
"""

from __future__ import annotations

import numpy as np

__all__ = ["rotor_angular_momentum_z", "rotor_gyroscopic_torque"]


def rotor_angular_momentum_z(
    rotor_velocities_rad_s, rotation_directions, rotor_inertia_kg_m2: float
) -> float:
    """Net propeller angular momentum about the body +z axis [kg*m^2/s].

    Args:
        rotor_velocities_rad_s: Per-rotor angular speeds [rad/s], non-negative.
        rotation_directions: Per-rotor ``rot_dir``, each +1 or -1, in the sense Pegasus
            uses for the reaction torque on the body.
        rotor_inertia_kg_m2: Polar moment of inertia of one rotor about its spin axis.

    Returns:
        The net angular momentum. Zero for a conventional quadrotor whose directions
        cancel, and maximal when every rotor turns the same way.

    Raises:
        ValueError: If the inputs differ in length, or the inertia is negative.
    """
    omega = np.asarray(rotor_velocities_rad_s, dtype=float).reshape(-1)
    direction = np.asarray(rotation_directions, dtype=float).reshape(-1)

    if omega.size != direction.size:
        raise ValueError(
            f"got {omega.size} rotor velocities and {direction.size} rotation directions"
        )
    if rotor_inertia_kg_m2 < 0.0:
        raise ValueError("rotor_inertia_kg_m2 must be >= 0")

    return float(-rotor_inertia_kg_m2 * np.sum(omega * direction))


def rotor_gyroscopic_torque(
    rotor_velocities_rad_s,
    rotation_directions,
    rotor_inertia_kg_m2: float,
    angular_velocity_body_rad_s,
) -> np.ndarray:
    """Gyroscopic moment the propellers exert on the airframe [N*m], body frame.

        h      = [0, 0, sum_i I omega_i * (-rot_dir_i)]
        M_gyro = -omega_body x h

    The minus sign is the reaction on the airframe: precessing the rotor disc takes a
    moment, and the airframe feels its equal and opposite.

    Args:
        rotor_velocities_rad_s: Per-rotor angular speeds [rad/s].
        rotation_directions: Per-rotor ``rot_dir``.
        rotor_inertia_kg_m2: Polar moment of inertia of one rotor.
        angular_velocity_body_rad_s: Body rates [p, q, r] in the body frame [rad/s].

    Returns:
        Array (3,) body-frame moment. Non-finite input returns zeros rather than
        propagating NaN into the physics engine.
    """
    omega_body = np.asarray(angular_velocity_body_rad_s, dtype=float).reshape(3)
    if not np.all(np.isfinite(omega_body)):
        return np.zeros(3)

    h_z = rotor_angular_momentum_z(
        rotor_velocities_rad_s, rotation_directions, rotor_inertia_kg_m2
    )
    if not np.isfinite(h_z):
        return np.zeros(3)

    # -omega x [0,0,h_z] worked out explicitly: the z component is identically zero, so
    # the propellers never contribute a yaw moment -- only roll/pitch cross-coupling.
    p, q, _ = omega_body
    return np.array([-q * h_z, p * h_z, 0.0])
