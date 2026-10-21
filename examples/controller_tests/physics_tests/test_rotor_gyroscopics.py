"""
| File: test_rotor_gyroscopics.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Tests for the propellers' gyroscopic reaction on the airframe.

WHY THIS TERM WAS MISSING, AND WHY IT MATTERS HERE
--------------------------------------------------
Upstream Pegasus neglects propeller angular momentum, and for a conventional quadrotor
that is correct to within rounding: ``rot_dir = [-1, -1, +1, +1]`` makes the four momenta
cancel. This airframe spins because every motor turns the same way, so they add instead --
to about 16 percent of the whole vehicle's angular momentum. Above roughly 1.5 rad/s of
body rate the resulting moment exceeds the rotating wing's entire aerodynamic moment,
which makes it the largest single term that was absent from the simulation.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import (
    HOVER_ROTOR_SPEED_RAD_S,
    ROTOR_INERTIA_KG_M2,
    rotor_angular_momentum_z,
    rotor_gyroscopic_torque,
)

SPIN_DRONE = [-1, -1, -1, -1]
"""This fork: every motor the same way, so the momenta add."""

STOCK_QUAD = [-1, -1, 1, 1]
"""Upstream Pegasus: opposed pairs, so the momenta cancel."""

HOVER = [HOVER_ROTOR_SPEED_RAD_S] * 4


# =======================================================================================
# Angular momentum
# =======================================================================================

def test_same_direction_rotors_add_their_momentum():
    """The premise of the whole term: four co-rotating propellers do not cancel."""
    h = rotor_angular_momentum_z(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2)
    expected = 4.0 * ROTOR_INERTIA_KG_M2 * HOVER_ROTOR_SPEED_RAD_S
    assert h == pytest.approx(expected, rel=1e-12)
    assert h == pytest.approx(0.2093, abs=5e-4)


def test_a_conventional_quadrotor_cancels_exactly():
    """With opposed pairs the net momentum is identically zero.

    This is why upstream Pegasus can ignore the term, and the test states that explicitly
    so the reason is recorded rather than inferred.
    """
    assert rotor_angular_momentum_z(HOVER, STOCK_QUAD, ROTOR_INERTIA_KG_M2) == 0.0


def test_momentum_sign_opposes_the_reaction_direction():
    """A propeller whose body reaction is negative is itself spinning positive.

    Pegasus applies ``sum c omega^2 rot_dir`` to the body. The propeller must therefore
    carry momentum of the opposite sign. Getting this backwards would make the gyroscopic
    coupling precess the wrong way, which is a sign error no magnitude check would catch.
    """
    h_neg = rotor_angular_momentum_z(HOVER, [-1] * 4, ROTOR_INERTIA_KG_M2)
    h_pos = rotor_angular_momentum_z(HOVER, [1] * 4, ROTOR_INERTIA_KG_M2)
    assert h_neg > 0.0
    assert h_pos < 0.0
    assert h_neg == pytest.approx(-h_pos)


def test_momentum_scales_linearly_with_speed_and_inertia():
    base = rotor_angular_momentum_z(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2)
    double_speed = rotor_angular_momentum_z(
        [2 * w for w in HOVER], SPIN_DRONE, ROTOR_INERTIA_KG_M2
    )
    double_inertia = rotor_angular_momentum_z(
        HOVER, SPIN_DRONE, 2 * ROTOR_INERTIA_KG_M2
    )
    assert double_speed == pytest.approx(2 * base)
    assert double_inertia == pytest.approx(2 * base)


def test_stopped_rotors_carry_no_momentum():
    assert rotor_angular_momentum_z([0, 0, 0, 0], SPIN_DRONE, ROTOR_INERTIA_KG_M2) == 0.0


# =======================================================================================
# The gyroscopic moment itself
# =======================================================================================

def test_torque_is_the_cross_product_of_body_rate_and_momentum():
    """Checked against the closed form, not against a recorded number."""
    omega = np.array([0.7, -1.3, 2.1])
    h_z = rotor_angular_momentum_z(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2)
    expected = -np.cross(omega, np.array([0.0, 0.0, h_z]))

    got = rotor_gyroscopic_torque(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, omega)
    np.testing.assert_allclose(got, expected, rtol=1e-12, atol=1e-15)


def test_roll_rate_produces_a_pitch_moment_and_vice_versa():
    """Gyroscopic coupling is 90 degrees out: roll feeds pitch, pitch feeds roll.

    This is the signature of the effect and the reason it matters for attitude control --
    it cross-couples the two axes a controller treats as independent.
    """
    h_z = rotor_angular_momentum_z(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2)

    roll = rotor_gyroscopic_torque(
        HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, [2.0, 0.0, -18.0]
    )
    assert roll[1] == pytest.approx(2.0 * h_z)      # pitch moment from roll rate
    assert roll[0] == pytest.approx(0.0)

    pitch = rotor_gyroscopic_torque(
        HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, [0.0, 2.0, -18.0]
    )
    assert pitch[0] == pytest.approx(-2.0 * h_z)    # roll moment from pitch rate
    assert pitch[1] == pytest.approx(0.0)


def test_yaw_rate_alone_produces_no_moment():
    """The rotor axis is the yaw axis, so spinning about it cannot precess the disc.

    Important for this vehicle specifically: the airframe's own large yaw rate must NOT
    generate a spurious gyroscopic moment.
    """
    torque = rotor_gyroscopic_torque(
        HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, [0.0, 0.0, -27.0]
    )
    np.testing.assert_allclose(torque, np.zeros(3), atol=1e-15)


def test_the_moment_never_has_a_yaw_component():
    """``-omega x [0,0,h]`` has no z component for any body rate, by construction."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        omega = rng.uniform(-30, 30, 3)
        torque = rotor_gyroscopic_torque(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, omega)
        assert torque[2] == 0.0


def test_the_moment_does_no_work_on_the_body():
    """A gyroscopic moment is perpendicular to the rate, so it cannot add or remove energy.

    If this failed the term would be injecting or draining rotational energy, which would
    show up as a slow drift in spin rate that is very hard to attribute.
    """
    rng = np.random.default_rng(1)
    for _ in range(50):
        omega = rng.uniform(-30, 30, 3)
        torque = rotor_gyroscopic_torque(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, omega)
        assert abs(float(np.dot(omega, torque))) < 1e-12 * max(1.0, np.abs(torque).max())


def test_a_conventional_quadrotor_feels_no_gyroscopic_moment():
    torque = rotor_gyroscopic_torque(
        HOVER, STOCK_QUAD, ROTOR_INERTIA_KG_M2, [3.0, -2.0, 5.0]
    )
    np.testing.assert_allclose(torque, np.zeros(3), atol=1e-18)


# =======================================================================================
# Magnitude, against the thing being studied
# =======================================================================================

def test_the_moment_exceeds_the_wings_at_realistic_body_rates():
    """The finding that justifies adding this term at all.

    The rotating wing's aerodynamic moment at 4 rad/s body rate is about 0.36 N*m.
    Anything comparable that is missing from the simulation dominates the conclusions, so
    the magnitude is asserted rather than left as a note.
    """
    wing_moment_at_4_rad_s = 0.364

    torque = rotor_gyroscopic_torque(
        HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, [4.0, 0.0, -18.0]
    )
    assert np.abs(torque).max() > 2.0 * wing_moment_at_4_rad_s


def test_non_finite_state_gives_zero_rather_than_propagating():
    """A diverged state must not put NaN into the physics engine."""
    for bad in ([np.nan, 0.0, -18.0], [0.0, np.inf, -18.0]):
        torque = rotor_gyroscopic_torque(HOVER, SPIN_DRONE, ROTOR_INERTIA_KG_M2, bad)
        np.testing.assert_array_equal(torque, np.zeros(3))


# =======================================================================================
# Schema
# =======================================================================================

def test_mismatched_rotor_counts_are_refused():
    with pytest.raises(ValueError):
        rotor_angular_momentum_z([100.0, 200.0], SPIN_DRONE, ROTOR_INERTIA_KG_M2)


def test_negative_inertia_is_refused():
    with pytest.raises(ValueError):
        rotor_angular_momentum_z(HOVER, SPIN_DRONE, -1.0)
