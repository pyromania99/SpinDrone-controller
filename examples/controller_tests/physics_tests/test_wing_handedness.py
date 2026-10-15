"""
| File: test_wing_handedness.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Tests that the panels are built to lift in the direction the airframe spins.

WHY THIS FILE EXISTS
--------------------
A rotor blade is a handed object. Mirror it and it still lifts, but only when turned the
other way. Spin it the wrong way and the aerofoil meets the air at a negative angle of
attack and pushes DOWN.

This was a real defect, not a hypothetical one. The panels were first built right-handed,
matching the design report's conventional rotor, while this airframe spins negative: all
four motors turn the same way, so their reaction torques add instead of cancelling. In
simulation the vehicle settled at about -27 rad/s and the wing produced -2.2 N -- two
newtons of downforce on top of 133 g of added mass, a wing that was purely a cost.

Nothing in the suite caught it, because every test evaluated the wing at a positive spin
rate. The model was right; the configuration described a blade nobody could fly. So these
tests check the three things that failure needed:

1. Mirroring really is an exact symmetry, so handedness costs nothing in performance.
2. The configured handedness produces LIFT at the spin the airframe actually reaches.
3. The configured handedness still agrees with the motor directions in the thrust curve,
   which is the fact the whole thing hangs on.
"""

from __future__ import annotations

import re

import numpy as np
import pytest

from _context import (
    DESIGN_SPIN_MAGNITUDE_RAD_S,
    DESIGN_SPIN_RATE_RAD_S,
    REPO_ROOT,
    RotatingWing,
    RotatingWingConfig,
    build_wing,
    build_wing_config,
)

# Measured in simulation: the body yaw rate the stock Iris settles at with the fork's
# default motor moment (6e-7), from the eigen sweep's recorded yaw_rate telemetry.
MEASURED_BODY_SPIN_RAD_S = -25.4


# =======================================================================================
# Mirroring is an exact symmetry
# =======================================================================================

def test_mirrored_blade_matches_the_original_turned_the_other_way():
    """A left-handed blade at -Omega must equal a right-handed blade at +Omega, exactly.

    This is the claim that makes handedness a free choice: reflecting the blade through a
    plane containing the spin axis leaves the aerofoil upright and the section geometry
    untouched, so thrust, torque magnitude and power are identical. If this failed, the
    mirror would be changing the blade rather than just re-orienting it -- which is what
    negating the PITCH would do, since a cambered section flown inverted is a different
    aerofoil.
    """
    right = RotatingWing(build_wing_config(rotation_sense=+1))
    left = RotatingWing(build_wing_config(rotation_sense=-1))

    rhs = right.hover_loads(+DESIGN_SPIN_MAGNITUDE_RAD_S)
    lhs = left.hover_loads(-DESIGN_SPIN_MAGNITUDE_RAD_S)

    assert lhs.thrust_n == pytest.approx(rhs.thrust_n, rel=1e-12)
    assert lhs.aero_power_w == pytest.approx(rhs.aero_power_w, rel=1e-12)
    # The braking torque has equal magnitude and opposite sign: each opposes its own spin.
    assert lhs.torque_z_nm == pytest.approx(-rhs.torque_z_nm, rel=1e-12)
    np.testing.assert_allclose(lhs.alpha_rad, rhs.alpha_rad, rtol=1e-12)
    np.testing.assert_allclose(lhs.cl, rhs.cl, rtol=1e-12)


def test_mirroring_does_not_change_the_angle_of_attack_distribution():
    """The sections must see the same flow either way; only the frame is reflected."""
    right = RotatingWing(build_wing_config(rotation_sense=+1))
    left = RotatingWing(build_wing_config(rotation_sense=-1))

    rhs = right.hover_loads(+DESIGN_SPIN_MAGNITUDE_RAD_S)
    lhs = left.hover_loads(-DESIGN_SPIN_MAGNITUDE_RAD_S)

    np.testing.assert_allclose(
        lhs.induced_velocity_m_s, rhs.induced_velocity_m_s, rtol=1e-12
    )
    np.testing.assert_allclose(lhs.reynolds, rhs.reynolds, rtol=1e-12)


# =======================================================================================
# The configured handedness is the right one
# =======================================================================================

def test_default_panels_lift_at_the_spin_the_airframe_actually_reaches():
    """The headline test: lift, not downforce, at the measured body spin rate.

    MEASURED_BODY_SPIN_RAD_S is telemetry, not a model output -- the yaw rate the vehicle
    settled at in simulation. Evaluating there is the whole point: a wing validated only
    at its design point can still be mounted backwards on the vehicle that carries it.
    """
    loads = build_wing().hover_loads(MEASURED_BODY_SPIN_RAD_S)

    assert loads.thrust_n > 0.0, (
        f"the default wing produces {loads.thrust_n:.3f} N at the airframe's measured "
        f"spin of {MEASURED_BODY_SPIN_RAD_S} rad/s -- it is mounted the wrong way round"
    )
    # Worth having and worth stating: at least a tenth of the 1.5 kg airframe's weight.
    assert loads.thrust_n > 0.1 * 1.5 * 9.80665


def test_wrong_handedness_produces_downforce():
    """The failure mode itself, asserted directly so it cannot come back silently."""
    wrong = build_wing(rotation_sense=-build_wing_config().rotation_sense)
    loads = wrong.hover_loads(MEASURED_BODY_SPIN_RAD_S)

    assert loads.thrust_n < 0.0, (
        "a wing of the wrong handedness must push DOWN; if this passes with positive "
        "thrust the sign convention has drifted"
    )


def test_the_wing_brakes_the_spin_whichever_handedness_it_has():
    """Drag opposes motion regardless of how the blade is built.

    Handedness decides the sign of the LIFT, not of the torque: an aerofoil dragged
    through air always resists. A model where the wrong-handed wing accelerated the spin
    would be creating energy.
    """
    for sense in (+1, -1):
        for spin in (+DESIGN_SPIN_MAGNITUDE_RAD_S, -DESIGN_SPIN_MAGNITUDE_RAD_S):
            loads = build_wing(rotation_sense=sense).hover_loads(spin)
            assert loads.torque_z_nm * spin < 0.0, (
                f"sense={sense:+d} at {spin:+.1f} rad/s produced an accelerating torque"
            )
            assert loads.aero_power_w > 0.0, (
                f"sense={sense:+d} at {spin:+.1f} rad/s generated power instead of "
                f"absorbing it"
            )


def test_design_spin_rate_used_by_the_suite_follows_the_configured_handedness():
    """The suite must validate the blade that actually flies.

    If DESIGN_SPIN_RATE_RAD_S were pinned positive while the shipped panels were built
    left-handed, every validation test would be checking a mirror image of the real wing
    -- which is exactly how the original defect survived.
    """
    assert np.sign(DESIGN_SPIN_RATE_RAD_S) == np.sign(build_wing_config().rotation_sense)
    assert abs(DESIGN_SPIN_RATE_RAD_S) == pytest.approx(DESIGN_SPIN_MAGNITUDE_RAD_S)


# =======================================================================================
# Agreement with the airframe
# =======================================================================================

def _default_rotor_directions():
    """Read the thrust curve's default ``rot_dir`` out of its source.

    The module cannot be imported here: ``pegasus.simulator.__init__`` pulls in Omniverse,
    and keeping this suite runnable without Isaac Sim is worth more than the convenience.
    Reading the literal is enough, because the literal is precisely the thing that must
    not change without the wing's handedness changing with it.
    """
    source = open(
        f"{REPO_ROOT}/extensions/pegasus.simulator/pegasus/simulator/logic/"
        "thrusters/quadratic_thrust_curve.py"
    ).read()
    match = re.search(r'rot_dir["\']\s*,\s*\[([-\d,\s]+)\]', source)
    assert match, "could not find the default rot_dir in quadratic_thrust_curve.py"
    return [int(token) for token in match.group(1).split(",")]


def test_panel_handedness_agrees_with_the_motor_directions():
    """The wing's handedness must match the yaw sense the motors actually drive.

    The airframe spins because every motor turns the same way, so the reaction torques
    add rather than cancel. The applied yaw torque is

        tau_z = sum_i c_i * omega_i^2 * rot_dir_i

    with every c_i and omega_i^2 positive, so the steady spin takes the sign of the
    rot_dir sum. The panels must be built to lift in that sense.

    This is the test that would have caught the original defect in one second rather than
    after a full simulation run.
    """
    rot_dir = _default_rotor_directions()

    assert len(set(rot_dir)) == 1, (
        f"rot_dir={rot_dir} mixes directions, so this is a conventional quadrotor that "
        f"does not spin; the spinning-wing premise no longer holds"
    )

    airframe_spin_sense = int(np.sign(sum(rot_dir)))
    configured = build_wing_config().rotation_sense

    assert configured == airframe_spin_sense, (
        f"panels are built for a {configured:+d} spin but the airframe's motors "
        f"(rot_dir={rot_dir}) drive it to a {airframe_spin_sense:+d} spin. The wing will "
        f"push DOWN. Set ROTATION_SENSE = {airframe_spin_sense} in "
        f"examples/utils/wing_params.py."
    )


# =======================================================================================
# Schema
# =======================================================================================

def test_rotation_sense_must_be_plus_or_minus_one():
    """It is a handedness, not a scale factor, so only two values are meaningful."""
    for bad in (0, 2, -3, 0.5):
        with pytest.raises(ValueError):
            RotatingWingConfig(rotation_sense=bad)


def test_both_handednesses_are_accepted():
    """Both must build, so the wrong-handed wing stays available as an ablation."""
    assert RotatingWingConfig(rotation_sense=+1).rotation_sense == +1
    assert RotatingWingConfig(rotation_sense=-1).rotation_sense == -1
