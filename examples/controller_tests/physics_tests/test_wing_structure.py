"""
| File: test_wing_structure.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Tests for the wing's mass and inertia model.

The wing has no USD geometry, so its mass and inertia are computed here and added to the
airframe's rigid body at spawn. If they were wrong -- or silently zero -- the aerodynamics
would still look right while the spin-up transient and the whole yaw-axis response were
substantially off, which is the kind of error that is very hard to spot in a trajectory
plot.

These tests check the inertia integral against closed-form results for geometries where
one exists, rather than against the model's own output.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import RotatingWing, RotatingWingConfig, build_wing, build_wing_config


# =======================================================================================
# Mass
# =======================================================================================

def test_blade_mass_follows_from_areal_density():
    """Mass must be areal density times the panel's own planform area."""
    wing = build_wing(blade_areal_density_kg_m2=2.0)
    area_per_blade = wing.blade_area_m2() / wing.config.num_blades

    assert wing.blade_mass_kg() == pytest.approx(2.0 * area_per_blade, rel=1e-9)


def test_explicit_blade_mass_overrides_areal_density():
    """A weighed mass must take precedence over the density estimate."""
    wing = build_wing(blade_mass_kg=0.123, blade_areal_density_kg_m2=99.0)
    assert wing.blade_mass_kg() == pytest.approx(0.123)


def test_total_wing_mass_scales_with_blade_count():
    """Total mass is per-blade mass times blade count."""
    for blades in (1, 2, 4):
        wing = build_wing(num_blades=blades, blade_mass_kg=0.05)
        mass, _ = wing.inertia_about_body_origin()
        assert mass == pytest.approx(0.05 * blades)


def test_zero_mass_gives_zero_inertia():
    """A massless wing must contribute nothing, with no division-by-zero."""
    wing = build_wing(blade_mass_kg=0.0)
    mass, inertia = wing.inertia_about_body_origin()

    assert mass == 0.0
    np.testing.assert_array_equal(inertia, np.zeros(3))


# =======================================================================================
# Inertia, against closed-form results
# =======================================================================================

def test_yaw_inertia_matches_the_uniform_rod_formula():
    """For a constant-chord blade, I_zz must match the analytic slender-rod result.

    A uniform rod of mass m spanning R1 to R2 about an axis through the origin has

        I = m * (R2^3 - R1^3) / (3 * (R2 - R1))

    A constant-chord panel of uniform areal density is exactly that, plus the chordwise
    term c^2/12 per unit mass. Testing against the closed form -- rather than against a
    previously recorded number -- is what makes this a check of the integral itself.
    """
    root, tip, chord, mass = 0.2, 0.5, 0.1, 0.08

    cfg = RotatingWingConfig(
        num_blades=1,
        blade_azimuths_deg=[0.0],
        root_radius_m=root,
        tip_radius_m=tip,
        station_radius_m=[root, tip],
        station_chord_m=[chord, chord],
        station_pitch_deg=[10.0, 10.0],
        blade_mass_kg=mass,
        num_stations=400,
    )
    _, inertia = RotatingWing(cfg).inertia_about_body_origin()

    rod = mass * (tip ** 3 - root ** 3) / (3.0 * (tip - root))
    chordwise = mass * chord ** 2 / 12.0

    assert inertia[2] == pytest.approx(rod + chordwise, rel=1e-3)


def test_two_opposed_blades_give_twice_one_blade():
    """Inertia is additive over blades, and symmetric blades contribute equally."""
    one = RotatingWing(
        build_wing_config(num_blades=1, blade_azimuths_deg=[0.0], blade_mass_kg=0.07)
    ).inertia_about_body_origin()[1]
    two = RotatingWing(
        build_wing_config(
            num_blades=2, blade_azimuths_deg=[0.0, 180.0], blade_mass_kg=0.07
        )
    ).inertia_about_body_origin()[1]

    np.testing.assert_allclose(two, 2.0 * one, rtol=1e-9)


def test_blades_along_x_contribute_almost_nothing_to_roll_inertia():
    """Mass lying on the x-axis has no moment arm about the x-axis.

    I_xx = sum dm (y^2 + z^2), and a blade along +/-x has y = 0 except for its chord
    width. So I_xx must be far smaller than I_yy and I_zz for the default two-bladed
    layout. A model that reported them equal would be ignoring the blade azimuth.
    """
    _, inertia = build_wing(num_blades=2).inertia_about_body_origin()

    assert inertia[0] < 0.1 * inertia[1], (
        f"I_xx={inertia[0]:.5f} is not small against I_yy={inertia[1]:.5f}"
    )
    assert inertia[2] > inertia[1] > inertia[0]


def test_four_blades_are_inertially_symmetric_in_roll_and_pitch():
    """A four-blade cross must give I_xx = I_yy; two opposed blades must not.

    This is a real and important asymmetry: a two-bladed wing makes the vehicle
    inertially asymmetric about roll versus pitch, which couples into the attitude
    dynamics once the airframe is spinning.
    """
    _, two = build_wing(num_blades=2).inertia_about_body_origin()
    _, four = build_wing(num_blades=4).inertia_about_body_origin()

    assert four[0] == pytest.approx(four[1], rel=1e-9)
    assert two[0] != pytest.approx(two[1], rel=1e-3)


def test_yaw_inertia_grows_with_span():
    """Moving mass outboard must raise I_zz, as r^2 demands."""
    short = build_wing(tip_radius_m=0.35, blade_mass_kg=0.07).inertia_about_body_origin()
    long = build_wing(tip_radius_m=0.50, blade_mass_kg=0.07).inertia_about_body_origin()

    assert long[1][2] > short[1][2]
    # Equal mass, so the difference is purely the moment arm.
    assert long[0] == pytest.approx(short[0])


def test_vertical_offset_adds_to_roll_and_pitch_inertia_only():
    """A wing plane raised above the body adds m*z^2 to I_xx and I_yy, not to I_zz.

    I_zz = sum dm (x^2 + y^2) contains no z, so raising the wing must leave it untouched.
    This is the parallel-axis theorem applied one axis at a time.
    """
    offset = 0.10
    mass_kg = 0.07

    flat = build_wing(blade_z_offset_m=0.0, blade_mass_kg=mass_kg)
    raised = build_wing(blade_z_offset_m=offset, blade_mass_kg=mass_kg)

    total_mass, flat_inertia = flat.inertia_about_body_origin()
    _, raised_inertia = raised.inertia_about_body_origin()

    expected_shift = total_mass * offset ** 2

    assert raised_inertia[0] - flat_inertia[0] == pytest.approx(expected_shift, rel=1e-9)
    assert raised_inertia[1] - flat_inertia[1] == pytest.approx(expected_shift, rel=1e-9)
    assert raised_inertia[2] == pytest.approx(flat_inertia[2], rel=1e-9)


def test_inertia_is_bounded_by_point_mass_at_the_tip():
    """No inertia may exceed that of the same mass concentrated at the tip.

    sum dm r^2 <= M * R_tip^2 for any distribution within the span. A violation would
    mean the radial weighting is wrong.
    """
    wing = build_wing()
    mass, inertia = wing.inertia_about_body_origin()
    ceiling = mass * wing.config.tip_radius_m ** 2

    # The chordwise term adds a little, so allow a small margin above the pure r^2 bound.
    assert inertia[2] <= ceiling * 1.05
    assert inertia[2] > mass * wing.config.root_radius_m ** 2


# =======================================================================================
# Magnitude against the real airframe
# =======================================================================================

# Measured from the stock Iris asset, extensions/.../assets/Robots/Iris/iris.usd
IRIS_BODY_MASS_KG = 1.5
IRIS_BODY_INERTIA = np.array([0.029125, 0.029125, 0.055225])


def test_wing_inertia_is_a_significant_fraction_of_the_airframe():
    """The default wing must add tens of percent to yaw inertia, not a rounding error.

    This is the test that justifies ``apply_inertia_to_body`` defaulting to True. If the
    contribution were negligible, leaving the wing massless would be defensible; it is
    not. The assertion is deliberately two-sided -- an implausibly large value would mean
    the areal density or the radial weighting had gone wrong.
    """
    _, inertia = build_wing().inertia_about_body_origin()
    yaw_fraction = inertia[2] / IRIS_BODY_INERTIA[2]

    assert 0.10 < yaw_fraction < 1.0, (
        f"wing adds {yaw_fraction * 100:.0f}% to I_zz, which is outside the plausible band"
    )


def test_wing_mass_is_a_plausible_fraction_of_the_airframe():
    """Default panel mass must be a sane fraction of the vehicle, not grams or kilograms."""
    mass, _ = build_wing().inertia_about_body_origin()
    assert 0.02 < mass / IRIS_BODY_MASS_KG < 0.5, f"wing mass {mass:.3f} kg is implausible"


def test_inertia_can_be_disabled():
    """``apply_inertia_to_body`` must be respected by the configuration.

    The flag is consumed by the vehicle class, which needs Isaac Sim, so this checks the
    configuration carries it rather than that the body was modified.
    """
    assert build_wing_config(apply_inertia_to_body=False).apply_inertia_to_body is False
    assert build_wing_config().apply_inertia_to_body is True


def test_negative_structural_parameters_are_refused():
    """Negative mass is unphysical and must not be silently accepted."""
    with pytest.raises(ValueError):
        RotatingWingConfig(blade_areal_density_kg_m2=-1.0)
    with pytest.raises(ValueError):
        RotatingWingConfig(blade_mass_kg=-0.01)
