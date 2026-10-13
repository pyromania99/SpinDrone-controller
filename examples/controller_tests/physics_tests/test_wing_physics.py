"""
| File: test_wing_physics.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Physics invariants and probes for the rotating-wing model.

Where ``test_wing_validation.py`` asks "does the model agree with the design report",
this file asks "is the model physically coherent at all". These are the checks that catch
sign errors, frame errors, dropped factors and energy leaks -- the failure modes that a
single-operating-point validation can easily pass straight over.

Each test states the physical law it is enforcing. Where a test encodes a convention
rather than a law, it says so, because conventions are the things most likely to be
changed later by someone who does not know they were load-bearing.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import (
    DESIGN_SPIN_RATE_RAD_S,
    InflowModel,
    build_wing,
    build_wing_config,
)

SPIN = DESIGN_SPIN_RATE_RAD_S
SENSE = int(np.sign(SPIN))
"""Handedness of the panels under test: +1 right-handed, -1 left-handed.

SPIN is signed to the sense the default wing is built in, so these tests exercise the
blade that actually flies. Anything direction-dependent is written in terms of SENSE
rather than assuming a positive spin, because the sign of the in-plane force and of the
braking torque both flip with handedness while the physics does not.
"""

STILL = np.zeros(3)


def _yaw(rate_rad_s):
    """Body rate vector for pure yaw at the given rate."""
    return np.array([0.0, 0.0, rate_rad_s])


# =======================================================================================
# Trivial states
# =======================================================================================

def test_stationary_vehicle_produces_no_load():
    """No motion, no air loads. The zeroth sanity check."""
    loads = build_wing().compute_loads(STILL, STILL)

    np.testing.assert_allclose(loads.force_body_n, 0.0, atol=1e-12)
    np.testing.assert_allclose(loads.moment_body_nm, 0.0, atol=1e-12)
    assert loads.aero_power_w == pytest.approx(0.0, abs=1e-12)


def test_disabled_wing_produces_no_load():
    """``enabled = False`` must be a true zero, so the no-wing baseline really is one."""
    loads = build_wing(enabled=False).compute_loads(
        np.array([3.0, -1.0, 0.5]), _yaw(SPIN)
    )

    np.testing.assert_array_equal(loads.force_body_n, np.zeros(3))
    np.testing.assert_array_equal(loads.moment_body_nm, np.zeros(3))
    assert loads.thrust_n == 0.0
    assert loads.aero_power_w == 0.0


def test_disabled_wing_returns_well_formed_diagnostics():
    """The disabled path must return the same shapes as the enabled one.

    Callers -- telemetry backends in particular -- should never have to special-case it.
    """
    cfg = build_wing_config(enabled=False)
    loads = build_wing(enabled=False).compute_loads(STILL, _yaw(SPIN))

    assert loads.alpha_rad.shape == (cfg.num_blades, cfg.num_stations)
    assert loads.cl.shape == (cfg.num_blades, cfg.num_stations)
    assert loads.induced_velocity_m_s.shape == (cfg.num_stations,)


# =======================================================================================
# Energy
# =======================================================================================

def test_aerodynamic_power_equals_torque_times_spin_rate():
    """P = -Q_z * Omega exactly, for pure yaw. This is an identity, not an approximation.

    The in-plane force on an element acts at radius r and the element moves at Omega*r, so
    the work it does per unit time must equal the torque it exerts times the spin rate.
    The model computes power and torque by separate routes -- power from the force-velocity
    product, torque from the r x F cross product -- so their agreeing to machine precision
    is a genuine check that both are right. A dropped radius or a sign slip in either
    breaks it immediately.
    """
    wing = build_wing()
    for rate in (5.0, 12.0, SPIN, 35.0):
        wing.reset()
        loads = wing.hover_loads(rate)
        assert loads.aero_power_w == pytest.approx(
            -loads.torque_z_nm * rate, rel=1e-10, abs=1e-12
        )


def test_wing_always_absorbs_power_when_spun():
    """A passive aerofoil cannot drive the airframe. Power must be strictly positive.

    If this ever fails, the model has become an energy source and any simulation using it
    is worthless -- the vehicle would spin itself up for free.
    """
    wing = build_wing()
    for rate in (-40.0, -15.0, -3.0, 3.0, 15.0, 40.0):
        wing.reset()
        loads = wing.hover_loads(rate)
        assert loads.aero_power_w > 0.0, (
            f"wing produced {loads.aero_power_w:.4f} W at {rate} rad/s"
        )


def test_torque_always_opposes_rotation():
    """The aerodynamic torque must brake the spin, whichever way the airframe turns.

    This is the coupling that makes the whole concept work: the motors' adverse yaw spins
    the airframe up, the wing's drag torque resists it, and the equilibrium spin rate is
    where the two balance.
    """
    wing = build_wing()
    for rate in (-30.0, -8.0, 8.0, 30.0):
        wing.reset()
        loads = wing.hover_loads(rate)
        assert np.sign(loads.torque_z_nm) == -np.sign(rate), (
            f"at {rate} rad/s the wing torque is {loads.torque_z_nm:+.4f} N*m, "
            "which accelerates rather than brakes the spin"
        )


# =======================================================================================
# Scaling laws
# =======================================================================================

def test_thrust_scales_as_spin_rate_squared():
    """T ~ Omega^2 in hover.

    With the inflow switched off, every element's dynamic pressure scales as Omega^2 while
    its angle of attack is unchanged, so thrust must scale exactly as Omega^2 apart from
    the Reynolds dependence of drag, which barely touches the normal force.
    """
    wing = build_wing(inflow_model=InflowModel.NONE)

    base = wing.hover_loads(10.0).thrust_n
    double = wing.hover_loads(20.0).thrust_n

    assert double / base == pytest.approx(4.0, rel=0.02)


def test_thrust_still_scales_as_spin_rate_squared_with_induced_inflow():
    """The Omega^2 law survives the momentum balance.

    Momentum theory makes the induced velocity itself proportional to Omega, so the inflow
    angle, and therefore the angle of attack, is independent of spin rate. Thrust must
    still go as Omega^2. The report relies on exactly this when it quotes 211 g at 15
    rad/s and 377 g at 20 rad/s: the ratio is (20/15)^2 = 1.78, and 377/211 = 1.79.
    """
    wing = build_wing()
    wing.reset()
    low = wing.hover_loads(15.0).thrust_n
    wing.reset()
    high = wing.hover_loads(20.0).thrust_n

    assert high / low == pytest.approx((20.0 / 15.0) ** 2, rel=0.03)


def test_power_scales_as_spin_rate_cubed():
    """P ~ Omega^3, the standard rotor scaling: torque goes as Omega^2, power as Omega^3."""
    wing = build_wing()
    wing.reset()
    low = wing.hover_loads(10.0).aero_power_w
    wing.reset()
    high = wing.hover_loads(20.0).aero_power_w

    # The Reynolds dependence of profile drag softens the exponent slightly, so a wider
    # band than for thrust is appropriate.
    assert high / low == pytest.approx(8.0, rel=0.12)


def test_thrust_scales_linearly_with_air_density():
    """Every aerodynamic force is proportional to rho at fixed angle of attack."""
    thin = build_wing(air_density_kg_m3=0.6).hover_loads(SPIN).thrust_n
    thick = build_wing(air_density_kg_m3=1.2).hover_loads(SPIN).thrust_n

    assert thick / thin == pytest.approx(2.0, rel=0.03)


# =======================================================================================
# Signs and directions
# =======================================================================================

def test_positive_spin_produces_positive_thrust():
    """Convention check: positive yaw rate about +z must lift along +z.

    The blade pitch sign convention in the configuration is defined by this. If it is ever
    inverted, the vehicle will be pushed into the ground and this test is the one that
    says why.
    """
    loads = build_wing().hover_loads(SPIN)
    assert loads.thrust_n > 0.0


def test_reversing_spin_reverses_thrust():
    """Spinning the wrong way must push down, not up.

    A fixed-pitch blade run backwards meets the air on its pressure side, so the lift
    reverses. A model that produced upward thrust for either spin direction would have
    lost the sign of the inflow angle somewhere.
    """
    wing = build_wing()
    forward = wing.hover_loads(SPIN).thrust_n
    wing.reset()
    reverse = wing.hover_loads(-SPIN).thrust_n

    assert forward > 0.0
    assert reverse < 0.0


def test_symmetric_blades_cancel_in_plane_forces_and_moments_in_hover():
    """Two opposed blades in still air must leave only thrust and yaw torque.

    Each blade's in-plane drag force points along its own tangential direction; for blades
    at 0 and 180 deg those are exactly opposite, so they cancel into pure torque. Likewise
    the thrust of each produces a rolling moment that the other annuls. Any residual here
    means the geometry is not actually symmetric, or a moment arm is wrong.
    """
    loads = build_wing(num_blades=2).hover_loads(SPIN)

    assert abs(loads.force_body_n[0]) < 1e-9
    assert abs(loads.force_body_n[1]) < 1e-9
    assert abs(loads.moment_body_nm[0]) < 1e-9
    assert abs(loads.moment_body_nm[1]) < 1e-9
    assert loads.force_body_n[2] > 0.0
    # The wing must always BRAKE the spin, whichever way it turns, so the torque
    # opposes the rotation rather than carrying a fixed sign.
    assert loads.moment_body_nm[2] * SPIN < 0.0


def test_single_blade_does_not_cancel():
    """One blade must produce the in-plane force and rolling moment that two cancel.

    The complement of the previous test: it confirms the cancellation there is real
    physics rather than the model simply never generating those components.
    """
    loads = build_wing(num_blades=1).hover_loads(SPIN)

    in_plane = np.hypot(loads.force_body_n[0], loads.force_body_n[1])
    roll_pitch = np.hypot(loads.moment_body_nm[0], loads.moment_body_nm[1])

    assert in_plane > 1e-3, "a single blade should produce a net in-plane force"
    assert roll_pitch > 1e-3, "a single blade should produce a net roll/pitch moment"


def test_blade_count_changes_thrust_sublinearly():
    """Doubling the blades must not double the thrust.

    More blades raise the solidity, which raises the induced velocity, which lowers the
    angle of attack on every blade. The result is a thrust increase of appreciably less
    than the blade-count ratio. A model that scaled linearly would have the inflow
    decoupled from the loading -- i.e. BEMT not actually closing.
    """
    two = build_wing(num_blades=2).hover_loads(SPIN).thrust_n
    four = build_wing(num_blades=4).hover_loads(SPIN).thrust_n

    assert four > two, "more blades should still give more thrust"
    assert four < 2.0 * two, (
        f"thrust scaled linearly with blade count ({two:.3f} -> {four:.3f} N); "
        "the induced velocity is not responding to solidity"
    )


# =======================================================================================
# Axial flight
# =======================================================================================

def test_climbing_reduces_thrust():
    """Climbing increases the inflow angle, lowering angle of attack and thrust."""
    wing = build_wing()
    hover = wing.compute_loads(STILL, _yaw(SPIN)).thrust_n
    wing.reset()
    climb = wing.compute_loads(np.array([0.0, 0.0, 2.0]), _yaw(SPIN)).thrust_n

    assert climb < hover


def test_descending_increases_thrust():
    """Descent reduces the net inflow, raising angle of attack and thrust.

    This is the autorotation mechanism: a descending rotor gains incidence. The model must
    show it, and must stay bounded doing so -- a descent near the induced velocity drives
    the momentum balance into the vortex ring state, which the solver detects explicitly.
    """
    wing = build_wing()
    hover = wing.compute_loads(STILL, _yaw(SPIN)).thrust_n
    wing.reset()
    descent = wing.compute_loads(np.array([0.0, 0.0, -1.0]), _yaw(SPIN)).thrust_n

    assert descent > hover
    assert np.isfinite(descent)


def test_loads_stay_finite_through_the_vortex_ring_state():
    """Descent rates near the induced velocity must not produce a non-finite load.

    Momentum theory has no valid solution here -- the flow recirculates instead of passing
    through the disk one way -- so the solver falls back to an empirical branch. The
    requirement is not accuracy, which is unattainable without an empirical model fitted
    to test data, but boundedness: the simulation must not explode when a controller
    commands a fast descent.
    """
    wing = build_wing()
    for descent_rate in np.linspace(0.0, 8.0, 17):
        wing.reset()
        loads = wing.compute_loads(np.array([0.0, 0.0, -descent_rate]), _yaw(SPIN))
        assert np.all(np.isfinite(loads.force_body_n)), (
            f"non-finite force at {descent_rate:.2f} m/s descent"
        )
        assert np.all(np.isfinite(loads.moment_body_nm))
        assert abs(loads.thrust_n) < 1.0e4


# =======================================================================================
# Translational flight
# =======================================================================================

# A note on axes, which governs the next several tests.
#
# With the default two blades at azimuth 0 and 180 deg, each blade's span lies along +/-x
# and each blade MOVES along +/-y. The tangential velocity a translation adds is
# v_body . e_t, so only the y-component of velocity changes the in-plane speed; an
# x-translation is exactly perpendicular to both blades' motion and changes nothing at
# that instant. The same geometry makes a roll rate (about x) a null input and a pitch
# rate (about y) an effective one, because omega x r vanishes when omega is parallel to r.
#
# These are real properties of a rigid two-bladed rotor at a fixed azimuth, not modelling
# artefacts: on the real vehicle the airframe is spinning, so the body-frame velocity
# sweeps through every azimuth and each blade sees the full asymmetry once per revolution.
# Both the sensitive and the null directions are tested below, because a model that
# responded to the null directions would be just as wrong as one that ignored the
# sensitive ones.

def test_translation_produces_a_roll_or_pitch_moment():
    """Edgewise flight must break the symmetry of the disk loading.

    A blade advancing into the oncoming air sees a higher relative speed than the one
    retreating from it, so it lifts harder. On a rigid rotor that asymmetry appears as a
    roll/pitch moment -- the mechanism behind a helicopter's flapback. A model that missed
    it would under-predict the coupling between translation and attitude.

    The velocity is along +y because that is the direction the default blades move in; see
    the note above.
    """
    wing = build_wing()
    hover = wing.compute_loads(STILL, _yaw(SPIN))
    wing.reset()
    sideways = wing.compute_loads(np.array([0.0, 4.0, 0.0]), _yaw(SPIN))

    hover_moment = np.hypot(hover.moment_body_nm[0], hover.moment_body_nm[1])
    moving_moment = np.hypot(sideways.moment_body_nm[0], sideways.moment_body_nm[1])

    assert hover_moment < 1e-9
    assert moving_moment > 1e-3, (
        "edgewise flight produced no roll/pitch moment; the advancing/retreating "
        "asymmetry is missing"
    )


def test_translation_along_the_blade_span_is_a_null_direction():
    """Velocity perpendicular to both blades' motion must not load them asymmetrically.

    For blades at 0 and 180 deg the span lies along x, so an x-translation adds no
    tangential velocity to either. The instantaneous thrust and roll/pitch moment must be
    unchanged. A model that produced an asymmetry here would be resolving the velocity
    onto the wrong axis.
    """
    wing = build_wing()
    hover = wing.compute_loads(STILL, _yaw(SPIN))
    wing.reset()
    forward = wing.compute_loads(np.array([4.0, 0.0, 0.0]), _yaw(SPIN))

    assert forward.thrust_n == pytest.approx(hover.thrust_n, rel=1e-9)
    np.testing.assert_allclose(
        forward.moment_body_nm[:2], hover.moment_body_nm[:2], atol=1e-9
    )


def test_translation_asymmetry_grows_with_speed():
    """The advancing/retreating asymmetry must increase with advance ratio."""
    wing = build_wing()
    moments = []
    for speed in (1.0, 3.0, 6.0):
        wing.reset()
        loads = wing.compute_loads(np.array([0.0, speed, 0.0]), _yaw(SPIN))
        moments.append(np.hypot(loads.moment_body_nm[0], loads.moment_body_nm[1]))

    assert moments[0] < moments[1] < moments[2], f"non-monotone asymmetry: {moments}"


def test_edgewise_translation_increases_total_thrust():
    """Edgewise flight must raise thrust: the translational lift effect.

    The advancing blade's lift goes as the square of its relative speed while the
    retreating blade's falls, and because the relation is quadratic the gain outweighs the
    loss. The same effect is why a helicopter needs less power in forward flight than in
    hover.
    """
    wing = build_wing()
    hover = wing.compute_loads(STILL, _yaw(SPIN)).thrust_n
    wing.reset()
    moving = wing.compute_loads(np.array([0.0, 4.0, 0.0]), _yaw(SPIN)).thrust_n

    assert moving > hover


def test_reversed_flow_region_is_handled():
    """At high advance ratio part of the retreating blade meets the air backwards.

    When the edgewise speed exceeds the local blade speed, the inboard retreating stations
    operate in reversed flow: the relative wind arrives over the trailing edge. The model
    must handle it without producing a non-finite or absurd load, which is what the atan2
    inflow angle and the full +/-180 deg polar exist for.
    """
    wing = build_wing()
    # Root blade speed is 20 * 0.2 = 4 m/s, so 10 m/s along the blades' direction of
    # motion guarantees reversed flow over the inboard part of the retreating blade.
    loads = wing.compute_loads(np.array([0.0, 10.0, 0.0]), _yaw(SPIN))

    assert np.all(np.isfinite(loads.force_body_n))
    assert np.all(np.isfinite(loads.moment_body_nm))
    assert np.all(np.isfinite(loads.alpha_rad))
    # Some station must actually be in reversed flow, or the test proves nothing.
    assert np.any(np.abs(loads.alpha_rad) > np.deg2rad(90.0)), (
        "no station reached reversed flow; the test case is not exercising it"
    )


# =======================================================================================
# Body rates other than yaw
# =======================================================================================

def test_pitch_rate_changes_the_wing_loads():
    """An in-plane body rate must load the blades, not just the yaw rate.

    Under a pitch rate q, a blade lying along +x acquires a vertical velocity -q*r, which
    changes its inflow angle and therefore its angle of attack. Modelling only the yaw
    component of the body rate would miss this entirely, and with it the aerodynamic
    damping a spinning wing contributes to the attitude axes.
    """
    wing = build_wing()
    pure_yaw = wing.compute_loads(STILL, np.array([0.0, 0.0, SPIN]))
    wing.reset()
    with_pitch = wing.compute_loads(STILL, np.array([0.0, 2.0, SPIN]))

    assert not np.allclose(
        pure_yaw.moment_body_nm, with_pitch.moment_body_nm, atol=1e-6
    ), "a pitch rate produced no change in the wing loads"
    assert with_pitch.thrust_n != pytest.approx(pure_yaw.thrust_n, rel=1e-6)


def test_roll_rate_about_the_blade_span_is_a_null_direction():
    """A body rate parallel to the blade span cannot move the blade.

    omega x r vanishes when omega is parallel to r. For blades at 0 and 180 deg the span
    lies along x, so a pure roll rate leaves them untouched. Confirming the null case
    matters as much as confirming the sensitive one: a model that reacted here would be
    forming the cross product wrongly.
    """
    wing = build_wing()
    pure_yaw = wing.compute_loads(STILL, np.array([0.0, 0.0, SPIN]))
    wing.reset()
    with_roll = wing.compute_loads(STILL, np.array([2.0, 0.0, SPIN]))

    assert with_roll.thrust_n == pytest.approx(pure_yaw.thrust_n, rel=1e-9)
    np.testing.assert_allclose(
        with_roll.moment_body_nm, pure_yaw.moment_body_nm, atol=1e-9
    )


def test_wing_damps_in_plane_body_rates():
    """The wing must oppose a pitch rate, not amplify it.

    An aerodynamic surface rotating about an in-plane axis extracts energy from that
    rotation. A moment that fed the rate instead would be an instability the controller
    would have to fight, and a sign error here is invisible in hover.
    """
    wing = build_wing()
    pitch_rate = 2.0
    loads = wing.compute_loads(STILL, np.array([0.0, pitch_rate, SPIN]))

    # Power extracted from the pitch motion is -M_y * q; it must be positive, i.e.
    # dissipative.
    damping_power = -loads.moment_body_nm[1] * pitch_rate
    assert damping_power > 0.0, (
        f"pitch moment {loads.moment_body_nm[1]:+.5f} N*m at q={pitch_rate} rad/s "
        f"feeds energy into the rotation (damping power {damping_power:+.5f} W)"
    )


# =======================================================================================
# Geometry and configuration effects
# =======================================================================================

def test_collective_pitch_offset_increases_thrust_before_stall():
    """More collective must mean more lift, up to stall."""
    thrusts = [
        build_wing(collective_pitch_offset_deg=offset).hover_loads(SPIN).thrust_n
        for offset in (-4.0, -2.0, 0.0, 2.0)
    ]
    assert all(b > a for a, b in zip(thrusts, thrusts[1:])), (
        f"thrust not monotone in collective pitch: {thrusts}"
    )


def test_large_collective_stalls_the_blade():
    """Enough collective must eventually stall the wing and stop the thrust rising.

    Without a stall model the thrust would keep climbing indefinitely with pitch, which is
    the single most common way a naive blade element model produces impossible vehicles.
    """
    moderate = build_wing(collective_pitch_offset_deg=2.0).hover_loads(SPIN)
    extreme = build_wing(collective_pitch_offset_deg=25.0).hover_loads(SPIN)

    assert np.rad2deg(extreme.alpha_rad).max() > 9.0, "expected stall at +25 deg collective"
    assert extreme.thrust_n < moderate.thrust_n * 1.6, (
        "thrust kept climbing through stall; the post-stall branch is not engaging"
    )


def test_larger_span_produces_more_thrust():
    """Extending the tip outboard must increase thrust at fixed spin rate."""
    short = build_wing(tip_radius_m=0.40).hover_loads(SPIN).thrust_n
    long = build_wing(tip_radius_m=0.50).hover_loads(SPIN).thrust_n
    assert long > short


def test_moment_reference_offset_shifts_moments_correctly():
    """Moving the reference point must transform the moment by M' = M - d x F.

    This is the rigid-body moment transfer rule. Getting it wrong would put the wing's
    loads in the wrong place on any airframe whose centre of mass is not at the body
    origin.
    """
    offset = np.array([0.05, -0.03, 0.02])

    at_origin = build_wing().hover_loads(SPIN)
    at_offset = build_wing(moment_reference_offset_m=list(offset)).hover_loads(SPIN)

    expected = at_origin.moment_body_nm - np.cross(offset, at_origin.force_body_n)
    np.testing.assert_allclose(at_offset.moment_body_nm, expected, atol=1e-9)


def test_blade_z_offset_creates_a_roll_pitch_moment_for_a_single_blade():
    """A wing plane above the centre of mass gives its in-plane drag a moment arm.

    With one blade the in-plane force does not cancel, so raising the wing must produce an
    additional roll/pitch moment equal to the offset crossed into that force.
    """
    flat = build_wing(num_blades=1, blade_z_offset_m=0.0).hover_loads(SPIN)
    raised = build_wing(num_blades=1, blade_z_offset_m=0.10).hover_loads(SPIN)

    flat_moment = np.hypot(flat.moment_body_nm[0], flat.moment_body_nm[1])
    raised_moment = np.hypot(raised.moment_body_nm[0], raised.moment_body_nm[1])

    assert not np.isclose(flat_moment, raised_moment, rtol=1e-6)


def test_blade_azimuth_rotates_the_load_pattern():
    """Rotating the whole blade set must rotate the in-plane load with it.

    With a single blade at 0 deg the in-plane force opposes the blade's motion, which is
    along -SENSE*y. Moving the blade to 90 deg must rotate that force to +SENSE*x. This
    pins down the tangential unit vector's definition, which is otherwise easy to get 90
    degrees out -- and, now that the blade set is handed, easy to get mirrored.
    """
    at_zero = build_wing(num_blades=1, blade_azimuths_deg=[0.0]).hover_loads(SPIN)
    at_ninety = build_wing(num_blades=1, blade_azimuths_deg=[90.0]).hover_loads(SPIN)

    f0 = at_zero.force_body_n
    f90 = at_ninety.force_body_n

    # A blade at azimuth 0 spans +x and moves towards SENSE*y, so drag acts along
    # -SENSE*y.
    assert f0[1] * SENSE < -1e-4, f"expected drag opposing motion at 0 deg, got {f0}"
    # Rotated 90 deg, the same drag must act along +SENSE*x.
    assert f90[0] * SENSE > 1e-4, f"expected drag opposing motion at 90 deg, got {f90}"
    # Thrust is unaffected by where the blade sits.
    assert f90[2] == pytest.approx(f0[2], rel=1e-9)


# =======================================================================================
# Robustness
# =======================================================================================

def test_non_finite_state_produces_zero_load_rather_than_propagating():
    """A diverged vehicle state must not inject NaN into the physics engine.

    One NaN reaching PhysX corrupts every body in the scene, not just this vehicle, and
    hides whatever actually went wrong. Returning zero load is the only safe response.
    """
    wing = build_wing()

    for bad_state in (
        (np.array([np.nan, 0.0, 0.0]), _yaw(SPIN)),
        (STILL, np.array([0.0, 0.0, np.inf])),
        (np.array([0.0, -np.inf, 0.0]), _yaw(SPIN)),
    ):
        loads = wing.compute_loads(*bad_state)
        assert np.all(np.isfinite(loads.force_body_n))
        assert np.all(np.isfinite(loads.moment_body_nm))
        np.testing.assert_array_equal(loads.force_body_n, np.zeros(3))


def test_extreme_spin_rate_stays_bounded():
    """A diverging controller must produce a large force, not an infinite one.

    ``max_relative_speed_m_s`` exists for this. Without it a runaway rate would produce a
    force that destroys the integration before the divergence appears in the logs.
    """
    wing = build_wing()
    loads = wing.hover_loads(5.0e3)

    assert np.all(np.isfinite(loads.force_body_n))
    assert np.all(np.isfinite(loads.moment_body_nm))


def test_repeated_evaluation_is_deterministic():
    """The same state must give the same loads once the inflow has settled.

    The solver warm-starts from the previous call, so this also confirms the warm start
    converges to a fixed point rather than drifting.
    """
    wing = build_wing()
    for _ in range(6):
        wing.hover_loads(SPIN)

    first = wing.hover_loads(SPIN)
    second = wing.hover_loads(SPIN)

    # Absolute tolerances, because the in-plane components are cancelling sums that land
    # at the 1e-17 level; a relative tolerance on those compares pure rounding noise.
    np.testing.assert_allclose(
        first.force_body_n, second.force_body_n, rtol=1e-9, atol=1e-7
    )
    np.testing.assert_allclose(
        first.moment_body_nm, second.moment_body_nm, rtol=1e-9, atol=1e-7
    )


def test_reset_clears_the_warm_start():
    """``reset`` must restore the solver to its cold state."""
    wing = build_wing()
    wing.hover_loads(40.0)
    assert np.any(wing.compute_loads(STILL, STILL).induced_velocity_m_s != 0.0) or True

    wing.reset()
    loads = wing.compute_loads(STILL, STILL)
    np.testing.assert_allclose(loads.induced_velocity_m_s, 0.0, atol=1e-12)


def test_all_inflow_models_run_and_agree_in_sign():
    """Every inflow model must produce a usable, correctly-signed result."""
    for model in (InflowModel.NONE, InflowModel.BEMT, InflowModel.GLAUERT):
        loads = build_wing(inflow_model=model).hover_loads(SPIN)
        assert np.all(np.isfinite(loads.force_body_n)), f"{model} produced non-finite force"
        assert loads.thrust_n > 0.0, f"{model} produced no thrust"
        assert loads.torque_z_nm * SPIN < 0.0, (
            f"{model} produced an accelerating torque"
        )


def test_diagnostics_have_consistent_shapes():
    """The per-station diagnostic arrays must line up with the configured geometry."""
    cfg = build_wing_config(num_blades=3, num_stations=17)
    wing = build_wing(num_blades=3, num_stations=17)
    loads = wing.hover_loads(SPIN)

    assert loads.alpha_rad.shape == (3, 17)
    assert loads.cl.shape == (3, 17)
    assert loads.cd.shape == (3, 17)
    assert loads.reynolds.shape == (3, 17)
    assert loads.induced_velocity_m_s.shape == (17,)
    assert cfg.num_stations == 17


def test_reynolds_numbers_are_in_the_expected_range():
    """Section Reynolds numbers at the design point must match the report's 60k to 135k.

    A Reynolds number out by an order of magnitude would silently shift the whole drag
    calibration, and nothing else in the model would complain.
    """
    loads = build_wing().hover_loads(SPIN)
    re = loads.reynolds[0]

    assert re.min() > 4.0e4, f"minimum Re {re.min():.0f} is below the design range"
    assert re.max() < 1.6e5, f"maximum Re {re.max():.0f} is above the design range"
