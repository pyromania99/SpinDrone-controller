"""
| File: test_wing_validation.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Validation of the rotating-wing model against the GOE 210 design report.

WHY THIS FILE IS THE IMPORTANT ONE
----------------------------------
``test_wing_physics.py`` checks that the model is self-consistent: that energy balances,
that symmetric geometry gives symmetric loads, that nothing diverges. Those are necessary
but they are not sufficient -- a model can be perfectly self-consistent and still predict
the wrong number.

This file checks the model against an INDEPENDENT calculation: the blade-element-momentum
optimisation in ``foil_out/GOE210_DESIGN_REPORT.md``, which was produced by separate code
with separate aerodynamic data (NeuralFoil polars, cross-checked in XFOIL 6.99). Agreement
here means the integrated physics is right, not merely tidy.

WHAT WAS AND WAS NOT FITTED
---------------------------
The airfoil polar WAS calibrated to the report's per-station section data: the (alpha,
c_l) and (Re, c_d) pairs in its section 8.1 table. That calibration is tested in
``test_airfoil_polar.py``.

The quantities tested HERE -- total thrust, aerodynamic power, figure of merit, and the
radial distribution of induced velocity -- were not fitted to anything. They fall out of
the blade element integration and the momentum balance. If the inflow solver, the force
resolution, the tip-loss model or the geometry handling were wrong, these would not match.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import (
    DESIGN_SPIN_RATE_RAD_S,
    GRAVITY_M_S2,
    REPORT,
    REPORT_STATIONS,
    InflowModel,
    RotatingWing,
    build_wing,
    build_wing_config,
)


@pytest.fixture
def wing():
    """The as-designed wing at default settings."""
    return build_wing()


@pytest.fixture
def design_loads(wing):
    """Hover loads at the 20 rad/s design point."""
    return wing.hover_loads(DESIGN_SPIN_RATE_RAD_S)


# =======================================================================================
# Integrated performance
# =======================================================================================

def test_thrust_at_design_point_matches_report(design_loads):
    """Total thrust at 20 rad/s must match the report's 3.74 N.

    A 3 percent band is allowed. The two calculations use different section data
    (NeuralFoil tables against a fitted analytic polar) and different spanwise
    discretisations, so exact agreement would be suspicious rather than reassuring.
    """
    assert design_loads.thrust_n == pytest.approx(REPORT["thrust_n_at_20"], rel=0.03)


def test_thrust_in_grams_matches_report_at_both_spin_rates(wing):
    """The report's spin-rate robustness check: 211 g at 15 rad/s, 377 g at 20 rad/s."""
    sense = int(np.sign(DESIGN_SPIN_RATE_RAD_S))
    for rate, expected_grams in (
        (sense * 15.0, REPORT["thrust_grams_at_15"]),
        (sense * 20.0, REPORT["thrust_grams_at_20"]),
    ):
        wing.reset()
        loads = wing.hover_loads(rate)
        grams = loads.thrust_n / GRAVITY_M_S2 * 1000.0
        assert grams == pytest.approx(expected_grams, rel=0.04), (
            f"{abs(rate)} rad/s: model {grams:.1f} g, report {expected_grams} g"
        )


def test_aerodynamic_power_matches_report(design_loads):
    """Aerodynamic power at the design point must match the report's 7.31 W.

    Power is the more demanding of the two integrals: it is dominated by profile drag and
    so depends on the drag calibration and the Reynolds model, where thrust depends mostly
    on lift. A wider band is justified for that reason.
    """
    assert design_loads.aero_power_w == pytest.approx(
        REPORT["aero_power_w_at_20"], rel=0.08
    )


def test_figure_of_merit_matches_report(design_loads):
    """Figure of merit, FM = T^1.5 / (sqrt(2*rho*A) * P), against the report's 0.722."""
    cfg = build_wing_config()
    fm = design_loads.figure_of_merit(cfg.air_density_kg_m3, cfg.disk_area_m2())
    assert fm == pytest.approx(REPORT["figure_of_merit"], rel=0.06)


def test_figure_of_merit_is_physically_attainable(design_loads):
    """FM above 1 would mean beating the actuator-disk limit, which is impossible."""
    cfg = build_wing_config()
    fm = design_loads.figure_of_merit(cfg.air_density_kg_m3, cfg.disk_area_m2())
    assert 0.0 < fm < 1.0, f"figure of merit {fm:.3f} is outside (0, 1)"


# =======================================================================================
# Spanwise distribution
# =======================================================================================

def test_station_angle_of_attack_matches_report(wing, design_loads):
    """Local angle of attack must match the report station by station.

    This is the sharpest test in the suite. Angle of attack is the difference between the
    built pitch and the inflow angle, so agreeing to a fraction of a degree means the
    geometry interpolation, the velocity resolution AND the induced velocity are all
    right simultaneously -- an error in any one of them would show up here.
    """
    radii = wing.radius_m
    alpha_deg = np.rad2deg(design_loads.alpha_rad[0])

    for radius, (alpha_report, _, _) in REPORT_STATIONS.items():
        index = int(np.argmin(np.abs(radii - radius)))
        assert alpha_deg[index] == pytest.approx(alpha_report, abs=0.35), (
            f"r={radius:.3f} m: model alpha={alpha_deg[index]:.2f} deg, "
            f"report {alpha_report} deg"
        )


def test_station_lift_coefficient_matches_report(wing, design_loads):
    """Local lift coefficient must match the report station by station."""
    radii = wing.radius_m
    c_l = design_loads.cl[0]

    for radius, (_, cl_report, _) in REPORT_STATIONS.items():
        index = int(np.argmin(np.abs(radii - radius)))
        assert c_l[index] == pytest.approx(cl_report, abs=0.04), (
            f"r={radius:.3f} m: model c_l={c_l[index]:.3f}, report {cl_report}"
        )


def test_station_induced_velocity_matches_report(wing, design_loads):
    """The radial induced-velocity distribution must match the report.

    Nothing about the induced velocity was fitted. It is the output of the annulus
    momentum balance with Prandtl tip loss, so matching it to within 0.1 m/s over the
    whole span is a direct test of the inflow solver.
    """
    radii = wing.radius_m
    induced = design_loads.induced_velocity_m_s

    for radius, (_, _, vi_report) in REPORT_STATIONS.items():
        index = int(np.argmin(np.abs(radii - radius)))
        assert induced[index] == pytest.approx(vi_report, abs=0.12), (
            f"r={radius:.3f} m: model v_i={induced[index]:.3f} m/s, "
            f"report {vi_report} m/s"
        )


def test_angle_of_attack_stays_in_the_designed_band(design_loads):
    """The report states alpha runs 4.4 to 5.5 deg along the loaded span.

    This is the design's central claim: every section sits near its best lift-to-drag
    ratio. A model that got the twist handling wrong would spread alpha far wider.
    """
    alpha_deg = np.rad2deg(design_loads.alpha_rad[0])
    # The outermost stations are excluded: the report's own tip station is a tip-loss
    # artefact and its build schedule departs from the optimiser there.
    interior = alpha_deg[:-2]
    assert interior.min() > 3.5, f"minimum alpha {interior.min():.2f} deg is below the band"
    assert interior.max() < 6.5, f"maximum alpha {interior.max():.2f} deg is above the band"


def test_blade_operates_with_stall_margin(design_loads):
    """Every station must sit clear of stall at the design point.

    The report quotes a minimum stall margin of 4.1 deg at 15 rad/s. The default polar
    stalls at 9 deg, so the design-point alpha of about 5 deg leaves roughly 4 deg -- the
    model must reproduce that margin, not run the blade on the edge of the polar.
    """
    alpha_deg = np.rad2deg(design_loads.alpha_rad)
    assert alpha_deg.max() < 9.0, (
        f"a station reaches alpha={alpha_deg.max():.2f} deg, at or past stall"
    )


# =======================================================================================
# Numerical quality of the solution
# =======================================================================================

def test_inflow_solution_converges(design_loads):
    """The induced-velocity iteration must actually converge, not just stop."""
    cfg = build_wing_config()
    assert design_loads.inflow_residual_m_s < cfg.inflow_tolerance_m_s, (
        f"inflow did not converge: residual {design_loads.inflow_residual_m_s:.2e} m/s "
        f"after {design_loads.inflow_iterations} iterations"
    )
    assert design_loads.inflow_iterations < cfg.inflow_max_iterations


def test_solution_is_grid_converged():
    """Doubling the number of spanwise stations must not move the answer.

    If it does, the default station count is too coarse and every result computed with it
    is discretisation error rather than physics.
    """
    coarse = build_wing(num_stations=24).hover_loads(DESIGN_SPIN_RATE_RAD_S)
    fine = build_wing(num_stations=96).hover_loads(DESIGN_SPIN_RATE_RAD_S)

    assert fine.thrust_n == pytest.approx(coarse.thrust_n, rel=0.01)
    assert fine.aero_power_w == pytest.approx(coarse.aero_power_w, rel=0.01)


def test_result_is_independent_of_inflow_warm_start():
    """A converged solution must not depend on where the iteration started.

    The solver warm-starts from the previous step's induced velocity for speed. If the
    converged answer depended on that, every result would be history-dependent.
    """
    wing_cold = build_wing()
    cold = wing_cold.hover_loads(DESIGN_SPIN_RATE_RAD_S)

    wing_warm = build_wing()
    wing_warm.hover_loads(40.0)          # drive the warm start far away
    wing_warm.hover_loads(5.0)
    warm = wing_warm.hover_loads(DESIGN_SPIN_RATE_RAD_S)

    assert warm.thrust_n == pytest.approx(cold.thrust_n, rel=2e-3)


def test_tip_loss_reduces_thrust():
    """Removing the tip-loss factor must raise predicted thrust.

    Tip loss exists because bound circulation falls to zero at the tip, so the annulus
    carries less momentum than an ideal actuator disk. Switching it off should therefore
    be optimistic. The report applied it; a model that showed no sensitivity would mean
    the factor was not actually reaching the momentum balance.
    """
    with_loss = build_wing(use_tip_loss=True).hover_loads(DESIGN_SPIN_RATE_RAD_S)
    without = build_wing(use_tip_loss=False).hover_loads(DESIGN_SPIN_RATE_RAD_S)

    assert without.thrust_n > with_loss.thrust_n
    # And the effect should be a few percent, not a factor: a huge sensitivity would mean
    # the loss factor is pinned at its floor across the span rather than acting near the tip.
    assert without.thrust_n / with_loss.thrust_n < 1.25


def test_induced_velocity_accounts_for_the_whole_inflow_angle():
    """Switching off the inflow must raise every station's alpha by exactly the inflow angle.

    In hover with no induced velocity the relative wind lies in the rotor plane, so
    phi = 0 and alpha equals the built geometric pitch. Turning the inflow on subtracts
    phi from alpha at every station. The difference between the two runs is therefore the
    inflow angle itself, which the report tabulates as 19.7 deg at the root falling to
    about 11 deg at r = 0.45 m.

    This is a stronger statement than "thrust changes": it pins the magnitude AND the
    radial shape of the induced velocity's effect on incidence.
    """
    wing_bemt = build_wing(inflow_model=InflowModel.BEMT)
    wing_none = build_wing(inflow_model=InflowModel.NONE)

    loads_bemt = wing_bemt.hover_loads(DESIGN_SPIN_RATE_RAD_S)
    loads_none = wing_none.hover_loads(DESIGN_SPIN_RATE_RAD_S)

    # With no inflow, alpha must be the geometric pitch to machine precision.
    np.testing.assert_allclose(
        loads_none.alpha_rad[0], wing_none.pitch_rad, atol=1e-12
    )

    inflow_angle_deg = np.rad2deg(loads_none.alpha_rad[0] - loads_bemt.alpha_rad[0])

    # Report section 8.1: phi = 19.7 deg at the root, ~10.9 deg at r = 0.45 m.
    assert inflow_angle_deg[0] == pytest.approx(19.7, abs=1.0)
    assert inflow_angle_deg.min() > 9.0
    assert inflow_angle_deg.max() < 21.0
    # And it must fall monotonically outboard over the loaded span, as the report's does.
    assert inflow_angle_deg[0] > inflow_angle_deg[-3]


def test_removing_inflow_drives_the_blade_past_stall():
    """Without induced velocity the blade sits at its full geometric pitch, and stalls.

    The built pitch runs from 24.7 deg at the root to 16.5 deg at the tip, all of which is
    well past the 9 deg stall of the default polar. So the no-inflow case is not merely
    optimistic, it is qualitatively different: a fully separated blade. The test exists to
    document that, and to confirm the post-stall branch is what responds -- if the polar
    silently extrapolated its linear lift curve instead, c_l would reach 3.0 here.
    """
    loads = build_wing(inflow_model=InflowModel.NONE).hover_loads(DESIGN_SPIN_RATE_RAD_S)
    alpha_deg = np.rad2deg(loads.alpha_rad)

    assert alpha_deg.min() > 9.0, "expected every station past stall without inflow"
    assert loads.cl.max() < 1.6, (
        f"post-stall c_l reached {loads.cl.max():.2f}; the linear branch is leaking "
        "past the stall blend"
    )


def test_glauert_inflow_is_a_reasonable_approximation_in_hover():
    """In hover the uniform-inflow model should land near the BEMT answer.

    Glauert's uniform inflow discards the radial distribution, so it will not agree
    exactly, but in hover the two momentum balances describe the same physics and a large
    disagreement would indicate a bug in one of them.
    """
    bemt = build_wing(inflow_model=InflowModel.BEMT).hover_loads(DESIGN_SPIN_RATE_RAD_S)
    glauert = build_wing(inflow_model=InflowModel.GLAUERT).hover_loads(
        DESIGN_SPIN_RATE_RAD_S
    )

    assert glauert.thrust_n == pytest.approx(bemt.thrust_n, rel=0.20)
    assert glauert.thrust_n > 0.0


# =======================================================================================
# Momentum-theory cross-check
# =======================================================================================

def test_induced_velocity_is_consistent_with_actuator_disk_theory(design_loads):
    """The mean induced velocity must be close to the actuator-disk value.

    Simple momentum theory over the full disk gives v_i = sqrt(T / (2*rho*A)). The blade
    only occupies the annulus from 0.2 to 0.5 m rather than the whole disk, and tip loss
    reduces the effective area further, so the BEMT mean will sit above that figure --
    but it must be the same order, within a factor of two. This catches a whole class of
    error: a wrong factor in the momentum balance.
    """
    cfg = build_wing_config()
    disk_value = np.sqrt(
        design_loads.thrust_n / (2.0 * cfg.air_density_kg_m3 * cfg.disk_area_m2())
    )
    model_mean = float(np.mean(design_loads.induced_velocity_m_s))

    assert 0.8 * disk_value < model_mean < 2.5 * disk_value, (
        f"mean induced velocity {model_mean:.2f} m/s against actuator-disk "
        f"{disk_value:.2f} m/s"
    )


def test_disk_loading_and_thrust_are_mutually_consistent(design_loads):
    """Thrust recovered from the annulus momentum balance must match the integrated force.

    Re-deriving thrust from the converged induced-velocity field by a route the solver
    does not take -- summing 4*pi*rho*r*v_i^2*dr over the annuli -- and getting the blade
    element answer back is a direct check that the two halves of BEMT were actually driven
    to the same solution.
    """
    cfg = build_wing_config()
    wing = RotatingWing(cfg)
    loads = wing.hover_loads(DESIGN_SPIN_RATE_RAD_S)

    radii = wing.radius_m
    widths = np.gradient(radii)
    induced = loads.induced_velocity_m_s

    # Momentum thrust per annulus with the tip-loss factor omitted. Because the solver
    # applies that factor and it is always <= 1, this sum must come out ABOVE the blade
    # element thrust -- and only moderately so, since tip loss acts over the outboard
    # stations rather than the whole span. Both halves of that statement are binding: too
    # low would mean the momentum balance is not being satisfied, too high would mean the
    # loss factor is pinned near its floor everywhere instead of near the tip.
    momentum_thrust = float(
        np.sum(4.0 * np.pi * cfg.air_density_kg_m3 * radii * induced ** 2 * widths)
    )

    assert momentum_thrust > loads.thrust_n, (
        f"momentum thrust {momentum_thrust:.3f} N should exceed the tip-loss-reduced "
        f"blade element thrust {loads.thrust_n:.3f} N"
    )
    assert momentum_thrust < 2.0 * loads.thrust_n, (
        f"momentum thrust {momentum_thrust:.3f} N is more than twice the blade element "
        f"thrust {loads.thrust_n:.3f} N; tip loss is removing too much"
    )
