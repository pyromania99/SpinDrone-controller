"""
| File: test_airfoil_polar.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Physics probes for the 2-D section polar models.

The polar is the innermost layer of the wing model: every force the simulation applies is
proportional to a c_l or a c_d that comes out of here. A polar that is smooth, bounded and
correctly signed over the full +/-180 deg range is a precondition for the blade element
solver producing anything meaningful, so these tests come first.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import (
    AnalyticPolar,
    TablePolar,
    goe210_polar,
    viterna_coefficients,
)


# =======================================================================================
# Lift curve, attached flow
# =======================================================================================

def test_lift_curve_slope_matches_configuration():
    """In attached flow the model must reproduce c_l = c_l0 + c_l_alpha * alpha exactly."""
    polar = AnalyticPolar(cl_zero_alpha=0.4, cl_alpha_per_rad=5.5, alpha_stall_pos_deg=12.0)

    alpha = np.deg2rad(np.array([-4.0, 0.0, 2.0, 5.0]))
    c_l, _ = polar.evaluate(alpha, 1.0e5)

    expected = 0.4 + 5.5 * alpha
    np.testing.assert_allclose(c_l, expected, rtol=1e-12, atol=1e-12)


def test_zero_alpha_lift_is_camber_term():
    """A cambered section lifts at zero incidence; a symmetric one does not."""
    cambered = AnalyticPolar(cl_zero_alpha=0.49, cl_alpha_per_rad=5.85)
    symmetric = AnalyticPolar(cl_zero_alpha=0.0, cl_alpha_per_rad=5.85)

    assert cambered.evaluate(0.0, 1e5)[0] == pytest.approx(0.49)
    assert symmetric.evaluate(0.0, 1e5)[0] == pytest.approx(0.0)


def test_symmetric_section_lift_is_odd_in_alpha():
    """A zero-camber section with symmetric stall must give c_l(-a) = -c_l(a)."""
    polar = AnalyticPolar(
        cl_zero_alpha=0.0,
        cl_alpha_per_rad=6.0,
        alpha_stall_pos_deg=11.0,
        alpha_stall_neg_deg=-11.0,
    )
    alpha = np.deg2rad(np.linspace(0.0, 179.0, 180))
    c_l_pos, _ = polar.evaluate(alpha, 1e5)
    c_l_neg, _ = polar.evaluate(-alpha, 1e5)

    np.testing.assert_allclose(c_l_neg, -c_l_pos, atol=1e-9)


def test_symmetric_section_drag_is_even_in_alpha():
    """Drag is direction-independent for a symmetric section: c_d(-a) = c_d(a)."""
    polar = AnalyticPolar(
        cl_zero_alpha=0.0,
        cl_alpha_per_rad=6.0,
        cl_at_cd_min=0.0,
        alpha_stall_pos_deg=11.0,
        alpha_stall_neg_deg=-11.0,
    )
    alpha = np.deg2rad(np.linspace(0.0, 179.0, 180))
    _, c_d_pos = polar.evaluate(alpha, 1e5)
    _, c_d_neg = polar.evaluate(-alpha, 1e5)

    np.testing.assert_allclose(c_d_neg, c_d_pos, atol=1e-9)


# =======================================================================================
# Full-range behaviour: the property that matters for a spinning wing
# =======================================================================================

def test_coefficients_finite_over_full_alpha_range():
    """No NaN or inf anywhere in +/-180 deg, at any Reynolds number the solver can reach.

    A single NaN here propagates to the applied force and destroys the whole scene, so
    this is checked densely rather than at a few sample points.
    """
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(-180.0, 180.0, 2001))

    for reynolds in (1.0e3, 1.0e4, 6.0e4, 1.35e5, 1.0e6, 1.0e8):
        c_l, c_d = polar.evaluate(alpha, reynolds)
        assert np.all(np.isfinite(c_l)), f"non-finite c_l at Re={reynolds:g}"
        assert np.all(np.isfinite(c_d)), f"non-finite c_d at Re={reynolds:g}"


def test_drag_is_never_negative():
    """A negative drag coefficient is an energy source. It must be impossible."""
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(-180.0, 180.0, 2001))

    for reynolds in (1.0e3, 5.0e4, 1.0e5, 1.0e7):
        _, c_d = polar.evaluate(alpha, reynolds)
        assert np.all(c_d >= 0.0), f"negative c_d at Re={reynolds:g}"


def test_coefficients_bounded_by_flat_plate_limit():
    """Neither coefficient may exceed the flat-plate scale the Viterna model is built on."""
    cd_max = 1.8
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(-180.0, 180.0, 2001))
    c_l, c_d = polar.evaluate(alpha, 1.0e5)

    assert np.max(np.abs(c_l)) < 2.0, "lift coefficient exceeds any physical section value"
    assert np.max(c_d) <= cd_max * 1.05, "drag coefficient exceeds the flat-plate ceiling"


def test_lift_vanishes_at_zero_and_plusminus_ninety_degrees():
    """A fully separated plate carries no lift broadside-on or edge-on.

    At exactly 90 deg the section is a flat plate normal to the flow: pure drag, no lift.
    The Viterna form satisfies this analytically, so it is a genuine check of the
    implementation rather than of the fit.
    """
    polar = AnalyticPolar(cl_zero_alpha=0.0, cl_alpha_per_rad=6.0)

    c_l_90, _ = polar.evaluate(np.deg2rad(90.0), 1e5)
    c_l_m90, _ = polar.evaluate(np.deg2rad(-90.0), 1e5)

    assert abs(float(c_l_90)) < 1e-9
    assert abs(float(c_l_m90)) < 1e-9


def test_drag_is_maximum_near_ninety_degrees():
    """Peak drag must occur broadside-on, not somewhere unphysical."""
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(0.0, 180.0, 1801))
    _, c_d = polar.evaluate(alpha, 1.0e5)

    alpha_peak_deg = np.rad2deg(alpha[int(np.argmax(c_d))])
    assert 80.0 <= alpha_peak_deg <= 100.0, (
        f"drag peaks at {alpha_peak_deg:.1f} deg, expected near 90 deg"
    )


def test_polar_is_continuous_across_the_stall_blend():
    """No step change in c_l or c_d anywhere, in particular across the stall blend.

    A discontinuity would show up in the simulation as an impulsive torque the instant a
    blade element crosses stall. The bound is set from the local gradient so the test
    measures smoothness, not merely finiteness.
    """
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(-30.0, 30.0, 6001))
    c_l, c_d = polar.evaluate(alpha, 1.0e5)

    d_alpha = alpha[1] - alpha[0]
    max_dcl = np.max(np.abs(np.diff(c_l))) / d_alpha
    max_dcd = np.max(np.abs(np.diff(c_d))) / d_alpha

    # The attached lift slope is 5.85 /rad; nothing in the blend may exceed it by much.
    assert max_dcl < 3.0 * 5.85, f"lift curve has a near-discontinuity, dcl/da = {max_dcl:.1f}"
    assert max_dcd < 10.0, f"drag curve has a near-discontinuity, dcd/da = {max_dcd:.1f}"


# =======================================================================================
# Reynolds number behaviour
# =======================================================================================

def test_drag_decreases_monotonically_with_reynolds_number():
    """Higher Reynolds number means thinner boundary layers and less skin friction."""
    polar = goe210_polar()
    reynolds = np.array([2.0e4, 5.0e4, 1.0e5, 2.0e5, 5.0e5])
    _, c_d = polar.evaluate(np.deg2rad(5.0), reynolds)

    assert np.all(np.diff(c_d) < 0.0), f"drag not monotone in Re: {c_d}"


def test_reynolds_scaling_is_clamped_at_both_ends():
    """Outside the clamp range the drag must stop changing, not diverge.

    Without the clamp, the (Re_ref/Re)^0.68 power law sends c_d to infinity as the blade
    slows through zero speed at the start of a run.
    """
    polar = goe210_polar()
    alpha = np.deg2rad(5.0)

    cd_below = polar.evaluate(alpha, polar.reynolds_min * 0.01)[1]
    cd_at_min = polar.evaluate(alpha, polar.reynolds_min)[1]
    cd_above = polar.evaluate(alpha, polar.reynolds_max * 100.0)[1]
    cd_at_max = polar.evaluate(alpha, polar.reynolds_max)[1]

    assert float(cd_below) == pytest.approx(float(cd_at_min), rel=1e-9)
    assert float(cd_above) == pytest.approx(float(cd_at_max), rel=1e-9)


def test_drag_bucket_minimum_is_at_the_configured_lift_coefficient():
    """The drag polar's minimum must sit where the configuration says it does."""
    polar = AnalyticPolar(
        cl_zero_alpha=0.0,
        cl_alpha_per_rad=6.0,
        cl_at_cd_min=0.5,
        cd_induced_factor=0.02,
        cd_min=0.01,
        alpha_stall_pos_deg=20.0,
        alpha_stall_neg_deg=-20.0,
    )
    alpha = np.linspace(-0.2, 0.3, 4001)
    c_l, c_d = polar.evaluate(alpha, polar.reynolds_ref)

    cl_at_min_drag = c_l[int(np.argmin(c_d))]
    assert float(cl_at_min_drag) == pytest.approx(0.5, abs=5e-3)
    assert float(np.min(c_d)) == pytest.approx(0.01, rel=1e-3)


# =======================================================================================
# Calibration against the design report
# =======================================================================================

def test_goe210_lift_matches_design_report_station_data():
    """The calibrated polar must reproduce the report's (alpha, c_l) pairs.

    Source: foil_out/GOE210_DESIGN_REPORT.md section 8.1.
    """
    polar = goe210_polar()

    # (alpha [deg], c_l) taken from the station table.
    cases = [(4.4, 0.96), (4.6, 0.97), (4.7, 0.97), (5.1, 1.01), (5.5, 1.04)]

    for alpha_deg, cl_report in cases:
        c_l, _ = polar.evaluate(np.deg2rad(alpha_deg), 1.0e5)
        assert float(c_l) == pytest.approx(cl_report, abs=0.03), (
            f"alpha={alpha_deg} deg: model c_l={float(c_l):.3f}, report {cl_report}"
        )


def test_goe210_drag_matches_design_report_reynolds_trend():
    """The calibrated drag must reproduce the report's c_d at the design lift.

    The report tabulates c_l/c_d per station; c_d = c_l / (c_l/c_d). Because c_l is nearly
    constant at ~1.0 along the span, those values isolate the Reynolds dependence.

    ON THE TOLERANCE
        12 percent, which is looser than the 8 percent a fit to these points alone would
        achieve. That is a deliberate trade. The drag model is fitted to two constraints
        at once -- these station drags AND the Reynolds trend of the best lift-to-drag
        point from section 5.4 -- and a single power law cannot satisfy both exactly. The
        residual is concentrated at the lowest-Reynolds station, the tip, which the report
        itself flags as its least trustworthy region and which contributes little to the
        integrated thrust and power.

        Getting the optimum's trend right matters more for this project, because it is
        what determines the twist when a blade is re-designed for a different span. The
        integrated quantities that follow from this calibration are checked in
        test_wing_validation.py and agree with the report to 1 percent on thrust.
    """
    polar = goe210_polar()

    # (Re, c_l, c_d) derived from section 8.1.
    cases = [
        (60.2e3, 0.97, 0.97 / 43.3),
        (75.6e3, 1.04, 1.04 / 54.2),
        (104.8e3, 1.00, 1.00 / 65.3),
        (135.2e3, 0.96, 0.96 / 74.3),
    ]

    for reynolds, cl_report, cd_report in cases:
        alpha = (cl_report - polar.cl_zero_alpha) / polar.cl_alpha_per_rad
        _, c_d = polar.evaluate(alpha, reynolds)
        assert float(c_d) == pytest.approx(cd_report, rel=0.12), (
            f"Re={reynolds:.0f}: model c_d={float(c_d):.5f}, report {cd_report:.5f}"
        )


def test_best_lift_to_drag_point_moves_with_reynolds_number():
    """The optimum angle of attack must rise as Reynolds number falls.

    This is the property the drag model is structured around, and the reason only the
    parasite term carries the Reynolds scaling. The report's section 5.4 measures c_l at
    maximum c_l/c_d falling from 0.96 at Re 100k to 0.89 at Re 200k; a model with a
    Reynolds-independent optimum would twist a re-designed blade to the wrong incidence
    over most of its span.
    """
    polar = goe210_polar()

    # (Re, c_l at max lift-to-drag) from section 5.4, chord 0.30 m, Re = 20000 * V.
    cases = [(100e3, 0.96), (140e3, 0.92), (200e3, 0.89)]

    optima = []
    for reynolds, cl_report in cases:
        alpha = np.deg2rad(np.linspace(-2.0, 12.0, 1401))
        c_l, c_d = polar.evaluate(alpha, reynolds)
        cl_at_best = float(c_l[int(np.argmax(c_l / np.maximum(c_d, 1e-9)))])
        optima.append(cl_at_best)
        assert cl_at_best == pytest.approx(cl_report, abs=0.03), (
            f"Re={reynolds:.0f}: model c_l at best L/D = {cl_at_best:.3f}, "
            f"report {cl_report}"
        )

    # And it must be monotone decreasing in Reynolds number.
    assert optima[0] > optima[1] > optima[2], f"optimum not monotone in Re: {optima}"


def test_goe210_root_operating_point_matches_report_remark():
    """The report states the blade root reaches c_l = 1.3 at alpha = 8 deg."""
    c_l, _ = goe210_polar().evaluate(np.deg2rad(8.0), 1.0e5)
    assert float(c_l) == pytest.approx(1.30, abs=0.04)


# =======================================================================================
# Viterna construction
# =======================================================================================

def test_viterna_matches_the_polar_at_the_stall_point():
    """The extrapolation must agree with the attached model where they are joined."""
    alpha_stall = np.deg2rad(12.0)
    cl_stall, cd_stall, cd_max = 1.35, 0.03, 1.8

    a1, b1, a2, b2 = viterna_coefficients(alpha_stall, cl_stall, cd_stall, cd_max)

    sin_s, cos_s = np.sin(alpha_stall), np.cos(alpha_stall)
    cl_check = a1 * np.sin(2 * alpha_stall) + a2 * cos_s ** 2 / sin_s
    cd_check = b1 * sin_s ** 2 + b2 * cos_s

    assert cl_check == pytest.approx(cl_stall, rel=1e-10)
    assert cd_check == pytest.approx(cd_stall, rel=1e-10)


def test_viterna_rejects_degenerate_stall_angles():
    """Zero and 90 deg make the matching expressions singular and must be refused."""
    with pytest.raises(ValueError):
        viterna_coefficients(0.0, 1.0, 0.02, 1.8)
    with pytest.raises(ValueError):
        viterna_coefficients(np.pi / 2, 1.0, 0.02, 1.8)


def test_analytic_polar_rejects_invalid_configuration():
    """Every physical invariant in the dataclass must actually be enforced."""
    with pytest.raises(ValueError):
        AnalyticPolar(alpha_stall_pos_deg=-1.0)
    with pytest.raises(ValueError):
        AnalyticPolar(alpha_stall_neg_deg=1.0)
    with pytest.raises(ValueError):
        AnalyticPolar(stall_blend_deg=0.0)
    with pytest.raises(ValueError):
        AnalyticPolar(cd_min=-0.01)
    with pytest.raises(ValueError):
        AnalyticPolar(reynolds_max=1.0e3, reynolds_min=1.0e4)


# =======================================================================================
# Tabulated polar
# =======================================================================================

def _simple_table():
    alpha = np.deg2rad(np.array([-10.0, 0.0, 10.0]))
    reynolds = np.array([5.0e4, 1.0e5])
    c_l = np.array([[-0.5, -0.6], [0.0, 0.0], [1.0, 1.2]])
    c_d = np.array([[0.05, 0.04], [0.01, 0.008], [0.06, 0.05]])
    return TablePolar(alpha, reynolds, c_l, c_d)


def test_table_polar_is_exact_at_grid_nodes():
    """Interpolation must be exact where data exists."""
    table = _simple_table()
    c_l, c_d = table.evaluate(np.deg2rad(10.0), 1.0e5)
    assert float(c_l) == pytest.approx(1.2)
    assert float(c_d) == pytest.approx(0.05)


def test_table_polar_interpolates_bilinearly():
    """Midway between four nodes the result is their mean."""
    table = _simple_table()
    c_l, _ = table.evaluate(np.deg2rad(5.0), 7.5e4)
    expected = 0.25 * (0.0 + 0.0 + 1.0 + 1.2)
    assert float(c_l) == pytest.approx(expected)


def test_table_polar_clamps_rather_than_extrapolating():
    """Off the end of the grid the polar must hold its edge value.

    Extrapolating a tabulated polar is the classic way to generate silent non-physical
    rotor loads, so the implementation clamps deliberately.
    """
    table = _simple_table()
    c_l_far, _ = table.evaluate(np.deg2rad(170.0), 1.0e5)
    c_l_edge, _ = table.evaluate(np.deg2rad(10.0), 1.0e5)
    assert float(c_l_far) == pytest.approx(float(c_l_edge))


def test_table_polar_rejects_malformed_input():
    """Shape and ordering mistakes must fail loudly at construction."""
    alpha = np.deg2rad(np.array([-10.0, 0.0, 10.0]))
    reynolds = np.array([5.0e4, 1.0e5])
    good = np.zeros((3, 2))

    with pytest.raises(ValueError):  # wrong table shape
        TablePolar(alpha, reynolds, np.zeros((2, 2)), good)
    with pytest.raises(ValueError):  # non-monotone alpha
        TablePolar(np.array([0.0, -1.0, 1.0]), reynolds, good, good)
    with pytest.raises(ValueError):  # non-monotone Re
        TablePolar(alpha, np.array([1.0e5, 5.0e4]), good, good)
    with pytest.raises(ValueError):  # degenerate grid
        TablePolar(np.array([0.0]), reynolds, np.zeros((1, 2)), np.zeros((1, 2)))


# =======================================================================================
# Interface contract
# =======================================================================================

def test_evaluate_broadcasts_and_preserves_shape():
    """The solver passes (n_blades, n_stations) arrays; the polar must handle them."""
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(0.0, 8.0, 12)).reshape(2, 6)
    reynolds = np.full((2, 6), 1.0e5)

    c_l, c_d = polar.evaluate(alpha, reynolds)
    assert c_l.shape == (2, 6)
    assert c_d.shape == (2, 6)


def test_polar_is_stateless():
    """Repeated evaluation must give identical results; no hidden state may accumulate."""
    polar = goe210_polar()
    alpha = np.deg2rad(np.linspace(-20.0, 20.0, 101))

    first = polar.evaluate(alpha, 1.0e5)
    for _ in range(5):
        polar.evaluate(np.deg2rad(45.0), 1.0e4)  # unrelated calls in between
    second = polar.evaluate(alpha, 1.0e5)

    np.testing.assert_array_equal(first[0], second[0])
    np.testing.assert_array_equal(first[1], second[1])


# =======================================================================================
# Post-stall drag ceiling, Viterna equation (13)
# =======================================================================================
# The flat-plate drag coefficient the Viterna branch asymptotes to is not a universal
# constant. The source paper gives it as a function of the blade's aspect ratio, and
# argues the point explicitly: a stubby blade sheds flow round its tip in deep stall and
# never reaches the 2-D value. Using 1.8 here overstated deeply stalled drag by more than
# half, which is invisible at the operating point and wrong everywhere else.
#
# Source: Viterna & Corrigan 1981, NTRS 19830010962 eq. (13), in resources/.

def test_viterna_cd_max_matches_the_published_correlation():
    """c_d,max = 1.11 + 0.018 * AR, checked at several aspect ratios."""
    from examples.utils.wing_params import viterna_cd_max

    for aspect_ratio in (1.0, 1.302, 8.0, 25.0, 50.0):
        assert viterna_cd_max(aspect_ratio) == pytest.approx(
            1.11 + 0.018 * aspect_ratio
        )


def test_viterna_cd_max_is_clamped_beyond_the_correlation_range():
    """The paper states AR <= 50. Past that the correlation is extrapolating."""
    from examples.utils.wing_params import viterna_cd_max

    assert viterna_cd_max(1.0e4) == pytest.approx(viterna_cd_max(50.0))


def test_viterna_cd_max_rejects_a_non_positive_aspect_ratio():
    """A blade of zero or negative span is a geometry error, not an extrapolation."""
    from examples.utils.wing_params import viterna_cd_max

    for bad in (0.0, -1.0):
        with pytest.raises(ValueError):
            viterna_cd_max(bad)


def test_stubby_blade_stays_well_below_the_two_dimensional_flat_plate_value():
    """The paper's own argument: a flat plate at AR 8 already measures below 1.3."""
    from examples.utils.wing_params import viterna_cd_max

    assert viterna_cd_max(8.0) < 1.3
    # This wing is stubbier still, so it must be lower again.
    assert viterna_cd_max(1.302) < viterna_cd_max(8.0)


def test_default_polar_takes_its_drag_ceiling_from_the_planform():
    """The shipped configuration must use the derived value, not the 2-D fallback.

    Guards the wiring: the polar is built before the config exists, so the planform has to
    be measured and passed in explicitly. If that plumbing broke, the polar would silently
    revert to 1.8 and nothing at the operating point would notice.
    """
    from examples.utils.wing_params import build_wing_config, viterna_cd_max

    config = build_wing_config()
    expected = viterna_cd_max(config.blade_aspect_ratio())

    assert config.polar.cd_max_flat_plate == pytest.approx(expected)
    assert config.polar.cd_max_flat_plate < 1.3, (
        "the polar fell back to the 2-D flat-plate value; the planform is not reaching it"
    )


def test_drag_ceiling_changes_deep_stall_but_not_the_operating_point():
    """The whole point of the correction: it must not disturb attached flow.

    If this test ever failed it would mean the post-stall branch had leaked into the
    attached-flow range, which is the blend's job to prevent.
    """
    from examples.utils.wing_params import goe210_polar

    two_d = goe210_polar()
    finite = goe210_polar(aspect_ratio=1.302)

    # At the operating incidence the two must be indistinguishable.
    _, cd_2d = two_d.evaluate(np.deg2rad(5.0), 1.0e5)
    _, cd_fin = finite.evaluate(np.deg2rad(5.0), 1.0e5)
    assert float(cd_fin) == pytest.approx(float(cd_2d), rel=1e-9)

    # Broadside, the finite blade must be far lower, and in the ratio of the ceilings.
    _, cd_2d_90 = two_d.evaluate(np.deg2rad(90.0), 1.0e5)
    _, cd_fin_90 = finite.evaluate(np.deg2rad(90.0), 1.0e5)
    assert float(cd_fin_90) < 0.7 * float(cd_2d_90)
    assert float(cd_fin_90) == pytest.approx(finite.cd_max_flat_plate, rel=1e-6)
