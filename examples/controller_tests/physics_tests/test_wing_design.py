"""
| File: test_wing_design.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Tests for the blade pitch-schedule designer.

WHY THE DESIGNER MATTERS
------------------------
The default build schedule was optimised for one geometry: a wing spanning r = 0.2 to
0.5 m at 20 rad/s. Its twist is not decoration -- each station's pitch is set so the
section meets the local relative wind near its best lift-to-drag angle. Move the span and
that twist is simply wrong.

So changing ``root_radius_m`` and ``tip_radius_m`` without re-deriving the pitch does not
give a smaller working blade; it gives the old twist applied at the wrong radii. The
designer exists so that retargeting the wing to a different airframe is a calculation
rather than a guess.

THE HEADLINE TEST
-----------------
``test_designer_reproduces_the_report_build_schedule``. Running the designer on the
geometry the design report optimised must recover the report's own pitch schedule. That
is a round trip through an independent calculation: if the inflow solver, the polar's
Reynolds behaviour, or the design iteration were wrong, the recovered twist would not
match.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import (
    DESIGN_SPIN_RATE_RAD_S,
    RotatingWing,
    RotatingWingConfig,
    build_wing_config,
)
from examples.utils.wing_params import (
    best_lift_to_drag_alpha,
    design_pitch_schedule,
    goe210_polar,
    scale_chord_to_span,
)


# =======================================================================================
# The round trip
# =======================================================================================

def test_designer_reproduces_the_report_build_schedule():
    """Designing the report's own geometry must recover the report's own twist.

    Tolerances are set on the loaded span. The outermost station is excluded: the report
    flags its own tip station as a tip-loss artefact ("the required pitch explodes there")
    and its recommended build schedule deliberately departs from the optimiser output at
    the tip.
    """
    config = build_wing_config()
    result = design_pitch_schedule(config, DESIGN_SPIN_RATE_RAD_S)

    assert result.converged

    report_pitch = np.asarray(config.station_pitch_deg)
    designed_pitch = np.asarray(result.station_pitch_deg)

    error = np.abs(designed_pitch - report_pitch)[:-1]

    assert error.mean() < 1.5, (
        f"mean pitch error {error.mean():.2f} deg\n"
        f"  report:    {report_pitch.round(1)}\n"
        f"  designed:  {designed_pitch.round(1)}"
    )
    assert error.max() < 3.0, f"worst pitch error {error.max():.2f} deg"


def test_designed_blade_runs_in_the_reported_angle_of_attack_band():
    """The report states its stations run at 4.4 to 5.5 deg. The designer must agree.

    The band is a consequence of designing each station to its own best lift-to-drag
    angle while Reynolds number falls outboard. Reproducing its WIDTH, not just its mean,
    is what tests the polar's Reynolds behaviour.
    """
    result = design_pitch_schedule(build_wing_config(), DESIGN_SPIN_RATE_RAD_S)
    alpha = result.achieved_alpha_deg

    assert alpha.min() > 3.5, f"minimum alpha {alpha.min():.2f} deg"
    assert alpha.max() < 6.5, f"maximum alpha {alpha.max():.2f} deg"
    assert alpha.max() - alpha.min() > 0.5, (
        "the designed blade has no spanwise variation in alpha; per-station targeting "
        "is not taking effect"
    )


def test_designed_blade_performance_matches_the_report():
    """A blade designed from scratch must perform like the report's."""
    result = design_pitch_schedule(build_wing_config(), DESIGN_SPIN_RATE_RAD_S)

    assert result.thrust_n == pytest.approx(3.74, rel=0.10)
    assert result.figure_of_merit == pytest.approx(0.722, rel=0.08)


# =======================================================================================
# Designing for a different geometry
# =======================================================================================

def test_designer_handles_a_rescaled_span():
    """Retargeting to a shorter span must converge and give a sane blade."""
    radii, chords = scale_chord_to_span(
        build_wing_config().station_radius_m,
        build_wing_config().station_chord_m,
        0.09,
        0.24,
    )
    config = build_wing_config(
        root_radius_m=0.09,
        tip_radius_m=0.24,
        station_radius_m=radii,
        station_chord_m=chords,
    )

    result = design_pitch_schedule(config, DESIGN_SPIN_RATE_RAD_S)

    assert result.converged
    assert np.all(np.isfinite(result.achieved_alpha_deg))
    assert result.thrust_n > 0.0
    # Every station must sit clear of stall. This blade runs at Re 17k-35k, where the
    # drag bucket deepens and the best lift-to-drag angle climbs towards stall; without
    # the designer's stall margin the iteration would chase that optimum across the stall
    # boundary and fail to converge.
    assert result.achieved_alpha_deg.max() < 9.0


def test_rescaled_blade_actually_needs_redesigning():
    """Moving the span without re-pitching must leave a measurably worse blade.

    This is the test that justifies the designer existing. Reusing the original twist at
    new radii puts the sections at the wrong incidence, so re-deriving the pitch must
    recover thrust.

    The metric is thrust rather than figure of merit. FM is a RATIO, and the designer
    maximises each section's lift-to-drag, not the whole rotor's FM; a lightly loaded
    blade can post a respectable FM while producing very little lift. Thrust at a fixed
    spin rate is the quantity the vehicle actually needs.
    """
    radii, chords = scale_chord_to_span(
        build_wing_config().station_radius_m,
        build_wing_config().station_chord_m,
        0.15,
        0.40,
    )
    naive = build_wing_config(
        root_radius_m=0.15,
        tip_radius_m=0.40,
        station_radius_m=radii,
        station_chord_m=chords,
    )

    naive_thrust = RotatingWing(naive).hover_loads(DESIGN_SPIN_RATE_RAD_S).thrust_n
    designed = design_pitch_schedule(naive, DESIGN_SPIN_RATE_RAD_S)

    assert designed.converged
    assert designed.thrust_n > naive_thrust, (
        f"redesign did not improve thrust: {naive_thrust:.3f} -> "
        f"{designed.thrust_n:.3f} N"
    )


def test_designed_stations_sit_at_their_intended_angle_of_attack():
    """After redesign, every station must be near its own best lift-to-drag angle.

    This is the designer's actual contract, stated directly rather than through a
    performance proxy: each station is placed at the angle of attack its local Reynolds
    number calls for.
    """
    radii, chords = scale_chord_to_span(
        build_wing_config().station_radius_m,
        build_wing_config().station_chord_m,
        0.15,
        0.40,
    )
    config = build_wing_config(
        root_radius_m=0.15,
        tip_radius_m=0.40,
        station_radius_m=radii,
        station_chord_m=chords,
    )

    result = design_pitch_schedule(config, DESIGN_SPIN_RATE_RAD_S)
    config.station_pitch_deg = result.station_pitch_deg

    wing = RotatingWing(config)
    loads = wing.hover_loads(DESIGN_SPIN_RATE_RAD_S)
    achieved = np.rad2deg(loads.alpha_rad[0])

    targets = np.array(
        [best_lift_to_drag_alpha(config.polar, re) for re in loads.reynolds[0]]
    )

    # 1.5 deg allows for the pitch being tabulated at nine stations and interpolated
    # across twenty-four, which cannot hold alpha exactly on target in between.
    np.testing.assert_allclose(achieved, targets, atol=1.5)


def test_scale_chord_to_span_preserves_planform_shape():
    """Rescaling must keep the taper ratio and place stations proportionally."""
    radii_in = [0.2, 0.35, 0.5]
    chords_in = [0.30, 0.20, 0.10]

    radii, chords = scale_chord_to_span(radii_in, chords_in, 0.1, 0.4)

    assert radii[0] == pytest.approx(0.1)
    assert radii[-1] == pytest.approx(0.4)
    assert radii[1] == pytest.approx(0.25)  # the midpoint stays the midpoint
    # Span ratio is 0.3/0.3 = 1.0 here, so chords are unchanged.
    np.testing.assert_allclose(chords, chords_in, rtol=1e-12)

    # Halving the span must halve the chords, preserving aspect ratio.
    _, half = scale_chord_to_span(radii_in, chords_in, 0.1, 0.25)
    np.testing.assert_allclose(half, np.asarray(chords_in) * 0.5, rtol=1e-12)


def test_scale_chord_to_span_rejects_an_inverted_span():
    """A tip inboard of the root is a geometry error, not something to extrapolate."""
    with pytest.raises(ValueError):
        scale_chord_to_span([0.2, 0.5], [0.3, 0.1], 0.4, 0.2)


# =======================================================================================
# Targeting behaviour
# =======================================================================================

def test_fixed_target_alpha_is_achieved_across_the_span():
    """With per-station targeting off, every station must reach the requested alpha."""
    result = design_pitch_schedule(
        build_wing_config(),
        DESIGN_SPIN_RATE_RAD_S,
        target_alpha_deg=5.0,
        per_station_best_ld=False,
    )

    assert result.converged
    # 1.0 deg, not zero: pitch is tabulated at nine stations and linearly interpolated
    # across the solver's twenty-four, so alpha cannot be held exactly flat in between.
    assert result.max_alpha_error_deg < 1.0, (
        f"alpha spread {result.max_alpha_error_deg:.2f} deg around the 5.0 deg target"
    )
    np.testing.assert_allclose(result.achieved_alpha_deg, 5.0, atol=1.0)


def test_best_lift_to_drag_alpha_matches_the_report():
    """The polar's optimum must sit where section 5.4 of the report puts it."""
    polar = goe210_polar()

    # (Re, alpha at max L/D [deg]) from section 5.4, chord 0.30 m.
    for reynolds, alpha_report in ((100e3, 4.6), (140e3, 4.1), (200e3, 3.8)):
        alpha = best_lift_to_drag_alpha(polar, reynolds)
        assert alpha == pytest.approx(alpha_report, abs=0.5), (
            f"Re={reynolds:.0f}: model {alpha:.2f} deg, report {alpha_report} deg"
        )


def test_best_lift_to_drag_alpha_does_not_wander_into_the_post_stall_branch():
    """The search must stay in attached flow.

    The Viterna branch can present a spurious local optimum at high incidence. Searching
    past stall and finding it would twist a designed blade into a permanently stalled
    state.
    """
    alpha = best_lift_to_drag_alpha(goe210_polar(), 1.0e5)
    assert -2.0 <= alpha <= 9.0


def test_designer_rejects_a_zero_spin_rate():
    """Without rotation there is no inflow angle, so the problem is undefined."""
    with pytest.raises(ValueError):
        design_pitch_schedule(build_wing_config(), 0.0)


def test_designer_rejects_an_invalid_relaxation():
    """Relaxation outside (0, 1] either never advances or diverges."""
    with pytest.raises(ValueError):
        design_pitch_schedule(build_wing_config(), 20.0, relaxation=0.0)
    with pytest.raises(ValueError):
        design_pitch_schedule(build_wing_config(), 20.0, relaxation=1.5)


def test_designer_does_not_mutate_the_configuration_it_is_given():
    """The caller's configuration must be left untouched.

    The designer works on a copy. If it mutated the input, a sweep that designed a blade
    and then built a vehicle from the same config object would silently fly a different
    geometry than it recorded.
    """
    config = build_wing_config()
    original_pitch = list(config.station_pitch_deg)
    original_offset = config.collective_pitch_offset_deg

    design_pitch_schedule(config, DESIGN_SPIN_RATE_RAD_S)

    assert config.station_pitch_deg == original_pitch
    assert config.collective_pitch_offset_deg == original_offset


def test_design_result_summary_is_readable():
    """The summary is what goes into a design log; it must contain the key numbers."""
    text = design_pitch_schedule(build_wing_config(), DESIGN_SPIN_RATE_RAD_S).summary()

    for token in ("thrust", "power", "FM", "pitch", "chord"):
        assert token in text, f"summary is missing {token!r}"
