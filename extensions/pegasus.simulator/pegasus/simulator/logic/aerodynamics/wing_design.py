"""
| File: wing_design.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Re-deriving a blade's pitch schedule when its geometry changes.

WHY THIS EXISTS
---------------
The default build schedule in :mod:`wing_config` was optimised by the GOE 210 design
report for one specific geometry: a wing spanning r = 0.2 to 0.5 m turning at 20 rad/s.
Its twist -- about 8 degrees from root to tip -- is not decoration. At every station the
pitch is set so that the blade meets the local relative wind at the angle of attack where
that section has its best lift-to-drag ratio:

    theta(r) = alpha_best + phi(r)

and the inflow angle phi(r) = atan(v_i / (Omega * r)) depends on the radius, the spin rate
AND the induced velocity, which in turn depends on the loading. Change the span, the spin
rate or the blade count, and every one of those changes with it.

So simply moving ``root_radius_m`` and ``tip_radius_m`` to fit a different airframe does
NOT give a working blade: it gives the old twist distribution applied at the wrong radii,
with the sections running at whatever angle of attack happens to result. They will be
badly off their best lift-to-drag ratio, and the inboard ones may be stalled.

:func:`design_pitch_schedule` solves that. It is the same fixed-point iteration the design
report used for its own blade ("Pitch is then set so alpha equals that value,
theta = alpha_best + phi, iterated 14 times"), run against the model in this package.

REFERENCES
----------
[1] foil_out/GOE210_DESIGN_REPORT.md section 4.4(b), "Best CL/CD at every station".
[2] Leishman, J. G., "Principles of Helicopter Aerodynamics", 2nd ed., 2006, ch. 3.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .rotating_wing import RotatingWing
from .wing_config import RotatingWingConfig

__all__ = [
    "PitchDesignResult",
    "best_lift_to_drag_alpha",
    "design_pitch_schedule",
    "scale_chord_to_span",
]


@dataclass
class PitchDesignResult:
    """Outcome of a pitch-schedule design run."""

    station_radius_m: List[float]
    """Radii the schedule is tabulated at [m]."""

    station_pitch_deg: List[float]
    """Designed pitch at each station [deg], chord-line angle to the rotor plane."""

    station_chord_m: List[float]
    """Chord at each station [m], carried through unchanged from the input."""

    achieved_alpha_deg: np.ndarray
    """Angle of attack each station actually reaches [deg]."""

    target_alpha_deg: float
    """The angle of attack that was aimed for [deg]."""

    thrust_n: float
    """Thrust of the designed blade at the design spin rate [N]."""

    aero_power_w: float
    """Aerodynamic power of the designed blade at the design spin rate [W]."""

    figure_of_merit: float
    """Hover figure of merit of the designed blade [-]."""

    converged: bool
    """Whether the iteration met its tolerance."""

    iterations: int
    """Iterations taken."""

    max_alpha_error_deg: float
    """Largest departure from the target angle of attack across the span [deg]."""

    def summary(self) -> str:
        """One-paragraph human-readable report, for a design log."""
        status = "converged" if self.converged else "DID NOT CONVERGE"
        return (
            f"Pitch schedule designed for alpha = {self.target_alpha_deg:.2f} deg "
            f"({status} in {self.iterations} iterations, "
            f"max alpha error {self.max_alpha_error_deg:.3f} deg)\n"
            f"  thrust {self.thrust_n:.3f} N ({self.thrust_n / 9.80665 * 1000:.0f} g), "
            f"power {self.aero_power_w:.2f} W, FM {self.figure_of_merit:.3f}\n"
            f"  radii  [m]   : {[round(r, 4) for r in self.station_radius_m]}\n"
            f"  chord  [m]   : {[round(c, 4) for c in self.station_chord_m]}\n"
            f"  pitch  [deg] : {[round(p, 2) for p in self.station_pitch_deg]}"
        )


#: Angle of attack to keep clear of stall when choosing a design point [deg]. A blade
#: designed exactly at its best lift-to-drag angle can have no margin left if that angle
#: happens to sit against the stall boundary, which is what occurs at very low Reynolds
#: number: the drag bucket deepens, the optimum climbs, and it eventually runs into stall.
#: Designing with a margin is ordinary practice and it also keeps the design iteration
#: well conditioned, since stations are not crossing in and out of stall between passes.
DEFAULT_STALL_MARGIN_DEG = 2.0


def best_lift_to_drag_alpha(
    polar,
    reynolds: float,
    alpha_search_deg: Tuple[float, float] = (-2.0, 12.0),
    samples: int = 281,
    stall_margin_deg: float = DEFAULT_STALL_MARGIN_DEG,
) -> float:
    """Find the angle of attack with the highest lift-to-drag ratio at a Reynolds number.

    Args:
        polar: An :class:`~.airfoil_polar.AirfoilPolar`.
        reynolds: Chord Reynolds number to evaluate at [-].
        alpha_search_deg: Inclusive search bounds [deg]. The default spans the attached
            range; searching past stall would find spurious optima on the Viterna branch.
        samples: Number of samples across the search range.
        stall_margin_deg: Angle to stay below the polar's stall angle [deg]. Applied only
            when the polar declares a stall angle, which the analytic polar does and a
            tabulated one does not.

    Returns:
        The best angle of attack [deg], never closer to stall than ``stall_margin_deg``.
    """
    lower, upper = alpha_search_deg

    # Respect the polar's own stall angle when it has one.
    stall_deg = getattr(polar, "alpha_stall_pos_deg", None)
    if stall_deg is not None:
        upper = min(upper, float(stall_deg) - stall_margin_deg)
    if upper <= lower:
        return float(lower)

    alpha_deg = np.linspace(lower, upper, samples)
    c_l, c_d = polar.evaluate(np.deg2rad(alpha_deg), reynolds)

    # Guard against division by a vanishing drag, which would otherwise win the search.
    ratio = c_l / np.maximum(c_d, 1.0e-9)
    return float(alpha_deg[int(np.argmax(ratio))])


def scale_chord_to_span(
    station_radius_m,
    station_chord_m,
    new_root_radius_m: float,
    new_tip_radius_m: float,
) -> Tuple[List[float], List[float]]:
    """Map a chord distribution onto a new span, preserving the planform shape.

    The stations are re-expressed in normalised span, then placed on the new span, and the
    chords are scaled by the span ratio. That keeps the blade geometrically similar, so its
    aspect ratio and taper are unchanged.

    This is a reasonable starting point for a resized blade, but it is NOT a re-optimised
    design: the Reynolds numbers change with the new chord and speed, which moves the best
    lift-to-drag angle of attack slightly. Follow it with :func:`design_pitch_schedule`.

    Args:
        station_radius_m: Original station radii [m].
        station_chord_m: Original chords [m].
        new_root_radius_m: Root radius of the resized blade [m].
        new_tip_radius_m: Tip radius of the resized blade [m].

    Returns:
        Tuple of (new_radii, new_chords), both lists of floats.

    Raises:
        ValueError: If the new span is not positive.
    """
    radii = np.asarray(station_radius_m, dtype=float)
    chords = np.asarray(station_chord_m, dtype=float)

    if new_tip_radius_m <= new_root_radius_m:
        raise ValueError("new_tip_radius_m must exceed new_root_radius_m")

    old_span = float(radii[-1] - radii[0])
    new_span = float(new_tip_radius_m - new_root_radius_m)
    if old_span <= 0.0:
        raise ValueError("the original station table has zero span")

    normalised = (radii - radii[0]) / old_span
    new_radii = new_root_radius_m + normalised * new_span
    new_chords = chords * (new_span / old_span)

    return [float(r) for r in new_radii], [float(c) for c in new_chords]


def design_pitch_schedule(
    config: RotatingWingConfig,
    spin_rate_rad_s: float,
    target_alpha_deg: Optional[float] = None,
    per_station_best_ld: bool = True,
    max_iterations: int = 40,
    tolerance_deg: float = 0.01,
    relaxation: float = 0.7,
) -> PitchDesignResult:
    """Solve for the pitch schedule that puts every station at a chosen angle of attack.

    THE ITERATION
        Each pass computes the blade's loads with the current pitch, reads the inflow angle
        phi at every station, and sets

            theta_new = alpha_target + phi

        under-relaxed to keep the induced velocity's response stable. Because phi depends
        on the loading and the loading depends on theta, this is a fixed point, reached in
        a handful of passes.

    Args:
        config: The geometry to design for. Its span, chord distribution, blade count,
            atmosphere and polar are used; its existing pitch schedule is only a starting
            guess and is replaced.
        spin_rate_rad_s: Spin rate to design at [rad/s]. Must be non-zero.
        target_alpha_deg: A single angle of attack to aim for at every station [deg].
            Ignored unless ``per_station_best_ld`` is False. When both are left at their
            defaults, each station is given its own best lift-to-drag angle.
        per_station_best_ld: When True (the default), every station is designed to its own
            best lift-to-drag angle of attack, evaluated at that station's local Reynolds
            number. This is the design report's method and reproduces its built twist.
            Set False to hold one angle of attack across the whole span.
        max_iterations: Iteration cap.
        tolerance_deg: Convergence tolerance on the pitch change between passes [deg].
        relaxation: Under-relaxation factor in (0, 1].

    Returns:
        A :class:`PitchDesignResult`. Feed ``station_radius_m``, ``station_chord_m`` and
        ``station_pitch_deg`` straight back into the configuration.

    Raises:
        ValueError: If the spin rate is zero or the relaxation is out of range.

    Example:
        >>> cfg = RotatingWingConfig(root_radius_m=0.09, tip_radius_m=0.24)
        >>> result = design_pitch_schedule(cfg, spin_rate_rad_s=20.0)
        >>> result.converged
        True
    """
    if spin_rate_rad_s == 0.0:
        raise ValueError("spin_rate_rad_s must be non-zero to define an inflow angle")
    if not (0.0 < relaxation <= 1.0):
        raise ValueError("relaxation must lie in (0, 1]")

    work = copy.deepcopy(config)
    work.enabled = True

    # Choose the target angle of attack if one was not given. A mid-span station at the
    # design spin rate is representative of the blade's Reynolds range.
    if target_alpha_deg is None:
        mid_radius = 0.5 * (work.root_radius_m + work.tip_radius_m)
        mid_chord = float(
            np.interp(
                mid_radius,
                np.asarray(work.station_radius_m, dtype=float),
                np.asarray(work.station_chord_m, dtype=float),
            )
        )
        representative_re = (
            abs(spin_rate_rad_s) * mid_radius * mid_chord / work.kinematic_viscosity_m2_s
        )
        target_alpha_deg = best_lift_to_drag_alpha(work.polar, representative_re)

    # Design on the tabulated stations themselves, so the result can be written straight
    # back into the configuration.
    design_radii = np.asarray(work.station_radius_m, dtype=float)
    pitch_deg = np.asarray(work.station_pitch_deg, dtype=float).copy()

    converged = False
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        work.station_pitch_deg = [float(p) for p in pitch_deg]
        work.collective_pitch_offset_deg = 0.0
        wing = RotatingWing(work)
        loads = wing.hover_loads(spin_rate_rad_s)

        # Inflow angle phi = theta - alpha, evaluated on the solver's own stations and
        # interpolated back onto the tabulated ones.
        solver_phi_deg = np.rad2deg(wing.pitch_rad) - np.rad2deg(loads.alpha_rad[0])
        phi_at_stations = np.interp(design_radii, wing.radius_m, solver_phi_deg)

        if per_station_best_ld:
            # Each station is given its OWN best lift-to-drag angle, evaluated at its
            # local Reynolds number. This is the design report's method: section Reynolds
            # number falls from about 135k at mid-span to 60k at the tip, and the best
            # angle of attack rises as it does, which is why the report's own stations
            # run at 4.4 deg inboard and 5.5 deg outboard rather than at one value.
            station_re = np.interp(design_radii, wing.radius_m, loads.reynolds[0])
            target_per_station = np.array(
                [best_lift_to_drag_alpha(work.polar, re) for re in station_re]
            )
        else:
            target_per_station = np.full_like(design_radii, target_alpha_deg)

        new_pitch = target_per_station + phi_at_stations
        change = float(np.max(np.abs(new_pitch - pitch_deg)))
        pitch_deg = pitch_deg + relaxation * (new_pitch - pitch_deg)

        if change < tolerance_deg:
            converged = True
            break

    # Evaluate the final design once more so the reported performance is the delivered one.
    work.station_pitch_deg = [float(p) for p in pitch_deg]
    wing = RotatingWing(work)
    loads = wing.hover_loads(spin_rate_rad_s)
    achieved_alpha = np.rad2deg(loads.alpha_rad[0])

    return PitchDesignResult(
        station_radius_m=[float(r) for r in design_radii],
        station_pitch_deg=[float(p) for p in pitch_deg],
        station_chord_m=[float(c) for c in work.station_chord_m],
        achieved_alpha_deg=achieved_alpha,
        target_alpha_deg=float(np.mean(achieved_alpha)) if per_station_best_ld
        else float(target_alpha_deg),
        thrust_n=loads.thrust_n,
        aero_power_w=loads.aero_power_w,
        figure_of_merit=loads.figure_of_merit(
            work.air_density_kg_m3, work.disk_area_m2()
        ),
        converged=converged,
        iterations=iterations,
        max_alpha_error_deg=float(np.max(np.abs(achieved_alpha - np.mean(achieved_alpha))))
        if per_station_best_ld
        else float(np.max(np.abs(achieved_alpha - target_alpha_deg))),
    )
