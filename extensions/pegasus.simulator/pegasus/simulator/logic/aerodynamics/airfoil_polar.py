"""
| File: airfoil_polar.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Two-dimensional airfoil section polar models, C_l(alpha, Re) and C_d(alpha, Re),
|              valid over the full +/-180 deg angle-of-attack range required by a rotating wing.

PURPOSE
-------
A blade element on a spinning wing sweeps a wide range of angle of attack. Near the design
point the section runs attached at alpha ~ 4-6 deg, but during spin-up, descent, gusts, or
reversed-flow operation the element can see arbitrary alpha, including fully separated flow
and flow on the pressure side. A polar that is only valid pre-stall will therefore produce
non-physical forces during exactly the transients this project needs to study.

This module provides the section aerodynamics in two interchangeable forms:

1. ``AnalyticPolar``  - a parametric model: linear lift curve with a smooth stall blend into
                        Viterna-Corrigan post-stall behaviour, and a drag polar with an
                        explicit Reynolds-number scaling.
2. ``TablePolar``     - bilinear interpolation of a tabulated (alpha, Re) polar, for when
                        XFOIL / NeuralFoil / wind-tunnel data is available.

Both satisfy the same interface, ``evaluate(alpha_rad, reynolds) -> (c_l, c_d)``, so the
blade element solver is agnostic to which one is installed.

SIGN AND ANGLE CONVENTIONS
--------------------------
* ``alpha`` is the angle between the section chord line and the local relative wind,
  positive nose-up (leading edge into the wind), in radians.
* ``c_l`` is positive in the direction that generates thrust for a positive-pitch blade.
* ``c_d`` is always >= 0.
* Angles are radians everywhere in the public API. Degrees appear only in configuration
  fields, which are suffixed ``_deg`` without exception.

REFERENCES
----------
[1] Viterna, L. A., and Corrigan, R. D., "Fixed-Pitch Rotor Performance of Large Horizontal
    Axis Wind Turbines", NASA CP-2230, DOE Publication CONF-810752, 1982.
    -- the post-stall / deep-stall extrapolation used here.
[2] Leishman, J. G., "Principles of Helicopter Aerodynamics", 2nd ed., Cambridge University
    Press, 2006, Ch. 7. -- blade element section aerodynamics conventions.
[3] foil_out/GOE210_DESIGN_REPORT.md (this repository) -- the GOE 210 design data that the
    default analytic coefficients are calibrated against. See ``goe210_polar()``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

__all__ = [
    "AirfoilPolar",
    "AnalyticPolar",
    "TablePolar",
    "goe210_polar",
    "viterna_coefficients",
]


# ---------------------------------------------------------------------------------------
# Numerical guards
# ---------------------------------------------------------------------------------------

#: Smallest Reynolds number the drag scaling is evaluated at. Below this the section is
#: effectively stalled anyway and the power-law extrapolation would diverge.
_REYNOLDS_FLOOR = 1.0e3

#: Angle (rad) below which sin(alpha) is treated as zero in the Viterna lift expression,
#: which contains a cos^2(alpha)/sin(alpha) term that is singular at alpha = 0.
_SIN_EPS = 1.0e-6


class AirfoilPolar:
    """Abstract interface for a 2-D section polar.

    Implementations must be pure functions of (alpha, Re): no internal state may change
    between calls. This keeps the blade element solver re-entrant and makes the physics
    tests reproducible.
    """

    def evaluate(self, alpha_rad, reynolds) -> Tuple[np.ndarray, np.ndarray]:
        """Return (c_l, c_d) for the given angle of attack and Reynolds number.

        Args:
            alpha_rad: Angle of attack [rad]. Scalar or array of any shape.
            reynolds: Chord Reynolds number [-]. Broadcastable against ``alpha_rad``.

        Returns:
            Tuple of (c_l, c_d) as arrays with the broadcast shape of the inputs.
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------------------
# Viterna-Corrigan post-stall extrapolation
# ---------------------------------------------------------------------------------------

VITERNA_MAX_ASPECT_RATIO = 50.0
"""Upper bound of validity for :func:`viterna_cd_max` [-], from the source correlation."""


def viterna_cd_max(aspect_ratio: float) -> float:
    """Flat-plate maximum drag coefficient for a finite blade, Viterna eq. (13).

        c_d,max = 1.11 + 0.018 * AR,   for AR <= 50

    Source: Viterna, L. A. and Corrigan, R. D., "Fixed Pitch Rotor Performance of Large
    Horizontal Axis Wind Turbines", NASA Lewis Research Center, 1981 (NTRS 19830010962),
    equation (13), reproduced in ``resources/19830010962.pdf``.

    WHY THIS IS NOT A CONSTANT
        The 2-D flat-plate value of roughly 1.8 to 2.0 belongs to an airfoil of INFINITE
        span. A stubby blade sheds flow round its tip in deep stall and never reaches it.
        The source paper makes the point directly: a flat plate of aspect ratio 8 already
        measures below 1.3, and using the 2-D value is identified there as a specific
        failure of classical theory at high incidence.

        For this wing the planform is very stubby -- 0.30 m of span at 0.23 m mean chord,
        AR about 1.3 -- so the correlation gives about 1.13, not 1.8. Using the 2-D value
        would overstate deeply stalled drag by more than half.

    Args:
        aspect_ratio: Blade aspect ratio, span / mean chord [-]. Must be > 0.

    Returns:
        The maximum drag coefficient to hand to :func:`viterna_coefficients`.

    Raises:
        ValueError: If ``aspect_ratio`` is not positive.
    """
    if aspect_ratio <= 0.0:
        raise ValueError(f"aspect_ratio must be > 0; got {aspect_ratio!r}")

    # Beyond AR 50 the correlation is extrapolating past its stated data, and the 2-D
    # limit is the physically meaningful ceiling anyway. Clamping is honest; raising would
    # be unhelpful, since a very slender blade is not an error.
    bounded = min(float(aspect_ratio), VITERNA_MAX_ASPECT_RATIO)
    return 1.11 + 0.018 * bounded


def viterna_coefficients(
    alpha_stall_rad: float,
    cl_stall: float,
    cd_stall: float,
    cd_max: float,
) -> Tuple[float, float, float, float]:
    """Compute the four Viterna-Corrigan coefficients (A1, B1, A2, B2).

    The Viterna model represents a fully separated section as a flat plate of drag
    coefficient ``cd_max``, matched in value to the measured polar at the stall point:

        c_l(alpha) = A1 * sin(2*alpha) + A2 * cos^2(alpha) / sin(alpha)
        c_d(alpha) = B1 * sin^2(alpha) + B2 * cos(alpha)

    Args:
        alpha_stall_rad: Angle of attack at the stall / match point [rad], must be > 0.
        cl_stall: Lift coefficient at the match point [-].
        cd_stall: Drag coefficient at the match point [-].
        cd_max: Flat-plate maximum drag coefficient [-], typically 1.1 to 2.0.

    Returns:
        (A1, B1, A2, B2).

    Raises:
        ValueError: If ``alpha_stall_rad`` is not strictly inside (0, pi/2), which would
            make the matching expressions singular.
    """
    if not (0.0 < alpha_stall_rad < 0.5 * np.pi):
        raise ValueError(
            f"alpha_stall_rad must lie in (0, pi/2); got {alpha_stall_rad!r}"
        )

    sin_s = np.sin(alpha_stall_rad)
    cos_s = np.cos(alpha_stall_rad)

    b1 = cd_max
    a1 = 0.5 * b1
    a2 = (cl_stall - cd_max * sin_s * cos_s) * sin_s / (cos_s * cos_s)
    b2 = (cd_stall - cd_max * sin_s * sin_s) / cos_s

    return float(a1), float(b1), float(a2), float(b2)


def _viterna_evaluate(alpha, a1, b1, a2, b2, alpha_match_rad):
    """Evaluate the Viterna lift and drag for arbitrary alpha [rad], vectorised.

    The model is derived for ``alpha_match <= alpha <= pi/2`` and is mirrored into the
    remaining quadrants, which is the standard treatment for rotor blade elements
    operating in reversed flow.

    THE SINGULARITY, AND WHAT IS DONE ABOUT IT
        The Viterna lift expression contains a ``cos^2(alpha) / sin(alpha)`` term that
        diverges as alpha approaches zero. In the folded coordinate used here that happens
        at BOTH ends of the range: near alpha = 0 (attached flow, where the blended
        attached-flow branch is in control anyway) and near alpha = +/-180 deg, which is a
        section sitting at near-zero incidence in fully reversed flow. The second case is
        reachable by a real spinning wing -- an element in the reversed-flow region passes
        through it every revolution in fast translation -- so it cannot be left to
        diverge.

        The fix follows standard rotorcraft practice: Viterna is evaluated only outboard
        of the match angle, and inboard of it the lift is ramped linearly to zero at zero
        folded incidence. Zero lift at zero incidence is the correct physical limit for a
        separated section, and the ramp keeps the result continuous at the match point.

        The drag expression ``b1*sin^2 + b2*cos`` is bounded everywhere and needs no such
        treatment.

    Args:
        alpha: Angle of attack [rad], any shape.
        a1, b1, a2, b2: Viterna coefficients from :func:`viterna_coefficients`.
        alpha_match_rad: The angle the coefficients were matched at [rad], > 0.

    Returns:
        Tuple of (c_l, c_d).
    """
    alpha = np.asarray(alpha, dtype=float)

    # Fold into the first quadrant, remembering the sign and the "flow from behind" flag.
    sign = np.where(alpha < 0.0, -1.0, 1.0)
    a_abs = np.abs(alpha)
    # Reflect angles beyond 90 deg: a section at 120 deg behaves like one at 60 deg with
    # the chord reversed, which flips the lift sign.
    reflected = a_abs > 0.5 * np.pi
    a_fold = np.where(reflected, np.pi - a_abs, a_abs)
    lift_sign = sign * np.where(reflected, -1.0, 1.0)

    sin_a = np.sin(a_fold)
    cos_a = np.cos(a_fold)

    # Evaluate the Viterna lift on a floored angle so the singular term never blows up,
    # then ramp it down over the inboard region.
    a_eval = np.maximum(a_fold, alpha_match_rad)
    sin_e = np.sin(a_eval)
    cos_e = np.cos(a_eval)
    sin_e = np.where(np.abs(sin_e) < _SIN_EPS, _SIN_EPS, sin_e)

    c_l_outer = a1 * np.sin(2.0 * a_eval) + a2 * (cos_e * cos_e) / sin_e

    # Linear ramp to zero lift at zero folded incidence, applied only inboard of the
    # match angle. Outboard of it the ramp factor is exactly 1, so nothing changes there.
    ramp = np.clip(a_fold / max(alpha_match_rad, _SIN_EPS), 0.0, 1.0)
    c_l = ramp * c_l_outer

    c_d = b1 * sin_a * sin_a + b2 * cos_a

    return lift_sign * c_l, np.maximum(c_d, 0.0)


# ---------------------------------------------------------------------------------------
# Analytic polar
# ---------------------------------------------------------------------------------------

@dataclass
class AnalyticPolar(AirfoilPolar):
    """Parametric section polar: linear lift, quadratic drag bucket, Viterna post-stall.

    PRE-STALL LIFT
        c_l = cl_zero_alpha + cl_alpha_per_rad * alpha

    PRE-STALL DRAG
        c_d = [cd_min + cd_induced_factor * (c_l - cl_at_cd_min)^2] * (Re_ref / Re)^cd_re_exponent

        The Reynolds scaling is applied to the whole bucket rather than to the parasite
        term alone. That choice is empirical: it reproduces the GOE 210 design-report
        section data to better than ~5 percent across Re = 60k to 135k, whereas scaling
        only the parasite term does not. See ``goe210_polar`` for the calibration.

    POST-STALL
        Viterna-Corrigan, matched at (+/-) the stall angle, blended over
        ``stall_blend_deg`` so that c_l and c_d stay C0-continuous and visibly smooth.

    All fields carry SI units or are dimensionless; angles in this dataclass are in degrees
    and are converted internally.
    """

    # -- Lift ---------------------------------------------------------------------------
    cl_zero_alpha: float = 0.0
    """Lift coefficient at alpha = 0 [-]. Non-zero for a cambered section."""

    cl_alpha_per_rad: float = 2.0 * np.pi
    """Lift-curve slope [1/rad]. Thin-airfoil theory gives 2*pi; real low-Re sections
    are lower."""

    alpha_stall_pos_deg: float = 12.0
    """Positive stall angle [deg]: the Viterna match point on the suction side."""

    alpha_stall_neg_deg: float = -10.0
    """Negative stall angle [deg]. Must be negative."""

    stall_blend_deg: float = 2.0
    """Width of the smooth blend between the linear and Viterna branches [deg]."""

    # -- Drag ---------------------------------------------------------------------------
    cd_min: float = 0.012
    """Minimum section drag coefficient at the reference Reynolds number [-]."""

    cl_at_cd_min: float = 0.3
    """Lift coefficient at which the drag bucket is minimum [-]."""

    cd_induced_factor: float = 0.012
    """Curvature of the drag bucket [-], multiplying (c_l - cl_at_cd_min)^2."""

    reynolds_ref: float = 1.0e5
    """Reynolds number at which ``cd_min`` is defined [-]."""

    cd_re_exponent: float = 0.5
    """Exponent n in the drag scaling (Re_ref / Re)^n [-]. Laminar skin friction gives
    0.5; fully turbulent is nearer 0.2."""

    reynolds_min: float = 5.0e3
    """Lower clamp on Reynolds number [-], guarding the power-law scaling."""

    reynolds_max: float = 5.0e6
    """Upper clamp on Reynolds number [-]."""

    cd_max_flat_plate: float = 1.8
    """Flat-plate drag coefficient used by the Viterna extrapolation [-]."""

    def __post_init__(self) -> None:
        self._validate()
        self._build_viterna()

    # -- construction -------------------------------------------------------------------

    def _validate(self) -> None:
        if self.alpha_stall_pos_deg <= 0.0:
            raise ValueError("alpha_stall_pos_deg must be > 0")
        if self.alpha_stall_neg_deg >= 0.0:
            raise ValueError("alpha_stall_neg_deg must be < 0")
        if self.stall_blend_deg <= 0.0:
            raise ValueError("stall_blend_deg must be > 0")
        if self.cd_min < 0.0:
            raise ValueError("cd_min must be >= 0")
        if self.reynolds_ref <= _REYNOLDS_FLOOR:
            raise ValueError("reynolds_ref must be a physically meaningful Reynolds number")
        if self.reynolds_min < _REYNOLDS_FLOOR:
            raise ValueError(f"reynolds_min must be >= {_REYNOLDS_FLOOR}")
        if self.reynolds_max <= self.reynolds_min:
            raise ValueError("reynolds_max must exceed reynolds_min")
        if self.cd_max_flat_plate <= 0.0:
            raise ValueError("cd_max_flat_plate must be > 0")

    def _build_viterna(self) -> None:
        """Pre-compute the Viterna coefficients for both stall sides."""
        a_pos = np.deg2rad(self.alpha_stall_pos_deg)
        a_neg = np.deg2rad(self.alpha_stall_neg_deg)

        # Match the extrapolation to the attached-flow model evaluated at the stall angle,
        # at the reference Reynolds number.
        cl_pos = self._linear_cl(a_pos)
        cd_pos = self._bucket_cd(cl_pos, self.reynolds_ref)
        self._vit_pos = viterna_coefficients(a_pos, cl_pos, cd_pos, self.cd_max_flat_plate)

        # The negative side is matched on the magnitude of the angle; the lift sign is
        # restored by _viterna_evaluate's odd symmetry.
        cl_neg = self._linear_cl(a_neg)
        cd_neg = self._bucket_cd(cl_neg, self.reynolds_ref)
        self._vit_neg = viterna_coefficients(
            abs(a_neg), abs(cl_neg), cd_neg, self.cd_max_flat_plate
        )

        self._alpha_stall_pos_rad = a_pos
        self._alpha_stall_neg_rad = a_neg
        self._blend_rad = np.deg2rad(self.stall_blend_deg)

    # -- attached-flow branch -----------------------------------------------------------

    def _linear_cl(self, alpha_rad):
        return self.cl_zero_alpha + self.cl_alpha_per_rad * np.asarray(alpha_rad, dtype=float)

    def _bucket_cd(self, c_l, reynolds):
        """Attached-flow drag: a Reynolds-scaled parasite term plus a fixed bucket.

            c_d = cd_min * (Re_ref / Re)^n  +  cd_induced_factor * (c_l - cl_at_cd_min)^2

        ONLY the parasite term carries the Reynolds scaling. That is both the physical
        choice and a necessary one:

        * Physically, the Reynolds sensitivity of a low-Re section lives in the boundary
          layer and the laminar separation bubble, which set the parasite drag. The
          lift-dependent term is governed by the pressure distribution and is far less
          Reynolds sensitive.
        * Numerically, it is what makes the polar's best lift-to-drag point MOVE with
          Reynolds number. The optimum sits at
          c_l = sqrt(cl_at_cd_min^2 + cd_min*(Re_ref/Re)^n / cd_induced_factor), which
          rises as Reynolds number falls -- exactly the trend the GOE 210 report
          documents (c_l = 0.89 at Re 200k rising to 0.96 at Re 100k). Scaling the whole
          bucket instead would leave the optimum fixed, and a blade twisted to that fixed
          optimum would be at the wrong incidence over most of its span.
        """
        re = np.clip(np.asarray(reynolds, dtype=float), self.reynolds_min, self.reynolds_max)
        parasite = self.cd_min * (self.reynolds_ref / re) ** self.cd_re_exponent
        lift_dependent = self.cd_induced_factor * (c_l - self.cl_at_cd_min) ** 2
        return parasite + lift_dependent

    # -- public API ---------------------------------------------------------------------

    def evaluate(self, alpha_rad, reynolds):
        """Return (c_l, c_d); see :meth:`AirfoilPolar.evaluate`."""
        alpha = np.asarray(alpha_rad, dtype=float)
        re = np.asarray(reynolds, dtype=float)

        # Attached-flow branch, evaluated everywhere (cheap, and needed for the blend).
        cl_lin = self._linear_cl(alpha)
        cd_lin = self._bucket_cd(cl_lin, re)

        # Separated branch, one Viterna fit per stall side.
        cl_v_pos, cd_v_pos = _viterna_evaluate(
            alpha, *self._vit_pos, alpha_match_rad=self._alpha_stall_pos_rad
        )
        cl_v_neg, cd_v_neg = _viterna_evaluate(
            alpha, *self._vit_neg, alpha_match_rad=abs(self._alpha_stall_neg_rad)
        )
        cl_vit = np.where(alpha >= 0.0, cl_v_pos, cl_v_neg)
        cd_vit = np.where(alpha >= 0.0, cd_v_pos, cd_v_neg)

        # NOTE: the Viterna drag is deliberately left without a Reynolds correction.
        # Deep-stall drag is dominated by pressure drag behind a fully separated shear
        # layer, which is only weakly Reynolds dependent, whereas the attached bucket is
        # strongly so. Applying the attached-flow power law to the separated branch would
        # overstate the Reynolds sensitivity of a stalled section.

        # Smooth blend. weight = 0 -> attached, 1 -> separated.
        weight = self._stall_weight(alpha)

        c_l = (1.0 - weight) * cl_lin + weight * cl_vit
        c_d = (1.0 - weight) * cd_lin + weight * cd_vit

        return c_l, np.maximum(c_d, 0.0)

    def _stall_weight(self, alpha):
        """Smoothstep blending weight, 0 in attached flow and 1 past stall."""
        blend = self._blend_rad

        # Distance past the relevant stall angle, normalised by the blend width.
        past_pos = (alpha - self._alpha_stall_pos_rad) / blend
        past_neg = (self._alpha_stall_neg_rad - alpha) / blend
        past = np.where(alpha >= 0.0, past_pos, past_neg)

        t = np.clip(past, 0.0, 1.0)
        # Cubic smoothstep: C1-continuous at both ends of the blend.
        return t * t * (3.0 - 2.0 * t)


# ---------------------------------------------------------------------------------------
# Tabulated polar
# ---------------------------------------------------------------------------------------

@dataclass
class TablePolar(AirfoilPolar):
    """Bilinear interpolation of a tabulated polar on an (alpha, Re) grid.

    Use this when real XFOIL / NeuralFoil / tunnel data is available. The table must
    already span the full alpha range of interest; this class does not extrapolate in
    alpha beyond the supplied grid, it clamps. Wrap the data with a Viterna extension
    before tabulating if full +/-180 deg coverage is needed.

    Attributes:
        alpha_grid_rad: Strictly increasing angles of attack [rad], shape (n_alpha,).
        reynolds_grid: Strictly increasing Reynolds numbers [-], shape (n_re,).
        cl_table: Lift coefficients [-], shape (n_alpha, n_re).
        cd_table: Drag coefficients [-], shape (n_alpha, n_re).
    """

    alpha_grid_rad: np.ndarray
    reynolds_grid: np.ndarray
    cl_table: np.ndarray
    cd_table: np.ndarray

    def __post_init__(self) -> None:
        self.alpha_grid_rad = np.asarray(self.alpha_grid_rad, dtype=float)
        self.reynolds_grid = np.asarray(self.reynolds_grid, dtype=float)
        self.cl_table = np.asarray(self.cl_table, dtype=float)
        self.cd_table = np.asarray(self.cd_table, dtype=float)

        n_a = self.alpha_grid_rad.size
        n_r = self.reynolds_grid.size
        expected = (n_a, n_r)
        if self.cl_table.shape != expected or self.cd_table.shape != expected:
            raise ValueError(
                f"cl_table and cd_table must both have shape {expected}; got "
                f"{self.cl_table.shape} and {self.cd_table.shape}"
            )
        if n_a < 2 or n_r < 2:
            raise ValueError("both grids need at least two points for interpolation")
        if np.any(np.diff(self.alpha_grid_rad) <= 0.0):
            raise ValueError("alpha_grid_rad must be strictly increasing")
        if np.any(np.diff(self.reynolds_grid) <= 0.0):
            raise ValueError("reynolds_grid must be strictly increasing")

    def evaluate(self, alpha_rad, reynolds):
        """Return (c_l, c_d); see :meth:`AirfoilPolar.evaluate`."""
        alpha = np.asarray(alpha_rad, dtype=float)
        re = np.asarray(reynolds, dtype=float)
        alpha, re = np.broadcast_arrays(alpha, re)

        c_l = self._bilinear(self.cl_table, alpha, re)
        c_d = self._bilinear(self.cd_table, alpha, re)
        return c_l, np.maximum(c_d, 0.0)

    def _bilinear(self, table, alpha, re):
        a_grid = self.alpha_grid_rad
        r_grid = self.reynolds_grid

        # Clamp to the table bounds rather than extrapolating: a polar extrapolated off
        # the end of its grid is the classic source of silent non-physical rotor loads.
        a = np.clip(alpha, a_grid[0], a_grid[-1])
        r = np.clip(re, r_grid[0], r_grid[-1])

        ia = np.clip(np.searchsorted(a_grid, a) - 1, 0, a_grid.size - 2)
        ir = np.clip(np.searchsorted(r_grid, r) - 1, 0, r_grid.size - 2)

        ta = (a - a_grid[ia]) / (a_grid[ia + 1] - a_grid[ia])
        tr = (r - r_grid[ir]) / (r_grid[ir + 1] - r_grid[ir])

        f00 = table[ia, ir]
        f10 = table[ia + 1, ir]
        f01 = table[ia, ir + 1]
        f11 = table[ia + 1, ir + 1]

        return (
            f00 * (1.0 - ta) * (1.0 - tr)
            + f10 * ta * (1.0 - tr)
            + f01 * (1.0 - ta) * tr
            + f11 * ta * tr
        )


# ---------------------------------------------------------------------------------------
# GOE 210 calibration
# ---------------------------------------------------------------------------------------

def goe210_polar(aspect_ratio: Optional[float] = None) -> AnalyticPolar:
    """Analytic polar calibrated to the GOE 210 blade design report in this repository.

    Args:
        aspect_ratio: Blade aspect ratio, span / mean chord [-]. When given, the
            post-stall flat-plate drag coefficient is derived from it with
            :func:`viterna_cd_max` rather than taking the 2-D value. Pass this
            whenever the planform is known; the default exists only so the polar can
            be built standalone.

    CALIBRATION SOURCE
        ``foil_out/GOE210_DESIGN_REPORT.md``, section 8.1 "Station table". That table
        lists, for thirteen radial stations of the as-designed blade, the local angle of
        attack, lift coefficient, lift-to-drag ratio and Reynolds number. Those are the
        data the coefficients below reproduce.

    LIFT CALIBRATION
        Fitting c_l = cl_0 + cl_alpha * alpha to the station table gives
        cl_0 = 0.49, cl_alpha = 5.85 /rad. Residuals against the report:

            alpha [deg]   report c_l   model c_l   error
            4.4           0.96         0.939       -2.2 %
            4.6           0.97         0.960       -1.0 %
            4.7           0.97         0.970        0.0 %
            5.1           1.01         1.011       +0.1 %
            5.5           1.04         1.052       +1.2 %

        The slope is below the thin-airfoil 2*pi, as expected for a 6.6 percent section at
        Re of order 1e5. The fit is also consistent with the report's independent remark
        that the blade root reaches c_l = 1.3 at alpha = 8 deg (model: 1.307).

    DRAG CALIBRATION
        The drag bucket is fitted by least squares to TWO independent constraints from the
        report, because matching only the first gets the bucket's depth right while
        leaving its minimum in the wrong place:

        1. All thirteen stations of the section 8.1 table, as (Re, c_l, c_d) triples with
           c_d = c_l / (c_l/c_d). Residual: 1.8 percent mean, 7.0 percent worst.
        2. The LOCATION of the best lift-to-drag point, from section 5.4, which lists
           c_l = 0.89 to 0.96 at maximum c_l/c_d. For this bucket shape the optimum sits
           at c_l = sqrt(cl_at_cd_min^2 + cd_min / cd_induced_factor), so that pins the
           ratio of the bucket's depth to its curvature.

        Because only the parasite term is Reynolds-scaled (see :meth:`_bucket_cd`), the
        best lift-to-drag point MOVES with Reynolds number, and the fit reproduces the
        report's measured trend closely:

            Re [k]   c_l at max L/D            max c_l/c_d
                     model / report            model / report
            100      0.962 / 0.96              62.5 / 63.6
            120      0.940 / 0.94              68.5 / 69.5
            140      0.922 / 0.92              74.0 / 74.4
            160      0.908 / 0.91              79.1 / 78.6
            200      0.886 / 0.89              88.6 / 85.2

        Over the blade's own Reynolds range the best angle of attack therefore runs from
        4.27 deg at Re 135k to 5.33 deg at Re 60k, which is why the report's optimiser
        placed its stations at 4.4 to 5.5 deg rather than at a single value.

        Constraint 2 matters more than it looks. Fitting drag at the operating point alone
        reproduces thrust and power at the design condition perfectly well, but puts the
        polar's best lift-to-drag angle at 6.4 deg instead of 4.2 deg. Any attempt to
        RE-DESIGN the blade for a different span then twists it to the wrong incidence --
        which is exactly what :func:`~.wing_design.design_pitch_schedule` does, and how
        this error was found.

        ``cd_re_exponent`` comes out at 0.60 against a reference Re of 135.2k. An exponent
        between the laminar 0.5 and turbulent 0.2 limits is the expected signature of a
        section with a transitioning laminar separation bubble in this Reynolds range.

        ``cl_at_cd_min`` is only weakly determined by the available data, since every
        station runs near c_l = 1.0. It is fixed at 0.40, a physically sensible bucket
        centre for a 6.6 percent cambered section; values from 0.0 to 0.4 fit the data
        equally well once ``cd_min`` and ``cd_induced_factor`` are refitted alongside.

    STALL
        The report does not tabulate stall. ``alpha_stall_pos_deg`` = 9.0 is set just past
        the highest incidence the report exercises (8 deg at c_l = 1.3, described there as
        "near the edge of the polar"), which places c_l,max at 1.41. Treat the post-stall
        branch as an engineering extrapolation, not as validated data -- the physics test
        suite checks that it is smooth and bounded, not that it is accurate.

    Returns:
        A fresh :class:`AnalyticPolar` instance. A new object is returned on every call so
        that callers may mutate it without disturbing other users.
    """
    return AnalyticPolar(
        # Lift: fitted to GOE210_DESIGN_REPORT.md section 8.1.
        cl_zero_alpha=0.49,
        cl_alpha_per_rad=5.85,
        alpha_stall_pos_deg=9.0,
        alpha_stall_neg_deg=-8.0,
        stall_blend_deg=2.0,
        # Drag: bucket shape and Reynolds power law, least-squares fitted to the station
        # table AND the location of the best lift-to-drag point. See the docstring.
        cd_min=0.011209,
        cl_at_cd_min=0.7031,
        cd_induced_factor=0.030879,
        reynolds_ref=1.352e5,
        cd_re_exponent=0.5705,
        reynolds_min=1.0e4,
        reynolds_max=1.0e6,
        # Post-stall: derived from the planform when it is known. 1.8 is the 2-D
        # flat-plate value and is only a fallback -- see viterna_cd_max for why a stubby
        # blade never reaches it.
        cd_max_flat_plate=(
            1.8 if aspect_ratio is None else viterna_cd_max(aspect_ratio)
        ),
    )
