"""
| File: wing_params.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: THE single file to edit when changing the spinning-wing dynamics.

HOW TO USE THIS FILE
--------------------
Everything that defines the wing's physics is a module-level constant below. Change a
number here and every sweep, test and simulation picks it up. Nothing else needs editing.

    from examples.utils.wing_params import build_wing_config
    cfg = build_wing_config()                 # as-designed GOE 210 wing
    cfg = build_wing_config(enabled=False)    # no-wing baseline
    cfg = build_wing_config(collective_pitch_offset_deg=-2.0, num_blades=4)

Any field of
:class:`~pegasus.simulator.logic.aerodynamics.wing_config.RotatingWingConfig` can also be
passed as a keyword to :func:`build_wing_config` for a one-off override, without editing
this file. Keyword overrides win over the constants below.

WHERE THE DEFAULTS COME FROM
----------------------------
``foil_out/GOE210_DESIGN_REPORT.md``, section 1 "Decision and design sheet". That is a
blade-element-momentum optimisation of a GOE 210 section for hover at 20 rad/s, with the
wing spanning r = 0.2 to 0.5 m, two blades, rho = 1.2 kg/m^3. The report predicts 3.74 N
of lift and 7.31 W of aerodynamic power at the design point.

The model in this repository reproduces that independently:

    Quantity              Report     Model     Error
    Thrust @ 20 rad/s     3.740 N    3.701 N   -1.0 %
    Thrust @ 15 rad/s     211 g      212 g     +0.6 %
    Aero power @ 20       7.31 W     7.01 W    -4.1 %
    Figure of merit       0.722      0.740     +2.5 %

and matches the report's per-station angle of attack to within 0.07 deg, lift coefficient
to within 0.01, and induced velocity to within 0.02 m/s. See
``examples/controller_tests/physics_tests/`` for the tests that assert this.

A NOTE ON GEOMETRY
------------------
The default span, r = 0.2 to 0.5 m, is the span the aerofoil was DESIGNED for and is the
only geometry for which the numbers above are validated. It is larger than the stock Iris
arm, so on that airframe the panel extends outboard past the motors. If the wing must fit
inboard of the motors instead, change ``ROOT_RADIUS_M`` and ``TIP_RADIUS_M`` below -- but
be aware that the chord and pitch schedule was optimised for the original span, and the
sections will no longer be running at their best lift-to-drag ratio. Re-optimising the
schedule for a new span is a design task, not a configuration change.
"""

from __future__ import annotations

import os
import sys
from typing import Any, Dict, Optional

# --------------------------------------------------------------------------------------
# Import the model. The aerodynamics subpackage is pure NumPy and has no Isaac Sim
# dependency, but importing it through ``pegasus.simulator.logic`` pulls in that package's
# __init__, which does. Import it directly by path so this module works both inside Isaac
# Sim and under a plain Python interpreter running the offline physics tests.
# --------------------------------------------------------------------------------------

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_AERO_DIR = os.path.join(
    _REPO_ROOT,
    "extensions", "pegasus.simulator", "pegasus", "simulator", "logic", "aerodynamics",
)


def _load_aerodynamics():
    """Import the aerodynamics package standalone, bypassing the Pegasus package chain."""
    if "spinning_wing_aero" in sys.modules:
        return sys.modules["spinning_wing_aero"]

    import importlib.util

    init_path = os.path.join(_AERO_DIR, "__init__.py")
    if not os.path.isfile(init_path):
        raise ImportError(
            f"Could not find the aerodynamics package at {_AERO_DIR!r}. "
            "wing_params.py expects to live at <repo>/examples/utils/."
        )

    spec = importlib.util.spec_from_file_location(
        "spinning_wing_aero", init_path, submodule_search_locations=[_AERO_DIR]
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["spinning_wing_aero"] = module
    spec.loader.exec_module(module)
    return module


_aero = _load_aerodynamics()

InflowModel = _aero.InflowModel
RotatingWing = _aero.RotatingWing
RotatingWingConfig = _aero.RotatingWingConfig
WingLoads = _aero.WingLoads
AirfoilPolar = _aero.AirfoilPolar
AnalyticPolar = _aero.AnalyticPolar
TablePolar = _aero.TablePolar
goe210_polar = _aero.goe210_polar
viterna_coefficients = _aero.viterna_coefficients
viterna_cd_max = _aero.viterna_cd_max
design_pitch_schedule = _aero.design_pitch_schedule
scale_chord_to_span = _aero.scale_chord_to_span
best_lift_to_drag_alpha = _aero.best_lift_to_drag_alpha


# ======================================================================================
#  EDIT BELOW THIS LINE
# ======================================================================================

# -- Master switch ---------------------------------------------------------------------

WING_ENABLED = True
"""Set False for the no-wing baseline. The ``--no-wing`` command-line flag overrides
this, so there is normally no reason to change it here."""


# -- Atmosphere ------------------------------------------------------------------------

AIR_DENSITY_KG_M3 = 1.2
"""Air density [kg/m^3]. The design report used 1.2 throughout. Sea-level ISA is 1.225."""

KINEMATIC_VISCOSITY_M2_S = 1.5e-5
"""Kinematic viscosity [m^2/s], used for the chord Reynolds number."""


# -- Blade planform --------------------------------------------------------------------

NUM_BLADES = 2
"""Number of aerofoil panels on the airframe. The design report's blade schedule assumes
two. Changing this changes the solidity and therefore the induced velocity, so the lift
does NOT simply scale with blade count."""

ROTATION_SENSE = -1
"""Handedness of the panels: the sign of body yaw rate they are built to lift in [-].

MUST MATCH THE DIRECTION THE AIRFRAME ACTUALLY SPINS, or the panels push down instead of
up. A rotor blade is a handed object: mirror it and it still works, but only turning the
other way.

This airframe spins negative. All four motors turn the same way
(``rot_dir = [-1, -1, -1, -1]``), so their reaction torques add instead of cancelling and
drive the body to a negative yaw rate -- measured at about -27 rad/s in simulation. Hence
-1: the panels are built left-handed.

The design report describes a conventional right-handed rotor, which is +1. Mirroring is
an exact symmetry of the blade element equations, so thrust, torque and power have
identical magnitudes either way; only the sense that produces them flips. Set +1 to model
the wrong-handed wing deliberately, as an ablation."""

BLADE_AZIMUTHS_DEG = None
"""Azimuth of each panel's span axis about +z measured from +x [deg], or None for uniform
spacing. With None and NUM_BLADES = 2 the panels sit at 0 and 180 deg.

Set this explicitly to model an asymmetric installation, e.g. [0.0, 90.0] for two panels
on adjacent arms, which produces a net in-plane force and a rolling moment that the
symmetric arrangement cancels."""

ROOT_RADIUS_M = 0.200
"""Inboard end of the lifting surface, from the spin axis [m]."""

TIP_RADIUS_M = 0.500
"""Outboard end of the lifting surface [m]."""

BLADE_Z_OFFSET_M = 0.0
"""Height of the wing plane above the body origin [m]. Non-zero values give the wing's
in-plane drag a moment arm about the roll and pitch axes, which matters for a panel
mounted above or below the arm centreline."""


# -- Build schedule --------------------------------------------------------------------
# foil_out/GOE210_DESIGN_REPORT.md section 1. Pitch is the chord-line angle to the rotor
# plane. Chord and pitch are linearly interpolated between these stations.

STATION_RADIUS_M = [0.200, 0.250, 0.300, 0.350, 0.400, 0.425, 0.450, 0.475, 0.500]
"""Radii of the tabulated stations [m], strictly increasing."""

STATION_CHORD_M = [0.300, 0.300, 0.297, 0.280, 0.197, 0.153, 0.124, 0.099, 0.080]
"""Chord at each station [m]."""

STATION_PITCH_DEG = [24.7, 22.6, 20.8, 19.4, 17.6, 16.3, 16.3, 16.6, 16.5]
"""Chord-line angle to the rotor plane at each station [deg]. Note the strong twist, about
8 deg root to tip: that is the main efficiency lever, not the chord."""

COLLECTIVE_PITCH_OFFSET_DEG = 0.0
"""Rigid offset added to every station's pitch [deg].

The most useful single knob in this file. Positive increases angle of attack everywhere,
raising lift until the blade stalls; negative unloads it. Also the right place to
represent a known built-in twist error: the design report measured 10-15 deg of
aeroelastic twist on an under-stiff 5 mm rod, which would appear here as a large negative
offset."""


# -- Structural properties (mass and inertia) ------------------------------------------

BLADE_AREAL_DENSITY_KG_M2 = 1.0
"""Mass per unit planform area of one panel [kg/m^2].

A carbon-skinned foam-core panel with an internal spar is roughly 0.8-1.5 kg/m^2 at this
scale. With the default planform this gives 67 g per blade, 133 g for the pair. Replace
with a weighed value once the panels exist."""

BLADE_MASS_KG = None
"""Mass of ONE panel [kg], or None to derive it from the areal density above."""

APPLY_INERTIA_TO_BODY = True
"""Add the wing's mass and inertia to the airframe rigid body at spawn.

This matters more than it sounds. The wing has no USD geometry, so without this it is
aerodynamically present but structurally massless. On the stock Iris the default panels
add 9 percent to mass but **28 percent to I_zz** -- the spin axis -- and 50 percent to
I_yy, which changes spin-up time and the entire yaw-axis response.

Set False only to study the aerodynamics in isolation, or if the airframe USD asset
already accounts for the panels."""


# -- Section aerodynamics --------------------------------------------------------------

USE_GOE210_POLAR = True
"""Use the GOE 210 polar calibrated against the design report. Set False to supply your
own via POLAR_OVERRIDE."""

POLAR_OVERRIDE = None
"""An :class:`AirfoilPolar` instance to use instead of the GOE 210 calibration, or None.

To use measured or XFOIL data, build a TablePolar:

    import numpy as np
    POLAR_OVERRIDE = TablePolar(
        alpha_grid_rad=np.deg2rad([-180, ..., 180]),
        reynolds_grid=[5e4, 1e5, 2e5],
        cl_table=...,   # shape (n_alpha, n_re)
        cd_table=...,
    )

Remember that a rotating wing sees the full +/-180 deg range during spin-up and in
reversed flow, so extend the table before tabulating -- TablePolar clamps at its grid
edges rather than extrapolating."""


# -- Induced velocity and numerics -------------------------------------------------------

INFLOW_MODEL = InflowModel.BEMT
"""One of InflowModel.BEMT, InflowModel.GLAUERT, InflowModel.NONE.

BEMT is the default and is what the design report used; it resolves the radial inflow
distribution and is accurate in hover and axial flight. GLAUERT applies a single uniform
inflow with an edgewise correction and is better behaved in fast translation. NONE
disables induced velocity entirely, which over-predicts thrust by roughly 25-30 percent
and exists for debugging."""

NUM_STATIONS = 24
"""Spanwise integration stations per blade. The solution is grid-converged here: going to
50 stations changes thrust by under 0.2 percent."""

INFLOW_MAX_ITERATIONS = 32
"""Iteration cap for the induced-velocity solve. At the design point it converges in
about 12."""

INFLOW_TOLERANCE_M_S = 1.0e-4
"""Convergence tolerance on induced velocity [m/s]."""

INFLOW_RELAXATION = 0.5
"""Under-relaxation factor, in (0, 1]. Lower is more stable near stall."""

USE_TIP_LOSS = True
"""Apply the Prandtl tip-loss factor. The design report did."""

USE_ROOT_LOSS = False
"""Apply the Prandtl root-loss factor.

Leave this off. The report applied tip loss only, and turning root loss on drops predicted
thrust about 10 percent below the report. Physically the blade root is bonded to the
airframe arm rather than being a free end, so a root-loss correction is not the right
model here."""

TIP_LOSS_FLOOR = 0.05
"""Lower clamp on the Prandtl factor, as used by the design report."""

MAX_RELATIVE_SPEED_M_S = 200.0
"""Ceiling on section relative speed [m/s], so that a diverging controller produces a
bounded force instead of destroying the integration."""


# -- Moment bookkeeping ------------------------------------------------------------------

MOMENT_REFERENCE_OFFSET_M = [0.0, 0.0, 0.0]
"""Vector from the body origin to the moment reference point [m], body frame. Zero is
correct when the body origin is the centre of mass, which holds for the stock Iris."""


# ======================================================================================
#  EDIT ABOVE THIS LINE
# ======================================================================================


def build_wing_config(**overrides: Any) -> RotatingWingConfig:
    """Build a :class:`RotatingWingConfig` from the constants above.

    Args:
        **overrides: Any field name of ``RotatingWingConfig``. Keyword overrides take
            precedence over the module constants, which makes this the right hook for
            command-line flags and for parameter sweeps.

    Returns:
        A validated configuration object. A new instance is returned on every call, so
        callers may mutate it freely.

    Raises:
        TypeError: If an override names a field that does not exist, which catches typos
            that would otherwise be silently ignored.
        ValueError: If the resulting configuration violates a physical invariant.

    Example:
        >>> cfg = build_wing_config(collective_pitch_offset_deg=-2.0)
        >>> cfg.collective_pitch_offset_deg
        -2.0
    """
    # The post-stall branch needs the blade's aspect ratio, so the planform is measured
    # first and the polar built from it. A stubby blade sheds flow round its tip in deep
    # stall and never reaches the 2-D flat-plate drag coefficient; Viterna's own
    # correlation, eq. (13), puts this blade at about 1.13 rather than 1.8.
    _span_m = TIP_RADIUS_M - ROOT_RADIUS_M
    _blade_area_m2 = 0.5 * sum(
        (STATION_CHORD_M[i] + STATION_CHORD_M[i + 1])
        * (STATION_RADIUS_M[i + 1] - STATION_RADIUS_M[i])
        for i in range(len(STATION_RADIUS_M) - 1)
    )
    _mean_chord_m = _blade_area_m2 / _span_m
    _blade_aspect_ratio = _span_m / _mean_chord_m

    polar = POLAR_OVERRIDE if POLAR_OVERRIDE is not None else (
        goe210_polar(aspect_ratio=_blade_aspect_ratio) if USE_GOE210_POLAR
        else AnalyticPolar()
    )

    settings: Dict[str, Any] = dict(
        enabled=WING_ENABLED,
        air_density_kg_m3=AIR_DENSITY_KG_M3,
        kinematic_viscosity_m2_s=KINEMATIC_VISCOSITY_M2_S,
        num_blades=NUM_BLADES,
        rotation_sense=ROTATION_SENSE,
        blade_azimuths_deg=BLADE_AZIMUTHS_DEG,
        root_radius_m=ROOT_RADIUS_M,
        tip_radius_m=TIP_RADIUS_M,
        blade_z_offset_m=BLADE_Z_OFFSET_M,
        station_radius_m=list(STATION_RADIUS_M),
        station_chord_m=list(STATION_CHORD_M),
        station_pitch_deg=list(STATION_PITCH_DEG),
        collective_pitch_offset_deg=COLLECTIVE_PITCH_OFFSET_DEG,
        blade_areal_density_kg_m2=BLADE_AREAL_DENSITY_KG_M2,
        blade_mass_kg=BLADE_MASS_KG,
        apply_inertia_to_body=APPLY_INERTIA_TO_BODY,
        polar=polar,
        inflow_model=INFLOW_MODEL,
        num_stations=NUM_STATIONS,
        inflow_max_iterations=INFLOW_MAX_ITERATIONS,
        inflow_tolerance_m_s=INFLOW_TOLERANCE_M_S,
        inflow_relaxation=INFLOW_RELAXATION,
        use_tip_loss=USE_TIP_LOSS,
        use_root_loss=USE_ROOT_LOSS,
        tip_loss_floor=TIP_LOSS_FLOOR,
        moment_reference_offset_m=list(MOMENT_REFERENCE_OFFSET_M),
        max_relative_speed_m_s=MAX_RELATIVE_SPEED_M_S,
    )

    unknown = set(overrides) - set(settings)
    if unknown:
        raise TypeError(
            f"unknown wing configuration field(s): {sorted(unknown)}. "
            f"Valid fields are: {sorted(settings)}"
        )

    settings.update(overrides)
    return RotatingWingConfig(**settings)


def build_wing(**overrides: Any) -> RotatingWing:
    """Build a ready-to-use :class:`RotatingWing` from the constants above.

    Args:
        **overrides: Passed through to :func:`build_wing_config`.

    Returns:
        A :class:`RotatingWing` built on the resulting configuration.
    """
    return RotatingWing(build_wing_config(**overrides))


def describe(config: Optional[RotatingWingConfig] = None) -> str:
    """Return a human-readable summary of the wing geometry and its hover performance.

    Useful at the top of a sweep log so that the run record states exactly what geometry
    produced it.

    Args:
        config: Configuration to describe. Defaults to :func:`build_wing_config`.
    """
    cfg = config if config is not None else build_wing_config()

    if not cfg.enabled:
        return "Rotating wing: DISABLED (no-wing baseline)"

    wing = RotatingWing(cfg)
    # Signed to the panels' own handedness. Evaluating at +20 regardless would print the
    # performance of a mirror image of the wing being flown -- for left-handed panels,
    # the downforce case.
    spin = cfg.rotation_sense * 20.0
    loads = wing.hover_loads(spin)
    fm = loads.figure_of_merit(cfg.air_density_kg_m3, cfg.disk_area_m2())

    return (
        "Rotating wing: ENABLED\n"
        f"  blades            : {cfg.num_blades} at {cfg.resolved_blade_azimuths_deg()} deg\n"
        f"  span              : r = {cfg.root_radius_m:.3f} to {cfg.tip_radius_m:.3f} m\n"
        f"  blade area        : {cfg.blade_area_m2():.4f} m^2  (solidity {cfg.solidity():.3f})\n"
        f"  collective offset : {cfg.collective_pitch_offset_deg:+.2f} deg\n"
        f"  inflow model      : {cfg.inflow_model}\n"
        f"  handedness        : {cfg.rotation_sense:+d} "
        f"({'left' if cfg.rotation_sense < 0 else 'right'}-handed panels)\n"
        f"  hover @ {spin:+.0f} rad/s: T = {loads.thrust_n:.3f} N "
        f"({loads.thrust_n / 9.80665 * 1000.0:.0f} g), "
        f"P = {loads.aero_power_w:.2f} W, "
        f"Q_z = {loads.torque_z_nm:+.4f} N*m, FM = {fm:.3f}"
    )


if __name__ == "__main__":
    print(describe())
