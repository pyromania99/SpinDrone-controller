"""
| File: _context.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Shared import bootstrap and tolerances for the rotating-wing physics tests.

These tests run under a plain Python interpreter -- no Isaac Sim, no Omniverse. That is
possible because the aerodynamics package depends on nothing but NumPy, and it is a
deliberate property worth protecting: the physics can be probed in under a second, so
there is no excuse for not running the suite after touching the model.

Run the suite with:

    python3 -m pytest examples/controller_tests/physics_tests -q

or, without pytest:

    python3 examples/controller_tests/physics_tests/run_physics_tests.py
"""

from __future__ import annotations

import os
import sys

REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Importing through the user-facing configuration module is intentional: it means the
# tests exercise the same entry point the simulation scripts use, so a break in that
# plumbing fails the suite rather than only failing at run time.
from examples.utils.wing_params import (  # noqa: E402
    AirfoilPolar,
    AnalyticPolar,
    InflowModel,
    RotatingWing,
    RotatingWingConfig,
    TablePolar,
    WingLoads,
    build_wing,
    build_wing_config,
    goe210_polar,
    viterna_coefficients,
)

__all__ = [
    "REPO_ROOT",
    "AirfoilPolar",
    "AnalyticPolar",
    "InflowModel",
    "RotatingWing",
    "RotatingWingConfig",
    "TablePolar",
    "WingLoads",
    "build_wing",
    "build_wing_config",
    "goe210_polar",
    "viterna_coefficients",
    "DESIGN_SPIN_RATE_RAD_S",
    "DESIGN_SPIN_MAGNITUDE_RAD_S",
    "REPORT",
    "REPORT_STATIONS",
    "GRAVITY_M_S2",
]


GRAVITY_M_S2 = 9.80665

DESIGN_SPIN_MAGNITUDE_RAD_S = 20.0
"""Magnitude of the spin rate the GOE 210 blade was optimised for [rad/s]."""

DESIGN_SPIN_RATE_RAD_S = build_wing_config().rotation_sense * DESIGN_SPIN_MAGNITUDE_RAD_S
"""The design spin rate, SIGNED to the handedness the default panels are built in.

The design report describes a conventional right-handed rotor turning at +20 rad/s. This
airframe spins the other way -- all four motors turn the same way, so their reaction
torques add to a negative yaw moment -- and the panels are therefore built left-handed
(``ROTATION_SENSE = -1``). Mirroring is an exact symmetry of the blade element equations,
so the report's numbers must be reproduced at -20 rad/s by a left-handed blade exactly as
at +20 rad/s by a right-handed one.

Deriving this from the configuration rather than hard-coding -20.0 is deliberate: it means
the validation suite follows the shipped handedness instead of silently testing a blade
nobody flies.
"""


# ---------------------------------------------------------------------------------------
# Reference data: foil_out/GOE210_DESIGN_REPORT.md
# ---------------------------------------------------------------------------------------
# These are the numbers an INDEPENDENT blade-element-momentum optimisation produced for
# this blade. They were not used to fit the model: the airfoil polar was calibrated to the
# report's section data (alpha, c_l, c_l/c_d, Re), but the thrust, power and figure of
# merit below are integrated quantities that the model has to get right on its own.

REPORT = {
    "thrust_n_at_20": 3.74,          # section 1 and section 8.1 totals
    "aero_power_w_at_20": 7.31,      # section 8.1 totals
    "figure_of_merit": 0.722,        # section 8.1 totals
    "thrust_grams_at_15": 211.0,     # section 1, spin-rate robustness
    "thrust_grams_at_20": 377.0,     # section 1, spin-rate robustness
}

REPORT_STATIONS = {
    # radius [m]: (alpha [deg], c_l [-], induced velocity [m/s])
    # foil_out/GOE210_DESIGN_REPORT.md section 8.1 "Station table".
    0.200: (5.1, 1.01, 1.43),
    0.225: (4.9, 0.99, 1.51),
    0.250: (4.9, 1.00, 1.60),
    0.275: (4.8, 0.99, 1.67),
    0.300: (4.6, 0.97, 1.74),
    0.325: (4.5, 0.96, 1.80),
    0.350: (4.4, 0.96, 1.87),
    0.400: (5.0, 1.00, 1.80),
    0.425: (5.0, 1.00, 1.70),
    0.450: (5.5, 1.04, 1.73),
    # The r = 0.475 and r = 0.500 stations are omitted: the report flags its own last
    # station as a tip-loss artefact, and the build schedule it recommends departs from
    # the optimiser output there.
}
