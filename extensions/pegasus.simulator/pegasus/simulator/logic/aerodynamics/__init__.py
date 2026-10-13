"""
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Aerodynamic models for airframe-mounted rotating wings.

This subpackage is deliberately free of any Isaac Sim, Omniverse or Pegasus dependency.
It imports nothing beyond NumPy, so the whole model can be exercised offline by the
physics test suite using a plain Python interpreter.
"""

from .airfoil_polar import (
    AirfoilPolar,
    AnalyticPolar,
    TablePolar,
    goe210_polar,
    viterna_coefficients,
    viterna_cd_max,
)
from .rotating_wing import RotatingWing, WingLoads
from .wing_design import (
    PitchDesignResult,
    best_lift_to_drag_alpha,
    design_pitch_schedule,
    scale_chord_to_span,
)
from .wing_config import InflowModel, RotatingWingConfig

__all__ = [
    "AirfoilPolar",
    "AnalyticPolar",
    "TablePolar",
    "goe210_polar",
    "viterna_coefficients",
    "viterna_cd_max",
    "RotatingWing",
    "WingLoads",
    "PitchDesignResult",
    "best_lift_to_drag_alpha",
    "design_pitch_schedule",
    "scale_chord_to_span",
    "InflowModel",
    "RotatingWingConfig",
]
