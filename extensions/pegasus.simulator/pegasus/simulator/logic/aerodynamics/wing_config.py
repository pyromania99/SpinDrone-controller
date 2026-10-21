"""
| File: wing_config.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Single-source configuration for the rotating-wing aerodynamic model.

EVERY physical and numerical parameter of the spinning-wing model lives in this file.
Nothing else in the aerodynamics package contains a hard-coded physical constant. To
change the vehicle, edit ``examples/utils/wing_params.py``, which builds on the defaults
defined here.

GEOMETRY CONVENTION
-------------------
The vehicle body frame is Pegasus' FLU convention: x forward, y left, z up, origin at the
body origin of the ``/body`` rigid body (assumed coincident with the centre of mass unless
``moment_reference_offset_m`` says otherwise).

Each blade is a straight, rigid, untwisted-in-sweep wing lying along a radial line in the
body xy-plane:

                        ^ +y (left)
                        |
             blade 1    |    blade 0
              (180 deg) | (0 deg)
                 -------+-------->  +x (forward)
                        |
                        |

``blade_azimuths_deg`` lists the angle of each blade's span axis measured from +x towards
+y (i.e. the standard right-handed angle about +z). The blade occupies radii from
``root_radius_m`` to ``tip_radius_m`` along that line.

``pitch_deg`` at a station is the angle between the section chord line and the rotor plane
(the body xy-plane), positive such that a blade advancing into its own leading edge
generates thrust along +z. This matches the "pitch measured from the rotation plane"
convention used throughout foil_out/GOE210_DESIGN_REPORT.md.

HANDEDNESS
----------
Which way the blade advances is set by ``rotation_sense``, not by the pitch. A right-handed
blade (+1) leads towards +azimuth and lifts under a positive yaw rate; a left-handed blade
(-1) is its mirror image and lifts under a negative one. This airframe spins negative, so
the default is -1. Mirroring reflects the blade through a plane containing the spin axis:
the aerofoil stays upright, the pitch stays positive, and only the sense of rotation that
produces lift changes. See ``rotation_sense`` for why this matters more than it sounds.

DEFAULTS
--------
The defaults below are the as-designed GOE 210 spinning wing from
``foil_out/GOE210_DESIGN_REPORT.md`` section 1, "Build schedule". That design was optimised
for hover at 20 rad/s with the wing spanning r = 0.2 to 0.5 m, two blades, rho = 1.2 kg/m^3,
and the report predicts 3.74 N of lift and 7.31 W of aerodynamic power there. The physics
test suite checks the model against exactly those numbers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .airfoil_polar import AirfoilPolar, goe210_polar

__all__ = ["RotatingWingConfig", "InflowModel", "trapezoid"]


# ``np.trapz`` was renamed to ``np.trapezoid`` in NumPy 2.0 and deprecated under the old
# name. Isaac Sim 5.1 ships NumPy 1.26, where only ``trapz`` exists, while a system Python
# used for the offline physics tests may well have NumPy 2.x. Bind once, here.
trapezoid = getattr(np, "trapezoid", None) or np.trapz


class InflowModel:
    """Enumeration of the available induced-velocity models.

    NONE
        No induced velocity. The blade sees only the vehicle's own motion. This
        over-predicts thrust substantially (by roughly 25-30 percent for this design) and
        exists for debugging and for isolating the inflow's contribution in tests.

    BEMT
        Blade-element momentum theory: an annulus-by-annulus momentum balance with a
        Prandtl tip- and root-loss factor, solved to convergence each call. This is the
        model used to produce the design report, and it is the default. Strictly it
        assumes axial flow, so its accuracy degrades as edgewise speed becomes comparable
        to tip speed.

    GLAUERT
        A single uniform induced velocity over the whole disk, from Glauert's momentum
        relation including an edgewise (forward-flight) term. Cheaper than BEMT and better
        behaved in fast translational flight, at the cost of losing the radial inflow
        distribution.

    DYNAMIC
        First-order dynamic inflow on the uniform component: the induced velocity is a
        STATE obeying ``dv/dt = (v_steady - v)/tau`` rather than an algebraic constraint
        resolved every step. Two consequences, both wanted here.

        First, physics. Inflow has mass. The apparent mass of the air column accelerated
        by the disc is ``m_a = (8/3) rho R^3`` -- the impermeable-disc value that
        Pitt & Peters (1980) confirm for the M11 term -- which for this wing gives a time
        constant of about 67 ms. That is 0.54 of the roll timescale at 8 rad/s and 0.81 of
        a 12 rad/s controller bandwidth, so treating inflow as instantaneous overstates
        aerodynamic damping and phase margin exactly where a controller study cares.

        Second, cost. There is no fixed-point iteration, so the per-step cost is constant
        instead of varying between one and thirty-two polar evaluations, which turns a
        bounded average into a bounded worst case.

        NOT INCLUDED: the first-harmonic inflow states (longitudinal and lateral), which
        need the full Pitt-Peters L-matrix as a function of wake skew angle. Only the
        uniform component is modelled here.
    """

    NONE = "none"
    BEMT = "bemt"
    GLAUERT = "glauert"
    DYNAMIC = "dynamic"

    ALL = (NONE, BEMT, GLAUERT, DYNAMIC)


# ---------------------------------------------------------------------------------------
# As-designed GOE 210 build schedule
# ---------------------------------------------------------------------------------------
# foil_out/GOE210_DESIGN_REPORT.md, section 1. Pitch is the chord-line angle to the rotor
# plane. The report's final station (r = 0.5 m) is given there as 16.5 deg "hold", because
# the optimiser's raw 32.4 deg at the tip is an artefact of the tip-loss factor; the build
# schedule value is the one used here.

_GOE210_STATION_RADIUS_M: List[float] = [0.200, 0.250, 0.300, 0.350, 0.400, 0.425, 0.450, 0.475, 0.500]
_GOE210_STATION_CHORD_M: List[float] = [0.300, 0.300, 0.297, 0.280, 0.197, 0.153, 0.124, 0.099, 0.080]
_GOE210_STATION_PITCH_DEG: List[float] = [24.7, 22.6, 20.8, 19.4, 17.6, 16.3, 16.3, 16.6, 16.5]


@dataclass
class RotatingWingConfig:
    """Complete parameter set for the rotating-wing aerodynamic model.

    All lengths are metres, all angles in this dataclass are degrees (fields are suffixed
    ``_deg``), all forces newtons, all moments newton-metres.
    """

    # -- master switch ------------------------------------------------------------------
    enabled: bool = True
    """When False the model returns zero force and zero moment. Used by the ``--no-wing``
    run mode so that the identical code path is exercised with and without the wing."""

    # -- atmosphere ---------------------------------------------------------------------
    air_density_kg_m3: float = 1.2
    """Air density [kg/m^3]. 1.2 is the value used throughout the design report."""

    kinematic_viscosity_m2_s: float = 1.5e-5
    """Kinematic viscosity [m^2/s], used for the chord Reynolds number. 1.5e-5 is the
    design report's value for sea-level air."""

    # -- blade planform -----------------------------------------------------------------
    num_blades: int = 2
    """Number of wing panels on the airframe [-]."""

    rotation_sense: int = -1
    """Sense of body yaw rate the blades are built to produce lift in: +1 or -1 [-].

    A rotor blade is a handed object. Mirror it and it still lifts, but only when turned
    the other way; spin it the wrong way and the aerofoil meets the air at a negative
    angle of attack and pushes DOWN.

    This airframe spins in one direction only, and it is the negative one. All four
    motors share a rotation direction (``rot_dir = [-1, -1, -1, -1]`` in
    ``QuadraticThrustCurve``), so their reaction torques add to a negative yaw moment and
    the vehicle settles at a negative body yaw rate -- measured at about -27 rad/s in
    simulation. The panels must therefore be built left-handed, which is what -1 means
    here.

    The design report tabulates a conventional right-handed rotor, so its numbers
    correspond to ``rotation_sense = +1``. Mirroring is an exact symmetry of the blade
    element equations: the magnitudes of thrust, torque and power are unchanged, only the
    sense of rotation that produces them is flipped. The physics suite asserts that
    symmetry directly, so the report validation holds for either value.

    Set this to +1 only to model a right-handed wing, e.g. for an ablation showing what
    the wrong handedness costs."""

    blade_azimuths_deg: Optional[List[float]] = None
    """Azimuth of each blade's span axis about +z, measured from +x [deg]. When None the
    blades are spaced uniformly, i.e. ``i * 360 / num_blades``, which is the symmetric
    arrangement assumed by the design report."""

    root_radius_m: float = 0.200
    """Inboard end of the lifting surface, measured from the body z-axis [m]."""

    tip_radius_m: float = 0.500
    """Outboard end of the lifting surface [m]."""

    station_radius_m: List[float] = field(default_factory=lambda: list(_GOE210_STATION_RADIUS_M))
    """Radii at which chord and pitch are tabulated [m], strictly increasing."""

    station_chord_m: List[float] = field(default_factory=lambda: list(_GOE210_STATION_CHORD_M))
    """Chord at each tabulated station [m]."""

    station_pitch_deg: List[float] = field(default_factory=lambda: list(_GOE210_STATION_PITCH_DEG))
    """Chord-line angle to the rotor plane at each tabulated station [deg]."""

    collective_pitch_offset_deg: float = 0.0
    """Rigid offset added to every station's pitch [deg]. The single most useful trim knob:
    it shifts the whole blade's operating angle of attack without re-tabulating the
    build schedule."""

    blade_z_offset_m: float = 0.0
    """Vertical offset of the wing plane above the body origin [m]. Non-zero values move
    the line of action of the wing thrust and therefore couple wing drag into pitch and
    roll moments."""

    # -- structural properties ----------------------------------------------------------
    blade_areal_density_kg_m2: float = 1.0
    """Mass per unit planform area of one blade panel [kg/m^2].

    A carbon-skinned foam-core panel with an internal carbon spar lands around
    0.8 to 1.5 kg/m^2 at this scale. With the default planform (0.069 m^2 per blade) 1.0
    gives 69 g per blade, 138 g for the pair. Set ``blade_mass_kg`` to override with a
    weighed value."""

    blade_mass_kg: Optional[float] = None
    """Mass of ONE blade panel [kg], or None to derive it from the areal density.

    Overrides ``blade_areal_density_kg_m2`` when set. Use the figure from the real
    hardware once the panels exist."""

    apply_inertia_to_body: bool = True
    """Add the wing's mass and inertia to the airframe's rigid body at spawn.

    The wing is modelled aerodynamically as a load on the existing ``/body``; it has no
    USD geometry of its own, so without this flag it would be massless. For a 0.3 m span
    panel on a small airframe the added yaw inertia is a large fraction of the bare
    vehicle's, so leaving it out makes spin-up and yaw-axis dynamics substantially wrong.

    Set False to study the aerodynamics in isolation, or when the airframe USD asset
    already includes the wing's mass."""

    # -- section aerodynamics -----------------------------------------------------------
    polar: AirfoilPolar = field(default_factory=goe210_polar)
    """2-D section polar. Defaults to the GOE 210 calibration; substitute a
    :class:`~.airfoil_polar.TablePolar` to use measured or XFOIL data."""

    # -- induced velocity ---------------------------------------------------------------
    inflow_model: str = InflowModel.BEMT
    """One of :class:`InflowModel`."""

    num_stations: int = 24
    """Number of spanwise integration stations per blade [-]. The report used 25 stations
    over the same span. Convergence with respect to this number is checked by the physics
    tests."""

    inflow_max_iterations: int = 24
    """Maximum fixed-point iterations for the induced velocity [-]."""

    inflow_tolerance_m_s: float = 1.0e-4
    """Convergence tolerance on the induced velocity [m/s]."""

    inflow_relaxation: float = 0.5
    """Under-relaxation factor for the induced-velocity iteration [-], in (0, 1]. Values
    near 1 converge faster but can oscillate when the blade is near stall."""

    use_tip_loss: bool = True
    """Apply the Prandtl tip-loss factor. The design report used it."""

    use_root_loss: bool = False
    """Apply the Prandtl root-loss factor, the inboard analogue of tip loss.

    Defaults to False for two reasons. First, the design report's BEMT model applied a
    tip-loss factor only, so leaving root loss off is what reproduces its numbers: with
    it on, predicted thrust falls about 10 percent below the report, and with it off the
    agreement is within 1 percent. Second, a root-loss factor models circulation falling
    to zero at a free inboard end, which is not the case here -- the blade root is bonded
    to the airframe arm, so the physical situation is closer to a wall boundary than to a
    free tip. Enabling it also makes the inflow iteration prone to oscillation, because
    the loss factor pins to ``tip_loss_floor`` over the inboard stations."""

    tip_loss_floor: float = 0.05
    """Lower clamp on the Prandtl loss factor [-]. The report clamps to 0.05, which
    prevents the induced velocity diverging as the factor approaches zero at the tip."""

    # -- moment bookkeeping -------------------------------------------------------------
    moment_reference_offset_m: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    """Vector from the body origin to the point about which wing moments are reported [m],
    expressed in the body frame. Leave at zero when the body origin is the centre of mass,
    which is the case for the stock Iris asset."""

    # -- numerical safety ---------------------------------------------------------------
    max_relative_speed_m_s: float = 200.0
    """Hard ceiling on the section relative speed [m/s]. A diverging controller can drive
    the rate arbitrarily high; without a clamp the resulting force would inject energy and
    destroy the integration before the divergence is visible in the logs."""

    def __post_init__(self) -> None:
        self.validate()

    # -- validation ---------------------------------------------------------------------

    def validate(self) -> None:
        """Check every invariant the solver depends on.

        Raises:
            ValueError: on the first violated invariant, naming the offending field.
        """
        if self.num_blades < 1:
            raise ValueError("num_blades must be >= 1")
        if self.rotation_sense not in (1, -1):
            raise ValueError(
                f"rotation_sense must be +1 or -1; got {self.rotation_sense!r}. It is a "
                "handedness, not a scale factor."
            )
        if self.air_density_kg_m3 <= 0.0:
            raise ValueError("air_density_kg_m3 must be > 0")
        if self.kinematic_viscosity_m2_s <= 0.0:
            raise ValueError("kinematic_viscosity_m2_s must be > 0")
        if self.root_radius_m < 0.0:
            raise ValueError("root_radius_m must be >= 0")
        if self.tip_radius_m <= self.root_radius_m:
            raise ValueError("tip_radius_m must exceed root_radius_m")
        if self.num_stations < 2:
            raise ValueError("num_stations must be >= 2")
        if self.inflow_model not in InflowModel.ALL:
            raise ValueError(
                f"inflow_model must be one of {InflowModel.ALL}; got {self.inflow_model!r}"
            )
        if not (0.0 < self.inflow_relaxation <= 1.0):
            raise ValueError("inflow_relaxation must lie in (0, 1]")
        if self.inflow_max_iterations < 1:
            raise ValueError("inflow_max_iterations must be >= 1")
        if not (0.0 < self.tip_loss_floor <= 1.0):
            raise ValueError("tip_loss_floor must lie in (0, 1]")
        if self.max_relative_speed_m_s <= 0.0:
            raise ValueError("max_relative_speed_m_s must be > 0")
        if self.blade_areal_density_kg_m2 < 0.0:
            raise ValueError("blade_areal_density_kg_m2 must be >= 0")
        if self.blade_mass_kg is not None and self.blade_mass_kg < 0.0:
            raise ValueError("blade_mass_kg must be >= 0 when given")

        n = len(self.station_radius_m)
        if n < 2:
            raise ValueError("station_radius_m needs at least two entries")
        if len(self.station_chord_m) != n or len(self.station_pitch_deg) != n:
            raise ValueError(
                "station_radius_m, station_chord_m and station_pitch_deg must be the "
                f"same length; got {n}, {len(self.station_chord_m)}, "
                f"{len(self.station_pitch_deg)}"
            )
        radii = np.asarray(self.station_radius_m, dtype=float)
        if np.any(np.diff(radii) <= 0.0):
            raise ValueError("station_radius_m must be strictly increasing")
        if np.any(np.asarray(self.station_chord_m, dtype=float) <= 0.0):
            raise ValueError("station_chord_m entries must all be > 0")

        if len(self.moment_reference_offset_m) != 3:
            raise ValueError("moment_reference_offset_m must have exactly three entries")

        azimuths = self.resolved_blade_azimuths_deg()
        if len(azimuths) != self.num_blades:
            raise ValueError(
                f"blade_azimuths_deg has {len(azimuths)} entries but num_blades is "
                f"{self.num_blades}"
            )

    # -- derived quantities -------------------------------------------------------------

    def resolved_blade_azimuths_deg(self) -> List[float]:
        """Blade azimuths [deg], filling in uniform spacing when none were given."""
        if self.blade_azimuths_deg is not None:
            return list(self.blade_azimuths_deg)
        return [i * 360.0 / self.num_blades for i in range(self.num_blades)]

    def disk_area_m2(self) -> float:
        """Area of the full swept disk [m^2], pi * R_tip^2.

        The full disk is used, not the annulus between root and tip, because that is the
        reference area the design report's figure of merit is defined against.
        """
        return float(np.pi * self.tip_radius_m ** 2)

    def blade_area_m2(self) -> float:
        """Total planform area of all blades [m^2], integrated from the station table."""
        radii = np.asarray(self.station_radius_m, dtype=float)
        chords = np.asarray(self.station_chord_m, dtype=float)
        return float(self.num_blades * trapezoid(chords, radii))

    def blade_aspect_ratio(self) -> float:
        """Aspect ratio of one blade [-]: span divided by mean chord.

        Used to set the post-stall flat-plate drag coefficient from Viterna's eq. (13)
        rather than from the 2-D value, which a blade this stubby never reaches.
        """
        span = self.tip_radius_m - self.root_radius_m
        mean_chord = (self.blade_area_m2() / self.num_blades) / span
        return span / mean_chord

    def solidity(self) -> float:
        """Rotor solidity [-]: total blade area divided by disk area."""
        return self.blade_area_m2() / self.disk_area_m2()
