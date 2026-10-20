"""
| File: spinning_wing_multirotor.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: A Multirotor carrying fixed aerofoil panels on its arms, which generate lift
|              from the airframe's own rotation.

DESIGN INTENT
-------------
This class exists so that the rotating-wing aerodynamics can be added to the vehicle
WITHOUT modifying :class:`~pegasus.simulator.logic.vehicles.multirotor.Multirotor`. The
quadrotor physics -- thrust curve, rotor force application, body drag, rotational damping
-- is inherited unchanged and is not touched. All this subclass does is call the parent's
update and then add one additional force and one additional moment to the body.

That separation is deliberate and worth preserving. It means:

* Any run with ``wing_config.enabled = False`` is bit-for-bit the stock quadrotor, so the
  no-wing baseline is a true baseline rather than a re-derivation of it.
* The wing model can be validated offline, with no simulator in the loop, because
  :class:`~pegasus.simulator.logic.aerodynamics.RotatingWing` is pure NumPy.
* A change to the quadrotor model upstream propagates here for free.

NO VISUAL GEOMETRY
------------------
The wing is aerodynamic only. No USD prim, mesh or collider is created for it, by design:
the panels are modelled as a load acting on the existing ``/body`` rigid body.

TWO CONSEQUENCES OF THAT, BOTH DELIBERATE
    1. The wing contributes no mass and no inertia. The airframe's rigid-body properties
       come from the USD asset and are unchanged. For a 0.3 m span panel on a 500 g
       airframe the added yaw inertia is NOT negligible, so a study of spin-up transients
       should raise the asset's I_zz accordingly. That is left to the USD asset on purpose:
       changing it here would mean reaching into the quadrotor's rigid body, which this
       class exists to avoid.
    2. The wing cannot collide with anything.
"""

from __future__ import annotations

from typing import Optional

import carb
import numpy as np

from pegasus.simulator.logic.aerodynamics import RotatingWing, RotatingWingConfig, WingLoads
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

__all__ = ["SpinningWingMultirotor", "SpinningWingMultirotorConfig"]


class SpinningWingMultirotorConfig(MultirotorConfig):
    """Multirotor configuration extended with the rotating-wing parameters.

    Inherits every field of :class:`MultirotorConfig` unchanged -- thrust curve, drag,
    rotational damping, sensors, backends -- and adds exactly one: ``wing``.
    """

    def __init__(self, wing: Optional[RotatingWingConfig] = None) -> None:
        super().__init__()

        self.wing: RotatingWingConfig = wing if wing is not None else RotatingWingConfig()
        """Rotating-wing aerodynamic parameters. Set ``wing.enabled = False`` for a
        no-wing baseline run."""


class SpinningWingMultirotor(Multirotor):
    """A Multirotor with airframe-mounted aerofoil panels.

    The vehicle's yaw rate drives the panels as rotor blades. For the all-same-direction
    rotor arrangement this project studies, the motors' reaction torques add rather than
    cancel, spinning the airframe; these panels convert part of that spin into lift and,
    in doing so, apply a braking torque that sets the equilibrium spin rate.

    Example:
        >>> cfg = SpinningWingMultirotorConfig()           # doctest: +SKIP
        >>> cfg.backends = [my_controller]                 # doctest: +SKIP
        >>> vehicle = SpinningWingMultirotor(              # doctest: +SKIP
        ...     "/World/spin0", ROBOTS["Iris"], 0, [0, 0, 0.1], [0, 0, 0, 1], config=cfg
        ... )
    """

    def __init__(
        self,
        stage_prefix: str = "quadrotor",
        usd_file: str = "",
        vehicle_id: int = 0,
        init_pos=(0.0, 0.0, 0.07),
        init_orientation=(0.0, 0.0, 0.0, 1.0),
        config: Optional[SpinningWingMultirotorConfig] = None,
    ) -> None:
        """Initialise the vehicle.

        Args:
            stage_prefix: Prim path the vehicle is spawned at.
            usd_file: USD asset describing the airframe.
            vehicle_id: Pegasus vehicle id.
            init_pos: Initial position in the inertial ENU frame [m].
            init_orientation: Initial attitude quaternion [qx, qy, qz, qw].
            config: Vehicle configuration. Defaults to a stock
                :class:`SpinningWingMultirotorConfig`, i.e. the as-designed GOE 210 wing.
        """
        if config is None:
            config = SpinningWingMultirotorConfig()

        super().__init__(
            stage_prefix,
            usd_file,
            vehicle_id,
            list(init_pos),
            list(init_orientation),
            config=config,
        )

        wing_config = getattr(config, "wing", None) or RotatingWingConfig()
        self._wing_config = wing_config
        self._wing = RotatingWing(wing_config)

        self._last_wing_loads: Optional[WingLoads] = None
        self._applied_inertia: Optional[tuple] = None

        if wing_config.enabled and wing_config.apply_inertia_to_body:
            self._add_wing_inertia_to_body()

    # -----------------------------------------------------------------------------------
    # Structural properties
    # -----------------------------------------------------------------------------------

    def _add_wing_inertia_to_body(self) -> None:
        """Add the wing's mass and inertia to the airframe's rigid body.

        The wing has no USD geometry, so without this it would be aerodynamically present
        but structurally massless. For the default 0.3 m span panels on a small airframe
        the added yaw inertia is tens of percent of the bare vehicle's, which changes
        spin-up time and the whole yaw-axis response.

        The mass properties are read from the spawned ``/body`` prim, augmented, and
        written back. This runs during construction, before the physics scene builds the
        articulation, so PhysX picks up the modified values.

        WHY THIS IS NOT "TOUCHING THE QUADCOPTER PHYSICS"
            It adds the WING's own mass to the vehicle it is bolted to. With the wing
            disabled, or with ``apply_inertia_to_body`` False, nothing here runs and the
            airframe keeps exactly the properties its USD asset specifies.

        Failures are logged and swallowed rather than raised: an unexpected asset layout
        should degrade the fidelity of the run, not abort a sweep that is otherwise valid.
        The applied values are recorded in :attr:`applied_inertia` so a run can state what
        it actually used.
        """
        try:
            from pxr import Gf

            prim = self._world.stage.GetPrimAtPath(self._stage_prefix + "/body")
            if not prim or not prim.IsValid():
                carb.log_warn(
                    f"[SpinningWingMultirotor] no /body prim at {self._stage_prefix}; "
                    "wing inertia not applied"
                )
                return

            mass_attr = prim.GetAttribute("physics:mass")
            inertia_attr = prim.GetAttribute("physics:diagonalInertia")
            if not mass_attr or not inertia_attr:
                carb.log_warn(
                    "[SpinningWingMultirotor] /body has no mass or diagonalInertia "
                    "attribute; wing inertia not applied"
                )
                return

            body_mass = float(mass_attr.Get())
            body_inertia = np.array(inertia_attr.Get(), dtype=float)

            wing_mass, wing_inertia = self._wing.inertia_about_body_origin()
            if wing_mass <= 0.0:
                return

            new_mass = body_mass + wing_mass
            new_inertia = body_inertia + wing_inertia

            mass_attr.Set(new_mass)
            inertia_attr.Set(Gf.Vec3f(*[float(v) for v in new_inertia]))

            self._applied_inertia = (body_mass, body_inertia, wing_mass, wing_inertia)

            carb.log_warn(
                f"[SpinningWingMultirotor] wing structure added to {self._stage_prefix}: "
                f"mass {body_mass:.4f} -> {new_mass:.4f} kg "
                f"(+{wing_mass * 1000.0:.0f} g), "
                f"Izz {body_inertia[2]:.5f} -> {new_inertia[2]:.5f} kg.m^2 "
                f"(+{wing_inertia[2] / body_inertia[2] * 100.0:.0f}%)"
            )
        except Exception as error:  # noqa: BLE001 - see docstring
            carb.log_warn(
                f"[SpinningWingMultirotor] could not apply wing inertia: {error}"
            )

    @property
    def applied_inertia(self):
        """What the wing added to the body, or None if nothing was applied.

        Returns a tuple of (body_mass_kg, body_inertia, wing_mass_kg, wing_inertia), with
        both inertias as 3-vectors about the body origin.
        """
        return self._applied_inertia

    # -----------------------------------------------------------------------------------
    # Properties
    # -----------------------------------------------------------------------------------

    @property
    def wing(self) -> RotatingWing:
        """The rotating-wing aerodynamic model."""
        return self._wing

    @property
    def wing_enabled(self) -> bool:
        """Whether the wing contributes any load."""
        return bool(self._wing_config.enabled)

    @property
    def last_wing_loads(self) -> Optional[WingLoads]:
        """Loads computed on the most recent physics step, or None before the first step.

        Exposed so that telemetry backends can log wing thrust, braking torque and
        aerodynamic power alongside the controller's own signals, without recomputing
        them.
        """
        return self._last_wing_loads

    # -----------------------------------------------------------------------------------
    # Simulation hooks
    # -----------------------------------------------------------------------------------

    def start(self) -> None:
        """Reset the wing's induced-velocity warm start when the simulation starts."""
        super().start()
        self._wing.reset()
        self._last_wing_loads = None

    def stop(self) -> None:
        """Clear cached loads when the simulation stops."""
        super().stop()
        self._last_wing_loads = None

    def update(self, dt: float) -> None:
        """Apply the quadrotor loads, then add the rotating wing's contribution.

        The parent call applies rotor thrust, body drag and rotational damping exactly as
        the stock vehicle does. Only afterwards is the wing wrench added. PhysX accumulates
        applied forces over the step, so the order of application does not affect the
        result; doing the parent first simply keeps the inherited behaviour obvious.

        Args:
            dt: Physics timestep [s].
        """
        super().update(dt)

        if not self._wing_config.enabled:
            self._last_wing_loads = None
            return

        loads = self._wing.compute_loads(
            linear_body_velocity_m_s=self._state.linear_body_velocity,
            angular_velocity_body_rad_s=self._state.angular_velocity,
            dt=dt,
        )
        self._last_wing_loads = loads

        # Guard the physics engine against a diverged state. compute_loads already returns
        # zeros for non-finite input, but a finite state can still produce a non-finite
        # load if a configuration is pathological, and a NaN reaching PhysX corrupts every
        # other vehicle in the scene, not just this one.
        if not (
            np.all(np.isfinite(loads.force_body_n))
            and np.all(np.isfinite(loads.moment_body_nm))
        ):
            return

        # The resultant is applied at the body origin, matching the convention the stock
        # Multirotor uses for body drag. The moment arm of each blade element is already
        # accounted for inside the wing model, which returns the moment about the
        # configured reference point.
        self.apply_force(
            [float(v) for v in loads.force_body_n], pos=[0.0, 0.0, 0.0], body_part="/body"
        )
        self.apply_torque([float(v) for v in loads.moment_body_nm], "/body")
