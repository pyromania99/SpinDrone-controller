"""
| File: rotating_wing.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Blade-element / momentum model of a rigid wing rotating with the airframe.

PHYSICAL PROBLEM
----------------
The vehicle is a quadrotor whose four rotors all turn the same way. Their reaction torques
therefore add instead of cancelling, and the airframe spins about its own z-axis. Fixed
aerofoil panels are bonded to the arms, inboard of the motors. Because they are rigidly
attached to a spinning body, those panels behave as rotor blades: the airframe's yaw rate
is their rotational speed, and they convert part of the otherwise parasitic spin into
useful lift.

This module computes the resultant aerodynamic force and moment of those panels, in the
body frame, as an explicit function of vehicle state. It is a pure function: no Isaac Sim
or Omniverse dependency, no mutable state carried between calls other than an optional
induced-velocity warm start. That makes it directly testable offline, which is how the
physics test suite validates it against the design report.

MODEL
-----
Classical blade element theory closed with momentum theory (BEMT), following Leishman [1]
ch. 3 and ch. 10:

1. The span of each blade is divided into ``num_stations`` elements.
2. For each element, the velocity of the element through the air is computed from the
   vehicle's body-frame linear velocity and body rates:

       v_element = v_body + omega_body x r_element

3. The element's velocity is resolved into a component in the rotor plane along the
   direction of rotation (U_T) and a component along the rotor axis (U_P), the latter
   augmented by the induced velocity:

       U_T = (v_body + omega x r) . e_t
       U_P = (v_body + omega x r) . z_hat + v_i

4. The inflow angle and angle of attack follow:

       phi   = atan2(U_P, U_T)
       alpha = theta(r) - phi

5. Section lift and drag come from the 2-D polar at the local Reynolds number, and are
   resolved back into rotor-plane and rotor-axis components:

       dF_z = dL*cos(phi) - dD*sin(phi)          (along +z, thrust)
       dF_t = -(dL*sin(phi) + dD*cos(phi))       (along +e_t, i.e. opposing rotation)

6. The induced velocity v_i is found by requiring the blade element thrust in each annulus
   to equal the momentum thrust through that annulus, with a Prandtl tip- and root-loss
   factor. See :meth:`RotatingWing._solve_inflow_bemt`.

WHAT IS AND IS NOT MODELLED
---------------------------
Modelled:
    * Full three-axis body rates, so roll and pitch rates correctly produce a once-per-
      revolution angle-of-attack variation, not just the yaw-rate contribution.
    * Translational flight, including the advancing/retreating asymmetry. Because the
      blades are rigid in the body frame, that asymmetry appears automatically through the
      body-frame velocity components.
    * Reversed flow (U_T < 0), through the use of atan2 and a full +/-180 deg polar.
    * Radial chord and pitch distribution (built twist).
    * Prandtl tip and root losses.

Not modelled, deliberately:
    * Blade flexibility and aeroelastic twist. The design report measures 10-15 deg of
      twist on a 5 mm rod, so this is a real effect on hardware; here the blade is rigid
      and ``collective_pitch_offset_deg`` is the knob to represent a mean built-in twist
      error.
    * Unsteady aerodynamics, dynamic stall, and shed-wake memory. The inflow responds
      instantaneously to a change in loading.
    * Radial (spanwise) flow in the section force. Blade element independence is assumed,
      which is standard and is good while the blade is not heavily swept or stalled.
    * The wing's mass and inertia. This module returns aerodynamic loads only; the rigid
      body properties come from the USD asset and are untouched.

REFERENCES
----------
[1] Leishman, J. G., "Principles of Helicopter Aerodynamics", 2nd ed., Cambridge
    University Press, 2006.
[2] Glauert, H., "A General Theory of the Autogyro", ARC R&M 1111, 1926.
    -- the edgewise induced-velocity relation used by the GLAUERT inflow model.
[3] foil_out/GOE210_DESIGN_REPORT.md (this repository) -- the blade this model defaults to
    and the data the physics tests validate against.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .wing_config import InflowModel, RotatingWingConfig, trapezoid

__all__ = ["RotatingWing", "WingLoads"]


@dataclass
class WingLoads:
    """Resultant aerodynamic loads and the diagnostics needed to interpret them.

    The force and moment are what the simulator applies. Everything else exists so that
    tests and analysis scripts can look inside the solution without re-running it.
    """

    force_body_n: np.ndarray
    """Resultant aerodynamic force on the airframe [N], body frame (FLU), shape (3,)."""

    moment_body_nm: np.ndarray
    """Resultant aerodynamic moment about the configured reference point [N*m], body
    frame, shape (3,)."""

    thrust_n: float
    """Component of the resultant force along the rotor axis +z [N]. Positive is useful
    lift."""

    torque_z_nm: float
    """Component of the resultant moment about +z [N*m]. For a wing being driven by the
    airframe's spin this is negative when the spin rate is positive, i.e. the wing brakes
    the rotation."""

    aero_power_w: float
    """Aerodynamic power absorbed by the wing [W], computed as the integral of the
    in-plane force against local blade speed. Equals -torque_z * spin_rate for pure yaw
    rotation, which the physics tests assert."""

    induced_velocity_m_s: np.ndarray
    """Converged induced velocity at each station [m/s], shape (num_stations,)."""

    alpha_rad: np.ndarray
    """Angle of attack per blade and station [rad], shape (num_blades, num_stations)."""

    cl: np.ndarray
    """Section lift coefficient, shape (num_blades, num_stations)."""

    cd: np.ndarray
    """Section drag coefficient, shape (num_blades, num_stations)."""

    reynolds: np.ndarray
    """Section chord Reynolds number, shape (num_blades, num_stations)."""

    inflow_residual_m_s: float
    """Final change in induced velocity between iterations [m/s]. A value above the
    configured tolerance means the inflow solution did not converge."""

    inflow_iterations: int
    """Number of iterations actually taken."""

    def figure_of_merit(self, air_density_kg_m3: float, disk_area_m2: float) -> float:
        """Hover figure of merit, FM = T^1.5 / (sqrt(2*rho*A) * P).

        Returns 0.0 when thrust or power is non-positive, which happens away from hover
        and would otherwise produce a meaningless or complex value.
        """
        if self.thrust_n <= 0.0 or self.aero_power_w <= 0.0:
            return 0.0
        ideal_power = self.thrust_n ** 1.5 / np.sqrt(2.0 * air_density_kg_m3 * disk_area_m2)
        return float(ideal_power / self.aero_power_w)


def _solve_glauert_inflow(
    loading: float, v_edge: float, climb_rate: float, initial_guess: float = 0.0,
    iterations: int = 3,
) -> float:
    """Solve Glauert's inflow relation for the uniform induced velocity.

        v * sqrt(V_edge^2 + (V_c + v)^2) = loading,      loading = T / (2 rho A)

    In hover this reduces to ``v = sqrt(loading)``; at high edgewise speed it tends to the
    fixed-wing limit ``v = loading / V_edge``.

    A FIXED number of Newton steps is used rather than iterating to a tolerance. The
    distinction matters: this is one scalar equation, so three steps cost a few dozen
    floating-point operations and the work is identical on every call. That is what keeps
    the dynamic-inflow model constant-time, unlike the blade-wise fixed point in the BEMT
    path which re-evaluates the section polar at every element on every pass.

    Dodic et al. (2023), in ``lit_survey/``, show that high-order multipoint methods solve
    this equation to acceptable accuracy in a single step given a good initial guess;
    Newton from the hover value is the conservative version of the same observation.

    Args:
        loading: Disc loading divided by twice the air density, ``T / (2 rho A)``. May be
            negative, which corresponds to downforce and a reversed inflow.
        v_edge: In-plane speed of the hub through the air [m/s], non-negative.
        climb_rate: Axial speed of the hub through the air [m/s].
        initial_guess: Starting value, normally the current inflow state.
        iterations: Newton steps. Three is ample from a sensible start.

    Returns:
        The induced velocity [m/s], carrying the sign of ``loading``.
    """
    sign = 1.0 if loading >= 0.0 else -1.0
    target = abs(loading)
    if target < 1.0e-12:
        return 0.0

    # Start from whichever of the hover solution and the current state is better scaled.
    v = abs(initial_guess)
    if v < 1.0e-6:
        v = np.sqrt(target)

    climb = sign * climb_rate
    for _ in range(iterations):
        root = np.sqrt(v_edge ** 2 + (climb + v) ** 2)
        root = max(root, 1.0e-9)
        f = v * root - target
        df = root + v * (climb + v) / root
        if abs(df) < 1.0e-12:
            break
        v = v - f / df
        if v < 0.0:
            v = 0.5 * np.sqrt(target)

    return float(sign * v)


class RotatingWing:
    """Blade-element / momentum model of the airframe-mounted rotating wing.

    The object is constructed once per vehicle and then called every physics step. It
    holds the discretised blade geometry (computed once) and the induced-velocity warm
    start (updated every call), nothing else.

    Example:
        >>> cfg = RotatingWingConfig()
        >>> wing = RotatingWing(cfg)
        >>> loads = wing.compute_loads(
        ...     linear_body_velocity_m_s=[0.0, 0.0, 0.0],
        ...     angular_velocity_body_rad_s=[0.0, 0.0, 20.0],
        ... )
        >>> round(loads.thrust_n, 2)
        3.71
    """

    def __init__(self, config: Optional[RotatingWingConfig] = None) -> None:
        """Build the discretised blade.

        Args:
            config: Parameter set. Defaults to the as-designed GOE 210 wing.
        """
        self._config = config if config is not None else RotatingWingConfig()
        self._config.validate()
        self._build_geometry()
        self._reset_inflow_state()

    # -----------------------------------------------------------------------------------
    # Construction
    # -----------------------------------------------------------------------------------

    def _build_geometry(self) -> None:
        """Discretise the blade span and pre-compute everything that never changes."""
        cfg = self._config

        # Midpoint rule over equal-width elements. Midpoints avoid evaluating the section
        # exactly at the tip, where the Prandtl factor is singular.
        edges = np.linspace(cfg.root_radius_m, cfg.tip_radius_m, cfg.num_stations + 1)
        self._radius_m = 0.5 * (edges[:-1] + edges[1:])
        self._element_width_m = np.diff(edges)

        # Chord and pitch interpolated from the build schedule. Linear interpolation is
        # used rather than a spline: the schedule is dense enough that a spline buys no
        # accuracy, and linear interpolation cannot overshoot into a negative chord.
        self._chord_m = np.interp(
            self._radius_m,
            np.asarray(cfg.station_radius_m, dtype=float),
            np.asarray(cfg.station_chord_m, dtype=float),
        )
        pitch_deg = np.interp(
            self._radius_m,
            np.asarray(cfg.station_radius_m, dtype=float),
            np.asarray(cfg.station_pitch_deg, dtype=float),
        )
        self._pitch_rad = np.deg2rad(pitch_deg + cfg.collective_pitch_offset_deg)

        # Blade azimuths and the two unit vectors that define each blade's local frame.
        azimuth_rad = np.deg2rad(np.asarray(cfg.resolved_blade_azimuths_deg(), dtype=float))
        self._azimuth_rad = azimuth_rad

        cos_psi = np.cos(azimuth_rad)[:, None]
        sin_psi = np.sin(azimuth_rad)[:, None]

        # Radial (span) unit vector, body frame, per blade. Shape (n_blades, 1, 3).
        self._e_radial = np.stack(
            [cos_psi, sin_psi, np.zeros_like(cos_psi)], axis=-1
        )
        # Tangential unit vector, the direction the blade's leading edge points. For a
        # right-handed blade (rotation_sense = +1) that is e_t = z_hat x e_r, the
        # direction a blade moves under a positive yaw rate. A left-handed blade is the
        # mirror image, so its leading edge points the other way.
        #
        # Negating this vector is a COMPLETE mirror of the blade, not a partial one, and
        # that is why it is the only change handedness needs. It is used in exactly two
        # places: to project the relative wind onto the chordwise direction, and to place
        # the resulting in-plane force. Flipping both together reflects the blade through
        # a plane containing the spin axis, leaving the aerofoil upright -- camber still
        # on the suction side -- and the pitch angle still positive about the mirrored
        # chord line. Negating the PITCH instead would be a different object: a cambered
        # section flown inverted, which performs far worse.
        #
        # Shape (n_blades, 1, 3).
        self._e_tangential = float(cfg.rotation_sense) * np.stack(
            [-sin_psi, cos_psi, np.zeros_like(sin_psi)], axis=-1
        )

        # Element position vectors in the body frame, shape (n_blades, n_stations, 3).
        self._position_m = (
            self._e_radial * self._radius_m[None, :, None]
            + np.array([0.0, 0.0, cfg.blade_z_offset_m])[None, None, :]
        )

        self._z_hat = np.array([0.0, 0.0, 1.0])
        self._moment_reference_m = np.asarray(cfg.moment_reference_offset_m, dtype=float)

    def _reset_inflow_state(self) -> None:
        """Clear the induced-velocity warm start."""
        self._induced_velocity_m_s = np.zeros_like(self._radius_m)

    def reset(self) -> None:
        """Reset the solver's warm start. Call when the vehicle is teleported or the
        episode restarts, so that a stale inflow solution does not leak across a
        discontinuity in state."""
        self._reset_inflow_state()

    # -----------------------------------------------------------------------------------
    # Properties
    # -----------------------------------------------------------------------------------

    @property
    def config(self) -> RotatingWingConfig:
        """The configuration this wing was built from."""
        return self._config

    @property
    def radius_m(self) -> np.ndarray:
        """Station radii [m], shape (num_stations,)."""
        return self._radius_m.copy()

    @property
    def chord_m(self) -> np.ndarray:
        """Station chords [m], shape (num_stations,)."""
        return self._chord_m.copy()

    @property
    def pitch_rad(self) -> np.ndarray:
        """Station pitch angles including the collective offset [rad]."""
        return self._pitch_rad.copy()

    # -----------------------------------------------------------------------------------
    # Main entry point
    # -----------------------------------------------------------------------------------

    def compute_loads(
        self,
        linear_body_velocity_m_s,
        angular_velocity_body_rad_s,
        dt: Optional[float] = None,
    ) -> WingLoads:
        """Compute the wing's aerodynamic force and moment for the current vehicle state.

        Args:
            linear_body_velocity_m_s: Velocity of the body origin through the air,
                expressed in the body frame (FLU) [m/s], shape (3,).
            angular_velocity_body_rad_s: Body angular rate [p, q, r] in the body frame
                [rad/s], shape (3,).
            dt: Physics timestep [s]. Required only by ``InflowModel.DYNAMIC``, which
                integrates the induced velocity as a state. The algebraic models ignore it.

        Returns:
            A :class:`WingLoads` carrying the resultant loads and the per-station
            diagnostics.

        Note:
            When ``config.enabled`` is False this returns a zero-load result immediately,
            without touching the polar. The ``--no-wing`` run mode therefore exercises the
            identical call path as the winged mode, differing only in the numbers.
        """
        cfg = self._config

        v_body = np.asarray(linear_body_velocity_m_s, dtype=float).reshape(3)
        omega = np.asarray(angular_velocity_body_rad_s, dtype=float).reshape(3)

        if not cfg.enabled:
            return self._zero_loads()

        if not (np.all(np.isfinite(v_body)) and np.all(np.isfinite(omega))):
            # A non-finite state means the integration has already diverged. Returning
            # zero load is the only safe action: propagating NaN into the physics engine
            # destroys the rest of the scene and hides the original fault.
            return self._zero_loads()

        # Velocity of each blade element through the air, body frame, shape (nb, ns, 3).
        # v_element = v_body + omega x r_element
        v_element = v_body[None, None, :] + np.cross(
            omega[None, None, :], self._position_m
        )

        # Resolve onto the blade's local tangential axis and the rotor axis. The radial
        # component is discarded: blade element independence.
        u_tangential = np.einsum("bsi,bti->bs", v_element, self._e_tangential)
        climb_rate = v_element[..., 2]

        induced, residual, iterations = self._solve_inflow(
            u_tangential, climb_rate, v_body, dt
        )

        loads = self._integrate_blade_elements(u_tangential, climb_rate, induced)
        loads.inflow_residual_m_s = residual
        loads.inflow_iterations = iterations
        return loads

    # -----------------------------------------------------------------------------------
    # Blade element integration
    # -----------------------------------------------------------------------------------

    def _section_state(self, u_tangential, climb_rate, induced):
        """Compute the per-element flow state and section coefficients.

        Args:
            u_tangential: In-plane velocity component [m/s], shape (nb, ns).
            climb_rate: Rotor-axis velocity component of the element [m/s], shape (nb, ns).
            induced: Induced velocity per station [m/s], shape (ns,).

        Returns:
            Tuple of (u_perp, v_rel, phi, alpha, c_l, c_d, reynolds), each shape (nb, ns)
            except as broadcast.
        """
        cfg = self._config

        u_perp = climb_rate + induced[None, :]

        v_rel = np.sqrt(u_tangential ** 2 + u_perp ** 2)
        v_rel = np.minimum(v_rel, cfg.max_relative_speed_m_s)

        phi = np.arctan2(u_perp, u_tangential)
        alpha = self._pitch_rad[None, :] - phi

        # Wrap the angle of attack into (-pi, pi]. Without this, a reversed-flow element
        # can report an angle outside the polar's mirrored domain.
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        reynolds = v_rel * self._chord_m[None, :] / cfg.kinematic_viscosity_m2_s
        c_l, c_d = cfg.polar.evaluate(alpha, reynolds)

        return u_perp, v_rel, phi, alpha, c_l, c_d, reynolds

    def _element_forces(self, v_rel, phi, c_l, c_d):
        """Section force per unit span, resolved into rotor-axis and in-plane parts.

        Returns:
            Tuple of (df_axial_per_m, df_tangential_per_m), each shape (nb, ns) [N/m].
            ``df_axial_per_m`` is along +z. ``df_tangential_per_m`` is along +e_t, so it
            is negative whenever the wing is being dragged through the air, which is the
            normal case.
        """
        cfg = self._config
        dyn_pressure = 0.5 * cfg.air_density_kg_m3 * v_rel ** 2 * self._chord_m[None, :]

        lift_per_m = dyn_pressure * c_l
        drag_per_m = dyn_pressure * c_d

        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)

        df_axial = lift_per_m * cos_phi - drag_per_m * sin_phi
        df_tangential = -(lift_per_m * sin_phi + drag_per_m * cos_phi)

        return df_axial, df_tangential

    def _integrate_blade_elements(self, u_tangential, climb_rate, induced) -> WingLoads:
        """Integrate the section loads into a resultant force and moment."""
        cfg = self._config

        _, v_rel, phi, alpha, c_l, c_d, reynolds = self._section_state(
            u_tangential, climb_rate, induced
        )
        df_axial, df_tangential = self._element_forces(v_rel, phi, c_l, c_d)

        # Element force vectors in the body frame, shape (nb, ns, 3).
        width = self._element_width_m[None, :, None]
        force_elements = (
            df_axial[..., None] * self._z_hat[None, None, :]
            + df_tangential[..., None] * self._e_tangential
        ) * width

        force_total = force_elements.sum(axis=(0, 1))

        # Moments about the configured reference point.
        lever = self._position_m - self._moment_reference_m[None, None, :]
        moment_total = np.cross(lever, force_elements).sum(axis=(0, 1))

        # Aerodynamic power: the in-plane force acting against the element's own in-plane
        # motion. This is the work the airframe's rotation must supply, and it is positive
        # when the wing absorbs power.
        power = float(-(df_tangential * u_tangential * self._element_width_m[None, :]).sum())

        return WingLoads(
            force_body_n=force_total,
            moment_body_nm=moment_total,
            thrust_n=float(force_total[2]),
            torque_z_nm=float(moment_total[2]),
            aero_power_w=power,
            induced_velocity_m_s=induced.copy(),
            alpha_rad=alpha,
            cl=c_l,
            cd=c_d,
            reynolds=reynolds,
            inflow_residual_m_s=0.0,
            inflow_iterations=0,
        )

    def _zero_loads(self) -> WingLoads:
        """A fully-formed zero result, so callers never special-case the disabled path."""
        shape = (self._config.num_blades, self._radius_m.size)
        zeros_2d = np.zeros(shape)
        return WingLoads(
            force_body_n=np.zeros(3),
            moment_body_nm=np.zeros(3),
            thrust_n=0.0,
            torque_z_nm=0.0,
            aero_power_w=0.0,
            induced_velocity_m_s=np.zeros_like(self._radius_m),
            alpha_rad=zeros_2d,
            cl=zeros_2d.copy(),
            cd=zeros_2d.copy(),
            reynolds=zeros_2d.copy(),
            inflow_residual_m_s=0.0,
            inflow_iterations=0,
        )

    # -----------------------------------------------------------------------------------
    # Induced velocity
    # -----------------------------------------------------------------------------------

    def _solve_inflow(
        self, u_tangential, climb_rate, v_body, dt: Optional[float] = None
    ) -> Tuple[np.ndarray, float, int]:
        """Dispatch to the configured induced-velocity model."""
        model = self._config.inflow_model

        if model == InflowModel.NONE:
            zero = np.zeros_like(self._radius_m)
            self._induced_velocity_m_s = zero
            return zero, 0.0, 0

        if model == InflowModel.BEMT:
            return self._solve_inflow_bemt(u_tangential, climb_rate)

        if model == InflowModel.GLAUERT:
            return self._solve_inflow_glauert(u_tangential, climb_rate, v_body)

        if model == InflowModel.DYNAMIC:
            return self._solve_inflow_dynamic(u_tangential, climb_rate, v_body, dt)

        raise ValueError(f"unhandled inflow model {model!r}")

    def _prandtl_loss(self, phi):
        """Prandtl tip- and root-loss factor, azimuthally averaged over the blades.

        The factor accounts for the finite number of blades: near the tip the bound
        circulation must fall to zero, so the annulus carries less momentum than an
        actuator disk of the same area would.

            f = (B/2) * (R_tip - r) / (r * |sin phi|)
            F = (2/pi) * arccos(exp(-f))

        with an analogous root term. Both are clamped from below by
        ``config.tip_loss_floor``: as F approaches zero the momentum balance would demand
        an unbounded induced velocity.

        Args:
            phi: Inflow angle per blade and station [rad], shape (nb, ns).

        Returns:
            Loss factor per station [-], shape (ns,).
        """
        cfg = self._config
        if not (cfg.use_tip_loss or cfg.use_root_loss):
            return np.ones_like(self._radius_m)

        # Azimuthal mean of |sin phi| over the blades: the Prandtl factor is a
        # disk-averaged correction, so it is evaluated on the mean inflow angle.
        sin_phi = np.mean(np.abs(np.sin(phi)), axis=0)
        sin_phi = np.maximum(sin_phi, 1.0e-6)

        half_b = 0.5 * cfg.num_blades
        factor = np.ones_like(self._radius_m)

        if cfg.use_tip_loss:
            f_tip = half_b * (cfg.tip_radius_m - self._radius_m) / (self._radius_m * sin_phi)
            factor = factor * (2.0 / np.pi) * np.arccos(
                np.clip(np.exp(-np.maximum(f_tip, 0.0)), 0.0, 1.0)
            )

        if cfg.use_root_loss and cfg.root_radius_m > 0.0:
            f_root = half_b * (self._radius_m - cfg.root_radius_m) / (
                self._radius_m * sin_phi
            )
            factor = factor * (2.0 / np.pi) * np.arccos(
                np.clip(np.exp(-np.maximum(f_root, 0.0)), 0.0, 1.0)
            )

        return np.maximum(factor, cfg.tip_loss_floor)

    def _solve_inflow_bemt(self, u_tangential, climb_rate) -> Tuple[np.ndarray, float, int]:
        """Annulus-by-annulus blade element momentum balance.

        For each annulus the blade element thrust per unit radius must equal the momentum
        thrust per unit radius through that annulus:

            dT/dr = 4 * pi * rho * r * F * (V_c + v_i) * v_i

        Solving that quadratic for v_i gives the closed form used below:

            v_i = 0.5 * ( -V_c + sqrt( V_c^2 + (dT/dr) / (pi * rho * r * F) ) )

        which is iterated to convergence because dT/dr itself depends on v_i. Taking the
        root in closed form rather than bisecting is both faster and better conditioned,
        and it vectorises across all stations at once.

        Descending flight drives the discriminant negative: this is the vortex ring state,
        where momentum theory has no valid solution because the flow is recirculating
        rather than passing through the disk one way. The solver detects it and falls back
        to the empirical Johnson/Leishman approximation of holding the induced velocity at
        the hover value, which keeps the loads bounded and roughly correct in magnitude.

        Returns:
            Tuple of (induced_velocity, final_residual, iterations_taken).
        """
        cfg = self._config
        rho = cfg.air_density_kg_m3

        induced = self._induced_velocity_m_s.copy()
        if induced.shape != self._radius_m.shape:
            induced = np.zeros_like(self._radius_m)

        # Annulus-mean climb rate: momentum theory is a disk-averaged statement.
        climb_mean = np.mean(climb_rate, axis=0)

        residual = 0.0
        iterations = 0

        for iterations in range(1, cfg.inflow_max_iterations + 1):
            _, v_rel, phi, _, c_l, c_d = self._section_state(
                u_tangential, climb_rate, induced
            )[:6]
            df_axial, _ = self._element_forces(v_rel, phi, c_l, c_d)

            # Total blade element thrust gradient for the annulus, summed over blades.
            dt_dr = df_axial.sum(axis=0)

            loss = self._prandtl_loss(phi)
            denominator = np.pi * rho * self._radius_m * loss

            discriminant = climb_mean ** 2 + dt_dr / denominator
            valid = discriminant >= 0.0

            induced_new = np.where(
                valid,
                0.5 * (-climb_mean + np.sqrt(np.maximum(discriminant, 0.0))),
                # Vortex ring state: hold the magnitude the hover relation would give.
                np.sign(dt_dr) * np.sqrt(np.abs(dt_dr) / np.maximum(denominator, 1e-12)) * 0.5,
            )

            relaxed = induced + cfg.inflow_relaxation * (induced_new - induced)
            residual = float(np.max(np.abs(relaxed - induced))) if relaxed.size else 0.0
            induced = relaxed

            if residual < cfg.inflow_tolerance_m_s:
                break

        self._induced_velocity_m_s = induced
        return induced, residual, iterations

    def _solve_inflow_glauert(
        self, u_tangential, climb_rate, v_body
    ) -> Tuple[np.ndarray, float, int]:
        """Uniform induced velocity from Glauert's edgewise momentum relation.

        A single induced velocity is applied over the whole disk, satisfying

            v_i = T / (2 * rho * A * V_eff),    V_eff = sqrt(V_edge^2 + (V_c + v_i)^2)

        where ``V_edge`` is the in-plane speed of the hub through the air. In hover this
        reduces to the familiar v_i = sqrt(T / (2*rho*A)); at high edgewise speed it tends
        to the fixed-wing limit v_i = T / (2*rho*A*V). It is cheaper and far better
        conditioned than BEMT in fast translation, but it discards the radial distribution
        of the inflow.

        Returns:
            Tuple of (induced_velocity_broadcast_to_stations, final_residual, iterations).
        """
        cfg = self._config
        rho = cfg.air_density_kg_m3
        area = cfg.disk_area_m2()

        # Edgewise speed of the hub through the air: the magnitude of the in-plane
        # components of the body-frame velocity. Taken directly from the vehicle state
        # rather than reconstructed from the element velocities, because the rotational
        # contribution only cancels exactly for a symmetric blade arrangement.
        v_edge = float(np.hypot(v_body[0], v_body[1]))

        uniform = float(np.mean(self._induced_velocity_m_s))
        climb_mean = float(np.mean(climb_rate))

        residual = 0.0
        iterations = 0

        for iterations in range(1, cfg.inflow_max_iterations + 1):
            induced_vec = np.full_like(self._radius_m, uniform)
            _, v_rel, phi, _, c_l, c_d = self._section_state(
                u_tangential, climb_rate, induced_vec
            )[:6]
            df_axial, _ = self._element_forces(v_rel, phi, c_l, c_d)
            thrust = float((df_axial * self._element_width_m[None, :]).sum())

            v_eff = np.sqrt(v_edge ** 2 + (climb_mean + uniform) ** 2)
            v_eff = max(v_eff, 1.0e-3)

            uniform_new = thrust / (2.0 * rho * area * v_eff)
            relaxed = uniform + cfg.inflow_relaxation * (uniform_new - uniform)
            residual = abs(relaxed - uniform)
            uniform = relaxed

            if residual < cfg.inflow_tolerance_m_s:
                break

        induced = np.full_like(self._radius_m, uniform)
        self._induced_velocity_m_s = induced
        return induced, residual, iterations

    def _solve_inflow_dynamic(
        self, u_tangential, climb_rate, v_body, dt: Optional[float]
    ) -> Tuple[np.ndarray, float, int]:
        """First-order dynamic inflow: the induced velocity is a state, not a constraint.

        THE POINT
            Momentum theory gives the induced velocity that *eventually* balances a given
            thrust. It does not say how fast the flow gets there, and the answer is not
            "instantly": accelerating the air column through the disc takes a moment,
            because that column has mass.

        THE TIME CONSTANT
            The apparent mass of the air accelerated by a disc is ``m_a = (8/3) rho R^3``
            -- the impermeable-disc value, which Pitt & Peters (1980) confirm is exactly
            the M11 apparent-mass term of their dynamic-inflow model. Linearising the
            momentum relation ``T = 2 rho A V_eff v`` about the operating point gives

                dT/dv = 2 rho A [ V_eff + v (V_c + v) / V_eff ]
                tau   = m_a / (dT/dv)

            which reduces to ``2R/(3 pi v)`` in hover and ``4R/(3 pi V)`` in fast edgewise
            flight. For this wing in hover it is about 67 ms.

        WHY IT IS ALSO CHEAPER
            The loads are evaluated at the CURRENT inflow state and the state is then
            advanced once. There is no fixed-point iteration, so the cost is the same on
            every step -- which is what a bounded worst case requires, as distinct from a
            bounded average.

        The update is the exact solution of the linear ODE over the step,
        ``v <- v + (v_steady - v)(1 - exp(-dt/tau))``, rather than a forward Euler step, so
        it stays stable for any dt relative to tau instead of only for dt < 2 tau.

        Args:
            u_tangential: Chordwise flow at each element [m/s].
            climb_rate: Axial flow at each element [m/s].
            v_body: Body-frame velocity [m/s], shape (3,).
            dt: Timestep [s].

        Returns:
            Tuple of (induced velocity per station, |v_steady - v| as the residual, 1).

        Raises:
            ValueError: If ``dt`` is missing or non-positive. Silently falling back to a
                steady solve would make the model quietly behave as a different one.
        """
        if dt is None or dt <= 0.0:
            raise ValueError(
                "InflowModel.DYNAMIC integrates the inflow in time and needs a positive "
                "dt; pass dt to compute_loads(), or use 'bemt'/'glauert' for a steady "
                "solve"
            )

        cfg = self._config
        rho = cfg.air_density_kg_m3
        area = cfg.disk_area_m2()
        radius = cfg.tip_radius_m

        # Loads at the inflow the flow actually has right now -- this is the state, not a
        # guess to be refined.
        uniform = float(np.mean(self._induced_velocity_m_s))
        induced = np.full_like(self._radius_m, uniform)
        _, v_rel, phi, _, c_l, c_d = self._section_state(
            u_tangential, climb_rate, induced
        )[:6]
        df_axial, _ = self._element_forces(v_rel, phi, c_l, c_d)
        thrust = float((df_axial * self._element_width_m[None, :]).sum())

        v_edge = float(np.hypot(v_body[0], v_body[1]))
        climb_mean = float(np.mean(climb_rate))

        # Quasi-steady target for the present thrust. This must SOLVE Glauert's relation
        #
        #     v sqrt(V_edge^2 + (V_c + v)^2) = T / (2 rho A)
        #
        # not divide the thrust by an effective velocity evaluated at the current inflow.
        # Doing the latter blows up from a cold start: with v = 0 in hover the effective
        # velocity collapses to its numerical floor and the implied target reaches
        # thousands of metres per second.
        uniform_steady = _solve_glauert_inflow(
            thrust / (2.0 * rho * area), v_edge, climb_mean, uniform
        )
        v_eff = max(
            np.sqrt(v_edge ** 2 + (climb_mean + uniform) ** 2), 1.0e-3
        )

        # Apparent mass and the linearised sensitivity.
        #
        # The sensitivity is evaluated at the TARGET operating point, not the current one.
        # Anchoring it at the current state degenerates on a cold start: with v = 0 the
        # effective velocity collapses to its floor, dT/dv goes to nearly zero and the
        # time constant runs away to tens of seconds, so the inflow would crawl up from
        # rest instead of establishing itself in the tens of milliseconds it physically
        # takes. At equilibrium the two choices coincide, so this costs nothing where the
        # linearisation is valid and fixes it where it is not.
        apparent_mass = (8.0 / 3.0) * rho * radius ** 3
        v_eff_ref = max(
            np.sqrt(v_edge ** 2 + (climb_mean + uniform_steady) ** 2), 1.0e-3
        )
        dT_dv = 2.0 * rho * area * (
            v_eff_ref + abs(uniform_steady) * (climb_mean + uniform_steady) / v_eff_ref
        )
        tau = apparent_mass / max(abs(dT_dv), 1.0e-6)
        tau = float(np.clip(tau, 1.0e-4, 10.0))

        blend = 1.0 - np.exp(-dt / tau)
        residual = abs(uniform_steady - uniform)
        uniform = uniform + (uniform_steady - uniform) * blend

        induced = np.full_like(self._radius_m, uniform)
        self._induced_velocity_m_s = induced
        self._inflow_time_constant_s = tau
        return induced, residual, 1

    @property
    def inflow_time_constant_s(self) -> float:
        """Time constant from the most recent dynamic-inflow update [s].

        Zero until :class:`InflowModel.DYNAMIC` has been stepped at least once.
        """
        return getattr(self, "_inflow_time_constant_s", 0.0)

    # -----------------------------------------------------------------------------------
    # Analysis helpers
    # -----------------------------------------------------------------------------------

    def hover_loads(self, spin_rate_rad_s: float) -> WingLoads:
        """Loads in steady hover at a given spin rate, with no translation.

        A convenience wrapper for analysis and testing; equivalent to calling
        :meth:`compute_loads` with zero linear velocity and a pure yaw rate.

        Args:
            spin_rate_rad_s: Airframe yaw rate about +z [rad/s]. Positive is the
                right-handed sense.
        """
        return self.compute_loads(
            linear_body_velocity_m_s=np.zeros(3),
            angular_velocity_body_rad_s=np.array([0.0, 0.0, spin_rate_rad_s]),
        )

    def blade_area_m2(self) -> float:
        """Total blade planform area from the discretised geometry [m^2].

        This integrates the same chord distribution the solver uses, so it will differ
        very slightly from :meth:`RotatingWingConfig.blade_area_m2`, which integrates the
        raw station table.
        """
        return float(
            self._config.num_blades * trapezoid(self._chord_m, self._radius_m)
        )

    def blade_mass_kg(self) -> float:
        """Mass of ONE blade panel [kg].

        Taken from ``config.blade_mass_kg`` when that is set, otherwise derived from the
        areal density and the discretised planform area.
        """
        cfg = self._config
        if cfg.blade_mass_kg is not None:
            return float(cfg.blade_mass_kg)

        area_per_blade = self.blade_area_m2() / max(cfg.num_blades, 1)
        return float(cfg.blade_areal_density_kg_m2 * area_per_blade)

    def inertia_about_body_origin(self) -> Tuple[float, np.ndarray]:
        """Mass and diagonal inertia of the whole wing about the body origin.

        MODEL
            Each blade's mass is distributed along its span in proportion to the local
            chord, which is what a panel of uniform areal density gives. For a station at
            radius r, azimuth psi, height z0, carrying mass dm spread over a chord c:

                I_xx = sum dm * (r^2 sin^2(psi) + z0^2 + (c^2/12) sin^2(psi + pi/2))
                I_yy = sum dm * (r^2 cos^2(psi) + z0^2 + (c^2/12) cos^2(psi + pi/2))
                I_zz = sum dm * (r^2 + c^2/12)

            The ``c^2/12`` terms are the chordwise second moment of a strip about its own
            span line; the chord lies along the tangential direction, hence the
            ``psi + pi/2``. They are small against ``r^2`` outboard but matter near the
            root where the chord is comparable to the radius.

            Products of inertia are not returned. For the symmetric blade arrangements
            this model is used with they vanish, and the simulator's rigid body takes a
            diagonal inertia anyway.

        Returns:
            Tuple of (total_wing_mass_kg, diagonal_inertia_kg_m2) with the inertia as a
            3-vector [I_xx, I_yy, I_zz].

        Note:
            The result is about the BODY ORIGIN, not about the wing's own centre of mass.
            That is what the simulator needs, since the airframe's inertia is expressed
            there too.
        """
        cfg = self._config

        mass_per_blade = self.blade_mass_kg()
        if mass_per_blade <= 0.0:
            return 0.0, np.zeros(3)

        # Spanwise mass distribution proportional to chord, normalised per blade.
        strip_area = self._chord_m * self._element_width_m
        total_area = float(np.sum(strip_area))
        if total_area <= 0.0:
            return 0.0, np.zeros(3)
        strip_mass = mass_per_blade * strip_area / total_area

        z0 = cfg.blade_z_offset_m
        chord_moment = self._chord_m ** 2 / 12.0

        i_xx = 0.0
        i_yy = 0.0
        i_zz = 0.0

        for azimuth in np.deg2rad(np.asarray(cfg.resolved_blade_azimuths_deg(), float)):
            sin_psi2 = np.sin(azimuth) ** 2
            cos_psi2 = np.cos(azimuth) ** 2
            # The chord lies along the tangential direction, i.e. at azimuth + 90 deg.
            sin_chord2 = np.sin(azimuth + 0.5 * np.pi) ** 2
            cos_chord2 = np.cos(azimuth + 0.5 * np.pi) ** 2

            r2 = self._radius_m ** 2
            i_xx += float(
                np.sum(strip_mass * (r2 * sin_psi2 + z0 ** 2 + chord_moment * sin_chord2))
            )
            i_yy += float(
                np.sum(strip_mass * (r2 * cos_psi2 + z0 ** 2 + chord_moment * cos_chord2))
            )
            i_zz += float(np.sum(strip_mass * (r2 + chord_moment)))

        total_mass = mass_per_blade * cfg.num_blades
        return total_mass, np.array([i_xx, i_yy, i_zz])
