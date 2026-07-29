#!/usr/bin/env python
"""
| File: robustness.py
| Description: The three perturbations the robustness study applies, in one place.

What this module is for
───────────────────────
The sweeps tune controllers against a plant that is exactly the plant they
model. That measures nominal performance and says nothing about what happens
when the model is wrong — which, on a real airframe, it always is. This module
supplies the three ways of being wrong that matter most here:

    inertia / mass mismatch   the vehicle is not what the controller thinks
    sensor lag                the state is late and smoothed
    wind                      an unmodelled external force

Each is a knob with a magnitude, so a controller's robustness can be reported
as the magnitude at which it stops flying rather than as an adjective.

The zero point is real now
──────────────────────────
This module would have been meaningless before examples/utils/vehicle_params.py
landed. Every controller carried a standing mass error (−7.3 % for Eigen and
PD, −25.9 % for MPC) and a standing inertia error, so "mismatch = 1.0" was not
a matched condition and the three controllers were not even mismatched by the
same amount. They now all model the composite vehicle exactly, and a scale
factor of 1.0 here is a genuinely matched plant.

Verified against the simulator
──────────────────────────────
Two mechanisms this module relies on were checked rather than assumed, in
examples/controller_tests/evaluations/probe_robustness.py:

    Writing mass/inertia through ``UsdPhysics.MassAPI`` is ignored while the
    physics scene is live and takes effect on the next ``world.reset()``.
    Callers must reset after ``apply_plant_scale`` or the run is nominal and
    will not say so.

    A force applied from inside ``Backend.update`` does reach the rigid body
    (measured: 2 N for 0.6 s moved the vehicle 0.209 m against a 0.000 m null),
    so wind needs no change to the Pegasus extension.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

try:                                    # Only needed for the plant-side scaling.
    from pxr import UsdPhysics
except ImportError:                     # pragma: no cover - non-Isaac interpreter
    UsdPhysics = None

import vehicle_params


def rotate_by_quaternion(vector, quaternion, inverse: bool = False) -> np.ndarray:
    """Rotate a vector by a [qx, qy, qz, qw] quaternion, in pure numpy.

    Written out rather than delegated to ``scipy.spatial.transform.Rotation`` so
    this module imports nothing from Isaac Sim or scipy and can be exercised
    from a plain interpreter — the same reason sweep_metrics.py stays pure
    numpy. The perturbations are the part of the pipeline most in need of
    unit tests, and a module that only runs inside a GPU simulator does not get
    them.

    ``inverse=True`` applies the conjugate, i.e. world → body.
    """
    q = np.asarray(quaternion, dtype=float)
    v = np.asarray(vector, dtype=float)
    qv = q[:3].copy()
    qw = float(q[3])
    if inverse:
        qv = -qv
    # v' = v + 2·qv × (qv × v + qw·v)
    t = np.cross(qv, v) + qw * v
    return v + 2.0 * np.cross(qv, t)


# ── Inertia / mass mismatch ──────────────────────────────────────────────────
#
# The perturbation is defined on the COMPOSITE vehicle, not on the /body link.
# Scaling the body link directly would mislabel the axis: the four rotors are
# 0.1186 kg that the body link does not carry, and their parallel-axis terms are
# 16-20 % of the composite inertia, so a "×2 inertia" applied to /body delivers
# ×1.84 to the vehicle. The solve below inverts that, and `verify_plant_scale`
# measures what was actually achieved rather than trusting the algebra.
#
# Mass and inertia are independent knobs. On a real object they are not — I
# scales as m·L² — but as a *model mismatch* axis, independence is the point:
# it separates "the hover feedforward is wrong" from "the torque model is
# wrong", which fail through completely different channels.

def nominal_decomposition(vehicle) -> dict:
    """Split the composite properties into body-link and everything-else parts.

    Returns the pieces ``solve_body_properties`` needs to hit a requested
    composite scale, or None if the physics interface cannot be read.
    """
    composite = vehicle_params.composite_mass_properties(vehicle)
    if composite is None:
        return None
    body = vehicle_params._rigid_body_properties(vehicle, vehicle_params.BODY_PART)
    if body is None:
        return None
    body_mass, body_moment, _ = body

    composite_inertia = np.array([composite["Ixx"], composite["Iyy"],
                                  composite["Izz"]], dtype=float)

    # Everything the body link does not contribute: the rotors' own moments
    # plus every parallel-axis term, including the body's own (tiny — the
    # composite COM sits ~1.7 mm from the body frame — but carried exactly so
    # the solve does not need to argue about it).
    remainder = composite_inertia - np.asarray(body_moment, dtype=float)

    return {
        "composite_mass": composite["mass"],
        "composite_inertia": composite_inertia,
        "body_mass": float(body_mass),
        "body_moment": np.asarray(body_moment, dtype=float),
        "other_mass": float(composite["mass"] - body_mass),
        "inertia_remainder": remainder,
    }


def solve_body_properties(decomposition: dict, mass_scale: float = 1.0,
                          inertia_scale: float = 1.0) -> dict:
    """Body-link mass and inertia that give the composite the requested scale.

    Raises ValueError when the request is not physically representable — a
    mass_scale small enough to need a negative body mass, say. Better to refuse
    at setup than to write a nonsense value and have PhysX quietly clamp it.
    """
    target_mass = mass_scale * decomposition["composite_mass"]
    body_mass = target_mass - decomposition["other_mass"]
    if body_mass <= 0.0:
        raise ValueError(
            f"mass_scale={mass_scale:g} needs a body-link mass of "
            f"{body_mass:.6f} kg: the rotors alone are "
            f"{decomposition['other_mass']:.6f} kg, so the composite cannot go "
            f"below {decomposition['other_mass'] / decomposition['composite_mass']:.4f}x")

    target_inertia = inertia_scale * decomposition["composite_inertia"]
    body_moment = target_inertia - decomposition["inertia_remainder"]
    if np.any(body_moment <= 0.0):
        raise ValueError(
            f"inertia_scale={inertia_scale:g} needs a body-link inertia of "
            f"{tuple(float(v) for v in body_moment)}, which is not positive. The "
            f"non-body contribution is {tuple(float(v) for v in decomposition['inertia_remainder'])} "
            f"kg·m², so the composite cannot go below "
            f"{float(np.max(decomposition['inertia_remainder'] / decomposition['composite_inertia'])):.4f}x")

    return {"mass": float(body_mass), "moment": body_moment}


def apply_plant_scale(vehicle, mass_scale: float = 1.0,
                      inertia_scale: float = 1.0,
                      decomposition: dict = None) -> dict:
    """Scale the vehicle's composite mass and inertia by the given factors.

    **The caller must call ``world.reset()`` afterwards.** PhysX reads mass
    properties at scene construction; the write is accepted immediately and
    ignored until the next reset (measured in probe_robustness.py). A driver
    that forgets this runs a nominal vehicle while its CSV says otherwise.

    Returns the decomposition and the values written, for logging.
    """
    if UsdPhysics is None:
        raise RuntimeError("pxr.UsdPhysics unavailable — plant scaling needs Isaac Sim")

    if decomposition is None:
        decomposition = nominal_decomposition(vehicle)
    if decomposition is None:
        raise RuntimeError("could not read the vehicle's nominal mass properties")

    target = solve_body_properties(decomposition, mass_scale, inertia_scale)

    prim = vehicle._current_stage.GetPrimAtPath(
        vehicle._stage_prefix + vehicle_params.BODY_PART)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"no body prim at {vehicle._stage_prefix}"
                           f"{vehicle_params.BODY_PART}")

    # MassAPI.Apply rather than GetAttribute().Set(): setting an attribute that
    # does not exist is a silent no-op, indistinguishable from a write PhysX
    # chose to ignore.
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(target["mass"])
    mass_api.CreateDiagonalInertiaAttr().Set(
        tuple(float(v) for v in target["moment"]))

    return {"decomposition": decomposition, "written": target,
            "mass_scale": float(mass_scale), "inertia_scale": float(inertia_scale)}


def verify_plant_scale(vehicle, nominal: dict, mass_scale: float,
                       inertia_scale: float, tol: float = 0.01) -> dict:
    """Measure the scale actually achieved, after ``world.reset()``.

    The solve above is exact given the decomposition, but the decomposition is
    read from PhysX and the write goes through USD — two representations that
    have no obligation to agree. Reporting the achieved scale keeps the axis
    label honest, and ``within_tolerance`` tells a driver whether to trust it.
    """
    achieved = vehicle_params.composite_mass_properties(vehicle)
    if achieved is None:
        return {"ok": False, "error": "could not read achieved mass properties"}

    achieved_inertia = np.array([achieved["Ixx"], achieved["Iyy"], achieved["Izz"]])
    mass_ratio = achieved["mass"] / nominal["composite_mass"]
    inertia_ratio = achieved_inertia / nominal["composite_inertia"]

    mass_err = abs(mass_ratio - mass_scale)
    inertia_err = float(np.max(np.abs(inertia_ratio - inertia_scale)))

    return {
        "ok": True,
        "mass_scale_achieved": float(mass_ratio),
        "inertia_scale_achieved": tuple(float(v) for v in inertia_ratio),
        "within_tolerance": bool(mass_err <= tol * max(mass_scale, 1.0)
                                 and inertia_err <= tol * max(inertia_scale, 1.0)),
        "achieved": achieved,
    }


# ── Sensor lag ───────────────────────────────────────────────────────────────

class SensorLag:
    """Delays and smooths the state a controller sees, leaving truth untouched.

    Two independent channels, because a real vehicle's are not remotely alike:

        position / linear velocity   GPS-like, 50-200 ms
        attitude / angular rate      IMU-like, 2-10 ms

    Lumping them into one number would misattribute every failure. The attitude
    loop is where this vehicle's gyroscopic coning mode lives (see
    examples/utils/sweep_task.py), so it is far more sensitive to its own few
    milliseconds than to a tenth of a second on position.

    Each channel applies a pure transport delay (a ring buffer, quantised to
    the physics step of 1/250 s = 4 ms) and then an optional first-order lag of
    time constant tau. Delay and lag are different failure modes — delay adds
    phase without attenuation and is what destabilises a loop; lag attenuates
    and is what a filter actually does — so they are separate knobs.

    Attitude is filtered componentwise on the quaternion and renormalised. That
    is not a proper interpolation on SO(3), but at 4 ms steps the rotation
    between consecutive samples is a fraction of a degree, where the difference
    from a slerp is far below the perturbation being applied. It would need
    revisiting for step sizes or rates an order of magnitude larger.
    """

    def __init__(self, dt: float, pos_delay: float = 0.0, att_delay: float = 0.0,
                 pos_tau: float = 0.0, att_tau: float = 0.0):
        self.dt = float(dt)
        if self.dt <= 0.0:
            raise ValueError(f"dt must be > 0, got {dt}")
        self.pos_delay = max(0.0, float(pos_delay))
        self.att_delay = max(0.0, float(att_delay))
        self.pos_tau = max(0.0, float(pos_tau))
        self.att_tau = max(0.0, float(att_tau))

        # A delay shorter than half a step rounds to zero: report it rather
        # than let a run claim a perturbation it did not receive.
        self.pos_steps = int(round(self.pos_delay / self.dt))
        self.att_steps = int(round(self.att_delay / self.dt))

        self.reset()

    @property
    def active(self) -> bool:
        return bool(self.pos_steps or self.att_steps or self.pos_tau or self.att_tau)

    def describe(self) -> str:
        return (f"pos delay {1e3 * self.pos_delay:.1f} ms ({self.pos_steps} steps), "
                f"tau {1e3 * self.pos_tau:.1f} ms; "
                f"att delay {1e3 * self.att_delay:.1f} ms ({self.att_steps} steps), "
                f"tau {1e3 * self.att_tau:.1f} ms")

    def reset(self) -> None:
        # maxlen = steps + 1 so the left end is exactly `steps` old once full.
        self._pos_buf = deque(maxlen=self.pos_steps + 1)
        self._att_buf = deque(maxlen=self.att_steps + 1)
        self._pos_filt = None
        self._vel_filt = None
        self._quat_filt = None
        self._rate_filt = None

    @staticmethod
    def _blend(previous, target, dt: float, tau: float):
        """One first-order step toward `target`. tau <= 0 passes straight through."""
        if tau <= 0.0:
            return np.array(target, dtype=float)
        if previous is None:
            return np.array(target, dtype=float)
        alpha = dt / (tau + dt)
        return np.asarray(previous, dtype=float) + alpha * (
            np.asarray(target, dtype=float) - np.asarray(previous, dtype=float))

    def apply(self, state):
        """Return the state a lagged controller should see.

        ``state`` is left untouched — the flight monitor and the truth log read
        it directly, and they must not inherit the perturbation. See
        ``RobustnessBackend.update_state``.
        """
        position = np.asarray(state.position, dtype=float).copy()
        velocity = np.asarray(state.linear_velocity, dtype=float).copy()
        attitude = np.asarray(state.attitude, dtype=float).copy()
        rate = np.asarray(state.angular_velocity, dtype=float).copy()

        self._pos_buf.append((position, velocity))
        self._att_buf.append((attitude, rate))

        # Before the buffer fills, the oldest sample available is the best
        # approximation of a delayed one. This only affects the first few
        # milliseconds of a run, during which the vehicle is stationary on the
        # ground, so it introduces no transient of its own.
        delayed_position, delayed_velocity = self._pos_buf[0]
        delayed_attitude, delayed_rate = self._att_buf[0]

        self._pos_filt = self._blend(self._pos_filt, delayed_position, self.dt, self.pos_tau)
        self._vel_filt = self._blend(self._vel_filt, delayed_velocity, self.dt, self.pos_tau)
        self._rate_filt = self._blend(self._rate_filt, delayed_rate, self.dt, self.att_tau)

        quat = self._blend(self._quat_filt, delayed_attitude, self.dt, self.att_tau)
        norm = float(np.linalg.norm(quat))
        quat = quat / norm if norm > 1e-9 else np.array([0.0, 0.0, 0.0, 1.0])
        self._quat_filt = quat

        return LaggedState(self._pos_filt.copy(), quat.copy(),
                           self._vel_filt.copy(), self._rate_filt.copy(), state)


class LaggedState:
    """A State-shaped view carrying perturbed values.

    Deliberately not a copy of the real ``State``: constructing one and then
    overwriting four fields leaves the derived quantities (``linear_body_velocity``,
    ``linear_acceleration``) silently inconsistent with the perturbed ones. Here
    they are recomputed where cheap and passed through where not, and the
    original is kept as ``truth`` so nothing needs to guess.
    """

    __slots__ = ("position", "attitude", "linear_velocity", "angular_velocity",
                 "linear_body_velocity", "linear_acceleration", "truth")

    def __init__(self, position, attitude, linear_velocity, angular_velocity, truth):
        self.position = position
        self.attitude = attitude
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.truth = truth
        # Consistent with the perturbed attitude and velocity rather than with
        # the true ones — a lagged estimator would derive it the same way.
        self.linear_body_velocity = rotate_by_quaternion(
            linear_velocity, attitude, inverse=True)
        # Not delayed: no controller here reads it, and a differentiated
        # estimate of it would be noise rather than a perturbation.
        self.linear_acceleration = np.asarray(
            truth.linear_acceleration, dtype=float).copy()


# ── Wind ─────────────────────────────────────────────────────────────────────

class WindField:
    """World-frame wind velocity over time, in three profiles.

    ``constant``    steady wind at ``speed`` along ``direction``. Tests
                    steady-state disturbance rejection, which on these
                    controllers is largely a question of whether they have
                    integral action at all: LPVMPCController has ki_pos/ki_z
                    and should drive the offset to zero; EigenController and
                    CascadedPDController have no integral term and must settle
                    at a permanent position offset. If a constant-wind run
                    shows no offset for those two, the wind is not being
                    applied and the harness is broken.

    ``gust``        a 1-cosine gust of ``gust_duration`` starting at
                    ``gust_start``, on top of the steady component. The
                    classical discrete gust shape; tests transient rejection
                    and recovery rather than steady error.

    ``turbulence``  an Ornstein-Uhlenbeck process with stationary standard
                    deviation ``speed`` and correlation time ``tau``, on top of
                    the steady component. Seeded, so a given seed reproduces a
                    given gust history exactly and controllers can be compared
                    on identical air.

    Stateful: call ``update(dt)`` once per physics step.
    """

    PROFILES = ("none", "constant", "gust", "turbulence")

    def __init__(self, profile: str = "none", speed: float = 0.0,
                 direction_deg: float = 0.0, gust_start: float = 3.0,
                 gust_duration: float = 1.0, gust_speed: float = None,
                 turbulence_tau: float = 1.0, vertical: float = 0.0,
                 seed: int = 0):
        if profile not in self.PROFILES:
            raise ValueError(f"unknown wind profile {profile!r}; "
                             f"expected one of {self.PROFILES}")
        self.profile = profile
        self.speed = float(speed)
        self.direction = math.radians(float(direction_deg))
        self.gust_start = float(gust_start)
        self.gust_duration = max(1e-6, float(gust_duration))
        self.gust_speed = float(gust_speed) if gust_speed is not None else float(speed)
        self.turbulence_tau = max(1e-3, float(turbulence_tau))
        self.vertical = float(vertical)
        self.seed = int(seed)

        self._heading = np.array([math.cos(self.direction),
                                  math.sin(self.direction), 0.0])
        self.reset()

    @property
    def active(self) -> bool:
        return self.profile != "none" and (self.speed != 0.0
                                           or self.gust_speed != 0.0
                                           or self.vertical != 0.0)

    def describe(self) -> str:
        if not self.active:
            return "none"
        text = (f"{self.profile} {self.speed:.2f} m/s at "
                f"{math.degrees(self.direction):.0f} deg")
        if self.profile == "gust":
            text += (f", gust {self.gust_speed:.2f} m/s for "
                     f"{self.gust_duration:.2f} s from t={self.gust_start:.2f} s")
        elif self.profile == "turbulence":
            text += f", sigma {self.gust_speed:.2f} m/s, tau {self.turbulence_tau:.2f} s, seed {self.seed}"
        if self.vertical:
            text += f", vertical {self.vertical:.2f} m/s"
        return text

    def reset(self) -> None:
        self.t = 0.0
        self._rng = np.random.default_rng(self.seed)
        self._ou = np.zeros(3)
        self.velocity = np.zeros(3)

    def update(self, dt: float) -> np.ndarray:
        """Advance one step and return the world-frame wind velocity [m/s]."""
        self.t += float(dt)
        wind = np.zeros(3)

        if self.profile == "none":
            self.velocity = wind
            return wind

        wind = wind + self.speed * self._heading
        wind[2] += self.vertical

        if self.profile == "gust":
            elapsed = self.t - self.gust_start
            if 0.0 <= elapsed <= self.gust_duration:
                shape = 0.5 * (1.0 - math.cos(2.0 * math.pi * elapsed / self.gust_duration))
                wind = wind + self.gust_speed * shape * self._heading

        elif self.profile == "turbulence":
            # Exact discretisation of the OU process, not Euler-Maruyama:
            #
            #     w_{k+1} = w_k·e^(-dt/tau) + sigma·sqrt(1 - e^(-2dt/tau))·N(0,1)
            #
            # This is the exact transition density, so the stationary standard
            # deviation is sigma for ANY step size. The Euler form carries an
            # O(dt/tau) bias in the variance, which would make the wind
            # magnitude quietly depend on the physics timestep — the same class
            # of bug as the 1/60-vs-1/250 step-count error in sim_clock.py.
            decay = math.exp(-dt / self.turbulence_tau)
            noise = self._rng.standard_normal(3)
            self._ou = (self._ou * decay
                        + self.gust_speed * math.sqrt(1.0 - decay * decay) * noise)
            # Turbulence is horizontal unless a vertical component was asked
            # for; the drag model has no vertical coefficient by default, so a
            # vertical component would be silently lost. See wind_force_body.
            gust = self._ou.copy()
            if self.vertical == 0.0:
                gust[2] = 0.0
            wind = wind + gust

        self.velocity = wind
        return wind


def drag_coefficients(vehicle, override=None) -> np.ndarray:
    """The vehicle's linear drag diagonal — the channel wind acts through.

    Reads the coefficients actually in use rather than restating them, so wind
    and drag cannot disagree about the same vehicle.

    Note the default is ``[0.50, 0.30, 0.0]``: **the vertical coefficient is
    zero**, so vertical wind produces no force at all. That is a property of
    the Pegasus drag model, not of this module. Pass an override to study
    vertical gusts.
    """
    if override is not None:
        return np.asarray(override, dtype=float)
    try:
        matrix = vehicle._drag._drag_coefficients
        return np.asarray(np.diag(matrix), dtype=float)
    except Exception:
        return np.array([0.50, 0.30, 0.0])


def wind_force_body(wind_world, attitude, coefficients) -> np.ndarray:
    """Body-frame force [N] produced by a world-frame wind velocity.

    Wind enters through the same linear drag law the simulator already applies.
    Pegasus computes ``F_drag = -D · v_body`` from the vehicle's own velocity;
    with air moving at ``w`` the relative velocity becomes ``v - w``, so the
    total is ``-D·(v_body - w_body)``. The simulator has already applied the
    first term, and this returns exactly the remainder::

        F_wind_body = D · Rᵀ · w_world

    Deriving it through the existing drag law rather than inventing a separate
    force keeps the disturbance physically consistent with the vehicle's own
    aerodynamics: a vehicle already moving downwind feels proportionally less.
    """
    wind_body = rotate_by_quaternion(wind_world, attitude, inverse=True)
    return np.asarray(coefficients, dtype=float) * wind_body


# ── Reducing a run table to a breakdown point ────────────────────────────────

BREAKDOWN_COLUMNS = ["controller", "axis", "motor_moment", "config_rank",
                     "n_runs", "last_surviving", "first_failing",
                     "nominal_score", "score_at_last_surviving",
                     "degradation_ratio"]


def breakdown_rows(rows, origin: float = 1.0) -> list:
    """Reduce per-run results to one breakdown row per configuration.

    ``origin`` is the unperturbed magnitude — 1.0 for the scale axes, 0.0 for
    lag and wind. Magnitudes are ordered by distance from it, so an axis that
    runs both above and below its matched value (inertia can be scaled either
    way) still reduces to a single boundary.

    Three decisions worth stating, because each one has a plausible-looking
    alternative that would be wrong:

    **A magnitude counts as survived only if every seed at it survived.** A
    configuration that falls out of the sky on one gust realisation in three
    has not survived that wind speed; averaging the seeds would hide it.

    **Both the last surviving and the first failing magnitude are reported.** A
    grid brackets the boundary, it does not locate it. Quoting one number would
    imply a resolution the sweep does not have.

    **A configuration that survived everything gets an empty first_failing**
    rather than the largest value tested, so "never broke over the range
    tested" cannot be read as "broke at the largest value tested".
    """
    groups = {}
    for row in rows:
        key = (row["controller"], row["axis"], row["motor_moment"],
               row["config_rank"])
        groups.setdefault(key, []).append(row)

    out = []
    for key, group in sorted(groups.items(), key=lambda kv: str(kv[0])):
        by_magnitude = {}
        for row in group:
            by_magnitude.setdefault(row["magnitude"], []).append(row)

        survived, failed = [], []
        for magnitude, runs in by_magnitude.items():
            ok = all(int(r.get("stable", 0)) == 1 and not r.get("failure")
                     for r in runs)
            (survived if ok else failed).append(magnitude)
        survived.sort(key=lambda m: abs(m - origin))
        failed.sort(key=lambda m: abs(m - origin))

        nominal = [r for r in group if r["magnitude"] == origin]
        nominal_score = (min(float(r["score"]) for r in nominal)
                         if nominal else float("nan"))
        edge = ([r for r in group if r["magnitude"] == survived[-1]]
                if survived else [])
        edge_score = (max(float(r["score"]) for r in edge)
                      if edge else float("nan"))
        ratio = (edge_score / nominal_score
                 if np.isfinite(nominal_score) and nominal_score > 0
                 and np.isfinite(edge_score) else float("nan"))

        out.append({
            "controller": key[0], "axis": key[1], "motor_moment": key[2],
            "config_rank": key[3], "n_runs": len(group),
            "last_surviving": survived[-1] if survived else None,
            "first_failing": failed[0] if failed else None,
            "nominal_score": nominal_score,
            "score_at_last_surviving": edge_score,
            "degradation_ratio": ratio,
        })
    return out


# ── The backend that ties it together ────────────────────────────────────────

class RobustnessBackend:
    """Wraps a controller, perturbs what it sees and what acts on it.

    Follows the ``Bounded*Backend`` pattern each sweep already uses — it owns
    the FlightMonitor and forwards everything else to an inner controller — and
    adds the two perturbations that have to live in the loop:

        wind        applied as a body-frame force from ``update``, which
                    Multirotor.update calls after its own drag and damping.
        sensor lag  applied to the state handed to the inner controller.

    The inertia perturbation is not here: it is a property of the vehicle, set
    once before ``world.reset()`` by ``apply_plant_scale``.

    Two invariants this class exists to hold
    ────────────────────────────────────────
    **The flight monitor sees truth.** Crash and takeoff detection must not
    inherit the perturbation, or the threshold for "crashed" becomes a function
    of how much lag was dialled in, and a heavily lagged run would be scored
    against a different failure criterion than a clean one.

    **The score is computed on truth.** ``ep_true`` is recorded here from the
    simulator's own state and written into the npz at ``stop``, where
    sweep_metrics.compute_metrics prefers it over the controller's ``ep``. A
    lagged controller logs the error it *believes* it has, which shrinks as the
    lag grows; scoring that would make delay look like an improvement.

    This class deliberately does not subclass ``Backend``: it is constructed
    outside Isaac in the unit tests, and the base class pulls in the extension.
    The vehicle assigns backends duck-typed, so the interface is what matters.
    """

    def __init__(self, inner, results_file, target_position, monitor,
                 wind: WindField = None, lag: SensorLag = None,
                 drag: np.ndarray = None, log=None):
        self.inner = inner
        self.results_file = results_file
        self.target = np.asarray(target_position, dtype=float)
        self.monitor = monitor
        self.wind = wind if wind is not None else WindField("none")
        self.lag = lag
        self.drag = drag
        self._log = log

        self._vehicle = None
        self.total_time = 0.0
        self._stopped = False
        self.position = np.zeros(3)
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # Truth traces, recorded independently of anything the controller logs.
        self.true_time = []
        self.true_position = []
        self.true_error = []
        self.wind_trace = []

    # The sweep loop and sweep_metrics both read these; FlightMonitor owns the
    # state behind them.
    @property
    def is_oob(self) -> bool:
        return self.monitor.failed

    @property
    def oob_time(self) -> float:
        return self.monitor.oob_time

    @property
    def vehicle(self):
        return self._vehicle

    def initialize(self, vehicle):
        self._vehicle = vehicle
        self.inner.initialize(vehicle)
        if self.drag is None:
            self.drag = drag_coefficients(vehicle)

    def update_state(self, state):
        # Truth first, and unconditionally: the monitor and the truth log read
        # this even on steps where the controller is no longer being driven.
        self.position = np.asarray(state.position, dtype=float).copy()
        self.attitude = np.asarray(state.attitude, dtype=float).copy()
        if self.is_oob:
            return
        self.inner.update_state(self.lag.apply(state) if self.lag is not None
                                else state)

    def update(self, dt: float):
        if self.is_oob:
            return np.zeros(4)
        self.total_time += dt

        # Wind advances on the true clock and is applied before the controller
        # runs, so the force lands in the same physics step as the thrust that
        # has to reject it.
        wind_velocity = self.wind.update(dt)
        if self.wind.active and self._vehicle is not None:
            force = wind_force_body(wind_velocity, self.attitude, self.drag)
            self._vehicle.apply_force(list(force), body_part="/body")

        thrusts = self.inner.update(dt)

        self.true_time.append(self.total_time)
        self.true_position.append(self.position.copy())
        self.true_error.append(self.target - self.position)
        self.wind_trace.append(np.asarray(wind_velocity, dtype=float).copy())
        return thrusts

    def check_oob(self, bounds) -> bool:
        """Advance the flight-phase monitor on TRUE position. True once over."""
        return self.monitor.update(self.total_time, self.position, bounds)

    def reset(self):
        self.total_time = 0.0
        self._stopped = False
        self.position = np.zeros(3)
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])
        self.true_time = []
        self.true_position = []
        self.true_error = []
        self.wind_trace = []
        self.monitor.reset()
        self.wind.reset()
        if self.lag is not None:
            self.lag.reset()
        self.inner.reset()

    def start(self):
        self.monitor.reset()
        self.wind.reset()
        if self.lag is not None:
            self.lag.reset()
        self.inner.start()

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        self.inner.stop()

        import os
        if not os.path.isfile(self.results_file):
            return
        try:
            data = dict(np.load(self.results_file, allow_pickle=True))
            data["oob_time"] = np.array(self.monitor.oob_time)
            data["takeoff_time"] = np.array(self.monitor.takeoff_time)
            data["min_altitude"] = np.array(self.monitor.min_altitude)
            data["max_excursion"] = np.array(self.monitor.max_excursion)
            data["failure"] = np.array(self.monitor.failure)

            # The truth traces. Length-matched to the controller's own log:
            # both are appended once per update() call, but a controller that
            # returns early before its first state (received_first_state) logs
            # nothing that step while this one still records truth. Trimming to
            # the shorter of the two keeps every column in the npz aligned,
            # which compute_metrics requires (it checks ep.shape[0] == len(t)).
            n_ctrl = len(np.atleast_1d(data.get("time", [])))
            n_true = len(self.true_time)
            n = min(n_ctrl, n_true) if n_ctrl else n_true
            if n > 0:
                data["ep_true"] = np.vstack(self.true_error[-n:])
                data["p_true"] = np.vstack(self.true_position[-n:])
                data["wind"] = np.vstack(self.wind_trace[-n:])
            np.savez(self.results_file, **data)
        except Exception as exc:
            if self._log is not None:
                self._log(f"[robustness] could not append truth traces: {exc}")

    def update_sensor(self, sensor_type, data):
        if not self.is_oob:
            self.inner.update_sensor(sensor_type, data)

    def update_graphical_sensor(self, sensor_type, data):
        if not self.is_oob:
            self.inner.update_graphical_sensor(sensor_type, data)

    def input_reference(self):
        return [0.0, 0.0, 0.0, 0.0] if self.is_oob else self.inner.input_reference()
