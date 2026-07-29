#!/usr/bin/env python
"""
| File: vehicle_params.py
| Description: The vehicle's real mass and inertia, as one shared computation.

Why this module exists
──────────────────────
Every controller here models the multirotor as a single rigid body and needs
two numbers from it: the total mass, for the ``m·g`` hover feedforward, and the
diagonal inertia, for the torque channel. All three were getting those numbers
differently, and all three were getting them wrong.

Measured on the Iris (probe_robustness.py)::

    USD authors on /body   mass = 1.5 kg,  diagonalInertia = (0.029125,
                                                              0.029125,
                                                              0.055225)
    PhysX per-link masses  body 1.500 + rotor0 0.029236 + rotor1 0.029236
                                      + rotor2 0.030071 + rotor3 0.030071
    TOTAL                  1.618615 kg   → hover 15.879 N, 3.970 N/rotor

and against that:

    EigenController         synced the /body link only        → 1.5 kg   (−7.3 %)
    CascadedPDController    synced the /body link's mass only → 1.5 kg   (−7.3 %)
    LPVMPCController        synced nothing, constructor value → 1.2 kg  (−25.9 %)

The four rotors are 0.1186 kg of vehicle that no hover term was paying for. The
inertia error is worse in relative terms: the rotors sit ~0.25 m out on the
arms, so their parallel-axis contribution is a substantial fraction of the body
link's own moment, and the ``/body`` value alone understates the real roll,
pitch and yaw inertia.

Why it matters, and where it does not
─────────────────────────────────────
Within one controller the error was a constant: every cell of every grid sweep
carried the identical −7.3 % (or −25.9 %) bias, so the *ranking* of gains those
sweeps produced is unaffected. They found the best gains for a controller that
believes it weighs 1.5 kg, which is a real operating point — just not the
intended one.

Across controllers it is not constant, and that is the problem. MPC competed
with a −25.9 % hover feedforward against Eigen and PD's −7.3 %, so part of any
measured difference between them was the modelling error rather than the
control law. It also leaves the robustness study with no zero point: an
"inertia mismatch" axis has to be a perturbation *away from a correct model*,
and γ = 1.0 was not that.

The rotor spin exception
────────────────────────
The rotors are separate articulation links on revolute joints about their own
Z. For roll and pitch that joint is rigid, so a rotor contributes both its own
moment and its parallel-axis term. About yaw it is free — the joint is driven
to a velocity for the propeller visual — so the rotor's *own* Izz is decoupled
from the airframe's yaw inertia and only the parallel-axis term m·(x² + y²)
belongs there. The distinction is small on this vehicle (a rotor's own Izz is
~8 % of its parallel-axis term, ~0.3 % of the total) but it costs nothing to
get right, and on an airframe with heavier props it would not stay small.

Failure behaviour
─────────────────
``composite_mass_properties`` returns None rather than a guess when the physics
interface cannot be read. Callers keep whatever model they already had and log
it — a controller silently reverting to a default is the failure mode this
module was written to end, so it must not be reintroduced here.
"""
from __future__ import annotations

import numpy as np

# Link names composing the vehicle, relative to its stage prefix. The rotor
# count matches QuadraticThrustCurve's default of four.
BODY_PART: str = "/body"
ROTOR_PARTS: tuple = tuple(f"/rotor{i}" for i in range(4))


def _rigid_body_properties(vehicle, part: str):
    """(mass, moment, com_offset) for one link, or None if unreadable."""
    try:
        dc = vehicle.get_dc_interface()
        handle = dc.get_rigid_body(vehicle._stage_prefix + part)
        if handle is None:
            return None
        props = dc.get_rigid_body_properties(handle)
        if props is None:
            return None
        mass = float(getattr(props, "mass", 0.0))
        if not np.isfinite(mass) or mass <= 0.0:
            return None
        moment = getattr(props, "moment", None)
        moment = (np.asarray([float(v) for v in moment], dtype=float)
                  if moment is not None else np.zeros(3))
        com = getattr(props, "cMassLocalPose", None)
        com = (np.asarray([float(v) for v in com], dtype=float)
               if com is not None else np.zeros(3))
        return mass, moment, com
    except Exception:
        return None


def _relative_positions(vehicle, parts) -> dict:
    """Position of each link's frame relative to the body frame, in body axes.

    Uses the same ``get_relative_body_poses`` call the thrust allocator uses
    (see Multirotor.force_and_torques_to_velocities), so the geometry here and
    the geometry the mixer assumes cannot drift apart.
    """
    out = {}
    try:
        dc = vehicle.get_dc_interface()
        body = dc.get_rigid_body(vehicle._stage_prefix + BODY_PART)
        handles, names = [], []
        for part in parts:
            handle = dc.get_rigid_body(vehicle._stage_prefix + part)
            if handle is not None:
                handles.append(handle)
                names.append(part)
        if body is None or not handles:
            return out
        poses = dc.get_relative_body_poses(body, handles)
        for name, pose in zip(names, poses):
            out[name] = np.asarray([float(pose.p[0]), float(pose.p[1]),
                                    float(pose.p[2])], dtype=float)
    except Exception:
        return out
    return out


def composite_mass_properties(vehicle) -> dict:
    """Total mass and diagonal inertia of the whole articulation.

    Returns ``{"mass", "Ixx", "Iyy", "Izz", "com", "links"}`` in the body frame,
    with the inertia taken about the composite centre of mass. Returns None if
    the body link cannot be read — see the module docstring on why this does
    not fall back to a default.

    Link orientations are assumed aligned with the body frame. That holds on the
    Iris (every rotor joint authors ``localRot0 = identity``) and on any
    conventional multirotor whose rotors point along the body Z.
    """
    body = _rigid_body_properties(vehicle, BODY_PART)
    if body is None:
        return None
    body_mass, body_moment, body_com = body

    links = {BODY_PART: (body_mass, body_moment, body_com)}
    rotor_positions = _relative_positions(vehicle, ROTOR_PARTS)
    for part in ROTOR_PARTS:
        props = _rigid_body_properties(vehicle, part)
        if props is None or part not in rotor_positions:
            continue
        mass, moment, com = props
        # The link's COM in body coordinates: where the link frame sits, plus
        # the COM offset the physics parser recorded within that frame.
        links[part] = (mass, moment, rotor_positions[part] + com)

    total_mass = float(sum(mass for mass, _, _ in links.values()))
    if total_mass <= 0.0:
        return None

    com = np.zeros(3)
    for mass, _, position in links.values():
        com += mass * position
    com /= total_mass

    inertia = np.zeros(3)
    for part, (mass, moment, position) in links.items():
        d = position - com
        # Parallel axis on the diagonal: I_xx += m·(dy² + dz²), and cyclically.
        parallel = mass * np.array([
            d[1] ** 2 + d[2] ** 2,
            d[0] ** 2 + d[2] ** 2,
            d[0] ** 2 + d[1] ** 2,
        ])
        own = np.array(moment, dtype=float)
        if part in ROTOR_PARTS:
            # A rotor's own spin inertia about Z is decoupled from airframe yaw
            # by its revolute joint — only the parallel-axis term belongs in
            # Izz. Roll and pitch are rigid through that joint, so both terms
            # count there. See the module docstring.
            own = np.array([own[0], own[1], 0.0])
        inertia += own + parallel

    return {
        "mass": total_mass,
        "Ixx": float(inertia[0]),
        "Iyy": float(inertia[1]),
        "Izz": float(inertia[2]),
        "com": com,
        "links": {part: float(mass) for part, (mass, _, _) in links.items()},
    }


def describe(props: dict) -> str:
    """One-line summary for a controller's startup log."""
    if not props:
        return "unavailable"
    return (f"m={props['mass']:.6f} kg  "
            f"I=({props['Ixx']:.6f}, {props['Iyy']:.6f}, {props['Izz']:.6f}) kg·m²  "
            f"hover={props['mass'] * 9.81:.4f} N")
