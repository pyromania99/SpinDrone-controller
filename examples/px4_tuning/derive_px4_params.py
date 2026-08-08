#!/usr/bin/env python3
"""
derive_px4_params.py — read the vehicle's true physical parameters out of Pegasus and
emit the PX4 airframe values that match them.

Why this exists
───────────────
The Isaac Sim controllers do not have a model of the vehicle, they READ one:
eigen_controller.py:164 calls _sync_physical_params_from_vehicle(), and
multirotor.py:199-215 builds its allocation matrix at runtime from the actual USD rotor
positions and thrust curve. So the prototype's model error is structurally zero.

PX4 cannot read any of it. MC_EIG_IXX/IYY/IZZ, MC_EIG_TRQ_MAX and CA_ROTOR*_CT/PX/PY are
all hand-transcribed into the airframe file, and the feedback linearization's gain error
is the product of every one of them. That difference - the prototype measures the plant,
the port is told about it - is the general reason a law that flies in Pegasus can fail
under PX4 while both "implement the same controller".

This closes that gap by deriving each PX4 value from the same source the prototype reads,
and diffing it against what the airframe currently says.

WHAT IS AND IS NOT SETTLED HERE
This script gives the values PX4 *should* hold if Pegasus is the truth. It does NOT prove
that a mismatch is what destabilises the vehicle above ~7 rad/s - that needs the flight
test described at the bottom of the output. Deriving a number and demonstrating it is the
cause are different claims and this only makes the first.

Spin does not affect any quantity read here (geometry, mass, inertia and the thrust curve
are all static), so the default motor_moment of 0 simply keeps the vehicle still while it
is measured. The empirical cross-check - run_step_id.py - DOES need motor_moment 0, so
that a world-frame step excites one body axis instead of rotating through roll and pitch
at the spin rate.

Usage
─────
  isaac_run examples/px4_tuning/derive_px4_params.py
  isaac_run examples/px4_tuning/derive_px4_params.py --input-scaling 1100
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Derive PX4 airframe params from Pegasus truth")
    p.add_argument("--input-scaling", type=float, default=1100.0,
                   help="omega = input_scaling * u, as single_vehicle.py sets it. CT is "
                        "rotor_constant * input_scaling^2, so this is not cosmetic.")
    p.add_argument("--motor-moment", type=float, default=0.0,
                   help="Only affects the derived CA_ROTOR*_KM. Nothing else read here "
                        "depends on spin.")
    return p.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True, "hide_ui": True,
                                "disable_viewport_updates": True,
                                "multi_gpu": False, "max_gpu_count": 1, "active_gpu": 0})

import numpy as np                                # noqa: E402
from isaacsim.core.api import World               # noqa: E402
from scipy.spatial.transform import Rotation      # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _cand = _parent / "extensions" / "pegasus.simulator"
    if _cand.exists():
        sys.path.insert(0, str(_cand))
        break
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS                  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface      # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.logic.thrusters.quadratic_thrust_curve import QuadraticThrustCurve  # noqa: E402
import vehicle_params                                                                  # noqa: E402

GRAVITY = 9.81

# What 10016_none_iris holds today, for the diff.
CURRENT = {
    "MC_EIG_IXX": 0.03477, "MC_EIG_IYY": 0.03185, "MC_EIG_IZZ": 0.06359,
    "MC_EIG_TRQ_MAX": 3.843, "MC_EIG_ALPHA": 0.0, "MC_EIG_BETA": 0.6,
    "MPC_THR_HOVER": 0.58,
    "CA_ROTOR0_PX": 0.1376, "CA_ROTOR1_PX": -0.1250,
    "CA_ROTOR2_PX": 0.1383, "CA_ROTOR3_PX": -0.1245,
    "CA_ROTOR0_PY": 0.2067, "CA_ROTOR1_PY": -0.2187,
    "CA_ROTOR2_PY": -0.2032, "CA_ROTOR3_PY": 0.2220,
    "CA_ROTOR0_CT": 10.344, "CA_ROTOR1_CT": 10.344,
    "CA_ROTOR2_CT": 10.344, "CA_ROTOR3_CT": 10.344,
    "CA_ROTOR0_KM": 0.05, "CA_ROTOR1_KM": 0.05,
    "CA_ROTOR2_KM": 0.05, "CA_ROTOR3_KM": 0.05,
}


def log(msg=""):
    print(f"DERIVE {msg}", flush=True)


def px4_torque_scale(ct, km, px, py):
    """MC_EIG_TRQ_MAX: the physical N.m that one unit of normalized torque produces.

    Replicates PX4 exactly rather than approximating it.
    ActuatorEffectivenessRotors.cpp builds, with axis = (0,0,-1) in FRD:
        moment = ct * (position x axis) - ct * km * axis   ->  (-ct*py, ct*px, ct*km)
    and ControlAllocationPseudoInverse::updateControlAllocationMatrixScale() takes
        scale = max over {roll,pitch} of sqrt(||pinv(B).col||^2 / (n_nonzero/2))
    applying the SAME scale to both axes. normalizeControlAllocationMatrix() then divides
    the mix by it, so B @ (pinv(B)/scale) @ tau_norm = tau_norm/scale, i.e. K = 1/scale.

    Note the yaw row is collinear with the thrust row whenever all four KM share a sign
    (rot_dir = -1,-1,-1,-1 here), making B rank 3. Verified numerically: the derived scale
    is 3.8430 for every KM from 0 to 0.1, so KM does not reach this number.
    """
    B = np.vstack([-ct * py, ct * px, ct * km, -ct])
    mix = np.linalg.pinv(B)
    scales = []
    for axis in (0, 1):
        col = mix[:, axis]
        n = int((np.abs(col) > 1e-3).sum())
        scales.append(np.sqrt((col ** 2).sum() / (n / 2.0)) if n else 1.0)
    return 1.0 / max(scales), np.linalg.matrix_rank(B)


def main():
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

    cfg = MultirotorConfig()
    cfg.backends = []
    cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": ARGS.motor_moment})
    vehicle = Multirotor("/World/derive_subject", ROBOTS["Iris"], 0, [0.0, 0.0, 3.0],
                         Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
                         config=cfg)
    world.reset()
    for _ in range(10):
        world.step(render=False)

    props = vehicle_params.composite_mass_properties(vehicle)
    if props is None:
        log("FAILED: could not read mass properties from the physics scene")
        return 1

    positions = vehicle_params._relative_positions(vehicle, vehicle_params.ROTOR_PARTS)
    thrusters = vehicle._thrusters
    rotor_constant = np.asarray(thrusters._rotor_constant, dtype=float)
    rolling = np.asarray(thrusters._rolling_moment_coefficient, dtype=float)
    damping = np.asarray(getattr(vehicle, "_rotational_damping", [0.02, 0.02, 0.035]), float)

    # Geometry: Pegasus is ENU/FLU, PX4 is NED/FRD. x is forward in both; y flips.
    P = np.array([positions[part] for part in vehicle_params.ROTOR_PARTS])
    ca_px, ca_py = P[:, 0], -P[:, 1]

    # CT is the coefficient of u^2, not of omega^2: single_vehicle.py drives the rotors
    # with omega = input_scaling * u, so F = rotor_constant * (input_scaling*u)^2.
    ca_ct = rotor_constant * ARGS.input_scaling ** 2
    # PX4's KM is a RATIO (moment_z = ct*km per unit thrust); Pegasus stores an absolute
    # coefficient against omega^2. Both divide out omega^2, so km_px4 = rolling/rotor_constant.
    ca_km = rolling / rotor_constant

    trq_max, rank = px4_torque_scale(ca_ct, ca_km, ca_px, ca_py)

    hover_n = props["mass"] * GRAVITY
    # Thrust is QUADRATIC in the normalized command: 4*CT*u^2 = m*g.
    thr_hover = float(np.sqrt(hover_n / (4.0 * ca_ct.mean())))

    derived = {
        "MC_EIG_IXX": props["Ixx"], "MC_EIG_IYY": props["Iyy"], "MC_EIG_IZZ": props["Izz"],
        "MC_EIG_TRQ_MAX": trq_max,
        # MC_EIG_ALPHA/BETA are the rate damping the linearization CANCELS. The simulator
        # applies -rotational_damping*omega (multirotor.py:145), so the value that cancels
        # it exactly is D/I. Reported, not recommended: over-cancelling ADDS energy, and
        # leaving it at 0 merely keeps damping the design did not ask for.
        "MC_EIG_ALPHA": float(damping[0] / props["Ixx"]),
        "MC_EIG_BETA": float(damping[2] / props["Izz"]),
        "MPC_THR_HOVER": thr_hover,
    }
    for i in range(4):
        derived[f"CA_ROTOR{i}_PX"] = float(ca_px[i])
        derived[f"CA_ROTOR{i}_PY"] = float(ca_py[i])
        derived[f"CA_ROTOR{i}_CT"] = float(ca_ct[i])
        derived[f"CA_ROTOR{i}_KM"] = float(ca_km[i])

    log("=" * 78)
    log("GROUND TRUTH read from the running simulation")
    log("=" * 78)
    log(vehicle_params.describe(props))
    log(f"  link masses      {props['links']}")
    log(f"  composite COM    {np.array2string(props['com'], precision=5)}")
    log(f"  rotor_constant   {rotor_constant}")
    log(f"  rolling coeff    {rolling}")
    log(f"  body damping     {damping}  N.m/(rad/s)   (multirotor.py:145)")
    log(f"  input_scaling    {ARGS.input_scaling}  ->  CT = rotor_constant * scaling^2")
    log(f"  effectiveness B rank {rank} (3 means yaw is collinear with thrust: all rotors "
        f"turn the same way)")

    log("")
    log("=" * 78)
    log("DERIVED PX4 VALUES vs the airframe as it stands")
    log("=" * 78)
    log(f"{'param':18} {'current':>12} {'derived':>12} {'ratio':>8}   note")
    worst = []
    for key, value in derived.items():
        current = CURRENT.get(key)
        if current is None:
            continue
        ratio = value / current if abs(current) > 1e-12 else float("inf")
        flag = ""
        if abs(current) > 1e-12 and abs(ratio - 1.0) > 0.02:
            flag = "  <-- MISMATCH"
            worst.append((key, current, value, ratio))
        elif abs(current) <= 1e-12 and abs(value) > 1e-9:
            flag = "  <-- currently zero"
            worst.append((key, current, value, float('nan')))
        log(f"{key:18} {current:12.5f} {value:12.5f} {ratio:8.3f}{flag}")

    log("")
    if not worst:
        log("No mismatch above 2%. The feedback linearization's model is correct, and the")
        log("1.33x gain error measured in flight CANNOT be explained by these parameters.")
        log("That REFUTES the model-error hypothesis - look at the delay/gain margin instead.")
    else:
        log(f"{len(worst)} parameter(s) disagree with the simulation by more than 2%.")
        log("This makes the model-error hypothesis viable. It does NOT confirm it.")

    log("")
    log("=" * 78)
    log("PASTE INTO ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris")
    log("=" * 78)
    for key in ("MC_EIG_IXX", "MC_EIG_IYY", "MC_EIG_IZZ", "MC_EIG_TRQ_MAX", "MPC_THR_HOVER"):
        log(f"param set-default {key} {derived[key]:.5f}")
    for i in range(4):
        for suffix in ("PX", "PY", "CT", "KM"):
            log(f"param set-default CA_ROTOR{i}_{suffix} {derived[f'CA_ROTOR{i}_{suffix}']:.5f}")
    log(f"# MC_EIG_ALPHA {derived['MC_EIG_ALPHA']:.5f} / MC_EIG_BETA {derived['MC_EIG_BETA']:.5f} "
        f"cancel the sim's body damping exactly.")
    log("# Do NOT apply those two without thinking: cancelling damping REMOVES stabilising")
    log("# margin, and over-estimating them adds energy. 0 is the conservative choice.")

    log("")
    log("=" * 78)
    log("WHAT THIS DOES NOT PROVE, AND THE TEST THAT WOULD")
    log("=" * 78)
    log("A parameter mismatch is not a demonstrated cause of the divergence above ~7 rad/s.")
    log("Two experiments settle it, in this order:")
    log("")
    log("1. EMPIRICAL CROSS-CHECK of the torque scale, at motor_moment 0 so a world-frame")
    log("   step excites one body axis:")
    log("     isaac_run examples/px4_flight/single_vehicle.py --keep-log --motor-moment 0")
    log("     python3 examples/px4_tuning/run_step_id.py --step 4 --alt 6")
    log("   If the measured per-axis K_eff matches the derived MC_EIG_TRQ_MAX above, the")
    log("   torque path is correct and any residual gain error is in the inertia.")
    log("")
    log("2. DOSE-RESPONSE, which is what actually tests causation. MC_EIG_TRQ_MAX scales")
    log("   the linearization gain g exactly (g ~ 1/TRQ_MAX) and applies live. At FIXED")
    log("   spin, sweep it and record the growth rate of the 10.8 rad/s mode; repeat at")
    log("   motor_moment 0 as the control.")
    log("     PREDICTED if model error is the destabiliser: growth depends strongly on")
    log("       TRQ_MAX at r=7.9 and weakly at r=0, with a minimum near the derived value.")
    log("     PREDICTED if it is only loop gain: same dependence at both spin rates.")
    log("     REFUTED if growth is flat against TRQ_MAX.")
    return 0


if __name__ == "__main__":
    try:
        code = main()
    finally:
        simulation_app.close()
    sys.exit(code)
