#!/usr/bin/env python3
"""
probe_geometry.py — read the vehicle's ACTUAL rotor geometry and check PX4 agrees.

Why this exists
───────────────
The eigen controller's roll axis delivers the torque PX4's model predicts (measured
K_eff 4.383 against an analytic 4.4325) while its pitch axis delivers 0.45x of it.
That asymmetry breaks the eigen law specifically, because MC_EIG_B cross-couples roll
and pitch on the assumption that they are equally effective.

Trying to identify the rotor arms by regression from flight logs does not work:
actuator_motors is logged at 100 ms (logged_topics.cpp:218), which is 7 samples per
cycle against the ~1.4 Hz mode, and the fit returns sign-flipped nonsense with a
flattering R^2. So read the geometry directly instead of inferring it.

``get_relative_body_poses`` is the same call Multirotor.force_and_torques_to_velocities
uses to build its own allocation matrix (multirotor.py:199), so what this prints is
exactly the geometry Pegasus flies - not an authored USD value that may differ from
what PhysX simulates.

The comparison target is the effectiveness that ActuatorEffectivenessRotors builds
from CA_ROTOR*_PX/PY (ActuatorEffectivenessRotors.cpp:199):

    moment = ct * (position x axis) - ct * km * axis,   axis = (0,0,-1) in FRD

so with position = (px, py, 0):  roll coeff = -py,  pitch coeff = +px.

Frames: Pegasus is ENU/FLU, PX4 is NED/FRD. x is forward in both; y and z flip. The
comparison below reports the raw FLU values and the FRD equivalent so a sign
convention cannot be mistaken for a geometry error.

Run with isaac_run. Prints and exits; writes nothing.
"""
from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "hide_ui": True,
                                "disable_viewport_updates": True,
                                "multi_gpu": False, "max_gpu_count": 1,
                                "active_gpu": 0})

import numpy as np                              # noqa: E402
import omni.timeline                            # noqa: E402
from isaacsim.core.api import World             # noqa: E402
from scipy.spatial.transform import Rotation    # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _cand = _parent / "extensions" / "pegasus.simulator"
    if _cand.exists():
        sys.path.insert(0, str(_cand))
        break
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS            # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
import vehicle_params                                                            # noqa: E402

# From ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris
CA_PX = [0.1515, -0.1515, 0.1515, -0.1515]
CA_PY = [0.245, -0.1875, -0.245, 0.1875]
CA_CT = 10.344


def log(m=""):
    print(f"GEOM {m}", flush=True)


def main():
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

    cfg = MultirotorConfig()
    cfg.backends = []
    v = Multirotor("/World/geom_subject", ROBOTS["Iris"], 0, [0.0, 0.0, 3.0],
                   Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(), config=cfg)
    world.reset()
    for _ in range(10):
        world.step(render=False)

    pos = vehicle_params._relative_positions(v, vehicle_params.ROTOR_PARTS)
    thr = v._thrusters
    rc = np.asarray(thr._rotor_constant, dtype=float)
    vmax = np.asarray(thr.max_rotor_velocity, dtype=float)

    log("=" * 72)
    log("ACTUAL rotor geometry (get_relative_body_poses - what the mixer uses)")
    log("=" * 72)
    log(f"{'rotor':8} {'x (fwd)':>10} {'y (left)':>10} {'z':>9}   [Pegasus ENU/FLU]")
    P = []
    for i, part in enumerate(vehicle_params.ROTOR_PARTS):
        p = pos.get(part)
        if p is None:
            log(f"  rotor{i}: UNREADABLE")
            continue
        P.append(p)
        log(f"  rotor{i:<2} {p[0]:10.5f} {p[1]:10.5f} {p[2]:9.5f}")
    P = np.array(P)

    log("")
    log("Same, converted to PX4 FRD (y and z negated):")
    log(f"{'rotor':8} {'PX (fwd)':>10} {'PY (right)':>11}      {'CA_ROTOR assumes':>18}")
    for i in range(len(P)):
        log(f"  rotor{i:<2} {P[i,0]:10.5f} {-P[i,1]:11.5f}      "
            f"PX={CA_PX[i]:+.4f} PY={CA_PY[i]:+.4f}")

    log("")
    log("=" * 72)
    log("Per-axis effectiveness: sum |arm| * CT, actual vs what PX4 believes")
    log("=" * 72)
    act_roll = float(np.abs(-P[:, 1]).sum()) * CA_CT
    act_pitch = float(np.abs(P[:, 0]).sum()) * CA_CT
    px4_roll = float(np.abs(CA_PY).sum()) * CA_CT
    px4_pitch = float(np.abs(CA_PX).sum()) * CA_CT
    log(f"  roll  : actual {act_roll:8.4f}   PX4 {px4_roll:8.4f}   ratio {act_roll/px4_roll:6.3f}")
    log(f"  pitch : actual {act_pitch:8.4f}   PX4 {px4_pitch:8.4f}   ratio {act_pitch/px4_pitch:6.3f}")
    log("")
    log(f"  actual roll/pitch asymmetry : {act_roll/act_pitch:6.3f}")
    log(f"  PX4    roll/pitch asymmetry : {px4_roll/px4_pitch:6.3f}")
    log("")
    log("  measured in flight (K_eff roll / K_eff pitch) : ~2.2")
    log("  if the actual ratio above is ~2.2, the geometry explains the asymmetry;")
    log("  if it is ~1.4 like PX4's, the cause is elsewhere.")

    log("")
    log(f"rotor_constant {rc}   max_rotor_velocity {vmax}")
    log(f"per-rotor ceiling {rc[0]*vmax[0]**2:.4f} N   (CA_ROTOR*_CT = {CA_CT})")
    log("")
    log("probe complete")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
