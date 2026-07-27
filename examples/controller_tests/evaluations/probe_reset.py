#!/usr/bin/env python3
"""
probe_reset.py — determine which API actually repositions a drone between passes.

The grid sweeps re-use one drone per parameter slot across many passes and
reset each with::

    vehicle.set_world_pose(position=spawn_pos, orientation=[0,0,0,1])
    vehicle.set_linear_velocity([0,0,0])
    vehicle.set_angular_velocity([0,0,0])

Telemetry says that does not work: on pass 2 every drone starts exactly where
pass 1 left it, and a drone that crashed in pass 1 begins pass 2 lying on the
floor. Pegasus drives vehicles through the PhysX dynamic-control interface, so
writing the USD prim transform is likely being overridden for an active rigid
body.

Rather than guess, this drops two drones, lets them fall well away from spawn,
then tries each candidate reset and reports where the drone actually starts the
next pass. Run it with isaac_run; it prints a table and exits.
"""
from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "hide_ui": True,
                                "disable_viewport_updates": True,
                                "multi_gpu": False, "max_gpu_count": 1,
                                "active_gpu": 0})

import carb                             # noqa: E402
import numpy as np                      # noqa: E402
import omni.timeline                    # noqa: E402
from isaacsim.core.api import World     # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _cand = _parent / "extensions" / "pegasus.simulator"
    if _cand.exists():
        sys.path.insert(0, str(_cand))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS      # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402

SPAWN_Z = 1.5
N = 2


def drone_z(vehicle):
    return float(vehicle._state.position[2])


def settle(world, steps=30):
    for _ in range(steps):
        world.step(render=False)


def main():
    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    settle(world, 20)

    spawns = [[-3.0 + 6.0 * i, 0.0, SPAWN_Z] for i in range(N)]
    vehicles = []
    for i, sp in enumerate(spawns):
        cfg = MultirotorConfig()
        cfg.backends = []          # no controller: the drone simply falls
        v = Multirotor(f"/World/probe_{i}", ROBOTS["Iris"], i, sp,
                       Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
                       config=cfg)
        vehicles.append(v)

    world.reset()
    settle(world, 10)

    carb.log_warn("PROBE " + "=" * 60)
    carb.log_warn(" RESET PROBE — where does the drone start the NEXT pass?")
    carb.log_warn("PROBE " + "=" * 60)
    carb.log_warn(f"PROBE spawn altitude = {SPAWN_Z} m; with no backend the drones fall to the floor")

    methods = [
        ("set_world_pose only (what the sweeps do today)", "pose"),
        ("world.reset() only", "reset"),
        ("world.reset() then set_world_pose", "reset+pose"),
    ]

    for label, mode in methods:
        # ── pass 1: let them fall away from spawn ────────────────────────────
        timeline.play()
        settle(world, 240)
        fell_to = [drone_z(v) for v in vehicles]
        timeline.stop()
        settle(world, 5)

        # ── apply the candidate reset ────────────────────────────────────────
        if mode in ("reset", "reset+pose"):
            world.reset()
        if mode in ("pose", "reset+pose"):
            for v, sp in zip(vehicles, spawns):
                v.set_world_pose(position=sp, orientation=[0., 0., 0., 1.])
                v.set_linear_velocity([0., 0., 0.])
                v.set_angular_velocity([0., 0., 0.])
        settle(world, 5)

        # ── pass 2: read the altitude at the very first simulated step ──────
        timeline.play()
        world.step(render=False)
        start_z = [drone_z(v) for v in vehicles]
        timeline.stop()
        settle(world, 5)

        ok = all(abs(z - SPAWN_Z) < 0.20 for z in start_z)
        carb.log_warn(f"PROBE {label}")
        carb.log_warn(f"PROBE   fell to      : {[f'{z:.3f}' for z in fell_to]}")
        carb.log_warn(f"PROBE   pass-2 start : {[f'{z:.3f}' for z in start_z]} (want ~{SPAWN_Z})")
        carb.log_warn(f"PROBE   RESTORES SPAWN: {'YES' if ok else 'NO'}")

    carb.log_warn("PROBE " + "=" * 60)
    for fn in ("stop", "clear_all_callbacks", "clear"):
        try:
            getattr(world, fn)()
        except Exception:
            pass
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        carb.log_warn("PROBE FAILED:\n" + traceback.format_exc())
        simulation_app.close()
        raise
