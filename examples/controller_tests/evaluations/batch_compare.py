#!/usr/bin/env python
"""
Batch controller comparison across multiple motor_moment values.

Spawns one drone per controller within each motor_moment sweep iteration,
runs headless, and saves per-combination telemetry.  Drones that leave
their bounding box are deleted mid-sim and their data is truncated.

Usage (via isaac_run):
    isaac_run examples/controller_tests/evaluations/batch_compare.py \
        --motor-moments 1.0e-7 2.7e-7 4.0e-7 \
        --controllers pd eigen \
        --duration 15 \
        --headless

Results land in:
    examples/controller_tests/results/mm_<value>/<ctrl>_statistics.npz
"""

from __future__ import annotations

import argparse
import os
import sys
import math
from pathlib import Path

import carb
from isaacsim import SimulationApp


# ---------------- CLI ---------------- #


def parse_args():
    p = argparse.ArgumentParser(description="Batch controller comparison")
    p.add_argument(
        "--motor-moments",
        nargs="+",
        type=float,
        default=[1e-7, 2e-7, 2.7e-7, 3.5e-7, 4e-7],
    )
    p.add_argument(
        "--controllers",
        nargs="+",
        type=str,
        default=None,
        help="controller keys to test (default: all registered)",
    )
    p.add_argument("--duration", type=float, default=15.0, help="sim seconds per run")
    p.add_argument("--spacing", type=float, default=5.0, help="metres between drones on X axis")
    p.add_argument("--headless", action="store_true", default=True)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    p.add_argument(
        "--bounds",
        nargs=6,
        type=float,
        default=[-10.0, 10.0, -10.0, 10.0, 0.0, 15.0],
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="OOB bounding box [xmin xmax ymin ymax zmin zmax]",
    )
    p.add_argument(
        "--goal-mode",
        type=str,
        choices=["spawn_xy", "random_xy_2m"],
        default="spawn_xy",
        help="Goal generation mode: spawn_xy (same x,y as spawn) or random_xy_2m (2m random XY offset from spawn)",
    )
    p.add_argument(
        "--goal-altitude",
        type=float,
        default=1.5,
        help="Target altitude used for hover goals [m]",
    )
    p.add_argument(
        "--goal-radius",
        type=float,
        default=2.0,
        help="XY radius [m] for random_xy_2m mode",
    )
    p.add_argument(
        "--goal-seed",
        type=int,
        default=7,
        help="Random seed for goal randomization",
    )
    return p.parse_args()


ARGS = parse_args()

simulation_app = SimulationApp({"headless": ARGS.headless})


# ---------------- Imports ---------------- #

import omni.timeline
import numpy as np

from isaacsim.core.api import World
from scipy.spatial.transform import Rotation

# Ensure the Pegasus extension is on sys.path
for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.state import State

# controller imports
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from cascaded_pd_controller import CascadedPDController
from eigen_controller import EigenController


# ---------------- Controller registry ---------------- #


CONTROLLER_REGISTRY = {
    "pd": lambda rf, target_pos: CascadedPDController(
        target_position=target_pos,
        results_file=rf,
    ),
    "eigen": lambda rf, target_pos: EigenController(
        target_position=target_pos,
        results_file=rf,
    ),
}


def build_target_position(spawn_position: list[float], rng: np.random.Generator) -> tuple[float, float, float]:
    if ARGS.goal_mode == "spawn_xy":
        return (spawn_position[0], spawn_position[1], float(ARGS.goal_altitude))

    theta = float(rng.uniform(0.0, 2.0 * math.pi))
    r = float(ARGS.goal_radius)
    return (
        spawn_position[0] + r * math.cos(theta),
        spawn_position[1] + r * math.sin(theta),
        float(ARGS.goal_altitude),
    )


# ---------------- Telemetry backend ---------------- #


class BoundedTelemetryBackend(Backend):
    def __init__(self, wrapped: Backend, results_file: str):
        super().__init__(config=None)
        self.wrapped = wrapped
        self.results_file = results_file

        self.total_time = 0.0
        self.oob_time = float("nan")
        self.is_oob = False
        self._stopped = False

        self.current_position = np.zeros(3)

    def initialize(self, vehicle):
        super().initialize(vehicle)
        self.wrapped.initialize(vehicle)

    def start(self):
        self.total_time = 0.0
        self.oob_time = float("nan")
        self.is_oob = False
        self._stopped = False
        self.wrapped.start()

    def stop(self):
        if self._stopped:
            return

        self._stopped = True
        self.wrapped.stop()

        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file))
                data["oob_time"] = np.array(self.oob_time)
                np.savez(self.results_file, **data)
            except Exception as e:
                carb.log_warn(f"Could not append oob_time: {e}")

    def reset(self):
        self.wrapped.reset()

    def update_sensor(self, sensor_type: str, data):
        if not self.is_oob:
            self.wrapped.update_sensor(sensor_type, data)

    def update_graphical_sensor(self, sensor_type: str, data):
        if not self.is_oob:
            self.wrapped.update_graphical_sensor(sensor_type, data)

    def update_state(self, state: State):
        self.current_position = state.position.copy()

        if not self.is_oob:
            self.wrapped.update_state(state)

    def input_reference(self):
        if self.is_oob:
            return [0.0, 0.0, 0.0, 0.0]

        return self.wrapped.input_reference()

    def update(self, dt: float):
        if self.is_oob:
            return

        self.total_time += dt
        self.wrapped.update(dt)

    def check_oob(self, bounds) -> bool:
        if self.is_oob:
            return False

        x, y, z = self.current_position

        if not (
            bounds[0] < x < bounds[1]
            and bounds[2] < y < bounds[3]
            and bounds[4] < z < bounds[5]
        ):
            self.is_oob = True
            self.oob_time = self.total_time

            carb.log_warn(
                f"OOB at {self.current_position} t={self.total_time:.3f}"
            )

            return True

        return False


# ---------------- Helpers ---------------- #


def make_thrust_curve(mm: float) -> QuadraticThrustCurve:
    return QuadraticThrustCurve(config={"motor_moment": mm})


def results_dir(base: Path, mm: float) -> Path:
    d = base / f"mm_{mm:.1e}"
    d.mkdir(parents=True, exist_ok=True)
    return d


# ---------------- Main ---------------- #


def main():
    timeline = omni.timeline.get_timeline_interface()
    world = None
    render_sim = not ARGS.headless
    rng = np.random.default_rng(ARGS.goal_seed)

    base_dir = Path(__file__).resolve().parent.parent
    results_base = base_dir / "results"

    controller_keys = ARGS.controllers or list(CONTROLLER_REGISTRY.keys())
    for key in controller_keys:
        if key not in CONTROLLER_REGISTRY:
            valid = ", ".join(CONTROLLER_REGISTRY.keys())
            raise ValueError(f"Unknown controller '{key}'. Valid: {valid}")

    # -------- create world once -------- #

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world

    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

    for _ in range(20):
        world.step(render=render_sim)

    # -------- spawn drones once -------- #

    drones: list[tuple[str, Multirotor, BoundedTelemetryBackend, list[float]]] = []

    for idx, ctrl in enumerate(controller_keys):
        rf = str(results_base / f"{ctrl}_tmp.npz")
        spawn_x = 2.3 + idx * ARGS.spacing
        spawn_position = [spawn_x, -1.5, 0.07]
        target_position = build_target_position(spawn_position, rng)

        backend = BoundedTelemetryBackend(
            CONTROLLER_REGISTRY[ctrl](rf, target_position),
            rf,
        )

        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = make_thrust_curve(ARGS.motor_moments[0])

        vehicle = Multirotor(
            f"/World/quadrotor_{ctrl}",
            ROBOTS["Iris"],
            idx,
            spawn_position,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=cfg,
        )

        carb.log_warn(
            f"{ctrl}: spawn={spawn_position} target={[round(v, 3) for v in target_position]}"
        )

        drones.append((ctrl, vehicle, backend, spawn_position))

    world.reset()

    for _ in range(10):
        world.step(render=render_sim)

    # -------- sweep -------- #

    bounds = ARGS.bounds

    for mm in ARGS.motor_moments:
        carb.log_warn(f"\n==== motor_moment {mm:.1e} ====")

        rdir = results_dir(results_base, mm)

        # update thrust model and reset state
        for ctrl, vehicle, backend, spawn_position in drones:
            # Multirotor stores the thrust model in its private _thrusters field
            # (initialized from MultirotorConfig.thrust_curve). Swap it here to
            # sweep motor_moment without respawning the articulation.
            vehicle._thrusters = make_thrust_curve(mm)

            vehicle.set_world_pose(
                position=spawn_position,
                orientation=[0.0, 0.0, 0.0, 1.0],
            )

            vehicle.set_linear_velocity([0.0, 0.0, 0.0])
            vehicle.set_angular_velocity([0.0, 0.0, 0.0])

            backend.reset()
            backend.start()

        timeline.play()

        sim_time = 0.0
        dt = 1.0 / 60.0

        while simulation_app.is_running() and sim_time < ARGS.duration:
            world.step(render=render_sim)
            sim_time += dt

            all_oob = True

            for ctrl, vehicle, backend, _ in drones:
                if backend.is_oob:
                    continue

                all_oob = False

                if backend.check_oob(bounds):
                    backend.stop()

            if all_oob:
                break

        timeline.stop()

        for ctrl, vehicle, backend, _ in drones:
            if not backend._stopped:
                backend.stop()

            src = backend.results_file
            dst = rdir / f"{ctrl}_statistics.npz"

            if os.path.isfile(src):
                try:
                    os.rename(src, str(dst))
                except Exception as e:
                    carb.log_warn(f"Could not move results {src} -> {dst}: {e}")

        carb.log_warn(f"results saved to {rdir}")

    carb.log_warn("\n=== batch complete ===")

    # Explicit teardown helps avoid PhysX tensor callbacks trying to create
    # simulation views after the USD stage is already closing.
    try:
        timeline.stop()
    except Exception:
        pass

    if world is not None:
        try:
            world.stop()
        except Exception:
            pass

        try:
            world.clear_all_callbacks()
        except Exception:
            pass

        try:
            world.clear()
        except Exception:
            pass

    simulation_app.close()


if __name__ == "__main__":
    main()
