#!/usr/bin/env python3
"""
single_vehicle.py — Fly a single multirotor with PX4 SITL as the flight-stack backend.

Unlike the controller_tests/ sweeps (which fly a custom in-sim Python
controller and never touch PX4), this spawns the vehicle with
PX4MavlinkBackend so Isaac Sim talks HIL mavlink to a real PX4 SITL
process launched from your local PX4-Autopilot checkout. Control the
vehicle externally (QGroundControl, MAVSDK, mavlink shell) once it
connects.

Usage
─────
  isaac_run examples/flight/single_vehicle.py
  isaac_run examples/flight/single_vehicle.py --px4-dir ~/PX4-Autopilot --headless --duration 30

  # keep the ulog, and spin the vehicle at ~9.8 rad/s
  isaac_run examples/flight/single_vehicle.py --keep-log --motor-moment 2.0e-7

Check the ulog reports SYS_AUTOSTART=10016 before drawing any conclusion from it. Every
log left in PX4-Autopilot/build/px4_sitl_default/rootfs/log/ is SYS_AUTOSTART=4001, a
Gazebo x500 - none of them came from this script, and none carry the Pegasus tuning.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import carb
from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Single vehicle flight with PX4 SITL as the backend")
    p.add_argument("--px4-dir", type=str, default=None,
                   help="Path to the PX4-Autopilot checkout (built with 'make px4_sitl_default none'). "
                        "Defaults to px4_dir in extensions/pegasus.simulator/config/configs.yaml.")
    p.add_argument("--px4-airframe", type=str, default="none_iris",
                   help="PX4_SIM_MODEL to launch. Defaults to none_iris - the 'external simulator' "
                        "airframe, which is what Pegasus is. It carries the Pegasus-specific rotor "
                        "and controller tuning; gazebo-classic_iris is left at stock so real Gazebo "
                        "runs are unaffected. Both dispatch to px4-rc.mavlinksim identically.")
    p.add_argument("--vehicle-id", type=int, default=0)
    p.add_argument("--physics-rate", type=float, default=250.0,
                   help="Isaac physics rate in Hz. 250 is the fastest this HIL loop closes - "
                        "PX4 answers one HIL_ACTUATOR_CONTROLS per HIL_SENSOR only up to that "
                        "rate, and the backend's lockstep waits for that answer. See the note "
                        "in PegasusApp.__init__ before raising it.")
    p.add_argument("--no-px4-autolaunch", dest="px4_autolaunch", action="store_false", default=True,
                   help="Do not start PX4 from this script; connect to one you launched yourself. "
                        "Prefer --keep-log over this: launching PX4 by hand out of "
                        "build/px4_sitl_default/rootfs reads the BUILD copy of the airframe "
                        "(rootfs/etc is a symlink into build/), which goes stale until you 'make'. "
                        "Autolaunch passes the source ROMFS directly, so airframe edits apply live.")
    p.add_argument("--keep-log", nargs="?", type=str, const="", default=None,
                   metavar="DIR",
                   help="Keep PX4's ulog. Without this, PX4LaunchTool gives PX4 a temporary "
                        "rootfs that is deleted on exit and takes the log with it - which is why "
                        "no PX4+Pegasus flight has ever left an inspectable log. Bare "
                        "--keep-log uses ./px4_runs/<timestamp>; pass a directory to choose one. "
                        "The ulog lands in <DIR>/log/<date>/<time>.ulg.")
    p.add_argument("--desaturation", choices=("uniform", "collective"),
                   default="uniform",
                   help="What the mixer gives up when a rotor saturates. 'uniform' scales every rotor down together, preserving the thrust/torque ratio and collapsing the collective when one rotor pins; 'collective' holds the mean and shrinks only the moments. 'uniform' is the default and the historical behaviour - every archived sweep and tuned gain set was flown against it, so changing it moves the plant.")
    p.add_argument("--rotor-tau", type=float, default=None,
                   help="Rotor/ESC spin-up time constant [s]. PLACEHOLDER 0.020 - the "
                        "midpoint of the 10-30 ms range FINDINGS section 6 cites for "
                        "hardware, NOT a measured value for this vehicle. Measure it with "
                        "examples/px4_tuning/measure_motor_lag.py and replace this. "
                        "0 restores the old instantaneous rotor model, which omits what is "
                        "probably the largest lag in the real loop.")
    p.add_argument("--motor-moment", type=float, default=0.0,
                   help="Rotor rolling-moment coefficient km [N.m/(rad/s)^2]. All four rotors "
                        "turn the same way (rot_dir -1,-1,-1,-1), so reaction torques add and "
                        "the vehicle spins up in yaw: r_ss ~ 4.9e7*km. 0 (default) is no spin; "
                        "1.0e-7 ~ 4.9 rad/s, 1.6e-7 ~ 7.8, 2.0e-7 ~ 9.8. Previously only "
                        "reachable by editing quadratic_thrust_curve.py's default.")
    p.add_argument("--duration", type=float, default=None,
                   help="Seconds to run before closing (default: run until the window is closed)")
    p.add_argument("--headless", action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()

simulation_app = SimulationApp({"headless": ARGS.headless})

# ── Late imports (must come after SimulationApp is constructed) ─────────────

import omni.timeline
from isaacsim.core.api import World
from scipy.spatial.transform import Rotation

for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.thrusters.quadratic_thrust_curve import (
    QuadraticThrustCurve, ROTOR_TAU_PLACEHOLDER)

# parse_args() runs before this import (it must, to build SimulationApp), so the default
# is resolved here instead of in the parser. --rotor-tau 0 stays 0: an explicit request
# for the old instantaneous model is not the same as "unset".
ROTOR_TAU_IS_DEFAULT = ARGS.rotor_tau is None
if ROTOR_TAU_IS_DEFAULT:
    ARGS.rotor_tau = ROTOR_TAU_PLACEHOLDER
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface


def _resolve_keep_log(keep_log) -> str:
    """Turn --keep-log into a directory for PX4 to run in, or None for the temp default."""
    if keep_log is None:
        return None

    import datetime
    import os

    root = keep_log or os.path.join(os.getcwd(), "px4_runs",
                                    datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    root = os.path.abspath(os.path.expanduser(root))
    os.makedirs(root, exist_ok=True)
    carb.log_warn(f"PX4 rootfs kept at '{root}' - ulog will be under {root}/log/")
    return root


def _write_run_config(rootfs: str) -> None:
    """Record the SIMULATOR-side settings next to the ulog.

    motor_moment lives in Pegasus, not PX4, so it appears nowhere in the ulog. Without
    this, a spin ladder is a set of logs distinguishable only by inferring km from the
    measured yaw rate - which is circular when the question under test is what spin does
    to stability, and simply wrong if the controller partially arrests the spin.
    """
    if rootfs is None:
        return

    import json
    import os

    config = {
        "motor_moment": ARGS.motor_moment,
        "expected_spin_rad_s": 4.9e7 * ARGS.motor_moment,   # r_ss ~ 4.9e7*km, FINDINGS 7
        # Simulator-side like motor_moment, so it appears nowhere in the ulog. A gain set
        # is only meaningful against the rotor_tau it was found at - without this recorded,
        # two runs with different plants are indistinguishable after the fact.
        "rotor_tau": ARGS.rotor_tau,
        # True only when the shared placeholder was used. Once measure_motor_lag.py gives
        # a real number and ROTOR_TAU_PLACEHOLDER is updated, this still reads True - so
        # treat it as "not explicitly chosen for this run", and trust the value itself.
        "rotor_tau_is_placeholder": ROTOR_TAU_IS_DEFAULT,
        "px4_airframe": ARGS.px4_airframe,
        "physics_rate_hz": ARGS.physics_rate,
        "vehicle_id": ARGS.vehicle_id,
        "duration_s": ARGS.duration,
    }
    path = os.path.join(rootfs, "run_config.json")
    with open(path, "w") as handle:
        json.dump(config, handle, indent=2)
    carb.log_warn(f"run config written to {path}: {config}")


def _resolve_px4_autolaunch(px4_dir: str) -> tuple[bool, str]:
    px4_root = Path(px4_dir).expanduser()
    px4_binary = px4_root / "build" / "px4_sitl_default" / "bin" / "px4"
    if px4_binary.exists():
        return True, str(px4_root)

    carb.log_warn(
        f"PX4 SITL binary not found at '{px4_binary}'. Disabling px4_autolaunch. "
        "Build it with 'make px4_sitl_default none' in that checkout, or pass --px4-dir."
    )
    return False, str(px4_root)


class PegasusApp:
    """Spawns one Iris multirotor flown by PX4 SITL over mavlink HIL."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()

        # 250 Hz, and not higher. Raising it looks like it should cut this loop's
        # dominant latency term - Pegasus reads input_reference() and applies rotor
        # forces before backend.update() runs, so an actuator message received during
        # step N lands at step N+1, and the IMU sample sent at step N was computed at
        # the end of step N-1, about two physics steps of pure ordering delay - but
        # the loop cannot be driven faster than PX4 answers it.
        #
        # Measured against this PX4 (v1.17.0-alpha1, none_iris): PX4 emits
        # HIL_ACTUATOR_CONTROLS at 250 Hz no matter how fast HIL_SENSOR arrives. At
        # physics_dt = 1/500 the backend stamps sensor messages 2 ms apart and PX4's
        # rate loop does run at 500 Hz (mc_controller_status.update_count confirms
        # it), but only every second sensor message draws a reply. PX4MavlinkBackend
        # blocks waiting for that reply, and PX4's clock only advances on the sensor
        # message the backend is withholding, so the pair wedges: PX4 finishes
        # booting and starts mavlink, then its clock stops, it prints
        # "ERROR [simulator_mavlink] poll timeout" once a second forever, and with
        # time frozen it never sends another heartbeat - so QGroundControl sees the
        # port but can never complete a connection.
        #
        # The backend's LOCKSTEP_TIMEOUT now turns that hang into a warning, so a
        # higher rate degrades rather than freezes - but it still degrades, roughly
        # one stalled step per two, until PX4's SITL output stage keeps up.
        #
        # Set through the interface rather than params.py so the other examples,
        # which are paired with airframes assuming the stock rate, are unaffected.
        self.pg.set_world_settings(physics_dt=1.0 / ARGS.physics_rate)

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        px4_dir = ARGS.px4_dir or self.pg.px4_path
        px4_airframe = ARGS.px4_airframe or self.pg.px4_default_airframe
        px4_autolaunch, px4_dir = _resolve_px4_autolaunch(px4_dir)

        if not ARGS.px4_autolaunch:
            px4_autolaunch = False
            carb.log_warn(f"px4_autolaunch disabled; expecting an external PX4 on port "
                          f"{4560 + ARGS.vehicle_id} (this script listens, PX4 connects in).")

        px4_rootfs = _resolve_keep_log(ARGS.keep_log)
        _write_run_config(px4_rootfs)

        config_multirotor = MultirotorConfig()
        config_multirotor.desaturation = ARGS.desaturation

        # Spin is an explicit run parameter, not a file edit. MultirotorConfig's default
        # QuadraticThrustCurve() picks up whatever motor_moment default happens to be in
        # quadratic_thrust_curve.py, which is how a "spinning vehicle" run could silently be
        # a non-spinning one. Same call the sweeps use (batch_compare.py:251).
        config_multirotor.thrust_curve = QuadraticThrustCurve(
            config={"motor_moment": ARGS.motor_moment, "rotor_tau": ARGS.rotor_tau})
        carb.log_warn(f"motor_moment = {ARGS.motor_moment:.3e} "
                      f"-> expected steady yaw spin ~ {4.9e7 * ARGS.motor_moment:.1f} rad/s")
        carb.log_warn(f"desaturation = {ARGS.desaturation}")
        if ARGS.rotor_tau > 0.0:
            carb.log_warn(
                f"rotor_tau = {1000 * ARGS.rotor_tau:.1f} ms"
                + (" (PLACEHOLDER from ROTOR_TAU_PLACEHOLDER - measure it, see "
                   "measure_motor_lag.py). Gains found against this value are only as "
                   "right as it is." if ROTOR_TAU_IS_DEFAULT else " (explicit)"))
        else:
            carb.log_warn("rotor_tau = 0: INSTANTANEOUS rotor model. The plant is missing "
                          "the ESC/motor lag hardware has; gains will not transfer.")

        # input_scaling/zero_position_armed are overridden from their 1000/100
        # defaults so the rotor map passes through the origin:
        #   omega = u * 1100        instead of   omega = 1000*u + 100
        # Pegasus then produces F = 8.54858e-6 * (1100u)^2 = 10.344 * u^2, which
        # is exactly the CT*u^2 model control_allocator uses, with the matching
        # CA_ROTOR*_CT 10.344 set in the none_iris airframe. With the default
        # +100 offset the curve misses the origin, so PX4's belief about rotor
        # thrust was wrong by a throttle-dependent 1.6-2.3x and the attitude
        # gains absorbed all of it.
        #
        # Set here rather than on PX4MavlinkBackendConfig's defaults on purpose:
        # multirotor.py, iris.py, iris_spin.py, ui_delegate.py and
        # backends_parser.py all construct that config too, and they are paired
        # with airframes that still assume the stock mapping.
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": ARGS.vehicle_id,
            "px4_autolaunch": px4_autolaunch,
            "px4_dir": px4_dir,
            "px4_vehicle_model": px4_airframe,
            "input_scaling": [1100.0, 1100.0, 1100.0, 1100.0],
            "zero_position_armed": [0.0, 0.0, 0.0, 0.0],
            # Match the physics rate so state reaches PX4 every step rather than
            # every other one. Note the vehicle's IMU is itself a 250 Hz sensor
            # (Multirotor's default), and backend.update() sends nothing on a step
            # with no new IMU sample - so this only buys anything if the sensor rate
            # is raised alongside it.
            "update_rate": ARGS.physics_rate,
            # None keeps the stock temp rootfs (and discards the ulog with it).
            "px4_rootfs_dir": px4_rootfs,
        })
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        Multirotor(
            "/World/quadrotor",
            ROBOTS["Iris"],
            ARGS.vehicle_id,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        self.world.reset()
        self.stop_sim = False

    def run(self):
        self.timeline.play()

        elapsed = 0.0
        physics_dt = self.pg._world_settings.get("physics_dt", 1.0 / 250.0)

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

            if ARGS.duration is not None:
                elapsed += physics_dt
                if elapsed >= ARGS.duration:
                    break

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
