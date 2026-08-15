#!/usr/bin/env python
"""
| File: 4_python_single_vehicle.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

import sys
from pathlib import Path

for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
import omni.ui as ui
from isaacsim.core.api import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve

# Import the custom python control backend
import sys, os
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from cascaded_pd_controller import CascadedPDController
from nonlinear_controller import NonlinearController
from eigen_controller import EigenController
from lpv_mpc_controller import LPVMPCController
import rotor_failure
import sweep_task as task

import numpy as np

# Auxiliary scipy module
from scipy.spatial.transform import Rotation

# Use pathlib for parsing the desired trajectory from a CSV file
from pathlib import Path


class PegasusApp:
    
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self, selected_controller: str, motor_moment: float = None,
                 failure_mode: str = "manual", failure_rotor: int = None,
                 failure_seed: int = None):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.

        failure_mode: "manual" waits for the FAIL ROTOR button, "auto" stops a
            rotor the moment the vehicle settles, "off" disables it entirely.
        failure_rotor: which rotor to stop, or None to pick at random.
        failure_seed: seeds that random choice so a run reproduces.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Allow time for the environment to load asynchronously
        # The load_environment method runs asynchronously, so we need to let it complete
        # before proceeding with vehicle spawning and world reset
        for _ in range(10):
            self.world.step(render=False)

        # Get the current directory used to read trajectories and save results
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        # Build selected backend from the controller registry
        self.controller = build_controller(selected_controller, self.curr_dir)
        carb.log_warn(f"Using controller: {selected_controller}")

        # Wrap it so one rotor can be stopped mid-flight. The wrapper forwards
        # every unknown attribute to the controller, so self.backend.w and the
        # rest of this file keep working unchanged.
        #
        # A FlightMonitor is attached only to gate the settle test on "actually
        # airborne" — this app has no bounds checking and does not end the run
        # on a crash, because the whole point is to watch what happens after.
        self.failure_mode = failure_mode
        if failure_mode == "off":
            self.backend = self.controller
        else:
            monitor = task.FlightMonitor(1.5, label="rotor-failure",
                                         log=carb.log_warn)
            self.backend = rotor_failure.RotorFailureBackend(
                self.controller,
                results_file=None,          # the controller writes its own npz
                rotor=failure_rotor,
                rng=np.random.default_rng(failure_seed),
                auto_fail=(failure_mode == "auto"),
                monitor=monitor,
                log=carb.log_warn,
            )

        # Create the vehicle 1
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor1 = MultirotorConfig()
        config_multirotor1.backends = [self.backend]
        if motor_moment is not None:
            config_multirotor1.thrust_curve = QuadraticThrustCurve(config={"motor_moment": motor_moment})

        Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            [2.3, -1.5, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor1,
        )

        # Arena the flight monitor judges takeoff against. Generous, since
        # nothing here terminates on leaving it.
        self._bounds = [-50., 50., -50., 50., 0., 50.]

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        # backend.start() is NOT called here: Vehicle's timeline callback invokes
        # it on play, after initialize() has attached the vehicle. Calling it
        # here would run before that and then be immediately redone.
        self.world.reset()
        self._setup_yaw_rate_overlay()

    def _setup_yaw_rate_overlay(self):
        has_failure = self.failure_mode != "off"
        self._yaw_window = ui.Window("Telemetry", width=300,
                                     height=210 if has_failure else 84)
        with self._yaw_window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Yaw Rate", alignment=ui.Alignment.CENTER)
                self._yaw_label = ui.Label("w_z: -- rad/s", alignment=ui.Alignment.CENTER)

                if not has_failure:
                    return

                ui.Separator(height=4)
                ui.Label("Rotor Failure", alignment=ui.Alignment.CENTER)
                self._settle_label = ui.Label("settling: --",
                                              alignment=ui.Alignment.CENTER)
                self._failure_label = ui.Label("no failure",
                                               alignment=ui.Alignment.CENTER)

                # Fires through the backend rather than mutating state here, so
                # the manual and automatic paths are the same one-shot code and
                # a press while the sim is paused is latched, not lost.
                self._fail_button = ui.Button(
                    "FAIL ROTOR", height=28,
                    clicked_fn=lambda: self.backend.trigger())

                with ui.HStack(spacing=4, height=24):
                    ui.Label("auto on settle:", width=100)
                    self._auto_check = ui.CheckBox(width=24)
                    self._auto_check.model.set_value(self.failure_mode == "auto")
                    self._auto_check.model.add_value_changed_fn(
                        lambda m: setattr(self.backend, "auto_fail", m.get_value_as_bool()))

    def _update_yaw_rate_overlay(self):
        if not hasattr(self, "_yaw_label"):
            return

        yaw_rate = None
        if hasattr(self.backend, "w") and len(self.backend.w) >= 3:
            yaw_rate = float(self.backend.w[2])

        if yaw_rate is None:
            self._yaw_label.text = "w_z: -- rad/s"
        else:
            self._yaw_label.text = f"w_z: {yaw_rate:+7.4f} rad/s"

        if not hasattr(self, "_failure_label"):
            return

        b = self.backend
        if b.has_failed:
            self._failure_label.text = (f"ROTOR {b.failed_rotor} STOPPED "
                                        f"@ t={b.failure_time:.2f}s")
            self._settle_label.text = (
                f"settled @ t={b.settle_time:.2f}s" if b.settle_time is not None
                else "failed before settling")
            self._fail_button.text = "FAILED"
            self._fail_button.enabled = False
        else:
            self._settle_label.text = ("SETTLED — ready" if b.is_settled
                                       else f"settling: |v|={np.linalg.norm(b.velocity):.2f} m/s")
            self._failure_label.text = "no failure"

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running():

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

            # Advance the flight monitor. Its return value is deliberately
            # ignored: it reports a crash, and this app exists to watch what
            # happens after the rotor stops rather than to end the run there.
            # Without this call the monitor never sees a position, `airborne`
            # stays False, and the settle test — and so auto-fail — can never
            # fire.
            if self.failure_mode != "off":
                self.backend.check_oob(self._bounds)

            self._update_yaw_rate_overlay()
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def build_controller(controller_key: str, curr_dir: str):
    """Factory/registry for all imported controllers."""

    controller_registry = {
        "pd": lambda: CascadedPDController(
            target_position=(0.0, 0.0, 1.5),
            results_file=curr_dir + "/results/controller_eval/cascaded_pd/statistics/cascaded_pd_statistics.npz",
        ),
        "nonlinear": lambda: NonlinearController(
            trajectory_file=None,  # Use built-in exponential trajectory
            results_file=curr_dir + "/results/controller_eval/nonlinear/statistics/nonlinear_statistics.npz",
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0],
        ),
        "eigen": lambda: EigenController(
            trajectory_file=None,  # Use built-in exponential trajectory
            results_file=curr_dir + "/results/controller_eval/eigen/statistics/eigen_statistics.npz",

        ),
        "mpc": lambda: LPVMPCController(
            trajectory_file=None,
            results_file=curr_dir + "/results/controller_eval/lpv_mpc/statistics/lpv_mpc_statistics.npz",
            N=12,            # Horizon Window Length
            q_weight=15.0,   # Tracking state weight
            r_weight=0.02,   # Control energy optimization weight
        )
    }

    if controller_key not in controller_registry:
        valid = ", ".join(controller_registry.keys())
        raise ValueError(f"Unknown controller '{controller_key}'. Valid options: {valid}")

    return controller_registry[controller_key]()


def main():
    print("Available controllers: pd, nonlinear, eigen, mpc")
    selected_controller = input("Select controller: ").strip().lower()

    motor_moment_str = input("Enter motor_moment (leave empty for default): ").strip()
    motor_moment = float(motor_moment_str) if motor_moment_str else None

    print("\nRotor failure: 'manual' = FAIL ROTOR button, 'auto' = stop one the")
    print("moment the vehicle settles, 'off' = disable. The button is present in")
    print("both manual and auto, and the auto checkbox can be toggled in flight.")
    failure_mode = input("Failure mode [manual/auto/off] (default manual): ").strip().lower()
    if failure_mode not in ("manual", "auto", "off"):
        failure_mode = "manual"

    failure_rotor = None
    failure_seed = None
    if failure_mode != "off":
        rotor_str = input("Rotor to stop 0-3 (leave empty for random): ").strip()
        failure_rotor = int(rotor_str) if rotor_str else None
        if failure_rotor is None:
            seed_str = input("Seed for the random choice (leave empty for none): ").strip()
            failure_seed = int(seed_str) if seed_str else None

    # Instantiate the template app
    pg_app = PegasusApp(selected_controller, motor_moment,
                        failure_mode=failure_mode,
                        failure_rotor=failure_rotor,
                        failure_seed=failure_seed)

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
