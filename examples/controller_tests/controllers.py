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
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State
import numpy as np

# Import the custom python control backend
import sys, os
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from cascaded_pd_controller import CascadedPDController
from nonlinear_controller import NonlinearController

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# Use pathlib for parsing the desired trajectory from a CSV file
from pathlib import Path


class PegasusApp:
    
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self, selected_controller: str):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
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
        backend = build_controller(selected_controller, self.curr_dir)
        telemetry_results = self.curr_dir + f"/results/{selected_controller}_telemetry.npz"
        self.backend = YawRateTelemetryBackend(
            backend,
            results_file=telemetry_results,
            log_yaw_rate=True,
            yaw_rate_log_period=0.25,
        )
        carb.log_warn(f"Using controller: {selected_controller}")

        # Create the vehicle 1
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor1 = MultirotorConfig()
        config_multirotor1.backends = [self.backend]

        Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            [2.3, -1.5, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor1,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()
        self._setup_yaw_rate_overlay()

    def _setup_yaw_rate_overlay(self):
        self._yaw_window = ui.Window("Telemetry", width=280, height=84)
        with self._yaw_window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Yaw Rate", alignment=ui.Alignment.CENTER)
                self._yaw_label = ui.Label("w_z: -- rad/s", alignment=ui.Alignment.CENTER)

    def _update_yaw_rate_overlay(self):
        if not hasattr(self, "_yaw_label"):
            return
        self._yaw_label.text = f"w_z: {self.backend.current_yaw_rate:+7.4f} rad/s"

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
            self._update_yaw_rate_overlay()
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


class YawRateTelemetryBackend(Backend):
    """Wrap any backend and add yaw-rate logging + optional telemetry file output."""

    def __init__(
        self,
        wrapped_backend: Backend,
        results_file: str = None,
        log_yaw_rate: bool = True,
        yaw_rate_log_period: float = 0.25,
    ):
        super().__init__(config=None)
        self.wrapped_backend = wrapped_backend
        self.results_file = results_file
        self.log_yaw_rate = log_yaw_rate
        self.yaw_rate_log_period = max(float(yaw_rate_log_period), 0.0)

        self.total_time = 0.0
        self.current_yaw_rate = 0.0
        self._last_log_time = -1.0

        self.time_vector = []
        self.yaw_rate_over_time = []

    def initialize(self, vehicle):
        super().initialize(vehicle)
        self.wrapped_backend.initialize(vehicle)

    def start(self):
        self.total_time = 0.0
        self.current_yaw_rate = 0.0
        self._last_log_time = -1.0
        self.time_vector = []
        self.yaw_rate_over_time = []
        self.wrapped_backend.start()

    def stop(self):
        self.wrapped_backend.stop()

        if self.results_file is not None and len(self.time_vector) > 0:
            np.savez(
                self.results_file,
                time=np.array(self.time_vector),
                yaw_rate=np.array(self.yaw_rate_over_time),
            )
            carb.log_warn("Yaw telemetry saved to: " + self.results_file)

    def update_sensor(self, sensor_type: str, data):
        self.wrapped_backend.update_sensor(sensor_type, data)

    def update_graphical_sensor(self, sensor_type: str, data):
        self.wrapped_backend.update_graphical_sensor(sensor_type, data)

    def reset(self):
        self.wrapped_backend.reset()

    def update_state(self, state: State):
        self.current_yaw_rate = float(state.angular_velocity[2])
        self.wrapped_backend.update_state(state)

    def input_reference(self):
        return self.wrapped_backend.input_reference()

    def update(self, dt: float):
        self.total_time += dt
        self.wrapped_backend.update(dt)

        self.time_vector.append(self.total_time)
        self.yaw_rate_over_time.append(self.current_yaw_rate)

        if self.log_yaw_rate and self.yaw_rate_log_period >= 0.0:
            should_log = self._last_log_time < 0.0
            if not should_log:
                should_log = (self.total_time - self._last_log_time) >= self.yaw_rate_log_period

            if should_log:
                carb.log_info(
                    "[YawRate] "
                    f"t={self.total_time:7.3f}s "
                    f"w_z={self.current_yaw_rate:+7.4f} rad/s"
                )
                self._last_log_time = self.total_time

def build_controller(controller_key: str, curr_dir: str):
    """Factory/registry for all imported controllers."""

    controller_registry = {
        "cascaded_pd": lambda: CascadedPDController(
            target_position=(0.0, 0.0, 1.5),
            results_file=curr_dir + "/results/cascaded_pd_statistics.npz",
        ),
        "nonlinear": lambda: NonlinearController(
            trajectory_file=None,  # Use built-in exponential trajectory
            results_file=curr_dir + "/results/nonlinear_statistics.npz",
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0],
        ),
    }

    if controller_key not in controller_registry:
        valid = ", ".join(controller_registry.keys())
        raise ValueError(f"Unknown controller '{controller_key}'. Valid options: {valid}")

    return controller_registry[controller_key]()


def main():
    print("Available controllers:cascaded_pd, nonlinear")
    selected_controller = input("Select controller: ").strip().lower()

    # Instantiate the template app
    pg_app = PegasusApp(selected_controller)

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
