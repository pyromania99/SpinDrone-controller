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

# Auxiliary scipy module
from scipy.spatial.transform import Rotation

# Use pathlib for parsing the desired trajectory from a CSV file
from pathlib import Path


class PegasusApp:
    
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self, selected_controller: str, motor_moment: float = None):
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
        self.backend = build_controller(selected_controller, self.curr_dir)
        carb.log_warn(f"Using controller: {selected_controller}")

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

        yaw_rate = None
        if hasattr(self.backend, "w") and len(self.backend.w) >= 3:
            yaw_rate = float(self.backend.w[2])

        if yaw_rate is None:
            self._yaw_label.text = "w_z: -- rad/s"
        else:
            self._yaw_label.text = f"w_z: {yaw_rate:+7.4f} rad/s"

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

    # Instantiate the template app
    pg_app = PegasusApp(selected_controller, motor_moment)

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
