#!/usr/bin/env python
"""
| File: sim_pegasus.py
| Description: Level 1 of the simulation ladder - Pegasus with a pure-Python
               controller, no PX4 and no ROS.

The controller is a Python ``Backend`` reading simulator state directly, so there
is no estimator and no transport delay in the loop. That is the whole point of
this level: whatever it measures is the control law and the plant, nothing else.

Derived from Pegasus Simulator's python single-vehicle example.
| Original author: Marcelo Jacinto and Joao Pinto
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.

Usually reached through ``fly.py --level 1``, which supplies these flags. Run it
directly and any flag you leave out is prompted for, which is the interactive
mode this script has always had.
"""
import argparse
import sys


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--controller", choices=("pd", "nonlinear", "eigen", "mpc"),
                   default=None, help="Which control law to fly.")
    p.add_argument("--motor-moment", type=float, default=None,
                   help="Rotor drag moment km. Sets the steady yaw spin "
                        "(r_ss ~ 4.9e7 * km). Omit for the controller's own default.")
    p.add_argument("--failure-mode", choices=("manual", "auto", "off"), default=None,
                   help="'manual' = FAIL ROTOR button, 'auto' = stop one the moment the "
                        "vehicle settles, 'off' = disable.")
    p.add_argument("--rotor", type=int, choices=[0, 1, 2, 3], default=None,
                   help="Which rotor to stop. Omit for a random choice.")
    p.add_argument("--seed", type=int, default=None,
                   help="Seed for the random rotor choice.")
    p.add_argument("--desaturation", choices=("uniform", "collective"), default=None,
                   help="What the mixer gives up when a rotor saturates. 'uniform' is "
                        "the default everywhere else in this repo; 'collective' holds "
                        "lift and is what lets the vehicle climb after a rotor stops.")
    p.add_argument("--rotor-tau", type=float, default=None, metavar="S",
                   help="Rotor/ESC spin-up time constant [s]. Default is the shared "
                        "placeholder (0.020). Pass 0 for instant response, which is "
                        "the plant every archive in results/ older than 2026-08-15 "
                        "was flown on - gains tuned there can be unstable against a "
                        "lag that did not exist when they were found.")
    p.add_argument("--seat", choices=("off", "fixed", "adaptive"), default="off",
                   help="Command-frame rotation of the roll/pitch wrench, to cancel the "
                        "phase rotation actuator lag applies under spin (brief v2). "
                        "'off' (default) builds no seat at all. 'fixed' sets the angle "
                        "from r*tau_a + atan(r*rotor_tau); 'adaptive' measures the "
                        "misalignment in flight. Eigen controller only.")
    p.add_argument("--seat-law", choices=("angle", "cross"), default="cross",
                   help="theta_s update law. 'cross' (default) drives on the raw "
                        "cross product |xd||xa|sin(alpha), so a starved channel "
                        "suppresses its own update and no eps gate is needed. "
                        "'angle' drives on atan2 alpha - full-size steps regardless "
                        "of signal. Note k is dimensional for 'cross'.")
    p.add_argument("--adapt-b", action="store_true",
                   help="Adapt the pole angle inside M_dyn (brief v3 §4). Independent "
                        "of --seat: it rotates nothing, it moves where the designed "
                        "closed-loop pole sits. Use with --seat off to run b alone.")
    p.add_argument("--max-lateral-accel", type=float, default=0.0,
                   help="Clamp the horizontal acceleration demand [m/s^2], scaled as a vector so the direction is preserved. 0 = unlimited (original).")
    p.add_argument("--waypoint-radius", type=float, default=0.0,
                   help="With --trajectory: advance to the next point on ARRIVAL within this radius [m] instead of on the CSV clock. The reference then waits for the vehicle, which caps position error at the radius and keeps the tilt demand bounded. 0 keeps the original time-based behaviour.")
    p.add_argument("--trajectory", default=None,
                   help="CSV of time,x,y,z to follow instead of holding the goal. Gives the SUSTAINED lateral demand a translation-direction observable needs - a one-shot move to a goal excites it for only a few seconds.")
    p.add_argument("--pole-gains", nargs="*", default=[], metavar="K=V",
                   help="PoleAdapter overrides, e.g. k_b=0.2 theta_b_max=1.4.")
    p.add_argument("--seat-gains", nargs="*", default=[], metavar="K=V",
                   help="Seat overrides, e.g. k=3 theta_max=1.4 r_min=8 k_sign=-1 "
                        "tau_a=0.002. Names are seat.Seat's constructor arguments.")
    p.add_argument("--gains", nargs="*", default=[], metavar="K=V",
                   help="Controller gain overrides, e.g. --gains Kp_att=60 "
                        "Kd_att=1.481 Kd_yaw_rate=0. Applied through "
                        "controller_factory, the same path the sweeps use, so the "
                        "names and per-axis broadcasting match an archive row. "
                        "Without this the controller flies its own defaults.")
    p.add_argument("--lateral", action="store_true", default=False,
                   help="Translate to the goal after taking off. Off by default so "
                        "the profile matches levels 2 and 3, which climb to altitude "
                        "and hold the takeoff point - a level-1 run that also "
                        "translates is not comparable with them.")
    p.add_argument("--no-takeoff-first", dest="takeoff_first", action="store_false",
                   default=True,
                   help="With --lateral, fly one diagonal to the goal (climbing and "
                        "translating together) instead of climbing first. That is how "
                        "every archived sweep in results/ was flown, so use it when "
                        "comparing against one. No effect without --lateral.")
    p.add_argument("--duration", type=float, default=None,
                   help="Stop after this many seconds of simulated time and write the "
                        "npz. Without it the run continues until the window is closed, "
                        "which never happens headless - so a headless run needs this.")
    p.add_argument("--headless", action="store_true", default=False)
    # Isaac appends its own argv, so ignore what is not ours.
    args = p.parse_known_args()[0]

    # Parsed here rather than after startup: a typo in --gains should cost
    # nothing, and main() does not run until Isaac has booted.
    parsed = {}
    for kv in args.gains:
        key, sep, val = kv.partition("=")
        if not sep:
            p.error(f"--gains expects K=V, got {kv!r}")
        try:
            parsed[key.strip()] = float(val)
        except ValueError:
            p.error(f"--gains {key.strip()}: {val!r} is not a number")
    args.gains = parsed

    seat_parsed = {}
    for kv in args.seat_gains:
        key, sep, val = kv.partition("=")
        if not sep:
            p.error(f"--seat-gains expects K=V, got {kv!r}")
        try:
            seat_parsed[key.strip()] = float(val)
        except ValueError:
            p.error(f"--seat-gains {key.strip()}: {val!r} is not a number")
    args.seat_gains = seat_parsed

    pole_parsed = {}
    # 'law' selects between the two pole adaptations and is a NAME, not a number, so it
    # has to survive the float() the other keys go through.
    _POLE_STR_KEYS = ("law",)
    for kv in args.pole_gains:
        key, sep, val = kv.partition("=")
        if not sep:
            p.error(f"--pole-gains expects K=V, got {kv!r}")
        key = key.strip()
        if key in _POLE_STR_KEYS:
            pole_parsed[key] = val.strip()
            continue
        try:
            pole_parsed[key] = float(val)
        except ValueError:
            p.error(f"--pole-gains {key}: {val!r} is not a number")
    args.pole_gains = pole_parsed
    return args


ARGS = parse_args()

# Imports to start Isaac Sim from this script
import carb                                                          # noqa: E402
from isaacsim import SimulationApp                                   # noqa: E402

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": ARGS.headless})

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
from pegasus.simulator.logic.thrusters.quadratic_thrust_curve import ROTOR_TAU_PLACEHOLDER

# Import the custom python control backend
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from cascaded_pd_controller import CascadedPDController
from nonlinear_controller import NonlinearController
from eigen_controller import EigenController
from seat import PoleAdapter, Seat
from lpv_mpc_controller import LPVMPCController
import rotor_failure
import sweep_task as task
import sim_clock
import controller_factory as cf

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
                 failure_seed: int = None, desaturation: str = "uniform",
                 takeoff_first: bool = True, lateral: bool = False,
                 gains: dict = None, rotor_tau: float = None,
                 seat: str = "off", seat_gains: dict = None,
                 adapt_b: bool = False, pole_gains: dict = None,
                 seat_law: str = "cross"):
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

        # The one anchored results tree, shared with every sweep, rather than a
        # directory beside this script.
        self.eval_dir = task.results_root() / "controller_eval"

        # Build selected backend from the controller registry
        self.controller = build_controller(selected_controller, self.eval_dir,
                                          gains=gains, km=motor_moment,
                                          seat=seat, seat_gains=seat_gains,
                                          adapt_b=adapt_b, pole_gains=pole_gains,
                                          seat_law=seat_law,
                                          rotor_tau=rotor_tau)
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
        # rotor_tau is applied whether or not motor_moment was given: the ESC/motor
        # spin-up is a property of the vehicle, not of the spin. Leaving it off here while
        # sim_px4.py and genetic_sweep.py both carry it is exactly the drift that
        # made those two model different vehicles - and this script is the Pegasus-only
        # half of every PX4 comparison, so it is the one that most needs to match.
        tau = ROTOR_TAU_PLACEHOLDER if rotor_tau is None else float(rotor_tau)
        config_multirotor1.thrust_curve = QuadraticThrustCurve(config={
            "motor_moment": motor_moment if motor_moment is not None else 0.0,
            "rotor_tau": tau,
        })
        carb.log_warn(f"[plant] rotor_tau = {tau*1e3:.1f} ms"
                      + ("  (instant response)" if tau == 0.0 else ""))
        # Which quantity the mixer sacrifices when a rotor saturates. "uniform"
        # is the default everywhere else; "collective" keeps lift and gives up
        # moment, which is what lets the vehicle climb after a rotor stops.
        self.desaturation = desaturation
        config_multirotor1.desaturation = desaturation

        SPAWN = [2.3, -1.5, 0.07]
        Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            SPAWN,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor1,
        )

        # Arena the flight monitor judges takeoff against. Generous, since
        # nothing here terminates on leaving it.
        self._bounds = [-50., 50., -50., 50., 0., 50.]

        # ── Flight profile: climb, then hold or translate ────────────────────
        #
        # Default is climb-and-hold over the spawn, with NO lateral leg, because
        # that is what levels 2 and 3 fly: offboard.py takes off to altitude and
        # then holds local NED (0, 0, -alt) - the takeoff point - for the whole
        # run. Level 1 used to fly a ground start at SPAWN toward a goal
        # elsewhere, so it alone was doing a translation, and the ladder's
        # differences included that instead of only the thing being measured.
        #
        # --lateral restores the translation, and then --no-takeoff-first
        # decides how: staged (climb to altitude first, then go) or as the
        # single diagonal every archived sweep in results/ was flown with.
        self._goal = None
        self._translated = True          # nothing pending
        goal = getattr(self.controller, "hover_target", None)

        if goal is None:
            # nonlinear_controller follows its own built-in trajectory and has no
            # hover_target to retarget; say so rather than silently doing nothing.
            carb.log_warn("[profile] controller is trajectory-driven (no "
                          "hover_target); flying it unmodified.")
        elif not lateral:
            hold = np.array([SPAWN[0], SPAWN[1], float(goal[2])], dtype=float)
            self.controller.hover_target = hold
            carb.log_warn(
                f"[profile] vertical only: climb to {hold[2]:.2f} m over the spawn "
                f"and hold. Matches levels 2/3. --lateral to translate.")
        elif takeoff_first:
            self._goal = np.array(goal, dtype=float)
            # Same altitude, but straight up from where it starts.
            self.controller.hover_target = np.array(
                [SPAWN[0], SPAWN[1], self._goal[2]], dtype=float)
            # 90% of the goal altitude, NOT FlightMonitor's TAKEOFF_ALTITUDE_FRAC
            # (0.5). That fraction answers "has it left the ground", which is a
            # crash gate; this needs "is it actually at altitude", and handing the
            # lateral goal over at half height would reintroduce the diagonal this
            # is meant to remove. Short of 1.0 so a small steady-state droop
            # cannot stall the handover forever.
            self._handover_alt = 0.90 * float(self._goal[2])
            self._translated = False
            carb.log_warn(
                f"[profile] climbing to {self._goal[2]:.2f} m over the spawn, then "
                f"translating to {tuple(round(float(v), 2) for v in self._goal)}")
        else:
            carb.log_warn(
                f"[profile] single diagonal from the spawn to "
                f"{tuple(round(float(v), 2) for v in np.asarray(goal, dtype=float))} "
                f"- climbing and translating together, as the archived sweeps did.")

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
                # Which mixer is flying matters more than anything else on this
                # panel once a rotor is stopped, so it is stated outright rather
                # than left to whatever was typed at the prompt.
                ui.Label(f"mixer: {self.desaturation}",
                         alignment=ui.Alignment.CENTER)
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

    def _maybe_translate(self) -> None:
        """Hand the real goal over once the vehicle has actually climbed.

        Called every step; a no-op after the handover, and when staging is off
        or the controller has no hover_target to stage.
        """
        if self._translated:
            return
        z = float(np.asarray(self.controller.p, dtype=float)[2])
        if z < self._handover_alt:
            return
        self.controller.hover_target = self._goal
        self._translated = True
        carb.log_warn(f"[takeoff-first] reached {z:.2f} m - translating to "
                      f"{tuple(round(float(v), 2) for v in self._goal)}")

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # Bounded when --duration is given, unbounded otherwise. Headless has no
        # window to close, so without a bound the run only ends when something
        # kills it - and a killed process never reaches timeline.stop(), which is
        # what makes the vehicle flush its npz.
        # Measured in SIMULATED seconds actually flown, not in steps. Counting
        # steps needs a step->time factor, and world.step(render=True) advances a
        # rendering frame that can contain several physics steps: a 25 s request
        # counted that way ran 100 s. elapsed_from_backends reads the time the
        # physics callback actually accumulated, so it holds whatever the
        # physics/rendering dt ratio is.
        if ARGS.duration is not None:
            carb.log_warn(f"[duration] stopping after {ARGS.duration:g} simulated s")

        # Rendering headless costs time and buys nothing - and it is what made a
        # step count mean four different things depending on the flag.
        render = not ARGS.headless

        while simulation_app.is_running():
            if ARGS.duration is not None:
                flown = sim_clock.elapsed_from_backends([self.backend])
                if flown >= ARGS.duration:
                    carb.log_warn(f"[duration] reached {flown:.2f} s; stopping")
                    break

            # Update the UI of the app and perform the physics step
            self.world.step(render=render)

            # Advance the flight monitor. Its return value is deliberately
            # ignored: it reports a crash, and this app exists to watch what
            # happens after the rotor stops rather than to end the run there.
            # Without this call the monitor never sees a position, `airborne`
            # stays False, and the settle test — and so auto-fail — can never
            # fire.
            if self.failure_mode != "off":
                self.backend.check_oob(self._bounds)

            self._maybe_translate()
            self._update_yaw_rate_overlay()
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def _make_seat(mode, overrides, rotor_tau, controller_key, law="cross"):
    """Build the seat, or None for no seat at all.

    None rather than a disabled object, so the eigen law genuinely has no seat in
    the 'off' arm. Anything but the eigen law gets None and is told why: the seat
    sits in that law's update(), and porting it to the other two would be
    duplication with nothing to learn from it.
    """
    if mode in (None, "off"):
        return None
    if controller_key != "eigen":
        carb.log_warn(f"[seat] ignored: --seat is wired into the eigen law only, "
                      f"and '{controller_key}' was selected")
        return None
    kwargs = dict(overrides or {})
    # rotor_tau IS the actuator lag this compensates, so default T to the plant's
    # own value. Only 'fixed' reads it; 'adaptive' never does.
    kwargs.setdefault("T", ROTOR_TAU_PLACEHOLDER if rotor_tau is None else float(rotor_tau))
    kwargs.setdefault("law", law)
    s = Seat(mode=mode, **kwargs)
    carb.log_warn(f"[seat] {s.describe()}")
    return s


def _make_pole_adapter(enabled, overrides, controller_key):
    """Build the adaptive pole angle, or None."""
    if not enabled:
        return None
    if controller_key != "eigen":
        carb.log_warn(f"[pole] ignored: --adapt-b is wired into the eigen law only, "
                      f"and '{controller_key}' was selected")
        return None
    pa = PoleAdapter(**dict(overrides or {}))
    carb.log_warn(f"[pole] {pa.describe()}")
    return pa


def build_controller(controller_key: str, eval_dir, gains=None, km=None,
                     seat="off", seat_gains=None, rotor_tau=None,
                     adapt_b=False, pole_gains=None, seat_law="cross"):
    """Factory/registry for all imported controllers.

    With ``gains``, construction is handed to controller_factory.make_controller -
    the same path genetic_sweep.py uses - so a gain set copied out of an archive
    row is interpreted identically here, including the scalar-to-per-axis
    broadcasting that Kp_att/Kd_att need on the PD controller. Without gains the
    registry below is used unchanged, so the no-flag behaviour is untouched.
    """
    if gains:
        results = {
            "pd": eval_dir / "cascaded_pd" / "statistics" / "cascaded_pd_statistics.npz",
            "nonlinear": eval_dir / "nonlinear" / "statistics" / "nonlinear_statistics.npz",
            "eigen": eval_dir / "eigen" / "statistics" / "eigen_statistics.npz",
            "mpc": eval_dir / "lpv_mpc" / "statistics" / "lpv_mpc_statistics.npz",
        }[controller_key]
        carb.log_warn(f"[gains] {controller_key}: "
                      + "  ".join(f"{k}={v:g}" for k, v in sorted(gains.items())))
        return cf.make_controller(
            controller_key, gains, str(results), (0.0, 0.0, 1.5), km=km,
            seat=_make_seat(seat, seat_gains, rotor_tau, controller_key, seat_law),
            pole_adapter=_make_pole_adapter(adapt_b, pole_gains, controller_key))

    controller_registry = {
        "pd": lambda: CascadedPDController(
            target_position=(0.0, 0.0, 1.5),
            results_file=str(eval_dir / "cascaded_pd" / "statistics" / "cascaded_pd_statistics.npz"),
        ),
        "nonlinear": lambda: NonlinearController(
            trajectory_file=ARGS.trajectory,
            waypoint_radius=ARGS.waypoint_radius,
            max_lateral_accel=ARGS.max_lateral_accel,
            results_file=str(eval_dir / "nonlinear" / "statistics" / "nonlinear_statistics.npz"),
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0],
        ),
        "eigen": lambda: EigenController(
            trajectory_file=ARGS.trajectory,
            waypoint_radius=ARGS.waypoint_radius,
            max_lateral_accel=ARGS.max_lateral_accel,
            results_file=str(eval_dir / "eigen" / "statistics" / "eigen_statistics.npz"),
            seat=_make_seat(seat, seat_gains, rotor_tau, controller_key, seat_law),
            pole_adapter=_make_pole_adapter(adapt_b, pole_gains, controller_key),
        ),
        "mpc": lambda: LPVMPCController(
            trajectory_file=ARGS.trajectory,
            waypoint_radius=ARGS.waypoint_radius,
            max_lateral_accel=ARGS.max_lateral_accel,
            results_file=str(eval_dir / "lpv_mpc" / "statistics" / "lpv_mpc_statistics.npz"),
            N=12,            # Horizon Window Length
            q_weight=15.0,   # Tracking state weight
            r_weight=0.02,   # Control energy optimization weight
        )
    }

    if controller_key not in controller_registry:
        valid = ", ".join(controller_registry.keys())
        raise ValueError(f"Unknown controller '{controller_key}'. Valid options: {valid}")

    return controller_registry[controller_key]()


def _interactive() -> bool:
    """Whether there is a human on stdin to answer a prompt.

    Under ``fly.py`` and in headless batch runs there is not, so an unset option
    has to fall back to its default rather than block forever on input().
    """
    try:
        return sys.stdin.isatty()
    except Exception:
        return False


def _ask(value, banner, prompt, default, valid=None, cast=None):
    """The CLI value if given, else a prompt, else the default."""
    if value is not None:
        return value
    if not _interactive():
        return default
    if banner:
        print(banner)
    raw = input(prompt).strip()
    if not raw:
        return default
    if cast is not None:
        try:
            return cast(raw)
        except ValueError:
            return default
    raw = raw.lower()
    return raw if (valid is None or raw in valid) else default


def main():
    selected_controller = _ask(
        ARGS.controller, "Available controllers: pd, nonlinear, eigen, mpc",
        "Select controller: ", "eigen", valid=("pd", "nonlinear", "eigen", "mpc"))

    motor_moment = _ask(
        ARGS.motor_moment, None,
        "Enter motor_moment (leave empty for default): ", None, cast=float)

    failure_mode = _ask(
        ARGS.failure_mode,
        "\nRotor failure: 'manual' = FAIL ROTOR button, 'auto' = stop one the\n"
        "moment the vehicle settles, 'off' = disable. The button is present in\n"
        "both manual and auto, and the auto checkbox can be toggled in flight.",
        "Failure mode [manual/auto/off] (default manual): ", "manual",
        valid=("manual", "auto", "off"))

    failure_rotor = ARGS.rotor
    failure_seed = ARGS.seed
    if failure_mode != "off" and failure_rotor is None:
        failure_rotor = _ask(None, None,
                             "Rotor to stop 0-3 (leave empty for random): ", None, cast=int)
        if failure_rotor is None and failure_seed is None:
            failure_seed = _ask(None, None,
                                "Seed for the random choice (leave empty for none): ",
                                None, cast=int)

    desaturation = _ask(
        ARGS.desaturation,
        "\nMixer desaturation: when a rotor is asked for more than it can give,\n"
        "'uniform' scales every rotor down together and the collective collapses;\n"
        "'collective' holds lift and gives up moment instead. 'uniform' is the\n"
        "default everywhere else in this repo; 'collective' is what lets the\n"
        "vehicle climb after a rotor stops.",
        "Desaturation [uniform/collective] (default uniform): ", "uniform",
        valid=("uniform", "collective"))

    # Instantiate the template app
    pg_app = PegasusApp(selected_controller, motor_moment,
                        failure_mode=failure_mode,
                        failure_rotor=failure_rotor,
                        failure_seed=failure_seed,
                        desaturation=desaturation,
                        takeoff_first=ARGS.takeoff_first,
                        lateral=ARGS.lateral,
                        gains=ARGS.gains,
                        rotor_tau=ARGS.rotor_tau,
                        seat=ARGS.seat,
                        seat_gains=ARGS.seat_gains,
                        adapt_b=ARGS.adapt_b,
                        pole_gains=ARGS.pole_gains,
                        seat_law=ARGS.seat_law)

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
