#!/usr/bin/env python
"""
| File: Eigen_control.py
| Description: Eigen-dynamics-based backend controller for Pegasus multirotors.
|
| Modified ONLY in the statistics tracking infrastructure to match the global 
| evaluation schema for exact 60-second multi-controller comparisons. 
| The core physics and torque generation laws remain completely untouched.
"""

import sys
from pathlib import Path

import carb
import numpy as np

from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State

# Sibling module in examples/utils. The sweeps put this directory on sys.path
# before importing the controller; the insert keeps a direct import working too.
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
from applied_thrust import applied_rotor_forces, max_rotor_force
from seat import PoleAdapter as SeatPoleAdapter
from seat import WrenchObserver as SeatWrenchObserver
import vehicle_params


class EigenController(Backend):
    """Eigen-dynamics-based position and attitude-rate backend controller."""

    # ── Default gains: the km = 6e-7 robust-tuned operating point ────────────
    #
    # Source: results/outer_robust/genetic_eigen_sweep/mm_6.0e-07, the knee of
    # the (score, effort_ac_ses) Pareto front. NSGA-II, pop 16 x 30 generations,
    # 496 evaluations, 6 genes (inner + outer position loop), 395/496 stable.
    #
    # Fitness was the WORST case over an inertia mismatch ladder — the plant
    # scaled to 1.0x, 0.5x and 0.25x of composite while the controller's model
    # was held at nominal — so these are gains that keep flying when the inertia
    # model is wrong, not gains that only look good on a perfectly known plant.
    # This configuration's per-scale scores were:
    #
    #     gamma = 1.00   score 0.0507
    #     gamma = 0.50   score 0.2156
    #     gamma = 0.25   score 0.3856   <- the reported worst case
    #
    # km stays 6.0e-7, which is what these were tuned at: on this airframe the
    # steady spin is r_ss ~ 5.16e7 * km, so km = 6e-7 is ~31 rad/s of yaw.
    #
    # Two of these sit where the PREVIOUS gene box could not reach. Kd_att's old
    # lower bound was 1.0 and the search jammed against it (95.9 % of stable
    # individuals now land below that old floor); wn_xy's old ceiling was 2.0.
    # Any archived Eigen gain set reading Kd_att = 1.0000 exactly is a wall
    # value, not an optimum.
    #
    # The nominal-tuned alternative from the same motor moment, if a run is
    # known to have a well-identified inertia and only nominal performance
    # matters (results/outer/genetic_eigen_sweep/mm_6.0e-07, score 0.0227):
    #
    #     kp_pos=4.0000, kd_pos=3.0985, Kp_att=3.8045, Kd_att=1.0000,
    #     wn_mdyn=26.6421, b_mdyn=0.0
    #
    # It scores better at gamma = 1 (0.0227 vs 0.0507) and worse everywhere
    # else, and its Kd_att is one of the wall values described above.
    #
    # mass/Ixx/Iyy/Izz below are pre-sync placeholders only —
    # _sync_physical_params_from_vehicle() overwrites all four from the vehicle
    # at start(). The real Iris composite is m = 1.6186 kg,
    # I = (0.034853, 0.031256, 0.062659).
    def __init__(
        self,
        trajectory_file: str = None,
        results_file: str = None,
        target_position=(0.0, 0.0, 1.5),
        kf=2.5e-5,
        km=6.0e-7,
        mass=1.2,
        gravity=9.81,
        kp_pos=3.1469,      # from wn_xy   = 1.7740  (kp = wn^2)
        kd_pos=3.8199,      # from zeta_xy = 1.0766  (kd = 2*zeta*wn)
        kp_z=15.0,          # not swept
        kd_z=8.0,           # not swept
        alpha=0.5,
        beta=0.6,
        Ixx=0.01,
        Iyy=0.01,
        Izz=0.02,
        Kp_att=1.1333,
        Kd_att=0.1202,      # below the old box floor of 1.0
        max_total_thrust=80.0,
        torque_max=None,
        wn_mdyn=37.5706,
        b_mdyn=0.0,         # legacy cross-coupling; see theta_mdyn
        theta_mdyn=None,    # degrees; when set, M_dyn = wn_mdyn * R(theta)
        max_lateral_accel=0.0,# [m/s^2] clamp on the resulting horizontal accel demand
        waypoint_radius=0.0,  # >0 advances the trajectory on ARRIVAL, not on the clock
        seat=None,          # examples/utils/seat.Seat, or None for no seat at all
        pole_adapter=None,  # examples/utils/seat.PoleAdapter, or None
    ):
        super().__init__(config=None)

        self.input_ref = [0.0, 0.0, 0.0, 0.0]

        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        self.w = np.zeros((3,))
        self.q = np.array([0.0, 0.0, 0.0, 1.0])

        self.kf = float(kf)
        self.km = float(km)
        self.m = float(mass)
        self.g = float(gravity)

        self.kp_pos = float(kp_pos)
        self.kd_pos = float(kd_pos)
        self.kp_z = float(kp_z)
        self.kd_z = float(kd_z)

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.Ixx = float(Ixx)
        self.Iyy = float(Iyy)
        self.Izz = float(Izz)

        self.Kp_att = float(Kp_att)
        self.Kd_att = float(Kd_att)
        self.wn_mdyn = float(wn_mdyn)
        self.b_mdyn  = float(b_mdyn)

        # M_dyn's 2x2 block as magnitude-and-angle instead of two Cartesian
        # entries.
        #
        # The block was ever only [[-a, -b], [b, -a]], which is already a
        # scaled rotation: magnitude hypot(a, b), angle atan2(b, a). Written
        # that way the two knobs are not independent - raising b to rotate the
        # block also grows its magnitude - and with a > 0, b >= 0 the angle can
        # never leave the first quadrant, so only a quarter turn is reachable.
        #
        # With theta_mdyn set, wn_mdyn is the magnitude and theta_mdyn is the
        # angle in degrees, unrestricted. The two are then independent: rotate
        # without changing how hard the block pulls, and rotate past 90 deg.
        #
        # Left None by default, in which case the block is built from
        # (wn_mdyn, b_mdyn) exactly as before - bit-identical, since the
        # conversion is the same rotation read a different way. Every archived
        # gain set therefore replays unchanged.
        self.theta_mdyn = None if theta_mdyn is None else float(theta_mdyn)

        self.max_total_thrust = float(max_total_thrust)

        # Per-axis torque ceiling [N.m], or None for the original unlimited behaviour.
        #
        # PX4 ALWAYS saturates here and this controller never did. EigenController.cpp
        # divides by MC_EIG_TRQ_MAX and constrains the result to [-1, 1], so a command
        # beyond +/-TRQ_MAX (3.843 N.m on this airframe) is simply not delivered. Every
        # PX4 divergence analysed so far sat at exactly 1.0 - saturated - which means a
        # search run without this ceiling is optimising against authority the real stack
        # does not have, and will happily pick gains that only work unsaturated.
        #
        # Left None by default so archived sweeps reproduce; genetic_sweep.py sets it.
        self.torque_max = None if torque_max is None else float(torque_max)
        self.hover_target = np.array(target_position, dtype=float)

        self.index = 0
        self.trajectory = None
        self.max_index = 1
        if trajectory_file is not None:
            self.trajectory = self.read_trajectory_from_csv(trajectory_file)
            self.max_index, _ = self.trajectory.shape

        self.total_time = 0.0
        self.received_first_state = False
        self._first_update = True

        self.prev_error_world_x = 0.0
        self.prev_error_world_y = 0.0
        self.prev_roll_error = 0.0
        self.prev_pitch_error = 0.0

        # ---- Upgraded Telemetry Logging Infrastructure ----
        self.results_file = results_file
        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.desired_angles_over_time = []
        self.attitude_error_over_time = []
        self.yaw_rate_over_time = []
        # Full body-frame rate [p, q, r] (FLU), not just yaw. The roll/pitch
        # rates are what identify a gyroscopic coning divergence downstream:
        # under spin the mode shows up as p and q growing in quadrature long
        # before position error moves. "angular_velocity" is the key
        # sweep_metrics.compute_metrics already looks for, so logging it also
        # activates the `oscillation` metric, which was permanently NaN.
        self.angular_velocity_over_time = []
        # Not a yaw reference — see unused_yaw_rate_cmd in update().
        self.unused_yaw_rate_cmd_over_time = []
        self.thrust_over_time = []          # Total lumped thrust scalar [N]
        self.torque_over_time = []          # [roll, pitch, yaw] control torques [N·m]
        self.motor_thrusts_over_time = []   # [u1, u2, u3, u4] output allocations [N]

        # Command-frame rotation of the wrench (brief v2). None means there is no
        # seat: update() never calls into seat.py, so an A/B against it compares two
        # code paths rather than one path with a flag in it.
        self.max_lateral_accel = max(float(max_lateral_accel), 0.0)
        self.waypoint_radius = max(float(waypoint_radius), 0.0)
        self.seat = seat
        # Adaptive pole angle (brief v3 §4). Independent of the seat: it rotates
        # nothing, it moves M_dyn's angle. Either, both, or neither may be present.
        self.pole_adapter = pole_adapter
        # The observer is a MEASUREMENT: it rotates nothing, feeds nothing back, and
        # touches no control path (see WrenchObserver.observe - it returns an angle and
        # mutates only its own history). So it runs on EVERY arm, including seat OFF.
        #
        # Until 2026-09-17 it was built only when a seat or a pole adapter existed, which
        # kept the seat-OFF arm out of seat.py entirely - a deliberate clean A/B, noted
        # above. The cost was that alpha, the thing the seat is supposed to be driving to
        # zero, was UNRECORDED on exactly the arm you want to compare against: the OFF
        # column of every seat figure was empty and there was no way to tell whether the
        # seat was reducing a misalignment that was there in the first place. Measuring it
        # on both arms is worth more than the A/B purity, because the measurement provably
        # cannot change the flight - which the re-fly checks by comparing the OFF arm
        # against its previous log.
        self.seat_observer = SeatWrenchObserver()
        self._v_prev = None                 # world xy velocity, for the extremum law
        self.seat_theta_over_time = []      # [rad] angle applied to the wrench
        self.seat_alpha_over_time = []      # [rad] measured misalignment, every arm
        self.seat_stepped_over_time = []    # 1 = the law stepped, 0 = gated, NaN = no law
        self.pole_theta_over_time = []      # [rad] adaptive offset on M_dyn's angle
        self.pole_alpha_over_time = []      # [rad] alpha_b, NaN when gated
        self.pole_cost_over_time = []       # [rad] extremum law's lateral-tracking cost

    def _sync_physical_params_from_vehicle(self):
        """Install the vehicle's real mass/inertia; keep defaults on failure.

        This used to read ``physics:mass`` and ``physics:diagonalInertia``
        straight off the ``/body`` prim, which is one link of a five-link
        articulation. It therefore missed the four rotors entirely: 0.1186 kg
        of vehicle, and their parallel-axis contribution to every inertia axis.
        Measured against the composite truth, the model this controller flew
        with was off by −7.3 % in mass, −16.4 % in Ixx, −6.8 % in Iyy and
        −11.9 % in Izz. See examples/utils/vehicle_params.py.

        Note for anyone reading old archives: this moves the operating point.
        The gains the sweeps found are the best gains for a controller that
        believed it weighed 1.5 kg with the body link's inertia; they are not
        re-derived by this change, only re-applied to a correct model.
        """
        if self.vehicle is None:
            return

        props = vehicle_params.composite_mass_properties(self.vehicle)
        if props is None:
            carb.log_warn(
                "[EigenController] Could not read mass/inertia from the physics "
                f"interface; keeping m={self.m:.4f}, "
                f"I=({self.Ixx:.5f}, {self.Iyy:.5f}, {self.Izz:.5f})")
            return

        self.m = props["mass"]
        self.Ixx = props["Ixx"]
        self.Iyy = props["Iyy"]
        self.Izz = props["Izz"]
        carb.log_warn(f"[EigenController] model synced: {vehicle_params.describe(props)}")

    @staticmethod
    def read_trajectory_from_csv(file_name: str):
        return np.genfromtxt(file_name, delimiter=",")

    def start(self):
        self._sync_physical_params_from_vehicle()
        self.reset_statistics()

    def stop(self):
        if self.results_file is None or len(self.time_vector) == 0:
            return

        # Explicit key mapping added to align flawlessly with compare_controllers_60s.py
        statistics = {
            "time": np.array(self.time_vector),
            "p": np.vstack(self.position_over_time),
            "desired_p": np.vstack(self.desired_position_over_time),
            "ep": np.vstack(self.position_error_over_time),
            "desired_angles": np.vstack(self.desired_angles_over_time),
            "eR": np.vstack(self.attitude_error_over_time),
            "yaw_rate": np.array(self.yaw_rate_over_time),
            "angular_velocity": np.vstack(self.angular_velocity_over_time)
            if self.angular_velocity_over_time else np.zeros((0, 3)),
            # Named to match the variable: this is NOT a tracked yaw reference.
            "unused_yaw_rate_cmd": np.array(self.unused_yaw_rate_cmd_over_time),
            "thrust": np.array(self.thrust_over_time),
            "torque": np.vstack(self.torque_over_time) if self.torque_over_time else np.zeros((0, 3)),
            "motor_thrusts": np.vstack(self.motor_thrusts_over_time) if self.motor_thrusts_over_time else np.zeros((0, 4)),
            "seat_theta": np.array(self.seat_theta_over_time),
            "seat_alpha": np.array(self.seat_alpha_over_time),
            "seat_stepped": np.array(self.seat_stepped_over_time),
            "pole_theta": np.array(self.pole_theta_over_time),
            "pole_alpha": np.array(self.pole_alpha_over_time),
            "pole_cost": np.array(self.pole_cost_over_time),
        }

        import os
        os.makedirs(os.path.dirname(self.results_file), exist_ok=True)
        # Create the directory before writing. A controller is handed an
        # absolute results path by its caller, and nothing guarantees the
        # parent exists - a fresh checkout, or a results tree that moved,
        # both leave np.savez raising FileNotFoundError at the very end of
        # an otherwise good flight, losing the whole run.
        Path(self.results_file).parent.mkdir(parents=True, exist_ok=True)
        np.savez(self.results_file, **statistics)
        carb.log_warn("Eigen Controller Statistics saved to: " + self.results_file)
        self.reset_statistics()

    def reset_statistics(self):
        self.index = 0
        self.total_time = 0.0
        # Wait for a fresh state before logging or acting again.
        #
        # This flag was set once, in __init__, and never cleared. The sweeps
        # reuse one controller object across many passes, and the backend's
        # update() runs before update_state() delivers the post-teleport state
        # — so every pass after the first logged one sample carrying the
        # PREVIOUS pass's position, velocity, attitude and body rates before
        # the real ones arrived. That single stale row is a genuine artefact:
        # it made p[0] read as wherever the last run ended (which looks exactly
        # like a failed reset), it seeded prev_error_world_* with a bogus error
        # producing a one-step derivative spike, and it corrupted any metric
        # reduced over the raw arrays — min-altitude most of all, since the
        # stale row is often a crashed drone lying on the floor.
        self.received_first_state = False

        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.desired_angles_over_time = []
        self.attitude_error_over_time = []
        self.yaw_rate_over_time = []
        # Full body-frame rate [p, q, r] (FLU), not just yaw. The roll/pitch
        # rates are what identify a gyroscopic coning divergence downstream:
        # under spin the mode shows up as p and q growing in quadrature long
        # before position error moves. "angular_velocity" is the key
        # sweep_metrics.compute_metrics already looks for, so logging it also
        # activates the `oscillation` metric, which was permanently NaN.
        self.angular_velocity_over_time = []
        # Not a yaw reference — see unused_yaw_rate_cmd in update().
        self.unused_yaw_rate_cmd_over_time = []
        self.thrust_over_time = []
        self.torque_over_time = []
        self.motor_thrusts_over_time = []
        self.seat_theta_over_time = []
        self.seat_alpha_over_time = []
        self.seat_stepped_over_time = []
        self._v_prev = None
        self.pole_theta_over_time = []
        self.pole_alpha_over_time = []
        self.pole_cost_over_time = []
        if self.pole_adapter is not None:
            self.pole_adapter.reset()
        # The sweeps reuse one controller across passes, so a converged angle must
        # not survive into the next flight.
        if self.seat is not None:
            self.seat.reset()
        # Unconditional: the observer now runs on every arm, so a seat-OFF pass in a
        # sweep would otherwise carry the previous pass's rate history into its first
        # few ticks.
        self.seat_observer.reset()

        self.prev_error_world_x = 0.0
        self.prev_error_world_y = 0.0
        self.prev_roll_error = 0.0
        self.prev_pitch_error = 0.0
        self._first_update = True

    def update_sensor(self, sensor_type: str, data): pass
    def update_graphical_sensor(self, sensor_type: str, data): pass
    def reset(self): pass

    def update_state(self, state: State):
        self.p = state.position
        self.v = state.linear_velocity
        self.w = state.angular_velocity
        self.q = np.array(state.attitude, dtype=float)
        self.received_first_state = True

    @staticmethod
    def _euler_from_quat_xyzw(quat):
        qx, qy, qz, qw = quat
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def input_reference(self):
        return self.input_ref

    def _reference_from_trajectory(self):
        """Current position reference: hover target, timed trajectory, or waypoints.

        TWO ADVANCE RULES, and the difference matters under spin.

          TIME (waypoint_radius = 0, the original): the reference marches on the CSV's
          own clock whatever the vehicle does. If the vehicle falls behind, the
          reference keeps going, the error grows, the demand grows with it, and the
          thing diverges. Measured at 25 rad/s with 20 ms of rotor lag: the climb held
          to 5 mm, then the timed translate ran away and the vehicle was on the ground
          by t = 20 s with 3.2 m of error.

          WAYPOINT (waypoint_radius > 0): advance only once the vehicle is actually
          within waypoint_radius of the current point. The reference WAITS. That caps
          the position error at roughly the acceptance radius by construction, so the
          tilt demand stays bounded no matter how slowly the vehicle is tracking - which
          is the property a spinning, lag-limited vehicle needs.
        """
        p_ref = self.hover_target.copy()
        if self.trajectory is None:
            return p_ref

        if self.waypoint_radius > 0.0:
            # Advance past every point already satisfied, so a cluster of close waypoints
            # does not take one tick each.
            while self.index < self.max_index - 1:
                row = self.trajectory[self.index]
                target = np.array([row[1], row[2], row[3]])
                if np.linalg.norm(np.asarray(self.p, dtype=float) - target) > self.waypoint_radius:
                    break
                self.index += 1
        elif self.index < self.max_index - 1 and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1

        row = self.trajectory[self.index]
        return np.array([row[1], row[2], row[3]])

    def mdyn_polar(self):
        """(magnitude, angle_in_radians) of M_dyn's 2x2 block.

        theta_mdyn set  -> (wn_mdyn, radians(theta_mdyn)), the two independent.
        theta_mdyn None -> (hypot(wn, b), atan2(b, wn)), which reproduces the
        old [[-a,-b],[b,-a]] block exactly.
        """
        if self.theta_mdyn is not None:
            return float(self.wn_mdyn), float(np.radians(self.theta_mdyn))
        return (float(np.hypot(self.wn_mdyn, self.b_mdyn)),
                float(np.arctan2(self.b_mdyn, self.wn_mdyn)))

    def update(self, dt: float):
        if not self.received_first_state:
            return

        self.total_time += dt
        dt_safe = max(float(dt), 1e-6)

        target_pos = self._reference_from_trajectory()

        roll, pitch, yaw = self._euler_from_quat_xyzw(self.q)

        pos_error_world = target_pos - self.p

        # OUTER-LOOP AUTHORITY LIMIT. The position PD is a pure gain on distance, so a
        # far goal asks for an acceleration the airframe cannot produce: at kp_pos =
        # 3.1469 a 20 m error demands 63 m/s^2, against a real ceiling of g*tan(tilt) ~
        # 5.7 m/s^2 at 30 deg. The controller then commands a tilt it cannot hold, the
        # thrust vector leaves the vertical, altitude collapses and the spin decays -
        # which is how a distant waypoint turns into a crash that looks like a control
        # failure but is really a saturation failure. Clamping the ERROR (not the output)
        # keeps the demand inside the envelope while preserving its DIRECTION, so a far
        # goal becomes 'fly at the cruise this tilt affords', not 'fall over trying'.
        pos_error_unclamped = np.array(pos_error_world, dtype=float)

        # On the very first step, seed prev_error from current error so that the
        # finite-difference derivative is zero rather than (error - 0) / dt,
        # which would produce an enormous phantom velocity spike at t=0.
        if self._first_update:
            self.prev_error_world_x = pos_error_unclamped[0]
            self.prev_error_world_y = pos_error_unclamped[1]

        error_rate_world_x = (pos_error_unclamped[0] - self.prev_error_world_x) / dt_safe
        error_rate_world_y = (pos_error_unclamped[1] - self.prev_error_world_y) / dt_safe
        self.prev_error_world_x = pos_error_unclamped[0]
        self.prev_error_world_y = pos_error_unclamped[1]

        acc_world_x_desired = self.kp_pos * pos_error_world[0] + self.kd_pos * error_rate_world_x
        acc_world_y_desired = self.kp_pos * pos_error_world[1] + self.kd_pos * error_rate_world_y

        # OUTER-LOOP AUTHORITY LIMIT, applied to the OUTPUT only.
        #
        # The position PD is a pure gain on distance, so a far goal asks for an
        # acceleration the airframe cannot make: at kp_pos = 3.1469 a 20 m error demands
        # 63 m/s^2 against a real ceiling of g*tan(tilt) ~ 5.7 m/s^2 at 30 deg. The
        # controller then commands a tilt it cannot hold, the thrust vector leaves the
        # vertical, altitude collapses and the spin decays - a saturation failure wearing
        # the costume of a control failure.
        #
        # Limiting the OUTPUT rather than the error is what keeps the PD intact: clamping
        # the error would also flatten the derivative it is taken from, removing the
        # damping. Measured when that was tried: 8.6 m/s on a 30 m leg instead of a 1.2 m/s
        # cruise, and a 2 m overshoot past the goal.
        #
        # Scaled as a VECTOR so the commanded direction survives. Clipping per-axis would
        # rotate the demand, which is precisely the error the rest of this file exists to
        # avoid.
        if self.max_lateral_accel > 0.0:
            an = float(np.hypot(acc_world_x_desired, acc_world_y_desired))
            if an > self.max_lateral_accel:
                sc = self.max_lateral_accel / an
                acc_world_x_desired *= sc
                acc_world_y_desired *= sc

        desired_roll = ((np.sin(yaw) * acc_world_x_desired) - (np.cos(yaw) * acc_world_y_desired)) / self.g
        desired_pitch = ((np.cos(yaw) * acc_world_x_desired) + (np.sin(yaw) * acc_world_y_desired)) / self.g

        z_error = pos_error_world[2]
        z_vel = self.v[2]

        cos_roll = np.cos(roll)
        cos_pitch = np.cos(pitch)
        tilt_den = max(abs(cos_roll * cos_pitch), 1e-3)
        base_thrust = (self.m * self.g) / (4.0 * tilt_den)
        altitude_correction = (self.kp_z * z_error - self.kd_z * z_vel) / 4.0

        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch

        p, q, r = self.w[0], self.w[1], self.w[2]

        # Seed attitude prev_errors on the first step for the same reason as above.
        if self._first_update:
            self.prev_roll_error  = roll_error
            self.prev_pitch_error = pitch_error
            self._first_update = False

        pitch_error_rate = (pitch_error - self.prev_pitch_error) / dt_safe
        roll_error_rate = (roll_error - self.prev_roll_error) / dt_safe

        desired_pitch_rate = self.Kp_att * pitch_error + self.Kd_att * pitch_error_rate
        desired_roll_rate = self.Kp_att * roll_error + self.Kd_att * roll_error_rate

        self.prev_pitch_error = pitch_error
        self.prev_roll_error = roll_error

        # Rebuilt every step: the sweeps mutate wn_mdyn/b_mdyn/theta_mdyn on a
        # live controller between runs, so this cannot be cached at __init__.
        mag, theta = self.mdyn_polar()
        if self.pole_adapter is not None:
            theta = theta + self.pole_adapter.theta_b     # adaptive pole angle (v3 §4)
        c, s_ = np.cos(theta), np.sin(theta)
        M_dyn = np.array([
            [-mag * c, -mag * s_, 0.0],
            [ mag * s_, -mag * c, 0.0],
            [0.0, 0.0, -self.beta],
        ])

        net_thrust = base_thrust * 4.0
        # NOTE: this has NO effect on the vehicle. It enters M_des only through
        # row 2, whose correction (corrections[2]) is discarded — tau is built
        # as [roll, pitch, 0.0] because a same-direction rotor set cannot
        # produce a commanded yaw torque while also producing lift. Kept
        # because M_des must be a 3-vector for the M_dyn algebra, and named
        # "unused_" so nobody reads the logged trace as a real yaw reference.
        unused_yaw_rate_cmd = net_thrust * self.km / (self.kf * 0.3)

        M_des = -M_dyn @ np.array([
            [desired_roll_rate],
            [desired_pitch_rate],
            [unused_yaw_rate_cmd],
        ])

        M = np.hstack([M_dyn, M_des])

        # A_r is the plant's own rate dynamics, which M_eff = M - A_r cancels so
        # that what remains is the designed M_dyn. Its off-diagonal block is the
        # gyroscopic (Euler) coupling, and it must carry the sign of the real
        # inertia difference:
        #
        #     Ixx p_dot = tau_x + (Iyy - Izz) q r
        #     Iyy q_dot = tau_y + (Izz - Ixx) r p
        #
        # It used to be hardcoded as [[., +r], [-r, .]], which is the sign for
        # Izz < Ixx — a rod. Every planar multirotor has Izz ~ Ixx + Iyy, so the
        # coefficient (Iyy - Izz)/Ixx is negative (-0.91 on the Iris) and the
        # hardcoded block had it backwards. Subtracting a term of the wrong sign
        # does not cancel the gyroscopic coupling, it applies it a second time,
        # so the closed loop kept ~2x the coupling instead of none.
        #
        # The error is proportional to r, so it is invisible at hover and grows
        # without bound under spin. Pushing the resulting torque back through
        # rigid-body dynamics, against the M_dyn(w - w_cmd) this block is
        # supposed to deliver:
        #
        #     r [rad/s]      intended        as written      with this fix
        #        0        +2.070  -4.740   +2.120  -4.765   +2.120  -4.765
        #       20        +2.070  -4.740   +4.029  -0.947   +2.120  -4.765
        #       60        +2.070  -4.740   +7.847  +6.690   +2.120  -4.765
        #
        # By r = 60 rad/s the pitch channel had the wrong sign outright. The
        # rate loop's delivered-to-commanded gain, unity at hover, fell to 0.40
        # at r = 20 and 0.16 at r = 60; with the fix it is flat in r. What is
        # left over is the -alpha drag model, which is the only thing A_r is
        # still meant to add.
        #
        # Taken from the synced inertia rather than hardcoding the ratio, which
        # cancels exactly for any Ixx, Iyy, Izz — an airframe with a different
        # mass distribution, or the pre-sync constructor defaults, both stay
        # correct, and the residual above is unchanged by Ixx != Iyy. The yaw row
        # has no equivalent entry: its coupling (Ixx - Iyy) p q is bilinear in
        # the state and cannot be written in this linear-in-w form, and it
        # vanishes for the symmetric Ixx = Iyy case anyway.
        gyro_pq = (self.Iyy - self.Izz) / self.Ixx * r
        gyro_qp = (self.Izz - self.Ixx) / self.Iyy * r

        A_r = np.array([
            [-self.alpha, gyro_pq, 0.0, 0.0],
            [gyro_qp, -self.alpha, 0.0, 0.0],
            [0.0, 0.0, -self.beta, 0.0],
        ])

        M_eff = M - A_r

        B_inv = np.diag([1.0 / self.Ixx, 1.0 / self.Iyy, 1.0 / self.Izz])
        B = np.linalg.inv(B_inv)

        w_aug = np.array([[p], [q], [r], [1.0]])

        # Physics preserved completely as requested:
        C = B @ M_eff
        corrections = C @ w_aug

        roll_correction = float(corrections[0, 0])
        pitch_correction = float(corrections[1, 0])

        u1 = base_thrust + altitude_correction - roll_correction + pitch_correction
        u2 = base_thrust + altitude_correction + roll_correction + pitch_correction
        u3 = base_thrust + altitude_correction + roll_correction - pitch_correction
        u4 = base_thrust + altitude_correction - roll_correction - pitch_correction

        commanded_thrusts = np.maximum(np.array([u1, u2, u3, u4]), 0.0)

        # Clip the collective to what the airframe can actually produce, not to
        # the nominal max_total_thrust alone.
        #
        # max_total_thrust defaults to 80 N against an achievable 4 x 10.34 =
        # 41.4 N, so it is 1.93x out of reach and never binds. Measured during a
        # stopped-rotor run: the altitude PD wound up to a 52.8 N demand — 1.28x
        # what the airframe can deliver — and the allocator then saturated hard
        # trying to serve it. Asking for thrust that does not exist does not
        # produce more lift, it produces more saturation.
        #
        # max_rotor_force() reads rotor_constant and max_rotor_velocity off the
        # live thruster model, so a re-motored airframe stays correct; it falls
        # back to the Iris defaults when there is no vehicle yet. The min keeps
        # an explicitly configured lower ceiling honoured.
        ceiling = self.max_total_thrust
        if self.vehicle is not None:
            thrusters = getattr(self.vehicle, "_thrusters", None)
            n_rotors = getattr(thrusters, "_num_rotors", 4) if thrusters else 4
            ceiling = min(ceiling, n_rotors * max_rotor_force(thrusters))

        total_thrust = float(np.clip(np.sum(commanded_thrusts), 0.0, ceiling))  # commanded
        tau = np.array([roll_correction, pitch_correction, 0.0])

        # Match PX4's saturation: per-axis clip, NOT a norm-preserving scale. PX4
        # constrains each axis independently (EigenController.cpp), so a roll command over
        # the ceiling is truncated while pitch passes through - which changes the direction
        # of the applied torque, not just its size. Scaling the vector instead would be a
        # different, gentler nonlinearity and would not reproduce the real behaviour.
        if self.torque_max is not None:
            tau = np.clip(tau, -self.torque_max, self.torque_max)

        # ── the seat (brief v2) ──────────────────────────────────────────────
        #
        # xd is the acceleration a LAGLESS actuator would have produced from this
        # wrench: tau/J, componentwise because Ixx != Iyy (0.0349 vs 0.0313 on the
        # Iris) and a non-uniform scaling is not a pure magnitude change - it
        # rotates, so it must be applied before the angle is taken.
        #
        # xa is rebuilt from the measured rate by subtracting the modelled drift,
        # which is A_r's own rate block - the same matrix the feedback
        # linearization above already uses, so no new dependence enters. See
        # seat.WrenchObserver for why that reconstruction is what a vehicle can do
        # and what it costs.
        alpha_meas = float("nan")      # the MEASUREMENT, logged on every arm
        seat_stepped = float("nan")    # whether the adaptation moved theta_s this tick
        if self.seat_observer is not None:
            xd = np.array([tau[0] / self.Ixx, tau[1] / self.Iyy])
            drift_x = np.array([-self.alpha * p + gyro_pq * q,
                                gyro_qp * p - self.alpha * q])
            alpha_meas = self.seat_observer.observe(xd, (p, q), drift_x, dt)
            if self.seat is not None and self.seat.mode == self.seat.FIXED:
                self.seat.update_fixed(r)
            elif self.seat is not None:
                # The raw cross product, for the CROSS law: |xd||xa| sin(alpha),
                # built from the SAME aligned pair the angle came from.
                dx, da = self.seat_observer.last_xd, self.seat_observer.last_xa
                drive = (float(dx[0] * da[1] - dx[1] * da[0])
                         if (dx is not None and da is not None) else None)
                seat_stepped = float(self.seat.adapt(alpha_meas, r, dt, drive=drive))

            # Adaptive pole angle. Applies no rotation either way - it takes effect
            # through M_dyn on the next tick.
            if self.pole_adapter is not None:
                if getattr(self.pole_adapter, "law", "gradient") == "extremum":
                    # EXTREMUM: the pair is TRANSLATIONAL, and that is the whole point.
                    # a_cmd comes from the position loop, so unlike the GRADIENT law's
                    # M_dyn.x it does not move when theta_b moves and cannot cancel
                    # itself out of the cost. Achieved accel is differenced from the
                    # world velocity - the only such signal this backend has.
                    a_cmd = np.array([acc_world_x_desired, acc_world_y_desired])
                    if self._v_prev is None:
                        a_meas = np.zeros(2)
                    else:
                        a_meas = (np.asarray(self.v, dtype=float)[:2]
                                  - self._v_prev) / max(dt, 1e-6)
                    self._v_prev = np.asarray(self.v, dtype=float)[:2].copy()
                    self.pole_adapter.adapt_extremum(a_cmd, a_meas, r, dt)
                else:
                    M2 = np.array([[-mag * c, -mag * s_], [mag * s_, -mag * c]])
                    self.pole_adapter.adapt(M2 @ np.array([p, q]),
                                            self.seat_observer.last_xa, r, dt)

            if self.seat is not None:
                tau[:2] = self.seat.rotate(tau)
                # Re-clip: the rotation can push an axis past the ceiling, and PX4's
                # allocator would truncate it there too.
                if self.torque_max is not None:
                    tau_unclipped = tau.copy()
                    tau = np.clip(tau, -self.torque_max, self.torque_max)
                    # Feed the clip back as the saturation flag, so this rung can run the
                    # same law as PX4 (Seat::setSaturated from rp_saturated). Level 1 never
                    # set it before 2026-09-19, which meant level 2 minus level 1 measured
                    # the GATE and not PX4 - the archived PX4 arms were gated on 12-80 % of
                    # ticks while these were gated on none. Which behaviour you get is
                    # Seat(sat_gate=...) / --seat-satgate, default 1 to match PX4's archive.
                    self.seat.saturated = bool(
                        np.any(np.abs(tau_unclipped[:2]) > np.abs(tau[:2]) + 1e-12))

        # motor_thrusts logs what the vehicle ACTUALLY applies, not this
        # mixer's idealisation of it. The old code clipped each u_i at
        # max_total_thrust (80 N) — the *total* limit, ~8x the true per-rotor
        # maximum of 10.34 N — so the logged effort was a quantity the
        # simulator never produced. The allocator also saturates by scaling
        # all rotors together rather than clipping them individually, which
        # the u1..u4 mixer cannot represent. See examples/utils/applied_thrust.py.
        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(total_thrust, tau)
            motor_thrusts = applied_rotor_forces(self.vehicle, self.input_ref)
        else:
            motor_thrusts = np.zeros(4)

        # ---- Logging Arrays Capture ----
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p.copy())
        self.desired_position_over_time.append(target_pos.copy())
        self.position_error_over_time.append(pos_error_world.copy())
        self.desired_angles_over_time.append(np.array([desired_roll, desired_pitch, yaw]))
        self.attitude_error_over_time.append(np.array([roll_error, pitch_error, 0.0]))
        self.yaw_rate_over_time.append(self.w[2])
        self.angular_velocity_over_time.append(np.asarray(self.w, dtype=float).copy())
        self.unused_yaw_rate_cmd_over_time.append(unused_yaw_rate_cmd)
        # Applied, not commanded — matches motor_thrusts above.
        self.thrust_over_time.append(float(np.sum(motor_thrusts)))
        self.torque_over_time.append(tau.copy())
        self.motor_thrusts_over_time.append(motor_thrusts.copy())
        self.seat_theta_over_time.append(self.seat.theta_s if self.seat else 0.0)
        # The MEASURED misalignment, on every arm. NOT self.seat.alpha: that is a record
        # of whether the law stepped (update_fixed and every gated path write NaN over
        # it), so logging it made alpha look unmeasurable on the OFF and FIXED arms when
        # it had simply never been asked for.
        self.seat_alpha_over_time.append(alpha_meas)
        self.seat_stepped_over_time.append(seat_stepped)
        self.pole_cost_over_time.append(
            getattr(self.pole_adapter, "cost", float("nan"))
            if self.pole_adapter else float("nan"))
        pa = self.pole_adapter
        self.pole_theta_over_time.append(pa.theta_b if pa else 0.0)
        self.pole_alpha_over_time.append(pa.alpha_b if pa else float("nan"))
