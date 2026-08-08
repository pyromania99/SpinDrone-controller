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
from applied_thrust import applied_rotor_forces
import vehicle_params


class EigenController(Backend):
    """Eigen-dynamics-based position and attitude-rate backend controller."""

    def __init__(
        self,
        trajectory_file: str = None,
        results_file: str = None,
        target_position=(0.0, 0.0, 1.5),
        kf=2.5e-5,
        km=6.0e-7,
        mass=1.2,
        gravity=9.81,
        kp_pos=0.50,
        kd_pos=1.0,
        kp_z=15.0,
        kd_z=8.0,
        alpha=0.5,
        beta=0.6,
        Ixx=0.01,
        Iyy=0.01,
        Izz=0.02,
        Kp_att=7,
        Kd_att=1.0,
        max_total_thrust=80.0,
        wn_mdyn=18,
        b_mdyn=10.2,
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

        self.max_total_thrust = float(max_total_thrust)
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
        }

        import os
        os.makedirs(os.path.dirname(self.results_file), exist_ok=True)
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
        p_ref = self.hover_target.copy()
        if self.trajectory is None:
            return p_ref
        if self.index < self.max_index - 1 and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1
        row = self.trajectory[self.index]
        return np.array([row[1], row[2], row[3]])

    def update(self, dt: float):
        if not self.received_first_state:
            return

        self.total_time += dt
        dt_safe = max(float(dt), 1e-6)

        target_pos = self._reference_from_trajectory()

        roll, pitch, yaw = self._euler_from_quat_xyzw(self.q)

        pos_error_world = target_pos - self.p

        # On the very first step, seed prev_error from current error so that the
        # finite-difference derivative is zero rather than (error - 0) / dt,
        # which would produce an enormous phantom velocity spike at t=0.
        if self._first_update:
            self.prev_error_world_x = pos_error_world[0]
            self.prev_error_world_y = pos_error_world[1]

        error_rate_world_x = (pos_error_world[0] - self.prev_error_world_x) / dt_safe
        error_rate_world_y = (pos_error_world[1] - self.prev_error_world_y) / dt_safe
        self.prev_error_world_x = pos_error_world[0]
        self.prev_error_world_y = pos_error_world[1]

        acc_world_x_desired = self.kp_pos * pos_error_world[0] + self.kd_pos * error_rate_world_x
        acc_world_y_desired = self.kp_pos * pos_error_world[1] + self.kd_pos * error_rate_world_y

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

        M_dyn = np.array([
            [-self.wn_mdyn, -self.b_mdyn, 0.0],
            [ self.b_mdyn, -self.wn_mdyn, 0.0],
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
        total_thrust = float(np.clip(np.sum(commanded_thrusts), 0.0, self.max_total_thrust))  # commanded
        tau = np.array([roll_correction, pitch_correction, 0.0])

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
