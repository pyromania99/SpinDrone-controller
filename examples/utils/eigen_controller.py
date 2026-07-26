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
        Kp_att=4.68,
        Kd_att=1.0,
        max_total_thrust=80.0,
        wn_mdyn=13.3,
        b_mdyn=6.720,
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
        self.desired_yaw_rate_over_time = []
        self.thrust_over_time = []          # Total lumped thrust scalar [N]
        self.torque_over_time = []          # [roll, pitch, yaw] control torques [N·m]
        self.motor_thrusts_over_time = []   # [u1, u2, u3, u4] output allocations [N]

    def _sync_physical_params_from_vehicle(self):
        """Try to source mass/inertia from the vehicle model; keep defaults on failure."""
        if self.vehicle is None:
            return

        try:
            stage_prefix = getattr(self.vehicle, "_stage_prefix", "")
            body_path = stage_prefix + "/body"

            body_prim = self.vehicle._current_stage.GetPrimAtPath(body_path)
            if not body_prim or not body_prim.IsValid():
                return

            mass_attr = body_prim.GetAttribute("physics:mass")
            if mass_attr and mass_attr.HasAuthoredValueOpinion():
                mass_value = mass_attr.Get()
                if mass_value is not None and mass_value > 0.0:
                    self.m = float(mass_value)

            inertia_attr = body_prim.GetAttribute("physics:diagonalInertia")
            if inertia_attr and inertia_attr.HasAuthoredValueOpinion():
                inertia = inertia_attr.Get()
                if inertia is not None and len(inertia) == 3:
                    ixx, iyy, izz = float(inertia[0]), float(inertia[1]), float(inertia[2])
                    if ixx > 0.0 and iyy > 0.0 and izz > 0.0:
                        self.Ixx = ixx
                        self.Iyy = iyy
                        self.Izz = izz
        except Exception as error:
            carb.log_warn(f"[EigenController] Could not sync mass/inertia from vehicle model: {error}")

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
            "desired_yaw_rate": np.array(self.desired_yaw_rate_over_time),
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

        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.desired_angles_over_time = []
        self.attitude_error_over_time = []
        self.yaw_rate_over_time = []
        self.desired_yaw_rate_over_time = []
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
        desired_yaw_rate = net_thrust * self.km / (self.kf * 0.3)

        M_des = -M_dyn @ np.array([
            [desired_roll_rate],
            [desired_pitch_rate],
            [desired_yaw_rate],
        ])

        M = np.hstack([M_dyn, M_des])

        A_r = np.array([
            [-self.alpha, r, 0.0, 0.0],
            [-r, -self.alpha, 0.0, 0.0],
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
        self.desired_yaw_rate_over_time.append(desired_yaw_rate)
        # Applied, not commanded — matches motor_thrusts above.
        self.thrust_over_time.append(float(np.sum(motor_thrusts)))
        self.torque_over_time.append(tau.copy())
        self.motor_thrusts_over_time.append(motor_thrusts.copy())
