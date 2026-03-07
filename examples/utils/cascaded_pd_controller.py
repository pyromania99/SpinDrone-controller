#!/usr/bin/env python
"""
| File: cascaded_pd_controller.py
| Description: Simple cascaded PD controller backend for Pegasus multirotors.
|
| This controller is inspired by the MuJoCo script-based controller pattern:
| 1) Outer loop (position PD) computes desired translational acceleration.
| 2) Middle conversion maps desired translational force to a desired attitude.
| 3) Inner loop (attitude PD) tracks roll/pitch and outputs body torques.
|
| Yaw heading is held from current attitude and damped in yaw-rate.
"""

import carb
import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State


class CascadedPDController(Backend):
    """Simple cascaded PD controller with rotation-based attitude mapping."""

    def __init__(
        self,
        trajectory_file: str = None,
        results_file: str = None,
        target_position=(0.0, 0.0, 1.5),
        Kp_xy=(0.10, 0.10),
        Kd_xy=(1.0, 1.0),
        Kp_z=15.0,
        Kd_z=8.0,
        Kp_att=(3.5, 3.5),
        Kd_att=(0.5, 0.5),
        Kd_yaw_rate=0.2,
        mass=1.50,
        gravity=9.81,
        max_tilt_deg=30.0,
        min_total_thrust=0.0,
        max_total_thrust=80.0,
        log_yaw_rate: bool = False,
        yaw_rate_log_period: float = 0.25,
    ):
        super().__init__(config=None)
        self.input_ref = [0.0, 0.0, 0.0, 0.0]

        self.p = np.zeros((3,))
        self.v = np.zeros((3,))
        self.w = np.zeros((3,))
        self.R: Rotation = Rotation.identity()

        self.Kp_xy = np.array(Kp_xy, dtype=float)
        self.Kd_xy = np.array(Kd_xy, dtype=float)
        self.Kp_z = float(Kp_z)
        self.Kd_z = float(Kd_z)
        self.Kp_att = np.array(Kp_att, dtype=float)
        self.Kd_att = np.array(Kd_att, dtype=float)
        self.Kd_yaw_rate = float(Kd_yaw_rate)

        self.m = float(mass)
        self.g = float(gravity)

        self.max_tilt = np.deg2rad(max_tilt_deg)
        self.min_total_thrust = float(min_total_thrust)
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

        self.log_yaw_rate = log_yaw_rate
        self.yaw_rate_log_period = max(float(yaw_rate_log_period), 0.0)
        self._last_yaw_rate_log_time = -1.0

        self.results_file = results_file
        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.desired_angles_over_time = []
        self.attitude_error_over_time = []
        self.yaw_rate_over_time = []
        self.desired_yaw_rate_over_time = []

    @staticmethod
    def read_trajectory_from_csv(file_name: str):
        return np.genfromtxt(file_name, delimiter=",")

    @staticmethod
    def vee(S):
        return np.array([-S[1, 2], S[0, 2], -S[0, 1]])

    def start(self):
        self.reset_statistics()

    def stop(self):
        if self.results_file is None:
            return

        if len(self.time_vector) == 0:
            return

        statistics = {
            "time": np.array(self.time_vector),
            "p": np.vstack(self.position_over_time),
            "desired_p": np.vstack(self.desired_position_over_time),
            "ep": np.vstack(self.position_error_over_time),
            "desired_angles": np.vstack(self.desired_angles_over_time),
            "eR": np.vstack(self.attitude_error_over_time),
            "yaw_rate": np.array(self.yaw_rate_over_time),
            "desired_yaw_rate": np.array(self.desired_yaw_rate_over_time),
        }

        np.savez(self.results_file, **statistics)
        carb.log_warn("Statistics saved to: " + self.results_file)
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
        self._last_yaw_rate_log_time = -1.0

    def update_sensor(self, sensor_type: str, data):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

    def reset(self):
        pass

    def update_state(self, state: State):
        self.p = state.position
        self.v = state.linear_velocity
        self.w = state.angular_velocity
        self.R = Rotation.from_quat(state.attitude)
        self.received_first_state = True

    def input_reference(self):
        return self.input_ref

    def _reference_from_trajectory(self):
        p_ref = self.hover_target.copy()
        v_ref = np.zeros((3,))
        a_ref = np.zeros((3,))

        if self.trajectory is None:
            return p_ref, v_ref, a_ref

        if self.index < self.max_index - 1 and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1

        row = self.trajectory[self.index]

        p_ref = np.array([row[1], row[2], row[3]])
        if row.shape[0] >= 7:
            v_ref = np.array([row[4], row[5], row[6]])
        if row.shape[0] >= 10:
            a_ref = np.array([row[7], row[8], row[9]])

        return p_ref, v_ref, a_ref

    def update(self, dt: float):
        if not self.received_first_state:
            return

        self.total_time += dt

        p_ref, v_ref, a_ref = self._reference_from_trajectory()

        # Outer-loop position PD in world frame
        e_p = p_ref - self.p
        e_v = v_ref - self.v

        acc_xy_des = self.Kp_xy * e_p[:2] + self.Kd_xy * e_v[:2] + a_ref[:2]
        a_z_des = self.Kp_z * e_p[2] + self.Kd_z * e_v[2] + a_ref[2]

        # Enforce max tilt in a rotation-consistent way by clamping lateral acceleration
        # magnitude to g*tan(max_tilt), independent of current yaw.
        max_lat_acc = self.g * np.tan(self.max_tilt)
        lat_acc_norm = np.linalg.norm(acc_xy_des)
        if lat_acc_norm > max_lat_acc and lat_acc_norm > 1e-6:
            acc_xy_des = acc_xy_des * (max_lat_acc / lat_acc_norm)

        # Build the desired force in world frame. Computing attitude from this vector
        # is more robust than manual roll/pitch sign mappings across frame conventions.
        F_des = self.m * np.array([acc_xy_des[0], acc_xy_des[1], self.g + a_z_des])
        F_des_norm = np.linalg.norm(F_des)
        if F_des_norm < 1e-6:
            F_des = np.array([0.0, 0.0, self.m * self.g])
            F_des_norm = np.linalg.norm(F_des)

        Z_b_des = F_des / F_des_norm

        # Keep current yaw as heading reference while controlling only roll/pitch.
        _, _, yaw = self.R.as_euler("xyz", degrees=False)
        X_c_des = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        Y_tmp = np.cross(Z_b_des, X_c_des)
        Y_tmp_norm = np.linalg.norm(Y_tmp)
        if Y_tmp_norm < 1e-6:
            X_c_des = np.array([1.0, 0.0, 0.0])
            Y_tmp = np.cross(Z_b_des, X_c_des)
            Y_tmp_norm = np.linalg.norm(Y_tmp)
            if Y_tmp_norm < 1e-6:
                Y_tmp = np.array([0.0, 1.0, 0.0])
                Y_tmp_norm = 1.0

        Y_b_des = Y_tmp / Y_tmp_norm
        X_b_des = np.cross(Y_b_des, Z_b_des)
        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]

        Z_B = self.R.as_matrix()[:, 2]
        u_1 = np.clip(F_des @ Z_B, self.min_total_thrust, self.max_total_thrust)

        # Inner-loop attitude PD: control roll/pitch with yaw-rate damping.
        R_cur = self.R.as_matrix()

        e_R = 0.5 * self.vee((R_des.T @ R_cur) - (R_cur.T @ R_des))

        tau_x = -self.Kp_att[0] * e_R[0] - self.Kd_att[0] * self.w[0]
        tau_y = -self.Kp_att[1] * e_R[1] - self.Kd_att[1] * self.w[1]
        tau_z = -self.Kd_yaw_rate * self.w[2]
        tau = np.array([tau_x, tau_y, tau_z])

        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)

        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p.copy())
        self.desired_position_over_time.append(p_ref.copy())
        self.position_error_over_time.append(e_p.copy())
        desired_roll, desired_pitch, desired_yaw = Rotation.from_matrix(R_des).as_euler("xyz", degrees=False)
        desired_roll = np.clip(desired_roll, -self.max_tilt, self.max_tilt)
        desired_pitch = np.clip(desired_pitch, -self.max_tilt, self.max_tilt)
        self.desired_angles_over_time.append(np.array([desired_roll, desired_pitch, desired_yaw]))
        self.attitude_error_over_time.append(e_R.copy())
        self.yaw_rate_over_time.append(self.w[2])
        self.desired_yaw_rate_over_time.append(0.0)

        if self.log_yaw_rate and self.yaw_rate_log_period >= 0.0:
            should_log = self._last_yaw_rate_log_time < 0.0
            if not should_log:
                should_log = (self.total_time - self._last_yaw_rate_log_time) >= self.yaw_rate_log_period

            if should_log:
                carb.log_info(
                    "[YawRate] "
                    f"t={self.total_time:7.3f}s "
                    f"w_z={self.w[2]:+7.4f} rad/s "
                    "ref=+0.0000 rad/s"
                )
                self._last_yaw_rate_log_time = self.total_time
