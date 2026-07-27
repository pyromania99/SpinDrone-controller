#!/usr/bin/env python
"""
| File: lpv_mpc_controller.py
| Description: LPV-MPC Predictive Inner-Loop Controller with a PID Outer Position Loop.
"""

import carb
import numpy as np
import osqp
from scipy import sparse

from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State


class LPVMPCController(Backend):
    """Linear Parameter-Varying Model Predictive Controller with Outer-Loop Integral Tracking."""

    def __init__(
        self,
        trajectory_file: str = None,
        results_file: str = None,
        target_position=(0.0, 0.0, 1.5),
        kf=2.5e-5,
        km=6.0e-7,
        mass=1.2,
        gravity=9.81,
        
        # --- Outer Loop Position Gains ---
        kp_pos=0.50,
        kd_pos=1.0,
        ki_pos=0.05,       # New: Lateral Integral Gain
        
        kp_z=15.0,
        kd_z=8.0,
        ki_z=2.0,          # New: Altitude Integral Gain
        
        # Anti-Windup Saturation Limits
        int_max_xy=2.0,    # Max lateral integral correction acceleration
        int_max_z=4.0,     # Max vertical integral correction thrust
        
        # --- Inner Loop Parameters ---
        alpha=0.5,    
        beta=0.6,     
        Ixx=0.01,
        Iyy=0.01,
        Izz=0.02,
        
        # --- MPC Hyperparameters ---
        N=12,         
        q_weight=10.0, 
        r_weight=0.01, 
        s_weight=0.1,  
        
        max_total_thrust=80.0,
        max_torque=5.0, 
    ):
        super().__init__(config=None)

        self.input_ref = [0.0, 0.0, 0.0, 0.0]

        # States and Constants
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
        self.ki_pos = float(ki_pos)
        
        self.kp_z = float(kp_z)
        self.kd_z = float(kd_z)
        self.ki_z = float(ki_z)
        
        self.int_max_xy = float(int_max_xy)
        self.int_max_z = float(int_max_z)

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.Ixx = float(Ixx)
        self.Iyy = float(Iyy)
        self.Izz = float(Izz)
        
        self.C = (self.Izz - self.Iyy) / self.Ixx

        self.N = int(N)
        self.q_weight = float(q_weight)
        self.r_weight = float(r_weight)
        self.s_weight = float(s_weight)
        self.max_torque = float(max_torque)
        
        self.max_total_thrust = float(max_total_thrust)
        self.hover_target = np.array(target_position, dtype=float)

        # --- Integral Accumulator States ---
        self.pos_int_world = np.zeros((3,))

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

        self.results_file = results_file
        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.yaw_rate_over_time = []
        # Full body-frame rate [p, q, r] (FLU), not just yaw. The roll/pitch
        # rates are what identify a gyroscopic coning divergence downstream:
        # under spin the mode shows up as p and q growing in quadrature long
        # before position error moves. "angular_velocity" is the key
        # sweep_metrics.compute_metrics already looks for, so logging it also
        # activates the `oscillation` metric, which was permanently NaN.
        self.angular_velocity_over_time = []
        self.thrust_over_time = []          
        self.torque_over_time = []          
        self.motor_thrusts_over_time = []   

    @staticmethod
    def read_trajectory_from_csv(file_name: str):
        return np.genfromtxt(file_name, delimiter=",")

    def start(self):
        self.reset_statistics()

    def stop(self):
        if self.results_file is None or len(self.time_vector) == 0:
            return

        statistics = {
            "time": np.array(self.time_vector),
            "p": np.vstack(self.position_over_time),
            "desired_p": np.vstack(self.desired_position_over_time),
            "ep": np.vstack(self.position_error_over_time),
            "yaw_rate": np.array(self.yaw_rate_over_time),
            "angular_velocity": np.vstack(self.angular_velocity_over_time)
            if self.angular_velocity_over_time else np.zeros((0, 3)),
            "thrust": np.array(self.thrust_over_time),
            "torque": np.vstack(self.torque_over_time) if self.torque_over_time else np.zeros((0, 3)),
            "motor_thrusts": np.vstack(self.motor_thrusts_over_time) if self.motor_thrusts_over_time else np.zeros((0, 4)),
        }
        import os
        os.makedirs(os.path.dirname(self.results_file), exist_ok=True)
        np.savez(self.results_file, **statistics)
        carb.log_warn("MPC Statistics saved to: " + self.results_file)
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
        self.pos_int_world = np.zeros((3,))
        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.yaw_rate_over_time = []
        # Full body-frame rate [p, q, r] (FLU), not just yaw. The roll/pitch
        # rates are what identify a gyroscopic coning divergence downstream:
        # under spin the mode shows up as p and q growing in quadrature long
        # before position error moves. "angular_velocity" is the key
        # sweep_metrics.compute_metrics already looks for, so logging it also
        # activates the `oscillation` metric, which was permanently NaN.
        self.angular_velocity_over_time = []
        self.thrust_over_time = []
        self.torque_over_time = []
        self.motor_thrusts_over_time = []
        self.prev_error_world_x = 0.0
        self.prev_error_world_y = 0.0
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

    def input_reference(self):
        return self.input_ref

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

        # ---- Outer Loop Position PID Loop ----
        pos_error_world = target_pos - self.p
        
        if self._first_update:
            self.prev_error_world_x = pos_error_world[0]
            self.prev_error_world_y = pos_error_world[1]
            self.prev_error_world_z = pos_error_world[2]
            self._first_update = False
        
        # 1. Update Integrals using rectangular integration
        self.pos_int_world += pos_error_world * dt_safe
        
        # 2. Apply Anti-Windup Clamping
        self.pos_int_world[0] = np.clip(self.pos_int_world[0], -self.int_max_xy, self.int_max_xy)
        self.pos_int_world[1] = np.clip(self.pos_int_world[1], -self.int_max_xy, self.int_max_xy)
        self.pos_int_world[2] = np.clip(self.pos_int_world[2], -self.int_max_z, self.int_max_z)

        error_rate_world_x = (pos_error_world[0] - self.prev_error_world_x) / dt_safe
        error_rate_world_y = (pos_error_world[1] - self.prev_error_world_y) / dt_safe
        self.prev_error_world_x = pos_error_world[0]
        self.prev_error_world_y = pos_error_world[1]

        # Desired accelerations with added integral terms
        acc_world_x_desired = (self.kp_pos * pos_error_world[0] + 
                               self.kd_pos * error_rate_world_x + 
                               self.ki_pos * self.pos_int_world[0])
                               
        acc_world_y_desired = (self.kp_pos * pos_error_world[1] + 
                               self.kd_pos * error_rate_world_y + 
                               self.ki_pos * self.pos_int_world[1])

        # Small-angle transform mapping acceleration to target kinematics
        desired_roll = ((np.sin(yaw) * acc_world_x_desired) - (np.cos(yaw) * acc_world_y_desired)) / self.g
        desired_pitch = ((np.cos(yaw) * acc_world_x_desired) + (np.sin(yaw) * acc_world_y_desired)) / self.g

        # Kinematic references for the inner loop MPC
        p_ref = 4.0 * (desired_roll - roll)
        q_ref = 4.0 * (desired_pitch - pitch)

        # ---- Altitude PID Loop Allocation ----
        z_error = pos_error_world[2]
        z_vel = self.v[2]
        
        cos_roll = np.cos(roll)
        cos_pitch = np.cos(pitch)
        tilt_den = max(abs(cos_roll * cos_pitch), 1e-3)
        
        base_thrust = (self.m * self.g) / (4.0 * tilt_den)
        
        # Added vertical integral correction term
        altitude_correction = (self.kp_z * z_error - 
                               self.kd_z * z_vel + 
                               self.ki_z * self.pos_int_world[2]) / 4.0

        # ---- Inner Loop LPV MPC Formulation ----
        p, q, r = self.w[0], self.w[1], self.w[2]

        A_c = np.array([
            [-self.alpha,  self.C * r],
            [-self.C * r, -self.alpha]
        ])
        B_c = np.array([
            [1.0 / self.Ixx, 0.0],
            [0.0, 1.0 / self.Iyy]
        ])

        A_d = np.eye(2) + A_c * dt_safe
        B_d = B_c * dt_safe

        Nx, Nu = 2, 2
        N = self.N

        Px = np.zeros((Nx * N, Nx))
        Hu = np.zeros((Nx * N, Nu * N))

        A_power = np.eye(2)
        for i in range(N):
            A_power = A_power @ A_d
            Px[i*Nx:(i+1)*Nx, :] = A_power
            for j in range(i + 1):
                if j == 0:
                    mat = np.eye(2)
                else:
                    mat = np.eye(2)
                    for k in range(j):
                        mat = mat @ A_d
                Hu[i*Nx:(i+1)*Nx, (i-j)*Nu:(i-j+1)*Nu] = mat @ B_d

        Q_bar = sparse.kron(sparse.eye(N), sparse.diags([self.q_weight, self.q_weight])).tocsr()
        R_bar = sparse.kron(sparse.eye(N), sparse.diags([self.r_weight, self.r_weight])).tocsr()

        X_ref = np.tile(np.array([[p_ref], [q_ref]]), (N, 1))
        x_k = np.array([[p], [q]])

        H = Hu.T @ Q_bar @ Hu + R_bar
        F = Hu.T @ Q_bar @ (Px @ x_k - X_ref)

        u_min = np.tile(np.array([[-self.max_torque], [-self.max_torque]]), (N, 1))
        u_max = np.tile(np.array([[self.max_torque], [self.max_torque]]), (N, 1))

        prob = osqp.OSQP()
        A_constraints = sparse.eye(Nu * N, format='csc')
        prob.setup(sparse.csc_matrix(H), F, A_constraints, u_min.flatten(), u_max.flatten(), verbose=False)
        res = prob.solve()

        if res.info.status == 'solved':
            u_opt = res.x[:Nu]
        else:
            u_opt = np.array([0.0, 0.0])

        roll_correction = float(u_opt[0])
        pitch_correction = float(u_opt[1])

        # Passive yaw estimation tracking layout
        u1 = base_thrust + altitude_correction - roll_correction + pitch_correction
        u2 = base_thrust + altitude_correction + roll_correction + pitch_correction
        u3 = base_thrust + altitude_correction + roll_correction - pitch_correction
        u4 = base_thrust + altitude_correction - roll_correction - pitch_correction

        motor_thrusts = np.maximum(np.array([u1, u2, u3, u4]), 0.0)
        total_thrust = float(np.clip(np.sum(motor_thrusts), 0.0, self.max_total_thrust))
        tau = np.array([roll_correction, pitch_correction, 0.0])

        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(total_thrust, tau)

        # Log Data Trace Elements
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p.copy())
        self.desired_position_over_time.append(target_pos.copy())
        self.position_error_over_time.append(pos_error_world.copy())
        self.yaw_rate_over_time.append(self.w[2])
        self.angular_velocity_over_time.append(np.asarray(self.w, dtype=float).copy())
        self.thrust_over_time.append(total_thrust)
        self.torque_over_time.append(tau.copy())
        self.motor_thrusts_over_time.append(motor_thrusts.copy())
