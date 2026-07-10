#!/usr/bin/env python
"""
| File: lpv_mpc_controller.py
| Description: LPV-MPC Predictive Inner-Loop Controller for Pegasus multirotors
|              derived from slow yaw-rate cross-coupling assumptions.
"""

import carb
import numpy as np
import osqp
from scipy import sparse

from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State


class LPVMPCController(Backend):
    """Linear Parameter-Varying Model Predictive Controller for Roll/Pitch rates."""

    def __init__(
        self,
        trajectory_file: str = None,
        results_file: str = None,
        target_position=(0.0, 0.0, 1.5),
        kf=2.5e-5,
        km=6.0e-7,
        mass=1.2,
        gravity=9.81,
        
        # Outer loop position gains
        kp_pos=0.50,
        kd_pos=1.0,
        kp_z=15.0,
        kd_z=8.0,
        
        # Physical Parameters from Handwritten Notes
        alpha=0.5,    # Damping gain
        beta=0.6,     # Yaw damping
        Ixx=0.01,
        Iyy=0.01,
        Izz=0.02,
        
        # MPC Design Hyperparameters
        N=12,         # Prediction Horizon Steps
        q_weight=10.0, # Tracking error penalization for p and q
        r_weight=0.01, # Motor energy/torque penalty
        s_weight=0.1,  # Slew rate constraint / smooth change penalty
        
        max_total_thrust=80.0,
        max_torque=5.0, # Max physical roll/pitch torque constraint [N-m]
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
        self.kp_z = float(kp_z)
        self.kd_z = float(kd_z)

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.Ixx = float(Ixx)
        self.Iyy = float(Iyy)
        self.Izz = float(Izz)
        
        # Structural cross-coupling constant from handwritten notes: (Izz - Iyy) / Ixx
        # Given Roll-Pitch symmetry (Ixx = Iyy = I), C is identical for both axes.
        self.C = (self.Izz - self.Iyy) / self.Ixx

        # MPC Hyperparameters
        self.N = int(N)
        self.q_weight = float(q_weight)
        self.r_weight = float(r_weight)
        self.s_weight = float(s_weight)
        self.max_torque = float(max_torque)
        
        self.max_total_thrust = float(max_total_thrust)
        self.hover_target = np.array(target_position, dtype=float)

        # Last applied torque storage to compute slew rate penalization
        self.u_prev = np.zeros((2, 1))

        self.index = 0
        self.trajectory = None
        self.max_index = 1
        if trajectory_file is not None:
            self.trajectory = self.read_trajectory_from_csv(trajectory_file)
            self.max_index, _ = self.trajectory.shape

        self.total_time = 0.0
        self.received_first_state = False

        self.prev_error_world_x = 0.0
        self.prev_error_world_y = 0.0

        self.results_file = results_file
        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
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
            "thrust": np.array(self.thrust_over_time),
            "torque": np.vstack(self.torque_over_time) if self.torque_over_time else np.zeros((0, 3)),
            "motor_thrusts": np.vstack(self.motor_thrusts_over_time) if self.motor_thrusts_over_time else np.zeros((0, 4)),
        }
        np.savez(self.results_file, **statistics)
        carb.log_warn("MPC Statistics saved to: " + self.results_file)
        self.reset_statistics()

    def reset_statistics(self):
        self.index = 0
        self.total_time = 0.0
        self.u_prev = np.zeros((2, 1))
        self.time_vector = []
        self.position_over_time = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        self.thrust_over_time = []
        self.torque_over_time = []
        self.motor_thrusts_over_time = []
        self.prev_error_world_x = 0.0
        self.prev_error_world_y = 0.0

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

    def _reference_from_trajectory(self):
        p_ref = self.hover_target.copy()
        if self.trajectory is None:
            return p_ref
        if self.index < self.max_index - 1 and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1
        row = self.trajectory[self.index]
        return np.array([row[1], row[2], row[3]])
    
    def input_reference(self):
        """
        Returns the computed motor control signals or reference 
        velocities required by the Pegasus vehicle interface.
        """
        return self.input_ref

    def update(self, dt: float):
        if not self.received_first_state:
            return

        self.total_time += dt
        dt_safe = max(float(dt), 1e-6)

        target_pos = self._reference_from_trajectory()
        roll, pitch, yaw = self._euler_from_quat_xyzw(self.q)

        # ---- Outer Loop Position Loop (Translation) ----
        pos_error_world = target_pos - self.p
        error_rate_world_x = (pos_error_world[0] - self.prev_error_world_x) / dt_safe
        error_rate_world_y = (pos_error_world[1] - self.prev_error_world_y) / dt_safe
        self.prev_error_world_x = pos_error_world[0]
        self.prev_error_world_y = pos_error_world[1]

        acc_world_x_desired = self.kp_pos * pos_error_world[0] + self.kd_pos * error_rate_world_x
        acc_world_y_desired = self.kp_pos * pos_error_world[1] + self.kd_pos * error_rate_world_y

        # Mapping lateral tracking to target angular kinematic rate profiles
        desired_roll = ((np.sin(yaw) * acc_world_x_desired) - (np.cos(yaw) * acc_world_y_desired)) / self.g
        desired_pitch = ((np.cos(yaw) * acc_world_x_desired) + (np.sin(yaw) * acc_world_y_desired)) / self.g

        # Basic kinematic P mappings for reference body rates
        # (acting as kinematic target profiles for the MPC inner loop)
        p_ref = 4.0 * (desired_roll - roll)
        q_ref = 4.0 * (desired_pitch - pitch)

        # ---- Altitude Vector Allocation ----
        z_error = pos_error_world[2]
        z_vel = self.v[2]
        cos_roll = np.cos(roll)
        cos_pitch = np.cos(pitch)
        tilt_den = max(abs(cos_roll * cos_pitch), 1e-3)
        base_thrust = (self.m * self.g) / (4.0 * tilt_den)
        altitude_correction = (self.kp_z * z_error - self.kd_z * z_vel) / 4.0

        # ---- LPV MPC Formulation (Handwritten State Dynamics Subsystem) ----
        p, q, r = self.w[0], self.w[1], self.w[2]

        # Continuous State-Space Matrices explicitly using the cross-coupling sign structure
        # x_dot = A*x + B*u -> Roll-pitch symmetric setup
        A_c = np.array([
            [-self.alpha,  self.C * r],
            [-self.C * r, -self.alpha]
        ])
        B_c = np.array([
            [1.0 / self.Ixx, 0.0],
            [0.0, 1.0 / self.Iyy]
        ])

        # Forward Euler Discretization Matching your derivation Steps
        A_d = np.eye(2) + A_c * dt_safe
        B_d = B_c * dt_safe

        # Construct stacked prediction matrices across Horizon N
        # X_pipeline = Px * x_k + Hu * U_pipeline
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

        # Objective Weight Structures
        Q_bar = sparse.kron(sparse.eye(N), sparse.diags([self.q_weight, self.q_weight])).tocsr()
        R_bar = sparse.kron(sparse.eye(N), sparse.diags([self.r_weight, self.r_weight])).tocsr()

        # Build trajectory reference vector
        X_ref = np.tile(np.array([[p_ref], [q_ref]]), (N, 1))
        x_k = np.array([[p], [q]])

        # Cast to standard Quadratic Program formulation: Min (1/2)*U^T*H*U + F^T*U
        H = Hu.T @ Q_bar @ Hu + R_bar
        F = Hu.T @ Q_bar @ (Px @ x_k - X_ref)

        # Physical constraints setup (torque limits)
        u_min = np.tile(np.array([[-self.max_torque], [-self.max_torque]]), (N, 1))
        u_max = np.tile(np.array([[self.max_torque], [self.max_torque]]), (N, 1))

        # Solve via OSQP 
        prob = osqp.OSQP()
        # Identity matrix structure used to frame bounding box array constraints
        A_constraints = sparse.eye(Nu * N, format='csc')
        prob.setup(sparse.csc_matrix(H), F, A_constraints, u_min.flatten(), u_max.flatten(), verbose=False)
        res = prob.solve()

        # Fallback handle if numerical matrix inversion fails
        if res.info.status == 'solved':
            u_opt = res.x[:Nu]
        else:
            u_opt = np.array([0.0, 0.0])

        roll_correction = float(u_opt[0])
        pitch_correction = float(u_opt[1])

        # Passive tracking allocation for Yaw based on steady-state model
        net_thrust = base_thrust * 4.0
        desired_yaw_rate = net_thrust * self.km / (self.kf * 0.3)

        # Compute motor thrusts matching the layout matrix configuration
        u1 = base_thrust + altitude_correction - roll_correction + pitch_correction
        u2 = base_thrust + altitude_correction + roll_correction + pitch_correction
        u3 = base_thrust + altitude_correction + roll_correction - pitch_correction
        u4 = base_thrust + altitude_correction - roll_correction - pitch_correction

        motor_thrusts = np.maximum(np.array([u1, u2, u3, u4]), 0.0)
        total_thrust = float(np.clip(np.sum(motor_thrusts), 0.0, self.max_total_thrust))
        tau = np.array([roll_correction, pitch_correction, 0.0])

        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(total_thrust, tau)

        # Logging Statistics
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p.copy())
        self.desired_position_over_time.append(target_pos.copy())
        self.position_error_over_time.append(pos_error_world.copy())
        self.thrust_over_time.append(total_thrust)
        self.torque_over_time.append(tau.copy())
        self.motor_thrusts_over_time.append(motor_thrusts.copy())
