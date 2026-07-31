#!/usr/bin/env python
"""
| File: lpv_mpc_controller.py
| Description: LPV-MPC Predictive Inner-Loop Controller with a PID Outer Position Loop.
"""

import sys
from pathlib import Path

import carb
import numpy as np
import osqp
from scipy import sparse

from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State

# Sibling module in examples/utils. The sweeps put this directory on sys.path
# before importing the controller; the insert keeps a direct import working too.
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
from applied_thrust import applied_rotor_forces
import vehicle_params


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

    def _sync_physical_params_from_vehicle(self):
        """Install the vehicle's real mass/inertia; keep defaults on failure.

        This controller previously synced *nothing* and flew on its constructor
        values of m=1.2 kg and diag(0.01, 0.01, 0.02), while the vehicle is
        1.6186 kg with diag(0.0349, 0.0313, 0.0627). That is −25.9 % on the
        hover feedforward and roughly −70 % on every inertia axis — the model's
        ``B = diag(1/Ixx, 1/Iyy)`` therefore claimed about 3.5x the torque
        authority the airframe has, so the MPC solved for control increments
        against a plant far more responsive than the real one.

        The other two controllers here synced mass (and, for Eigen, inertia)
        from the vehicle, so this was also the largest single source of
        asymmetry in any cross-controller comparison: MPC was competing at
        −25.9 % hover error against their −7.3 %. Archives written before this
        change are at a materially different operating point, and the q/r
        weights chosen there were chosen for the wrong plant.

        See examples/utils/vehicle_params.py.
        """
        if self.vehicle is None:
            return

        props = vehicle_params.composite_mass_properties(self.vehicle)
        if props is None:
            carb.log_warn(
                "[LPVMPCController] Could not read mass/inertia from the physics "
                f"interface; keeping m={self.m:.4f}, "
                f"I=({self.Ixx:.5f}, {self.Iyy:.5f}, {self.Izz:.5f})")
            return

        self.m = props["mass"]
        self.Ixx = props["Ixx"]
        self.Iyy = props["Iyy"]
        self.Izz = props["Izz"]
        # C is cached from the inertias at construction and used every step as
        # the gyroscopic cross-coupling coefficient in the LPV A matrix, so it
        # has to be recomputed here or the sync would leave the model
        # internally inconsistent.
        self.C = (self.Izz - self.Iyy) / self.Ixx
        carb.log_warn(f"[LPVMPCController] model synced: {vehicle_params.describe(props)}"
                      f"  C={self.C:.6f}")

    def start(self):
        self._sync_physical_params_from_vehicle()
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

        # Powers of A_d computed once, not rebuilt per block.
        #
        # This inner loop used to recompute A_d^j from the identity for every
        # (i, j) pair, making the prediction matrices O(N^3) matrix products
        # where O(N^2) suffices — ~1800 tiny numpy calls per timestep at N=22,
        # at 250 Hz, for every drone in a sweep. Measured: 1.900 ms -> 0.337 ms
        # for the build at N=22, and 2.384 ms -> 0.822 ms for the whole step
        # including the QP, i.e. 7.2 s -> 2.5 s of controller compute per 12 s
        # run per drone.
        #
        # The result is bit-identical, not merely algebraically equal: both
        # forms evaluate (((I @ A_d) @ A_d) ... @ A_d) with the same number of
        # multiplications in the same left-associated order, starting from the
        # identity. Verified with exact array equality (not allclose) over 500
        # random LPV operating points — A_d carries the yaw rate, so it changes
        # every step — across horizons N = 5, 12, 22, 25.
        #
        # The per-step osqp.OSQP() construction below is untouched and remains
        # the other half of the cost. It cannot simply be hoisted: A_d depends
        # on the scheduling parameter, so H genuinely changes every step.
        powers = [np.eye(2)]
        for _ in range(N):
            powers.append(powers[-1] @ A_d)

        for i in range(N):
            Px[i*Nx:(i+1)*Nx, :] = powers[i + 1]
            for j in range(i + 1):
                Hu[i*Nx:(i+1)*Nx, (i-j)*Nu:(i-j+1)*Nu] = powers[j] @ B_d

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

        commanded_thrusts = np.maximum(np.array([u1, u2, u3, u4]), 0.0)
        total_thrust = float(np.clip(np.sum(commanded_thrusts), 0.0, self.max_total_thrust))
        tau = np.array([roll_correction, pitch_correction, 0.0])

        # motor_thrusts logs what the vehicle ACTUALLY applies, not this mixer's
        # idealisation of it. EigenController and CascadedPDController were both
        # converted to applied_rotor_forces; this one was missed, so every MPC
        # archive's effort metric was computed from a quantity the simulator
        # never produced. The u1..u4 reconstruction ignores two things the
        # allocator does: it saturates by scaling all four rotors together
        # rather than clipping them individually, and it bounds each rotor at
        # rotor_constant × max_rotor_velocity² = 10.34 N. The reconstruction
        # exceeded that ceiling on every logged run (13.6-17.4 N per rotor),
        # which is what the smoke test's thrust-over assertion detects.
        # See examples/utils/applied_thrust.py.
        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(total_thrust, tau)
            motor_thrusts = applied_rotor_forces(self.vehicle, self.input_ref)
        else:
            motor_thrusts = np.zeros(4)

        # Log Data Trace Elements
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p.copy())
        self.desired_position_over_time.append(target_pos.copy())
        self.position_error_over_time.append(pos_error_world.copy())
        self.yaw_rate_over_time.append(self.w[2])
        self.angular_velocity_over_time.append(np.asarray(self.w, dtype=float).copy())
        # Applied, not commanded — consistent with motor_thrusts above and with
        # the other two controllers.
        self.thrust_over_time.append(float(np.sum(motor_thrusts)))
        self.torque_over_time.append(tau.copy())
        self.motor_thrusts_over_time.append(motor_thrusts.copy())
