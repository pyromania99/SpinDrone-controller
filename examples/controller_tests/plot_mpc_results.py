#!/usr/bin/env python
import os
import numpy as np
import matplotlib.pyplot as plt

# Update this path if your results directory is located elsewhere
file_path = "examples/controller_tests/results/lpv_mpc_statistics.npz"

if not os.path.exists(file_path):
    print(f"Error: Could not find stats file at {file_path}")
    exit(1)

# Load the compressed data file
with np.load(file_path) as data:
    t = data['time']
    p = data['p']
    p_ref = data['desired_p']
    ep = data['ep']
    torque = data['torque']
    motor_thrusts = data['motor_thrusts']

# Create a clean canvas layout for analyzing tracking performance
plt.figure(figsize=(12, 10))

# --- Plot 1: 3D Position Tracking Over Time ---
plt.subplot(2, 1, 1)
plt.plot(t, p_ref[:, 0], 'r--', label='X Ref', alpha=0.8)
plt.plot(t, p[:, 0], 'r-', label='X Actual')
plt.plot(t, p_ref[:, 1], 'g--', label='Y Ref', alpha=0.8)
plt.plot(t, p[:, 1], 'g-', label='Y Actual')
plt.plot(t, p_ref[:, 2], 'b--', label='Z Ref', alpha=0.8)
plt.plot(t, p[:, 2], 'b-', label='Z Actual')
plt.title('LPV-MPC Trajectory Profile Tracking Performance')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.grid(True, linestyle=':')
plt.legend(loc='upper right')

# --- Plot 2: Absolute Tracking Error ---
plt.subplot(2, 1, 2)
plt.plot(t, ep[:, 0], 'r', label='Error X')
plt.plot(t, ep[:, 1], 'g', label='Error Y')
plt.plot(t, ep[:, 2], 'b', label='Error Z')
plt.xlabel('Time [s]')
plt.ylabel('Error Margin [m]')
plt.title('Translation Tracking Errors')
plt.grid(True, linestyle=':')
plt.legend(loc='upper right')

plt.show()
