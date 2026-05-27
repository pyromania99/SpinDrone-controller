#!/usr/bin/env python
"""
| File: compare_controllers_60s.py
| Description: Post-processing script to compare position tracking errors and 
|              integrated motor energy consumption limited strictly to the first 60 seconds.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

RESULTS_DIR = "examples/controller_tests/results"
CONTROLLERS = {
    "Cascaded PD": "cascaded_pd_statistics.npz",
    "Eigen Controller": "eigen_statistics.npz",
    "LPV-MPC": "lpv_mpc_statistics.npz"
}

def analyze_energy_and_error_60s():
    summary_data = {}
    T_LIMIT = 60.0  # Strict upper limit in seconds
    
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    colors = {"Cascaded PD": "tab:gray", "Eigen Controller": "tab:orange", "LPV-MPC": "tab:blue"}
    
    print("=" * 70)
    print(f"{'CONTROLLER':<20} | {'RMS ERROR (m)':<15} | {'60s ENERGY METRIC (N^1.5·s)':<25}")
    print("=" * 70)
    
    for name, filename in CONTROLLERS.items():
        path = os.path.join(RESULTS_DIR, filename)
        
        if not os.path.exists(path):
            print(f"Skipping {name}: File not found at {path}")
            continue
            
        data = np.load(path)
        
        if 'time' not in data or 'ep' not in data:
            print(f"Skipping {name}: Log arrays are corrupted or missing.")
            continue
            
        t_full = data['time']
        ep_full = data['ep']
        
        # Create a boolean mask to clip data strictly up to 60 seconds
        mask = t_full <= T_LIMIT
        t = t_full[mask]
        ep = ep_full[mask]
        
        if len(t) == 0:
            print(f"Skipping {name}: No data found within the first 60 seconds.")
            continue

        # Calculate Root-Mean-Square (RMS) Euclidean Error over the 60s window
        euclidean_error = np.linalg.norm(ep, axis=1)
        rms_error = np.sqrt(np.mean(euclidean_error**2))
        
        # --- Energy Quantification Engine (Limited to 60s) ---
        total_energy = 0.0
        if 'motor_thrusts' in data and len(data['motor_thrusts']) > 0:
            motor_thrusts = data['motor_thrusts'][mask]
            power_proxy = np.sum(motor_thrusts ** 1.5, axis=1)
            total_energy = np.trapz(power_proxy, t)
        elif 'thrust' in data:
            lumped_thrust = data['thrust'][mask]
            per_motor_approx = lumped_thrust / 4.0
            power_proxy = 4.0 * (per_motor_approx ** 1.5)
            total_energy = np.trapz(power_proxy, t)
            
        summary_data[name] = {
            "rms_error": rms_error,
            "energy": total_energy,
            "time": t,
            "ep": ep
        }
        
        print(f"{name:<20} | {rms_error:<15.4f} | {total_energy:<25.2f}")
        
        # Plot sliced component tracking errors
        axes[0].plot(t, ep[:, 0], color=colors[name], label=name, alpha=0.85)
        axes[1].plot(t, ep[:, 1], color=colors[name], label=name, alpha=0.85)
        axes[2].plot(t, ep[:, 2], color=colors[name], label=name, alpha=0.85)

    print("=" * 70)

    if not summary_data:
        print("No valid controller data found to plot.")
        return

    # --- Formatting the Plots ---
    axes[0].set_ylabel("X Position Error [m]")
    axes[0].set_title("Multi-Controller Flight Trajectory Error Analysis (0-60s)")
    axes[0].grid(True, linestyle=":")
    axes[0].legend(loc="upper right")
    axes[0].set_xlim([0, T_LIMIT])
    
    axes[1].set_ylabel("Y Position Error [m]")
    axes[1].grid(True, linestyle=":")
    
    axes[2].set_ylabel("Z Position Error [m]")
    axes[2].set_xlabel("Time [s]")
    axes[2].grid(True, linestyle=":")
    
    plt.tight_layout()
    
    # --- Energy Summary Bar Chart ---
    plt.figure(figsize=(10, 4))
    names = list(summary_data.keys())
    energies = [summary_data[n]["energy"] for n in names]
    bar_colors = [colors[n] for n in names]
    
    plt.barh(names, energies, color=bar_colors, alpha=0.8, height=0.5)
    plt.xlabel("Total Integrated Energy Metric [$N^{1.5} \cdot s$] (Lower is Better)")
    plt.title("Actuator Allocation Efficiency Comparison (0-60s Windows)")
    plt.grid(axis='x', linestyle=":")
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_energy_and_error_60s()
