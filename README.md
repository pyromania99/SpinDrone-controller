# Spinning Drone Controller Analysis

A custom dynamics analysis platform built to develop and evaluate a nonlinear controller for a strictly **Spinning Unmanned Aerial Vehicle (UAV)** dynamically configured with all core motors spinning in the identical rotational direction. 

This repository leverages the [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)—a high-fidelity framework powered by NVIDIA Omniverse and Isaac Sim. Controller development happens against a pure-Python rigid-body path with no PX4 or ROS in the loop, which keeps the maths fast to iterate on; a separate PX4 SITL harness then flies the same gains on the real autopilot to check that what worked in the prototype survives an estimator and a transport delay.

---

## ⚡ Out-of-the-Box Setup (DevContainers)

To completely eliminate environment setup anxiety, this repository is fully dockerized with **VS Code Dev Containers**. The container automatically installs Isaac Sim dependencies and maps live workspace editing environments seamlessly so you can focus strictly on mathematical controller logic.

We feature two natively supported Dev Container profiles:
- **Linux Environment** (`.devcontainer/linux/devcontainer.json`)
- **Windows Subsystem for Linux (WSL)** (`.devcontainer/wsl/devcontainer.json`)

By selecting "Rebuild and Reopen in Container" in VS Code, your python endpoints dynamically spawn right alongside the Isaac Sim environment.

### System Prerequisites

Because Isaac Sim requires deep hardware rendering access, your host machine **must** meet the following hardware and software dependencies to use the dockerized environment:
1. **NVIDIA GPU** (RTX tier strongly recommended for Omniverse rendering).
2. **Docker Engine**.
3. **[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)**: *Extremely Important.* This is required to passthrough the NVIDIA hardware directly into the Docker Container interface footprint.
4. **VS Code** with the **Dev Containers** Extension.

---

## 🚀 Repository Structure Overview

- `extensions/`: The core Pegasus Simulator extension logic. This gets automatically installed as an editable module (`-e`) into the container.
- `examples/utils/`: The controllers themselves — `eigen_controller.py`, `cascaded_pd_controller.py` and `lpv_mpc_controller.py` — plus the shared task, metric and vehicle-parameter definitions every sweep is built on.
- `examples/flight/`: **Every way of actually flying something.** The three simulation levels are launched from here, and the offboard scripts that drive a running PX4 over MAVLink live alongside them. See its own README.
- `examples/sweeps/`: Searching the gain space — the genetic Pareto search, the eigenvalue and robustness sweeps. These fly too, but many vehicles at once rather than one.
- `examples/tools/`: Everything that supports flight without doing it — `px4_tuning/` (ulog analysis), `evaluations/` (runs to numbers), `visualization/` (plots and reports), `scripts/` (the bash drivers) and `archive/` (superseded).
- `examples/results/`: Generated output from every sweep and flight. Gitignored; rebuildable.
- `notes/`: The research record — `FINDINGS.md` (why each piece of the PX4 work exists) and `NEXT_STEPS.md` (what to run next, and in what order).
- `Dockerfile`: Found at the root of the project, executing environment configurations while wiring up `container_user` to seamlessly align with your host permissions.

---

## 📝 Citation & Original Framework

The baseline architectural simulation engine used within this repository belongs to the Pegasus Simulator team. If the base simulation backend is utilized, please reference the original framework:

```
@INPROCEEDINGS{10556959,
  author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
  booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
  year={2024},
  keywords={Simulation;Robot sensing systems;Real-time systems;Sensor systems;Sensors;Task analysis},
  doi={10.1109/ICUAS60882.2024.10556959}
}
```
