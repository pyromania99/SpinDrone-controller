# Spinning Drone Controller Analysis

A custom dynamics analysis platform built to develop and evaluate a nonlinear controller for a strictly **Spinning Unmanned Aerial Vehicle (UAV)** dynamically configured with all core motors spinning in the identical rotational direction. 

This repository leverages the [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)—a high-fidelity framework powered by NVIDIA Omniverse and Isaac Sim—to rapidly execute pure Python mathematics and rigid-body simulations without the bloat of PX4 or ROS.

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

Because the goal of this repository is strictly spinning dynamics evaluation, irrelevant backend controllers (like PX4/Ardupilot variants) have been surgically removed.

- `extensions/`: The core Pegasus Simulator extension logic. This gets automatically installed as an editable module (`-e`) into the container.
- `examples/utils/nonlinear_controller.py`: The custom nonlinear analysis controller backend designed to process the spinning logic.
- `examples/controller_tests/`: Eigenvalue and stability sweep mathematics frameworks used to empirically track physical tuning outputs.
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
