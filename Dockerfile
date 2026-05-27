# ---------------------------------------------------------------------------
# Spinning Drone Controller — Isaac Sim Dev Image
# ---------------------------------------------------------------------------
# Base image: NVIDIA Isaac Sim 4.5 release (Pegasus 5.1.0 targets this tag).
# Check https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim
# for the latest tag before building.
# ---------------------------------------------------------------------------
FROM nvcr.io/nvidia/isaac-sim:4.5.0

# Use bash explicitly so that shell expansions work correctly in RUN steps.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---------------------------------------------------------------------------
# System-level dependencies
# ---------------------------------------------------------------------------
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    sudo \
    wget \
    python3-pip \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Project workspace
# ---------------------------------------------------------------------------
# Copy the entire project so that examples/, trajectories/, and results/ are
# available inside the container at build time.
# The workspace folder is ALSO bind-mounted at runtime by devcontainer.json,
# so live edits on the host are immediately reflected inside the container.
WORKDIR /workspace/spin-drone-controller
COPY . /workspace/spin-drone-controller/

# ---------------------------------------------------------------------------
# Python dependencies (baked into the image)
# ---------------------------------------------------------------------------
# ISAACSIM_PATH must be set so that Pegasus setup.py can locate the Kit app
# config file (isaacsim.exp.base.kit) for patching.
ENV ISAACSIM_PATH=/isaac-sim
RUN cd /workspace/spin-drone-controller/extensions/pegasus.simulator && \
    /isaac-sim/python.sh -m pip install --no-cache-dir -e .

# ---------------------------------------------------------------------------
# Non-root container user (used by the Dev Container "remoteUser" setting)
# ---------------------------------------------------------------------------
RUN useradd -m -s /bin/bash container_user && \
    mkdir -p /etc/sudoers.d && \
    echo "container_user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/container_user && \
    chmod 0440 /etc/sudoers.d/container_user && \
    chown -R container_user:container_user /workspace

# DO NOT switch to container_user here.
# The Dev Container runtime handles the user switch via the "remoteUser" field
# in devcontainer.json, which also ensures UID mapping to your host user.

# ---------------------------------------------------------------------------
# Default entrypoint (override at runtime or via devcontainer.json)
# ---------------------------------------------------------------------------
# For headless simulation, uncomment and adjust the CMD below:
# ENTRYPOINT ["/isaac-sim/python.sh"]
# CMD ["/workspace/spin-drone-controller/examples/4_python_single_vehicle.py"]
