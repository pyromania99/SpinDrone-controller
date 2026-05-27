# -----------------------------------------------------------------------------
# Spin Drone Controller — Isaac Sim + Pegasus Development Environment
# -----------------------------------------------------------------------------
# Base Image:
#   NVIDIA Isaac Sim 4.5.0
#
# This image is intended for:
#   - Pegasus Simulator development
#   - Custom drone controller experimentation
#   - MPC / geometric / nonlinear controller testing
#   - Headless or GUI Isaac workflows
#
# Recommended workflow:
#   - Build image once
#   - Mount repo as live volume
#   - Iterate on controllers without rebuilding
#
# Build:
#   DOCKER_BUILDKIT=1 docker build -t spin-drone .
#
# Run:
#   xhost +local:docker
#
#   docker run --gpus all -it \
#       --network host \
#       -e DISPLAY=$DISPLAY \
#       -v /tmp/.X11-unix:/tmp/.X11-unix \
#       -v ~/Spin_drone-controller:/workspace/spin-drone-controller \
#       -v isaac-kit-cache:/isaac-cache/kit \
#       -v isaac-ov-cache:/isaac-cache/ov \
#       spin-drone
# -----------------------------------------------------------------------------

FROM nvcr.io/nvidia/isaac-sim:4.5.0

# -----------------------------------------------------------------------------
# Shell
# -----------------------------------------------------------------------------

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# -----------------------------------------------------------------------------
# Environment Variables
# -----------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

# Isaac Sim root path
ENV ISAACSIM_PATH=/isaac-sim

# Better Python logging behavior
ENV PYTHONUNBUFFERED=1

# Avoid pip cache bloat
ENV PIP_NO_CACHE_DIR=1

# -----------------------------------------------------------------------------
# System Dependencies
# -----------------------------------------------------------------------------

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    vim \
    sudo \
    wget \
    curl \
    unzip \
    build-essential \
    python3-pip \
    python3-dev \
    mesa-utils \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Create Non-Root User
# -----------------------------------------------------------------------------

ARG USERNAME=container_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} \
            --gid ${USER_GID} \
            -m \
            -s /bin/bash \
            ${USERNAME} && \
    mkdir -p /etc/sudoers.d && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# -----------------------------------------------------------------------------
# Workspace
# -----------------------------------------------------------------------------

WORKDIR /workspace

# -----------------------------------------------------------------------------
# Copy ONLY Pegasus Extension
# -----------------------------------------------------------------------------
# This dramatically improves Docker caching.
# Editing controllers will NOT trigger Pegasus reinstall.
# -----------------------------------------------------------------------------

COPY extensions/pegasus.simulator /tmp/pegasus.simulator

# -----------------------------------------------------------------------------
# Install Pegasus into Isaac Sim Python
# -----------------------------------------------------------------------------

RUN cd /tmp/pegasus.simulator && \
    /isaac-sim/python.sh -m pip install --upgrade pip setuptools wheel && \
    /isaac-sim/python.sh -m pip install -e .

# -----------------------------------------------------------------------------
# Additional Python Packages
# -----------------------------------------------------------------------------
# Add your controller/research dependencies here.
# -----------------------------------------------------------------------------

RUN /isaac-sim/python.sh -m pip install \
    numpy \
    scipy \
    matplotlib \
    pandas \
    transforms3d \
    pynput

# -----------------------------------------------------------------------------
# Isaac Sim Cache Directories
# -----------------------------------------------------------------------------
# These SHOULD be mounted as Docker volumes for much faster startup.
# -----------------------------------------------------------------------------

RUN mkdir -p \
    /isaac-cache/kit \
    /isaac-cache/ov \
    /isaac-cache/pip

ENV OMNI_USER_CACHE_DIR=/isaac-cache/kit
ENV OMNI_DATA_DIR=/isaac-cache/ov
ENV PIP_CACHE_DIR=/isaac-cache/pip

# -----------------------------------------------------------------------------
# Project Workspace
# -----------------------------------------------------------------------------
# Your repo will be mounted here live from host:
#
#   ~/Spin_drone-controller
#
# This means:
#   - No rebuilds for controller edits
#   - Live VSCode development
#   - Faster experimentation
# -----------------------------------------------------------------------------

WORKDIR /workspace/spin-drone-controller

# -----------------------------------------------------------------------------
# Python Path
# -----------------------------------------------------------------------------

ENV PYTHONPATH=/workspace/spin-drone-controller:$PYTHONPATH

# -----------------------------------------------------------------------------
# Permissions
# -----------------------------------------------------------------------------

RUN chown -R ${USERNAME}:${USERNAME} /workspace

# -----------------------------------------------------------------------------
# Default User
# -----------------------------------------------------------------------------

USER ${USERNAME}

# -----------------------------------------------------------------------------
# Default Command
# -----------------------------------------------------------------------------

CMD ["/bin/bash"]

# -----------------------------------------------------------------------------
# Optional Headless Entrypoint
# -----------------------------------------------------------------------------
# Uncomment if desired:
#
# ENTRYPOINT ["/isaac-sim/python.sh"]
#
# CMD ["/workspace/spin-drone-controller/examples/4_python_single_vehicle.py"]
#
# -----------------------------------------------------------------------------#
