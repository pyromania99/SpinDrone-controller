# Use NVIDIA Isaac Sim base image corresponding to version 5.1.0 
# (You may need to verify the exact tag from the NVIDIA NGC registry depending on the host's driver version.)
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Set the working directory inside the container
WORKDIR /workspace/spin-drone-controller

# It is good practice to install any system-level dependencies you might need here
USER root
RUN apt-get update && apt-get install -y \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Copy the essential project files
COPY extensions/ /workspace/spin-drone-controller/extensions/

# Install the Python dependencies so they are baked into the Docker image
# The pip in /isaac-sim/python.sh is configured to work with the omniverse libraries
# Pegasus Simulator requires ISAACSIM_PATH to be explicitly set
ENV ISAACSIM_PATH=/isaac-sim
RUN cd /workspace/spin-drone-controller/extensions/pegasus.simulator && \
    /isaac-sim/python.sh -m pip install -e .

# Create a non-root user specifically for Devcontainer logic
RUN groupadd -g 1000 container_user && \
    useradd -u 1000 -g container_user -m -s /bin/bash container_user && \
    chown -R container_user:container_user /workspace

USER container_user

# Optional: Add any user setup or specific commands to launch the custom controller
# For headless simulation execution:
# ENTRYPOINT ["/isaac-sim/python.sh"]
# CMD ["src/my_spinning_drone_controller.py"]
