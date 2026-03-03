#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Configuration
VENV_DIR=".venv"
ROS_DISTRO="jazzy"
WORKSPACE_DIR=$(pwd)

echo "[INFO] Starting system setup..."

# ---------------------------------------------------------
# 1. System Dependencies (APT)
# ---------------------------------------------------------
# Check and install system dependencies
DEPENDENCIES=(
    "ros-$ROS_DISTRO-foxglove-bridge"
    "ros-$ROS_DISTRO-image-transport-plugins"
    "ros-$ROS_DISTRO-topic-tools" 
    "ros-$ROS_DISTRO-image-proc"
)

for pkg in "${DEPENDENCIES[@]}"; do
    if ! dpkg -l | grep -q "$pkg"; then
        echo "[INFO] $pkg not found. Installing..."
        sudo apt update
        sudo apt install -y "$pkg"
    fi
done

# ---------------------------------------------------------
# 2. Python Virtual Environment (VENV)
# ---------------------------------------------------------
if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating Python virtual environment in $VENV_DIR..."
    python3 -m venv $VENV_DIR --system-site-packages
fi

echo "[INFO] Activating virtual environment..."
source $VENV_DIR/bin/activate

# ---------------------------------------------------------
# 3. Python Dependencies (PIP within VENV)
# ---------------------------------------------------------
echo "[INFO] Checking Python dependencies for YOLO..."
# Upgrade pip first to avoid issues
pip install --upgrade pip
# Install specific versions required by YOLO node
pip install "numpy<2" "opencv-python<4.12" ultralytics colcon-common-extensions

# ---------------------------------------------------------
# 4. Build Workspace
# ---------------------------------------------------------
echo "[INFO] Building workspace (symlink-install)..."
# We are inside the venv, so colcon will use the venv's python
colcon build --symlink-install

# ---------------------------------------------------------
# 5. Source and Launch
# ---------------------------------------------------------
echo "[INFO] Sourcing ROS 2 environment..."
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

export PYTHONPATH=$WORKSPACE_DIR/.venv/lib/python3.12/site-packages:$PYTHONPATH

echo "----------------------------------------"
echo "[INFO] System ready. Starting launch file..."
echo "----------------------------------------"

# Launch the main pipeline
ros2 launch psaf_launch main_psaf1.launch.py
