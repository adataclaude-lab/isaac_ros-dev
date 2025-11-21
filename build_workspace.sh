#!/bin/bash

# Build script for isaac_ros-dev workspace
# This script builds the dualarm packages

set -e

echo "Building dualarm packages..."

# Determine workspace directory
if [ -d "/workspaces/isaac_ros-dev" ]; then
    WORKSPACE_DIR="/workspaces/isaac_ros-dev"
elif [ -d "/home/user/isaac_ros-dev" ]; then
    WORKSPACE_DIR="/home/user/isaac_ros-dev"
else
    echo "Error: Could not find isaac_ros-dev workspace"
    exit 1
fi

echo "Using workspace: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo "Sourcing ROS 2 Foxy..."
    source /opt/ros/foxy/setup.bash
else
    echo "Error: ROS 2 installation not found in /opt/ros/"
    echo "Please ensure ROS 2 is installed."
    exit 1
fi

# Check for colcon
if ! command -v colcon &> /dev/null; then
    echo "Error: colcon not found. Please install with:"
    echo "  sudo apt update && sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Build packages
echo "Building packages with colcon..."
colcon build --packages-select dualarm_description dualarm_moveit_config --symlink-install

echo ""
echo "✓ Build complete! Now source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Then try launching again:"
echo "  ros2 launch dualarm_moveit_config demo.launch.py"
echo "  ros2 launch dualarm_moveit_config isaac_sim_bridge.launch.py"
