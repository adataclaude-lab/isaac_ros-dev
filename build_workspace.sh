#!/bin/bash

# Build script for isaac_ros-dev workspace
# This script builds the dualarm packages

set -e

echo "Building dualarm packages..."

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "Warning: ROS 2 installation not found in standard location"
fi

# Build packages
cd /workspaces/isaac_ros-dev || cd /home/user/isaac_ros-dev

# Try colcon build
if command -v colcon &> /dev/null; then
    colcon build --packages-select dualarm_description dualarm_moveit_config --symlink-install
else
    echo "Error: colcon not found. Please install with:"
    echo "  sudo apt update && sudo apt install python3-colcon-common-extensions"
    exit 1
fi

echo ""
echo "Build complete! Now source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Then try launching again:"
echo "  ros2 launch dualarm_moveit_config demo.launch.py"
