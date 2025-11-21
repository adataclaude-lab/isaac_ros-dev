#!/bin/bash
# Script to install moveit_py for ROS2

set -e

echo "Installing MoveIt2 Python bindings..."

# Detect ROS distribution
if [ -n "$ROS_DISTRO" ]; then
    echo "Detected ROS distribution: $ROS_DISTRO"
else
    echo "ROS_DISTRO not set. Please source your ROS2 setup first."
    exit 1
fi

# Try to install moveit_py package
PACKAGE_NAME="ros-${ROS_DISTRO}-moveit-py"
echo "Attempting to install $PACKAGE_NAME..."

sudo apt update
sudo apt install -y "$PACKAGE_NAME"

echo ""
echo "✓ Installation complete!"
echo ""
echo "Now try running your command again:"
echo "  ros2 run dualarm_moveit_config simple_ee_control.py separate"
