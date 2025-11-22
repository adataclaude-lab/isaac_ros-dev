#!/bin/bash
# Script to install moveit_py for ROS2

set -e

echo "Installing MoveIt2 with Python bindings..."

# Detect ROS distribution
if [ -n "$ROS_DISTRO" ]; then
    echo "Detected ROS distribution: $ROS_DISTRO"
else
    echo "ROS_DISTRO not set. Please source your ROS2 setup first."
    exit 1
fi

# Update package list
sudo apt update

# Try different installation methods
echo ""
echo "Method 1: Installing complete MoveIt2 suite..."
if sudo apt install -y ros-${ROS_DISTRO}-moveit 2>/dev/null; then
    echo "✓ MoveIt2 installed successfully"
else
    echo "Method 1 failed, trying alternative packages..."

    echo ""
    echo "Method 2: Installing MoveIt2 core packages..."
    sudo apt install -y \
        ros-${ROS_DISTRO}-moveit-common \
        ros-${ROS_DISTRO}-moveit-core \
        ros-${ROS_DISTRO}-moveit-ros-planning \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface \
        ros-${ROS_DISTRO}-moveit-planners \
        ros-${ROS_DISTRO}-moveit-simple-controller-manager \
        python3-pybind11 2>/dev/null || {
            echo ""
            echo "⚠ Binary packages not available. You may need to build from source."
            echo ""
            echo "To build from source:"
            echo "  cd ~/ws_moveit2"
            echo "  git clone -b humble https://github.com/moveit/moveit2.git src/moveit2"
            echo "  rosdep install -r --from-paths src --ignore-src --rosdistro humble -y"
            echo "  colcon build --packages-select moveit_py"
            exit 1
        }
fi

# Verify installation
echo ""
echo "Verifying moveit_py installation..."
if python3 -c "import moveit_py" 2>/dev/null; then
    echo "✓ moveit_py is available!"
    echo ""
    echo "Now try running your command again:"
    echo "  ros2 run dualarm_moveit_config simple_ee_control.py separate"
else
    echo "⚠ moveit_py module not found after installation."
    echo "You may need to build moveit_py from source."
    echo ""
    echo "See: https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html"
fi
