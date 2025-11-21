#!/bin/bash

# Build script to run inside Isaac ROS container
# Usage: ./build_in_container.sh

set -e

echo "=== Building dualarm_moveit_config in container ==="

# Navigate to workspace
cd /workspaces/isaac_ros-dev || cd /home/user/isaac_ros-dev || {
    echo "Error: Could not find workspace directory"
    exit 1
}

echo "Working directory: $(pwd)"

# Check if we're in a container with ROS sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS environment not sourced!"
    echo "Please run this script inside the Isaac ROS container."
    echo ""
    echo "To enter the container:"
    echo "  cd ~/workspaces/isaac_ros-dev"
    echo "  src/isaac_ros_common/scripts/run_dev.sh"
    echo ""
    echo "Then run this script again."
    exit 1
fi

echo "ROS Distribution: $ROS_DISTRO"

# Check for colcon
if ! command -v colcon &> /dev/null; then
    echo "Error: colcon not found"
    exit 1
fi

# Build packages
echo ""
echo "Building packages..."
colcon build --packages-select dualarm_description dualarm_moveit_config --symlink-install

echo ""
echo "✓ Build complete!"
echo ""
echo "Now source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Then you can run:"
echo "  ros2 launch dualarm_moveit_config isaac_sim_bridge.launch.py"
echo "  ros2 run dualarm_moveit_config simple_ee_control.py separate"
