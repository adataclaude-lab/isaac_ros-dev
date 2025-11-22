#!/bin/bash
# Script to build moveit_py from source for ROS2 Humble

set -e

echo "=========================================="
echo "Building moveit_py from source"
echo "=========================================="
echo ""

# Detect ROS distribution
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS_DISTRO not set. Please source your ROS2 setup first."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Detected ROS distribution: $ROS_DISTRO"
echo ""

# Set workspace directory
WORKSPACE_DIR="${HOME}/ws_moveit"
echo "Workspace directory: $WORKSPACE_DIR"
echo ""

# Create workspace
echo "Step 1: Creating workspace..."
mkdir -p ${WORKSPACE_DIR}/src
cd ${WORKSPACE_DIR}

# Clone moveit2 repository
echo ""
echo "Step 2: Cloning moveit2 repository (branch: $ROS_DISTRO)..."
if [ ! -d "src/moveit2" ]; then
    git clone -b ${ROS_DISTRO} https://github.com/moveit/moveit2.git src/moveit2
    echo "✓ Cloned moveit2 repository"
else
    echo "✓ moveit2 repository already exists, updating..."
    cd src/moveit2
    git pull
    cd ${WORKSPACE_DIR}
fi

# Install dependencies
echo ""
echo "Step 3: Installing dependencies with rosdep..."
echo "This may take a few minutes..."
sudo apt update
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

# Build only moveit_py
echo ""
echo "Step 4: Building moveit_py package..."
echo "This may take 5-10 minutes..."
colcon build --packages-select moveit_py --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "To use moveit_py, source the workspace:"
    echo "  source ${WORKSPACE_DIR}/install/setup.bash"
    echo ""
    echo "Add this to your ~/.bashrc to make it permanent:"
    echo "  echo 'source ${WORKSPACE_DIR}/install/setup.bash' >> ~/.bashrc"
    echo ""
    echo "Then verify the installation:"
    echo "  python3 -c 'import moveit_py; print(\"moveit_py imported successfully!\")'"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed"
    echo "=========================================="
    echo ""
    echo "Please check the error messages above."
    exit 1
fi
