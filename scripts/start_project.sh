#!/bin/bash

echo "Building and starting Offboard Control project..."

# Get the project root directory (parent of scripts folder)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

# Wait for other components to be ready
echo "Waiting for Gazebo and uORB client to be ready..."
sleep 20

# Build the project
echo "Building project..."

# Source ROS setup
source /opt/ros/humble/setup.bash

# Build with colcon
if [ -d "build" ]; then
    colcon build --packages-select custom_offboard_control
else
    colcon build
fi

# Source the install setup
source install/setup.bash

echo "Build completed."

# Run the offboard control node
echo "Starting offboard control node..."
# ros2 run custom_offboard_control offboard_control_node
ros2 run custom_offboard_control offboard_control_sequenced_node