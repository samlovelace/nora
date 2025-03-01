#!/bin/bash

# Navigate to the ROS2 workspace
cd ~/dev/cpp/abv/nora_ws || { echo "ROS2 workspace not found!"; exit 1; }

# Build the package
echo "Building NORA (Navigation using Optitrack for Robot Autonomy) ..."
colcon build --packages-select nora nora_idl

# Check if the build succeeded
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Source the workspace
echo "Sourcing workspace setup..."
source install/setup.bash

# export the library path 
export LD_LIBRARY_PATH=~/dev/cpp/abv/nora_ws/src/nora/dependencies/NatNetSDK4.1/lib:$LD_LIBRARY_PATH

# Run the ROS2 node
echo "Running nora module..."
ros2 run nora nora
