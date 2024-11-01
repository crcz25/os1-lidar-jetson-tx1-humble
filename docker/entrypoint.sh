#!/bin/bash

# Start SSH server
#sudo service ssh start

# Source ROS
#source /opt/ros/humble/setup.bash

# Check if the workspace needs to be built, if so, build it, otherwise source it
#if [ ! -d "/ament_ws/install" ]; then
#    echo "Building the workspace"
#    # Build the workspace
#    colcon build --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release' -Wall -Wextra -Wpedantic
#fi

# Print out the arguments passed to the container
echo "Provided arguments: $@"
# Execute the command passed to the container
exec "$@"
