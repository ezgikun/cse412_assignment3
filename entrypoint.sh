#!/bin/bash
set -e

# Source the base ROS environment
source /opt/ros/humble/setup.bash

# Source the installed environment of our workspace
source /ws/install/setup.bash

# Start the launch file [cite: 85]
ros2 launch launch/my_project.launch.py

# Allow CMD to run so that the container doesn't close after the script completes
exec "$@"
