#!/bin/bash
set -e

# Source the ROS 2 setup script
source /opt/ros/jazzy/setup.bash

# Source the workspace setup script if it exists
if [ -f "/root/colcon_ws/install/setup.bash" ]; then
  source /root/colcon_ws/install/setup.bash
fi

exec "$@"