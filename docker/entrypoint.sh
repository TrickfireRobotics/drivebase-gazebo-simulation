#!/usr/bin/env bash
set -e
# Source ROS 2
source /opt/ros/humble/setup.bash || true
# Source workspace if built
if [ -f /ws/ros2_ws/install/setup.bash ]; then
  source /ws/ros2_ws/install/setup.bash
fi
exec "$@"