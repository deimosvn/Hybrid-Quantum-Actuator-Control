#!/usr/bin/env bash
# Source ROS 2 and the built workspace, then exec the given command.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/ros_ws/install/setup.bash" ]; then
  source "/ros_ws/install/setup.bash"
fi

exec "$@"
