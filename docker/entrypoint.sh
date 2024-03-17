#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"
ROS_DOMAIN_ID=24

# Source the base workspace, if built
if [ -f /ros2_ws/install/setup.bash ]
then
  source /ros2_ws/install/setup.bash
  echo "Sourced base workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  source /overlay_ws/install/setup.bash
  echo "Sourced overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"