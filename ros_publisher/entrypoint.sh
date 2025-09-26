#!/bin/bash
set -e

echo "=== ROS Publisher Starting ==="

# Source ROS environment and workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Check ROS environment
echo "ROS_MASTER_URI: ${ROS_MASTER_URI:-http://localhost:11311}"
echo "ROS_HOSTNAME: ${ROS_HOSTNAME:-localhost}"
echo "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"

# Set working directory to catkin workspace
cd /catkin_ws

# Execute main command
exec "$@"
