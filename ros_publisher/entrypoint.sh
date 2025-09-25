#!/bin/bash
set -e

echo "=== ROS Publisher Starting ==="

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Check ROS environment
echo "ROS_MASTER_URI: ${ROS_MASTER_URI:-http://localhost:11311}"
echo "ROS_HOSTNAME: ${ROS_HOSTNAME:-localhost}"

# Wait for ROS master
echo "Waiting for ROS master..."
until timeout 1 bash -c "echo >/dev/tcp/${ROS_MASTER_URI#http://}" 2>/dev/null || timeout 1 bash -c "echo >/dev/tcp/localhost/11311" 2>/dev/null; do
    echo "ROS master not available, waiting..."
    sleep 2
done

echo "ROS master found, starting publisher..."

# Execute main command
exec "$@"
