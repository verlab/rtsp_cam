#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Set decoder type from environment variable
if [ ! -z "$DECODER_TYPE" ]; then
    echo "Setting decoder type to: $DECODER_TYPE"
    # This could be used to modify configuration at runtime
fi

# Create photo directory if it doesn't exist
mkdir -p /tmp/rtsp_cam_photos

# Print some info
echo "RTSP Camera ROS 2 Package"
echo "ROS Distribution: $ROS_DISTRO"
echo "Decoder Type: ${DECODER_TYPE:-software}"
echo "ROS Domain ID: ${ROS_DOMAIN_ID:-0}"

# Execute the command
exec "$@"
