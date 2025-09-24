#!/bin/bash
set -e

# Entrypoint script for RTSP Camera ROS node on Jetson
echo "Starting RTSP Camera ROS node on Jetson..."

# Check if we're running on Jetson
if [ -f /etc/nv_tegra_release ]; then
    echo "Detected Jetson device:"
    cat /etc/nv_tegra_release
else
    echo "Warning: Not running on Jetson device, hardware acceleration may not be available"
fi

# Check for NVIDIA runtime
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA runtime detected:"
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || echo "Warning: nvidia-smi failed"
else
    echo "Warning: NVIDIA runtime not available"
fi

# Set up library paths for Jetson native environment
export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-mmapi:/opt/nvidia/vpi2/lib64:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"
export GST_PLUGIN_PATH="/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/lib/aarch64-linux-gnu/tegra-mmapi:${GST_PLUGIN_PATH}"

# Fix libgomp TLS issue for GStreamer libav plugin (if libgomp exists)
if [ -f "/usr/lib/aarch64-linux-gnu/libgomp.so.1" ]; then
    export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1:${LD_PRELOAD}"
    echo "Applied libgomp TLS fix"
else
    echo "libgomp not found, skipping TLS fix"
fi

# Check GStreamer installation
echo "Checking GStreamer installation..."
gst-inspect-1.0 --version

# Check for key GStreamer plugins
echo "Checking for hardware decoder plugins..."

# Check for nvv4l2decoder (Jetson hardware decoder)
if gst-inspect-1.0 nvv4l2decoder &> /dev/null; then
    echo "✓ nvv4l2decoder plugin found (Jetson hardware decoder)"
    export PREFERRED_DECODER="jetson"
else
    echo "✗ nvv4l2decoder plugin not found"
    echo "Note: This is expected when running Ubuntu base image instead of L4T"
fi

# Check for nvh264dec (NVIDIA GPU decoder)
if gst-inspect-1.0 nvh264dec &> /dev/null; then
    echo "✓ nvh264dec plugin found (NVIDIA GPU decoder)"
    if [ -z "$PREFERRED_DECODER" ]; then
        export PREFERRED_DECODER="nvidia"
    fi
else
    echo "✗ nvh264dec plugin not found"
fi

# Check for software decoder
if gst-inspect-1.0 avdec_h264 &> /dev/null; then
    echo "✓ avdec_h264 plugin found (software decoder)"
    if [ -z "$PREFERRED_DECODER" ]; then
        export PREFERRED_DECODER="software"
    fi
else
    echo "✗ avdec_h264 plugin not found - this is critical!"
    exit 1
fi

echo "Using decoder type: ${PREFERRED_DECODER}"

# Set up ROS environment
echo "Setting up ROS environment..."
source /opt/ros/noetic/setup.bash

# Source catkin workspace if it exists
if [ -f /catkin_ws/devel/setup.bash ]; then
    echo "Sourcing catkin workspace..."
    source /catkin_ws/devel/setup.bash
else
    echo "Warning: Catkin workspace not found, using ROS core only"
fi

# Override decoder type parameter if preferred decoder is set
if [ -n "$PREFERRED_DECODER" ]; then
    echo "Setting decoder_type parameter to: $PREFERRED_DECODER"
fi

# Start ROS core if not already running (for standalone mode)
if [ "$1" = "roscore" ]; then
    echo "Starting ROS core..."
    exec roscore
fi

# If no arguments provided, start the default launch file
if [ $# -eq 0 ]; then
    echo "Starting RTSP camera launch file with decoder type: ${PREFERRED_DECODER:-jetson}"
    exec roslaunch rtsp_cam rtsp_cam.launch decoder_type:="${PREFERRED_DECODER:-jetson}"
fi

# Execute the provided command
echo "Executing command: $@"
exec "$@"