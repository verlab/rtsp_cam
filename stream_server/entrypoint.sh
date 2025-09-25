#!/bin/bash
set -e

echo "=== Stream Server Starting ==="

# Check for NVIDIA hardware
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA runtime detected"
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || echo "Warning: nvidia-smi failed"
else
    echo "Warning: NVIDIA runtime not available"
fi

# Check GStreamer hardware elements
echo "Checking GStreamer hardware elements..."
if gst-inspect-1.0 nvv4l2decoder &>/dev/null; then
    echo "✓ nvv4l2decoder available (Jetson hardware decoder)"
else
    echo "✗ nvv4l2decoder not found"
fi

if gst-inspect-1.0 nvh264dec &>/dev/null; then
    echo "✓ nvh264dec available"
else
    echo "✗ nvh264dec not found"
fi

if gst-inspect-1.0 nvvidconv &>/dev/null; then
    echo "✓ nvvidconv available (hardware video converter)"
else
    echo "✗ nvvidconv not found"
fi

# Check shared memory directory
mkdir -p /dev/shm
echo "Shared memory directory: /dev/shm"

# Execute main command
exec "$@"
