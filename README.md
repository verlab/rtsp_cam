# RTSP Camera ROS 1 Noetic Package

A comprehensive ROS 1 Noetic package for generic RTSP cameras with streaming, PTZ control, and photo capture capabilities. Optimized for Jetson Orin NX with hardware acceleration support.

## Features

- **Universal RTSP Support**: Works with any RTSP-compatible camera
- **Jetson Orin NX Optimized**: Hardware acceleration using nvv4l2decoder
- **Multiple Decoder Options**: Software, NVIDIA GPU, Jetson hardware acceleration
- **Dual Stream Support**: Main and secondary RTSP streams with H.264/H.265 encoding
- **High Performance**: Uses GStreamer for efficient video processing
- **PTZ Control**: Pan, Tilt, Zoom control via HTTP interfaces (when supported)
- **Photo Capture**: Service-based photo capture with multiple methods
- **Configurable**: Easy configuration through YAML files
- **ROS 1 Noetic Native**: Built for ROS 1 Noetic with catkin build system

## Hardware Decoder Support

| Decoder Type | GStreamer Elements | Use Case |
|--------------|-------------------|----------|
| `software` | `avdec_h264`, `avdec_h265` | CPU-only systems, development |
| `nvidia` | `nvh264dec`, `nvh265dec` | NVIDIA GPU systems (RTX, GTX, etc.) |
| `jetson` | `nvv4l2decoder` | NVIDIA Jetson platforms (Nano, Xavier, Orin NX) |
| `vaapi` | `vaapih264dec`, `vaapih265dec` | Intel/AMD GPU systems |

**Recommended for Jetson Orin NX**: Use `jetson` decoder type for optimal performance.

## Quick Start for Jetson Orin NX

### Docker-based Installation (Recommended)

The easiest way to run this package on Jetson Orin NX is using Docker with the L4T base image:

```bash
# Clone the repository
git clone <repository-url> rtsp_cam
cd rtsp_cam

# Build and run with Docker
cd docker
./build_jetson.sh
docker-compose -f docker-compose.jetson.yml up -d
```

### Manual Installation (Alternative)

If you prefer to install directly on the Jetson:

#### Prerequisites

```bash
# Install ROS 1 Noetic (Ubuntu 20.04)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Install GStreamer and dependencies
sudo apt install -y \
    python3-pip python3-opencv \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools \
    libgirepository1.0-dev python3-gi python3-requests

# Install ROS 1 packages
sudo apt install -y \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-camera-info-manager ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs ros-noetic-std-srvs

# For Jetson Orin NX support
sudo apt install -y \
    nvidia-l4t-gstreamer \
    gstreamer1.0-nvv4l2 \
    gstreamer1.0-plugins-nvcodec
```

#### Build Package

```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone package
git clone <repository-url> rtsp_cam

# Build
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Configuration

Edit `config/rtsp_cam_config.yaml`:

```yaml
# Camera connection
camera_ip: "192.168.1.100"
username: "admin" 
password: "password"

# RTSP URLs (customize for your camera)
main_stream_url: "rtsp://admin:password@192.168.1.100:554/stream1"
sub_stream_url: "rtsp://admin:password@192.168.1.100:554/stream2"

# Hardware decoding: "software", "nvidia", "jetson", "vaapi"
decoder_type: "jetson"  # Optimized for Jetson Orin NX

# Stream settings
main_stream_enabled: true
sub_stream_enabled: false
frame_rate: 30

# PTZ control (optional, for PTZ cameras)
ptz_enabled: true
http_port: 80

# Photo capture
photo_save_path: "/tmp/rtsp_cam_photos"
```

## Usage

### Docker (Recommended for Jetson)

```bash
# Start the complete system
docker-compose -f docker/docker-compose.jetson.yml up -d

# View logs
docker-compose -f docker/docker-compose.jetson.yml logs -f

# Stop the system
docker-compose -f docker/docker-compose.jetson.yml down

# Development mode
docker-compose -f docker/docker-compose.jetson.yml run rtsp-cam-dev
```

### Manual Installation

```bash
# Source the workspace
source ~/catkin_ws/devel/setup.bash

# Launch all nodes
roslaunch rtsp_cam rtsp_cam.launch

# Individual nodes
rosrun rtsp_cam rtsp_cam_node.py
rosrun rtsp_cam rtsp_ptz_controller.py
rosrun rtsp_cam rtsp_photo_service.py
```

### ROS 1 Interface

#### Published Topics
- `/camera/main/image_raw` (sensor_msgs/Image) - Main stream
- `/camera/sub/image_raw` (sensor_msgs/Image) - Secondary stream  
- `/camera/main/camera_info` (sensor_msgs/CameraInfo) - Main stream camera info
- `/camera/sub/camera_info` (sensor_msgs/CameraInfo) - Sub stream camera info
- `/camera/ptz_status` (std_msgs/String) - PTZ status (if available)
- `/camera/joint_states` (sensor_msgs/JointState) - PTZ joint states (if available)

#### Subscribed Topics
- `/camera/ptz_cmd` (geometry_msgs/Twist) - PTZ commands (if available)

#### Services
- `/camera/take_photo` (std_srvs/Trigger) - Capture photo

### Examples

#### View Camera Stream
```bash
ros2 run rqt_image_view rqt_image_view /camera/main/image_raw
```

#### Control PTZ (if available)
```bash
# Pan right
ros2 topic pub /camera/ptz_cmd geometry_msgs/msg/Twist "{linear: {y: 0.5}}"

# Tilt up  
ros2 topic pub /camera/ptz_cmd geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Zoom in
ros2 topic pub /camera/ptz_cmd geometry_msgs/msg/Twist "{linear: {z: 0.5}}"

# Stop
ros2 topic pub /camera/ptz_cmd geometry_msgs/msg/Twist "{}"
```

#### Capture Photo
```bash
ros2 service call /camera/take_photo std_srvs/srv/Trigger
```

## Performance Optimization

### Hardware Acceleration

Configure the appropriate decoder in your config:

```yaml
# For NVIDIA GPUs
decoder_type: "nvidia"

# For Jetson devices  
decoder_type: "jetson"

# For Intel/AMD GPUs
decoder_type: "vaapi"

# For CPU-only (fallback)
decoder_type: "software"
```

### Custom GStreamer Pipelines

For advanced users, you can specify custom pipelines:

```yaml
gst_pipeline_main: "rtspsrc location=rtsp://... ! rtph264depay ! nvh264dec ! videoconvert ! appsink"
```

## Docker Deployment

See [docker/README.md](docker/README.md) for detailed Docker deployment instructions.

### Quick Docker Commands

```bash
# CPU version
docker-compose --profile cpu up -d

# NVIDIA GPU version (requires nvidia-docker2)
docker-compose --profile nvidia up -d

# Jetson version
docker-compose --profile jetson up -d

# Development mode
docker-compose --profile dev up -d
docker exec -it rtsp_cam_dev bash
```

## Camera Compatibility

This package works with any RTSP-compatible camera including:

- IP cameras (Hikvision, Dahua, Axis, etc.)
- USB cameras with RTSP servers
- Network cameras with ONVIF support
- Security camera systems
- Web cameras with RTSP capability

### Common RTSP URL Formats

```bash
# Generic format
rtsp://username:password@ip:port/path

# Hikvision
rtsp://admin:password@192.168.1.100:554/Streaming/Channels/101

# Dahua
rtsp://admin:password@192.168.1.100:554/cam/realmonitor?channel=1&subtype=0

# Axis
rtsp://root:password@192.168.1.100:554/axis-media/media.amp
```

## Troubleshooting

### Common Issues

1. **No video stream**: Check RTSP URL, credentials, and network connectivity
2. **Poor performance**: Try hardware decoder or reduce resolution
3. **Build errors**: Ensure all dependencies are installed
4. **PTZ not working**: PTZ requires camera-specific HTTP commands

### Debug Commands

```bash
# Test RTSP stream
ffplay rtsp://username:password@camera_ip:554/stream_path

# Check GStreamer capabilities  
gst-inspect-1.0 | grep h264

# View ROS 2 topics
ros2 topic list
ros2 topic echo /camera/main/image_raw
```

### Performance Tuning

```yaml
# Reduce latency
buffer_size: 1
frame_rate: 15

# Custom pipeline for low latency
gst_pipeline_main: "rtspsrc location=... latency=0 ! rtph264depay ! h264parse ! nvh264dec ! videoconvert ! appsink max-buffers=1 drop=true"
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable  
5. Submit a pull request

## License

This project is licensed under the MIT License.

## Author

- **Name**: rezeck
- **Email**: rezeck@ufmg.br

## Acknowledgments

Based on the original Jidetech camera driver, generalized for universal RTSP camera support.