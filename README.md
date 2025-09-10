# RTSP Camera ROS 2 Package

A comprehensive ROS 2 package for generic RTSP cameras with streaming, PTZ control, and photo capture capabilities. Supports multiple hardware decoding options including CPU, NVIDIA GPU, and Jetson Nano.

## Features

- **Universal RTSP Support**: Works with any RTSP-compatible camera
- **Multiple Decoder Options**: Software, NVIDIA GPU, Jetson Nano hardware acceleration
- **Dual Stream Support**: Main and secondary RTSP streams with H.264/H.265 encoding
- **High Performance**: Uses GStreamer for efficient video processing
- **PTZ Control**: Pan, Tilt, Zoom control via HTTP interfaces (when supported)
- **Photo Capture**: Service-based photo capture with multiple methods
- **Configurable**: Easy configuration through YAML files
- **Docker Support**: Multi-platform containerization with hardware acceleration
- **ROS 2 Native**: Built with modern ROS 2 best practices

## Hardware Decoder Support

| Decoder Type | GStreamer Elements | Use Case |
|--------------|-------------------|----------|
| `software` | `avdec_h264`, `avdec_h265` | CPU-only systems, development |
| `nvidia` | `nvh264dec`, `nvh265dec` | NVIDIA GPU systems (RTX, GTX, etc.) |
| `jetson` | `nvv4l2decoder` | NVIDIA Jetson platforms (Nano, Xavier, Orin) |
| `vaapi` | `vaapih264dec`, `vaapih265dec` | Intel/AMD GPU systems |

## Quick Start with Docker

### CPU-only deployment
```bash
cd rtsp_cam/docker
docker-compose --profile cpu up -d
```

### NVIDIA GPU deployment
```bash
# Requires nvidia-docker2
cd rtsp_cam/docker
docker-compose --profile nvidia up -d
```

### Jetson Nano deployment
```bash
cd rtsp_cam/docker
docker-compose --profile jetson up -d
```

## Installation from Source

### Prerequisites

```bash
# Install ROS 2 Jazzy (Ubuntu 24.04)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop

# Install GStreamer and dependencies
sudo apt install \
    python3-pip python3-opencv \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools \
    libgirepository1.0-dev python3-gi python3-requests

# Install ROS 2 packages
sudo apt install \
    ros-jazzy-cv-bridge ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs ros-jazzy-std-srvs

# For NVIDIA GPU support (optional)
sudo apt install gstreamer1.0-plugins-nvcodec

# For Jetson support (optional, on Jetson devices)
sudo apt install nvidia-l4t-gstreamer gstreamer1.0-nvv4l2
```

### Build Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone package
git clone <repository-url> rtsp_cam

# Build
cd ~/ros2_ws
colcon build --packages-select rtsp_cam
source install/setup.bash
```

## Configuration

Edit `config/rtsp_cam_config.yaml`:

```yaml
rtsp_cam:
  ros__parameters:
    # Camera connection
    camera_ip: "192.168.1.100"
    username: "admin" 
    password: "password"
    
    # RTSP URLs (customize for your camera)
    main_stream_url: "rtsp://admin:password@192.168.1.100:554/stream1"
    sub_stream_url: "rtsp://admin:password@192.168.1.100:554/stream2"
    
    # Hardware decoding: "software", "nvidia", "jetson", "vaapi"
    decoder_type: "software"
    
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

### Launch Complete System

```bash
ros2 launch rtsp_cam rtsp_cam.launch.py
```

### Individual Nodes

```bash
# Camera streaming
ros2 run rtsp_cam rtsp_cam_node

# PTZ controller (if camera supports it)
ros2 run rtsp_cam rtsp_ptz_controller

# Photo service
ros2 run rtsp_cam rtsp_photo_service
```

### ROS 2 Interface

#### Published Topics
- `/camera/main/image_raw` (sensor_msgs/Image) - Main stream
- `/camera/sub/image_raw` (sensor_msgs/Image) - Secondary stream  
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera info
- `/camera/ptz_status` (std_msgs/String) - PTZ status (if available)

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