# Docker Deployment for RTSP Camera Package

This directory contains Docker files for deploying the RTSP Camera ROS 2 package with different hardware acceleration options.

## Supported Configurations

- **CPU**: Software decoding using `avdec_h264/h265`
- **NVIDIA GPU**: Hardware decoding using `nvh264dec/nvh265dec`
- **Jetson Nano**: Hardware decoding using `nvv4l2decoder`

## Quick Start

### CPU-only deployment
```bash
docker-compose --profile cpu up -d
```

### NVIDIA GPU deployment
```bash
# Requires nvidia-docker2
docker-compose --profile nvidia up -d
```

### Jetson Nano deployment
```bash
docker-compose --profile jetson up -d
```

### Development mode
```bash
docker-compose --profile dev up -d
docker exec -it rtsp_cam_dev bash
```

## Configuration

1. Edit `../config/rtsp_cam_config.yaml` with your camera settings
2. Set your RTSP URLs, credentials, and decoder preferences
3. Launch with the appropriate profile

## Building Images

### Build all variants
```bash
# CPU version
docker build -f Dockerfile --target cpu -t rtsp_cam:cpu-jazzy ..

# NVIDIA version
docker build -f Dockerfile --target nvidia -t rtsp_cam:nvidia-jazzy ..

# Jetson version
docker build -f Dockerfile --target jetson -t rtsp_cam:jetson-jazzy ..
```

## Environment Variables

- `ROS_DOMAIN_ID`: ROS 2 domain ID (default: 0)
- `DECODER_TYPE`: Decoder type (software, nvidia, jetson)
- `NVIDIA_VISIBLE_DEVICES`: GPU visibility for NVIDIA runtime

## Volumes

- Configuration: `../config/rtsp_cam_config.yaml` → `/ros2_ws/src/rtsp_cam/config/rtsp_cam_config.yaml`
- Photos: `/tmp/rtsp_cam_photos` → `/tmp/rtsp_cam_photos`

## Topics and Services

The containerized service exposes the same ROS 2 interface:

- `/camera/main/image_raw` - Main stream images
- `/camera/sub/image_raw` - Secondary stream images  
- `/camera/ptz_cmd` - PTZ control commands
- `/camera/take_photo` - Photo capture service

## Testing

Test camera connectivity:
```bash
# View images
ros2 run rqt_image_view rqt_image_view /camera/main/image_raw

# Control PTZ
ros2 topic pub /camera/ptz_cmd geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}}"

# Take photo
ros2 service call /camera/take_photo std_srvs/srv/Trigger
```
