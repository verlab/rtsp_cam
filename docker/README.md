# Docker Deployment for RTSP Camera Package

This directory contains Docker files for deploying the RTSP Camera ROS 2 package with profile-based hardware configurations.

## Supported Configurations

- **CPU**: Software decoding using `avdec_h264/h265`
- **NVIDIA GPU**: Hardware decoding using `nvh264dec/nvh265dec`
- **Jetson Nano**: Hardware decoding using `nvv4l2decoder`
- **Development**: Interactive development environment

## Quick Start

The Docker Compose configuration uses profiles to handle different hardware platforms with a single service definition.

### CPU-only deployment
```bash
docker compose --profile cpu up -d
```

### NVIDIA GPU deployment
```bash
# Requires nvidia-docker2
docker compose --profile nvidia up -d
```

### Jetson Nano deployment
```bash
docker compose --profile jetson up -d
```

### Development mode
```bash
docker compose --profile dev up -d
docker exec -it rtsp_cam_dev bash
```

## Configuration

1. **Camera Settings**: Edit `../config/rtsp_cam_config.yaml` with your camera settings
2. **Environment Variables**: Use environment variables to customize deployment
3. **Photo Directory**: Set `PHOTO_DIR` environment variable to change photo storage location

### Using Environment Variables

You can customize deployments using environment variables:

```bash
# CPU deployment with custom photo directory
PHOTO_DIR=/custom/photo/path docker compose --profile cpu up -d

# NVIDIA GPU deployment with custom ROS domain
ROS_DOMAIN_ID=5 docker compose --profile nvidia up -d

# Jetson Nano deployment with custom settings
ROS_DOMAIN_ID=10 PHOTO_DIR=/jetson/photos docker compose --profile jetson up -d

# Development mode with custom ROS distribution
ROS_DISTRO=humble docker compose --profile dev up -d
```

## Architecture

The new Docker Compose structure uses:

- **Single service definition** with profile-based hardware selection
- **Environment variables** for platform-specific configurations
- **Multi-stage Dockerfile** with targeted builds for each platform
- **Conditional configurations** for devices, runtimes, and volumes

### Profile Benefits

✅ **Simplified Management**: One service instead of four separate services  
✅ **Cleaner Syntax**: `--profile cpu` vs managing multiple service names  
✅ **Environment Consistency**: Standardized configuration across platforms  
✅ **Easier Maintenance**: Single point of configuration changes  

## Building Images

### Build specific variants
```bash
# CPU version
docker build -f Dockerfile --target cpu -t rtsp_cam:cpu-jazzy ..

# NVIDIA version
docker build -f Dockerfile --target nvidia -t rtsp_cam:nvidia-jazzy ..

# Jetson version
docker build -f Dockerfile --target jetson -t rtsp_cam:jetson-jazzy ..
```

### Build with profiles
```bash
# Profiles will automatically build the correct target
docker compose --profile nvidia build
```

## Environment Variables

### Core Variables
- `ROS_DOMAIN_ID`: ROS 2 domain ID (default: 0)
- `PHOTO_DIR`: Photo storage directory (default: `/tmp/rtsp_cam_photos`)
- `ROS_DISTRO`: ROS 2 distribution (default: jazzy)

### Platform-Specific Variables
- `DECODER_TYPE`: Decoder type (software, nvidia, jetson)
- `NVIDIA_VISIBLE_DEVICES`: GPU visibility for NVIDIA runtime
- `NVIDIA_DRIVER_CAPABILITIES`: NVIDIA driver capabilities

## Volumes

- **Configuration**: `../config/rtsp_cam_config.yaml` → `/ros2_ws/src/rtsp_cam/config/rtsp_cam_config.yaml`
- **Photos**: `$PHOTO_DIR` → `/tmp/rtsp_cam_photos`
- **Development**: `../` → `/ros2_ws/src/rtsp_cam` (dev profile only)
- **Jetson Libraries**: `/usr/lib/aarch64-linux-gnu/tegra` (Jetson profile only)

## ROS 2 Interface

The containerized service exposes the same ROS 2 interface across all platforms:

### Topics
- `/camera/main/image_raw` - Main stream images
- `/camera/sub/image_raw` - Secondary stream images  
- `/camera/ptz_cmd` - PTZ control commands

### Services
- `/camera/take_photo` - Photo capture service

## Testing

Test camera connectivity from host system:
```bash
# View images
ros2 run rqt_image_view rqt_image_view /camera/main/image_raw

# Control PTZ
ros2 topic pub /camera/ptz_cmd geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}}"

# Take photo
ros2 service call /camera/take_photo std_srvs/srv/Trigger
```

## Troubleshooting

### Common Issues

**NVIDIA GPU not detected:**
```bash
# Verify nvidia-docker2 installation
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

**Jetson devices not accessible:**
```bash
# Check device permissions
ls -la /dev/nvhost-*
# May need to run with --privileged for development
```

**Configuration not updating:**
```bash
# Rebuild with no-cache to pick up config changes
docker compose --profile cpu build --no-cache
```
