# Multi-stage Dockerfile for RTSP Camera ROS 2 Package
# Supports CPU, NVIDIA GPU, and Jetson Nano configurations

ARG ROS_DISTRO=jazzy
ARG UBUNTU_VERSION=24.04

# Base stage with ROS 2 Jazzy
FROM ros:${ROS_DISTRO}-ros-base AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    libgirepository1.0-dev \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    curl \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    requests \
    numpy

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy package source
COPY . /ros2_ws/src/rtsp_cam/

# Build the package
RUN cd /ros2_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select rtsp_cam --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# =================== NVIDIA GPU Stage ===================
FROM base as nvidia

# Install NVIDIA GStreamer plugins
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-nvcodec \
    libnvidia-encode1 \
    libnvidia-decode1 \
    && rm -rf /var/lib/apt/lists/*

ENV DECODER_TYPE=nvidia

WORKDIR /ros2_ws
EXPOSE 11311
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rtsp_cam", "rtsp_cam.launch.py"]

# =================== Jetson Nano Stage ===================
FROM base as jetson

# Note: Jetson-specific packages are only available when running on actual Jetson hardware
# with NVIDIA's L4T repositories. For cross-compilation or building on x86, we skip these
# packages and rely on runtime volume mounts for Tegra libraries.

# Install available GStreamer plugins that work across platforms
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

# Try to install Jetson-specific packages if available (will fail silently if not on Jetson)
RUN apt-get update && \
    (apt-get install -y nvidia-l4t-gstreamer gstreamer1.0-nvv4l2 || true) && \
    rm -rf /var/lib/apt/lists/*

ENV DECODER_TYPE=jetson

WORKDIR /ros2_ws
EXPOSE 11311
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rtsp_cam", "rtsp_cam.launch.py"]

# =================== CPU-only Stage ===================
FROM base as cpu

ENV DECODER_TYPE=software

WORKDIR /ros2_ws
EXPOSE 11311
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rtsp_cam", "rtsp_cam.launch.py"]
