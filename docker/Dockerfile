# Dockerfile for RTSP Camera ROS 1 Noetic on Jetson Orin NX
# Based on NVIDIA L4T (Linux for Tegra) with ROS 1 Noetic

ARG L4T_VERSION=r36.4.0
ARG ROS_DISTRO=noetic

# Use NVIDIA L4T base image for Jetson Orin NX
FROM nvcr.io/nvidia/l4t-jetpack:${L4T_VERSION}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PYTHONUNBUFFERED=1
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Set up ROS 1 Noetic repository
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-dev \
    python3-setuptools \
    python3-numpy \
    python3-requests \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    libgirepository1.0-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-nvv4l2 \
    nvidia-l4t-gstreamer \
    gstreamer1.0-plugins-nvcodec \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 1 Noetic
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 1 dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-message-runtime \
    ros-${ROS_DISTRO}-message-generation \
    ros-${ROS_DISTRO}-rospy \
    ros-${ROS_DISTRO}-rostest \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    numpy \
    PyGObject

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy package source
COPY . /catkin_ws/src/rtsp_cam/

# Build the package
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set environment variables for Jetson
ENV DECODER_TYPE=jetson
ENV CUDA_VISIBLE_DEVICES=0

# Expose ROS master port
EXPOSE 11311

# Set working directory
WORKDIR /catkin_ws

# Default command (can be overridden in docker-compose)
CMD ["bash"]
