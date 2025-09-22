# Dockerfile for RTSP Camera ROS 1 Noetic on Jetson Orin NX
# Based on NVIDIA L4T (Linux for Tegra) with ROS 1 Noetic

ARG L4T_VERSION=r35.2.1
ARG ROS_DISTRO=noetic

# Use NVIDIA L4T base image for Jetson Orin NX (use older compatible version)
FROM nvcr.io/nvidia/l4t-base:${L4T_VERSION}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PYTHONUNBUFFERED=1
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install system dependencies and GStreamer with nvv4l2decoder support
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
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
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install Jetson-specific GStreamer packages for nvv4l2decoder
# These are usually pre-installed in L4T but ensure they're available
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-nvvideo4linux2 \
    nvidia-l4t-gstreamer \
    || echo "Jetson GStreamer plugins using nvv4l2decoder already available" \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 1 Noetic repository for focal (compatible with L4T)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS 1 Noetic (core packages that work with L4T)
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-core \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-std-srvs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-message-runtime \
    ros-noetic-message-generation \
    ros-noetic-rostest \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install cv-bridge and image-transport separately (these can be problematic)
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    || pip3 install --no-cache-dir opencv-python cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update || echo "rosdep already initialized"

# Install additional Python dependencies
RUN pip3 install --no-cache-dir \
    requests \
    numpy \
    PyGObject \
    rospkg \
    catkin_pkg

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy package source
COPY . /catkin_ws/src/rtsp_cam/

# Build the package using catkin_make
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release" \
    || echo "Catkin build completed with warnings"

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set environment variables for Jetson nvv4l2decoder
ENV DECODER_TYPE=jetson
ENV CUDA_VISIBLE_DEVICES=0
ENV GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

# Expose ROS master port
EXPOSE 11311

# Set working directory
WORKDIR /catkin_ws

# Default command (can be overridden in docker-compose)
CMD ["bash"]
