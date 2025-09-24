# Dockerfile for RTSP Camera ROS 1 Noetic on Jetson Orin Nano
# Using Ubuntu 20.04 base with Jetson runtime libraries mounted

ARG ROS_DISTRO=noetic

# Jetson Deployment Dockerfile - Build this on Jetson Orin Nano  
# Using 35.3.1 - verified available and signed for Orin Nano + ROS 1 Noetic
FROM nvcr.io/nvidia/l4t-base:35.4.1

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PYTHONUNBUFFERED=1
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install system dependencies and GStreamer
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    python3-pip \
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
    libffi-dev \
    git \
    build-essential \
    cmake \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install additional GStreamer dependencies (Jetson packages already in L4T base)
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    && rm -rf /var/lib/apt/lists/*

# Note: libgomp TLS fix will be handled at runtime in entrypoint

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

# Install image-transport and camera-info-manager (cv-bridge removed)
RUN apt-get update && apt-get install -y \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update || echo "rosdep already initialized"

# Install additional Python dependencies (avoid opencv-python - use Jetson's)
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

# Copy entrypoint script
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Install the Python package and build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws/src/rtsp_cam && \
    pip3 install -e . && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release && \
    echo 'Catkin workspace built successfully' && \
    ls -la /catkin_ws/devel/"

# Source the workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set environment variables for Jetson hardware acceleration
ENV DECODER_TYPE=jetson
ENV CUDA_VISIBLE_DEVICES=0
ENV GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/usr/lib/aarch64-linux-gnu/tegra-mmapi
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-mmapi:/opt/nvidia/vpi2/lib64:/usr/lib/aarch64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/catkin_ws/src:/usr/lib/python3/dist-packages:$PYTHONPATH
ENV OPENCV_LOG_LEVEL=ERROR
# Jetson specific environment
ENV NVMEDIA_ENABLE_TRACKING=1
ENV NVMEDIA_ENABLE_PROFILING=0

# Expose ROS master port
EXPOSE 11311

# Set working directory
WORKDIR /catkin_ws

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command (can be overridden in docker-compose)
CMD []
