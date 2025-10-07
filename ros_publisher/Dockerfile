# ROS Publisher - Ubuntu 24.04 with ROS 2 Jazzy
FROM ubuntu:24.04

ARG ROS_DISTRO=jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    python3-numpy \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy with image_transport and GUI tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-rclpy \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-theora-image-transport \
    ros-jazzy-image-view \
    ros-jazzy-tf2-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-image-view \
    python3-rosdep \
    python3-opencv \
    python3-colcon-common-extensions \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update || echo "rosdep already initialized"

# Install Python dependencies
RUN pip3 install numpy

# Create working directory
WORKDIR /app

# Copy entire ROS package
COPY scripts/ /app/scripts/
COPY ros_publisher/entrypoint.sh /app/
COPY config/ /app/config/
COPY launch/ /app/launch/
COPY package.xml /app/
COPY CMakeLists.txt /app/
RUN chmod +x /app/entrypoint.sh

# Set up ROS 2 workspace with proper package structure
RUN mkdir -p /colcon_ws/src/rtsp_cam && \
    cp -r /app/* /colcon_ws/src/rtsp_cam/ && \
    cd /colcon_ws && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source ROS 2 environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["ros2", "launch", "rtsp_cam", "rtsp_camera.launch.py"]
