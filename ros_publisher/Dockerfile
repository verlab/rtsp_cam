# ROS Publisher - Ubuntu 20.04 with ROS 1 Noetic
FROM ubuntu:20.04

ARG ROS_DISTRO=noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 1 Noetic repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS 1 Noetic with image_transport and GUI tools
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-core \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-compressed-image-transport \
    ros-noetic-theora-image-transport \
    ros-noetic-image-view \
    ros-noetic-tf2-ros \
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-image-view \
    python3-rosdep \
    python3-opencv \
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

# Set up ROS workspace with proper package structure
RUN mkdir -p /catkin_ws/src/rtsp_cam && \
    cp -r /app/* /catkin_ws/src/rtsp_cam/ && \
    cd /catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["roslaunch", "rtsp_cam", "rtsp_camera.launch"]
