cmake_minimum_required(VERSION 3.0.2)
project(rtsp_cam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  cv_bridge
  image_transport
  camera_info_manager
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS rospy sensor_msgs std_msgs std_srvs geometry_msgs cv_bridge image_transport camera_info_manager message_runtime
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python modules
catkin_install_python(PROGRAMS
  scripts/rtsp_cam_node.py
  scripts/rtsp_ptz_controller.py
  scripts/rtsp_photo_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY
  launch/
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install Python package
install(DIRECTORY
  rtsp_cam/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
) 