cmake_minimum_required(VERSION 3.0.2)
project(rtsp_cam)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  compressed_image_transport
  theora_image_transport
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    image_transport
    cv_bridge
    compressed_image_transport
    theora_image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python executables (conditionally based on what exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/rtsp_streamer_node.py")
  catkin_install_python(PROGRAMS
    scripts/rtsp_streamer_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/rtsp_publisher_node.py")
  catkin_install_python(PROGRAMS
    scripts/rtsp_publisher_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.json" PATTERN "*.yaml"
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add tests here if needed
endif()