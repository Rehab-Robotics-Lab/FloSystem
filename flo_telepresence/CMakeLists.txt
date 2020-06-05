cmake_minimum_required(VERSION 2.8.3)
project(flo_telepresence)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flo_telepresence
#  CATKIN_DEPENDS audio_common cv_camera realsense2_camera roslibjs rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   src/display_cv.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
