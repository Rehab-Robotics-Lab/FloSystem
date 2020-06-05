cmake_minimum_required(VERSION 2.8.3)
project(flo_humanoid_defs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
     FILES
   JointTarget.msg
 )

## Generate actions in the 'action' folder
add_action_files(
     DIRECTORY action
   FILES
   Move.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flo_humanoid
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
CATKIN_DEPENDS  std_msgs  actionlib_msgs message_runtime
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
