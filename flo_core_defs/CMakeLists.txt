cmake_minimum_required(VERSION 2.8.3)
project(flo_core_defs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  flo_humanoid_defs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
     FILES
     GameCommand.msg
     GameCommandOptions.msg
     GameDef.msg
     GameState.msg
     Pose.msg
     PoseSeq.msg
     StepDef.msg
     DBUpdate.msg
     GameAction.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetPoseID.srv
   GetPoseSeqID.srv
   SearchPose.srv
   SearchPoseSeq.srv
   SearchUtterance.srv
   SetPose.srv
   SetPoseSeq.srv
   SetUtterance.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   flo_humanoid_defs
 )


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flo_humanoid
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
CATKIN_DEPENDS
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
