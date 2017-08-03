cmake_minimum_required(VERSION 2.8.3)
project(prac2cram)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  roslisp_utilities
  rospy
  std_msgs
  alexandria
  cram_language
  yason
  message_generation
)


## add new message and service files here! 

add_service_files(

  FILES
  CancelSim.srv
  Prac2Cram.srv
  Prac2Cram2.srv
  POSTRequest.srv
)

add_message_files(

  FILES
  ActionRole.msg
  ActionCore.msg
  CRAMTick.msg
  Task.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment_6_1
  CATKIN_DEPENDS roslisp roslisp_utilities rospy alexandria std_msgs message_runtime cram_language yason
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

