cmake_minimum_required(VERSION 2.8.3)
project(prac2cram)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  rospy
  std_msgs
  message_generation
)


## add new message and service files here! 

add_service_files(

  FILES
  Prac2Cram.srv
)

add_message_files(

  FILES
  ActionRole.msg
  ActionCore.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment_6_1
  CATKIN_DEPENDS roslisp std_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

