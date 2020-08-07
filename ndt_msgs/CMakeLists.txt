cmake_minimum_required(VERSION 2.8.3)
project(ndt_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NDT2dParameters.msg
)

## Generate services in the 'srv' folder
add_service_files( 
  FILES
  Registration.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

link_directories(/usr/local/include)

add_executable(message_test test/message_test.cpp)
