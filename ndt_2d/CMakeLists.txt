cmake_minimum_required(VERSION 2.8.3)
project(ndt_2d)

add_compile_options(-std=c++11 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  sensor_msgs
  visualization_msgs
  cmake_modules
  msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres 1.4.1 REQUIRED)

add_message_files(
  FILES
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ndt_service_node 
  src/ndt_service.cpp
)

#add_executable(${PROJECT_NAME}_save_laserscan_node src/save_laserscan.cpp)

add_executable(${PROJECT_NAME}_test_registration_node 
  src/test_registration.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(ndt_service_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

#target_link_libraries(${PROJECT_NAME}_save_laserscan_node
#   ${catkin_LIBRARIES}
#)

target_link_libraries(${PROJECT_NAME}_test_registration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(map_test test/map_test.cpp)
  catkin_add_gtest(math_test test/math_test.cpp)
  catkin_add_gtest(scan_test test/scan_test.cpp)
  target_link_libraries(map_test ${catkin_LIBRARIES})
  target_link_libraries(math_test ${catkin_LIBRARIES})
  target_link_libraries(scan_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()

