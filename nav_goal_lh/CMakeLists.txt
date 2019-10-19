cmake_minimum_required(VERSION 2.8.3)
project(nav_goal_lh)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  sensor_msgs
#  message_filters
#  nav_msgs
#  message_generation
  gps_common
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   position.msg
   current_gps.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
#  nav_msgs
#  sensor_msgs
  std_msgs
)

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
add_library(nav_goal SHARED

   src/ekf.cpp
   src/fusion.cpp
   src/utils.cpp
   src/pid.cpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES nav_goal
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  #      ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control src/control.cpp)
add_executable(getgps src/getgps.cpp)

target_link_libraries(getgps ${catkin_LIBRARIES})
target_link_libraries(control ${catkin_LIBRARIES} nav_goal)
