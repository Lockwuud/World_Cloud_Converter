cmake_minimum_required(VERSION 3.0.2)
project(world_cloud_converter)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

add_definitions(-DORIGIN_AT_CORNER)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  message_generation
  rog_map
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ../../../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # 添加 PCL 的 include 路径
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        -ldw
)

## Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})

add_library(kdtree
        include/utils/kdtree.c
)

## Declare a C++ executable and add source files
add_executable(${PROJECT_NAME}_node
  src/cloud_converter.cpp
)

## Link the executable with catkin libraries and PCL
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}  # 链接 PCL 的库
)

add_executable(map_node
        src/map_node.cpp
)
target_link_libraries(map_node
        ${THIRD_PARTY}
)