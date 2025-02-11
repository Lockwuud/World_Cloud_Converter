cmake_minimum_required(VERSION 3.0.2)
project(world_cloud_converter)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -DMY_DEF=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMY_DEF=1" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMY_DEF=1" )

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
)

find_package(CUDA REQUIRED)
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
  ${PCL_INCLUDE_DIRS} 
  ${CUDA_INCLUDE_DIRS} 
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        -ldw
)

## Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})

## 时间软同步
add_executable(${PROJECT_NAME}_node
  src/cloud_converter.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}  # 链接 PCL 的库
)

## 时间软同步——CUDA并行
cuda_add_executable(cloud_converter_cuda_node src/cloud_converter_cuda.cu)
target_link_libraries(cloud_converter_cuda_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}  
  ${CUDA_LIBRARIES}
)

