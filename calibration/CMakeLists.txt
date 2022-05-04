cmake_minimum_required(VERSION 2.8.3)
project(calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-g -O3 -ftree-vectorize)

# Set build type
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  # base
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  tf
  tf_conversions
  message_filters
  message_generation
)

find_package(base)
find_package(PCL 1.8 REQUIRED)

# add_message_files(
#   FILES
#   CalibrationReference.msg
#   CalibrationResultInfo.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
) 

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)


add_executable(calibration_node  
  src/calibration_node.cpp
  src/utils.cpp
)
add_dependencies(calibration_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
  ${base_EXPORTED_TARGETS}
)
target_link_libraries(calibration_node
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)