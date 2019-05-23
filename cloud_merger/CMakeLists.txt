cmake_minimum_required(VERSION 2.8.3)
project(cloud_merger)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cloud_merger
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs cv_bridge image_transport
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(cloud_merger src/node.cpp src/CloudMerger.cpp)

target_link_libraries(cloud_merger ${catkin_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
