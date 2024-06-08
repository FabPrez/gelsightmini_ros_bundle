cmake_minimum_required(VERSION 3.0.2)
project(pc_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  pcl_ros
  pcl_conversions
)

catkin_package(
)

include_directories(
  include
  ${PCL_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pc_visualizer src/pc_visualizer.cpp src/pc_visualizer_node.cpp)
target_link_libraries(pc_visualizer ${catkin_LIBRARIES})
