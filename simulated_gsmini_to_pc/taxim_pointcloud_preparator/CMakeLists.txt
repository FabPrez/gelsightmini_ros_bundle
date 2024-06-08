cmake_minimum_required(VERSION 3.0.2)
project(taxim_pointcloud_preparator)

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
add_service_files(
  FILES
  SavePointcloud.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${PCL_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(taxim_pointcloud_preparator src/taxim_pointcloud_preparator.cpp src/taxim_pointcloud_preparator_node.cpp)
target_link_libraries(taxim_pointcloud_preparator ${catkin_LIBRARIES})
