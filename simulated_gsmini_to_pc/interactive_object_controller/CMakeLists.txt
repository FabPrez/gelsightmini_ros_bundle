cmake_minimum_required(VERSION 3.0.2)
project(interactive_object_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
  taxim_pointcloud_preparator
)

generate_dynamic_reconfigure_options(
  config/object_controller.cfg
)

# add_service_files(
#   FILES
#   SavePointcloud.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${PCL_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(interactive_object_controller src/interactive_object_controller.cpp src/interactive_object_controller_node.cpp)
target_link_libraries(interactive_object_controller ${catkin_LIBRARIES})
