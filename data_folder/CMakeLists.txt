cmake_minimum_required(VERSION 3.0.2)
project(data_folder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${PCL_DIR}
  ${catkin_INCLUDE_DIRS}
)

# add_executable(data_folder src/pc_visualizer.cpp src/pc_visualizer_node.cpp)
# target_link_libraries(pc_visualizer ${catkin_LIBRARIES})
