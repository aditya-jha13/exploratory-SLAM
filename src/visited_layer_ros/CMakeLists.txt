cmake_minimum_required(VERSION 3.0.2)
project(visited_layer_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Libraries and plugin export to be added with implementation.

