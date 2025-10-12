cmake_minimum_required(VERSION 3.0.2)
project(frontier_ranker_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  nav_msgs
  tf2_ros
  costmap_2d
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/frontier_ranker_node.py
  scripts/visited_grid_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
