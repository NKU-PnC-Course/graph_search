cmake_minimum_required(VERSION 3.0.2)
project(graph_search)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_search
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/graph_search_ros_node.cpp
  src/graph_search_ros.cpp
  src/graph_search.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(start_goal_publisher src/start_goal_publisher.cpp)
target_link_libraries(start_goal_publisher ${catkin_LIBRARIES})