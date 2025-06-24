cmake_minimum_required(VERSION 3.0.2)
project(snake_explorer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  actionlib
  move_base_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(snake_explorer src/snake_explorer.cpp)
target_link_libraries(snake_explorer ${catkin_LIBRARIES})
