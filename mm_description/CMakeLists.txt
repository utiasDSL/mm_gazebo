cmake_minimum_required(VERSION 2.8.3)
project(mm_description)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  gazebo_msgs
)

catkin_package(
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_base src/odom_to_base.cpp)

target_link_libraries(odom_to_base
   ${catkin_LIBRARIES}
)
