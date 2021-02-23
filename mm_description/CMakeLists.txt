cmake_minimum_required(VERSION 2.8.3)
project(mm_description)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  gazebo_msgs
)

catkin_package()
