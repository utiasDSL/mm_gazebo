cmake_minimum_required(VERSION 2.8.3)
project(mm_gazebo)

find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  roscpp
  tf
  tf_conversions
)

catkin_package()

install(DIRECTORY launch Media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
# add_executable(ground_truth_publisher src/ground_truth_publisher.cc)
#  target_link_libraries(ground_truth_publisher
#    ${catkin_LIBRARIES}
#  )


