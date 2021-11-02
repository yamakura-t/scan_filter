cmake_minimum_required(VERSION 3.0.2)
project(scan_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan_filter src/scan_filter.cpp)
target_link_libraries(scan_filter ${catkin_LIBRARIES})
add_dependencies(scan_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

