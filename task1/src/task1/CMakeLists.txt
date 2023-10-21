cmake_minimum_required(VERSION 3.0.2)
project(task1)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}  # 添加catkin包含目录     解决fatal error: ros/ros.h: No such file or directory  # include <ros/ros.h>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${OpenCV_DEFINTIONS})



set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "/home/agilex/Desktop/Software/task1/src/task1")

add_executable(task1_1 src/exp1.cpp)
add_executable(task1_2 src/exp2.cpp)
add_executable(task1_3 src/exp3.cpp)
add_executable(task1_4 src/exp4.cpp)

target_link_libraries(task1_1
  ${catkin_LIBRARIES}
)

target_link_libraries(task1_2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(task1_3
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(task1_4
  ${catkin_LIBRARIES}
)
