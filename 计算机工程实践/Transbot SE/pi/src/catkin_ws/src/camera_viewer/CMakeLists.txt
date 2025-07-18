cmake_minimum_required(VERSION 3.0.2)
project(camera_viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_viewer_node src/camera_viewer_node.cpp)
target_link_libraries(camera_viewer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
