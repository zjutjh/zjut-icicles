
FILE(GLOB_RECURSE sourcefiles "*.cpp" "*.h")

add_library(kcftracker ${sourcefiles})
target_link_libraries(kcftracker ${catkin_LIBRARIES})

