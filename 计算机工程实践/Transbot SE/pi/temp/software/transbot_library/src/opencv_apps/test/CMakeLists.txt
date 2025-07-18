# Tests simple calibration dataset
catkin_download_test_data(face_detector_withface_test_diamondback.bag http://download.ros.org/data/face_detector/face_detector_withface_test_diamondback.bag
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 59126117e049e69d577b7ee27251a6f8
  )
catkin_download_test_data(vslam_tutorial.bag http://download.ros.org/data/vslam_system/vslam_tutorial.bag
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  MD5 f5aece448b7af00a38a993eb71400806
  )
add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag
  DEPENDS vslam_tutorial.bag
  COMMAND rosbag reindex vslam_tutorial.bag --output-dir ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(vslam_tutorial_bag DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag)
add_dependencies(tests vslam_tutorial_bag)

# test data for face recognition
add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/face_data
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/face_data.tar.gz
  COMMAND ${CMAKE_COMMAND} -E tar zxf face_data.tar.gz
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Extracting face_data.tar.gz")
add_custom_target(face_data DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/face_data)
add_dependencies(tests face_data)

#add_rostest(test-apps.test)
add_rostest(test-adding_images.test ARGS gui:=false)
add_rostest(test-discrete_fourier_transform.test ARGS gui:=false)
add_rostest(test-smoothing.test ARGS gui:=false)
add_rostest(test-pyramids.test ARGS gui:=false)
add_rostest(test-threshold.test ARGS gui:=false)
add_rostest(test-rgb_color_filter.test ARGS gui:=false)
add_rostest(test-hls_color_filter.test ARGS gui:=false)
add_rostest(test-hsv_color_filter.test ARGS gui:=false)
add_rostest(test-edge_detection.test ARGS gui:=false)

add_rostest(test-hough_lines.test ARGS gui:=false)
add_rostest(test-hough_circles.test ARGS gui:=false)
add_rostest(test-find_contours.test ARGS gui:=false)
add_rostest(test-convex_hull.test ARGS gui:=false)
add_rostest(test-general_contours.test ARGS gui:=false)
add_rostest(test-contour_moments.test ARGS gui:=false)
if(cv_bridge_VERSION VERSION_LESS "1.11.9") # hydro skip face_detection.test
elseif("$ENV{ROS_DISTRO}" STREQUAL "noetic") # noetic uses opencv4 (need to use_opencv3 for some reason...)
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv4:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv4:=true)
elseif("$ENV{ROS_DISTRO}" STREQUAL "melodic") # melodic uses opencv3
  add_rostest(test-face_detection.test ARGS gui:=false)
  add_rostest(test-face_recognition.test ARGS gui:=false)
elseif(OpenCV_VERSION VERSION_LESS "3.0")
  add_rostest(test-face_detection.test ARGS gui:=false)
  add_rostest(test-face_recognition.test ARGS gui:=false)
elseif(OpenCV_VERSION VERSION_LESS "3.2")
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_1:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_1:=true)
elseif(OpenCV_VERSION VERSION_LESS "3.3")
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_2:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_2:=true)
else()
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_3:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_3:=true)
endif()
add_rostest(test-goodfeature_track.test ARGS gui:=false)
add_rostest(test-corner_harris.test ARGS gui:=false)
add_rostest(test-camshift.test ARGS gui:=false)
add_rostest(test-fback_flow.test ARGS gui:=false)
add_rostest(test-lk_flow.test ARGS gui:=false)
add_rostest(test-people_detect.test ARGS gui:=false)
add_rostest(test-phase_corr.test ARGS gui:=false)
add_rostest(test-segment_objects.test ARGS gui:=false)
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  add_rostest(test-simple_flow.test ARGS gui:=false)
endif()
add_rostest(test-watershed_segmentation.test ARGS gui:=false)

add_rostest(test-simple_example.test ARGS gui:=false)
add_rostest(test-simple_compressed_example.test ARGS gui:=false)

if(roslaunch_VERSION VERSION_LESS "1.11.1")
  message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
else()
  file(GLOB LAUNCH_FILES *.test)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()


