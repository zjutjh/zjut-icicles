cmake_minimum_required(VERSION 2.8.3)
project(opencv_apps)

## https://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
if(CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "C++11 activated.")
    add_definitions("-std=gnu++11")
  elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
    message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
    add_definitions("-std=gnu++0x")
  else ()
    message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
  endif()
else(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure message_generation image_transport nodelet roscpp sensor_msgs std_msgs std_srvs)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV Components: ${OpenCV_LIB_COMPONENTS}")
if(OpenCV_VERSION VERSION_LESS "3.0" OR TARGET opencv_optflow)
  set(OPENCV_HAVE_OPTFLOW TRUE)
endif()
# Supporting CompressedImage in cv_bridge has been started from 1.11.9 (2015-11-29)
# note : hydro 1.10.18, indigo : 1.11.13 ...
# https://github.com/ros-perception/vision_opencv/pull/70
if(cv_bridge_VERSION VERSION_LESS "1.11.9")
  add_definitions("-DCV_BRIDGE_COMPRESSED_IMAGE_IS_NOT_SUPPORTED")
endif()
# Supporting CvtColorForDisplay in cv_bridge has been started from 1.11.9 (2015-11-29)
if(cv_bridge_VERSION VERSION_LESS "1.11.9")
  add_definitions("-DCV_BRIDGE_CVT_COLOR_FOR_DISPLAY_IS_NOT_SUPPORTED")
endif()
# Supporting CvtColorForDisplayOptions in cv_bridge has been started from 1.11.13 (2016-07-11)
if(cv_bridge_VERSION VERSION_LESS "1.11.13")
  add_definitions("-DCV_BRIDGE_CVT_COLOR_FOR_DISPLAY_OPTION_IS_NOT_SUPPORTED")
endif()

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/DiscreteFourierTransform.cfg
  cfg/AddingImages.cfg
  cfg/Smoothing.cfg
  cfg/Pyramids.cfg
  cfg/EdgeDetection.cfg cfg/HoughLines.cfg cfg/HoughCircles.cfg
  cfg/FindContours.cfg cfg/ConvexHull.cfg cfg/GeneralContours.cfg cfg/ContourMoments.cfg
  cfg/FaceDetection.cfg
  cfg/FaceRecognition.cfg
  cfg/GoodfeatureTrack.cfg
  cfg/CornerHarris.cfg
  #
  cfg/CamShift.cfg
  cfg/FBackFlow.cfg
  cfg/LKFlow.cfg
  cfg/PeopleDetect.cfg
  cfg/PhaseCorr.cfg
  cfg/SegmentObjects.cfg
  cfg/SimpleFlow.cfg
  cfg/Threshold.cfg
  cfg/RGBColorFilter.cfg
  cfg/HLSColorFilter.cfg
  cfg/HSVColorFilter.cfg
  cfg/WatershedSegmentation.cfg
  )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Point2D.msg
   Point2DStamped.msg
   Point2DArray.msg
   Point2DArrayStamped.msg
   Rect.msg
   RectArray.msg
   RectArrayStamped.msg
   Flow.msg
   FlowStamped.msg
   FlowArray.msg
   FlowArrayStamped.msg
   Size.msg
   Face.msg
   FaceArray.msg
   FaceArrayStamped.msg
   Line.msg
   LineArray.msg
   LineArrayStamped.msg
   RotatedRect.msg
   RotatedRectStamped.msg
   RotatedRectArray.msg
   RotatedRectArrayStamped.msg
   Circle.msg
   CircleArray.msg
   CircleArrayStamped.msg
   Moment.msg
   MomentArray.msg
   MomentArrayStamped.msg
   Contour.msg
   ContourArray.msg
   ContourArrayStamped.msg
)

add_service_files(
  FILES
  FaceRecognitionTrain.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure message_runtime nodelet roscpp sensor_msgs std_msgs std_srvs
#                DEPENDS OpenCV
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Macro to add nodelets
macro(opencv_apps_add_nodelet node_name nodelet_cppfile)
  set(NODE_NAME ${node_name})
  set(NODELET_NAME opencv_apps/${node_name})
  configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
  add_executable(${node_name}_exe ${node_name}.cpp)
  SET_TARGET_PROPERTIES(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
  target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  list(APPEND _opencv_apps_nodelet_cppfiles ${nodelet_cppfile})
  list(APPEND _opencv_apps_nodelet_targets ${node_name}_exe)
endmacro()

# https://github.com/Itseez/opencv/blob/2.4/samples/cpp/

# calib3d
  # ./tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
  # ./tutorial_code/calib3d/real_time_pose_estimation/src/main_detection.cpp
  # ./tutorial_code/calib3d/real_time_pose_estimation/src/main_registration.cpp
  # ./tutorial_code/calib3d/stereoBM/SBM_Sample.cpp

# core
opencv_apps_add_nodelet(adding_images src/nodelet/adding_images_nodelet.cpp) # ./tutorial_code/core/AddingImages/AddingImages.cpp
opencv_apps_add_nodelet(discrete_fourier_transform src/nodelet/discrete_fourier_transform_nodelet.cpp)  # ./tutorial_code/core/discrete_fourier_transform/discrete_fourier_transform.cpp
  # ./tutorial_code/core/file_input_output/file_input_output.cpp
  # ./tutorial_code/core/how_to_scan_images/how_to_scan_images.cpp
  # ./tutorial_code/core/interoperability_with_OpenCV_1/interoperability_with_OpenCV_1.cpp
  # ./tutorial_code/core/ippasync/ippasync_sample.cpp
  # ./tutorial_code/core/mat_mask_operations/mat_mask_operations.cpp
  # ./tutorial_code/core/Matrix/Drawing_1.cpp
  # ./tutorial_code/core/Matrix/Drawing_2.cpp
  # ./tutorial_code/core/mat_the_basic_image_container/mat_the_basic_image_container.cpp

# features2D
  # ./tutorial_code/features2D/AKAZE_match.cpp
  # ./tutorial_code/features2D/AKAZE_tracking/planar_tracking.cpp
  # ./tutorial_code/gpu/gpu-basics-similarity/gpu-basics-similarity.cpp

# highGUi
  # ./tutorial_code/HighGUI/AddingImagesTrackbar.cpp
  # ./tutorial_code/HighGUI/BasicLinearTransformsTrackbar.cpp

# Histograms_Matching
  # ./tutorial_code/Histograms_Matching/calcBackProject_Demo1.cpp
  # ./tutorial_code/Histograms_Matching/calcBackProject_Demo2.cpp
  # ./tutorial_code/Histograms_Matching/calcHist_Demo.cpp
  #  ./tutorial_code/Histograms_Matching/compareHist_Demo.cpp
  # ./tutorial_code/Histograms_Matching/EqualizeHist_Demo.cpp
  # ./tutorial_code/Histograms_Matching/MatchTemplate_Demo.cpp

# imagecodecs
  # ./tutorial_code/imgcodecs/GDAL_IO/gdal-image.cpp

# ImgProc
  # ./tutorial_code/ImgProc/BasicLinearTransforms.cpp
  # ./tutorial_code/ImgProc/Morphology_1.cpp
  # ./tutorial_code/ImgProc/Morphology_2.cpp
  # ./tutorial_code/ImgProc/Morphology_3.cpp
opencv_apps_add_nodelet(pyramids src/nodelet/pyramids_nodelet.cpp)  # ./tutorial_code/ImgProc/Pyramids.cpp
opencv_apps_add_nodelet(smoothing src/nodelet/smoothing_nodelet.cpp)  # ./tutorial_code/ImgProc/Smoothing.cpp
opencv_apps_add_nodelet(threshold src/nodelet/threshold_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold.cpp
opencv_apps_add_nodelet(rgb_color_filter src/nodelet/color_filter_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold_inRange.cpp
opencv_apps_add_nodelet(hls_color_filter src/nodelet/color_filter_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold_inRange.cpp
opencv_apps_add_nodelet(hsv_color_filter src/nodelet/color_filter_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold_inRange.cpp

# ImgTrans
opencv_apps_add_nodelet(edge_detection src/nodelet/edge_detection_nodelet.cpp) # ./tutorial_code/ImgTrans/CannyDetector_Demo.cpp
                                                                                                             # ./tutorial_code/ImgTrans/Laplace_Demo.cpp
                                                                                                             # ./tutorial_code/ImgTrans/Sobel_Demo.cpp
opencv_apps_add_nodelet(hough_lines src/nodelet/hough_lines_nodelet.cpp) # ./tutorial_code/ImgTrans/HoughLines_Demo.cpp
opencv_apps_add_nodelet(hough_circles src/nodelet/hough_circles_nodelet.cpp) # ./tutorial_code/ImgTrans/HoughCircle_Demo.cpp
 # ./tutorial_code/ImgTrans/copyMakeBorder_demo.cpp
 # ./tutorial_code/ImgTrans/filter2D_demo.cpp
 # ./tutorial_code/ImgTrans/Geometric_Transforms_Demo.cpp
 # ./tutorial_code/ImgTrans/imageSegmentation.cpp
 # ./tutorial_code/ImgTrans/Remap_Demo.cpp

# introduction
 # ./tutorial_code/introduction/display_image/display_image.cpp
 # ./tutorial_code/introduction/windows_visual_studio_Opencv/introduction_windows_vs.cpp

# ml
 # ./tutorial_code/ml/introduction_to_pca/introduction_to_pca.cpp
 # ./tutorial_code/ml/introduction_to_svm/introduction_to_svm.cpp
 # ./tutorial_code/ml/non_linear_svms/non_linear_svms.cpp

# objectDetection
 # ./tutorial_code/objectDetection/objectDetection2.cpp
 # ./tutorial_code/objectDetection/objectDetection.cpp

# photo
 # ./tutorial_code/photo/decolorization/decolor.cpp
 # ./tutorial_code/photo/hdr_imaging/hdr_imaging.cpp
 # ./tutorial_code/photo/non_photorealistic_rendering/npr_demo.cpp
 # ./tutorial_code/photo/seamless_cloning/cloning_demo.cpp
 # ./tutorial_code/photo/seamless_cloning/cloning_gui.cpp

# ShapeDescriptors
opencv_apps_add_nodelet(find_contours src/nodelet/find_contours_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/findContours_demo.cpp
opencv_apps_add_nodelet(convex_hull src/nodelet/convex_hull_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/hull_demo.cpp
opencv_apps_add_nodelet(general_contours src/nodelet/general_contours_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/generalContours_demo2.cpp
opencv_apps_add_nodelet(contour_moments src/nodelet/contour_moments_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/moments_demo.cpp
  # ./tutorial_code/ShapeDescriptors/generalContours_demo1.cpp
  # ./tutorial_code/ShapeDescriptors/pointPolygonTest_demo.cpp

# TrackingMotion
opencv_apps_add_nodelet(goodfeature_track src/nodelet/goodfeature_track_nodelet.cpp) # ./tutorial_code/TrackingMotion/goodFeaturesToTrack_Demo.cpp
  # ./tutorial_code/TrackingMotion/cornerDetector_Demo.cpp
opencv_apps_add_nodelet(corner_harris src/nodelet/corner_harris_nodelet.cpp)  # ./tutorial_code/TrackingMotion/cornerHarris_Demo.cpp
  # ./tutorial_code/TrackingMotion/cornerSubPix_Demo.cpp

# videoio
 # ./tutorial_code/video/bg_sub.cpp
 # ./tutorial_code/videoio/video-input-psnr-ssim/video-input-psnr-ssim.cpp
 # ./tutorial_code/videoio/video-write/video-write.cpp

# viz
 # ./tutorial_code/viz/creating_widgets.cpp
 # ./tutorial_code/viz/launching_viz.cpp
 # ./tutorial_code/viz/transformations.cpp
 # ./tutorial_code/viz/widget_pose.cpp

# xfeature
 # ./tutorial_code/xfeatures2D/LATCH_match.cpp

# ./3calibration.cpp
# ./autofocus.cpp
# ./bgfg_segm.cpp
# ./calibration.cpp
opencv_apps_add_nodelet(camshift src/nodelet/camshift_nodelet.cpp) # ./camshiftdemo.cpp
# ./cloning_demo.cpp
# ./cloning_gui.cpp
# ./connected_components.cpp
# ./contours2.cpp
# ./convexhull.cpp
# ./cout_mat.cpp
# ./create_mask.cpp
# ./dbt_face_detection.cpp
# ./delaunay2.cpp
# ./demhist.cpp
# ./detect_blob.cpp
# ./detect_mser.cpp
# ./dft.cpp
# ./distrans.cpp
# ./drawing.cpp
# ./edge.cpp
# ./em.cpp
# ./example_cmake/example.cpp
opencv_apps_add_nodelet(face_detection src/nodelet/face_detection_nodelet.cpp) # ./facedetect.cpp
opencv_apps_add_nodelet(face_recognition src/nodelet/face_recognition_nodelet.cpp)
# ./facial_features.cpp
opencv_apps_add_nodelet(fback_flow src/nodelet/fback_flow_nodelet.cpp) # ./fback.cpp
# ./ffilldemo.cpp
# ./filestorage_base64.cpp
# ./filestorage.cpp
# ./fitellipse.cpp
# ./grabcut.cpp
# ./houghcircles.cpp
# ./houghlines.cpp
# ./image_alignment.cpp
# ./image.cpp
# ./imagelist_creator.cpp
# ./image_sequence.cpp
# ./inpaint.cpp
# ./intelperc_capture.cpp
# ./kalman.cpp
# ./kmeans.cpp
# ./laplace.cpp
# ./letter_recog.cpp
opencv_apps_add_nodelet(lk_flow src/nodelet/lk_flow_nodelet.cpp) # ./lkdemo.cpp
# ./logistic_regression.cpp
# ./lsd_lines.cpp
# ./mask_tmpl.cpp
# ./matchmethod_orb_akaze_brisk.cpp
# ./minarea.cpp
# ./morphology2.cpp
# ./neural_network.cpp
# ./npr_demo.cpp
# ./opencv_version.cpp
# ./openni_capture.cpp
# ./pca.cpp
opencv_apps_add_nodelet(people_detect src/nodelet/people_detect_nodelet.cpp) # ./peopledetect.cpp
opencv_apps_add_nodelet(phase_corr src/nodelet/phase_corr_nodelet.cpp) # ./phase_corr.cpp
# ./points_classifier.cpp
# ./polar_transforms.cpp
opencv_apps_add_nodelet(segment_objects src/nodelet/segment_objects_nodelet.cpp) # ./segment_objects.cpp
# ./select3dobj.cpp
# ./shape_example.cpp
# ./smiledetect.cpp
# ./squares.cpp
# ./starter_imagelist.cpp
# ./starter_video.cpp
# ./stereo_calib.cpp
# ./stereo_match.cpp
# ./stitching.cpp
# ./stitching_detailed.cpp
# ./train_HOG.cpp
# ./train_svmsgd.cpp
# ./tree_engine.cpp
# ./tvl1_optical_flow.cpp
# ./videostab.cpp
opencv_apps_add_nodelet(watershed_segmentation src/nodelet/watershed_segmentation_nodelet.cpp) # ./watershed.cpp

# ros examples
opencv_apps_add_nodelet(simple_example src/nodelet/simple_example_nodelet.cpp)
opencv_apps_add_nodelet(simple_compressed_example src/nodelet/simple_compressed_example_nodelet.cpp)

# https://github.com/Itseez/opencv/blob/2.4/samples/cpp/simpleflow_demo.cpp
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  opencv_apps_add_nodelet(simple_flow src/nodelet/simple_flow_nodelet.cpp)
endif()
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/bgfg_gmg.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/hybridtrackingsample.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/linemod.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/retinaDemo.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/video_dmtx.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/video_homography.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/videocapture_pvapi.cpp

add_library(${PROJECT_NAME} SHARED
  src/nodelet/nodelet.cpp
  ${_opencv_apps_nodelet_cppfiles}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${_opencv_apps_nodelet_targets}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch test scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

## test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  if(roslaunch_VERSION VERSION_LESS "1.11.1")
    message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
  else()
    file(GLOB LAUNCH_FILES launch/*.launch)
    foreach(LAUNCH_FILE ${LAUNCH_FILES})
      roslaunch_add_file_check(${LAUNCH_FILE})
    endforeach()
  endif()
  add_subdirectory(test)
endif()
