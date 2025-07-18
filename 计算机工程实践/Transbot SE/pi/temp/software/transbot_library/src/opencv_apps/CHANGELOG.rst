^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opencv_apps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2020-08-19)
------------------
* .travis.yml: add noetic test (`#108 <https://github.com/ros-perception/opencv_apps/issues/108>`_)

  * support oepncv4 for face detection launch/test files
  * convert img_gray every loop, this fixes
    [ERROR] [1597757072.115502734]: Image processing error: Matrix operand is an empty matrix. checkOperandsExist ../modules/core/src/matrix_expressions.cpp 23
    errors
  * check CV_VERSION_VERSION > 4
  * add g++ static to package.xml
  * use python3 for noetic
  * .travis.yml : add CHECK_PYTHON3_COMPILE
  * .travis.yml: add noetic test

* fix travis (`#106 <https://github.com/ros-perception/opencv_apps/issues/106>`_)

  * add more package to CATKIN_DEPENDS
  * add image_view to run_depends

* add hot fix for hydro test (`#98 <https://github.com/ros-perception/opencv_apps/issues/98>`_)

  * add doublequote arount $TEST
  * add hot fix for hydro test

* Improved variable names and comments (`#95 <https://github.com/ros-perception/opencv_apps/issues/95>`_)

  * Improved variable names and comments: The comments and variable names were the opposite of what was functionally happening in the code. No functional change to this commit, just better readability and maintainability.

* setup EoL repository (`#96 <https://github.com/ros-perception/opencv_apps/issues/96>`_)

* Contributors: Gus Crowards, Kei Okada

2.0.1 (2019-04-22)
------------------
* support catkin_lint and clang-format tests in travis.yml (`#93 <https://github.com/ros-perception/opencv_apps/issues/93>`_)

  * override is not supported gcc4.6 (12.04), remove this fix and add NOLINT
  * clang-tidy code need c++11
  * fix code by run-clang-tidy -fix
  * fix format by clang-format
  * fix CMakeLists.txt and package.xml for catkin_lint
  * support catkin_lint and clang-format tests

* add queue_size parameter to all nodes, see `#83 <https://github.com/ros-perception/opencv_apps/issues/83>`_ (`#92 <https://github.com/ros-perception/opencv_apps/issues/92>`_)

  * add queue_size arg to launch files
  * sometimes simple_example_test fails with 'average rate (36.121Hz) exceeded maximum (35.000Hz)'

* add queue_size parameter to all nodes, see `#83 <https://github.com/ros-perception/opencv_apps/issues/83>`_
* add melodic badge
* Contributors: Furushchev, Hironori Fujimoto, Kei Okada, higashide, iory, moju zhao
* add melodic badge
* Add lk flow params trackbar (`#78 <https://github.com/ros-perception/opencv_apps/issues/78>`_)
* Remove duplication of add_library for simple_flow (`#88 <https://github.com/ros-perception/opencv_apps/issues/88>`_)
   ${_opencv_apps_nodelet_cppfiles} adds simple_flow to library but also
   ${${PROJECT_NAME}_EXTRA_FILES} does same thing

* Do not pefrom face recognition process without the trained data (`#91 <https://github.com/ros-perception/opencv_apps/issues/91>`_)

  * Add warning logger to prompt face data tranining
  * Add a check to decide whether to perform the face recognition in callback function, according to the content of th

* fback_flow: add option to set 'queue_size' (`#83 <https://github.com/ros-perception/opencv_apps/issues/83>`_)
* travis.yml: add melodic and remove jade (`#84 <https://github.com/ros-perception/opencv_apps/issues/84>`_)
* [face_detection.launch] Fixed path of haarcascade xml for OpenCV-3.3.1 (`#79 <https://github.com/ros-perception/opencv_apps/issues/79>`_)
* Contributors: Yuki Furuta, Hironori Fujimoto, Kei Okada, Taichi Higashide, Iory Yanokura, Moju Zhao

2.0.0 (2017-11-20)
------------------
* Fix namespace and pkg name of nodelets (Closes (`#21 <https://github.com/ros-perception/opencv_apps/issues/21>`_)) (`#74 <https://github.com/ros-perception/opencv_apps/issues/74>`_)
  Fix namespace and pkg name of nodelets
* Add pyramids_nodelet (`#37 <https://github.com/ros-perception/opencv_apps/issues/37>`_)
  * use toCvCopy instead of CvShare in adding_images
* adding_images_nodelt :support different size of images (`#57 <https://github.com/ros-perception/opencv_apps/issues/57>`_)
* fix contour moment program (`#66 <https://github.com/ros-perception/opencv_apps/issues/66>`_)
  * contour_moments_nodelet.cpp: remove redundant codes, use input encoding
  * contour_moments_nodelet.cpp: sort contours by the area
  * contour_moments_nodelet.cpp: remove tailing NR from NODELET_INFO

* fix for opencv 3.3.1 (`#71 <https://github.com/ros-perception/opencv_apps/issues/71>`_)
  * fix launch/test fiels for opencv3.3
  * goodFeaturesTrack takes useHarriesDetector == false
  * opencv 3.3.1 has newer FaceRecognizer

* Contributors: Kei Okada, Iori Yanokura

1.12.0 (2017-07-14)
-------------------
* [src/node/standalone_nodelet_exec.cpp.in] workaround for freezing imshow on kinetic (`#67 <https://github.com/ros-perception/opencv_apps/issues/67>`_)
  * use ros::param::set instead of ros::NodeHandle("~"), that did not output NODELET_INFO
  * workaround for freezing imshow on kinetic

* [launch/hough_circles.launch] Corrected a typo and applied the node_name argument (`#69 <https://github.com/ros-perception/opencv_apps/issues/69>`_ )
* [face_recognition] add nodelet / script / message files for face recognition (new) `#63 <https://github.com/ros-perception/opencv_apps/issues/63>`_ from furushchev/face-recognition-new

  * add face_recognition nodelet / test
    cfg/FaceRecognition.cfg
    launch/face_recognition.launch
    scripts/face_recognition_trainer.py
    src/nodelet/face_recognition_nodelet.cpp

  * [Face.msg] add label / confidence for face recognition
  * [CMakeLists.txt] remove duplicate msg: RectArrayStamped.msg

* cfg/*.cfg : Set useless use_camera_info flag to false in default (`#58 <https://github.com/ros-perception/opencv_apps/issues/58>`_ )
* Contributors: Kei Okada, Kentaro Wada, Yuki Furuta, wangl5

1.11.15 (2017-03-26)
--------------------

* New Nodes

  * [color_filter_nodelet.cpp] Add color_filter nodelet (`#48 <https://github.com/ros-perception/opencv_apps/issues/48>`_)
    * use BGR2HSB, support H from 0-360 and 350 - 360+a
    * Unified hsl -> hls
    * Add hsv_color_filter test
    * Modified hls_color_filter's test paramter.  Extracting skin color.
  * [corner_harris_nodelet.cpp] Add corner-harris (`#38 <https://github.com/ros-perception/opencv_apps/issues/38>`_ )
  * [discrete_fourier_transform_nodelet.cpp] Add discrete_fourier_transform_nodelet (`#36 <https://github.com/ros-perception/opencv_apps/issues/36>`_ )

* New Feature

 * [face_detection_nodelet.cpp] publish face roi image (`#40 <https://github.com/ros-perception/opencv_apps/issues/40>`_ )
    * [face_detection_nodelet.cpp] fix: use encoding BGR8 on conversion from cv::Mat to sensor_msgs/Image

* Fix / Improvement

 * [adding_images_nodelet.cpp] Fix AddingImages (`#52 <https://github.com/ros-perception/opencv_apps/issues/52>`_)
    * CvtColorForDisplay is not supported until 1.11.9 (hydro)
    * CvtColorForDisplayOptions is supported in 1.11.13
    * Rename topic ~info to camera_info for consistency
    * Do dynamic scaling for float/double images
    * Support adding images whose encodings are same kind, For example adding rgb8 + bgr8
    * display using cvtColorForDisplay
    * Clarify with auto_beta for auto beta settings
    * Check input encoding consistency
    * Add arbitrary dtype images
    * AddingImages: enable to set beta param if use_data is true
  * [face_detection] add test for face_detection/face_image topic  (`#49 <https://github.com/ros-perception/opencv_apps/issues/49>`_)
    * test/CMakeLists.txt : skip face_detection.test
    * [test/test-face_detection.test] add test for face_image
  * [.travis.sh] bugfix: test for opencv3 `#45 <https://github.com/ros-perception/opencv_apps/issues/45>`_
    * [.travis.sh] bugfix: use --upstream for rosinstall_generator to fetch not only metapackage
    - [.travis.sh] run test only opencv_apps package (not dep packages)
    - [.travis.sh] build compressed_image_transport from source if opencv3 is enabled
    - [package.xml] use compressed_image_transport for test_depend instead of meta package image_transport_plugins
  * [doc] Better package description. (`#43 <https://github.com/ros-perception/opencv_apps/issues/43>`_)
  * watershed_segmentation_nodelet.cpp : Fix typo in warnnige message  (`#34 <https://github.com/ros-perception/opencv_apps/issues/34>`_)
  * Create README.md

* Contributors: Isaac I.Y. Saito, Kei Okada, Kentaro Wada, Yuki Furuta, Iori Yanokura

1.11.14 (2016-09-12)
--------------------

* Force convert to bgr for display (`#30 <https://github.com/ros-perception/opencv_apps/issues/30>`_)

  * add include sensor_msgs/image_encodings for old image_encodings
  * force conver to bgr8 using sensor_msgs::image_encodings::BGR8

* Add more nodes from opencv sample codes

  * [smoothing] Add smoothing filter sample code, test, launch files (`#32 <https://github.com/ros-perception/opencv_apps/issues/32>`_)
  * [threshold] add threshold sample code (`#27 <https://github.com/ros-perception/opencv_apps/issues/27>`_)
  * [adding_image] add adding_image sample code (`#29 <https://github.com/ros-perception/opencv_apps/issues/29>`_)

* Add launch files for opencv_apps nodes

  * separate launch and test files (`#20 <https://github.com/ros-perception/opencv_apps/issues/20>`_)

* Add hydro travis testing (`#22 <https://github.com/ros-perception/opencv_apps/issues/22>`_)

  * test/CMakeLists.txt : catkin_download_test_data not working with DESTINATION . for hydro
  * cv_bridge before 1.11.9 does not suport CompressedImage in cv_bridge
  * lk_flow : need to explicitly include sensor_msgs/image_endcodings.h
  * simple_compressed_example_nodelet.cpp : need to include sensor_msgs/CompressedImage explicitly on hydro
  * .travis.yml : add hydro testing

* Minor Fixes

  * update gitignore to avoid test png data (`#28 <https://github.com/ros-perception/opencv_apps/issues/28>`_)
  * fix  hough_circles for input frame color (`#13 <https://github.com/ros-perception/opencv_apps/issues/13>`_ )
  * CMakeLists.txt update list of opencv tutorial codes (`#25 <https://github.com/ros-perception/opencv_apps/issues/25>`_)
  * fix face_detection.launch to accept args for cascade xml for opencv3 (`#20 <https://github.com/ros-perception/opencv_apps/issues/20>`_)
  * CMakeLists.txt : add install rule for launch (`#20 <https://github.com/ros-perception/opencv_apps/issues/20>`_)
  * add launch/*.launch files (from test/*.test) to reuse launch files (`#20 <https://github.com/ros-perception/opencv_apps/issues/20>`_)
  * CMakeLists.txt: on roslaunch 1.11.1, roslaunch_add_file check fails with unsupported doc attributes (`#20 <https://github.com/ros-perception/opencv_apps/issues/20>`_)
  * 

* Add test for simple_example / simple_compressed_example (`#24 <https://github.com/ros-perception/opencv_apps/issues/24>`_)

  * add retry for simple_example/simple_compressed_example test, not sure why it fails.. on travis
  * package.xml : add image_transport_plugins to test_depend for republish node in test-simple_compressed_example.test
  * add test for simple_example/simple_compressed_example
  * simple_example_nodlet.cpp / simple_compressed_example_nodelet.cpp : support debug_view param
  * .travis.sh : add catkin_test_results --verbose

* Support kinetic on travis (`#15 <https://github.com/ros-perception/opencv_apps/issues/15>`_)

  * test/test-face-detection.test : add haarcascade data from opencv3 package directory
  * use docekr to run trusty/xenial .travis.sh

* Modified enabling use_camera_info by rosparam (`#18 <https://github.com/ros-perception/opencv_apps/issues/18>`_)
  
  * Enabling dynamic_reconfigure in private nodelet handler

* Enable to set min_distance_between_circles param, publish debug message (`#14 <https://github.com/ros-perception/opencv_apps/issues/14>`_)

  * hough_circles : fix to set dp_int to dp
  * hough_circles : enable to set min_distance_between_circles
  * hough_circles : add debug_image_publisher
  * hough_circles : fix bugs on createTrackver uses gaussian_blur_size for sigma x/y

* Contributors: Kei Okada, Iori Yanokura

1.11.13 (2016-06-01)
--------------------
* Add parameter to people_detector `#9 <https://github.com/ros-perception/opencv_apps/issues/9>`_
* hough_circles: enable to set double value to the HoughCircle params `#8 <https://github.com/ros-perception/opencv_apps/issues/8>`_

  * hough_circle enable to set gaussian_blue_size and kernel sigma from cfg
  * hough_circles: fix default/min/max value of cfg
  * hough_circle: enable to set db to 100
  * circle_hough: dp, accumrate_threshold, canny_threshold is double, not int

* Add parameter to hough_circles_nodelet `#7 <https://github.com/ros-perception/opencv_apps/issues/7>`_
* Add parameter to hough_lines_nodelet `#6 <https://github.com/ros-perception/opencv_apps/issues/6>`_
* Add parameter to edge_detection_nodelet(canny) `#5 <https://github.com/ros-perception/opencv_apps/issues/5>`_
* Simplify source tree by removing duplicated node codes `#4 <https://github.com/ros-perception/opencv_apps/issues/4>`_  Closes `#3 <https://github.com/ros-perception/opencv_apps/issues/3>`_
* fix .travis file
* copy Travis and .gitignore from vision_opencv
* geometry_msgs doesn't get used by opencv_apps, but std_msgs does. (`#119 <https://github.com/ros-perception/vision_opencv/pull/119>`_)
* Contributors: Kei Okada, Kentaro Wada, Lucas Walter, Vincent Rabaud, IorI Yanokura

1.11.12 (2016-03-10)
--------------------
* relax test condition
* fix test hz to 5 hz, tested on core i7 3.2G
* Refactor opencv_apps to remove duplicated codes to handle connection of
  topics.
  1. Add opencv_apps::Nodelet class to handle connection and disconnection of
  topics.
  2. Update nodelets of opencv_apps to inhereit opencv_apps::Nodelet class
  to remove duplicated codes.
* Contributors: Kei Okada, Ryohei Ueda

1.11.11 (2016-01-31)
--------------------
* check if opencv_contrib is available
* Use respawn instead of watch
* Contributors: Kei Okada, trainman419

1.11.10 (2016-01-16)
--------------------
* enable simple_flow on opencv3, https://github.com/ros-perception/vision_opencv/commit/8ed5ff5c48b4c3d270cd8216175cf6a8441cb046 can revert https://github.com/ros-perception/vision_opencv/commit/89a933aef7c11acdb75a17c46bfcb60389b25baa
* lk_flow_nodeletcpp, fback_flow_nodelet.cpp: need to copy input image to gray
* opencv_apps: add test programs, this will generate images for wiki
* fix OpenCV3 build
* phase_corr: fix display, bigger circle and line
* goodfeature_track_nodelet.cpp: publish good feature points as corners
* set image encoding to bgr8
* convex_hull: draw hull with width 4
* watershed_segmentatoin_nodelet.cpp: output segmented area as contours and suppot add_seed_points as input of segmentatoin seed
* src/nodelet/segment_objects_nodelet.cpp: change output image topic name from segmented to image
* Convert rgb image to bgr in lk_flow
* [oepncv_apps] fix bug for segment_objects_nodelet.cpp
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Vincent Rabaud

1.11.9 (2015-11-29)
-------------------
* Accept grayscale images as input as well
* Add format enum for easy use and choose format.
* Contributors: Felix Mauch, talregev

1.11.8 (2015-07-15)
-------------------
* simplify the OpenCV3 compatibility
* fix image output of fback_flow
* fix error: ISO C++ forbids initialization of member for gcc 4.6
* add std_srvs
* add std_srvs
* fix error: ISO C++ forbids initialization of member for gcc 4.6
* get opencv_apps to compile with OpenCV3
* fix licenses for Kei
* add opencv_apps, proposed in `#40 <https://github.com/ros-perception/vision_opencv/issues/40>`_
* Contributors: Kei Okada, Vincent Rabaud, Yuto Inagaki
