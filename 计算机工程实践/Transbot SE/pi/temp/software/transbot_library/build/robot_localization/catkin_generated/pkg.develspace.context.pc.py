# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "/root/software/transbot_library/devel/include;/root/software/transbot_library/src/robot_localization/include;/usr/include/eigen3".split(';') if "/root/software/transbot_library/devel/include;/root/software/transbot_library/src/robot_localization/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "cmake_modules;diagnostic_msgs;diagnostic_updater;eigen_conversions;geographic_msgs;geometry_msgs;message_filters;message_runtime;nav_msgs;roscpp;sensor_msgs;std_msgs;std_srvs;tf2;tf2_geometry_msgs;tf2_ros;xmlrpcpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lekf;-lfilter_base;-lfilter_utilities;-lnavsat_transform;-lros_filter;-lros_filter_utilities;-lrobot_localization_estimator;-lros_robot_localization_listener;-lukf;-lyaml-cpp".split(';') if "-lekf;-lfilter_base;-lfilter_utilities;-lnavsat_transform;-lros_filter;-lros_filter_utilities;-lrobot_localization_estimator;-lros_robot_localization_listener;-lukf;-lyaml-cpp" != "" else []
PROJECT_NAME = "robot_localization"
PROJECT_SPACE_DIR = "/root/software/transbot_library/devel"
PROJECT_VERSION = "2.4.3"
