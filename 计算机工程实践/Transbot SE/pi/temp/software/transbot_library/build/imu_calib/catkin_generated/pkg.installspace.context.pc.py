# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "cmake_modules;roscpp;sensor_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-laccel_calib".split(';') if "-laccel_calib" != "" else []
PROJECT_NAME = "imu_calib"
PROJECT_SPACE_DIR = "/root/software/transbot_library/install"
PROJECT_VERSION = "0.0.0"
