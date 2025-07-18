cmake_minimum_required(VERSION 2.8.3)
project(imu_calib)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        sensor_msgs
        transbot_msgs
        )

find_package(Eigen REQUIRED)
pkg_check_modules(YAML yaml-cpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES accel_calib
        CATKIN_DEPENDS cmake_modules roscpp sensor_msgs
        DEPENDS Eigen yaml-cpp
)

###########
## Build ##
###########

include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${YAML_INCLUDEDIR}
)

# accel_calib library
add_library(accel_calib
        src/accel_calib/accel_calib.cpp
        )
target_link_libraries(accel_calib
        ${YAML_LIBRARIES}
        )

# do_calib node
add_executable(do_calib src/do_calib_node.cpp src/do_calib.cpp)
add_dependencies(do_calib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(do_calib
        accel_calib
        ${catkin_LIBRARIES}
        )

# apply_calib node
add_executable(apply_calib src/apply_calib_node.cpp src/apply_calib.cpp)
add_dependencies(apply_calib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(apply_calib
        accel_calib
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# executables and libraries
install(TARGETS accel_calib do_calib apply_calib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
