cmake_minimum_required(VERSION 2.8.3)
project(srrg2_proslam)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_solver
  srrg2_qgl_viewport
  srrg2_slam_interfaces
  srrg_hbst
)

#ds fight sloppyness: extend cmake compile flags to be more restrictive
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")

#ds disable qt version tagging (seems to cause trouble on Ubuntu 18.04)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DQT_NO_VERSION_TAGGING")

#ds enable HBST code for descriptor merging and opencv support TODO move to cmake_modules?
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")

#ds retrieve default cmake build flags
include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)

#define catkin package
catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_proslam_sensor_processing_library
    srrg2_proslam_registration_library
    srrg2_proslam_mapping_library
    srrg2_proslam_tracking_library
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
    srrg2_solver
    srrg2_qgl_viewport
    srrg2_slam_interfaces
  DEPENDS
    QGLViewer
    GLUT
    OpenCV
)

#ds includes provided by this package (restrictive compile flags will be applied!)
include_directories(src)

#ds specify additional locations of header files
#ds treating them as system includes to suppress a massive wave of warnings originating from those header files (!)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${QGLViewer_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#ds harmless logging only
message("${PROJECT_NAME}|using Eigen version: '3' (${EIGEN3_INCLUDE_DIR})")
message("${PROJECT_NAME}|using ROS version: '$ENV{ROS_DISTRO}' ($ENV{ROS_ROOT})")
message("${PROJECT_NAME}|using OpenCV version: '${OpenCV_VERSION}' (${OpenCV_DIR})")

link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)
add_subdirectory(benchmarks)
