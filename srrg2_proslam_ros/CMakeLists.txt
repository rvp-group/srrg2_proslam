cmake_minimum_required(VERSION 2.8.3)
project(srrg2_proslam_ros)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_solver
  srrg2_core_ros
  srrg2_proslam
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

find_package(Eigen3 REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
    srrg2_solver
    srrg2_core_ros
    srrg2_proslam
  DEPENDS
    QGLViewer
    GLUT
)

include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${QGLViewer_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)

