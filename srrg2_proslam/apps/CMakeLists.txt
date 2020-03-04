add_executable(compute_sparse_stereo_gt compute_sparse_stereo_gt.cpp)
target_link_libraries(compute_sparse_stereo_gt
  srrg2_proslam_adaptation_library
)

add_executable(example_feature_extractor example_feature_extractor.cpp)
target_link_libraries(example_feature_extractor
  srrg2_proslam_adaptation_library
)

add_executable(example_feature_matcher example_feature_matcher.cpp)
target_link_libraries(example_feature_matcher
  srrg2_proslam_adaptation_library
)

add_executable(example_projective_search_patterns example_projective_search_patterns.cpp)
target_link_libraries(example_projective_search_patterns
  srrg2_proslam_adaptation_library
)

add_executable(example_unproject_rgbd example_unproject_rgbd.cpp)
target_link_libraries(example_unproject_rgbd
  srrg2_proslam_adaptation_library
  pthread
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
)

add_executable(example_triangulate_rigid_stereo example_triangulate_rigid_stereo.cpp)
target_link_libraries(example_triangulate_rigid_stereo
  srrg2_proslam_tracking_library
  pthread
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
)

add_executable(example_stereo_projective_tracking example_stereo_projective_tracking.cpp)
target_link_libraries(example_stereo_projective_tracking
  srrg2_proslam_tracking_library
)

add_executable(example_slam_3d example_slam_3d.cpp)
target_link_libraries(example_slam_3d
  srrg2_proslam_tracking_library
  pthread
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
)

add_executable(convert_stereo_to_srrg2 convert_stereo_to_srrg2.cpp)
target_link_libraries(convert_stereo_to_srrg2
  ${catkin_LIBRARIES}
)

add_executable(convert_rgbd_to_srrg2 convert_rgbd_to_srrg2.cpp)
target_link_libraries(convert_rgbd_to_srrg2
  ${catkin_LIBRARIES}
)

add_executable(app_benchmark app_benchmark.cpp)
target_link_libraries(app_benchmark
  srrg2_proslam_tracking_library
  ${catkin_LIBRARIES}
)