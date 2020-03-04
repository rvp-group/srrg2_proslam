add_library(srrg2_proslam_mapping_library SHARED
  scene_clipper_projective_3d.cpp
  triangulator_rigid_stereo.cpp
  mergers/merger_correspondence_projective_depth_3d.cpp
  mergers/merger_projective_impl.cpp
  mergers/merger_projective_depth_ekf_impl.cpp
  mergers/merger_projective_rigid_stereo_ekf_impl.cpp
  mergers/merger_projective_rigid_stereo_impl.cpp
  mergers/merger_projective_rigid_stereo_triangulation_impl.cpp
  landmarks/filters/projective_point_ekf.cpp
  landmarks/filters/projective_depth_point_ekf.cpp
  landmarks/filters/stereo_projective_point_ekf.cpp
  landmarks/landmark_estimator_ekf_impl.cpp
  landmarks/landmark_estimator_pose_based_smoother_impl.cpp
  landmarks/landmark_estimator_weighted_mean_impl.cpp
  instances.cpp
)

target_link_libraries(srrg2_proslam_mapping_library
  srrg2_proslam_adaptation_library
  ${catkin_LIBRARIES}
)
