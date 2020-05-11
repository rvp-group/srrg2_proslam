add_library(srrg2_proslam_mapping_library SHARED
  scene_clipper_projective_3d.cpp
  triangulator_rigid_stereo.cpp
  mergers/merger_correspondence_projective_depth_3d.cpp
  instances.cpp
)

target_link_libraries(srrg2_proslam_mapping_library
  srrg2_proslam_sensor_processing_library
  srrg2_image_library
  ${catkin_LIBRARIES}
)
