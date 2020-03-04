add_library(srrg2_proslam_tracking_library SHARED
  instances.cpp
  tracker_slice_projective_depth.cpp
)

target_link_libraries(srrg2_proslam_tracking_library
  srrg2_proslam_localization_library
  srrg2_proslam_mapping_library
)
