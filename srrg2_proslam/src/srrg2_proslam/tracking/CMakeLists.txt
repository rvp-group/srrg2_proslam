add_library(srrg2_proslam_tracking_library SHARED
  instances.cpp
)

target_link_libraries(srrg2_proslam_tracking_library
  srrg2_proslam_registration_library
  srrg2_proslam_mapping_library
)
