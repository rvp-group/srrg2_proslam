add_library(srrg2_proslam_localization_library SHARED
  instances.cpp
  aligner_slice_projective.cpp
)

target_link_libraries(srrg2_proslam_localization_library
  srrg2_proslam_adaptation_library
  srrg2_solver_projective_library
  ${catkin_LIBRARIES}
)
