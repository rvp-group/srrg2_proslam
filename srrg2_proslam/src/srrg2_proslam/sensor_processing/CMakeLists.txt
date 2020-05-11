add_library(srrg2_proslam_sensor_processing_library SHARED
  feature_extractors/intensity_feature_extractor_base.cpp
  feature_extractors/intensity_feature_extractor_binned.cpp
  feature_extractors/intensity_feature_extractor_selective.cpp
  raw_data_preprocessor_stereo_projective.cpp
  raw_data_preprocessor_monocular_depth.cpp
  instances.cpp
)

target_link_libraries(srrg2_proslam_sensor_processing_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
