#pragma once
#include "feature_extractors/intensity_feature_extractor_binned.h"
#include "feature_extractors/intensity_feature_extractor_selective.h"

#include "raw_data_preprocessor_monocular_depth.h"
#include "raw_data_preprocessor_stereo_projective.h"

namespace srrg2_proslam {

  void srrg2_proslam_sensor_processing_registerTypes() __attribute__((constructor));

} // namespace srrg2_proslam
