#include "instances.h"

#include <srrg_pcl/instances.h>

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  void srrg2_proslam_sensor_processing_registerTypes() {
    srrg2_core::point_cloud_registerTypes();

    BOSS_REGISTER_CLASS(IntensityFeatureExtractorBinned2D);
    BOSS_REGISTER_CLASS(IntensityFeatureExtractorBinned3D);
    BOSS_REGISTER_CLASS(IntensityFeatureExtractorSelective2D);
    BOSS_REGISTER_CLASS(IntensityFeatureExtractorSelective3D);

    BOSS_REGISTER_CLASS(RawDataPreprocessorStereoProjective);
    BOSS_REGISTER_CLASS(RawDataPreprocessorMonocularDepth);
  }

} // namespace srrg2_proslam
