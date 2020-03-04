#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_messages/instances.h>
#include <srrg_slam_interfaces/measurement_adaptor.h>

#include "intensity_feature_extractor_binned.h"
#include "intensity_feature_extractor_selective.h"

namespace srrg2_slam_interfaces {

  //! descriptor-based measurement adaptor interface that incorporates previous observations
  template <typename DestinationCloudType_, typename FeatureCloudType_>
  class MeasurementAdaptorDescriptorBased_ : public MeasurementAdaptor_<DestinationCloudType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FeatureCloudType         = FeatureCloudType_;
    using FeatureExtractorBaseType = IntensityFeatureExtractor_<FeatureCloudType>;

    PARAM(srrg2_core::PropertyConfigurable_<FeatureExtractorBaseType>,
          feature_extractor,
          "feature extractor used to detect keypoints and compute descriptors",
          std::shared_ptr<FeatureExtractorBaseType>(
            new IntensityFeatureExtractorBinned_<FeatureCloudType>()),
          nullptr);

    void setProjections(const FeatureCloudType* projections_, const size_t& projection_radius_) {
      _projections       = projections_;
      _projection_radius = projection_radius_;
    }

  protected:
    const FeatureCloudType* _projections = nullptr;
    size_t _projection_radius            = 0;
  };

} // namespace srrg2_slam_interfaces
