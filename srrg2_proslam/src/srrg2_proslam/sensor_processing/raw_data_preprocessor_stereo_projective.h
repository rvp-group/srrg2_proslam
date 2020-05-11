#pragma once
#include "raw_data_preprocessor_descriptor_based.hpp"
#include "srrg2_proslam/registration/correspondence_finders/correspondence_finder_descriptor_based_epipolar.h"

namespace srrg2_proslam {

  class RawDataPreprocessorStereoProjective : public RawDataPreprocessorDescriptorBased_<
                                                srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                                srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      RawDataPreprocessorDescriptorBased_<srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                          srrg2_core::PointIntensityDescriptor3fVectorCloud>;
    using FeatureExtractorBaseType = IntensityFeatureExtractor_<FeatureCloudType>;

    PARAM(srrg2_core::PropertyConfigurable_<FeatureExtractorBaseType>,
          feature_extractor_right,
          "feature extractor used to detect keypoints and compute descriptors in the right frame",
          std::shared_ptr<FeatureExtractorBaseType>(new IntensityFeatureExtractorBinned3D()),
          nullptr);

    PARAM(srrg2_core::PropertyConfigurable_<CorrespondenceFinderDescriptorBasedBruteforce3D3D>,
          correspondence_finder,
          "descriptor-based correspondence finder used to compute stereo matches",
          CorrespondenceFinderDescriptorBasedEpipolar3D3DPtr(
            new CorrespondenceFinderDescriptorBasedEpipolar3D3D()),
          nullptr);

    PARAM(srrg2_core::PropertyString,
          topic_camera_left,
          "left rgb image topic [/camera_left/image_raw]",
          "/camera_left/image_raw",
          0);
    PARAM(srrg2_core::PropertyString,
          topic_camera_right,
          "right rgb image topic [/camera_right/image_raw]",
          "/camera_right/image_raw",
          0);

    bool setRawData(srrg2_core::BaseSensorMessagePtr msg_) override;
    void compute() override;

  protected:
    // ds buffered left and right image data messages TODO purge?
    srrg2_core::ImageMessagePtr _image_message_left  = nullptr;
    srrg2_core::ImageMessagePtr _image_message_right = nullptr;
  };

  using RawDataPreprocessorStereoProjectivePtr =
    std::shared_ptr<RawDataPreprocessorStereoProjective>;

} // namespace srrg2_slam_interfaces
