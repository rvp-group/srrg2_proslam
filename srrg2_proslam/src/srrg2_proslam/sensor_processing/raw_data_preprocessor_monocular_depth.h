#pragma once
#include "raw_data_preprocessor_descriptor_based.hpp"

namespace srrg2_proslam {

  class RawDataPreprocessorMonocularDepth : public RawDataPreprocessorDescriptorBased_<
                                              srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                              srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      RawDataPreprocessorDescriptorBased_<srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                          srrg2_core::PointIntensityDescriptor3fVectorCloud>;

    // ds TODO make this part of the images?
    PARAM(srrg2_core::PropertyString,
          topic_rgb,
          "rgb image topic [/camera/rgb/image]",
          "/camera/rgb/image",
          0);
    PARAM(srrg2_core::PropertyString,
          topic_depth,
          "topic depth image [/camera/depth/image]",
          "/camera/depth/image",
          0);
    PARAM(srrg2_core::PropertyFloat,
          depth_scaling_factor_to_meters,
          "scaling factor used to obtain depth in meters from pixel values",
          1.0,
          nullptr);

    bool setRawData(srrg2_core::BaseSensorMessagePtr msg_) override;
    void compute() override;

  protected:
    //! TODO standarize input and purge
    template <typename DepthImageType_>
    void _readDepth(srrg2_core::BaseImage* image_depth_raw_, const float& depth_scaling_factor_);

    // ds buffered camera and depth image data messages TODO purge?
    srrg2_core::ImageMessagePtr _intensity_message = nullptr;
    srrg2_core::ImageMessagePtr _depth_message     = nullptr;
  };

  using RawDataPreprocessorMonocularDepthPtr = std::shared_ptr<RawDataPreprocessorMonocularDepth>;

} // namespace srrg2_slam_interfaces
