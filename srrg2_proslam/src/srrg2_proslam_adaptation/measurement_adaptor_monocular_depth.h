#pragma once
#include "measurement_adaptor_descriptor_based.hpp"

namespace srrg2_slam_interfaces {

  class MeasurementAdaptorMonocularDepth
    : public MeasurementAdaptorDescriptorBased_<srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                                srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      MeasurementAdaptorDescriptorBased_<srrg2_core::PointIntensityDescriptor3fVectorCloud,
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

    bool setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) override;
    void compute() override;

  protected:
    //! TODO standarize input and purge
    template <typename DepthImageType_>
    void _readDepth(srrg2_core::BaseImage* image_depth_raw_, const float& depth_scaling_factor_);

    // ds buffered camera and depth image data messages TODO purge?
    srrg2_core::ImageMessagePtr _intensity_message = nullptr;
    srrg2_core::ImageMessagePtr _depth_message     = nullptr;
  };

  using MeasurementAdaptorMonocularDepthPtr = std::shared_ptr<MeasurementAdaptorMonocularDepth>;

} // namespace srrg2_slam_interfaces
