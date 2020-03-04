#pragma once
#include "intensity_feature_extractor_base.h"

namespace srrg2_slam_interfaces {

  //! intensity feature extractor, detects keypoints and computes descriptors
  //! for each keypoint populating a IntensityFeatureVector
  template <typename PointCloudType_>
  class IntensityFeatureExtractorSelective_ : public IntensityFeatureExtractor_<PointCloudType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointCloudType = PointCloudType_;
    using BaseType       = IntensityFeatureExtractor_<PointCloudType_>;
    using ThisType       = IntensityFeatureExtractorSelective_<PointCloudType>;
    using PointType      = typename PointCloudType::PointType;

    PARAM(srrg2_core::PropertyBool,
          enable_full_distance_to_left,
          "enables complete detection from the projection to the left image border",
          false,
          nullptr);

    PARAM(srrg2_core::PropertyBool,
          enable_full_distance_to_right,
          "enables complete detection from the projection to the right image border",
          false,
          nullptr);

    PARAM(srrg2_core::PropertyBool,
          enable_seeding_when_tracking,
          "enables new point seeding when in tracking mode",
          true,
          nullptr);

  public:
    //! initialize module
    void init() override;

    //! detects keypoints in the image
    void computeKeypoints(const cv::Mat& image_, std::vector<cv::KeyPoint>& keypoints_) override;
  };

  // ds configurations
  using IntensityFeatureExtractorSelective2D =
    IntensityFeatureExtractorSelective_<srrg2_core::PointIntensityDescriptor2fVectorCloud>;
  using IntensityFeatureExtractorSelective3D =
    IntensityFeatureExtractorSelective_<srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using IntensityFeatureExtractorSelective2DPtr =
    std::shared_ptr<IntensityFeatureExtractorSelective2D>;
  using IntensityFeatureExtractorSelective3DPtr =
    std::shared_ptr<IntensityFeatureExtractorSelective3D>;

} // namespace srrg2_slam_interfaces
