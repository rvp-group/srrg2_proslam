#pragma once
#include "intensity_feature_extractor_base.h"

namespace srrg2_slam_interfaces {

  //! intensity feature extractor, detects keypoints and computes descriptors
  //! for each keypoint populating a IntensityFeatureVector
  template <typename PointCloudType_>
  class IntensityFeatureExtractorBinned_ : public IntensityFeatureExtractor_<PointCloudType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointCloudType = PointCloudType_; // ds "dest" type
    using BaseType       = IntensityFeatureExtractor_<PointCloudType>;
    using ThisType       = IntensityFeatureExtractorBinned_<PointCloudType>;
    using PointType      = typename PointCloudType::PointType;

    PARAM(srrg2_core::PropertyInt,
          number_of_detectors_horizontal,
          "number of detectors on the horizontal image axis (cols)",
          3,
          &this->_config_changed);

    PARAM(srrg2_core::PropertyInt,
          number_of_detectors_vertical,
          "number of detectors on the vertical image axis (rows)",
          3,
          &this->_config_changed);

  public:
    //! initialize module (e.g. feature detection grid) - required to be called on config change
    void init() override;

    //! clears containers and frees dynamically allocated memory
    void clear() override;

    //! detects keypoints in the image
    void computeKeypoints(const cv::Mat& image_, std::vector<cv::KeyPoint>& keypoints_) override;

    //! additional accessors
    const std::vector<cv::Rect>& detectionRegions() const {
      return _detection_regions;
    }

  protected:
    //! number of detectors (the same for all image streams)
    size_t _number_of_detection_regions                     = 0;
    size_t _target_number_of_keypoints_per_detection_region = 0;

    //! image region for each detector (the same for all image streams)
    std::vector<cv::Rect> _detection_regions;

    //! mapping between keypoint coordinates (locked image dimension) and region index
    srrg2_core::Matrix_<size_t> _keypoint_coordinates_to_region_index;
  };

  // ds configurations
  using IntensityFeatureExtractorBinned2D =
    IntensityFeatureExtractorBinned_<srrg2_core::PointIntensityDescriptor2fVectorCloud>;
  using IntensityFeatureExtractorBinned3D =
    IntensityFeatureExtractorBinned_<srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using IntensityFeatureExtractorBinned2DPtr = std::shared_ptr<IntensityFeatureExtractorBinned2D>;
  using IntensityFeatureExtractorBinned3DPtr = std::shared_ptr<IntensityFeatureExtractorBinned3D>;

} // namespace srrg2_slam_interfaces
