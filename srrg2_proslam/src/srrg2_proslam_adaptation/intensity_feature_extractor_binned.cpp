#include "intensity_feature_extractor_binned.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename PointCloudType_>
  void IntensityFeatureExtractorBinned_<PointCloudType_>::init() {
    if (!BaseType::_config_changed) {
      return;
    }

    // ds invalidate bad configurations on the spot
    if (param_number_of_detectors_vertical.value() <= 0) {
      throw std::runtime_error("IntensityFeatureExtractor::init|ERROR: invalid number of vertical "
                               "detectors (check configuration!)");
    }
    if (param_number_of_detectors_horizontal.value() <= 0) {
      throw std::runtime_error("IntensityFeatureExtractor::init|ERROR: invalid number of "
                               "horizontal detectors (check configuration!)");
    }
    if (BaseType::_image_rows == 0) {
      throw std::runtime_error("IntensityFeatureExtractor::init|ERROR: invalid number of image "
                               "rows (check configuration!)");
    }
    if (BaseType::_image_cols == 0) {
      throw std::runtime_error("IntensityFeatureExtractor::init|ERROR: invalid number of image "
                               "cols (check configuration!)");
    }

    BaseType::_setDetector(BaseType::param_detector_type.value(),
                           BaseType::param_target_number_of_keypoints.value(),
                           BaseType::param_detector_threshold.value(),
                           BaseType::_keypoint_detector);

    // ds allocate descriptor extractor
    BaseType::_setDescriptor(BaseType::param_descriptor_type.value(),
                             BaseType::_descriptor_extractor);

    // ds cache (and not trigger the changed flag with non-const access) TODO uagh
    const size_t number_of_detectors_vertical =
      const_cast<const PropertyInt&>(param_number_of_detectors_vertical).value();
    assert(number_of_detectors_vertical > 0 && "invalid configuration");
    const size_t number_of_detectors_horizontal =
      const_cast<const PropertyInt&>(param_number_of_detectors_horizontal).value();
    assert(number_of_detectors_horizontal > 0 && "invalid configuration");

    // ds allocate and initialize detector grid structure
    _number_of_detection_regions = number_of_detectors_vertical * number_of_detectors_horizontal;
    _detection_regions.reserve(_number_of_detection_regions);
    const float pixel_rows_per_detector =
      static_cast<float>(BaseType::_image_rows) / number_of_detectors_vertical;
    const float pixel_cols_per_detector =
      static_cast<float>(BaseType::_image_cols) / number_of_detectors_horizontal;
    for (size_t r = 0; r < number_of_detectors_vertical; ++r) {
      for (size_t c = 0; c < number_of_detectors_horizontal; ++c) {
        // ds compute ROI
        cv::Rect region(std::round(c * pixel_cols_per_detector),
                        std::round(r * pixel_rows_per_detector),
                        pixel_cols_per_detector,
                        pixel_rows_per_detector);

        // ds check for non-captured pixels in the last region
        if (r == number_of_detectors_vertical - 1 && c == number_of_detectors_horizontal - 1) {
          region.width  = BaseType::_image_cols - region.x;
          region.height = BaseType::_image_rows - region.y;
        }

        // ds bookkeep region
        _detection_regions.emplace_back(region);
      }
    }
    _target_number_of_keypoints_per_detection_region =
      static_cast<float>(
        const_cast<const PropertyInt&>(BaseType::param_target_number_of_keypoints).value()) /
      _number_of_detection_regions;

    // ds initialize keypoint coordinates to detector index mapping
    if (_keypoint_coordinates_to_region_index.capacity() <
        BaseType::_image_rows * BaseType::_image_cols) {
      _keypoint_coordinates_to_region_index.reserve(BaseType::_image_rows * BaseType::_image_cols);
    }
    _keypoint_coordinates_to_region_index.resize(BaseType::_image_rows, BaseType::_image_cols);
    for (size_t r = 0; r < BaseType::_image_rows; ++r) {
      const size_t row_region =
        std::floor(r / pixel_rows_per_detector) * number_of_detectors_horizontal;
      for (size_t c = 0; c < BaseType::_image_cols; ++c) {
        _keypoint_coordinates_to_region_index(r, c) = row_region + c / pixel_cols_per_detector;
        assert(_keypoint_coordinates_to_region_index(r, c) < _number_of_detection_regions);
      }
    }

    // ds configuration is valid at this point
    assert(BaseType::_keypoint_detector);
    assert(BaseType::_descriptor_extractor);
    std::cerr << "IntensityFeatureExtractorBinned_::init|initialized for image sources ["
              << BaseType::_image_rows << " x " << BaseType::_image_cols << "]";
    std::cerr << " with detector grid [" << number_of_detectors_vertical << " x "
              << number_of_detectors_horizontal << "]" << std::endl;
    std::cerr << "IntensityFeatureExtractorBinned_::init|point detector type: "
              << BaseType::param_detector_type.value() << std::endl;
    std::cerr << "IntensityFeatureExtractorBinned_::init|descriptor type: "
              << BaseType::param_descriptor_type.value()
              << " size (Bytes): " << BaseType::_descriptor_extractor->descriptorSize()
              << std::endl;
    BaseType::init();
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractorBinned_<PointCloudType_>::clear() {
    _detection_regions.clear();
    _keypoint_coordinates_to_region_index.clear();
    BaseType::clear();
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractorBinned_<PointCloudType_>::computeKeypoints(
    const cv::Mat& image_,
    std::vector<cv::KeyPoint>& keypoints_) {
    PROFILE_TIME("IntensityFeatureExtractorBinned_::computeKeypoints");
    assert(image_.rows > 0);
    assert(image_.cols > 0);
    if (static_cast<size_t>(image_.rows) != BaseType::_image_rows ||
        static_cast<size_t>(image_.cols) != BaseType::_image_cols || BaseType::_config_changed) {
      BaseType::_image_rows = image_.rows;
      BaseType::_image_cols = image_.cols;
      init();
    }
    keypoints_.clear();

    if (_detection_regions.empty()) {
      std::cerr
        << "IntensityFeatureExtractorBinned_::computeKeypoints|WARNING: no detection regions set, "
           "no detection will be performed"
        << std::endl;
      return;
    }

    // ds detect keypoints with the current threshold
    {
      PROFILE_TIME("IntensityFeatureExtractorBinned_::detectKeypoints");
      if (BaseType::_keypoint_detection_mask_is_set) {
        BaseType::_keypoint_detector->detect(
          image_, keypoints_, BaseType::_keypoint_detection_mask);
      } else {
        BaseType::_keypoint_detector->detect(image_, keypoints_);
      }
    }

    // ds check for complete detection failure
    if (keypoints_.empty()) {
      std::cerr
        << "IntensityFeatureExtractorBinned_::computeKeypoints|WARNING: no keypoints detected"
        << std::endl;
      return;
    }

    // ds TODO uagh blast this casting to avoid triggering the config changed boolean
    const size_t target_number_of_keypoints(
      const_cast<const PropertyInt&>(BaseType::param_target_number_of_keypoints).value());

    // ds bucket the keypoints into the corresponding regions (exploit order!)
    // ds do this without copying the keypoints (i.e. keypoints_ might be temporarily invalid)
    // ds only perform binning if mask is not set
    if (!BaseType::_keypoint_detection_mask_is_set) {
      PROFILE_TIME("IntensityFeatureExtractorBinned_::filterKeypoints");
      std::vector<std::vector<cv::KeyPoint>> keypoints_per_region;
      keypoints_per_region.resize(_number_of_detection_regions);
      for (cv::KeyPoint& keypoint : keypoints_) {
        const size_t& r = keypoint.pt.y;
        const size_t& c = keypoint.pt.x;
        assert(r < static_cast<size_t>(image_.rows));
        assert(c < static_cast<size_t>(image_.cols));
        keypoints_per_region[_keypoint_coordinates_to_region_index(r, c)].emplace_back(
          std::move(keypoint));
      }

      // ds for each bucket we have to pick a target number of keypoints
      keypoints_.clear();
      keypoints_.reserve(target_number_of_keypoints);
      for (std::vector<cv::KeyPoint>& keypoints_available : keypoints_per_region) {
        // ds check if we can skip selecting
        if (keypoints_available.size() < _target_number_of_keypoints_per_detection_region) {
          // ds pick all available keypoints
          keypoints_.insert(
            keypoints_.end(), keypoints_available.begin(), keypoints_available.end());
        } else {
          // ds sort keypoint vector by decreasing response TODO make this selectable
          std::sort(keypoints_available.begin(),
                    keypoints_available.end(),
                    [](const cv::KeyPoint& a_, const cv::KeyPoint& b_) {
                      return a_.response > b_.response;
                    });

          // ds keep only the best subset according to above criteria
          keypoints_.insert(keypoints_.end(),
                            keypoints_available.begin(),
                            keypoints_available.begin() +
                              _target_number_of_keypoints_per_detection_region);
        }
      }
    }

    // ds reset keypoint detection masking
    BaseType::_keypoint_detection_mask_is_set = false;
    //    std::cerr << "IntensityFeatureExtractorBinned_::computeKeypoints|# binned points: "
    //              << keypoints_.size() << std::endl;
    assert(keypoints_.size() <= static_cast<size_t>(target_number_of_keypoints));
  }

#define SPECIALIZE_TEMPLATE(PointCloudType_)                                         \
  template void IntensityFeatureExtractorBinned_<PointCloudType_>::init();           \
  template void IntensityFeatureExtractorBinned_<PointCloudType_>::clear();          \
  template void IntensityFeatureExtractorBinned_<PointCloudType_>::computeKeypoints( \
    const cv::Mat& image_, std::vector<cv::KeyPoint>& keypoints_)

  // ds TODO add your point types here
  SPECIALIZE_TEMPLATE(PointIntensityDescriptor2fVectorCloud);
  SPECIALIZE_TEMPLATE(PointIntensityDescriptor3fVectorCloud);

} // namespace srrg2_slam_interfaces
