#include "intensity_feature_extractor_selective.h"

namespace srrg2_proslam {
  using namespace srrg2_core;

  template <typename PointCloudType_>
  void IntensityFeatureExtractorSelective_<PointCloudType_>::init() {
    if (!BaseType::_config_changed) {
      return;
    }

    // ds invalidate bad configurations on the spot
    if (BaseType::_image_rows == 0) {
      throw std::runtime_error(
        "IntensityFeatureExtractorSelective_::init|ERROR: invalid number of image "
        "rows (check configuration!)");
    }
    if (BaseType::_image_cols == 0) {
      throw std::runtime_error(
        "IntensityFeatureExtractorSelective_::init|ERROR: invalid number of image "
        "cols (check configuration!)");
    }

    // ds set up detector
    BaseType::_setDetector(BaseType::param_detector_type.value(),
                           BaseType::param_target_number_of_keypoints.value(),
                           BaseType::param_detector_threshold.value(),
                           BaseType::_keypoint_detector);

    // ds allocate descriptor extractor
    BaseType::_setDescriptor(BaseType::param_descriptor_type.value(),
                             BaseType::_descriptor_extractor);

    // ds configuration is valid at this point
    assert(BaseType::_keypoint_detector);
    assert(BaseType::_descriptor_extractor);
    std::cerr << "IntensityFeatureExtractor::init|initialized for image sources ["
              << BaseType::_image_rows << " x " << BaseType::_image_cols << "]" << std::endl;
    std::cerr << "IntensityFeatureExtractor::init|point detector type: "
              << BaseType::param_detector_type.value() << std::endl;
    std::cerr << "IntensityFeatureExtractor::init|descriptor type: "
              << BaseType::param_descriptor_type.value()
              << " size (Bytes): " << BaseType::_descriptor_extractor->descriptorSize()
              << std::endl;
    BaseType::init();
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractorSelective_<PointCloudType_>::computeKeypoints(
    const cv::Mat& image_,
    std::vector<cv::KeyPoint>& keypoints_) {
    PROFILE_TIME("IntensityFeatureExtractorSelective_::computeKeypoints");
    assert(image_.rows > 0);
    assert(image_.cols > 0);
    if (static_cast<size_t>(image_.rows) != BaseType::_image_rows ||
        static_cast<size_t>(image_.cols) != BaseType::_image_cols || BaseType::_config_changed) {
      BaseType::_image_rows = image_.rows;
      BaseType::_image_cols = image_.cols;
      init();
    }
    keypoints_.clear();

    // ds if projections are available we are in tracking phase
    if (BaseType::_projections && !BaseType::_projections->empty()) {
      PROFILE_TIME("IntensityFeatureExtractorSelective_::detectKeypoints (tracking)");
      constexpr int16_t base_radius = 10;
      const int16_t radius_pixels   = BaseType::_projection_detection_radius + base_radius;
      assert(radius_pixels < 16383 && "bigger data type needed");
      const int16_t radius_x2 = 2 * radius_pixels;
      assert(BaseType::_image_cols < 32768 && "bigger data type needed");
      assert(BaseType::_image_rows < 32768 && "bigger data type needed");

      // ds compute detection masks
      cv::Mat detection_mask_tracking(
        BaseType::_image_rows, BaseType::_image_cols, CV_8UC1, cv::Scalar(0));
      cv::Mat detection_mask_seeding(
        BaseType::_image_rows, BaseType::_image_cols, CV_8UC1, cv::Scalar(1));

      // ds depending on the selected projection range (full bars can be enabled) TODO refactor
      if (param_enable_full_distance_to_left.value() &&
          param_enable_full_distance_to_right.value()) {
        for (const PointType& projection_center : *BaseType::_projections) {
          assert(projection_center.coordinates()(1) >= 0 &&
                 projection_center.coordinates()(1) < BaseType::_image_rows);
          assert(projection_center.coordinates()(0) >= 0 &&
                 projection_center.coordinates()(0) < BaseType::_image_cols);
          const int16_t row    = std::round(projection_center.coordinates()(1));
          const int16_t tl_row = std::max(row - radius_pixels, 0);
          const int16_t height =
            std::min(radius_x2, static_cast<int16_t>(BaseType::_image_rows - tl_row));
          const cv::Rect2i detection_region(0, tl_row, BaseType::_image_cols, height);
          detection_mask_tracking(detection_region).setTo(1);
          detection_mask_seeding(detection_region).setTo(0); // ds maybe final inversion is faster
        }
      } else if (param_enable_full_distance_to_left.value()) {
        for (const PointType& projection_center : *BaseType::_projections) {
          assert(projection_center.coordinates()(1) >= 0 &&
                 projection_center.coordinates()(1) < BaseType::_image_rows);
          assert(projection_center.coordinates()(0) >= 0 &&
                 projection_center.coordinates()(0) < BaseType::_image_cols);
          const int16_t row    = std::round(projection_center.coordinates()(1));
          const int16_t col    = std::round(projection_center.coordinates()(0));
          const int16_t tl_row = std::max(row - radius_pixels, 0);
          const int16_t height =
            std::min(radius_x2, static_cast<int16_t>(BaseType::_image_rows - tl_row));
          const cv::Rect2i detection_region(0, tl_row, col, height);
          detection_mask_tracking(detection_region).setTo(1);
          detection_mask_seeding(detection_region).setTo(0); // ds maybe final inversion is faster
        }
      } else if (param_enable_full_distance_to_right.value()) {
        for (const PointType& projection_center : *BaseType::_projections) {
          assert(projection_center.coordinates()(1) >= 0 &&
                 projection_center.coordinates()(1) < BaseType::_image_rows);
          assert(projection_center.coordinates()(0) >= 0 &&
                 projection_center.coordinates()(0) < BaseType::_image_cols);
          const int16_t row    = std::round(projection_center.coordinates()(1));
          const int16_t col    = std::round(projection_center.coordinates()(0));
          const int16_t tl_row = std::max(row - radius_pixels, 0);
          const int16_t width  = BaseType::_image_cols - col;
          const int16_t height =
            std::min(radius_x2, static_cast<int16_t>(BaseType::_image_rows - tl_row));
          const cv::Rect2i detection_region(col, tl_row, width, height);
          detection_mask_tracking(detection_region).setTo(1);
          detection_mask_seeding(detection_region).setTo(0); // ds maybe final inversion is faster
        }
      } else {
        for (const PointType& projection_center : *BaseType::_projections) {
          assert(projection_center.coordinates()(1) >= 0 &&
                 projection_center.coordinates()(1) < BaseType::_image_rows);
          assert(projection_center.coordinates()(0) >= 0 &&
                 projection_center.coordinates()(0) < BaseType::_image_cols);
          const int16_t row    = std::round(projection_center.coordinates()(1));
          const int16_t col    = std::round(projection_center.coordinates()(0));
          const int16_t tl_row = std::max(row - radius_pixels, 0);
          const int16_t tl_col = std::max(col - radius_pixels, 0);
          const int16_t width =
            std::min(radius_x2, static_cast<int16_t>(BaseType::_image_cols - tl_col));
          const int16_t height =
            std::min(radius_x2, static_cast<int16_t>(BaseType::_image_rows - tl_row));
          const cv::Rect2i detection_region(tl_col, tl_row, width, height);
          detection_mask_tracking(detection_region).setTo(1);
          detection_mask_seeding(detection_region).setTo(0); // ds maybe final inversion is faster
        }
      }

      // ds detect keypoints with the current threshold in the projected regions (track
      // candidates)
      BaseType::_keypoint_detector->detect(image_, keypoints_, detection_mask_tracking);
      //      std::cerr << "IntensityFeatureExtractorSelective_::computeKeypoints|# detected points
      //      for "
      //                   "tracking: "
      //                << keypoints_.size();

      // ds check for complete detection failure
      if (keypoints_.empty()) {
        std::cerr << "IntensityFeatureExtractorSelective_::computeKeypoints|WARNING: no "
                     "keypoints detected "
                     "for projected regions: "
                  << BaseType::_projections->size();
      }
      //      std::cerr << " with radius (px): " << radius_pixels << " ("
      //                << BaseType::_projection_detection_radius << " + " << base_radius << ")"
      //                << std::endl;

      // ds seed new points in the unmasked image regions (map integration optional)
      if (param_enable_seeding_when_tracking.value()) {
        std::vector<cv::KeyPoint> keypoints_new;
        BaseType::_keypoint_detector->detect(image_, keypoints_new, detection_mask_seeding);
        keypoints_.insert(keypoints_.end(), keypoints_new.begin(), keypoints_new.end());
        //        std::cerr
        //          << "IntensityFeatureExtractorSelective_::computeKeypoints|# seeded additional
        //          points: "
        //          << keypoints_new.size() << std::endl;
      }

      // ds reset projections for next call - return into seeding eventually
      BaseType::_projections = nullptr;
    }

    // ds no projections available to mask, full seeding required
    else {
      PROFILE_TIME("IntensityFeatureExtractorSelective_::detectKeypoints (seeding)");

      // ds compute point candidates using the seeding detector without mask - if not set externally
      if (BaseType::_keypoint_detection_mask_is_set) {
        BaseType::_keypoint_detector->detect(
          image_, keypoints_, BaseType::_keypoint_detection_mask);
      } else {
        BaseType::_keypoint_detector->detect(image_, keypoints_);
      }
      //      std::cerr << "IntensityFeatureExtractorSelective_::computeKeypoints|# seeded points: "
      //                << keypoints_.size() << std::endl;

      // ds check for complete detection failure
      if (keypoints_.empty()) {
        std::cerr << "IntensityFeatureExtractorSelective_::computeKeypoints|WARNING: unable to "
                     "seed keypoints"
                  << std::endl;
        return;
      }
    }

    // ds reset keypoint detection masking
    BaseType::_keypoint_detection_mask_is_set = false;
  }

#define SPECIALIZE_TEMPLATE(PointCloudType_)                                            \
  template void IntensityFeatureExtractorSelective_<PointCloudType_>::init();           \
  template void IntensityFeatureExtractorSelective_<PointCloudType_>::computeKeypoints( \
    const cv::Mat& image_, std::vector<cv::KeyPoint>& keypoints_)

  // ds TODO add your point types here
  SPECIALIZE_TEMPLATE(PointIntensityDescriptor2fVectorCloud);
  SPECIALIZE_TEMPLATE(PointIntensityDescriptor3fVectorCloud);

} // namespace srrg2_slam_interfaces
