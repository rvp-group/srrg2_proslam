#include "intensity_feature_extractor_base.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  // ds available detector/descriptor types
#if CV_MAJOR_VERSION == 2

  std::vector<std::string> FeatureExtractor::available_detectors = {"FAST"};

  std::vector<std::string> FeatureExtractor::available_descriptors = {"BRIEF-256", "ORB-256"};

#elif CV_MAJOR_VERSION == 3

  std::vector<std::string> FeatureExtractor::available_detectors = {"BRISK-512",
                                                                    "FAST",
                                                                    "GFTT",
                                                                    "MSER",
                                                                    "ORB-256"};

  std::vector<std::string> FeatureExtractor::available_descriptors = {"BRIEF-256",
                                                                      "BRISK-512",
                                                                      "FREAK-512",
                                                                      "ORB-256"};

#endif

  //! detects keypoints in the image in srrg2 format
  template <typename PointCloudType_>
  void IntensityFeatureExtractor_<PointCloudType_>::computeKeypoints(const cv::Mat& image_,
                                                                     PointCloudType& keypoints_) {
    // ds TODO blast this indirection once we have an srrg2 implementation for keypoint detection
    std::vector<cv::KeyPoint> keypoints_opencv;
    computeKeypoints(image_, keypoints_opencv);
    keypoints_.reserve(keypoints_opencv.size());
    for (cv::KeyPoint& keypoint_opencv : keypoints_opencv) {
      PointType keypoint;
      keypoint.coordinates()(0) = keypoint_opencv.pt.x;
      keypoint.coordinates()(1) = keypoint_opencv.pt.y;
      keypoint.intensity()      = image_.at<uchar>(keypoint_opencv.pt);
      keypoints_.emplace_back(keypoint);
    }
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractor_<PointCloudType_>::computeDescriptors(
    const cv::Mat& image_,
    std::vector<cv::KeyPoint>& keypoints_,
    cv::Mat& descriptors_) {
    PROFILE_TIME("IntensityFeatureExtractor::computeDescriptors");
    assert(_descriptor_extractor);
    _descriptor_extractor->compute(image_, keypoints_, descriptors_);
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractor_<PointCloudType_>::compute(const cv::Mat& image_) {
    PROFILE_TIME("IntensityFeatureExtractor::compute");
    if (!_features) {
      std::cerr << "IntensityFeatureExtractor::compute|WARNING: target feature buffer not set, "
                   "ignoring call"
                << std::endl;
      return;
    }

    // ds detect new keypoints in each image region
    std::vector<cv::KeyPoint> keypoints;
    computeKeypoints(image_, keypoints);

    // ds extract descriptors for detected keypoints
    cv::Mat descriptors;
    computeDescriptors(image_, keypoints, descriptors);

    // ds refresh buffer
    _features->clear();
    _features->reserve(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
      const cv::KeyPoint& keypoint = keypoints[i];
      PointType feature;
      feature.coordinates()(0) = keypoint.pt.x;
      feature.coordinates()(1) = keypoint.pt.y;
      feature.intensity()      = image_.at<uchar>(keypoint.pt);
      feature.descriptor()     = descriptors.row(i);
      _features->emplace_back(feature);
    }
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractor_<PointCloudType_>::compute(const BaseImage* image_) {
    // ds TODO remove this wrap once we are independent of opencv
    // ds TODO add checks for supported image types
    cv::Mat image_opencv;
    image_->toCv(image_opencv);
    assert(static_cast<size_t>(image_opencv.rows) == image_->rows());
    assert(static_cast<size_t>(image_opencv.cols) == image_->cols());
    assert(image_opencv.channels() == image_->numChannels());
    compute(image_opencv);
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractor_<PointCloudType_>::_setDetector(
    const std::string& detector_type_,
    const size_t& target_number_of_keypoints_,
    const float& detector_threshold_,
    cv::Ptr<cv::FeatureDetector>& detector_) {
#if CV_MAJOR_VERSION == 2
    // ds TODO implement full cascade
    if (detector_type_ == "GFTT") {
      constexpr double quality = 0.01;
      detector_                = new cv::GFTTDetector(
        target_number_of_keypoints_, quality, param_target_bin_width_pixels.value());
    } else {
      detector_ = new cv::FastFeatureDetector(detector_threshold_,
                                              param_enable_non_maximum_suppression.value());
    }
#else
    // ds TODO thresholds?
    if (detector_type_ == "BRISK-512") {
      detector_ = cv::BRISK::create(detector_threshold_);
    } else if (detector_type_ == "ORB-256") {
      detector_ = cv::ORB::create(target_number_of_keypoints_);
    } else if (detector_type_ == "MSER") {
      detector_ = cv::MSER::create(detector_threshold_);
    } else if (detector_type_ == "FAST") {
      detector_ = cv::FastFeatureDetector::create(detector_threshold_,
                                                  param_enable_non_maximum_suppression.value());
    } else if (detector_type_ == "GFTT") {
      constexpr double quality = 0.01;
      detector_                = cv::GFTTDetector::create(
        target_number_of_keypoints_, quality, param_target_bin_width_pixels.value());
    } else {
      // ds defaults are the devil
      throw std::runtime_error(
        "IntensityFeatureExtractor::_setDetector|ERROR: unknown detector type chosen: " +
        detector_type_);
    }
#endif
  }

  template <typename PointCloudType_>
  void IntensityFeatureExtractor_<PointCloudType_>::_setDescriptor(
    const std::string& descriptor_type_,
    cv::Ptr<cv::DescriptorExtractor>& descriptor_) {
#if CV_MAJOR_VERSION == 2
    if (descriptor_type_ == "BRIEF-256") {
      _descriptor_extractor = new cv::BriefDescriptorExtractor(32);
    } else if (descriptor_type_ == "ORB-256") {
      _descriptor_extractor = new cv::OrbDescriptorExtractor();
    } else {
      // ds default
      _descriptor_extractor = new cv::OrbDescriptorExtractor();
    }
#elif CV_MAJOR_VERSION == 3
    if (descriptor_type_ == "BRIEF-256") {
#ifdef SRRG_OPENCV_CONTRIB
      _descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
#else
      _descriptor_extractor = cv::ORB::create();
#endif
    } else if (descriptor_type_ == "ORB-256") {
      _descriptor_extractor = cv::ORB::create();
    } else if (descriptor_type_ == "BRISK-512") {
      _descriptor_extractor = cv::BRISK::create();
    } else if (descriptor_type_ == "FREAK-512") {
#ifdef SRRG_OPENCV_CONTRIB
      _descriptor_extractor = cv::xfeatures2d::FREAK::create();
#else
      _descriptor_extractor = cv::ORB::create();
#endif
    } else {
      // ds defaults are the devil
      throw std::runtime_error(
        "IntensityFeatureExtractor::_setDescriptor|ERROR: unknown descriptor type chosen: " +
        descriptor_type_);
    }
#endif
  }

#define SPECIALIZE_TEMPLATE(PointCloudType_)                                                   \
  template void IntensityFeatureExtractor_<PointCloudType_>::init();                           \
  template void IntensityFeatureExtractor_<PointCloudType_>::clear();                          \
  template void IntensityFeatureExtractor_<PointCloudType_>::computeKeypoints(                 \
    const cv::Mat& image_, PointCloudType_& keypoints_);                                       \
  template void IntensityFeatureExtractor_<PointCloudType_>::computeDescriptors(               \
    const cv::Mat& image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_);      \
  template void IntensityFeatureExtractor_<PointCloudType_>::compute(const cv::Mat& image_);   \
  template void IntensityFeatureExtractor_<PointCloudType_>::compute(const BaseImage* image_); \
  template void IntensityFeatureExtractor_<PointCloudType_>::_setDetector(                     \
    const std::string& detector_type_,                                                         \
    const size_t& target_number_of_keypoints_,                                                 \
    const float& detector_threshold_,                                                          \
    cv::Ptr<cv::FeatureDetector>& detector_);                                                  \
  template void IntensityFeatureExtractor_<PointCloudType_>::_setDescriptor(                   \
    const std::string& descriptor_type_, cv::Ptr<cv::DescriptorExtractor>& descriptor_)

  // ds TODO add your point types here
  SPECIALIZE_TEMPLATE(PointIntensityDescriptor2fVectorCloud);
  SPECIALIZE_TEMPLATE(PointIntensityDescriptor3fVectorCloud);

} // namespace srrg2_slam_interfaces
