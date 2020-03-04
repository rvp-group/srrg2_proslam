#pragma once
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <srrg_config/configurable.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_image/image.h>
#include <srrg_pcl/point_types.h>
#include <srrg_property/property.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_slam_interfaces {

// TODO move/purge these
#define TO_OPENCV_2F(VECTOR_) cv::Point2f(VECTOR_(0), VECTOR_(1))
#define TO_OPENCV_3F(VECTOR_) cv::Point3f(VECTOR_(0), VECTOR_(1), VECTOR_(2))
#define TO_EIGEN_2F(VECTOR_) Vector2f(VECTOR_.y, VECTOR_.x)
#define TO_EIGEN_3F(VECTOR_) Vector3f(VECTOR_.y, VECTOR_.x, VECTOR_.z)

  class FeatureExtractor : public srrg2_core::Configurable, public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(srrg2_core::PropertyString,
          descriptor_type,
          "OpenCV descriptor type (BRIEF-256, BRIEF-512, ORB-256, FREAK-512, ..)",
          "ORB-256",
          nullptr);

    PARAM(srrg2_core::PropertyString,
          detector_type,
          "OpenCV detector type for point tracking (FAST, MSER, GFTT, BRISK-512, ORB-256, ..)",
          "FAST",
          nullptr);

    PARAM(srrg2_core::PropertyFloat,
          detector_threshold,
          "scalar that maps to the first parameter of the chosen detector (if applicable)",
          10,
          nullptr);

    PARAM(srrg2_core::PropertyFloat,
          target_bin_width_pixels,
          "minimum required distance between detected features (if applicable)",
          10,
          nullptr);

    PARAM(srrg2_core::PropertyBool,
          enable_non_maximum_suppression,
          "enables non maximum suppression (filtering) of detected features (if applicable)",
          true,
          nullptr);

    PARAM(srrg2_core::PropertyInt,
          target_number_of_keypoints,
          "target number of keypoints to detect (accumulative over all detectors)",
          500,
          &_config_changed);

    static std::vector<std::string> available_detectors;
    static std::vector<std::string> available_descriptors;

  protected:
    //! capture configuration update
    bool _config_changed = true;
  };

  //! intensity feature extractor, detects keypoints and computes descriptors
  //! for each keypoint populating a IntensityFeatureVector
  template <typename PointCloudType_>
  class IntensityFeatureExtractor_ : public FeatureExtractor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointCloudType = PointCloudType_; // ds "dest" type
    using BaseType       = FeatureExtractor;
    using ThisType       = IntensityFeatureExtractor_<PointCloudType>;
    using PointType      = typename PointCloudType::PointType;

    //! destructor, free all unmanaged memory
    virtual ~IntensityFeatureExtractor_() {
      clear();
    }

  public:
    //! initialize module (e.g. feature detection grid) - required to be called on config change
    virtual void init() {
      BaseType::_config_changed = false;
    }

    //! clears containers and frees dynamically allocated memory
    virtual void clear() {
      // ds implementation is optional
    }

    //! set target feature buffer
    void setFeatures(PointCloudType* features_) {
      _features = features_;
    }

    //! set optional projections buffer to enable masked detection
    void setProjections(const PointCloudType* projections_,
                        const size_t& projection_detection_radius_) {
      assert(projection_detection_radius_ < 16383 && "bigger data type needed");
      _projections                 = projections_;
      _projection_detection_radius = projection_detection_radius_;
    }

    //! detects keypoints in the image - implementation is mandatory
    virtual void computeKeypoints(const cv::Mat& image_, std::vector<cv::KeyPoint>& keypoints_) = 0;

    //! detects keypoints in the image in srrg2 format
    void computeKeypoints(const cv::Mat& image_, PointCloudType& keypoints_);

    //! computes descriptors in the image
    //! (this operation will remove keypoints without descriptors!)
    void computeDescriptors(const cv::Mat& image_,
                            std::vector<cv::KeyPoint>& keypoints_,
                            cv::Mat& descriptors_);

    //! detects keypoint in the image and computes descriptors for each keypoint
    //! note that keypoints without descriptors are discarded!
    void compute(const cv::Mat& image_);

    //! detects keypoint in the image and computes descriptors for each keypoint
    //! note that keypoints without descriptors are discarded!
    void compute(const srrg2_core::BaseImage* image_);

    //! sets keypoint detection mask (cv::Mat with non-zero values in the region of interest)
    void setKeypointDetectionMask(const cv::Mat& mask_) {
      _keypoint_detection_mask        = mask_;
      _keypoint_detection_mask_is_set = true;
    }

    //! getters for autoconfigured parameters
    const size_t& imageRows() const {
      return _image_rows;
    }
    size_t& imageRows() {
      return _image_rows;
    }
    const size_t& imageCols() const {
      return _image_cols;
    }
    size_t& imageCols() {
      return _image_cols;
    }

  protected:
    //! detector setter for point seeding/tracking, parameters will be set if applicable
    void _setDetector(const std::string& detector_type_,
                      const size_t& target_number_of_keypoints_,
                      const float& detector_threshold_,
                      cv::Ptr<cv::FeatureDetector>& detector_);

    //! descriptor setter for point seeding and tracking
    void _setDescriptor(const std::string& descriptor_type_,
                        cv::Ptr<cv::DescriptorExtractor>& descriptor_);

    //! automatically cached parameters based on input
    size_t _image_rows = 0;
    size_t _image_cols = 0;

    //! last computation buffer (features)
    PointCloudType* _features = nullptr;

    //! [optional] projections to enable masked detection
    const PointCloudType* _projections = nullptr;

    //! [optional] search radius for detection keypoints around projection
    int16_t _projection_detection_radius = 0;

    //! [optional] keypoint detection mask
    cv::Mat _keypoint_detection_mask;
    bool _keypoint_detection_mask_is_set = false;

    //! selected feature detector for point tracking
    cv::Ptr<cv::FeatureDetector> _keypoint_detector;

    //! selected descriptor extractor
    cv::Ptr<cv::DescriptorExtractor> _descriptor_extractor;
  };

  // ds configurations
  using IntensityFeatureExtractorBase2D =
    IntensityFeatureExtractor_<srrg2_core::PointIntensityDescriptor2fVectorCloud>;
  using IntensityFeatureExtractorBase3D =
    IntensityFeatureExtractor_<srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using IntensityFeatureExtractorBase2DPtr = std::shared_ptr<IntensityFeatureExtractorBase2D>;
  using IntensityFeatureExtractorBase3DPtr = std::shared_ptr<IntensityFeatureExtractorBase3D>;

} // namespace srrg2_slam_interfaces
