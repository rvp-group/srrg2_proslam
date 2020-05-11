#pragma once
#include <unordered_map>

#include <srrg2_slam_interfaces/instances.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_test/synthetic_world.hpp>

#include "srrg2_proslam/tracking/instances.h"

// ds TODO burst these bastards
using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_proslam;

// ds statistics helper
template <typename RealType_ = float>
class Statistics {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using RealType    = RealType_;
  using Vector3Type = Vector_<RealType, 3>;
  using Matrix3Type = MatrixN_<RealType, 3>;

  void addEntry(const Vector3Type& error_meters_, const Matrix3Type& covariance_) {
    const Vector3Type error_meters_absolute(
      std::fabs(error_meters_.x()), std::fabs(error_meters_.y()), std::fabs(error_meters_.z()));
    accumulated_errors_meters += error_meters_absolute;
    if (error_meters_absolute.norm() < minimum_error_meters.norm()) {
      minimum_error_meters = error_meters_absolute;
    } else if (error_meters_absolute.norm() > maximum_error_meters.norm()) {
      maximum_error_meters = error_meters_absolute;
    }
    const Vector3Type variance(covariance_(0, 0), covariance_(1, 1), covariance_(2, 2));
    accumulated_variance += variance;
    if (variance.norm() < minimum_variance.norm()) {
      minimum_variance = variance;
      if (maximum_variance.norm() == 0) {
        maximum_variance = variance;
      }
    } else if (variance.norm() > maximum_variance.norm()) {
      maximum_variance = variance;
      if (minimum_variance.norm() == 1) {
        minimum_variance = variance;
      }
    }
    ++number_of_entries;
  }

  void print(const std::string& label_ = "") {
    if (number_of_entries > 0) {
      std::cerr << "Statistics|" << label_ << "| number of samples: " << number_of_entries
                << std::endl;
      Vector3Type mean_estimation_error = accumulated_errors_meters;
      mean_estimation_error /= number_of_entries;
      std::cerr << "Statistics|" << label_ << "| mean estimation error (m): ("
                << mean_estimation_error.transpose() << ") [(" << minimum_error_meters.transpose()
                << "), (" << maximum_error_meters.transpose() << ")]" << std::endl;
      Vector3Type mean_variance = accumulated_variance;
      mean_variance /= number_of_entries;
      std::cerr << "Statistics|" << label_ << "| mean variance (m): (" << mean_variance.transpose()
                << ") [(" << minimum_variance.transpose() << "), (" << maximum_variance.transpose()
                << ")]" << std::endl;
    }
  }

  void clear() {
    accumulated_errors_meters.setZero();
    minimum_error_meters.setOnes();
    maximum_error_meters.setZero();
    accumulated_variance.setZero();
    minimum_variance.setOnes();
    maximum_variance.setZero();
    number_of_entries = 0;
  }

protected:
  Vector3Type accumulated_errors_meters = Vector3Type::Zero();
  Vector3Type minimum_error_meters      = Vector3Type::Ones();
  Vector3Type maximum_error_meters      = Vector3Type::Zero();
  Vector3Type accumulated_variance      = Vector3Type::Zero();
  Vector3Type minimum_variance          = Vector3Type::Ones();
  Vector3Type maximum_variance          = Vector3Type::Zero();
  size_t number_of_entries              = 0;
};
using StatisticsFloat  = Statistics<float>;
using StatisticsDouble = Statistics<double>;

// ds TODO define base class for dataset-based test fixtures
template <typename RealType_ = float>
class Synthetic : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using RealType       = RealType_;
  using Vector2Type    = Vector_<RealType, 2>;
  using Vector3Type    = Vector_<RealType, 3>;
  using Vector4Type    = Vector_<RealType, 4>;
  using Matrix3Type    = MatrixN_<RealType, 3>;
  using IsometryType   = Isometry3_<RealType>;
  using AngleAxisType  = AngleAxis_<RealType>;
  using IsometryVector = std::vector<IsometryType, Eigen::aligned_allocator<IsometryType>>;
  using Point2DVector  = std::vector<Vector2Type, Eigen::aligned_allocator<Vector2Type>>;
  using Point3DVector  = std::vector<Vector3Type, Eigen::aligned_allocator<Vector3Type>>;
  using Point4DVector  = std::vector<Vector4Type, Eigen::aligned_allocator<Vector4Type>>;
  enum class TransitionSampleType { Rotation, Translation, Transform };
  using ProjectorType    = PointIntensityDescriptor3fProjectorPinhole;
  using ProjectorTypePtr = std::shared_ptr<ProjectorType>;

  void generateContinousTransitions(const TransitionSampleType& sample_type_,
                                    const size_t number_of_samples_                    = 100,
                                    const RealType& standard_deviation_point_position_ = 0,
                                    const RealType& standard_deviation_motion_         = 0,
                                    const RealType& standard_deviation_measurement_    = 0) {
    camera_transitions_ground_truth.clear();
    camera_transitions_noisy.clear();
    point_positions_ground_truth.clear();
    point_positions_noisy.clear();
    point_projections_ground_truth.clear();
    point_projections_noisy.clear();
    point_projections_depth_ground_truth.clear();
    point_projections_depth_noisy.clear();
    stereo_point_projections_ground_truth.clear();
    stereo_point_projections_noisy.clear();

    camera_transitions_ground_truth.reserve(number_of_samples_);
    camera_transitions_noisy.reserve(number_of_samples_);
    point_positions_ground_truth.reserve(number_of_samples_ + 1);
    point_positions_noisy.reserve(number_of_samples_ + 1);
    point_projections_ground_truth.reserve(number_of_samples_ + 1);
    point_projections_noisy.reserve(number_of_samples_ + 1);
    point_projections_depth_ground_truth.reserve(number_of_samples_ + 1);
    point_projections_depth_noisy.reserve(number_of_samples_ + 1);
    stereo_point_projections_ground_truth.reserve(number_of_samples_ + 1);
    stereo_point_projections_noisy.reserve(number_of_samples_ + 1);

    // ds sample initial point (in the first camera frame)
    // ds the more samples are desired the farther away we put the point initially
    Vector3Type point_in_camera(0, 0, std::sqrt(number_of_samples_));
    point_positions_ground_truth.emplace_back(point_in_camera);
    point_positions_noisy.emplace_back(point_in_camera +
                                       _getRandomVector<3>(0, standard_deviation_point_position_));
    Vector2Type point_projection(_projectPinhole(point_in_camera));
    point_projections_ground_truth.emplace_back(point_projection);
    point_projections_noisy.emplace_back(point_projection +
                                         _getRandomVector<2>(0, standard_deviation_measurement_));
    Vector3Type point_projection_depth(
      point_projection(0), point_projection(1), point_in_camera(2));
    point_projections_depth_ground_truth.emplace_back(point_projection_depth);
    point_projections_depth_noisy.emplace_back(
      point_projection_depth + _getRandomVector<3>(0, standard_deviation_measurement_));

    // ds project corresponding rigid stereo point
    Vector2Type point_projection_right(_projectPinhole(point_in_camera, stereo_camera_baseline));
    Vector4Type stereo_point_projection;
    stereo_point_projection << point_projection.x(), point_projection.y(),
      point_projection_right.x(), point_projection_right.y();
    stereo_point_projections_ground_truth.emplace_back(stereo_point_projection);
    stereo_point_projections_noisy.emplace_back(
      stereo_point_projection + _getRandomVector<4>(0, standard_deviation_measurement_));

    // ds accumulated rotations (to stay within reasonable field of view)
    constexpr RealType maximum_accumulated_rotation = M_PI / 4;
    RealType accumulated_rotation_angle_x           = 0;
    RealType accumulated_rotation_angle_y           = 0;
    RealType accumulated_rotation_angle_z           = 0;

    // ds sample transitions and compute corresponding point positions and projections
    // ds TODO proper noisy propagation of point position (now we propagate with gt)
    // ds TODO enable proper projection sampling, currently they're always visible
    for (size_t i = 0; i < number_of_samples_; ++i) {
      // ds sample true and noisy transition
      IsometryType motion(IsometryType::Identity());

      // ds sample in the desired dimensions
      if (sample_type_ == TransitionSampleType::Rotation ||
          sample_type_ == TransitionSampleType::Transform) {
        Vector3Type angles_radians(Vector3Type::Random());
        angles_radians *= M_PI / 36.0; // ds maximum +-5 deg change

        // ds flip angles that would cause the trajectory to go overboard
        if (std::fabs(accumulated_rotation_angle_x + angles_radians.x()) >
            maximum_accumulated_rotation) {
          angles_radians.x() = -angles_radians.x();
        }
        if (std::fabs(accumulated_rotation_angle_y + angles_radians.y()) >
            maximum_accumulated_rotation) {
          angles_radians.y() = -angles_radians.y();
        }
        if (std::fabs(accumulated_rotation_angle_z + angles_radians.z()) >
            maximum_accumulated_rotation) {
          angles_radians.z() = -angles_radians.z();
        }

        // ds apply angles
        motion.rotate(AngleAxisType(angles_radians.x(), Vector3Type::UnitX()));
        motion.rotate(AngleAxisType(angles_radians.y(), Vector3Type::UnitY()));
        motion.rotate(AngleAxisType(angles_radians.z(), Vector3Type::UnitZ()));
        accumulated_rotation_angle_x += angles_radians.x();
        accumulated_rotation_angle_y += angles_radians.y();
        accumulated_rotation_angle_z += angles_radians.z();
      }
      if (sample_type_ == TransitionSampleType::Translation ||
          sample_type_ == TransitionSampleType::Transform) {
        motion.translation() = Vector3Type::Random();
        motion.translation() /= 10.0; // ds maximum +- 10 cm
      }

      // ds compute point position for that motion
      const Vector3Type point_in_camera_previous(point_in_camera);
      point_in_camera = motion * point_in_camera_previous;

      // ds if the point lies behind the camera (nasty motion)
      if (point_in_camera.z() <= 0) {
        // ds flip translation in z and invert rotation
        motion.translation().z()   = -motion.translation().z();
        const Matrix3Type rotation = motion.linear().inverse();
        motion.linear()            = rotation;

        // ds recompute point with fixed motion
        point_in_camera = motion * point_in_camera_previous;
      }
      camera_transitions_ground_truth.emplace_back(motion);
      assert(point_in_camera.z() > 0);
      point_positions_ground_truth.emplace_back(point_in_camera);

      // ds sample in the desired dimensions
      IsometryType motion_noisy(motion);
      if (sample_type_ == TransitionSampleType::Rotation ||
          sample_type_ == TransitionSampleType::Transform) {
        const Vector3Type angles_radians(_getRandomVector<3>(0, standard_deviation_motion_));
        motion_noisy.rotate(AngleAxisType(angles_radians.x(), Vector3Type::UnitX()));
        motion_noisy.rotate(AngleAxisType(angles_radians.y(), Vector3Type::UnitY()));
        motion_noisy.rotate(AngleAxisType(angles_radians.z(), Vector3Type::UnitZ()));
      }
      if (sample_type_ == TransitionSampleType::Translation ||
          sample_type_ == TransitionSampleType::Transform) {
        motion_noisy.translation() += _getRandomVector<3>(0, standard_deviation_motion_);
      }

      // ds check noisy point validity
      Vector3Type point_in_camera_noisy(motion_noisy * point_in_camera_previous);
      if (point_in_camera_noisy.z() <= 0) {
        // ds flip translation in z and invert rotation
        motion_noisy.translation().z() = -motion_noisy.translation().z();
        const Matrix3Type rotation     = motion_noisy.linear().inverse();
        motion_noisy.linear()          = rotation;

        // ds recompute point with fixed motion
        point_in_camera_noisy = motion_noisy * point_in_camera_previous;
      }
      camera_transitions_noisy.emplace_back(motion_noisy);
      assert(point_in_camera_noisy.z() > 0);
      point_positions_noisy.emplace_back(point_in_camera_noisy);

      // ds compute point projections
      point_projection = _projectPinhole(point_in_camera);
      point_projections_ground_truth.emplace_back(point_projection);
      const Vector2Type point_projection_noisy(
        point_projection + _getRandomVector<2>(0, standard_deviation_measurement_));
      point_projections_noisy.emplace_back(point_projection_noisy);

      // ds compute point projections with depth
      point_projection_depth =
        Vector3Type(point_projection(0), point_projection(1), point_in_camera(2));
      point_projections_depth_ground_truth.emplace_back(point_projection_depth);
      point_projections_depth_noisy.emplace_back(
        point_projection_depth + _getRandomVector<3>(0, standard_deviation_measurement_));

      // ds project corresponding rigid stereo point
      point_projection_right = _projectPinhole(point_in_camera, stereo_camera_baseline);

      // ds we want to have a valid horizontal disparity at all times
      assert(point_projection.x() > point_projection_right.x());
      stereo_point_projection << point_projection.x(), point_projection.y(),
        point_projection_right.x(), point_projection_right.y();
      stereo_point_projections_ground_truth.emplace_back(stereo_point_projection);
      const Vector2Type point_projection_right_noisy(
        point_projection_right + _getRandomVector<2>(0, standard_deviation_measurement_));
      Vector4Type stereo_point_projection_noisy;
      stereo_point_projection_noisy << point_projection_noisy.x(), point_projection_noisy.y(),
        point_projection_right_noisy.x(), point_projection_right_noisy.y();
      stereo_point_projections_noisy.emplace_back(stereo_point_projection_noisy);
    }
  }

protected:
  void SetUp() override {
    // ds constant seed for reproducible tests
    srand(0);
    random_device = std::mt19937(0);

    // ds prepare scenario
    points_in_world.clear();
    points_in_world.reserve(100);
    for (size_t i = 0; i < 100; ++i) {
      PointIntensityDescriptor_<3, RealType> point;
      point.coordinates() = Vector3_<RealType>::Random() * 10.0f;
      points_in_world.emplace_back(point);
    }
    points_in_world_copy = points_in_world;
    sensor_in_robot.setIdentity();
    //    sensor_in_robot.translation() = Vector3_<RealType>(0.2, 0.3, 0.4);
    //    sensor_in_robot.linear()      = geometry3d::a2r(Vector3_<RealType>(0, M_PI * 0.05, 0));

    robot_in_world.setIdentity();
    robot_in_world.translation() += Vector3_<RealType>(2, 2.5, 0);
    robot_in_world.rotate(Eigen::AngleAxis<RealType>(-0.2 * M_PI, Vector3_<RealType>::UnitZ()));
    world_in_robot = robot_in_world.inverse();

    camera_current_in_world = robot_in_world * sensor_in_robot;
    world_in_camera_current = camera_current_in_world.inverse();
    points_in_camera.clear();
    points_in_camera.reserve(100);
    correspondences.clear();
    correspondences.reserve(100);
    for (size_t i = 0; i < 100; ++i) {
      PointIntensityDescriptor_<3, RealType> point;
      point.coordinates() = world_in_camera_current * points_in_world[i].coordinates();
      points_in_camera.emplace_back(point);
      correspondences.emplace_back(Correspondence(i, i, 0));
    }

    // ds allocate a realistic camera matrix
    camera_projection_matrix << fx, 0, image_cols * .5, 0, fy, image_rows * .5, 0, 0, 1;

    // ds define a rectified stereo baseline [px]
    stereo_camera_baseline << 250, 0, 0;

    projector.reset(new ProjectorType());
    projector->setCameraMatrix(camera_projection_matrix.template cast<float>());
    projector->param_canvas_cols.setValue(image_cols);
    projector->param_canvas_rows.setValue(image_rows);

    platform.reset(new Platform);

    Isometry3f tf_right_in_left    = Isometry3f::Identity();
    tf_right_in_left.translation() = camera_projection_matrix.inverse().template cast<float>() *
                                     stereo_camera_baseline.template cast<float>();

    TransformEventPtr tf1_event(
      new TransformEvent(0, "camera_right", tf_right_in_left, "camera_left"));
    platform->addEvent(tf1_event);

    TransformEventPtr tf2_event(
      new TransformEvent(0, "camera_left", sensor_in_robot.template cast<float>(), "base_frame"));
    platform->addEvent(tf2_event);
  }

  void TearDown() override {
    correspondences.clear();
    camera_transitions_ground_truth.clear();
    camera_transitions_noisy.clear();
    point_positions_ground_truth.clear();
    point_positions_noisy.clear();
    point_projections_ground_truth.clear();
    point_projections_noisy.clear();
  }

  Vector2Type _projectPinhole(const Vector3Type& point_in_camera_,
                              const Vector3Type& baseline_ = Vector3Type::Zero()) const {
    Vector3Type point_homogeneous = camera_projection_matrix * point_in_camera_;
    point_homogeneous -= baseline_;
    point_homogeneous /= point_homogeneous.z();
    return point_homogeneous.head(2);
  }

  // ds TODO split into proper random module
  RealType _getRandomScalar(const RealType& mean_, const RealType& standard_deviation_) {
    std::normal_distribution<RealType> distribution(mean_, standard_deviation_);
    return distribution(random_device);
  }
  template <size_t Dim_>
  Vector_<RealType, Dim_> _getRandomVector(const RealType& mean_,
                                           const RealType& standard_deviation_) {
    Vector_<RealType, Dim_> random_vector;
    std::normal_distribution<RealType> distribution(mean_, standard_deviation_);
    for (size_t i = 0; i < Dim_; ++i) {
      random_vector(i) = distribution(random_device);
    }
    return random_vector;
  }

  // ds random number generation - constant seed for reproducibility TODO move
  std::mt19937 random_device;

  // ds testables
  Isometry3_<RealType> robot_in_world;
  Isometry3_<RealType> world_in_robot;
  Isometry3_<RealType> camera_current_in_world;
  Isometry3_<RealType> world_in_camera_current;
  Isometry3_<RealType> sensor_in_robot;
  PointIntensityDescriptorVectorCloud<3, RealType> points_in_world;
  PointIntensityDescriptorVectorCloud<3, RealType> points_in_world_copy;
  PointIntensityDescriptorVectorCloud<3, RealType> points_in_camera;
  CorrespondenceVector correspondences;
  Matrix3Type camera_projection_matrix;
  Vector3Type stereo_camera_baseline;
  RealType fx                = 450;
  RealType fy                = 450;
  RealType image_cols        = 600;
  RealType image_rows        = 400;
  ProjectorTypePtr projector = nullptr;

  // ds manually triggered scenarios
  IsometryVector camera_transitions_ground_truth;
  IsometryVector camera_transitions_noisy;
  Point3DVector point_positions_ground_truth;
  Point3DVector point_positions_noisy;
  Point2DVector point_projections_ground_truth;
  Point2DVector point_projections_noisy;
  Point3DVector point_projections_depth_ground_truth;
  Point3DVector point_projections_depth_noisy;
  Point4DVector stereo_point_projections_ground_truth;
  Point4DVector stereo_point_projections_noisy;

  // srrg platform
  PlatformPtr platform;
};
using SyntheticFloat  = Synthetic<float>;
using SyntheticDouble = Synthetic<double>;

class SceneFlow : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
    _filepath_configurations = srrg2_test::filepath_folder_srrg2 + "/srrg2_proslam/configurations";
    _filepath_test_data      = srrg2_test::filepath_folder_srrg2 + "/srrg2_proslam/test_data";

    // ds load srrg types
    messages_registerTypes();
    srrg2_proslam_tracking_registerTypes();

    // ds load raw image data from test folder and verify input
    const cv::Mat image_left_opencv(
      cv::imread(_filepath_test_data + "/scene_flow/image_left.png", CV_LOAD_IMAGE_GRAYSCALE));
    const cv::Mat image_right_opencv(
      cv::imread(_filepath_test_data + "/scene_flow/image_right.png", CV_LOAD_IMAGE_GRAYSCALE));
    ASSERT_EQ(image_left_opencv.rows, image_rows);
    ASSERT_EQ(image_left_opencv.cols, image_cols);
    ASSERT_EQ(image_left_opencv.rows, image_right_opencv.rows);
    ASSERT_EQ(image_left_opencv.cols, image_right_opencv.cols);

    // ds convert messages to generic format TODO make_shared still unusuable?
    // ds image interfaces are very cumbersome TODO add advanced constructors/setImage
    scene_flow_image_left  = new ImageUInt8();
    scene_flow_image_right = new ImageUInt8();
    scene_flow_image_left->fromCv(image_left_opencv);
    scene_flow_image_right->fromCv(image_right_opencv);
    ASSERT_EQ(scene_flow_image_left->rows(), image_rows);
    ASSERT_EQ(scene_flow_image_right->cols(), image_cols);

    // ds upcast and set measurements to adaptor TODO remove smart pointer passing by reference
    MessagePackPtr message_pack = MessagePackPtr(new MessagePack());
    image_message_left  = ImageMessagePtr(new ImageMessage("/camera_left/image_raw", "0", 0, 0.0));
    image_message_right = ImageMessagePtr(new ImageMessage("/camera_right/image_raw", "0", 0, 0.0));
    image_message_left->setImage(scene_flow_image_left);
    image_message_right->setImage(scene_flow_image_right);
    message_pack->messages = {image_message_left, image_message_right};
    rigid_stereo_message   = message_pack;

    // ds load ground truth data
    loadDisparityGT(detector_threshold);
  }

  void TearDown() override {
    disparity_gt.clear();
  }

protected:
  void loadDisparityGT(const int threshold_) {
    // ds load stereo matching ground truth
    std::ifstream file_gt_stereo_matches(_filepath_test_data +
                                         "/scene_flow/gt_stereo_matching_threshold-" +
                                         std::to_string(threshold_) + ".txt");
    ASSERT_TRUE(file_gt_stereo_matches.good());
    ASSERT_TRUE(file_gt_stereo_matches.is_open());

    // ds ground truth storage, indexed by row and col (subsequently)
    disparity_gt.clear();
    disparity_gt.reserve(10000);

    // ds read formatted input
    int coordinate_left_row;
    int coordinate_left_col;
    float coordinate_right_row;
    float coordinate_right_col;
    float disparity;
    while (file_gt_stereo_matches >> coordinate_left_row >> coordinate_left_col >>
           coordinate_right_row >> coordinate_right_col >> disparity) {
      ASSERT_IN_RANGE(coordinate_left_row, 0, static_cast<int>(scene_flow_image_left->rows()));
      ASSERT_IN_RANGE(coordinate_left_col, 0, static_cast<int>(scene_flow_image_left->cols()));
      ASSERT_IN_RANGE(coordinate_right_row, 0.0, scene_flow_image_right->rows());
      ASSERT_IN_RANGE(coordinate_right_col, 0.0, scene_flow_image_right->cols());
      ASSERT_GT(disparity, 0.0);

      // ds populate ground truth storage
      auto iterator_row = disparity_gt.find(coordinate_left_row);
      if (iterator_row != disparity_gt.end()) {
        // ds we already have an entry for this row coordinate
        // ds by construction we cannot have an entry for the current column
        iterator_row->second.insert(std::make_pair(coordinate_left_col, disparity));
      } else {
        // ds we have to add a new entry for this row coordinate
        std::unordered_map<size_t, float> gt_disparity_per_row;
        gt_disparity_per_row.insert(std::make_pair(coordinate_left_col, disparity));
        disparity_gt.insert(std::make_pair(coordinate_left_row, gt_disparity_per_row));
      }
    }
    ASSERT_FALSE(disparity_gt.empty());
  }

  void evaluateStereoMatches(PointIntensityDescriptor4fVectorCloud& stereo_points_,
                             size_t& number_of_inliers_) {
    number_of_inliers_ = 0;
    for (const PointIntensityDescriptor4f& stereo_point : stereo_points_) {
      const size_t row_left  = stereo_point.coordinates()(1);
      const size_t col_left  = stereo_point.coordinates()(0);
      const size_t col_right = stereo_point.coordinates()(2);

      // ds compute horizontal disparity error and count inliers
      try {
        const float disparity           = disparity_gt.at(row_left).at(col_left);
        const float disparity_estimated = std::fabs(col_left - col_right);
        const float disparity_error     = std::fabs(disparity - disparity_estimated);
        if (disparity_error < _maximum_disparity_error_pixels) {
          ++number_of_inliers_;
        }
      } catch (const std::out_of_range& /*exception*/) {
        // ds no ground truth disparity available (not fatal)
      }
    }
  }

protected:
  // ds fixture configuration
  std::string _filepath_test_data             = "";
  std::string _filepath_configurations        = "";
  const float _maximum_disparity_error_pixels = 1.0;
  const size_t image_rows                     = 540;
  const size_t image_cols                     = 960;
  const int detector_threshold                = 100;

  // ds fixture buffers
  std::unordered_map<size_t, std::unordered_map<size_t, float>> disparity_gt;
  ImageUInt8* scene_flow_image_left;
  ImageUInt8* scene_flow_image_right;
  BaseSensorMessagePtr rigid_stereo_message = nullptr;
  ImageMessagePtr image_message_left        = nullptr;
  ImageMessagePtr image_message_right       = nullptr;
};

class ICL : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
    _filepath_configurations = srrg2_test::filepath_folder_srrg2 + "/srrg2_proslam/configurations";
    _filepath_test_data      = srrg2_test::filepath_folder_srrg2 + "/srrg2_proslam/test_data";

    // ds load srrg types
    messages_registerTypes();
    srrg2_proslam_tracking_registerTypes();

    // ds get an adaptor to generate processed measurements
    RawDataPreprocessorMonocularDepth adaptor;
    adaptor.param_feature_extractor->param_target_number_of_keypoints.setValue(500);
    adaptor.param_feature_extractor->param_detector_threshold.setValue(5);
    adaptor.param_depth_scaling_factor_to_meters.setValue(1.0); // ds ICL has true depth images
    adaptor.param_topic_rgb.setValue("/camera/rgb/image_color");
    adaptor.param_topic_depth.setValue("/camera/depth/image");

    // ds set camera calibration
    camera_calibration_matrix << 481.2, 0, 319.5, 0, -481, 239.5, 0, 0, 1;

    // ds obtain sensor message packs
    setMonocularDepthMessage(_filepath_test_data + "/icl/image_rgb_0.png",
                             _filepath_test_data + "/icl/image_depth_0.pgm",
                             image_depth_00,
                             image_intensity_00,
                             message_00);
    setMonocularDepthMessage(_filepath_test_data + "/icl/image_rgb_1.png",
                             _filepath_test_data + "/icl/image_depth_1.pgm",
                             image_depth_01,
                             image_intensity_01,
                             message_01);
    setMonocularDepthMessage(_filepath_test_data + "/icl/image_rgb_50.png",
                             _filepath_test_data + "/icl/image_depth_50.pgm",
                             image_depth_50,
                             image_intensity_50,
                             message_50);

    // ds set ground truth transforms, the googley way
    Isometry3f camera_00_in_world(Isometry3f::Identity());
    camera_00_in_world.translation() << 0, 0, -2.25;
    camera_00_in_world.linear() = Quaternionf(1, 0, 0, 0).toRotationMatrix();
    Isometry3f camera_01_in_world(Isometry3f::Identity());
    camera_01_in_world.translation() << 0.000466347, 0.00895357, -2.24935;
    camera_01_in_world.linear() =
      Quaternionf(0.999999, -0.00101358, 0.00052453, -0.000231475).toRotationMatrix();
    Isometry3f camera_50_in_world(Isometry3f::Identity());
    camera_50_in_world.translation() << 0.129723, 0.00959134, -2.25525;
    Quaternionf orientation_50_in_world;
    camera_50_in_world.linear() =
      Quaternionf(0.995539, -0.00521396, 0.0821083, 0.0461804).toRotationMatrix();

    Isometry3f world_in_camera_00 = camera_00_in_world.inverse();
    Isometry3f world_in_camera_01 = camera_01_in_world.inverse();

    camera_01_in_00 = world_in_camera_00 * camera_01_in_world;
    camera_50_in_01 = world_in_camera_01 * camera_50_in_world;
    camera_50_in_00 = world_in_camera_00 * camera_50_in_world;

    ASSERT_NOTNULL(message_00);
    ASSERT_NOTNULL(message_01);
    ASSERT_NOTNULL(message_50);

    // ds configure unprojector and projector
    PointIntensityDescriptor3fUnprojectorPinhole unprojector;
    unprojector.param_range_min.setValue(0.1);
    unprojector.param_range_max.setValue(10.0);
    unprojector.setCameraMatrix(camera_calibration_matrix);
    projector = PointIntensityDescriptor3fProjectorPinholePtr(
      new PointIntensityDescriptor3fProjectorPinhole());
    projector->setCameraMatrix(camera_calibration_matrix);
    projector->param_canvas_cols.setValue(image_cols);
    projector->param_canvas_rows.setValue(image_rows);

    // ds adapt point measurements from UV and D to UV-D
    ASSERT_TRUE(adaptor.setRawData(message_00));
    adaptor.setMeas(&measurements_projective_depth_00);
    adaptor.compute();
    ASSERT_TRUE(adaptor.setRawData(message_01));
    adaptor.setMeas(&measurements_projective_depth_01);
    adaptor.compute();
    ASSERT_TRUE(adaptor.setRawData(message_50));
    adaptor.setMeas(&measurements_projective_depth_50);
    adaptor.compute();

    // ds unproject points (sparse)
    unprojector.compute(measurements_projective_depth_00, points_in_camera_00);
    ASSERT_EQ(points_in_camera_00.size(), measurements_projective_depth_00.size());
    unprojector.compute(measurements_projective_depth_01, points_in_camera_01);
    ASSERT_EQ(points_in_camera_01.size(), measurements_projective_depth_01.size());
    unprojector.compute(measurements_projective_depth_50, points_in_camera_50);
    ASSERT_EQ(points_in_camera_50.size(), measurements_projective_depth_50.size());

    // ds unproject points (dense)
    points_in_camera_00_dense.resize(image_depth_00->size());
    const size_t number_of_points =
      unprojector.compute(points_in_camera_00_dense.data(), *image_depth_00);
    points_in_camera_00_dense.resize(number_of_points);
    ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

    // ds compute mirror correspondences
    correspondences_camera_00_from_00.clear();
    correspondences_camera_00_from_00.reserve(points_in_camera_00.size());
    for (size_t index = 0; index < points_in_camera_00.size(); ++index) {
      correspondences_camera_00_from_00.emplace_back(Correspondence(index, index, 0));
    }

    // ds compute ideal correspondences using the known camera motion
    correspondences_camera_01_from_00.clear();
    correspondences_camera_01_from_00.reserve(points_in_camera_00.size());
    PointIntensityDescriptor3fVectorCloud points_01_from_00(points_in_camera_00);
    //  was ->  points_01_from_00.transformInPlace(camera_01_in_00);
    points_01_from_00.transformInPlace(camera_01_in_00.inverse());
    // srrg ----- end
    std::set<int> added_indices;
    for (size_t index_00 = 0; index_00 < points_01_from_00.size(); ++index_00) {
      const PointIntensityDescriptor3f& point_00 = points_01_from_00[index_00];

      // ds look for the closest measurement in appearance and geometry space
      float distance_appearance_best = 100; // ds descriptor bits
      float distance_geometry_best   = 0.1; // ds meters squared
      int index_best_01              = -1;
      for (size_t index_01 = 0; index_01 < points_in_camera_01.size(); ++index_01) {
        // ds horrible way to enforce bijective correspondences - ignore already consumed indices
        if (added_indices.count(index_01)) {
          continue;
        }
        const PointIntensityDescriptor3f& point_01 = points_in_camera_01[index_01];
        const float distance_appearance = point_01.field<2>().distance(point_00.field<2>());
        const float distance_geometry =
          (point_01.coordinates() - point_00.coordinates()).squaredNorm();

        if (distance_appearance < distance_appearance_best &&
            distance_geometry < distance_geometry_best) {
          distance_appearance_best = distance_appearance;
          distance_geometry_best   = distance_geometry;
          index_best_01            = index_01;
        }
      }

      // ds add only the best correspondences
      if (index_best_01 != -1) {
        correspondences_camera_01_from_00.emplace_back(
          Correspondence(index_00, index_best_01, distance_appearance_best));
        added_indices.insert(index_best_01);
      }
    }

    // ds grab raw measurements (before adaption)
    measurements_projective_50.clear();
    measurements_projective_50.reserve(measurements_projective_depth_50.size());
    for (const PointIntensityDescriptor3f& point_depth : measurements_projective_depth_50) {
      PointIntensityDescriptor2f point_projective;
      point_projective.coordinates()(0) = point_depth.coordinates()(0);
      point_projective.coordinates()(1) = point_depth.coordinates()(1);
      point_projective.descriptor()     = point_depth.descriptor();
      measurements_projective_50.emplace_back(point_projective);
    }
  }

  void TearDown() override {
  }

protected:
  void setMonocularDepthMessage(const std::string& filepath_image_intensity_,
                                const std::string& filepath_image_depth_,
                                ImageFloat*& image_depth_,
                                ImageUInt8*& image_intensity_,
                                BaseSensorMessagePtr& monocular_depth_message_) {
    // ds load raw image data from test folder and verify input
    const cv::Mat image_intensity_opencv(
      cv::imread(filepath_image_intensity_, CV_LOAD_IMAGE_GRAYSCALE));
    const cv::Mat image_depth_opencv(cv::imread(filepath_image_depth_, CV_LOAD_IMAGE_ANYDEPTH));
    ASSERT_EQ(image_intensity_opencv.rows, image_rows);
    ASSERT_EQ(image_intensity_opencv.cols, image_cols);
    ASSERT_EQ(image_intensity_opencv.rows, image_depth_opencv.rows);
    ASSERT_EQ(image_intensity_opencv.cols, image_depth_opencv.cols);

    // ds convert images to srrg and proper format for projection/unprojection
    ImageUInt16 image_depth_temporary;
    image_depth_temporary.fromCv(image_depth_opencv);
    image_depth_ = new ImageFloat();
    image_depth_temporary.convertTo(*image_depth_, 1e-3);
    image_intensity_ = new ImageUInt8();
    image_intensity_->fromCv(image_intensity_opencv);
    ASSERT_EQ(image_intensity_->rows(), static_cast<size_t>(image_rows));
    ASSERT_EQ(image_depth_->cols(), static_cast<size_t>(image_cols));

    // ds upcast and set measurements to adaptor TODO remove smart pointer passing by reference
    MessagePackPtr intensity_depth_message(new MessagePack());
    ImageMessagePtr message_intensity(new ImageMessage("/camera/rgb/image_color", "0", 0, 0.0));
    ImageMessagePtr message_depth(new ImageMessage("/camera/depth/image", "0", 0, 0.0));
    message_intensity->setImage(image_intensity_);
    message_depth->setImage(image_depth_);
    intensity_depth_message->messages = {message_intensity, message_depth};

    // ds set out parameter
    monocular_depth_message_ = intensity_depth_message;
  }

protected:
  // ds fixture configuration
  std::string _filepath_test_data      = "";
  std::string _filepath_configurations = "";
  Matrix3f camera_calibration_matrix;
  const size_t image_rows = 480;
  const size_t image_cols = 640;
  PointIntensityDescriptor3fProjectorPinholePtr projector;

  // ds measurements - in order of increasing abstraction
  ImageUInt8 *image_intensity_00 = nullptr, *image_intensity_01 = nullptr,
             *image_intensity_50 = nullptr;
  ImageFloat *image_depth_00 = nullptr, *image_depth_01 = nullptr, *image_depth_50 = nullptr;
  BaseSensorMessagePtr message_00 = nullptr;
  BaseSensorMessagePtr message_01 = nullptr;
  BaseSensorMessagePtr message_50 = nullptr;

  PointIntensityDescriptor2fVectorCloud measurements_projective_50;
  PointIntensityDescriptor3fVectorCloud measurements_projective_depth_00;
  PointIntensityDescriptor3fVectorCloud measurements_projective_depth_01;
  PointIntensityDescriptor3fVectorCloud measurements_projective_depth_50;

  // ds correspondences
  CorrespondenceVector correspondences_camera_00_from_00;
  CorrespondenceVector correspondences_camera_01_from_00;

  // ds map data
  PointIntensityDescriptor3fVectorCloud points_in_camera_00;
  PointIntensityDescriptor3fVectorCloud points_in_camera_01;
  PointIntensityDescriptor3fVectorCloud points_in_camera_50;
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_dense;

  // ds motion
  Isometry3f camera_01_in_00;
  Isometry3f camera_50_in_01;
  Isometry3f camera_50_in_00;
};

class KITTI : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void SetUp() override {
    _filepath_configurations = srrg2_test::filepath_folder_srrg2 + "/srrg2_proslam/configurations";
    _filepath_test_data      = srrg2_test::filepath_folder_srrg2 + "/srrg2_proslam/test_data";

    // ds load srrg types
    messages_registerTypes();
    srrg2_proslam_tracking_registerTypes();

    // ds set rigid stereo camera calibration
    camera_calibration_matrix << 718.856, 0, 607.193, 0, 718.856, 185.216, 0, 0, 1;
    baseline_right_in_left_pixels << 386.1448, 0, 0;

    // srrg set the platform
    //   create a TFEvent and push it to the platform
    Isometry3f tf_right_in_left = Isometry3f::Identity();
    tf_right_in_left.translation() << 0.537166, 0, 0;
    TransformEventPtr tf_event(
      new TransformEvent(0, "camera_right", tf_right_in_left, "camera_left"));
    _platform.reset(new Platform);
    _platform->addEvent(tf_event);

    // ds allocate a projector
    projector = PointIntensityDescriptor3fProjectorPinholePtr(
      new PointIntensityDescriptor3fProjectorPinhole());
    projector->setCameraMatrix(camera_calibration_matrix);
    projector->param_canvas_cols.setValue(image_cols);
    projector->param_canvas_rows.setValue(image_rows);
    projector->param_range_min.setValue(0.1f);
    projector->param_range_max.setValue(1000.0f);

    // ds allocate adaptor and merger
    adaptor = RawDataPreprocessorStereoProjectivePtr(new RawDataPreprocessorStereoProjective());
    ASSERT_NOTNULL(adaptor->param_correspondence_finder.value());
    adaptor->param_correspondence_finder->param_maximum_descriptor_distance.setValue(50);
    adaptor->param_correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.8);
    adaptor->param_feature_extractor.setValue(
      IntensityFeatureExtractorBase3DPtr(new IntensityFeatureExtractorBinned3D()));
    adaptor->param_feature_extractor->param_target_number_of_keypoints.setValue(500);
    adaptor->param_feature_extractor->param_detector_threshold.setValue(detector_threshold);
    adaptor->param_feature_extractor->param_detector_type.setValue("FAST");
    adaptor->param_feature_extractor_right.setValue(adaptor->param_feature_extractor.value());
    MergerRigidStereoTriangulation merger;
    merger.param_maximum_distance_appearance.setValue(50);
    merger.param_triangulator->setPlatform(_platform);
    merger.param_triangulator->param_projector.setValue(projector);
    merger.param_triangulator->param_minimum_disparity_pixels.setValue(0);

    // ds load data from disk - TODO refactor this ambiguous bitch
    messages.resize(5);
    images_intensity_left.resize(5);
    images_intensity_right.resize(5);
    measurements.resize(5);
    for (size_t i = 0; i < 5; ++i) {
      const std::string image_number     = std::to_string(i);
      const std::string image_name_left  = "image_left_" + image_number + ".png";
      const std::string image_name_right = "image_right_" + image_number + ".png";
      setStereoMessage(_filepath_test_data + "/kitti/city/" + image_name_left,
                       _filepath_test_data + "/kitti/city/" + image_name_right,
                       messages[i],
                       images_intensity_left[i],
                       images_intensity_right[i]);
    }
    highway_messages.resize(2);
    highway_images_intensity_left.resize(2);
    highway_images_intensity_right.resize(2);
    highway_measurements.resize(2);
    for (size_t i = 0; i < 2; ++i) {
      const std::string image_number     = std::to_string(274 + i);
      const std::string image_name_left  = "image_left_" + image_number + ".png";
      const std::string image_name_right = "image_right_" + image_number + ".png";
      setStereoMessage(_filepath_test_data + "/kitti/highway/" + image_name_left,
                       _filepath_test_data + "/kitti/highway/" + image_name_right,
                       highway_messages[i],
                       highway_images_intensity_left[i],
                       highway_images_intensity_right[i]);
    }
    for (const BaseSensorMessagePtr& message : messages) {
      ASSERT_NOTNULL(message);
    }

    // ds set ground truth transforms (retardedly copied from 00_gt.txt for the above mapping)
    // ds sequence 00
    // ds TODO kill it with fire
    Isometry3f camera_00_in_world;
    camera_00_in_world.translation() << 5.551115e-17, 3.330669e-16, -4.440892e-16;
    camera_00_in_world.linear() << 1.000000e+00, 9.043680e-12, 2.326809e-11, 9.043683e-12,
      1.000000e+00, 2.392370e-10, 2.326810e-11, 2.392370e-10, 9.999999e-01;
    Isometry3f camera_01_in_world;
    camera_01_in_world.translation() << -4.690294e-02, -2.839928e-02, 8.586941e-01;
    camera_01_in_world.linear() << 9.999978e-01, 5.272628e-04, -2.066935e-03, -5.296506e-04,
      9.999992e-01, -1.154865e-03, 2.066324e-03, 1.155958e-03, 9.999971e-01;
    Isometry3f camera_02_in_world;
    camera_02_in_world.translation() << -9.374345e-02, -5.676064e-02, 1.716275e+00;
    camera_02_in_world.linear() << 9.999910e-01, 1.048972e-03, -4.131348e-03, -1.058514e-03,
      9.999968e-01, -2.308104e-03, 4.128913e-03, 2.312456e-03, 9.999887e-01;
    Isometry3f camera_03_in_world;
    camera_03_in_world.translation() << -1.406429e-01, -8.515762e-02, 2.574964e+00;
    camera_03_in_world.linear() << 9.999796e-01, 1.566466e-03, -6.198571e-03, -1.587952e-03,
      9.999927e-01, -3.462706e-03, 6.193102e-03, 3.472479e-03, 9.999747e-01;
    Isometry3f camera_04_in_world;
    camera_04_in_world.translation() << -1.874858e-01, -1.135202e-01, 3.432648e+00;
    camera_04_in_world.linear() << 9.999637e-01, 2.078471e-03, -8.263498e-03, -2.116664e-03,
      9.999871e-01, -4.615826e-03, 8.253797e-03, 4.633149e-03, 9.999551e-01;

    Isometry3f world_in_camera_00 = camera_00_in_world.inverse();
    Isometry3f world_in_camera_01 = camera_01_in_world.inverse();

    camera_01_in_00 = world_in_camera_00 * camera_01_in_world;
    camera_02_in_00 = world_in_camera_00 * camera_02_in_world;
    camera_03_in_00 = world_in_camera_00 * camera_03_in_world;
    camera_04_in_00 = world_in_camera_00 * camera_04_in_world;
    camera_02_in_01 = world_in_camera_01 * camera_02_in_world;

    // ds sequence 01
    Isometry3f camera_274_in_world;
    camera_274_in_world.translation() << 4.949713e+02, 5.038016e+00, -2.224476e+02;
    camera_274_in_world.linear() << -6.337949e-01, -2.127668e-02, 7.732085e-01, 2.566601e-02,
      9.984927e-01, 4.851420e-02, -7.730752e-01, 5.059322e-02, -6.322934e-01;
    Isometry3f camera_275_in_world;
    camera_275_in_world.translation() << 4.970247e+02, 5.116707e+00, -2.241389e+02;
    camera_275_in_world.linear() << -6.339828e-01, -2.092726e-02, 7.730640e-01, 2.586268e-02,
      9.985009e-01, 4.823971e-02, -7.729146e-01, 5.057664e-02, -6.324911e-01;

    highway_camera_275_in_274 = camera_274_in_world.inverse() * camera_275_in_world;

    // ds adapt measurements
    ASSERT_TRUE(adaptor->setRawData(messages[0]));
    adaptor->setMeas(&measurements[0]);
    adaptor->compute();
    points_00_in_image_00.reserve(measurements[0].size());
    for (const PointIntensityDescriptor4f& measurement : measurements[0]) {
      PointIntensityDescriptor3f point;
      point.coordinates()(0) = measurement.coordinates()(0);
      point.coordinates()(1) = measurement.coordinates()(1);
      point.descriptor()     = measurement.descriptor();
      points_00_in_image_00.emplace_back(point);
    }

    // ds compute 3d points in left camera frame (from last state)
    merger.param_triangulator->setDest(&points_in_camera_00);
    merger.param_triangulator->setMoving(&measurements[0]);
    merger.param_triangulator->compute();
    ASSERT_EQ(merger.param_triangulator->indicesInvalidated().size(), 0);
    ASSERT_EQ(points_in_camera_00.size(), measurements[0].size());

    // ds project points in camera 01
    projector->setCameraPose(camera_01_in_00);
    std::vector<int> indices_projected_01_to_camera_00;
    projector->compute(points_in_camera_00,
                       points_00_in_camera_01,
                       points_00_in_image_01,
                       indices_projected_01_to_camera_00);

    // ds adapt measurements
    ASSERT_TRUE(adaptor->setRawData(messages[1]));
    adaptor->setMeas(&measurements[1]);
    adaptor->setProjections(&points_00_in_image_01, 0);
    adaptor->compute();

    // ds compute 3d points in left camera frame (from last state)
    merger.param_triangulator->setDest(&points_in_camera_01);
    merger.param_triangulator->setMoving(&measurements[1]);
    merger.param_triangulator->compute();
    ASSERT_EQ(merger.param_triangulator->indicesInvalidated().size(), 0);
    ASSERT_EQ(points_in_camera_01.size(), measurements[1].size());

    // ds project points in camera 02
    projector->setCameraPose(camera_02_in_00);
    std::vector<int> indices_projected_02_to_camera_00;
    projector->compute(points_in_camera_00,
                       points_00_in_camera_02,
                       points_00_in_image_02,
                       indices_projected_02_to_camera_00);

    // ds adapt measurements
    ASSERT_TRUE(adaptor->setRawData(messages[2]));
    adaptor->setMeas(&measurements[2]);
    adaptor->setProjections(&points_00_in_image_02, 0);
    adaptor->compute();

    // ds compute mirror correspondences
    correspondences_camera_00_from_00.clear();
    correspondences_camera_00_from_00.reserve(points_in_camera_00.size());
    for (size_t index = 0; index < points_in_camera_00.size(); ++index) {
      correspondences_camera_00_from_00.emplace_back(Correspondence(index, index, 0));
    }

    // ds compute ground truth correspondences by projecting 00 into 01
    correspondences_camera_01_from_00.clear();
    correspondences_camera_01_from_00.reserve(points_in_camera_00.size());
    correspondences_camera_00_from_01.clear();
    correspondences_camera_00_from_01.reserve(points_in_camera_00.size());

    // ds for all measured points in image 01
    constexpr float maximum_projection_error       = 5;
    constexpr float maximum_projection_error_noisy = 10;
    std::unordered_set<size_t> indices_00_picked;
    std::unordered_set<size_t> indices_00_picked_noisy;
    for (size_t index_01 = 0; index_01 < measurements[1].size(); ++index_01) {
      const PointIntensityDescriptor4f& point_measured(measurements[1][index_01]);
      const auto& descriptor_field = point_measured.template field<2>();

      // ds look for the nearest projection from 00
      float projection_error_best    = maximum_projection_error_noisy;
      float descriptor_distance_best = 0;
      size_t index_00_best           = 0;
      for (size_t j = 0; j < points_00_in_image_01.size(); ++j) {
        const PointIntensityDescriptor3f& point_projected(points_00_in_image_01[j]);
        const float projection_error =
          (point_measured.coordinates().head(2) - point_projected.coordinates().head(2)).norm();
        if (projection_error < projection_error_best) {
          projection_error_best    = projection_error;
          index_00_best            = indices_projected_01_to_camera_00[j];
          descriptor_distance_best = descriptor_field.distance(point_projected.template field<2>());
        }
      }

      // ds if the projection is in decent range and has not been picked already
      if (projection_error_best < maximum_projection_error_noisy &&
          indices_00_picked_noisy.count(index_00_best) == 0 && descriptor_distance_best < 50) {
        correspondences_camera_01_from_00_noisy.emplace_back(
          Correspondence(index_01, index_00_best, descriptor_distance_best));
        indices_00_picked_noisy.insert(index_00_best);
      }

      // ds if the projection is in close range and has not been picked already
      if (projection_error_best < maximum_projection_error &&
          indices_00_picked.count(index_00_best) == 0 && descriptor_distance_best < 50) {
        correspondences_camera_01_from_00.emplace_back(
          Correspondence(index_01, index_00_best, descriptor_distance_best));
        correspondences_camera_00_from_01.emplace_back(
          Correspondence(index_00_best, index_01, descriptor_distance_best));
        indices_00_picked.insert(index_00_best);
      }
    }

    // ds adapt highway measurements
    ASSERT_TRUE(adaptor->setRawData(highway_messages[0]));
    adaptor->setMeas(&highway_measurements[0]);
    adaptor->compute();
    merger.param_triangulator->setDest(&highway_points_in_camera_274);
    merger.param_triangulator->setMoving(&highway_measurements[0]);
    merger.param_triangulator->compute();
    ASSERT_EQ(merger.param_triangulator->indicesInvalidated().size(), 0);
    ASSERT_EQ(highway_points_in_camera_274.size(), highway_measurements[0].size());
    ASSERT_TRUE(adaptor->setRawData(highway_messages[1]));
    adaptor->setMeas(&highway_measurements[1]);
    adaptor->compute();
    merger.param_triangulator->setDest(&highway_points_in_camera_275);
    merger.param_triangulator->setMoving(&highway_measurements[1]);
    merger.param_triangulator->compute();
    ASSERT_EQ(merger.param_triangulator->indicesInvalidated().size(), 0);
    ASSERT_EQ(highway_points_in_camera_275.size(), highway_measurements[1].size());
  }

  void TearDown() override {
  }

protected:
  void setStereoMessage(const std::string& filepath_image_intensity_left_,
                        const std::string& filepath_image_intensity_right_,
                        BaseSensorMessagePtr& stereo_message_,
                        ImageUInt8*& image_left_,
                        ImageUInt8*& image_right_) {
    // ds load raw image data from test folder and verify input
    const cv::Mat image_intensity_left_opencv(
      cv::imread(filepath_image_intensity_left_, CV_LOAD_IMAGE_GRAYSCALE));
    const cv::Mat image_intensity_right_opencv(
      cv::imread(filepath_image_intensity_right_, CV_LOAD_IMAGE_GRAYSCALE));
    ASSERT_EQ(image_intensity_left_opencv.rows, image_rows);
    ASSERT_EQ(image_intensity_left_opencv.cols, image_cols);
    ASSERT_EQ(image_intensity_right_opencv.rows, image_intensity_left_opencv.rows);
    ASSERT_EQ(image_intensity_right_opencv.cols, image_intensity_left_opencv.cols);

    // ds upcast and set measurements to adaptor TODO remove smart pointer passing by reference
    MessagePackPtr stereo_message(new MessagePack());
    ImageMessagePtr image_message_left(new ImageMessage("/camera_left/image_raw", "0", 0, 0.0));
    ImageMessagePtr image_message_right(new ImageMessage("/camera_right/image_raw", "0", 0, 0.0));
    image_left_ = new ImageUInt8();
    image_left_->fromCv(image_intensity_left_opencv);
    image_message_left->setImage(image_left_);
    image_right_ = new ImageUInt8();
    image_right_->fromCv(image_intensity_right_opencv);
    image_message_right->setImage(image_right_);
    stereo_message->messages = {image_message_left, image_message_right};
    stereo_message_          = stereo_message;
  }

protected:
  // ds fixture configuration
  std::string _filepath_test_data      = "";
  std::string _filepath_configurations = "";
  const size_t image_rows              = 376;
  const size_t image_cols              = 1241;
  const int detector_threshold         = 15;
  Matrix3f camera_calibration_matrix;
  Vector3f baseline_right_in_left_pixels;

  // ds pinhole point projector
  PointIntensityDescriptor3fProjectorPinholePtr projector = nullptr;

  // ds stereo adaptor
  RawDataPreprocessorStereoProjectivePtr adaptor = nullptr;

  PlatformPtr _platform = nullptr;

public:
  // ds measurements
  std::vector<BaseSensorMessagePtr> messages;
  std::vector<ImageUInt8*> images_intensity_left;
  std::vector<ImageUInt8*> images_intensity_right;
  std::vector<PointIntensityDescriptor4fVectorCloud,
              Eigen::aligned_allocator<PointIntensityDescriptor4fVectorCloud>>
    measurements;

  // ds correspondences
  CorrespondenceVector correspondences_camera_00_from_00;
  CorrespondenceVector correspondences_camera_01_from_00;
  CorrespondenceVector correspondences_camera_01_from_00_noisy;
  CorrespondenceVector correspondences_camera_00_from_01;

  // ds map data
  PointIntensityDescriptor3fVectorCloud points_in_camera_00;
  PointIntensityDescriptor3fVectorCloud points_in_camera_01;

  // ds points transforms
  PointIntensityDescriptor3fVectorCloud points_00_in_camera_01;
  PointIntensityDescriptor3fVectorCloud points_00_in_camera_02;

  // ds projections
  PointIntensityDescriptor3fVectorCloud points_00_in_image_00;
  PointIntensityDescriptor3fVectorCloud points_00_in_image_01;
  PointIntensityDescriptor3fVectorCloud points_00_in_image_02;

  // ds motion
  Isometry3f camera_01_in_00;
  Isometry3f camera_02_in_00;
  Isometry3f camera_03_in_00;
  Isometry3f camera_04_in_00;
  Isometry3f camera_02_in_01;

  // ds sequence 01 - TODO refactor UASSSS
  std::vector<BaseSensorMessagePtr> highway_messages;
  std::vector<ImageUInt8*> highway_images_intensity_left;
  std::vector<ImageUInt8*> highway_images_intensity_right;
  std::vector<PointIntensityDescriptor4fVectorCloud,
              Eigen::aligned_allocator<PointIntensityDescriptor4fVectorCloud>>
    highway_measurements;
  PointIntensityDescriptor3fVectorCloud highway_points_in_camera_274;
  PointIntensityDescriptor3fVectorCloud highway_points_in_camera_275;
  Isometry3f highway_camera_275_in_274;
};
