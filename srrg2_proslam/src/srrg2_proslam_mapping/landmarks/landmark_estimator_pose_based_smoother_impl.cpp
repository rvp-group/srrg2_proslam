#include "landmark_estimator_pose_based_smoother.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename MeasurementVectorType_, typename LandmarkType_>
  void LandmarkEstimatorPoseBasedSmoother_<MeasurementVectorType_, LandmarkType_>::compute() {
    PROFILE_TIME("LandmarkEstimatorPoseBasedSmoother::compute");
    assert(BaseType::_landmark);
    assert(BaseType::_landmark->statistics().value);
    assert(BaseType::_landmark->status == POINT_STATUS::Valid);
    BaseType::_landmark->statistics().setIsInlier(false);

    // ds fetch all past measurements and add current measurement to statistics
    BaseType::_landmark->statistics().addMeasurement(
      PointStatisticsField3D::CameraMeasurement(BaseType::_measurement.head(LandmarkType_::Dim),
                                                BaseType::_landmark_estimate_in_tracker,
                                                BaseType::_tracker_from_world,
                                                BaseType::_world_from_tracker));
    const PointStatisticsField3D::CameraMeasurementVector& measurements =
      BaseType::_landmark->statistics().measurements();
    const Vector3f& initial_coordinates_in_world(BaseType::_landmark->statistics().state());
    const float& maximum_kernel_error = param_maximum_reprojection_error_pixels_squared.value();

    // ds variable to optimize
    Vector3f world_coordinates(BaseType::_landmark->statistics().state());

    // ds if the minimum number of measurements is not reached yet
    if (measurements.size() < param_minimum_number_of_measurements_for_optimization.value()) {
      _setMeanCoordinatesInWorld(measurements, world_coordinates);

      // ds perform geometric distance consistency check
      if ((world_coordinates - initial_coordinates_in_world).squaredNorm() <
          BaseType::param_maximum_distance_geometry_meters_squared.value()) {
        BaseType::_landmark->coordinates() = BaseType::_scene_from_world * world_coordinates;
        BaseType::_landmark->statistics().setState(world_coordinates);
        BaseType::_landmark->statistics().setIsInlier(true);
        BaseType::_landmark->statistics().setNumberOfOptimizations(measurements.size());
      }
      return;
    }

    // ds optimize using gauss newton descent TODO refactor with projective for proper optimization
    _total_error_squared_previous = 0;
    size_t number_of_inliers      = 0;
    for (size_t i = 0; i < param_maximum_number_of_iterations.value(); ++i) {
      _H.setZero();
      _b.setZero();
      float total_error_squared = 0;
      size_t number_of_outliers = 0;

      // ds for each measurement
      for (const PointStatisticsField3D::CameraMeasurement& measurement : measurements) {
        _omega.setIdentity();
        _omega(2, 2) *= 10.0f; // ds compensate depth to pixel scale

        // ds sample current state in measurement context
        const Vector3f camera_coordinates = measurement.camera_from_world * world_coordinates;
        if (camera_coordinates.z() <= 0) {
          ++number_of_outliers;
          continue;
        }
        const Vector3f point_homogeneous = _camera_matrix * camera_coordinates;
        const float& c                   = point_homogeneous.z();
        const float inverse_c            = 1.0f / c;
        const float inverse_c_squared    = inverse_c * inverse_c;
        const Vector3f point_in_image    = point_homogeneous / c;

        // ds compute error
        const Vector3f error(point_in_image(0) - measurement.point_in_image(0),
                             point_in_image(1) - measurement.point_in_image(1),
                             c - measurement.point_in_camera(2));

        // ds update chi
        const float error_squared = error.transpose() * _omega * error;
        total_error_squared += error_squared;

        // ds saturated kernel
        if (error_squared > maximum_kernel_error) {
          _omega *= maximum_kernel_error / error_squared;
          ++number_of_outliers;
        }

        // ds get the jacobian of the  transform part: R TODO store in measurement
        const Matrix3f jacobian_linear = _camera_matrix * measurement.camera_from_world.linear();

        // ds get the jacobian of the homogeneous division
        Matrix3f jacobian_homogeneous;
        jacobian_homogeneous << inverse_c, 0, -point_homogeneous.x() * inverse_c_squared, 0,
          inverse_c, -point_homogeneous.y() * inverse_c_squared, 0, 0, 1;
        const Matrix3f jacobian            = jacobian_homogeneous * jacobian_linear;
        const Matrix3f jacobian_transposed = jacobian.transpose();

        // ds accumulate - jacobian is symmetric! (R^T == R)
        _H += jacobian_transposed * _omega * jacobian;
        _b += jacobian_transposed * _omega * error;
      }

      // ds update state
      world_coordinates += _H.fullPivLu().solve(-_b);
      assert(!world_coordinates.hasNaN());
      number_of_inliers = measurements.size() - number_of_outliers;

      // ds check convergence
      if (std::fabs(total_error_squared - _total_error_squared_previous) <
          param_convergence_criterion_minimum_chi2_delta.value()) {
        // ds terminate prematurely
        break;
      }
      _total_error_squared_previous = total_error_squared;
    }

    // ds if the number of inliers is higher than the best so far
    if (number_of_inliers > BaseType::_landmark->statistics().numberOfOptimizations()) {
      // ds update global landmark position and covariance
      BaseType::_landmark->statistics().addOptimizationResult(world_coordinates);
      BaseType::_landmark->statistics().setIsInlier(true);
      assert(BaseType::_landmark->statistics().numberOfOptimizations() <= measurements.size());
    }

    // ds if optimization failed and we have less inliers than outliers - reset guess
    else {
      // ds reset estimate based on overall mean coordinates (brutal)
      _setMeanCoordinatesInWorld(measurements, world_coordinates);
      BaseType::_landmark->statistics().setState(world_coordinates);
      assert(BaseType::_landmark->statistics().numberOfOptimizations() < measurements.size());
    }

    // ds update local landmark position
    BaseType::_landmark->coordinates() = BaseType::_scene_from_world * world_coordinates;
  }

  template <typename MeasurementVectorType_, typename LandmarkType_>
  void LandmarkEstimatorPoseBasedSmoother_<MeasurementVectorType_, LandmarkType_>::
    _setMeanCoordinatesInWorld(const PointStatisticsField3D::CameraMeasurementVector& measurements_,
                               Vector3f& coordinates_in_world_) const {
    Vector3f world_coordinates_accumulated(Vector3f::Zero());
    for (const PointStatisticsField3D::CameraMeasurement& measurement : measurements_) {
      world_coordinates_accumulated += measurement.world_from_camera * measurement.point_in_camera;
    }
    coordinates_in_world_ = world_coordinates_accumulated / measurements_.size();
  }

} // namespace srrg2_slam_interfaces
