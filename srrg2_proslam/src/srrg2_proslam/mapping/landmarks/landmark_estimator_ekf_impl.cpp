#include "landmark_estimator_ekf.h"

namespace srrg2_proslam {
  using namespace srrg2_core;

  template <typename MeasurementVectorType_, typename LandmarkType_>
  void LandmarkEstimatorEKF_<MeasurementVectorType_, LandmarkType_>::setTransforms(
    const EstimateType& measurement_in_world_,
    const EstimateType& measurement_in_scene_) {
    BaseType::setTransforms(measurement_in_world_, measurement_in_scene_);
    assert(param_filter.value());
    param_filter->setWorldInSensor(
      BaseType::_world_in_sensor.template cast<double>() /*, TODO include proper covariance*/);
  }

  template <typename MeasurementVectorType_, typename LandmarkType_>
  void LandmarkEstimatorEKF_<MeasurementVectorType_, LandmarkType_>::compute() {
    PROFILE_TIME("LandmarkEstimatorEKF::compute");
    assert(BaseType::_landmark);
    assert(BaseType::_landmark->statistics().value);
    assert(param_filter.value());
    assert(BaseType::_landmark->status == POINT_STATUS::Valid);
    BaseType::_landmark->statistics().setIsInlier(false);

    // ds retrieve measurement covariance TODO find a clean way to channel through
    MeasurementMatrixType measurement_covariance(MeasurementMatrixType::Identity());
    measurement_covariance *= param_minimum_state_element_covariance.value();

    // ds evaluate disparity
    //    const float horizontal_disparity = measurement(0) - measurement(2);
    //    const float vertical_disparity   = measurement(1) - measurement(3);
    //    Matrix4d measurement_covariance  = Matrix4d::Identity();
    //    measurement_covariance *= 10.0 / (1.0 + horizontal_disparity + vertical_disparity);

    // ds TODO move this visualization only crap
    BaseType::_landmark->statistics().setProjection(BaseType::_measurement.head(2));

    // ds set state (landmark position) and covariance
    const Vector_<float, StateDim> initial_coordinates_in_world(
      BaseType::_landmark->statistics().state());
    StateVectorType state(initial_coordinates_in_world.template cast<double>());
    StateMatrixType covariance(
      BaseType::_landmark->statistics().covariance().template cast<double>());

    // ds enforce minimum covariance on diagonal before filtering
    for (size_t i = 0; i < StateDim; ++i) {
      covariance(i, i) = std::max(covariance(i, i), param_minimum_state_element_covariance.value());
    }

    // ds filter estimate
    param_filter->setState(&state);
    param_filter->setCovariance(&covariance);
    param_filter->setMeasurement(BaseType::_measurement.template cast<double>(),
                                 measurement_covariance);
    param_filter->compute();

    // ds drop invalid state optimizations
    if (state.z() <= 0 ||
        covariance.squaredNorm() > param_maximum_covariance_norm_squared.value()) {
      // ds skip update
      return;
    }

    // ds compute point in scene and perform geometric distance change check
    // ds the filter has transitioned the point into the current tracker coordinate frame
    const Vector_<float, StateDim> coordinates_in_world(BaseType::_sensor_in_world *
                                                        state.template cast<float>());
    if ((coordinates_in_world - initial_coordinates_in_world).squaredNorm() >
        BaseType::param_maximum_distance_geometry_meters_squared.value()) {
      // ds skip update
      return;
    }

    // ds update global landmark position and covariance
    BaseType::_landmark->statistics().addOptimizationResult(coordinates_in_world,
                                                            covariance.template cast<float>());
    BaseType::_landmark->statistics().setIsInlier(true);

    // ds update local landmark position
    BaseType::_landmark->coordinates() =
      BaseType::_world_in_local_map * coordinates_in_world.template cast<float>();
  }

} // namespace srrg2_proslam
