#include "landmark_estimator_weighted_mean.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename MeasurementVectorType_, typename LandmarkType_>
  void LandmarkEstimatorWeightedMean_<MeasurementVectorType_, LandmarkType_>::compute() {
    PROFILE_TIME("LandmarkEstimatorWeightedMean::compute");
    assert(BaseType::_landmark);
    assert(BaseType::_landmark->statistics().value);
    assert(BaseType::_landmark->status == POINT_STATUS::Valid);
    BaseType::_landmark->statistics().setIsInlier(false);

    // ds compute landmark coordinate and weight update based on set estimate
    // ds TODO integrate covariance for individual weighting along each axis
    // ds note that the raw measurement is not used here
    const LandmarkCoordinatesType& initial_coordinates_in_world =
      BaseType::_landmark->statistics().state();
    const LandmarkCoordinatesType& coordinates_update =
      BaseType::_world_from_tracker * BaseType::_landmark_estimate_in_tracker;

    const LandmarkCoordinatesType coordinates_in_world =
      ((BaseType::_landmark->statistics().numberOfOptimizations() + 1) *
         initial_coordinates_in_world +
       coordinates_update) /
      (2 + BaseType::_landmark->statistics().numberOfOptimizations());

    // ds perform geometric distance change check
    if ((coordinates_in_world - initial_coordinates_in_world).squaredNorm() >
        BaseType::param_maximum_distance_geometry_meters_squared.value()) {
      // ds skip update
      return;
    }

    // ds update global landmark position and covariance
    BaseType::_landmark->statistics().addOptimizationResult(coordinates_in_world);
    BaseType::_landmark->statistics().setIsInlier(true);

    // ds update local landmark position
    BaseType::_landmark->coordinates() = BaseType::_scene_from_world * coordinates_in_world;
  }

} // namespace srrg2_slam_interfaces
