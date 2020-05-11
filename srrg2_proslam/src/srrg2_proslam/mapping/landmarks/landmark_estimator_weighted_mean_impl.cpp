#include "landmark_estimator_weighted_mean.h"

namespace srrg2_proslam {
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
    const LandmarkCoordinatesType& coordinates_in_world_update =
      BaseType::_sensor_in_world * BaseType::_landmark_in_sensor;

    const float num_optimiz_plus_1 = BaseType::_landmark->statistics().numberOfOptimizations() + 1;

    const LandmarkCoordinatesType coordinates_in_world =
      (num_optimiz_plus_1 * initial_coordinates_in_world + coordinates_in_world_update) /
      (num_optimiz_plus_1 + 1);

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
    BaseType::_landmark->coordinates() = BaseType::_world_in_local_map * coordinates_in_world;
  }

} // namespace srrg2_proslam
