#include "fixtures.hpp"
#include "srrg2_proslam_mapping/landmarks/filters/projective_depth_point_ekf.h"

// ds sampling configuration for synthetic tests
constexpr size_t number_of_runs        = 100;
constexpr size_t number_of_transitions = 100;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(SyntheticDouble, ProjectiveDepthPointEKF_Transforms_FullNoise) {
  generateContinousTransitions(
    TransitionSampleType::Transform, number_of_transitions, 0.0, 0.01, 1.0);

  // ds initialize filter with perfect initial guess
  ProjectiveDepthPointEKF<3, 3, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);

  // ds accumulate statistics over multiple runs from an initial guess
  StatisticsDouble statistics;
  for (size_t i = 0; i < number_of_runs; ++i) {
    // ds set state and covariance
    Vector3d state(point_positions_ground_truth[0]);
    filter.setState(&state);
    Matrix3d covariance(Matrix3d::Identity());
    filter.setCovariance(&covariance);

    // ds process all samples
    for (size_t j = 0; j < number_of_transitions; ++j) {
      Matrix3d transition_covariance(Matrix3d::Identity());
      transition_covariance *= 0.1 /*generous noise estimate*/;
      Matrix3d measurement_covariance(Matrix3d::Identity());
      measurement_covariance *= 10.0 /*generous noise estimate*/;
      filter.setTransition(camera_transitions_noisy[j], transition_covariance);
      filter.setMeasurement(point_projections_depth_noisy[j + 1], measurement_covariance);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 10.0);
      ASSERT_LT(covariance.norm(), 100.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}
