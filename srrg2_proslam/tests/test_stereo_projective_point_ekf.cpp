#include "fixtures.hpp"
#include "srrg2_proslam/mapping/landmarks/filters/projective_depth_point_ekf.h"
#include "srrg2_proslam/mapping/landmarks/filters/stereo_projective_point_ekf.h"

#include "srrg2_proslam/mapping/instances.cpp"

// ds sampling configuration for synthetic tests
constexpr size_t number_of_runs        = 100;
constexpr size_t number_of_transitions = 100;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(SyntheticDouble, StereoProjectivePointEKF_Translations_ZeroNoise) {
  generateContinousTransitions(TransitionSampleType::Translation, number_of_transitions);

  // ds initialize filter with perfect initial guess
  StereoProjectivePointEKF<3, 4, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);
  filter.setBaseline(stereo_camera_baseline);

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
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(stereo_point_projections_noisy[j + 1]);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));
      ASSERT_LT(error_meters.norm(), std::numeric_limits<double>::min());
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, StereoProjectivePointEKF_Rotations_ZeroNoise) {
  generateContinousTransitions(TransitionSampleType::Rotation, number_of_transitions);

  // ds initialize filter with perfect initial guess
  StereoProjectivePointEKF<3, 4, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);
  filter.setBaseline(stereo_camera_baseline);

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
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(stereo_point_projections_noisy[j + 1]);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));
      ASSERT_LT(error_meters.norm(), std::numeric_limits<double>::min());
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, StereoProjectivePointEKF_Transforms_ZeroNoise) {
  generateContinousTransitions(TransitionSampleType::Transform, number_of_transitions);

  // ds initialize filter with perfect initial guess
  StereoProjectivePointEKF<3, 4, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);
  filter.setBaseline(stereo_camera_baseline);

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
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(stereo_point_projections_noisy[j + 1]);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));
      ASSERT_LT(error_meters.norm(), std::numeric_limits<double>::min());
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, StereoProjectivePointEKF_Transforms_FullNoise) {
  generateContinousTransitions(
    TransitionSampleType::Transform, number_of_transitions, 0.0, 0.01, 1.0);

  // ds initialize filters
  ProjectivePointEKF<3, 2, double> filter_mono;
  filter_mono.setCameraMatrix(camera_projection_matrix);
  ProjectiveDepthPointEKF<3, 3, double> filter_mono_depth;
  filter_mono_depth.setCameraMatrix(camera_projection_matrix);
  StereoProjectivePointEKF<3, 4, double> filter_bino;
  filter_bino.setCameraMatrix(camera_projection_matrix);
  filter_bino.setBaseline(stereo_camera_baseline);

  // ds accumulate statistics over multiple runs from an initial guess
  StatisticsDouble statistics_mono;
  StatisticsDouble statistics_mono_depth;
  StatisticsDouble statistics_bino;
  for (size_t i = 0; i < number_of_runs; ++i) {
    // ds set state and covariance handles to the filters
    Vector3d state_mono(point_positions_ground_truth[0]);
    filter_mono.setState(&state_mono);
    Matrix3d covariance_mono(Matrix3d::Identity());
    filter_mono.setCovariance(&covariance_mono);
    Vector3d state_mono_depth(point_positions_ground_truth[0]);
    filter_mono_depth.setState(&state_mono_depth);
    Matrix3d covariance_mono_depth(Matrix3d::Identity());
    filter_mono_depth.setCovariance(&covariance_mono_depth);
    Vector3d state_bino(point_positions_ground_truth[0]);
    filter_bino.setState(&state_bino);
    Matrix3d covariance_bino(Matrix3d::Identity());
    filter_bino.setCovariance(&covariance_bino);

    // ds process all samples
    for (size_t j = 0; j < number_of_transitions; ++j) {
      Matrix3d transition_covariance(Matrix3d::Identity());
      transition_covariance *= 0.1 /*generous noise estimate*/;
      Matrix2d measurement_covariance_mono(Matrix2d::Identity());
      measurement_covariance_mono *= 10.0 /*generous noise estimate*/;
      Matrix3d measurement_covariance_mono_depth(Matrix3d::Identity());
      measurement_covariance_mono_depth *= 10.0 /*generous noise estimate*/;
      Matrix4d measurement_covariance_bino(Matrix4d::Identity());
      measurement_covariance_bino *= 10.0 /*generous noise estimate*/;

      // ds propagate monocular filter
      filter_mono.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter_mono.setMeasurement(point_projections_noisy[j + 1], measurement_covariance_mono);
      filter_mono.compute();

      // ds propagate monocular filter with depth
      filter_mono_depth.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter_mono_depth.setMeasurement(point_projections_depth_noisy[j + 1],
                                       measurement_covariance_mono_depth);
      filter_mono_depth.compute();

      // ds propagate binocular filter
      filter_bino.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter_bino.setMeasurement(stereo_point_projections_noisy[j + 1],
                                 measurement_covariance_bino);
      filter_bino.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_mono_meters((state_mono - point_positions_ground_truth[j + 1]));
      const Vector3d error_mono_depth_meters(
        (state_mono_depth - point_positions_ground_truth[j + 1]));
      const Vector3d error_bino_meters((state_bino - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_mono_meters.norm(), 1.0);
      statistics_mono.addEntry(error_mono_meters, covariance_mono);
      ASSERT_LT(error_mono_depth_meters.norm(), 1.0);
      statistics_mono_depth.addEntry(error_mono_depth_meters, covariance_mono_depth);
      ASSERT_LT(error_bino_meters.norm(), 1.0);
      statistics_bino.addEntry(error_bino_meters, covariance_bino);
    }
  }
  statistics_mono.print("monocular");
  statistics_mono_depth.print("monodepth");
  statistics_bino.print("binocular");
}
