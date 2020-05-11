#include "fixtures.hpp"
#include "srrg2_proslam/mapping/landmarks/filters/projective_point_ekf.h"

#include "srrg2_proslam/mapping/instances.cpp"

// ds sampling configuration for synthetic tests
constexpr size_t number_of_runs        = 10;
constexpr size_t number_of_transitions = 100;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Translations_ZeroNoise) {
  generateContinousTransitions(TransitionSampleType::Translation, number_of_transitions);

  // ds initialize filter
  ProjectivePointEKF<3, 2, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);

  // ds accumulate statistics over multiple runs from an initial guess
  StatisticsDouble statistics;
  for (size_t i = 0; i < number_of_runs; ++i) {
    // ds set state and covariance
    Vector3d state(point_positions_ground_truth[0]);
    filter.setState(&state);
    Matrix3d covariance(Matrix3d::Zero());
    filter.setCovariance(&covariance);

    // ds process all samples
    for (size_t j = 0; j < number_of_transitions; ++j) {
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(point_projections_noisy[j + 1], Matrix2d::Identity());
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), std::numeric_limits<double>::min());
      ASSERT_LT(covariance.norm(), 1.1);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Translations_TransitionNoise) {
  generateContinousTransitions(TransitionSampleType::Translation, number_of_transitions, 0.0, 0.01);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      transition_covariance *= 0.1 /*overestimating noise*/;
      filter.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter.setMeasurement(point_projections_noisy[j + 1], Matrix2d::Identity());
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 1.0);
      ASSERT_LT(covariance.norm(), 10.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Translations_MeasurementNoise) {
  generateContinousTransitions(
    TransitionSampleType::Translation, number_of_transitions, 0.0, 0.0, 1.0);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      Matrix2d measurement_covariance(Matrix2d::Identity());
      measurement_covariance *= 10.0 /*generous noise estimate*/;
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(point_projections_noisy[j + 1], measurement_covariance);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 1.0);
      ASSERT_LT(covariance.norm(), 10.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Translations_FullNoise) {
  generateContinousTransitions(
    TransitionSampleType::Translation, number_of_transitions, 0.0, 0.01, 1.0);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      Matrix2d measurement_covariance(Matrix2d::Identity());
      measurement_covariance *= 10.0 /*generous noise estimate*/;
      filter.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter.setMeasurement(point_projections_noisy[j + 1], measurement_covariance);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 1.0);
      ASSERT_LT(covariance.norm(), 10.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Rotations_ZeroNoise) {
  generateContinousTransitions(TransitionSampleType::Rotation, number_of_transitions);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);

  // ds accumulate statistics over multiple runs from an initial guess
  StatisticsDouble statistics;
  for (size_t i = 0; i < number_of_runs; ++i) {
    // ds set state and covariance
    Vector3d state(point_positions_ground_truth[0]);
    filter.setState(&state);
    Matrix3d covariance(Matrix3d::Zero());
    filter.setCovariance(&covariance);

    // ds process all samples
    for (size_t j = 0; j < number_of_transitions; ++j) {
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(point_projections_noisy[j + 1], Matrix2d::Identity());
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), std::numeric_limits<double>::min());
      ASSERT_LT(covariance.norm(), 1.1);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Rotations_TransitionNoise) {
  generateContinousTransitions(TransitionSampleType::Rotation, number_of_transitions, 0.0, 0.01);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      filter.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter.setMeasurement(point_projections_noisy[j + 1], Matrix2d::Identity());
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 1.0);
      ASSERT_LT(covariance.norm(), 10.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Rotations_MeasurementNoise) {
  generateContinousTransitions(
    TransitionSampleType::Rotation, number_of_transitions, 0.0, 0.0, 1.0);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      Matrix2d measurement_covariance(Matrix2d::Identity());
      measurement_covariance *= 10.0 /*generous noise estimate*/;
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(point_projections_noisy[j + 1], measurement_covariance);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 1.0);
      ASSERT_LT(covariance.norm(), 10.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Rotations_FullNoise) {
  generateContinousTransitions(
    TransitionSampleType::Rotation, number_of_transitions, 0.0, 0.01, 0.1);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      Matrix2d measurement_covariance(Matrix2d::Identity());
      measurement_covariance *= 10.0 /*generous noise estimate*/;
      filter.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter.setMeasurement(point_projections_noisy[j + 1], measurement_covariance);
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), 1.0);
      ASSERT_LT(covariance.norm(), 100.0);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Transforms_ZeroNoise) {
  generateContinousTransitions(TransitionSampleType::Transform, number_of_transitions);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
  filter.setCameraMatrix(camera_projection_matrix);

  // ds accumulate statistics over multiple runs from an initial guess
  StatisticsDouble statistics;
  for (size_t i = 0; i < number_of_runs; ++i) {
    // ds set state and covariance
    Vector3d state(point_positions_ground_truth[0]);
    filter.setState(&state);
    Matrix3d covariance(Matrix3d::Zero());
    filter.setCovariance(&covariance);

    // ds process all samples
    for (size_t j = 0; j < number_of_transitions; ++j) {
      filter.setWorldInSensor(camera_transitions_noisy[j]);
      filter.setMeasurement(point_projections_noisy[j + 1], Matrix2d::Identity());
      filter.compute();

      // ds compute state estimation error and covariance
      const Vector3d error_meters((state - point_positions_ground_truth[j + 1]));

      // ds check if they remain within test boundaries
      ASSERT_LT(error_meters.norm(), std::numeric_limits<double>::min());
      ASSERT_LT(covariance.norm(), 1.1);
      statistics.addEntry(error_meters, covariance);
    }
  }
  statistics.print();
}

TEST_F(SyntheticDouble, ProjectivePointEKF_Transforms_FullNoise) {
  generateContinousTransitions(
    TransitionSampleType::Transform, number_of_transitions, 0.0, 0.01, 1.0);

  // ds initialize filter with perfect initial guess
  ProjectivePointEKF<3, 2, double> filter;
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
      Matrix2d measurement_covariance(Matrix2d::Identity());
      measurement_covariance *= 10.0 /*generous noise estimate*/;
      filter.setWorldInSensor(camera_transitions_noisy[j], transition_covariance);
      filter.setMeasurement(point_projections_noisy[j + 1], measurement_covariance);
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
