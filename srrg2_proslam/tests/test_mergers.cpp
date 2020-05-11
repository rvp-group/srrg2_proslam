#include "fixtures.hpp"
#include "srrg2_proslam/mapping/landmarks/landmark_estimator_ekf.h"
#include "srrg2_proslam/mapping/landmarks/landmark_estimator_pose_based_smoother.h"
#include "srrg2_proslam/mapping/landmarks/landmark_estimator_weighted_mean.h"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(SyntheticFloat, MergerCorrespondence3D_FullIntersection) {
  MergerCorrespondencePointIntensityDescriptor3f merger;

  //  merger.param_unprojector->setCameraPose(camera_current_in_world);
  merger.setMeasurementInScene(camera_current_in_world);
  merger.setScene(&points_in_world);
  merger.setMeasurement(&points_in_camera);
  merger.setCorrespondences(&correspondences);
  merger.compute();

  // ds no new points must be added
  ASSERT_EQ(points_in_world.size(), 100);
  ASSERT_EQ(points_in_world_copy.size(), 100);

  // ds point must not have moved
  for (size_t i = 0; i < 100; ++i) {
    ASSERT_LT((points_in_world[i].coordinates() - points_in_world_copy[i].coordinates()).norm(),
              1e-3);
  }
}

TEST_F(ICL, MergerCorrespondence3DSparse00To00) {
  MergerCorrespondencePointIntensityDescriptor3f merger;
  merger.param_maximum_response.setValue(50);
  merger.param_maximum_distance_geometry_squared.setValue(0.25);

  // ds set buffers to processor (no motion and identical measurements)
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  ASSERT_EQ(points_in_camera_00.size(), 321);
  merger.setMeasurement(&points_in_camera_00);
  ASSERT_EQ(points_in_camera_00.size(), 321);
  merger.setCorrespondences(&correspondences_camera_00_from_00);
  ASSERT_EQ(correspondences_camera_00_from_00.size(), 321);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(ICL, MergerCorrespondence3D_Sparse00To01) {
  MergerCorrespondencePointIntensityDescriptor3f merger;
  merger.param_maximum_response.setValue(50);
  merger.param_maximum_distance_geometry_squared.setValue(0.25);
  merger.param_target_number_of_merges.setValue(1000);

  // ds set buffers to processor
  merger.setMeasurementInScene(camera_01_in_00);
  merger.setScene(&points_in_camera_00);
  ASSERT_EQ(points_in_camera_00.size(), 321);
  merger.setMeasurement(&points_in_camera_01);
  ASSERT_EQ(points_in_camera_01.size(), 338);
  merger.setCorrespondences(&correspondences_camera_01_from_00);
  ASSERT_EQ(correspondences_camera_00_from_00.size(), 321);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_01.size(), 338);

  // ds the scene points vary as new points will be added
  ASSERT_EQ(points_in_camera_00.size(), 431);

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(),
            points_in_camera_00_backup.size() + points_in_camera_01.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 0.2);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 0.2);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 0.2);
  }
}

TEST_F(KITTI, MergerCorrespondence3D_00To00) {
  MergerCorrespondencePointIntensityDescriptor3f merger;
  merger.param_maximum_response.setValue(50);
  merger.param_maximum_distance_geometry_squared.setValue(25);

  // ds set buffers to processor (no motion and identical measurements)
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&points_in_camera_00);
  merger.setCorrespondences(&correspondences_camera_00_from_00);

  // ds backup points for assertions
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(KITTI, MergerCorrespondencePointIntensityDescriptor3f_00To01) {
  MergerCorrespondencePointIntensityDescriptor3f merger;
  merger.param_maximum_response.setValue(50);
  merger.param_maximum_distance_geometry_squared.setValue(25);

  // ds set buffers to processor
  merger.setMeasurementInScene(camera_01_in_00);
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&points_in_camera_01 /* passing triangulated measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_01);

  // ds backup points for assertions
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);
  const size_t number_of_measurements = points_in_camera_01.size();

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_01.size(), number_of_measurements);

  // ds the scene points vary as new points will be added
  ASSERT_GT(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(),
            points_in_camera_00_backup.size() + points_in_camera_01.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 2);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 2);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 2);
  }
}

TEST_F(ICL, MergerCorrespondenceProjectiveDepth3D_Sparse_00To00) {
  MergerCorrespondenceProjectiveDepth3D merger;
  merger.param_maximum_response.setValue(50);
  merger.param_maximum_distance_geometry_squared.setValue(0.25);
  merger.param_unprojector->setCameraMatrix(camera_calibration_matrix);
  merger.param_target_number_of_merges.setValue(1000);

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements_projective_depth_00 /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_00);

  // ds backup points for assertions
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(ICL, MergerCorrespondenceProjectiveDepth3D_Sparse_00To01) {
  MergerCorrespondenceProjectiveDepth3D merger;
  merger.param_maximum_response.setValue(50);
  merger.param_maximum_distance_geometry_squared.setValue(0.25);
  merger.param_unprojector->param_canvas_rows.setValue(image_rows);
  merger.param_unprojector->param_canvas_cols.setValue(image_cols);
  merger.param_unprojector->setCameraMatrix(camera_calibration_matrix);
  merger.param_target_number_of_merges.setValue(1000);

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements_projective_depth_01 /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_01_from_00);

  // ds backup points for assertions
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(measurements_projective_depth_01.size(), 338);

  // ds the scene points vary as new points will be added
  ASSERT_EQ(points_in_camera_00.size(), 431);

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(),
            points_in_camera_00_backup.size() + measurements_projective_depth_01.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 0.25);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 0.25);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 0.25);
  }
}

TEST_F(ICL, 00To00_MergerCorrespondenceProjectiveDepthEKF_Sparse) {
  MergerProjectiveDepthEKF merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_unprojector->param_canvas_rows.setValue(image_rows);
  merger.param_unprojector->param_canvas_cols.setValue(image_cols);
  merger.param_unprojector->setCameraMatrix(camera_calibration_matrix);
  merger.param_projector->param_canvas_rows.setValue(image_rows);
  merger.param_projector->param_canvas_cols.setValue(image_cols);
  merger.param_projector->setCameraMatrix(camera_calibration_matrix);
  merger.param_target_number_of_merges.setValue(1000);

  // ds configure landmark estimator
  LandmarkEstimatorProjectiveDepthEKF3DPtr landmark_estimator(
    new LandmarkEstimatorProjectiveDepthEKF3D());
  landmark_estimator->param_filter.setValue(
    ProjectiveDepthPointEKF3DPtr(new ProjectiveDepthPointEKF3D()));
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setMeasurementInWorld(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements_projective_depth_00 /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_00);

  // ds backup points for assertions
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(ICL, 00To01_MergerCorrespondenceProjectiveDepthEKF_Sparse) {
  MergerProjectiveDepthEKF merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_unprojector->param_canvas_rows.setValue(image_rows);
  merger.param_unprojector->param_canvas_cols.setValue(image_cols);
  merger.param_unprojector->setCameraMatrix(camera_calibration_matrix);
  merger.param_projector->param_canvas_rows.setValue(image_rows);
  merger.param_projector->param_canvas_cols.setValue(image_cols);
  merger.param_projector->setCameraMatrix(camera_calibration_matrix);
  merger.param_target_number_of_merges.setValue(1000);

  // ds configure landmark estimator
  LandmarkEstimatorProjectiveDepthEKF3DPtr landmark_estimator(
    new LandmarkEstimatorProjectiveDepthEKF3D());
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(0.01);
  landmark_estimator->param_filter.setValue(
    ProjectiveDepthPointEKF3DPtr(new ProjectiveDepthPointEKF3D()));
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setMeasurementInWorld(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements_projective_depth_01 /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_01_from_00);

  // ds backup points for assertions
  PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(measurements_projective_depth_01.size(), 338);

  // ds the scene points vary as new points will be added
  ASSERT_EQ(points_in_camera_00.size(), 337);

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(),
            points_in_camera_00_backup.size() + measurements_projective_depth_01.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 0.1);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 0.1);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 0.1);
  }
}

TEST_F(KITTI, 00To00_MergerTriangulation_WeightedMean) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoTriangulation merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorWeightedMean4D3DPtr landmark_estimator(new LandmarkEstimatorWeightedMean4D3D());
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setMeasurementInWorld(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements[0] /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_00);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(KITTI, 00To00_MergerTriangulation_PoseBasedSmoother) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoTriangulation merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorPoseBasedSmoother4D3DPtr landmark_estimator(
    new LandmarkEstimatorPoseBasedSmoother4D3D());
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  merger.param_landmark_estimator.setValue(landmark_estimator);

  ASSERT_EQ(points_00_in_image_00.size(), points_in_camera_00.size());
  for (size_t i = 0; i < points_in_camera_00.size(); ++i) {
    PointIntensityDescriptor3f& point = points_in_camera_00[i];
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
    point.statistics().addMeasurement(
      srrg2_core::PointStatisticsField3D::CameraMeasurement(points_00_in_image_00[i].coordinates(),
                                                            point.coordinates(),
                                                            Isometry3f::Identity(),
                                                            Isometry3f::Identity()));
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setMeasurementInWorld(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements[0] /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_00);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(KITTI, 00To01_MergerTriangulation_WeightedMean) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoTriangulation merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorWeightedMean4D3DPtr landmark_estimator(new LandmarkEstimatorWeightedMean4D3D());
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(camera_01_in_00);
  merger.setMeasurementInWorld(camera_01_in_00);
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements[1] /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_01);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);
  const size_t number_of_measurements = measurements[1].size();

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(measurements[1].size(), number_of_measurements);

  // ds the scene points vary as new points will be added
  ASSERT_GT(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(), points_in_camera_00_backup.size() + measurements[1].size());

  // ds TODO check merge result against ground truth - currently we only detect divergence
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 5);
  }
}

TEST_F(KITTI, 00To01_MergerTriangulation_PoseBasedSmoother) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoTriangulation merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorPoseBasedSmoother4D3DPtr landmark_estimator(
    new LandmarkEstimatorPoseBasedSmoother4D3D());
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  ASSERT_EQ(points_00_in_image_00.size(), points_in_camera_00.size());
  for (size_t i = 0; i < points_in_camera_00.size(); ++i) {
    PointIntensityDescriptor3f& point = points_in_camera_00[i];
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
    point.statistics().addMeasurement(
      srrg2_core::PointStatisticsField3D::CameraMeasurement(points_00_in_image_00[i].coordinates(),
                                                            point.coordinates(),
                                                            Isometry3f::Identity(),
                                                            Isometry3f::Identity()));
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(camera_01_in_00);
  merger.setMeasurementInWorld(camera_01_in_00);
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements[1] /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_01);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);
  const size_t number_of_measurements = measurements[1].size();

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(measurements[1].size(), number_of_measurements);

  // ds the scene points vary as new points will be added
  ASSERT_GT(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(), points_in_camera_00_backup.size() + measurements[1].size());

  // ds TODO check merge result against ground truth - currently we only detect divergence
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 5);
  }
}

TEST_F(KITTI, 00To01_MergerTriangulation_Pruned_WeightedMean) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoTriangulation merger;
  merger.param_maximum_distance_appearance.setValue(50);
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorWeightedMean4D3DPtr landmark_estimator(new LandmarkEstimatorWeightedMean4D3D());
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  merger.param_landmark_estimator.setValue(landmark_estimator);

  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds compute points in camera 01
  PointIntensityDescriptor4fVectorCloud points_measured_in_camera_01;
  points_measured_in_camera_01.reserve(points_in_camera_00.size());
  CorrespondenceVector correspondences;
  correspondences.reserve(points_in_camera_00.size());
  Isometry3f camera_00_in_01 = camera_01_in_00.inverse();
  for (size_t i = 0; i < points_in_camera_00.size(); ++i) {
    PointIntensityDescriptor4f point;
    const Vector3f point_in_camera_01 = camera_00_in_01 * points_in_camera_00[i].coordinates();
    if (point_in_camera_01.z() <= 0) {
      continue;
    }

    Vector3f point_in_image_left  = camera_calibration_matrix * point_in_camera_01;
    Vector3f point_in_image_right = point_in_image_left - baseline_right_in_left_pixels;
    point_in_image_left /= point_in_image_left.z();
    point_in_image_right /= point_in_image_right.z();
    ASSERT_EQ(point_in_image_left(1), point_in_image_right(1));
    ASSERT_GE(point_in_image_left(0), point_in_image_right(0));
    if (point_in_image_left(0) - point_in_image_right(0) <
        merger.param_triangulator->param_minimum_disparity_pixels.value()) {
      continue;
    }
    if (point_in_image_left(0) < 0 || point_in_image_left(1) < 0 ||
        point_in_image_left(0) >= image_cols || point_in_image_left(1) >= image_rows) {
      continue;
    }

    point.coordinates()(0) = point_in_image_left(0);
    point.coordinates()(1) = point_in_image_left(1);
    point.coordinates()(2) = point_in_image_right(0);
    point.coordinates()(3) = point_in_image_right(1);
    point.descriptor()     = points_in_camera_00[i].descriptor();
    correspondences.emplace_back(Correspondence(i, points_measured_in_camera_01.size(), 0));
    points_measured_in_camera_01.emplace_back(point);
  }
  ASSERT_EQ(points_measured_in_camera_01.size(), 145);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(camera_01_in_00);
  merger.setMeasurementInWorld(camera_01_in_00);
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&points_measured_in_camera_01);
  merger.setCorrespondences(&correspondences);
  const size_t number_of_measurements = points_measured_in_camera_01.size();

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_measured_in_camera_01.size(), number_of_measurements);

  // ds full merge success no points added!
  ASSERT_EQ(points_in_camera_00.size(), number_of_measurements);

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    ASSERT_LT(
      (points_in_camera_00[i].coordinates() - points_in_camera_00_backup[i].coordinates()).norm(),
      0.01);
  }
}

TEST_F(KITTI, 00To00_MergerStereoProjectiveEKF) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoProjectiveEKF merger;
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorStereoProjectiveEKF3DPtr landmark_estimator(
    new LandmarkEstimatorStereoProjectiveEKF3D());
  landmark_estimator->param_maximum_covariance_norm_squared.setValue(1);
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(1);
  landmark_estimator->param_minimum_state_element_covariance.setValue(0.01);
  landmark_estimator->param_filter.setValue(
    StereoProjectivePointEKF3DPtr(new StereoProjectivePointEKF3D()));
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(Isometry3f::Identity());
  merger.setMeasurementInWorld(Isometry3f::Identity());
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements[0] /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_00);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds check merge result against backup scene - element order must remain intact
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 1e-5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 1e-5);
  }
}

TEST_F(KITTI, 00To01_MergerStereoProjectiveEKF) {
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    new PointIntensityDescriptor3fProjectorPinhole());
  projector->setCameraMatrix(camera_calibration_matrix);
  projector->param_canvas_rows.setValue(image_rows);
  projector->param_canvas_cols.setValue(image_cols);

  MergerRigidStereoProjectiveEKF merger;
  merger.param_projector.setValue(projector);
  merger.param_triangulator->param_projector.setValue(projector);
  merger.param_triangulator->setPlatform(_platform);

  // ds configure landmark estimator
  LandmarkEstimatorStereoProjectiveEKF3DPtr landmark_estimator(
    new LandmarkEstimatorStereoProjectiveEKF3D());
  landmark_estimator->param_maximum_covariance_norm_squared.setValue(100);
  landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(100);
  landmark_estimator->param_minimum_state_element_covariance.setValue(0.01);
  landmark_estimator->param_filter.setValue(
    StereoProjectivePointEKF3DPtr(new StereoProjectivePointEKF3D()));
  merger.param_landmark_estimator.setValue(landmark_estimator);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
    point.statistics().setState(point.coordinates());
  }

  // ds set buffers to processor
  merger.setMeasurementInScene(camera_01_in_00);
  merger.setMeasurementInWorld(camera_01_in_00);
  merger.setScene(&points_in_camera_00);
  merger.setMeasurement(&measurements[1] /*passing measurements*/);
  merger.setCorrespondences(&correspondences_camera_00_from_01);

  // ds backup points for assertions
  const PointIntensityDescriptor3fVectorCloud points_in_camera_00_backup(points_in_camera_00);
  const size_t number_of_measurements = measurements[1].size();

  // ds merge points
  merger.compute();

  // ds this operation must not modify measurements
  ASSERT_EQ(measurements[1].size(), number_of_measurements);

  // ds the scene points vary as new points will be added
  ASSERT_GT(points_in_camera_00.size(), points_in_camera_00_backup.size());

  // ds consistency
  ASSERT_LE(points_in_camera_00.size(), points_in_camera_00_backup.size() + measurements[1].size());

  // ds TODO check merge result against ground truth - currently we only detect divergence
  for (size_t i = 0; i < points_in_camera_00_backup.size(); ++i) {
    const PointIntensityDescriptor3f& point_before_merge = points_in_camera_00_backup[i];
    const PointIntensityDescriptor3f& point_merged       = points_in_camera_00[i];
    ASSERT_NEAR(point_before_merge.coordinates()(0), point_merged.coordinates()(0), 5);
    ASSERT_NEAR(point_before_merge.coordinates()(1), point_merged.coordinates()(1), 5);
    ASSERT_NEAR(point_before_merge.coordinates()(2), point_merged.coordinates()(2), 5);
  }
}
