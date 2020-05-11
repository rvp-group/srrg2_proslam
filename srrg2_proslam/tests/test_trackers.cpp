#include "fixtures.hpp"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(ICL, 00To00_Tracker_ProjectiveBruteforce_MergerDefault) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "icl.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorProjectiveDepthPtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorProjectiveDepth>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveDepthPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveDepth>(0);
  ASSERT_NOTNULL(aligner_slice);
  RawDataPreprocessorMonocularDepthPtr adaptor =
    tracker_slice->param_adaptor.value<RawDataPreprocessorMonocularDepth>();
  ASSERT_NOTNULL(adaptor);
  adaptor->param_depth_scaling_factor_to_meters.setValue(1.0 /*test only*/);

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  MergerProjectiveDepthEKFPtr merger =
    tracker_slice->param_merger.getSharedPtr<MergerProjectiveDepthEKF>();
  ASSERT_NOTNULL(merger);
  ASSERT_NOTNULL(merger->param_unprojector.value());
  merger->param_unprojector->setCameraMatrix(camera_calibration_matrix);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderProjectiveCircle3D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle3D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds repeatedly process the same measurements
  size_t number_of_points_measured = 0;
  size_t number_of_points_in_map   = 0;
  for (size_t step = 0; step < 3; ++step) {
    // ds process measurement
    tracker_slice->param_measurement_slice_name.setValue("points");
    aligner_slice->param_fixed_slice_name.setValue("points");
    tracker->setRawData(message_00);
    Property_<PointIntensityDescriptor3fVectorCloud*>* measurement_property =
      tracker->measurementContainer().property<Property_<PointIntensityDescriptor3fVectorCloud*>>(
        "points");
    ASSERT_NOTNULL(measurement_property);
    tracker->compute();

    // ds check map sanity
    Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
      global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
    ASSERT_NOTNULL(global_map_property);

    // ds set points based on first processed measurement
    if (step == 0) {
      number_of_points_measured = measurement_property->value()->size();
      number_of_points_in_map   = global_map_property->value()->size();
      ASSERT_GT(number_of_points_in_map, 0);
      ASSERT_GE(number_of_points_measured, number_of_points_in_map);
    }

    // ds the robot should not move and the map not grow
    ASSERT_EQ(tracker->numFramesProcessed(), step + 1);
    ASSERT_EQ(measurement_property->value()->size(), number_of_points_measured);
    ASSERT_EQ(global_map_property->value()->size(), number_of_points_in_map);
    ASSERT_LT(geometry3d::t2tnq(tracker->robotInLocalMap()).norm(), 1e-5);
  }
}

TEST_F(ICL, 00To50_Tracker_ProjectiveBruteforce) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "icl.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorProjectiveDepthPtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorProjectiveDepth>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveDepthPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveDepth>(0);
  ASSERT_NOTNULL(aligner_slice);
  RawDataPreprocessorMonocularDepthPtr adaptor =
    tracker_slice->param_adaptor.value<RawDataPreprocessorMonocularDepth>();
  ASSERT_NOTNULL(adaptor);
  adaptor->param_depth_scaling_factor_to_meters.setValue(1.0 /*test only*/);

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  MergerProjectiveDepthEKFPtr merger =
    tracker_slice->param_merger.getSharedPtr<MergerProjectiveDepthEKF>();
  ASSERT_NOTNULL(merger);
  ASSERT_NOTNULL(merger->param_unprojector.value());
  merger->param_unprojector->setCameraMatrix(camera_calibration_matrix);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds process first frame
  tracker->setRawData(message_00);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(tracker->numFramesProcessed(), 1);
  Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
    global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
  ASSERT_NOTNULL(global_map_property);

  // ds process second frame
  tracker->setRawData(message_01);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 2);

  // ds jump 50 frames ahead (not to brutal in ICL living room 0)
  tracker->setRawData(message_50);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 3);

  // ds evaluate final error (tracker estimate is already inverted)
  const Vector6f error = geometry3d::t2tnq(tracker->robotInLocalMap().inverse() * camera_50_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.02);
  ASSERT_LT_ABS(error(1), 0.02);
  ASSERT_LT_ABS(error(2), 0.02);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To00_Tracker_ProjectiveBruteforce_MergerDefault) {
  using MergerBaseType = MergerRigidStereo_<srrg2_core::Isometry3f,
                                            srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                            srrg2_core::PointIntensityDescriptor4fVectorCloud>;

  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorStereoProjectivePtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorStereoProjective>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveStereo>(0);
  ASSERT_NOTNULL(aligner_slice);
  aligner_slice->param_robustifier->param_chi_threshold.setValue(1000);
  ASSERT_NOTNULL(tracker_slice->param_adaptor.value());
  std::shared_ptr<MergerBaseType> merger =
    tracker_slice->param_merger.getSharedPtr<MergerBaseType>();
  ASSERT_NOTNULL(merger);
  ASSERT_NOTNULL(merger->param_triangulator.value());
  ASSERT_NOTNULL(merger->param_triangulator->param_projector.value());

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper->param_projector->param_canvas_rows.setValue(image_rows);
  clipper->param_projector->param_canvas_cols.setValue(image_cols);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderProjectiveCircle4D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);
  finder->param_minimum_descriptor_distance.setValue(25);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder->param_minimum_search_radius_pixels.setValue(5);

  // srrg set projector to aligner slice
  ASSERT_NOTNULL(_platform);
  aligner_slice->setPlatform(_platform);
  merger->param_triangulator->setPlatform(_platform);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds repeatedly process the same measurements
  size_t number_of_points_measured = 0;
  size_t number_of_points_in_map   = 0;
  for (size_t step = 0; step < 3; ++step) {
    // ds check map sanity
    Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
      global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
    ASSERT_NOTNULL(global_map_property);

    // ds process measurement
    tracker_slice->param_measurement_slice_name.setValue("points");
    aligner_slice->param_fixed_slice_name.setValue("points");
    tracker->setRawData(messages[0]);
    Property_<PointIntensityDescriptor4fVectorCloud*>* measurement_property =
      tracker->measurementContainer().property<Property_<PointIntensityDescriptor4fVectorCloud*>>(
        "points");
    ASSERT_NOTNULL(measurement_property);
    tracker->compute();

    // ds set points based on first processed measurement
    if (step == 0) {
      number_of_points_measured = measurement_property->value()->size();
      number_of_points_in_map   = global_map_property->value()->size();
      ASSERT_GT(number_of_points_in_map, 0);
      ASSERT_GE(number_of_points_measured, number_of_points_in_map);
    }

    // ds the robot should not move and the map not grow
    ASSERT_EQ(tracker->numFramesProcessed(), step + 1);
    ASSERT_EQ(measurement_property->value()->size(), number_of_points_measured);
    ASSERT_EQ(global_map_property->value()->size(), number_of_points_in_map);
    ASSERT_LT(geometry3d::t2tnq(tracker->robotInLocalMap()).norm(), 1e-5);
  }
}

TEST_F(KITTI, 00To04_Tracker_ProjectiveBF_NoMerges) {
  using MergerBaseType = MergerRigidStereo_<srrg2_core::Isometry3f,
                                            srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                            srrg2_core::PointIntensityDescriptor4fVectorCloud>;
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorStereoProjectivePtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorStereoProjective>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveStereo>(0);
  ASSERT_NOTNULL(aligner_slice);
  aligner_slice->param_robustifier->param_chi_threshold.setValue(1000);
  ASSERT_NOTNULL(tracker_slice->param_adaptor.value());
  std::shared_ptr<MergerBaseType> merger =
    tracker_slice->param_merger.getSharedPtr<MergerBaseType>();
  ASSERT_NOTNULL(merger);
  // ds disable point merges (by setting infeasible thresholds)
  merger->param_maximum_distance_appearance.setValue(0);
  merger->param_landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(0);
  ASSERT_NOTNULL(merger->param_triangulator.value());

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper->param_projector->param_canvas_rows.setValue(image_rows);
  clipper->param_projector->param_canvas_cols.setValue(image_cols);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderProjectiveCircle4D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);
  finder->param_minimum_descriptor_distance.setValue(25);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder->param_minimum_search_radius_pixels.setValue(5);
  finder->param_maximum_search_radius_pixels.setValue(50);
  finder->param_minimum_matching_ratio.setValue(0.1);

  // ds configure measurement adaptor
  tracker_slice->param_adaptor.setValue(adaptor);

  // srrg set projector to aligner slice
  ASSERT_NOTNULL(_platform);
  aligner_slice->setPlatform(_platform);
  merger->param_triangulator->setPlatform(_platform);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds process first stereo frame
  tracker->setRawData(messages[0]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(tracker->numFramesProcessed(), 1);
  Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
    global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
  ASSERT_NOTNULL(global_map_property);

  tracker->setRawData(messages[1]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 2);

  tracker->setRawData(messages[2]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 3);

  tracker->setRawData(messages[3]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 4);

  tracker->setRawData(messages[4]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 5);

  // ds evaluate final error (tracker estimate is already inverted)
  const Vector6f error = geometry3d::t2tnq(tracker->robotInLocalMap().inverse() * camera_04_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.2);
  ASSERT_LT_ABS(error(1), 0.2);
  ASSERT_LT_ABS(error(2), 0.7);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To04_Tracker_ProjectiveBruteforce_MergerTriangulation_WeightedMean) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorStereoProjectivePtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorStereoProjective>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveStereo>(0);
  ASSERT_NOTNULL(aligner_slice);
  aligner_slice->param_robustifier->param_chi_threshold.setValue(1000);
  ASSERT_NOTNULL(tracker_slice->param_adaptor.value());
  MergerRigidStereoTriangulationPtr merger =
    manager.getByName<MergerRigidStereoTriangulation>("merger_triangulation");
  ASSERT_NOTNULL(merger);
  merger->param_maximum_distance_appearance.setValue(50);
  merger->param_landmark_estimator.setValue(
    manager.getByName<LandmarkEstimatorWeightedMean4D3D>("landmark_estimator_weighted_mean"));
  merger->param_landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  tracker_slice->param_merger.setValue(merger);
  ASSERT_NOTNULL(merger->param_triangulator.value());

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper->param_projector->param_canvas_rows.setValue(image_rows);
  clipper->param_projector->param_canvas_cols.setValue(image_cols);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderProjectiveCircle4D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);
  finder->param_minimum_descriptor_distance.setValue(25);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder->param_minimum_search_radius_pixels.setValue(5);
  finder->param_maximum_search_radius_pixels.setValue(50);
  finder->param_minimum_matching_ratio.setValue(0.1);

  // ds configure measurement adaptor
  tracker_slice->param_adaptor.setValue(adaptor);

  // srrg set projector to aligner slice
  ASSERT_NOTNULL(_platform);
  aligner_slice->setPlatform(_platform);
  merger->param_triangulator->setPlatform(_platform);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds process first stereo frame
  tracker->setRawData(messages[0]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(tracker->numFramesProcessed(), 1);
  Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
    global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
  ASSERT_NOTNULL(global_map_property);

  tracker->setRawData(messages[1]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 2);

  tracker->setRawData(messages[2]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 3);

  tracker->setRawData(messages[3]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 4);

  tracker->setRawData(messages[4]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 5);

  // ds evaluate final error (tracker estimate is already inverted)
  const Vector6f error = geometry3d::t2tnq(tracker->robotInLocalMap().inverse() * camera_04_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.2);
  ASSERT_LT_ABS(error(1), 0.2);
  ASSERT_LT_ABS(error(2), 0.7);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To04_Tracker_ProjectiveBruteforce_MergerEKF) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorStereoProjectivePtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorStereoProjective>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveStereo>(0);
  ASSERT_NOTNULL(aligner_slice);
  aligner_slice->param_robustifier->param_chi_threshold.setValue(1000);
  ASSERT_NOTNULL(tracker_slice->param_adaptor.value());
  MergerRigidStereoProjectiveEKFPtr merger =
    manager.getByName<MergerRigidStereoProjectiveEKF>("merger_ekf");
  ASSERT_NOTNULL(merger);
  merger->param_maximum_distance_appearance.setValue(50);
  merger->param_landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  tracker_slice->param_merger.setValue(merger);
  ASSERT_NOTNULL(merger->param_triangulator.value());

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper->param_projector->param_canvas_rows.setValue(image_rows);
  clipper->param_projector->param_canvas_cols.setValue(image_cols);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderProjectiveCircle4D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);
  finder->param_minimum_descriptor_distance.setValue(25);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder->param_minimum_search_radius_pixels.setValue(5);
  finder->param_maximum_search_radius_pixels.setValue(50);
  finder->param_minimum_matching_ratio.setValue(0.1);

  // ds configure measurement adaptor
  tracker_slice->param_adaptor.setValue(adaptor);

  // srrg set projector to aligner slice
  ASSERT_NOTNULL(_platform);
  tracker->setPlatform(_platform);
  //  aligner_slice->setPlatform(_platform);
  //  merger->param_triangulator->setPlatform(_platform);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds process first stereo frame
  tracker->setRawData(messages[0]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(tracker->numFramesProcessed(), 1);
  Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
    global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
  ASSERT_NOTNULL(global_map_property);

  tracker->setRawData(messages[1]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 2);

  tracker->setRawData(messages[2]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 3);

  tracker->setRawData(messages[3]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 4);

  tracker->setRawData(messages[4]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 5);

  // ds evaluate final error (tracker estimate is already inverted)
  const Vector6f error = geometry3d::t2tnq(tracker->robotInLocalMap().inverse() * camera_04_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.2);
  ASSERT_LT_ABS(error(1), 0.2);
  ASSERT_LT_ABS(error(2), 0.7);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To04_Tracker_ProjectiveBruteforce_MergerTriangulation_PoseBasedSmoother) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorStereoProjectivePtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorStereoProjective>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveStereo>(0);
  ASSERT_NOTNULL(aligner_slice);
  aligner_slice->param_robustifier->param_chi_threshold.setValue(1000);
  ASSERT_NOTNULL(tracker_slice->param_adaptor.value());
  MergerRigidStereoTriangulationPtr merger =
    manager.getByName<MergerRigidStereoTriangulation>("merger_triangulation");
  ASSERT_NOTNULL(merger);
  merger->param_maximum_distance_appearance.setValue(50);
  merger->param_landmark_estimator.setValue(
    manager.getByName<LandmarkEstimatorPoseBasedSmoother4D3D>("landmark_estimator_smoother"));
  merger->param_landmark_estimator->param_maximum_distance_geometry_meters_squared.setValue(25);
  tracker_slice->param_merger.setValue(merger);
  ASSERT_NOTNULL(merger->param_triangulator.value());

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper->param_projector->param_canvas_rows.setValue(image_rows);
  clipper->param_projector->param_canvas_cols.setValue(image_cols);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderProjectiveCircle4D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);
  finder->param_minimum_descriptor_distance.setValue(25);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder->param_minimum_search_radius_pixels.setValue(5);
  finder->param_maximum_search_radius_pixels.setValue(50);
  finder->param_minimum_matching_ratio.setValue(0.1);

  // ds configure measurement adaptor
  tracker_slice->param_adaptor.setValue(adaptor);

  // srrg set projector to aligner slice
  ASSERT_NOTNULL(_platform);
  aligner_slice->setPlatform(_platform);
  merger->param_triangulator->setPlatform(_platform);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds process first stereo frame
  tracker->setRawData(messages[0]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(tracker->numFramesProcessed(), 1);
  Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
    global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
  ASSERT_NOTNULL(global_map_property);

  tracker->setRawData(messages[1]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 2);

  tracker->setRawData(messages[2]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 3);

  tracker->setRawData(messages[3]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 4);

  tracker->setRawData(messages[4]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 5);

  // ds evaluate final error (tracker estimate is already inverted)
  const Vector6f error = geometry3d::t2tnq(tracker->robotInLocalMap().inverse() * camera_04_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.2);
  ASSERT_LT_ABS(error(1), 0.2);
  ASSERT_LT_ABS(error(2), 0.7);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To04_Tracker_Bruteforce_MergerEKF) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  std::shared_ptr<MultiTracker3D> tracker = manager.getByName<MultiTracker3D>("tracker");
  ASSERT_NOTNULL(tracker);

  // ds verify deep configuration
  ASSERT_NOTNULL(tracker->param_aligner.value());
  MultiAligner3DQRPtr aligner = tracker->param_aligner.getSharedPtr<MultiAligner3DQR>();
  ASSERT_NOTNULL(aligner);
  ASSERT_EQ(tracker->param_slice_processors.size(), 2);
  TrackerSliceProcessorStereoProjectivePtr tracker_slice =
    tracker->param_slice_processors.getSharedPtr<TrackerSliceProcessorStereoProjective>(0);
  ASSERT_NOTNULL(tracker_slice);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr aligner_slice =
    aligner->param_slice_processors.getSharedPtr<AlignerSliceProcessorProjectiveStereo>(0);
  ASSERT_NOTNULL(aligner_slice);
  aligner_slice->param_robustifier->param_chi_threshold.setValue(1000);
  ASSERT_NOTNULL(tracker_slice->param_adaptor.value());
  MergerRigidStereoProjectiveEKFPtr merger =
    manager.getByName<MergerRigidStereoProjectiveEKF>("merger_ekf");
  ASSERT_NOTNULL(merger);
  tracker_slice->param_merger.setValue(merger);
  ASSERT_NOTNULL(merger->param_triangulator.value());

  // ds configure camera matrices
  SceneClipperProjective3DPtr clipper =
    tracker_slice->param_clipper.getSharedPtr<SceneClipperProjective3D>();
  ASSERT_NOTNULL(clipper);
  ASSERT_NOTNULL(clipper->param_projector.value());
  clipper->param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper->param_projector->param_canvas_rows.setValue(image_rows);
  clipper->param_projector->param_canvas_cols.setValue(image_cols);

  // ds configure projector of projective correspondence finder
  CorrespondenceFinderDescriptorBasedBruteforce4D3DPtr finder(
    new CorrespondenceFinderDescriptorBasedBruteforce4D3D());
  ASSERT_NOTNULL(finder);
  aligner_slice->param_finder.setValue(finder);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);

  // ds configure measurement adaptor
  tracker_slice->param_adaptor.setValue(adaptor);

  // srrg set projector to aligner slice
  ASSERT_NOTNULL(_platform);
  aligner_slice->setPlatform(_platform);
  merger->param_triangulator->setPlatform(_platform);

  // ds set global map buffer
  PropertyContainerDynamic global_map;
  tracker->setScene(&global_map);
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(aligner->status(), MultiAligner3DQR::Status::Fail);

  // ds process first stereo frame
  tracker->setRawData(messages[0]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Initializing);
  ASSERT_EQ(tracker->numFramesProcessed(), 1);
  Property_<PointIntensityDescriptor3fVectorCloud*>* global_map_property =
    global_map.property<Property_<PointIntensityDescriptor3fVectorCloud*>>("points");
  ASSERT_NOTNULL(global_map_property);

  tracker->setRawData(messages[1]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 2);

  tracker->setRawData(messages[2]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 3);

  tracker->setRawData(messages[3]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 4);

  tracker->setRawData(messages[4]);
  tracker->compute();
  ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);
  ASSERT_EQ(tracker->numFramesProcessed(), 5);

  // ds evaluate final error (tracker estimate is already inverted)
  const Vector6f error = geometry3d::t2tnq(tracker->robotInLocalMap().inverse() * camera_04_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.2);
  ASSERT_LT_ABS(error(1), 0.2);
  ASSERT_LT_ABS(error(2), 0.7); // ds TODO uagh uagh uagh
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}
/*
using SyntheticWorldWithDescriptorsSE3 =
  srrg2_test::SyntheticWorld<3, float, PointIntensityDescriptor3f, PointIntensityDescriptor2f>;

TEST_F(SyntheticWorldWithDescriptorsSE3, TrackerProjective) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree2D3DPtr correspondence_finder(
    new CorrespondenceFinderProjectiveKDTree2D3D);
  correspondence_finder->param_maximum_descriptor_distance.setValue(75);
  correspondence_finder->param_minimum_descriptor_distance.setValue(25);
  correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  correspondence_finder->param_maximum_search_radius_pixels.setValue(50);
  correspondence_finder->param_minimum_number_of_points_per_cluster.setValue(10);
  correspondence_finder->param_projector->param_canvas_cols.setValue(canvas_size(0));
  correspondence_finder->param_projector->param_canvas_rows.setValue(canvas_size(1));
  correspondence_finder->param_projector->param_range_min.setValue(0.1);
  correspondence_finder->param_projector->param_range_max.setValue(1000);
  correspondence_finder->param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  size_t number_of_poses = 10;
  Isometry3f start       = Isometry3f::Identity();
  Isometry3f goal        = Isometry3f::Identity();
  goal.translation() += Vector3f(10, 10, 10);
  const Vector3f pose_deviation(0.01, 0.01, 0.01);

  generateTrajectory(number_of_poses, start, goal, pose_deviation);
}
*/
