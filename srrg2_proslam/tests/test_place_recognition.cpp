#include "fixtures.hpp"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(KITTI, MultiLoopDetectorHBST3D_FASTORB00To00) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds wrap measurements into a local map
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  local_map_00.setGraphId(0);
  LocalMap3D local_map_00_copy;
  local_map_00_copy.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  local_map_00_copy.setGraphId(1);

  // ds configure place recognition module
  MultiLoopDetectorHBST3D loop_detector;
  loop_detector.param_relocalize_min_inliers.setValue(points_in_camera_00.size() - 1);
  loop_detector.param_relocalize_min_inliers_ratio.setValue(0.99);
  loop_detector.param_maximum_descriptor_distance.setValue(1);
  loop_detector.param_minimum_age_difference_to_candidates.setValue(0);
  MultiAligner3DQRPtr loop_aligner(new MultiAligner3DQR());
  AlignerSlice3DPtr aligner_slice(new AlignerSlice3D());
  aligner_slice->param_fixed_slice_name.setValue("points");
  aligner_slice->param_moving_slice_name.setValue("points");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector.param_relocalize_aligner.setValue(loop_aligner);

  // ds test the place recognition module
  loop_detector.setCurrentLocalMap(&local_map_00);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 0);
  loop_detector.setCurrentLocalMap(&local_map_00_copy);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 1);
  ASSERT_EQ(loop_detector.indices()[0], 0);

  // ds perfect scenario
  CorrespondenceVector correspondences(std::move(loop_detector.correspondences(0)));
  ASSERT_EQ(correspondences.size(), points_in_camera_00.size());
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_EQ(correspondence.fixed_idx, correspondence.moving_idx);
    ASSERT_EQ(correspondence.response, 0);
  }
}

TEST_F(KITTI, MultiLoopDetectorHBST2D_FASTORB00To00) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor2fVectorCloud*>;
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("FAST");
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features_00.size(), 866);

  // ds wrap measurements into an abstract container;
  LocalMap2D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &features_00));
  local_map_00.setGraphId(0);
  LocalMap2D local_map_00_copy;
  local_map_00_copy.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &features_00));
  local_map_00_copy.setGraphId(1);

  // ds configure place recognition module
  MultiLoopDetectorHBST2D loop_detector;
  loop_detector.param_relocalize_min_inliers.setValue(features_00.size() - 1);
  loop_detector.param_relocalize_min_inliers_ratio.setValue(0.99);
  loop_detector.param_maximum_descriptor_distance.setValue(1);
  loop_detector.param_minimum_age_difference_to_candidates.setValue(0);
  MultiAligner2DPtr loop_aligner(new MultiAligner2D());
  AlignerSlice2DPtr aligner_slice(new AlignerSlice2D());
  aligner_slice->param_fixed_slice_name.setValue("points");
  aligner_slice->param_moving_slice_name.setValue("points");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector.param_relocalize_aligner.setValue(loop_aligner);

  // ds test the place recognition module
  loop_detector.setCurrentLocalMap(&local_map_00);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 0);
  loop_detector.setCurrentLocalMap(&local_map_00_copy);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 1);
  ASSERT_EQ(loop_detector.indices()[0], 0);

  // ds perfect scenario
  CorrespondenceVector correspondences(std::move(loop_detector.correspondences(0)));
  ASSERT_EQ(correspondences.size(), features_00.size());
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_EQ(correspondence.fixed_idx, correspondence.moving_idx);
    ASSERT_EQ(correspondence.response, 0);
  }
}

TEST_F(KITTI, MultiLoopDetectorHBST2D_FASTORB00To01) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor2fVectorCloud*>;
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("FAST");
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features_00.size(), 866);
  PointIntensityDescriptor2fVectorCloud features_01;
  extractor.setFeatures(&features_01);
  extractor.compute(images_intensity_left[1]);
  ASSERT_EQ(features_01.size(), 874);

  // ds wrap measurements into an abstract container;
  LocalMap2D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &features_00));
  local_map_00.setGraphId(0);
  LocalMap2D local_map_01;
  local_map_01.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &features_01));
  local_map_01.setGraphId(1);

  // ds configure place recognition module
  MultiLoopDetectorHBST2D loop_detector;
  loop_detector.param_relocalize_min_inliers.setValue(100);
  loop_detector.param_relocalize_min_inliers_ratio.setValue(0.1);
  loop_detector.param_maximum_descriptor_distance.setValue(25);
  loop_detector.param_minimum_age_difference_to_candidates.setValue(0);
  MultiAligner2DPtr loop_aligner(new MultiAligner2D());
  AlignerSlice2DPtr aligner_slice(new AlignerSlice2D());
  aligner_slice->param_fixed_slice_name.setValue("points");
  aligner_slice->param_moving_slice_name.setValue("points");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector.param_relocalize_aligner.setValue(loop_aligner);

  // ds test the place recognition module
  loop_detector.setCurrentLocalMap(&local_map_00);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 0);
  loop_detector.setCurrentLocalMap(&local_map_01);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 1);
  ASSERT_EQ(loop_detector.indices()[0], 0);

  // ds evaluate correspondences TODO projective verification
  CorrespondenceVector correspondences(std::move(loop_detector.correspondences(0)));
  ASSERT_EQ(correspondences.size(), 259);
  for (size_t i = 0; i < correspondences.size(); ++i) {
    ASSERT_LT(correspondences[i].response, 25);
  }
}

TEST_F(ICL, MultiLoopDetectorHBST2D_FASTORB00To50) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor2fVectorCloud*>;
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("FAST");
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 538);
  PointIntensityDescriptor2fVectorCloud features_50;
  extractor.setFeatures(&features_50);
  extractor.compute(image_intensity_50);
  ASSERT_EQ(features_50.size(), 451);

  // ds wrap measurements into an abstract container;
  LocalMap2D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &features_00));
  local_map_00.setGraphId(0);
  LocalMap2D local_map_50;
  local_map_50.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &features_50));
  local_map_50.setGraphId(1);

  // ds configure place recognition module
  MultiLoopDetectorHBST2D loop_detector;
  loop_detector.param_relocalize_min_inliers.setValue(100);
  loop_detector.param_relocalize_min_inliers_ratio.setValue(0.1);
  loop_detector.param_maximum_descriptor_distance.setValue(25);
  loop_detector.param_minimum_age_difference_to_candidates.setValue(0);
  MultiAligner2DPtr loop_aligner(new MultiAligner2D());
  AlignerSlice2DPtr aligner_slice(new AlignerSlice2D());
  aligner_slice->param_fixed_slice_name.setValue("points");
  aligner_slice->param_moving_slice_name.setValue("points");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector.param_relocalize_aligner.setValue(loop_aligner);

  // ds test the place recognition module
  loop_detector.setCurrentLocalMap(&local_map_00);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 0);
  loop_detector.setCurrentLocalMap(&local_map_50);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 1);
  ASSERT_EQ(loop_detector.indices()[0], 0);

  // ds evaluate correspondences TODO projective verification
  CorrespondenceVector correspondences(std::move(loop_detector.correspondences(0)));
  ASSERT_EQ(correspondences.size(), 103);
  for (size_t i = 0; i < correspondences.size(); ++i) {
    ASSERT_LT(correspondences[i].response, 25);
  }
}

TEST_F(ICL, MultiLoopDetectorHBST3D_FASTORB00To01) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds wrap measurements into an abstract container;
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &measurements_projective_depth_00));
  local_map_00.setGraphId(0);
  LocalMap3D local_map_01;
  local_map_01.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &measurements_projective_depth_01));
  local_map_01.setGraphId(1);

  // ds configure place recognition module
  MultiLoopDetectorHBST3D loop_detector;
  loop_detector.param_relocalize_min_inliers.setValue(250);
  loop_detector.param_relocalize_min_inliers_ratio.setValue(0.35);
  loop_detector.param_maximum_descriptor_distance.setValue(50);
  loop_detector.param_minimum_age_difference_to_candidates.setValue(0);
  MultiAligner3DQRPtr loop_aligner(new MultiAligner3DQR());
  AlignerSlice3DPtr aligner_slice(new AlignerSlice3D());
  aligner_slice->param_fixed_slice_name.setValue("points");
  aligner_slice->param_moving_slice_name.setValue("points");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector.param_relocalize_aligner.setValue(loop_aligner);

  // ds test the place recognition module
  loop_detector.setCurrentLocalMap(&local_map_00);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 0);
  loop_detector.setCurrentLocalMap(&local_map_01);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 1);
  ASSERT_EQ(loop_detector.indices()[0], 0);

  // ds evaluate correspondences based on ground truth camera motion
  CorrespondenceVector correspondences(std::move(loop_detector.correspondences(0)));
  ASSERT_EQ(correspondences.size(), 213);
  size_t number_of_invalid_correspondences = 0;
  for (const Correspondence& correspondence_query : correspondences) {
    ASSERT_LT(correspondence_query.response, 50);
    for (const Correspondence& correspondence_reference : correspondences_camera_01_from_00) {
      if (correspondence_query.fixed_idx == correspondence_reference.moving_idx) {
        if (correspondence_query.moving_idx != correspondence_reference.fixed_idx) {
          ++number_of_invalid_correspondences;
        }
      }
    }
  }
  ASSERT_LE(number_of_invalid_correspondences, 65);
}

TEST_F(KITTI, MultiLoopDetectorHBST3D_FASTORB00To01) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds wrap measurements into an abstract container;
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  local_map_00.setGraphId(0);
  LocalMap3D local_map_01;
  local_map_01.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_01));
  local_map_01.setGraphId(1);

  // ds configure place recognition module
  MultiLoopDetectorHBST3D loop_detector;
  loop_detector.param_relocalize_min_inliers.setValue(50);
  loop_detector.param_relocalize_min_inliers_ratio.setValue(0.25);
  loop_detector.param_maximum_descriptor_distance.setValue(50);
  loop_detector.param_minimum_age_difference_to_candidates.setValue(0);
  MultiAligner3DQRPtr loop_aligner(new MultiAligner3DQR());
  AlignerSlice3DPtr aligner_slice(new AlignerSlice3D());
  aligner_slice->param_fixed_slice_name.setValue("points");
  aligner_slice->param_moving_slice_name.setValue("points");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector.param_relocalize_aligner.setValue(loop_aligner);

  // ds test the place recognition module
  loop_detector.setCurrentLocalMap(&local_map_00);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 0);
  loop_detector.setCurrentLocalMap(&local_map_01);
  loop_detector.computeCorrespondences();
  loop_detector.addPreviousQuery();
  ASSERT_EQ(loop_detector.indices().size(), 1);
  ASSERT_EQ(loop_detector.indices()[0], 0);

  // ds evaluate correspondences based on ground truth camera motion
  CorrespondenceVector correspondences(std::move(loop_detector.correspondences(0)));
  ASSERT_EQ(correspondences.size(), 74);
  size_t number_of_invalid_correspondences = 0;
  for (const Correspondence& correspondence_query : correspondences) {
    ASSERT_LT(correspondence_query.response, 50);
    for (const Correspondence& correspondence_reference : correspondences_camera_01_from_00) {
      if (correspondence_query.fixed_idx == correspondence_reference.moving_idx) {
        if (correspondence_query.moving_idx != correspondence_reference.fixed_idx) {
          ++number_of_invalid_correspondences;
        }
      }
    }
  }
  ASSERT_LE(number_of_invalid_correspondences, 48);
}
