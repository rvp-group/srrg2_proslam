#include "fixtures.hpp"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(KITTI, IntensityFeatureExtractorBinned_FASTORB) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("FAST");
  extractor.param_number_of_detectors_horizontal.setValue(1);
  extractor.param_number_of_detectors_vertical.setValue(1);
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features;
  extractor.setFeatures(&features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features.size(), 887);
}

TEST_F(KITTI, IntensityFeatureExtractorBinned_MSERORB) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("MSER");
  extractor.param_number_of_detectors_horizontal.setValue(1);
  extractor.param_number_of_detectors_vertical.setValue(1);
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features;
  extractor.setFeatures(&features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_GT(features.size(), 700); // ds TODO troubleshoot inconsistency on Ubuntus
}

TEST_F(KITTI, IntensityFeatureExtractorBinned_GFTTORB) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("GFTT");
  extractor.param_number_of_detectors_horizontal.setValue(1);
  extractor.param_number_of_detectors_vertical.setValue(1);
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);
  extractor.param_target_bin_width_pixels.setValue(10);

  // ds extract features and verify count (GFTT default values are conservative)
  PointIntensityDescriptor2fVectorCloud features;
  extractor.setFeatures(&features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features.size(), 753);
}

TEST_F(KITTI, IntensityFeatureExtractorBinned_ORBORB) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_detector_type.setValue("ORB-256");
  extractor.param_number_of_detectors_horizontal.setValue(1);
  extractor.param_number_of_detectors_vertical.setValue(1);
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features;
  extractor.setFeatures(&features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features.size(), 1000);
}

TEST_F(KITTI, IntensityFeatureExtractorBinned_BRISKBRISK) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_descriptor_type.setValue("BRISK-512");
  extractor.param_detector_type.setValue("BRISK-512");
  extractor.param_number_of_detectors_horizontal.setValue(1);
  extractor.param_number_of_detectors_vertical.setValue(1);
  extractor.param_target_number_of_keypoints.setValue(1000);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features;
  extractor.setFeatures(&features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features.size(), 1000);
}

TEST_F(ICL, IntensityFeatureExtractor_BinnedFeatureDensityRegulation_1x1) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_number_of_detectors_horizontal.setValue(1);
  extractor.param_number_of_detectors_vertical.setValue(1);
  extractor.param_target_number_of_keypoints.setValue(300);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 259);
  PointIntensityDescriptor2fVectorCloud features_01;
  extractor.setFeatures(&features_01);
  extractor.compute(image_intensity_01);
  ASSERT_EQ(features_01.size(), 254);
}

TEST_F(ICL, IntensityFeatureExtractorBinned_FeatureDensityRegulation_3x3) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_number_of_detectors_horizontal.setValue(3);
  extractor.param_number_of_detectors_vertical.setValue(3);
  extractor.param_target_number_of_keypoints.setValue(300);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 220);
  PointIntensityDescriptor2fVectorCloud features_01;
  extractor.setFeatures(&features_01);
  extractor.compute(image_intensity_01);
  ASSERT_EQ(features_01.size(), 228);
}

TEST_F(KITTI, IntensityFeatureExtractorBinned_FeatureDensityRegulation_3x3) {
  IntensityFeatureExtractorBinned2D extractor;

  // ds configure and initialize extractor
  extractor.param_number_of_detectors_horizontal.setValue(3);
  extractor.param_number_of_detectors_vertical.setValue(3);
  extractor.param_target_number_of_keypoints.setValue(300);
  extractor.param_detector_threshold.setValue(5);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00_left;
  extractor.setFeatures(&features_00_left);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features_00_left.size(), 272);
  PointIntensityDescriptor2fVectorCloud features_01_left;
  extractor.setFeatures(&features_01_left);
  extractor.compute(images_intensity_left[1]);
  ASSERT_EQ(features_01_left.size(), 280);

  // ds extract features and verify count (must be LE target_number_of_keypoints)
  PointIntensityDescriptor2fVectorCloud features_00_right;
  extractor.setFeatures(&features_00_right);
  extractor.compute(images_intensity_right[0]);
  ASSERT_EQ(features_00_right.size(), 270);
  PointIntensityDescriptor2fVectorCloud features_01_right;
  extractor.setFeatures(&features_01_right);
  extractor.compute(images_intensity_right[1]);
  ASSERT_EQ(features_01_right.size(), 271);
}

TEST_F(KITTI, IntensityFeatureExtractorSelective_GFFT_ORB256) {
  IntensityFeatureExtractorSelective2D extractor;

  // ds configure and initialize extractor
  extractor.param_detector_type.setValue("GFTT");
  extractor.param_descriptor_type.setValue("ORB-256");
  extractor.param_target_number_of_keypoints.setValue(100);
  extractor.param_target_bin_width_pixels.setValue(10);
  extractor.param_enable_seeding_when_tracking.setValue(false);

  // ds SEEDING with GFTT: extract features and verify count
  PointIntensityDescriptor2fVectorCloud initial_features;
  extractor.setFeatures(&initial_features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(initial_features.size(), 94);

  // ds change into tracking mode
  extractor.param_target_number_of_keypoints.setValue(1000);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 100
  PointIntensityDescriptor2fVectorCloud candidate_features_100;
  extractor.setFeatures(&candidate_features_100);
  extractor.setProjections(&initial_features, 100 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_100.size(), 719);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 50
  PointIntensityDescriptor2fVectorCloud candidate_features_50;
  extractor.setFeatures(&candidate_features_50);
  extractor.setProjections(&initial_features, 50 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_50.size(), 581);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 10
  PointIntensityDescriptor2fVectorCloud candidate_features_10;
  extractor.setFeatures(&candidate_features_10);
  extractor.setProjections(&initial_features, 10 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_10.size(), 294);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 5
  PointIntensityDescriptor2fVectorCloud candidate_features_05;
  extractor.setFeatures(&candidate_features_05);
  extractor.setProjections(&initial_features, 5 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_05.size(), 237);
}

TEST_F(KITTI, IntensityFeatureExtractorSelective_GFFT_BRIEF256) {
  IntensityFeatureExtractorSelective2D extractor;

  // ds configure and initialize extractor
  extractor.param_detector_type.setValue("GFTT");
  extractor.param_descriptor_type.setValue("BRIEF-256");
  extractor.param_target_number_of_keypoints.setValue(100);
  extractor.param_target_bin_width_pixels.setValue(10);
  extractor.param_enable_seeding_when_tracking.setValue(false);

  // ds SEEDING with GFTT: extract features and verify count
  PointIntensityDescriptor2fVectorCloud initial_features;
  extractor.setFeatures(&initial_features);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(initial_features.size(), 94);

  // ds change into tracking mode
  extractor.param_target_number_of_keypoints.setValue(1000);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 100
  PointIntensityDescriptor2fVectorCloud candidate_features_100;
  extractor.setFeatures(&candidate_features_100);
  extractor.setProjections(&initial_features, 100 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_100.size(), 719);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 50
  PointIntensityDescriptor2fVectorCloud candidate_features_50;
  extractor.setFeatures(&candidate_features_50);
  extractor.setProjections(&initial_features, 50 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_50.size(), 581);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 10
  PointIntensityDescriptor2fVectorCloud candidate_features_10;
  extractor.setFeatures(&candidate_features_10);
  extractor.setProjections(&initial_features, 10 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_10.size(), 294);

  // ds TRACKING with FAST: provide "projections" and extract candidates with radius = 5
  PointIntensityDescriptor2fVectorCloud candidate_features_05;
  extractor.setFeatures(&candidate_features_05);
  extractor.setProjections(&initial_features, 5 /*detection radius*/);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(candidate_features_05.size(), 237);
}
