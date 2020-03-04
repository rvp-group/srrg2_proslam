#include "fixtures.hpp"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(SceneFlow, MeasurementAdaptorStereoProjectiveBruteforce) {
  MeasurementAdaptorStereoProjective adaptor;
  adaptor.param_feature_extractor_right.setValue(adaptor.param_feature_extractor.value());
  adaptor.param_feature_extractor->param_target_number_of_keypoints.setValue(500);
  adaptor.param_feature_extractor->param_detector_threshold.setValue(5);
  adaptor.param_correspondence_finder.setValue(CorrespondenceFinderDescriptorBasedBruteforce3D3DPtr(
    new CorrespondenceFinderDescriptorBasedBruteforce3D3D()));
  adaptor.param_correspondence_finder->param_maximum_descriptor_distance.setValue(100);
  adaptor.param_correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.8);

  // ds final adaptor output: pairs of corresponding 2d image coordinates and descriptors
  PointIntensityDescriptor4fVectorCloud stereo_points;

  // ds adapt measurements and store results in points
  ASSERT_TRUE(adaptor.setMeasurement(rigid_stereo_message));
  adaptor.setDest(&stereo_points);
  adaptor.compute();

  // ds validate expected feature extraction and matching result
  ASSERT_EQ(stereo_points.size(), static_cast<size_t>(83));

  // ds check disparities for all correspondences
  size_t number_of_inlier_stereo_matches = 0;
  evaluateStereoMatches(stereo_points, number_of_inlier_stereo_matches);
  ASSERT_EQ(number_of_inlier_stereo_matches, static_cast<size_t>(43));
}

TEST_F(SceneFlow, MeasurementAdaptorStereoProjectiveEpipolar) {
  MeasurementAdaptorStereoProjective adaptor;
  adaptor.param_feature_extractor_right.setValue(adaptor.param_feature_extractor.value());
  adaptor.param_feature_extractor->param_target_number_of_keypoints.setValue(500);
  adaptor.param_feature_extractor->param_detector_threshold.setValue(5);
  adaptor.param_correspondence_finder->param_maximum_descriptor_distance.setValue(100);
  adaptor.param_correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.8);

  // ds final adaptor output: pairs of corresponding 2d image coordinates and descriptors
  PointIntensityDescriptor4fVectorCloud stereo_points;

  // ds adapt measurements and store results in points
  ASSERT_TRUE(adaptor.setMeasurement(rigid_stereo_message));
  adaptor.setDest(&stereo_points);
  adaptor.compute();

  // ds validate expected feature extraction and matching result
  ASSERT_EQ(stereo_points.size(), static_cast<size_t>(115));

  // ds check disparities for all correspondences
  size_t number_of_inlier_stereo_matches = 0;
  evaluateStereoMatches(stereo_points, number_of_inlier_stereo_matches);
  ASSERT_EQ(number_of_inlier_stereo_matches, static_cast<size_t>(59));
}

TEST_F(ICL, MeasurementAdaptorMonocularDepth) {
  MeasurementAdaptorMonocularDepth adaptor;
  adaptor.param_feature_extractor->param_target_number_of_keypoints.setValue(500);
  adaptor.param_feature_extractor->param_detector_threshold.setValue(5);

  // ds final adaptor output: image coordinates with corresponding depth and descriptor
  PointIntensityDescriptor3fVectorCloud points_with_depth_and_descriptor;

  // ds adapt measurements and store results in points
  ASSERT_TRUE(adaptor.setMeasurement(message_00));
  adaptor.setDest(&points_with_depth_and_descriptor);
  adaptor.compute();

  // ds validate expected feature extraction result
  ASSERT_EQ(points_with_depth_and_descriptor.size(), static_cast<size_t>(321));

  // ds check coordinates, intensity and depth values
  for (const PointIntensityDescriptor3f& point : points_with_depth_and_descriptor) {
    const size_t row         = std::rint(point.coordinates()(1));
    const size_t col         = std::rint(point.coordinates()(0));
    const float depth_meters = point.coordinates()(2);

    ASSERT_IN_RANGE(row, static_cast<size_t>(0), image_rows);
    ASSERT_IN_RANGE(col, static_cast<size_t>(0), image_cols);
    ASSERT_NEAR(point.intensity(), image_intensity_00->at(row, col), 1e-5);
    ASSERT_GE(depth_meters, 0.0f);
    ASSERT_NEAR(depth_meters, image_depth_00->at(row, col), 1e-5);
  }
}

TEST_F(KITTI, MeasurementAdaptorStereoProjectiveBruteforce) {
  MeasurementAdaptorStereoProjective adaptor;
  adaptor.param_feature_extractor_right.setValue(adaptor.param_feature_extractor.value());
  adaptor.param_feature_extractor->param_target_number_of_keypoints.setValue(500);
  adaptor.param_feature_extractor->param_detector_threshold.setValue(5);
  adaptor.param_correspondence_finder.setValue(CorrespondenceFinderDescriptorBasedBruteforce3D3DPtr(
    new CorrespondenceFinderDescriptorBasedBruteforce3D3D()));
  adaptor.param_correspondence_finder->param_maximum_descriptor_distance.setValue(100);
  adaptor.param_correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.8);

  // ds final adaptor output: pairs of corresponding 2d image coordinates and descriptors
  PointIntensityDescriptor4fVectorCloud stereo_points;

  // ds adapt measurements and store results in points
  ASSERT_TRUE(adaptor.setMeasurement(messages[0]));
  adaptor.setDest(&stereo_points);
  adaptor.compute();

  // ds validate expected feature extraction and matching result
  ASSERT_EQ(stereo_points.size(), static_cast<size_t>(213));
}

TEST_F(KITTI, MeasurementAdaptorStereoProjectiveEpipolar) {
  MeasurementAdaptorStereoProjective adaptor;
  adaptor.param_feature_extractor_right.setValue(adaptor.param_feature_extractor.value());
  adaptor.param_feature_extractor->param_target_number_of_keypoints.setValue(500);
  adaptor.param_feature_extractor->param_detector_threshold.setValue(5);
  adaptor.param_correspondence_finder->param_maximum_descriptor_distance.setValue(100);
  adaptor.param_correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.8);

  // ds final adaptor output: pairs of corresponding 2d image coordinates and descriptors
  PointIntensityDescriptor4fVectorCloud stereo_points;

  // ds adapt measurements and store results in points
  ASSERT_TRUE(adaptor.setMeasurement(messages[0]));
  adaptor.setDest(&stereo_points);
  adaptor.compute();

  // ds validate expected feature extraction and matching result
  ASSERT_EQ(stereo_points.size(), static_cast<size_t>(177));
}
