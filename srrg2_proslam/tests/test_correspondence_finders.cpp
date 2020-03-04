#include "fixtures.hpp"

using namespace srrg2_test;
using namespace srrg2_core;

using SyntheticWorldWithDescriptorsSE3 =
  SyntheticWorld<3, float, PointIntensityDescriptor3f, PointIntensityDescriptor2f>;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(ICL, 00To00_CorrespondenceFinderBruteforce) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderDescriptorBasedBruteforce2D2D correspondence_finder;

  // ds extract features
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 321);

  // ds match features against themselves
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_00);
  correspondence_finder.setMoving(&features_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, must be ideal!
  // ds we might loose some matches in case that two or more perfect candidates are available
  // ds this is highly unrealistic for real datasets but can happen more often for synthetic ones
  ASSERT_EQ(correspondences.size(), 319);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_EQ(correspondence.fixed_idx, correspondence.moving_idx);
    ASSERT_EQ(correspondence.response, 0.0f);
  }
}

TEST_F(ICL, 00To01_CorrespondenceFinderBruteforce) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderDescriptorBasedBruteforce2D2D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);

  // ds extract features
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 321);
  PointIntensityDescriptor2fVectorCloud features_01;
  extractor.setFeatures(&features_01);
  extractor.compute(image_intensity_01);
  ASSERT_EQ(features_01.size(), 338);

  // ds match features
  CorrespondenceVector correspondences_00_to_01;
  correspondence_finder.setFixed(&features_00);
  correspondence_finder.setMoving(&features_01);
  correspondence_finder.setCorrespondences(&correspondences_00_to_01);
  correspondence_finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences_00_to_01.size(), 226);
  for (const Correspondence& correspondence : correspondences_00_to_01) {
    ASSERT_LE(correspondence.response, 50.0f);
  }

  // ds bruteforce matching must be bijective!
  CorrespondenceVector correspondences_01_to_00;
  correspondence_finder.setFixed(&features_01);
  correspondence_finder.setMoving(&features_00);
  correspondence_finder.setCorrespondences(&correspondences_01_to_00);
  correspondence_finder.compute();
  ASSERT_EQ(correspondences_01_to_00.size(), correspondences_00_to_01.size());
  size_t number_of_checked_correspondences = 0;
  for (const Correspondence& correspondence_00_to_01 : correspondences_00_to_01) {
    // ds the correspondences are shuffled
    for (const Correspondence& correspondence_01_to_00 : correspondences_01_to_00) {
      if (correspondence_00_to_01.fixed_idx == correspondence_01_to_00.moving_idx) {
        ASSERT_EQ(correspondence_00_to_01.moving_idx, correspondence_01_to_00.fixed_idx);
        ++number_of_checked_correspondences;
        break;
      }
    }
  }
  ASSERT_EQ(number_of_checked_correspondences, correspondences_01_to_00.size());
}

TEST_F(ICL, 00To50_CorrespondenceFinderBruteforce) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderDescriptorBasedBruteforce2D2D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);

  // ds extract features
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 321);
  PointIntensityDescriptor2fVectorCloud features_50;
  extractor.setFeatures(&features_50);
  extractor.compute(image_intensity_50);
  ASSERT_EQ(features_50.size(), 261);

  // ds match features
  CorrespondenceVector correspondences_00_to_50;
  correspondence_finder.setFixed(&features_00);
  correspondence_finder.setMoving(&features_50);
  correspondence_finder.setCorrespondences(&correspondences_00_to_50);
  correspondence_finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences_00_to_50.size(), 117);
  for (const Correspondence& correspondence : correspondences_00_to_50) {
    ASSERT_LE(correspondence.response, 50.0f);
  }

  // ds bruteforce matching must be bijective!
  CorrespondenceVector correspondences_50_to_00;
  correspondence_finder.setFixed(&features_50);
  correspondence_finder.setMoving(&features_00);
  correspondence_finder.setCorrespondences(&correspondences_50_to_00);
  correspondence_finder.compute();
  ASSERT_EQ(correspondences_50_to_00.size(), correspondences_00_to_50.size());
  size_t number_of_checked_correspondences = 0;
  for (const Correspondence& correspondence_00_to_50 : correspondences_00_to_50) {
    // ds the correspondences are shuffled
    for (const Correspondence& correspondence_50_to_00 : correspondences_50_to_00) {
      if (correspondence_00_to_50.fixed_idx == correspondence_50_to_00.moving_idx) {
        ASSERT_EQ(correspondence_00_to_50.moving_idx, correspondence_50_to_00.fixed_idx);
        ++number_of_checked_correspondences;
        break;
      }
    }
  }
  ASSERT_EQ(number_of_checked_correspondences, correspondences_50_to_00.size());
}

TEST_F(KITTI, 00To00_CorrespondenceFinderEpipolar) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderDescriptorBasedEpipolar2D2D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);

  // ds extract features from left and right camera image
  PointIntensityDescriptor2fVectorCloud features_left;
  extractor.setFeatures(&features_left);
  extractor.compute(images_intensity_left[0]);
  ASSERT_EQ(features_left.size(), 446);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_left);
  correspondence_finder.setMoving(&features_left);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), features_left.size());
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_EQ(correspondence.fixed_idx, correspondence.moving_idx);
    ASSERT_EQ(correspondence.response, 0.0f);
  }
}

TEST_F(KITTI, 00LeftToRight_CorrespondenceFinderBruteforce) {
  IntensityFeatureExtractorBinned2D extractor_left;
  extractor_left.param_target_number_of_keypoints.setValue(500);
  extractor_left.param_detector_threshold.setValue(5);
  IntensityFeatureExtractorBinned2D extractor_right;
  extractor_right.param_target_number_of_keypoints.setValue(500);
  extractor_right.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderDescriptorBasedBruteforce2D2D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);

  // ds extract features from left and right camera image
  PointIntensityDescriptor2fVectorCloud features_left;
  PointIntensityDescriptor2fVectorCloud features_right;
  extractor_left.setFeatures(&features_left);
  extractor_right.setFeatures(&features_right);
  extractor_left.compute(images_intensity_left[0]);
  extractor_right.compute(images_intensity_right[0]);
  ASSERT_EQ(features_left.size(), 446);
  ASSERT_EQ(features_right.size(), 444);

  // ds match features
  CorrespondenceVector correspondences_left_to_right;
  correspondence_finder.setFixed(&features_left);
  correspondence_finder.setMoving(&features_right);
  correspondence_finder.setCorrespondences(&correspondences_left_to_right);
  correspondence_finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences_left_to_right.size(), 237);
  for (const Correspondence& correspondence : correspondences_left_to_right) {
    ASSERT_LE(correspondence.response, 50.0f);
  }

  // ds bruteforce matching must be bijective!
  CorrespondenceVector correspondences_right_to_left;
  correspondence_finder.setFixed(&features_right);
  correspondence_finder.setMoving(&features_left);
  correspondence_finder.setCorrespondences(&correspondences_right_to_left);
  correspondence_finder.compute();
  ASSERT_EQ(correspondences_right_to_left.size(), correspondences_left_to_right.size());
  size_t number_of_checked_correspondences = 0;
  for (const Correspondence& correspondence_left_to_right : correspondences_left_to_right) {
    // ds the correspondences are shuffled
    for (const Correspondence& correspondence_right_to_left : correspondences_right_to_left) {
      if (correspondence_left_to_right.fixed_idx == correspondence_right_to_left.moving_idx) {
        ASSERT_EQ(correspondence_left_to_right.moving_idx, correspondence_right_to_left.fixed_idx);
        ++number_of_checked_correspondences;
        break;
      }
    }
  }
  ASSERT_EQ(number_of_checked_correspondences, correspondences_right_to_left.size());
}

TEST_F(KITTI, 00LeftToRight_CorrespondenceFinderEpipolar) {
  IntensityFeatureExtractorBinned2D extractor_left;
  extractor_left.param_target_number_of_keypoints.setValue(500);
  extractor_left.param_detector_threshold.setValue(5);
  IntensityFeatureExtractorBinned2D extractor_right;
  extractor_right.param_target_number_of_keypoints.setValue(500);
  extractor_right.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderDescriptorBasedEpipolar2D2D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_maximum_disparity_pixels.setValue(100);

  // ds extract features from left and right camera image
  PointIntensityDescriptor2fVectorCloud features_left;
  PointIntensityDescriptor2fVectorCloud features_right;
  extractor_left.setFeatures(&features_left);
  extractor_right.setFeatures(&features_right);
  extractor_left.compute(images_intensity_left[0]);
  extractor_right.compute(images_intensity_right[0]);
  ASSERT_EQ(features_left.size(), 446);
  ASSERT_EQ(features_right.size(), 444);

  // ds match features for epipolar thickness = 0
  {
    correspondence_finder.param_epipolar_line_thickness_pixels.setValue(0);
    CorrespondenceVector correspondences;
    correspondence_finder.setFixed(&features_left);
    correspondence_finder.setMoving(&features_right);
    correspondence_finder.setCorrespondences(&correspondences);
    correspondence_finder.compute();

    // ds validate matching result
    ASSERT_EQ(correspondences.size(), 150);
    for (const Correspondence& correspondence : correspondences) {
      ASSERT_LE(correspondence.response, 50.0f);
    }
  }

  // ds match features for epipolar thickness = 1
  {
    correspondence_finder.param_epipolar_line_thickness_pixels.setValue(1);
    CorrespondenceVector correspondences;
    correspondence_finder.setFixed(&features_left);
    correspondence_finder.setMoving(&features_right);
    correspondence_finder.setCorrespondences(&correspondences);
    correspondence_finder.compute();

    // ds validate matching result
    ASSERT_EQ(correspondences.size(), 241);
    for (const Correspondence& correspondence : correspondences) {
      ASSERT_LE(correspondence.response, 50.0f);
    }
  }
}

TEST_F(ICL, 00To00_CorrespondenceFinder_ProjectiveKDTree_IdentityEstimate) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveKDTree2D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(10);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide pose estimate
  correspondence_finder.setEstimate(Isometry3f::Identity());

  // ds extract features
  PointIntensityDescriptor2fVectorCloud features_00;
  extractor.setFeatures(&features_00);
  extractor.compute(image_intensity_00);
  ASSERT_EQ(features_00.size(), 321);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_00);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, some matches are dropped due to Lowes check
  ASSERT_EQ(correspondences.size(), 319);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_EQ(correspondence.fixed_idx, correspondence.moving_idx);
    ASSERT_EQ(correspondence.response, 0.0f);
  }
}

TEST_F(ICL, 00To50_CorrespondenceFinder_ProjectiveKDTree_PerfectEstimate) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveKDTree2D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(10);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide accuracte pose estimate
  correspondence_finder.setEstimate(camera_50_from_00);

  // ds extract features
  PointIntensityDescriptor2fVectorCloud features_50;
  extractor.setFeatures(&features_50);
  extractor.compute(image_intensity_50);
  ASSERT_EQ(features_50.size(), 261);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_50);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, few matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 120);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }
}

TEST_F(ICL, 00To50_CorrespondenceFinder_ProjectiveKDTree_IdentityEstimate) {
  IntensityFeatureExtractorBinned2D extractor;
  extractor.param_target_number_of_keypoints.setValue(500);
  extractor.param_detector_threshold.setValue(5);

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveKDTree2D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(10);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide pose estimate
  correspondence_finder.setEstimate(Isometry3f::Identity());

  // ds extract features
  PointIntensityDescriptor2fVectorCloud features_50;
  extractor.setFeatures(&features_50);
  extractor.compute(image_intensity_50);
  ASSERT_EQ(features_50.size(), 261);

  // ds narrow maximum search radius
  correspondence_finder.param_maximum_search_radius_pixels.setValue(10);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_50);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, many matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 2);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }

  // ds increase maximum search radius
  correspondence_finder.param_maximum_search_radius_pixels.setValue(100);

  // ds match features
  correspondence_finder.setFixed(&features_50);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, less matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 21);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }
}

TEST_F(KITTI, 00To01_ProjectiveKDTree_PerfectEstimate) {
  IntensityFeatureExtractorBase3DPtr extractor = adaptor->param_feature_extractor.value();

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveKDTree3D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_minimum_search_radius_pixels.setValue(10);
  correspondence_finder.param_maximum_search_radius_pixels.setValue(10);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(1000);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide accuracte pose estimate
  correspondence_finder.setEstimate(camera_01_from_00);

  // ds extract features
  PointIntensityDescriptor3fVectorCloud features_01;
  extractor->setFeatures(&features_01);
  extractor->setProjections(&points_00_in_image_01,
                            correspondence_finder.param_maximum_search_radius_pixels.value());
  extractor->compute(images_intensity_left[1]);
  ASSERT_EQ(features_01.size(), 458);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_01);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, few matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 82);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }
}

TEST_F(KITTI, 00To01_ProjectiveBF_PerfectEstimate) {
  IntensityFeatureExtractorBase3DPtr extractor = adaptor->param_feature_extractor.value();

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveCircle3D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_minimum_search_radius_pixels.setValue(10);
  correspondence_finder.param_maximum_search_radius_pixels.setValue(10);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(1000);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide accuracte pose estimate
  correspondence_finder.setEstimate(camera_01_from_00);

  // ds extract features
  PointIntensityDescriptor3fVectorCloud features_01;
  extractor->setFeatures(&features_01);
  extractor->setProjections(&points_00_in_image_01,
                            correspondence_finder.param_maximum_search_radius_pixels.value());
  extractor->compute(images_intensity_left[1]);
  ASSERT_EQ(features_01.size(), 458);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_01);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, few matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 90);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }
}

TEST_F(KITTI, 00To01_ProjectiveKDTree_IdentityEstimate) {
  IntensityFeatureExtractorBase3DPtr extractor = adaptor->param_feature_extractor.value();

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveKDTree3D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(1000);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide pose estimate
  correspondence_finder.setEstimate(Isometry3f::Identity());

  // ds narrow search radius
  correspondence_finder.param_maximum_search_radius_pixels.setValue(10);
  correspondence_finder.param_minimum_search_radius_pixels.setValue(10);

  // ds extract features
  PointIntensityDescriptor3fVectorCloud features_01;
  extractor->setFeatures(&features_01);
  extractor->setProjections(&points_00_in_image_00,
                            correspondence_finder.param_maximum_search_radius_pixels.value());
  extractor->compute(images_intensity_left[1]);
  ASSERT_EQ(features_01.size(), 458);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_01);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, many matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 36);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }

  // ds increase search radius
  correspondence_finder.param_maximum_search_radius_pixels.setValue(100);
  correspondence_finder.param_minimum_search_radius_pixels.setValue(100);

  // ds match features
  correspondence_finder.setFixed(&features_01);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, few matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 104);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }
}

TEST_F(KITTI, 00To02_ProjectiveKDTree_PerfectEstimate) {
  IntensityFeatureExtractorBase3DPtr extractor = adaptor->param_feature_extractor.value();

  // ds configure a 2d-2d bruteforce correspondence finder
  CorrespondenceFinderProjectiveKDTree3D3D correspondence_finder;
  correspondence_finder.param_maximum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_minimum_descriptor_distance.setValue(50.0f);
  correspondence_finder.param_maximum_distance_ratio_to_second_best.setValue(0.9);
  correspondence_finder.param_maximum_search_radius_pixels.setValue(10);
  correspondence_finder.param_minimum_search_radius_pixels.setValue(10);
  correspondence_finder.param_projector->param_canvas_cols.setValue(image_cols);
  correspondence_finder.param_projector->param_canvas_rows.setValue(image_rows);
  correspondence_finder.param_projector->param_range_min.setValue(0.1);
  correspondence_finder.param_projector->param_range_max.setValue(1000);
  correspondence_finder.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide accuracte pose estimate
  correspondence_finder.setEstimate(camera_02_from_00);

  // ds extract features
  PointIntensityDescriptor3fVectorCloud features_02;
  extractor->setFeatures(&features_02);
  extractor->setProjections(&points_00_in_image_02,
                            correspondence_finder.param_maximum_search_radius_pixels.value());
  extractor->compute(images_intensity_left[2]);
  ASSERT_EQ(features_02.size(), 444);

  // ds match features
  CorrespondenceVector correspondences;
  correspondence_finder.setFixed(&features_02);
  correspondence_finder.setMoving(&points_in_camera_00);
  correspondence_finder.setCorrespondences(&correspondences);
  correspondence_finder.compute();

  // ds validate matching result, few matches lie out of the field of view
  ASSERT_EQ(correspondences.size(), 56);
  for (const Correspondence& correspondence : correspondences) {
    ASSERT_LE(correspondence.response, 50.0f);
  }
}

TEST_F(KITTI, BruteforceVersusProjectiveBruteforce) {
  // ds configure correspondence finders
  constexpr float maximum_descriptor_distance = 50;
  constexpr float maximum_lowes_ratio         = 0.5;
  CorrespondenceFinderDescriptorBasedBruteforce4D3D cf_bruteforce;
  cf_bruteforce.param_maximum_descriptor_distance.setValue(maximum_descriptor_distance);
  cf_bruteforce.param_maximum_distance_ratio_to_second_best.setValue(maximum_lowes_ratio);
  CorrespondenceFinderProjectiveCircle4D3D cf_projective;
  cf_projective.param_maximum_descriptor_distance.setValue(maximum_descriptor_distance);
  cf_projective.param_minimum_descriptor_distance.setValue(maximum_descriptor_distance);
  cf_projective.param_maximum_distance_ratio_to_second_best.setValue(maximum_lowes_ratio);
  cf_projective.param_projector->param_canvas_cols.setValue(image_cols);
  cf_projective.param_projector->param_canvas_rows.setValue(image_rows);
  cf_projective.param_projector->param_range_min.setValue(0.1);
  cf_projective.param_projector->param_range_max.setValue(1000);
  cf_projective.param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds provide accuracte pose estimate (no effect for BF)
  cf_bruteforce.setEstimate(camera_01_from_00);
  cf_projective.setEstimate(camera_01_from_00);

  // ds match features with bruteforce
  CorrespondenceVector correspondences_bruteforce;
  cf_bruteforce.setFixed(&measurements[1]);
  cf_bruteforce.setMoving(&points_in_camera_00);
  cf_bruteforce.setCorrespondences(&correspondences_bruteforce);
  cf_bruteforce.compute();

  // ds evaluate different search radii
  for (size_t search_radius_pixels = 5; search_radius_pixels < 1000; search_radius_pixels *= 5) {
    cf_projective.param_minimum_search_radius_pixels.setValue(search_radius_pixels);
    cf_projective.param_maximum_search_radius_pixels.setValue(search_radius_pixels);

    // ds match features with projective cf
    CorrespondenceVector correspondences_projective;
    cf_projective.setFixed(&measurements[1]);
    cf_projective.setMoving(&points_in_camera_00);
    cf_projective.setCorrespondences(&correspondences_projective);
    cf_projective.compute();

    // ds validate matching result
    size_t number_of_gt_correspondences = 0;
    size_t number_of_bf_correspondences = 0;
    for (const Correspondence& correspondence_projective : correspondences_projective) {
      for (const Correspondence& correspondence_gt : correspondences_camera_01_from_00) {
        if (correspondence_projective.fixed_idx == correspondence_gt.fixed_idx) {
          if (correspondence_projective.moving_idx == correspondence_gt.moving_idx) {
            ++number_of_gt_correspondences;
          }
        }
      }
      for (const Correspondence& correspondence_bruteforce : correspondences_bruteforce) {
        if (correspondence_projective.fixed_idx == correspondence_bruteforce.fixed_idx) {
          if (correspondence_projective.moving_idx == correspondence_bruteforce.moving_idx) {
            ++number_of_bf_correspondences;
          }
        }
      }
    }

    // ds we require minimum ratios of consistent correspondences for all radii
    const float matching_bf_correspondence_ratio =
      static_cast<float>(number_of_bf_correspondences) / correspondences_projective.size();
    const float matching_gt_correspondence_ratio =
      static_cast<float>(number_of_gt_correspondences) / correspondences_projective.size();
    std::cerr << "#correspondences PR: " << correspondences_projective.size();
    std::cerr << " BF: " << correspondences_bruteforce.size()
              << " (overlap: " << matching_bf_correspondence_ratio << ")";
    std::cerr << " GT: " << correspondences_camera_01_from_00.size()
              << " (overlap: " << matching_gt_correspondence_ratio << ")" << std::endl;
    ASSERT_GT(matching_gt_correspondence_ratio, 0.6);
    ASSERT_GT(matching_bf_correspondence_ratio, 0.7);
  }
}

TEST_F(SyntheticWorldWithDescriptorsSE3, Bruteforce_NoMotionNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderDescriptorBasedBruteforce2D3D finder;
  finder.param_maximum_descriptor_distance.setValue(50);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);

  // ds set sensor poses
  finder.setEstimate(Isometry3f::Identity());

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (const Correspondence& correspondence : correspondences_sensor_to_world[0]) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    points_in_world[correspondence.moving_idx].descriptor()    = descriptor;
    points_in_sensor[0][correspondence.fixed_idx].descriptor() = descriptor;
  }
  for (const Correspondence& correspondence : correspondences_canvas_to_sensor[0]) {
    points_in_sensor_projected_in_canvas[0][correspondence.fixed_idx].descriptor() =
      points_in_sensor[0][correspondence.moving_idx].descriptor();
  }

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[0]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), points_in_sensor_projected_in_canvas[0].size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_to_sensor[0]) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}

TEST_F(SyntheticWorldWithDescriptorsSE3, ProjectiveKDTree_NoMotionNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree2D3D finder;
  finder.param_maximum_descriptor_distance.setValue(75);
  finder.param_minimum_descriptor_distance.setValue(25);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_maximum_search_radius_pixels.setValue(1);
  finder.param_minimum_number_of_points_per_cluster.setValue(10);
  finder.param_projector->param_canvas_cols.setValue(canvas_size(0));
  finder.param_projector->param_canvas_rows.setValue(canvas_size(1));
  finder.param_projector->param_range_min.setValue(0.1);
  finder.param_projector->param_range_max.setValue(1000);
  finder.param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  finder.setEstimate(Isometry3f::Identity());

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (const Correspondence& correspondence : correspondences_sensor_to_world[0]) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    points_in_world[correspondence.moving_idx].descriptor()    = descriptor;
    points_in_sensor[0][correspondence.fixed_idx].descriptor() = descriptor;
  }
  for (const Correspondence& correspondence : correspondences_canvas_to_sensor[0]) {
    points_in_sensor_projected_in_canvas[0][correspondence.fixed_idx].descriptor() =
      points_in_sensor[0][correspondence.moving_idx].descriptor();
  }

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[0]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), points_in_sensor_projected_in_canvas[0].size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_to_sensor[0]) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}

TEST_F(SyntheticWorldWithDescriptorsSE3, ProjectiveKDTree_TranslationNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree2D3D finder;
  finder.param_maximum_descriptor_distance.setValue(75);
  finder.param_minimum_descriptor_distance.setValue(25);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_maximum_search_radius_pixels.setValue(1);
  finder.param_minimum_number_of_points_per_cluster.setValue(10);
  finder.param_projector->param_canvas_cols.setValue(canvas_size(0));
  finder.param_projector->param_canvas_rows.setValue(canvas_size(1));
  finder.param_projector->param_range_min.setValue(0.1);
  finder.param_projector->param_range_max.setValue(1000);
  finder.param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  Isometry3f pose(Isometry3f::Identity());
  sensor_poses.push_back(pose);
  pose.translation() += Vector3f(0, 0, -1);
  sensor_poses.push_back(pose);
  finder.setEstimate(pose /*perfect initial guess*/);

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate world points with unique random descriptors
  for (PointIntensityDescriptor3f& point_in_world : points_in_world) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    point_in_world.descriptor() = descriptor;
  }

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (size_t i = 0; i < 2; ++i) {
    for (const Correspondence& correspondence : correspondences_sensor_to_world[i]) {
      points_in_sensor[i][correspondence.fixed_idx].descriptor() =
        points_in_world[correspondence.moving_idx].descriptor();
    }
    for (const Correspondence& correspondence : correspondences_canvas_to_sensor[i]) {
      points_in_sensor_projected_in_canvas[i][correspondence.fixed_idx].descriptor() =
        points_in_sensor[i][correspondence.moving_idx].descriptor();
    }
  }

  // ds check for correspondences to the first pose
  CorrespondenceVector correspondences_canvas_1_to_sensor_0;
  correspondences_canvas_1_to_sensor_0.reserve(points_in_sensor_projected_in_canvas[1].size());
  for (const Correspondence& correspondence_1 : correspondences_canvas_to_world[1]) {
    for (const Correspondence& correspondence_0 : correspondences_sensor_to_world[0]) {
      if (correspondence_1.moving_idx == correspondence_0.moving_idx) {
        correspondences_canvas_1_to_sensor_0.emplace_back(
          Correspondence(correspondence_1.fixed_idx, correspondence_0.fixed_idx));
      }
    }
  }
  ASSERT_EQ(correspondences_canvas_1_to_sensor_0.size(), 98);

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[1]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), correspondences_canvas_1_to_sensor_0.size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_1_to_sensor_0) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}

TEST_F(SyntheticWorldWithDescriptorsSE3, ProjectiveKDTree_RotationNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree2D3D finder;
  finder.param_maximum_descriptor_distance.setValue(75);
  finder.param_minimum_descriptor_distance.setValue(25);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_maximum_search_radius_pixels.setValue(1);
  finder.param_minimum_number_of_points_per_cluster.setValue(10);
  finder.param_projector->param_canvas_cols.setValue(canvas_size(0));
  finder.param_projector->param_canvas_rows.setValue(canvas_size(1));
  finder.param_projector->param_range_min.setValue(0.1);
  finder.param_projector->param_range_max.setValue(1000);
  finder.param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  Isometry3f pose(Isometry3f::Identity());
  sensor_poses.push_back(pose);
  pose.rotate(AngleAxisf(0.25 * M_PI, Vector3f::UnitX()));
  sensor_poses.push_back(pose);
  finder.setEstimate(pose /*perfect initial guess*/);

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate world points with unique random descriptors
  for (PointIntensityDescriptor3f& point_in_world : points_in_world) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    point_in_world.descriptor() = descriptor;
  }

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (size_t i = 0; i < 2; ++i) {
    for (const Correspondence& correspondence : correspondences_sensor_to_world[i]) {
      points_in_sensor[i][correspondence.fixed_idx].descriptor() =
        points_in_world[correspondence.moving_idx].descriptor();
    }
    for (const Correspondence& correspondence : correspondences_canvas_to_sensor[i]) {
      points_in_sensor_projected_in_canvas[i][correspondence.fixed_idx].descriptor() =
        points_in_sensor[i][correspondence.moving_idx].descriptor();
    }
  }

  // ds check for correspondences to the first pose
  CorrespondenceVector correspondences_canvas_1_to_sensor_0;
  correspondences_canvas_1_to_sensor_0.reserve(points_in_sensor_projected_in_canvas[1].size());
  for (const Correspondence& correspondence_1 : correspondences_canvas_to_world[1]) {
    for (const Correspondence& correspondence_0 : correspondences_sensor_to_world[0]) {
      if (correspondence_1.moving_idx == correspondence_0.moving_idx) {
        correspondences_canvas_1_to_sensor_0.emplace_back(
          Correspondence(correspondence_1.fixed_idx, correspondence_0.fixed_idx));
      }
    }
  }
  ASSERT_EQ(correspondences_canvas_1_to_sensor_0.size(), 48);

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[1]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), correspondences_canvas_1_to_sensor_0.size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_1_to_sensor_0) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}

TEST_F(SyntheticWorldWithDescriptorsSE3, ProjectiveSquare_NoMotionNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveSquare2D3D finder;
  finder.param_minimum_descriptor_distance.setValue(25);
  finder.param_maximum_descriptor_distance.setValue(75);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_minimum_search_radius_pixels.setValue(1);
  finder.param_projector->param_canvas_cols.setValue(canvas_size(0));
  finder.param_projector->param_canvas_rows.setValue(canvas_size(1));
  finder.param_projector->param_range_min.setValue(0.1);
  finder.param_projector->param_range_max.setValue(1000);
  finder.param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  finder.setEstimate(Isometry3f::Identity());

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (const Correspondence& correspondence : correspondences_sensor_to_world[0]) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    points_in_world[correspondence.moving_idx].descriptor()    = descriptor;
    points_in_sensor[0][correspondence.fixed_idx].descriptor() = descriptor;
  }
  for (const Correspondence& correspondence : correspondences_canvas_to_sensor[0]) {
    points_in_sensor_projected_in_canvas[0][correspondence.fixed_idx].descriptor() =
      points_in_sensor[0][correspondence.moving_idx].descriptor();
  }

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[0]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), points_in_sensor_projected_in_canvas[0].size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_to_sensor[0]) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}

TEST_F(SyntheticWorldWithDescriptorsSE3, ProjectiveCircle_NoMotionNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveCircle2D3D finder;
  finder.param_minimum_descriptor_distance.setValue(25);
  finder.param_maximum_descriptor_distance.setValue(75);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_minimum_search_radius_pixels.setValue(2 /*precision TODO verify*/);
  finder.param_maximum_search_radius_pixels.setValue(2 /*precision TODO verify*/);
  finder.param_projector->param_canvas_cols.setValue(canvas_size(0));
  finder.param_projector->param_canvas_rows.setValue(canvas_size(1));
  finder.param_projector->param_range_min.setValue(0.1);
  finder.param_projector->param_range_max.setValue(1000);
  finder.param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  finder.setEstimate(Isometry3f::Identity());

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (const Correspondence& correspondence : correspondences_sensor_to_world[0]) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    points_in_world[correspondence.moving_idx].descriptor()    = descriptor;
    points_in_sensor[0][correspondence.fixed_idx].descriptor() = descriptor;
  }
  for (const Correspondence& correspondence : correspondences_canvas_to_sensor[0]) {
    points_in_sensor_projected_in_canvas[0][correspondence.fixed_idx].descriptor() =
      points_in_sensor[0][correspondence.moving_idx].descriptor();
  }

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[0]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), points_in_sensor_projected_in_canvas[0].size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_to_sensor[0]) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}

TEST_F(SyntheticWorldWithDescriptorsSE3, ProjectiveRhombus_NoMotionNoNoise) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveRhombus2D3D finder;
  finder.param_minimum_descriptor_distance.setValue(25);
  finder.param_maximum_descriptor_distance.setValue(75);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_minimum_search_radius_pixels.setValue(2 /*precision TODO verify*/);
  finder.param_maximum_search_radius_pixels.setValue(2 /*precision TODO verify*/);
  finder.param_projector->param_canvas_cols.setValue(canvas_size(0));
  finder.param_projector->param_canvas_rows.setValue(canvas_size(1));
  finder.param_projector->param_range_min.setValue(0.1);
  finder.param_projector->param_range_max.setValue(1000);
  finder.param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  finder.setEstimate(Isometry3f::Identity());

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 99);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 99);

  // ds populate corresponding points with matching, random 32-bit descriptors
  for (const Correspondence& correspondence : correspondences_sensor_to_world[0]) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    points_in_world[correspondence.moving_idx].descriptor()    = descriptor;
    points_in_sensor[0][correspondence.fixed_idx].descriptor() = descriptor;
  }
  for (const Correspondence& correspondence : correspondences_canvas_to_sensor[0]) {
    points_in_sensor_projected_in_canvas[0][correspondence.fixed_idx].descriptor() =
      points_in_sensor[0][correspondence.moving_idx].descriptor();
  }

  // ds match features
  CorrespondenceVector correspondences;
  finder.setFixed(&points_in_sensor_projected_in_canvas[0]);
  finder.setMoving(&points_in_sensor[0]);
  finder.setCorrespondences(&correspondences);
  finder.compute();

  // ds validate matching result
  ASSERT_EQ(correspondences.size(), points_in_sensor_projected_in_canvas[0].size());
  size_t number_of_correct_correspondences = 0;
  for (Correspondence& correspondence : correspondences) {
    for (Correspondence& correspondence_truth : correspondences_canvas_to_sensor[0]) {
      if (correspondence.fixed_idx == correspondence_truth.fixed_idx) {
        ASSERT_EQ(correspondence.moving_idx, correspondence_truth.moving_idx);
        ++number_of_correct_correspondences;
      }
    }
  }
  ASSERT_EQ(number_of_correct_correspondences, correspondences.size());
}
