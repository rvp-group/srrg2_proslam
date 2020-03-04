#include <iostream>

#include "srrg2_proslam_adaptation/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_proslam;

const char* banner[] = {"This program computes a set of keypoints for the provided image using "
                        "individual detectors for image regions",
                        0};

int main(int argc_, char** argv_) {
  srrg2_proslam_adaptation_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString filepath_image_query(
      &command_line_parser, "m", "moving-image",
                            "file path for test moving image", "");
  ArgumentString filepath_image_reference(
      &command_line_parser, "f", "fixed-image",
                            "file path for test fixed image (measurement)", "");
  ArgumentString filepath_image_depth(
      &command_line_parser, "d", "depth-image",
                            "file path for test depth image (projective matching) for moving", "");
  ArgumentInt target_number_of_keypoints(
      &command_line_parser, "n", "number-of-keypoints",
                            "number of keypoints to detect", 100);
  ArgumentString name_search_method(
      &command_line_parser, "a", "algorithm",
                            "target matching algorithm: {bruteforce, epipolar, projective}",
                            "bruteforce");
  ArgumentInt parameter_detector_threshold(
      &command_line_parser, "dt", "detector-threshold",
                            "desired keypoint detector threshold",
                            10);
  ArgumentFlag flag_disable_non_maximum_suppression(
      &command_line_parser, "dn", "disable-non-maximum-suppression",
                            "disable non maximum suppression for feature detection");
  ArgumentFlag flag_enable_point_selection(
      &command_line_parser, "es", "enable-selection",
                            "enables feature selection and tracking");
  ArgumentInt parameter_target_bin_width_pixels(
      &command_line_parser, "bw", "bin-width",
                            "target bin width (pixels)",
                            10);
  ArgumentFlag flag_enable_demosaicing(
      &command_line_parser, "ed", "enable-demosaicing",
                            "trigger demosaicing of (e.g. bayer encoded) image input");

  // clang-format on
  command_line_parser.parse();
  if (!filepath_image_query.isSet() || filepath_image_query.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!filepath_image_reference.isSet() || filepath_image_reference.value().empty()) {
    printBanner(banner);
    return 0;
  }

  // ds read opencv images from disk
  cv::Mat image_moving;
  cv::Mat image_fixed;
  if (flag_enable_demosaicing.isSet()) {
    image_moving = cv::imread(filepath_image_query.value(), CV_LOAD_IMAGE_UNCHANGED);
    image_fixed  = cv::imread(filepath_image_reference.value(), CV_LOAD_IMAGE_UNCHANGED);
    cv::cvtColor(image_moving, image_moving, CV_BayerGB2GRAY);
    cv::cvtColor(image_fixed, image_fixed, CV_BayerGB2GRAY);
  } else {
    image_moving = cv::imread(filepath_image_query.value(), CV_LOAD_IMAGE_GRAYSCALE);
    image_fixed  = cv::imread(filepath_image_reference.value(), CV_LOAD_IMAGE_GRAYSCALE);
  }
  cv::Mat image_depth_moving;
  if (filepath_image_depth.isSet()) {
    image_depth_moving = cv::imread(filepath_image_depth.value(), CV_LOAD_IMAGE_ANYDEPTH);
  }

  // ds instantiate feature extraction units
  IntensityFeatureExtractorBase3DPtr extractor_fixed;
  IntensityFeatureExtractorBase3DPtr extractor_moving;
  if (flag_enable_point_selection.isSet()) {
    extractor_fixed =
      IntensityFeatureExtractorSelective3DPtr(new IntensityFeatureExtractorSelective3D());
    extractor_fixed->param_detector_type.setValue("GFTT");
    extractor_fixed->param_target_bin_width_pixels.setValue(
      parameter_target_bin_width_pixels.value());
    IntensityFeatureExtractorSelective3DPtr extractor_moving_selective(
      new IntensityFeatureExtractorSelective3D());
    extractor_moving_selective->param_detector_type.setValue("GFTT");
    extractor_moving_selective->param_enable_full_distance_to_left.setValue(true);
    extractor_moving_selective->param_enable_full_distance_to_right.setValue(false);
    extractor_moving_selective->param_enable_seeding_when_tracking.setValue(false);
    extractor_moving_selective->param_enable_non_maximum_suppression.setValue(
      !flag_disable_non_maximum_suppression.isSet());
    extractor_moving = extractor_moving_selective;
  } else {
    IntensityFeatureExtractorBinned3DPtr extractor_fixed_binned(
      new IntensityFeatureExtractorBinned3D());
    extractor_fixed_binned->param_number_of_detectors_horizontal.setValue(3);
    extractor_fixed_binned->param_number_of_detectors_vertical.setValue(3);
    extractor_fixed = extractor_fixed_binned;
    extractor_fixed->param_detector_type.setValue("FAST");
    IntensityFeatureExtractorBinned3DPtr extractor_moving_binned(
      new IntensityFeatureExtractorBinned3D());
    extractor_moving_binned->param_number_of_detectors_horizontal.setValue(3);
    extractor_moving_binned->param_number_of_detectors_vertical.setValue(3);
    extractor_moving = extractor_moving_binned;
    extractor_moving->param_detector_type.setValue("FAST");
  }

  // ds common parameters
  extractor_fixed->param_target_number_of_keypoints.setValue(target_number_of_keypoints.value());
  extractor_fixed->param_detector_threshold.setValue(parameter_detector_threshold.value());
  extractor_moving->param_target_number_of_keypoints.setValue(target_number_of_keypoints.value());
  extractor_moving->param_detector_threshold.setValue(parameter_detector_threshold.value());

  // ds feature vectors to fill
  PointIntensityDescriptor3fVectorCloud features_moving;
  PointIntensityDescriptor3fVectorCloud features_fixed;

  // ds detect keypoints in both images using the extraction unit
  SystemUsageCounter::tic();
  extractor_fixed->setFeatures(&features_fixed);
  extractor_fixed->compute(image_fixed);
  double duration_seconds = SystemUsageCounter::toc();
  std::cerr << "image [FIXED]: " << filepath_image_reference.value() << std::endl;
  std::cerr << "  # target keypoints: " << extractor_fixed->param_target_number_of_keypoints.value()
            << std::endl;
  std::cerr << "  # extracted features: " << features_fixed.size() << std::endl;
  std::cerr << "  extraction duration (s): " << duration_seconds
            << " (Hz): " << 1 / duration_seconds << std::endl;
  SystemUsageCounter::tic();
  extractor_moving->setFeatures(&features_moving);
  extractor_moving->setProjections(&features_fixed, 0);
  extractor_moving->compute(image_moving);
  duration_seconds = SystemUsageCounter::toc();
  std::cerr << "image [MOVING]: " << filepath_image_query.value() << std::endl;
  std::cerr << "  # target keypoints: "
            << extractor_moving->param_target_number_of_keypoints.value() << std::endl;
  std::cerr << "  # extracted features: " << features_moving.size() << std::endl;
  std::cerr << "  extraction duration (s): " << duration_seconds
            << " (Hz): " << 1 / duration_seconds << std::endl;

  // ds instantiate the corresponding feature matcher
  std::shared_ptr<CorrespondenceFinder_<Isometry3f,
                                        PointIntensityDescriptor3fVectorCloud,
                                        PointIntensityDescriptor3fVectorCloud>>
    matcher;
  if (name_search_method.value() == "bruteforce") {
    CorrespondenceFinderDescriptorBasedBruteforce3D3DPtr matcher_bruteforce(
      new CorrespondenceFinderDescriptorBasedBruteforce3D3D());
    matcher_bruteforce->param_maximum_descriptor_distance.setValue(75);
    matcher_bruteforce->param_maximum_distance_ratio_to_second_best.setValue(0.8);
    matcher = matcher_bruteforce;
  } else if (name_search_method.value() == "epipolar") {
    CorrespondenceFinderDescriptorBasedEpipolar3D3DPtr matcher_epipolar(
      new CorrespondenceFinderDescriptorBasedEpipolar3D3D());
    matcher_epipolar->param_maximum_descriptor_distance.setValue(75);
    matcher_epipolar->param_maximum_distance_ratio_to_second_best.setValue(0.8);
    matcher = matcher_epipolar;
  } else {
    std::cerr << "ERROR: unknown search method: " << name_search_method.value() << std::endl;
    return 0;
  }
  assert(matcher);

  // ds match features
  CorrespondenceVector matches;
  SystemUsageCounter::tic();
  matcher->setFixed(&features_fixed);
  matcher->setMoving(&features_moving);
  matcher->setCorrespondences(&matches);
  matcher->compute();
  duration_seconds = SystemUsageCounter::toc();
  std::cerr << "  # matches: " << matches.size() << std::endl;
  std::cerr << "  matching duration (s): " << duration_seconds << " (Hz): " << 1 / duration_seconds
            << std::endl;

  // ds draw detected keypoints
  const cv::Point2f offset_to_bottom_image(0, image_moving.rows);
  cv::Mat display_image;
  if (image_depth_moving.rows > 0 && image_depth_moving.cols > 0) {
    cv::Mat image_depth_display;
    image_depth_moving.convertTo(image_depth_display, CV_8UC1, 0.03 /*hacky visibility scaling*/);
    cv::vconcat(image_moving, image_depth_display, display_image);
  } else {
    cv::vconcat(image_moving, image_fixed, display_image);
  }
  cv::cvtColor(display_image, display_image, cv::COLOR_GRAY2BGR);

  // ds draw detected points
  for (const PointIntensityDescriptor3f& keypoint : features_moving) {
    cv::circle(display_image, TO_OPENCV_2F(keypoint.coordinates()), 2, cv::Scalar(255, 0, 0), -1);
  }
  for (const PointIntensityDescriptor3f& keypoint : features_fixed) {
    cv::circle(display_image, TO_OPENCV_2F(keypoint.coordinates()), 3, cv::Scalar(0, 0, 255), 1);
    cv::circle(display_image,
               TO_OPENCV_2F(keypoint.coordinates()) + offset_to_bottom_image,
               2,
               cv::Scalar(255, 0, 0),
               -1);
    cv::circle(display_image,
               TO_OPENCV_2F(keypoint.coordinates()) + offset_to_bottom_image,
               25,
               cv::Scalar(0, 0, 255),
               1);
    cv::putText(display_image,
                std::to_string(size_t(keypoint.coordinates()(1))) + "/" +
                  std::to_string(size_t(keypoint.coordinates()(0))),
                TO_OPENCV_2F(keypoint.coordinates()) + offset_to_bottom_image,
                cv::FONT_HERSHEY_PLAIN,
                0.8,
                cv::Scalar(0, 0, 255));
  }

  // ds if available - draw the detector regions
  if (!flag_enable_point_selection.isSet()) {
    IntensityFeatureExtractorBinned3DPtr extractor =
      std::dynamic_pointer_cast<IntensityFeatureExtractorBinned3D>(extractor_fixed);
    const std::vector<cv::Rect>& detection_regions(extractor->detectionRegions());
    for (const cv::Rect& region : detection_regions) {
      // ds slight overlap of the detector regions is desired!
      cv::rectangle(display_image, region, cv::Scalar(255, 255, 255));
      cv::Rect region_reference(region);
      region_reference.y += offset_to_bottom_image.y;
      cv::rectangle(display_image, region_reference, cv::Scalar(255, 255, 255));
    }
  }

  // ds draw the matches and compute a crappy histogram of vertical distances
  size_t number_of_vertical_distances_0             = 0;
  size_t number_of_vertical_distances_1             = 0;
  size_t number_of_vertical_distances_2             = 0;
  size_t number_of_vertical_distances_3_and_greater = 0;
  for (const Correspondence& match : matches) {
    cv::line(display_image,
             TO_OPENCV_2F(features_moving[match.moving_idx].coordinates()),
             TO_OPENCV_2F(features_fixed[match.fixed_idx].coordinates()) + offset_to_bottom_image,
             cv::Scalar(0, 255, 0),
             1);

    // ds count vertical coordinate offset (only informative for horizontal stereo)
    const size_t vertical_delta_pixels =
      std::fabs(features_moving[match.moving_idx].coordinates()(1) -
                features_fixed[match.fixed_idx].coordinates()(1));
    switch (vertical_delta_pixels) {
      case 0: {
        ++number_of_vertical_distances_0;
        break;
      }
      case 1: {
        ++number_of_vertical_distances_1;
        break;
      }
      case 2: {
        ++number_of_vertical_distances_2;
        break;
      }
      default: {
        ++number_of_vertical_distances_3_and_greater;
        break;
      }
    }
  }
  std::cerr << "number_of_vertical_distances_0: " << number_of_vertical_distances_0 << std::endl;
  std::cerr << "number_of_vertical_distances_1: " << number_of_vertical_distances_1 << std::endl;
  std::cerr << "number_of_vertical_distances_2: " << number_of_vertical_distances_2 << std::endl;
  std::cerr << "number_of_vertical_distances_3_and_greater: "
            << number_of_vertical_distances_3_and_greater << std::endl;
  cv::imshow("feature_matching (top: moving, bot: fixed)", display_image);
  cv::waitKey(0);
  return 0;
}
