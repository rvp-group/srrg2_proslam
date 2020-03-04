#include <iostream>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg2_proslam_adaptation/instances.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_proslam;

const char* banner[] = {"This program computes a set of keypoints for the provided image using "
                        "individual detectors for image regions",
                        0};

int main(int argc_, char** argv_) {
  srrg2_proslam_adaptation_registerTypes();
  Profiler::enable_logging = true;

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_image_moving(
      &command_line_parser, "m", "moving-image",
                            "file path for test moving image", "");
  ArgumentString argument_image_fixed(
      &command_line_parser, "f", "fixed-image",
                            "file path for test fixed image (measurement)", "");
  ArgumentInt target_number_of_keypoints(
      &command_line_parser, "n", "number-of-keypoints",
                            "number of keypoints to detect", 100);
  ArgumentInt parameter_detector_threshold(
      &command_line_parser, "dt", "detector-threshold",
                            "desired keypoint detector threshold",
                            10);
  ArgumentInt parameter_target_bin_width_pixels(
      &command_line_parser, "bw", "bin-width",
                            "target bin width (pixels)",
                            10);
  ArgumentFlag flag_enable_point_seeding(
      &command_line_parser, "es", "enable-seeding",
                            "enables feature seeding and tracking");
  ArgumentFlag flag_enable_detection_left(
      &command_line_parser, "el", "enable-left",
                            "enables feature detection to the left image border");
  ArgumentFlag flag_enable_detection_right(
      &command_line_parser, "er", "enable-right",
                            "enables feature detection to the right image border");
  constexpr size_t number_of_iterations = 20;
  // clang-format on

  command_line_parser.parse();
  std::string filepath_image_fixed(argument_image_fixed.value());
  std::string filepath_image_moving(argument_image_moving.value());
  if (!argument_image_fixed.isSet() || filepath_image_fixed.empty()) {
    if (argument_image_moving.isSet() && !filepath_image_moving.empty()) {
      filepath_image_fixed = filepath_image_moving;
    } else {
      printBanner(banner);
      return 0;
    }
  }
  if (!argument_image_moving.isSet() || filepath_image_moving.empty()) {
    if (argument_image_fixed.isSet() && !filepath_image_fixed.empty()) {
      filepath_image_moving = filepath_image_fixed;
    } else {
      printBanner(banner);
      return 0;
    }
  }

  // ds read opencv images from disk
  std::cerr << "reading image FIXED: " << filepath_image_fixed << std::endl;
  const cv::Mat image_moving(cv::imread(filepath_image_fixed, CV_LOAD_IMAGE_GRAYSCALE));
  std::cerr << "reading image MOVING: " << filepath_image_moving << std::endl;
  const cv::Mat image_fixed(cv::imread(filepath_image_moving, CV_LOAD_IMAGE_GRAYSCALE));

  // ds instantiate a feature extraction unit
  IntensityFeatureExtractorBase3DPtr extractor(nullptr);
  if (flag_enable_point_seeding.isSet()) {
    IntensityFeatureExtractorSelective3DPtr extractor_selective(
      new IntensityFeatureExtractorSelective3D());
    extractor_selective->param_enable_full_distance_to_left.setValue(
      flag_enable_detection_left.isSet());
    extractor_selective->param_enable_full_distance_to_right.setValue(
      flag_enable_detection_right.isSet());
    extractor = extractor_selective;
  } else {
    extractor = IntensityFeatureExtractorBinned3DPtr(new IntensityFeatureExtractorBinned3D());
  }
  assert(extractor);
  extractor->param_target_number_of_keypoints.setValue(target_number_of_keypoints.value());
  extractor->param_detector_threshold.setValue(parameter_detector_threshold.value());
  extractor->param_target_bin_width_pixels.setValue(parameter_target_bin_width_pixels.value());

  // ds feature vectors to fill
  PointIntensityDescriptor3fVectorCloud features_moving;
  PointIntensityDescriptor3fVectorCloud features_fixed;

  // ds detect keypoints in the first image (seeding)
  SystemUsageCounter::tic();
  extractor->setFeatures(&features_moving);
  extractor->compute(image_moving);
  double duration_seconds = SystemUsageCounter::toc();
  std::cerr << "MOVING image (seeding): " << argument_image_moving.value() << std::endl;
  std::cerr << "  # target keypoints: " << target_number_of_keypoints.value() << std::endl;
  std::cerr << "  # extracted features: " << features_moving.size() << std::endl;
  std::cerr << "  extraction duration (s): " << duration_seconds
            << " (Hz): " << 1 / duration_seconds << std::endl;

  // ds detect keypoints in the second image (tracking)
  for (size_t i = 0; i < number_of_iterations; ++i) {
    const size_t detection_radius_pixels(std::round(100.0 / (i + 1)));
    SystemUsageCounter::tic();
    extractor->setFeatures(&features_fixed);
    extractor->setProjections(&features_moving, detection_radius_pixels);
    extractor->compute(image_fixed);
    duration_seconds = SystemUsageCounter::toc();
    std::cerr << "-------------------------------------------------------------------" << std::endl;
    std::cerr << "iteration: " << i << std::endl;
    std::cerr << "FIXED image (tracking): " << argument_image_fixed.value() << std::endl;
    std::cerr << "  search radius (px): " << detection_radius_pixels << std::endl;
    std::cerr << "  # projections (MOVING): " << features_moving.size() << std::endl;
    std::cerr << "  # extracted features: " << features_fixed.size() << std::endl;
    std::cerr << "  extraction duration (s): " << duration_seconds
              << " (Hz): " << 1 / duration_seconds << std::endl;

    // ds draw detected keypoints
    const cv::Point2f offset_to_bottom_image(0, image_fixed.rows);
    cv::Mat display_image;
    cv::vconcat(image_fixed, image_moving, display_image);
    cv::cvtColor(display_image, display_image, cv::COLOR_GRAY2BGR);

    // ds draw detected points
    for (const PointIntensityDescriptor3f& keypoint : features_fixed) {
      cv::circle(display_image, TO_OPENCV_2F(keypoint.coordinates()), 2, cv::Scalar(255, 0, 0), -1);
    }
    for (const PointIntensityDescriptor3f& keypoint : features_moving) {
      cv::circle(display_image,
                 TO_OPENCV_2F(keypoint.coordinates()) + offset_to_bottom_image,
                 2,
                 cv::Scalar(0, 0, 255),
                 -1);
    }

    // ds draw the detector regions for region-based detection
    if (!flag_enable_point_seeding.isSet()) {
      IntensityFeatureExtractorBinned3DPtr binned_extractor(
        std::dynamic_pointer_cast<IntensityFeatureExtractorBinned3D>(extractor));
      const std::vector<cv::Rect>& detection_regions(binned_extractor->detectionRegions());
      for (const cv::Rect& region : detection_regions) {
        // ds slight overlap of the detector regions is desired!
        cv::rectangle(display_image, region, cv::Scalar(255, 255, 255));
        cv::Rect region_reference(region);
        region_reference.y += offset_to_bottom_image.y;
        cv::rectangle(display_image, region_reference, cv::Scalar(255, 255, 255));
      }
    }

    cv::imshow("feature extraction (top: fixed, bot: moving)", display_image);
    if (cv::waitKey(0) == 27 /*ASCII ESC*/) {
      break;
    }

    // ds if the selected algorithm exhibits no change over iterations
    if (!flag_enable_point_seeding.isSet()) {
      // ds stop after one iteration
      break;
    }
  }
  return 0;
}
