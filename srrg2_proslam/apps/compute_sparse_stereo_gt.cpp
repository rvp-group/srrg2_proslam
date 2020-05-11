#include <iostream>

// ds data processing
#include <srrg_messages/instances.h>

// ds helpers
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg2_proslam/sensor_processing/instances.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_proslam;

// clang-format off
const char* banner[] = {"This program computes sparse rigid stereo matching ground truth",
                        "usage: TODO infer from ParseCommandLine",
                        0};
// clang-format on

// ds Portable FloatMap reader TODO move to srrg2_core
ImageFloat readPFM(const std::string& filepath_);

int main(int argc_, char** argv_) {
  srrg2_proslam_sensor_processing_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString filepath_image_left(
    &command_line_parser, "l", "image-left",
                          "file path for left image", "");
  ArgumentString filepath_image_right(
    &command_line_parser, "r", "image-right",
                          "file path for right image", "");
  ArgumentString filepath_image_disparity(
    &command_line_parser, "d", "image-disparity",
                          "file path for disparity image (left to right)", "");
  ArgumentInt detector_threshold(
    &command_line_parser, "t", "detector-threshold",
                          "detector threshold for keypoint detection", 10);
  // clang-format on
  command_line_parser.parse();
  if (!filepath_image_left.isSet() || filepath_image_left.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!filepath_image_right.isSet() || filepath_image_right.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!filepath_image_disparity.isSet() || filepath_image_disparity.value().empty()) {
    printBanner(banner);
    return 0;
  }

  // ds read opencv images from disk
  const cv::Mat image_left(cv::imread(filepath_image_left.value(), CV_LOAD_IMAGE_GRAYSCALE));
  const cv::Mat image_right(cv::imread(filepath_image_right.value(), CV_LOAD_IMAGE_GRAYSCALE));
  const ImageFloat image_disparity(readPFM(filepath_image_disparity.value()));
  if (image_left.rows != image_right.rows || image_left.cols != image_right.cols) {
    std::cerr << "ERROR: image right has inconsistent dimension w.r.t. image left" << std::endl;
    return 0;
  }
  if (static_cast<size_t>(image_left.rows) != image_disparity.rows() ||
      static_cast<size_t>(image_left.cols) != image_disparity.cols()) {
    cv::Mat test;
    image_disparity.toCv(test);
    cv::imshow("test", test);
    cv::waitKey(0);

    std::cerr << "ERROR: disparity image has inconsistent dimension w.r.t. image left" << std::endl;
    return 0;
  }

  // ds instantiate a feature extraction unit
  IntensityFeatureExtractorBase2DPtr feature_extractor_left(
    new IntensityFeatureExtractorBinned2D());

  // ds detect keypoints using the extraction unit
  PointIntensityDescriptor2fVectorCloud keypoints_left;
  feature_extractor_left->computeKeypoints(image_left, keypoints_left);
  std::cerr << " # detected keypoints (LEFT): " << keypoints_left.size() << std::endl;

  // ds out file containing validated stereo matches with disparity values
  std::ofstream result_file("gt_stereo_matching_threshold-" +
                            std::to_string(detector_threshold.value()) + ".txt");

  // ds draw detecte keypoints and the detector grid
  cv::Mat stereo_image;
  cv::vconcat(image_left, image_right, stereo_image);
  cv::cvtColor(stereo_image, stereo_image, cv::COLOR_GRAY2BGR);
  size_t number_of_stereo_points = 0;
  for (const PointIntensityDescriptor2f& keypoint_left : keypoints_left) {
    const Vector2f& coordinates_left(keypoint_left.coordinates());

    // ds retrieve horizontal disparity value for current keypoint
    // ds the cast is safe since the keypoints are detected at pixel precision
    const float disparity = image_disparity.at(coordinates_left.cast<int>());

    // ds skip point if invalid
    if (disparity <= 0) {
      continue;
    }

    // ds compute point coordinates right using the disparity
    const Vector2f coordinates_right = coordinates_left - Vector2f(0, disparity);

    // ds skip point if out of image
    if (coordinates_right.y() < 0) {
      continue;
    }

    // ds dump stereo match to file (ordered ascending in row coordinate)
    result_file << coordinates_left.transpose() << " " << coordinates_right.transpose() << " "
                << disparity << std::endl;

    // ds draw point coordinates
    cv::circle(stereo_image, TO_OPENCV_2F(coordinates_left), 2, cv::Scalar(255, 0, 0), 1);
    cv::circle(stereo_image,
               TO_OPENCV_2F(coordinates_right) + cv::Point2f(0, image_left.rows),
               2,
               cv::Scalar(255, 0, 0),
               1);

    // ds label matches
    cv::putText(stereo_image,
                std::to_string(number_of_stereo_points),
                TO_OPENCV_2F(coordinates_left) + cv::Point2f(5, 5),
                cv::FONT_HERSHEY_PLAIN,
                0.5,
                cv::Scalar(0, 0, 255));
    cv::putText(stereo_image,
                std::to_string(number_of_stereo_points),
                TO_OPENCV_2F(coordinates_right) + cv::Point2f(5, 5 + image_left.rows),
                cv::FONT_HERSHEY_PLAIN,
                0.5,
                cv::Scalar(0, 0, 255));
    ++number_of_stereo_points;
  }
  result_file.close();
  std::cerr << " # valid stereo matches (LEFT & RIGHT): " << number_of_stereo_points << " ("
            << static_cast<float>(number_of_stereo_points) / keypoints_left.size() << ")"
            << std::endl;
  cv::imshow("stereo_matching", stereo_image);
  cv::waitKey(0);
  return 0;
}

ImageFloat readPFM(const std::string& filepath_) {
  ImageFloat disparity_image;
  disparity_image.resize(0, 0);

  // ds open raw file
  std::ifstream stream(filepath_, std::ifstream::binary);
  if (!stream.good() || !stream.is_open()) {
    std::cerr << "readPFM|ERROR: could not open file: " << filepath_ << std::endl;
    return disparity_image;
  }

  // ds start parsing the file header
  std::string format("");
  std::string dimensions("");
  std::string endianess("");
  std::getline(stream, format);
  std::getline(stream, dimensions);
  std::getline(stream, endianess);
  if (format.find("Pf") == std::string::npos) {
    std::cerr << "readPFM|ERROR: PFM format not supported: " << format << std::endl;
    return disparity_image;
  }
  if (std::stod(endianess) != -1) {
    std::cerr << "readPFM|ERROR: endianess not supported: " << endianess << std::endl;
    return disparity_image;
  }

  // ds extract image dimensions
  const size_t index = dimensions.find_first_of(' ');
  const size_t cols  = std::stoi(dimensions.substr(0, index));
  const size_t rows  = std::stoi(dimensions.substr(index + 1, dimensions.length() - index));

  // ds allocate image
  disparity_image.resize(rows, cols);

  // ds parse and simultaneously set image data (bottom up)
  char* buffer                 = new char[sizeof(float)];
  size_t number_of_read_values = 0;
  while (stream.read(buffer, sizeof(float))) {
    float value = 0;
    std::memcpy(&value, buffer, sizeof(float));
    const size_t row                        = number_of_read_values / cols;
    const size_t col                        = number_of_read_values % cols;
    disparity_image.at(rows - row - 1, col) = value;
    ++number_of_read_values;
  }
  delete buffer;

  // ds check for data consistency
  if (number_of_read_values != rows * cols) {
    std::cerr << "ERROR: inconsistent number of values read: " << number_of_read_values
              << std::endl;
    disparity_image.resize(0, 0);
    return disparity_image;
  }
  return disparity_image;
}
