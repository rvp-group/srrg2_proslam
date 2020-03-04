#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts a folder containing stereo imagery to SRRG format",
                        0};
// clang-format on

void setCalibrationKITTI(const std::string& file_name_calibration_,
                         Matrix3f& camera_calibration_matrix_,
                         Vector3f& baseline_pixels_);

// ds TODO kill me with fire
int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString message_folder (
    &command_line_parser, "f",  "folder", "folder containing raw images", "");
  ArgumentString timestamps (
    &command_line_parser, "t",  "txtio", "txtio logfile in depracted format", "");
  ArgumentString argument_ground_truth (
    &command_line_parser, "gt",  "ground-truth", "optional file containing (left) campera pose ground truth data", "");
  ArgumentString output_filename (
    &command_line_parser, "o",  "o", "target output message file name", "messages.json");
  ArgumentString parameter_calibration_file (
    &command_line_parser, "c",  "calibration", "calibration file name", "calib.txt");

  // clang-format on
  command_line_parser.parse();
  if (!message_folder.isSet() || message_folder.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!timestamps.isSet() || timestamps.value().empty()) {
    throw std::runtime_error("ERROR: timestamps not set, but must be provided");
  }
  std::cerr << "using timestamps data: " << timestamps.value() << std::endl;

  // ds buffer ground truth camera poses (if available)
  StdVectorEigenIsometry3f camera_poses_in_world;
  camera_poses_in_world.reserve(10000);
  if (argument_ground_truth.isSet() && !argument_ground_truth.value().empty()) {
    std::cerr << "using ground truth data: " << argument_ground_truth.value() << std::endl;

    // ds open ground truth file in KITTI format: # R t (3x4)
    std::ifstream ground_truth_file(argument_ground_truth.value(), std::ios::in);
    if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
      throw std::runtime_error("ERROR: unable to read ground truth file: " +
                               argument_ground_truth.value());
    }

    // ds read file by tokens
    double r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz;
    while (ground_truth_file >> r00 >> r01 >> r02 >> tx >> r10 >> r11 >> r12 >> ty >> r20 >> r21 >>
           r22 >> tz) {
      // ds assemble isometry
      Isometry3f pose(Isometry3f::Identity());
      pose.translation() = srrg2_core::Vector3f(tx, ty, tz);
      pose.linear() << r00, r01, r02, r10, r11, r12, r20, r21, r22;
      camera_poses_in_world.emplace_back(pose);
    }
    std::cerr << "read camera poses: " << camera_poses_in_world.size() << std::endl;
  }

  // ds load kitti calibration
  Matrix3f camera_calibration_matrix(Matrix3f::Zero());
  Vector3f baseline_to_camera_left;
  setCalibrationKITTI(
    parameter_calibration_file.value(), camera_calibration_matrix, baseline_to_camera_left);
  Isometry3f transformation_right_from_left(Isometry3f::Identity());
  transformation_right_from_left.translation() =
    camera_calibration_matrix.fullPivLu().solve(baseline_to_camera_left);
  std::cerr << transformation_right_from_left.matrix() << std::endl;
  std::cerr << "reconstructed baseline: "
            << camera_calibration_matrix * transformation_right_from_left.translation()
            << std::endl;
  std::cerr << "press [ENTER] to start conversion" << std::endl;
  getchar();

  // ds configure serializer
  Serializer serializer;
  serializer.setFilePath(output_filename.value());

  // ds parse timestamp file (hardcoded deprecated txtio)
  if (!timestamps.value().empty()) {
    std::ifstream timestamps_file(timestamps.value());
    assert(timestamps_file.good());
    assert(timestamps_file.is_open());
    std::string line_buffer;
    size_t number_of_written_messages = 0;
    StdVectorEigenIsometry3f::const_iterator iterator_pose(camera_poses_in_world.begin());
    while (std::getline(timestamps_file, line_buffer)) {
      // ds parse line and assemble stereo message
      const size_t a = line_buffer.find_first_of(' ') + 1;
      const size_t b = line_buffer.find_first_of(' ', a) + 1;
      const size_t c = line_buffer.find_first_of(' ', b) + 1;
      const size_t d = line_buffer.find_first_of(' ', c) + 1;
      const size_t e = line_buffer.find_first_of(' ', d) + 1;
      ImageMessagePtr image_message(new ImageMessage(line_buffer.substr(a, b - a - 1),
                                                     line_buffer.substr(b, c - b - 1),
                                                     std::stoi(line_buffer.substr(c, d - c - 1)),
                                                     std::stod(line_buffer.substr(d, e - d - 1))));
      if (image_message->frame_id.value() == "camera_left") {
        std::cerr << "L";
      } else {
        std::cerr << "R";
      }

      // ds parse image file name and load from disk
      const size_t f               = line_buffer.find(message_folder.value());
      const size_t g               = line_buffer.find(".pgm", f);
      const std::string image_name = line_buffer.substr(f, g - f + 4);
      cv::Mat image_opencv         = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
      assert(image_opencv.rows != 0);
      assert(image_opencv.cols != 0);
      ImageUInt8* base_image(new ImageUInt8());
      base_image->fromCv(image_opencv);
      image_message->setImage(base_image);
      image_message->image_rows.setValue(image_opencv.rows);
      image_message->image_cols.setValue(image_opencv.cols);
      serializer.writeObject(*image_message);

      // ds also produce a camera info message
      CameraInfoMessagePtr camera_message(
        new CameraInfoMessage(image_message->topic.value() + "/info",
                              image_message->frame_id.value(),
                              image_message->seq.value(),
                              image_message->timestamp.value()));
      camera_message->depth_scale.setValue(1.0f);
      camera_message->projection_model.setValue("pinhole");
      camera_message->distortion_model.setValue("undistorted");
      camera_message->camera_matrix.setValue(camera_calibration_matrix);
      serializer.writeObject(*camera_message);

      // ds provide relative transform between left and right camera
      TransformEventsMessagePtr transform_message(
        new TransformEventsMessage("/tf",
                                   image_message->frame_id.value(),
                                   image_message->seq.value(),
                                   image_message->timestamp.value()));
      TransformEvent event_stereo(image_message->timestamp.value(),
                                  "camera_right",
                                  transformation_right_from_left,
                                  "camera_left");
      transform_message->events.resize(1);
      transform_message->events.setValue(0, event_stereo);

      // ds look for auxiliary data that is synchronized with the left camera
      if (image_message->frame_id.value() == "camera_left") {
        // ds if ground truth data is available
        if (iterator_pose != camera_poses_in_world.end()) {
          transform_message->events.resize(2);
          // ds also add left camera in world transform
          TransformEvent event_gt(
            image_message->timestamp.value(), "camera_left", *iterator_pose, "world");
          transform_message->events.setValue(1, event_gt);
          ++iterator_pose;
          std::cerr << "G";
        } else {
          std::cerr << "_";
        }
      }

      // ds write to disk
      serializer.writeObject(*transform_message);
      ++number_of_written_messages;
    }
    std::cerr << std::endl;
    std::cerr << "written messages: " << number_of_written_messages << std::endl;
  }
  return 0;
}

void setCalibrationKITTI(const std::string& file_name_calibration_,
                         Matrix3f& camera_calibration_matrix_,
                         Vector3f& baseline_pixels_) {
  // ds load camera matrix - for now only KITTI parsing
  std::ifstream file_calibration(file_name_calibration_, std::ifstream::in);
  std::string line_buffer("");
  std::getline(file_calibration, line_buffer);
  if (line_buffer.empty()) {
    throw std::runtime_error("invalid camera calibration file provided");
  }
  std::istringstream stream_left(line_buffer);
  baseline_pixels_.setZero();
  camera_calibration_matrix_.setIdentity();

  // ds parse in fixed order
  std::string filler("");
  stream_left >> filler;
  stream_left >> camera_calibration_matrix_(0, 0);
  stream_left >> filler;
  stream_left >> camera_calibration_matrix_(0, 2);
  stream_left >> filler;
  stream_left >> filler;
  stream_left >> camera_calibration_matrix_(1, 1);
  stream_left >> camera_calibration_matrix_(1, 2);

  // ds read second projection matrix to obtain the horizontal offset
  std::getline(file_calibration, line_buffer);
  std::istringstream stream_right(line_buffer);
  stream_right >> filler;
  stream_right >> filler;
  stream_right >> filler;
  stream_right >> filler;
  stream_right >> baseline_pixels_(0);
  file_calibration.close();
  std::cerr << "setCalibrationKITTI|loaded camera calibration matrix: \n"
            << camera_calibration_matrix_ << std::endl;
  std::cerr << "setCalibrationKITTI|with baseline (pixels): " << baseline_pixels_.transpose()
            << std::endl;
}
