#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts a folder containing RGB (and depth) imagery to SRRG format",
                        0};
// clang-format on

// ds TODO kill me with fire
int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString folder_images_rgb (
    &command_line_parser, "fr",  "folder-rgb", "folder containing RGB images", "");
  ArgumentString folder_images_depth (
    &command_line_parser, "fd",  "folder-depth", "folder containing Depth images", "");
  ArgumentString timestamps (
    &command_line_parser, "t",  "timestamps", "optional file containing timestamp data", "");
  ArgumentString message_target (
    &command_line_parser, "s",  "source", "generated source file name", "out.json");

  const std::string topic_intensity = "/camera/rgb/image_color";
  const std::string topic_depth     = "/camera/depth/image";

  // clang-format on
  command_line_parser.parse();
  if (!folder_images_rgb.isSet() || folder_images_rgb.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (message_target.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!timestamps.value().empty()) {
    std::cerr << "using timestamps data: " << timestamps.value() << std::endl;
  }

  // ds configure serializer
  Serializer serializer;
  serializer.setFilePath(message_target.value());

  // ds super duper hardcoding for ICL
  Matrix3f camera_calibration_matrix(Matrix3f::Zero());
  camera_calibration_matrix << 481.2, 0, 319.5, 0, -481, 239.5, 0, 0, 1;

  // ds parse timestamp file (hardcoded deprecated txtio)
  if (!timestamps.value().empty()) {
    std::ifstream timestamps_file(timestamps.value());
    assert(timestamps_file.good());
    assert(timestamps_file.is_open());
    std::string line_buffer;
    size_t number_of_written_messages_intensity = 0;
    size_t number_of_written_messages_depth     = 0;
    size_t number_of_skipped_messages           = 0;

    // ds read line by line
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

      // ds check for RGB or depth image string
      if (image_message->topic.value() == topic_intensity) {
        // ds parse image file name
        const size_t f               = line_buffer.rfind(folder_images_rgb.value());
        const size_t g               = line_buffer.find(".png" /*ICL*/, f);
        const std::string image_name = line_buffer.substr(f, g - f + 4);

        // ds load image from disk and convert
        cv::Mat image_opencv = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
        if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
          std::cerr << "\nWARNING: skipping invalid image: " << image_name
                    << " with dimensions: " << image_opencv.rows << " x " << image_opencv.cols
                    << std::endl;
          ++number_of_skipped_messages;
          continue;
        }
        cv::imshow("processed image RGB", image_opencv);
        cv::waitKey(1);
        ImageUInt8 base_image;
        base_image.fromCv(image_opencv);
        image_message->setImage(&base_image);
        serializer.writeObject(*image_message);
        std::cerr << "I";
        ++number_of_written_messages_intensity;
      } else if (image_message->topic.value() == topic_depth) {
        if (!folder_images_depth.isSet()) {
          std::cerr << "\nWARNING: skipped message with topic: " << image_message->frame_id.value()
                    << " (folder not set)" << std::endl;
          ++number_of_skipped_messages;
          continue;
        }

        // ds parse image file name
        const size_t f               = line_buffer.rfind(folder_images_depth.value());
        const size_t g               = line_buffer.find(".pgm" /*ICL*/, f);
        const std::string image_name = line_buffer.substr(f, g - f + 4);

        // ds load image from disk and convert
        cv::Mat image_opencv = cv::imread(image_name, CV_LOAD_IMAGE_ANYDEPTH);
        if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
          std::cerr << "\nWARNING: skipping invalid image: " << image_name
                    << " with dimensions: " << image_opencv.rows << " x " << image_opencv.cols
                    << std::endl;
          ++number_of_skipped_messages;
          continue;
        }
        cv::imshow("processed image Depth", image_opencv);
        cv::waitKey(1);
        ImageUInt16 image_depth;
        image_depth.fromCv(image_opencv);
        image_message->setImage(&image_depth); /*ImageFloat takes up more than double the space*/
        serializer.writeObject(*image_message);
        std::cerr << "D";
        ++number_of_written_messages_depth;
      } else {
        std::cerr << "\nWARNING: skipped message with topic: " << image_message->topic.value()
                  << std::endl;
        ++number_of_skipped_messages;
      }

      // ds always produce a camera info message
      CameraInfoMessagePtr camera_message(
        new CameraInfoMessage(image_message->topic.value() + "/info",
                              image_message->frame_id.value(),
                              image_message->seq.value(),
                              image_message->timestamp.value()));
      camera_message->depth_scale.setValue(1e-3f);
      camera_message->projection_model.setValue("pinhole");
      camera_message->distortion_model.setValue("undistorted");
      camera_message->camera_matrix.setValue(camera_calibration_matrix);
      serializer.writeObject(*camera_message);
    }
    std::cerr << std::endl;
    std::cerr << "written messages RGB: " << number_of_written_messages_intensity << std::endl;
    std::cerr << "  written messages D: " << number_of_written_messages_depth << std::endl;
    std::cerr << "    skipped messages: " << number_of_skipped_messages << std::endl;
  }
  return 0;
}
