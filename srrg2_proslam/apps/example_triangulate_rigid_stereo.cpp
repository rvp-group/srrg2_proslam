#include <iostream>
#include <thread>

#include <srrg_config/configurable_manager.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/parse_command_line.h>

#include "srrg2_proslam/sensor_processing/instances.h"
#include "srrg2_proslam/tracking/instances.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_qgl_viewport;
using namespace srrg2_proslam;

// clang-format off
const char* banner[] = {"This program computes a sparse 3D reconstruction for a rectified stereo image pair",
                        "The 3D points are anchored in the left camera coordinate frame",
                        "usage: [options] -image-left/l </path/to/image> -image-right/r </path/to/image> -c <configuration> ",
                        0};
// clang-format on

// ds dummy configuration generation
void generateConfig(ConfigurableManager& manager_, const std::string& filepath_);

// ds visualization routines
void visualizeStereoMatches(const cv::Mat& image_left_,
                            const cv::Mat& image_right_,
                            const PointIntensityDescriptor4fVectorCloud& stereo_points_);
void visualizeTriangulatedPoints(ViewerCanvasPtr canvas_,
                                 const PointIntensityDescriptor3fVectorCloud& points_);

int main(int argc_, char** argv_) {
  messages_registerTypes();
  srrg2_proslam_tracking_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString config_file (
    &command_line_parser, "c",  "configuration file",
                          "config file to read/write", "measurement_adaptor_stereo_projective.conf");
  ArgumentString filepath_image_left(
    &command_line_parser, "l", "image-left",
                          "file path for left image", "");
  ArgumentString filepath_image_right(
    &command_line_parser, "r", "image-right",
                          "file path for right image", "");

  // clang-format on
  command_line_parser.parse();
  ConfigurableManager manager;
  if (!config_file.isSet()) {
    generateConfig(manager, config_file.value());
    std::cerr << "ADJUST camera calibration and rerun with -c" << std::endl;
    return 0;
  }
  if (!filepath_image_left.isSet() || filepath_image_left.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!filepath_image_right.isSet() || filepath_image_right.value().empty()) {
    printBanner(banner);
    return 0;
  }

  // ds allocate adaptor from configuration
  manager.read(config_file.value());
  RawDataPreprocessorStereoProjectivePtr adaptor =
    manager.getByName<RawDataPreprocessorStereoProjective>("adaptor_stereo_projective");
  if (!adaptor) {
    std::cerr << "ERROR: could not instantiate adaptor from configuration file: "
              << config_file.value() << std::endl;
    return 0;
  }

  // ds allocate a triangulation unit from configuration
  TriangulatorRigidStereoDescriptorsPtr triangulator =
    manager.getByName<TriangulatorRigidStereoDescriptors>("triangulator");
  if (!triangulator) {
    std::cerr << "ERROR: could not instantiate triangulator from configuration file: "
              << config_file.value() << std::endl;
    return 0;
  }

  // ds read opencv images from disk (measurements)
  const cv::Mat image_left_opencv(cv::imread(filepath_image_left.value(), CV_LOAD_IMAGE_GRAYSCALE));
  const cv::Mat image_right_opencv(
    cv::imread(filepath_image_right.value(), CV_LOAD_IMAGE_GRAYSCALE));
  if (image_left_opencv.size != image_right_opencv.size) {
    std::cerr << "ERROR: images have non-identical dimensions" << std::endl;
    return 0;
  }

  // ds convert messages to generic format TODO make_shared still unusuable?
  // ds image interfaces are very cumbersome TODO add advanced constructors/setImage
  MessagePackPtr stereo_message(new MessagePack());
  ImageMessagePtr image_left(new ImageMessage("/left", "0", 0, 0.0));
  ImageUInt8 image_left_grey;
  image_left_grey.fromCv(image_left_opencv);
  image_left->setImage(&image_left_grey);
  ImageMessagePtr image_right(new ImageMessage("/right", "0", 0, 0.0));
  ImageUInt8 image_right_grey;
  image_right_grey.fromCv(image_right_opencv);
  image_right->setImage(&image_right_grey);
  stereo_message->messages = {image_left, image_right};

  // ds upcast and set measurements to adaptor TODO remove smart pointer passing by reference
  BaseSensorMessagePtr message = stereo_message;
  adaptor->setRawData(message);

  // ds adapt measurements and store results in stereo points
  PointIntensityDescriptor4fVectorCloud stereo_points;
  adaptor->setMeas(&stereo_points);
  adaptor->compute();

  // ds platform (TODO load from disk instead of hardcoded KITTI 00)
  PlatformPtr platform(new Platform());
  Matrix3f camera_calibration_matrix;
  Isometry3f stereo_baseline = Isometry3f::Identity();

  //  {
  //    // ds KITTI 00
  //    camera_calibration_matrix << 718.856, 0, 607.193, 0, 718.856, 185.216, 0, 0, 1;
  //    stereo_baseline.translation() = Vector3f(-0.537166, 0, 0);
  //  }

  {
    // ds EuRoC MH_easy
    camera_calibration_matrix << 435.262, 0, 367.415, 0, 435.262, 252.171, 0, 0, 1;
    stereo_baseline.translation() = Vector3f(-0.110078, 0, 0);
  }

  TransformEventPtr event(new TransformEvent(0, "camera_right", stereo_baseline, "camera_left"));
  platform->addEvent(event);
  triangulator->setPlatform(platform);

  //  {
  //    // ds KITTI 00
  //    triangulator->param_projector->param_canvas_cols.setValue(1241);
  //    triangulator->param_projector->param_canvas_rows.setValue(376);
  //  }

  {
    // ds EuRoC MH_easy
    triangulator->param_projector->param_canvas_cols.setValue(752);
    triangulator->param_projector->param_canvas_rows.setValue(480);
  }

  triangulator->param_projector->setCameraMatrix(camera_calibration_matrix);

  // ds triangulated adapated measurements to 3d points
  PointIntensityDescriptor3fVectorCloud triangulated_points_in_camera_left;
  triangulator->setDest(&triangulated_points_in_camera_left);
  triangulator->setMoving(&stereo_points);
  triangulator->compute();
  std::cerr << "# triangulated points: " << triangulated_points_in_camera_left.size() << "/"
            << stereo_points.size() << std::endl;

  // ds visualize detected features and stereo matches
  visualizeStereoMatches(image_left_opencv, image_right_opencv, stereo_points);

  // ds launch viewer and processing thread (points are not changed at this point)
  QApplication qapp(argc_, argv_);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer(argc_, argv_, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("example_triangulate_rigid_stereo");
  std::thread processing_thread(
    visualizeTriangulatedPoints, canvas, triangulated_points_in_camera_left);

  // ds display viewer until termination signal is received
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

void generateConfig(ConfigurableManager& manager_, const std::string& filepath_) {
  // ds put measurement adaptor and triangulator into the same configuration
  manager_.create<RawDataPreprocessorStereoProjective>("measurement_adaptor_stereo_projective");
  manager_.create<TriangulatorRigidStereoDescriptors>("rigid_stereo_triangulator");
  manager_.write(filepath_);
  std::cerr << "generated default configuration: " << filepath_ << std::endl;
}

void visualizeStereoMatches(const cv::Mat& image_left_,
                            const cv::Mat& image_right_,
                            const PointIntensityDescriptor4fVectorCloud& stereo_points_) {
  // ds draw stereo matching features (in color)
  cv::Mat stereo_image;
  cv::vconcat(image_left_, image_right_, stereo_image);
  cv::cvtColor(stereo_image, stereo_image, cv::COLOR_GRAY2BGR);
  for (const PointIntensityDescriptor4f& stereo_point : stereo_points_) {
    cv::line(stereo_image,
             TO_OPENCV_2F(stereo_point.coordinates().head<2>()),
             TO_OPENCV_2F(stereo_point.coordinates().tail<2>()) + cv::Point2f(0, image_left_.rows),
             cv::Scalar(0, 255, 0));
  }
  cv::imshow("stereo_image (top: left camera, bot: right camera)", stereo_image);

  // ds let opencv breathe
  // ds removing this line can cause the shared viewer to hang on sick installations
  cv::waitKey(1);
}

void visualizeTriangulatedPoints(ViewerCanvasPtr canvas_,
                                 const PointIntensityDescriptor3fVectorCloud& points_) {
  // ds convert points to pure 3d for rendering
  // ds TODO to avoid adding methods to the rendering we should have polymorphic point clouds
  // ds TODO polymorphism of point intensity is BROKEN since it does not inherit from Point_
  Point3fVectorCloud points_visualization;
  points_visualization.reserve(points_.size());
  for (const PointIntensityDescriptor3f& point_raw : points_) {
    Point3f point;
    point.coordinates() = point_raw.coordinates();
    points_visualization.emplace_back(point);
  }

  // ds enter visualization loop
  while (ViewerCoreSharedQGL::isRunning()) {
    canvas_->putPoints(points_visualization);
    canvas_->flush();
  }
}
