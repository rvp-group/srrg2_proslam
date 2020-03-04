#include <iostream>
#include <thread>

// ds disk i/o
#include <srrg_boss/serializable.h>
#include <srrg_messages/instances.h>

// ds helpers
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

// ds measurement adaption
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_unprojector.h>

// ds visualization
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

// ds visualization routine
void unproject(ViewerCanvasPtr canvas_, const Point3fVectorCloud& points_3d_);

// clang-format off
const char* banner[] = {"This program computes the 3D unprojection for a single RGB-D image pair",
                        "usage: [options] -image-rgb/i </path/to/image> -image-depth/d "
                        "</path/to/image> ",
                        0};
// clang-format on

int main(int argc_, char** argv_) {
  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString filepath_image_depth(
    &command_line_parser, "d", "image-depth", "file path for depth image", "main");
  ArgumentString filepath_image_rgb(
    &command_line_parser, "i", "image-rgb", "file path for RGB image", "main");
  // clang-format on
  command_line_parser.parse();
  if (!filepath_image_depth.isSet() || filepath_image_depth.value().empty()) {
    printBanner(banner);
    return 0;
  }
  if (!filepath_image_rgb.isSet() || filepath_image_rgb.value().empty()) {
    printBanner(banner);
    return 0;
  }

  // ds read opencv images from disk
  const cv::Mat image_depth_opencv(
    cv::imread(filepath_image_depth.value(), CV_LOAD_IMAGE_ANYDEPTH));
  const cv::Mat image_rgb_opencv(cv::imread(filepath_image_rgb.value(), CV_LOAD_IMAGE_UNCHANGED));
  const cv::Mat image_grey_opencv(cv::imread(filepath_image_rgb.value(), CV_LOAD_IMAGE_GRAYSCALE));

  // ds display loaded images
  cv::imshow("image_depth", image_depth_opencv * 5 /*amplify value for better visibility*/);
  cv::imshow("image_rgb", image_rgb_opencv);
  cv::imshow("image_grey_opencv", image_grey_opencv);
  cv::waitKey(1);

  // ds convert opencv image to srrg2
  ImageUInt16 image_depth;
  image_depth.fromCv(image_depth_opencv);
  ImageUInt8 image_grey;
  image_grey.fromCv(image_grey_opencv);

  // ds TODO load camera matrix from disk (.conf or .boss)
  // ds this matrix is hardcoded for the example images in (srrg2_proslam/test_data/)
  Matrix3f K;
  K << 269.853, 0, 157.051, 0, 269.733, 113.118, 0, 0, 1;

  // ds allocate an unprojector
  PointUnprojector_<PinholeUnprojection, Point3fVectorCloud> unprojector;
  unprojector.setCameraMatrix(K);
  unprojector.param_range_min.setValue(0.1f);
  unprojector.param_range_max.setValue(10.0f);

  // ds convert input to proper format TODO avoid this?
  ImageFloat image_depth_float;
  image_depth.convertTo(image_depth_float, 1e-3);
  ImageFloat image_grey_float;
  image_grey.convertTo(image_grey_float, 1.f / 255);

  // ds point cloud to be generated from RGB-D data through unproject (preallocate memory)
  Point3fVectorCloud unprojected_points_3d;
  unprojected_points_3d.resize(image_depth_float.size());

  // ds compute unprojection for all points
  SystemUsageCounter::tic();
  const size_t number_of_points =
    unprojector.compute(unprojected_points_3d.data(), image_depth_float /*, image_grey_float*/);
  const double duration_seconds = SystemUsageCounter::toc();
  unprojected_points_3d.resize(number_of_points);
  std::cerr << "# unprojected points: " << number_of_points << std::endl;
  std::cerr << "unprojection duration (s): " << duration_seconds
            << " (Hz): " << 1 / duration_seconds << std::endl;

  // ds launch viewer and processing thread
  QApplication qapp(argc_, argv_);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer(argc_, argv_, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("test_unproject_rgbd");
  std::thread processing_thread(unproject, canvas, unprojected_points_3d);

  // ds display viewer until termination signal is received
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

void unproject(ViewerCanvasPtr canvas_, const Point3fVectorCloud& points_3d_) {
  while (ViewerCoreSharedQGL::isRunning()) {
    canvas_->putPoints(points_3d_);
    canvas_->flush();
  }
}
