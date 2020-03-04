#include <iostream>

#include "srrg2_proslam_tracking/instances.h"
#include "srrg_config/configurable_manager.h"
#include "srrg_messages/instances.h"
#include "srrg_slam_interfaces/instances.h"
#include "srrg_system_utils/parse_command_line.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_proslam;

const char* banner[] = {"This program is intended for evaluating isolated frame-to-frame feature "
                        "tracking for a sequence of images (w/o) motion information",
                        0};

int main(int argc_, char** argv_) {
  messages_registerTypes();
  srrg2_proslam_tracking_registerTypes();
  Profiler::enable_logging = true;

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString config_file(
      &command_line_parser, "c",  "configuration",
                            "config file for dataset playback", "");
  ArgumentString name_search_method(
      &command_line_parser, "a", "algorithm",
                            "target matching algorithm: {bruteforce, projective_kd, projective_circle}", "bruteforce");
  ArgumentString parameter_ground_truth_poses(
      &command_line_parser, "gt", "ground-truth",
                            "ground truth poses (only KITTI format atm)", "");
  ArgumentFlag flag_enable_point_seeding(
      &command_line_parser, "es", "enable-seeding",
                            "enables feature seeding and tracking");

  // clang-format on
  command_line_parser.parse();
  if (!config_file.isSet()) {
    std::cerr << "ERROR: no configuration file provided" << std::endl;
    return -1;
  }

  // ds set up dataset playback from configuration
  ConfigurableManager manager;
  manager.read(config_file.value());
  MessageSynchronizedSourcePtr synchronizer = manager.getByName<MessageSynchronizedSource>("sync");
  if (!synchronizer) {
    std::cerr << "ERROR: could not load dataset from configuration file: " << config_file.value()
              << std::endl;
    return -1;
  }

  // ds check if we have ground truth data
  std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>> ground_truth_poses;
  if (parameter_ground_truth_poses.isSet() && !parameter_ground_truth_poses.value().empty()) {
    std::cerr << "loading ground truth poses from: " << parameter_ground_truth_poses.value()
              << std::endl;

    // ds open file
    std::ifstream ground_truth_file(parameter_ground_truth_poses.value(), std::ifstream::in);

    // ds read file by tokens
    double r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz;
    while (ground_truth_file >> r00 >> r01 >> r02 >> tx >> r10 >> r11 >> r12 >> ty >> r20 >> r21 >>
           r22 >> tz) {
      Isometry3f pose(srrg2_core::Isometry3f::Identity());
      pose.translation() = srrg2_core::Vector3f(tx, ty, tz);
      pose.linear() << r00, r01, r02, r10, r11, r12, r20, r21, r22;
      ground_truth_poses.push_back(pose);
    }
    ground_truth_file.clear();
    std::cerr << "loaded ground truth pose: " << ground_truth_poses.size() << std::endl;
  }

  // ds load target feature handlers from configuration
  CorrespondenceFinderDescriptorBasedBruteforce4D3DPtr matcher = nullptr;
  float projection_uncertainty_radius_pixels                   = 10;
  if (name_search_method.value() == "bruteforce") {
    matcher = manager.getByName<CorrespondenceFinderDescriptorBasedBruteforce4D3D>("cf_bruteforce");
  } else if (name_search_method.value() == "projective_kd") {
    CorrespondenceFinderProjectiveKDTree4D3DPtr matcher_projective =
      manager.getByName<CorrespondenceFinderProjectiveKDTree4D3D>("cf_projective_kd");
    projection_uncertainty_radius_pixels =
      matcher_projective->param_maximum_search_radius_pixels.value();
    matcher = matcher_projective;
  } else if (name_search_method.value() == "projective_circle") {
    CorrespondenceFinderProjectiveCircle4D3DPtr matcher_projective =
      manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
    projection_uncertainty_radius_pixels =
      matcher_projective->param_maximum_search_radius_pixels.value();
    matcher = matcher_projective;
  } else {
    std::cerr << "ERROR: unknown matcher: " << name_search_method.value() << std::endl;
    return -1;
  }
  if (!matcher) {
    std::cerr << "ERROR: unable to load feature matcher from configuration" << std::endl;
    return -1;
  }

  // ds load measurement adaptor from configuration
  MeasurementAdaptorStereoProjectivePtr adaptor(
    manager.getByName<MeasurementAdaptorStereoProjective>("adaptor_stereo_projective"));
  if (!adaptor) {
    std::cerr << "ERROR: unable to load measurement adaptor from configuration" << std::endl;
    return -1;
  }

  // ds load projector
  PointIntensityDescriptor3fProjectorPinholePtr projector(
    manager.getByName<PointIntensityDescriptor3fProjectorPinhole>("projector"));
  if (!projector) {
    std::cerr << "ERROR: unable to load projector from configuration" << std::endl;
    return -1;
  }

  // ds load triangulator module
  TriangulatorRigidStereoDescriptorsPtr triangulator(
    manager.getByName<TriangulatorRigidStereoDescriptors>("triangulator"));
  if (!triangulator) {
    std::cerr << "ERROR: unable to load triangulator from configuration" << std::endl;
    return -1;
  }
  triangulator->setPlatform(PlatformPtr(new Platform()));

  // ds rolling tracking buffer (maximum reach in the past is defined by window_size)
  PointIntensityDescriptor3fVectorCloud points_in_camera_previous;
  PointIntensityDescriptor4fVectorCloud stereo_matches_previous;
  cv::Mat image_previous;

  // ds process messages
  size_t number_of_processed_messages = 0;
  while (BaseSensorMessagePtr message = synchronizer->getMessage()) {
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    if (!message_pack || message_pack->messages.size() < 5) {
      std::cerr << "WARNING: skipping message: " << message->name() << std::endl;
      continue;
    }

    // ds update platform
    TransformEventsMessagePtr transform_message =
      std::dynamic_pointer_cast<TransformEventsMessage>(message_pack->messages.at(4));
    if (transform_message) {
      triangulator->platform()->add(transform_message);
    }

    ImageMessagePtr image_message =
      std::dynamic_pointer_cast<ImageMessage>(message_pack->messages.at(0));
    if (!image_message) {
      std::cerr << "WARNING: skipping message: " << image_message->name() << std::endl;
      continue;
    }
    const BaseImage* image_base = image_message->image();
    if (!image_base) {
      std::cerr << "WARNING: unable to load image from disk" << std::endl;
      continue;
    }

    // ds fetch camera info
    CameraInfoMessagePtr camera_info_message =
      std::dynamic_pointer_cast<CameraInfoMessage>(message_pack->messages.at(2));
    Matrix3f camera_calibration_matrix(Matrix3f::Identity());
    if (camera_info_message) {
      camera_calibration_matrix = camera_info_message->camera_matrix.value();
    }

    // ds configure projector
    projector->setCameraMatrix(camera_calibration_matrix);
    projector->param_canvas_cols.setValue(image_base->cols());
    projector->param_canvas_rows.setValue(image_base->rows());

    // ds previous points in current image plane
    PointIntensityDescriptor3fVectorCloud projections;

    // ds if we have ground truth data
    PointIntensityDescriptor3fVectorCloud points_from_previous_in_camera_current;
    std::vector<int> indices_points_projected_to_previous;
    if (number_of_processed_messages > 0 &&
        number_of_processed_messages < ground_truth_poses.size()) {
      // ds compute relative camera motion
      const Isometry3f motion(ground_truth_poses[number_of_processed_messages].inverse() *
                              ground_truth_poses[number_of_processed_messages - 1]);

      // ds compute projections
      projector->setCameraPose(motion.inverse());
      projector->compute(points_in_camera_previous,
                         points_from_previous_in_camera_current,
                         projections,
                         indices_points_projected_to_previous);

      // ds check if we have to provide a pose estimate
      if (name_search_method.value() == "projective_circle") {
        std::dynamic_pointer_cast<CorrespondenceFinderProjectiveCircle4D3D>(matcher)->setEstimate(
          motion);
      }
    }

    // ds use previous observations as projections
    else {
      projections.reserve(stereo_matches_previous.size());
      for (size_t i = 0; i < stereo_matches_previous.size(); ++i) {
        const PointIntensityDescriptor4f& point_in_image_previous(stereo_matches_previous[i]);
        PointIntensityDescriptor3f point_in_image_current;
        point_in_image_current.coordinates()(0) = point_in_image_previous.coordinates()(0);
        point_in_image_current.coordinates()(1) = point_in_image_previous.coordinates()(1);
        projections.emplace_back(point_in_image_current);
        indices_points_projected_to_previous.push_back(i);
      }
    }

    // ds adapt stereo measurements
    PointIntensityDescriptor4fVectorCloud stereo_matches;
    adaptor->setMeasurement(message);
    adaptor->setProjections(&projections, projection_uncertainty_radius_pixels);
    adaptor->setDest(&stereo_matches);
    adaptor->compute();

    // ds match against the tracking pool (invidually)
    CorrespondenceVector correspondences;
    matcher->setFixed(&stereo_matches);
    matcher->setMoving(&points_in_camera_previous);
    matcher->setCorrespondences(&correspondences);
    if (!image_previous.empty()) {
      matcher->compute();
    }

    std::cerr << "message: " << number_of_processed_messages;
    std::cerr << " extracted stereo features: " << stereo_matches.size();
    std::cerr << " tracking feature pool: " << stereo_matches_previous.size();
    if (!correspondences.empty()) {
      std::cerr << " correspondences: " << correspondences.size();
      std::cerr << " ("
                << static_cast<double>(correspondences.size()) / stereo_matches_previous.size()
                << ")";
    }
    std::cerr << std::endl;

    // ds convert to opencv for easy visualization
    cv::Mat image_current;
    image_base->toCv(image_current);
    cv::Mat canvas_opencv;
    if (image_previous.empty()) {
      canvas_opencv = image_current.clone();
    } else {
      cv::vconcat(image_current, image_previous, canvas_opencv);
    }
    cv::cvtColor(canvas_opencv, canvas_opencv, CV_GRAY2RGB);

    // ds draw current features in current image in blue
    for (const auto& feature : stereo_matches) {
      cv::circle(canvas_opencv, TO_OPENCV_2F(feature.coordinates()), 2, cv::Scalar(255, 0, 0), -1);
    }

    // ds compute color codes for past features (we don't have ids)
    std::vector<cv::Scalar> colors_moving;
    colors_moving.reserve(points_in_camera_previous.size());
    for (size_t i = 0; i < points_in_camera_previous.size(); ++i) {
      cv::Scalar color(0, 0, 255);
      cv::randn(color, cv::Scalar(155, 155, 155), cv::Scalar(100, 100, 100));
      colors_moving.emplace_back(color);
    }
    assert(stereo_matches_previous.size() == points_in_camera_previous.size());

    // ds draw previous feature positions in the current image and the past image
    const cv::Point2f offset(0, image_current.rows);
    for (size_t i = 0; i < stereo_matches_previous.size(); ++i) {
      const auto& feature(stereo_matches_previous[i]);
      const cv::Scalar color(colors_moving[i]);
      cv::circle(canvas_opencv, TO_OPENCV_2F(feature.coordinates()), 2, color, -1);
      cv::circle(canvas_opencv, TO_OPENCV_2F(feature.coordinates()) + offset, 2, color, -1);
    }

    // ds draw correspondences from current features to tracking pool features
    std::set<int> matched_indices_previous;
    for (const Correspondence& correspondence : correspondences) {
      // ds highlighting the track
      cv::line(canvas_opencv,
               TO_OPENCV_2F(stereo_matches[correspondence.fixed_idx].coordinates()),
               TO_OPENCV_2F(stereo_matches_previous[correspondence.moving_idx].coordinates()),
               colors_moving[correspondence.moving_idx],
               1);
      matched_indices_previous.insert(correspondence.moving_idx);
    }

    // ds draw projected previous feature positions in the current image with color coded circles
    for (size_t i = 0; i < projections.size(); ++i) {
      const int& index_previous = indices_points_projected_to_previous[i];
      if (matched_indices_previous.count(index_previous) == 1) {
        cv::circle(canvas_opencv,
                   TO_OPENCV_2F(projections[i].coordinates()),
                   projection_uncertainty_radius_pixels,
                   colors_moving[index_previous],
                   1);
      }
    }

    // ds display image blocking
    cv::imshow(argv_[0], canvas_opencv);
    if (cv::waitKey(0) == 27 /*ESC key on keyboard*/) {
      std::cerr << "received [ESC], terminating" << std::endl;
      break;
    }

    // ds triangulate current stereo points
    points_in_camera_previous.clear();
    triangulator->setDest(&points_in_camera_previous);
    triangulator->setMoving(&stereo_matches);
    triangulator->compute();
    assert(points_in_camera_previous.size() <= stereo_matches.size());
    const std::unordered_set<size_t>& indices_invalidated(triangulator->indicesInvalidated());
    const size_t number_of_lost_points = indices_invalidated.size();
    if (number_of_lost_points > 0) {
      std::cerr << "lost points in triangulation: " << number_of_lost_points << std::endl;

      // ds erase invalid points from our buffers
      for (const size_t index_to_erase : indices_invalidated) {
        // ds assign the current last element to the one that will be erased (overwritting it)
        // ds erase the last element in the vector (otherwise there would be a duplicate now)
        // ds the original index order is not preserved, but in this example we don't depend on it
        points_in_camera_previous[index_to_erase] = points_in_camera_previous.back();
        points_in_camera_previous.pop_back();
        stereo_matches[index_to_erase] = stereo_matches.back();
        stereo_matches.pop_back();
      }
    }

    // ds bookkeeping - note that no merging is perfomed so we can observe handling of duplicates!
    stereo_matches_previous = stereo_matches;
    image_previous          = image_current;
    ++number_of_processed_messages;
  }
}
