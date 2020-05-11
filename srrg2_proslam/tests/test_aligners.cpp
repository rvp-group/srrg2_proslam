#include "fixtures.hpp"

using StereoFactor    = FactorCorrespondenceDriven_<SE3RectifiedStereoProjectiveErrorFactor,
                                                 PointIntensityDescriptor4fVectorCloud,
                                                 PointIntensityDescriptor3fVectorCloud>;
using StereoFactorPtr = std::shared_ptr<StereoFactor>;

using SyntheticWorldWithDescriptorsSE3 =
  srrg2_test::SyntheticWorld<3, float, PointIntensityDescriptor3f, PointIntensityDescriptor2f>;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(SyntheticWorldWithDescriptorsSE3, AlignerSliceProcessorProjective) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree2D3DPtr correspondence_finder(
    new CorrespondenceFinderProjectiveKDTree2D3D);
  correspondence_finder->param_maximum_descriptor_distance.setValue(75);
  correspondence_finder->param_minimum_descriptor_distance.setValue(25);
  correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  correspondence_finder->param_maximum_search_radius_pixels.setValue(50);
  correspondence_finder->param_minimum_number_of_points_per_cluster.setValue(10);
  correspondence_finder->param_projector->param_canvas_cols.setValue(canvas_size(0));
  correspondence_finder->param_projector->param_canvas_rows.setValue(canvas_size(1));
  correspondence_finder->param_projector->param_range_min.setValue(0.1);
  correspondence_finder->param_projector->param_range_max.setValue(10);
  correspondence_finder->param_projector->setCameraMatrix(projection_matrix);

  // ds set sensor poses
  Isometry3f pose(Isometry3f::Identity());
  sensor_poses.push_back(pose);
  pose.translation() += Vector3f(0, 0, -1);
  pose.linear() = geometry3d::a2r(Vector3f(0.001, 0.001, -0.001));

  sensor_poses.push_back(pose);

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_GT(points_in_sensor[0].size(), 90);
  ASSERT_GT(points_in_sensor_projected_in_canvas[0].size(), 90);

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

  // srrg projector for camera information
  AlignerSliceProcessorProjectivePtr slice_projective(new AlignerSliceProcessorProjective);
  //  slice_projective->param_base_frame_id.setValue("base_frame");
  //  slice_projective->param_frame_id.setValue("camera_left");
  slice_projective->param_fixed_slice_name.setValue("points");
  slice_projective->param_moving_slice_name.setValue("points");
  slice_projective->param_finder.setValue(correspondence_finder);
  slice_projective->param_projector.setValue(correspondence_finder->param_projector.value());

  MultiAligner3DQRPtr aligner(new MultiAligner3DQR);
  aligner->param_slice_processors.pushBack(slice_projective);
  aligner->param_max_iterations.setValue(10);

  PointIntensityDescriptor2fVectorCloud points_in_camera_fixed =
    points_in_sensor_projected_in_canvas[1];
  PointIntensityDescriptor3fVectorCloud points_in_world_moving = points_in_sensor[0];

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor2fVectorCloud*> measurements_property(
    slice_projective->param_fixed_slice_name.value(),
    "",
    nullptr,
    &points_in_camera_fixed,
    nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> scene_property(
    slice_projective->param_moving_slice_name.value(),
    "",
    nullptr,
    &points_in_world_moving,
    nullptr);

  for (PointIntensityDescriptor2f& pc : points_in_camera_fixed) {
    pc.statistics().allocate();
  }
  for (PointIntensityDescriptor3f& pw : points_in_world_moving) {
    pw.statistics().allocate();
  }
  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_property);
  //  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&scene_property);
  //  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  //  aligner->setPlatform(platform);
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  //  std::cerr << aligner->iterationStats() << std::endl;
  // ds sizes should not have changed
  ASSERT_EQ(measurements_property.value()->size(),
            static_cast<size_t>(points_in_camera_fixed.size()));
  ASSERT_EQ(scene_property.value()->size(), static_cast<size_t>(points_in_world_moving.size()));

  std::cerr << "estimate: " << geometry3d::t2tnq(aligner->movingInFixed()).transpose() << std::endl;
  std::cerr << "r_in_w  : " << geometry3d::t2tnq(pose).transpose() << std::endl;

  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * pose);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.15);
  ASSERT_LT_ABS(error(1), 0.15);
  ASSERT_LT_ABS(error(2), 0.15);
  ASSERT_LT_ABS(error(3), 0.005);
  ASSERT_LT_ABS(error(4), 0.005);
  ASSERT_LT_ABS(error(5), 0.005);
}

TEST_F(SyntheticWorldWithDescriptorsSE3, AlignerSliceProcessorProjectiveWithSensor) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree2D3DPtr correspondence_finder(
    new CorrespondenceFinderProjectiveKDTree2D3D);
  correspondence_finder->param_maximum_descriptor_distance.setValue(75);
  correspondence_finder->param_minimum_descriptor_distance.setValue(25);
  correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  correspondence_finder->param_maximum_search_radius_pixels.setValue(50);
  correspondence_finder->param_minimum_number_of_points_per_cluster.setValue(10);
  correspondence_finder->param_projector->param_canvas_cols.setValue(canvas_size(0));
  correspondence_finder->param_projector->param_canvas_rows.setValue(canvas_size(1));
  correspondence_finder->param_projector->param_range_min.setValue(0.1);
  correspondence_finder->param_projector->param_range_max.setValue(10);
  correspondence_finder->param_projector->setCameraMatrix(projection_matrix);

  Isometry3f sensor_in_robot    = Isometry3f::Identity();
  sensor_in_robot.translation() = Vector3f(0.2, 0.3, 0.4);
  sensor_in_robot.linear()      = geometry3d::a2r(Vector3f(0, M_PI * 0.05, 0));

  // ds set sensor poses
  Isometry3f pose(Isometry3f::Identity());
  sensor_poses.push_back(pose * sensor_in_robot);
  pose.translation() += Vector3f(0, 0, -1);
  sensor_poses.push_back(pose * sensor_in_robot);

  PlatformPtr platform(new Platform);

  TransformEventPtr tf2_event(
    new TransformEvent(0, "camera_left", sensor_in_robot.template cast<float>(), "base_frame"));
  platform->addEvent(tf2_event);

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_GT(points_in_sensor[0].size(), 90);
  ASSERT_GT(points_in_sensor_projected_in_canvas[0].size(), 90);

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

  // srrg projector for camera information
  AlignerSliceProcessorProjectiveWithSensorPtr slice_projective(
    new AlignerSliceProcessorProjectiveWithSensor);
  slice_projective->param_base_frame_id.setValue("base_frame");
  slice_projective->param_frame_id.setValue("camera_left");
  slice_projective->param_fixed_slice_name.setValue("points");
  slice_projective->param_moving_slice_name.setValue("points");
  slice_projective->param_finder.setValue(correspondence_finder);
  slice_projective->param_projector.setValue(correspondence_finder->param_projector.value());

  MultiAligner3DQRPtr aligner(new MultiAligner3DQR);
  aligner->param_slice_processors.pushBack(slice_projective);
  aligner->param_max_iterations.setValue(10);

  PointIntensityDescriptor2fVectorCloud points_in_camera_fixed =
    points_in_sensor_projected_in_canvas[1];
  PointIntensityDescriptor3fVectorCloud points_in_world_moving = points_in_world;

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor2fVectorCloud*> measurements_property(
    slice_projective->param_fixed_slice_name.value(),
    "",
    nullptr,
    &points_in_camera_fixed,
    nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> scene_property(
    slice_projective->param_moving_slice_name.value(),
    "",
    nullptr,
    &points_in_world_moving,
    nullptr);

  for (PointIntensityDescriptor2f& pc : points_in_camera_fixed) {
    pc.statistics().allocate();
  }
  for (PointIntensityDescriptor3f& pw : points_in_world_moving) {
    pw.statistics().allocate();
  }
  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_property);
  //  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&scene_property);
  //  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  aligner->setPlatform(platform);
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  //  std::cerr << aligner->iterationStats() << std::endl;
  // ds sizes should not have changed
  ASSERT_EQ(measurements_property.value()->size(),
            static_cast<size_t>(points_in_camera_fixed.size()));
  ASSERT_EQ(scene_property.value()->size(), static_cast<size_t>(points_in_world_moving.size()));

  //  std::cerr << "estimate: " << geometry3d::t2tnq(aligner->movingInFixed()).transpose() <<
  //  std::endl; std::cerr << "inv_est : " <<
  //  geometry3d::t2tnq(aligner->movingInFixed()).transpose()
  //            << std::endl;
  //  std::cerr << "r_in_w  : " << geometry3d::t2tnq(pose).transpose() << std::endl;

  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * pose);

  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.15);
  ASSERT_LT_ABS(error(1), 0.15);
  ASSERT_LT_ABS(error(2), 0.15);
  ASSERT_LT_ABS(error(3), 0.005);
  ASSERT_LT_ABS(error(4), 0.005);
  ASSERT_LT_ABS(error(5), 0.005);
}
TEST_F(SyntheticWorldWithDescriptorsSE3, AlignerSliceProcessorProjectiveDepthWithSensor) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  CorrespondenceFinderProjectiveKDTree3D3DPtr correspondence_finder(
    new CorrespondenceFinderProjectiveKDTree3D3D);
  correspondence_finder->param_maximum_descriptor_distance.setValue(75);
  correspondence_finder->param_minimum_descriptor_distance.setValue(25);
  correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  correspondence_finder->param_maximum_search_radius_pixels.setValue(50);
  correspondence_finder->param_minimum_number_of_points_per_cluster.setValue(10);
  correspondence_finder->param_projector->param_canvas_cols.setValue(canvas_size(0));
  correspondence_finder->param_projector->param_canvas_rows.setValue(canvas_size(1));
  correspondence_finder->param_projector->param_range_min.setValue(0.1);
  correspondence_finder->param_projector->param_range_max.setValue(1000);
  correspondence_finder->param_projector->setCameraMatrix(projection_matrix);

  Isometry3f sensor_in_robot    = Isometry3f::Identity();
  sensor_in_robot.translation() = Vector3f(0.2, 0.3, 0.4);
  sensor_in_robot.linear()      = geometry3d::a2r(Vector3f(0, M_PI * 0.05, 0));

  // ds set sensor poses
  Isometry3f pose(Isometry3f::Identity());
  sensor_poses.push_back(pose * sensor_in_robot);
  pose.translation() += Vector3f(0, 0, -1);
  pose.linear().noalias() = geometry3d::a2r(Vector3f(0.1, 0.01, -0.001));
  sensor_poses.push_back(pose * sensor_in_robot);

  PlatformPtr platform(new Platform);

  TransformEventPtr tf2_event(
    new TransformEvent(0, "camera_left", sensor_in_robot.template cast<float>(), "base_frame"));
  platform->addEvent(tf2_event);

  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), 96);
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), 96);

  // ds populate world points with unique random descriptors
  for (PointIntensityDescriptor3f& point_in_world : points_in_world) {
    cv::Mat descriptor(1, 32, CV_8UC1);
    cv::randn(descriptor, cv::Scalar(0), cv::Scalar(100));
    point_in_world.descriptor() = descriptor;
  }

  PointIntensityDescriptor3fVectorCloud points_in_camera_fixed;
  points_in_camera_fixed.reserve(points_in_camera_fixed.size());
  // ds populate corresponding points with matching, random 32-bit descriptors
  for (size_t i = 0; i < 2; ++i) {
    for (const Correspondence& correspondence : correspondences_sensor_to_world[i]) {
      points_in_sensor[i][correspondence.fixed_idx].descriptor() =
        points_in_world[correspondence.moving_idx].descriptor();
    }
    for (const Correspondence& correspondence : correspondences_canvas_to_sensor[i]) {
      auto& pspc        = points_in_sensor_projected_in_canvas[i][correspondence.fixed_idx];
      pspc.descriptor() = points_in_sensor[i][correspondence.moving_idx].descriptor();
      if (i == 1) {
        PointIntensityDescriptor3f p;
        p.coordinates().head<2>() = pspc.coordinates();
        p.coordinates().z() = points_in_sensor[i][correspondence.moving_idx].coordinates().z();
        p.intensity()       = pspc.intensity();
        p.descriptor()      = pspc.descriptor();
        points_in_camera_fixed.emplace_back(p);
      }
    }
  }

  // srrg projector for camera information
  AlignerSliceProcessorProjectiveDepthWithSensorPtr slice_projective(
    new AlignerSliceProcessorProjectiveDepthWithSensor);
  slice_projective->param_base_frame_id.setValue("base_frame");
  slice_projective->param_frame_id.setValue("camera_left");
  slice_projective->param_fixed_slice_name.setValue("points");
  slice_projective->param_moving_slice_name.setValue("points");
  slice_projective->param_finder.setValue(correspondence_finder);
  slice_projective->param_projector.setValue(correspondence_finder->param_projector.value());

  MultiAligner3DQRPtr aligner(new MultiAligner3DQR);
  aligner->param_slice_processors.pushBack(slice_projective);
  aligner->param_max_iterations.setValue(10);
  PointIntensityDescriptor3fVectorCloud points_in_world_moving = points_in_world;

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor3fVectorCloud*> measurements_property(
    slice_projective->param_fixed_slice_name.value(),
    "",
    nullptr,
    &points_in_camera_fixed,
    nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> scene_property(
    slice_projective->param_moving_slice_name.value(),
    "",
    nullptr,
    &points_in_world_moving,
    nullptr);

  for (PointIntensityDescriptor3f& pc : points_in_camera_fixed) {
    pc.statistics().allocate();
  }
  for (PointIntensityDescriptor3f& pw : points_in_world_moving) {
    pw.statistics().allocate();
  }
  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_property);
  //  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&scene_property);
  //  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  aligner->setPlatform(platform);
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();

  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  //  std::cerr << aligner->iterationStats() << std::endl;
  // ds sizes should not have changed
  ASSERT_EQ(measurements_property.value()->size(),
            static_cast<size_t>(points_in_camera_fixed.size()));
  ASSERT_EQ(scene_property.value()->size(), static_cast<size_t>(points_in_world_moving.size()));

  std::cerr << "estimate: " << geometry3d::t2tnq(aligner->movingInFixed().inverse()).transpose()
            << std::endl;
  std::cerr << "r_in_w  : " << geometry3d::t2tnq(pose).transpose() << std::endl;

  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * pose);

  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.15);
  ASSERT_LT_ABS(error(1), 0.15);
  ASSERT_LT_ABS(error(2), 0.15);
  ASSERT_LT_ABS(error(3), 0.005);
  ASSERT_LT_ABS(error(4), 0.005);
  ASSERT_LT_ABS(error(5), 0.005);
}

TEST_F(SyntheticWorldWithDescriptorsSE3, AlignerSliceProcessorProjectiveStereoWithSensor) {
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;
  offset_second_sensor << 0.5, 0, 0;
  CorrespondenceFinderProjectiveKDTree4D3DPtr correspondence_finder(
    new CorrespondenceFinderProjectiveKDTree4D3D);
  correspondence_finder->param_maximum_descriptor_distance.setValue(75);
  correspondence_finder->param_minimum_descriptor_distance.setValue(25);
  correspondence_finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);
  correspondence_finder->param_maximum_search_radius_pixels.setValue(50);
  correspondence_finder->param_minimum_number_of_points_per_cluster.setValue(10);
  correspondence_finder->param_projector->param_canvas_cols.setValue(canvas_size(0));
  correspondence_finder->param_projector->param_canvas_rows.setValue(canvas_size(1));
  correspondence_finder->param_projector->param_range_min.setValue(0.1);
  correspondence_finder->param_projector->param_range_max.setValue(10);
  correspondence_finder->param_projector->setCameraMatrix(projection_matrix);

  sensor_in_robot               = Isometry3f::Identity();
  sensor_in_robot.translation() = Vector3f(0.2, 0.3, 0.4);
  sensor_in_robot.linear()      = geometry3d::a2r(Vector3f(0, M_PI * 0.05, 0));

  // ds set sensor poses
  Isometry3f pose(Isometry3f::Identity());
  sensor_poses.push_back(pose * sensor_in_robot);
  pose.translation() += Vector3f(0.5, -0.5, -1);
  pose.linear().noalias() = geometry3d::a2r(Vector3f(0.1, 0.01, -0.001));
  sensor_poses.push_back(pose * sensor_in_robot);

  PlatformPtr platform(new Platform);

  Isometry3f tf_right_in_left    = Isometry3f::Identity();
  tf_right_in_left.translation() = offset_second_sensor;

  TransformEventPtr tf2_event(new TransformEvent(0, "camera_left", sensor_in_robot, "base_frame"));
  platform->addEvent(tf2_event);

  TransformEventPtr tf1_event(
    new TransformEvent(0, "camera_right", tf_right_in_left, "camera_left"));
  platform->addEvent(tf1_event);

  std::cerr << platform << std::endl;
  // ds generate uniform scenario
  constexpr size_t number_of_points = 100;
  const Vector3f mean(10, 10, 10);
  const Vector3f deviation(5, 5, 5);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_GT(points_in_sensor[0].size(), 90);
  ASSERT_GT(points_in_sensor_projected_in_canvas[0].size(), 90);

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

  // srrg projector for camera information
  AlignerSliceProcessorProjectiveStereoWithSensorPtr slice_projective(
    new AlignerSliceProcessorProjectiveStereoWithSensor);
  slice_projective->param_base_frame_id.setValue("base_frame");
  slice_projective->param_frame_id.setValue("camera_left");
  slice_projective->param_fixed_slice_name.setValue("points");
  slice_projective->param_moving_slice_name.setValue("points");
  slice_projective->param_finder.setValue(correspondence_finder);
  slice_projective->param_projector.setValue(correspondence_finder->param_projector.value());

  MultiAligner3DQRPtr aligner(new MultiAligner3DQR);
  aligner->param_slice_processors.pushBack(slice_projective);
  aligner->param_max_iterations.setValue(10);

  size_t number_points_in_sensor = points_in_sensor_projected_in_canvas[1].size();
  PointIntensityDescriptor4fVectorCloud points_in_camera_fixed;
  points_in_camera_fixed.reserve(number_points_in_sensor);
  for (size_t i = 0; i < number_points_in_sensor; ++i) {
    PointIntensityDescriptor4f p;
    p.coordinates().head<2>() << points_in_sensor_projected_in_canvas[1][i].coordinates();
    p.coordinates().tail<2>() << points_in_sensor_projected_in_canvas_second[1][i].coordinates();
    p.descriptor() = points_in_sensor_projected_in_canvas[1][i].descriptor();
    points_in_camera_fixed.emplace_back(p);
  }
  PointIntensityDescriptor3fVectorCloud points_in_world_moving = points_in_world;

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor4fVectorCloud*> measurements_property(
    slice_projective->param_fixed_slice_name.value(),
    "",
    nullptr,
    &points_in_camera_fixed,
    nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> scene_property(
    slice_projective->param_moving_slice_name.value(),
    "",
    nullptr,
    &points_in_world_moving,
    nullptr);

  for (PointIntensityDescriptor4f& pc : points_in_camera_fixed) {
    pc.statistics().allocate();
  }
  for (PointIntensityDescriptor3f& pw : points_in_world_moving) {
    pw.statistics().allocate();
  }
  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_property);
  //  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&scene_property);
  //  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  aligner->setPlatform(platform);
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  //  std::cerr << aligner->iterationStats() << std::endl;
  // ds sizes should not have changed
  ASSERT_EQ(measurements_property.value()->size(),
            static_cast<size_t>(points_in_camera_fixed.size()));
  ASSERT_EQ(scene_property.value()->size(), static_cast<size_t>(points_in_world_moving.size()));

  std::cerr << "w_in_r_est: " << geometry3d::t2tnq(aligner->movingInFixed()).transpose()
            << std::endl;

  std::cerr << "r_in_w_est: " << geometry3d::t2tnq(aligner->movingInFixed().inverse()).transpose()
            << std::endl;
  std::cerr << "r_in_w    : " << geometry3d::t2tnq(pose).transpose() << std::endl;

  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * pose);

  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.15);
  ASSERT_LT_ABS(error(1), 0.15);
  ASSERT_LT_ABS(error(2), 0.15);
  ASSERT_LT_ABS(error(3), 0.005);
  ASSERT_LT_ABS(error(4), 0.005);
  ASSERT_LT_ABS(error(5), 0.005);
}

TEST_F(KITTI, 00To01_SE3StereoPositErrorFactorInfoDiagonal_CFGT) {
  Solver solver;
  ASSERT_NOTNULL(solver.param_algorithm.value());
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(100);

  // ds allocate robustifier (same for all factors)
  RobustifierSaturated robustifier;
  robustifier.param_chi_threshold.setValue(1000);

  // ds create a stereo posit factor
  StereoFactorPtr factor(new StereoFactor());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(Vector2f(image_cols, image_rows));
  factor->setBaselineLeftInRightPixels(
    -baseline_right_in_left_pixels /*we need the l_in_r -> we put a minus*/);
  factor->setFixed(measurements[1]);
  factor->setMoving(points_in_camera_00);
  factor->setCorrespondences(correspondences_camera_01_from_00);
  factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 3>(1, 2, 1));
  factor->setRobustifier(&robustifier);
  ASSERT_EQ(factor->size(), correspondences_camera_01_from_00.size());

  // ds setup graph
  FactorGraphPtr graph(new FactorGraph());

  // create a variable, set an id and add it to the graph
  std::shared_ptr<VariableSE3QuaternionRightAD> variable(new VariableSE3QuaternionRightAD());
  variable->setGraphId(0);
  graph->addVariable(variable);
  solver.setGraph(graph);

  // ds hook up factor, graph and solver
  graph->addFactor(factor);
  graph->bindFactors();
  ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

  // ds optimize from zero guess
  variable->setEstimate(Isometry3f::Identity());
  solver.compute();
  const Vector6f error = geometry3d::t2tnq(variable->estimate() * camera_01_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.15);
  ASSERT_LT_ABS(error(1), 0.15);
  ASSERT_LT_ABS(error(2), 0.15);
  ASSERT_LT_ABS(error(3), 0.005);
  ASSERT_LT_ABS(error(4), 0.005);
  ASSERT_LT_ABS(error(5), 0.005);
}

TEST_F(KITTI, 00To01_SE3StereoPositErrorFactorInfoDiagonal_CFProjectiveBF) {
  // ds compute correspondences from adapted measurements
  CorrespondenceFinderProjectiveCircle4D3D finder;
  finder.param_projector.setValue(projector);
  finder.param_minimum_descriptor_distance.setValue(100);
  finder.param_maximum_descriptor_distance.setValue(100);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_minimum_matching_ratio.setValue(0.1);
  finder.param_minimum_number_of_iterations.setValue(10);
  finder.param_minimum_search_radius_pixels.setValue(5);
  finder.param_maximum_search_radius_pixels.setValue(5);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds set perfect estimate
  finder.setLocalMapInSensor(camera_01_in_00.inverse());

  CorrespondenceVector correspondences_01_from_00;
  finder.setFixed(&measurements[1]);
  finder.setMoving(&points_in_camera_00);
  finder.setCorrespondences(&correspondences_01_from_00);
  for (size_t i = 0; i < 100; ++i) {
    finder.compute();
  }

  Solver solver;
  ASSERT_NOTNULL(solver.param_algorithm.value());
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(100);

  // ds allocate robustifier (same for all factors)
  RobustifierSaturated robustifier;
  robustifier.param_chi_threshold.setValue(1000);

  // ds create a stereo posit factor
  StereoFactorPtr factor(new StereoFactor());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(Vector2f(image_cols, image_rows));
  factor->setBaselineLeftInRightPixels(
    -baseline_right_in_left_pixels /*we need the l_in_r -> we put a minus*/);
  factor->setFixed(measurements[1]);
  factor->setMoving(points_in_camera_00);
  factor->setCorrespondences(correspondences_01_from_00);
  factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 3>(1, 2, 1));
  factor->setRobustifier(&robustifier);
  ASSERT_EQ(factor->size(), correspondences_01_from_00.size());

  // ds compute disparity average
  float accumulated_disparity_pixels = 0;
  std::vector<float> disparities;
  for (const Correspondence& correspondence : correspondences_01_from_00) {
    const float disparity_pixels = measurements[1][correspondence.fixed_idx].coordinates()(0) -
                                   measurements[1][correspondence.fixed_idx].coordinates()(2);
    accumulated_disparity_pixels += disparity_pixels;
    disparities.push_back(disparity_pixels);
  }
  std::cerr << "# correspondences: " << disparities.size() << std::endl;
  const float mean_disparity_pixels = accumulated_disparity_pixels / disparities.size();
  std::cerr << "mean disparity (px): " << mean_disparity_pixels << std::endl;

  // ds optimize from zero guess without individual point weighting
  {
    // create a variable, set an id and add it to the graph
    FactorGraphPtr graph(new FactorGraph());
    std::shared_ptr<VariableSE3QuaternionRightAD> variable(new VariableSE3QuaternionRightAD());
    variable->setGraphId(0);
    graph->addVariable(variable);
    solver.setGraph(graph);
    graph->addFactor(factor);
    graph->bindFactors();
    ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

    // ds optimize
    variable->setEstimate(Isometry3f::Identity());
    solver.compute();
    const Vector6f error = geometry3d::t2tnq(variable->estimate() * camera_01_in_00);
    std::cerr << "error (manifold): " << error.transpose() << std::endl;
    ASSERT_LT_ABS(error(0), 0.1);
    ASSERT_LT_ABS(error(1), 0.1);
    ASSERT_LT_ABS(error(2), 0.1);
    ASSERT_LT_ABS(error(3), 0.005);
    ASSERT_LT_ABS(error(4), 0.005);
    ASSERT_LT_ABS(error(5), 0.005);
  }

  // ds optimize from zero guess with individual point weighting
  {
    // create a variable, set an id and add it to the graph
    FactorGraphPtr graph(new FactorGraph());
    std::shared_ptr<VariableSE3QuaternionRightAD> variable(new VariableSE3QuaternionRightAD());
    variable->setGraphId(0);
    graph->addVariable(variable);
    solver.setGraph(graph);
    graph->addFactor(factor);
    graph->bindFactors();
    ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

    // ds set individual weights to factors
    factor->setMeanDisparityPixels(mean_disparity_pixels);

    // ds optimize
    variable->setEstimate(Isometry3f::Identity());
    solver.compute();
    const Vector6f error = geometry3d::t2tnq(variable->estimate() * camera_01_in_00);
    std::cerr << "error (manifold): " << error.transpose() << " (WEIGHTED)" << std::endl;
    ASSERT_LT_ABS(error(0), 0.1);
    ASSERT_LT_ABS(error(1), 0.1);
    ASSERT_LT_ABS(error(2), 0.1);
    ASSERT_LT_ABS(error(3), 0.005);
    ASSERT_LT_ABS(error(4), 0.005);
    ASSERT_LT_ABS(error(5), 0.005);
  }
}

TEST_F(KITTI, Highway_274To275_SE3StereoPositErrorFactorInfoDiagonal_CFProjectiveBF) {
  // ds compute correspondences from adapted measurements
  CorrespondenceFinderProjectiveCircle4D3D finder;
  finder.param_projector.setValue(projector);
  finder.param_minimum_descriptor_distance.setValue(100);
  finder.param_maximum_descriptor_distance.setValue(100);
  finder.param_maximum_distance_ratio_to_second_best.setValue(0.5);
  finder.param_minimum_matching_ratio.setValue(0.1);
  finder.param_minimum_number_of_iterations.setValue(10);
  finder.param_minimum_search_radius_pixels.setValue(5);
  finder.param_maximum_search_radius_pixels.setValue(5);

  // ds allocate statistics fields of points in camera 274
  for (PointIntensityDescriptor3f& point : highway_points_in_camera_274) {
    point.statistics().allocate();
  }

  // ds set perfect estimate
  finder.setLocalMapInSensor(highway_camera_275_in_274.inverse());

  CorrespondenceVector correspondences_275_from_274;
  finder.setFixed(&highway_measurements[1]);
  finder.setMoving(&highway_points_in_camera_274);
  finder.setCorrespondences(&correspondences_275_from_274);
  for (size_t i = 0; i < 100; ++i) {
    finder.compute();
  }

  Solver solver;
  ASSERT_NOTNULL(solver.param_algorithm.value());
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(100);

  // ds allocate robustifier (same for all factors)
  RobustifierSaturated robustifier;
  robustifier.param_chi_threshold.setValue(1000);

  // ds create a stereo posit factor
  StereoFactorPtr factor(new StereoFactor());
  factor->setVariableId(0, 0);

  // setup the factor
  factor->setEnabled(true);
  factor->setCameraMatrix(camera_calibration_matrix);
  factor->setImageDim(Vector2f(image_cols, image_rows));
  factor->setBaselineLeftInRightPixels(
    -baseline_right_in_left_pixels /*we need the l_in_r -> we put a minus*/);
  factor->setFixed(highway_measurements[1]);
  factor->setMoving(highway_points_in_camera_274);
  factor->setCorrespondences(correspondences_275_from_274);
  factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 3>(1, 2, 1));
  factor->setRobustifier(&robustifier);
  ASSERT_EQ(factor->size(), correspondences_275_from_274.size());

  // ds compute disparity average
  float accumulated_disparity_pixels = 0;
  std::vector<float> disparities;
  for (const Correspondence& correspondence : correspondences_275_from_274) {
    const float disparity_pixels =
      highway_measurements[1][correspondence.fixed_idx].coordinates()(0) -
      highway_measurements[1][correspondence.fixed_idx].coordinates()(2);
    accumulated_disparity_pixels += disparity_pixels;
    disparities.push_back(disparity_pixels);
  }
  std::cerr << "# correspondences: " << disparities.size() << std::endl;
  const float mean_disparity_pixels = accumulated_disparity_pixels / disparities.size();
  std::cerr << "mean disparity (px): " << mean_disparity_pixels << std::endl;

  // ds optimize from zero guess without individual point weighting
  {
    // create a variable, set an id and add it to the graph
    FactorGraphPtr graph(new FactorGraph());
    std::shared_ptr<VariableSE3QuaternionRightAD> variable(new VariableSE3QuaternionRightAD());
    variable->setGraphId(0);
    graph->addVariable(variable);
    solver.setGraph(graph);
    graph->addFactor(factor);
    graph->bindFactors();
    ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

    // ds optimize
    variable->setEstimate(Isometry3f::Identity());
    solver.compute();
    const Vector6f error = geometry3d::t2tnq(variable->estimate() * highway_camera_275_in_274);
    std::cerr << "error (manifold): " << error.transpose() << std::endl;
    ASSERT_LT_ABS(error(0), 0.1);
    ASSERT_LT_ABS(error(1), 0.1);
    ASSERT_LT_ABS(error(2), 0.1);
    ASSERT_LT_ABS(error(3), 0.005);
    ASSERT_LT_ABS(error(4), 0.005);
    ASSERT_LT_ABS(error(5), 0.005);
  }

  // ds optimize from zero guess with individual point weighting
  {
    // create a variable, set an id and add it to the graph
    FactorGraphPtr graph(new FactorGraph());
    std::shared_ptr<VariableSE3QuaternionRightAD> variable(new VariableSE3QuaternionRightAD());
    variable->setGraphId(0);
    graph->addVariable(variable);
    solver.setGraph(graph);
    graph->addFactor(factor);
    graph->bindFactors();
    ASSERT_EQ(graph->factors().size(), static_cast<size_t>(1));

    // ds set individual weights to factors
    factor->setMeanDisparityPixels(mean_disparity_pixels);

    // ds optimize
    variable->setEstimate(Isometry3f::Identity());
    solver.compute();
    const Vector6f error = geometry3d::t2tnq(variable->estimate() * highway_camera_275_in_274);
    std::cerr << "error (manifold): " << error.transpose() << " (WEIGHTED)" << std::endl;
    ASSERT_LT_ABS(error(0), 0.1);
    ASSERT_LT_ABS(error(1), 0.1);
    ASSERT_LT_ABS(error(2), 0.1);
    ASSERT_LT_ABS(error(3), 0.005);
    ASSERT_LT_ABS(error(4), 0.005);
    ASSERT_LT_ABS(error(5), 0.005);
  }
}

TEST_F(ICL, 00To50_AlignerProjective_Bruteforce) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "icl.conf");
  MultiAligner3DQRPtr aligner = manager.getByName<MultiAligner3DQR>("aligner");
  ASSERT_NOTNULL(aligner);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveDepthPtr slice_projective_depth =
    std::dynamic_pointer_cast<AlignerSliceProcessorProjectiveDepth>(
      aligner->param_slice_processors.getSharedPtr(0));
  ASSERT_NOTNULL(slice_projective_depth);

  // ds configure a projective only slice
  AlignerSliceProcessorProjectivePtr slice_projective(new AlignerSliceProcessorProjective());
  ASSERT_NOTNULL(slice_projective);
  ASSERT_NOTNULL(slice_projective->param_finder.value());
  ASSERT_NOTNULL(slice_projective->param_robustifier.value());
  aligner->param_slice_processors.setValue(0, slice_projective);

  // ds set bruteforce matcher to slice
  CorrespondenceFinderDescriptorBasedBruteforce2D3DPtr finder =
    manager.getByName<CorrespondenceFinderDescriptorBasedBruteforce2D3D>("cf_bruteforce_2d");
  ASSERT_NOTNULL(finder);
  slice_projective->param_finder.setValue(finder);
  slice_projective->param_projector.setValue(projector);
  slice_projective->param_diagonal_info_matrix.setValue(Vector2f(1, 1));

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor2fVectorCloud*> measurements_projective_50_as_property(
    slice_projective->param_fixed_slice_name.value(),
    "",
    nullptr,
    &measurements_projective_50,
    nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> points_in_camera_00_as_property(
    slice_projective->param_moving_slice_name.value(), "", nullptr, &points_in_camera_00, nullptr);
  srrg2_core::StdDequeEigenIsometry3f trajectory_chunk;
  Property_<StdDequeEigenIsometry3f*> trajectory_chunk_as_property(
    "trajectory_chunk", "", nullptr, &trajectory_chunk, nullptr);

  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_projective_50_as_property);
  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&points_in_camera_00_as_property);
  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  // ds sizes should not have changed
  ASSERT_EQ(measurements_projective_50_as_property.value()->size(),
            static_cast<size_t>(measurements_projective_50.size()));
  ASSERT_EQ(points_in_camera_00_as_property.value()->size(),
            static_cast<size_t>(points_in_camera_00.size()));

  // ds validate relative error in translation and rotation
  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * camera_50_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.01);
  ASSERT_LT_ABS(error(1), 0.01);
  ASSERT_LT_ABS(error(2), 0.01);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(ICL, 00To50_AlignerProjectiveDepth_Bruteforce) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "icl.conf");
  MultiAligner3DQRPtr aligner = manager.getByName<MultiAligner3DQR>("aligner");
  ASSERT_NOTNULL(aligner);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveDepthPtr slice =
    std::dynamic_pointer_cast<AlignerSliceProcessorProjectiveDepth>(
      aligner->param_slice_processors.getSharedPtr(0));
  ASSERT_NOTNULL(slice);
  ASSERT_NOTNULL(slice->param_robustifier.value());

  // ds set bruteforce matcher to slice
  CorrespondenceFinderDescriptorBasedBruteforce3D3DPtr finder =
    manager.getByName<CorrespondenceFinderDescriptorBasedBruteforce3D3D>("cf_bruteforce_3d");
  ASSERT_NOTNULL(finder);
  slice->param_finder.setValue(finder);
  slice->param_projector.setValue(projector);
  slice->param_diagonal_info_matrix.setValue(Vector3f(1, 1, 10));

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor3fVectorCloud*> measurements_projective_50_as_property(
    slice->param_fixed_slice_name.value(), "", nullptr, &measurements_projective_depth_50, nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> points_in_camera_00_as_property(
    slice->param_moving_slice_name.value(), "", nullptr, &points_in_camera_00, nullptr);
  srrg2_core::StdDequeEigenIsometry3f trajectory_chunk;
  Property_<StdDequeEigenIsometry3f*> trajectory_chunk_as_property(
    "trajectory_chunk", "", nullptr, &trajectory_chunk, nullptr);

  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_projective_50_as_property);
  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&points_in_camera_00_as_property);
  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  // ds sizes should not have changed
  ASSERT_EQ(measurements_projective_50_as_property.value()->size(),
            static_cast<size_t>(measurements_projective_depth_50.size()));
  ASSERT_EQ(points_in_camera_00_as_property.value()->size(),
            static_cast<size_t>(points_in_camera_00.size()));

  // ds validate relative error in translation and rotation
  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * camera_50_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.01);
  ASSERT_LT_ABS(error(1), 0.01);
  ASSERT_LT_ABS(error(2), 0.01);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(ICL, 00To50_AlignerProjectiveDepth_ProjectiveBF) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "icl.conf");
  MultiAligner3DQRPtr aligner = manager.getByName<MultiAligner3DQR>("aligner");
  ASSERT_NOTNULL(aligner);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveDepthPtr slice =
    std::dynamic_pointer_cast<AlignerSliceProcessorProjectiveDepth>(
      aligner->param_slice_processors.getSharedPtr(0));
  ASSERT_NOTNULL(slice);
  ASSERT_NOTNULL(slice->param_robustifier.value());

  // ds configure finder
  CorrespondenceFinderProjectiveCircle3D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle3D3D>("cf_projective_circle");
  ASSERT_NOTNULL(finder);
  finder->param_projector.setValue(projector);
  slice->param_finder.setValue(finder);
  slice->param_projector.setValue(projector);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurements(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor3fVectorCloud*> measurements_projective_50_as_property(
    slice->param_fixed_slice_name.value(), "", nullptr, &measurements_projective_depth_50, nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> points_in_camera_00_as_property(
    slice->param_moving_slice_name.value(), "", nullptr, &points_in_camera_00, nullptr);
  srrg2_core::StdDequeEigenIsometry3f trajectory_chunk;
  Property_<StdDequeEigenIsometry3f*> trajectory_chunk_as_property(
    "trajectory_chunk", "", nullptr, &trajectory_chunk, nullptr);

  // ds set map (moving) and measurement (fixed properties)
  measurements->addProperty(&measurements_projective_50_as_property);
  measurements->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&points_in_camera_00_as_property);
  map->addProperty(&trajectory_chunk_as_property);

  // ds set aligner working buffers
  aligner->setFixed(measurements);
  aligner->setMoving(map);
  aligner->setMovingInFixed(camera_50_in_00);
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  // ds sizes should not have changed
  ASSERT_EQ(measurements_projective_50_as_property.value()->size(),
            static_cast<size_t>(measurements_projective_depth_50.size()));
  ASSERT_EQ(points_in_camera_00_as_property.value()->size(),
            static_cast<size_t>(points_in_camera_00.size()));

  // ds validate relative error in translation and rotation
  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * camera_50_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.01);
  ASSERT_LT_ABS(error(1), 0.01);
  ASSERT_LT_ABS(error(2), 0.01);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To01_Aligner_Bruteforce) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  MultiAligner3DQRPtr aligner = manager.getByName<MultiAligner3DQR>("aligner");
  ASSERT_NOTNULL(aligner);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr slice =
    std::dynamic_pointer_cast<AlignerSliceProcessorProjectiveStereo>(
      aligner->param_slice_processors.getSharedPtr(0));
  ASSERT_NOTNULL(slice);
  ASSERT_NOTNULL(slice->param_robustifier.value());
  slice->param_robustifier->param_chi_threshold.setValue(1000);
  slice->setPlatform(_platform);

  // ds configure and set bruteforce matcher to slice
  CorrespondenceFinderDescriptorBasedBruteforce4D3DPtr finder(
    new CorrespondenceFinderDescriptorBasedBruteforce4D3D());
  ASSERT_NOTNULL(finder);
  slice->param_finder.setValue(finder);
  slice->param_projector.setValue(projector);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurement_container(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor4fVectorCloud*> measurements_01_as_property(
    slice->param_fixed_slice_name.value(), "", nullptr, &measurements[1], nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> points_in_camera_00_as_property(
    slice->param_moving_slice_name.value(), "", nullptr, &points_in_camera_00, nullptr);
  srrg2_core::StdDequeEigenIsometry3f trajectory_chunk;
  Property_<StdDequeEigenIsometry3f*> trajectory_chunk_as_property(
    "trajectory_chunk", "", nullptr, &trajectory_chunk, nullptr);

  // ds set map (moving) and measurement (fixed properties)
  measurement_container->addProperty(&measurements_01_as_property);
  measurement_container->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&points_in_camera_00_as_property);
  map->addProperty(&trajectory_chunk_as_property);

  // ds test configuration
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);

  // ds set aligner working buffers
  aligner->setFixed(measurement_container);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  // ds sizes should not have changed
  ASSERT_EQ(measurements_01_as_property.value()->size(),
            static_cast<size_t>(measurements[1].size()));
  ASSERT_EQ(points_in_camera_00_as_property.value()->size(),
            static_cast<size_t>(points_in_camera_00.size()));

  // ds validate relative error in translation and rotation
  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * camera_01_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.1);
  ASSERT_LT_ABS(error(1), 0.1);
  ASSERT_LT_ABS(error(2), 0.20);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To01_Aligner_ProjectiveCircle) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  MultiAligner3DQRPtr aligner = manager.getByName<MultiAligner3DQR>("aligner");
  ASSERT_NOTNULL(aligner);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr slice =
    std::dynamic_pointer_cast<AlignerSliceProcessorProjectiveStereo>(
      aligner->param_slice_processors.getSharedPtr(0));
  ASSERT_NOTNULL(slice);
  ASSERT_NOTNULL(slice->param_robustifier.value());
  slice->param_robustifier->param_chi_threshold.setValue(1000);
  slice->setPlatform(_platform);

  // ds configure finder to test projector
  CorrespondenceFinderProjectiveCircle4D3DPtr finder =
    manager.getByName<CorrespondenceFinderProjectiveCircle4D3D>("cf_projective_circle");
  finder->param_projector.setValue(projector);
  slice->param_finder.setValue(finder);
  ASSERT_NOTNULL(finder);
  slice->param_projector.setValue(projector);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurement_container(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor4fVectorCloud*> measurements_01_as_property(
    slice->param_fixed_slice_name.value(), "", nullptr, &measurements[1], nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> points_in_camera_00_as_property(
    slice->param_moving_slice_name.value(), "", nullptr, &points_in_camera_00, nullptr);
  srrg2_core::StdDequeEigenIsometry3f trajectory_chunk;
  Property_<StdDequeEigenIsometry3f*> trajectory_chunk_as_property(
    "trajectory_chunk", "", nullptr, &trajectory_chunk, nullptr);

  // ds set map (moving) and measurement (fixed properties)
  measurement_container->addProperty(&measurements_01_as_property);
  measurement_container->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&points_in_camera_00_as_property);
  map->addProperty(&trajectory_chunk_as_property);

  // ds test configuration
  finder->param_minimum_descriptor_distance.setValue(50);
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.8);
  finder->param_minimum_search_radius_pixels.setValue(10);
  finder->param_maximum_search_radius_pixels.setValue(50);
  finder->param_number_of_solver_iterations_per_projection.setValue(5);

  // ds set aligner working buffers
  aligner->setFixed(measurement_container);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  aligner->compute();
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  // ds sizes must not have changed
  ASSERT_EQ(measurements_01_as_property.value()->size(),
            static_cast<size_t>(measurements[1].size()));
  ASSERT_EQ(points_in_camera_00_as_property.value()->size(),
            static_cast<size_t>(points_in_camera_00.size()));

  // ds validate relative error in translation and rotation
  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * camera_01_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.05);
  ASSERT_LT_ABS(error(1), 0.05);
  ASSERT_LT_ABS(error(2), 0.20);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(KITTI, 00To02_Aligner_Bruteforce) {
  ConfigurableManager manager;
  manager.read(_filepath_configurations + "/" + "kitti.conf");
  MultiAligner3DQRPtr aligner = manager.getByName<MultiAligner3DQR>("aligner");
  ASSERT_NOTNULL(aligner);
  ASSERT_NOTNULL(aligner->param_solver.value());
  ASSERT_EQ(aligner->param_slice_processors.size(), 2);
  AlignerSliceProcessorProjectiveStereoPtr slice =
    std::dynamic_pointer_cast<AlignerSliceProcessorProjectiveStereo>(
      aligner->param_slice_processors.getSharedPtr(0));
  ASSERT_NOTNULL(slice);
  ASSERT_NOTNULL(slice->param_robustifier.value());
  slice->param_robustifier->param_chi_threshold.setValue(1000);
  slice->setPlatform(_platform);

  // ds set bruteforce matcher to slice
  CorrespondenceFinderDescriptorBasedBruteforce4D3DPtr finder(
    new CorrespondenceFinderDescriptorBasedBruteforce4D3D());
  slice->param_finder.setValue(finder);
  slice->param_projector.setValue(projector);

  // ds allocate statistics fields of points in camera 00
  for (PointIntensityDescriptor3f& point : points_in_camera_00) {
    point.statistics().allocate();
  }

  // ds allocate scene properties TODO uagh kill it with fire
  // ds the dynamic containers need to be dynamically allocated as they are freed internally!
  PropertyContainerDynamic* measurement_container(new PropertyContainerDynamic());
  Property_<PointIntensityDescriptor4fVectorCloud*> measurements_02_as_property(
    slice->param_fixed_slice_name.value(), "", nullptr, &measurements[2], nullptr);
  PropertyContainerDynamic* map = new PropertyContainerDynamic();
  Property_<PointIntensityDescriptor3fVectorCloud*> points_in_camera_00_as_property(
    slice->param_moving_slice_name.value(), "", nullptr, &points_in_camera_00, nullptr);
  srrg2_core::StdDequeEigenIsometry3f trajectory_chunk;
  Property_<StdDequeEigenIsometry3f*> trajectory_chunk_as_property(
    "trajectory_chunk", "", nullptr, &trajectory_chunk, nullptr);

  // ds set map (moving) and measurement (fixed properties)
  measurement_container->addProperty(&measurements_02_as_property);
  measurement_container->addProperty(&trajectory_chunk_as_property);
  map->addProperty(&points_in_camera_00_as_property);
  map->addProperty(&trajectory_chunk_as_property);

  // ds test configuration
  finder->param_maximum_descriptor_distance.setValue(100);
  finder->param_maximum_distance_ratio_to_second_best.setValue(0.5);

  // ds set aligner working buffers
  aligner->setFixed(measurement_container);
  aligner->setMoving(map);
  aligner->setMovingInFixed(Isometry3f::Identity());
  ASSERT_EQ(aligner->status(), AlignerBase::Status::Fail);

  // ds compute relative transform between fixed and moving
  std::cerr << "computing estimate" << std::endl;
  aligner->compute();
  std::cerr << "computed estimate" << std::endl;
  ASSERT_TRUE(aligner->status() == AlignerBase::Success);

  // ds sizes should not have changed
  ASSERT_EQ(measurements_02_as_property.value()->size(),
            static_cast<size_t>(measurements[2].size()));
  ASSERT_EQ(points_in_camera_00_as_property.value()->size(),
            static_cast<size_t>(points_in_camera_00.size()));

  // ds validate relative error in translation and rotation
  const Vector6f error = geometry3d::t2tnq(aligner->movingInFixed() * camera_02_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.1);
  ASSERT_LT_ABS(error(1), 0.1);
  ASSERT_LT_ABS(error(2), 0.35);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}
