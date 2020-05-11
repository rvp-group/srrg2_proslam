#include "fixtures.hpp"
#include "srrg2_proslam/mapping/landmarks/landmark_estimator_ekf.h"
#include "srrg2_proslam/mapping/landmarks/landmark_estimator_pose_based_smoother.h"
#include "srrg2_proslam/mapping/landmarks/landmark_estimator_weighted_mean.h"

#include "srrg2_proslam/mapping/instances.cpp"

using namespace srrg2_core;
using namespace srrg2_test;

// ds extend synthetic world for landmark tests to compress code
using SyntheticWorldDescriptorsSE3 =
  SyntheticWorld<3, float, PointIntensityDescriptor3f, PointIntensityDescriptor2f>;
class LandmarkWorldNoNoise : public SyntheticWorldDescriptorsSE3 {
public:
  void createWorldWithLandmarks(PointIntensityDescriptor3fVectorCloud& landmarks_,
                                std::unordered_map<size_t, size_t>& indices_world_to_landmark_);
  void validateLandmarks(const PointIntensityDescriptor3fVectorCloud& landmarks_) const;
  const Vector3f getPointUnprojected(const Vector2f& point_in_image_,
                                     const float& depth_meters_) const;
};
class LandmarkWorldGaussianNoise : public SyntheticWorldDescriptorsSE3 {};

int main(int argc_, char** argv_) {
  Profiler::enable_logging = true;
  return srrg2_test::runTests(argc_, argv_);
}

TEST_F(LandmarkWorldNoNoise, LandmarkEstimatorWeightedMean3D3D) {
  // ds create world and landmarks in that world (from first frame)
  PointIntensityDescriptor3fVectorCloud landmarks;
  std::unordered_map<size_t, size_t> indices_world_to_landmark;
  createWorldWithLandmarks(landmarks, indices_world_to_landmark);

  // ds landmark point estimator to test
  LandmarkEstimatorWeightedMean3D3D estimator;

  // ds update landmarks for the remaining poses (and observations)
  for (size_t index_pose = 1; index_pose < sensor_poses.size(); ++index_pose) {
    const PointIntensityDescriptor3fVectorCloud& points_in_sensor_current(
      points_in_sensor[index_pose]);
    const PointIntensityDescriptor2fVectorCloud& points_in_image(
      points_in_sensor_projected[index_pose]);

    // ds sensor pose is the same for all correspondences
    estimator.setTransforms(sensor_poses[index_pose], sensor_poses[index_pose]);

    // ds for all correspondences of the current pose
    for (const Correspondence& correspondence : correspondences_sensor_to_world[index_pose]) {
      const size_t index_measurement = correspondence.fixed_idx;
      auto iterator                  = indices_world_to_landmark.find(correspondence.moving_idx);
      if (iterator != indices_world_to_landmark.end()) {
        // ds unproject measurements to retrieve 3D point measurement
        const float depth_meters(points_in_sensor_current[index_measurement].coordinates()(2));
        ASSERT_GT(depth_meters, 0);
        const Vector3f point_in_camera(
          getPointUnprojected(points_in_image[index_measurement].coordinates(), depth_meters));

        // ds update landmark position estimate using the estimator
        estimator.setLandmarkInSensor(point_in_camera);
        estimator.setLandmark(&landmarks[iterator->second]);
        estimator.compute();
      }
    }
  }

  // ds validate final landmark estimates
  validateLandmarks(landmarks);
}

TEST_F(LandmarkWorldNoNoise, LandmarkEstimatorProjectiveEKF3D) {
  // ds create world and landmarks in that world (from first frame)
  PointIntensityDescriptor3fVectorCloud landmarks;
  std::unordered_map<size_t, size_t> indices_world_to_landmark;
  createWorldWithLandmarks(landmarks, indices_world_to_landmark);

  // ds landmark point estimator to test
  LandmarkEstimatorProjectiveEKF3D estimator;
  ProjectivePointEKF3DPtr filter(new ProjectivePointEKF3D());
  filter->setCameraMatrix(projection_matrix.cast<double>());
  estimator.param_filter.setValue(filter);
  estimator.param_minimum_state_element_covariance.setValue(0.01);

  // ds update landmarks for the remaining poses (and observations)
  for (size_t index_pose = 1; index_pose < sensor_poses.size(); ++index_pose) {
    const PointIntensityDescriptor2fVectorCloud& points_in_image(
      points_in_sensor_projected[index_pose]);

    // ds update filter for current pose
    estimator.setTransforms(sensor_poses[index_pose], sensor_poses[index_pose]);

    // ds for all correspondences of the current pose
    for (const Correspondence& correspondence : correspondences_sensor_to_world[index_pose]) {
      const size_t index_measurement = correspondence.fixed_idx;
      auto iterator                  = indices_world_to_landmark.find(correspondence.moving_idx);
      if (iterator != indices_world_to_landmark.end()) {
        // ds adapt measurement (merge phase state)
        const PointIntensityDescriptor2f& point_in_image(points_in_image[index_measurement]);

        // ds update landmark position estimate using the estimator
        estimator.setMeasurement(point_in_image.coordinates());
        estimator.setLandmark(&landmarks[iterator->second]);
        estimator.compute();
      }
    }
  }

  // ds validate final landmark estimates
  validateLandmarks(landmarks);
}

TEST_F(LandmarkWorldNoNoise, LandmarkEstimatorProjectiveDepthEKF3D) {
  // ds create world and landmarks in that world (from first frame)
  PointIntensityDescriptor3fVectorCloud landmarks;
  std::unordered_map<size_t, size_t> indices_world_to_landmark;
  createWorldWithLandmarks(landmarks, indices_world_to_landmark);

  // ds landmark point estimator to test
  LandmarkEstimatorProjectiveDepthEKF3D estimator;
  ProjectiveDepthPointEKF3DPtr filter(new ProjectiveDepthPointEKF3D());
  filter->setCameraMatrix(projection_matrix.cast<double>());
  estimator.param_filter.setValue(filter);
  estimator.param_minimum_state_element_covariance.setValue(0.01);

  // ds update landmarks for the remaining poses (and observations)
  for (size_t index_pose = 1; index_pose < sensor_poses.size(); ++index_pose) {
    const PointIntensityDescriptor3fVectorCloud& points_in_sensor_current(
      points_in_sensor[index_pose]);
    const PointIntensityDescriptor2fVectorCloud& points_in_image(
      points_in_sensor_projected[index_pose]);

    // ds update filter for current pose
    estimator.setTransforms(sensor_poses[index_pose], sensor_poses[index_pose]);

    // ds for all correspondences of the current pose
    for (const Correspondence& correspondence : correspondences_sensor_to_world[index_pose]) {
      const size_t index_measurement = correspondence.fixed_idx;
      auto iterator                  = indices_world_to_landmark.find(correspondence.moving_idx);
      if (iterator != indices_world_to_landmark.end()) {
        // ds adapt measurement (merge phase state)
        const PointIntensityDescriptor2f& point_in_image(points_in_image[index_measurement]);
        const float depth_meters(points_in_sensor_current[index_measurement].coordinates()(2));
        ASSERT_GT(depth_meters, 0);
        const Vector3f measurement_uvd(
          point_in_image.coordinates()(0), point_in_image.coordinates()(1), depth_meters);

        // ds update landmark position estimate using the estimator
        estimator.setMeasurement(measurement_uvd);
        estimator.setLandmark(&landmarks[iterator->second]);
        estimator.compute();
      }
    }
  }

  // ds validate final landmark estimates
  validateLandmarks(landmarks);
}

TEST_F(LandmarkWorldNoNoise, LandmarkEstimatorStereoProjectiveEKF3D) {
  // ds create world and landmarks in that world (from first frame)
  PointIntensityDescriptor3fVectorCloud landmarks;
  std::unordered_map<size_t, size_t> indices_world_to_landmark;
  createWorldWithLandmarks(landmarks, indices_world_to_landmark);

  // ds landmark point estimator to test
  LandmarkEstimatorStereoProjectiveEKF3D estimator;
  StereoProjectivePointEKF3DPtr filter(new StereoProjectivePointEKF3D());
  filter->setCameraMatrix(projection_matrix.cast<double>());
  filter->setBaseline(baseline_to_second_sensor_pixelmeters.cast<double>());
  estimator.param_filter.setValue(filter);
  estimator.param_minimum_state_element_covariance.setValue(0.01);

  // ds update landmarks for the remaining poses (and observations)
  for (size_t index_pose = 1; index_pose < sensor_poses.size(); ++index_pose) {
    const PointIntensityDescriptor2fVectorCloud& points_in_image_left(
      points_in_sensor_projected_in_canvas[index_pose]);
    const PointIntensityDescriptor2fVectorCloud& points_in_image_right(
      points_in_sensor_projected_in_canvas_second[index_pose]);

    // ds update filter for current pose
    estimator.setTransforms(sensor_poses[index_pose], sensor_poses[index_pose]);

    // ds for all correspondences of the current pose
    for (const Correspondence& correspondence : correspondences_sensor_to_world[index_pose]) {
      const size_t index_measurement = correspondence.fixed_idx;
      auto iterator                  = indices_world_to_landmark.find(correspondence.moving_idx);
      if (iterator != indices_world_to_landmark.end()) {
        // ds adapt measurement (merge phase state)
        const PointIntensityDescriptor2f& point_in_image_left(
          points_in_image_left[index_measurement]);
        const PointIntensityDescriptor2f& point_in_image_right(
          points_in_image_right[index_measurement]);
        const Vector4f measurement_stereo(point_in_image_left.coordinates()(0),
                                          point_in_image_left.coordinates()(1),
                                          point_in_image_right.coordinates()(0),
                                          point_in_image_right.coordinates()(1));

        // ds update landmark position estimate using the estimator
        estimator.setMeasurement(measurement_stereo);
        estimator.setLandmark(&landmarks[iterator->second]);
        estimator.compute();
      }
    }
  }

  // ds validate final landmark estimates
  validateLandmarks(landmarks);
}

TEST_F(LandmarkWorldNoNoise, LandmarkEstimatorPoseBasedSmoother4D3D) {
  // ds create world and landmarks in that world (from first frame)
  PointIntensityDescriptor3fVectorCloud landmarks;
  std::unordered_map<size_t, size_t> indices_world_to_landmark;
  createWorldWithLandmarks(landmarks, indices_world_to_landmark);

  // ds landmark point estimator to test
  LandmarkEstimatorPoseBasedSmoother4D3D estimator;
  estimator.setCameraMatrix(projection_matrix);

  // ds update landmarks for the remaining poses (and observations)
  for (size_t index_pose = 1; index_pose < sensor_poses.size(); ++index_pose) {
    const PointIntensityDescriptor2fVectorCloud& points_in_image_left(
      points_in_sensor_projected[index_pose]);
    const PointIntensityDescriptor2fVectorCloud& points_in_image_right(
      points_in_sensor_projected_second[index_pose]);
    const PointIntensityDescriptor3fVectorCloud& points_in_sensor_current(
      points_in_sensor[index_pose]);

    // ds update filter for current pose
    estimator.setTransforms(sensor_poses[index_pose], sensor_poses[index_pose]);

    // ds for all correspondences of the current pose
    for (const Correspondence& correspondence : correspondences_sensor_to_world[index_pose]) {
      const size_t index_measurement = correspondence.fixed_idx;
      auto iterator                  = indices_world_to_landmark.find(correspondence.moving_idx);
      if (iterator != indices_world_to_landmark.end()) {
        // ds adapt measurement (merge phase state)
        const PointIntensityDescriptor2f& point_in_image_left(
          points_in_image_left[index_measurement]);
        const PointIntensityDescriptor2f& point_in_image_right(
          points_in_image_right[index_measurement]);
        const Vector4f measurement_stereo(point_in_image_left.coordinates()(0),
                                          point_in_image_left.coordinates()(1),
                                          point_in_image_right.coordinates()(0),
                                          point_in_image_right.coordinates()(1));
        const float depth_meters(points_in_sensor_current[index_measurement].coordinates()(2));
        ASSERT_GT(depth_meters, 0);
        const Vector3f point_in_camera(
          getPointUnprojected(point_in_image_left.coordinates(), depth_meters));

        // ds update landmark position estimate using the estimator
        estimator.setMeasurement(measurement_stereo);
        estimator.setLandmarkInSensor(point_in_camera);
        estimator.setLandmark(&landmarks[iterator->second]);
        estimator.compute();
      }
    }
  }

  // ds validate final landmark estimates
  validateLandmarks(landmarks);
}

void LandmarkWorldNoNoise::createWorldWithLandmarks(
  PointIntensityDescriptor3fVectorCloud& landmarks_,
  std::unordered_map<size_t, size_t>& indices_world_to_landmark_) {
  // ds test configuration
  constexpr size_t number_of_points = 1000;
  constexpr size_t number_of_poses  = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 200, 200;
  Matrix3f initial_position_covariance(Matrix3f::Identity());
  initial_position_covariance *= 0.01; // ds 10*10 cm^2

  // ds stereo setup
  offset_second_sensor = Vector3f(0.5 /*0.5m horizontal baseline*/, 0, 0);

  // ds generate uniform scenario
  const Isometry3f start_pose(Isometry3f::Identity());
  Isometry3f final_pose(Isometry3f::Identity());
  final_pose.translation() += Vector3f(10, 10, 10);
  const Vector3f pose_deviation(0.01, 0.01, 0.01);
  generateTrajectory(number_of_poses, start_pose, final_pose, pose_deviation);
  ASSERT_EQ(sensor_poses.size(), number_of_poses);
  const Vector3f mean_map(10, 10, 10);
  const Vector3f deviation_map(10, 10, 10);
  generateMap(number_of_points, mean_map, deviation_map, SensorType::Camera);
  ASSERT_EQ(points_in_world.size(), number_of_points);

  // ds seed landmarks from first set of observations
  landmarks_.clear();
  landmarks_.reserve(points_in_sensor[0].size());
  indices_world_to_landmark_.clear();
  indices_world_to_landmark_.reserve(points_in_sensor[0].size());
  for (size_t i = 0; i < points_in_sensor[0].size(); ++i) {
    const PointIntensityDescriptor3f& point_in_camera(points_in_sensor[0][i]);
    Vector3f coordinates_in_camera(point_in_camera.coordinates());
    PointIntensityDescriptor3f point_in_image(point_in_camera);
    point_in_image.coordinates().head(2) = points_in_sensor_projected[0][i].coordinates();

    // ds add minimal noise to the initial guess (fails validation if not marginalized)
    coordinates_in_camera += 1e-4 * Vector3f::Random();

    // ds unproject point to obtain world coordinates (i.e. landmark coordinates)
    PointIntensityDescriptor3f new_landmark;
    new_landmark.coordinates() = sensor_poses[0] * coordinates_in_camera;

    // ds initialize statistics field
    new_landmark.statistics().allocate();
    new_landmark.statistics().setState(new_landmark.coordinates());
    new_landmark.statistics().setCovariance(initial_position_covariance);
    new_landmark.statistics().addMeasurement(
      PointStatisticsField3D::CameraMeasurement(point_in_image.coordinates(),
                                                point_in_camera.coordinates(),
                                                sensor_poses[0],
                                                sensor_poses[0].inverse()));

    // ds store landmark in map
    landmarks_.emplace_back(new_landmark);

    // ds test bookkeeping only
    indices_world_to_landmark_.insert(
      std::make_pair(correspondences_sensor_to_world[0][i].moving_idx, i));
  }
}

void LandmarkWorldNoNoise::validateLandmarks(
  const PointIntensityDescriptor3fVectorCloud& landmarks_) const {
  // ds validate final landmark estimates based on true world position (all in the same map)
  ASSERT_EQ(landmarks_.size(), points_in_sensor[0].size());
  float mean_absolute_position_error_meters = 0;
  for (size_t index_landmark = 0; index_landmark < landmarks_.size(); ++index_landmark) {
    const size_t index_world_point = correspondences_sensor_to_world[0][index_landmark].moving_idx;
    //    ASSERT_NEAR_EIGEN(landmarks_[index_landmark].coordinates().cast<float>(),
    //                      points_in_world[index_world_point].coordinates(),
    //                      0.001 /*1 mm*/);
    ASSERT_NEAR_EIGEN(landmarks_[index_landmark].statistics().state().cast<float>(),
                      points_in_world[index_world_point].coordinates(),
                      0.001 /*1 mm*/);
    mean_absolute_position_error_meters +=
      (landmarks_[index_landmark].statistics().state().cast<float>() -
       points_in_world[index_world_point].coordinates())
        .norm();
  }
  std::cerr << "LandmarkWorldNoNoise::validateLandmarks|mean error (m): "
            << mean_absolute_position_error_meters / landmarks_.size() << std::endl;
}

const Vector3f LandmarkWorldNoNoise::getPointUnprojected(const Vector2f& point_in_image_,
                                                         const float& depth_meters_) const {
  // ds readability
  const float f_x = projection_matrix(0, 0);
  const float f_y = projection_matrix(1, 1);
  const float c_x = projection_matrix(0, 2);
  const float c_y = projection_matrix(1, 2);
  return Vector3f(depth_meters_ / f_x * (point_in_image_(0) - c_x),
                  depth_meters_ / f_y * (point_in_image_(1) - c_y),
                  depth_meters_);
}
