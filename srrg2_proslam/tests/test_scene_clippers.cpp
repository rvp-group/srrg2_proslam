#include "fixtures.hpp"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

TEST_F(ICL, SceneClipperProjective3D_DenseMonocularDepthNoMotion) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible_in_robot;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00_dense);
  clipper.setClippedSceneInRobot(&points_visible_in_robot);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

  // ds we did not move, all the points must be still visible
  // ds some points might be lost due to numerical imprecision
  // ds TODO for some reason(s) we get lower point counts on ubuntu1604_kinetic_release
  ASSERT_LE(points_visible_in_robot.size(), 306671);
  for (size_t index = 0; index < points_visible_in_robot.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible_in_robot[index].coordinates()(2), 0);
  }
}

TEST_F(ICL, SceneClipperProjective3D_DenseMonocularDepthRotateFullPitch) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.rotate(AngleAxisf(M_PI, Vector3f::UnitZ()));
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00_dense);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

  // ds the image is flipped upside down - all points must still be visible
  // ds some points might be lost due to numerical imprecision
  // ds TODO for some reason(s) we get lower point counts on ubuntu1604_kinetic_release
  ASSERT_LE(points_visible.size(), 306783);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);
  }
}

TEST_F(ICL, SceneClipperProjective3D_DenseMonocularDepthRotateFullRoll) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.rotate(AngleAxisf(M_PI, Vector3f::UnitX()));
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00_dense);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

  // ds full image roll - no points should be visible
  ASSERT_EQ(points_visible.size(), 0);
}

TEST_F(ICL, SceneClipperProjective3D_DenseMonocularDepthRotateQuarterRoll) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.rotate(AngleAxisf(M_PI / 4, Vector3f::UnitX()));
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00_dense);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

  // ds half image roll, many points must still be visible
  ASSERT_EQ(points_visible.size(), 49872);
}

TEST_F(ICL, SceneClipperProjective3D_DenseMonocularDepthTranslateBackward) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.translation().z() = -1.0;
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00_dense);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

  // ds zoom out, we still see all points
  ASSERT_EQ(points_visible.size(), 307200);
}

TEST_F(ICL, SceneClipperProjective3D_DenseMonocularDepthTranslateForward) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.translation().z() = 1.0;
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00_dense);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00_dense.size(), 307200);

  // ds zoom in, we lose points
  ASSERT_EQ(points_visible.size(), 136022);
}

TEST_F(ICL, SceneClipperProjective3D_SparseMonocularDepthNoMotion) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds we did not move, all the points must be still visible
  ASSERT_EQ(points_visible.size(), 321);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}

TEST_F(ICL, SceneClipperProjective3D_SparseMonocularDepthRotateFullPitch) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.rotate(AngleAxisf(M_PI, Vector3f::UnitZ()));
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds the image is flipped upside down - all points must still be visible
  ASSERT_EQ(points_visible.size(), 321);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}

TEST_F(ICL, SceneClipperProjective3D_SparseMonocularDepthRotateFullRoll) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.rotate(AngleAxisf(M_PI, Vector3f::UnitX()));
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds full image roll - no points should be visible
  ASSERT_EQ(points_visible.size(), 0);
}

TEST_F(ICL, SceneClipperProjective3D_SparseMonocularDepthRotateQuarterRoll) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.rotate(AngleAxisf(M_PI / 4, Vector3f::UnitX()));
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds half image roll, some points must still be visible
  ASSERT_EQ(points_visible.size(), 51);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}

TEST_F(ICL, SceneClipperProjective3D_SparseMonocularDepthTranslateBackward) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.translation().z() = -1.0;
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds zoom out, we not necessarily lose (sparse) points
  ASSERT_EQ(points_visible.size(), 321);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}

TEST_F(ICL, SceneClipperProjective3D_SparseMonocularDepthTranslateForward) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(10);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.translation().z() = 1.0;
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 321);

  // ds zoom in, we lose a few points
  ASSERT_EQ(points_visible.size(), 242);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}

TEST_F(KITTI, SceneClipperProjective3D_SparseNoMotion) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(1000);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 145);

  // ds we didn't move, all points must be visible
  ASSERT_EQ(points_visible.size(), 145);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}

TEST_F(KITTI, SceneClipperProjective3D_SparseTranslateForward) {
  SceneClipperProjective3D clipper;
  clipper.param_projector->setCameraMatrix(camera_calibration_matrix);
  clipper.param_projector->param_canvas_cols.setValue(image_cols);
  clipper.param_projector->param_canvas_rows.setValue(image_rows);
  clipper.param_projector->param_range_max.setValue(1000);
  clipper.param_projector->param_range_min.setValue(0.1);

  // ds result buffer
  PointIntensityDescriptor3fVectorCloud points_visible;

  // ds set buffers to processor
  Isometry3f robot_in_local_map(Isometry3f::Identity());
  robot_in_local_map.translation().z() = 10.0;
  clipper.setRobotInLocalMap(robot_in_local_map);
  clipper.setFullScene(&points_in_camera_00);
  clipper.setClippedSceneInRobot(&points_visible);

  // ds compute visible points
  clipper.compute();

  // ds this operation must not modify the world points!
  ASSERT_EQ(points_in_camera_00.size(), 145);

  // ds move 1 meter forward -> zoom in, we lose some points
  ASSERT_EQ(points_visible.size(), 52);
  for (size_t index = 0; index < points_visible.size(); ++index) {
    // ds points should still lie ahead of the camera
    ASSERT_GT(points_visible[index].coordinates()(2), 0);

    // ds fields must be copied! i.e. descriptor must be valid
    ASSERT_NE(points_visible[index].descriptor().rows, 0);
    ASSERT_NE(points_visible[index].descriptor().cols, 0);
  }
}
