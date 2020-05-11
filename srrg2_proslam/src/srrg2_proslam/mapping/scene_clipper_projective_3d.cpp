#include "scene_clipper_projective_3d.h"
#include <srrg2_slam_interfaces/registration/correspondence_finder.h>
#include <srrg_pcl/point.h>

namespace srrg2_proslam {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  void SceneClipperProjective3D::compute() {
    PROFILE_TIME("SceneClipperProjective3D::compute");
    _status = Status::Error;
    if (!param_projector.value()) {
      throw std::runtime_error("SceneClipperProjective3D::compute|ERROR: missing projector");
    }
    if (!_clipped_scene_in_robot) {
      throw std::runtime_error("SceneClipperProjective3D::compute|ERROR: missing clipped scene");
    }
    if (!_full_scene) {
      throw std::runtime_error("SceneClipperProjective3D::compute|ERROR: missing global scene");
    }
    if (_full_scene->empty()) {
      std::cerr
        << FG_YELLOW(
             "SceneClipperProjective3D::compute|WARNING: global scene is empty, no clipping "
             "will be performed")
        << std::endl;
      _status = Status::Ready;
      return;
    }
    _clipped_scene_in_robot->clear();
    _global_indices.clear();

    // ds cache projector
    const ProjectoryTypePtr projector = param_projector.value();
    assert(projector->param_canvas_rows.value() > 0);
    assert(projector->param_canvas_cols.value() > 0);
    assert(projector->param_range_min.value() > 0);
    assert(projector->param_range_max.value() > 0);
    assert(projector->cameraMatrix().norm() > 0);
    assert(projector->cameraMatrix() != Matrix3f::Identity());

    // ds set current perspective
    assert(_robot_in_local_map.matrix().norm() > 0);
    assert(_robot_in_sensor.matrix().norm() > 0);

    projector->setCameraPose(_robot_in_local_map * _sensor_in_robot);

    // ds point projections that can be reused for e.g. alignment or measurement adaptation
    PointIntensityDescriptor3fVectorCloud projections;

    // ds project points onto the current canvas
    // ds obtaining points in camera frame, image plane and their indices
    projector->compute(*_full_scene, *_clipped_scene_in_robot, projections, _global_indices);

    if (_clipped_scene_in_robot->empty()) {
      std::cerr << FG_YELLOW("SceneClipperProjective3D::compute|WARNING: clipped empty scene")
                << std::endl;
    }

    // srrg move the local scene in robot's coords
    if (ThisType::_sensor_in_robot.matrix() != EstimateType::Identity().matrix()) {
      _clipped_scene_in_robot->transformInPlace<Isometry>(ThisType::_sensor_in_robot);
    }

    // ds done
    _status = Status::Successful;
  }

} // namespace srrg2_proslam
