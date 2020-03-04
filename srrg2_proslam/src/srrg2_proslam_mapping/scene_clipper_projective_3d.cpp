#include "scene_clipper_projective_3d.h"
#include "srrg_pcl/point.h"
#include "srrg_slam_interfaces/correspondence_finder.h"

namespace srrg2_proslam {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  void SceneClipperProjective3D::compute() {
    PROFILE_TIME("SceneClipperProjective3D::compute");
    _status = Status::Error;
    if (!param_projector.value()) {
      throw std::runtime_error("SceneClipperProjective3D::compute|ERROR: missing projector");
    }
    if (!_local_scene) {
      throw std::runtime_error("SceneClipperProjective3D::compute|ERROR: missing local scene");
    }
    if (!_global_scene) {
      throw std::runtime_error("SceneClipperProjective3D::compute|ERROR: missing global scene");
    }
    if (_global_scene->empty()) {
      std::cerr
        << FG_YELLOW(
             "SceneClipperProjective3D::compute|WARNING: global scene is empty, no clipping "
             "will be performed")
        << std::endl;
      return;
    }
    _local_scene->clear();
    _global_indices.clear();

    // ds cache projector
    const ProjectoryPtrType projector = param_projector.value();
    assert(projector->param_canvas_rows.value() > 0);
    assert(projector->param_canvas_cols.value() > 0);
    assert(projector->param_range_min.value() > 0);
    assert(projector->param_range_max.value() > 0);
    assert(projector->cameraMatrix().norm() > 0);
    assert(projector->cameraMatrix() != Matrix3f::Identity());

    // ds set current perspective
    assert(_transform.matrix().norm() > 0);
    assert(_robot_in_sensor.matrix().norm() > 0);
    projector->setCameraPose(_robot_in_sensor * _transform);

    // ds point projections that can be reused for e.g. alignment or measurement adaptation
    PointIntensityDescriptor3fVectorCloud projections;

    // ds project points onto the current canvas
    // ds obtaining points in camera frame, image plane and their indices
    projector->compute(*_global_scene, *_local_scene, projections, _global_indices);

    PointIntensityDescriptor3f acc;
    acc.coordinates().setZero();
    for (const auto& p : *ThisType::_global_scene) {
      acc += p;
    }

    if (_local_scene->empty()) {
      std::cerr << FG_YELLOW("SceneClipperProjective3D::compute|WARNING: clipped empty scene")
                << std::endl;
    }

    // srrg move the local scene in robot's coords
    _local_scene->transformInPlace<Isometry>(ThisType::_sensor_in_robot);

    // ds done
    _status = Status::Ready;
  }

} // namespace srrg2_proslam
