#include "merger_correspondence_projective_depth_3d.h"

namespace srrg2_proslam {
  using namespace srrg2_core;

  void MergerCorrespondenceProjectiveDepth3D::compute() {
    PROFILE_TIME("MergerCorrespondenceProjectiveDepth3D::compute");
    assert(param_unprojector.value());
    assert(_scene);
    assert(_moving);
    _status = Initializing;

    // ds check if we can skip computation
    if (!_scene_changed_flag && !_moving_changed_flag && !_correspondences_changed_flag &&
        !_transform_changed_flag) {
      _status = Success;
      return;
    }

    // ds check valid unprojector configuration
    assert(param_unprojector->cameraMatrix().norm() > 0);
    assert(param_unprojector->cameraMatrix() != Matrix3f::Identity()); // ds TODO warning

    // ds adapt moving UV-D for merging in 3D: unproject points (sparse)
    // ds TODO this could be done in place if moving were not const
    PointIntensityDescriptor3fVectorCloud moving_in_camera_frame;
    param_unprojector->compute(*_moving, moving_in_camera_frame);

    // ds update moving and delegate to base - correspondences and scene remain intact
    BaseType::setMoving(&moving_in_camera_frame);
    BaseType::compute();
  }

} // namespace srrg2_proslam
