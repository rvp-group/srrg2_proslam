#include "merger_projective_rigid_stereo_triangulation.h"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  bool MergerRigidStereoTriangulation_<TransformType_, FixedType_, MovingType_>::_updatePoint(
    const MeasurementPointType& measured_point_,
    ScenePointType& scene_point_) {
    assert(BaseType::param_landmark_estimator.value());
    assert(BaseType::param_triangulator.value());

    // ds TODO encapsulate in virtual method
    {
      // ds check disparity
      const float& row_left(measured_point_.coordinates()(0));
      const float& row_right(measured_point_.coordinates()(2));
      const float disparity_in_cols = row_left - row_right;
      assert(disparity_in_cols >= 0);
      if (disparity_in_cols <
          BaseType::param_triangulator->param_minimum_disparity_pixels.value()) {
        // ds point cannot be triangulated, merge skipped
        return false;
      }

      // ds compute point in camera frame (triangulation)
      Vector3f point_in_camera;
      BaseType::param_triangulator->triangulateRectifiedMidpoint(row_left,
                                                                 measured_point_.coordinates()(1),
                                                                 row_right,
                                                                 measured_point_.coordinates()(3),
                                                                 point_in_camera);
      assert(point_in_camera(2) > 0);

      // ds feed auxiliary data to estimator
      BaseType::param_landmark_estimator->setLandmarkInSensor(point_in_camera);
    }
    return BaseType::_updatePoint(measured_point_, scene_point_);
  }

} // namespace srrg2_proslam
