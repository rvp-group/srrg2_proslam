#pragma once
#include "landmarks/filters/projective_depth_point_ekf.h"
#include "landmarks/filters/stereo_projective_point_ekf.h"
#include "landmarks/landmark_estimator_ekf.h"
#include "landmarks/landmark_estimator_pose_based_smoother.h"
#include "landmarks/landmark_estimator_weighted_mean.h"
#include "mergers/merger_correspondence_projective_depth_3d.h"
#include "mergers/merger_projective_depth_ekf.h"
#include "mergers/merger_projective_rigid_stereo_ekf.h"
#include "mergers/merger_projective_rigid_stereo_triangulation.h"

#include "scene_clipper_projective_3d.h"
#include "triangulator_rigid_stereo.h"
/*
#define SPECIALIZE_TEMPLATE(Class_, StateDimension_, ErrorDimension_, RealType_, RealTypeToken_) \
  using Class_##StateDimension_##RealTypeToken_##ErrorDimension_##RealTypeToken_ =               \
    Class_<StateDimension_, ErrorDimension_, RealType_>;
*/
namespace srrg2_proslam {
  //  SPECIALIZE_TEMPLATE(ProjectivePointEKF, 3, 2, float, f);
  //  SPECIALIZE_TEMPLATE(ProjectivePointEKF, 3, 3, float, f);
  //  SPECIALIZE_TEMPLATE(ProjectivePointEKF, 3, 4, float, f);
  //  SPECIALIZE_TEMPLATE(ProjectivePointEKF, 3, 2, double, d);
  //  SPECIALIZE_TEMPLATE(ProjectivePointEKF, 3, 2, double, d);
  //  SPECIALIZE_TEMPLATE(ProjectivePointEKF, 3, 4, double, d);
  //
  //  SPECIALIZE_TEMPLATE(ProjectiveDepthPointEKF, 3, 3, float, f);
  //  SPECIALIZE_TEMPLATE(ProjectiveDepthPointEKF, 3, 3, double, d);
  //
  //  SPECIALIZE_TEMPLATE(StereoProjectivePointEKF, 3, 4, float, f);
  //  SPECIALIZE_TEMPLATE(StereoProjectivePointEKF, 3, 4, double, d);

  void srrg2_proslam_mapping_registerTypes() __attribute__((constructor));

} // namespace srrg2_proslam
