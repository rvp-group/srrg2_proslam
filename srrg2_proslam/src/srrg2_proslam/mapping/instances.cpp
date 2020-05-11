#include "instances.h"

#include "landmarks/filters/projective_depth_point_ekf_impl.cpp"
#include "landmarks/filters/projective_point_ekf_impl.cpp"
#include "landmarks/filters/stereo_projective_point_ekf_impl.cpp"

#include "landmarks/landmark_estimator_ekf_impl.cpp"
#include "landmarks/landmark_estimator_pose_based_smoother_impl.cpp"
#include "landmarks/landmark_estimator_weighted_mean_impl.cpp"

#include "mergers/merger_projective_depth_ekf_impl.cpp"
#include "mergers/merger_projective_impl.cpp"
#include "mergers/merger_projective_rigid_stereo_ekf_impl.cpp"
#include "mergers/merger_projective_rigid_stereo_impl.cpp"
#include "mergers/merger_projective_rigid_stereo_triangulation_impl.cpp"

#ifndef BOSS_REGISTER_CLASS_LINKER_FRIENDLY
#define BOSS_REGISTER_CLASS_LINKER_FRIENDLY(CLASS_) \
  CLASS_ dummy_##CLASS_;                            \
  BOSS_REGISTER_CLASS(CLASS_)
#endif

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  void srrg2_proslam_mapping_registerTypes() {
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(TriangulatorRigidStereoDescriptors);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(ProjectivePointEKF3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(ProjectiveDepthPointEKF3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(StereoProjectivePointEKF3D);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorWeightedMean2D3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorWeightedMean3D3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorWeightedMean4D3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorProjectiveEKF3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorProjectiveDepthEKF3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorStereoProjectiveEKF3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorPoseBasedSmoother2D3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorPoseBasedSmoother3D3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LandmarkEstimatorPoseBasedSmoother4D3D);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(SceneClipperProjective3D);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerCorrespondenceProjectiveDepth3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerProjectiveDepthEKF);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerRigidStereoTriangulation);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerRigidStereoProjectiveEKF);
  }

} // namespace srrg2_proslam
