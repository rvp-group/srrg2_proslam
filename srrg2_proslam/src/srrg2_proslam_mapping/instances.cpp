#include "instances.h"

#include "landmarks/landmark_estimator_ekf_impl.cpp"
#include "landmarks/landmark_estimator_pose_based_smoother_impl.cpp"
#include "landmarks/landmark_estimator_weighted_mean_impl.cpp"

#include "mergers/merger_projective_depth_ekf_impl.cpp"
#include "mergers/merger_projective_impl.cpp"
#include "mergers/merger_projective_rigid_stereo_ekf_impl.cpp"
#include "mergers/merger_projective_rigid_stereo_impl.cpp"
#include "mergers/merger_projective_rigid_stereo_triangulation_impl.cpp"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  void srrg2_proslam_mapping_registerTypes() {
    BOSS_REGISTER_CLASS(TriangulatorRigidStereoDescriptors);

    BOSS_REGISTER_CLASS(ProjectivePointEKF3D);
    BOSS_REGISTER_CLASS(ProjectiveDepthPointEKF3D);
    BOSS_REGISTER_CLASS(StereoProjectivePointEKF3D);

    BOSS_REGISTER_CLASS(LandmarkEstimatorWeightedMean2D3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorWeightedMean3D3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorWeightedMean4D3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorProjectiveEKF3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorProjectiveDepthEKF3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorStereoProjectiveEKF3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorPoseBasedSmoother2D3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorPoseBasedSmoother3D3D);
    BOSS_REGISTER_CLASS(LandmarkEstimatorPoseBasedSmoother4D3D);

    BOSS_REGISTER_CLASS(SceneClipperProjective3D);

    BOSS_REGISTER_CLASS(MergerCorrespondence3D);
    BOSS_REGISTER_CLASS(MergerCorrespondenceProjectiveDepth3D);
    BOSS_REGISTER_CLASS(MergerProjectiveDepthEKF);
    BOSS_REGISTER_CLASS(MergerRigidStereoTriangulation);
    BOSS_REGISTER_CLASS(MergerRigidStereoProjectiveEKF);
  }

} // namespace srrg2_proslam
