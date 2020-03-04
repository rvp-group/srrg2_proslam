#include "merger_projective_rigid_stereo.h"
#include "srrg2_proslam_mapping/landmarks/landmark_estimator_pose_based_smoother.h"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerRigidStereo_<TransformType_, FixedType_, MovingType_>::_precompute() {
    if (!param_triangulator.value()) {
      throw std::runtime_error("MergerRigidStereo_::_precompute|ERROR: triangulator not set");
    };
    if (!param_triangulator->param_projector.value()) {
      throw std::runtime_error(
        "MergerRigidStereo_::_precompute|ERROR: triangulator projector not set");
    }
    if (param_triangulator->param_projector.value() != BaseType::param_projector.value()) {
      throw std::runtime_error("MergerRigidStereo_::_precompute|ERROR: duplicate projectors set");
    }
    param_triangulator->initializeBaseline();
    assert(BaseType::param_landmark_estimator.value());

    // ds if we have a smoother estimator - maximum vomit
    using LandmarkEstimatorType =
      LandmarkEstimatorPoseBasedSmoother_<typename MeasurementPointType::VectorType,
                                          ScenePointType>;
    std::shared_ptr<LandmarkEstimatorType> landmark_estimator_smoother =
      BaseType::param_landmark_estimator.template getSharedPtr<LandmarkEstimatorType>();
    if (landmark_estimator_smoother) {
      landmark_estimator_smoother->setCameraMatrix(
        param_triangulator->param_projector->cameraMatrix());
    }

    // ds transition estimate TODO enable proper covariance
    //    const TransformType_ measurement_in_scene = BaseType::_transform.inverse();
    //    Matrix3d transition_covariance(Matrix3d::Identity());
    //    transition_covariance *= measurement_in_scene.translation().norm() / 100.0;

    // ds update landmark estimator with current global pose (landmark exist over local maps)
    // ds and the transform to the local map (scene) coordinate frame
    BaseType::param_landmark_estimator->setTransforms(BaseType::_tracker_in_world,
                                                      BaseType::_transform);
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  bool MergerRigidStereo_<TransformType_, FixedType_, MovingType_>::_isBetterForAddition(
    const MeasurementPointType& point_a_,
    const MeasurementPointType& point_b_) const {
    const float disparity_a = point_a_.coordinates()(0) - point_a_.coordinates()(2);
    assert(disparity_a >= 0);
    const float disparity_b = point_b_.coordinates()(0) - point_b_.coordinates()(2);
    assert(disparity_b >= 0);

    // ds if the candidate disparity is higher than the occupying one
    // ds TODO add matching distance as further requirement
    return (disparity_a > disparity_b);
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerRigidStereo_<TransformType_, FixedType_, MovingType_>::_adaptFromMeasurementToScene(
    const MeasurementPointCloudType& points_in_measurement_space_,
    ScenePointCloudType& points_in_scene_space_) const {
    points_in_scene_space_.clear();
    if (points_in_measurement_space_.empty()) {
      return;
    }

    // ds triangulate points in current left camera frame
    assert(param_triangulator.value());
    param_triangulator->setDest(&points_in_scene_space_);
    param_triangulator->setMoving(&points_in_measurement_space_);
    param_triangulator->compute();
    if (points_in_scene_space_.empty()) {
      std::cerr << "MergerRigidStereo_::_adaptFromMeasurementToScene|WARNING: all "
                   "triangulations failed"
                << std::endl;
    }
  }

} // namespace srrg2_proslam
