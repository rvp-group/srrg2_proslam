#include "merger_projective_depth_ekf.h"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerProjectiveDepthEKF_<TransformType_, FixedType_, MovingType_>::_precompute() {
    using LandmarkEstimatorEKFType =
      LandmarkEstimatorEKF_<typename MeasurementPointType::VectorType, ScenePointType>;
    assert(BaseType::param_projector.value());
    assert(BaseType::param_projector->cameraMatrix().norm() > 0);
    assert(BaseType::param_landmark_estimator.value());
    if (!param_unprojector.value()) {
      throw std::runtime_error("MergerProjectiveDepthEKF_::_precompute|ERROR: unprojector not set");
    }
    assert(param_unprojector->cameraMatrix().norm() > 0);
    if ((param_unprojector->cameraMatrix() - BaseType::param_projector->cameraMatrix()).norm() >
        1e-5) {
      throw std::runtime_error(
        "MergerProjectiveDepthEKF_::_precompute|ERROR: mismatching camera matrices");
    }

    // ds TODO move this ugly bastard to initialization
    std::shared_ptr<LandmarkEstimatorEKFType> landmark_estimator_ekf =
      std::dynamic_pointer_cast<LandmarkEstimatorEKFType>(
        BaseType::param_landmark_estimator.value());
    assert(landmark_estimator_ekf);
    assert(landmark_estimator_ekf->param_filter.value());
    landmark_estimator_ekf->param_filter->setCameraMatrix(
      param_unprojector->cameraMatrix().template cast<double>());

    // ds transition estimate TODO enable proper covariance
    //    const TransformType_ measurement_in_scene = BaseType::_transform.inverse();
    //    Matrix3d transition_covariance(Matrix3d::Identity());
    //    transition_covariance *= measurement_in_scene.translation().norm();

    // ds update landmark estimator with current global pose (landmark exist over local maps)
    // ds and the transform to the local map (scene) coordinate frame
    BaseType::param_landmark_estimator->setTransforms(BaseType::_measurement_in_world,
                                                      BaseType::_measurement_in_scene);
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  bool MergerProjectiveDepthEKF_<TransformType_, FixedType_, MovingType_>::_isBetterForAddition(
    const MeasurementPointType& point_a_,
    const MeasurementPointType& point_b_) const {
    assert(point_a_.coordinates()(2) >= 0);
    assert(point_b_.coordinates()(2) >= 0);

    // ds if the candidate depth is lower than the occupying one we prefer it
    return (point_a_.coordinates()(2) < point_b_.coordinates()(2));
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void
  MergerProjectiveDepthEKF_<TransformType_, FixedType_, MovingType_>::_adaptFromMeasurementToScene(
    const MeasurementPointCloudType& points_in_measurement_space_,
    ScenePointCloudType& points_in_scene_space_) const {
    points_in_scene_space_.clear();
    if (points_in_measurement_space_.empty()) {
      return;
    }

    // ds unproject points in current left camera frame
    assert(param_unprojector.value());
    param_unprojector->compute(points_in_measurement_space_, points_in_scene_space_);
    if (points_in_scene_space_.empty()) {
      std::cerr << FG_RED("MergerProjectiveDepthEKF_::_adaptFromMeasurementToScene|WARNING: all "
                          "unprojections failed");
      std::cerr << " (target: " << points_in_measurement_space_.size() << ")" << std::endl;
    }
  }

} // namespace srrg2_proslam
