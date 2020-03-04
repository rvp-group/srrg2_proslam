#include "merger_projective_rigid_stereo_ekf.h"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerRigidStereoProjectiveEKF_<TransformType_, FixedType_, MovingType_>::_precompute() {
    using LandmarkEstimatorEKFType =
      LandmarkEstimatorEKF_<typename MeasurementPointType::VectorType, ScenePointType>;
    using StereoProjectiveEKFType = StereoProjectivePointEKF<3, 4, double>;
    MergerRigidStereo_<TransformType_, FixedType_, MovingType_>::_precompute();
    assert(BaseType::param_triangulator.value());
    assert(BaseType::param_triangulator->param_projector.value());
    assert(BaseType::param_landmark_estimator.value());

    // ds TODO move this monstrosity to initialization
    std::shared_ptr<LandmarkEstimatorEKFType> landmark_estimator_ekf =
      BaseType::param_landmark_estimator.template getSharedPtr<LandmarkEstimatorEKFType>();
    assert(landmark_estimator_ekf);
    assert(BaseType::param_triangulator->baseline().norm() > 0);
    std::shared_ptr<StereoProjectiveEKFType> filter =
      landmark_estimator_ekf->param_filter.template getSharedPtr<StereoProjectiveEKFType>();
    assert(filter);
    filter->setCameraMatrix(
      BaseType::param_triangulator->param_projector->cameraMatrix().template cast<double>());
    filter->setBaseline(BaseType::param_triangulator->baseline().template cast<double>());
  }

} // namespace srrg2_proslam
