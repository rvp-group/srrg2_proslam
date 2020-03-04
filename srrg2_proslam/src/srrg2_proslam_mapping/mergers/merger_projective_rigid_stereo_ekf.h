#pragma once
#include "merger_projective_rigid_stereo.h"
#include "srrg2_proslam_mapping/landmarks/landmark_estimator_ekf.h"

namespace srrg2_proslam {

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class MergerRigidStereoProjectiveEKF_
    : public MergerRigidStereo_<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType             = MergerRigidStereo_<TransformType_, FixedType_, MovingType_>;
    using ScenePointType       = typename FixedType_::value_type;
    using MeasurementPointType = typename MovingType_::value_type;

    virtual ~MergerRigidStereoProjectiveEKF_() {
    }

    //! this subclass fully reuses its parent compute routine

  protected:
    //! pre-compute routine, used to initialize certain modules of subclasses
    virtual void _precompute() override;
  };

  using MergerRigidStereoProjectiveEKF =
    MergerRigidStereoProjectiveEKF_<srrg2_core::Isometry3f,
                                    srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                    srrg2_core::PointIntensityDescriptor4fVectorCloud>;
  using MergerRigidStereoProjectiveEKFPtr = std::shared_ptr<MergerRigidStereoProjectiveEKF>;

} // namespace srrg2_proslam
