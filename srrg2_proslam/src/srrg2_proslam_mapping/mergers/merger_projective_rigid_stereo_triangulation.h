#pragma once
#include "merger_projective_rigid_stereo.h"

namespace srrg2_proslam {

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class MergerRigidStereoTriangulation_
    : public MergerRigidStereo_<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType             = MergerRigidStereo_<TransformType_, FixedType_, MovingType_>;
    using ScenePointType       = typename FixedType_::value_type;
    using MeasurementPointType = typename MovingType_::value_type;

    virtual ~MergerRigidStereoTriangulation_() {
    }

    //! this subclass fully reuses its parent compute routine

  protected:
    //! this subclass fully reuses its parent's _precompute routine

    //! point update method
    //! this method integrates an adapted measurement into an existing scene point
    //! returns true upon successful merge
    virtual bool _updatePoint(const MeasurementPointType& measured_point_,
                              ScenePointType& scene_point_) override;
  };

  using MergerRigidStereoTriangulation =
    MergerRigidStereoTriangulation_<srrg2_core::Isometry3f,
                                    srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                    srrg2_core::PointIntensityDescriptor4fVectorCloud>;
  using MergerRigidStereoTriangulationPtr = std::shared_ptr<MergerRigidStereoTriangulation>;

} // namespace srrg2_proslam
