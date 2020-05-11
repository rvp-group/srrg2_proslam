#pragma once
#include "merger_projective.h"
#include "srrg2_proslam/mapping/triangulator_rigid_stereo.h"

namespace srrg2_proslam {

  // ds rigid stereo projective merger interface
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class MergerRigidStereo_ : public MergerProjective_<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType                   = MergerProjective_<TransformType_, FixedType_, MovingType_>;
    using ScenePointCloudType        = FixedType_;
    using ScenePointType             = typename ScenePointCloudType::value_type;
    using MeasurementPointCloudType  = MovingType_;
    using MeasurementPointType       = typename MeasurementPointCloudType::value_type;
    static constexpr size_t DepthDim = ScenePointType::Dim - 1;

    PARAM(srrg2_core::PropertyConfigurable_<TriangulatorRigidStereoDescriptors>,
          triangulator,
          "rigid stereo triangulation unit",
          TriangulatorRigidStereoDescriptorsPtr(new TriangulatorRigidStereoDescriptors()),
          nullptr);

    virtual ~MergerRigidStereo_() {
    }

    //! this subclass fully reuses its parent compute routine

  protected:
    //! initialize rigid stereo merging
    virtual void _precompute() override;

    //! checks if point_a_ is more suitable than point_b_ for addition for rigid stereo
    virtual bool _isBetterForAddition(const MeasurementPointType& point_a_,
                                      const MeasurementPointType& point_b_) const override;

    //! adapts points from measurement cloud to scene cloud via triangulation
    virtual void
    _adaptFromMeasurementToScene(const MeasurementPointCloudType& points_in_measurement_space_,
                                 ScenePointCloudType& points_in_scene_space_) const override;
  };

} // namespace srrg2_proslam
