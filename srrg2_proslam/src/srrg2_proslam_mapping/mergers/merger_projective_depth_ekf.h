#pragma once
#include "merger_projective.h"
#include "srrg2_proslam_mapping/landmarks/landmark_estimator_ekf.h"

namespace srrg2_proslam {

  // ds projective depth merger interface
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class MergerProjectiveDepthEKF_
    : public MergerProjective_<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType                  = MergerProjective_<TransformType_, FixedType_, MovingType_>;
    using ScenePointCloudType       = FixedType_;
    using ScenePointType            = typename ScenePointCloudType::value_type;
    using MeasurementPointCloudType = MovingType_;
    using MeasurementPointType      = typename MeasurementPointCloudType::value_type;
    using UnprojectoryType    = srrg2_core::PointUnprojectorPinhole_<MeasurementPointCloudType>;
    using UnprojectoryPtrType = std::shared_ptr<UnprojectoryType>;

    PARAM(srrg2_core::PropertyConfigurable_<UnprojectoryType>,
          unprojector,
          "point unprojector to obtain points in camera frame from adapted UV-D measurements",
          UnprojectoryPtrType(new UnprojectoryType()),
          nullptr);

    virtual ~MergerProjectiveDepthEKF_() {
    }

    //! this subclass fully reuses its parent compute routine

  protected:
    //! pre-compute routine, used to initialize certain modules of subclasses
    virtual void _precompute() override;

    //! checks if point_a_ is more suitable than point_b_ for addition for UVD
    virtual bool _isBetterForAddition(const MeasurementPointType& point_a_,
                                      const MeasurementPointType& point_b_) const override;

    //! adapts points from measurement cloud to scene cloud via unprojection
    virtual void
    _adaptFromMeasurementToScene(const MeasurementPointCloudType& points_in_measurement_space_,
                                 ScenePointCloudType& points_in_scene_space_) const override;
  };

  using MergerProjectiveDepthEKF =
    MergerProjectiveDepthEKF_<srrg2_core::Isometry3f,
                              srrg2_core::PointIntensityDescriptor3fVectorCloud,
                              srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using MergerProjectiveDepthEKFPtr = std::shared_ptr<MergerProjectiveDepthEKF>;

} // namespace srrg2_proslam
