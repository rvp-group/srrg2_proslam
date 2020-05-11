#pragma once
#include "projective_point_ekf.h"

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_ = double>
  class ProjectiveDepthPointEKF
    : public ProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType              = ProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>;
    using StateMatrixType       = typename BaseType::StateMatrixType;
    using MeasurementVectorType = typename BaseType::MeasurementVectorType;
    virtual ~ProjectiveDepthPointEKF() {
    }

  protected:
    virtual void _computeMeasurementPrediction() override;
  };

  using ProjectiveDepthPointEKF3D    = ProjectiveDepthPointEKF<3, 3, double>;
  using ProjectiveDepthPointEKF3DPtr = std::shared_ptr<ProjectiveDepthPointEKF3D>;

} // namespace srrg2_proslam
