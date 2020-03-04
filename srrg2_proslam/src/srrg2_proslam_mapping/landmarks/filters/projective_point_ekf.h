#pragma once
#include "point_ekf_base.hpp"

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_ = double>
  class ProjectivePointEKF : public PointEKFBase<StateDimension_, ErrorDimension_, RealType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType              = PointEKFBase<StateDimension_, ErrorDimension_, RealType_>;
    using StateMatrixType       = typename BaseType::StateMatrixType;
    using MeasurementVectorType = typename BaseType::MeasurementVectorType;
    virtual ~ProjectivePointEKF() {
    }

    // ds parameter setter
    void setCameraMatrix(const StateMatrixType& camera_matrix_);

  protected:
    virtual void _computeMeasurementPrediction() override;

    // ds parameters
    RealType_ _f_x = 0;
    RealType_ _f_y = 0;
    RealType_ _c_x = 0;
    RealType_ _c_y = 0;
  };

  using ProjectivePointEKF3D    = ProjectivePointEKF<3, 2, double>;
  using ProjectivePointEKF3DPtr = std::shared_ptr<ProjectivePointEKF3D>;

} // namespace srrg2_proslam
