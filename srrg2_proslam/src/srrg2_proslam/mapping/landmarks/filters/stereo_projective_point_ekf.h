#pragma once
#include "projective_point_ekf.h"

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_ = double>
  class StereoProjectivePointEKF
    : public ProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType              = ProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>;
    using StateVectorType       = typename BaseType::StateVectorType;
    using MeasurementVectorType = typename BaseType::MeasurementVectorType;
    virtual ~StereoProjectivePointEKF() {
    }

    // ds parameter setter
    void setBaseline(const StateVectorType& baseline_pixels_);

  protected:
    virtual void _computeMeasurementPrediction() override;

    // ds parameters
    RealType_ _b_x = 0;
    RealType_ _b_y = 0;
  };

  using StereoProjectivePointEKF3D    = StereoProjectivePointEKF<3, 4, double>;
  using StereoProjectivePointEKF3DPtr = std::shared_ptr<StereoProjectivePointEKF3D>;

} // namespace srrg2_proslam
