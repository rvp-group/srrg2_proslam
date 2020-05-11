#include "projective_point_ekf.h"

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_>
  void ProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>::setCameraMatrix(
    const StateMatrixType& camera_matrix_) {
    _f_x = camera_matrix_(0, 0);
    _f_y = camera_matrix_(1, 1);
    _c_x = camera_matrix_(0, 2);
    _c_y = camera_matrix_(1, 2);
  }

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_>
  void
  ProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>::_computeMeasurementPrediction() {
    assert(BaseType::_state);
    //    assert(BaseType::_f_x > 0); TODO enforce regular K
    //    assert(BaseType::_f_y > 0);
    //    assert(BaseType::_c_x > 0);
    //    assert(BaseType::_c_y > 0);

    // ds cache math
    const RealType_& x = BaseType::_state->x();
    const RealType_& y = BaseType::_state->y();
    const RealType_& z = BaseType::_state->z();
    assert(z > 0);
    const RealType_ z_2      = z * z;
    const RealType_ fx_x     = _f_x * x;
    const RealType_ fy_y     = _f_y * y;
    const RealType_ _fx_by_z = _f_x / z;
    const RealType_ _fy_by_z = _f_y / z;

    // ds predict measurement based on predicted state
    BaseType::_predicted_measurement(0) = _fx_by_z * x + _c_x;
    BaseType::_predicted_measurement(1) = _fy_by_z * y + _c_y;

    // clang-format off
    // ds compute measurement gradient change
    BaseType::_measurement_jacobian <<
        _fx_by_z,      0.0, -fx_x / z_2,
             0.0, _fy_by_z, -fy_y / z_2;
    // clang-format on
  }
} // namespace srrg2_proslam
