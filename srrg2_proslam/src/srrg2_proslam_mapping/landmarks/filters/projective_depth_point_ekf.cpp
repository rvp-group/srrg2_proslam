#include "projective_depth_point_ekf.h"

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_>
  void ProjectiveDepthPointEKF<StateDimension_, ErrorDimension_, RealType_>::
    _computeMeasurementPrediction() {
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
    const RealType_ fx_x     = BaseType::_f_x * x;
    const RealType_ fy_y     = BaseType::_f_y * y;
    const RealType_ _fx_by_z = BaseType::_f_x / z;
    const RealType_ _fy_by_z = BaseType::_f_y / z;

    // ds predict measurement based on predicted state
    BaseType::_predicted_measurement(0) = _fx_by_z * x + BaseType::_c_x;
    BaseType::_predicted_measurement(1) = _fy_by_z * y + BaseType::_c_y;
    BaseType::_predicted_measurement(2) = z;

    // ds compute measurement gradient change
    BaseType::_measurement_jacobian << _fx_by_z, 0.0, -fx_x / z_2, 0.0, _fy_by_z, -fy_y / z_2, 0, 0,
      1.0;
  }

#define SPECIALIZE_TEMPLATE(StateDimension_, ErrorDimension_, RealType_)               \
  template void ProjectiveDepthPointEKF<StateDimension_, ErrorDimension_, RealType_>:: \
    _computeMeasurementPrediction()

  SPECIALIZE_TEMPLATE(3, 3, float);
  SPECIALIZE_TEMPLATE(3, 3, double);

} // namespace srrg2_proslam
