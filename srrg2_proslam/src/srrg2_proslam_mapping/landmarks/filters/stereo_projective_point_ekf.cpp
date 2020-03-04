#include "stereo_projective_point_ekf.h"

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_>
  void StereoProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>::setBaseline(
    const StateVectorType& baseline_pixels_) {
    _b_x = baseline_pixels_.x();
    _b_y = baseline_pixels_.y();
  }

  template <size_t StateDimension_, size_t ErrorDimension_, typename RealType_>
  void StereoProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>::
    _computeMeasurementPrediction() {
    assert(BaseType::_state);
    assert(_b_x != 0 || _b_y != 0);
    assert(BaseType::_f_x > 0);
    assert(BaseType::_f_y > 0);
    assert(BaseType::_c_x > 0);
    assert(BaseType::_c_y > 0);

    // ds cache math
    const RealType_& z = BaseType::_state->z();
    assert(z > 0);
    const RealType_ z_2      = z * z;
    const RealType_ fx_x     = BaseType::_f_x * BaseType::_state->x();
    const RealType_ fy_y     = BaseType::_f_y * BaseType::_state->y();
    const RealType_ _fx_by_z = BaseType::_f_x / z;
    const RealType_ _fy_by_z = BaseType::_f_y / z;

    // ds predict measurement based on predicted state
    const RealType_ x_h = fx_x + BaseType::_c_x * z;
    const RealType_ y_h = fy_y + BaseType::_c_y * z;

    // ds compute image coordinates
    BaseType::_predicted_measurement(0 /*u_L*/) = x_h / z;
    BaseType::_predicted_measurement(1 /*v_L*/) = y_h / z;
    BaseType::_predicted_measurement(2 /*u_R*/) = (x_h + _b_x) / z;
    BaseType::_predicted_measurement(3 /*v_R*/) = (y_h + _b_y) / z;

    // ds compute measurement gradient change
    BaseType::_measurement_jacobian << _fx_by_z, 0.0, -fx_x / z_2, 0.0, _fy_by_z, -fy_y / z_2,
      _fx_by_z, 0.0, -(fx_x + _b_x) / z_2, 0.0, _fy_by_z, -(fy_y + _b_y) / z_2;
  }

#define SPECIALIZE_TEMPLATE(StateDimension_, ErrorDimension_, RealType_)                \
  template void                                                                         \
  StereoProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>::setBaseline(   \
    const StateVectorType& baseline_pixels_);                                           \
  template void StereoProjectivePointEKF<StateDimension_, ErrorDimension_, RealType_>:: \
    _computeMeasurementPrediction()

  SPECIALIZE_TEMPLATE(3, 4, float);
  SPECIALIZE_TEMPLATE(3, 4, double);

} // namespace srrg2_proslam
