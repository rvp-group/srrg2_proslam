#pragma once
#include <iomanip>

#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/platform.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_proslam {

  template <size_t StateDimension_, size_t ErrorDimension, typename RealType_ = double>
  class PointEKFBase : public srrg2_core::Configurable,
                       public srrg2_core::PlatformUser,
                       public srrg2_core::Profiler {
  protected:
    // ds the only method subclasses must make available!
    virtual void _computeMeasurementPrediction() = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using StateVectorType       = srrg2_core::Vector_<RealType_, StateDimension_>;
    using StateMatrixType       = srrg2_core::MatrixN_<RealType_, StateDimension_>;
    using MeasurementVectorType = srrg2_core::Vector_<RealType_, ErrorDimension>;
    using MeasurementMatrixType = srrg2_core::MatrixN_<RealType_, ErrorDimension>;
    using ErrorVectorType       = srrg2_core::Vector_<RealType_, ErrorDimension>;
    using TransformType         = srrg2_core::Isometry_<RealType_, StateDimension_>;
    virtual ~PointEKFBase(){};

    // ds public input interface - shouldn't be required to change
    void setState(StateVectorType* state_) {
      _state = state_;
    }
    void setCovariance(StateMatrixType* covariance_) {
      _covariance = covariance_;
    }
    void setWorldInSensor(const TransformType& world_in_sensor_,
                          const StateMatrixType& covariance_ = StateMatrixType::Zero()) {
      _world_in_sensor            = world_in_sensor_;
      _world_in_sensor_covariance = covariance_;
    }
    void
    setMeasurement(const MeasurementVectorType& measurement_,
                   const MeasurementMatrixType& covariance_ = MeasurementMatrixType::Identity()) {
      _measurement            = measurement_;
      _measurement_covariance = covariance_;
    }
    void compute() {
      PROFILE_TIME("PointEKFBase::compute");
      _predict();
      _correct();
    }

    // ds public output interface - shouldn't be required to change
    const StateVectorType* state() const {
      return _state;
    }
    const StateMatrixType* covariance() const {
      return _covariance;
    }

  protected:
    // ds subclasses should not need to override this step
    void _predict() {
      PROFILE_TIME("PointEKFBase::predict");
      assert(_state);
      assert(_covariance);

      // ds compute state covariance for transition (based on current state)
      const StateMatrixType transition_jacobian(_world_in_sensor.linear());
      *_covariance = transition_jacobian * *_covariance * transition_jacobian.transpose() +
                     _world_in_sensor_covariance;
      assert(!_covariance->hasNaN());

      // ds transit into predicted state according to provided transition
      *_state = _world_in_sensor * *_state;
      assert(!_state->hasNaN());
    }

    // ds subclasses should not need to override this step
    void _correct() {
      PROFILE_TIME("PointEKFBase::correct");
      assert(_state);
      assert(!_state->hasNaN());
      assert(_covariance);
      assert(!_covariance->hasNaN());
      assert(_measurement_covariance.norm() > 0);

      // ds virtual dispatch
      _computeMeasurementPrediction();
      assert(!_predicted_measurement.hasNaN());
      assert(!_measurement_jacobian.hasNaN());

      // ds cache measurement transpose
      const Eigen::Matrix<RealType_, StateDimension_, ErrorDimension>
        measurement_jacobian_transposed = _measurement_jacobian.transpose();

      // ds include measurement uncertainty to compute the kalman gain
      const Eigen::Matrix<RealType_, StateDimension_, ErrorDimension> kalman_gain =
        *_covariance * measurement_jacobian_transposed *
        (_measurement_covariance +
         _measurement_jacobian * *_covariance * measurement_jacobian_transposed)
          .inverse();
      assert(!kalman_gain.hasNaN());

      // ds correct state and state covariance
      *_state += kalman_gain * (_measurement - _predicted_measurement);
      const RealType_ covariance_norm_before_correction = _covariance->norm();
      assert(!_state->hasNaN());

      *_covariance =
        (StateMatrixType::Identity() - kalman_gain * _measurement_jacobian) * *_covariance;
      const RealType_ covariance_norm_after_correction = _covariance->norm();
      assert(!_covariance->hasNaN());

      // ds check if covariance norm has increased after correction (faulty correction)
      // ds to reduce spam for floating point imprecision we require an increase of an epsilon
      if (covariance_norm_after_correction > 1.00001 * covariance_norm_before_correction) {
        const std::streamsize precision = std::cerr.precision(); // ds bookkeep default precision
        std::cerr << std::setprecision(5)                        /*raise precision*/
                  << "EKFBase::_correct|WARNING: covariance increased from "
                  << covariance_norm_before_correction << " [before] "
                  << " to " << covariance_norm_after_correction << " [after] (filter might diverge)"
                  << std::setprecision(precision) /*reset precision to default*/ << std::endl;
      }
    }

    // ds input: transitions and measurements
    TransformType _world_in_sensor              = TransformType::Identity();
    StateMatrixType _world_in_sensor_covariance = StateMatrixType::Zero();
    MeasurementVectorType _measurement;
    MeasurementMatrixType _measurement_covariance;

    // ds ouput: estimates
    StateVectorType* _state      = nullptr;
    StateMatrixType* _covariance = nullptr;

    // ds working buffers (must be computed by subclass in _computeMeasurementPrediction!)
    MeasurementVectorType _predicted_measurement;
    Eigen::Matrix<RealType_, ErrorDimension, StateDimension_> _measurement_jacobian;
  };

} // namespace srrg2_proslam
