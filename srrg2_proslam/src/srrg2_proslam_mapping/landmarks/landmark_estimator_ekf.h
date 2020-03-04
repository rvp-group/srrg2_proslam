#pragma once
#include "filters/projective_depth_point_ekf.h"
#include "filters/projective_point_ekf.h"
#include "filters/stereo_projective_point_ekf.h"
#include "landmark_estimator_base.hpp"

namespace srrg2_slam_interfaces {

  template <typename MeasurementVectorType_, typename LandmarkType_>
  class LandmarkEstimatorEKF_
    : public LandmarkEstimatorBase_<MeasurementVectorType_, LandmarkType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType     = LandmarkEstimatorBase_<MeasurementVectorType_, LandmarkType_>;
    using EstimateType = typename BaseType::EstimateType;
    static constexpr size_t StateDim       = LandmarkType_::Dim;
    static constexpr size_t MeasurementDim = MeasurementVectorType_::RowsAtCompileTime;

    // ds we locally operate in double precision to guarantee precise matrix inversions
    using MeasurementMatrixType = srrg2_core::MatrixN_<double, MeasurementDim>;
    using StateVectorType       = srrg2_core::Vector_<double, StateDim>;
    using StateMatrixType       = srrg2_core::MatrixN_<double, StateDim>;
    using FilterType = srrg2_proslam::ProjectivePointEKF<StateDim, MeasurementDim, double>;

    PARAM(srrg2_core::PropertyConfigurable_<FilterType>,
          filter,
          "filter instance used to refine the landmark position estimate",
          nullptr,
          nullptr);

    PARAM(srrg2_core::PropertyDouble,
          minimum_state_element_covariance,
          "minimum per element covariance value (prohibits ill-shaped uncertainties)",
          0.01,
          nullptr);

    PARAM(srrg2_core::PropertyDouble,
          maximum_covariance_norm_squared,
          "maximum permitted covariance matrix Frobenius norm value for merging",
          1,
          nullptr);

    void setTransforms(const EstimateType& tracker_from_world_,
                       const EstimateType& scene_from_tracker_) override;
    void compute() override;
  };

  using LandmarkEstimatorProjectiveEKF3D =
    LandmarkEstimatorEKF_<srrg2_core::Vector2f, srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorProjectiveEKF3DPtr = std::shared_ptr<LandmarkEstimatorProjectiveEKF3D>;

  using LandmarkEstimatorProjectiveDepthEKF3D =
    LandmarkEstimatorEKF_<srrg2_core::Vector3f, srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorProjectiveDepthEKF3DPtr =
    std::shared_ptr<LandmarkEstimatorProjectiveDepthEKF3D>;

  using LandmarkEstimatorStereoProjectiveEKF3D =
    LandmarkEstimatorEKF_<srrg2_core::Vector4f, srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorStereoProjectiveEKF3DPtr =
    std::shared_ptr<LandmarkEstimatorStereoProjectiveEKF3D>;

} // namespace srrg2_slam_interfaces
