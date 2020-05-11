#pragma once
#include "landmark_estimator_base.hpp"

namespace srrg2_proslam {

  template <typename MeasurementVectorType_, typename LandmarkType_>
  class LandmarkEstimatorWeightedMean_
    : public LandmarkEstimatorBase_<MeasurementVectorType_, LandmarkType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType                = LandmarkEstimatorBase_<MeasurementVectorType_, LandmarkType_>;
    using LandmarkCoordinatesType = srrg2_core::Vector_<float, LandmarkType_::Dim>;

    void compute() override;
  };

  using LandmarkEstimatorWeightedMean2D3D =
    LandmarkEstimatorWeightedMean_<srrg2_core::Vector2f, srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorWeightedMean2D3DPtr = std::shared_ptr<LandmarkEstimatorWeightedMean2D3D>;

  using LandmarkEstimatorWeightedMean3D3D =
    LandmarkEstimatorWeightedMean_<srrg2_core::Vector3f, srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorWeightedMean3D3DPtr = std::shared_ptr<LandmarkEstimatorWeightedMean3D3D>;

  using LandmarkEstimatorWeightedMean4D3D =
    LandmarkEstimatorWeightedMean_<srrg2_core::Vector4f, srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorWeightedMean4D3DPtr = std::shared_ptr<LandmarkEstimatorWeightedMean4D3D>;

} // namespace srrg2_slam_interfaces
