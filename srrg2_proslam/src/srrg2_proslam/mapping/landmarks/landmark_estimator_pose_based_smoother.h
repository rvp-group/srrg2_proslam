#pragma once
#include <srrg_pcl/camera_matrix_owner.h>

#include "landmark_estimator_base.hpp"

namespace srrg2_proslam {

  template <typename MeasurementVectorType_, typename LandmarkType_>
  class LandmarkEstimatorPoseBasedSmoother_
    : public LandmarkEstimatorBase_<MeasurementVectorType_, LandmarkType_>
  /*, public srrg2_core::CameraMatrixOwner_<LandmarkType_::Dim> breaks seralization*/ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType     = LandmarkEstimatorBase_<MeasurementVectorType_, LandmarkType_>;
    using EstimateType = typename BaseType::EstimateType;

    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_number_of_iterations,
          "maximum number of LS iterations",
          100,
          nullptr);

    PARAM(srrg2_core::PropertyFloat,
          convergence_criterion_minimum_chi2_delta,
          "convergence delta",
          1e-5,
          nullptr);

    PARAM(srrg2_core::PropertyFloat,
          maximum_reprojection_error_pixels_squared,
          "maximum kernel reprojection error (pixels squared)",
          100,
          nullptr);

    PARAM(srrg2_core::PropertyUnsignedInt,
          minimum_number_of_measurements_for_optimization,
          "minimum number of required measurements before optimizing (otherwise averaging)",
          3,
          nullptr);

    void compute() override;

    // ds TODO purge once multiple serialization inheritance for camera owners is resolved
    void setCameraMatrix(const srrg2_core::Matrix3f& camera_matrix_) {
      _camera_matrix = camera_matrix_;
    }

  protected:
    //! micro helper
    void _setMeanCoordinatesInWorld(
      const srrg2_core::PointStatisticsField3D::CameraMeasurementVector& measurements_,
      srrg2_core::Vector3f& coordinates_in_world_) const;

    // ds TODO purge once multiple serialization inheritance for camera owners is resolved
    srrg2_core::Matrix3f _camera_matrix = srrg2_core::Matrix3f::Zero();

    // ds optimization buffers TODO templatize and re-use solver
    srrg2_core::Matrix3f _H             = srrg2_core::Matrix3f::Zero();
    srrg2_core::Vector3f _b             = srrg2_core::Vector3f::Zero();
    srrg2_core::Matrix3f _omega         = srrg2_core::Matrix3f::Identity();
    float _total_error_squared_previous = 0;
  };

  using LandmarkEstimatorPoseBasedSmoother2D3D =
    LandmarkEstimatorPoseBasedSmoother_<srrg2_core::Vector2f,
                                        srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorPoseBasedSmoother2D3DPtr =
    std::shared_ptr<LandmarkEstimatorPoseBasedSmoother2D3D>;

  using LandmarkEstimatorPoseBasedSmoother3D3D =
    LandmarkEstimatorPoseBasedSmoother_<srrg2_core::Vector3f,
                                        srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorPoseBasedSmoother3D3DPtr =
    std::shared_ptr<LandmarkEstimatorPoseBasedSmoother3D3D>;

  using LandmarkEstimatorPoseBasedSmoother4D3D =
    LandmarkEstimatorPoseBasedSmoother_<srrg2_core::Vector4f,
                                        srrg2_core::PointIntensityDescriptor3f>;
  using LandmarkEstimatorPoseBasedSmoother4D3DPtr =
    std::shared_ptr<LandmarkEstimatorPoseBasedSmoother4D3D>;

} // namespace srrg2_slam_interfaces
