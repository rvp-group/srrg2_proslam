#pragma once

#include <srrg_data_structures/platform.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_property/property_eigen.h>
#include <srrg_slam_interfaces/multi_aligner_slice.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_projective_depth_error_factor.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_projective_error_factor.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_rectified_stereo_projective_error_factor.h>

#include "srrg2_proslam_adaptation/correspondence_finders/correspondence_finder_projective_circle.h"
#include "srrg2_proslam_adaptation/correspondence_finders/correspondence_finder_projective_kdtree.h"
#include "srrg2_proslam_adaptation/correspondence_finders/correspondence_finder_projective_square.h"

namespace srrg2_proslam {

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceProjective_
    : public srrg2_slam_interfaces::MultiAlignerSlice_<FactorType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType      = FactorType_;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::value_type;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::value_type;
    using ThisType        = AlignerSliceProjective_<FactorType, FixedType, MovingType>;
    using BaseType = srrg2_slam_interfaces::MultiAlignerSlice_<FactorType, FixedType, MovingType>;
    using EstimateType = typename BaseType::EstimateType;
    using CFTypeBase =
      srrg2_slam_interfaces::CorrespondenceFinder_<EstimateType, FixedType, MovingType>;
    using ProjectiveCFType = srrg2_slam_interfaces::
      CorrespondenceFinderProjectiveBase<EstimateType, FixedType, MovingType>;

    static constexpr int ErrorDim = FactorType::ErrorDim;
    using DiagInfoVector          = srrg2_core::Vector_<float, ErrorDim>;

    PARAM(srrg2_core::PropertyEigen_<DiagInfoVector>,
          diagonal_info_matrix,
          "value of the information matrix's diagonal",
          DiagInfoVector::Zero(),
          nullptr);

    PARAM(
      srrg2_core::PropertyConfigurable_<srrg2_core::CameraMatrixOwner_<MovingType::GeometryDim>>,
      projector,
      "link to a projector where to take the infor for the factor",
      nullptr,
      nullptr);

    AlignerSliceProjective_();

    virtual ~AlignerSliceProjective_() = default;

    void bindFixed() override;

    void setupFactor() override;

    void setEstimate(const EstimateType& est_) override;

    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override;
  };

  using AlignerSliceProjective =
    AlignerSliceProjective_<srrg2_solver::SE3ProjectiveErrorFactor,
                            srrg2_core::PointIntensityDescriptor2fVectorCloud,
                            srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using AlignerSliceProjectivePtr = std::shared_ptr<AlignerSliceProjective>;

  // srrg w/ sensor
  class AlignerSliceProjectiveWithSensor
    : public AlignerSliceProjective_<srrg2_solver::SE3ProjectiveWithSensorErrorFactor,
                                     srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                     srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = AlignerSliceProjective_<srrg2_solver::SE3ProjectiveWithSensorErrorFactor,
                                             srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                             srrg2_core::PointIntensityDescriptor3fVectorCloud>;
    void setupFactor() override {
      BaseType::setupFactor();
      ThisType::setupFactorWithSensor("AlignerSliceProjectiveWithSensor");
    }
  };
  using AlignerSliceProjectiveWithSensorPtr = std::shared_ptr<AlignerSliceProjectiveWithSensor>;

  using AlignerSliceProjectiveDepth =
    AlignerSliceProjective_<srrg2_solver::SE3ProjectiveDepthErrorFactor,
                            srrg2_core::PointIntensityDescriptor3fVectorCloud,
                            srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using AlignerSliceProjectiveDepthPtr = std::shared_ptr<AlignerSliceProjectiveDepth>;

  class AlignerSliceProjectiveDepthWithSensor
    : public AlignerSliceProjective_<srrg2_solver::SE3ProjectiveDepthWithSensorErrorFactorAD,
                                     srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                     srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      AlignerSliceProjective_<srrg2_solver::SE3ProjectiveDepthWithSensorErrorFactorAD,
                              srrg2_core::PointIntensityDescriptor3fVectorCloud,
                              srrg2_core::PointIntensityDescriptor3fVectorCloud>;

    void setupFactor() override {
      BaseType::setupFactor();
      ThisType::setupFactorWithSensor("AlignerSliceProjectiveDepthWithSensor");
    }
  };

  using AlignerSliceProjectiveDepthWithSensorPtr =
    std::shared_ptr<AlignerSliceProjectiveDepthWithSensor>;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceStereoProjective_
    : public AlignerSliceProjective_<FactorType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType      = FactorType_;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::value_type;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::value_type;
    using ThisType        = AlignerSliceStereoProjective_<FactorType, FixedType, MovingType>;
    using BaseType        = AlignerSliceProjective_<FactorType, FixedType, MovingType>;
    using EstimateType    = typename BaseType::EstimateType;

    PARAM(srrg2_core::PropertyString,
          frame_camera_left,
          "topic for the camera left info",
          "camera_left",
          nullptr);

    PARAM(srrg2_core::PropertyString,
          frame_camera_right,
          "topic for the camera right info",
          "camera_right",
          nullptr);

    PARAM(srrg2_core::PropertyBool,
          enable_inverse_depth_weighting,
          "toggles point weighting by inverse depth",
          false,
          nullptr);

    PARAM(srrg2_core::PropertyBool,
          enable_point_covariance_integration,
          "toggles individual point covariance integration",
          false,
          nullptr);

    virtual ~AlignerSliceStereoProjective_() = default;

    void bindFixed() override;

    void setupFactor() override;

  protected:
    //! cache the baseline measurement from platform
    srrg2_core::Vector3f _baseline_right_in_left_pixelsmeters = srrg2_core::Vector3f::Zero();
    bool _baseline_set                                        = false;
    float _mean_disparity                                     = 0;
  };

  using AlignerSliceStereoProjective =
    AlignerSliceStereoProjective_<srrg2_solver::SE3RectifiedStereoProjectiveErrorFactor,
                                  srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                  srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using AlignerSliceStereoProjectivePtr = std::shared_ptr<AlignerSliceStereoProjective>;

  class AlignerSliceStereoProjectiveWithSensor
    : public AlignerSliceStereoProjective_<
        srrg2_solver::SE3RectifiedStereoProjectiveWithSensorErrorFactor,
        srrg2_core::PointIntensityDescriptor4fVectorCloud,
        srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      AlignerSliceStereoProjective_<srrg2_solver::SE3RectifiedStereoProjectiveWithSensorErrorFactor,
                                    srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                    srrg2_core::PointIntensityDescriptor3fVectorCloud>;

    void setupFactor() override {
      BaseType::setupFactor();
      ThisType::setupFactorWithSensor("AlignerSliceStereoProjectiveWithSensor");
    }
  };

  using AlignerSliceStereoProjectiveWithSensorPtr =
    std::shared_ptr<AlignerSliceStereoProjectiveWithSensor>;

} // namespace srrg2_proslam
