#pragma once

#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h>
#include <srrg_data_structures/platform.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_property/property_eigen.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_projective_depth_error_factor.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_projective_error_factor.h>
#include <srrg_solver/variables_and_factors/types_projective/se3_rectified_stereo_projective_error_factor.h>

#include "correspondence_finders/correspondence_finder_projective_circle.h"
#include "correspondence_finders/correspondence_finder_projective_kdtree.h"
#include "correspondence_finders/correspondence_finder_projective_square.h"

namespace srrg2_proslam {

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceProcessorProjective_
    : public srrg2_slam_interfaces::AlignerSliceProcessor_<FactorType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType      = FactorType_;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::value_type;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::value_type;
    using ThisType        = AlignerSliceProcessorProjective_<FactorType, FixedType, MovingType>;
    using BaseType =
      srrg2_slam_interfaces::AlignerSliceProcessor_<FactorType, FixedType, MovingType>;
    using EstimateType = typename BaseType::EstimateType;
    using CFTypeBase =
      srrg2_slam_interfaces::CorrespondenceFinder_<EstimateType, FixedType, MovingType>;
    using ProjectiveCFType =
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

    AlignerSliceProcessorProjective_();

    virtual ~AlignerSliceProcessorProjective_() = default;

    void bindFixed() override;

    void setupFactor() override;

    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override;
  };

  using AlignerSliceProcessorProjective =
    AlignerSliceProcessorProjective_<srrg2_solver::SE3ProjectiveErrorFactor,
                                     srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                     srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using AlignerSliceProcessorProjectivePtr = std::shared_ptr<AlignerSliceProcessorProjective>;

  // srrg w/ sensor
  class AlignerSliceProcessorProjectiveWithSensor
    : public AlignerSliceProcessorProjective_<srrg2_solver::SE3ProjectiveWithSensorErrorFactor,
                                              srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                              srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      AlignerSliceProcessorProjective_<srrg2_solver::SE3ProjectiveWithSensorErrorFactor,
                                       srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                       srrg2_core::PointIntensityDescriptor3fVectorCloud>;
    void setupFactor() override {
      BaseType::setupFactor();
      ThisType::setupFactorWithSensor("AlignerSliceProcessorProjectiveWithSensor");
    }
  };
  using AlignerSliceProcessorProjectiveWithSensorPtr =
    std::shared_ptr<AlignerSliceProcessorProjectiveWithSensor>;

  using AlignerSliceProcessorProjectiveDepth =
    AlignerSliceProcessorProjective_<srrg2_solver::SE3ProjectiveDepthErrorFactor,
                                     srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                     srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using AlignerSliceProcessorProjectiveDepthPtr =
    std::shared_ptr<AlignerSliceProcessorProjectiveDepth>;

  class AlignerSliceProcessorProjectiveDepthWithSensor
    : public AlignerSliceProcessorProjective_<srrg2_solver::SE3ProjectiveDepthWithSensorErrorFactor,
                                              srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                              srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      AlignerSliceProcessorProjective_<srrg2_solver::SE3ProjectiveDepthWithSensorErrorFactor,
                                       srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                       srrg2_core::PointIntensityDescriptor3fVectorCloud>;

    void setupFactor() override {
      BaseType::setupFactor();
      ThisType::setupFactorWithSensor("AlignerSliceProjectiveDepthWithSensor");
    }
  };

  using AlignerSliceProcessorProjectiveDepthWithSensorPtr =
    std::shared_ptr<AlignerSliceProcessorProjectiveDepthWithSensor>;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceProcessorProjectiveStereo_
    : public AlignerSliceProcessorProjective_<FactorType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType      = FactorType_;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::value_type;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::value_type;
    using ThisType     = AlignerSliceProcessorProjectiveStereo_<FactorType, FixedType, MovingType>;
    using BaseType     = AlignerSliceProcessorProjective_<FactorType, FixedType, MovingType>;
    using EstimateType = typename BaseType::EstimateType;

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

    virtual ~AlignerSliceProcessorProjectiveStereo_() = default;

    void bindFixed() override;

    void setupFactor() override;

  protected:
    //! cache the baseline measurement from platform
    srrg2_core::Vector3f _baseline_left_in_right_pixelsmeters = srrg2_core::Vector3f::Zero();
    bool _baseline_set                                        = false;
    float _mean_disparity                                     = 0;
  };

  using AlignerSliceProcessorProjectiveStereo =
    AlignerSliceProcessorProjectiveStereo_<srrg2_solver::SE3RectifiedStereoProjectiveErrorFactor,
                                           srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                           srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using AlignerSliceProcessorProjectiveStereoPtr =
    std::shared_ptr<AlignerSliceProcessorProjectiveStereo>;

  class AlignerSliceProcessorProjectiveStereoWithSensor
    : public AlignerSliceProcessorProjectiveStereo_<
        srrg2_solver::SE3RectifiedStereoProjectiveWithSensorErrorFactor,
        srrg2_core::PointIntensityDescriptor4fVectorCloud,
        srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = AlignerSliceProcessorProjectiveStereo_<
      srrg2_solver::SE3RectifiedStereoProjectiveWithSensorErrorFactor,
      srrg2_core::PointIntensityDescriptor4fVectorCloud,
      srrg2_core::PointIntensityDescriptor3fVectorCloud>;

    void setupFactor() override {
      BaseType::setupFactor();
      ThisType::setupFactorWithSensor("AlignerSliceStereoProjectiveWithSensor");
    }
  };

  using AlignerSliceProcessorProjectiveStereoWithSensorPtr =
    std::shared_ptr<AlignerSliceProcessorProjectiveStereoWithSensor>;

} // namespace srrg2_proslam
