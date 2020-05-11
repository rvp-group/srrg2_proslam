#pragma once
#include <srrg2_slam_interfaces/mapping/merger.h>
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_projector_types.h>
#include <srrg_pcl/point_unprojector_types.h>

#include "srrg2_proslam/mapping/landmarks/landmark_estimator_base.hpp"

namespace srrg2_proslam {

  // ds binning bookkeeping TODO refactor... maybe with tuple
  using BinMap =
    std::unordered_map<size_t /*row*/, std::unordered_map<size_t /*cols*/, size_t /*index*/>>;

  // ds base projective merger interface
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class MergerProjective_
    : public srrg2_slam_interfaces::MergerCorrespondence_<TransformType_, FixedType_, MovingType_>,
      public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType =
      srrg2_slam_interfaces::MergerCorrespondence_<TransformType_, FixedType_, MovingType_>;
    using ScenePointCloudType       = FixedType_;
    using ScenePointType            = typename ScenePointCloudType::value_type;
    using MeasurementPointCloudType = MovingType_;
    using MeasurementPointType      = typename MeasurementPointCloudType::value_type;
    using ProjectoryType            = srrg2_core::PointProjectorPinhole_<ScenePointCloudType>;
    using ProjectoryTypePtr         = std::shared_ptr<ProjectoryType>;
    using LandmarkEstimatorType =
      LandmarkEstimatorBase_<typename MeasurementPointType::VectorType, ScenePointType>;
    using LandmarkEstimatorTypePtr = std::shared_ptr<LandmarkEstimatorType>;

    PARAM(srrg2_core::PropertyConfigurable_<LandmarkEstimatorType>,
          landmark_estimator,
          "landmark estimator used to refine landmark positions in the map (structure-only)",
          nullptr,
          nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<ProjectoryType>,
          projector,
          "pinhole projector used for projective merging",
          ProjectoryTypePtr(new ProjectoryType()),
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          maximum_distance_appearance,
          "maximum permitted correspondence response for merging a point",
          50,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          number_of_row_bins,
          "number of bins in row direction (feature density regulation)",
          10,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          number_of_col_bins,
          "number of bins in column direction (feature density regulation)",
          30,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          target_merge_ratio,
          "target merge ratio (#merges/#correspondences)",
          0.5,
          nullptr);
    PARAM(srrg2_core::PropertyBool,
          enable_conservative_addition,
          "enables minimal addition of new points (instead all) if merge ratio is not reached",
          false,
          nullptr);

    virtual ~MergerProjective_() {
    }

    virtual void compute() override;

  protected:
    //! pre-compute routine, used to initialize certain modules of subclasses
    virtual void _precompute() {
      // ds optional overriding
    }

    //! this method integrates an adapted measurement into an existing scene point
    //! returns true upon successful merge
    virtual bool _updatePoint(const MeasurementPointType& measured_point_,
                              ScenePointType& scene_point_);

    //! adds points to the scene according to the specified binning grid (if enabled, otherwise all)
    //! added points are added to the occupied_bin_map_ for subsequent/repeated calls
    void _addPoints(BinMap& occupied_bin_map_, const size_t& number_of_points_to_add_);

    //! checks if point_a_ is more suitable than point_b_ for addition - default never overwrite
    virtual bool _isBetterForAddition(const MeasurementPointType& point_a_,
                                      const MeasurementPointType& point_b_) const {
      return false;
    }

    //! adapts points from measurement cloud to scene cloud (e.g. unprojection, triangulation, ..)
    virtual void
    _adaptFromMeasurementToScene(const MeasurementPointCloudType& points_in_measurement_space_,
                                 ScenePointCloudType& points_in_scene_space_) const = 0;

    //! initializes a point as landmark for addition
    void _initializeLandmark(const MeasurementPointType& measurement_,
                             ScenePointType& point_,
                             const TransformType_& sensor_in_world_,
                             const TransformType_& world_in_sensor_) const;

    //! current binning grid size - autoconfigured in each compute call
    float _row_bin_width_pixels = 0;
    float _col_bin_width_pixels = 0;
  };

} // namespace srrg2_proslam
