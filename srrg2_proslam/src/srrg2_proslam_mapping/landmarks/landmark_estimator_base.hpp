#pragma once

#include <srrg_config/configurable.h>
#include <srrg_pcl/point_types.h>
#include <srrg_property/property.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_slam_interfaces {

  //! landmark estimator interface (abstract base class)
  template <typename MeasurementVectorType_, typename LandmarkType_>
  class LandmarkEstimatorBase_ : public srrg2_core::Configurable, public srrg2_core::Profiler {
  public:
    using MeasurementCoordinatesType = MeasurementVectorType_;
    using LandmarkType               = LandmarkType_;
    using LandmarkCoordinatesType    = srrg2_core::Vector_<float, LandmarkType::Dim>;
    using EstimateType               = srrg2_core::Isometry_<float, LandmarkType::Dim>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(srrg2_core::PropertyFloat,
          maximum_distance_geometry_meters_squared,
          "maximum distance in geometry (i.e. 3D point distance L2 norm) in meters squared",
          1,
          nullptr);

    virtual ~LandmarkEstimatorBase_() {
    }

    //! sets the current measurement associated with the landmark
    void setMeasurement(const MeasurementCoordinatesType& measurement_) {
      _measurement = measurement_;
    }

    //! sets the landmark associated with the current measurement
    void setLandmark(LandmarkType* landmark_) {
      _landmark = landmark_;
    }

    //! sets estimate for landmark position in tracker coordinate frame (!)
    void setEstimateInTracker(const LandmarkCoordinatesType& landmark_estimate_in_tracker_) {
      _landmark_estimate_in_tracker = landmark_estimate_in_tracker_;
    }

    //! sets the absolute sensor pose at which the current measurement was obtained and associated
    //! sets the relative sensor pose in local scene coordinate frame, e.g. the merger transform^-1
    //! we require to know both transforms to handle local maps with global landmarks
    virtual void setTransforms(const EstimateType& tracker_from_world_,
                               const EstimateType& scene_from_tracker_) {
      _tracker_from_world = tracker_from_world_;
      _world_from_tracker = _tracker_from_world.inverse();
      _scene_from_world   = scene_from_tracker_ * _tracker_from_world;
    }

    //! performs structure-only filtering/optimization on the landmark (and its statistics)
    virtual void compute() = 0;

  protected:
    MeasurementCoordinatesType _measurement;
    LandmarkType* _landmark = nullptr;
    LandmarkCoordinatesType _landmark_estimate_in_tracker;
    EstimateType _tracker_from_world;
    EstimateType _scene_from_world;

    // ds cached variables
    EstimateType _world_from_tracker;
  };

} // namespace srrg2_slam_interfaces
