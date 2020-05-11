#pragma once

#include <srrg_config/configurable.h>
#include <srrg_pcl/point_types.h>
#include <srrg_property/property.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_proslam {

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
    void setLandmarkInSensor(const LandmarkCoordinatesType& landmark_in_sensor_) {
      _landmark_in_sensor = landmark_in_sensor_;
    }

    //! sets the absolute sensor pose at which the current measurement was obtained and associated
    //! sets the relative sensor pose in local scene coordinate frame, e.g. the merger transform^-1
    //! we require to know both transforms to handle local maps with global landmarks
    virtual void setTransforms(const EstimateType& measurement_in_world_,
                               const EstimateType& measurement_in_scene_) {
      _sensor_in_world     = measurement_in_world_;
      _sensor_in_local_map = measurement_in_scene_;

      _world_in_sensor    = _sensor_in_world.inverse();
      _local_map_in_world = _sensor_in_world * _sensor_in_local_map.inverse();
      _world_in_local_map = _sensor_in_local_map * _world_in_sensor;
    }

    //! performs structure-only filtering/optimization on the landmark (and its statistics)
    virtual void compute() = 0;

  protected:
    MeasurementCoordinatesType _measurement;
    LandmarkType* _landmark                     = nullptr;
    LandmarkCoordinatesType _landmark_in_sensor = LandmarkCoordinatesType::Zero();

    EstimateType _sensor_in_world     = EstimateType::Identity();
    EstimateType _sensor_in_local_map = EstimateType::Identity();
    EstimateType _world_in_sensor     = EstimateType::Identity();

    EstimateType _local_map_in_world = EstimateType::Identity();
    EstimateType _world_in_local_map = EstimateType::Identity();
  };

} // namespace srrg2_proslam
