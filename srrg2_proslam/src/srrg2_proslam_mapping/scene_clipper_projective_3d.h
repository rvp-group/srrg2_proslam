#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_intensity_descriptor.h>
#include <srrg_pcl/point_projector_types.h>
#include <srrg_slam_interfaces/scene_clipper.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_proslam {

  class SceneClipperProjective3D
    : public srrg2_slam_interfaces::
        SceneClipper_<srrg2_core::Isometry3f, srrg2_core::PointIntensityDescriptor3fVectorCloud>,
      public srrg2_core::Profiler {
  public:
    using ThisType = SceneClipperProjective3D;
    using BaseType =
      SceneClipper_<srrg2_core::Isometry3f, srrg2_core::PointIntensityDescriptor3fVectorCloud>;
    using SceneType         = typename BaseType::SceneType;
    using ProjectoryType    = srrg2_core::PointIntensityDescriptor3fProjectorPinhole;
    using ProjectoryPtrType = srrg2_core::PointIntensityDescriptor3fProjectorPinholePtr;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(srrg2_core::PropertyConfigurable_<ProjectoryType>,
          projector,
          "pinhole projector used to determine whether points lie in the current view",
          ProjectoryPtrType(new ProjectoryType()),
          nullptr);
    void compute() override;
    const std::vector<int> globalIndices() const override {
      return _global_indices;
    }

  protected:
    //! mapping from local to global indices (required for e.g. correspondence-based merging)
    std::vector<int> _global_indices;
  };

  using SceneClipperProjective3DPtr = std::shared_ptr<SceneClipperProjective3D>;

} // namespace srrg2_proslam
