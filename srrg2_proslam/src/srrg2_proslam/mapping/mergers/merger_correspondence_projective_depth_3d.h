#pragma once
#include <srrg_pcl/point_unprojector_types.h>
#include <srrg2_slam_interfaces/mapping/merger_correspondence_homo.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_proslam {

  class MergerCorrespondenceProjectiveDepth3D
    : public srrg2_slam_interfaces::MergerCorrespondencePointIntensityDescriptor3f,
      public srrg2_core::Profiler {
  public:
    using BaseType = srrg2_slam_interfaces::MergerCorrespondencePointIntensityDescriptor3f;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(
      srrg2_core::PropertyConfigurable_<srrg2_core::PointIntensityDescriptor3fUnprojectorPinhole>,
      unprojector,
      "point unprojector used to obtain points in camera frame from adapted UV-D measurements",
      srrg2_core::PointIntensityDescriptor3fUnprojectorPinholePtr(
        new srrg2_core::PointIntensityDescriptor3fUnprojectorPinhole()),
      nullptr);

    void compute() override;
  };

  using MergerCorrespondenceProjectiveDepth3DPtr =
    std::shared_ptr<MergerCorrespondenceProjectiveDepth3D>;

} // namespace srrg2_proslam
