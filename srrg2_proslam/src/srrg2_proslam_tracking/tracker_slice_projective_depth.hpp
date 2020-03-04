#pragma once
#include <srrg_pcl/point_types.h>
#include <srrg_slam_interfaces/multi_tracker_slice.h>

#include "srrg2_proslam_adaptation/measurement_adaptor_monocular_depth.h"
#include "srrg2_proslam_mapping/mergers/merger_correspondence_projective_depth_3d.h"
#include "srrg2_proslam_mapping/scene_clipper_projective_3d.h"

namespace srrg2_proslam {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  class TrackerSliceProjectiveDepth
    : public MultiTrackerSlice_<Isometry3f,
                                PointIntensityDescriptor3fVectorCloud,
                                PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~TrackerSliceProjectiveDepth() = default;

    void merge() override;
    void clip() override;
  };

  using TrackerSliceProjectiveDepthPtr = std::shared_ptr<TrackerSliceProjectiveDepth>;

} // namespace srrg2_proslam
