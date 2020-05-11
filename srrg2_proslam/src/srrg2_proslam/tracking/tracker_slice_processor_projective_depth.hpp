#pragma once
#include <srrg_pcl/point_types.h>
#include <srrg2_slam_interfaces/trackers/tracker_slice_processor.h>

#include "srrg2_proslam/mapping/mergers/merger_correspondence_projective_depth_3d.h"
#include "srrg2_proslam/mapping/scene_clipper_projective_3d.h"
#include "srrg2_proslam/sensor_processing/raw_data_preprocessor_monocular_depth.h"

namespace srrg2_proslam {
  //  using namespace srrg2_core;
  //  using namespace srrg2_slam_interfaces;

  //  class TrackerSliceProcessorProjectiveDepth
  //    : public TrackerSliceProcessor_<Isometry3f,
  //                                    PointIntensityDescriptor3fVectorCloud,
  //                                    PointIntensityDescriptor3fVectorCloud> {
  //  public:
  //    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //    virtual ~TrackerSliceProcessorProjectiveDepth() = default;
  //
  //    //    void merge() override;
  //    //    void clip() override;
  //  };

  using TrackerSliceProcessorProjectiveDepth = srrg2_slam_interfaces::TrackerSliceProcessor_<
    srrg2_core::Isometry3f,
    srrg2_core::PointIntensityDescriptor3fVectorCloud,
    srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using TrackerSliceProcessorProjectiveDepthPtr =
    std::shared_ptr<TrackerSliceProcessorProjectiveDepth>;

} // namespace srrg2_proslam
