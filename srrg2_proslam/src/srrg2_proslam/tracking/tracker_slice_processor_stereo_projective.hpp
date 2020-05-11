#pragma once
#include <srrg_pcl/point_types.h>
#include <srrg2_slam_interfaces/trackers/tracker_slice_processor.h>

#include "srrg2_proslam/sensor_processing/raw_data_preprocessor_stereo_projective.h"
#include "srrg2_proslam/mapping/mergers/merger_projective.h"
#include "srrg2_proslam/mapping/scene_clipper_projective_3d.h"

namespace srrg2_proslam {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  class TrackerSliceProcessorStereoProjective
    : public TrackerSliceProcessor_<Isometry3f,
                                    PointIntensityDescriptor4fVectorCloud,
                                    PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = TrackerSliceProcessor_<Isometry3f,
                                            PointIntensityDescriptor4fVectorCloud,
                                            PointIntensityDescriptor3fVectorCloud>;

    //! propagate platform to stereo modules that need it
    void setPlatform(PlatformPtr platform_) override {
      BaseType::setPlatform(platform_);

      // ds set platform to merger TODO refactor uagh
      MergerRigidStereoTriangulationPtr merger_stereo =
        param_merger.getSharedPtr<MergerRigidStereoTriangulation>();
      if (!merger_stereo) {
        MergerRigidStereoProjectiveEKFPtr merger_stereo_projective =
          param_merger.getSharedPtr<MergerRigidStereoProjectiveEKF>();
        if (!merger_stereo_projective) {
          throw std::runtime_error("TrackerSliceProcessorStereoProjective::setPlatform|invalid merger type");
        } else {
          merger_stereo_projective->param_triangulator->setPlatform(platform_);
        }
      } else {
        merger_stereo->param_triangulator->setPlatform(platform_);
      }
    }
  };

  using TrackerSliceProcessorStereoProjectivePtr =
    std::shared_ptr<TrackerSliceProcessorStereoProjective>;

} // namespace srrg2_proslam
