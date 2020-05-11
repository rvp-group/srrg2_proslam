#include "instances.h"

#include "multi_tracker_3dqr.h"
#include <srrg2_slam_interfaces/trackers/tracker_slice_processor_impl.cpp>

namespace srrg2_proslam {

  // ds help the poor, confused linker
  namespace dummy {
    static TrackerSliceProcessorProjectiveDepth TrackerSliceProcessorProjectiveDepth
      __attribute__((unused));
    static TrackerSliceProcessorStereoProjective TrackerSliceProcessorStereoProjective
      __attribute__((unused));
    static MultiTracker3DQR MultiTracker3DQR __attribute__((unused));
  } // namespace dummy

  void srrg2_proslam_tracking_registerTypes() {
    srrg2_proslam_registration_registerTypes();
    srrg2_proslam_mapping_registerTypes();

    BOSS_REGISTER_CLASS(TrackerSliceProcessorProjectiveDepth);
    BOSS_REGISTER_CLASS(TrackerSliceProcessorStereoProjective);
    BOSS_REGISTER_CLASS(MultiTracker3DQR);
  }

} // namespace srrg2_proslam
