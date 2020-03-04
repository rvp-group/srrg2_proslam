#include "instances.h"

#include "multi_tracker_3dqr.h"
#include "srrg_slam_interfaces/multi_tracker_slice_impl.cpp"

namespace srrg2_proslam {

  // ds help the poor, confused linker
  namespace dummy {
    static TrackerSliceProjectiveDepth TrackerSliceProjectiveDepth;
    static TrackerSliceStereoProjective TrackerSliceStereoProjective;
    static MultiTracker3DQR MultiTracker3DQR;
  } // namespace dummy

  void srrg2_proslam_tracking_registerTypes() {
    srrg2_proslam_localization_registerTypes();
    srrg2_proslam_mapping_registerTypes();

    BOSS_REGISTER_CLASS(TrackerSliceProjectiveDepth);
    BOSS_REGISTER_CLASS(TrackerSliceStereoProjective);
    BOSS_REGISTER_CLASS(MultiTracker3DQR);
  }

} // namespace srrg2_proslam
