#include <srrg_slam_interfaces/multi_tracker.h>

namespace srrg2_proslam {
  class MultiTracker3DQR : public srrg2_slam_interfaces::MultiTracker3D {
  public:
    using ThisType              = MultiTracker3DQR;
    using BaseType              = srrg2_slam_interfaces::MultiTracker3D;
    MultiTracker3DQR()          = default;
    virtual ~MultiTracker3DQR() = default;

    virtual void _updateTrackerEstimate(const EstimateType& aligner_estimate_) {
      EstimateType to_multiply = aligner_estimate_;
      std::cerr << "al est: " << FG_BWHITE(geometry3d::t2v(to_multiply).transpose()) << std::endl;

      _tracker_estimate = _tracker_estimate * to_multiply;
      std::cerr << "result: " << FG_BWHITE(geometry3d::t2v(_tracker_estimate).transpose())
                << std::endl;
      setEstimate(_tracker_estimate);
    }
  };
} // namespace srrg2_proslam
