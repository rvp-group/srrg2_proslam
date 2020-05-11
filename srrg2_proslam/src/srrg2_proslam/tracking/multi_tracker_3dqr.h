#include <srrg2_slam_interfaces/trackers/multi_tracker.h>

namespace srrg2_proslam {
  //  class MultiTracker3DQR : public srrg2_slam_interfaces::MultiTracker3D {
  //  public:
  //    using ThisType              = MultiTracker3DQR;
  //    using BaseType              = srrg2_slam_interfaces::MultiTracker3D;
  //    MultiTracker3DQR()          = default;
  //    virtual ~MultiTracker3DQR() = default;
  //
  //    virtual void _updateRobotInLocalMap(const EstimateType& moving_in_fixed_) {
  //      EstimateType to_multiply = moving_in_fixed_;
  //      std::cerr << "al est: " << FG_BWHITE(geometry3d::t2v(to_multiply).transpose()) <<
  //      std::endl;
  //
  //      _robot_in_local_map = _robot_in_local_map * to_multiply;
  //      std::cerr << "result: " << FG_BWHITE(geometry3d::t2v(_robot_in_local_map).transpose())
  //                << std::endl;
  //      setRobotInLocalMap(_robot_in_local_map);
  //    }
  //  };

  using MultiTracker3DQR = srrg2_slam_interfaces::MultiTracker3D;
} // namespace srrg2_proslam
