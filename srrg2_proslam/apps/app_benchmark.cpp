#include <map>
#include <unordered_map>

#include <srrg_config/configurable_manager.h>
#include <srrg_messages/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_slam_interfaces/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include "srrg2_proslam_tracking/instances.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;
using namespace srrg2_proslam;

const std::string exe_name("app_benchmark");
#define LOG std::cerr << exe_name + "|"

class SLAMBenchmark : public MultiGraphSLAM3D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseType     = MultiGraphSLAM3D;
  using ThisType     = SLAMBenchmark;
  using LocalMapType = BaseType::LocalMapType;

  struct StampedIsometry3f {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StampedIsometry3f(const double& ts_, const Isometry3f& iso_) {
      timestamp = ts_;
      isometry  = iso_;
    }
    double timestamp    = -1.f;
    Isometry3f isometry = Isometry3f::Identity();
  };

  using TimestampIsometry3fMap = std::map<double,
                                          Isometry3f,
                                          std::less<double>,
                                          Eigen::aligned_allocator<std::pair<double, Isometry3f>>>;
  using StampedIsometry3fVector =
    std::vector<StampedIsometry3f, Eigen::aligned_allocator<StampedIsometry3f>>;
  using LocalMapLocalPosesMap =
    std::map<LocalMapType*,
             StampedIsometry3fVector,
             std::less<LocalMapType*>,
             Eigen::aligned_allocator<std::pair<LocalMapType*, StampedIsometry3fVector>>>;
  using TimestampLocalMapUMap = std::unordered_map<double, LocalMapType*>;
  SLAMBenchmark() : BaseType() {
  }
  virtual ~SLAMBenchmark() = default;

  void benchmarkCompute() {
    // ds optional initialization TODO rework this
    auto initializer = param_initializer.value();
    if (initializer && !initializer->isInitialized()) {
      std::cerr << "SLAMBenchmark::benchmarkCompute|initializing" << std::endl;

      // ds set shared platform if needed TODO rework this
      if (!_platform) {
        ThisType::setPlatform(PlatformPtr(new Platform()));
      }

      // ds run initializer
      initializer->setMeasurement(_measurement);
      initializer->initialize();

      // ds skip processing if insufficient data is available for initialization
      if (!initializer->isInitialized()) {
        return;
      }
    }

    if (!_graph) {
      std::cerr << "SLAMBenchmark::benchmarkCompute|graph not set, creating instance" << std::endl;
      _graph = FactorGraphPtr(new FactorGraph);
    }

    auto tracker = param_tracker.value();
    if (!tracker) {
      throw std::runtime_error("SLAMBenchmark::benchmarkCompute|tracker not set");
    }

    auto splitting_criterion = param_splitting_criterion.value();
    if (!splitting_criterion) {
      throw std::runtime_error("SLAMBenchmark::benchmarkCompute|splitting criterion not set");
    }
    splitting_criterion->setSlamAlgorithm(this);
    tracker->setPlatform(this->_platform);

    if (!_current_local_map) {
      std::cerr << "SLAMBenchmark::benchmarkCompute|creating first map!" << std::endl;
      makeNewMap();
      tracker->setScene(&_current_local_map->dynamic_properties);
      tracker->setEstimate(_robot_pose_in_local_map);
    }

    // global pose
    tracker->setMeasurement(_measurement);
    tracker->adaptMeasurements();
    tracker->align(); // ia I really do not like this name

    // ia BENCHMARKING STUFF
    {
      // ia check if the current local map has already been indexed
      if (!_stamped_keyframe_trajectory.count(_current_ts)) {
        _stamped_keyframe_trajectory.insert(std::make_pair(_current_ts, _current_local_map));
      }

      // ia check if we have indexed also the local poses inside the map
      if (!_full_trajectory.count(_current_local_map)) {
        StampedIsometry3fVector empty_vector;
        empty_vector.clear();
        _full_trajectory.insert(std::make_pair(_current_local_map, empty_vector));
      }

      _full_trajectory.at(_current_local_map)
        .push_back(StampedIsometry3f(_current_ts, tracker->estimate()));
    }

    switch (tracker->status()) {
      case TrackerBase::Initializing:
        std::cerr << FG_YELLOW("SLAMBenchmark::benchmarkCompute|initializing!") << std::endl;
        break;
      case TrackerBase::Initialized:
        std::cerr << FG_YELLOW("SLAMBenchmark::benchmarkCompute|ready to go!") << std::endl;
        break;
      case TrackerBase::Tracking: {
        _robot_pose_in_local_map = tracker->estimate();
        splitting_criterion->compute();
        if (!splitting_criterion->hasToSplit()) {
          break;
        }
        GraphItemPtrSet_<LoopClosurePtrType> relocalize_closures;
        loopDetect();
        loopValidate(relocalize_closures);
        optimize();
        relocalize(relocalize_closures);
        if (!_relocalized_closure) {
          // ds a new sub-scene (e.g. local map will be populated)
          makeNewMap(1);
        } else {
          // ds we do not have to create a new local scene but instead reload an old one
          std::cerr << FG_GREEN(
                         "SLAMBenchmark::benchmarkCompute|reloading old local map (graph ID: ")
                    << _current_local_map->graphId() << FG_GREEN(")") << std::endl;

          // ds the correspondences from the closures can be re-used by the tracker for merging
          // ds it is fine if the correspondences are destroyed by scope after the merge
          tracker->setClosure(_relocalized_closure->correspondences,
                              _relocalized_closure->measurement().inverse(),
                              _robot_pose_in_local_map);

          // ds reset closure buffer for next iteration (potentially without relocalization)
          _relocalized_closure = nullptr;
        }
        tracker->setScene(&_current_local_map->dynamic_properties);
        tracker->setEstimate(_robot_pose_in_local_map); // ds estimate is set to I after makeNewMap
        break;
      }
      case TrackerBase::Lost:
        std::flush(std::cerr);
        std::cerr << FG_YELLOW("SLAMBenchmark::benchmarkCompute|local map lost") << std::endl;
        std::flush(std::cerr);
        makeNewMap(0.1);
        tracker->setScene(&_current_local_map->dynamic_properties);
        // tracker->setEstimate(_robot_pose_in_local_map);
        break;
      case TrackerBase::Error:
        std::flush(std::cerr);
        std::cerr << FG_YELLOW("SLAMBenchmark::benchmarkCompute|tracker status ERROR") << std::endl;
        std::flush(std::cerr);
        break;
      default:
        throw std::runtime_error("SLAMBenchmark::benchmarkCompute|unknown tracker status");
    }

    // ds set global pose of current tracker scene to the tracker
    // ds this information will be used for global landmark updates in the merge phase
    tracker->setSceneInWorld(_current_local_map->estimate().inverse());
    tracker->merge();
    ++_number_of_processed_measurements;
  }

  // ia the only one overridable is this one :)
  bool putMessage(BaseSensorMessagePtr measurement_) override {
    _current_ts = measurement_->timestamp.value();
    BaseType::setMeasurement(measurement_);
    ThisType::benchmarkCompute();
    return true;
  }

  void unrollFullTrajectory(TimestampIsometry3fMap& unrolled_trajectory_) const {
    unrolled_trajectory_.clear();
    for (const auto& entry : _full_trajectory) {
      for (const auto& stamped_pose : entry.second) {
        const Isometry3f global_pose = entry.first->estimate() * stamped_pose.isometry;
        unrolled_trajectory_.insert(std::make_pair(stamped_pose.timestamp, global_pose));
      }
    }
  }

  void saveTrajectoryKitti(const std::string& filename_) const {
    // ia reorder the fucking trajectory according to timestamp
    TimestampIsometry3fMap ordered_global_trajectory;
    unrollFullTrajectory(ordered_global_trajectory);

    std::ofstream stream(filename_, std::ofstream::out);
    if (!stream.good() || !stream.is_open()) {
      throw std::runtime_error("SLAMBenchmark::saveTrajectoryKitti|unable to open file: " +
                               filename_);
    }
    stream << std::fixed;
    stream << std::setprecision(9);
    for (const auto& entry : ordered_global_trajectory) {
      _writeKittiFormat(stream, entry.second);
      // stream << stamped_pose.timestamp << " ";
      stream << std::endl;
    }
  }

  void saveTrajectoryTUM(const std::string& filename_) const {
    // ia reorder the fucking trajectory according to timestamp
    TimestampIsometry3fMap ordered_global_trajectory;
    unrollFullTrajectory(ordered_global_trajectory);

    std::ofstream stream(filename_, std::ofstream::out);
    if (!stream.good() || !stream.is_open()) {
      throw std::runtime_error("SLAMBenchmark::saveTrajectoryTUM|unable to open file: " +
                               filename_);
    }
    stream << std::fixed;
    stream << std::setprecision(9);
    for (const auto& entry : ordered_global_trajectory) {
      _writeTUMFormat(stream, entry.first, entry.second);
      stream << std::endl;
    }
  }

protected:
  LocalMapLocalPosesMap _full_trajectory;
  TimestampLocalMapUMap _stamped_keyframe_trajectory;
  double _current_ts = -1.f;

  inline void _writeKittiFormat(std::ostream& stream_, const Isometry3f& pose_) const {
    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 4; ++c) {
        stream_ << pose_.matrix()(r, c) << " ";
      }
    }
  }

  inline void
  _writeTUMFormat(std::ostream& stream_, const double& ts_, const Isometry3f& pose_) const {
    const auto v = geometry3d::t2tqxyzw(pose_);
    stream_ << ts_ << " ";
    for (int i = 0; i < v.rows(); ++i) {
      stream_ << v[i] << " ";
    }
  }
};

int main(int argc, char** argv) {
  srrg2_core::point_cloud_registerTypes();
  srrg2_core::messages_registerTypes();
  srrg2_slam_interfaces::slam_interfaces_registerTypes();
  srrg2_proslam::srrg2_proslam_tracking_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_config_name(&cmd, "c", "config", "configuration to use in this bench", "");
  ArgumentString arg_dataset(&cmd, "d", "dataset", "dataset filename BOSS ONLY", "");
  ArgumentString arg_slam_name(&cmd, "sn", "slam-name", "slam name in the config", "slam");
  ArgumentString arg_sync_name(&cmd, "ssn", "sync-name", "sync name in the config", "sync");
  ArgumentString arg_source_name(&cmd, "fsn", "source-name", "source name in the config", "source");
  ArgumentString arg_trajectory(&cmd, "t", "trajectory", "trajectory filename", "");
  // ArgumentString arg_trajectory_type(
  //   &cmd, "tt", "trajectory-type", "choose between { kitti , tum }", "kitti");
  cmd.parse();

  if (!arg_config_name.isSet() || !arg_dataset.isSet()) {
    LOG << FG_RED("ERROR, not enough arguments");
    std::cerr << cmd.options() << std::endl;
    throw std::runtime_error(exe_name + "|ERROR, invalid shell arguments");
  }

  if (!isAccessible(arg_config_name.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access config file [" +
                             arg_config_name.value() + " ]");
  }

  if (!isAccessible(arg_dataset.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access dataset [" + arg_dataset.value() +
                             " ]");
  }

  // ia load config and get objects
  LOG << "reading configuration [ " << arg_config_name.value() << " ]\n";
  ConfigurableManager manager;
  manager.read(arg_config_name.value());

  auto slammer_ptr = manager.getByName<MultiGraphSLAM3D>(arg_slam_name.value());
  auto sync_ptr    = manager.getByName<MessageSynchronizedSource>(arg_sync_name.value());
  auto source_ptr  = manager.getByName<MessageFileSource>(arg_source_name.value());

  if (slammer_ptr == nullptr || sync_ptr == nullptr || source_ptr == nullptr) {
    throw std::runtime_error(exe_name + "|ERROR, invalid config");
  }

  SLAMBenchmark benchamin;
  benchamin.param_closure_validator.setValue(slammer_ptr->param_closure_validator.value());
  benchamin.param_global_solver.setValue(slammer_ptr->param_global_solver.value());
  benchamin.param_initializer.setValue(slammer_ptr->param_initializer.value());
  benchamin.param_loop_detector.setValue(slammer_ptr->param_loop_detector.value());
  benchamin.param_relocalizer.setValue(slammer_ptr->param_relocalizer.value());
  benchamin.param_splitting_criterion.setValue(slammer_ptr->param_splitting_criterion.value());
  benchamin.param_tf_topic.setValue(slammer_ptr->param_tf_topic.value());
  benchamin.param_tracker.setValue(slammer_ptr->param_tracker.value());

  LOG << "opening dataset [ " << arg_dataset.value() << " ]\n";
  source_ptr->open(arg_dataset.value());

  // ia start processing the thing
  LOG << "start processing\n";
  BaseSensorMessagePtr msg = nullptr;
  while ((msg = sync_ptr->getMessage())) {
    benchamin.putMessage(msg);
  }

  if (arg_trajectory.isSet()) {
    LOG << "saving trajectory in [ " << arg_trajectory.value() << " ]\n";
    benchamin.saveTrajectoryKitti(arg_trajectory.value() + "_KITTI");
    benchamin.saveTrajectoryTUM(arg_trajectory.value() + "_TUM");
  }

  // if (arg_trajectory.isSet()) {
  //   if (arg_trajectory_type.value() == "kitti") {
  //     LOG << "writing trajectory in [ " << arg_trajectory.value() << " ] - format [ "
  //         << FG_CYAN(arg_trajectory_type.value()) << " ]\n";
  //     // benchamin.saveTrajectoryKitti(arg_trajectory.value());
  //   } else if (arg_trajectory_type.value() == "tum") {
  //     LOG << "writing trajectory in [ " << arg_trajectory.value() << " ] - format [ "
  //         << FG_CYAN(arg_trajectory_type.value()) << " ]\n";
  //   } else {
  //     throw std::runtime_error(exe_name + "|ERROR, invalid trajectory format [" +
  //                              arg_trajectory_type.value() + " ]");
  //   }
  // }

  return 0;
}