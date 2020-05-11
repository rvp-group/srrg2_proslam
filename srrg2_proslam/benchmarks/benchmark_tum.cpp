#include <srrg_benchmark/slam_benchmark_suite_tum.hpp>
#include <srrg_pcl/instances.h>
#include <srrg2_slam_interfaces/instances.h>

#include "srrg2_proslam/tracking/instances.h"

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;

#ifdef CURRENT_SOURCE_DIR
const std::string current_source_directory = CURRENT_SOURCE_DIR;
#else
const std::string current_source_directory = "./";
#endif

// ds target metrics for this benchmark TODO move to CI variables or file in repository
const Vector3f maximum_mean_translation_rmse               = Vector3f(0.05, 0.05, 0.05);
const Vector3f maximum_standard_deviation_translation_rmse = Vector3f(0.25, 0.25, 0.25);
const Vector3f maximum_mean_rotation_rmse                  = Vector3f(3, 3, 3);
const Vector3f maximum_standard_deviation_rotation_rmse    = Vector3f(3, 3, 3);

int main(int argc_, char** argv_) {
  srrg2_core::messages_registerTypes();
  srrg2_core::point_cloud_registerTypes();
  srrg2_proslam::srrg2_proslam_tracking_registerTypes();
  Profiler::enable_logging = true;

  // ds load a laser slam assembly from configuration
  ConfigurableManager manager;
  manager.read(current_source_directory + "/../../configurations/tum.conf");
  MultiGraphSLAM3DPtr slammer = manager.getByName<MultiGraphSLAM3D>("slam");
  assert(slammer);

  // ds enable open loop benchmark by removing closure capabilities
  slammer->param_closure_validator.setValue(nullptr);
  slammer->param_loop_detector.setValue(nullptr);
  slammer->param_relocalizer.setValue(nullptr);

  // ds instantiate the benchmark utility
  SLAMBenchmarkSuiteSE3Ptr benchamin(new SLAMBenchmarkSuiteTUM());

  // ds load complete dataset and ground truth from disk
  benchamin->loadDataset("messages.json");
  benchamin->loadGroundTruth("gt.txt");

  // ds process all messages and feed benchamin with computed estimates
  // ds TODO clearly this does not account for PGO/BA which happens retroactively
  while (BaseSensorMessagePtr message = benchamin->getMessage()) {
    const double system_time_start_seconds = getTime();
    slammer->setRawData(message);
    slammer->compute();
    const double processing_duration_seconds = getTime() - system_time_start_seconds;
    benchamin->setPoseEstimate(
      slammer->robotInWorld(), message->timestamp.value(), processing_duration_seconds);
  }

  // ds save trajectory for external benchmark plot generation
  benchamin->writeTrajectoryToFile("trajectory.txt");

  // ds run benchmark evaluation
  benchamin->compute();

  // ds evaluate if target metrics have not been met
  if (benchamin->isRegression(maximum_mean_translation_rmse,
                              maximum_standard_deviation_translation_rmse,
                              maximum_mean_rotation_rmse,
                              maximum_standard_deviation_rotation_rmse)) {
    return -1;
  } else {
    return 0;
  }
}
