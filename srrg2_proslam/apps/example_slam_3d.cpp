#include <iostream>
#include <thread>

#include "srrg2_proslam_tracking/instances.h"
#include "srrg_config/configurable_manager.h"
#include "srrg_messages/instances.h"
#include "srrg_qgl_viewport/viewer_core_shared_qgl.h"
#include "srrg_slam_interfaces/instances.h"
#include "srrg_system_utils/parse_command_line.h"

using namespace srrg2_proslam;
using namespace srrg2_qgl_viewport;
using namespace srrg2_slam_interfaces;

// clang-format off
const char* banner[] = {"This program computes the perspective change and global structure for a sequence of images",
                        "The 3D points are anchored in the leftmost camera coordinate frame",
                        0};
// clang-format on

enum class SLAMType { ProjectiveDepth, StereoProjective, Invalid };

// ds processing function
void process(ViewerCanvasPtr canvas_,
             MessageSynchronizedSourcePtr synchronizer_,
             MultiGraphSLAM3DPtr slammer_);

// ds dummy configuration generation
void generateConfig(ConfigurableManager& manager_,
                    const std::string& filepath_,
                    const SLAMType& type_);

int main(int argc_, char** argv_) {
  messages_registerTypes();
  srrg2_proslam_tracking_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString config_file (
    &command_line_parser, "c",  "configuration",
                          "config file to read/write", "slam_3d.conf");
  ArgumentString slam_type_parameter(
    &command_line_parser, "tt", "slam-type",
                          "selected tracker type (stereo, rgbd, ..)", "stereo");
  // clang-format on
  command_line_parser.parse();
  ConfigurableManager manager;
  if (!config_file.isSet()) {
    if (slam_type_parameter.value() == "stereo") {
      generateConfig(manager, config_file.value(), SLAMType::StereoProjective);
    } else {
      generateConfig(manager, config_file.value(), SLAMType::ProjectiveDepth);
    }
    std::cerr << "ADJUST calibration data and rerun with -c" << std::endl;
    return -1;
  }

  // ds allocate tracker from configuration
  manager.read(config_file.value());
  MultiGraphSLAM3DPtr slammer = manager.getByName<MultiGraphSLAM3D>("slam");
  if (!slammer) {
    std::cerr << "ERROR: could not instantiate tracker from configuration file: "
              << config_file.value() << std::endl;
    return -1;
  }

  // ds set up dataset playback from configuration
  MessageSynchronizedSourcePtr synchronizer = manager.getByName<MessageSynchronizedSource>("sync");
  if (!synchronizer) {
    std::cerr << "ERROR: could not load dataset from configuration file: " << config_file.value()
              << std::endl;
    return -1;
  }

  // ds launch viewer and processing thread (points are not changed at this point)
  QApplication qapp(argc_, argv_);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer(argc_, argv_, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("example_tracker");
  std::thread processing_thread(process, canvas, synchronizer, slammer);

  // ds display viewer until termination signal is received
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

void process(ViewerCanvasPtr canvas_,
             MessageSynchronizedSourcePtr synchronizer_,
             MultiGraphSLAM3DPtr slammer_) {
  while (BaseSensorMessagePtr message = synchronizer_->getMessage()) {
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    if (!message_pack) {
      std::cerr << "WARNING: skipping message: " << message->name() << std::endl;
      continue;
    }

    // ds set measurements to module
    slammer_->setMeasurement(message_pack);

    // ds estimate current state
    slammer_->compute();
    //    std::cerr << slammer_->param_tracker->iterationStats();

    // ds obtain local map points
    MultiGraphSLAM3D::LocalMapType* local_map = slammer_->currentLocalMap();
    assert(local_map);
    assert(local_map->dynamic_properties.property("points"));
    Property_<PointIntensityDescriptor3fVectorCloud*>* local_map_points =
      local_map->dynamic_properties.property<Property_<PointIntensityDescriptor3fVectorCloud*>>(
        "points");
    assert(local_map_points);
    assert(local_map_points->value());

    // ds print statistics
    //    std::cerr << "motion estimate (relative): " << geometry3d::t2tnq(motion_guess).transpose()
    //              << std::endl;
    //    std::cerr << " final estimate (absolute): "
    //              << geometry3d::t2tnq(slammer_->robotPose()).transpose() << std::endl;
    std::cerr << "frame: " << slammer_->param_tracker->numFramesProcessed()
              << " | current local map size: " << local_map_points->value()->size()
              << " | # total local maps: " << slammer_->graph()->variables().size()
              << " | tracker status: " << slammer_->param_tracker->status() << std::endl;
    FactorGraphPtr graph = slammer_->graph();
    assert(graph);

    // ds visualize trajectory and current local map
    Vector3f* trajectory = new Vector3f[graph->variables().size()];
    size_t i             = 0;
    for (auto variable : graph->variables()) {
      const LocalMap3D* local_map = dynamic_cast<const LocalMap3D*>(variable.second);
      assert(local_map);
      trajectory[i] = local_map->estimate().translation();
      ++i;
    }
    canvas_->putLine(graph->variables().size(), trajectory);
    canvas_->pushColor();
    canvas_->setColor(Vector3f(0, 0, 0));
    canvas_->multMatrix(
      (slammer_->robotPose() * slammer_->robotPoseInCurrentLocalMap().inverse()).matrix());
    canvas_->putPoints(*local_map_points->value());
    canvas_->popMatrix();
    canvas_->popAttribute();
    canvas_->flush();
    delete[] trajectory;

    // ds terminate prematurely if viewer is closed
    if (!ViewerCoreSharedQGL::isRunning()) {
      break;
    }
  }
}

void generateConfig(ConfigurableManager& manager_,
                    const std::string& filepath_,
                    const SLAMType& type_) {
  MultiGraphSLAM3DPtr slammer = manager_.create<MultiGraphSLAM3D>("slammer");
  MultiAligner3DQRPtr aligner = manager_.create<MultiAligner3DQR>("aligner");
  MultiTracker3DPtr tracker   = manager_.create<MultiTracker3D>("tracker");
  slammer->param_tracker.setValue(tracker);
  tracker->param_aligner.setValue(aligner);
  switch (type_) {
    case SLAMType::ProjectiveDepth: {
      std::cerr << "generateConfig|generating for projective depth tracker" << std::endl;
      AlignerSliceProjectiveDepthPtr aligner_slice =
        manager_.create<AlignerSliceProjectiveDepth>("aligner_slice_projective_depth");
      CorrespondenceFinderProjectiveKDTree3D3DPtr finder =
        aligner_slice->param_finder.value<CorrespondenceFinderProjectiveKDTree3D3D>();
      aligner->param_slice_processors.pushBack(aligner_slice);
      TrackerSliceProjectiveDepthPtr tracker_slice =
        manager_.create<TrackerSliceProjectiveDepth>("tracker_slice_projective_depth");
      MeasurementAdaptorMonocularDepthPtr adaptor =
        manager_.create<MeasurementAdaptorMonocularDepth>("adaptor_projective_depth");
      MergerCorrespondenceProjectiveDepth3DPtr merger =
        manager_.create<MergerCorrespondenceProjectiveDepth3D>("merger_projective_depth");
      SceneClipperProjective3DPtr clipper =
        manager_.create<SceneClipperProjective3D>("clipper_projective_depth");
      clipper->param_projector.setValue(finder->param_projector.value());
      tracker_slice->param_adaptor.setValue(adaptor);
      tracker_slice->param_clipper.setValue(clipper);
      tracker_slice->param_merger.setValue(merger);
      tracker->param_slice_processors.pushBack(tracker_slice);
      break;
    }
    case SLAMType::StereoProjective: {
      std::cerr << "generateConfig|generating for stereo tracker" << std::endl;
      AlignerSliceStereoProjectivePtr aligner_slice =
        manager_.create<AlignerSliceStereoProjective>("aligner_slice_stereo_projective");
      CorrespondenceFinderProjectiveKDTree4D3DPtr finder =
        aligner_slice->param_finder.value<CorrespondenceFinderProjectiveKDTree4D3D>();
      aligner->param_slice_processors.pushBack(aligner_slice);
      TrackerSliceStereoProjectivePtr tracker_slice =
        manager_.create<TrackerSliceStereoProjective>("tracker_slice_stereo_projective");
      MeasurementAdaptorStereoProjectivePtr adaptor =
        manager_.create<MeasurementAdaptorStereoProjective>("adaptor_stereo_projective");
      MergerRigidStereoTriangulationPtr merger =
        manager_.create<MergerRigidStereoTriangulation>("merger_stereo_projective");
      SceneClipperProjective3DPtr clipper =
        manager_.create<SceneClipperProjective3D>("clipper_stereo_projective");
      clipper->param_projector.setValue(finder->param_projector.value());
      tracker_slice->param_adaptor.setValue(adaptor);
      tracker_slice->param_clipper.setValue(clipper);
      tracker_slice->param_merger.setValue(merger);
      tracker->param_slice_processors.pushBack(tracker_slice);
      break;
    }
    default:;
  }

  // ds allocate motion model for tracker (and aligner)
  MultiTrackerSliceEstimationBuffer3DPtr tracker_slice_pose_history(
    new MultiTrackerSliceEstimationBuffer3D());
  tracker_slice_pose_history->param_measurement_slice_name.setValue("trajectory_chunk");
  tracker_slice_pose_history->param_scene_slice_name.setValue("trajectory_chunk");
  tracker_slice_pose_history->param_adaptor.setValue(
    MeasurementAdaptorTrackerEstimate3DPtr(new MeasurementAdaptorTrackerEstimate3D()));
  tracker->param_slice_processors.pushBack(tracker_slice_pose_history);
  MultiAlignerSliceMotionModel3DPtr aligner_slice_motion_model(
    new MultiAlignerSliceMotionModel3D());
  aligner_slice_motion_model->param_fixed_slice_name.setValue("trajectory_chunk");
  aligner_slice_motion_model->param_moving_slice_name.setValue("trajectory_chunk");
  aligner_slice_motion_model->param_motion_model.setValue(
    MotionModelConstantVelocity3DPtr(new MotionModelConstantVelocity3D()));
  aligner->param_slice_processors.pushBack(aligner_slice_motion_model);

  // ds allocate loop closing facilities (identical for stereo and rgbd)
  MultiLoopDetectorHBST3DPtr loop_detector =
    manager_.create<MultiLoopDetectorHBST3D>("loop_detector");

  // ds configure loop closing aligner and slices (3D-3D)
  MultiAligner3DQRPtr loop_aligner = manager_.create<MultiAligner3DQR>("loop_aligner");
  AlignerSlice3DPtr aligner_slice  = manager_.create<AlignerSlice3D>("aligner_slice_3d");
  loop_aligner->param_slice_processors.pushBack(aligner_slice);
  loop_detector->param_relocalize_aligner.setValue(loop_aligner);
  slammer->param_loop_detector.setValue(loop_detector);

  // ds specify viewpoint dependent local map splitting criterion
  slammer->param_splitting_criterion.setValue(
    manager_.create<LocalMapSplittingCriterionVisibility3D>("splitting_criterion"));

  // ds store configuration on disk
  manager_.write(filepath_);
  std::cerr << "generated default configuration: " << filepath_ << std::endl;
}
