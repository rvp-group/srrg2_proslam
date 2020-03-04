#include "tracker_slice_projective_depth.hpp"

namespace srrg2_proslam {

  void TrackerSliceProjectiveDepth::merge() {
    sanityCheck("MultiTrackerSlice_::merge()");
    assert(param_merger.value());

    // ds prepare merger
    param_merger->setScene(_scene_slice);
    param_merger->setMoving(&_adapted_slice);

    param_merger->setTransform(ThisType::_tracker_estimate * ThisType::_sensor_in_robot);

    // ds check if the merger supports correspondences (defined by its dynamic type)
    MergerCorrespondencePtrType merger_correspondence =
      std::dynamic_pointer_cast<MergerCorrespondenceType>(param_merger.value());
    if (!merger_correspondence) {
      // ds perform regular merge if no correspondences are required
      param_merger->compute();
      return;
    }

    // ds set global tracker pose estimate (global landmark refinement)
    merger_correspondence->setTrackerInWorld(
      //        ThisType::_robot_in_sensor *
      ThisType::_tracker_in_world * ThisType::_sensor_in_robot);

    // ds if loop closure data is available for correspondence based merging
    if (_have_loop_closure_correspondences) {
      // ds correspondence-based closure merger must be available
      if (!param_closure_merger.value()) {
        throw std::runtime_error("MultiTrackerSlice::merge|ERROR: correspondence-based merger for "
                                 "closure aligment not available");
      }

      // ds prepare merger and compute - moving, correspondences and transform have been set
      param_closure_merger->setScene(_scene_slice);
      param_closure_merger->compute();

      // ds free correspondences set in closure update
      _correspondences.clear();
      _have_loop_closure_correspondences = false;
      return;
    }

    // ds clipped scene (which carries correspondences) must be available for correspondence merger!
    if (!ThisType::_clipped_scene) {
      throw std::runtime_error(
        "MultiTrackerSlice::merge|ERROR: auxiliary data not available for correspondence import");
    }

    // ds grab correspondences used for fixed measurement alignment from scene object
    CorrespondenceVector global_correspondences;
    using PropertyCorrespondenceVector = Property_<CorrespondenceVector>;
    PropertyCorrespondenceVector* correspondence_property =
      ThisType::_clipped_scene->template property<PropertyCorrespondenceVector>(
        ThisType::param_measurement_slice_name.value() +
        CorrespondenceFinderBase::auxiliary_data_suffix_correspondences);
    if (correspondence_property) {
      const CorrespondenceVector& local_correspondences = correspondence_property->value();

      // ds set correspondences between fixed and scene as determined by aligner
      // ds NOTE that the correspondences have been computed between fixed and local scene
      // ds for the merger we have to flip the correspondences and map them to the global scene
      // ds TODO figure out a workaround for this overhead here
      global_correspondences.reserve(local_correspondences.size());
      const std::vector<int>& local_to_global(param_clipper->globalIndices());
      assert(local_to_global.size() <= _scene_slice->size());
      assert(local_to_global.size() >= local_correspondences.size());
      for (const Correspondence& correspondence : local_correspondences) {
        assert(static_cast<size_t>(correspondence.moving_idx) < local_to_global.size());
        global_correspondences.emplace_back(
          Correspondence(local_to_global[correspondence.moving_idx] /* map to global scene */,
                         correspondence.fixed_idx,
                         correspondence.response));
      }

      // ds set correspondences and merge
      merger_correspondence->setCorrespondences(&global_correspondences);
      merger_correspondence->compute();
    } else {
      // ds perform regular merge (we are in a new local map)
      merger_correspondence->compute();
    }
  }

  void TrackerSliceProjectiveDepth::clip() {
    sanityCheck("MultiTrackerSlice_::clip()");
    param_clipper->setGlobalScene(_scene_slice);
    param_clipper->setLocalScene(&_clipped_scene_slice);
    param_clipper->setAuxiliaryData(ThisType::_clipped_scene,
                                    ThisType::param_scene_slice_name.value());

    param_clipper->setTransform(ThisType::_sensor_in_robot * ThisType::_tracker_estimate *
                                ThisType::_sensor_in_robot);
    param_clipper->setSensorInRobot(ThisType::_sensor_in_robot);

    param_clipper->compute();
  }
} // namespace srrg2_proslam
