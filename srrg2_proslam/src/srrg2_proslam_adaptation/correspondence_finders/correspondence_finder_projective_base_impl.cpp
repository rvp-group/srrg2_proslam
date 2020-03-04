#include "correspondence_finder_projective_base.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveBase<TransformType_, FixedType_, MovingType_>::
    _addCorrespondenceCandidate(
      const int& index_fixed_,
      const int& index_moving_,
      const float& response_,
      CorrespondenceCandidateMap& correspondence_candidate_map_fixed_,
      CorrespondenceCandidateMap& correspondence_candidate_map_moving_) const {
    Correspondence correspondence_best(index_fixed_, index_moving_, response_);

    // ds check if the fixed index has been already picked in the fixed indexed buffer
    auto iterator_correspondences = correspondence_candidate_map_fixed_.find(index_fixed_);
    if (iterator_correspondences != correspondence_candidate_map_fixed_.end()) {
      assert(!iterator_correspondences->second.empty());
      iterator_correspondences->second.push_back(correspondence_best);
    } else {
      // ds source index is free, register a new correspondence
      correspondence_candidate_map_fixed_.insert(
        std::make_pair(index_fixed_, CorrespondenceVector(1, correspondence_best)));
    }

    // ds insert the correspondence in the moving indexed buffer as well
    iterator_correspondences = correspondence_candidate_map_moving_.find(index_moving_);
    if (iterator_correspondences != correspondence_candidate_map_moving_.end()) {
      assert(!iterator_correspondences->second.empty());
      iterator_correspondences->second.push_back(correspondence_best);
    } else {
      // ds source index is free, register a new correspondence
      correspondence_candidate_map_moving_.insert(
        std::make_pair(index_moving_, CorrespondenceVector(1, correspondence_best)));
    }
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveBase<TransformType_, FixedType_, MovingType_>::
    _filterCorrespondences(CorrespondenceCandidateMap& correspondence_candidate_map_fixed_,
                           CorrespondenceCandidateMap& correspondence_candidate_map_moving_,
                           const float& maximum_descriptor_distance_,
                           const float& maximum_distance_ratio_,
                           CorrespondenceVector& correspondence_candidates_filtered_) const {
    correspondence_candidates_filtered_.clear();
    correspondence_candidates_filtered_.reserve(correspondence_candidate_map_fixed_.size());

    // ds compute correspondence candidates
    for (auto& candidate_map_element_fixed : correspondence_candidate_map_fixed_) {
      CorrespondenceVector& correspondence_candidates_fixed = candidate_map_element_fixed.second;

      // ds find best and second best candidate in appearance
      size_t index_best            = 0;
      float response_lowest        = std::numeric_limits<float>::max();
      float response_second_lowest = std::numeric_limits<float>::max();
      for (size_t i = 0; i < correspondence_candidates_fixed.size(); ++i) {
        assert(candidate_map_element_fixed.first ==
               static_cast<size_t>(correspondence_candidates_fixed[i].fixed_idx));
        const float current_response = correspondence_candidates_fixed[i].response;
        if (current_response < response_lowest) {
          response_second_lowest = response_lowest;
          response_lowest        = current_response;
          index_best             = i;
        } else if (current_response < response_second_lowest) {
          response_second_lowest = current_response;
        }
      }

      // ds check if matching distance metric holds
      if (response_lowest < maximum_descriptor_distance_ &&
          response_lowest / response_second_lowest < maximum_distance_ratio_) {
        Correspondence& correspondence_best = correspondence_candidates_fixed[index_best];
        const size_t index_moving_best      = correspondence_best.moving_idx;

        // ds retrieve best correspondence indexed in moving, element available by construction
        assert(correspondence_candidate_map_moving_.find(index_moving_best) !=
               correspondence_candidate_map_moving_.end());
        const CorrespondenceVector& correspondence_candidates_moving =
          correspondence_candidate_map_moving_.at(index_moving_best);
        assert(!correspondence_candidates_moving.empty());
        response_lowest = std::numeric_limits<float>::max();
        index_best      = 0;
        for (size_t i = 0; i < correspondence_candidates_moving.size(); ++i) {
          assert(index_moving_best ==
                 static_cast<size_t>(correspondence_candidates_moving[i].moving_idx));
          const float current_response = correspondence_candidates_moving[i].response;
          if (current_response < response_lowest) {
            response_lowest = current_response;
            index_best      = i;
          }
        }

        // ds if the mapping is bijective (i.e. same fixed for best in moving as for best in fixed)
        if (correspondence_candidates_moving[index_best].fixed_idx ==
            correspondence_best.fixed_idx) {
          // ds add the best candidate
          correspondence_candidates_filtered_.emplace_back(correspondence_best);
        }
      }
    }
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveBase<TransformType_, FixedType_, MovingType_>::compute() {
    ThisType::_preCompute();

    // ds if fixed or moving has changed we have to start a new optimization
    if (ThisType::_fixed_changed_flag || ThisType::_moving_changed_flag || _config_changed) {
      ThisType::_fixed_changed_flag  = false;
      ThisType::_moving_changed_flag = false;

      // ds if not set - initialize dynamic parameters (should occur only once per session)
      if ((_search_radius_pixels == 0 && _descriptor_distance == 0) || _config_changed) {
        _search_radius_pixels = param_maximum_search_radius_pixels.value();
        _descriptor_distance  = param_minimum_descriptor_distance.value();
        std::cerr << "CorrespondenceFinderProjective::compute|initialized search radius (px): "
                  << _search_radius_pixels;
        std::cerr << " descriptor distance: " << _descriptor_distance;
        std::cerr << " maximum distance ratio: "
                  << ThisType::param_maximum_distance_ratio_to_second_best.value() << std::endl;
      }

      // ds initialize convergence criterion
      _has_converged     = false;
      _current_iteration = 0;
      _estimate_moving_from_fixed_previous.setIdentity();

      // ds initialize NN database
      _initializeDatabase();

      // ds reconfigured
      _config_changed = false;
    }

    // ds if neither fixed nor moving changed and we have converged
    // ds we keep the last computation state of correspondences
    if (!ThisType::_fixed_changed_flag && !ThisType::_moving_changed_flag && _has_converged) {
      // ds correspondences are not touched
      ThisType::_postCompute();
      return;
    }
    PROFILE_TIME("CorrespondenceFinderProjective::compute");

    // ds we recompute all projections into the fixed frame
    assert(ThisType::_estimate_moving_from_fixed.matrix().norm() > 0);

    // ds we need to project the points - cache projector
    if (!param_projector.value()) {
      throw std::runtime_error("CorrespondenceFinderProjective::compute|ERROR: projector not set");
    }
    assert(param_projector->param_canvas_rows.value() > 0);
    assert(param_projector->param_canvas_cols.value() > 0);
    assert(param_projector->cameraMatrix().norm() > 0);
    assert(param_projector->cameraMatrix() != Matrix3f::Identity());

    // ds set current perspective fixed w.r.t. to moving (verify call!)
    // ds currently the tracker is providing the aligner with the inverse
    param_projector->setCameraPose(ThisType::_estimate_moving_from_fixed.inverse());

    // ds periodically obtain moving point projections in the fixed frame based on new estimate
    // ds always recompute projections for iterations 0 and 1 (initial guess and refined estimate)
    if (_current_iteration % param_number_of_solver_iterations_per_projection.value() == 0 ||
        _current_iteration == 1) {
      MovingCloudType points_in_camera; // ds not used
      param_projector->compute(
        *ThisType::_moving, points_in_camera, _points_in_image, _indices_projected_to_moving);
      if (_points_in_image.empty()) {
        std::cerr << FG_RED("CorrespondenceFinderProjective::compute|WARNING: all "
                            "projections failed")
                  << std::endl;
      }
    } else {
      // ds nothing new to compute - keep previous correspondences
      _estimate_moving_from_fixed_previous = ThisType::_estimate_moving_from_fixed;
      ++_current_iteration;
      ThisType::_postCompute();
      return;
    }

    // ds compute projection estimate change
    const float estimate_change_norm =
      geometry3d::t2tnq(param_projector->cameraPose() * _estimate_moving_from_fixed_previous)
        .norm();
    _estimate_moving_from_fixed_previous = ThisType::_estimate_moving_from_fixed;

    // ds correspondence candidate maps (indexed by fixed and moving for bijective matching)
    CorrespondenceCandidateMap correspondence_candidate_map_fixed;
    correspondence_candidate_map_fixed.reserve(ThisType::_fixed->size());
    CorrespondenceCandidateMap correspondence_candidate_map_moving;
    correspondence_candidate_map_moving.reserve(ThisType::_moving->size());

    // ds process the projection with the respective source index
    for (size_t index_projected = 0; index_projected < _points_in_image.size(); ++index_projected) {
      assert(_points_in_image[index_projected].status == Valid);

      // ds virtual dispatch
      _findNearestNeighbors(_points_in_image[index_projected],
                            _indices_projected_to_moving[index_projected],
                            correspondence_candidate_map_fixed,
                            correspondence_candidate_map_moving);
    }

    // ds filter correspondence candidates
    CorrespondenceVector correspondence_candidates_filtered;
    _filterCorrespondences(correspondence_candidate_map_fixed,
                           correspondence_candidate_map_moving,
                           _descriptor_distance,
                           ThisType::param_maximum_distance_ratio_to_second_best.value(),
                           correspondence_candidates_filtered);

    // ds compute matching success ratio to flag bad situations
    const float matching_ratio =
      static_cast<float>(correspondence_candidates_filtered.size()) / ThisType::_fixed->size();
    //    std::cerr << "estimate: \n" << ThisType::_estimate_moving_from_fixed.matrix() <<
    //    std::endl; std::cerr << "[" << _current_iteration; std::cerr << "] search radius: " <<
    //    _search_radius_pixels; std::cerr << " descriptor distance: " << _descriptor_distance;
    //    std::cerr << " matching ratio: " << matching_ratio << " ("
    //              << correspondence_candidates_filtered.size() << "/" << ThisType::_fixed->size()
    //              << ")";
    //    std::cerr << " # projections: " << _points_in_image.size() << "/" <<
    //    ThisType::_moving->size()
    //              << std::endl;

    // ds if we have an insufficient matching ratio
    if (matching_ratio < ThisType::param_minimum_matching_ratio.value()) {
      std::cerr << "CorrespondenceFinderProjective::compute|low matching ratio: " << matching_ratio
                << " (" << correspondence_candidates_filtered.size() << "/"
                << ThisType::_fixed->size()
                << ") target: " << ThisType::param_minimum_matching_ratio.value() << std::endl;

      // ds if we are not running at maximum tolerance yet
      if (_search_radius_pixels < param_maximum_search_radius_pixels.value() ||
          _descriptor_distance > param_minimum_descriptor_distance.value()) {
        // ds increase search radius
        _search_radius_pixels = ThisType::param_maximum_search_radius_pixels.value();

        // ds decrease descriptor matching distance tolerance
        _descriptor_distance = ThisType::param_minimum_descriptor_distance.value();

        // ds trigger an internal repeat (otherwise the external aligner might terminate us)
        // ds this situation MUST only occur when we have drastic motion changes
        // ds that are not well described by our current motion model
        std::cerr << FG_YELLOW("CorrespondenceFinderProjective|WARNING: bad initial guess - "
                               "triggering internal repeat with increased search radius")
                  << std::endl;

        // ds if the loss is fatal
        if (matching_ratio == 0) {
          std::cerr << FG_RED("CorrespondenceFinderProjective|WARNING: complete track loss - "
                              "fallback to identity motion guess")
                    << std::endl;

          // ds reset motion estimate for next iteration TODO hook solver estimate to this
          ThisType::_estimate_moving_from_fixed.setIdentity();
          _current_iteration = 0;
        } else {
          ++_current_iteration;
        }
        return ThisType::compute(); // ds recursion is terminated as soon as threshold are saturated
      }
    }

    // ds update correspondences
    assert(ThisType::_correspondences && "_correspondences not set");
    ThisType::_correspondences->swap(correspondence_candidates_filtered);

    // ds check for termination
    if (estimate_change_norm < param_maximum_estimate_change_norm_for_convergence.value() &&
        _current_iteration > param_minimum_number_of_iterations.value()) {
      // ds disable any further iterations for this fixed/moving combination
      _has_converged = true;

      // ds if we have a sufficiently high matching ratio
      if (matching_ratio > ThisType::param_minimum_matching_ratio.value()) {
        // ds reduce search radius for next iteration
        _search_radius_pixels =
          std::max(_search_radius_pixels - param_search_radius_step_size_pixels.value(),
                   param_minimum_search_radius_pixels.value());

        // ds increase descriptor matching distance tolerance
        _descriptor_distance =
          std::min(_descriptor_distance + param_descriptor_distance_step_size_pixels.value(),
                   ThisType::param_maximum_descriptor_distance.value());
      }
    }

    // ds update bookkeeping
    ++_current_iteration;
    ThisType::_postCompute();
  }

} // namespace srrg2_slam_interfaces
