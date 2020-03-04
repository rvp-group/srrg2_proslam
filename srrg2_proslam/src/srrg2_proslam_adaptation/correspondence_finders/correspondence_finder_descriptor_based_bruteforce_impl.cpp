#include "correspondence_finder_descriptor_based_bruteforce.h"

namespace srrg2_slam_interfaces {

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>::
    compute() {
    _preCompute();

    // ds if neither fixed nor moving changed we keep the last computation state
    // ds (independent of estimate)
    if (!ThisType::_fixed_changed_flag && !ThisType::_moving_changed_flag) {
      return;
    }
    PROFILE_TIME("CorrespondenceFinderDescriptorBasedBruteforce::compute");

    // ds cache and prepare bookkeeping
    const size_t number_of_descriptors_fixed  = ThisType::_fixed->size();
    const size_t number_of_descriptors_moving = ThisType::_moving->size();
    const float maximum_descriptor_distance   = param_maximum_descriptor_distance.value();
    std::vector<Correspondence> correspondence_candidates;
    correspondence_candidates.reserve(number_of_descriptors_fixed * number_of_descriptors_moving);

    // ds bookkeeping for Lowe's check
    std::unordered_map<int, std::vector<float>> best_distances_for_fixed;
    std::unordered_map<int, std::vector<float>> best_distances_for_moving;
    best_distances_for_fixed.reserve(number_of_descriptors_fixed);
    best_distances_for_moving.reserve(number_of_descriptors_moving);

    // ds perform brute-force search with bijective mapping check
    for (size_t index_fixed = 0; index_fixed < number_of_descriptors_fixed; ++index_fixed) {
      assert((*ThisType::_fixed)[index_fixed].status == POINT_STATUS::Valid);
      const auto& descriptor_field = (*ThisType::_fixed)[index_fixed].template field<2>();
      assert(descriptor_field.value.rows == 1);
      assert(descriptor_field.value.cols > 1);
      auto iterator_fixed =
        best_distances_for_fixed.insert(std::make_pair(index_fixed, std::vector<float>(0))).first;
      iterator_fixed->second.reserve(number_of_descriptors_moving);

      // ds evaluate all moving descriptors
      for (size_t index_moving = 0; index_moving < number_of_descriptors_moving; ++index_moving) {
        assert((*ThisType::_moving)[index_moving].status == POINT_STATUS::Valid);
        assert((*ThisType::_moving)[index_moving].template field<2>().value.rows == 1);
        assert((*ThisType::_moving)[index_moving].template field<2>().value.cols > 1);

        // ds compute descriptor distance
        const float descriptor_distance =
          descriptor_field.distance((*ThisType::_moving)[index_moving].template field<2>());
        assert(descriptor_distance >= 0);

        // ds if the match is within thresholds
        if (descriptor_distance < maximum_descriptor_distance) {
          correspondence_candidates.emplace_back(
            Correspondence(index_fixed, index_moving, descriptor_distance));
          iterator_fixed->second.emplace_back(descriptor_distance);

          // ds check if moving bookkeeping is already existing (from a previous fixed round)
          auto iterator_moving = best_distances_for_moving.find(index_moving);
          if (iterator_moving != best_distances_for_moving.end()) {
            iterator_moving->second.emplace_back(descriptor_distance);
          } else {
            iterator_moving =
              best_distances_for_moving
                .insert(std::make_pair(index_moving, std::vector<float>(1, descriptor_distance)))
                .first;
            iterator_moving->second.reserve(number_of_descriptors_fixed);
          }
        }
      }

      // ds sort fixed distances in ascending order (completed at this point)
      std::sort(iterator_fixed->second.begin(), iterator_fixed->second.end());
    }

    // ds handle trivial cases
    ThisType::_correspondences->clear();
    if (correspondence_candidates.empty()) {
      _postCompute();
      return;
    } else if (correspondence_candidates.size() == 1) {
      ThisType::_correspondences->reserve(1);
      ThisType::_correspondences->emplace_back(correspondence_candidates.back());
      _postCompute();
      return;
    }

    // ds sort correspondence candidate vector in ascending descriptor distance
    std::sort(
      correspondence_candidates.begin(),
      correspondence_candidates.end(),
      [](const Correspondence& a_, const Correspondence& b_) { return a_.response < b_.response; });

    // ds sort moving candidates for Lowe's
    for (auto& distances : best_distances_for_moving) {
      std::sort(distances.second.begin(), distances.second.end());
    }

    // ds assemble correspondences from candidates
    std::unordered_set<int> registered_indices_moving;
    std::unordered_set<int> registered_indices_fixed;

    // ds prepare correspondence sliding window pool
    CorrespondenceVector correspondence_pool(1, correspondence_candidates.front());

    // ds skip first candidate (already in pool)
    ThisType::_correspondences->reserve(number_of_descriptors_fixed);
    for (size_t i = 1; i < correspondence_candidates.size(); ++i) {
      const Correspondence& correspondence_candidate = correspondence_candidates[i];

      // ds if the correspondence can still be registered
      if (registered_indices_fixed.count(correspondence_candidate.fixed_idx) == 0 &&
          registered_indices_moving.count(correspondence_candidate.moving_idx) == 0) {
        // ds check if we are still in the same distance pool
        if (correspondence_candidate.response == correspondence_pool.back().response) {
          correspondence_pool.push_back(correspondence_candidate);
        } else {
          // ds process the pool
          assert(correspondence_candidate.response > correspondence_pool.back().response);
          _processCorrespondencePool(correspondence_pool,
                                     best_distances_for_fixed,
                                     best_distances_for_moving,
                                     registered_indices_fixed,
                                     registered_indices_moving);

          // ds pool has been processed
          correspondence_pool.clear();

          // ds check if we still can add the candidate to the pool
          if (registered_indices_fixed.count(correspondence_candidate.fixed_idx) == 0 &&
              registered_indices_moving.count(correspondence_candidate.moving_idx) == 0) {
            correspondence_pool.push_back(correspondence_candidate);
          }
        }
      }

      // ds if either set of indices is completed we can terminate prematurely
      if (registered_indices_fixed.size() == number_of_descriptors_fixed ||
          registered_indices_moving.size() == number_of_descriptors_moving) {
        break;
      }
    }

    // ds check if we still have correspondences in the pool
    if (!correspondence_pool.empty()) {
      // ds process the remaining pool
      _processCorrespondencePool(correspondence_pool,
                                 best_distances_for_fixed,
                                 best_distances_for_moving,
                                 registered_indices_fixed,
                                 registered_indices_moving);
    }

    _postCompute();
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  bool CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>::
    checkLowesRatio(const float& descriptor_distance_best_,
                    const float& descriptor_distance_other_,
                    const float& maximum_ratio_) const {
    assert(descriptor_distance_best_ <= descriptor_distance_other_);

    // ds trivial case (also catches 0/0)
    if (descriptor_distance_best_ == descriptor_distance_other_) {
      return false;
    } else {
      // ds compute ratio (dodging double zero cases)
      const float ratio_best_to_second_best =
        descriptor_distance_best_ / descriptor_distance_other_;
      if (ratio_best_to_second_best < maximum_ratio_) {
        return true;
      } else {
        return false;
      }
    }
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  bool CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>::
    checkLowesRatio(const float& descriptor_distance_best_,
                    const std::vector<float>& descriptor_distances_other_,
                    const float& maximum_ratio_) const {
    assert(!descriptor_distances_other_.empty());
    if (descriptor_distances_other_.size() == 1) {
      assert(descriptor_distances_other_.back() == descriptor_distance_best_);
      return true;
    }

    float descriptor_distance_second_best = descriptor_distance_best_;
    for (const float& distance : descriptor_distances_other_) {
      if (distance > descriptor_distance_best_) {
        descriptor_distance_second_best = distance;
        break;
      }
    }
    return checkLowesRatio(
      descriptor_distance_best_, descriptor_distance_second_best, maximum_ratio_);
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>::
    _preCompute() {
    if (!ThisType::_fixed) {
      throw std::runtime_error("CorrespondenceFinderDescriptorBased::compute|ERROR: fixed not set");
    }
    if (!ThisType::_moving) {
      throw std::runtime_error(
        "CorrespondenceFinderDescriptorBased::compute|ERROR: moving not set");
    }
    if (!ThisType::_correspondences) {
      throw std::runtime_error(
        "CorrespondenceFinderDescriptorBased::compute|ERROR: correspondences not set");
    }

    // ds inform in case of low/no candidates
    if (ThisType::_fixed->empty()) {
      std::cerr << FG_YELLOW(
                     "CorrespondenceFinderDescriptorBased::compute|WARNING: no points in fixed")
                << std::endl;
    }
    if (ThisType::_moving->empty()) {
      std::cerr << FG_YELLOW(
                     "CorrespondenceFinderDescriptorBased::compute|WARNING: no points in moving")
                << std::endl;
    }
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>::
    _postCompute() {
    // ds do not trigger a new compute for the same input query
    ThisType::_fixed_changed_flag    = false;
    ThisType::_moving_changed_flag   = false;
    ThisType::_estimate_changed_flag = false;
    assert(ThisType::_correspondences);
    if (ThisType::_correspondences->empty()) {
      std::cerr
        << FG_YELLOW(
             "CorrespondenceFinderDescriptorBased::compute|WARNING: no correspondences found")
        << std::endl;
    }
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>::
    _processCorrespondencePool(
      CorrespondenceVector& correspondence_pool_,
      std::unordered_map<int, std::vector<float>>& best_distances_for_fixed_,
      std::unordered_map<int, std::vector<float>>& best_distances_for_moving_,
      std::unordered_set<int>& registered_indices_fixed_,
      std::unordered_set<int>& registered_indices_moving_) {
    const float maximum_distance_ratio_to_second_best =
      param_maximum_distance_ratio_to_second_best.value();

    // ds we have to evaluate the whole pool for crossed fixed/moving matches
    for (size_t j = 0; j < correspondence_pool_.size(); ++j) {
      bool is_unique_in_pool = true;
      for (size_t k = 0; k < correspondence_pool_.size(); ++k) {
        assert(correspondence_pool_[j].response == correspondence_pool_[k].response);
        if (j != k) {
          // ds if there is a conflicting mapping (with identical distance)
          if (correspondence_pool_[j].fixed_idx == correspondence_pool_[k].fixed_idx ||
              correspondence_pool_[j].moving_idx == correspondence_pool_[k].moving_idx) {
            is_unique_in_pool = false;
          }
        }
      }

      // ds if no conflict was detected for this matching distance
      if (is_unique_in_pool) {
        const Correspondence& correspondence = correspondence_pool_[j];
        auto iterator_fixed  = best_distances_for_fixed_.find(correspondence.fixed_idx);
        auto iterator_moving = best_distances_for_moving_.find(correspondence.moving_idx);
        assert(iterator_fixed != best_distances_for_fixed_.end());
        assert(iterator_moving != best_distances_for_moving_.end());
        assert(!iterator_fixed->second.empty());
        assert(!iterator_moving->second.empty());

        // ds perform Lowe's ratio check on competing correspondences for moving and fixed
        if (checkLowesRatio(correspondence.response,
                            iterator_fixed->second,
                            maximum_distance_ratio_to_second_best) &&
            checkLowesRatio(correspondence.response,
                            iterator_moving->second,
                            maximum_distance_ratio_to_second_best)) {
          // ds register the cross-checked correspondence
          ThisType::_correspondences->emplace_back(correspondence);
          registered_indices_fixed_.insert(correspondence.fixed_idx);
          registered_indices_moving_.insert(correspondence.moving_idx);
        }
      }
    }
  }

} // namespace srrg2_slam_interfaces
