#include "correspondence_finder_hbst.h"

namespace srrg2_proslam {
  using namespace srrg2_core;
  template <typename LocalMapType_>
  inline void CorrespondenceFinderHBST_<LocalMapType_>::compute() {
    _indices.clear();
    _correspondences_per_reference.clear();
    if (!_current_local_map || !_query_local_map_points) {
      throw std::runtime_error("CorrespondenceFinderHBST_::compute|ERROR: no local map set");
    }

    if (_query_local_map_points->empty()) {
      std::cerr << "MultiLoopDetectorHBST::compute|WARNING: query descriptor vector is empty"
                << std::endl;
      _current_local_map = nullptr;
      return;
    }

    // ds if we have to update the database configuration
    // ds this should be done only at startup as it might corrupt the database!
    if (_config_changed) {
#ifdef SRRG_MERGE_DESCRIPTORS
      DatabaseType::maximum_distance_for_merge = param_maximum_distance_for_merge.value();
#endif
      DatabaseType::Node::maximum_depth        = param_maximum_depth.value();
      DatabaseType::Node::maximum_leaf_size    = param_maximum_leaf_size.value();
      DatabaseType::Node::maximum_partitioning = param_maximum_partitioning.value();
      _config_changed                          = false;
    }

    // ds check if we have unadded matchables
    if (!_query_matchables.empty()) {
      // ds we have to free the matchables from the previous call
      for (const DatabaseType::Matchable* matchable : _query_matchables) {
        delete matchable;
      }
      _query_matchables.clear();
    }

    // ds cache
    const size_t number_of_query_descriptors = _query_local_map_points->size();
    const PointDescriptorVectorType& points_fixed(*_query_local_map_points);
    _query_matchables.reserve(number_of_query_descriptors);
    assert(_current_local_map->graphId() >= 0);

    // ds determine query index - for a new local map it corresponds to the database size
    uint64_t index_query = _database.size();

    // ds check if we already registered this local map in our database (by graph identifier)
    const auto iterator = _graph_id_to_database_index.find(_current_local_map->graphId());
    if (iterator != _graph_id_to_database_index.end()) {
      // ds pick original index for matching
      index_query = iterator->second;
    }

    // ds convert the descriptors to HBST matchables, linking by increasing point index
    for (uint64_t index_descriptor = 0; index_descriptor < number_of_query_descriptors;
         ++index_descriptor) {
      if (points_fixed[index_descriptor].status == POINT_STATUS::Valid) {
        _query_matchables.emplace_back(new DatabaseType::Matchable(
          index_descriptor, points_fixed[index_descriptor].descriptor(), index_query));
      }
    }

    // ds query the database for matches - match addition has to be triggered externally
    DatabaseType::MatchVectorMap matches_per_reference;
    _database.match(_query_matchables,
                    matches_per_reference,
                    ThisType::param_maximum_descriptor_distance.value());

    // ds filter match vector according to closure constraints
    _indices.reserve(matches_per_reference.size());
    _correspondences_per_reference.reserve(matches_per_reference.size());
    for (const auto& entry : matches_per_reference) {
      // ds check if age difference between the query and reference candidate is sufficiently high
      if (std::fabs(index_query - entry.first) >
          param_minimum_age_difference_to_candidates.value()) {
        const DatabaseType::MatchVector& matches = entry.second;
        const size_t& number_of_matches          = matches.size();
        if (number_of_matches > ThisType::param_relocalize_min_inliers.value()) {
          const size_t index_reference = entry.first;

          // ds register filtered candidates for optional external manipulation
          CorrespondenceVector correspondences;
          _computeCorrespondencesFromMatches(matches, correspondences);
          _correspondences_per_reference.insert(
            std::make_pair(index_reference, std::move(correspondences)));
          _indices.emplace_back(index_reference);
        }
      }
    }
  }

  template <typename LocalMapType_>
  inline void CorrespondenceFinderHBST_<LocalMapType_>::_computeCorrespondencesFromMatches(
    const DatabaseType::MatchVector& matches_,
    srrg2_core::CorrespondenceVector& correspondences_) const {
    correspondences_.clear();

    // ds convert matches into correspondence candidates by best distance TODO add Lowe check
    std::unordered_map<size_t /*reference*/, Correspondence> candidates;
    candidates.reserve(matches_.size());
    for (const DatabaseType::Match& match : matches_) {
      // ds only pick non-ambiguous matches
      if (match.object_references.size() == 1) {
        const size_t& index_descriptor_reference = match.object_references[0];
        auto iterator                            = candidates.find(index_descriptor_reference);
        if (iterator != candidates.end()) {
          if (match.distance < iterator->second.response) {
            iterator->second.response   = match.distance;
            iterator->second.fixed_idx  = match.object_query;
            iterator->second.moving_idx = index_descriptor_reference;
          }
        } else {
          candidates.insert(std::make_pair(
            index_descriptor_reference,
            Correspondence(match.object_query, index_descriptor_reference, match.distance)));
        }
      }
    }

    // ds register filtered candidates
    correspondences_.reserve(matches_.size());
    for (auto& candidate : candidates) {
      correspondences_.emplace_back(std::move(candidate.second));
    }
  }

} // namespace srrg2_proslam
