#include "correspondence_finder_projective_kdtree.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveKDTree<TransformType_, FixedType_, MovingType_>::
    _initializeDatabase() {
    assert(ThisType::_fixed);
    PROFILE_TIME("CorrespondenceFinderDescriptorBasedProjectiveKDTree::initializeDatabase");

    // ds TODO add maximum leaf range parameter to KDTree to avoid redundant rebuilds
    // ds build a KD tree for the fixed points (based on keypoint coordinates)
    KDTree<float, 2>::VectorTDVector fixed_measurements;
    fixed_measurements.reserve(ThisType::_fixed->size());
    for (const FixedPointType& point_measured : *ThisType::_fixed) {
      fixed_measurements.emplace_back(point_measured.template value<0>().head(2));
    }

    // ds partition measurements in a 2d tree
    _database_fixed = std::shared_ptr<const KDTree<float, 2>>(
      new const KDTree<float, 2>(fixed_measurements,
                                 ThisType::_search_radius_pixels,
                                 param_minimum_number_of_points_per_cluster.value()));
    assert(_database_fixed->numPoints() == ThisType::_fixed->size());
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveKDTree<TransformType_, FixedType_, MovingType_>::
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& correspondence_candidates_fixed_,
                          CorrespondenceCandidateMap& correspondence_candidates_moving_) const {
    assert(_database_fixed);
    assert(_database_fixed->numPoints() == ThisType::_fixed->size());
    assert(_database_fixed->numNodes() > 0);

    // ds caching
    const float maximum_distance_squared =
      ThisType::_search_radius_pixels * ThisType::_search_radius_pixels;
    const float& maximum_descriptor_distance = ThisType::param_maximum_descriptor_distance.value();
    const Vector2f& coordinates_moving       = query_.template value<0>().head(2);

    // ds query the database for the nearest neighbors - minimum in geometry
    KDTree<float, 2>::VectorTDVector candidates_coordinates_fixed;
    std::vector<int> candidates_indices;
    _database_fixed->findNeighbors(candidates_coordinates_fixed,
                                   candidates_indices,
                                   coordinates_moving,
                                   maximum_distance_squared);

    // ds evaluate neighorbs for minimum distance in appearance
    size_t index_fixed_best               = 0;
    float descriptor_distance_best        = maximum_descriptor_distance;
    float descriptor_distance_second_best = std::numeric_limits<float>::max();
    for (const int& index_fixed : candidates_indices) {
      // ds compute descriptor distance
      const float descriptor_distance =
        (*ThisType::_fixed)[index_fixed].template field<2>().distance(query_.template field<2>());

      // ds check thresholds
      if (descriptor_distance < descriptor_distance_best) {
        descriptor_distance_second_best = descriptor_distance_best;
        descriptor_distance_best        = descriptor_distance;
        index_fixed_best                = index_fixed;
      } else if (descriptor_distance < descriptor_distance_second_best) {
        descriptor_distance_second_best = descriptor_distance;
      }
    }

    // ds check if the distance qualifies for a match (Lowes)
    if (descriptor_distance_best < maximum_descriptor_distance) {
      ThisType::_addCorrespondenceCandidate(index_fixed_best,
                                            query_index_,
                                            descriptor_distance_best,
                                            correspondence_candidates_fixed_,
                                            correspondence_candidates_moving_);
    }
  }

} // namespace srrg2_slam_interfaces
