#include "correspondence_finder_projective_circle.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveCircle<TransformType_, FixedType_, MovingType_>::
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& correspondence_candidates_fixed_,
                          CorrespondenceCandidateMap& correspondence_candidates_moving_) const {
    assert(ThisType::param_projector.value());
    assert(ThisType::_database_fixed.size() == ThisType::_fixed->size());
    const Vector2f& image_coordinates_query = query_.coordinates().head(2);
    const int16_t row                       = std::round(image_coordinates_query(1));
    const int16_t col                       = std::round(image_coordinates_query(0));
    assert(row < 32767 && "bigger bookkeeping needed");
    assert(col < 32767 && "bigger bookkeeping needed");
    assert(row <= ThisType::param_projector->param_canvas_rows.value() && "outside image frame");
    assert(col <= ThisType::param_projector->param_canvas_cols.value() && "outside image frame");
    assert(row >= 0 && "outside image frame");
    assert(col >= 0 && "outside image frame");

    // ds compute row search range (same for all search patterns)
    const int16_t row_min = row - ThisType::_search_radius_pixels;
    const int16_t row_max = row + ThisType::_search_radius_pixels + 1;

    // ds cache
    const auto& query_descriptor_field = query_.template field<2>();
    const int32_t radius_squared =
      ThisType::_search_radius_pixels * ThisType::_search_radius_pixels;

    // ds evaluate neighorbs for minimum distance in appearance
    size_t index_fixed_best               = 0;
    size_t index_fixed_second_best        = 0;
    float descriptor_distance_best        = std::numeric_limits<float>::max();
    float descriptor_distance_second_best = std::numeric_limits<float>::max();

    // ds query database for all points in that region TODO loop over minor dimension for max speed
    auto iterator_lattice = ThisType::_database_fixed.begin();
    while (iterator_lattice != ThisType::_database_fixed.end() && iterator_lattice->row < row_min) {
      // ds stage 1: arrive at the interesting row indices
      ++iterator_lattice;
    }

    // ds scan all valid rows
    while (iterator_lattice != ThisType::_database_fixed.end() && iterator_lattice->row < row_max) {
      assert(iterator_lattice->row >= row_min);

      // ds compute permitted radial search range for this row
      const int32_t height = iterator_lattice->row - row;
      assert(radius_squared >= height * height);
      const int32_t width = std::sqrt(radius_squared - height * height) + 1;

      // ds stage 2: check all candidates that have column indices in range
      if (iterator_lattice->col > col - width && iterator_lattice->col < col + width) {
        assert(static_cast<size_t>(iterator_lattice->index) < ThisType::_fixed->size());
        // ds compute distances
        const float descriptor_distance =
          (*ThisType::_fixed)[iterator_lattice->index].template field<2>().distance(
            query_descriptor_field);

        // ds check thresholds
        if (descriptor_distance < descriptor_distance_best) {
          descriptor_distance_second_best = descriptor_distance_best;
          descriptor_distance_best        = descriptor_distance;
          index_fixed_second_best         = index_fixed_best;
          index_fixed_best                = iterator_lattice->index;
        } else if (descriptor_distance < descriptor_distance_second_best) {
          descriptor_distance_second_best = descriptor_distance;
          index_fixed_second_best         = iterator_lattice->index;
        }
      }
      ++iterator_lattice;
    }

    // ds check if we have found at least one candidate
    if (descriptor_distance_best < std::numeric_limits<float>::max()) {
      ThisType::_addCorrespondenceCandidate(index_fixed_best,
                                            query_index_,
                                            descriptor_distance_best,
                                            correspondence_candidates_fixed_,
                                            correspondence_candidates_moving_);

      // ds if there is a second best candidate
      if (descriptor_distance_second_best < std::numeric_limits<float>::max()) {
        ThisType::_addCorrespondenceCandidate(index_fixed_second_best,
                                              query_index_,
                                              descriptor_distance_second_best,
                                              correspondence_candidates_fixed_,
                                              correspondence_candidates_moving_);
      }
    }
  }

} // namespace srrg2_slam_interfaces
