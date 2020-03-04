#include "correspondence_finder_projective_square.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveSquare<TransformType_, FixedType_, MovingType_>::
    _initializeDatabase() {
    assert(ThisType::_fixed);
    PROFILE_TIME("CorrespondenceFinderProjectiveSquare::initializeDatabase");

    // ds or if fixed has changed we have to build the database
    const size_t& number_of_measurements(ThisType::_fixed->size());
    _database_fixed.clear();
    _database_fixed.reserve(number_of_measurements);
    for (size_t i = 0; i < number_of_measurements; ++i) {
      const Vector2f& coordinates((*ThisType::_fixed)[i].coordinates().head(2));
      assert(coordinates(0) >= 0 && "outside image frame");
      assert(coordinates(1) >= 0 && "outside image frame");
      assert(coordinates(0) < 32767 && "bigger bookkeeping needed");
      assert(coordinates(1) < 32767 && "bigger bookkeeping needed");
      assert(i < 32767 && "bigger bookkeeping needed");
      _database_fixed.emplace_back(Element(coordinates(1), coordinates(0), i));
    }

    // ds sort database by ascending row indices
    std::sort(_database_fixed.begin(),
              _database_fixed.end(),
              [](const Element& a_, const Element& b_) { return a_.row < b_.row; });
    assert(_database_fixed.size() == number_of_measurements);
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void CorrespondenceFinderProjectiveSquare<TransformType_, FixedType_, MovingType_>::
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& correspondence_candidates_fixed_,
                          CorrespondenceCandidateMap& correspondence_candidates_moving_) const {
    assert(ThisType::param_projector.value());
    assert(_database_fixed.size() == ThisType::_fixed->size());
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

    // ds compute valid column search range
    const int16_t col_min = col - ThisType::_search_radius_pixels - 1;
    const int16_t col_max = col + ThisType::_search_radius_pixels + 1;

    // ds cache
    const auto& query_descriptor_field = query_.template field<2>();

    // ds evaluate neighorbs for minimum distance in appearance
    size_t index_fixed_best               = 0;
    size_t index_fixed_second_best        = 0;
    float descriptor_distance_best        = std::numeric_limits<float>::max();
    float descriptor_distance_second_best = std::numeric_limits<float>::max();

    // ds query database for all points in that region TODO loop over minor dimension for max speed
    auto iterator_lattice = _database_fixed.begin();
    while (iterator_lattice != _database_fixed.end() && iterator_lattice->row < row_min) {
      // ds stage 1: arrive at the interesting row indices
      ++iterator_lattice;
    }

    // ds scan all valid rows
    while (iterator_lattice != _database_fixed.end() && iterator_lattice->row < row_max) {
      assert(iterator_lattice->row >= row_min);

      // ds stage 2: check all candidates that have column indices in range
      if (iterator_lattice->col > col_min && iterator_lattice->col < col_max) {
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
