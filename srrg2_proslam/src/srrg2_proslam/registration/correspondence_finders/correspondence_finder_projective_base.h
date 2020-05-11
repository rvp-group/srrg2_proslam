#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/kd_tree.hpp>
#include <srrg_pcl/point_projector_types.h>

#include "correspondence_finder_descriptor_based_bruteforce.h"

namespace srrg2_proslam {

  using CorrespondenceCandidateMap =
    std::unordered_map<size_t /*fixed index*/, srrg2_core::CorrespondenceVector>;

  //! projective descriptor matcher base (with variable projection search radius)
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderProjectiveBase
    : public CorrespondenceFinderDescriptorBasedBruteforce<TransformType_,
                                                           FixedType_,
                                                           MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = CorrespondenceFinderProjectiveBase<TransformType_, FixedType_, MovingType_>;
    using TransformType     = TransformType_;
    using FixedCloudType    = FixedType_;
    using MovingCloudType   = MovingType_;
    using FixedPointType    = typename FixedCloudType::PointType;
    using MovingPointType   = typename MovingCloudType::PointType;
    using ProjectoryType    = srrg2_core::PointProjectorPinhole_<MovingCloudType>;
    using ProjectoryTypePtr = std::shared_ptr<ProjectoryType>;

    PARAM(srrg2_core::PropertyFloat,
          minimum_descriptor_distance,
          "minimum permitted descriptor distance for a match (initial)",
          25.0f,
          &_config_changed);
    PARAM(srrg2_core::PropertyFloat,
          descriptor_distance_step_size_pixels,
          "descriptor distance step size (increase/decrease)",
          5,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_search_radius_pixels,
          "maximum projective region search radius in pixels",
          100,
          &_config_changed);
    PARAM(srrg2_core::PropertyUnsignedInt,
          minimum_search_radius_pixels,
          "minimum projective region search radius in pixels",
          10,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          search_radius_step_size_pixels,
          "search radius step size (increase/decrease) in pixels",
          5,
          nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<ProjectoryType>,
          projector,
          "pinhole projector used for projective descriptor matching",
          ProjectoryTypePtr(new ProjectoryType()),
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          minimum_number_of_iterations,
          "minimum number of iterations to perform recomputes (forced)",
          10,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          maximum_estimate_change_norm_for_convergence,
          "maximum allowed transform estimate change threshold for assuming convergence",
          1e-5,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          number_of_solver_iterations_per_projection,
          "minimum number of solver iterations (on estimate) to await before reprojecting points",
          25,
          nullptr);

    virtual ~CorrespondenceFinderProjectiveBase() {
    }

    void compute() override;

    //! current correspondence search radius in the image plane (no config change triggered)
    void setSearchradiusPixels(const size_t& search_radius_pixels_) {
      _search_radius_pixels = search_radius_pixels_;
      _config_changed       = false;
    }
    const size_t& searchRadiusPixels() const {
      return _search_radius_pixels;
    }
    size_t searchRadiusPixelsSquared() const {
      return _search_radius_pixels * _search_radius_pixels;
    }

    //! current descriptor distance for matching (no config change triggered)
    void setDescriptorDistance(const float& descriptor_distance_) {
      _descriptor_distance = descriptor_distance_;
      _config_changed      = false;
    }

  protected:
    //! initializes the nearest neighbor database on fixed points (measurements)
    virtual void _initializeDatabase() = 0;

    //! finds nearest neighbor in the projection database
    virtual void _findNearestNeighbors(const MovingPointType& query_,
                                       const size_t& query_index_,
                                       CorrespondenceCandidateMap& neighbors_fixed_,
                                       CorrespondenceCandidateMap& neighbors_moving_) const = 0;

    //! adds correspondence candidate to maps indexed in fixed and moving
    void _addCorrespondenceCandidate(
      const int& index_fixed_,
      const int& index_moving_,
      const float& response_,
      CorrespondenceCandidateMap& correspondence_candidate_map_fixed_,
      CorrespondenceCandidateMap& correspondence_candidate_map_moving_) const;

    //! filters correspondences by best matching distance and ratio
    void _filterCorrespondences(
      CorrespondenceCandidateMap& correspondence_candidate_map_fixed_,
      CorrespondenceCandidateMap& correspondence_candidate_map_moving_,
      const float& maximum_descriptor_distance_,
      const float& maximum_distance_ratio_,
      srrg2_core::CorrespondenceVector& correspondence_candidates_filtered_) const;

    virtual void draw(const CorrespondenceCandidateMap& neighbors_fixed_,
                      const CorrespondenceCandidateMap& neighbors_moving_,
                      const srrg2_core::CorrespondenceVector& filtered_correspondences_) {
    }

  protected:
    //! configuration change
    bool _config_changed = true;

    //! current nearest neighbor search radius (dynamic)
    //! the value is initialized at the maximum and decreases as the tracking is stable
    size_t _search_radius_pixels = 0;

    //! current descriptor matching distance (dynamic)
    //! the value is initialized at the minimum and increases as the tracking is stable
    float _descriptor_distance = 0;

    //! previous camera pose estimate (used in auto termination criterion)
    TransformType _local_map_in_sensor_previous;

    //! boolean triggered if matching ratio gain is insufficient
    //! currently we require strict increasing matching ratios (no intermediate drops tolerated)
    bool _has_converged = false;

    //! current iteration count (incremented for subsequent calls as long as fixed and moving stay)
    size_t _current_iteration = 0;

    //! cached projection data (for repeated calls)
    MovingCloudType _points_in_image;
    std::vector<int> _indices_projected_to_moving;
  };

} // namespace srrg2_proslam
