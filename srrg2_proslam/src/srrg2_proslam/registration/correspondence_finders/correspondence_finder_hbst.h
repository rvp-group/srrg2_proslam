#pragma once
#include "srrg2_slam_interfaces/registration/correspondence_finder.h"
#include <srrg_hbst/types/binary_tree.hpp>
#include <srrg_pcl/point_types.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_proslam {
  template <typename LocalMapType_>
  class CorrespondenceFinderHBST_ : public srrg2_slam_interfaces::CorrespondenceFinderBase,
                                    public srrg2_core::Profiler {
    using LocalMapType = LocalMapType_;
    using EstimateType = typename LocalMapType::EstimateType;

    using ThisType        = CorrespondenceFinderHBST_<LocalMapType_>;
    using BaseType        = srrg2_slam_interfaces::CorrespondenceFinderBase;
    using LocalMapTypePtr = std::shared_ptr<LocalMapType>;

    //! supported descriptor types (if cast to these types fails, detection fails)
    using PointDescriptorType = srrg2_core::PointIntensityDescriptor_<EstimateType::Dim, float>;
    using PointDescriptorVectorType =
      srrg2_core::PointIntensityDescriptorVectorCloud<EstimateType::Dim, float>;
    using DescriptorPropertyType = srrg2_core::Property_<PointDescriptorVectorType*>;

    //! selected database type (currently set at compile time)
    using DatabaseType = srrg_hbst::BinaryTree256<uint64_t>;
    PARAM(srrg2_core::PropertyFloat,
          maximum_descriptor_distance,
          "maximum permitted descriptor distance for a match",
          25.0f,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_leaf_size,
          "maximum size of a leaf (i.e. number of descriptors) required before splitting",
          100,
          &_config_changed);
    PARAM(srrg2_core::PropertyFloat,
          maximum_partitioning,
          "maximum partitioning tolerance (i.e. 0.0 would be a perfect split)",
          0.1,
          &_config_changed);
    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_depth,
          "maximum permitted tree depth (maximum = descriptor bit size)",
          16,
          &_config_changed);
    PARAM(srrg2_core::PropertyFloat,
          maximum_distance_for_merge,
          "maximum descriptor distance permitted for internal merging (HBST)",
          0,
          &_config_changed);
    PARAM(srrg2_core::PropertyUnsignedInt,
          minimum_age_difference_to_candidates,
          "minimum required age difference between query and database reference",
          0,
          &_config_changed);

    virtual ~CorrespondenceFinderHBST_() {
      for (const DatabaseType::Matchable* matchable : _query_matchables) {
        delete matchable;
      }
      _query_matchables.clear();

      _database.clear(true);
    }

    /**
     * @brief sets the current local map (not used in autonomy)
     * @param[in] current local map
     */
    void setCurrentLocalMapAndPoints(LocalMapType* local_map_,
                                     const PointDescriptorVectorType* query_local_map_points_) {
      _current_local_map      = local_map_;
      _query_local_map_points = query_local_map_points_;
    }

    void compute();

  protected:
    /**
     * @brief getter for descriptors in local map
     * @param[in] local_map: target local map
     * @oaram[in] slice_name: choose the slice you want to check in given local map
     * @param[out] descriptors: set of descriptors for points in local map @ slice_name
     */
    void _retrieveDescriptorsFromLocalMap(LocalMapType* local_map_,
                                          const std::string& slice_point_cloud_name_,
                                          const PointDescriptorVectorType*& descriptors_);

    /**
     * @brief compute the correspondences indices for give matches
     * @param[in] matches_: vector of matches
     * @param[out] correspondences: vector of correspondences
     */
    void
    _computeCorrespondencesFromMatches(const DatabaseType::MatchVector& matches_,
                                       srrg2_core::CorrespondenceVector& correspondences_) const;

    bool _config_changed = true; /**< config change control flag*/

    LocalMapTypePtr _current_local_map =
      nullptr; /**<currently set local map (overwritten if picked automatically from SLAM system)*/

    DatabaseType::MatchableVector _query_matchables; /**< converted query matchables from previous
                                                      * query, buffered for delayed addition
                                                      */

    DatabaseType _database; /**< the HBST database containing descriptors arranged in a binary tree
    since the descriptor bit size has to be determined at compile we lock it for 256 bits ftm*/
    std::unordered_map<size_t, size_t>
      _graph_id_to_database_index; /**< mapping from local map graph identifiers to database indices
                                    * (monotonically increasing) this is required since we might
                                    * merge/relocalize local maps without adding them to the graph
                                    */
    std::vector<LocalMapType*> _local_maps_in_database; /**<local maps referenced in the database
                                                           (monotonically increasing)*/

    std::vector<size_t>
      _indices; /**<match vector buffer - only valid for last query!!! TODO eventually purge*/
    std::unordered_map<size_t, srrg2_core::CorrespondenceVector>
      _correspondences_per_reference; /**< correspondences stored per reference index*/

    const PointDescriptorVectorType* _query_local_map_points =
      nullptr; /**<instantiated fixed local map points*/
    const PointDescriptorVectorType* _reference_local_map_points =
      nullptr; /**<instantiated moving local map points*/
  };

} // namespace srrg2_proslam
