#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/kd_tree.hpp>
#include <srrg_pcl/point_projector_types.h>

#include "correspondence_finder_projective_base.h"

namespace srrg2_slam_interfaces {

  //! kd-tree based projective correspondence finder
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderProjectiveKDTree
    : public CorrespondenceFinderProjectiveBase<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = CorrespondenceFinderProjectiveKDTree<TransformType_, FixedType_, MovingType_>;
    using FixedPointType  = typename FixedType_::PointType;
    using MovingPointType = typename MovingType_::PointType;

    virtual ~CorrespondenceFinderProjectiveKDTree() {
    }

    PARAM(srrg2_core::PropertyUnsignedInt,
          minimum_number_of_points_per_cluster,
          "minimum number of points in the clusters",
          10,
          nullptr);

  protected:
    virtual void _initializeDatabase() override;

    virtual void
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& neighbors_fixed_,
                          CorrespondenceCandidateMap& neighbors_moving_) const override;

  protected:
    //! database for fixed measurements (only needs to be updated if fixed changed)
    std::shared_ptr<const srrg2_core::KDTree<float, 2>> _database_fixed = nullptr;
  };

  using CorrespondenceFinderProjectiveKDTree2D3D =
    CorrespondenceFinderProjectiveKDTree<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderProjectiveKDTree3D3D =
    CorrespondenceFinderProjectiveKDTree<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderProjectiveKDTree4D3D =
    CorrespondenceFinderProjectiveKDTree<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderProjectiveKDTree2D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveKDTree2D3D>;
  using CorrespondenceFinderProjectiveKDTree3D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveKDTree3D3D>;
  using CorrespondenceFinderProjectiveKDTree4D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveKDTree4D3D>;

} // namespace srrg2_slam_interfaces
