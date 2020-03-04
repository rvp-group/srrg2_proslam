#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/kd_tree.hpp>
#include <srrg_pcl/point_projector_types.h>

#include "correspondence_finder_projective_base.h"

namespace srrg2_slam_interfaces {

  //! local brute-force matcher based projective correspondence finder
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderProjectiveSquare
    : public CorrespondenceFinderProjectiveBase<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = CorrespondenceFinderProjectiveSquare<TransformType_, FixedType_, MovingType_>;
    using FixedPointType  = typename FixedType_::PointType;
    using MovingPointType = typename MovingType_::PointType;

    virtual ~CorrespondenceFinderProjectiveSquare() {
    }

  protected:
    virtual void _initializeDatabase() override;

    virtual void
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& neighbors_fixed_,
                          CorrespondenceCandidateMap& neighbors_moving_) const override;

  protected:
    //! database for fixed measurements (only needs to be updated if fixed changed)
    //! this is basically a 2d feature lattice grid with equal size as the image
    //! shorts cover [−32,767, +32,767] which should be enough for image dimension and index
    //! we want to keep this data structure as small as possible s.t. it can be cached by the CPU
    struct Element {
      Element(const int16_t row_, const int16_t col_, const int16_t index_) :
        row(row_),
        col(col_),
        index(index_) {
      }
      int16_t row;   // ds image row of the feature's keypoint position
      int16_t col;   // ds image col of the feature's keypoint position
      int16_t index; // ds index to the feature point inside the fixed cloud
    };
    std::vector<Element> _database_fixed; // ds feature's ordered by row index
  };

  using CorrespondenceFinderProjectiveSquare2D3D =
    CorrespondenceFinderProjectiveSquare<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderProjectiveSquare3D3D =
    CorrespondenceFinderProjectiveSquare<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderProjectiveSquare4D3D =
    CorrespondenceFinderProjectiveSquare<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderProjectiveSquare2D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveSquare2D3D>;
  using CorrespondenceFinderProjectiveSquare3D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveSquare3D3D>;
  using CorrespondenceFinderProjectiveSquare4D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveSquare4D3D>;

} // namespace srrg2_slam_interfaces
