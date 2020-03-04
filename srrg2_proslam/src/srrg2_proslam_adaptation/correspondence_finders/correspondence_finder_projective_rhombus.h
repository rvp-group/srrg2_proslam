#pragma once

#include "correspondence_finder_projective_square.h"

namespace srrg2_slam_interfaces {

  //! local brute-force matcher based projective correspondence finder
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderProjectiveRhombus
    : public CorrespondenceFinderProjectiveSquare<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = CorrespondenceFinderProjectiveRhombus<TransformType_, FixedType_, MovingType_>;
    using FixedPointType  = typename FixedType_::PointType;
    using MovingPointType = typename MovingType_::PointType;

    virtual ~CorrespondenceFinderProjectiveRhombus() {
    }

  protected:
    virtual void
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& neighbors_fixed_,
                          CorrespondenceCandidateMap& neighbors_moving_) const override;
  };

  using CorrespondenceFinderProjectiveRhombus2D3D =
    CorrespondenceFinderProjectiveRhombus<srrg2_core::Isometry3f,
                                          srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                          srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderProjectiveRhombus3D3D =
    CorrespondenceFinderProjectiveRhombus<srrg2_core::Isometry3f,
                                          srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                          srrg2_core::PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderProjectiveRhombus4D3D =
    CorrespondenceFinderProjectiveRhombus<srrg2_core::Isometry3f,
                                          srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                          srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderProjectiveRhombus2D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveRhombus2D3D>;
  using CorrespondenceFinderProjectiveRhombus3D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveRhombus3D3D>;
  using CorrespondenceFinderProjectiveRhombus4D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveRhombus4D3D>;

} // namespace srrg2_slam_interfaces
