#pragma once

#include "correspondence_finder_projective_square.h"

namespace srrg2_proslam {

  //! local brute-force matcher based projective correspondence finder
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderProjectiveCircle
    : public CorrespondenceFinderProjectiveSquare<TransformType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = CorrespondenceFinderProjectiveCircle<TransformType_, FixedType_, MovingType_>;
    using FixedPointType  = typename FixedType_::PointType;
    using MovingPointType = typename MovingType_::PointType;

    virtual ~CorrespondenceFinderProjectiveCircle() {
    }

  protected:
    virtual void
    _findNearestNeighbors(const MovingPointType& query_,
                          const size_t& query_index_,
                          CorrespondenceCandidateMap& neighbors_fixed_,
                          CorrespondenceCandidateMap& neighbors_moving_) const override;
  };

  using CorrespondenceFinderProjectiveCircle2D3D =
    CorrespondenceFinderProjectiveCircle<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor2fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderProjectiveCircle3D3D =
    CorrespondenceFinderProjectiveCircle<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderProjectiveCircle4D3D =
    CorrespondenceFinderProjectiveCircle<srrg2_core::Isometry3f,
                                         srrg2_core::PointIntensityDescriptor4fVectorCloud,
                                         srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderProjectiveCircle2D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveCircle2D3D>;
  using CorrespondenceFinderProjectiveCircle3D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveCircle3D3D>;
  using CorrespondenceFinderProjectiveCircle4D3DPtr =
    std::shared_ptr<CorrespondenceFinderProjectiveCircle4D3D>;

} // namespace srrg2_proslam
