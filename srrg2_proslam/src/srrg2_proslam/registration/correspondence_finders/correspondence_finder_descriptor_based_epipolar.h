#pragma once
#include "correspondence_finder_descriptor_based_bruteforce.h"

namespace srrg2_proslam {
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderDescriptorBasedEpipolar
    : public CorrespondenceFinderDescriptorBasedBruteforce<TransformType_,
                                                           FixedType_,
                                                           MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType =
      CorrespondenceFinderDescriptorBasedEpipolar<TransformType_, FixedType_, MovingType_>;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::PointType;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::PointType;
    using DescriptorType  = typename FixedPointType::DescriptorField;

    PARAM(srrg2_core::PropertyUnsignedInt,
          maximum_disparity_pixels,
          "maximum disparity search range in pixels",
          100,
          nullptr);

    PARAM(srrg2_core::PropertyUnsignedInt,
          epipolar_line_thickness_pixels,
          "epipolar line search thickness in pixels (0 for perfect horizontal calibration)",
          0,
          nullptr);

    virtual ~CorrespondenceFinderDescriptorBasedEpipolar() {
    }

    virtual void compute() override;
  };

  using CorrespondenceFinderDescriptorBasedEpipolar2D2D =
    CorrespondenceFinderDescriptorBasedEpipolar<Isometry3f,
                                                PointIntensityDescriptor2fVectorCloud,
                                                PointIntensityDescriptor2fVectorCloud>;
  using CorrespondenceFinderDescriptorBasedEpipolar3D3D =
    CorrespondenceFinderDescriptorBasedEpipolar<Isometry3f,
                                                PointIntensityDescriptor3fVectorCloud,
                                                PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderDescriptorBasedEpipolar2D2DPtr =
    std::shared_ptr<CorrespondenceFinderDescriptorBasedEpipolar2D2D>;
  using CorrespondenceFinderDescriptorBasedEpipolar3D3DPtr =
    std::shared_ptr<CorrespondenceFinderDescriptorBasedEpipolar3D3D>;

} // namespace srrg2_proslam
