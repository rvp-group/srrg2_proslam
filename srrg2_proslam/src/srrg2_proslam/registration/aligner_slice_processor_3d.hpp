#pragma once
#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h>

namespace srrg2_proslam {

  class AlignerSliceProcessor3D : public srrg2_slam_interfaces::AlignerSliceProcessor_<
                                    srrg2_solver::SE3Point2PointErrorFactor,
                                    srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                    srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    using BaseType = srrg2_slam_interfaces::AlignerSliceProcessor_<
      srrg2_solver::SE3Point2PointErrorFactor,
      srrg2_core::PointIntensityDescriptor3fVectorCloud,
      srrg2_core::PointIntensityDescriptor3fVectorCloud>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AlignerSliceProcessor3D(){};
    virtual void setupFactor() override {
      BaseType::setupFactor();
      _factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 3>(1, 1, 1));
    }
  };

  using AlignerSliceProcessor3DPtr = std::shared_ptr<AlignerSliceProcessor3D>;

} // namespace srrg2_proslam
