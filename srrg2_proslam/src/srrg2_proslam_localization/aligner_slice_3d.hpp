#pragma once
#include <srrg_slam_interfaces/multi_aligner_slice.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h>

namespace srrg2_proslam {

  class AlignerSlice3D : public srrg2_slam_interfaces::MultiAlignerSlice_<
                           srrg2_solver::SE3Point2PointErrorFactor,
                           srrg2_core::PointIntensityDescriptor3fVectorCloud,
                           srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AlignerSlice3D(){};
    virtual void setupFactor() override {
      _factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 3>(1, 1, 1));
    }
  };

  using AlignerSlice3DPtr = std::shared_ptr<AlignerSlice3D>;

} // namespace srrg2_proslam
