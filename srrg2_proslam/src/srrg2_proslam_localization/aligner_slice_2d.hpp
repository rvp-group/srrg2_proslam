#pragma once
#include <srrg_slam_interfaces/multi_aligner_slice.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_point2point_error_factor.h>

namespace srrg2_proslam {

  class AlignerSlice2D : public srrg2_slam_interfaces::MultiAlignerSlice_<
                           srrg2_solver::SE2Point2PointErrorFactor,
                           srrg2_core::PointIntensityDescriptor2fVectorCloud,
                           srrg2_core::PointIntensityDescriptor2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AlignerSlice2D(){};
    virtual void setupFactor() override {
      _factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 2>(1, 1));
    }
  };

  using AlignerSlice2DPtr = std::shared_ptr<AlignerSlice2D>;

} // namespace srrg2_proslam
