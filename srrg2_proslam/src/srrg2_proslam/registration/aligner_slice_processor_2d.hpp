#pragma once
#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_point2point_error_factor.h>

namespace srrg2_proslam {

  class AlignerSliceProcessor2D : public srrg2_slam_interfaces::AlignerSliceProcessor_<
                           srrg2_solver::SE2Point2PointErrorFactor,
                           srrg2_core::PointIntensityDescriptor2fVectorCloud,
                           srrg2_core::PointIntensityDescriptor2fVectorCloud> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~AlignerSliceProcessor2D(){};
    virtual void setupFactor() override {
      _factor->setInformationMatrix(Eigen::DiagonalMatrix<float, 2>(1, 1));
    }
  };

  using AlignerSliceProcessor2DPtr = std::shared_ptr<AlignerSliceProcessor2D>;

} // namespace srrg2_proslam
