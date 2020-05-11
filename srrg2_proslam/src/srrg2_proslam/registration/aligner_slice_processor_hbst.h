#pragma once
#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h>
#include <srrg_pcl/point_types.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h>

namespace srrg2_proslam {

  class AlignerSliceProcessorHBST : srrg2_slam_interfaces::AlignerSliceProcessor_<
                                      srrg2_solver::SE3Point2PointErrorFactor,
                                      srrg2_core::PointIntensityDescriptor3fVectorCloud,
                                      srrg2_core::PointIntensityDescriptor3fVectorCloud> {
  public:
  protected:
  };
} // namespace srrg2_proslam
