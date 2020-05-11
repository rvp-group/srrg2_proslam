#pragma once
#include <srrg2_slam_interfaces/instances.h>
namespace srrg2_proslam {

  class LocalMapSelectorHBST
    : public srrg2_slam_interfaces::LocalMapSelector_<srrg2_slam_interfaces::MultiGraphSLAM3D> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SLAMAlgorithmType     = srrg2_slam_interfaces::MultiGraphSLAM3D;
    using LoopClosureType       = SLAMAlgorithmType::LoopClosureType;
    using LocalMapType          = LoopClosureType::LocalMapType;
    using EstimateType          = LocalMapType::EstimateType;
    using InformationMatrixType = LoopClosureType::InformationMatrixType;
  };
} // namespace srrg2_proslam
