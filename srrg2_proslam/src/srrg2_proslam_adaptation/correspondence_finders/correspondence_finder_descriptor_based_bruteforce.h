#pragma once
#include <srrg_system_utils/profiler.h>
#include <srrg_viewer/drawable_base.h>
#include <unordered_set>

#include "srrg2_proslam_adaptation/intensity_feature_extractor_base.h"
#include "srrg_slam_interfaces/correspondence_finder.h"

namespace srrg2_slam_interfaces {

  // ds TODO split in CorrespondenceFinderDescriptorBased base class
  template <typename TransformType_, typename FixedType_, typename MovingType_>
  class CorrespondenceFinderDescriptorBasedBruteforce
    : public CorrespondenceFinder_<TransformType_, FixedType_, MovingType_>,
      public Profiler {
    using ThisType =
      CorrespondenceFinderDescriptorBasedBruteforce<TransformType_, FixedType_, MovingType_>;
    using TransformType = TransformType_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyFloat,
          maximum_descriptor_distance,
          "maximum permitted descriptor distance for a match",
          50.0f,
          nullptr);
    PARAM(PropertyFloat,
          maximum_distance_ratio_to_second_best,
          "Lowe's distance to drop ambiguous match candidates",
          0.9f,
          nullptr);
    PARAM(PropertyFloat,
          minimum_matching_ratio,
          "desired minimum matching ratio with current configuration (signals transgressions)",
          0.25f,
          nullptr);

    virtual ~CorrespondenceFinderDescriptorBasedBruteforce() {
    }

    // ds TODO move up?
    virtual void setEstimate(const TransformType& estimate_moving_from_fixed_) override {
      _estimate_moving_from_fixed = estimate_moving_from_fixed_;
      _estimate_changed_flag      = true;
    }

    //! computes exhaustively bijective (!) matches with Lowe's check
    virtual void compute() override;

    //! checks if the Lowe's ratio is guaranteed between the best and second best descriptor(s)
    //! literature: https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf
    virtual bool checkLowesRatio(const float& descriptor_distance_best_,
                                 const float& descriptor_distance_other_,
                                 const float& maximum_ratio_) const final;

    //! checks if the Lowe's ratio is guaranteed between the best and second best descriptor(s)
    //! literature: https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf
    virtual bool checkLowesRatio(const float& descriptor_distance_best_,
                                 const std::vector<float>& descriptor_distances_other_,
                                 const float& maximum_ratio_) const final;

  protected:
    //! helper functions to pre/post validate input and log unexpected behavior
    void _preCompute();
    void _postCompute();
    void _processCorrespondencePool(
      CorrespondenceVector& correspondence_pool_,
      std::unordered_map<int, std::vector<float>>& best_distances_for_fixed_,
      std::unordered_map<int, std::vector<float>>& best_distances_for_moving_,
      std::unordered_set<int>& registered_indices_fixed_,
      std::unordered_set<int>& registered_indices_moving_);

    //! spatial estimate between moving and fixed TODO move up?
    TransformType _estimate_moving_from_fixed;
    bool _estimate_changed_flag = false;
  };

  using CorrespondenceFinderDescriptorBasedBruteforce2D2D =
    CorrespondenceFinderDescriptorBasedBruteforce<Isometry3f,
                                                  PointIntensityDescriptor2fVectorCloud,
                                                  PointIntensityDescriptor2fVectorCloud>;
  using CorrespondenceFinderDescriptorBasedBruteforce2D3D =
    CorrespondenceFinderDescriptorBasedBruteforce<Isometry3f,
                                                  PointIntensityDescriptor2fVectorCloud,
                                                  PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderDescriptorBasedBruteforce3D3D =
    CorrespondenceFinderDescriptorBasedBruteforce<Isometry3f,
                                                  PointIntensityDescriptor3fVectorCloud,
                                                  PointIntensityDescriptor3fVectorCloud>;
  using CorrespondenceFinderDescriptorBasedBruteforce4D3D =
    CorrespondenceFinderDescriptorBasedBruteforce<Isometry3f,
                                                  PointIntensityDescriptor4fVectorCloud,
                                                  PointIntensityDescriptor3fVectorCloud>;

  using CorrespondenceFinderDescriptorBasedBruteforce2D2DPtr =
    std::shared_ptr<CorrespondenceFinderDescriptorBasedBruteforce2D2D>;
  using CorrespondenceFinderDescriptorBasedBruteforce2D3DPtr =
    std::shared_ptr<CorrespondenceFinderDescriptorBasedBruteforce2D3D>;
  using CorrespondenceFinderDescriptorBasedBruteforce3D3DPtr =
    std::shared_ptr<CorrespondenceFinderDescriptorBasedBruteforce3D3D>;
  using CorrespondenceFinderDescriptorBasedBruteforce4D3DPtr =
    std::shared_ptr<CorrespondenceFinderDescriptorBasedBruteforce4D3D>;

} // namespace srrg2_slam_interfaces
