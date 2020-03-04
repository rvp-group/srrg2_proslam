#include "correspondence_finder_descriptor_based_epipolar.h"

namespace srrg2_slam_interfaces {

  //! internal bookkeeping for fast correspondence search
  //! not linking to the actual PointIntensityDescriptor_ saves memory and dereferencing
  template <typename PointType_>
  struct Feature {
    Feature(const PointType_& point_, int32_t index_) :
      row(point_.coordinates()(1)), /*image coordinate Y*/
      col(point_.coordinates()(0)), /*image coordinate X*/
      unsorted_index(index_) {
    }
    Feature() : row(-1), col(-1), unsorted_index(-1) {
    }

    int32_t row;            //! the row in which the feature was detected
    int32_t col;            //! the row in which the feature was detected
    int32_t unsorted_index; //! the index of the feature object in the original vector
  };

  template <typename PointType_>
  using FeatureVector = std::vector<Feature<PointType_>>;

  template <typename PointCloudType_>
  void _sortFeatureVector(const PointCloudType_* features_,
                          FeatureVector<typename PointCloudType_::PointType>& feature_vector_) {
    feature_vector_.clear();
    feature_vector_.reserve(features_->size());

    // ds TODO enable simultaneous insertion and sorting
    for (size_t index = 0; index < features_->size(); ++index) {
      feature_vector_.emplace_back(
        Feature<typename PointCloudType_::PointType>(features_->at(index), index));
    }
    std::sort(feature_vector_.begin(),
              feature_vector_.end(),
              [](const Feature<typename PointCloudType_::PointType>& a_,
                 const Feature<typename PointCloudType_::PointType>& b_) {
                return ((a_.row < b_.row) || (a_.row == b_.row && a_.col < b_.col));
              });
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void
  CorrespondenceFinderDescriptorBasedEpipolar<TransformType_, FixedType_, MovingType_>::compute() {
    ThisType::_preCompute();

    // ds if neither fixed nor moving changed we keep the last computation state
    if (!ThisType::_fixed_changed_flag && !ThisType::_moving_changed_flag) {
      return;
    }
    PROFILE_TIME("CorrespondenceFinderDescriptorBasedEpipolar::compute");

    // ds allocate sufficient space for matches
    ThisType::_correspondences->clear();
    ThisType::_correspondences->reserve(ThisType::_fixed->size());

    // ds cached parameters
    const float& maximum_descriptor_distance = ThisType::param_maximum_descriptor_distance.value();
    const float& maximum_distance_ratio_to_second_best =
      ThisType::param_maximum_distance_ratio_to_second_best.value();
    const int32_t maximum_disparity_pixels = param_maximum_disparity_pixels.value();

    // ds prepare vectors for stereo matching
    FeatureVector<FixedPointType> features_left;
    FeatureVector<MovingPointType> features_right;
    _sortFeatureVector<FixedType>(ThisType::_fixed, features_left);
    _sortFeatureVector<MovingType>(ThisType::_moving, features_right);

    // ds vertical offsets to consider
    std::vector<int32_t> row_offsets;
    row_offsets.push_back(0);
    for (int32_t row_offset = 1;
         row_offset < 1 + static_cast<int32_t>(param_epipolar_line_thickness_pixels.value());
         ++row_offset) {
      row_offsets.push_back(row_offset);
      row_offsets.push_back(-row_offset);
    }

    // ds for each offset
    for (const int32_t row_offset_pixels : row_offsets) {
      // ds running variable
      uint32_t index_right = 0;

      // ds indices to remove from candidate buffers
      std::set<size_t> matched_indices_left;
      std::set<size_t> matched_indices_right;

      // ds loop over all left keypoints
      for (size_t index_left = 0; index_left < features_left.size(); ++index_left) {
        // ds if there are no more points on the right to match against - stop
        if (index_right == features_right.size()) {
          break;
        }

        // ds the right keypoints are on an higher row - skip left to get lower
        while (features_left[index_left].row + row_offset_pixels <
               features_right[index_right].row) {
          ++index_left;
          if (index_left == features_left.size()) {
            break;
          }
        }
        if (index_left == features_left.size()) {
          break;
        }

        // ds we're on the epipolar line for the left feature - cache it
        const int& row_left               = features_left[index_left].row + row_offset_pixels;
        const int& col_left               = features_left[index_left].col;
        const size_t& unsorted_index_left = features_left[index_left].unsorted_index;

        // ds cache fixed descriptor field (must have distance method)
        const DescriptorType& descriptor_left =
          ThisType::_fixed->at(unsorted_index_left).template field<2>();

        // ds the right keypoints are on an lower row - skip right to get higher (heh)
        while (row_left > features_right[index_right].row) {
          ++index_right;
          if (index_right == features_right.size()) {
            break;
          }
        }
        if (index_right == features_right.size()) {
          break;
        }

        // ds search bookkeeping
        size_t index_search_right             = index_right;
        float descriptor_distance_best        = std::numeric_limits<float>::max();
        float descriptor_distance_second_best = std::numeric_limits<float>::max();
        size_t index_best_right               = 0;

        // ds scan epipolar line for current keypoint at idx_L - exhaustive
        while (row_left == features_right[index_search_right].row &&
               index_search_right < features_right.size()) {
          const int32_t disparity_pixels = col_left - features_right[index_search_right].col;

          // ds invalid disparity stop condition
          if (disparity_pixels < 0) {
            break;
          }

          // ds if disparity is too high (we skip comparison)
          if (disparity_pixels > maximum_disparity_pixels) {
            ++index_search_right;
            continue;
          }

          // ds cache moving descriptor field (must have distance method)
          const DescriptorType& descriptor_right =
            ThisType::_moving->at(features_right[index_search_right].unsorted_index)
              .template field<2>();

          // ds compute descriptor distance for the stereo match candidates
          const int descriptor_distance = descriptor_left.distance(descriptor_right);
          if (descriptor_distance < descriptor_distance_best) {
            descriptor_distance_second_best = descriptor_distance_best;
            descriptor_distance_best        = descriptor_distance;
            index_best_right                = index_search_right;
          } else if (descriptor_distance < descriptor_distance_second_best) {
            descriptor_distance_second_best = descriptor_distance;
          }

          // ds advance search
          ++index_search_right;
        }

        // ds if something was found
        if (descriptor_distance_best < maximum_descriptor_distance &&
            descriptor_distance_best / descriptor_distance_second_best <
              maximum_distance_ratio_to_second_best) {
          const size_t& unsorted_index_right = features_right[index_best_right].unsorted_index;

          // ds register the stereo match
          ThisType::_correspondences->emplace_back(
            Correspondence(unsorted_index_left, unsorted_index_right, descriptor_distance_best));

          // ds reduce search space (this eliminates all structurally conflicting matches)
          index_right = index_best_right + 1;

          // ds pruning for recursive epipolar line search
          matched_indices_left.insert(index_left);
          matched_indices_right.insert(index_best_right);
        }
      }

      // ds prune remaining candidates for multiline matching - without breaking the ordering
      size_t index_keep = 0;
      for (size_t i = 0; i < features_left.size(); ++i) {
        if (matched_indices_left.count(i) == 0) {
          features_left[index_keep] = std::move(features_left[i]);
          ++index_keep;
        }
      }
      features_left.resize(index_keep);
      index_keep = 0;
      for (size_t i = 0; i < features_right.size(); ++i) {
        if (matched_indices_right.count(i) == 0) {
          features_right[index_keep] = std::move(features_right[i]);
          ++index_keep;
        }
      }
      features_right.resize(index_keep);
    }

    // ds compute matching success ratio to flag bad situations
    const float matching_ratio =
      static_cast<float>(ThisType::_correspondences->size()) / ThisType::_fixed->size();
    if (matching_ratio < ThisType::param_minimum_matching_ratio.value()) {
      std::cerr << "CorrespondenceFinderDescriptorBasedEpipolar::compute|low matching ratio: "
                << matching_ratio << " (" << ThisType::_correspondences->size() << "/"
                << ThisType::_fixed->size()
                << ") target: " << ThisType::param_minimum_matching_ratio.value() << std::endl;
    }

    ThisType::_postCompute();
  }

} // namespace srrg2_slam_interfaces
