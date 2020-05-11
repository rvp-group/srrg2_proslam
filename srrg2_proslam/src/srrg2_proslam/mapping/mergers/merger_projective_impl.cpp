#include "merger_projective.h"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;
  using namespace srrg2_core;

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerProjective_<TransformType_, FixedType_, MovingType_>::compute() {
    PROFILE_TIME("MergerProjective::compute");
    assert(BaseType::_scene);
    assert(BaseType::_measurement);
    BaseType::_status = BaseType::Error;

    // ds check if we can skip computation (including all sub classes)
    if (!BaseType::_scene_changed_flag && !BaseType::_meas_changed_flag &&
        !BaseType::_correspondences_changed_flag && !BaseType::_meas_in_scene_changed_flag) {
      BaseType::_status = BaseType::Success;
      return;
    }

    // ds projector is mandatory
    if (!param_projector.value()) {
      throw std::runtime_error("MergerProjective::compute|ERROR: projector not set");
    }

    // ds virtual dispatch (initialize eventual subclasses)
    _precompute();

    // ds cache
    const size_t number_of_points_measured = BaseType::_measurement->size();
    const size_t number_of_points_to_merge = BaseType::param_target_number_of_merges.value();
    _row_bin_width_pixels = static_cast<float>(param_projector->param_canvas_rows.value()) /
                            static_cast<float>(param_number_of_row_bins.value());
    _col_bin_width_pixels = static_cast<float>(param_projector->param_canvas_cols.value()) /
                            static_cast<float>(param_number_of_col_bins.value());
    const float maximum_descriptor_distance = param_maximum_distance_appearance.value();
    if (_row_bin_width_pixels < 1) {
      std::cerr << "projector's canvas size: " << param_projector->param_canvas_rows.value() << " "
                << param_projector->param_canvas_cols.value() << std::endl;
      throw std::runtime_error("MergerProjective::compute|ERROR: row bin width must be at "
                               "least 1 pixel - reduce param_number_of_row_bins");
    }
    if (_col_bin_width_pixels < 1) {
      std::cerr << "projector's canvas size: " << param_projector->param_canvas_rows.value() << " "
                << param_projector->param_canvas_cols.value() << std::endl;
      throw std::runtime_error("MergerProjective::compute|ERROR: col bin width must be at "
                               "least 1 pixel - reduce param_number_of_col_bins");
    }

    // ds binning map, storing occupied bin rows to cols (only needed when binning is enabled)
    BinMap occupied_bin_map;
    occupied_bin_map.reserve(param_number_of_row_bins.value());
    //    std::map<size_t, size_t> number_of_optimizations;

    // ds if there are no correspondences - we triangulate all points and add them as is
    if (!BaseType::_correspondences || BaseType::_correspondences->empty()) {
      _addPoints(occupied_bin_map, number_of_points_to_merge);
    } else {
      // ds attempt to update all point states for which we have correspondences
      size_t number_of_merged_points = 0;
      for (const Correspondence& correspondence : *BaseType::_correspondences) {
        assert(static_cast<size_t>(correspondence.fixed_idx) < BaseType::_scene->size());
        assert(static_cast<size_t>(correspondence.moving_idx) < BaseType::_measurement->size());
        ScenePointType& scene_point((*BaseType::_scene)[correspondence.fixed_idx]);
        assert(scene_point.status == POINT_STATUS::Valid);
        scene_point.statistics().setIsInlier(false);

        // ds scene point must be visible for the current transform
        assert((BaseType::_measurement_in_scene * scene_point.coordinates()).z() > 0);

        // ds if matching distance is too high
        if (correspondence.response > maximum_descriptor_distance) {
          // ds skip update TODO handle this properly
          continue;
        }

        // ds set measurement
        const size_t& index_measurement(correspondence.moving_idx);
        const MeasurementPointType& measured_point((*BaseType::_measurement)[index_measurement]);
        assert(measured_point.status == POINT_STATUS::Valid);

        // ds compute bin indices in left camera frame
        assert(MeasurementPointType::Dim >= 2);
        const size_t bin_row = std::round(measured_point.coordinates()(1) / _row_bin_width_pixels);
        const size_t bin_col = std::round(measured_point.coordinates()(0) / _col_bin_width_pixels);
        assert(bin_row <= static_cast<size_t>(param_number_of_row_bins.value()));
        assert(bin_col <= static_cast<size_t>(param_number_of_col_bins.value()));

        // ds TODO move this a scope up
        if (BaseType::param_enable_binning.value()) {
          // ds block bin (required to regularize subsequent point additions)
          auto iterator_row = occupied_bin_map.find(bin_row);
          if (iterator_row != occupied_bin_map.end()) {
            auto iterator_col = iterator_row->second.find(bin_col);
            if (iterator_col == iterator_row->second.end()) {
              iterator_row->second.insert(std::make_pair(bin_col, index_measurement));
              //              number_of_optimizations.insert(std::make_pair(
              //                index_measurement,
              //                scene_point.statistics().numberOfOptimizations()));
            } else {
              //              // ds if the current candidate is more mature
              //              const size_t current_number_of_optimizations =
              //                scene_point.statistics().numberOfOptimizations();
              //              if (current_number_of_optimizations >
              //                  number_of_optimizations.at(iterator_col->second)) {
              //                // ds replace the reference candidate by the current
              //                iterator_col->second = index_measurement;
              //                number_of_optimizations.insert(
              //                  std::make_pair(index_measurement,
              //                  current_number_of_optimizations));
              //              } else {
              continue; // ds skip multiple merges in the same bin
                        //              }
            }
          } else {
            // ds bin is not blocked yet - block it
            std::unordered_map<size_t, size_t> column_candidates;
            column_candidates.insert(std::make_pair(bin_col, index_measurement));
            occupied_bin_map.insert(std::make_pair(bin_row, column_candidates));
            //            number_of_optimizations.insert(
            //              std::make_pair(index_measurement,
            //              scene_point.statistics().numberOfOptimizations()));
          }
        }

        // ds integrate the measurement into the scene point - checking for geometric divergence
        // ds dynamic dispatch
        if (_updatePoint(measured_point, scene_point)) {
          ++number_of_merged_points;
        } else {
          // ds disable point - TODO handle this better
          // scene_point.status = POINT_STATUS::Invalid;
        }
      }
      //      std::cerr << "MergerProjective::compute|merged points: " << number_of_merged_points;
      //      std::cerr << "/" << BaseType::_correspondences->size() << "/" <<
      //      number_of_points_measured
      //                << std::endl;

      // ds evaluate merge success
      assert(number_of_merged_points <= number_of_points_measured);
      const float merge_ratio =
        static_cast<float>(number_of_merged_points) / BaseType::_correspondences->size();
      if (number_of_merged_points == 0) {
        std::cerr << FG_RED("MergerProjective::compute|WARNING: all merge attempts failed ("
                            << BaseType::_correspondences->size() << ")")
                  << std::endl;
      } else if (merge_ratio < param_target_merge_ratio.value()) {
        std::cerr << FG_YELLOW(
                       "MergerProjective::compute|WARNING: low merge ratio: " << merge_ratio;
                       std::cerr << " (" << number_of_merged_points << "/"
                                 << BaseType::_correspondences->size() << ")")
                  << std::endl;
      }

      // ds if merge target was not reached and we have not merged all measured points
      // ds we try to add new points
      if (number_of_merged_points < number_of_points_to_merge &&
          number_of_merged_points < number_of_points_measured) {
        // ds required additions to reach merge goal
        const size_t number_of_points_to_add =
          std::min(number_of_points_to_merge - number_of_merged_points,
                   number_of_points_measured - number_of_merged_points);
        _addPoints(occupied_bin_map, number_of_points_to_add);
      }
    }

    // ds fine if still here
    BaseType::_status = BaseType::Success;
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  bool MergerProjective_<TransformType_, FixedType_, MovingType_>::_updatePoint(
    const MeasurementPointType& measured_point_,
    ScenePointType& scene_point_) {
    assert(param_landmark_estimator.value());

    // ds update landmark position estimate using the estimator
    param_landmark_estimator->setMeasurement(measured_point_.coordinates());
    param_landmark_estimator->setLandmark(&scene_point_);
    param_landmark_estimator->compute();

    // ds if the update was successful
    if (scene_point_.statistics().isInlier()) {
      // ds copy point fields TODO automate
      scene_point_.descriptor() = std::move(measured_point_.descriptor());
      return true;
    } else {
      return false;
    }
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerProjective_<TransformType_, FixedType_, MovingType_>::_addPoints(
    BinMap& occupied_bin_map_,
    const size_t& number_of_points_to_add_) {
    // ds cache
    const size_t number_of_points_measured      = BaseType::_measurement->size();
    const size_t number_of_points_initial_scene = BaseType::_scene->size();

    // ds point candidates in merge space (camera)
    ScenePointCloudType points_in_camera_to_add;
    MeasurementPointCloudType points_in_image_to_add;

    // ds if binning is desired (i.e. processing moving)
    if (BaseType::param_enable_binning.value()) {
      // ds computed binned candidates
      BinMap occupied_bin_map_addition;
      points_in_image_to_add.reserve(number_of_points_measured);
      for (size_t i = 0; i < number_of_points_measured; ++i) {
        // ds compute bin indices in left camera frame
        const MeasurementPointType& point_in_image((*BaseType::_measurement)[i]);
        const size_t bin_row = std::round(point_in_image.coordinates()(1) / _row_bin_width_pixels);
        const size_t bin_col = std::round(point_in_image.coordinates()(0) / _col_bin_width_pixels);

        assert(bin_row <= static_cast<size_t>(param_number_of_row_bins.value()));
        assert(bin_col <= static_cast<size_t>(param_number_of_col_bins.value()));

        // ds check if the bin is not occupied by any tracked point
        auto iterator_row_tracked = occupied_bin_map_.find(bin_row);
        if (iterator_row_tracked != occupied_bin_map_.end()) {
          // ds row and col are already occupied by a tracked point
          if (iterator_row_tracked->second.find(bin_col) != iterator_row_tracked->second.end()) {
            continue;
          }
        }

        // ds the bin is available for addition
        auto iterator_row = occupied_bin_map_addition.find(bin_row);
        if (iterator_row != occupied_bin_map_addition.end()) {
          auto iterator_col = iterator_row->second.find(bin_col);
          if (iterator_col != iterator_row->second.end()) {
            const size_t& index_occupying(iterator_col->second);

            // ds if the candidate is more suitable than the currently occupying point
            if (_isBetterForAddition(point_in_image, points_in_image_to_add[index_occupying])) {
              // ds replace the occupying point with the candidate
              points_in_image_to_add[index_occupying] = point_in_image;
            }
          } else {
            // ds new point in bin column!
            iterator_row->second.insert(std::make_pair(bin_col, points_in_image_to_add.size()));
            points_in_image_to_add.emplace_back(point_in_image);
          }
        } else {
          // ds new point in bin row and column!
          std::unordered_map<size_t, size_t> column_candidates;
          column_candidates.insert(std::make_pair(bin_col, points_in_image_to_add.size()));
          occupied_bin_map_addition.insert(std::make_pair(bin_row, column_candidates));
          points_in_image_to_add.emplace_back(point_in_image);
        }
      }
    } else {
      // ds consider all measurements
      points_in_image_to_add = *BaseType::_measurement;
    }

    // ds adapt measurements to scene cloud (via e.g. unprojection, triangulation, ..)
    _adaptFromMeasurementToScene(points_in_image_to_add, points_in_camera_to_add);
    // assert(points_in_camera_to_add.size() == points_in_image_to_add.size());

    // ds by default we add all points
    size_t target_scene_size = number_of_points_initial_scene + points_in_camera_to_add.size();

    // ds if conservative addition is preferred
    if (param_enable_conservative_addition.value()) {
      throw std::runtime_error("conservative addition is currently disabled");
      //      // ds sort points in increasing depth
      //      std::sort(points_in_camera_to_add.begin(),
      //                points_in_camera_to_add.end(),
      //                [](const ScenePointType& a_, const ScenePointType& b_) {
      //                  return (a_.coordinates()(2) < b_.coordinates()(2));
      //                });
      //
      //      // ds only add as many points as necessary
      //      target_scene_size = number_of_points_initial_scene + number_of_points_to_add_;
    }
    BaseType::_scene->reserve(target_scene_size);

    // ds add from the triangulated points that passed the binning
    // srrg this assert does not hold when using real depht camera data
    // assert(points_in_camera_to_add.size() == points_in_image_to_add.size());
    for (size_t i = 0; i < points_in_camera_to_add.size(); ++i) {
      ScenePointType& point_in_camera = points_in_camera_to_add[i];
      if (point_in_camera.status == POINT_STATUS::Valid) {
        // ds initialize point as landmark
        _initializeLandmark(points_in_image_to_add[i],
                            point_in_camera,
                            BaseType::_measurement_in_world,
                            BaseType::_world_in_measurement);

        // ds move local point coordinates into scene (local map) coordinate frame
        point_in_camera.template transformInPlace<srrg2_core::Isometry, TransformType_>(
          BaseType::_measurement_in_scene);
        // ds add new point in scene frame
        BaseType::_scene->emplace_back(point_in_camera);

        // ds if we reached the target points - terminate
        if (BaseType::_scene->size() == target_scene_size) {
          break;
        }
      }
    }
    //    std::cerr << "MergerProjective_::_addPoints|added new points: "
    //              << BaseType::_scene->size() - number_of_points_initial_scene
    //              << " (scene size old: " << number_of_points_initial_scene
    //              << " new: " << BaseType::_scene->size() << ")" << std::endl;
  }

  template <typename TransformType_, typename FixedType_, typename MovingType_>
  void MergerProjective_<TransformType_, FixedType_, MovingType_>::_initializeLandmark(
    const MeasurementPointType& measurement_,
    ScenePointType& point_in_sensor_,
    const TransformType_& sensor_in_world_,
    const TransformType_& world_in_sensor_) const {
    // ds populate global statistics (landmark components)
    point_in_sensor_.statistics().allocate();
    point_in_sensor_.statistics().setState((sensor_in_world_ * point_in_sensor_.coordinates()));
    point_in_sensor_.statistics().setCovariance(Matrix3f::Identity());
    point_in_sensor_.statistics().setIsInlier(true);

    // ds add current measurement to statistics (optionally digested by landmark estimator)
    point_in_sensor_.statistics().addMeasurement(
      srrg2_core::PointStatisticsField3D::CameraMeasurement(measurement_.coordinates().head(3),
                                                            point_in_sensor_.coordinates(),
                                                            sensor_in_world_,
                                                            world_in_sensor_));
  }

} // namespace srrg2_proslam
