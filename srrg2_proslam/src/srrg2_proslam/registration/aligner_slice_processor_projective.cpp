#include "aligner_slice_processor_projective.h"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  AlignerSliceProcessorProjective_<FactorType_, FixedType_, MovingType_>::
    AlignerSliceProcessorProjective_() {
    ThisType::param_fixed_slice_name.setValue("points");
    ThisType::param_moving_slice_name.setValue("points");
    ThisType::param_finder.setValue(std::shared_ptr<CFTypeBase>(
      new CorrespondenceFinderProjectiveKDTree<EstimateType, FixedType, MovingType>()));
    ThisType::param_finder->setName("aligner_correspondence_finder");
    ThisType::param_robustifier.setValue(std::shared_ptr<srrg2_solver::RobustifierSaturated>(
      new srrg2_solver::RobustifierSaturated()));
    ThisType::param_robustifier->setName("aligner_robustifier");
    ThisType::setName("aligner_slice_projective");
    ThisType::param_robustifier->param_chi_threshold.setValue(100 * 100);
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorProjective_<FactorType_, FixedType_, MovingType_>::bindFixed() {
    BaseType::bindFixed();
    // ds postprocess fixed here
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorProjective_<FactorType_, FixedType_, MovingType_>::setupFactor() {
    // ds NOTE this method is assumed to be called once for all correspondences!
    // ds it is assumed that fixed and moving slices and correspondences are set and valid
    assert(BaseType::_fixed_slice);
    assert(BaseType::_moving_slice);
    assert(param_projector.value() && "AlignerSliceProjective_|projector not set");

    // ds set factor specific parameters
    ThisType::_camera_matrix = param_projector->cameraMatrix();
    ThisType::_factor->setCameraMatrix(ThisType::_camera_matrix);
    ThisType::_factor->setImageDim(srrg2_core::Vector2f(
      param_projector->param_canvas_cols.value(), param_projector->param_canvas_rows.value()));

    // ds allocate information matrix buffer (no cost if already at correct size)
    BaseType::_fixed_information_matrix_vector.resize(BaseType::_fixed_slice->size());

    // ds set information buffer for all correspondences (we leverage on moving knowledge)
    // ds increase weighting proportional to point age TODO incorporate properly
    constexpr size_t minimum_number_of_updates = 2;
    for (const Correspondence& correspondence : BaseType::_correspondences) {
      const MovingPointType& point_moving((*BaseType::_moving_slice)[correspondence.moving_idx]);
      DiagInfoVector diagonal_info(param_diagonal_info_matrix.value());
      if (point_moving.statistics().numberOfOptimizations() > minimum_number_of_updates) {
        diagonal_info *= (1 + std::log(point_moving.statistics().numberOfOptimizations()));
      }

      // ds information buffer will be accessed by fixed index in linearization
      BaseType::_fixed_information_matrix_vector[correspondence.fixed_idx] =
        diagonal_info.asDiagonal();
    }

    // ds adjust robustifier threshold based on current finder tolerance
    assert(ThisType::param_robustifier.value());
    //    auto projective_finder(ThisType::param_finder.template getSharedPtr<ProjectiveCFType>());
    //    if (projective_finder) {
    //      // ds if we have a sufficient number of correspondences
    //      if (ThisType::_correspondences.size() > 50) {
    //        // ds adjust robustifier threshold proportional to CF search radius
    //        ThisType::param_robustifier->param_chi_threshold.setValue(
    //          projective_finder->searchRadiusPixelsSquared());
    //      } else {
    //        // ds reset chi
    //        ThisType::param_robustifier->param_chi_threshold.setValue(1000);
    //      }
    //    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorProjectiveStereo_<FactorType_, FixedType_, MovingType_>::bindFixed() {
    BaseType::bindFixed();

    // ds postprocess fixed here - update mean disparity
    if (!BaseType::_fixed_slice->empty()) {
      float accumulated_disparity = 0;
      for (const FixedPointType& point_fixed : *BaseType::_fixed_slice) {
        accumulated_disparity += point_fixed.coordinates()(0) - point_fixed.coordinates()(2);
      }
      _mean_disparity = accumulated_disparity / BaseType::_fixed_slice->size();
    } else {
      _mean_disparity = 0;
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorProjectiveStereo_<FactorType_, FixedType_, MovingType_>::setupFactor() {
    // ds minimal setup
    BaseType::setupFactor(); // ^
    if (!_baseline_set) {
      srrg2_core::Isometry3f left_camera_in_right;
      assert(ThisType::_platform && "AlignerSliceStereoProjective_|platform not present");
      ThisType::_platform->getTransform(
        left_camera_in_right, param_frame_camera_left.value(), param_frame_camera_right.value());
      _baseline_left_in_right_pixelsmeters =
        ThisType::param_projector->cameraMatrix() * left_camera_in_right.translation();
      _baseline_set = true;
    }
    ThisType::_factor->setBaselineLeftInRightPixels(_baseline_left_in_right_pixelsmeters);

    // ds enables depth based point weighting
    if (param_enable_inverse_depth_weighting.value()) {
      assert(_mean_disparity > 0);

      // ds use normalized disparity as weight for translation contribution in jac: (0.01+d,1)*I
      ThisType::_factor->setMeanDisparityPixels(_mean_disparity);
    }

    // ds enables point covariance integration into weighting
    // ds experimental and probably doesn't make sense to include the update-only filter input here
    if (param_enable_point_covariance_integration.value()) {
      //      // ds compute moving point position in fixed camera frame
      //      assert(BaseType::_aligner);
      //      const MovingPointType& point_moving(
      //        (*BaseType::_moving_slice)[BaseType::_current_it->moving_idx]);
      //      const EstimateType& estimate_fixed_from_moving = BaseType::_aligner->estimate();
      //      const Vector3f& point_in_camera(estimate_fixed_from_moving *
      //      point_moving.coordinates());
      //
      //      // ds compute point specific information matrix
      //      assert(BaseType::_fixed_slice);
      //      assert(BaseType::_moving_slice);
      //      const FixedPointType& point_fixed(
      //        (*BaseType::_fixed_slice)[BaseType::_current_it->fixed_idx]);
      //      const Vector4f& measurement(point_fixed.coordinates());
      //      const Matrix4d& measurement_covariance(point_fixed.statistics().covariance());
      //      const size_t&
      //      number_of_optimizations(point_moving.statistics().numberOfOptimizations()); const
      //      Matrix3d& point_in_camera_covariance(point_moving.statistics().covariance());
      //      assert(!point_in_camera_covariance.hasNaN());
      //
      //      std::cerr << "motion: " << estimate_fixed_from_moving.translation().transpose() <<
      //      std::endl; std::cerr << "index fixed: " << BaseType::_current_it->fixed_idx <<
      //      std::endl; std::cerr << "measurement: " << measurement.transpose() << std::endl;
      //      std::cerr << "measurement covariance: \n" << measurement_covariance << std::endl;
      //      std::cerr << "index moving: " << BaseType::_current_it->moving_idx << std::endl;
      //      std::cerr << "point # optimizations: " << number_of_optimizations << std::endl;
      //      std::cerr << "point in camera: " << point_in_camera.transpose() << std::endl;
      //      std::cerr << "point in camera covariance: \n" << point_in_camera_covariance <<
      //      std::endl; std::cerr <<
      //      "-----------------------------------------------------------------" << std::endl;
      //
      //      // ds compute moving point covariance in fixed camera frame (eh)
      //      const float& z = point_in_camera(2);
      //      assert(z > 0);
      //      const float f_x     = BaseType::_camera_matrix(0, 0);
      //      const float f_y     = BaseType::_camera_matrix(1, 1);
      //      const float b_x     = _baseline_right_in_left_pixelsmeters(0);
      //      const float b_y     = _baseline_right_in_left_pixelsmeters(1);
      //      const float z_2     = z * z;
      //      const float fx_x    = f_x * point_in_camera(0);
      //      const float fy_y    = f_y * point_in_camera(1);
      //      const float fx_by_z = f_x / z;
      //      const float fy_by_z = f_y / z;
      //
      //      // ds jacobian parts of the stereo projection
      //      Matrix4_3f jacobian_projection;
      //      jacobian_projection << fx_by_z, 0.0, -fx_x / z_2, 0.0, fy_by_z, -fy_y / z_2, fx_by_z,
      //      0.0,
      //        -(fx_x + b_x) / z_2, 0.0, fy_by_z, -(fy_y + b_y) / z_2;
      //      assert(!jacobian_projection.hasNaN());
      //      std::cerr << "jacobian stereo projection: \n" << jacobian_projection << std::endl;
      //      const Matrix4f point_in_image_covariance = jacobian_projection *
      //                                                 point_in_camera_covariance.cast<float>() *
      //                                                 jacobian_projection.transpose();
      //      std::cerr << "point on image covariance: \n" << point_in_image_covariance <<
      //      std::endl;
      //
      //      std::cerr << "-----------------------------------------------------------------" <<
      //      std::endl; std::cerr << "final information matrix: " << std::endl; Vector3f
      //      updated_info_diagonal(ThisType::_factor->informationMatrix().diagonal());
      //      updated_info_diagonal += Vector3f(1 / point_in_image_covariance(0, 0),
      //                                        1 / point_in_image_covariance(1, 1),
      //                                        1 / point_in_image_covariance(2, 2));
      //      ThisType::_factor->setInformationMatrix(updated_info_diagonal.asDiagonal());
      //      std::cerr << ThisType::_factor->informationMatrix().diagonal() << std::endl;
    }
  }

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  void AlignerSliceProcessorProjective_<FactorType_, FixedType_, MovingType_>::draw(
    srrg2_core::ViewerCanvasPtr canvas_) const {
    if (MovingPointType::Dim >= 2 && ThisType::_moving_slice) {
      // ds buffer lines to draw (instead of spawning N tiny packets) TODO ugly in heap
      const size_t buffer_size = 2 * ThisType::_correspondences.size();
      std::vector<Vector3f> line_buffer;
      line_buffer.reserve(buffer_size);

      const Vector3f& sensor_in_robot_coords = ThisType::_sensor_in_robot.translation();

      // ds lines are drawn w.r.t. current robot pose, hence external viewpoint adjustment is
      // needed
      for (const Correspondence& correspondence : ThisType::_correspondences) {
        const auto& point_coordinates(
          (*ThisType::_moving_slice)[correspondence.moving_idx].coordinates());

        line_buffer.emplace_back(sensor_in_robot_coords);

        Vector3f line(Vector3f::Zero());
        line(0) = point_coordinates(0);
        line(1) = point_coordinates(1);
        if (MovingPointType::Dim == 3) {
          line(2) = point_coordinates(2);
        }
        line_buffer.emplace_back(line);
      }
      assert(line_buffer.size() == buffer_size);

      // ds serialize lines in bulk (1 packet)
      canvas_->pushColor();
      canvas_->setColor(srrg2_core::ColorPalette::color3fRed());
      canvas_->putLine(buffer_size, &line_buffer[0]);
      canvas_->popAttribute();

      // srrg sensor pose
      canvas_->pushMatrix();
      canvas_->multMatrix(ThisType::_sensor_in_robot.matrix());
      canvas_->pushColor();
      canvas_->setColor(srrg2_core::ColorPalette::color3fCyan());
      canvas_->putSphere(0.05);
      canvas_->popAttribute();
      canvas_->popMatrix();
    }
  }

  // ia commented because it's too much work for me, faccio il mircolosi
  // template <typename FactorType_, typename FixedType_, typename MovingType_>
  // void AlignerSliceProjective_<FactorType_, FixedType_, MovingType_>::draw(cv::Mat& canvas_)
  // const {
  //   if (canvas_.empty() || !ThisType::_fixed_slice) {
  //     return;
  //   }
  //   assert(FixedPointType::Dim >= 2);

  //   // ds draw ALL measured point coordinates in BLUE
  //   for (const FixedPointType& point : *ThisType::_fixed_slice) {
  //     cv::circle(canvas_,
  //                cv::Point2f(point.coordinates()(0), point.coordinates()(1)),
  //                1,
  //                cv::Scalar(255, 0, 0),
  //                -1);
  //   }
  //   if (!ThisType::_moving_slice) {
  //     return;
  //   }
  //   assert(MovingPointType::Dim == 3);

  //   // ds draw correspondence finder state
  //   ThisType::param_finder->draw(canvas_);

  //   // ds cache transform and camera matrix
  //   const EstimateType& fixed_from_moving(ThisType::_aligner->estimate());

  //   // ds statistics to draw in the banner
  //   size_t number_of_inlier_correspondences = 0;
  //   size_t accumulated_track_length         = 0;
  //   size_t minimum_track_length             = std::numeric_limits<size_t>::max();
  //   size_t maximum_track_length             = 0;
  //   double accumulated_uncertainty_in_x     = 0;
  //   double accumulated_uncertainty_in_y     = 0;

  //   // ds for all points with correspondences (i.e. tracks)
  //   for (const Correspondence& correspondence : ThisType::_correspondences) {
  //     // ds draw inlier points with correspondences and their previous measurement in GREEN
  //     const FixedPointType& point_current((*ThisType::_fixed_slice)[correspondence.fixed_idx]);
  //     const cv::Point2f point_current_in_image(point_current.coordinates()(0),
  //                                              point_current.coordinates()(1));
  //     MovingPointType point_previous((*ThisType::_moving_slice)[correspondence.moving_idx]);
  //     if (point_previous.statistics().isInlier()) {
  //       ++number_of_inlier_correspondences;
  //       const auto& projection_previous(point_previous.statistics().projection());
  //       const cv::Point2f point_previous_in_image(projection_previous(0),
  //       projection_previous(1));

  //       // ds draw tracking distance in pixel space
  //       cv::line(canvas_, point_current_in_image, point_previous_in_image, cv::Scalar(0, 255,
  //       0));

  //       // ds transform point from scene into fixed camera frame and project point onto image
  //       assert(fixed_from_moving.matrix().norm() > 0);
  //       point_previous.template transformInPlace<Isometry, EstimateType>(fixed_from_moving);
  //       const Vector3f camera_coordinates(point_previous.coordinates());
  //       assert(ThisType::_camera_matrix.matrix().norm() > 0);
  //       point_previous.template transformInPlace<PinholeProjection, EstimateType>(
  //         ThisType::_camera_matrix);
  //       const cv::Point2f projection_estimate(point_previous.coordinates()(0),
  //                                             point_previous.coordinates()(1));

  //       // ds draw refined projection and error
  //       cv::circle(canvas_, projection_estimate, 3, cv::Scalar(0, 0, 255), 1);
  //       cv::line(canvas_, point_current_in_image, projection_estimate, cv::Scalar(0, 0, 255));
  //       cv::circle(canvas_, point_current_in_image, 3, cv::Scalar(0, 255, 0));

  //       // ds draw number of observations next to measurement
  //       const size_t track_length = point_previous.statistics().numberOfOptimizations();
  //       cv::putText(canvas_,
  //                   std::to_string(track_length),
  //                   point_current_in_image + cv::Point2f(5, 5),
  //                   cv::FONT_HERSHEY_PLAIN,
  //                   0.5,
  //                   cv::Scalar(0, 0, 255));

  //       //        // ds project covariance onto image by sampling the noise on the point (camera)
  //       //        // ds TODO include proper direction of uncertainty (requires jac)
  //       //        MovingPointType
  //       //        point_noisy((*ThisType::_moving_slice)[correspondence.moving_idx]);
  //       //        point_noisy.template transformInPlace<Isometry,
  //       EstimateType>(fixed_from_moving);
  //       //        const auto& covariance_in_camera(point_previous.statistics().covariance());
  //       //        for (size_t i = 0; i < EstimateType::Dim; ++i) {
  //       //          // ds TODO for very low covariances we go below zero, this can be dropped
  //       once
  //       //          we double point_noisy.coordinates()(i) +=
  //       //          std::sqrt(std::fabs(covariance_in_camera(i, i)));
  //       //        }
  //       //        point_noisy.template transformInPlace<PinholeProjection, EstimateType>(
  //       //          ThisType::_camera_matrix);
  //       //        const cv::Point2f projection_noisy(point_noisy.coordinates()(0),
  //       //                                           point_noisy.coordinates()(1));
  //       //        const cv::Size axes(3 + std::fabs(projection_noisy.x - projection_estimate.x),
  //       //                            3 + std::fabs(projection_noisy.y - projection_estimate.y));
  //       //
  //       //        // ds draw projected covariance of moving on fixed with refined estimate in RED
  //       //        cv::ellipse(canvas_, projection_estimate, axes, 0, 0, 360 /*zz*/, cv::Scalar(0,
  //       0,
  //       //        255), 1);

  //       // ds update stats for hud
  //       accumulated_track_length += track_length;
  //       if (track_length > maximum_track_length) {
  //         maximum_track_length = track_length;
  //       }
  //       if (track_length < minimum_track_length) {
  //         minimum_track_length = track_length;
  //       }
  //       accumulated_uncertainty_in_x += 0; // point_noisy.coordinates()(0);
  //       accumulated_uncertainty_in_y += 0; // point_noisy.coordinates()(1);
  //     }
  //   }

  //   // ds draw information bar
  //   cv::rectangle(
  //     canvas_, cv::Point2f(0, 0), cv::Point2f(canvas_.cols, 14), cv::Scalar(0, 0, 0), -1);
  //   std::stringstream stream;
  //   stream << "| meas points: " << ThisType::_fixed_slice->size();
  //   stream << " scene points: " << ThisType::_moving_slice->size();
  //   stream << " corr: " << ThisType::_correspondences.size();
  //   stream << " (inl: " << number_of_inlier_correspondences << ")";
  //   if (number_of_inlier_correspondences > 0) {
  //     stream << " | mean track length: "
  //            << static_cast<double>(accumulated_track_length) / number_of_inlier_correspondences;
  //     stream << " (min: " << minimum_track_length << ", max: " << maximum_track_length << ") ";
  //     stream << "| mean uncertainty in u: "
  //            << accumulated_uncertainty_in_x / number_of_inlier_correspondences
  //            << " in v: " << accumulated_uncertainty_in_y / number_of_inlier_correspondences
  //            << " |";
  //   }
  //   cv::putText(canvas_,
  //               stream.str(),
  //               cv::Point2f(0, 11),
  //               cv::FONT_HERSHEY_PLAIN,
  //               0.8,
  //               cv::Scalar(0, 255, 0));
  // }

} // namespace srrg2_proslam
