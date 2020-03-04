#include "triangulator_rigid_stereo.h"

namespace srrg2_core {

  //! triangulates current set of correspondences in the left camera frame
  template <class MeasurementType_, class DestType_>
  void TriangulatorRigidStereo<MeasurementType_, DestType_>::compute() {
    Profiler::Timer(this, "TriangulatorRigidStereo::compute");
    if (!_stereo_intensity_matches) {
      std::cerr << "TriangulatorRigidStereo::compute|ERROR: input not set" << std::endl;
      return;
    }
    if (!ThisType::_dest) {
      std::cerr << "TriangulatorRigidStereo::compute|ERROR: result buffer not set" << std::endl;
      return;
    }

    // ds no effect if already initialized TODO move
    initializeBaseline();

    // ds prepare output triangulated point vector TODO ownership is not THIS, correct?
    ThisType::_dest->clear();
    ThisType::_dest->reserve(_stereo_intensity_matches->size());
    _indices_invalidated.clear();

    // ds for each correspondence
    for (size_t index = 0; index < _stereo_intensity_matches->size(); ++index) {
      const PointIntensityDescriptor4f& stereo_match = _stereo_intensity_matches->at(index);
      PointType triangulated_point;
      triangulated_point.status = Invalid;

      // ds cache/readability
      const ScalarType x_L = stereo_match.coordinates()(0);
      const ScalarType y_L = stereo_match.coordinates()(1);
      const ScalarType x_R = stereo_match.coordinates()(2);
      const ScalarType y_R = stereo_match.coordinates()(3);

      // ds skip point if horizontal disparity is insufficient
      if (x_L - x_R < param_minimum_disparity_pixels.value()) {
        _indices_invalidated.insert(index);

        // ds we insert an invalid point to maintain the input vector size and outer correspondences
        ThisType::_dest->emplace_back(triangulated_point);
        continue;
      }

      // ds triangulate point with selected method
      // ds TODO copy fields recursively or hardcode, currently values from left are copied
      triangulated_point.template descriptor() = stereo_match.descriptor();
      triangulated_point.template intensity()  = stereo_match.intensity();
      triangulateRectifiedMidpoint(x_L, y_L, x_R, y_R, triangulated_point.coordinates());
      triangulated_point.status = Valid;
      ThisType::_dest->emplace_back(triangulated_point);
    }
    assert(ThisType::_dest->size() == _stereo_intensity_matches->size());
  }

  //! point-wise triangulation method TODO make class generic and multiple methods available
  template <class MeasurementType_, class DestType_>
  void TriangulatorRigidStereo<MeasurementType_, DestType_>::triangulateRectifiedMidpoint(
    const ScalarType& x_L_,
    const ScalarType& y_L_,
    const ScalarType& x_R_,
    const ScalarType& y_R_,
    Vector3Type& coordinates_in_camera_left_) const {
    assert(_baseline_set);
    assert(_b_x < 0 && _f_x != 0 && _f_y != 0);
    assert(x_L_ >= x_R_);
    assert(x_L_ - x_R_ >= param_minimum_disparity_pixels.value());

    // ds compute point depth - initial guess is infinity depth
    ScalarType depth_meters = param_infinity_depth_meters.value();

    // ds if point has non-zero disparity we can triangulate
    if (x_L_ > x_R_) {
      depth_meters = _b_x / (x_R_ - x_L_);
    }
    coordinates_in_camera_left_.z() = depth_meters;

    // ds compute x in camera (regular)
    coordinates_in_camera_left_.x() = 1 / _f_x * (x_L_ - _c_x) * depth_meters;

    // ds average in case we have an epipolar offset in v
    coordinates_in_camera_left_.y() = 1 / _f_y * ((y_L_ + y_R_) / 2 - _c_y) * depth_meters;
  }

  template <class MeasurementType_, class DestType_>
  void TriangulatorRigidStereo<MeasurementType_, DestType_>::initializeBaseline() {
    assert(param_projector.value() && "projector not set");
    if (!_baseline_set) {
      const CameraCalibrationMatrixType& camera_calibration_matrix =
        param_projector->cameraMatrix();
      // ds buffer current camera configuration (cheap and we avoid having more flying booleans)
      // ds the camera calibration parameters could change for each frame (online calibration)
      _f_x = camera_calibration_matrix(0, 0);
      _f_y = camera_calibration_matrix(1, 1);
      _c_x = camera_calibration_matrix(0, 2);
      _c_y = camera_calibration_matrix(1, 2);

      Isometry3f _right_camera_in_left;
      assert(this->_platform && "TriangulatorRigidStereo::initializeBaseline|platform not present");
      _platform->getTransform(
        _right_camera_in_left, param_frame_camera_right.value(), param_frame_camera_left.value());
      _baseline_left_to_right = camera_calibration_matrix * _right_camera_in_left.translation();
      _b_x                    = _baseline_left_to_right(0);
      _baseline_set           = true;
    }
  }

  // ds explicit template specializations (add missing types here)
  template void TriangulatorRigidStereo<PointIntensityDescriptor4fVectorCloud,
                                        PointIntensityDescriptor3fVectorCloud>::compute();

  template void TriangulatorRigidStereo<PointIntensityDescriptor4fVectorCloud,
                                        PointIntensityDescriptor3fVectorCloud>::
    triangulateRectifiedMidpoint(const float& x_L_,
                                 const float& y_L_,
                                 const float& x_R_,
                                 const float& y_R_,
                                 Vector3Type& coordinates_in_camera_left_) const;

  template void
  TriangulatorRigidStereo<PointIntensityDescriptor4fVectorCloud,
                          PointIntensityDescriptor3fVectorCloud>::initializeBaseline();
} // namespace srrg2_core
