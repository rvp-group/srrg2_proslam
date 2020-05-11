#pragma once
#include <unordered_set>

#include <srrg2_slam_interfaces/registration/correspondence_finder.h>
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/platform.h>
#include <srrg_pcl/camera_matrix_owner.h>
#include <srrg_pcl/point_intensity_descriptor.h>
#include <srrg_property/property.h>
#include <srrg_property/property_eigen.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_proslam {

  template <class MeasurementType_, class DestType_>
  class TriangulatorRigidStereo : public srrg2_core::Configurable,
                                  public srrg2_core::PlatformUser,
                                  public srrg2_core::Profiler {
    using ScalarType  = typename MeasurementType_::PointType::Scalar;
    using ThisType    = TriangulatorRigidStereo<MeasurementType_, DestType_>;
    using BaseType    = srrg2_core::Configurable;
    using Vector2Type = Eigen::Matrix<ScalarType, 2, 1>;
    // TODO fix point_cloud.h for using VectorType2 = typename MeasurementType_::PlainVectorType;
    using PointArrayType = MeasurementType_;
    using Vector3Type    = Eigen::Matrix<ScalarType, 3, 1>;
    // TODO fix point_cloud.h for using VectorType3 = typename DestType_::PlainVectorType;
    using PointType                   = typename DestType_::PointType;
    using PointCloudType              = DestType_;
    using ProjectorType               = srrg2_core::CameraMatrixOwner_<PointType::GeometryDim>;
    using CameraCalibrationMatrixType = typename ProjectorType::CameraMatrixType;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(srrg2_core::PropertyFloat,
          minimum_disparity_pixels,
          "minimum required disparity before attempting triangulation",
          1,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          infinity_depth_meters,
          "depth value of a point with zero disparity",
          std::sqrt(std::numeric_limits<ScalarType>::max()),
          nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<ProjectorType>,
          projector,
          "link to a projector where to take the infor for the factor",
          nullptr,
          nullptr);
    PARAM(srrg2_core::PropertyString,
          frame_camera_left,
          "topic for the camera left info",
          "camera_left",
          nullptr);
    PARAM(srrg2_core::PropertyString,
          frame_camera_right,
          "topic for the camera right info",
          "camera_right",
          nullptr);

    //! target result buffer
    void setDest(PointCloudType* dest_) {
      _dest = dest_;
    }

    void setMoving(const PointArrayType* stereo_intensity_matches_) {
      _stereo_intensity_matches = stereo_intensity_matches_;
    }

    //! triangulates current set of correspondences in the left camera frame
    //! dest has the same size as moving (matches), with .status = Invalid for invalid points
    void compute();

    //! point-wise triangulation method TODO make class generic and multiple methods available
    void triangulateRectifiedMidpoint(const ScalarType& x_L_,
                                      const ScalarType& y_L_,
                                      const ScalarType& x_R_,
                                      const ScalarType& y_R_,
                                      Vector3Type& coordinates_in_camera_left_) const;

    //! invalidated measurements
    const std::unordered_set<size_t>& indicesInvalidated() const {
      return _indices_invalidated;
    }

    //! initialize baseline based on set platform and camera matrix (called in compute)
    void initializeBaseline();

    //! retrieve current stereo baseline
    const Vector3Type& baselineRigthInLeft() const {
      return _baseline_right_in_left;
    }

  protected:
    //! selected camera calibration parameters (to avoid caching them all over the place)
    ScalarType _b_x = 0;
    ScalarType _f_x = 0;
    ScalarType _f_y = 0;
    ScalarType _c_x = 0;
    ScalarType _c_y = 0;

    //! currently active correspondences and data (TODO cleanup template)
    const PointArrayType* _stereo_intensity_matches = nullptr;

    //! current triangulated points (TODO cleanup template)
    PointCloudType* _dest = nullptr;

    //! invalidated measurements
    std::unordered_set<size_t> _indices_invalidated;

    //! cache the baseline measurement from platform
    Vector3Type _baseline_right_in_left = Vector3Type::Zero();
    bool _baseline_set                  = false;
  };

  using TriangulatorRigidStereoDescriptors =
    TriangulatorRigidStereo<srrg2_core::PointIntensityDescriptor4fVectorCloud,
                            srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using TriangulatorRigidStereoDescriptorsPtr = std::shared_ptr<TriangulatorRigidStereoDescriptors>;
} // namespace srrg2_proslam
