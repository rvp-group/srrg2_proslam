#include "measurement_adaptor_stereo_projective.h"

namespace srrg2_slam_interfaces {

  bool MeasurementAdaptorStereoProjective::setMeasurement(BaseSensorMessagePtr measurement_) {
    PROFILE_TIME("MeasurementAdaptorStereoProjective::setMeasurement");
    BaseType::setMeasurement(measurement_);
    _image_message_left  = nullptr;
    _image_message_right = nullptr;
    _status              = Error;

    // ds measurement must be a pack of exactly two images (stereo) - fails also for nullptr
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(measurement_);
    if (!message_pack) {
      throw std::runtime_error("MeasurementAdaptorStereoProjective::setMeasurement|received "
                               "invalid message (not a message pack)");
    }
    if (message_pack->messages.size() < 2) {
      throw std::runtime_error("MeasurementAdaptorStereoProjective::setMeasurement|received "
                               "invalid message pack (size: " +
                               std::to_string(message_pack->messages.size()) + " != 2)");
    }

    // ds check left and right image message types - TODO softcode order?
    _image_message_left = srrg2_slam_interfaces::extractMessage<srrg2_core::ImageMessage>(
      measurement_, param_topic_camera_left.value());
    if (!_image_message_left) {
      throw std::runtime_error("MeasurementAdaptorStereoProjective::setMeasurement|ERROR, "
                               "ImageMessage not found - topic [ " +
                               param_topic_camera_left.value() + " ]");
    }

    _image_message_right = srrg2_slam_interfaces::extractMessage<srrg2_core::ImageMessage>(
      measurement_, param_topic_camera_right.value());
    if (!_image_message_right) {
      throw std::runtime_error("MeasurementAdaptorStereoProjective::setMeasurement|ERROR, "
                               "ImageMessage not found - topic [ " +
                               param_topic_camera_right.value() + " ]");
    }

    _status = Initializing;
    return true;
  }

  void MeasurementAdaptorStereoProjective::compute() {
    PROFILE_TIME("MeasurementAdaptorStereoProjective::compute");
    _status = Error;
    if (!_measurement || !_image_message_left || !_image_message_right) {
      throw std::runtime_error("MeasurementAdaptorStereoProjective::compute|measurement not set");
    }

    // ds if measurement hasn't been changed ignore the invocation
    if (!_measurement_changed_flag) {
      _status = Ready;
      return;
    }
    _dest->clear();

    // ds load image data from disk - can take some time so we measure it
    const BaseImage* image_left  = nullptr;
    const BaseImage* image_right = nullptr;
    {
      PROFILE_TIME("MeasurementAdaptorStereoProjective::loadImageFromDisk");
      image_left  = _image_message_left->image();
      image_right = _image_message_right->image();
      assert(image_left && image_right);
    }

    // ds detect features in both images
    PointIntensityDescriptor3fVectorCloud features_left;
    PointIntensityDescriptor3fVectorCloud features_right;
    {
      PROFILE_TIME("MeasurementAdaptorStereoProjective::extractFeatures");

      // ds compute features in left camera frame
      param_feature_extractor->setFeatures(&features_left);
      param_feature_extractor->setProjections(_projections, _projection_radius);
      param_feature_extractor->compute(image_left);

      // ds compute features in right camera frame (can be the same extractor instance)
      // ds switch to fine-grained detection for triangulation, using left points as priors
      param_feature_extractor_right->setFeatures(&features_right);
      param_feature_extractor_right->setProjections(&features_left, _projection_radius);
      param_feature_extractor_right->compute(image_right);
      _projections = nullptr;
    }

    // ds consistency
    assert(image_left->rows() == param_feature_extractor->imageRows());
    assert(image_left->cols() == param_feature_extractor->imageCols());
    assert(image_left->rows() == param_feature_extractor_right->imageRows());
    assert(image_left->cols() == param_feature_extractor_right->imageCols());

    // ds compute stereo matches
    CorrespondenceVector stereo_matches;
    {
      PROFILE_TIME("MeasurementAdaptorStereoProjective::computeMatches");
      param_correspondence_finder->setFixed(&features_left);
      param_correspondence_finder->setMoving(&features_right);
      param_correspondence_finder->setCorrespondences(&stereo_matches);
      param_correspondence_finder->compute();
    }

    // ds assemble stereo matches to Point4f (apapted measurement)
    // ds TODO remove the need for this costly copying and pasting
    _dest->reserve(stereo_matches.size());
    for (const Correspondence& stereo_match : stereo_matches) {
      PointIntensityDescriptor4f stereo_point;
      stereo_point.coordinates()(0) = features_left[stereo_match.fixed_idx].coordinates()(0);
      stereo_point.coordinates()(1) = features_left[stereo_match.fixed_idx].coordinates()(1);
      stereo_point.coordinates()(2) = features_right[stereo_match.moving_idx].coordinates()(0);
      stereo_point.coordinates()(3) = features_right[stereo_match.moving_idx].coordinates()(1);

      // ds TODO check what information to keep, do it recursively over templates
      stereo_point.descriptor() = features_left[stereo_match.fixed_idx].descriptor();
      stereo_point.intensity()  = features_left[stereo_match.fixed_idx].intensity();

      // ds evaluate disparity
      const float horizontal_disparity =
        stereo_point.coordinates()(0) - stereo_point.coordinates()(2);
      const float vertical_disparity =
        stereo_point.coordinates()(1) - stereo_point.coordinates()(3);

      // ds if a bruteforce matcher is used, invalid disparities can be obtained
      if (horizontal_disparity < 0 || vertical_disparity < 0) {
        continue;
      }

      // ds store point
      _dest->emplace_back(stereo_point);
    }
    _status = Ready;
  }

} // namespace srrg2_slam_interfaces
