#pragma once
#include "srrg2_proslam/mapping/instances.h"
#include "srrg2_proslam/registration/instances.h"

#include "tracker_slice_processor_projective_depth.hpp"
#include "tracker_slice_processor_stereo_projective.hpp"

namespace srrg2_proslam {
  void srrg2_proslam_tracking_registerTypes() __attribute__((constructor));

} // namespace srrg2_proslam
