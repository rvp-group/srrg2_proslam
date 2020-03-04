#pragma once
#include "srrg2_proslam_localization/instances.h"
#include "srrg2_proslam_mapping/instances.h"

#include "tracker_slice_projective_depth.hpp"
#include "tracker_slice_stereo_projective.hpp"

namespace srrg2_proslam {
  void srrg2_proslam_tracking_registerTypes() __attribute__((constructor));

} // namespace srrg2_proslam
