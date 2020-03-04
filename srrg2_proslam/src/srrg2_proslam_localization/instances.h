#pragma once
#include "srrg2_proslam_adaptation/instances.h"

#include "aligner_slice_2d.hpp"
#include "aligner_slice_3d.hpp"
#include "aligner_slice_projective.h"

namespace srrg2_proslam {

  void srrg2_proslam_localization_registerTypes() __attribute__((constructor));

} // namespace srrg2_proslam
