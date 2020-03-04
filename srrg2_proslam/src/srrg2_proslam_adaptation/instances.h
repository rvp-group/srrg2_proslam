#pragma once
#include "intensity_feature_extractor_binned.h"
#include "intensity_feature_extractor_selective.h"

#include "correspondence_finders/correspondence_finder_descriptor_based_bruteforce.h"
#include "correspondence_finders/correspondence_finder_descriptor_based_epipolar.h"
#include "correspondence_finders/correspondence_finder_projective_circle.h"
#include "correspondence_finders/correspondence_finder_projective_kdtree.h"
#include "correspondence_finders/correspondence_finder_projective_rhombus.h"
#include "correspondence_finders/correspondence_finder_projective_square.h"

#include "measurement_adaptor_monocular_depth.h"
#include "measurement_adaptor_stereo_projective.h"

namespace srrg2_proslam {

  void srrg2_proslam_adaptation_registerTypes() __attribute__((constructor)); 

} // namespace srrg2_proslam
