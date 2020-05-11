#pragma once
#include "aligner_slice_processor_2d.hpp"
#include "aligner_slice_processor_3d.hpp"
#include "aligner_slice_processor_projective.h"

#include "correspondence_finders/correspondence_finder_descriptor_based_bruteforce.h"
#include "correspondence_finders/correspondence_finder_descriptor_based_epipolar.h"
#include "correspondence_finders/correspondence_finder_projective_circle.h"
#include "correspondence_finders/correspondence_finder_projective_kdtree.h"
#include "correspondence_finders/correspondence_finder_projective_rhombus.h"
#include "correspondence_finders/correspondence_finder_projective_square.h"

namespace srrg2_proslam {

  void srrg2_proslam_registration_registerTypes() __attribute__((constructor));

} // namespace srrg2_proslam
