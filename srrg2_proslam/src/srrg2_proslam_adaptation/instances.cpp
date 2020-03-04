#include "instances.h"

#include <srrg_pcl/instances.h>

#include "correspondence_finders/correspondence_finder_descriptor_based_bruteforce_impl.cpp"
#include "correspondence_finders/correspondence_finder_descriptor_based_epipolar_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_base_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_circle_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_kdtree_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_rhombus_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_square_impl.cpp"

namespace srrg2_proslam {
  using namespace srrg2_slam_interfaces;

  void srrg2_proslam_adaptation_registerTypes() {
    point_cloud_registerTypes();

    BOSS_REGISTER_CLASS(IntensityFeatureExtractorBinned2D);
    BOSS_REGISTER_CLASS(IntensityFeatureExtractorBinned3D);
    BOSS_REGISTER_CLASS(IntensityFeatureExtractorSelective2D);
    BOSS_REGISTER_CLASS(IntensityFeatureExtractorSelective3D);

    BOSS_REGISTER_CLASS(CorrespondenceFinderDescriptorBasedBruteforce2D2D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderDescriptorBasedBruteforce2D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderDescriptorBasedBruteforce3D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderDescriptorBasedBruteforce4D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderDescriptorBasedEpipolar2D2D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderDescriptorBasedEpipolar3D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveKDTree2D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveKDTree3D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveKDTree4D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveSquare2D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveSquare3D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveSquare4D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveCircle2D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveCircle3D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveCircle4D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveRhombus2D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveRhombus3D3D);
    BOSS_REGISTER_CLASS(CorrespondenceFinderProjectiveRhombus4D3D);

    BOSS_REGISTER_CLASS(MeasurementAdaptorStereoProjective);
    BOSS_REGISTER_CLASS(MeasurementAdaptorMonocularDepth);
  }

} // namespace srrg2_proslam
