#include "instances.h"

#include <srrg2_slam_interfaces/instances.h>
#include <srrg2_slam_interfaces/registration/aligners/aligner_slice_processor_impl.cpp>
#include <srrg_config/configurable_visual_shell.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg_solver/utils/factor_graph_utils/instances.h>
#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include <srrg_solver/variables_and_factors/types_projective/instances.h>

#include "aligner_slice_processor_projective.cpp"
#include "correspondence_finders/correspondence_finder_descriptor_based_bruteforce_impl.cpp"
#include "correspondence_finders/correspondence_finder_descriptor_based_epipolar_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_base_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_circle_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_kdtree_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_rhombus_impl.cpp"
#include "correspondence_finders/correspondence_finder_projective_square_impl.cpp"
#include "srrg2_proslam/sensor_processing/instances.h"

namespace srrg2_proslam {

  // ds help the poor, confused linker
  namespace dummy {
    static AlignerSliceProcessor2D AlignerSlice2D;
    static AlignerSliceProcessor3D AlignerSlice3D;
    static AlignerSliceProcessorProjective AlignerSliceProjective __attribute__((unused));
    static AlignerSliceProcessorProjectiveDepth AlignerSliceProjectiveDepth __attribute__((unused));
    static AlignerSliceProcessorProjectiveStereo AlignerSliceStereoProjective
      __attribute__((unused));
    static AlignerSliceProcessorProjectiveWithSensor AlignerSliceProjectiveWithSensor
      __attribute__((unused));
    static AlignerSliceProcessorProjectiveDepthWithSensor AlignerSliceProjectiveDepthWithSensor
      __attribute__((unused));
    static AlignerSliceProcessorProjectiveStereoWithSensor AlignerSliceStereoProjectiveWithSensor
      __attribute__((unused));
  } // namespace dummy

  void srrg2_proslam_registration_registerTypes() {
    srrg2_solver::solver_registerTypes();
    srrg2_solver::linear_solver_registerTypes();
    srrg2_solver::variables_and_factors_2d_registerTypes();
    srrg2_solver::variables_and_factors_3d_registerTypes();
    srrg2_solver::variables_and_factors_projective_registerTypes();
    srrg2_solver::solver_utils_registerTypes();
    srrg2_slam_interfaces::srrg2_slam_interfaces_registerTypes();
    srrg2_proslam_sensor_processing_registerTypes();

    BOSS_REGISTER_CLASS(AlignerSliceProcessor2D);
    BOSS_REGISTER_CLASS(AlignerSliceProcessor3D);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorProjective);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorProjectiveDepth);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorProjectiveStereo);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorProjectiveWithSensor);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorProjectiveDepthWithSensor);
    BOSS_REGISTER_CLASS(AlignerSliceProcessorProjectiveStereoWithSensor);
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
  }

} // namespace srrg2_proslam
