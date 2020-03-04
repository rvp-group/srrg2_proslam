#include "instances.h"

#include <srrg_slam_interfaces/instances.h>
#include <srrg_slam_interfaces/multi_aligner_slice_impl.cpp>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg_solver/utils/factor_graph_utils/instances.h>
#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include <srrg_solver/variables_and_factors/types_projective/instances.h>

#include "aligner_slice_projective.cpp"

namespace srrg2_proslam {

  // ds help the poor, confused linker
  namespace dummy {
    static AlignerSlice2D AlignerSlice2D;
    static AlignerSlice3D AlignerSlice3D;
    static AlignerSliceProjective AlignerSliceProjective __attribute__((unused));
    static AlignerSliceProjectiveDepth AlignerSliceProjectiveDepth __attribute__((unused));
    static AlignerSliceStereoProjective AlignerSliceStereoProjective __attribute__((unused));
    static AlignerSliceProjectiveWithSensor AlignerSliceProjectiveWithSensor
      __attribute__((unused));
    static AlignerSliceProjectiveDepthWithSensor AlignerSliceProjectiveDepthWithSensor
      __attribute__((unused));
    static AlignerSliceStereoProjectiveWithSensor AlignerSliceStereoProjectiveWithSensor
      __attribute__((unused));
  } // namespace dummy

  void srrg2_proslam_localization_registerTypes() {
    srrg2_solver::solver_registerTypes();
    srrg2_solver::linear_solver_registerTypes();
    srrg2_solver::registerTypes2D();
    srrg2_solver::registerTypes3D();
    srrg2_solver::projective_registerTypes();
    srrg2_solver::solver_utils_registerTypes();
    srrg2_slam_interfaces::slam_interfaces_registerTypes();
    srrg2_proslam_adaptation_registerTypes();

    BOSS_REGISTER_CLASS(AlignerSlice2D);
    BOSS_REGISTER_CLASS(AlignerSlice3D);
    BOSS_REGISTER_CLASS(AlignerSliceProjective);
    BOSS_REGISTER_CLASS(AlignerSliceProjectiveDepth);
    BOSS_REGISTER_CLASS(AlignerSliceStereoProjective);
    BOSS_REGISTER_CLASS(AlignerSliceProjectiveWithSensor);
    BOSS_REGISTER_CLASS(AlignerSliceProjectiveDepthWithSensor);
    BOSS_REGISTER_CLASS(AlignerSliceStereoProjectiveWithSensor);
  }

} // namespace srrg2_proslam
