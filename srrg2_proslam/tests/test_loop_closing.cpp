#include "fixtures.hpp"

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_, true /*use test folder*/);
}

// ds utility function for mocking closure addition to the graph
template <typename ClosureContainer_>
static void addClosuresToGraph(ClosureContainer_ closures_, FactorGraphPtr graph_) {
  if (closures_.empty()) {
    return;
  }
  for (auto lc : closures_) {
    graph_->addFactor(lc);
  }
  graph_->bindFactors();
}

TEST_F(KITTI, MultiLoopDetectorHBST3D_FASTORB00To00) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds load loop detector and surroundings from existing configuration
  // ds TODO leaking as manager is still broken
  ConfigurableManager* manager = new ConfigurableManager();
  manager->read(_filepath_configurations + "/" + "kitti.conf");
  MultiGraphSLAM3DPtr slammer = manager->getByName<MultiGraphSLAM3D>("slam");
  ASSERT_NOTNULL(slammer);
  MultiAligner3DQRPtr aligner = manager->getByName<MultiAligner3DQR>("loop_aligner");
  ASSERT_NOTNULL(aligner);
  MultiLoopDetectorHBST3DPtr loop_detector =
    manager->getByName<MultiLoopDetectorHBST3D>("loop_detector");
  ASSERT_NOTNULL(loop_detector);
  loop_detector->setSLAMAlgorithm(slammer.get() /* uagh*/);

  // ds enable loop closing after each local map
  loop_detector->param_minimum_age_difference_to_candidates.setValue(0);

  // ds populate a local map
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  LocalMap3D local_map_00_copy;
  local_map_00_copy.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));

  // ds allocate an empty graph and populate it with the local maps
  FactorGraphPtr graph(new FactorGraph());
  slammer->setGraph(graph);
  graph->addVariable(VariableBasePtr(&local_map_00));
  graph->addVariable(VariableBasePtr(&local_map_00_copy));
  ASSERT_EQ(graph->variables().size(), 2);

  // ds compute closures for first local map
  slammer->setCurrentLocalMap(&local_map_00);
  loop_detector->compute(); // initial query (no reference candidates available)
  ASSERT_EQ(loop_detector->indices().size(), 0);
  loop_detector->addPreviousQuery();

  // ds compute closures for second local map
  slammer->setCurrentLocalMap(&local_map_00_copy);
  loop_detector->compute(); // single reference candidate (previous query) available
  ASSERT_EQ(loop_detector->detectedClosures().size(), 1);
  loop_detector->addPreviousQuery();

  // ds verify computed loop closure
  addClosuresToGraph(loop_detector->detectedClosures(), graph);
  const LoopClosure3DPtr closure = loop_detector->detectedClosures().back();
  ASSERT_LT(geometry3d::t2tnq(closure->measurement()).norm(), 1e-5);
  ASSERT_EQ(closure->correspondences.size(), closure->num_correspondences);
  ASSERT_EQ(closure->num_correspondences, 145);
}

TEST_F(KITTI, MultiLoopDetectorHBST3D_FASTORB00To01) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds load loop detector and surroundings from existing configuration
  // ds TODO leaking as manager is still broken
  ConfigurableManager* manager = new ConfigurableManager();
  manager->read(_filepath_configurations + "/" + "kitti.conf");
  MultiGraphSLAM3DPtr slammer = manager->getByName<MultiGraphSLAM3D>("slam");
  ASSERT_NOTNULL(slammer);
  MultiAligner3DQRPtr aligner = manager->getByName<MultiAligner3DQR>("loop_aligner");
  ASSERT_NOTNULL(aligner);
  MultiLoopDetectorHBST3DPtr loop_detector =
    manager->getByName<MultiLoopDetectorHBST3D>("loop_detector");
  ASSERT_NOTNULL(loop_detector);
  loop_detector->setSLAMAlgorithm(slammer.get() /* uagh*/);

  // ds enable loop closing after each local map
  loop_detector->param_minimum_age_difference_to_candidates.setValue(0);
  loop_detector->param_maximum_descriptor_distance.setValue(25);
  loop_detector->param_relocalize_min_inliers.setValue(25);

  // ds populate local maps
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  LocalMap3D local_map_01;
  local_map_01.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_01));

  // ds allocate an empty graph
  FactorGraphPtr graph(new FactorGraph());
  slammer->setGraph(graph);

  // ds add local maps to graph
  graph->addVariable(VariableBasePtr(&local_map_00));
  graph->addVariable(VariableBasePtr(&local_map_01));
  ASSERT_EQ(graph->variables().size(), 2);

  // ds compute closures for first local map
  slammer->setCurrentLocalMap(&local_map_00);
  loop_detector->compute(); // initial query (no reference candidates available)
  ASSERT_EQ(loop_detector->indices().size(), 0);
  loop_detector->addPreviousQuery();

  // ds compute closures for second local map
  slammer->setCurrentLocalMap(&local_map_01);
  loop_detector->compute(); // single reference candidate (previous query) available
  ASSERT_EQ(loop_detector->detectedClosures().size(), 1);
  loop_detector->addPreviousQuery();

  // ds verify computed loop closure
  addClosuresToGraph(loop_detector->detectedClosures(), graph);
  const LoopClosure3DPtr closure = loop_detector->detectedClosures().back();
  ASSERT_EQ(closure->correspondences.size(), closure->num_correspondences);
  ASSERT_EQ(closure->num_correspondences, 45);
  const Vector6f error = geometry3d::t2tnq(closure->measurement() * camera_01_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.2);
  ASSERT_LT_ABS(error(1), 0.2);
  ASSERT_LT_ABS(error(2), 0.5);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(ICL, MultiLoopDetectorHBST3D_FASTORB00To01) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds load loop detector and surroundings from existing configuration
  // ds TODO leaking as manager is still broken
  ConfigurableManager* manager = new ConfigurableManager();
  manager->read(_filepath_configurations + "/" + "icl.conf");
  MultiGraphSLAM3DPtr slammer = manager->getByName<MultiGraphSLAM3D>("slam");
  ASSERT_NOTNULL(slammer);
  MultiAligner3DQRPtr aligner = manager->getByName<MultiAligner3DQR>("loop_aligner");
  ASSERT_NOTNULL(aligner);
  MultiLoopDetectorHBST3DPtr loop_detector =
    manager->getByName<MultiLoopDetectorHBST3D>("loop_detector");
  ASSERT_NOTNULL(loop_detector);
  loop_detector->setSLAMAlgorithm(slammer.get() /* uagh*/);

  // ds enable loop closing after each local map
  loop_detector->param_minimum_age_difference_to_candidates.setValue(0);

  // ds populate local maps
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  LocalMap3D local_map_01;
  local_map_01.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_01));

  // ds allocate an empty graph
  FactorGraphPtr graph(new FactorGraph());
  slammer->setGraph(graph);

  // ds add local maps to graph
  graph->addVariable(VariableBasePtr(&local_map_00));
  graph->addVariable(VariableBasePtr(&local_map_01));
  ASSERT_EQ(graph->variables().size(), 2);

  // ds compute closures for first local map
  slammer->setCurrentLocalMap(&local_map_00);
  loop_detector->compute(); // initial query (no reference candidates available)
  ASSERT_EQ(loop_detector->indices().size(), 0);
  loop_detector->addPreviousQuery();

  // ds compute closures for second local map
  slammer->setCurrentLocalMap(&local_map_01);
  loop_detector->compute(); // single reference candidate (previous query) available
  ASSERT_EQ(loop_detector->detectedClosures().size(), 1);
  loop_detector->addPreviousQuery();

  // ds verify computed loop closure
  addClosuresToGraph(loop_detector->detectedClosures(), graph);
  const LoopClosure3DPtr closure = loop_detector->detectedClosures().back();
  ASSERT_EQ(closure->correspondences.size(), closure->num_correspondences);
  ASSERT_EQ(closure->num_correspondences, 206);
  const Vector6f error = geometry3d::t2tnq(closure->measurement() * camera_01_in_00);
  std::cerr << "error (manifold): " << error.transpose() << std::endl;
  ASSERT_LT_ABS(error(0), 0.05);
  ASSERT_LT_ABS(error(1), 0.05);
  ASSERT_LT_ABS(error(2), 0.05);
  ASSERT_LT_ABS(error(3), 0.01);
  ASSERT_LT_ABS(error(4), 0.01);
  ASSERT_LT_ABS(error(5), 0.01);
}

TEST_F(ICL, MultiLoopDetectorHBST3D_FASTORB00To01And50) {
  using PropertyDescriptors = Property_<PointIntensityDescriptor3fVectorCloud*>;

  // ds load loop detector and surroundings from existing configuration
  // ds TODO leaking as manager is still broken
  ConfigurableManager* manager = new ConfigurableManager();
  manager->read(_filepath_configurations + "/" + "icl.conf");
  MultiGraphSLAM3DPtr slammer = manager->getByName<MultiGraphSLAM3D>("slam");
  ASSERT_NOTNULL(slammer);
  MultiAligner3DQRPtr aligner = manager->getByName<MultiAligner3DQR>("loop_aligner");
  ASSERT_NOTNULL(aligner);
  MultiLoopDetectorHBST3DPtr loop_detector =
    manager->getByName<MultiLoopDetectorHBST3D>("loop_detector");
  ASSERT_NOTNULL(loop_detector);
  loop_detector->setSLAMAlgorithm(slammer.get() /* uagh*/);

  // ds enable loop closing after each local map
  loop_detector->param_minimum_age_difference_to_candidates.setValue(0);
  loop_detector->param_maximum_descriptor_distance.setValue(75);

  // ds populate local maps
  LocalMap3D local_map_00;
  local_map_00.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_00));
  LocalMap3D local_map_01;
  local_map_01.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_01));
  LocalMap3D local_map_50;
  local_map_50.dynamic_properties.addProperty(
    new PropertyDescriptors("points", "points", nullptr, &points_in_camera_50));

  // ds populate graph
  FactorGraphPtr graph(new FactorGraph());
  slammer->setGraph(graph);
  graph->addVariable(VariableBasePtr(&local_map_00));
  graph->addVariable(VariableBasePtr(&local_map_01));
  graph->addVariable(VariableBasePtr(&local_map_50));
  ASSERT_EQ(graph->variables().size(), 3);

  // ds compute closures for first local map
  slammer->setCurrentLocalMap(&local_map_00);
  loop_detector->compute(); // initial query (no reference candidates available)
  ASSERT_EQ(loop_detector->indices().size(), 0);
  loop_detector->addPreviousQuery();

  // ds compute closures for second local map35
  slammer->setCurrentLocalMap(&local_map_01);
  loop_detector->compute(); // single reference candidate (previous query) available
  ASSERT_EQ(loop_detector->detectedClosures().size(), 1);
  loop_detector->addPreviousQuery();

  // ds compute closures for second local map
  slammer->setCurrentLocalMap(&local_map_50);
  loop_detector->compute(); // two reference candidates available
  ASSERT_EQ(loop_detector->detectedClosures().size(), 2);
  loop_detector->addPreviousQuery();

  // ds verify computed loop closures - must be ordered in increasing graph id
  addClosuresToGraph(loop_detector->detectedClosures(), graph);
  const LoopClosure3DPtr closure_0050 = loop_detector->detectedClosures()[0];
  ASSERT_EQ(closure_0050->source()->graphId(), 2 /*50*/);
  ASSERT_EQ(closure_0050->target()->graphId(), 0);
  ASSERT_EQ(closure_0050->correspondences.size(), closure_0050->num_correspondences);
  ASSERT_EQ(closure_0050->num_correspondences, 139);
  const Vector6f error_0050 = geometry3d::t2tnq(closure_0050->measurement() * camera_50_in_00);
  ASSERT_LT_ABS(error_0050(0), 0.1);
  ASSERT_LT_ABS(error_0050(1), 0.1);
  ASSERT_LT_ABS(error_0050(2), 0.1);
  ASSERT_LT_ABS(error_0050(3), 0.05);
  ASSERT_LT_ABS(error_0050(4), 0.05);
  ASSERT_LT_ABS(error_0050(5), 0.05);
  const LoopClosure3DPtr closure_0150 = loop_detector->detectedClosures()[1];
  ASSERT_EQ(closure_0150->source()->graphId(), 2 /*50*/);
  ASSERT_EQ(closure_0150->target()->graphId(), 1);
  ASSERT_EQ(closure_0150->correspondences.size(), closure_0150->num_correspondences);
  ASSERT_EQ(closure_0150->num_correspondences, 141);
  const Vector6f error_0051 = geometry3d::t2tnq(closure_0150->measurement() * camera_50_in_01);
  ASSERT_LT_ABS(error_0051(0), 0.1);
  ASSERT_LT_ABS(error_0051(1), 0.1);
  ASSERT_LT_ABS(error_0051(2), 0.1);
  ASSERT_LT_ABS(error_0051(3), 0.05);
  ASSERT_LT_ABS(error_0051(4), 0.05);
  ASSERT_LT_ABS(error_0051(5), 0.05);
}
