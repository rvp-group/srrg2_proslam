add_executable(test_unproject_rgbd_node test_unproject_rgbd_node.cpp)
target_link_libraries(test_unproject_rgbd_node
  ${catkin_LIBRARIES}
)

add_executable(test_unproject_rigid_stereo_node test_unproject_rigid_stereo_node.cpp)
target_link_libraries(test_unproject_rigid_stereo_node
  ${catkin_LIBRARIES}
)

