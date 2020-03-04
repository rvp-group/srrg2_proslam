add_executable(benchmark_icl benchmark_icl.cpp)
target_compile_definitions(benchmark_icl PRIVATE CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(benchmark_icl
  srrg2_proslam_tracking_library
  ${catkin_LIBRARIES}
)

add_executable(benchmark_tum benchmark_tum.cpp)
target_compile_definitions(benchmark_tum PRIVATE CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(benchmark_tum
  srrg2_proslam_tracking_library
  ${catkin_LIBRARIES}
)

add_executable(benchmark_kitti benchmark_kitti.cpp)
target_compile_definitions(benchmark_kitti PRIVATE CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(benchmark_kitti
  srrg2_proslam_tracking_library
  ${catkin_LIBRARIES}
)

add_executable(benchmark_malaga benchmark_malaga.cpp)
target_compile_definitions(benchmark_malaga PRIVATE CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(benchmark_malaga
  srrg2_proslam_tracking_library
  ${catkin_LIBRARIES}
)

add_executable(benchmark_euroc benchmark_euroc.cpp)
target_compile_definitions(benchmark_euroc PRIVATE CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(benchmark_euroc
  srrg2_proslam_tracking_library
  ${catkin_LIBRARIES}
)
