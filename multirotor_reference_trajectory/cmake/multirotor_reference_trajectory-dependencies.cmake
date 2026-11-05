include(CMakeFindDependencyMacro)

find_dependency(Eigen3 CONFIG)
find_dependency(xgc2_math 0.5.12 CONFIG)
find_dependency(xgc2_state_machine CONFIG)

list(APPEND multirotor_reference_trajectory_INCLUDE_DIRS ${XGC2_STATE_MACHINE_INCLUDE_DIRS})
list(REMOVE_DUPLICATES multirotor_reference_trajectory_INCLUDE_DIRS)

list(APPEND multirotor_reference_trajectory_LIBRARIES
  Eigen3::Eigen
  xgc2_math::trajectory
)
list(REMOVE_DUPLICATES multirotor_reference_trajectory_LIBRARIES)
