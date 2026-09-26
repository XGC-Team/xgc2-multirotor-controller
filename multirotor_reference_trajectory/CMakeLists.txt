cmake_minimum_required(VERSION 3.0.2)
project(multirotor_reference_trajectory)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  multirotor_reference_trajectory_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math REQUIRED CONFIG)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multirotor_reference_trajectory_core
  CATKIN_DEPENDS geometry_msgs multirotor_reference_trajectory_msgs nav_msgs roscpp rospy
                 std_msgs
  CFG_EXTRAS multirotor_reference_trajectory-dependencies.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## The ROS-free runtime: the reference trajectory state machine and its
## trajectory models. The library gets no ROS include directory and links no
## ROS library; -Wl,--no-undefined makes any ROS symbol it used fail the
## link. The node (the ROS edge), the tests and the harness link it.
add_library(multirotor_reference_trajectory_core
  src/multirotor_reference_trajectory_runtime.cpp
  src/state_machine/self_check_state.cpp
  src/state_machine/ready_state.cpp
  src/state_machine/planning_state.cpp
  src/state_machine/active_state.cpp
)
set_target_properties(multirotor_reference_trajectory_core PROPERTIES INCLUDE_DIRECTORIES "")
target_include_directories(multirotor_reference_trajectory_core BEFORE PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(multirotor_reference_trajectory_core
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  pthread
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_property(TARGET multirotor_reference_trajectory_core APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")
endif()

add_executable(multirotor_reference_trajectory_node
  src/multirotor_reference_trajectory_main.cpp
  src/multirotor_reference_trajectory_node.cpp
  src/input/reference_input_producer.cpp
  src/output/reference_output_consumer.cpp
)
add_dependencies(multirotor_reference_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                      ${catkin_EXPORTED_TARGETS})
target_link_libraries(multirotor_reference_trajectory_node
  multirotor_reference_trajectory_core
  xgc2_math::trajectory
  xgc2_state_machine::state_machine
  ${catkin_LIBRARIES}
  pthread
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS multirotor_reference_trajectory_core multirotor_reference_trajectory_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(multirotor_reference_trajectory_unit_test
    test/multirotor_reference_trajectory_unit_test.cpp
  )
  target_link_libraries(multirotor_reference_trajectory_unit_test
    xgc2_math::trajectory
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(multirotor_reference_trajectory_smoke_test
    test/multirotor_reference_trajectory_smoke_test.cpp
  )
  target_link_libraries(multirotor_reference_trajectory_smoke_test
    xgc2_math::trajectory
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(multirotor_reference_trajectory_runtime_test
    test/multirotor_reference_trajectory_runtime_test.cpp
  )
  add_dependencies(multirotor_reference_trajectory_runtime_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                 ${catkin_EXPORTED_TARGETS})
  target_link_libraries(multirotor_reference_trajectory_runtime_test
    multirotor_reference_trajectory_core
    xgc2_math::trajectory
    xgc2_state_machine::state_machine
    ${catkin_LIBRARIES}
    pthread
  )
endif()

# Deterministic replay harness for refactoring the runtime (test/replay).
option(REFERENCE_TRAJECTORY_REPLAY_HARNESS "Build the reference trajectory replay harness" OFF)
if(REFERENCE_TRAJECTORY_REPLAY_HARNESS)
  add_executable(${PROJECT_NAME}_replay_harness
    test/replay/reference_replay_harness.cpp
  )
  add_dependencies(${PROJECT_NAME}_replay_harness ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                                  ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_replay_harness
    multirotor_reference_trajectory_core
    xgc2_math::trajectory
    xgc2_state_machine::state_machine
    ${catkin_LIBRARIES}
    pthread
  )
endif()
