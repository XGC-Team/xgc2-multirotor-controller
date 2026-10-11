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
find_package(xgc2_math 0.5.12 REQUIRED CONFIG)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

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

## The aggregation module (xgc2-module module ABI): the same ROS-free core behind typed ports. Like the
## core it gets no ROS include directory and links no ROS library; -Wl,--no-undefined makes any ROS
## symbol it used fail the link. Only the module entry point is exported (module_support/exports.map).
set(MODULE_SUPPORT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../module_support/include)
add_library(px4_multirotor_reference_module MODULE module/reference_module.cpp)
set_target_properties(px4_multirotor_reference_module PROPERTIES
  INCLUDE_DIRECTORIES ""
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  INSTALL_RPATH "$ORIGIN"
)
target_include_directories(px4_multirotor_reference_module BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/module
  ${MODULE_SUPPORT_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
)
target_compile_definitions(px4_multirotor_reference_module PRIVATE
  PX4_MODULE_VERSION="${${PROJECT_NAME}_VERSION}")
target_link_libraries(px4_multirotor_reference_module PRIVATE
  multirotor_reference_trajectory_core
  ${JSONCPP_LIBRARIES}
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(px4_multirotor_reference_module PRIVATE -Wall -Wextra)
  set_property(TARGET px4_multirotor_reference_module APPEND_STRING PROPERTY LINK_FLAGS
    " -Wl,--no-undefined -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/../module_support/exports.map")
endif()

install(TARGETS px4_multirotor_reference_module
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
  add_executable(reference_payloads_layout_c test/payloads_layout.c)
  target_include_directories(reference_payloads_layout_c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set_target_properties(reference_payloads_layout_c PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES)
  add_test(NAME reference_payloads_layout_c COMMAND reference_payloads_layout_c)

  add_executable(reference_payloads_api_cpp test/payloads_api.cpp)
  target_include_directories(reference_payloads_api_cpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set_target_properties(reference_payloads_api_cpp PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)
  add_test(NAME reference_payloads_api_cpp COMMAND reference_payloads_api_cpp)

  # module_support is shared by the two modules of this repository; its tests run with this package.
  catkin_add_gtest(module_support_test ../module_support/test/owner_thread_test.cpp)
  if(TARGET module_support_test)
    target_compile_features(module_support_test PRIVATE cxx_std_17)
    target_include_directories(module_support_test BEFORE PRIVATE ${MODULE_SUPPORT_INCLUDE_DIR})
    target_link_libraries(module_support_test ${catkin_LIBRARIES} pthread)
  endif()

  # The module as a shared library, through an in-test host.
  catkin_add_gtest(reference_module_test test/reference_module_test.cpp)
  # The module against the ROS node's behavior on the scripted request stream.
  catkin_add_gtest(reference_module_parity_test test/reference_module_parity_test.cpp)
  foreach(module_test reference_module_test reference_module_parity_test)
    if(TARGET ${module_test})
      add_dependencies(${module_test} px4_multirotor_reference_module
                                      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      target_include_directories(${module_test} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/module ${CMAKE_CURRENT_SOURCE_DIR}/test ${MODULE_SUPPORT_INCLUDE_DIR})
      target_compile_definitions(${module_test} PRIVATE
        REFERENCE_MODULE_PATH="$<TARGET_FILE:px4_multirotor_reference_module>"
        REFERENCE_REPLAY_STREAM="${CMAKE_CURRENT_SOURCE_DIR}/test/replay/reference-requests.stream")
      target_link_libraries(${module_test} multirotor_reference_trajectory_core
                                           ${catkin_LIBRARIES} dl pthread)
    endif()
  endforeach()

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
