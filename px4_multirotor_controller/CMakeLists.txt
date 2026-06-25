cmake_minimum_required(VERSION 3.0.2)
project(px4_multirotor_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
set(MULTIROTOR_CONTROLLER_CATKIN_COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  ros1_utils
  sensor_msgs
  mavros_msgs
  multirotor_reference_trajectory
  hover_thrust_estimator
  estimator_vrpn_px4_rotor_state
)

find_package(catkin REQUIRED COMPONENTS
  ${MULTIROTOR_CONTROLLER_CATKIN_COMPONENTS}
)
find_package(xgc2_acados REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math REQUIRED CONFIG)

set(MULTIROTOR_HOVER_THRUST_INCLUDE_DIRS ${hover_thrust_estimator_INCLUDE_DIRS})
if(hover_thrust_estimator_DIR)
  get_filename_component(MULTIROTOR_HOVER_THRUST_PREFIX
    "${hover_thrust_estimator_DIR}/../../.." ABSOLUTE)
  if(EXISTS "${MULTIROTOR_HOVER_THRUST_PREFIX}/include")
    list(PREPEND MULTIROTOR_HOVER_THRUST_INCLUDE_DIRS
      "${MULTIROTOR_HOVER_THRUST_PREFIX}/include")
  endif()
endif()
if(MULTIROTOR_HOVER_THRUST_INCLUDE_DIRS)
  list(REMOVE_DUPLICATES MULTIROTOR_HOVER_THRUST_INCLUDE_DIRS)
endif()

## Find Eigen3 (required for MpcTrajectoryState)
find_package(Eigen3 REQUIRED)

function(multirotor_append_existing_paths output_var)
  set(_paths "${${output_var}}")
  foreach(_path IN LISTS ARGN)
    if(_path)
      list(APPEND _paths "${_path}")
    endif()
  endforeach()
  if(_paths)
    list(REMOVE_DUPLICATES _paths)
  endif()
  set(${output_var} "${_paths}" PARENT_SCOPE)
endfunction()

if(COMMAND xgc2_acados_require)
  xgc2_acados_require()
endif()

set(MULTIROTOR_ACADOS_INCLUDE_DIRS ${XGC2_ACADOS_INCLUDE_DIRS})
set(MULTIROTOR_ACADOS_LIBRARIES ${XGC2_ACADOS_LIBRARIES})
set(MULTIROTOR_ACADOS_LIBRARY_DIRS ${XGC2_ACADOS_LIBRARY_DIRS})
set(MULTIROTOR_ACADOS_PYTHONPATH
  ${XGC2_ACADOS_PYTHONPATH}
  ${XGC2_ACADOS_PYTHON_PATH}
)
set(MULTIROTOR_ACADOS_SOURCE_DIR "${XGC2_ACADOS_SOURCE_DIR}")
set(MULTIROTOR_ACADOS_INSTALL_DIR "${XGC2_ACADOS_INSTALL_DIR}")
if(NOT MULTIROTOR_ACADOS_SOURCE_DIR AND EXISTS "/opt/xgc2/acados/interfaces/acados_template")
  set(MULTIROTOR_ACADOS_SOURCE_DIR "/opt/xgc2/acados")
endif()
if(NOT MULTIROTOR_ACADOS_INSTALL_DIR AND EXISTS "/opt/xgc2/acados/lib")
  set(MULTIROTOR_ACADOS_INSTALL_DIR "/opt/xgc2/acados")
endif()
set(ACADOS_SOURCE_DIR "${MULTIROTOR_ACADOS_SOURCE_DIR}")
set(ACADOS_INSTALL_DIR "${MULTIROTOR_ACADOS_INSTALL_DIR}")
if(ACADOS_SOURCE_DIR)
  multirotor_append_existing_paths(MULTIROTOR_ACADOS_INCLUDE_DIRS
    "${ACADOS_SOURCE_DIR}"
    "${ACADOS_SOURCE_DIR}/interfaces"
  )
  multirotor_append_existing_paths(MULTIROTOR_ACADOS_PYTHONPATH
    "${ACADOS_SOURCE_DIR}/interfaces/acados_template"
  )
endif()
if(ACADOS_INSTALL_DIR)
  multirotor_append_existing_paths(MULTIROTOR_ACADOS_INCLUDE_DIRS
    "${ACADOS_INSTALL_DIR}/include"
    "${ACADOS_INSTALL_DIR}/include/hpipm/include"
    "${ACADOS_INSTALL_DIR}/include/blasfeo/include"
  )
  multirotor_append_existing_paths(MULTIROTOR_ACADOS_LIBRARY_DIRS
    "${ACADOS_INSTALL_DIR}/lib"
  )
endif()
foreach(_acados_library IN LISTS MULTIROTOR_ACADOS_LIBRARIES)
  if(IS_ABSOLUTE "${_acados_library}")
    get_filename_component(_acados_library_dir "${_acados_library}" DIRECTORY)
    multirotor_append_existing_paths(MULTIROTOR_ACADOS_LIBRARY_DIRS "${_acados_library_dir}")
  endif()
endforeach()
if(MULTIROTOR_ACADOS_INCLUDE_DIRS)
  list(REMOVE_DUPLICATES MULTIROTOR_ACADOS_INCLUDE_DIRS)
endif()
if(MULTIROTOR_ACADOS_LIBRARIES)
  list(REMOVE_DUPLICATES MULTIROTOR_ACADOS_LIBRARIES)
endif()
if(MULTIROTOR_ACADOS_LIBRARY_DIRS)
  list(REMOVE_DUPLICATES MULTIROTOR_ACADOS_LIBRARY_DIRS)
endif()
if(MULTIROTOR_ACADOS_PYTHONPATH)
  list(REMOVE_DUPLICATES MULTIROTOR_ACADOS_PYTHONPATH)
endif()
string(REPLACE ";" ":" MULTIROTOR_ACADOS_LIBRARY_PATH_ENV "${MULTIROTOR_ACADOS_LIBRARY_DIRS}")
string(REPLACE ";" ":" MULTIROTOR_ACADOS_PYTHONPATH_ENV "${MULTIROTOR_ACADOS_PYTHONPATH}")

set(MULTIROTOR_UAV_NMPC_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated/nmpc/uav_nmpc")
set(MULTIROTOR_UAV_NMPC_TOOL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools")
set(MULTIROTOR_UAV_NMPC_SOLVER_C "${MULTIROTOR_UAV_NMPC_GENERATED_DIR}/acados_solver_uav_nmpc.c")
set(MULTIROTOR_UAV_NMPC_SOLVER_H "${MULTIROTOR_UAV_NMPC_GENERATED_DIR}/acados_solver_uav_nmpc.h")
set(MULTIROTOR_UAV_NMPC_HORIZON "1.0")
set(MULTIROTOR_UAV_NMPC_STEPS "10")
set(MULTIROTOR_UAV_NMPC_GENERATOR_INPUTS
  "${MULTIROTOR_UAV_NMPC_TOOL_PATH}/nmpc/controller.py"
  "${MULTIROTOR_UAV_NMPC_TOOL_PATH}/nmpc/dynamics.py"
  "${MULTIROTOR_UAV_NMPC_TOOL_PATH}/nmpc/export_ros_generated_code.py"
  "${MULTIROTOR_UAV_NMPC_TOOL_PATH}/nmpc/math_utils.py"
  "${MULTIROTOR_UAV_NMPC_TOOL_PATH}/nmpc/references.py"
)
set(MULTIROTOR_UAV_NMPC_PYTHON_CACHE_DIR "${CATKIN_DEVEL_PREFIX}/.cache/px4_multirotor_controller/pycache")
set(MULTIROTOR_UAV_NMPC_MPL_CACHE_DIR "${CATKIN_DEVEL_PREFIX}/.cache/px4_multirotor_controller/matplotlib")

set(MULTIROTOR_UAV_NMPC_REGENERATE FALSE)
if(NOT EXISTS "${MULTIROTOR_UAV_NMPC_SOLVER_C}" OR NOT EXISTS "${MULTIROTOR_UAV_NMPC_SOLVER_H}")
  set(MULTIROTOR_UAV_NMPC_REGENERATE TRUE)
else()
  file(READ "${MULTIROTOR_UAV_NMPC_SOLVER_H}" MULTIROTOR_UAV_NMPC_SOLVER_HEADER)
  string(FIND "${MULTIROTOR_UAV_NMPC_SOLVER_HEADER}" "#define UAV_NMPC_N      ${MULTIROTOR_UAV_NMPC_STEPS}" MULTIROTOR_UAV_NMPC_STEPS_MATCH)
  string(FIND "${MULTIROTOR_UAV_NMPC_SOLVER_HEADER}" "#define UAV_NMPC_NS     14" MULTIROTOR_UAV_NMPC_SLACK_MATCH)
  if(MULTIROTOR_UAV_NMPC_STEPS_MATCH EQUAL -1 OR MULTIROTOR_UAV_NMPC_SLACK_MATCH EQUAL -1)
    set(MULTIROTOR_UAV_NMPC_REGENERATE TRUE)
  endif()
  foreach(_nmpc_generator_input IN LISTS MULTIROTOR_UAV_NMPC_GENERATOR_INPUTS)
    if("${_nmpc_generator_input}" IS_NEWER_THAN "${MULTIROTOR_UAV_NMPC_SOLVER_C}")
      set(MULTIROTOR_UAV_NMPC_REGENERATE TRUE)
    endif()
  endforeach()
endif()

if(MULTIROTOR_UAV_NMPC_REGENERATE)
  find_program(PYTHON3_EXECUTABLE python3)
  if(NOT PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "python3 is required to generate the package-local UAV NMPC acados code")
  endif()
  file(MAKE_DIRECTORY
    "${MULTIROTOR_UAV_NMPC_PYTHON_CACHE_DIR}"
    "${MULTIROTOR_UAV_NMPC_MPL_CACHE_DIR}"
  )

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      ACADOS_SOURCE_DIR=${MULTIROTOR_ACADOS_SOURCE_DIR}
      ACADOS_ROOT=${MULTIROTOR_ACADOS_SOURCE_DIR}
      XGC2_ACADOS_SOURCE_DIR=${MULTIROTOR_ACADOS_SOURCE_DIR}
      XGC2_ACADOS_INSTALL_DIR=${MULTIROTOR_ACADOS_INSTALL_DIR}
      XGC2_ACADOS_LIBRARY_DIRS=${MULTIROTOR_ACADOS_LIBRARY_PATH_ENV}
      XGC2_ACADOS_PYTHONPATH=${MULTIROTOR_ACADOS_PYTHONPATH_ENV}
      LD_LIBRARY_PATH=${MULTIROTOR_ACADOS_LIBRARY_PATH_ENV}:$ENV{LD_LIBRARY_PATH}
      PYTHONPATH=${MULTIROTOR_UAV_NMPC_TOOL_PATH}:${MULTIROTOR_ACADOS_PYTHONPATH_ENV}:$ENV{PYTHONPATH}
      PYTHONPYCACHEPREFIX=${MULTIROTOR_UAV_NMPC_PYTHON_CACHE_DIR}
      MPLCONFIGDIR=${MULTIROTOR_UAV_NMPC_MPL_CACHE_DIR}
      ${PYTHON3_EXECUTABLE} -B -m nmpc.export_ros_generated_code
      --output-dir ${MULTIROTOR_UAV_NMPC_GENERATED_DIR}
      --horizon ${MULTIROTOR_UAV_NMPC_HORIZON}
      --steps ${MULTIROTOR_UAV_NMPC_STEPS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE MULTIROTOR_UAV_NMPC_EXPORT_STATUS
  )
  if(NOT MULTIROTOR_UAV_NMPC_EXPORT_STATUS EQUAL 0)
    message(FATAL_ERROR "failed to generate package-local UAV NMPC acados code")
  endif()
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetRuntimeParameters.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs nav_msgs sensor_msgs mavros_msgs ros1_utils multirotor_reference_trajectory hover_thrust_estimator estimator_vrpn_px4_rotor_state
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  BEFORE
  include
  "${CATKIN_DEVEL_PREFIX}/include"
  ${MULTIROTOR_HOVER_THRUST_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${MULTIROTOR_UAV_NMPC_GENERATED_DIR}
  ${MULTIROTOR_ACADOS_INCLUDE_DIRS}
  ${acados_INCLUDE_DIRS}
)

file(GLOB MULTIROTOR_UAV_NMPC_MODEL_SOURCES "${MULTIROTOR_UAV_NMPC_GENERATED_DIR}/uav_nmpc_model/*.c")
file(GLOB MULTIROTOR_UAV_NMPC_COST_SOURCES "${MULTIROTOR_UAV_NMPC_GENERATED_DIR}/uav_nmpc_cost/*.c")
file(GLOB MULTIROTOR_UAV_NMPC_CONSTRAINT_SOURCES "${MULTIROTOR_UAV_NMPC_GENERATED_DIR}/uav_nmpc_constraints/*.c")
set(MULTIROTOR_UAV_NMPC_GENERATED_SOURCES
  ${MULTIROTOR_UAV_NMPC_SOLVER_C}
  ${MULTIROTOR_UAV_NMPC_MODEL_SOURCES}
  ${MULTIROTOR_UAV_NMPC_COST_SOURCES}
  ${MULTIROTOR_UAV_NMPC_CONSTRAINT_SOURCES}
)

add_library(px4_multirotor_controller_uav_nmpc_generated STATIC ${MULTIROTOR_UAV_NMPC_GENERATED_SOURCES})
target_include_directories(px4_multirotor_controller_uav_nmpc_generated PUBLIC
  ${MULTIROTOR_UAV_NMPC_GENERATED_DIR}
  ${MULTIROTOR_ACADOS_INCLUDE_DIRS}
  ${acados_INCLUDE_DIRS}
)
add_dependencies(px4_multirotor_controller_uav_nmpc_generated ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4_multirotor_controller_uav_nmpc_generated
  ${MULTIROTOR_ACADOS_LIBRARIES}
  m
)
set_target_properties(px4_multirotor_controller_uav_nmpc_generated PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(px4_multirotor_controller_uav_nmpc_runtime
  src/nmpc/uav_nmpc_solver.cpp
  src/uav/nmpc_tracking_backend.cpp
)
target_include_directories(px4_multirotor_controller_uav_nmpc_runtime BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CATKIN_DEVEL_PREFIX}/include
)
add_dependencies(px4_multirotor_controller_uav_nmpc_runtime ${catkin_EXPORTED_TARGETS})
add_dependencies(px4_multirotor_controller_uav_nmpc_runtime px4_multirotor_controller_uav_nmpc_generated)
target_link_libraries(px4_multirotor_controller_uav_nmpc_runtime
  px4_multirotor_controller_uav_nmpc_generated
  xgc2_math::control
  ${catkin_LIBRARIES}
  "-Wl,--no-as-needed"
  ${MULTIROTOR_ACADOS_LIBRARIES}
  "-Wl,--as-needed"
  m
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/drone_ros_node.cpp
  src/drone_controller.cpp
  src/input/command_input_producer.cpp
  src/input/sensor_input_producer.cpp
  src/input/trajectory_input_producer.cpp
  src/service/runtime_parameter_service.cpp
  src/output/control_output_consumer.cpp
  src/output/debug_output_consumer.cpp
  src/output/nmpc_output_consumer.cpp
  src/output/px4_service_output_consumer.cpp
  src/output/reference_activation_output_consumer.cpp
  src/uav/nmpc_result_buffer.cpp
  src/uav/active_trajectory_cache.cpp
  src/uav/state_machine/debug_monitor_state.cpp
  src/uav/state_machine/health_monitor_state.cpp
  src/uav/state_machine/self_check_state.cpp
  src/uav/state_machine/ready_state.cpp
  src/uav/state_machine/takeoff_init_state.cpp
  src/uav/state_machine/takeoff_offboard_request_state.cpp
  src/uav/state_machine/takeoff_arm_request_state.cpp
  src/uav/state_machine/takeoff_ascending_state.cpp
  src/uav/state_machine/hover_state.cpp
  src/uav/state_machine/landing_state.cpp
  src/uav/state_machine/custom1_state.cpp
)

target_include_directories(${PROJECT_NAME}_node BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CATKIN_DEVEL_PREFIX}/include
  ${MULTIROTOR_HOVER_THRUST_INCLUDE_DIRS}
)

## Add dependencies
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link
target_link_libraries(${PROJECT_NAME}_node
  px4_multirotor_controller_uav_nmpc_runtime
  xgc2_state_machine::state_machine
  xgc2_math::control
  xgc2_math::geometry
  xgc2_math::trajectory
  ${catkin_LIBRARIES}
  pthread
)

#############
## Install ##
#############

install(TARGETS px4_multirotor_controller_uav_nmpc_runtime ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "results" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(px4_multirotor_controller_nmpc_event_runtime_test
    test/nmpc_event_runtime_test.cpp
    src/uav/nmpc_result_buffer.cpp
    src/uav/active_trajectory_cache.cpp
  )
  if(TARGET px4_multirotor_controller_nmpc_event_runtime_test)
    target_include_directories(px4_multirotor_controller_nmpc_event_runtime_test
      BEFORE PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CATKIN_DEVEL_PREFIX}/include
    )
    add_dependencies(px4_multirotor_controller_nmpc_event_runtime_test
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(px4_multirotor_controller_nmpc_event_runtime_test
      px4_multirotor_controller_uav_nmpc_runtime
      ${catkin_LIBRARIES}
      gtest_main
      xgc2_math::control
      xgc2_math::geometry
      xgc2_math::trajectory
    )
  endif()
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
