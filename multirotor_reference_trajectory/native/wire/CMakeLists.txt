cmake_minimum_required(VERSION 3.8)
project(ReferenceTrajectoryNative LANGUAGES NONE)
include(CMakePackageConfigHelpers)
add_library(reference_trajectory_native_wire INTERFACE)
add_library(ReferenceTrajectoryNative::Wire ALIAS reference_trajectory_native_wire)
set_target_properties(reference_trajectory_native_wire PROPERTIES EXPORT_NAME Wire)
target_include_directories(reference_trajectory_native_wire INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>)
# Records are C11-compatible; only the C++ serializer needs C++17. A language
# test is legal for compile options, but not for linked compile features.
set_property(TARGET reference_trajectory_native_wire PROPERTY INTERFACE_COMPILE_OPTIONS
  "$<$<COMPILE_LANGUAGE:CXX>:-std=c++17>")
install(FILES
  ../../include/multirotor_reference_trajectory/reference_wire_v1.h
  ../../include/multirotor_reference_trajectory/reference_wire.hpp
  DESTINATION include/multirotor_reference_trajectory)
install(TARGETS reference_trajectory_native_wire EXPORT ReferenceTrajectoryNativeTargets)
install(EXPORT ReferenceTrajectoryNativeTargets NAMESPACE ReferenceTrajectoryNative::
  DESTINATION share/cmake/ReferenceTrajectoryNative)
configure_package_config_file(ReferenceTrajectoryNativeConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ReferenceTrajectoryNativeConfig.cmake
  INSTALL_DESTINATION share/cmake/ReferenceTrajectoryNative)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ReferenceTrajectoryNativeConfig.cmake
  DESTINATION share/cmake/ReferenceTrajectoryNative)
