# Product-owned native controller adapter

`libctl_px4.so` is an ABI adapter of this controller product. Runtime supplies
only `XgcRuntime::SDK`; it does not own configuration, health policy or NMPC.
ROS and native both use `ControllerDriver`, `SensorStatistics` and
`NmpcExecution`. The adapters marshal messages and consume domain outputs.

The native default profile is generated directly from `config/uav_nmpc.yaml`.
ROS uses its authored private parameter namespace (loaded by its launch file).
Both call the same configuration reader and validation policy. Partial ROS
parameters retain the former C++ defaults, including the original safety limits;
they do not silently inherit the simulation profile's larger limits. Native flat
keys only translate transport spelling, such as `smc_k1` to `smc/k1`.

Statistics retain the original ten-message window, 10 Hz statistics, 1 Hz
heartbeat and inclusive 2.5 s communication timeout. Receipt clocks are supplied
by the adapter. No second native heartbeat approximation is used.

NMPC captures sensor, reference, configuration and request time on the owner
thread. One worker retains the original busy/pending rejection. Each control
entry has a generation; requests also have controller-lifetime monotonic IDs.
Exit and stop invalidate in-flight results. An old generation cannot store,
publish diagnostics or supply a target to a new entry. Stop drops pending work
and joins the worker; start begins with clean pending/busy flags. Mathematical
tracking strategies and thresholds remain in their original implementations.

Enable `PX4_CONTROLLER_NATIVE_ADAPTER=ON` when configuring the owning catkin
workspace. Installed SDK consumption uses
`find_package(XgcRuntimeSDK CONFIG REQUIRED)` and links `XgcRuntime::SDK`.
Set `CMAKE_PREFIX_PATH` to the installed SDK and wire-owner prefixes. Native
records/codecs are imported from `XgcRoboticsInterfaces::Interfaces`,
`HoverThrustNative::Wire`, `RigidStateNative::Wire` and
`ReferenceTrajectoryNative::Wire`.
No Runtime source headers or private wire copies are used.
FCU arming and mode requests use `fcu_request` (`xgc.fcu_request/1`) for the
ROS edge's MAVROS service caller.
The native ELF installs beside the owning controller core; its headers-only SDK
is not a runtime dependency. The package assembler includes it when built.

Migration starts from sync-runtime commit
`441d397e729bc1c26011b5a80a55d8be827fae27`. Removal of the former runtime-owned
adapter/build script and switching runtime consumers belong to the integration
owner's serial window. Until that switch, the former adapter is not evidence for
this product's driver or for a complete 6DoF closed loop.
