# multirotor_reference_trajectory

Reference trajectory generator of the multirotor chain: it takes analytic requests (hold, circle,
circle with entry, figure eight, line, lemniscate, helices, torus knot) and sampled references,
validates them against the dynamic limits, activates one, and publishes the active reference and a
status.

The package builds three things from one implementation:

| Artifact | What |
|---|---|
| `libmultirotor_reference_trajectory_core.so` | The ROS-free runtime `ReferenceTrajectoryRuntime`, its state machine (`SelfCheck`, `Ready`, `Active`) and the curves. It links no ROS library. |
| `multirotor_reference_trajectory_node` | The ROS node: `ReferenceInputProducer` and `ReferenceOutputConsumer` turn ROS topics into calls of the runtime and back. Behavior, topics and parameters are those of the previous releases. |
| `libpx4_multirotor_reference_module.so` | The xgc2-module module `px4_multirotor_reference`. It runs the same runtime with typed ports instead of ROS topics. |

## The module

Module name `px4_multirotor_reference`, entry symbol `xgc2_module_entry` (the library exports nothing
else), module ABI major 2.

| Port | Dir | Kind (queue) | Schema id | ROS counterpart of the node |
|---|---|---|---|---|
| `analytic` | in | event (8) | `xgc2.px4.reference_analytic.v1` | `alg/multirotor_reference_trajectory/request/analytic` |
| `sampled` | in | event (4) | `xgc2.px4.reference_sampled.v1` | `alg/multirotor_reference_trajectory/request/sampled` |
| `reset` | in | event (4) | `xgc2.px4.reference_reset.v1` | `alg/multirotor_reference_trajectory/reset` |
| `status` | out | state | `xgc2.px4.reference_status.v1` | `alg/multirotor_reference_trajectory/status` |
| `active_analytic` | out | state | `xgc2.px4.reference_analytic.v1` | `alg/multirotor_reference_trajectory/active/analytic` |
| `active_sampled` | out | state | `xgc2.px4.reference_sampled.v1` | `alg/multirotor_reference_trajectory/active/sampled` |

The payloads are fixed-size POD structs in `multirotor_reference_trajectory/payloads.h` (installed with
the package, checked by `static_assert` for size and field offsets, compiled as C11 and C++ by the
tests). They mirror the ROS messages field for field: times are sec/nsec, the header keeps `seq`,
the stamp and `frame_id`. A layout never changes; a different layout gets a new schema id. The
variable-length fields have a capacity: at most 16 curve parameters (the curves define up to 9),
at most 1024 sampled points and a `frame_id` of at most 31 characters. The controller package
(`px4_multirotor_controller`) uses the analytic and sampled payloads for the active reference it
tracks and for the activation request it sends.

### Behavior

- **One step is one iteration of the node's main loop.** The module asks the host for a step period
  of `1 / main_frequency` (100 Hz by default). A step reads the requests that arrived on the three
  event ports, applies them in the order of their sample stamps, updates the runtime at the host
  clock, and writes what the runtime's output events ask for (`status` at `status_rate`, the active
  reference at `active_publish_rate`).
- **Request stamps.** The stamp the producer gives to `write_commit` is the time the request was
  received, in the host clock domain; it is the time of the request's event, like `ros::Time::now()`
  in the node's callbacks. The header and `start` stamps inside a payload are the message's own.
- **The analytic port has several writers in an entity**: the operator or a planner, and the
  controller's activation request. Event channels accept any number of writers.
- **Outputs are state ports**: a reader always gets the newest status and active reference.
- **Health.** `report(health, detail)` carries the runtime state (`SelfCheck`, `Ready`, `Active`,
  `Fault`). The health is degraded after an output could not be written and ok again after the next
  successful write.
- **Lifecycle.** `create` parses the configuration (a bad one fails with `XGC2_ERR_INVALID` and a log
  line naming the key). `start` starts the runtime, `stop` stops stepping. A `configure` while running
  restarts the runtime with the new configuration, which drops the active reference (the runtime's
  `setConfig` resets it, as in the node). A request the runtime rejects, or one that does not fit the
  payload, is logged and skipped.
- **Not ports.** The node's `visualization/reference_path` (a `nav_msgs/Path` for viewers) is not a
  port.

### Configuration

A JSON object whose keys and defaults are the node's private parameters
(`config/multirotor_reference_trajectory.yaml`): `main_frequency` (Hz), `status_rate`,
`active_publish_rate`, `validation_sample_dt`, `trajectory_timeout`, `min_lead_time`, `max_velocity`,
`max_acceleration`, `max_jerk`, `max_snap` (0 disables a limit) and `min_specific_thrust`. A key the
module does not know is an error, so the node's topic and visualization parameters are not keys.

```toml
[[module]]
name = "reference"
path = "/opt/ros/noetic/lib/libpx4_multirotor_reference_module.so"

[[instance]]
name = "reference"
module = "reference"
[instance.config]
active_publish_rate = 10.0
max_velocity = 3.0
[instance.bind]
analytic = "ref.request.analytic"
active_analytic = "ref.active.analytic"
active_sampled = "ref.active.sampled"
```

## MAVROS edge module: what it must map for the reference

The reference module has no MAVROS input. The ROS side of the entity (the edge module) carries the
operator's requests in and the reference out:

| Port | ROS topic (type) | Mapping |
|---|---|---|
| `analytic` (in) | `alg/multirotor_reference_trajectory/request/analytic` (`multirotor_reference_trajectory_msgs/AnalyticReference`) | One sample per message, `stamp_ns` = receipt time: `header` (`seq`, stamp, `frame_id`), `request_id`, `trajectory_id`, `revision`, `flags`, `start_time` as `start_sec`/`start_nsec`, `analytic_type`, `params` as `params_len` and `params[]`, `duration`, `origin` as `origin_position` and `origin_q_xyzw`. |
| `sampled` (in) | `.../request/sampled` (`SampledReference`) | `header`, `trajectory_id`, `revision`, `flags`, `start_time`, `sample_dt`, `points` as `points_len` and `points[]` (`t_from_start`, `position`, `velocity`, `acceleration`, `jerk`, `snap`, `yaw`, `yaw_rate`, `yaw_accel`). |
| `reset` (in) | `.../reset` (`std_msgs/Empty`) | One empty sample. |
| `status` (out) | `.../status` (`ReferenceStatus`, latched) | `header`, `state`, `active_type`, `flags`, `active_trajectory_id`, `active_revision`. |
| `active_analytic` (out) | `.../active/analytic` (latched) | The inverse of `analytic`. |
| `active_sampled` (out) | `.../active/sampled` (latched) | The inverse of `sampled`. |

A request with more than 16 parameters, more than 1024 points or a `frame_id` of 32 characters or more
cannot be represented: the edge refuses it (log and count) and commits no sample. The controller
module's activation request goes to the `analytic` channel directly; the edge needs no mapping for
it. The controller README lists the controller's own ROS inputs and outputs.
`test/replay/reference_node_path.h` is the node's behavior without ROS, and the module tests map
messages with the same field correspondence.

## Tests

```bash
catkin build multirotor_reference_trajectory                       # core, node, module
catkin build multirotor_reference_trajectory --make-args tests     # the test executables
catkin test multirotor_reference_trajectory
```

| Test | What it checks |
|---|---|
| `reference_payloads_layout_c`, `reference_payloads_api_cpp` | The payload header compiles as C11 and C++ and the layout is the one the `static_assert`s state. |
| `reference_module_test` | The module with an in-test host (`module_support/include/module_support/test_host.hpp`): descriptor, configuration (valid and invalid), start/period, an analytic and a sampled request becoming the active reference, the publication rate, rejected requests, `reset`, `stop`/`start`, a live `configure`, a refused output write degrading the health, and requests of several ports processed in arrival order. |
| `reference_module_parity_test` | The module against the ROS node's behavior without ROS (`test/replay/reference_node_path.h`) on the request stream `test/replay/reference-requests.stream` (22 requests of every kind, among them an invalid sampled one; regenerated by `make_reference_stream.py`). Both get the same requests at the same nanosecond ticks (10 ms); after every tick the status, active analytic and active sampled messages must be equal, in the order of the node's output events, with every double compared as its bits. |
| the existing runtime and node tests | `multirotor_reference_trajectory_runtime_test`, `..._unit_test`, `..._smoke_test` are unchanged. |

`-DREFERENCE_TRAJECTORY_REPLAY_HARNESS=ON` builds `multirotor_reference_trajectory_replay_harness`,
which writes the node path's outputs for a request stream as bit-exact text, for comparing two builds
of the runtime with `cmp`.
