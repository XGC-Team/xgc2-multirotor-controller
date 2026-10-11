# px4_multirotor_controller

PX4/MAVROS multirotor controller: a state machine (self check, takeoff, hover, landing, trajectory
tracking) around four tracking backends (`px4_local`, `smc`, `dfbc`, `nmpc`).

The package builds three things from one implementation:

| Artifact | What |
|---|---|
| `libpx4_multirotor_controller_core.so`, `libpx4_multirotor_controller_uav_nmpc_runtime.so` | The ROS-free core: `DroneController`, its state machine and tracking strategies, `ControllerDriver` (receive statistics, the NMPC worker thread), the input handling shared by every front end (`driver/command_input.h`, `driver/trajectory_ingress.h`), and the acados NMPC solver. They link no ROS library. |
| `px4_multirotor_controller_node` | The ROS node. Its input producers and output consumers turn ROS topics and MAVROS services into calls of the core and back. Its behavior, topics and parameters are those of the previous releases. |
| `libpx4_multirotor_controller_module.so` | The xgc2-module module `px4_multirotor_controller`. It runs the same core with typed ports instead of ROS topics. |

The module is one half of the chain of an entity: the reference trajectory generator
(`multirotor_reference_trajectory`, module `px4_multirotor_reference`) is the other, and the entity's
MAVROS edge module is the third. The edge module is not part of this package yet; the section
[MAVROS edge module](#mavros-edge-module-what-it-must-map) says what it must map.

## The module

Module name `px4_multirotor_controller`, entry symbol `xgc2_module_entry` (the library exports nothing
else), module ABI major 2. The ports, in descriptor order (the inputs come first, so the index of an
input is its bit in `changed_inputs`):

| Port | Dir | Kind (queue) | Schema id | Required | ROS counterpart of the node |
|---|---|---|---|---|---|
| `state_estimate` | in | event (8) | `xgc2.px4.state_estimate.v1` | | `alg/state_estimator/state` |
| `local_pose` | in | event (8) | `xgc2.px4.pose.v1` | yes | `mavros/local_position/pose` |
| `local_velocity` | in | event (8) | `xgc2.px4.velocity.v1` | yes | `mavros/local_position/velocity_local` |
| `imu` | in | event (16) | `xgc2.px4.imu.v1` | yes | `mavros/imu/data` |
| `fcu_state` | in | event (8) | `xgc2.px4.fcu_state.v1` | yes | `mavros/state` |
| `battery` | in | event (4) | `xgc2.px4.battery.v1` | | `mavros/battery` |
| `vrpn_pose` | in | event (8) | `xgc2.px4.pose.v1` | yes | `pose` (the canonical pose) |
| `command` | in | event (8) | `xgc2.px4.command.v1` | | `/command` |
| `alg_setpoint` | in | event (8) | `xgc2.px4.position_target.v1` | | `alg/setpoint_raw/local` |
| `hover_thrust` | in | event (8) | `xgc2.px4.hover_thrust.v1` | | `hover_thrust/estimate_state` |
| `ref_active_analytic` | in | state | `xgc2.px4.reference_analytic.v1` | | `alg/multirotor_reference_trajectory/active/analytic` |
| `ref_active_sampled` | in | state | `xgc2.px4.reference_sampled.v1` | | `alg/multirotor_reference_trajectory/active/sampled` |
| `setpoint` | out | state | `xgc2.px4.position_target.v1` | | `mavros/setpoint_raw/local` |
| `attitude_rate` | out | state | `xgc2.px4.attitude_rate_target.v1` | | `mavros/setpoint_raw/attitude` |
| `fcu_request` | out | event (8) | `xgc2.px4.fcu_request.v1` | | services `mavros/cmd/command`, `mavros/set_mode` |
| `status` | out | state | `xgc2.px4.controller_status.v1` | | `custom/statustext` |
| `ref_request` | out | event (4) | `xgc2.px4.reference_analytic.v1` | | `alg/multirotor_reference_trajectory/request/analytic` |

The payloads are fixed-size POD structs of two public headers, installed with the package and
checked by `static_assert` for size and field offsets (the tests also compile them as C11):

- `px4_multirotor_controller/payloads.h`: `xgc2.px4.state_estimate.v1`, `pose.v1`, `velocity.v1`,
  `imu.v1`, `fcu_state.v1`, `battery.v1`, `command.v1`, `position_target.v1`, `hover_thrust.v1`,
  `attitude_rate_target.v1`, `fcu_request.v1`, `controller_status.v1`.
- `multirotor_reference_trajectory/payloads.h` (package `multirotor_reference_trajectory`):
  `xgc2.px4.reference_analytic.v1`, `reference_sampled.v1`, `reference_status.v1`,
  `reference_reset.v1`.

A layout never changes; a different layout gets a new schema id.

### Behavior

- **A step of the period is one iteration of the node's 1 kHz loop.** `start()` asks the host for a
  1 ms period. A step takes the samples that arrived (event ports in order, the state ports whose bit
  is set in `changed_inputs`), applies them in the order of their sample stamps, updates the
  controller at the host clock, and handles the controller's output events the way the node's output
  consumers do. A step that has only samples to deliver (the host also steps on input commits) applies
  them and leaves the update to the next period, like the node's callbacks between two iterations of
  its loop; a step of a wake updates at once.
- **Threads.** The host calls an instance from any of its worker threads, a different one each time,
  while the state machine library binds a machine to the thread that first updates it. The module
  builds, updates and destroys the controller on a thread of its own (`module_support::OwnerThread`);
  the calling thread reads the inputs and writes the outputs with the host API and waits meanwhile.
- **Sensor inputs are event ports.** The controller's receive statistics and its frame counters are part
  of its safety logic and count every message; a state port would merge a burst of two messages within
  one step. The active reference is a state port: only the newest one counts.
- **Sample stamps.** The stamp the producer gives to `write_commit` is the time the message was
  received, in the host clock domain; it is the time of the receive statistics and of the input event,
  like `ros::Time::now()` in the node's callbacks. The stamp fields inside a payload are the message's
  own header stamp (sec/nsec). The estimator and hover-thrust stamps are compared with the controller's
  clock, so they must be in the host clock domain too.
- **NMPC.** The solver runs on a worker thread owned by `ControllerDriver`. When a solve completes the
  worker calls `host->wake()`, so the step that consumes the result does not wait for the next period.
  The worker writes no port, so no output is flagged `XGC2_PORT_ASYNC_WRITER`. `stop()` joins the worker;
  no `wake()` is called after it returns.
- **Outputs.** `setpoint` and `attitude_rate` are written when the controller publishes them (the
  thrust is limited to [0, 1] as in the node); `fcu_request` carries one event per arm/disarm or
  mode request; `status` carries the control state name (`SelfCheck`, `Ready`, `TakeoffInit`,
  `TakeoffArmRequest`, `TakeoffOffboardRequest`, `TakeoffAscending`, `Hover`, `Landing`, `Custom1`);
  `ref_request` carries the reference activation request for the reference module (frame `map`).
- **Health.** `report(health, detail)` carries the control state name as the detail. The health is
  degraded while the last output could not be written (full event queue, a text that does not fit its
  payload) and ok again after a step whose writes all succeeded.
- **Lifecycle.** `create` parses the configuration (a bad one fails with `XGC2_ERR_INVALID` and a log
  line naming the key). `start` builds the controller afresh (`SelfCheck`, no history); `stop` tears
  it down. A `configure` on the ground (`SelfCheck` or `Ready`) takes effect at once; while the
  controller is flying it is refused with `XGC2_ERR_STATE` and the running configuration stays. A
  callback that throws is logged and becomes `XGC2_ERR_INTERNAL`; an input that cannot be applied is
  logged and skipped.
- **Not ports.** What the node publishes for observers is not a port: sensor statistics, state machine
  events, tracking error, VRPN quality, NMPC debug samples and predicted paths, the `/xgc/record_facts`
  load receipt. The core still computes the statistics the safety logic uses.

### Configuration

A JSON object with the structure of `config/uav_nmpc.yaml` (a key path `smc/k1` is
`{"smc": {"k1": ...}}`; in a manifest it is the `[instance.config]` table). Keys that are absent keep the
values of that file, which is compiled into the core, not the defaults of the C++ structs. Values are
read and validated by the same code as the node's parameters (`driver/controller_config.h`).

- `world_boundary_json` is required: a string, `"null"` for no geofence or the boundary as JSON text.
  The geofence is never defaulted.
- A key the controller does not read is an error, so a typo cannot leave a default in place. The
  node-only parameters are not keys of the module: `debug_print`, `state_estimate_topic`,
  `vrpn_pose_topic`, `vrpn_quality_*` (topics belong to the edge, the rest is node telemetry).
- `control_frequency` is not a key: the module asks the host for the node's fixed 1 kHz.

```toml
[instance.config]
world_boundary_json = "null"
tracking_backend = "dfbc"       # px4_local (default), smc, dfbc or nmpc
takeoff_altitude = 1.5
[instance.config.dfbc]
position_natural_frequency = [2.0, 2.0, 2.2]
```

### Wiring in an entity manifest

Instances start in manifest order, so list the consumers of an event channel before its producers
(the reference module takes requests from the controller and the operator; the controller takes events
from the edge). Channel names are the entity's choice; state channels have one writer, event channels
any number. The reference module's `analytic` port is an event channel on purpose: the operator or a
planner and the controller's `ref_request` can both write it.

```toml
[[module]]
name = "reference"
path = "/opt/ros/noetic/lib/libpx4_multirotor_reference_module.so"
[[module]]
name = "controller"
path = "/opt/ros/noetic/lib/libpx4_multirotor_controller_module.so"

[[instance]]
name = "reference"
module = "reference"
[instance.bind]
analytic = "ref.request.analytic"
sampled = "ref.request.sampled"
reset = "ref.reset"
status = "ref.status"
active_analytic = "ref.active.analytic"
active_sampled = "ref.active.sampled"

[[instance]]
name = "controller"
module = "controller"
[instance.config]
world_boundary_json = "null"
[instance.bind]
state_estimate = "est.state"
local_pose = "mavros.local_pose"
local_velocity = "mavros.local_velocity"
imu = "mavros.imu"
fcu_state = "mavros.state"
battery = "mavros.battery"
vrpn_pose = "pose"
command = "command"
alg_setpoint = "planner.setpoint"
hover_thrust = "hover_thrust"
ref_active_analytic = "ref.active.analytic"
ref_active_sampled = "ref.active.sampled"
setpoint = "mavros.setpoint"
attitude_rate = "mavros.attitude_rate"
fcu_request = "mavros.fcu_request"
status = "controller.status"
ref_request = "ref.request.analytic"

# The entity's MAVROS edge module is listed last. It writes est.state, mavros.local_pose,
# mavros.local_velocity, mavros.imu, mavros.state, mavros.battery, pose, command, planner.setpoint,
# hover_thrust, ref.request.sampled and ref.reset, and reads mavros.setpoint,
# mavros.attitude_rate, mavros.fcu_request and controller.status.
```

Leave `period_ms` out: a module chooses its step period itself when it starts (1 ms for the controller,
`main_frequency` for the reference module), and without a manifest period the host applies its default
step budget of 50 ms instead of one period. The required inputs (`local_pose`, `local_velocity`, `imu`,
`fcu_state`, `vrpn_pose`) keep the instance from being ready until a producer is bound;
`xgc2-module-host --check` reports the ones without one.

## MAVROS edge module: what it must map

The edge module of a PX4 entity is the ROS1 side of this chain. It is not built yet; this section is
its contract. `test/replay/ros_payload_mapping.h` is the mapping as executable code (one function per
input message type); the module tests and the parity test feed the module with it, and the table below
states the same mapping in prose.

### Inputs: ROS to the controller's ports

For every received message the edge commits **one** sample (never merge two messages), in arrival order,
with `stamp_ns` = the time the message was received (`MessageEvent` receipt time, in the host clock
domain), and the header stamp of the message in the payload's `stamp_sec`/`stamp_nsec` (0 when the
message has none). Quaternions are x, y, z, w as in ROS.

| Port | ROS topic (type) | Payload fields |
|---|---|---|
| `state_estimate` | `alg/state_estimator/state` (`rigid_state_estimator_msgs/RigidStateEstimate`) | `position`, `velocity`, `orientation_xyzw`, `angular_velocity`, `linear_acceleration`, `gravity`, `accel_bias`, `filter_inertial_stamp_sec`, `filter_pose_stamp_sec`, `last_vrpn_pose_stamp_sec`, `flags`, `estimator_state` |
| `local_pose` | `mavros/local_position/pose` (`geometry_msgs/PoseStamped`) | `position`, `orientation_xyzw` |
| `local_velocity` | `mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`) | `linear` (the angular part is not used) |
| `imu` | `mavros/imu/data` (`sensor_msgs/Imu`) | the stamp only: the controller counts arrivals and rate, it reads no IMU value |
| `fcu_state` | `mavros/state` (`mavros_msgs/State`) | `connected`, `armed`, `guided`, `manual_input` (0/1), `system_status`, `mode` (NUL-terminated, at most 31 characters) |
| `battery` | `mavros/battery` (`sensor_msgs/BatteryState`) | `percentage` (telemetry only) |
| `vrpn_pose` | `pose` (`geometry_msgs/PoseStamped`, the canonical pose; parameter `vrpn_pose_topic` of the node) | `position`, `orientation_xyzw` |
| `command` | `/command` (`std_msgs/String`) | `text` (NUL-terminated, at most 63 characters) |
| `alg_setpoint` | `alg/setpoint_raw/local` (`mavros_msgs/PositionTarget`) | `position`, `velocity`, `acceleration` (= `acceleration_or_force`), `yaw`, `yaw_rate`, `type_mask`, `coordinate_frame` |
| `hover_thrust` | `hover_thrust/estimate_state` (`hover_thrust_estimator_msgs/HoverThrustEstimate`) | `hover_thrust`, `flags`; a zero header stamp means unstamped and the controller then uses the receipt time |

`ref_active_analytic` and `ref_active_sampled` are wired to the reference module's outputs of the same
meaning; the edge does not map them. The node subscribes to its topics with a queue of 5; the module's
event ports hold 4 to 16 samples, which a 1 kHz step never fills at the rates of these topics.

A message the payload cannot represent is **refused, not truncated**: the edge logs it and counts it,
and does not commit a sample. That is a `mode` of 32 characters or more (the longest PX4 custom mode
name has 18), and a `command` of 64 characters or more (the node ignores a longer string, and no command
is longer than 7). A sample the host refuses because the event queue is full is a counted drop of the
host; the edge should log it.

### Outputs: the controller's ports to ROS

| Port | ROS output | Mapping |
|---|---|---|
| `setpoint` (state) | `mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`) | `header.stamp` = the payload stamp, `header.frame_id` = `map`, `position`, `velocity`, `acceleration_or_force` = `acceleration`, `yaw`, `yaw_rate`, `type_mask`, `coordinate_frame`. Publish each new sample once; the controller writes it when it publishes (the node's rate). |
| `attitude_rate` (state) | `mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`) | `header.stamp` = the payload stamp, `type_mask` = `IGNORE_ATTITUDE`, `orientation.w` = 1, `body_rate`, `thrust` (already in [0, 1]). |
| `fcu_request` (event) | services `mavros/cmd/command` and `mavros/set_mode` | `kind` 1 (`XGC2_PX4_FCU_REQUEST_ARM`): `mavros_msgs/CommandLong`, `command` 400 (`MAV_CMD_COMPONENT_ARM_DISARM`), `param1` = 1.0 to arm and 0.0 to disarm, the other parameters 0. `kind` 2 (`XGC2_PX4_FCU_REQUEST_MODE`): `mavros_msgs/SetMode`, `custom_mode` = `mode`. |
| `status` (state) | `custom/statustext` (`std_msgs/String`) | `data` = `state`. |

Service calls block: the edge makes them on a thread of its own, in request order, never in `step`, and
logs a failed call (the node does the same and does not retry; the controller re-requests while it
waits in `TakeoffArmRequest` or `TakeoffOffboardRequest`). The `ref_request` output is wired to the
reference module; an edge that also publishes it to ROS does so for observers only.

### Clock

The controller reads `now_ns()` of the host as its time. In simulation the entity runs with
`clock.mode = "external"` and the edge publishes the clock channel from `/clock`; the stamps of the
messages the edge forwards are then in that domain, as the controller's staleness checks require.

## Tests

```bash
catkin build px4_multirotor_controller                       # core, node, module
catkin build px4_multirotor_controller --make-args tests     # the test executables
catkin test px4_multirotor_controller
```

| Test | What it checks |
|---|---|
| `controller_payloads_layout_c`, `controller_payloads_api_cpp` | The payload headers compile as C11 and C++ and the layout is the one the `static_assert`s state. |
| `controller_module_test` | The module with an in-test host (`module_support/include/module_support/test_host.hpp`): the descriptor, the configuration (valid and invalid), start/period, the takeoff sequence on worker threads that change at every call, commands, a step with only samples, live `configure`, `stop`/`start`, a full request queue, and closed loops with a scripted vehicle and the reference module for DFBC and NMPC, including `wake()` from the solver thread and none after `stop()`. |
| `controller_module_parity_test` | The module against the ROS node's behavior without ROS (`test/replay/controller_node_path.h`: the input producers' handling of a message and one iteration of the control loop, with the NMPC solved inline). Both get the same recorded messages and integer-nanosecond ticks; every setpoint, attitude-rate command, flight controller request, status and reference request must be equal bit for bit after every tick, and the flight states must agree. Inputs: three recorded software-plant flights (`px4_local`, `dfbc`, `nmpc`, `test/replay/data`) and a scripted flight that reaches DFBC and NMPC tracking (`test/replay/scripted_flight.h`). |
| `controller_transport_parity` (rostest) | The real `SensorInputProducer` on a private ROS master against the module: the messages arrive through ROS topics on one side and as mapped payloads on the other (the sensors speak twice, then fall silent); both must show the same flight state, receive statistics and heartbeat, and leave `Ready` for the sensor timeout at the same tick. |
| `chain_in_module_host_nmpc`, `chain_in_module_host_dfbc` (opt-in) | The reference module, the controller module and a scripted vehicle (`test/host/scripted_vehicle.cpp`, in place of the MAVROS edge) in the real `xgc2-module-host`, flown through takeoff, hover, `custom1` and `land` (`test/host/run_chain.py`). They check, from the host's health report, what only the real host shows: all instances stay healthy, the controller asks the reference module once and gets its active reference, and every NMPC solve wakes the host and is answered with an attitude-rate command. They need the host binary, which another repository builds: configure with `-DXGC2_MODULE_HOST=/path/to/xgc2-module-host` and run `ctest -R chain_in_module_host`. |
| the existing core and node tests | `controller_driver_test`, the state machine, strategies, world boundary and the Python tests of `.github/workflows/ci.yml` are unchanged. |

The fixtures in `test/replay/data` are recorded software-plant flights converted with
`test/replay/bag_to_stream.py` and cut with `test/replay/trim_stream.py` (31 s of `px4_local`, 16 s of
`dfbc` and `nmpc`; DFBC and NMPC flights carry the estimate, hover thrust and reference messages).

`-DPX4_CONTROLLER_REPLAY_HARNESS=ON` builds `px4_multirotor_controller_replay_harness`, which writes the
node path's outputs of a stream as bit-exact text for comparing two builds of the core with `cmp`
(`replay_harness STREAM OUT.txt [px4_local|dfbc|nmpc|smc]`).
