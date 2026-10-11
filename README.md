# XGC2 Multirotor Controller

ROS1 multirotor controller product repository for XGC2 robots.

Packages:

- `multirotor_reference_trajectory`: multirotor reference trajectory generation
  runtime, ROS publishers and the `px4_multirotor_reference` xgc2-module module.
- `px4_multirotor_controller`: PX4/MAVROS multirotor controller with
  state-machine runtime and UAV NMPC tracking, its ROS node and the
  `px4_multirotor_controller` xgc2-module module.

Each package builds a ROS-free core library that both its ROS node and its module
run, so there is one implementation of the controller and one of the reference
generator. `module_support` holds what the two modules share (the module ABI
header, a strict JSON configuration reader and the in-test host); it is not a
catkin package and nothing in it is installed.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-multirotor-controller
```

The package installs the ROS nodes, the cores, the two module libraries
(`libpx4_multirotor_reference_module.so`, `libpx4_multirotor_controller_module.so`)
and the public payload headers `multirotor_reference_trajectory/payloads.h` and
`px4_multirotor_controller/payloads.h` (schema ids `xgc2.px4.*`). It requires no
runtime SDK, no robotics-interfaces package and no estimator node or wire package;
the estimators' ROS message packages remain its only link to them.

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch world_boundary_json:=null
```

## Modules

The modules run in an `xgc2-module` host next to the entity's other modules and
exchange typed payloads in process instead of ROS messages. Their ports, payloads,
configuration and behavior are documented in
[`px4_multirotor_controller/README.md`](px4_multirotor_controller/README.md) and
[`multirotor_reference_trajectory/README.md`](multirotor_reference_trajectory/README.md),
including the ROS inputs and outputs the entity's MAVROS edge module must map (the edge
module is not part of this repository yet).

## Tests

```bash
catkin build multirotor_reference_trajectory px4_multirotor_controller
catkin build multirotor_reference_trajectory px4_multirotor_controller --make-args tests
catkin test multirotor_reference_trajectory px4_multirotor_controller
```

Each module has unit tests with an in-test host and a parity test against the ROS
node's behavior without ROS, on recorded flights and scripted flights
(`px4_multirotor_controller/test/replay`). `.xgc2/scripts/run_source_tests.sh` runs the
CI path with `catkin_make`.

## Control-state source

The controller consumes only
`rigid_state_estimator_msgs/RigidStateEstimate`. Raw VRPN pose remains a
diagnostic consistency input; raw VRPN twist is neither subscribed nor accepted
as a control-state source. Simulation uses the same fused-state boundary as the
future onboard deployment.

## Reference inputs

The reference runtime publishes analytic and sampled references, including
circle and torus-knot entry trajectories. UAV waypoint optimization and external
active polynomial ingestion are retired. NMPC and DFBC retain the analytic and
sampled cache; PX4_LOCAL and SMC retain PositionTarget ingestion, PVA lifting,
Custom1, and their existing output and takeoff/landing paths.

This revision requires the 1.4 message contract. All consumers of UAV
ReferenceStatus must be rebuilt together because its ROS1 MD5 changed.
