# XGC2 Multirotor Controller

ROS1 multirotor controller product repository for XGC2 robots.

Packages:

- `multirotor_reference_trajectory`: multirotor reference trajectory generation
  runtime and ROS publishers.
- `px4_multirotor_controller`: PX4/MAVROS multirotor controller with
  state-machine runtime and UAV NMPC tracking.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-multirotor-controller
```

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch
```

## Control-state modes

The controller keeps one executable and selects the control-state provider at
launch time:

- `state_source:=state_estimator` keeps the estimator/hover-thrust/NMPC
  deployment path.
- `state_source:=vrpn_direct` treats the configured VRPN pose and twist as the
  trusted control state. For high-fidelity PX4 simulation, pair it with
  `tracking_backend:=px4_local_raw` to forward position, velocity, and
  acceleration references without running an additional XGC2 inner loop.

Example:

```bash
roslaunch px4_multirotor_controller uav_nmpc_controller.launch \
  ns:=uav1 state_source:=vrpn_direct tracking_backend:=px4_local_raw
```
