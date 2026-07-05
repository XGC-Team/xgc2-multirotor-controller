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
