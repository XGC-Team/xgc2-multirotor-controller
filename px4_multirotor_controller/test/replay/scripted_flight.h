#pragma once

// A scripted flight as a replay stream: the messages the controller node would receive during a
// takeoff, a hover and a trajectory tracking, with the vehicle's answers scripted open loop (the
// recorded streams hold no DFBC or NMPC tracking). It exercises the attitude-rate outputs and the
// NMPC solver, which the recordings do not reach.
//
// With skip_takeoff_init_disarm the controller waits in TakeoffArmRequest until the flight
// controller reports the vehicle armed, then ascends until the altitude is reached:
//   t = takeoff_at  the operator's `takeoff`
//   t = arm_at      armed (the controller's arm request is answered by the script)
//   t = arm_at + 0.5 s  the vehicle climbs at 1 m/s to the takeoff altitude
//   t = track_at    the operator's `custom1`; the controller asks for a reference and the script
//                   answers with an active hold reference 0.3 s later, published at 10 Hz as the
//                   reference module does.

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_node_path.h"

namespace controller_replay {

struct ScriptedFlight {
    uint64_t t0_ns{100000000000ULL};  // 100 s
    double duration{14.0};
    double takeoff_at{3.0};
    double arm_at{5.0};
    double takeoff_altitude{1.5};
    double track_at{10.0};
};

inline std::vector<Record> scriptedFlight(const ScriptedFlight& f) {
    namespace msgs = multirotor_reference_trajectory_msgs;
    std::vector<Record> records;
    auto add = [&](uint64_t t_ns, RecordKind kind, std::vector<uint8_t> bytes) {
        records.push_back(Record{t_ns, kind, std::move(bytes)});
    };
    auto stampOf = [](uint64_t t_ns) { return ros::Time().fromNSec(t_ns); };
    auto altitude = [&](double t) {
        const double climb = t - (f.arm_at + 0.5);
        return climb <= 0.0 ? 0.0 : std::min(f.takeoff_altitude, climb);
    };

    const uint64_t ms = 1000000ULL;
    const uint64_t end = static_cast<uint64_t>(f.duration * 1000.0) * ms;
    bool takeoff_sent = false, track_sent = false;
    for (uint64_t at = 0; at <= end; at += ms) {
        const uint64_t t_ns = f.t0_ns + at;
        const double t = static_cast<double>(at) * 1e-9;
        const ros::Time stamp = stampOf(t_ns);
        const uint64_t tick = at / ms;
        const double z = altitude(t);

        if (tick % 5 == 0) {  // sensor_msgs/Imu, 200 Hz: only its arrival matters
            sensor_msgs::Imu imu;
            imu.header.stamp = stamp;
            imu.orientation.w = 1.0;
            imu.linear_acceleration.z = 9.8066;
            add(t_ns, kImu, encode(imu));
        }
        if (tick % 10 == 0) {  // the canonical pose and the fused estimate, 100 Hz
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0;
            add(t_ns, kVrpnPose, encode(pose));
            rigid_state_estimator_msgs::RigidStateEstimate estimate;
            estimate.header.stamp = stamp;
            estimate.estimator_state = 3;  // RUNNING
            estimate.position.z = z;
            estimate.orientation.w = 1.0;
            estimate.gravity.z = -9.8066;
            estimate.filter_inertial_stamp_sec = static_cast<double>(t_ns) * 1e-9;
            estimate.filter_pose_stamp_sec = static_cast<double>(t_ns) * 1e-9;
            estimate.last_vrpn_pose_stamp_sec = static_cast<double>(t_ns) * 1e-9;
            add(t_ns, kEstimate, encode(estimate));
        }
        if (tick % 20 == 0) {  // MAVROS pose, velocity and state, hover thrust, 50 Hz
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0;
            add(t_ns, kLocalPose, encode(pose));
            geometry_msgs::TwistStamped velocity;
            velocity.header.stamp = stamp;
            add(t_ns, kLocalVelocity, encode(velocity));
            mavros_msgs::State state;
            state.header.stamp = stamp;
            state.connected = true;
            state.armed = t >= f.arm_at;
            state.mode = "OFFBOARD";
            state.system_status = 4;
            add(t_ns, kFcuState, encode(state));
            hover_thrust_estimator_msgs::HoverThrustEstimate hover;
            hover.header.stamp = stamp;
            hover.hover_thrust = 0.5;
            add(t_ns, kHoverThrust, encode(hover));
        }
        if (tick % 1000 == 0) {
            sensor_msgs::BatteryState battery;
            battery.header.stamp = stamp;
            battery.percentage = 0.9f;
            add(t_ns, kBattery, encode(battery));
        }
        if (!takeoff_sent && t >= f.takeoff_at) {
            std_msgs::String command;
            command.data = "takeoff";
            add(t_ns, kCommand, encode(command));
            takeoff_sent = true;
        }
        if (!track_sent && t >= f.track_at) {
            std_msgs::String command;
            command.data = "custom1";
            add(t_ns, kCommand, encode(command));
            track_sent = true;
        }
        if (t >= f.track_at + 0.3 && tick % 100 == 0) {  // the reference module's 10 Hz
            msgs::AnalyticReference reference;
            reference.header.stamp = stamp;
            reference.request_id = 1;
            reference.trajectory_id = 1;
            reference.revision = 1;
            reference.analytic_type = msgs::AnalyticReference::ANALYTIC_HOLD;
            reference.duration = 60.0;
            reference.origin.position.z = f.takeoff_altitude;
            reference.origin.orientation.w = 1.0;
            reference.params = {0.0, 0.0, f.takeoff_altitude};
            add(t_ns, kActiveAnalytic, encode(reference));
        }
    }
    return records;
}

}  // namespace controller_replay
