#!/usr/bin/env python3
"""Compile the production output consumer against deterministic boundary doubles.

Run: python3 px4_multirotor_controller/test/control_output_scheduling_test.py
This tests dispatch/message construction, not roscpp transport timing or SITL.
The executor double deliberately cannot run queued tasks until drain() is called.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SUPPORT = r'''
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>
namespace ros {
struct Time {
    double value{0.0};
    static Time now() { return Time{clock()}; }
    static double& clock() { static double t = 10.0; return t; }
};
}
struct Header { ros::Time stamp; std::string frame_id; };
struct Vec3 { double x{0}, y{0}, z{0}; };
struct Quaternion { double x{0}, y{0}, z{0}, w{0}; };
namespace mavros_msgs {
struct PositionTarget {
    Header header;
    uint8_t coordinate_frame{0};
    uint16_t type_mask{0};
    Vec3 position, velocity, acceleration_or_force;
    double yaw{0}, yaw_rate{0};
};
struct AttitudeTarget {
    static constexpr uint8_t IGNORE_ATTITUDE = 128;
    Header header;
    uint8_t type_mask{0};
    Quaternion orientation;
    Vec3 body_rate;
    double thrust{0};
};
}
namespace ros {
using Message = std::variant<mavros_msgs::PositionTarget, mavros_msgs::AttitudeTarget>;
struct Publication { std::string topic; Message message; };
inline std::vector<Publication>& publications() {
    static std::vector<Publication> value;
    return value;
}
struct Publisher {
    std::string topic;
    template<class T> void publish(T message) const {
        publications().push_back({topic, std::move(message)});
    }
};
struct NodeHandle {
    template<class T> Publisher advertise(const std::string& topic, uint32_t) {
        return Publisher{topic};
    }
};
}
namespace state_machine {
struct Event { uint32_t id; };
namespace runtime {
struct EventConsumer {
    virtual ~EventConsumer() = default;
    virtual std::string name() const = 0;
    virtual bool handle(const Event&) = 0;
};
template<class Context> struct Task {
    virtual ~Task() = default;
    virtual void execute(Context&) = 0;
};
template<class Context> struct LambdaTask : Task<Context> {
    std::function<void(Context&)> fn;
    LambdaTask(std::string, std::function<void(Context&)> callback) : fn(std::move(callback)) {}
    void execute(Context& context) override { fn(context); }
};
template<class Context> struct AsyncTaskExecutor {
    std::vector<std::unique_ptr<Task<Context>>> pending;
    void pushTask(std::unique_ptr<Task<Context>> task) { pending.push_back(std::move(task)); }
    void drain(Context& context) {
        for (auto& task : pending) task->execute(context);
        pending.clear();
    }
};
}
}
namespace px4_multirotor_controller {
namespace output_event_type {
constexpr uint32_t PUBLISH_SETPOINT = 101;
constexpr uint32_t PUBLISH_ATTITUDE_RATE_TARGET = 102;
}
struct Setpoint {
    double x{0}, y{0}, z{0}, vx{0}, vy{0}, vz{0}, ax{0}, ay{0}, az{0};
    double qx{0}, qy{0}, qz{0}, qw{1}, yaw_rate{0};
    uint8_t coordinate_frame{1};
    uint16_t type_mask{3072};
};
struct AttitudeRateTarget {
    double body_rate_x{0}, body_rate_y{0}, body_rate_z{0}, thrust{0};
};
struct DroneController {
    Setpoint setpoint;
    AttitudeRateTarget attitude;
    const Setpoint& getSetpoint() const { return setpoint; }
    const AttitudeRateTarget& getAttitudeRateTarget() const { return attitude; }
};
inline double clamp(double v, double lo, double hi) { return std::clamp(v, lo, hi); }
inline double quaternionToYaw(double x, double y, double z, double w) {
    return std::atan2(2 * (w*z+x*y), 1 - 2 * (y*y+z*z));
}
}
'''
TEST = r'''
#include <cassert>
#include <iostream>
#include "px4_multirotor_controller/output/control_output_consumer.h"
int main() {
    using namespace px4_multirotor_controller;
    ros::NodeHandle nh;
    state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> rpc;
    DroneController controller;
    ControlOutputConsumer consumer(nh, rpc, controller, 5);
    // The RPC worker is unavailable throughout this entire producer sequence.
    for (int i = 0; i < 50; ++i) {
        ros::Time::clock() = 10.0 + i * 0.02;
        controller.setpoint.x = i;
        controller.setpoint.y = -i;
        controller.setpoint.z = 2;
        controller.setpoint.vx = .3;
        controller.setpoint.ay = .4;
        controller.setpoint.type_mask = 3523;
        assert(consumer.handle({output_event_type::PUBLISH_SETPOINT}));
        assert(ros::publications().size() == static_cast<size_t>(i + 1));
        const auto& publication = ros::publications().back();
        assert(publication.topic == "mavros/setpoint_raw/local");
        const auto& msg = std::get<mavros_msgs::PositionTarget>(publication.message);
        assert(msg.position.x == i && msg.position.y == -i && msg.position.z == 2);
        assert(msg.velocity.x == .3 && msg.acceleration_or_force.y == .4);
        assert(msg.type_mask == 3523 && msg.coordinate_frame == 1);
        assert(msg.header.frame_id == "map" && msg.header.stamp.value == ros::Time::clock());
    }
    controller.attitude = {.1, -.2, .3, 1.5};
    assert(consumer.handle({output_event_type::PUBLISH_ATTITUDE_RATE_TARGET}));
    auto msg = std::get<mavros_msgs::AttitudeTarget>(ros::publications().back().message);
    assert(ros::publications().back().topic == "mavros/setpoint_raw/attitude");
    assert(msg.type_mask == mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE);
    assert(msg.body_rate.x == .1 && msg.body_rate.y == -.2 && msg.body_rate.z == .3);
    assert(msg.orientation.w == 1 && msg.thrust == 1);
    controller.attitude.thrust = -.1;
    consumer.handle({output_event_type::PUBLISH_ATTITUDE_RATE_TARGET});
    assert(std::get<mavros_msgs::AttitudeTarget>(ros::publications().back().message).thrust == 0);
    assert(!consumer.handle({999}));
    assert(rpc.pending.empty());
    const auto count = ros::publications().size();
    rpc.drain(nh); // RPC recovery must not replay any old local/attitude targets.
    assert(ros::publications().size() == count);
    std::cout << "52 immediate publications; 0 RPC-queued samples; 0 recovery replays\n";
}
'''


class ControlOutputSchedulingTest(unittest.TestCase):
    def test_real_consumer_does_not_queue_control_behind_rpc(self):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "C++17 compiler is required")
        with tempfile.TemporaryDirectory(prefix="xgc2-output-test-") as directory:
            root = Path(directory)
            (root / "support.hpp").write_text(SUPPORT)
            for name in (
                "ros/ros.h", "mavros_msgs/AttitudeTarget.h", "mavros_msgs/PositionTarget.h",
                "state_machine/runtime/async_task_executor.hpp",
                "state_machine/runtime/event_dispatcher.hpp",
                "px4_multirotor_controller/drone_controller.h",
                "px4_multirotor_controller/common/types.h",
                "px4_multirotor_controller/nmpc/nmpc_math_utils.h",
            ):
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#include "support.hpp"\n')
            main = root / "test.cpp"
            main.write_text(TEST)
            binary = root / "test"
            subprocess.run([
                compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
                "-I", str(root), "-I", str(PACKAGE / "include"),
                str(PACKAGE / "src/output/control_output_consumer.cpp"), str(main),
                "-o", str(binary),
            ], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
