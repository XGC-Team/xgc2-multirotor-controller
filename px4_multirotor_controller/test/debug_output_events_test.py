#!/usr/bin/env python3
"""Compile the production debug output consumer against deterministic boundary doubles.

Run: python3 px4_multirotor_controller/test/debug_output_events_test.py
DebugMonitor emits PUBLISH_STATE_MACHINE_EVENTS on every 1 kHz control tick.
This checks that the per-tick alg/state_machine_events work (processed-event
snapshot, task allocation, worker wake-up) happens only while the topic has a
subscriber, and that a subscriber still gets one message per tick with that
tick's event ids. The 5 Hz status outputs keep their unconditional queueing.
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
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>
template <class... Args> inline void rosInfoDouble(const char*, const Args&...) {}
#define ROS_INFO(...) rosInfoDouble(__VA_ARGS__)
namespace ros {
struct Time {
    static Time now() { return Time{}; }
};
}
namespace std_msgs {
struct String { std::string data; };
struct UInt32MultiArray { std::vector<uint32_t> data; };
struct Float32MultiArray { std::vector<float> data; };
}
namespace ros {
using Message = std::variant<std_msgs::String, std_msgs::UInt32MultiArray, std_msgs::Float32MultiArray>;
struct Publication { std::string topic; Message message; };
inline std::vector<Publication>& publications() {
    static std::vector<Publication> value;
    return value;
}
inline uint32_t& subscribers(const std::string& topic) {
    static std::map<std::string, uint32_t> value;
    return value[topic];
}
struct Publisher {
    std::string topic;
    template<class T> void publish(const T& message) const {
        publications().push_back({topic, message});
    }
    uint32_t getNumSubscribers() const { return subscribers(topic); }
};
struct NodeHandle {
    template<class T> Publisher advertise(const std::string& topic, uint32_t) {
        return Publisher{topic};
    }
};
}
namespace ros1_utils {
struct PositionQualityStats {
    bool x_is_valid{true}, y_is_valid{true}, z_is_valid{true}, frame_is_valid{true};
    bool position_jump_detected{false};
    double last_jump_magnitude{0.0}, effective_frequency_hz{-1.0};
    double x_mean_deviation{0.0}, y_mean_deviation{0.0}, z_mean_deviation{0.0};
};
}
namespace state_machine {
struct Event { uint32_t id; };
struct ProcessedEventRecord { Event event; };
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
struct Time {};
inline Time toCoreTime(const ros::Time&) { return Time{}; }
namespace output_event_type {
constexpr uint32_t PUBLISH_CONTROLLER_STATUS = 10007;
constexpr uint32_t PUBLISH_SENSOR_STATS = 10008;
constexpr uint32_t PUBLISH_STATE_MACHINE_EVENTS = 10009;
constexpr uint32_t PRINT_SENSOR_DEBUG = 10010;
constexpr uint32_t PUBLISH_TRACKING_ERROR = 10012;
}
namespace region_type {
constexpr uint32_t CONTROL = 2;
}
struct SensorData {
    struct TopicStats {
        double frequency_hz{-1.0}, dt_max{0.0}, time_since_last_msg{0.0}, jitter{0.0};
        bool is_active{false}, is_new{false};
    };
    double x{0}, y{0}, z{0}, vx{0}, vy{0}, vz{0}, qx{0}, qy{0}, qz{0}, qw{1};
    double wx{0}, wy{0}, wz{0}, battery_percentage{0}, uav_state_estimate_stamp{0};
    uint8_t uav_state_estimator_state{0};
    uint32_t uav_state_estimator_flags{0};
    bool fcu_connected{false}, fcu_armed{false}, fcu_guided{false}, fcu_manual_input{false};
    std::string fcu_mode;
    uint8_t fcu_system_status{0};
    TopicStats uav_state_estimate_stats, local_pos_stats, local_velocity_stats, imu_stats,
        state_stats, battery_stats, vrpn_pose_stats;
};
struct Vector3 {
    double values[3]{0.0, 0.0, 0.0};
    double x() const { return values[0]; }
    double y() const { return values[1]; }
    double z() const { return values[2]; }
};
struct UavReferencePoint { Vector3 position; };
struct StateMachineDouble {
    std::vector<state_machine::ProcessedEventRecord> processed;
    int snapshots{0};
    std::vector<state_machine::ProcessedEventRecord> currentEvents() {
        ++snapshots;
        return processed;
    }
    std::string currentStateName(uint32_t) const { return "Hover"; }
};
struct TrajectoryCacheDouble {
    bool sample(const Time&, UavReferencePoint&) const { return false; }
};
struct DroneController {
    StateMachineDouble machine;
    TrajectoryCacheDouble cache;
    StateMachineDouble& getStateMachine() { return machine; }
    TrajectoryCacheDouble& activeTrajectoryCache() { return cache; }
};
}
'''
TEST = r'''
#include <cassert>
#include <iostream>
#include "px4_multirotor_controller/output/debug_output_consumer.h"
int main() {
    using namespace px4_multirotor_controller;
    ros::NodeHandle nh;
    state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> worker;
    DroneController controller;
    SensorData sensor;
    ros1_utils::PositionQualityStats quality;
    const bool debug_print = false;
    DebugOutputConsumer consumer(nh, worker, controller, sensor, quality, debug_print, 5);
    const state_machine::Event tick{output_event_type::PUBLISH_STATE_MACHINE_EVENTS};
    const std::string topic = "alg/state_machine_events";

    // No subscriber: 1000 ticks are handled with no snapshot, task or publish.
    controller.machine.processed = {{{50}}, {{52}}};
    for (int i = 0; i < 1000; ++i)
        assert(consumer.handle(tick));
    assert(worker.pending.empty());
    assert(controller.machine.snapshots == 0);
    assert(ros::publications().empty());

    // Subscribed: one message per tick with that tick's ids, in order.
    ros::subscribers(topic) = 1;
    for (uint32_t i = 0; i < 3; ++i) {
        controller.machine.processed = {{{50 + i}}, {{55}}};
        assert(consumer.handle(tick));
        assert(worker.pending.size() == 1);
        worker.drain(nh);
        const auto& publication = ros::publications().back();
        assert(publication.topic == topic);
        const auto& msg = std::get<std_msgs::UInt32MultiArray>(publication.message);
        assert((msg.data == std::vector<uint32_t>{50 + i, 55}));
    }
    // A tick without processed events still publishes its empty array.
    controller.machine.processed.clear();
    assert(consumer.handle(tick));
    worker.drain(nh);
    assert(std::get<std_msgs::UInt32MultiArray>(ros::publications().back().message).data.empty());
    assert(ros::publications().size() == 4 && controller.machine.snapshots == 4);

    // Subscriber gone: back to no per-tick work.
    ros::subscribers(topic) = 0;
    assert(consumer.handle(tick));
    assert(worker.pending.empty() && controller.machine.snapshots == 4);

    // The 5 Hz status outputs keep queueing whether or not anyone listens.
    assert(consumer.handle({output_event_type::PUBLISH_CONTROLLER_STATUS}));
    assert(consumer.handle({output_event_type::PUBLISH_SENSOR_STATS}));
    assert(consumer.handle({output_event_type::PUBLISH_TRACKING_ERROR}));
    assert(consumer.handle({output_event_type::PRINT_SENSOR_DEBUG}));
    assert(worker.pending.size() == 3);
    worker.drain(nh);
    assert(ros::publications().size() == 4 + 1 + 8 + 1);
    assert(std::get<std_msgs::String>(ros::publications()[4].message).data == "Hover");
    assert(!consumer.handle({999}));
    std::cout << "1000 unsubscribed ticks: 0 snapshots, 0 tasks; "
                 "4 subscribed ticks: 4 messages in order\n";
}
'''


class DebugOutputEventsTest(unittest.TestCase):
    def test_per_tick_events_work_only_while_subscribed(self):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "C++17 compiler is required")
        with tempfile.TemporaryDirectory(prefix="xgc2-debug-output-test-") as directory:
            root = Path(directory)
            (root / "support.hpp").write_text(SUPPORT)
            for name in (
                "ros/ros.h", "ros1_utils/param_utils.h", "ros1_utils/topic_stats.h",
                "std_msgs/Float32MultiArray.h", "std_msgs/String.h",
                "std_msgs/UInt32MultiArray.h",
                "state_machine/runtime/async_task_executor.hpp",
                "state_machine/runtime/event_dispatcher.hpp",
                "px4_multirotor_controller/drone_controller.h",
                "px4_multirotor_controller/ros_time_conversion.h",
                "px4_multirotor_controller/common/types.h",
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
                str(PACKAGE / "src/output/debug_output_consumer.cpp"), str(main),
                "-o", str(binary),
            ], check=True, timeout=60)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
