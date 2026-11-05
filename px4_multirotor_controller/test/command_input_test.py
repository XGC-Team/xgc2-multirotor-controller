#!/usr/bin/env python3
"""Exercise the production command callback through a deterministic ROS boundary."""
from pathlib import Path
import subprocess
import tempfile

PACKAGE = Path(__file__).resolve().parents[1]
SUPPORT = r'''
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
namespace std_msgs { struct String { using ConstPtr = std::shared_ptr<const String>; std::string data; }; }
namespace state_machine {
using EventId = uint32_t;
struct Status { bool ok() const { return true; } std::string message; };
struct EventTimestamp { double seconds; };
struct Event { EventId id; EventTimestamp stamp; std::string source;
Event(EventId value, EventTimestamp time): id(value), stamp(time) {} };
}
namespace ros {
struct Subscriber {};
struct Time { static Time now() { return {}; } double toSec() const { return 1; } };
struct NodeHandle {
std::function<void(const std_msgs::String::ConstPtr&)> callback;
template<class T> Subscriber subscribe(const char*, uint32_t,
 void (T::*method)(const std_msgs::String::ConstPtr&), T* object) {
 callback = [=](const std_msgs::String::ConstPtr& value){ (object->*method)(value); }; return {};
}
};
}
#define ROS_ERROR(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_INFO(...) ((void)0)
#define ROS_DEBUG(...) ((void)0)
namespace px4_multirotor_controller { namespace event_type {
constexpr uint32_t TAKEOFF_REQUESTED=1, LANDING_REQUESTED=2, HOVER_REQUESTED=3, TRAJECTORY_TRACKING_REQUESTED=4;
} }
'''
MAIN = r'''
#include <cassert>
#include <vector>
#include "px4_multirotor_controller/input/command_input_producer.h"
#include "px4_multirotor_controller/common/types.h"
int main() {
 using px4_multirotor_controller::FlightCommand;
 ros::NodeHandle node; std::vector<std::pair<FlightCommand, double>> commands;
 px4_multirotor_controller::CommandInputProducer input(node, [&](FlightCommand command, double time) {
 commands.emplace_back(command, time); return true; }, 10);
 auto send=[&](const std::string& command) { auto m=std::make_shared<std_msgs::String>(); m->data=command; node.callback(m); };
 for(auto command:{"custom1","Custom1","CUSTOM1","start","Start","START","track","Track","TRACK"}) { send(command); assert(commands.back().first==FlightCommand::Track); }
 for(auto command:{"land","Land","LAND"}) { send(command); assert(commands.back().first==FlightCommand::Land); }
 for(auto command:{"hover","Hover","HOVER"}) { send(command); assert(commands.back().first==FlightCommand::Hover); }
 for(auto command:{"takeoff","Takeoff","TAKEOFF"}) { send(command); assert(commands.back().first==FlightCommand::Takeoff); }
 const auto count=commands.size(); send("stop"); send("Stop"); send("STOP"); send("unknown"); send("tAkEoFf"); send(""); node.callback(nullptr); assert(commands.size()==count);
 for(const auto& command:commands) assert(command.second==1.0);
}
'''
with tempfile.TemporaryDirectory(prefix="xgc-command-input-") as directory:
    root = Path(directory)
    (root / "support.h").write_text(SUPPORT)
    for name in ["ros/ros.h", "std_msgs/String.h", "state_machine/state_machine.hpp", "px4_multirotor_controller/common/types.h"]:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('#include "support.h"\n')
    (root / "main.cpp").write_text(MAIN)
    binary = root / "test"
    subprocess.run(["g++", "-std=c++17", "-I" + str(root), "-I" + str(PACKAGE / "include"), str(PACKAGE / "src/input/command_input_producer.cpp"), str(root / "main.cpp"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("Production command callback: custom1 tracks, land lands, stop is not remapped, Hover remains independent")
