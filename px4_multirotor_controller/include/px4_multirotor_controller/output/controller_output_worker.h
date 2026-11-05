#pragma once

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <ros/ros.h>
#include <ros1_utils/topic_stats.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <state_machine/ordered_worker.hpp>

#include <array>
#include <string>
#include <variant>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

struct OutputArm { bool arm{false}; ros::ServiceClient* client{nullptr}; };
struct OutputMode { std::string mode; ros::ServiceClient* client; };
struct OutputEvents { ros::Publisher publisher; std_msgs::UInt32MultiArray message; };
struct OutputTracking { ros::Publisher publisher; std_msgs::Float32MultiArray message; };
struct OutputReference {
    ros::Publisher publisher;
    multirotor_reference_trajectory_msgs::AnalyticReference message;
};
struct OutputSensorStats {
    std::array<ros::Publisher, 8> publishers;
    std::array<SensorData::TopicStats, 7> stats;
    ros1_utils::PositionQualityStats quality;
};
struct OutputSensorDebug { SensorData sensor; };

// Only the controller's formerly queued FCU/reference/diagnostic work belongs
// here. One FIFO preserves their original order and their existing worker.
using ControllerOutput = std::variant<OutputArm, OutputMode, OutputEvents, OutputTracking,
                                      OutputReference, OutputSensorStats, OutputSensorDebug>;

class ControllerOutputWorker {
   public:
    ControllerOutputWorker() = default;
    ~ControllerOutputWorker();
    ControllerOutputWorker(const ControllerOutputWorker&) = delete;
    ControllerOutputWorker& operator=(const ControllerOutputWorker&) = delete;

    void start();
    void stop();
    void post(ControllerOutput output);

   private:
    static void execute(const ControllerOutput& output);
    using Execute = void (*)(const ControllerOutput&);
    state_machine::OrderedWorker<ControllerOutput, Execute> worker_{&ControllerOutputWorker::execute};
};

}  // namespace px4_multirotor_controller
