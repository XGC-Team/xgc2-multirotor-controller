#pragma once

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>
#include <thread>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/driver/nmpc_execution.h"

namespace px4_multirotor_controller {

class NmpcOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    NmpcOutputConsumer(ros::NodeHandle& nh, NmpcExecution& execution,
                       uint32_t queue_size);
    ~NmpcOutputConsumer() override;

    std::string name() const override {
        return "NmpcOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    void publishDebug(uint64_t sequence, const Time& stamp, const UavNmpcTrackingBackend& backend);
    void publishPrediction(const Time& stamp, const UavNmpcTrackingBackend& backend);
    NmpcExecution& execution_;
    ros::Publisher debug_pub_;
    ros::Publisher predicted_path_pub_;
    ros::Publisher predicted_poses_pub_;


};

}  // namespace px4_multirotor_controller
