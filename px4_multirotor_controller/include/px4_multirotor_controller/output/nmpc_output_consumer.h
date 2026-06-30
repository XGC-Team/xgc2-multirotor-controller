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
#include "px4_multirotor_controller/uav/nmpc_tracking_backend.h"

namespace px4_multirotor_controller {

class NmpcOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    NmpcOutputConsumer(ros::NodeHandle& nh, DroneController& controller, EventSink event_sink,
                       uint32_t queue_size);
    ~NmpcOutputConsumer() override;

    std::string name() const override {
        return "NmpcOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    struct Request {
        uint64_t sequence{0};
        ros::Time now;
        SensorData sensor;
        std::vector<Se3Reference> references;
    };

    void workerLoop();
    void reject(uint64_t sequence, int solver_status);
    void postResultEvent(uint64_t sequence, bool success);
    void publishPrediction(const ros::Time& stamp);

    ros::NodeHandle nh_;
    DroneController& controller_;
    EventSink event_sink_;
    UavNmpcTrackingBackend backend_;
    ros::Publisher predicted_path_pub_;
    ros::Publisher predicted_poses_pub_;

    std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;
    bool stop_{false};
    bool busy_{false};
    bool has_pending_{false};
    Request pending_;
};

}  // namespace px4_multirotor_controller
