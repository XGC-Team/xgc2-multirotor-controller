#pragma once

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/ReferenceStatus.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <state_machine/ordered_worker.hpp>

#include <variant>

namespace multirotor_reference_trajectory {

using ReferenceMessage = std::variant<multirotor_reference_trajectory_msgs::ReferenceStatus,
                                      multirotor_reference_trajectory_msgs::AnalyticReference,
                                      multirotor_reference_trajectory_msgs::SampledReference,
                                      nav_msgs::Path>;
struct ReferencePublication { ros::Publisher publisher; ReferenceMessage message; };

// The reference node's original single FIFO publication worker.
class ReferenceOutputWorker {
   public:
    ReferenceOutputWorker() = default;
    ~ReferenceOutputWorker();
    ReferenceOutputWorker(const ReferenceOutputWorker&) = delete;
    ReferenceOutputWorker& operator=(const ReferenceOutputWorker&) = delete;
    void start();
    void stop();
    void post(ReferencePublication publication);

   private:
    static void execute(const ReferencePublication& publication);
    using Execute = void (*)(const ReferencePublication&);
    state_machine::OrderedWorker<ReferencePublication, Execute> worker_{&ReferenceOutputWorker::execute};
};

}  // namespace multirotor_reference_trajectory
