#pragma once

#include <ros/ros.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>
#include <vector>

#include "multirotor_reference_trajectory/input/reference_input_producer.h"
#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "multirotor_reference_trajectory/output/reference_output_consumer.h"

namespace multirotor_reference_trajectory {

class ReferenceTrajectoryNode {
   public:
    explicit ReferenceTrajectoryNode(ros::NodeHandle& nh);
    ~ReferenceTrajectoryNode();

    void run(double main_frequency_hz);

   private:
    void loadParams();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ReferenceTrajectoryRuntime runtime_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_executor_;
    ::state_machine::runtime::EventDispatcher output_dispatcher_;
    std::unique_ptr<ReferenceInputProducer> input_producer_;

    uint32_t queue_size_{10U};
    std::string analytic_topic_{"alg/multirotor_reference_trajectory/request/analytic"};
    std::string waypoint_topic_{"alg/multirotor_reference_trajectory/request/waypoint"};
    std::string sampled_topic_{"alg/multirotor_reference_trajectory/request/sampled"};
    std::string reset_topic_{"alg/multirotor_reference_trajectory/reset"};
    std::string status_topic_{"alg/multirotor_reference_trajectory/status"};
    std::string active_analytic_topic_{"alg/multirotor_reference_trajectory/active/analytic"};
    std::string active_polynomial_topic_{"alg/multirotor_reference_trajectory/active/polynomial"};
    std::string active_sampled_topic_{"alg/multirotor_reference_trajectory/active/sampled"};
    ReferenceTrajectoryConfig config_{};
};

}  // namespace multirotor_reference_trajectory
