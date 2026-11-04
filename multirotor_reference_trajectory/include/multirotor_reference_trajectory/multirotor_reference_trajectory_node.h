#pragma once

#include <ros/ros.h>
#include <ros/callback_queue_interface.h>

#include <memory>
#include <optional>
#include <state_machine/runtime/async_task_executor.hpp>
#include <string>

#include "multirotor_reference_trajectory/input/reference_input_producer.h"
#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "multirotor_reference_trajectory/output/reference_output_consumer.h"

namespace multirotor_reference_trajectory {

class ReferenceTrajectoryNode {
   public:
    explicit ReferenceTrajectoryNode(ros::NodeHandle& nh);
    ~ReferenceTrajectoryNode();

    void run();

   private:
    void loadParams();
    void requestTurn();
    void runTurn();
    void onDeadline(const ros::TimerEvent&);
    void armDeadline(double now);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ReferenceTrajectoryRuntime runtime_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_executor_;
    std::unique_ptr<ReferenceOutputConsumer> output_consumer_;
    std::unique_ptr<ReferenceInputProducer> input_producer_;
    ros::CallbackInterfacePtr turn_callback_;
    ros::Timer deadline_timer_;
    std::optional<double> armed_deadline_;
    bool turn_queued_{false};

    uint32_t queue_size_{10U};
    std::string analytic_topic_{"alg/multirotor_reference_trajectory/request/analytic"};
    std::string sampled_topic_{"alg/multirotor_reference_trajectory/request/sampled"};
    std::string reset_topic_{"alg/multirotor_reference_trajectory/reset"};
    std::string status_topic_{"alg/multirotor_reference_trajectory/status"};
    std::string active_analytic_topic_{"alg/multirotor_reference_trajectory/active/analytic"};
    std::string active_sampled_topic_{"alg/multirotor_reference_trajectory/active/sampled"};
    std::string reference_path_topic_{
        "alg/multirotor_reference_trajectory/visualization/reference_path"};
    double reference_path_sample_dt_{0.02};
    double reference_path_preview_duration_{20.0};
    ReferenceTrajectoryConfig config_{};
};

}  // namespace multirotor_reference_trajectory
