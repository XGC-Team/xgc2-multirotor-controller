#pragma once

#include <ros/ros.h>
#include <ros1_utils/topic_stats.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/input/command_input_producer.h"
#include "px4_multirotor_controller/input/sensor_input_producer.h"
#include "px4_multirotor_controller/input/trajectory_input_producer.h"
#include "px4_multirotor_controller/output/px4_service_output_consumer.h"
#include "px4_multirotor_controller/service/runtime_parameter_service.h"

namespace px4_multirotor_controller {

class DroneRosNode {
   public:
    explicit DroneRosNode(ros::NodeHandle& nh);
    ~DroneRosNode();

    void run(double frequency);

   private:
    void controlLoopCallback();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);
    void loadControllerConfig();
    void loadVrpnQualityConfig();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    SensorData sensor_data_;
    DroneController controller_;

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_event_executor_;
    ::state_machine::runtime::EventDispatcher output_event_dispatcher_;
    Px4ServiceOutputConsumer* px4_service_consumer_{nullptr};

    ros1_utils::PositionQualityStats vrpn_quality_stats_;
    bool debug_print_{true};

    std::unique_ptr<SensorInputProducer> sensor_input_producer_;
    std::unique_ptr<CommandInputProducer> command_input_producer_;
    std::unique_ptr<TrajectoryInputProducer> trajectory_input_producer_;
    std::unique_ptr<RuntimeParameterService> runtime_parameter_service_;
};

}  // namespace px4_multirotor_controller
