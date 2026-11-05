#pragma once

#include <ros/ros.h>
#include <ros/callback_queue_interface.h>
#include <ros1_utils/topic_stats.h>
#include <state_machine/ros1_queued_owner.hpp>

#include <memory>
#include <mutex>
#include <optional>

#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/input/command_input_producer.h"
#include "px4_multirotor_controller/input/sensor_input_producer.h"
#include "px4_multirotor_controller/input/trajectory_input_producer.h"
#include "px4_multirotor_controller/output/px4_service_output_consumer.h"

namespace px4_multirotor_controller {

class ControlOutputConsumer;
class DebugOutputConsumer;
class NmpcOutputConsumer;
class ReferenceActivationOutputConsumer;

class DroneRosNode {
   public:
    explicit DroneRosNode(ros::NodeHandle& nh);
    ~DroneRosNode();

    void run();

   private:
    void requestTurn();
    void runTurn();
    void armDeadline();
    void dispatchOutputEffects(const std::vector<FlightEffect>& effects);
    void loadControllerConfig();
    void loadVrpnQualityConfig();
    void publishLoadFact(const std::string& vrpn_pose_topic,
                         const std::string& state_estimate_topic);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher record_facts_publisher_;
    ControllerLoadFact load_fact_;
    bool load_fact_pending_{false};

    SensorData sensor_data_;
    ControllerDriver driver_;
    DroneController& controller_;

    ControllerOutputWorker output_worker_;
    std::unique_ptr<Px4ServiceOutputConsumer> px4_service_consumer_;
    std::unique_ptr<ControlOutputConsumer> control_consumer_;
    std::unique_ptr<DebugOutputConsumer> debug_consumer_;
    std::unique_ptr<ReferenceActivationOutputConsumer> reference_consumer_;
    std::unique_ptr<NmpcOutputConsumer> nmpc_consumer_;

    ros1_utils::PositionQualityStats vrpn_quality_stats_;
    bool debug_print_{true};

    std::unique_ptr<SensorInputProducer> sensor_input_producer_;
    std::unique_ptr<CommandInputProducer> command_input_producer_;
    std::unique_ptr<TrajectoryInputProducer> trajectory_input_producer_;
    std::optional<state_machine::Ros1QueuedTurn> turn_;
    std::optional<state_machine::Ros1DomainDeadline<uint64_t>> deadline_;
    std::mutex turn_mutex_;
    bool accepting_turns_{true};
};

}  // namespace px4_multirotor_controller
