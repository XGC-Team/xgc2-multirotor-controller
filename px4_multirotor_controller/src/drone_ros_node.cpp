#include "px4_multirotor_controller/drone_ros_node.h"

#include <ros1_utils/namespace_utils.h>
#include <ros1_utils/param_utils.h>
#include <std_msgs/String.h>
#include <time.h>

#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/output/control_output_consumer.h"
#include "px4_multirotor_controller/output/debug_output_consumer.h"
#include "px4_multirotor_controller/output/nmpc_output_consumer.h"
#include "px4_multirotor_controller/output/reference_activation_output_consumer.h"
#include "px4_multirotor_controller/ros_time_conversion.h"
#include "xgc2_math/geometry/math_helpers.h"

namespace px4_multirotor_controller {
namespace {

constexpr uint32_t kRosQueueSize = 5;

std::uint64_t clockNanoseconds(clockid_t clock) {
    timespec stamp{};
    if (clock_gettime(clock, &stamp) != 0) {
        throw std::runtime_error("cannot capture original controller load clock");
    }
    return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000000ULL +
           static_cast<std::uint64_t>(stamp.tv_nsec);
}

std::string resolveTopicName(const ros::NodeHandle& nh, const std::string& topic) {
    return nh.resolveName(topic);
}

}  // namespace

DroneRosNode::DroneRosNode(ros::NodeHandle& nh)
    : nh_(nh),
      nh_private_("~"),
      driver_(sensor_data_, [this] { requestTurn(); }),
      controller_(driver_.controller()) {
    ROS_INFO("[DroneRosNode] Initializing...");

    ros1_utils::getParamWithLog(nh_private_, "debug_print", debug_print_, "Debug print");
    loadControllerConfig();
    turn_.emplace(*nh_.getCallbackQueue(), [this] { runTurn(); });
    deadline_.emplace(nh_, [this] { requestTurn(); });

    px4_service_consumer_ = std::make_unique<Px4ServiceOutputConsumer>(nh_, output_worker_);
    control_consumer_ = std::make_unique<ControlOutputConsumer>(
        nh_, controller_, kRosQueueSize);
    debug_consumer_ = std::make_unique<DebugOutputConsumer>(
        nh_, output_worker_, controller_, sensor_data_, vrpn_quality_stats_, debug_print_,
        kRosQueueSize);
    reference_consumer_ = std::make_unique<ReferenceActivationOutputConsumer>(
        nh_, output_worker_, controller_, kRosQueueSize);
    nmpc_consumer_ = std::make_unique<NmpcOutputConsumer>(nh_, driver_.nmpc(), kRosQueueSize);

    auto post_command = [this](FlightCommand command, double receipt_sec) {
        const bool accepted = controller_.postCommand(command, receipt_sec);
        if (accepted) requestTurn();
        return accepted;
    };

    sensor_input_producer_ = std::make_unique<SensorInputProducer>(
        nh_, sensor_data_, vrpn_quality_stats_, driver_.statistics(), kRosQueueSize,
        [this] {
            if (px4_service_consumer_) {
                px4_service_consumer_->initializeClientsIfNeeded();
            }
        },
        &driver_, [this] { requestTurn(); });
    loadVrpnQualityConfig();
    std::string state_estimate_topic;
    std::string vrpn_pose_topic;
    nh_private_.param<std::string>("state_estimate_topic", state_estimate_topic,
                                   "alg/state_estimator/state");
    nh_private_.param<std::string>("vrpn_pose_topic", vrpn_pose_topic, "pose");
    sensor_input_producer_->setStateEstimateTopic(state_estimate_topic);
    sensor_input_producer_->setVrpnPoseTopic(vrpn_pose_topic);
    // Receipt topics are the ones just handed to the producer at its set
    // boundary, never a second parameter observation made earlier at setConfig.
    publishLoadFact(vrpn_pose_topic, state_estimate_topic);

    command_input_producer_ =
        std::make_unique<CommandInputProducer>(nh_, post_command, kRosQueueSize);
    trajectory_input_producer_ = std::make_unique<TrajectoryInputProducer>(
        nh_, controller_, sensor_data_, kRosQueueSize, [this] { requestTurn(); });

    output_worker_.start();
    sensor_input_producer_->start();
    driver_.start(toCoreTime(ros::Time::now()));
    requestTurn();

    ROS_INFO("[DroneRosNode] Initialized (with async output event executor)");
    ROS_INFO("[DroneRosNode] Subscribed topics:");
    if (sensor_data_.feedback_source == FeedbackSource::RIGID_ESTIMATE)
        ROS_INFO("  - %s (control state)", resolveTopicName(nh_, state_estimate_topic).c_str());
    else
        ROS_INFO("  - mavros_local control state (declared local ENU frame)");
    ROS_INFO("  - mavros/local_position/pose");
    ROS_INFO("  - mavros/local_position/velocity_local");
    ROS_INFO("  - mavros/imu/data");
    ROS_INFO("  - mavros/state");
    ROS_INFO("  - mavros/battery (telemetry only)");
    ROS_INFO("  - %s (check only)", resolveTopicName(nh_, vrpn_pose_topic).c_str());
    ROS_INFO("  - alg/setpoint_raw/local");
    ROS_INFO("  - alg/multirotor_reference_trajectory/active/analytic");
    ROS_INFO("  - alg/multirotor_reference_trajectory/active/sampled");
    ROS_INFO("  - %s", driver_.internalHoverThrust()
                           ? "mavros/setpoint_raw/target_attitude (internal hover-thrust runtime)"
                           : "hover_thrust/estimate_state");
    ROS_INFO("  - /command (global)");
    ROS_INFO("[DroneRosNode] Publishing topics:");
    ROS_INFO("  - mavros/setpoint_raw/local");
    ROS_INFO("  - mavros/setpoint_raw/attitude");
    ROS_INFO("  - custom/statustext");
    ROS_INFO("  - alg/multirotor_reference_trajectory/request/analytic");
}

DroneRosNode::~DroneRosNode() {
    ROS_INFO("[DroneRosNode] Shutting down...");

    deadline_->cancel();
    {
        std::lock_guard<std::mutex> lock(turn_mutex_);
        accepting_turns_ = false;
        turn_->cancel();
    }
    // 显式停止异步输出事件队列
    // 关键：必须在 nh_ 析构前停止，确保后台线程不再访问 nh_
    // stop() 会阻塞等待线程完全退出，并清空待处理队列
    output_worker_.stop();
    driver_.stop();

    // 注：生产者、消费者、ROS 订阅者和发布者在析构时自动取消注册。

    ROS_INFO("[DroneRosNode] Shutdown complete");
}

void DroneRosNode::run() {
    ros::spin();
}

void DroneRosNode::requestTurn() {
    std::lock_guard<std::mutex> lock(turn_mutex_);
    if (!accepting_turns_)
        return;
    turn_->request();
}

void DroneRosNode::runTurn() {
    {
        std::lock_guard<std::mutex> lock(turn_mutex_);
        turn_->beginTurn();
    }
    const double current_time = ros::Time::now().toSec();
    sensor_input_producer_->syncStats();
    const bool continuation = driver_.update(Time(current_time));
    dispatchOutputEffects(controller_.flightEffects());
    sensor_input_producer_->resetNewFlags();
    armDeadline();
    if (continuation) requestTurn();
}

void DroneRosNode::armDeadline() {
    const auto next = driver_.nextWake(true);
    const uint64_t now = ros::Time::now().toNSec();
    if (!next || *next <= now) {
        deadline_->cancel();
        if (next) requestTurn();
        return;
    }
    deadline_->arm(*next, [next, now] {
        ros::Duration delay;
        delay.fromNSec(static_cast<int64_t>(*next - now));
        return delay;
    });
}

void DroneRosNode::dispatchOutputEffects(const std::vector<FlightEffect>& effects) {
    for (const auto& effect : effects) {
        try {
            if (px4_service_consumer_->handle(effect) || control_consumer_->handle(effect) ||
                debug_consumer_->handle(effect) || reference_consumer_->handle(effect) ||
                nmpc_consumer_->handle(effect)) {
                continue;
            }
            ROS_WARN("[DroneRosNode] Unhandled output effect");
        } catch (const std::exception& ex) {
            ROS_WARN("[DroneRosNode] Output consumer failed: %s", ex.what());
        } catch (...) {
            ROS_WARN("[DroneRosNode] Output consumer failed with unknown exception");
        }
    }
}

void DroneRosNode::loadControllerConfig() {
    ControllerParameters parameters(
        [this](const std::string& key, ControllerParameterValue& value) {
            return std::visit([this, &key](auto& out) { return nh_private_.getParam(key, out); },
                              value);
        },
        [](bool warning, const std::string& text) {
            if (warning)
                ROS_WARN("%s", text.c_str());
            else
                ROS_INFO("%s", text.c_str());
        },
        false);
    const ControllerConfig config =
        readControllerConfig(parameters, ros1_utils::currentNameFromNamespacePrefix("/uav"));
    const std::string tracking_backend =
        config.tracking_backend == TrackingBackend::NMPC
            ? "nmpc"
            : config.tracking_backend == TrackingBackend::DFBC
                  ? "dfbc"
                  : config.tracking_backend == TrackingBackend::SMC ? "smc" : "px4_local";
    // 将配置传递给控制器
    controller_.setConfig(config);
    sensor_data_.feedback_source = config.feedback_source;
    // Capture the original load clocks at the setConfig boundary only. The
    // receipt is published after the sensor producer accepts its topics.
    try {
        load_fact_.ros_time_ns = ros::Time::now().toNSec();
        load_fact_.unix_time_ns = clockNanoseconds(CLOCK_REALTIME);
        load_fact_.monotonic_time_ns = clockNanoseconds(CLOCK_MONOTONIC);
        load_fact_.node_name = ros::this_node::getName();
        load_fact_.instance_id = boost::uuids::to_string(boost::uuids::random_generator()());
        load_fact_.tracking_backend = tracking_backend;
        load_fact_.position_distance_limit_metres =
            HealthMonitorState::kPositionDistanceLimitMetres;
        load_fact_pending_ = true;
    } catch (const std::exception&) {
        load_fact_pending_ = false;
        ROS_WARN("[DroneRosNode] Loaded controller configuration; Record load receipt unavailable");
    }
}

void DroneRosNode::publishLoadFact(const std::string& vrpn_pose_topic,
                                   const std::string& state_estimate_topic) {
    if (!load_fact_pending_) {
        return;
    }
    load_fact_pending_ = false;
    // One latched load receipt, outside the control loop. A late recorder gets
    // these original clocks, never a fabricated new application timestamp.
    // Receipt transport is evidence only and cannot change flight admission.
    try {
        load_fact_.canonical_pose_topic = nh_.resolveName(vrpn_pose_topic);
        load_fact_.local_pose_topic = nh_.resolveName("mavros/local_position/pose");
        load_fact_.world_pose_topic =
            trackingUsesFusedEstimate(controller_.getConfig().tracking_backend) &&
                    controller_.getConfig().feedback_source == FeedbackSource::RIGID_ESTIMATE
                ? nh_.resolveName(state_estimate_topic)
                : load_fact_.local_pose_topic;
        std_msgs::String message;
        message.data =
            controllerLoadFactJSON(controller_.getConfig().safety.world_boundary, load_fact_);
        record_facts_publisher_ = nh_.advertise<std_msgs::String>("/xgc/record_facts", 1, true);
        record_facts_publisher_.publish(message);
    } catch (const std::exception&) {
        ROS_WARN("[DroneRosNode] Loaded controller configuration; Record load receipt unavailable");
    }
}

void DroneRosNode::loadVrpnQualityConfig() {
    // 读取私有参数（使用私有命名空间句柄）
    ros1_utils::PositionQualityConfig config;
    ros1_utils::getParamWithLog(nh_private_, "vrpn_quality_window_size", config.window_size,
                                "VRPN window size");
    ros1_utils::getParamWithLog(nh_private_, "vrpn_duplicate_threshold", config.duplicate_threshold,
                                "VRPN dup threshold");
    ros1_utils::getParamWithLog(nh_private_, "vrpn_jump_threshold", config.jump_threshold,
                                "VRPN jump threshold");

    if (sensor_input_producer_) {
        sensor_input_producer_->setVrpnQualityConfig(config);
    }
}

}  // namespace px4_multirotor_controller
