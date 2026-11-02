#include "px4_multirotor_controller/drone_ros_node.h"

#include <ros/callback_queue.h>
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
#include <utility>
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
      driver_(sensor_data_),
      controller_(driver_.controller()),
      output_event_executor_(nh) {
    ROS_INFO("[DroneRosNode] Initializing...");

    ros1_utils::getParamWithLog(nh_private_, "debug_print", debug_print_, "Debug print");
    loadControllerConfig();

    auto px4_consumer = std::make_unique<Px4ServiceOutputConsumer>(nh_, output_event_executor_);
    px4_service_consumer_ = px4_consumer.get();
    output_event_dispatcher_.addConsumer(std::move(px4_consumer));
    output_event_dispatcher_.addConsumer(std::make_unique<ControlOutputConsumer>(
        nh_, output_event_executor_, controller_, kRosQueueSize));
    output_event_dispatcher_.addConsumer(std::make_unique<DebugOutputConsumer>(
        nh_, output_event_executor_, controller_, sensor_data_, vrpn_quality_stats_, debug_print_,
        kRosQueueSize));
    output_event_dispatcher_.addConsumer(std::make_unique<ReferenceActivationOutputConsumer>(
        nh_, output_event_executor_, controller_, kRosQueueSize));

    auto post_input_event = [this](::state_machine::Event event) {
        return controller_.getStateMachine().postEvent(std::move(event));
    };

    output_event_dispatcher_.addConsumer(
        std::make_unique<NmpcOutputConsumer>(nh_, driver_.nmpc(), kRosQueueSize));

    sensor_input_producer_ = std::make_unique<SensorInputProducer>(
        nh_, sensor_data_, vrpn_quality_stats_, driver_.statistics(), kRosQueueSize,
        post_input_event,
        [this] {
            if (px4_service_consumer_) {
                px4_service_consumer_->initializeClientsIfNeeded();
            }
        },
        &driver_);
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
        std::make_unique<CommandInputProducer>(nh_, post_input_event, kRosQueueSize);
    trajectory_input_producer_ = std::make_unique<TrajectoryInputProducer>(
        nh_, controller_, sensor_data_, post_input_event, kRosQueueSize);

    output_event_executor_.start();
    sensor_input_producer_->start();
    driver_.start(toCoreTime(ros::Time::now()));

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

    // 显式停止异步输出事件队列
    // 关键：必须在 nh_ 析构前停止，确保后台线程不再访问 nh_
    // stop() 会阻塞等待线程完全退出，并清空待处理队列
    output_event_executor_.stop();
    driver_.stop();

    // 注：生产者、消费者、ROS 订阅者和发布者在析构时自动取消注册。

    ROS_INFO("[DroneRosNode] Shutdown complete");
}

void DroneRosNode::run(double frequency) {
    const double nominal_hz = std::isfinite(frequency) && frequency > 0.0 ? frequency : 100.0;
    ROS_INFO("[DroneRosNode] Starting demand-driven control loop (nominal %.1f Hz)", nominal_hz);

    ::state_machine::adt::AdaptiveScheduleTracker schedule(1.0 / nominal_hz);
    ros::CallbackQueue* queue = ros::getGlobalCallbackQueue();
    double wait_sec = 0.0;

    while (ros::ok()) {
        // 1. 按需阻塞等待回调队列或下一个状态周期截止时间（新消息到达立即0延迟唤醒）
        if (queue != nullptr) {
            queue->callAvailable(ros::WallDuration(wait_sec));
        } else {
            ros::spinOnce();
        }

        // 2. 执行控制循环
        controlLoopCallback();

        // 3. 清除所有新数据标志
        if (sensor_input_producer_) {
            sensor_input_producer_->resetNewFlags();
        }

        // 4. 根据当前状态机的 TickPolicy（SelfCheck/Ready 低频，飞行态全频）计算下次等待时长
        const double now_sec = ros::Time::now().toSec();
        wait_sec = std::max(
            1.0 / nominal_hz,
            schedule.nextWaitDurationSec(controller_.tickPolicy(nominal_hz), now_sec));
    }

    ROS_INFO("[DroneRosNode] Control loop exited");
}

void DroneRosNode::controlLoopCallback() {
    // 高频控制循环

    // 获取当前时间（转换为秒）
    double current_time = ros::Time::now().toSec();

    // 0. Topic stats are written on the ROS side; hand the core its copy.
    if (sensor_input_producer_) {
        sensor_input_producer_->syncStats();
    }

    // 1. 更新控制器（传入当前时间用于频率控制）
    // 传感器数据通过引用自动同步，无需拷贝
    driver_.update(Time(current_time));

    // 2. 消费状态机输出事件，并把阻塞型工作派发给异步执行器（非阻塞）
    dispatchOutputEvents(controller_.getStateMachine().currentOutputEvents());
}

void DroneRosNode::dispatchOutputEvents(const std::vector<::state_machine::Event>& events) {
    const auto result = output_event_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[DroneRosNode] Unhandled output event id: %u", static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[DroneRosNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
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
