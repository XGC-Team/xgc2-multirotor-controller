#include "px4_multirotor_controller/drone_ros_node.h"

#include <ros1_utils/namespace_utils.h>
#include <ros1_utils/param_utils.h>

#include <cmath>
#include <memory>
#include <utility>

#include "px4_multirotor_controller/output/control_output_consumer.h"
#include "px4_multirotor_controller/output/debug_output_consumer.h"
#include "px4_multirotor_controller/output/nmpc_output_consumer.h"
#include "px4_multirotor_controller/output/reference_activation_output_consumer.h"
#include "xgc2_math/geometry/math_helpers.h"

namespace px4_multirotor_controller {
namespace {

constexpr uint32_t kRosQueueSize = 5;

std::string resolveTopicName(const ros::NodeHandle& nh, const std::string& topic) {
    return nh.resolveName(topic);
}

}  // namespace

DroneRosNode::DroneRosNode(ros::NodeHandle& nh)
    : nh_(nh),
      nh_private_("~"),
      controller_(sensor_data_),  // 传递 sensor_data_ 引用给控制器
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
        std::make_unique<NmpcOutputConsumer>(controller_, post_input_event));

    sensor_input_producer_ = std::make_unique<SensorInputProducer>(
        nh_, sensor_data_, vrpn_quality_stats_, kRosQueueSize, post_input_event, [this] {
            if (px4_service_consumer_) {
                px4_service_consumer_->initializeClientsIfNeeded();
            }
        });
    loadVrpnQualityConfig();
    std::string state_estimate_topic;
    std::string vrpn_pose_topic;
    std::string vrpn_twist_topic;
    nh_private_.param<std::string>("state_estimate_topic", state_estimate_topic,
                                   "alg/state_estimator/state");
    nh_private_.param<std::string>("vrpn_pose_topic", vrpn_pose_topic, "pose");
    nh_private_.param<std::string>("vrpn_twist_topic", vrpn_twist_topic, "twist");
    sensor_input_producer_->setStateEstimateTopic(state_estimate_topic);
    sensor_input_producer_->setVrpnTopics(vrpn_pose_topic, vrpn_twist_topic);

    command_input_producer_ =
        std::make_unique<CommandInputProducer>(nh_, post_input_event, kRosQueueSize);
    trajectory_input_producer_ = std::make_unique<TrajectoryInputProducer>(
        nh_, sensor_data_, controller_.activeTrajectoryCache(),
        [this] { return controller_.getConfig(); }, post_input_event,
        [this](const MpcTrajectoryState& trajectory) {
            controller_.mpcTrajectoryBuffer().cachePending(trajectory);
        },
        kRosQueueSize);
    runtime_parameter_service_ =
        std::make_unique<RuntimeParameterService>(nh_private_, controller_);

    output_event_executor_.start();
    sensor_input_producer_->start();

    ROS_INFO("[DroneRosNode] Initialized (with async output event executor)");
    ROS_INFO("[DroneRosNode] Subscribed topics:");
    ROS_INFO("  - %s (control state)", resolveTopicName(nh_, state_estimate_topic).c_str());
    ROS_INFO("  - mavros/local_position/pose (check only)");
    ROS_INFO("  - mavros/local_position/velocity_local (check only)");
    ROS_INFO("  - mavros/imu/data (check only)");
    ROS_INFO("  - mavros/state");
    ROS_INFO("  - mavros/battery");
    ROS_INFO("  - %s (check only)", resolveTopicName(nh_, vrpn_pose_topic).c_str());
    ROS_INFO("  - %s (check only)", resolveTopicName(nh_, vrpn_twist_topic).c_str());
    ROS_INFO("  - alg/setpoint_raw/local");
    ROS_INFO("  - alg/multirotor_reference_trajectory/active/analytic");
    ROS_INFO("  - alg/multirotor_reference_trajectory/active/polynomial");
    ROS_INFO("  - alg/multirotor_reference_trajectory/active/sampled");
    ROS_INFO("  - hover_thrust/estimate_state");
    ROS_INFO("  - /command (global)");
    ROS_INFO("[DroneRosNode] Services:");
    ROS_INFO("  - ~set_runtime_parameters");
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

    // 注：生产者、消费者、ROS 订阅者和发布者在析构时自动取消注册。

    ROS_INFO("[DroneRosNode] Shutdown complete");
}

void DroneRosNode::run(double frequency) {
    ROS_INFO("[DroneRosNode] Starting control loop at %.1f Hz", frequency);

    // ============================================================
    // 线程模型：ROS/FSM 主循环单线程，输出事件由单 worker 异步消费
    // ============================================================
    // 执行顺序：
    // 1. ros::spinOnce()      - 处理所有待处理的ROS回调（非阻塞）
    // 2. controlLoopCallback() - 执行控制逻辑
    // 3. resetNewFlags()       - 清除新数据标志
    // 4. rate.sleep()          - 维持固定频率
    //
    // 线程安全保证：
    // - ROS 订阅回调、上下文更新、FSM update 在主线程中串行执行
    // - 输出消费者在主线程快照必要数据，再把发布/服务任务放入 worker
    // ============================================================

    ros::Rate rate(frequency);

    while (ros::ok()) {
        // 处理ROS回调（订阅者消息）
        ros::spinOnce();

        // 执行控制循环
        controlLoopCallback();

        // 清除所有新数据标志
        if (sensor_input_producer_) {
            sensor_input_producer_->resetNewFlags();
        }

        // 按设定频率休眠
        rate.sleep();
    }

    ROS_INFO("[DroneRosNode] Control loop exited");
}

void DroneRosNode::controlLoopCallback() {
    // 高频控制循环

    // 获取当前时间（转换为秒）
    double current_time = ros::Time::now().toSec();

    // 1. 更新控制器（传入当前时间用于频率控制）
    // 传感器数据通过引用自动同步，无需拷贝
    controller_.update(current_time);

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
    // 读取私有参数（使用私有命名空间句柄）
    ControllerConfig config;
    ros1_utils::getParamWithLog(nh_private_, "takeoff_altitude", config.takeoff_altitude,
                                "Takeoff altitude (m)");
    const std::string uav_name = ros1_utils::currentNameFromNamespacePrefix("/uav");
    double per_uav_takeoff_altitude = config.takeoff_altitude;
    if (!uav_name.empty() &&
        nh_private_.getParam("takeoff_altitudes/" + uav_name, per_uav_takeoff_altitude)) {
        if (std::isfinite(per_uav_takeoff_altitude) && per_uav_takeoff_altitude > 0.0) {
            config.takeoff_altitude = per_uav_takeoff_altitude;
            ROS_INFO("[DroneRosNode] Per-UAV takeoff altitude for %s: %.3f m", uav_name.c_str(),
                     config.takeoff_altitude);
        } else {
            ROS_WARN(
                "[DroneRosNode] Invalid per-UAV takeoff altitude for %s: %.3f, "
                "keeping %.3f m",
                uav_name.c_str(), per_uav_takeoff_altitude, config.takeoff_altitude);
        }
    }
    nh_private_.param("skip_takeoff_init_disarm", config.skip_takeoff_init_disarm,
                      config.skip_takeoff_init_disarm);
    ROS_INFO("[DroneRosNode] Skip TakeoffInit DISARM and ALTCTL gate: %s",
             config.skip_takeoff_init_disarm ? "enabled" : "disabled");

    config.planning_period = nh_private_.param(
        "planning_period", nh_private_.param("sampling_time", config.planning_period));
    if (!std::isfinite(config.planning_period) || config.planning_period <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid planning_period/sampling_time; using 0.100 s");
        config.planning_period = 0.1;
    }
    ROS_INFO("[DroneRosNode] MPC planning period: %.3f s", config.planning_period);

    // ========== MPC轨迹跟踪控制模式 ==========
    // 读取控制模式 (0=PX4_CASCADE_PID, 1=PURE_SLIDING_MODE, 2=HYBRID_CONTROL)
    int control_mode_int = 0;
    ros1_utils::getParamWithLog(nh_private_, "control_mode", control_mode_int, "Control mode");

    // 转换为枚举类型（防御性检查：确保值在有效范围内）
    if (control_mode_int < 0 || control_mode_int > 2) {
        ROS_WARN("[DroneRosNode] Invalid control_mode=%d, using default PX4_CASCADE_PID",
                 control_mode_int);
        control_mode_int = 0;
    }
    config.control_mode = static_cast<ControlMode>(control_mode_int);

    std::string tracking_backend = "legacy_mpc_lifter";
    nh_private_.param("tracking_backend", tracking_backend, tracking_backend);
    if (tracking_backend == "legacy_mpc_lifter" || tracking_backend == "legacy") {
        config.tracking_backend = TrackingBackend::LEGACY_MPC_LIFTER;
    } else if (tracking_backend == "nmpc_attitude_rate" || tracking_backend == "nmpc") {
        config.tracking_backend = TrackingBackend::NMPC_ATTITUDE_RATE;
    } else {
        ROS_WARN("[DroneRosNode] Unknown tracking_backend=%s, using legacy_mpc_lifter",
                 tracking_backend.c_str());
        config.tracking_backend = TrackingBackend::LEGACY_MPC_LIFTER;
        tracking_backend = "legacy_mpc_lifter";
    }
    ROS_INFO("[DroneRosNode] Tracking backend: %s", tracking_backend.c_str());

    // ========== 偏航角控制开关 ==========
    ros1_utils::getParamWithLog(nh_private_, "enable_yaw_control", config.enable_yaw_control,
                                "Enable yaw control");

    // ========== UAV NMPC 后端参数 ==========
    nh_private_.param("nmpc/control_period", config.nmpc.control_period,
                      config.nmpc.control_period);
    nh_private_.param("nmpc/prediction_horizon", config.nmpc.prediction_horizon,
                      config.nmpc.prediction_horizon);
    nh_private_.param("nmpc/gravity", config.nmpc.gravity, config.nmpc.gravity);
    nh_private_.param("nmpc/hover_thrust_ratio", config.nmpc.hover_thrust_ratio,
                      config.nmpc.hover_thrust_ratio);
    nh_private_.param("nmpc/min_hover_thrust", config.nmpc.min_hover_thrust,
                      config.nmpc.min_hover_thrust);
    nh_private_.param("nmpc/max_hover_thrust", config.nmpc.max_hover_thrust,
                      config.nmpc.max_hover_thrust);
    nh_private_.param("nmpc/specific_thrust_min", config.nmpc.specific_thrust_min,
                      config.nmpc.specific_thrust_min);
    nh_private_.param("nmpc/specific_thrust_max", config.nmpc.specific_thrust_max,
                      config.nmpc.specific_thrust_max);
    nh_private_.param("nmpc/enable_timing_log", config.nmpc.enable_timing_log,
                      config.nmpc.enable_timing_log);
    nh_private_.param("nmpc/log_period", config.nmpc.log_period, config.nmpc.log_period);

    nh_private_.param("hover_thrust/enabled", config.nmpc.hover_thrust_enabled,
                      config.nmpc.hover_thrust_enabled);
    nh_private_.param("nmpc/hover_thrust_enabled", config.nmpc.hover_thrust_enabled,
                      config.nmpc.hover_thrust_enabled);
    nh_private_.param("hover_thrust/timeout", config.nmpc.hover_thrust_timeout,
                      config.nmpc.hover_thrust_timeout);
    nh_private_.param("nmpc/hover_thrust_timeout", config.nmpc.hover_thrust_timeout,
                      config.nmpc.hover_thrust_timeout);
    nh_private_.param("nmpc/solve_timeout", config.nmpc.solve_timeout, config.nmpc.solve_timeout);
    nh_private_.param("nmpc/result_timeout", config.nmpc.result_timeout,
                      config.nmpc.result_timeout);
    nh_private_.param("nmpc/reference_timeout", config.nmpc.reference_timeout,
                      config.nmpc.reference_timeout);
    nh_private_.param("nmpc/reference_start_delay", config.nmpc.reference_start_delay,
                      config.nmpc.reference_start_delay);
    nh_private_.param("nmpc/reference_duration", config.nmpc.reference_duration,
                      config.nmpc.reference_duration);
    nh_private_.param("nmpc/reference_radius", config.nmpc.reference_radius,
                      config.nmpc.reference_radius);
    nh_private_.param("nmpc/reference_line_speed", config.nmpc.reference_line_speed,
                      config.nmpc.reference_line_speed);
    nh_private_.param("nmpc/reference_height", config.nmpc.reference_height,
                      config.nmpc.reference_height);
    nh_private_.param("nmpc/reference_z_amplitude", config.nmpc.reference_z_amplitude,
                      config.nmpc.reference_z_amplitude);
    nh_private_.param("nmpc/reference_z_frequency", config.nmpc.reference_z_frequency,
                      config.nmpc.reference_z_frequency);
    nh_private_.param("nmpc/reference_entry_duration", config.nmpc.reference_entry_duration,
                      config.nmpc.reference_entry_duration);

    if (!std::isfinite(config.nmpc.control_period) || config.nmpc.control_period <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/control_period; using 0.010 s");
        config.nmpc.control_period = 0.01;
    }
    if (!std::isfinite(config.nmpc.prediction_horizon) || config.nmpc.prediction_horizon <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/prediction_horizon; using 1.000 s");
        config.nmpc.prediction_horizon = 1.0;
    }
    if (!std::isfinite(config.nmpc.gravity) || config.nmpc.gravity <= 1e-6) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/gravity; using 9.8066");
        config.nmpc.gravity = 9.8066;
    }
    config.nmpc.hover_thrust_ratio =
        xgc2_math::math_helpers::clamp(config.nmpc.hover_thrust_ratio, 0.05, 0.95);
    config.nmpc.min_hover_thrust =
        xgc2_math::math_helpers::clamp(config.nmpc.min_hover_thrust, 0.0, 1.0);
    config.nmpc.max_hover_thrust = xgc2_math::math_helpers::clamp(
        config.nmpc.max_hover_thrust, config.nmpc.min_hover_thrust, 1.0);
    if (!std::isfinite(config.nmpc.hover_thrust_timeout) ||
        config.nmpc.hover_thrust_timeout <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid hover_thrust timeout; using 0.500 s");
        config.nmpc.hover_thrust_timeout = 0.5;
    }
    if (!std::isfinite(config.nmpc.solve_timeout) || config.nmpc.solve_timeout <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/solve_timeout; using 0.030 s");
        config.nmpc.solve_timeout = 0.03;
    }
    if (!std::isfinite(config.nmpc.result_timeout) || config.nmpc.result_timeout <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/result_timeout; using 0.100 s");
        config.nmpc.result_timeout = 0.1;
    }
    if (!std::isfinite(config.nmpc.reference_timeout) || config.nmpc.reference_timeout <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_timeout; using 0.500 s");
        config.nmpc.reference_timeout = 0.5;
    }
    if (!std::isfinite(config.nmpc.reference_start_delay) ||
        config.nmpc.reference_start_delay < 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_start_delay; using 0.200 s");
        config.nmpc.reference_start_delay = 0.2;
    }
    if (!std::isfinite(config.nmpc.reference_duration) || config.nmpc.reference_duration <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_duration; using 60.000 s");
        config.nmpc.reference_duration = 60.0;
    }
    if (!std::isfinite(config.nmpc.reference_radius) || config.nmpc.reference_radius <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_radius; using 3.000 m");
        config.nmpc.reference_radius = 3.0;
    }
    if (!std::isfinite(config.nmpc.reference_line_speed) ||
        config.nmpc.reference_line_speed < 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_line_speed; using 3.000 m/s");
        config.nmpc.reference_line_speed = 3.0;
    }
    if (!std::isfinite(config.nmpc.reference_height) || config.nmpc.reference_height <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_height; using 3.000 m");
        config.nmpc.reference_height = 3.0;
    }
    if (!std::isfinite(config.nmpc.reference_z_amplitude) ||
        config.nmpc.reference_z_amplitude < 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_z_amplitude; using 1.000 m");
        config.nmpc.reference_z_amplitude = 1.0;
    }
    if (!std::isfinite(config.nmpc.reference_z_frequency) ||
        config.nmpc.reference_z_frequency <= 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_z_frequency; using 0.500 rad/s");
        config.nmpc.reference_z_frequency = 0.5;
    }
    if (!std::isfinite(config.nmpc.reference_entry_duration) ||
        config.nmpc.reference_entry_duration < 0.0) {
        ROS_WARN("[DroneRosNode] Invalid nmpc/reference_entry_duration; using 5.000 s");
        config.nmpc.reference_entry_duration = 5.0;
    }
    if (config.tracking_backend == TrackingBackend::NMPC_ATTITUDE_RATE) {
        if (!config.nmpc.hover_thrust_enabled) {
            ROS_WARN(
                "[DroneRosNode] UAV NMPC requires hover thrust estimate; "
                "forcing hover_thrust/enabled=true");
            config.nmpc.hover_thrust_enabled = true;
        }
        ROS_INFO(
            "[DroneRosNode] UAV NMPC: dt=%.3f horizon=%.3f gravity=%.4f "
            "hover=%.3f estimator=required hover_timeout=%.3f "
            "solve_timeout=%.3f reference_timeout=%.3f "
            "reference=circle_entry radius=%.2f speed=%.2f height=%.2f z_amp=%.2f",
            config.nmpc.control_period, config.nmpc.prediction_horizon, config.nmpc.gravity,
            config.nmpc.hover_thrust_ratio, config.nmpc.hover_thrust_timeout,
            config.nmpc.solve_timeout, config.nmpc.reference_timeout, config.nmpc.reference_radius,
            config.nmpc.reference_line_speed, config.nmpc.reference_height,
            config.nmpc.reference_z_amplitude);
    }

    // ========== 滑模控制器参数 ==========
    // 仅在使用滑模控制器时才加载参数（模式1和模式2）
    if (config.control_mode != ControlMode::PX4_CASCADE_PID) {
        ros1_utils::getParamWithLog(nh_private_, "sliding_k1", config.sliding_mode.k1,
                                    "Sliding k1");
        ros1_utils::getParamWithLog(nh_private_, "sliding_k2", config.sliding_mode.k2,
                                    "Sliding k2");
        ros1_utils::getParamWithLog(nh_private_, "sliding_epsilon", config.sliding_mode.epsilon,
                                    "Sliding epsilon");

        // 参数验证
        if (config.sliding_mode.k1 <= 0.0 || config.sliding_mode.k2 <= 0.0 ||
            config.sliding_mode.epsilon <= 0.0) {
            ROS_ERROR(
                "[DroneRosNode] Invalid sliding mode parameters (must be > 0), "
                "using defaults");
            config.sliding_mode.k1 = 3.0;
            config.sliding_mode.k2 = 3.0;
            config.sliding_mode.epsilon = 0.5;
        }
    }

    // ========== 安全限制参数 ==========
    // 位置围栏
    ros1_utils::getParamWithLog(nh_private_, "fence_x_min", config.safety.fence_x_min,
                                "Fence X min (m)");
    ros1_utils::getParamWithLog(nh_private_, "fence_x_max", config.safety.fence_x_max,
                                "Fence X max (m)");
    ros1_utils::getParamWithLog(nh_private_, "fence_y_min", config.safety.fence_y_min,
                                "Fence Y min (m)");
    ros1_utils::getParamWithLog(nh_private_, "fence_y_max", config.safety.fence_y_max,
                                "Fence Y max (m)");
    ros1_utils::getParamWithLog(nh_private_, "fence_z_min", config.safety.fence_z_min,
                                "Fence Z min (m)");
    ros1_utils::getParamWithLog(nh_private_, "fence_z_max", config.safety.fence_z_max,
                                "Fence Z max (m)");

    // 位置跳变检测
    ros1_utils::getParamWithLog(nh_private_, "position_jump_threshold",
                                config.safety.position_jump_threshold,
                                "Position jump threshold (m)");

    // 速度限制
    ros1_utils::getParamWithLog(nh_private_, "max_velocity_xy", config.safety.max_velocity_xy,
                                "Max velocity XY (m/s)");
    ros1_utils::getParamWithLog(nh_private_, "max_velocity_z", config.safety.max_velocity_z,
                                "Max velocity Z (m/s)");

    // 加速度饱和检测
    ros1_utils::getParamWithLog(nh_private_, "acc_saturation_xy", config.safety.acc_saturation_xy,
                                "Acc saturation XY (m/s²)");
    ros1_utils::getParamWithLog(nh_private_, "acc_saturation_z", config.safety.acc_saturation_z,
                                "Acc saturation Z (m/s²)");

    // 将配置传递给控制器
    controller_.setConfig(config);
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
