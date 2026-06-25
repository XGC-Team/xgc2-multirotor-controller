#include "px4_multirotor_controller/uav/state_machine/custom1_state.h"

#include <cstdint>
#include <utility>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

namespace px4_multirotor_controller {
namespace {

constexpr uint16_t kHoverPositionVelocityTypeMask = 0b110111000000;

}  // namespace

// ========== Custom1State 实现 ==========

Custom1State::Custom1State(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult Custom1State::onEnter(::state_machine::StateContext& ctx) {
    // 获取配置
    const auto& config = controller_.getConfig();
    trajectory_lifter_ = TrajectoryLifter(TrajectoryLifterConfig{config.planning_period});

    if (config.tracking_backend == TrackingBackend::NMPC_ATTITUDE_RATE) {
        controller_.logInfo("[Custom1State] Entering UAV NMPC Attitude-Rate Tracking Mode");
        controller_.activeTrajectoryCache().clear();
        request_sequence_ = 0;
        in_flight_sequence_ = 0;
        consumed_result_sequence_ = 0;
        request_in_flight_ = false;
        last_request_time_ = 0.0;
        request_deadline_ = 0.0;
        last_success_time_ = 0.0;
        last_publish_time_ = 0.0;
        consecutive_failures_ = 0;
        nmpc_reference_seen_ = false;
        reference_exit_event_posted_ = false;

        ::state_machine::Event activation_event(
            output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION,
            ::state_machine::EventTimestamp{controller_.getCurrentTime()});
        activation_event.source = "custom1_state";
        const auto status = ctx.emitOutput(std::move(activation_event));
        if (!status.ok()) {
            controller_.logWarn("[Custom1State] Failed to emit reference activation event: %s",
                                status.message.c_str());
        }
        return {};
    }

    controller_.logInfo("[Custom1State] Entering MPC Continuous Trajectory Mode");

    controller_.logInfo(
        "[Custom1State] Using TrajectoryLifter to generate 100Hz continuous "
        "references from %.3fs MPC planning",
        config.planning_period);
    controller_.logInfo(
        "[Custom1State] Dual trigger: new MPC data (immediate) OR 10ms "
        "timeout (periodic)");

    // 打印控制模式
    const char* mode_str = "UNKNOWN";
    switch (config.control_mode) {
        case ControlMode::PX4_CASCADE_PID:
            mode_str = "PX4_CASCADE_PID (pos+vel+acc_feedforward)";
            break;
        case ControlMode::PURE_SLIDING_MODE:
            mode_str = "PURE_SLIDING_MODE (only acc=u_h+u_f)";
            break;
        case ControlMode::HYBRID_CONTROL:
            mode_str = "HYBRID_CONTROL (pos+vel+acc_sliding)";
            break;
    }
    controller_.logInfo("[Custom1State] Control mode: %s", mode_str);

    // 初始化滑模控制器参数（模式1和2需要）
    if (config.control_mode != ControlMode::PX4_CASCADE_PID) {
        sliding_controller_.setParams(config.sliding_mode.k1, config.sliding_mode.k2,
                                      config.sliding_mode.epsilon);
        controller_.logInfo("[Custom1State] Sliding mode params: k1=%.2f, k2=%.2f, eps=%.3f",
                            config.sliding_mode.k1, config.sliding_mode.k2,
                            config.sliding_mode.epsilon);
    }

    // 初始化发布时间戳
    last_publish_time_ = controller_.getCurrentTime();
    return {};
}

::state_machine::ActionResult Custom1State::onTick(::state_machine::StateContext& ctx) {
    const double current_time = controller_.getCurrentTime();
    if (controller_.getConfig().tracking_backend == TrackingBackend::NMPC_ATTITUDE_RATE) {
        handleNmpcEventMode(ctx, current_time);
        return {};
    }

    // ===== MPC 轨迹跟踪控制 =====
    const bool frame_switched = updateActiveMpcFrame(current_time);

    auto& mpc_traj = controller_.mpcTrajectoryBuffer().active();
    if (!mpc_traj.is_valid) {
        return {};
    }

    if (!frame_switched && !shouldPublish(current_time)) {
        return {};
    }

    // 1. 生成参考轨迹（MPC插值结果，只读）
    multirotor_reference_trajectory_ = trajectory_lifter_.lift(mpc_traj, ros::Time(current_time));

    // 2. 根据控制模式计算控制输出
    computeControlOutput();

    // 3. 输出事件只传递发布语义，实际控制量保存在控制器上下文中
    controller_.getSetpoint() = control_output_;
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));

    last_publish_time_ = current_time;
    return {};
}

void Custom1State::handleNmpcEventMode(::state_machine::StateContext& ctx, double current_time) {
    consumeNmpcResult(ctx, current_time);

    if (request_in_flight_ && current_time > request_deadline_) {
        ++consecutive_failures_;
        request_in_flight_ = false;
        ::state_machine::Event timeout_event(event_type::INPUT_NMPC_SOLVE_TIMED_OUT,
                                             ::state_machine::EventTimestamp{current_time});
        timeout_event.source = "custom1_state";
        timeout_event.correlation_id = in_flight_sequence_;
        ctx.postInternalEvent(std::move(timeout_event));
        publishBackupSetpoint(ctx, current_time);
    }

    if (!controller_.activeTrajectoryCache().valid(
            ros::Time(current_time), controller_.getConfig().nmpc.reference_timeout)) {
        if (nmpc_reference_seen_) {
            postReferenceExit(ctx, current_time, event_type::INPUT_REFERENCE_TRAJECTORY_LOST,
                              "reference stream stopped or became stale");
        } else {
            if (shouldRunEvery(nmpc_wait_log_timer_, 1.0, true)) {
                controller_.logWarn(
                    "[Custom1State] Waiting for UAV reference trajectory before NMPC "
                    "solve");
            }
            if (shouldPublish(current_time)) {
                publishCurrentHoverSetpoint(ctx, current_time);
            }
        }
        return;
    }
    nmpc_reference_seen_ = true;

    if (referenceWillFinishBeforeNextHorizon(current_time)) {
        postReferenceExit(ctx, current_time, event_type::REFERENCE_TRAJECTORY_FINISHED,
                          "reference horizon reached planned trajectory end");
        return;
    }

    if (shouldDispatchNmpc(current_time)) {
        dispatchNmpcRequest(ctx, current_time);
    }
}

void Custom1State::consumeNmpcResult(::state_machine::StateContext& ctx, double current_time) {
    NmpcSolveResult result;
    if (!controller_.nmpcResultBuffer().consumeNewerThan(consumed_result_sequence_, result)) {
        return;
    }
    consumed_result_sequence_ = result.sequence;

    if (request_in_flight_ && result.sequence == in_flight_sequence_) {
        request_in_flight_ = false;
    }

    if (!request_in_flight_ && result.sequence == in_flight_sequence_ &&
        current_time > request_deadline_) {
        return;
    }

    if (result.sequence != in_flight_sequence_ && result.sequence < request_sequence_) {
        return;
    }

    if (!result.success) {
        if (result.solver_status == nmpc_solver_status::kReferenceSamplingFailed) {
            if (referenceWillFinishBeforeNextHorizon(current_time)) {
                postReferenceExit(ctx, current_time, event_type::REFERENCE_TRAJECTORY_FINISHED,
                                  "reference horizon reached planned trajectory end");
            } else {
                postReferenceExit(ctx, current_time, event_type::INPUT_REFERENCE_TRAJECTORY_LOST,
                                  "reference horizon sampling failed");
            }
            return;
        }
        ++consecutive_failures_;
        if (last_success_time_ > 0.0 &&
            current_time - last_success_time_ <= controller_.getConfig().nmpc.result_timeout) {
            ctx.emitOutput(::state_machine::Event(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET,
                                                  ::state_machine::EventTimestamp{current_time}));
            return;
        }
        publishBackupSetpoint(ctx, current_time);
        return;
    }

    consecutive_failures_ = 0;
    last_success_time_ = current_time;
    controller_.getAttitudeRateTarget() = result.target;
    ctx.emitOutput(::state_machine::Event(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET,
                                          ::state_machine::EventTimestamp{current_time}));
}

void Custom1State::dispatchNmpcRequest(::state_machine::StateContext& ctx, double current_time) {
    ++request_sequence_;
    in_flight_sequence_ = request_sequence_;
    request_in_flight_ = true;
    const double period = controller_.getConfig().nmpc.control_period;
    if (last_request_time_ <= 0.0) {
        last_request_time_ = current_time;
    } else {
        last_request_time_ += period;
        if (current_time - last_request_time_ > period) {
            last_request_time_ = current_time;
        }
    }
    request_deadline_ = current_time + controller_.getConfig().nmpc.solve_timeout;

    ::state_machine::Event event(output_event_type::REQUEST_NMPC_SOLVE,
                                 ::state_machine::EventTimestamp{current_time});
    event.source = "custom1_state";
    event.correlation_id = request_sequence_;
    ctx.emitOutput(std::move(event));
}

void Custom1State::publishBackupSetpoint(::state_machine::StateContext& ctx, double current_time) {
    UavReferencePoint reference;
    Setpoint backup;
    if (controller_.activeTrajectoryCache().sample(
            ros::Time(current_time), controller_.getConfig().nmpc.reference_timeout, reference)) {
        backup.x = reference.position.x();
        backup.y = reference.position.y();
        backup.z = reference.position.z();
        backup.vx = reference.velocity.x();
        backup.vy = reference.velocity.y();
        backup.vz = reference.velocity.z();
        backup.ax = 0.0;
        backup.ay = 0.0;
        backup.az = 0.0;
        backup.type_mask = kHoverPositionVelocityTypeMask;
    } else {
        const auto& sensor = controller_.getSensorData();
        backup.x = sensor.x;
        backup.y = sensor.y;
        backup.z = sensor.z;
        backup.vx = 0.0;
        backup.vy = 0.0;
        backup.vz = 0.0;
        backup.ax = 0.0;
        backup.ay = 0.0;
        backup.az = 0.0;
        backup.qx = sensor.qx;
        backup.qy = sensor.qy;
        backup.qz = sensor.qz;
        backup.qw = sensor.qw;
        backup.yaw_rate = 0.0;
        backup.type_mask = kHoverPositionVelocityTypeMask;
    }
    backup.coordinate_frame = 1;
    controller_.getSetpoint() = backup;
    ctx.emitOutput(::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                          ::state_machine::EventTimestamp{current_time}));
    last_publish_time_ = current_time;
}

void Custom1State::publishCurrentHoverSetpoint(::state_machine::StateContext& ctx,
                                               double current_time) {
    const auto& sensor = controller_.getSensorData();
    Setpoint backup;
    backup.x = sensor.x;
    backup.y = sensor.y;
    backup.z = sensor.z;
    backup.vx = 0.0;
    backup.vy = 0.0;
    backup.vz = 0.0;
    backup.ax = 0.0;
    backup.ay = 0.0;
    backup.az = 0.0;
    backup.qx = sensor.qx;
    backup.qy = sensor.qy;
    backup.qz = sensor.qz;
    backup.qw = sensor.qw;
    backup.yaw_rate = 0.0;
    backup.type_mask = kHoverPositionVelocityTypeMask;
    backup.coordinate_frame = 1;
    controller_.getSetpoint() = backup;
    ctx.emitOutput(::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                          ::state_machine::EventTimestamp{current_time}));
    last_publish_time_ = current_time;
}

void Custom1State::postReferenceExit(::state_machine::StateContext& ctx, double current_time,
                                     uint32_t event_id, const char* reason) {
    if (reference_exit_event_posted_) {
        if (shouldPublish(current_time)) {
            publishCurrentHoverSetpoint(ctx, current_time);
        }
        return;
    }

    reference_exit_event_posted_ = true;
    request_in_flight_ = false;
    publishCurrentHoverSetpoint(ctx, current_time);

    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{current_time});
    event.source = "custom1_state";
    ctx.postInternalEvent(std::move(event));
    if (event_id == event_type::REFERENCE_TRAJECTORY_FINISHED) {
        controller_.logInfo("[Custom1State] %s; switching to Hover", reason);
    } else {
        controller_.logWarn("[Custom1State] %s; switching to Hover", reason);
    }
}

bool Custom1State::referenceWillFinishBeforeNextHorizon(double current_time) const {
    double remaining = 0.0;
    if (!controller_.activeTrajectoryCache().finiteTimeRemaining(
            ros::Time(current_time), controller_.getConfig().nmpc.reference_timeout, remaining)) {
        return false;
    }

    const double stage_dt = controller_.getConfig().nmpc.prediction_horizon /
                            static_cast<double>(UavNmpcSolver::horizonSteps());
    const double horizon_sample_span =
        stage_dt * static_cast<double>(UavNmpcSolver::horizonSteps() + 1);
    return remaining <= horizon_sample_span + 1.0e-6;
}

bool Custom1State::shouldDispatchNmpc(double current_time) const {
    if (request_in_flight_) {
        return false;
    }
    const double period = controller_.getConfig().nmpc.control_period;
    return last_request_time_ <= 0.0 || current_time - last_request_time_ >= period;
}

bool Custom1State::updateActiveMpcFrame(double current_time) {
    const ros::Time current_ros_time(current_time);
    auto& active_traj = controller_.mpcTrajectoryBuffer().active();
    if (!active_traj.is_valid) {
        if (!controller_.mpcTrajectoryBuffer().hasPending()) {
            return false;
        }

        const auto& pending_traj = controller_.mpcTrajectoryBuffer().pending();
        const ros::Time activation_time =
            pending_traj.planning_time.isZero() ? current_ros_time : pending_traj.planning_time;
        if ((current_ros_time - activation_time).toSec() < 0.0) {
            return false;
        }
        return controller_.mpcTrajectoryBuffer().promotePending(activation_time);
    }

    const double planning_period = trajectory_lifter_.config().planning_period;
    const double active_age = (current_ros_time - active_traj.planning_time).toSec();
    if (active_age < planning_period) {
        return false;
    }

    if (controller_.mpcTrajectoryBuffer().hasPending()) {
        const auto& pending_traj = controller_.mpcTrajectoryBuffer().pending();
        const ros::Time activation_time =
            pending_traj.planning_time.isZero()
                ? active_traj.planning_time + ros::Duration(planning_period)
                : pending_traj.planning_time;
        if ((current_ros_time - activation_time).toSec() >= 0.0) {
            return controller_.mpcTrajectoryBuffer().promotePending(activation_time);
        }
        return false;
    }

    if (shouldRunEvery(trajectory_wait_log_timer_, 1.0, true)) {
        controller_.logWarn(
            "[TrajectoryLifter] Active MPC frame age reached %.3f s "
            "(age %.3f s), no pending MPC frame yet",
            planning_period, active_age);
    }
    return false;
}

bool Custom1State::shouldPublish(double current_time) const {
    double dt_since_last_publish = current_time - last_publish_time_;
    bool periodic_trigger = dt_since_last_publish >= publish_period_;
    return periodic_trigger;
}

void Custom1State::computeTrackingErrors(const SensorData& sensor_data, Eigen::Vector3d& e_p,
                                         Eigen::Vector3d& e_v) const {
    // 误差定义：实际 - 参考（与论文和MATLAB一致）
    // ė_p = v_actual - v_ref = e_v
    // ė_v = a_actual - a_ref = (u_h + u_f) - u_h = u_f
    e_p << sensor_data.x - multirotor_reference_trajectory_.x,
        sensor_data.y - multirotor_reference_trajectory_.y,
        sensor_data.z - multirotor_reference_trajectory_.z;

    e_v << sensor_data.vx - multirotor_reference_trajectory_.vx,
        sensor_data.vy - multirotor_reference_trajectory_.vy,
        sensor_data.vz - multirotor_reference_trajectory_.vz;
}

void Custom1State::computeControlOutput() {
    const auto& config = controller_.getConfig();
    const auto& sensor = controller_.getSensorData();

    switch (config.control_mode) {
        case ControlMode::PX4_CASCADE_PID: {
            // 模式0: 发送 pos + vel + acc_feedforward
            // 直接复制参考轨迹（acc已经是MPC前馈u_h）
            control_output_ = multirotor_reference_trajectory_;
            break;
        }

        case ControlMode::PURE_SLIDING_MODE:
        case ControlMode::HYBRID_CONTROL: {
            // 模式1和2: 都使用滑模控制 acc = u_h + u_f
            // 差异由 type_mask 控制（模式1忽略pos+vel，模式2使用pos+vel）
            Eigen::Vector3d e_p, e_v;
            computeTrackingErrors(sensor, e_p, e_v);

            // 提取MPC前馈加速度 u_h
            Eigen::Vector3d u_h(multirotor_reference_trajectory_.ax,
                                multirotor_reference_trajectory_.ay,
                                multirotor_reference_trajectory_.az);

            // 计算滑模反馈 u_f
            Eigen::Vector3d u_f = sliding_controller_.computeFeedback(e_p, e_v);

            // 复制参考轨迹（包含pos+vel+姿态等所有字段）
            control_output_ = multirotor_reference_trajectory_;

            // 覆盖加速度为滑模控制量 = u_h + u_f
            control_output_.ax = u_h.x() + u_f.x();
            control_output_.ay = u_h.y() + u_f.y();
            control_output_.az = u_h.z() + u_f.z();
            break;
        }
    }

    // 统一设置type_mask（后处理，控制PX4使用哪些字段）
    setTypeMask(config.control_mode, config.enable_yaw_control);
}

void Custom1State::setTypeMask(ControlMode mode, bool enable_yaw) {
    switch (mode) {
        case ControlMode::PX4_CASCADE_PID:
        case ControlMode::HYBRID_CONTROL:
            // 使用 pos + vel + acc，忽略 yaw + yaw_rate
            // bit 11-0: yaw_rate yaw FORCE az ay ax vz vy vx pz py px
            //           1        1    0     0  0  0  0  0  0  0  0  0
            control_output_.type_mask = 0b110000000000;
            break;
        case ControlMode::PURE_SLIDING_MODE:
            // 只使用 acc，忽略 pos + vel + yaw + yaw_rate
            // bit 11-0: yaw_rate yaw FORCE az ay ax vz vy vx pz py px
            //           1        1    0     0  0  0  1  1  1  1  1  1
            control_output_.type_mask = 0b110000111111;
            break;
    }

    if (enable_yaw) {
        control_output_.type_mask &= ~(1 << 10);  // 清除 bit 10，启用 yaw
    }
}

::state_machine::ActionResult Custom1State::onExit(::state_machine::StateContext&) {
    nmpc_wait_log_timer_.stop();
    trajectory_wait_log_timer_.stop();

    if (controller_.getConfig().tracking_backend == TrackingBackend::NMPC_ATTITUDE_RATE) {
        controller_.logInfo("[Custom1State] Exiting UAV NMPC Attitude-Rate Tracking Mode");
        request_in_flight_ = false;
        return {};
    }

    controller_.logInfo("[Custom1State] Exiting MPC Continuous Trajectory Mode");
    return {};
}

}  // namespace px4_multirotor_controller
