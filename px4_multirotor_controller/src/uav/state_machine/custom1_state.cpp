#include "px4_multirotor_controller/uav/state_machine/custom1_state.h"

#include <algorithm>
#include <cstdint>
#include <utility>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

namespace px4_multirotor_controller {

Custom1State::Custom1State(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult Custom1State::onEnter(::state_machine::StateContext& ctx) {
    const auto& config = controller_.getConfig();
    controller_.clearCustom1Request();
    command_time_ = controller_.getCurrentTime();
    hover_x_ = sensor_checks::worldX(controller_.getSensorData(), config.tracking_backend);
    hover_y_ = sensor_checks::worldY(controller_.getSensorData(), config.tracking_backend);
    hover_z_ = sensor_checks::worldZ(controller_.getSensorData(), config.tracking_backend);
    tracking_armed_ = false;
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
    sync_strategy_entered_ = false;
    nmpc_stale_output_log_timer_.reset();

    if (config.tracking_backend == TrackingBackend::NMPC) {
        controller_.logInfo("[Custom1State] Custom1 latched (nmpc); holding hover until reference");
        controller_.activeTrajectoryCache().clear();
        ::state_machine::Event activation_event(
            output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION,
            ::state_machine::EventTimestamp{command_time_});
        activation_event.source = "custom1_state";
        const auto status = ctx.emitOutput(std::move(activation_event));
        if (!status.ok()) {
            controller_.logWarn("[Custom1State] Failed to emit reference activation event: %s",
                                status.message.c_str());
        }
        return {};
    }
    if (config.tracking_backend == TrackingBackend::DFBC) {
        controller_.logInfo("[Custom1State] Custom1 latched (dfbc); holding hover until reference");
        controller_.activeTrajectoryCache().clear();
        dfbc_strategy_.configure(config);
        ::state_machine::Event activation_event(
            output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION,
            ::state_machine::EventTimestamp{command_time_});
        activation_event.source = "custom1_state";
        const auto status = ctx.emitOutput(std::move(activation_event));
        if (!status.ok()) {
            controller_.logWarn("[Custom1State] Failed to emit reference activation event: %s",
                                status.message.c_str());
        }
        return {};
    }

    controller_.logInfo(
        "[Custom1State] Custom1 latched (px4_local); holding hover until world PV/PVA");
    px4_local_raw_strategy_.configure(config);
    px4_local_raw_strategy_.enter(controller_.getSensorData(), ros::Time(command_time_));
    return {};
}

::state_machine::ActionResult Custom1State::onTick(::state_machine::StateContext& ctx) {
    const double current_time = controller_.getCurrentTime();
    const auto backend = controller_.getConfig().tracking_backend;
    if (backend == TrackingBackend::NMPC) {
        handleNmpcEventMode(ctx, current_time);
        return {};
    }
    if (backend == TrackingBackend::DFBC) {
        handleSynchronousAttitudeRateMode(ctx, current_time);
        return {};
    }
    handlePx4LocalPassThrough(ctx, current_time);
    return {};
}

void Custom1State::handlePx4LocalPassThrough(::state_machine::StateContext& ctx,
                                             double current_time) {
    auto& buffer = controller_.mpcTrajectoryBuffer();
    if (buffer.hasPending()) {
        buffer.promotePending(buffer.pending().planning_time);
    }
    const auto& sample = buffer.active();
    const auto& config = controller_.getConfig();
    if (!passThroughMayTakeSetpoint(sample, command_time_, current_time,
                                    config.nmpc.reference_timeout, hover_x_, hover_y_, hover_z_,
                                    config.nmpc.plan_hover_xy_tol, config.nmpc.plan_hover_z_tol,
                                    tracking_armed_)) {
        if (tracking_armed_) {
            postReferenceExit(ctx, current_time, event_type::INPUT_REFERENCE_TRAJECTORY_LOST,
                              "px4_local reference stopped or became stale");
            return;
        }
        if (shouldRunEvery(trajectory_wait_log_timer_, 1.0, true)) {
            controller_.logWarn(
                "[Custom1State] Waiting for world-frame PV/PVA before taking setpoint");
        }
        if (shouldPublish(current_time)) {
            publishCurrentHoverSetpoint(ctx, current_time);
        }
        return;
    }

    if (!tracking_armed_) {
        tracking_armed_ = true;
        controller_.logInfo("[Custom1State] px4_local takeover after aligned reference");
    }
    if (!shouldPublish(current_time)) {
        return;
    }

    controller_.getSetpoint() = liftWorldLocal(sample, ros::Time(current_time),
                                               config.local_type_mask, config.enable_yaw_control);
    ctx.emitOutput(::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                          ::state_machine::EventTimestamp{current_time}));
    last_publish_time_ = current_time;
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
    tracking_armed_ = true;

    if (referenceWillFinishBeforeNextHorizon(current_time)) {
        postReferenceExit(ctx, current_time, event_type::REFERENCE_TRAJECTORY_FINISHED,
                          "reference horizon reached planned trajectory end");
        return;
    }

    const double result_timeout = controller_.getConfig().nmpc.result_timeout;
    if (last_success_time_ > 0.0 && result_timeout > 0.0 &&
        current_time - last_success_time_ > result_timeout) {
        if (shouldRunEvery(nmpc_stale_output_log_timer_, 1.0, true)) {
            controller_.logWarn(
                "[Custom1State] NMPC attitude-rate output stale for %.3f s; "
                "publishing backup local setpoint",
                current_time - last_success_time_);
        }
        if (shouldPublish(current_time)) {
            publishBackupSetpoint(ctx, current_time);
        }
    }

    if (shouldDispatchNmpc(current_time)) {
        dispatchNmpcRequest(ctx, current_time);
    }
}

void Custom1State::handleSynchronousAttitudeRateMode(::state_machine::StateContext& ctx,
                                                     double current_time) {
    const ControllerConfig config = controller_.getConfig();
    const ros::Time now(current_time);

    dfbc_strategy_.configure(config);
    if (!sync_strategy_entered_) {
        sync_strategy_entered_ = dfbc_strategy_.enter(controller_.getSensorData(), now);
    }

    if (!controller_.activeTrajectoryCache().valid(now, config.nmpc.reference_timeout)) {
        if (nmpc_reference_seen_) {
            postReferenceExit(ctx, current_time, event_type::INPUT_REFERENCE_TRAJECTORY_LOST,
                              "reference stream stopped or became stale");
        } else {
            if (shouldRunEvery(nmpc_wait_log_timer_, 1.0, true)) {
                controller_.logWarn(
                    "[Custom1State] Waiting for UAV reference trajectory before synchronous "
                    "tracking");
            }
            if (shouldPublish(current_time)) {
                publishCurrentHoverSetpoint(ctx, current_time);
            }
        }
        return;
    }
    nmpc_reference_seen_ = true;
    tracking_armed_ = true;

    if (referenceWillFinishBeforeNextSynchronousUpdate(current_time)) {
        postReferenceExit(ctx, current_time, event_type::REFERENCE_TRAJECTORY_FINISHED,
                          "reference horizon reached planned trajectory end");
        return;
    }

    if (!shouldRunSynchronousStrategy(current_time)) {
        return;
    }

    UavReferencePoint reference;
    if (!controller_.activeTrajectoryCache().sample(now, config.nmpc.reference_timeout,
                                                    reference)) {
        postReferenceExit(ctx, current_time, event_type::INPUT_REFERENCE_TRAJECTORY_LOST,
                          "reference sampling failed");
        return;
    }

    if (!sync_strategy_entered_) {
        publishCurrentHoverSetpoint(ctx, current_time);
        return;
    }

    TrackingStrategyInput input;
    input.sensor = controller_.getSensorData();
    input.reference = reference;
    input.now = now;
    TrackingStrategyResult result;
    if (!dfbc_strategy_.update(input, result)) {
        ++consecutive_failures_;
        if (shouldRunEvery(trajectory_wait_log_timer_, 1.0, true)) {
            controller_.logWarn("[Custom1State] synchronous tracking update failed: %s",
                                result.message.c_str());
        }
        publishBackupSetpoint(ctx, current_time);
        return;
    }

    consecutive_failures_ = 0;
    last_success_time_ = current_time;
    controller_.getAttitudeRateTarget() = result.attitude_rate_target;
    ctx.emitOutput(::state_machine::Event(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET,
                                          ::state_machine::EventTimestamp{current_time}));
    last_publish_time_ = current_time;
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

    const double result_age = current_time - result.stamp.toSec();
    const double result_timeout = controller_.getConfig().nmpc.result_timeout;
    if (!request_in_flight_ && result.sequence == in_flight_sequence_ &&
        current_time > request_deadline_ && result_timeout > 0.0 && result_age > result_timeout) {
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
            last_publish_time_ = current_time;
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
    last_publish_time_ = current_time;
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
        backup.type_mask = kHoverPositionVelocityTypeMask;
    } else {
        publishCurrentHoverSetpoint(ctx, current_time);
        return;
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
    const auto backend = controller_.getConfig().tracking_backend;
    Setpoint backup;
    backup.x = sensor_checks::worldX(sensor, backend);
    backup.y = sensor_checks::worldY(sensor, backend);
    backup.z = sensor_checks::worldZ(sensor, backend);
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
    tracking_armed_ = false;
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

bool Custom1State::referenceWillFinishBeforeNextSynchronousUpdate(double current_time) const {
    double remaining = 0.0;
    if (!controller_.activeTrajectoryCache().finiteTimeRemaining(
            ros::Time(current_time), controller_.getConfig().nmpc.reference_timeout, remaining)) {
        return false;
    }

    const double period = std::max(1.0e-3, synchronousStrategyPeriod());
    return remaining <= period + 1.0e-6;
}

bool Custom1State::shouldDispatchNmpc(double current_time) const {
    if (request_in_flight_) {
        return false;
    }
    const double period = controller_.getConfig().nmpc.control_period;
    return last_request_time_ <= 0.0 || current_time - last_request_time_ >= period;
}

bool Custom1State::shouldRunSynchronousStrategy(double current_time) const {
    const double period = std::max(1.0e-3, synchronousStrategyPeriod());
    return last_publish_time_ <= 0.0 || current_time - last_publish_time_ >= period;
}

double Custom1State::synchronousStrategyPeriod() const {
    return dfbc_strategy_.period();
}

bool Custom1State::shouldPublish(double current_time) const {
    return current_time - last_publish_time_ >= publish_period_;
}

::state_machine::ActionResult Custom1State::onExit(::state_machine::StateContext&) {
    nmpc_wait_log_timer_.stop();
    trajectory_wait_log_timer_.stop();
    tracking_armed_ = false;
    const auto backend = controller_.getConfig().tracking_backend;
    if (backend == TrackingBackend::NMPC) {
        controller_.logInfo("[Custom1State] Exiting UAV NMPC Attitude-Rate Tracking Mode");
    } else if (backend == TrackingBackend::DFBC) {
        controller_.logInfo("[Custom1State] Exiting UAV DFBC Attitude-Rate Tracking Mode");
        dfbc_strategy_.exit();
        sync_strategy_entered_ = false;
    } else {
        controller_.logInfo("[Custom1State] Exiting PX4 local pass-through");
        px4_local_raw_strategy_.exit();
        sync_strategy_entered_ = false;
    }
    request_in_flight_ = false;
    return {};
}

}  // namespace px4_multirotor_controller
