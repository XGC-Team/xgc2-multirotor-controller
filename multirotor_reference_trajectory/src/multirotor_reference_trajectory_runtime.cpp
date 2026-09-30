#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "multirotor_reference_trajectory/state_machine/active_state.h"
#include "multirotor_reference_trajectory/state_machine/ready_state.h"
#include "multirotor_reference_trajectory/state_machine/self_check_state.h"

namespace multirotor_reference_trajectory {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

Eigen::Vector3d pointToVector(const reference::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d vectorToEigen(const reference::Vector3& value) {
    return Eigen::Vector3d(value.x, value.y, value.z);
}

double yawFromQuaternion(const reference::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return finiteOr(std::atan2(siny_cosp, cosy_cosp), 0.0);
}

double adjustedStartTime(double requested, double now, double min_lead_time) {
    const double minimum = now + std::max(0.0, min_lead_time);
    if (!std::isfinite(requested) || requested <= 0.0) {
        return minimum;
    }
    return std::max(requested, minimum);
}

double paramAt(const reference::AnalyticReference& msg, size_t index, double fallback) {
    return index < msg.params.size() && std::isfinite(msg.params[index]) ? msg.params[index]
                                                                         : fallback;
}

}  // namespace

ReferenceTrajectoryRuntime::ReferenceTrajectoryRuntime() {
    reset();
}

void ReferenceTrajectoryRuntime::setConfig(const ReferenceTrajectoryConfig& config) {
    config_ = config;
    if (!std::isfinite(config_.status_rate_hz) || config_.status_rate_hz <= 0.0) {
        config_.status_rate_hz = 10.0;
    }
    if (!std::isfinite(config_.active_publish_rate_hz) || config_.active_publish_rate_hz <= 0.0) {
        config_.active_publish_rate_hz = 10.0;
    }
    if (!std::isfinite(config_.validation_sample_dt) || config_.validation_sample_dt <= 0.0) {
        config_.validation_sample_dt = 0.02;
    }
    if (!std::isfinite(config_.trajectory_timeout) || config_.trajectory_timeout < 0.0) {
        config_.trajectory_timeout = 0.5;
    }
    if (!std::isfinite(config_.min_lead_time) || config_.min_lead_time < 0.0) {
        config_.min_lead_time = 0.2;
    }
    reset();
}

void ReferenceTrajectoryRuntime::reset() {
    state_ = reference::ReferenceStatus::STATE_SELF_CHECK;
    current_time_sec_ = 0.0;
    flags_ = 0U;
    pending_kind_ = PendingKind::kNone;
    active_type_ = trajectory::TrajectoryModelType::kNone;
    active_trajectory_id_ = 0U;
    active_revision_ = 0U;
    active_start_sec_ = 0.0;
    active_duration_ = 0.0;
    active_evaluator_.reset();
    active_analytic_ = reference::AnalyticReference{};
    active_sampled_ = reference::SampledReference{};
    setupMachine();
}

sm::Status ReferenceTrajectoryRuntime::postEvent(sm::Event event) {
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

void ReferenceTrajectoryRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    const auto transition_result = machine_->update({64, 64, false});
    const auto tick_result =
        transition_result.status.ok() ? machine_->update({64, 64, true}) : transition_result;
    if (!tick_result.status.ok()) {
        flags_ |= trajectory::kFlagInvalidInput;
        state_ = reference::ReferenceStatus::STATE_SELF_CHECK;
    }
}

bool ReferenceTrajectoryRuntime::acceptAnalytic(const reference::AnalyticReference& msg) {
    uint32_t flags = 0U;
    auto evaluator = buildAnalyticEvaluator(msg, flags);
    if (!evaluator) {
        flags_ |= flags;
        return false;
    }
    pending_analytic_ = msg;
    pending_analytic_.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_kind_ = PendingKind::kAnalytic;
    return true;
}

bool ReferenceTrajectoryRuntime::acceptSampled(const reference::SampledReference& msg) {
    trajectory::SampledEvaluator3 evaluator;
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, evaluator, flags)) {
        flags_ |= flags;
        return false;
    }
    pending_sampled_ = msg;
    pending_sampled_.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_kind_ = PendingKind::kSampled;
    return true;
}

bool ReferenceTrajectoryRuntime::activatePending() {
    if (pending_kind_ == PendingKind::kNone) {
        return active_evaluator_ != nullptr;
    }
    if (pending_kind_ == PendingKind::kAnalytic) {
        uint32_t flags = 0U;
        auto evaluator = buildAnalyticEvaluator(pending_analytic_, flags);
        if (!evaluator) {
            flags_ |= flags;
            pending_kind_ = PendingKind::kNone;
            return false;
        }
        setActiveAnalytic(pending_analytic_, std::move(evaluator), flags);
        pending_kind_ = PendingKind::kNone;
        return true;
    }
    if (pending_kind_ == PendingKind::kSampled) {
        auto evaluator = std::make_unique<trajectory::SampledEvaluator3>();
        uint32_t flags = 0U;
        if (!buildSampledEvaluator(pending_sampled_, *evaluator, flags)) {
            flags_ |= flags;
            pending_kind_ = PendingKind::kNone;
            return false;
        }
        setActiveSampled(pending_sampled_, std::move(evaluator), flags);
        pending_kind_ = PendingKind::kNone;
        return true;
    }
    return false;
}

bool ReferenceTrajectoryRuntime::activeExpired(double now_sec) const {
    if (!active_evaluator_) {
        return true;
    }
    if (active_duration_ <= 0.0) {
        return false;
    }
    return now_sec > active_start_sec_ + active_duration_ + config_.trajectory_timeout;
}

bool ReferenceTrajectoryRuntime::hasPendingReference() const {
    return pending_kind_ != PendingKind::kNone;
}

void ReferenceTrajectoryRuntime::enterState(uint8_t state) {
    state_ = state;
}

reference::ReferenceStatus ReferenceTrajectoryRuntime::makeStatus(double stamp_sec) const {
    reference::ReferenceStatus status;
    status.header.stamp = Time(stamp_sec);
    status.state = state_;
    status.flags = flags_;
    status.active_trajectory_id = active_trajectory_id_;
    status.active_revision = active_revision_;
    status.active_type = reference::ReferenceStatus::TYPE_NONE;
    if (active_type_ == trajectory::TrajectoryModelType::kAnalytic) {
        status.active_type = reference::ReferenceStatus::TYPE_ANALYTIC;

    } else if (active_type_ == trajectory::TrajectoryModelType::kSampled) {
        status.active_type = reference::ReferenceStatus::TYPE_SAMPLED;
    }
    return status;
}

void ReferenceTrajectoryRuntime::setupMachine() {
    auto builder = sm::StateMachine::builder("ReferenceTrajectoryStateMachine");
    builder.region(region_type::REFERENCE)
        .name("reference")
        .order(0)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Ready)
        .name("Ready")
        .impl(std::make_unique<ReadyState>(*this))
        .state(state_type::Active)
        .name("Active")
        .impl(std::make_unique<ActiveState>(*this))
        .endRegion();

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Ready)
        .on(event_type::CONFIG_READY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Active)
        .on(event_type::ANALYTIC_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Active)
        .on(event_type::SAMPLED_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Active)
        .on(event_type::ANALYTIC_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Active)
        .on(event_type::SAMPLED_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::SelfCheck)
        .on(event_type::PLAN_FAILED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Ready)
        .on(event_type::TRAJECTORY_EXPIRED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::SelfCheck)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::SelfCheck)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::REQUEST);
    auto machine_result = builder.build();
    requireOk(machine_result.status, "build reference trajectory state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start reference trajectory state machine");
}

std::unique_ptr<trajectory::TrajectoryEvaluator3>
ReferenceTrajectoryRuntime::buildAnalyticEvaluator(const reference::AnalyticReference& msg,
                                                   uint32_t& flags) const {
    flags = msg.flags;
    const bool has_duration = msg.duration > 0.0;
    const double duration = has_duration ? msg.duration : 60.0;
    const Eigen::Vector3d origin = pointToVector(msg.origin.position);
    const double origin_yaw = yawFromQuaternion(msg.origin.orientation);
    const double radius = paramAt(msg, 0U, 3.0);
    const double line_speed = paramAt(msg, 1U, 3.0);
    const double height = paramAt(msg, 2U, 3.0);
    const double z_amplitude = paramAt(msg, 3U, 1.0);
    const double z_frequency = paramAt(msg, 4U, 0.5);
    const double entry_duration = paramAt(msg, 5U, 5.0);
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    center.x() = paramAt(msg, 6U, center.x());
    center.y() = paramAt(msg, 7U, center.y());

    std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator;
    switch (msg.analytic_type) {
        case reference::AnalyticReference::ANALYTIC_HOLD: {
            trajectory::HoldCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.position = origin;
            params.position.z() = height;
            params.yaw = origin_yaw;
            evaluator = std::make_unique<trajectory::HoldCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_CIRCLE:
        case reference::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE: {
            trajectory::CircleCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            params.height = height;
            params.z_amplitude =
                msg.analytic_type == reference::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE
                    ? z_amplitude
                    : 0.0;
            params.z_frequency = z_frequency;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_FIGURE_EIGHT: {
            trajectory::FigureEightCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.origin = origin;
            params.radius = radius;
            params.line_speed = line_speed;
            params.height = height;
            evaluator = std::make_unique<trajectory::FigureEightCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_LINE: {
            trajectory::LineCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = has_duration ? duration : params.duration;
            params.start = origin;
            params.target = Eigen::Vector3d(paramAt(msg, 0U, origin.x() + 1.0),
                                            paramAt(msg, 1U, origin.y() + 1.0),
                                            paramAt(msg, 2U, origin.z() + 1.0));
            params.target_velocity = Eigen::Vector3d(paramAt(msg, 3U, 0.0), paramAt(msg, 4U, 0.0),
                                                     paramAt(msg, 5U, 0.0));
            params.start_velocity = Eigen::Vector3d(paramAt(msg, 6U, 0.0), paramAt(msg, 7U, 0.0),
                                                    paramAt(msg, 8U, 0.0));
            evaluator = std::make_unique<trajectory::LineCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_LEMNISCATE: {
            trajectory::LemniscateCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = has_duration ? duration : params.duration;
            params.origin = origin;
            params.radius = paramAt(msg, 0U, 1.0);
            params.omega = paramAt(msg, 1U, 0.9);
            params.height = paramAt(msg, 2U, 1.0);
            evaluator = std::make_unique<trajectory::LemniscateCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_HELIX_YZ: {
            trajectory::HelixYzCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = has_duration ? duration : params.duration;
            params.origin = origin;
            params.radius = paramAt(msg, 0U, 1.0);
            params.omega = paramAt(msg, 1U, 1.5);
            params.linear_scale = paramAt(msg, 2U, 10.0);
            evaluator = std::make_unique<trajectory::HelixYzCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_HELIX_XY: {
            trajectory::HelixXyCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = has_duration ? duration : params.duration;
            params.origin = origin;
            params.radius = paramAt(msg, 0U, 1.0);
            params.omega = paramAt(msg, 1U, 0.9);
            params.linear_scale = paramAt(msg, 2U, 10.0);
            evaluator = std::make_unique<trajectory::HelixXyCurveEvaluator3>(params);
            break;
        }
        case reference::AnalyticReference::ANALYTIC_TORUS_KNOT: {
            trajectory::TorusKnotCurveParameters3 torus_params;
            torus_params.flags = msg.flags;
            torus_params.duration = has_duration ? duration : torus_params.duration;
            torus_params.origin = origin;
            torus_params.omega = paramAt(msg, 0U, 0.3);
            torus_params.scale = paramAt(msg, 1U, 0.3);
            torus_params.yaw = origin_yaw;
            const double torus_entry_duration = paramAt(msg, 2U, 0.0);
            if (msg.params.size() >= 6U) {
                torus_params.origin =
                    Eigen::Vector3d(paramAt(msg, 3U, origin.x()), paramAt(msg, 4U, origin.y()),
                                    paramAt(msg, 5U, origin.z()));
            }
            if (torus_entry_duration > 0.0) {
                trajectory::TorusKnotEntryCurveParameters3 params;
                params.flags = msg.flags;
                params.duration = has_duration ? duration : params.duration;
                params.start = origin;
                params.origin_yaw = origin_yaw;
                params.entry_duration = torus_entry_duration;
                params.torus = torus_params;
                params.torus.duration =
                    std::max(0.0, params.duration - std::max(0.0, torus_entry_duration));
                evaluator = std::make_unique<trajectory::TorusKnotEntryCurveEvaluator3>(params);
            } else {
                evaluator = std::make_unique<trajectory::TorusKnotCurveEvaluator3>(torus_params);
            }
            break;
        }
        case reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
        default: {
            trajectory::CircleEntryCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.origin = origin;
            params.origin_yaw = origin_yaw;
            params.entry_duration = entry_duration;
            params.circle.flags = msg.flags;
            params.circle.duration = std::max(0.0, duration - std::max(0.0, entry_duration));
            params.circle.center = center;
            params.circle.radius = radius;
            params.circle.line_speed = line_speed;
            params.circle.height = height;
            params.circle.z_amplitude = z_amplitude;
            params.circle.z_frequency = z_frequency;
            evaluator = std::make_unique<trajectory::CircleEntryCurveEvaluator3>(params);
            break;
        }
    }

    if (!evaluator) {
        flags |= trajectory::kFlagInvalidInput;
        return nullptr;
    }
    flags |= trajectory::TrajectoryValidator3::validate(*evaluator, config_.limits,
                                                        config_.validation_sample_dt);
    if ((flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) != 0U) {
        return nullptr;
    }
    return evaluator;
}

bool ReferenceTrajectoryRuntime::buildSampledEvaluator(const reference::SampledReference& msg,
                                                       trajectory::SampledEvaluator3& evaluator,
                                                       uint32_t& flags) const {
    flags = msg.flags;
    std::vector<trajectory::SampledPoint3> samples;
    samples.reserve(msg.points.size());
    for (const auto& point : msg.points) {
        trajectory::SampledPoint3 sample;
        sample.t = point.t_from_start;
        sample.flat.position = pointToVector(point.position);
        sample.flat.velocity = vectorToEigen(point.velocity);
        sample.flat.acceleration = vectorToEigen(point.acceleration);
        sample.flat.jerk = vectorToEigen(point.jerk);
        sample.flat.snap = vectorToEigen(point.snap);
        sample.flat.yaw = point.yaw;
        sample.flat.yaw_rate = point.yaw_rate;
        sample.flat.yaw_accel = point.yaw_accel;
        samples.push_back(sample);
    }
    if (!evaluator.setSamples(std::move(samples))) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator3::validate(evaluator, config_.limits,
                                                        config_.validation_sample_dt);
    return (flags & (trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite)) == 0U;
}

void ReferenceTrajectoryRuntime::setActiveAnalytic(
    const reference::AnalyticReference& msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator, uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kAnalytic;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = evaluator ? evaluator->duration() : 0.0;
    active_evaluator_ = std::move(evaluator);
    active_analytic_ = msg;
    flags_ = flags;
}

void ReferenceTrajectoryRuntime::setActiveSampled(
    const reference::SampledReference& msg,
    std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator, uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kSampled;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = evaluator ? evaluator->duration() : 0.0;
    active_evaluator_ = std::move(evaluator);
    active_sampled_ = msg;
    flags_ = flags;
}

}  // namespace multirotor_reference_trajectory
