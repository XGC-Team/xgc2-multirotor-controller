#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <state_machine/adt_machine.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirotor_reference_trajectory {
namespace {

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

// The reference owner has one value state. Periodic publication phase belongs
// to the state that entered it, while the last reference belongs to the
// runtime because Ready status continues to report it after expiry.
class ReferenceMachine {
    struct Gate {
        double last{0.0};
        bool initialized{false};

        bool due(double now, double rate) {
            const double period = 1.0 / rate;
            if (!initialized || now - last >= period) {
                last = now;
                initialized = true;
                return true;
            }
            return false;
        }

        double next(double now, double rate) const {
            if (!initialized)
                return now;
            return std::nextafter(last + 1.0 / rate, std::numeric_limits<double>::infinity());
        }
    };
    struct SelfCheck {};
    struct Ready { Gate status; };
    struct Active { Gate status; Gate active; };
    struct Turn {};
    using Transition = ::state_machine::adt::Transition<SelfCheck, Ready, Active>;

    static Transition ready(ReferenceTrajectoryRuntime& runtime) {
        Ready state;
        state.status.due(runtime.currentTime(), runtime.config().status_rate_hz);
        runtime.emit(ReferenceEffectKind::Status);
        return ::state_machine::adt::Enter{std::move(state)};
    }

    static Transition active(ReferenceTrajectoryRuntime& runtime) {
        runtime.activatePending();
        Active state;
        state.status.due(runtime.currentTime(), runtime.config().status_rate_hz);
        state.active.due(runtime.currentTime(), runtime.config().active_publish_rate_hz);
        runtime.emit(ReferenceEffectKind::Status);
        runtime.emitActive();
        return ::state_machine::adt::Enter{std::move(state)};
    }

    struct Spec {
        using State = std::variant<SelfCheck, Ready, Active>;
        using Time = double;

        static Transition handle(SelfCheck&, const Turn&, ReferenceTrajectoryRuntime& runtime) {
            // A request accepted during SelfCheck is consumed in the same
            // owner turn instead of being lost on the initial transition.
            return runtime.hasPendingReference() ? ReferenceMachine::active(runtime)
                                                 : ReferenceMachine::ready(runtime);
        }
        static Transition handle(Ready& state, const Turn&, ReferenceTrajectoryRuntime& runtime) {
            if (runtime.hasPendingReference())
                return ReferenceMachine::active(runtime);
            if (state.status.due(runtime.currentTime(), runtime.config().status_rate_hz))
                runtime.emit(ReferenceEffectKind::Status);
            return ::state_machine::adt::Stay{};
        }
        static Transition handle(Active& state, const Turn&, ReferenceTrajectoryRuntime& runtime) {
            if (runtime.hasPendingReference())
                return ReferenceMachine::active(runtime);
            if (runtime.activeExpired(runtime.currentTime()))
                return ReferenceMachine::ready(runtime);
            if (state.status.due(runtime.currentTime(), runtime.config().status_rate_hz))
                runtime.emit(ReferenceEffectKind::Status);
            if (state.active.due(runtime.currentTime(), runtime.config().active_publish_rate_hz))
                runtime.emitActive();
            return ::state_machine::adt::Stay{};
        }
    };

   public:
    ReferenceMachine() : machine_(SelfCheck{}, 0.0) {}

    void update(ReferenceTrajectoryRuntime& runtime) {
        auto transition = machine_.dispatch(Turn{}, runtime);
        machine_.apply(std::move(transition), runtime.currentTime());
    }

    uint8_t state() const {
        const auto& value = machine_.state();
        if (std::holds_alternative<SelfCheck>(value))
            return reference::ReferenceStatus::STATE_SELF_CHECK;
        if (std::holds_alternative<Ready>(value))
            return reference::ReferenceStatus::STATE_READY;
        return reference::ReferenceStatus::STATE_ACTIVE;
    }

    std::optional<double> nextWake(const ReferenceTrajectoryRuntime& runtime) const {
        const auto& value = machine_.state();
        if (std::holds_alternative<SelfCheck>(value) || runtime.hasPendingReference())
            return runtime.currentTime();
        if (const auto* ready = std::get_if<Ready>(&value))
            return ready->status.next(runtime.currentTime(), runtime.config().status_rate_hz);
        const auto& active = std::get<Active>(value);
        double next = std::min(active.status.next(runtime.currentTime(), runtime.config().status_rate_hz),
                               active.active.next(runtime.currentTime(), runtime.config().active_publish_rate_hz));
        if (const auto expiry = runtime.activeExpiry())
            next = std::min(next, *expiry);
        return next;
    }

   private:
    ::state_machine::adt::AdtMachine<Spec> machine_;
};

ReferenceTrajectoryRuntime::ReferenceTrajectoryRuntime() {
    effects_.reserve(2);
    reset();
}

ReferenceTrajectoryRuntime::~ReferenceTrajectoryRuntime() = default;

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
    current_time_sec_ = 0.0;
    flags_ = 0U;
    pending_ = std::monostate{};
    active_ = std::monostate{};
    effects_.clear();
    machine_ = std::make_unique<ReferenceMachine>();
}

void ReferenceTrajectoryRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    effects_.clear();
    machine_->update(*this);
}

std::optional<double> ReferenceTrajectoryRuntime::nextWake() const {
    return machine_->nextWake(*this);
}

uint8_t ReferenceTrajectoryRuntime::currentState() const {
    return machine_->state();
}

void ReferenceTrajectoryRuntime::emit(ReferenceEffectKind kind) {
    effects_.push_back({kind, current_time_sec_});
}

void ReferenceTrajectoryRuntime::emitActive() {
    if (activeType() == trajectory::TrajectoryModelType::kAnalytic)
        emit(ReferenceEffectKind::ActiveAnalytic);
    else if (activeType() == trajectory::TrajectoryModelType::kSampled)
        emit(ReferenceEffectKind::ActiveSampled);
}

bool ReferenceTrajectoryRuntime::acceptAnalytic(const reference::AnalyticReference& msg,
                                                double receipt_sec) {
    uint32_t flags = 0U;
    auto evaluator = buildAnalyticEvaluator(msg, flags);
    if (!evaluator) {
        flags_ |= flags;
        return false;
    }
    auto adjusted = msg;
    adjusted.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), receipt_sec, config_.min_lead_time));
    pending_ = AnalyticReference{std::move(adjusted), std::move(evaluator), flags};
    return true;
}

bool ReferenceTrajectoryRuntime::acceptSampled(const reference::SampledReference& msg,
                                               double receipt_sec) {
    auto evaluator = std::make_unique<trajectory::SampledEvaluator3>();
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, *evaluator, flags)) {
        flags_ |= flags;
        return false;
    }
    auto adjusted = msg;
    adjusted.start_time =
        Time(adjustedStartTime(msg.start_time.toSec(), receipt_sec, config_.min_lead_time));
    pending_ = SampledReference{std::move(adjusted), std::move(evaluator), flags};
    return true;
}

bool ReferenceTrajectoryRuntime::activatePending() {
    if (std::holds_alternative<std::monostate>(pending_)) {
        return evaluator() != nullptr;
    }
    if (const auto* analytic = std::get_if<AnalyticReference>(&pending_))
        flags_ = analytic->flags;
    else
        flags_ = std::get<SampledReference>(pending_).flags;
    active_ = std::move(pending_);
    pending_ = std::monostate{};
    return true;
}

bool ReferenceTrajectoryRuntime::activeExpired(double now_sec) const {
    const auto* active_evaluator = evaluator();
    if (!active_evaluator) {
        return true;
    }
    const double duration = active_evaluator->duration();
    if (duration <= 0.0) {
        return false;
    }
    const auto* analytic = std::get_if<AnalyticReference>(&active_);
    const double start = analytic ? analytic->message.start_time.toSec()
                                  : std::get<SampledReference>(active_).message.start_time.toSec();
    return now_sec > start + duration + config_.trajectory_timeout;
}

bool ReferenceTrajectoryRuntime::hasPendingReference() const {
    return !std::holds_alternative<std::monostate>(pending_);
}

std::optional<double> ReferenceTrajectoryRuntime::activeExpiry() const {
    const auto* active_evaluator = evaluator();
    if (!active_evaluator)
        return current_time_sec_;
    const double duration = active_evaluator->duration();
    if (duration <= 0.0)
        return std::nullopt;
    const auto* analytic = std::get_if<AnalyticReference>(&active_);
    const double start = analytic ? analytic->message.start_time.toSec()
                                  : std::get<SampledReference>(active_).message.start_time.toSec();
    return std::nextafter(start + duration + config_.trajectory_timeout,
                          std::numeric_limits<double>::infinity());
}

trajectory::TrajectoryModelType ReferenceTrajectoryRuntime::activeType() const {
    if (std::holds_alternative<AnalyticReference>(active_)) {
        return trajectory::TrajectoryModelType::kAnalytic;
    }
    if (std::holds_alternative<SampledReference>(active_)) {
        return trajectory::TrajectoryModelType::kSampled;
    }
    return trajectory::TrajectoryModelType::kNone;
}

uint32_t ReferenceTrajectoryRuntime::activeTrajectoryId() const {
    if (const auto* analytic = std::get_if<AnalyticReference>(&active_)) {
        return analytic->message.trajectory_id;
    }
    if (const auto* sampled = std::get_if<SampledReference>(&active_)) {
        return sampled->message.trajectory_id;
    }
    return 0U;
}

uint32_t ReferenceTrajectoryRuntime::activeRevision() const {
    if (const auto* analytic = std::get_if<AnalyticReference>(&active_)) {
        return analytic->message.revision;
    }
    if (const auto* sampled = std::get_if<SampledReference>(&active_)) {
        return sampled->message.revision;
    }
    return 0U;
}

const reference::AnalyticReference& ReferenceTrajectoryRuntime::activeAnalyticMessage() const {
    return std::get<AnalyticReference>(active_).message;
}

const reference::SampledReference& ReferenceTrajectoryRuntime::activeSampledMessage() const {
    return std::get<SampledReference>(active_).message;
}

const trajectory::TrajectoryEvaluator3* ReferenceTrajectoryRuntime::evaluator() const {
    if (const auto* analytic = std::get_if<AnalyticReference>(&active_)) {
        return analytic->evaluator.get();
    }
    if (const auto* sampled = std::get_if<SampledReference>(&active_)) {
        return sampled->evaluator.get();
    }
    return nullptr;
}

reference::ReferenceStatus ReferenceTrajectoryRuntime::makeStatus(double stamp_sec) const {
    reference::ReferenceStatus status;
    status.header.stamp = Time(stamp_sec);
    status.state = currentState();
    status.flags = flags_;
    status.active_trajectory_id = activeTrajectoryId();
    status.active_revision = activeRevision();
    status.active_type = reference::ReferenceStatus::TYPE_NONE;
    if (activeType() == trajectory::TrajectoryModelType::kAnalytic) {
        status.active_type = reference::ReferenceStatus::TYPE_ANALYTIC;

    } else if (activeType() == trajectory::TrajectoryModelType::kSampled) {
        status.active_type = reference::ReferenceStatus::TYPE_SAMPLED;
    }
    return status;
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

}  // namespace multirotor_reference_trajectory
