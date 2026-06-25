#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "multirotor_reference_trajectory/state_machine/active_state.h"
#include "multirotor_reference_trajectory/state_machine/fault_state.h"
#include "multirotor_reference_trajectory/state_machine/planning_state.h"
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

Eigen::Vector3d pointToVector(const geometry_msgs::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d vectorToEigen(const geometry_msgs::Vector3& value) {
    return Eigen::Vector3d(value.x, value.y, value.z);
}

Eigen::Quaterniond quaternionToEigen(const geometry_msgs::Quaternion& value) {
    return Eigen::Quaterniond(value.w, value.x, value.y, value.z);
}

geometry_msgs::Point toPoint(const Eigen::Vector3d& value) {
    geometry_msgs::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
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

trajectory::WaypointConstraintType3 constraintType(uint8_t value) {
    switch (value) {
        case WaypointReferenceRequest::CONSTRAINT_SPHERE:
            return trajectory::WaypointConstraintType3::kSphere;
        case WaypointReferenceRequest::CONSTRAINT_BOX:
            return trajectory::WaypointConstraintType3::kBox;
        case WaypointReferenceRequest::CONSTRAINT_GATE:
            return trajectory::WaypointConstraintType3::kGate;
        case WaypointReferenceRequest::CONSTRAINT_POINT:
        default:
            return trajectory::WaypointConstraintType3::kPoint;
    }
}

void appendCoefficients(const std::vector<double>& input, std::vector<double>& output) {
    output.insert(output.end(), input.begin(), input.end());
}

double paramAt(const AnalyticReference& msg, size_t index, double fallback) {
    return index < msg.params.size() && std::isfinite(msg.params[index]) ? msg.params[index]
                                                                         : fallback;
}

bool buildWaypointProblemFromMessage(const WaypointReferenceRequest& msg,
                                     const ReferenceTrajectoryConfig& config,
                                     trajectory::WaypointProblem3& problem, uint32_t& flags) {
    flags = msg.flags;
    problem.flags = msg.flags;
    problem.segment_times = msg.segment_times;
    problem.start_velocity = vectorToEigen(msg.start_velocity);
    problem.start_acceleration = vectorToEigen(msg.start_acceleration);
    problem.end_velocity = vectorToEigen(msg.end_velocity);
    problem.end_acceleration = vectorToEigen(msg.end_acceleration);
    problem.desired_speed = msg.desired_speed > 0.0 ? msg.desired_speed : 1.0;
    problem.time_weight = msg.time_weight > 0.0 ? msg.time_weight : 1.0;
    problem.max_iterations = msg.max_iterations > 0U ? static_cast<int>(msg.max_iterations) : 80;
    problem.rel_cost_tol = msg.rel_cost_tol > 0.0 ? msg.rel_cost_tol : 1.0e-5;
    problem.dynamic_penalty_weight = 1000.0;
    problem.limits.max_velocity = msg.max_velocity;
    problem.limits.max_acceleration = msg.max_acceleration;
    problem.limits.max_jerk = msg.max_jerk;
    problem.limits.max_snap = msg.max_snap;
    problem.limits.max_body_rate = msg.max_body_rate;
    problem.limits.max_tilt = msg.max_tilt;
    problem.limits.min_specific_thrust =
        msg.min_thrust > 0.0 ? msg.min_thrust : config.limits.min_specific_thrust;
    problem.limits.max_specific_thrust = msg.max_thrust;
    problem.validation_sample_dt = config.validation_sample_dt;
    if ((!msg.constraint_types.empty() && msg.constraint_types.size() != msg.waypoints.size()) ||
        (!msg.region_size.empty() && msg.region_size.size() != msg.waypoints.size())) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    problem.constraints.reserve(msg.waypoints.size());
    for (size_t i = 0; i < msg.waypoints.size(); ++i) {
        trajectory::WaypointConstraint3 constraint;
        constraint.position = pointToVector(msg.waypoints[i].position);
        constraint.orientation = quaternionToEigen(msg.waypoints[i].orientation);
        constraint.type = msg.constraint_types.empty() ? trajectory::WaypointConstraintType3::kPoint
                                                       : constraintType(msg.constraint_types[i]);
        if (!msg.region_size.empty()) {
            constraint.size = vectorToEigen(msg.region_size[i]);
        }
        problem.constraints.push_back(std::move(constraint));
    }
    if (problem.constraints.size() < 2U ||
        (!problem.segment_times.empty() &&
         problem.segment_times.size() + 1U != problem.constraints.size())) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    return true;
}

}  // namespace

ReferenceTrajectoryRuntime::ReferenceTrajectoryRuntime() {
    planning_worker_ = std::thread(&ReferenceTrajectoryRuntime::planningWorkerLoop, this);
    reset();
}

ReferenceTrajectoryRuntime::~ReferenceTrajectoryRuntime() {
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        planning_stop_ = true;
    }
    planning_condition_.notify_all();
    if (planning_worker_.joinable()) {
        planning_worker_.join();
    }
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
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        ++planning_generation_;
        expected_planning_sequence_ = 0U;
        has_completed_plan_ = false;
        completed_plan_ = PlanningResult{};
        clearPlanningQueuesLocked();
    }
    state_ = ReferenceStatus::STATE_SELF_CHECK;
    current_time_sec_ = 0.0;
    flags_ = 0U;
    pending_kind_ = PendingKind::kNone;
    active_type_ = trajectory::TrajectoryModelType::kNone;
    active_trajectory_id_ = 0U;
    active_revision_ = 0U;
    active_start_sec_ = 0.0;
    active_duration_ = 0.0;
    active_evaluator_.reset();
    active_analytic_ = AnalyticReference{};
    active_sampled_ = SampledReference{};
    active_polynomial_ = ActivePolynomialReference{};
    setupMachine();
}

sm::Status ReferenceTrajectoryRuntime::postEvent(sm::Event event) {
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

void ReferenceTrajectoryRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    drainPlanningResults(now_sec);
    const auto transition_result = machine_->update({64, 64, false});
    const auto tick_result =
        transition_result.status.ok() ? machine_->update({64, 64, true}) : transition_result;
    if (!tick_result.status.ok()) {
        flags_ |= trajectory::kFlagInvalidInput;
        state_ = ReferenceStatus::STATE_FAULT;
    }
}

bool ReferenceTrajectoryRuntime::acceptAnalytic(const AnalyticReference& msg) {
    uint32_t flags = 0U;
    auto evaluator = buildAnalyticEvaluator(msg, flags);
    if (!evaluator) {
        flags_ |= flags;
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        ++planning_generation_;
        expected_planning_sequence_ = 0U;
        has_completed_plan_ = false;
        completed_plan_ = PlanningResult{};
        clearPlanningQueuesLocked();
    }
    pending_analytic_ = msg;
    pending_analytic_.start_time = ros::Time(
        adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_kind_ = PendingKind::kAnalytic;
    return true;
}

bool ReferenceTrajectoryRuntime::acceptSampled(const SampledReference& msg) {
    trajectory::SampledEvaluator3 evaluator;
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, evaluator, flags)) {
        flags_ |= flags;
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        ++planning_generation_;
        expected_planning_sequence_ = 0U;
        has_completed_plan_ = false;
        completed_plan_ = PlanningResult{};
        clearPlanningQueuesLocked();
    }
    pending_sampled_ = msg;
    pending_sampled_.start_time = ros::Time(
        adjustedStartTime(msg.start_time.toSec(), current_time_sec_, config_.min_lead_time));
    pending_kind_ = PendingKind::kSampled;
    return true;
}

bool ReferenceTrajectoryRuntime::acceptWaypoint(const WaypointReferenceRequest& msg) {
    trajectory::WaypointProblem3 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblem(msg, problem, flags)) {
        flags_ |= flags;
        return false;
    }
    pending_waypoint_ = msg;
    pending_kind_ = PendingKind::kWaypoint;
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
    if (pending_kind_ == PendingKind::kWaypoint) {
        if (!has_completed_plan_ || !completed_plan_.success || !completed_plan_.evaluator) {
            flags_ |= trajectory::kFlagOptimizationFailure;
            return false;
        }
        PlanningResult result = std::move(completed_plan_);
        has_completed_plan_ = false;
        std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator = std::move(result.evaluator);
        setActivePolynomial(std::move(result.msg), std::move(evaluator), result.flags);
        pending_kind_ = PendingKind::kNone;
        return true;
    }
    return false;
}

bool ReferenceTrajectoryRuntime::requestPendingWaypointPlan() {
    if (pending_kind_ != PendingKind::kWaypoint) {
        flags_ |= trajectory::kFlagInvalidInput;
        return false;
    }
    PlanningRequest request;
    request.now_sec = current_time_sec_;
    request.active_revision = active_revision_;
    request.config = config_;
    request.msg = pending_waypoint_;
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        request.sequence = ++planning_sequence_;
        request.generation = planning_generation_;
        expected_planning_sequence_ = request.sequence;
        has_completed_plan_ = false;
        completed_plan_ = PlanningResult{};
        clearPlanningQueuesLocked();
        planning_requests_.push(std::move(request));
    }
    planning_condition_.notify_one();
    return true;
}

void ReferenceTrajectoryRuntime::planningWorkerLoop() {
    while (true) {
        PlanningRequest request;
        {
            std::unique_lock<std::mutex> lock(planning_mutex_);
            planning_condition_.wait(
                lock, [this] { return planning_stop_ || !planning_requests_.empty(); });
            if (planning_stop_) {
                return;
            }
            request = std::move(planning_requests_.front());
            planning_requests_.pop();
        }

        PlanningResult result = solveWaypointPlan(request);
        {
            std::lock_guard<std::mutex> lock(planning_mutex_);
            planning_results_.push(std::move(result));
        }
    }
}

ReferenceTrajectoryRuntime::PlanningResult ReferenceTrajectoryRuntime::solveWaypointPlan(
    const PlanningRequest& request) const {
    PlanningResult result;
    result.sequence = request.sequence;
    result.generation = request.generation;
    result.flags = request.msg.flags;

    trajectory::WaypointProblem3 problem;
    uint32_t flags = 0U;
    if (!buildWaypointProblemFromMessage(request.msg, request.config, problem, flags)) {
        result.flags |= flags;
        result.success = false;
        return result;
    }

    auto evaluator = std::make_unique<trajectory::PiecewisePolynomialEvaluator3>();
    trajectory::MincoWaypointSolver3 solver;
    if (!solver.solve(problem, *evaluator, &flags)) {
        result.flags |= flags | trajectory::kFlagOptimizationFailure;
        result.success = false;
        return result;
    }

    ActivePolynomialReference msg;
    msg.header = request.msg.header;
    msg.header.stamp = ros::Time(request.now_sec);
    msg.trajectory_id = request.msg.trajectory_id;
    msg.revision = request.msg.revision;
    if (msg.revision == 0U) {
        msg.revision = request.active_revision + 1U;
    }
    msg.flags = flags | request.msg.flags;
    msg.start_time = ros::Time(adjustedStartTime(request.msg.header.stamp.toSec(), request.now_sec,
                                                 request.config.min_lead_time));
    msg.duration = evaluator->duration();
    msg.order = evaluator->order();
    for (const auto& segment : evaluator->segments()) {
        msg.segment_durations.push_back(segment.duration);
        appendCoefficients(segment.x, msg.coeff_x);
        appendCoefficients(segment.y, msg.coeff_y);
        appendCoefficients(segment.z, msg.coeff_z);
        appendCoefficients(segment.yaw, msg.coeff_yaw);
    }

    result.success = true;
    result.flags = msg.flags;
    result.msg = std::move(msg);
    result.evaluator = std::move(evaluator);
    return result;
}

void ReferenceTrajectoryRuntime::drainPlanningResults(double now_sec) {
    while (true) {
        PlanningResult result;
        {
            std::lock_guard<std::mutex> lock(planning_mutex_);
            if (planning_results_.empty()) {
                return;
            }
            result = std::move(planning_results_.front());
            planning_results_.pop();
        }

        const bool stale = result.generation != planning_generation_ ||
                           result.sequence != expected_planning_sequence_;
        if (stale) {
            continue;
        }

        const bool success = result.success;
        if (success) {
            flags_ = result.flags;
            completed_plan_ = std::move(result);
            has_completed_plan_ = true;
        } else {
            flags_ |= result.flags;
            has_completed_plan_ = false;
        }

        sm::Event event(success ? event_type::PLAN_SUCCEEDED : event_type::PLAN_FAILED,
                        sm::EventTimestamp{now_sec});
        event.category = sm::EventCategory::kInput;
        event.source = "planning_worker";
        event.correlation_id = expected_planning_sequence_;
        const auto status = postEvent(std::move(event));
        if (!status.ok()) {
            flags_ |= trajectory::kFlagInvalidInput;
        }
    }
}

void ReferenceTrajectoryRuntime::clearPlanningQueuesLocked() {
    std::queue<PlanningRequest> empty_requests;
    std::queue<PlanningResult> empty_results;
    planning_requests_.swap(empty_requests);
    planning_results_.swap(empty_results);
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

void ReferenceTrajectoryRuntime::enterState(uint8_t state) {
    state_ = state;
}

ReferenceStatus ReferenceTrajectoryRuntime::makeStatus(double stamp_sec) const {
    ReferenceStatus status;
    status.header.stamp = ros::Time(stamp_sec);
    status.state = state_;
    status.flags = flags_;
    status.active_trajectory_id = active_trajectory_id_;
    status.active_revision = active_revision_;
    status.active_type = ReferenceStatus::TYPE_NONE;
    if (active_type_ == trajectory::TrajectoryModelType::kAnalytic) {
        status.active_type = ReferenceStatus::TYPE_ANALYTIC;
    } else if (active_type_ == trajectory::TrajectoryModelType::kPolynomial) {
        status.active_type = ReferenceStatus::TYPE_POLYNOMIAL;
    } else if (active_type_ == trajectory::TrajectoryModelType::kSampled) {
        status.active_type = ReferenceStatus::TYPE_SAMPLED;
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
        .state(state_type::Planning)
        .name("Planning")
        .impl(std::make_unique<PlanningState>(*this))
        .state(state_type::Active)
        .name("Active")
        .impl(std::make_unique<ActiveState>(*this))
        .state(state_type::Fault)
        .name("Fault")
        .impl(std::make_unique<FaultState>(*this))
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
        .from(state_type::Ready)
        .to(state_type::Planning)
        .on(event_type::WAYPOINT_RECEIVED)
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
        .to(state_type::Planning)
        .on(event_type::WAYPOINT_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::Active)
        .on(event_type::ANALYTIC_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::Active)
        .on(event_type::SAMPLED_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::Planning)
        .on(event_type::WAYPOINT_RECEIVED)
        .priority(transition_priority::REQUEST);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::Active)
        .on(event_type::PLAN_SUCCEEDED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Planning)
        .to(state_type::Fault)
        .on(event_type::PLAN_FAILED)
        .priority(transition_priority::FAULT);
    builder.transition()
        .from(state_type::Active)
        .to(state_type::Fault)
        .on(event_type::PLAN_FAILED)
        .priority(transition_priority::FAULT);
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
    builder.transition()
        .from(state_type::Fault)
        .to(state_type::SelfCheck)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::REQUEST);

    auto machine_result = builder.build();
    requireOk(machine_result.status, "build reference trajectory state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start reference trajectory state machine");
}

std::unique_ptr<trajectory::TrajectoryEvaluator3>
ReferenceTrajectoryRuntime::buildAnalyticEvaluator(const AnalyticReference& msg,
                                                   uint32_t& flags) const {
    flags = msg.flags;
    const double duration = msg.duration > 0.0 ? msg.duration : 60.0;
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
        case AnalyticReference::ANALYTIC_HOLD: {
            trajectory::HoldCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.position = origin;
            params.position.z() = height;
            params.yaw = origin_yaw;
            evaluator = std::make_unique<trajectory::HoldCurveEvaluator3>(params);
            break;
        }
        case AnalyticReference::ANALYTIC_CIRCLE:
        case AnalyticReference::ANALYTIC_HEIGHT_CIRCLE: {
            trajectory::CircleCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            params.height = height;
            params.z_amplitude =
                msg.analytic_type == AnalyticReference::ANALYTIC_HEIGHT_CIRCLE ? z_amplitude : 0.0;
            params.z_frequency = z_frequency;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator3>(params);
            break;
        }
        case AnalyticReference::ANALYTIC_FIGURE_EIGHT: {
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
        case AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
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

bool ReferenceTrajectoryRuntime::buildSampledEvaluator(const SampledReference& msg,
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

bool ReferenceTrajectoryRuntime::buildWaypointProblem(const WaypointReferenceRequest& msg,
                                                      trajectory::WaypointProblem3& problem,
                                                      uint32_t& flags) const {
    return buildWaypointProblemFromMessage(msg, config_, problem, flags);
}

void ReferenceTrajectoryRuntime::setActiveAnalytic(
    const AnalyticReference& msg, std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
    uint32_t flags) {
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
    const SampledReference& msg, std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
    uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kSampled;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = evaluator ? evaluator->duration() : 0.0;
    active_evaluator_ = std::move(evaluator);
    active_sampled_ = msg;
    flags_ = flags;
}

void ReferenceTrajectoryRuntime::setActivePolynomial(
    ActivePolynomialReference msg, std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
    uint32_t flags) {
    active_type_ = trajectory::TrajectoryModelType::kPolynomial;
    active_trajectory_id_ = msg.trajectory_id;
    active_revision_ = msg.revision;
    active_start_sec_ = msg.start_time.toSec();
    active_duration_ = msg.duration;
    active_evaluator_ = std::move(evaluator);
    active_polynomial_ = std::move(msg);
    flags_ = flags;
}

}  // namespace multirotor_reference_trajectory
