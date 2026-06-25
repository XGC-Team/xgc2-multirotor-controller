#include "px4_multirotor_controller/uav/active_trajectory_cache.h"

#include <algorithm>
#include <cmath>

#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {
namespace control = xgc2_math::control;
namespace trajectory = xgc2_math::trajectory;
namespace {

Eigen::Vector3d toVector(const geometry_msgs::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d toVector(const geometry_msgs::Vector3& vector) {
    return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::isfinite(siny_cosp) && std::isfinite(cosy_cosp) ? std::atan2(siny_cosp, cosy_cosp)
                                                                : 0.0;
}

bool fatalReferenceFlags(uint32_t flags) {
    constexpr uint32_t kFatal = trajectory::kFlagInvalidInput | trajectory::kFlagNonFinite |
                                trajectory::kFlagLowThrust | trajectory::kFlagYawSingularity;
    return (flags & kFatal) != 0U;
}

double paramAt(const multirotor_reference_trajectory::AnalyticReference& msg, size_t index,
               double fallback) {
    return msg.params.size() > index && std::isfinite(msg.params[index]) ? msg.params[index]
                                                                         : fallback;
}

void appendCoefficients(const std::vector<double>& flat, size_t offset, size_t count,
                        std::vector<double>& out) {
    out.assign(flat.begin() + static_cast<std::ptrdiff_t>(offset),
               flat.begin() + static_cast<std::ptrdiff_t>(offset + count));
}

}  // namespace

bool ActiveTrajectoryCache::updateAnalytic(
    const multirotor_reference_trajectory::AnalyticReference& msg, const ros::Time& received_time) {
    uint32_t flags = 0U;
    auto evaluator = buildAnalyticEvaluator(msg, flags);
    if (!evaluator || fatalReferenceFlags(flags)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::shared_ptr<const trajectory::TrajectoryEvaluator3>(std::move(evaluator));
    type_ = trajectory::TrajectoryModelType::kAnalytic;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    ++sequence_;
    start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (start_time_.isZero()) {
        start_time_ = received_time;
    }
    received_time_ = received_time;
    flags_ = flags;
    return true;
}

bool ActiveTrajectoryCache::updatePolynomial(
    const multirotor_reference_trajectory::ActivePolynomialReference& msg,
    const ros::Time& received_time) {
    auto evaluator = std::make_unique<trajectory::PiecewisePolynomialEvaluator3>();
    uint32_t flags = 0U;
    if (!buildPolynomialEvaluator(msg, *evaluator, flags) || fatalReferenceFlags(flags)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::shared_ptr<const trajectory::TrajectoryEvaluator3>(std::move(evaluator));
    type_ = trajectory::TrajectoryModelType::kPolynomial;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    ++sequence_;
    start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (start_time_.isZero()) {
        start_time_ = received_time;
    }
    received_time_ = received_time;
    flags_ = flags | msg.flags;
    return true;
}

bool ActiveTrajectoryCache::updateSampled(
    const multirotor_reference_trajectory::SampledReference& msg, const ros::Time& received_time) {
    auto evaluator = std::make_unique<trajectory::SampledEvaluator3>();
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, *evaluator, flags) || fatalReferenceFlags(flags)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::shared_ptr<const trajectory::TrajectoryEvaluator3>(std::move(evaluator));
    type_ = trajectory::TrajectoryModelType::kSampled;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    ++sequence_;
    start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (start_time_.isZero()) {
        start_time_ = received_time;
    }
    received_time_ = received_time;
    flags_ = flags | msg.flags;
    return true;
}

void ActiveTrajectoryCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_.reset();
    type_ = trajectory::TrajectoryModelType::kNone;
    trajectory_id_ = 0U;
    revision_ = 0U;
    sequence_ = 0U;
    start_time_ = ros::Time();
    received_time_ = ros::Time();
    flags_ = 0U;
}

bool ActiveTrajectoryCache::sample(const ros::Time& now, double timeout,
                                   UavReferencePoint& sample) const {
    std::shared_ptr<const trajectory::TrajectoryEvaluator3> evaluator;
    ros::Time local_start;
    ros::Time local_received;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!evaluator_ || start_time_.isZero() ||
            (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
            return false;
        }
        evaluator = evaluator_;
        local_start = start_time_;
        local_received = received_time_;
    }
    (void)local_received;
    if (!evaluator) {
        return false;
    }
    trajectory::FlatOutput3 flat;
    const double t = std::max(0.0, (now - local_start).toSec());
    if (!evaluator->evaluate(t, flat)) {
        return false;
    }
    sample = toPoint(flat, t);
    return true;
}

bool ActiveTrajectoryCache::sampleHorizon(const ros::Time& now, double stage_dt, int horizon_steps,
                                          double timeout, double gravity,
                                          std::vector<control::Se3Reference>& references) const {
    if (horizon_steps <= 0 || stage_dt <= 0.0) {
        return false;
    }

    std::shared_ptr<const trajectory::TrajectoryEvaluator3> evaluator;
    ros::Time local_start;
    ros::Time local_received;
    uint32_t local_flags = 0U;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!evaluator_ || start_time_.isZero() ||
            (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
            return false;
        }
        evaluator = evaluator_;
        local_start = start_time_;
        local_received = received_time_;
        local_flags = flags_;
    }
    (void)local_received;
    if (!evaluator || fatalReferenceFlags(local_flags)) {
        return false;
    }

    const int sample_count = horizon_steps + 2;
    const double t0 = std::max(0.0, (now - local_start).toSec());
    const double t_last = t0 + static_cast<double>(sample_count - 1) * stage_dt;
    if (evaluator->duration() > 0.0 && t_last > evaluator->duration() + 1.0e-6) {
        return false;
    }

    trajectory::FlatnessMapper3 mapper(gravity, 0.1);
    references.clear();
    references.reserve(static_cast<size_t>(sample_count));
    for (int i = 0; i < sample_count; ++i) {
        const double t = t0 + static_cast<double>(i) * stage_dt;
        trajectory::FlatOutput3 flat;
        if (!evaluator->evaluate(t, flat)) {
            return false;
        }
        const auto full = mapper.map(flat);
        if (fatalReferenceFlags(full.flags)) {
            return false;
        }
        references.push_back(toNmpcReference(full));
    }
    return true;
}

uint64_t ActiveTrajectoryCache::sequence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sequence_;
}

uint32_t ActiveTrajectoryCache::trajectoryId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return trajectory_id_;
}

uint32_t ActiveTrajectoryCache::revision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return revision_;
}

bool ActiveTrajectoryCache::valid(const ros::Time& now, double timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return evaluator_ != nullptr && !start_time_.isZero() && !fatalReferenceFlags(flags_) &&
           (timeout <= 0.0 || (now - received_time_).toSec() <= timeout);
}

bool ActiveTrajectoryCache::finiteTimeRemaining(const ros::Time& now, double timeout,
                                                double& remaining) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!evaluator_ || start_time_.isZero() || fatalReferenceFlags(flags_) ||
        (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
        return false;
    }

    const double duration = evaluator_->duration();
    if (!std::isfinite(duration) || duration <= 0.0) {
        return false;
    }

    const double elapsed = std::max(0.0, (now - start_time_).toSec());
    remaining = duration - elapsed;
    return std::isfinite(remaining);
}

bool ActiveTrajectoryCache::finiteVector(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

std::unique_ptr<trajectory::TrajectoryEvaluator3> ActiveTrajectoryCache::buildAnalyticEvaluator(
    const multirotor_reference_trajectory::AnalyticReference& msg, uint32_t& flags) {
    flags = msg.flags;
    const double duration = msg.duration > 0.0 ? msg.duration : 60.0;
    const Eigen::Vector3d origin = toVector(msg.origin.position);
    const double origin_yaw = yawFromQuaternion(msg.origin.orientation);
    const double radius = paramAt(msg, 0U, 3.0);
    const double line_speed = paramAt(msg, 1U, 3.0);
    const double height = paramAt(msg, 2U, 3.0);
    const double z_amplitude = paramAt(msg, 3U, 1.0);
    const double z_frequency = paramAt(msg, 4U, 0.5);
    const double entry_duration = paramAt(msg, 5U, 5.0);
    Eigen::Vector2d center(origin.x(), origin.y());
    center.x() = paramAt(msg, 6U, center.x());
    center.y() = paramAt(msg, 7U, center.y());

    std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator;
    switch (msg.analytic_type) {
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HOLD: {
            trajectory::HoldCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.position = origin;
            params.position.z() = height;
            params.yaw = origin_yaw;
            evaluator = std::make_unique<trajectory::HoldCurveEvaluator3>(params);
            break;
        }
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE: {
            trajectory::CircleCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            params.height = height;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator3>(params);
            break;
        }
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE: {
            trajectory::CircleCurveParameters3 params;
            params.flags = msg.flags;
            params.duration = duration;
            params.center = center;
            params.radius = radius;
            params.line_speed = line_speed;
            params.height = height;
            params.z_amplitude = z_amplitude;
            params.z_frequency = z_frequency;
            evaluator = std::make_unique<trajectory::CircleCurveEvaluator3>(params);
            break;
        }
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_FIGURE_EIGHT: {
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
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
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
    flags |= trajectory::TrajectoryValidator3::validate(*evaluator, trajectory::TrajectoryLimits3{},
                                                        0.02);
    return fatalReferenceFlags(flags) ? nullptr : std::move(evaluator);
}

bool ActiveTrajectoryCache::buildPolynomialEvaluator(
    const multirotor_reference_trajectory::ActivePolynomialReference& msg,
    trajectory::PiecewisePolynomialEvaluator3& evaluator, uint32_t& flags) {
    flags = msg.flags;
    const size_t coeffs_per_segment = static_cast<size_t>(msg.order) + 1U;
    const size_t segment_count = msg.segment_durations.size();
    if (msg.order < 1U || segment_count == 0U ||
        msg.coeff_x.size() != segment_count * coeffs_per_segment ||
        msg.coeff_y.size() != msg.coeff_x.size() || msg.coeff_z.size() != msg.coeff_x.size()) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    const bool has_yaw = msg.coeff_yaw.size() == segment_count * coeffs_per_segment;
    std::vector<trajectory::PolynomialSegment3> segments;
    segments.reserve(segment_count);
    for (size_t i = 0; i < segment_count; ++i) {
        trajectory::PolynomialSegment3 segment;
        segment.duration = msg.segment_durations[i];
        const size_t offset = i * coeffs_per_segment;
        appendCoefficients(msg.coeff_x, offset, coeffs_per_segment, segment.x);
        appendCoefficients(msg.coeff_y, offset, coeffs_per_segment, segment.y);
        appendCoefficients(msg.coeff_z, offset, coeffs_per_segment, segment.z);
        if (has_yaw) {
            appendCoefficients(msg.coeff_yaw, offset, coeffs_per_segment, segment.yaw);
        }
        segments.push_back(std::move(segment));
    }
    if (!evaluator.setSegments(std::move(segments), msg.order)) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator3::validate(evaluator, trajectory::TrajectoryLimits3{},
                                                        0.02);
    return !fatalReferenceFlags(flags);
}

bool ActiveTrajectoryCache::buildSampledEvaluator(
    const multirotor_reference_trajectory::SampledReference& msg,
    trajectory::SampledEvaluator3& evaluator, uint32_t& flags) {
    flags = msg.flags;
    std::vector<trajectory::SampledPoint3> samples;
    samples.reserve(msg.points.size());
    for (const auto& point : msg.points) {
        trajectory::SampledPoint3 sample;
        sample.t = point.t_from_start;
        sample.flat.position = toVector(point.position);
        sample.flat.velocity = toVector(point.velocity);
        sample.flat.acceleration = toVector(point.acceleration);
        sample.flat.jerk = toVector(point.jerk);
        sample.flat.snap = toVector(point.snap);
        sample.flat.yaw = point.yaw;
        sample.flat.yaw_rate = point.yaw_rate;
        sample.flat.yaw_accel = point.yaw_accel;
        samples.push_back(sample);
    }
    if (!evaluator.setSamples(std::move(samples))) {
        flags |= trajectory::kFlagInvalidInput;
        return false;
    }
    flags |= trajectory::TrajectoryValidator3::validate(evaluator, trajectory::TrajectoryLimits3{},
                                                        0.02);
    return !fatalReferenceFlags(flags);
}

UavReferencePoint ActiveTrajectoryCache::toPoint(const trajectory::FlatOutput3& flat, double t) {
    UavReferencePoint point;
    point.t_from_start = t;
    point.position = flat.position;
    point.velocity = flat.velocity;
    point.acceleration = flat.acceleration;
    point.jerk = flat.jerk;
    point.snap = flat.snap;
    point.yaw = flat.yaw;
    point.yaw_rate = flat.yaw_rate;
    point.yaw_accel = flat.yaw_accel;
    point.flags = flat.flags;
    return point;
}

control::Se3Reference ActiveTrajectoryCache::toNmpcReference(
    const trajectory::FullStateReference3& full) {
    control::Se3Reference reference;
    reference.state.position = full.position;
    reference.state.velocity = full.velocity;
    reference.state.attitude = full.attitude;
    reference.state.body_rate = full.body_rate;
    reference.control.body_z_specific_force = full.specific_thrust;
    reference.control.angular_acceleration = full.angular_acceleration;
    reference.flags = full.flags;
    return reference;
}

}  // namespace px4_multirotor_controller
