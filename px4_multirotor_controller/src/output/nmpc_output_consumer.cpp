#include "px4_multirotor_controller/output/nmpc_output_consumer.h"

#include <px4_multirotor_controller_msgs/NmpcDebugSample.h>

#include <cmath>
#include <utility>

namespace px4_multirotor_controller {
namespace {

geometry_msgs::Vector3 vector3FromEigen(const Eigen::Vector3d& vector) {
    geometry_msgs::Vector3 value;
    value.x = vector.x();
    value.y = vector.y();
    value.z = vector.z();
    return value;
}

geometry_msgs::Pose poseFromState(const Se3StateVector& state) {
    geometry_msgs::Pose pose;
    pose.position.x = state(0);
    pose.position.y = state(1);
    pose.position.z = state(2);
    const double qw = state(6);
    const double qx = state(7);
    const double qy = state(8);
    const double qz = state(9);
    const double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (std::isfinite(q_norm) && q_norm > 1.0e-9) {
        pose.orientation.w = qw / q_norm;
        pose.orientation.x = qx / q_norm;
        pose.orientation.y = qy / q_norm;
        pose.orientation.z = qz / q_norm;
    } else {
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
    }
    return pose;
}

geometry_msgs::Twist twistFromState(const Se3StateVector& state) {
    geometry_msgs::Twist twist;
    twist.linear.x = state(3);
    twist.linear.y = state(4);
    twist.linear.z = state(5);
    twist.angular.x = state(10);
    twist.angular.y = state(11);
    twist.angular.z = state(12);
    return twist;
}

}  // namespace

NmpcOutputConsumer::NmpcOutputConsumer(ros::NodeHandle& nh, DroneController& controller,
                                       EventSink event_sink, uint32_t queue_size)
    : nh_(nh), controller_(controller), event_sink_(std::move(event_sink)) {
    debug_pub_ = nh_.advertise<px4_multirotor_controller_msgs::NmpcDebugSample>(
        "alg/nmpc/debug_sample", queue_size);
    predicted_path_pub_ = nh_.advertise<nav_msgs::Path>("alg/nmpc/predicted_path", queue_size);
    predicted_poses_pub_ =
        nh_.advertise<geometry_msgs::PoseArray>("alg/nmpc/predicted_poses", queue_size);
    backend_.configure(controller_.getConfig());
    worker_ = std::thread(&NmpcOutputConsumer::workerLoop, this);
}

NmpcOutputConsumer::~NmpcOutputConsumer() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    backend_.exit();
}

bool NmpcOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::REQUEST_NMPC_SOLVE) {
        return false;
    }

    const uint64_t sequence = event.correlation_id;
    const ControllerConfig config = controller_.getConfig();
    const ros::Time now(event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec());
    Request request;
    request.sequence = sequence;
    request.now = now;
    request.sensor = controller_.getSensorData();

    const double stage_dt =
        config.nmpc.prediction_horizon / static_cast<double>(UavNmpcSolver::horizonSteps());
    if (!controller_.activeTrajectoryCache().sampleHorizon(
            now, stage_dt, UavNmpcSolver::horizonSteps(), config.nmpc.reference_timeout,
            config.nmpc.gravity, request.references)) {
        reject(sequence, nmpc_solver_status::kReferenceSamplingFailed);
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || has_pending_) {
            reject(sequence, nmpc_solver_status::kDispatcherBusy);
            return true;
        }
        pending_ = std::move(request);
        has_pending_ = true;
    }
    condition_.notify_one();
    return true;
}

void NmpcOutputConsumer::workerLoop() {
    bool entered = false;
    while (true) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stop_ || has_pending_; });
            if (stop_) {
                return;
            }
            request = std::move(pending_);
            has_pending_ = false;
            busy_ = true;
        }

        NmpcSolveResult result;
        result.sequence = request.sequence;
        result.stamp = request.now;
        backend_.configure(controller_.getConfig());
        if (request.sequence == 1) {
            backend_.exit();
            entered = false;
        }
        if (!entered) {
            entered = backend_.enter(request.sensor);
        }
        if (entered) {
            result.success =
                backend_.compute(request.sensor, request.references, request.now, result.target);
            result.solver_status = backend_.status();
            result.solve_time_ms = backend_.solveTimeMs();
            publishDebug(request.sequence, request.now);
            if (result.success) {
                publishPrediction(request.now);
            }
        } else {
            result.success = false;
            result.solver_status = nmpc_solver_status::kBackendUnavailable;
        }

        controller_.nmpcResultBuffer().store(result);
        postResultEvent(request.sequence, result.success);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
        }
    }
}

void NmpcOutputConsumer::reject(uint64_t sequence, int solver_status) {
    NmpcSolveResult result;
    result.sequence = sequence;
    result.success = false;
    result.solver_status = solver_status;
    result.stamp = ros::Time::now();
    controller_.nmpcResultBuffer().store(result);
    postResultEvent(sequence, false);
}

void NmpcOutputConsumer::postResultEvent(uint64_t sequence, bool success) {
    if (!event_sink_) {
        return;
    }
    ::state_machine::Event event(
        success ? event_type::INPUT_NMPC_SOLVE_SUCCEEDED : event_type::INPUT_NMPC_SOLVE_FAILED,
        ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = "nmpc_output_consumer";
    event.correlation_id = sequence;
    (void)event_sink_(std::move(event));
}

void NmpcOutputConsumer::publishDebug(uint64_t sequence, const ros::Time& stamp) {
    const NmpcDebugData& debug = backend_.lastDebugData();
    if (!debug.valid) {
        return;
    }

    px4_multirotor_controller_msgs::NmpcDebugSample msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.sequence = sequence;
    msg.success = debug.success;
    msg.solver_status = debug.solver_status;
    msg.solve_time_ms = debug.solve_time_ms;
    msg.state_estimate_stamp_sec = debug.state_estimate_stamp_sec;
    msg.filter_inertial_stamp_sec = debug.filter_inertial_stamp_sec;
    msg.filter_pose_stamp_sec = debug.filter_pose_stamp_sec;
    msg.last_vrpn_pose_stamp_sec = debug.last_vrpn_pose_stamp_sec;
    msg.state_pose = poseFromState(debug.state);
    msg.state_twist = twistFromState(debug.state);
    msg.reference_pose = poseFromState(debug.reference);
    msg.reference_twist = twistFromState(debug.reference);
    msg.horizon_pose = poseFromState(debug.horizon_reference);
    msg.reference_acceleration = vector3FromEigen(debug.reference_acceleration);
    msg.position_error = vector3FromEigen(debug.position_error);
    msg.velocity_error = vector3FromEigen(debug.velocity_error);
    msg.omega_error = vector3FromEigen(debug.omega_error);
    msg.body_rate_command = vector3FromEigen(debug.body_rate_command);
    msg.predicted_body_rate = vector3FromEigen(debug.predicted_body_rate);
    msg.alpha_command = vector3FromEigen(debug.angular_acceleration_command);
    msg.reference_alpha = vector3FromEigen(debug.reference_control.segment<3>(1));
    msg.specific_thrust_command = debug.optimal_control(0);
    msg.normalized_thrust_raw = debug.normalized_thrust_raw;
    msg.normalized_thrust_command = debug.normalized_thrust_command;
    msg.reference_specific_thrust = debug.reference_control(0);
    msg.hover_thrust = debug.hover_thrust;
    msg.initial_hover_thrust = debug.initial_hover_thrust;
    msg.thrust_actual_estimate = debug.thrust_actual_estimate;
    msg.last_commanded_specific_thrust = debug.last_commanded_specific_thrust;
    msg.effective_specific_thrust_min = debug.effective_specific_thrust_min;
    msg.effective_specific_thrust_max = debug.effective_specific_thrust_max;
    msg.normalized_thrust_min_saturated = debug.normalized_thrust_min_saturated;
    msg.normalized_thrust_max_saturated = debug.normalized_thrust_max_saturated;
    msg.roll_rate_saturated = debug.roll_rate_saturated;
    msg.pitch_rate_saturated = debug.pitch_rate_saturated;
    msg.yaw_rate_saturated = debug.yaw_rate_saturated;
    msg.roll_alpha_saturated = debug.roll_alpha_saturated;
    msg.pitch_alpha_saturated = debug.pitch_alpha_saturated;
    msg.yaw_alpha_saturated = debug.yaw_alpha_saturated;
    debug_pub_.publish(msg);
}

void NmpcOutputConsumer::publishPrediction(const ros::Time& stamp) {
    nav_msgs::Path path;
    geometry_msgs::PoseArray poses;
    path.header.stamp = stamp;
    path.header.frame_id = "world";
    poses.header = path.header;

    const auto& predicted_states = backend_.predictedStates();
    const size_t count = backend_.predictedStateCount();
    path.poses.reserve(count);
    poses.poses.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const geometry_msgs::Pose pose = poseFromState(predicted_states[i]);
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = path.header;
        stamped_pose.pose = pose;
        path.poses.push_back(stamped_pose);
        poses.poses.push_back(pose);
    }

    predicted_path_pub_.publish(path);
    predicted_poses_pub_.publish(poses);
}

}  // namespace px4_multirotor_controller
