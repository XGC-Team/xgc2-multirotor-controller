#include "px4_multirotor_controller/output/nmpc_output_consumer.h"

#include <cmath>
#include <utility>

namespace px4_multirotor_controller {
namespace {

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

}  // namespace

NmpcOutputConsumer::NmpcOutputConsumer(ros::NodeHandle& nh, DroneController& controller,
                                       EventSink event_sink, uint32_t queue_size)
    : nh_(nh), controller_(controller), event_sink_(std::move(event_sink)) {
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
