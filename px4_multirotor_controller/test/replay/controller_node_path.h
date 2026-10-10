#pragma once

// The ROS node's behavior without ROS: what the input producers do with a received message and what
// one iteration of DroneRosNode::run() does with time. The replay harness traces it, and the module
// tests compare the px4_multirotor_controller module with it. Messages are decoded from their ROS
// serialization, as the node receives them; receipt times are the record times.
//
// The receive statistics follow ros1_utils::TopicStatsManager, which the node used before
// SensorStatistics (a 0.1 s timer with the 2.5 s timeout), and the NMPC request is solved inline,
// as NmpcOutputConsumer's worker solves it; the result arrives at the next update, as if the worker
// had finished at once. Both are independent of the module's ControllerDriver on purpose: the module
// must reproduce them.

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/serialization.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <zlib.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/driver/command_input.h"
#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "px4_multirotor_controller/ros_reference_conversion.h"
#include "px4_multirotor_controller/ros_time_conversion.h"
#include "px4_multirotor_controller/uav/nmpc_tracking_backend.h"
#include "px4_multirotor_controller/uav/reference_activation.h"

namespace controller_replay {

namespace pmc = px4_multirotor_controller;
namespace sm = state_machine;

// Record kinds of a replay stream (bag_to_stream.py).
enum RecordKind : uint8_t {
    kEstimate = 1,
    kLocalPose = 2,
    kLocalVelocity = 3,
    kImu = 4,
    kFcuState = 5,
    kBattery = 6,
    kVrpnPose = 7,
    kCommand = 8,
    kAlgSetpoint = 9,
    kHoverThrust = 10,
    kActiveAnalytic = 11,
    kActiveSampled = 13,
};

struct Record {
    uint64_t t_ns;
    uint8_t kind;
    std::vector<uint8_t> data;
};

// A replay stream, plain or gzip-compressed (the recorded fixtures in data/ are).
inline std::vector<Record> readStream(const std::string& path) {
    gzFile in = gzopen(path.c_str(), "rb");
    if (in == nullptr)
        throw std::runtime_error("cannot open " + path);
    auto read = [&](void* buffer, unsigned length) {
        return length == 0 || gzread(in, buffer, length) == static_cast<int>(length);
    };
    char magic[8];
    if (!read(magic, 8) || std::memcmp(magic, "PMCRPLY1", 8) != 0) {
        gzclose(in);
        throw std::runtime_error("not a replay stream: " + path);
    }
    std::vector<Record> records;
    for (;;) {
        uint64_t t = 0;
        uint8_t kind = 0;
        uint32_t len = 0;
        if (!read(&t, 8))
            break;
        if (!read(&kind, 1) || !read(&len, 4)) {
            gzclose(in);
            throw std::runtime_error("truncated stream");
        }
        Record r{t, kind, std::vector<uint8_t>(len)};
        if (!read(r.data.data(), len)) {
            gzclose(in);
            throw std::runtime_error("truncated stream");
        }
        records.push_back(std::move(r));
    }
    gzclose(in);
    return records;
}

template <typename M>
M decode(const std::vector<uint8_t>& d) {
    M m;
    ros::serialization::IStream s(const_cast<uint8_t*>(d.data()), static_cast<uint32_t>(d.size()));
    ros::serialization::deserialize(s, m);
    return m;
}

template <typename M>
std::vector<uint8_t> encode(const M& m) {
    std::vector<uint8_t> bytes(ros::serialization::serializationLength(m));
    ros::serialization::OStream s(bytes.data(), static_cast<uint32_t>(bytes.size()));
    ros::serialization::serialize(s, m);
    return bytes;
}

// What an output event of the controller leads to, as the node's output consumers handle it.
struct OutputEvent {
    sm::Event event;
    pmc::Setpoint setpoint;                        // PUBLISH_SETPOINT
    pmc::AttitudeRateTarget attitude;              // PUBLISH_ATTITUDE_RATE_TARGET
    pmc::reference::AnalyticReference activation;  // PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION
    std::string status;                            // PUBLISH_CONTROLLER_STATUS
    bool solved{false};                            // REQUEST_NMPC_SOLVE, solved inline
    pmc::NmpcSolveResult solve;
};

class NodePath {
   public:
    // The tracking configuration of config/uav_nmpc.yaml: the values that differ from
    // ControllerConfig's defaults (world_boundary_json null), and the backend.
    static pmc::ControllerConfig profileConfig(pmc::TrackingBackend backend) {
        pmc::ControllerConfig config;
        config.takeoff_altitude = 2.3;
        config.local_type_mask = 3072;
        config.dfbc.acceleration_correction_enabled = true;
        config.nmpc.hover_thrust_enabled = true;
        config.safety.position_jump_threshold = 100.0;
        config.safety.max_velocity_xy = 1000.0;
        config.safety.max_velocity_z = 1000.0;
        config.safety.acc_saturation_xy = 1000.0;
        config.safety.acc_saturation_z = 1000.0;
        config.tracking_backend = backend;
        return config;
    }

    // t0: the time the statistics start at (the node's first loop iteration).
    NodePath(const pmc::ControllerConfig& config, double t0) : controller_(sensor_), next_stats_(t0 + 0.1) {
        controller_.setConfig(config);
        nmpc_backend_.configure(controller_.getConfig());
        tracks_ = {
            {kEstimate, {&sensor_.uav_state_estimate_stats, {}}},
            {kLocalPose, {&sensor_.local_pos_stats, {}}},
            {kLocalVelocity, {&sensor_.local_velocity_stats, {}}},
            {kImu, {&sensor_.imu_stats, {}}},
            {kFcuState, {&sensor_.state_stats, {}}},
            {kBattery, {&sensor_.battery_stats, {}}},
            {kVrpnPose, {&sensor_.vrpn_pose_stats, {}}},
        };
    }

    // The input producer's callback for the record, received at the record's time. Returns the
    // lines the harness traces for it.
    std::string receive(const Record& r) {
        const pmc::Time receipt = pmc::Time().fromNSec(r.t_ns);
        const double now = receipt.toSec();
        runStatsTimer(now);
        if (auto it = tracks_.find(r.kind); it != tracks_.end()) {
            Track& track = it->second;
            track.times.push_back(now);
            if (track.times.size() > 10)
                track.times.pop_front();
            track.stats->last_message_time = receipt;
            track.stats->is_active = true;
            track.stats->is_new = true;
        }
        switch (r.kind) {
            case kEstimate: {
                const auto m = decode<rigid_state_estimator_msgs::RigidStateEstimate>(r.data);
                sensor_.x = m.position.x;
                sensor_.y = m.position.y;
                sensor_.z = m.position.z;
                sensor_.vx = m.velocity.x;
                sensor_.vy = m.velocity.y;
                sensor_.vz = m.velocity.z;
                sensor_.qx = m.orientation.x;
                sensor_.qy = m.orientation.y;
                sensor_.qz = m.orientation.z;
                sensor_.qw = m.orientation.w;
                sensor_.wx = m.angular_velocity.x;
                sensor_.wy = m.angular_velocity.y;
                sensor_.wz = m.angular_velocity.z;
                sensor_.ax = m.linear_acceleration.x;
                sensor_.ay = m.linear_acceleration.y;
                sensor_.az = m.linear_acceleration.z;
                sensor_.gx = m.gravity.x;
                sensor_.gy = m.gravity.y;
                sensor_.gz = m.gravity.z;
                sensor_.accel_bias_x = m.accel_bias.x;
                sensor_.accel_bias_y = m.accel_bias.y;
                sensor_.accel_bias_z = m.accel_bias.z;
                sensor_.uav_state_estimator_state = m.estimator_state;
                sensor_.uav_state_estimator_flags = m.flags;
                sensor_.uav_state_estimate_stamp = m.header.stamp.toSec();
                sensor_.uav_state_filter_inertial_stamp = m.filter_inertial_stamp_sec;
                sensor_.uav_state_filter_pose_stamp = m.filter_pose_stamp_sec;
                sensor_.uav_state_last_vrpn_pose_stamp = m.last_vrpn_pose_stamp_sec;
                post(pmc::event_type::INPUT_UAV_STATE_ESTIMATE_UPDATED, now,
                     "alg/state_estimator/state");
                break;
            }
            case kLocalPose: {
                const auto m = decode<geometry_msgs::PoseStamped>(r.data);
                sensor_.local_x = m.pose.position.x;
                sensor_.local_y = m.pose.position.y;
                sensor_.local_z = m.pose.position.z;
                sensor_.local_qx = m.pose.orientation.x;
                sensor_.local_qy = m.pose.orientation.y;
                sensor_.local_qz = m.pose.orientation.z;
                sensor_.local_qw = m.pose.orientation.w;
                post(pmc::event_type::INPUT_LOCAL_POSITION_UPDATED, now, "mavros/local_position/pose");
                break;
            }
            case kLocalVelocity: {
                const auto m = decode<geometry_msgs::TwistStamped>(r.data);
                sensor_.local_vx = m.twist.linear.x;
                sensor_.local_vy = m.twist.linear.y;
                sensor_.local_vz = m.twist.linear.z;
                post(pmc::event_type::INPUT_LOCAL_VELOCITY_UPDATED, now,
                     "mavros/local_position/velocity_local");
                break;
            }
            case kImu:
                post(pmc::event_type::INPUT_IMU_UPDATED, now, "mavros/imu/data");
                break;
            case kFcuState: {
                const auto m = decode<mavros_msgs::State>(r.data);
                sensor_.fcu_connected = m.connected;
                sensor_.fcu_armed = m.armed;
                sensor_.fcu_guided = m.guided;
                sensor_.fcu_manual_input = m.manual_input;
                sensor_.fcu_mode = m.mode;
                sensor_.fcu_system_status = m.system_status;
                post(pmc::event_type::INPUT_FCU_STATE_UPDATED, now, "mavros/state");
                break;
            }
            case kBattery: {
                const auto m = decode<sensor_msgs::BatteryState>(r.data);
                sensor_.battery_percentage = m.percentage;
                post(pmc::event_type::INPUT_BATTERY_UPDATED, now, "mavros/battery");
                break;
            }
            case kVrpnPose: {
                const auto m = decode<geometry_msgs::PoseStamped>(r.data);
                sensor_.vrpn_x = m.pose.position.x;
                sensor_.vrpn_y = m.pose.position.y;
                sensor_.vrpn_z = m.pose.position.z;
                sensor_.vrpn_qx = m.pose.orientation.x;
                sensor_.vrpn_qy = m.pose.orientation.y;
                sensor_.vrpn_qz = m.pose.orientation.z;
                sensor_.vrpn_qw = m.pose.orientation.w;
                post(pmc::event_type::INPUT_VRPN_POSE_UPDATED, now, "pose");
                break;
            }
            case kCommand: {
                // CommandInputProducer::commandCallback
                const auto m = decode<std_msgs::String>(r.data);
                if (const auto event = pmc::commandInputEvent(m.data); event)
                    post(*event, now, "command");
                break;
            }
            case kAlgSetpoint: {  // TrajectoryInputProducer::algSetpointCallback
                const pmc::ControllerConfig cfg = controller_.getConfig();
                if (cfg.tracking_backend != pmc::TrackingBackend::PX4_LOCAL &&
                    cfg.tracking_backend != pmc::TrackingBackend::SMC)
                    break;
                const auto m = decode<mavros_msgs::PositionTarget>(r.data);
                pmc::PositionTargetIngress ingress;
                ingress.position = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
                ingress.velocity = Eigen::Vector3d(m.velocity.x, m.velocity.y, m.velocity.z);
                ingress.acceleration = Eigen::Vector3d(m.acceleration_or_force.x, m.acceleration_or_force.y,
                                                       m.acceleration_or_force.z);
                ingress.yaw = m.yaw;
                ingress.yaw_rate = m.yaw_rate;
                ingress.type_mask = m.type_mask;
                ingress.coordinate_frame = m.coordinate_frame;
                ingress.header_stamp = m.header.stamp.isZero() ? pmc::Time() : pmc::toCoreTime(m.header.stamp);
                ingress.receipt_time = receipt;
                const pmc::MpcTrajectoryState traj =
                    pmc::ingestPositionTarget(ingress, cfg.tracking_backend, cfg.px4_local_lift);
                if (pmc::usesStageEffectiveTime(cfg.tracking_backend, cfg.px4_local_lift) && !traj.is_valid)
                    break;
                pmc::cacheTrajectorySample(controller_.mpcTrajectoryBuffer(), traj, receipt, cfg);
                post(pmc::event_type::INPUT_MPC_TRAJECTORY_UPDATED, now, "alg/setpoint_raw/local");
                break;
            }
            case kHoverThrust: {  // TrajectoryInputProducer::hoverThrustCallback
                const auto m = decode<hover_thrust_estimator_msgs::HoverThrustEstimate>(r.data);
                if (!std::isfinite(m.hover_thrust) || m.hover_thrust <= 0.0 || m.hover_thrust >= 1.0) {
                    sensor_.hover_thrust_estimate_available = false;
                    sensor_.hover_thrust_estimate_flags = m.flags;
                    break;
                }
                sensor_.hover_thrust_estimate = m.hover_thrust;
                sensor_.hover_thrust_estimate_stamp = (m.header.stamp.isZero() ? receipt.toSec() : m.header.stamp.toSec());
                sensor_.hover_thrust_estimate_available = true;
                sensor_.hover_thrust_estimate_flags = m.flags;
                post(pmc::event_type::INPUT_HOVER_THRUST_UPDATED, now, "hover_thrust/estimate_state");
                break;
            }
            case kActiveAnalytic:
            case kActiveSampled: {  // TrajectoryInputProducer::active{Analytic,Sampled}Callback
                auto& cache = controller_.activeTrajectoryCache();
                bool accepted = false;
                const char* source = "";
                if (r.kind == kActiveAnalytic) {
                    accepted = cache.updateAnalytic(
                        pmc::toCoreReference(
                            decode<multirotor_reference_trajectory_msgs::AnalyticReference>(r.data)),
                        receipt);
                    source = "alg/multirotor_reference_trajectory/active/analytic";
                } else {
                    accepted = cache.updateSampled(
                        pmc::toCoreReference(
                            decode<multirotor_reference_trajectory_msgs::SampledReference>(r.data)),
                        receipt);
                    source = "alg/multirotor_reference_trajectory/active/sampled";
                }
                if (accepted)
                    post(pmc::event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED, now, source);
                break;
            }
            default:
                throw std::runtime_error("unknown record kind");
        }
        return {};
    }

    // One iteration of the control loop at time t, after ros::spinOnce(): the statistics timer, the
    // controller update, the output events as the node's consumers handle them.
    std::vector<OutputEvent> update(double t) {
        runStatsTimer(t);
        controller_.update(t);
        std::vector<OutputEvent> handled;
        for (const auto& e : controller_.getStateMachine().currentOutputEvents()) {
            OutputEvent out;
            out.event = e;
            switch (e.id) {
                case pmc::output_event_type::PUBLISH_SETPOINT:
                    out.setpoint = controller_.getSetpoint();
                    break;
                case pmc::output_event_type::PUBLISH_ATTITUDE_RATE_TARGET:
                    out.attitude = controller_.getAttitudeRateTarget();
                    break;
                case pmc::output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION:
                    out.activation = activation_.make(e.timestamp > 0.0 ? e.timestamp : t,
                                                      controller_.getSensorData(), controller_.getConfig());
                    break;
                case pmc::output_event_type::PUBLISH_CONTROLLER_STATUS:
                    out.status = controller_.getStateMachine().currentStateName(pmc::region_type::CONTROL);
                    if (out.status.empty())
                        out.status = "Unknown";
                    break;
                case pmc::output_event_type::REQUEST_NMPC_SOLVE:
                    solve(e, t, out);
                    break;
                default:
                    break;
            }
            handled.push_back(std::move(out));
        }
        for (auto& [kind, track] : tracks_)
            track.stats->is_new = false;
        return handled;
    }

    std::string state() {
        return controller_.getStateMachine().currentStateName(pmc::region_type::CONTROL);
    }
    pmc::DroneController& controller() { return controller_; }

   private:
    // ros1_utils::TopicStatsManager, reduced to what the core reads.
    struct Track {
        pmc::SensorData::TopicStats* stats{nullptr};
        std::deque<double> times;
    };

    void runStatsTimer(double until) {
        for (; next_stats_ <= until; next_stats_ += 0.1) {
            for (auto& [kind, track] : tracks_) {
                if (track.times.size() >= 2)
                    track.stats->is_active = (next_stats_ - track.times.back()) <= 2.5;
            }
        }
    }

    void post(sm::EventId id, double now, const char* source) {
        sm::Event event(id, sm::EventTimestamp{now});
        event.source = source;
        controller_.getStateMachine().postEvent(std::move(event));
    }

    // NmpcExecution::handle + workerLoop, inline: the node's NMPC dispatcher with the solve done at
    // once instead of on its worker thread.
    void solve(const sm::Event& e, double t, OutputEvent& out) {
        if (!std::isfinite(e.timestamp))
            return;
        const auto token = e.payload.find("control_generation");
        if (token == e.payload.end() || !std::holds_alternative<int64_t>(token->second))
            return;
        const uint64_t generation = static_cast<uint64_t>(std::get<int64_t>(token->second));
        if (!controller_.nmpcResultBuffer().isGenerationActive(generation))
            return;
        const uint64_t sequence = e.correlation_id;
        const pmc::ControllerConfig cfg = controller_.getConfig();
        const pmc::Time now(e.timestamp > 0.0 ? e.timestamp : t);
        const double stage_dt =
            cfg.nmpc.prediction_horizon / static_cast<double>(pmc::UavNmpcSolver::horizonSteps());
        std::vector<pmc::Se3Reference> references;
        pmc::NmpcSolveResult result;
        result.sequence = sequence;
        result.control_generation = generation;
        if (!controller_.activeTrajectoryCache().sampleHorizon(
                now, stage_dt, pmc::UavNmpcSolver::horizonSteps(), cfg.nmpc.gravity, references)) {
            result.success = false;
            result.solver_status = pmc::nmpc_solver_status::kReferenceSamplingFailed;
            result.stamp = pmc::Time(t);
        } else {
            const pmc::SensorData sensor = controller_.getSensorData();
            result.stamp = now;
            nmpc_backend_.configure(cfg);
            if (entered_generation_ != generation) {
                nmpc_backend_.exit();
                nmpc_entered_ = false;
                entered_generation_ = generation;
            }
            if (!nmpc_entered_)
                nmpc_entered_ = nmpc_backend_.enter(sensor);
            if (nmpc_entered_) {
                result.success = nmpc_backend_.compute(sensor, references, now, result.target);
                result.solver_status = nmpc_backend_.status();
            } else {
                result.success = false;
                result.solver_status = pmc::nmpc_solver_status::kBackendUnavailable;
            }
        }
        if (controller_.nmpcResultBuffer().store(result)) {
            sm::Event done(result.success ? pmc::event_type::INPUT_NMPC_SOLVE_SUCCEEDED
                                          : pmc::event_type::INPUT_NMPC_SOLVE_FAILED,
                           sm::EventTimestamp{t});
            done.source = "nmpc_output_consumer";
            done.correlation_id = sequence;
            done.payload["control_generation"] = static_cast<int64_t>(generation);
            controller_.getStateMachine().postEvent(std::move(done));
        }
        out.solved = true;
        out.solve = result;
    }

    pmc::SensorData sensor_;
    pmc::DroneController controller_;
    pmc::ReferenceActivation activation_;       // ReferenceActivationOutputConsumer
    pmc::UavNmpcTrackingBackend nmpc_backend_;  // NmpcOutputConsumer
    bool nmpc_entered_{false};
    uint64_t entered_generation_{0};
    std::map<uint8_t, Track> tracks_;
    double next_stats_;
};

}  // namespace controller_replay
