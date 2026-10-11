// A module for the chain test in the real module host (run_chain.py): the entity's MAVROS edge and
// the vehicle behind it, scripted. It feeds px4_multirotor_controller the sensor payloads of a
// vehicle in real time, carries out the controller's flight controller requests, lets the vehicle
// follow the controller's altitude setpoint, and plays the operator: `takeoff`, then `custom1` once
// the controller hovers, then `land` after a few seconds of tracking. Its health detail names the
// phase, so the test can follow the flight through the host's health report.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include "px4_multirotor_controller/common/state_estimate_status.h"
#include "px4_multirotor_controller/payloads.h"
#include "xgc2/module.h"

namespace {

enum Port : uint32_t {
    kSetpoint,
    kAttitudeRate,
    kFcuRequest,
    kStatus,
    kStateEstimate,
    kLocalPose,
    kLocalVelocity,
    kImu,
    kFcuState,
    kBattery,
    kVrpnPose,
    kCommand,
    kHoverThrust,
    kPortCount
};

template <class T>
constexpr xgc2_port_desc describe(const char* name, xgc2_port_direction direction,
                                  xgc2_port_kind kind, const char* schema, uint32_t depth) {
    return {name,
            static_cast<uint32_t>(direction),
            static_cast<uint32_t>(kind),
            schema,
            sizeof(T),
            alignof(T),
            depth,
            0};
}

const xgc2_port_desc kPorts[kPortCount] = {
    describe<xgc2_px4_position_target_v1>("setpoint", XGC2_PORT_IN, XGC2_PORT_STATE,
                                          XGC2_PX4_POSITION_TARGET_SCHEMA, 0),
    describe<xgc2_px4_attitude_rate_target_v1>("attitude_rate", XGC2_PORT_IN, XGC2_PORT_STATE,
                                               XGC2_PX4_ATTITUDE_RATE_TARGET_SCHEMA, 0),
    describe<xgc2_px4_fcu_request_v1>("fcu_request", XGC2_PORT_IN, XGC2_PORT_EVENT,
                                      XGC2_PX4_FCU_REQUEST_SCHEMA, 8),
    describe<xgc2_px4_controller_status_v1>("status", XGC2_PORT_IN, XGC2_PORT_STATE,
                                            XGC2_PX4_CONTROLLER_STATUS_SCHEMA, 0),
    describe<xgc2_px4_state_estimate_v1>("state_estimate", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                         XGC2_PX4_STATE_ESTIMATE_SCHEMA, 8),
    describe<xgc2_px4_pose_v1>("local_pose", XGC2_PORT_OUT, XGC2_PORT_EVENT, XGC2_PX4_POSE_SCHEMA,
                               8),
    describe<xgc2_px4_velocity_v1>("local_velocity", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                   XGC2_PX4_VELOCITY_SCHEMA, 8),
    describe<xgc2_px4_imu_v1>("imu", XGC2_PORT_OUT, XGC2_PORT_EVENT, XGC2_PX4_IMU_SCHEMA, 16),
    describe<xgc2_px4_fcu_state_v1>("fcu_state", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                    XGC2_PX4_FCU_STATE_SCHEMA, 8),
    describe<xgc2_px4_battery_v1>("battery", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                  XGC2_PX4_BATTERY_SCHEMA, 4),
    describe<xgc2_px4_pose_v1>("vrpn_pose", XGC2_PORT_OUT, XGC2_PORT_EVENT, XGC2_PX4_POSE_SCHEMA,
                               8),
    describe<xgc2_px4_command_v1>("command", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                  XGC2_PX4_COMMAND_SCHEMA, 8),
    describe<xgc2_px4_hover_thrust_v1>("hover_thrust", XGC2_PORT_OUT, XGC2_PORT_EVENT,
                                       XGC2_PX4_HOVER_THRUST_SCHEMA, 8),
};

constexpr int kLogInfo = 1;
constexpr int64_t kPeriodNs = 1000000;  // 1 kHz

// Emits a message at a rate in real time; after a stall it skips the missed ones.
struct Stream {
    int64_t period_ns;
    int64_t next_ns{0};
    bool due(int64_t now) {
        if (next_ns == 0)
            next_ns = now;
        if (now < next_ns)
            return false;
        next_ns += period_ns;
        if (next_ns < now - 3 * period_ns)
            next_ns = now + period_ns;
        return true;
    }
};

class Vehicle {
   public:
    Vehicle(const xgc2_host_api* host, void* ctx) : host_(host), ctx_(ctx) {}

    xgc2_status start() {
        host_->set_period_ns(ctx_, kPeriodNs);
        started_ns_ = host_->now_ns(ctx_);
        return XGC2_OK;
    }

    xgc2_status step(const xgc2_step_ctx* c) {
        const int64_t now = c->now_ns;
        const double dt = last_ns_ != 0 ? static_cast<double>(now - last_ns_) * 1e-9 : 0.0;
        last_ns_ = now;
        readOutputsOfTheController(now);
        followSetpoint(dt);
        emitSensors(now);
        playOperator(now);
        if (phase_ != reported_phase_) {
            reported_phase_ = phase_;
            host_->report(ctx_, 0, phase_.c_str());
            host_->log(ctx_, kLogInfo,
                       ("phase " + phase_ + " (controller " + state_ + ")").c_str());
        }
        return XGC2_OK;
    }

   private:
    static void stamp(int64_t ns, uint32_t& sec, uint32_t& nsec) {
        sec = static_cast<uint32_t>(ns / 1000000000LL);
        nsec = static_cast<uint32_t>(ns % 1000000000LL);
    }

    template <class T>
    void emit(uint32_t port, const T& payload, int64_t now) {
        void* slot = host_->write_begin(ctx_, port);
        if (slot == nullptr)
            return;  // a full queue is counted by the host
        std::memcpy(slot, &payload, sizeof payload);
        host_->write_commit(ctx_, port, now);
    }

    void readOutputsOfTheController(int64_t now) {
        xgc2_sample_view view{};
        while (host_->read_next(ctx_, kFcuRequest, &view) == XGC2_OK) {
            const auto* request = static_cast<const xgc2_px4_fcu_request_v1*>(view.data);
            if (request->kind == XGC2_PX4_FCU_REQUEST_MODE) {
                mode_ = std::string(request->mode, strnlen(request->mode, sizeof request->mode));
            } else if (request->kind == XGC2_PX4_FCU_REQUEST_ARM) {
                armed_ = request->arm != 0;
            }
        }
        if (host_->read_latest(ctx_, kSetpoint, &view) == XGC2_OK) {
            target_z_ = static_cast<const xgc2_px4_position_target_v1*>(view.data)->position[2];
            have_setpoint_ = true;
        }
        if (host_->read_latest(ctx_, kStatus, &view) == XGC2_OK) {
            const auto* status = static_cast<const xgc2_px4_controller_status_v1*>(view.data);
            const std::string next(status->state, strnlen(status->state, sizeof status->state));
            if (next != state_) {
                state_ = next;
                state_since_ns_ = now;
            }
        }
    }

    void followSetpoint(double dt) {
        if (armed_ && mode_ == "OFFBOARD" && have_setpoint_ && z_ < target_z_)
            z_ = std::min(target_z_, z_ + dt);  // 1 m/s
    }

    void emitSensors(int64_t now) {
        uint32_t sec, nsec;
        stamp(now, sec, nsec);
        if (pose50_.due(now)) {
            xgc2_px4_pose_v1 pose{};
            pose.stamp_sec = sec;
            pose.stamp_nsec = nsec;
            pose.position[2] = z_;
            pose.orientation_xyzw[3] = 1.0;
            emit(kLocalPose, pose, now);
            xgc2_px4_velocity_v1 velocity{};
            velocity.stamp_sec = sec;
            velocity.stamp_nsec = nsec;
            emit(kLocalVelocity, velocity, now);
            xgc2_px4_fcu_state_v1 fcu{};
            fcu.stamp_sec = sec;
            fcu.stamp_nsec = nsec;
            fcu.connected = 1;
            fcu.armed = armed_ ? 1 : 0;
            fcu.system_status = 4;
            std::strncpy(fcu.mode, mode_.c_str(), sizeof fcu.mode - 1);
            emit(kFcuState, fcu, now);
            xgc2_px4_hover_thrust_v1 hover{};
            hover.stamp_sec = sec;
            hover.stamp_nsec = nsec;
            hover.hover_thrust = 0.5;
            emit(kHoverThrust, hover, now);
        }
        if (imu200_.due(now)) {
            xgc2_px4_imu_v1 imu{};
            imu.stamp_sec = sec;
            imu.stamp_nsec = nsec;
            emit(kImu, imu, now);
        }
        if (pose100_.due(now)) {
            xgc2_px4_pose_v1 pose{};
            pose.stamp_sec = sec;
            pose.stamp_nsec = nsec;
            pose.position[2] = z_;
            pose.orientation_xyzw[3] = 1.0;
            emit(kVrpnPose, pose, now);
            xgc2_px4_state_estimate_v1 estimate{};
            estimate.stamp_sec = sec;
            estimate.stamp_nsec = nsec;
            estimate.position[2] = z_;
            estimate.orientation_xyzw[3] = 1.0;
            estimate.gravity[2] = -9.8066;
            const double t = static_cast<double>(now) * 1e-9;
            estimate.filter_inertial_stamp_sec = t;
            estimate.filter_pose_stamp_sec = t;
            estimate.last_vrpn_pose_stamp_sec = t;
            estimate.estimator_state = px4_multirotor_controller::state_estimate::STATE_RUNNING;
            emit(kStateEstimate, estimate, now);
        }
        if (battery1_.due(now)) {
            xgc2_px4_battery_v1 battery{};
            battery.stamp_sec = sec;
            battery.stamp_nsec = nsec;
            battery.percentage = 0.9;
            emit(kBattery, battery, now);
        }
    }

    void command(const char* text, int64_t now) {
        xgc2_px4_command_v1 payload{};
        std::strncpy(payload.text, text, sizeof payload.text - 1);
        emit(kCommand, payload, now);
        host_->log(ctx_, kLogInfo, (std::string("command ") + text).c_str());
    }

    void playOperator(int64_t now) {
        const double since_start = static_cast<double>(now - started_ns_) * 1e-9;
        const double in_state = static_cast<double>(now - state_since_ns_) * 1e-9;
        if (phase_ == "start" && since_start > 1.0) {
            command("takeoff", now);
            phase_ = "takeoff";
        } else if (phase_ == "takeoff" && state_ == "Hover" && in_state > 1.0) {
            command("custom1", now);
            phase_ = "tracking";
        } else if (phase_ == "tracking" && state_ == "Custom1" && in_state > 3.0) {
            command("land", now);
            phase_ = "landing";
        }
    }

    const xgc2_host_api* host_;
    void* ctx_;
    int64_t started_ns_{0}, last_ns_{0}, state_since_ns_{0};
    double z_{0.0}, target_z_{0.0};
    bool armed_{false}, have_setpoint_{false};
    std::string mode_{"MANUAL"}, state_{"unknown"}, phase_{"start"}, reported_phase_;
    Stream pose50_{20000000}, imu200_{5000000}, pose100_{10000000}, battery1_{1000000000};
};

Vehicle* vehicle(xgc2_instance* handle) {
    return reinterpret_cast<Vehicle*>(handle);
}

xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config*, xgc2_instance** out) {
    *out = reinterpret_cast<xgc2_instance*>(new Vehicle(host, ctx));
    return XGC2_OK;
}
xgc2_status configure(xgc2_instance*, const xgc2_config*) {
    return XGC2_OK;
}
xgc2_status start(xgc2_instance* handle) {
    return vehicle(handle)->start();
}
xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* ctx) {
    return vehicle(handle)->step(ctx);
}
xgc2_status stop(xgc2_instance*) {
    return XGC2_OK;
}
void destroy(xgc2_instance* handle) {
    delete vehicle(handle);
}

const xgc2_module_desc kDescriptor = {
    XGC2_MODULE_ABI_MAJOR,
    XGC2_MODULE_ABI_MINOR,
    "px4_scripted_vehicle",
    "1",
    kPorts,
    kPortCount,
    &create,
    &configure,
    &start,
    &step,
    &stop,
    &destroy,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc2_module_desc* xgc2_module_entry(void) {
    return &kDescriptor;
}
