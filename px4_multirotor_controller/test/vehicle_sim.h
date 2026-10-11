#pragma once

// A scripted vehicle around a controller module in an in-test host. It delivers the inputs the
// entity's ROS edge would (MAVROS pose, velocity, IMU and state, the canonical pose, the fused
// state estimate, the hover thrust estimate) at their usual rates, answers the module's flight
// controller requests (mode and arming) and lets the vehicle follow the position setpoint upwards.
// With a reference module attached it also carries the reference requests and the active reference
// between the two modules, as the host's channels do.

#include <cstdint>
#include <cstring>
#include <functional>
#include <module_support/test_host.hpp>
#include <string>
#include <vector>

#include "multirotor_reference_trajectory/payloads.h"
#include "px4_multirotor_controller/common/state_estimate_status.h"
#include "px4_multirotor_controller/payloads.h"

namespace vehicle_sim {

using module_support::TestHost;

constexpr int64_t kMs = 1000000;
constexpr int64_t kStart = 100000000000LL;  // 100 s

class Vehicle {
   public:
    explicit Vehicle(TestHost& controller) : controller_(controller), now_(kStart) {
        controller_.setNow(now_);
    }

    // The reference module the controller's ref_request goes to and whose active reference it
    // reads.
    void attachReference(TestHost* reference) {
        reference_ = reference;
    }

    int64_t now() const {
        return now_;
    }
    double seconds() const {
        return static_cast<double>(now_) * 1e-9;
    }
    double altitude() const {
        return z_;
    }
    const std::string& mode() const {
        return mode_;
    }
    bool armed() const {
        return armed_;
    }
    std::string state() const {
        return controller_.lastReportDetail();
    }

    void setSensorsOff(bool off) {
        sensors_off_ = off;
    }
    void setHoverThrust(bool on) {
        hover_thrust_ = on;
    }
    void setEstimate(bool on) {
        estimate_ = on;
    }

    // The requests, setpoints and attitude-rate commands seen so far.
    std::vector<xgc2_px4_fcu_request_v1> requests;
    std::vector<xgc2_px4_reference_analytic_v1> reference_requests;
    std::vector<xgc2_px4_attitude_rate_target_v1> attitude;
    uint64_t setpoints{0};
    xgc2_px4_position_target_v1 last_setpoint{};
    std::vector<std::string> states;  // control states in the order they were entered

    void step() {
        now_ += kMs;
        controller_.setNow(now_);
        ++tick_;
        deliverSensors();
        if (controller_.step() != XGC2_OK)
            failed = true;
        collect();
        if (reference_ != nullptr)
            stepReference();
        controller_.clearOutputs();
        followSetpoint();
    }

    // An extra step at the current time, as the host runs after a wake().
    void wakeStep() {
        if (controller_.step(XGC2_STEP_WAKE) != XGC2_OK)
            failed = true;
        collect();
        controller_.clearOutputs();
    }

    void run(double duration) {
        const int64_t count = static_cast<int64_t>(duration * 1000.0 + 0.5);
        for (int64_t i = 0; i < count; ++i)
            step();
    }

    // True when `done` became true within `limit` seconds.
    bool runUntil(const std::function<bool()>& done, double limit) {
        const int64_t count = static_cast<int64_t>(limit * 1000.0 + 0.5);
        for (int64_t i = 0; i < count; ++i) {
            if (done())
                return true;
            step();
        }
        return done();
    }

    bool failed{false};

   private:
    static void stamp(int64_t ns, uint32_t& sec, uint32_t& nsec) {
        sec = static_cast<uint32_t>(ns / 1000000000LL);
        nsec = static_cast<uint32_t>(ns % 1000000000LL);
    }

    void deliverSensors() {
        if (sensors_off_)
            return;
        if (tick_ % 20 == 0) {  // 50 Hz
            xgc2_px4_pose_v1 pose{};
            stamp(now_, pose.stamp_sec, pose.stamp_nsec);
            pose.position[0] = x_;
            pose.position[1] = y_;
            pose.position[2] = z_;
            pose.orientation_xyzw[3] = 1.0;
            controller_.push("local_pose", pose, now_);
            xgc2_px4_velocity_v1 velocity{};
            stamp(now_, velocity.stamp_sec, velocity.stamp_nsec);
            controller_.push("local_velocity", velocity, now_);
            xgc2_px4_fcu_state_v1 fcu{};
            stamp(now_, fcu.stamp_sec, fcu.stamp_nsec);
            fcu.connected = 1;
            fcu.armed = armed_ ? 1 : 0;
            fcu.system_status = 4;
            std::strncpy(fcu.mode, mode_.c_str(), sizeof fcu.mode - 1);
            controller_.push("fcu_state", fcu, now_);
            if (hover_thrust_) {
                xgc2_px4_hover_thrust_v1 hover{};
                stamp(now_, hover.stamp_sec, hover.stamp_nsec);
                hover.hover_thrust = 0.5;
                controller_.push("hover_thrust", hover, now_);
            }
        }
        if (tick_ % 5 == 0) {  // 200 Hz
            xgc2_px4_imu_v1 imu{};
            stamp(now_, imu.stamp_sec, imu.stamp_nsec);
            controller_.push("imu", imu, now_);
        }
        if (tick_ % 10 == 0) {  // 100 Hz
            xgc2_px4_pose_v1 pose{};
            stamp(now_, pose.stamp_sec, pose.stamp_nsec);
            pose.position[0] = x_;
            pose.position[1] = y_;
            pose.position[2] = z_;
            pose.orientation_xyzw[3] = 1.0;
            controller_.push("vrpn_pose", pose, now_);
            if (estimate_) {
                xgc2_px4_state_estimate_v1 e{};
                stamp(now_, e.stamp_sec, e.stamp_nsec);
                e.position[0] = x_;
                e.position[1] = y_;
                e.position[2] = z_;
                e.orientation_xyzw[3] = 1.0;
                e.gravity[2] = -9.8066;
                e.filter_inertial_stamp_sec = seconds();
                e.filter_pose_stamp_sec = seconds();
                e.last_vrpn_pose_stamp_sec = seconds();
                e.estimator_state = px4_multirotor_controller::state_estimate::STATE_RUNNING;
                controller_.push("state_estimate", e, now_);
            }
        }
    }

    // What the module wrote in the last step.
    void collect() {
        for (const auto& s : controller_.outputs("fcu_request")) {
            const auto request = s.as<xgc2_px4_fcu_request_v1>();
            requests.push_back(request);
            if (request.kind == XGC2_PX4_FCU_REQUEST_MODE)
                mode_ = request.mode;
            if (request.kind == XGC2_PX4_FCU_REQUEST_ARM)
                armed_ = request.arm != 0;
        }
        setpoints += controller_.outputs("setpoint").size();
        if (!controller_.outputs("setpoint").empty()) {
            last_setpoint =
                controller_.outputs("setpoint").back().as<xgc2_px4_position_target_v1>();
        }
        for (const auto& s : controller_.outputs("attitude_rate")) {
            attitude.push_back(s.as<xgc2_px4_attitude_rate_target_v1>());
        }
        for (const auto& s : controller_.outputs("ref_request")) {
            reference_requests.push_back(s.as<xgc2_px4_reference_analytic_v1>());
        }
        const std::string now_state = controller_.lastReportDetail();
        if (!now_state.empty() && (states.empty() || states.back() != now_state)) {
            states.push_back(now_state);
        }
    }

    // The reference module runs at its own 100 Hz; its input is the controller's request, its
    // output the controller's active reference.
    void stepReference() {
        for (const auto& s : controller_.outputs("ref_request")) {
            reference_->push("analytic", s.as<xgc2_px4_reference_analytic_v1>(), s.stamp_ns);
        }
        if (tick_ % 10 != 0)
            return;
        reference_->setNow(now_);
        if (reference_->step() != XGC2_OK)
            failed = true;
        for (const auto& s : reference_->outputs("active_analytic")) {
            controller_.push("ref_active_analytic", s.as<xgc2_px4_reference_analytic_v1>(), now_);
        }
        for (const auto& s : reference_->outputs("active_sampled")) {
            xgc2_px4_reference_sampled_v1 payload = s.as<xgc2_px4_reference_sampled_v1>();
            controller_.push("ref_active_sampled", payload, now_);
        }
        reference_->clearOutputs();
    }

    // The flight controller follows the position setpoint's altitude once armed in OFFBOARD.
    void followSetpoint() {
        if (!armed_ || mode_ != "OFFBOARD" || setpoints == 0)
            return;
        const double target = last_setpoint.position[2];
        const double step = 1.0e-3;  // 1 m/s
        if (z_ < target)
            z_ = z_ + step < target ? z_ + step : target;
    }

    TestHost& controller_;
    TestHost* reference_{nullptr};
    int64_t now_;
    uint64_t tick_{0};
    double x_{0.0}, y_{0.0}, z_{0.0};
    std::string mode_{"MANUAL"};
    bool armed_{false};
    bool sensors_off_{false};
    bool hover_thrust_{true};
    bool estimate_{true};
};

}  // namespace vehicle_sim
