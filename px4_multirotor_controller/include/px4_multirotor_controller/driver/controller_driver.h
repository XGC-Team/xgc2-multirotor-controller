#pragma once

#include <hover_thrust_estimator/hover_thrust_estimator_runtime.h>

#include <atomic>
#include <functional>
#include <memory>
#include <optional>

#include "px4_multirotor_controller/common/core_log.h"
#include "px4_multirotor_controller/driver/body_inertial_input.h"
#include "px4_multirotor_controller/driver/nmpc_execution.h"
#include "px4_multirotor_controller/driver/sensor_statistics.h"
#include "px4_multirotor_controller/driver/trajectory_ingress.h"

namespace px4_multirotor_controller {
// Owns domain scheduling and async work. The ROS node and the module only marshal
// inputs, supply the domain clock, and consume outputs; no transport-specific solver
// or second health policy lives beside this driver.
class ControllerDriver {
   public:
    // on_nmpc_result is called after every NMPC solve result (success, failure or
    // rejection) has been stored, from the solver's worker thread or
    // from the thread that requested the solve. The module wakes its host with it, so the
    // update that consumes the result runs now and not at the next period.
    explicit ControllerDriver(SensorData& sensor, std::function<void()> on_nmpc_result = {})
        : sensor_(sensor),
          controller_(sensor),
          statistics_(sensor),
          on_nmpc_result_(std::move(on_nmpc_result)),
          nmpc_(
              controller_, [this] { return Time().fromNSec(clock_ns_.load()); },
              [this] {
                  if (on_nmpc_result_)
                      on_nmpc_result_();
              }) {}
    DroneController& controller() {
        return controller_;
    }
    SensorStatistics& statistics() {
        return statistics_;
    }
    NmpcExecution& nmpc() {
        return nmpc_;
    }
    bool internalHoverThrust() const {
        return controller_.getConfig().hover_thrust.source == HoverThrustSource::INTERNAL;
    }
    bool usesNumericalImu() const {
        return sensor_.feedback_source == FeedbackSource::MAVROS_LOCAL || hover_thrust_ != nullptr;
    }
    void ingestBodyImu(const BodyInertialInput& input, double receipt_sec) {
        if (sensor_.feedback_source == FeedbackSource::MAVROS_LOCAL)
            ingestBodyInertial(sensor_, input, imu_gravity_);
        if (hover_thrust_)
            checkHoverInput(
                hover_thrust_->ingestImu(input.specific_force.z(), input.stamp_sec, receipt_sec));
    }
    void ingestThrust(double thrust, bool ignored, double stamp_sec, double receipt_sec) {
        if (hover_thrust_)
            checkHoverInput(hover_thrust_->ingestThrust(thrust, ignored, stamp_sec, receipt_sec));
    }
    void ingestAltitude(double altitude, double stamp_sec, double receipt_sec) {
        if (hover_thrust_)
            checkHoverInput(hover_thrust_->ingestAltitude(altitude, stamp_sec, receipt_sec));
    }
    void ingestPairedLocalPv(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
        sensor_.paired_local_pv = SensorData::LocalPv{position, velocity};
    }
    const hover_thrust_estimator::HoverThrustEstimatorRuntime* hoverThrustRuntime() const {
        return hover_thrust_.get();
    }
    void start(const Time& now) {
        const auto config = controller_.getConfig();
        sensor_.feedback_source = config.feedback_source;
        imu_gravity_ = config.nmpc.gravity;
        if (config.hover_thrust.source == HoverThrustSource::INTERNAL) {
            hover_thrust_ = std::make_unique<hover_thrust_estimator::HoverThrustEstimatorRuntime>();
            hover_thrust_->setConfig(config.hover_thrust.estimator);
            last_hover_update_sec_ = now.toSec();
            next_hover_deadline_sec_ = hover_thrust_->nextWake(now.toSec()).next_deadline_sec;
        }
        clock_ns_.store(now.toNSec());
        statistics_.start(now);
        nmpc_.start();
        active_ = true;
    }
    void update(const Time& now) {
        if (!active_)
            return;
        clock_ns_.store(now.toNSec());
        statistics_.advance(now);
        const double now_sec = now.toSec();
        if (hover_thrust_ &&
            (hover_thrust_->hasPendingInput() || now_sec < last_hover_update_sec_ ||
             (next_hover_deadline_sec_ && now_sec >= *next_hover_deadline_sec_))) {
            hover_thrust_->update(now_sec);
            last_hover_update_sec_ = now_sec;
            next_hover_deadline_sec_ = hover_thrust_->nextWake(now_sec).next_deadline_sec;
            for (const auto& estimate : hover_thrust_->publishedEstimates()) {
                ingestHoverThrust(sensor_, estimate.output.hover_thrust, estimate.output.flags,
                                  estimate.stamp_sec, now_sec);
            }
        }
        controller_.update(now.toSec());
    }
    // In-flight completion is joined before return; no callback can outlive the
    // caller. Pending work is dropped and new dispatch is rejected by the same
    // original worker stop/busy gate used by ROS.
    void stop() {
        active_ = false;
        nmpc_.stop();
        hover_thrust_.reset();
        next_hover_deadline_sec_.reset();
    }

   private:
    static void checkHoverInput(const ::state_machine::Status& status) {
        if (!status.ok())
            PMC_LOG_ERROR("[ControllerDriver] hover thrust input: %s", status.message.c_str());
    }
    SensorData& sensor_;
    DroneController controller_;
    std::unique_ptr<hover_thrust_estimator::HoverThrustEstimatorRuntime> hover_thrust_;
    std::optional<double> next_hover_deadline_sec_;
    double last_hover_update_sec_{0.0};
    SensorStatistics statistics_;
    std::atomic<uint64_t> clock_ns_{0};
    std::function<void()> on_nmpc_result_;
    NmpcExecution nmpc_;
    double imu_gravity_{9.8066};
    bool active_{false};
};
}  // namespace px4_multirotor_controller
