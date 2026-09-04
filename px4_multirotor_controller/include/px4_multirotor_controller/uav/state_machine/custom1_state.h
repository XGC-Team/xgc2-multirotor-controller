#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/state_machine/timing.h"
#include "px4_multirotor_controller/tracking/dfbc_attitude_rate_strategy.h"
#include "px4_multirotor_controller/tracking/px4_local_raw_strategy.h"

namespace px4_multirotor_controller {

class DroneController;

class Custom1State : public ::state_machine::State {
   public:
    explicit Custom1State(DroneController& controller);
    ~Custom1State() override = default;

    std::string name() const override {
        return "Custom1";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    DroneController& controller_;

    double last_publish_time_{0.0};
    double publish_period_{0.01};
    double command_time_{0.0};
    double hover_x_{0.0};
    double hover_y_{0.0};
    double hover_z_{0.0};
    bool tracking_armed_{false};

    uint64_t request_sequence_{0};
    uint64_t in_flight_sequence_{0};
    uint64_t consumed_result_sequence_{0};
    bool request_in_flight_{false};
    double last_request_time_{0.0};
    double request_deadline_{0.0};
    double last_success_time_{0.0};
    uint32_t consecutive_failures_{0};
    bool nmpc_reference_seen_{false};
    bool reference_exit_event_posted_{false};
    ::state_machine::runtime::Timer<> nmpc_wait_log_timer_;
    ::state_machine::runtime::Timer<> nmpc_stale_output_log_timer_;
    ::state_machine::runtime::Timer<> trajectory_wait_log_timer_;
    DfbcAttitudeRateStrategy dfbc_strategy_;
    Px4LocalRawStrategy px4_local_raw_strategy_;
    bool sync_strategy_entered_{false};

    void handleNmpcEventMode(::state_machine::StateContext& ctx, double current_time);
    void handleSynchronousAttitudeRateMode(::state_machine::StateContext& ctx, double current_time);
    void handlePx4LocalPassThrough(::state_machine::StateContext& ctx, double current_time);
    void consumeNmpcResult(::state_machine::StateContext& ctx, double current_time);
    void dispatchNmpcRequest(::state_machine::StateContext& ctx, double current_time);
    void publishBackupSetpoint(::state_machine::StateContext& ctx, double current_time);
    void publishCurrentHoverSetpoint(::state_machine::StateContext& ctx, double current_time);
    void postReferenceExit(::state_machine::StateContext& ctx, double current_time,
                           uint32_t event_id, const char* reason);
    bool referenceWillFinishBeforeNextHorizon(double current_time) const;
    bool referenceWillFinishBeforeNextSynchronousUpdate(double current_time) const;
    bool shouldDispatchNmpc(double current_time) const;
    bool shouldRunSynchronousStrategy(double current_time) const;
    double synchronousStrategyPeriod() const;
    bool shouldPublish(double current_time) const;
};

}  // namespace px4_multirotor_controller
