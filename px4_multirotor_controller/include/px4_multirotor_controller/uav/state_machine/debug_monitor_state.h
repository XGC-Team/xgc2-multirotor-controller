#pragma once

#include <state_machine/state_machine.hpp>

namespace px4_multirotor_controller {

class DroneController;

class DebugMonitorState final : public ::state_machine::State {
   public:
    explicit DebugMonitorState(DroneController& controller);

    std::string name() const override {
        return "DebugMonitor";
    }

   protected:
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void emit(::state_machine::StateContext& ctx, ::state_machine::EventId event_id,
              double timestamp) const;

    static constexpr double STATUS_PUBLISH_INTERVAL = 0.1;
    static constexpr double DEBUG_PRINT_INTERVAL = 1.0;

    DroneController& controller_;
    double last_status_publish_time_{0.0};
    double last_debug_print_time_{0.0};
    bool status_publish_initialized_{false};
    bool debug_print_initialized_{false};
};

}  // namespace px4_multirotor_controller
