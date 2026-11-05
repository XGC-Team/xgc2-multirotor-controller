#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <type_traits>
#include <variant>
#include <vector>

#include <state_machine/adt_machine.hpp>

#include "px4_multirotor_controller/driver/command_input.h"
#include "px4_multirotor_controller/uav/state_machine/custom1_state.h"
#include "px4_multirotor_controller/uav/state_machine/debug_monitor_state.h"
#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"
#include "px4_multirotor_controller/uav/state_machine/hover_state.h"
#include "px4_multirotor_controller/uav/state_machine/landing_state.h"
#include "px4_multirotor_controller/uav/state_machine/ready_state.h"
#include "px4_multirotor_controller/uav/state_machine/self_check_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_arm_request_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_ascending_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_init_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_offboard_request_state.h"

namespace px4_multirotor_controller {

class DroneController;
class FlightRuntime;
struct FlightEnter {};
struct FlightTick {};
struct FlightExit {};
struct FlightChoose {};

// A single flight state owns its local data. Health and diagnostics have their
// original controller lifetime and run before and after the active flight leaf.
struct FlightSpec {
    using State = std::variant<SelfCheckState, ReadyState, TakeoffInitState,
                               TakeoffOffboardRequestState, TakeoffArmRequestState,
                               TakeoffAscendingState, HoverState, Custom1State, LandingState>;
    using Time = std::chrono::steady_clock::time_point;
    using Outcome = ::state_machine::adt::Transition<SelfCheckState, ReadyState, TakeoffInitState,
        TakeoffOffboardRequestState, TakeoffArmRequestState, TakeoffAscendingState, HoverState,
        Custom1State, LandingState>;

    template <class S> static void handle(S& state, const FlightEnter&) { state.onEnter(); }
    template <class S> static void handle(S& state, const FlightExit&) { state.onExit(); }
    template <class S> static void handle(S& state, const FlightTick&) {
        if constexpr (!std::is_same_v<S, ReadyState>) state.onTick();
    }
    template <class S> static void handle(S& state, const FlightCommand& command) {
        if constexpr (std::is_same_v<S, HoverState> || std::is_same_v<S, TakeoffAscendingState>)
            state.onEvent(command);
    }

    static Outcome handle(SelfCheckState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(ReadyState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(TakeoffInitState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(TakeoffOffboardRequestState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(TakeoffArmRequestState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(TakeoffAscendingState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(HoverState&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(Custom1State&, const FlightChoose&, FlightRuntime&);
    static Outcome handle(LandingState&, const FlightChoose&, FlightRuntime&);
};

class FlightRuntime {
   public:
    explicit FlightRuntime(DroneController& controller);
    bool postCommand(FlightCommand command);
    void onFreshImu();
    bool update();
    std::optional<double> nextWake() const;
    void emitSignal(FlightSignal signal);
    FlightState flightState() const;
    std::chrono::steady_clock::duration elapsed() const;
    const HealthMonitorState::PositionDistance& positionDistance() const;
    const std::vector<uint32_t>& diagnosticEvents() const { return diagnostic_events_; }

   private:
    friend struct FlightSpec;
    static FlightSpec::Time asClock(double seconds);
    double entryDeadline(double seconds, bool strict) const;
    bool hasCommand(FlightCommand command) const;
    bool hasSignal(FlightSignal signal) const;
    void selectSignal(FlightSignal signal);
    std::optional<FlightSpec::Outcome> normalPreemption();
    std::optional<FlightSpec::Outcome> takeoffPreemption();

    DroneController& controller_;
    HealthMonitorState health_;
    DebugMonitorState debug_;
    ::state_machine::adt::AdtMachine<FlightSpec> state_;
    mutable std::mutex commands_mutex_;
    std::deque<FlightCommand> pending_commands_;
    std::vector<FlightCommand> commands_;
    std::vector<FlightSignal> signals_;
    std::vector<FlightSignal> deferred_signals_;
    std::optional<size_t> selected_signal_;
    bool selected_command_{false};
    std::vector<uint32_t> diagnostic_events_;
};

}  // namespace px4_multirotor_controller
