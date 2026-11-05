#include "px4_multirotor_controller/uav/flight_state.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {
namespace {
using ::state_machine::adt::Enter;
using ::state_machine::adt::Stay;

uint32_t diagnosticId(FlightCommand command) {
    switch (command) {
        case FlightCommand::Takeoff: return event_type::TAKEOFF_REQUESTED;
        case FlightCommand::Land: return event_type::LANDING_REQUESTED;
        case FlightCommand::Hover: return event_type::HOVER_REQUESTED;
        case FlightCommand::Track: return event_type::TRAJECTORY_TRACKING_REQUESTED;
    }
    return event_type::UNKNOWN;
}
}  // namespace

FlightSpec::Time FlightRuntime::asClock(double seconds) {
    return FlightSpec::Time(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds)));
}

FlightRuntime::FlightRuntime(DroneController& controller)
    : controller_(controller), health_(controller), debug_(controller),
      state_(SelfCheckState(controller), asClock(controller.getCurrentTime())) {
    state_.dispatch(FlightEnter{});
}

bool FlightRuntime::postCommand(FlightCommand command) {
    std::lock_guard<std::mutex> lock(commands_mutex_);
    // These are the original engine's pending capacity and turn batch limit.
    if (pending_commands_.size() >= 4096)
        return false;
    pending_commands_.push_back(command);
    return true;
}

void FlightRuntime::emitSignal(FlightSignal signal) {
    signals_.push_back(signal);
    diagnostic_events_.push_back(static_cast<uint32_t>(signal));
}

void FlightRuntime::onFreshImu() {
    // The input owner invokes this once per delivered IMU; the real signals
    // join the same turn's health facts, without an empty-turn edge latch.
    health_.onFreshImu(signals_);
}

bool FlightRuntime::hasCommand(FlightCommand command) const {
    return std::find(commands_.begin(), commands_.end(), command) != commands_.end();
}

bool FlightRuntime::hasSignal(FlightSignal signal) const {
    return std::find(signals_.begin(), signals_.end(), signal) != signals_.end();
}

void FlightRuntime::selectSignal(FlightSignal signal) {
    const auto it = std::find(signals_.begin(), signals_.end(), signal);
    selected_signal_ = static_cast<size_t>(std::distance(signals_.begin(), it));
}

std::optional<FlightSpec::Outcome> FlightRuntime::normalPreemption() {
    // Rule order of the existing Normal parent; sensor edges precede Land.
    constexpr std::array<FlightSignal, 11> safety_order = {
        FlightSignal::Geofence, FlightSignal::VelocityXY, FlightSignal::VelocityZ,
        FlightSignal::SaturationXY, FlightSignal::SaturationZ, FlightSignal::EstimateUnusable,
        FlightSignal::TimeoutEstimate, FlightSignal::TimeoutLocalPosition,
        FlightSignal::TimeoutLocalVelocity, FlightSignal::TimeoutImu, FlightSignal::TimeoutFcuState};
    for (const auto signal : safety_order) {
        if (hasSignal(signal)) {
            selectSignal(signal);
            return FlightSpec::Outcome{Enter{LandingState(controller_)}};
        }
    }
    if (hasCommand(FlightCommand::Land)) {
        selected_command_ = true;
        return FlightSpec::Outcome{Enter{LandingState(controller_)}};
    }
    return std::nullopt;
}

std::optional<FlightSpec::Outcome> FlightRuntime::takeoffPreemption() {
    if (auto normal = normalPreemption()) return normal;
    if (health_.positionDistance().exceeded)
        return FlightSpec::Outcome{Enter{LandingState(controller_)}};
    return std::nullopt;
}

bool FlightRuntime::update() {
    commands_.clear();
    bool more_commands = false;
    {
        std::lock_guard<std::mutex> lock(commands_mutex_);
        const size_t take = std::min<size_t>(64, pending_commands_.size());
        for (size_t i = 0; i < take; ++i) {
            commands_.push_back(pending_commands_.front());
            pending_commands_.pop_front();
        }
        more_commands = !pending_commands_.empty();
    }
    const size_t fresh_imu_signals = signals_.size();
    signals_.insert(signals_.end(), deferred_signals_.begin(), deferred_signals_.end());
    deferred_signals_.clear();
    diagnostic_events_.clear();
    for (const auto command : commands_) diagnostic_events_.push_back(diagnosticId(command));
    selected_signal_.reset();
    selected_command_ = false;

    health_.onTick();                         // HEALTH, using the previous setpoint
    for (size_t i = 0; i < fresh_imu_signals; ++i)
        diagnostic_events_.push_back(static_cast<uint32_t>(signals_[i]));
    const size_t control_signal_start = signals_.size();
    state_.dispatch(FlightTick{});            // active CONTROL leaf
    auto choice = state_.dispatch(FlightChoose{}, *this);
    const bool transitioned = !std::holds_alternative<Stay>(choice);
    if (transitioned) {
        if (!selected_signal_ && !selected_command_) diagnostic_events_.push_back(0);
        state_.dispatch(FlightExit{});
        state_.apply(std::move(choice), asClock(controller_.getCurrentTime()));
        state_.dispatch(FlightEnter{});
    } else {
        for (const auto command : commands_) state_.dispatch(command);
    }
    // Unconsumed events generated by CONTROL were deferred by the old engine
    // for one turn; events from HEALTH were not.
    for (size_t i = control_signal_start; i < signals_.size(); ++i)
        if (!selected_signal_ || *selected_signal_ != i) deferred_signals_.push_back(signals_[i]);
    // The old owner handled a remaining batch, a deferred completion, and the
    // newly entered state's next turn without waiting for another input.
    debug_.onTick();                          // DEBUG, after the transition
    signals_.clear();
    return transitioned || more_commands || !deferred_signals_.empty();
}

double FlightRuntime::entryDeadline(double seconds, bool strict) const {
    using Duration = std::chrono::steady_clock::duration;
    auto delta = std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
    const double represented = std::chrono::duration<double>(delta).count();
    if (represented < seconds || (strict && represented == seconds))
        delta += Duration{1};
    const auto target = state_.entered_at() + delta;
    double due = std::chrono::duration<double>(target.time_since_epoch()).count();
    if (asClock(due) < target)
        due = std::nextafter(due + std::chrono::duration<double>(Duration{1}).count(),
                             std::numeric_limits<double>::infinity());
    return due;
}

std::optional<double> FlightRuntime::nextWake() const {
    auto due = earlierDeadline(health_.nextWake(), debug_.nextWake());
    std::visit([&](const auto& state) {
        using S = std::decay_t<decltype(state)>;
        if constexpr (!std::is_same_v<S, ReadyState>)
            due = earlierDeadline(due, state.nextWake());
        if constexpr (std::is_same_v<S, TakeoffInitState> ||
                      std::is_same_v<S, TakeoffOffboardRequestState> ||
                      std::is_same_v<S, TakeoffArmRequestState>) {
            due = earlierDeadline(due, entryDeadline(10.0, true));
        } else if constexpr (std::is_same_v<S, TakeoffAscendingState>) {
            if (const auto timeout = state.timeoutAfterEntry())
                due = earlierDeadline(due, entryDeadline(*timeout, true));
        } else if constexpr (std::is_same_v<S, LandingState>) {
            if (const auto timeout = state.timeoutAfterEntry())
                due = earlierDeadline(due, entryDeadline(*timeout, false));
        }
    }, state_.state());
    return due;
}

std::chrono::steady_clock::duration FlightRuntime::elapsed() const {
    return asClock(controller_.getCurrentTime()) - state_.entered_at();
}

const HealthMonitorState::PositionDistance& FlightRuntime::positionDistance() const {
    return health_.positionDistance();
}

FlightState FlightRuntime::flightState() const {
    return std::visit([](const auto& state) {
        using S = std::decay_t<decltype(state)>;
        if constexpr (std::is_same_v<S, SelfCheckState>) return FlightState::SelfCheck;
        else if constexpr (std::is_same_v<S, ReadyState>) return FlightState::Ready;
        else if constexpr (std::is_same_v<S, TakeoffInitState>) return FlightState::TakeoffInit;
        else if constexpr (std::is_same_v<S, TakeoffOffboardRequestState>) return FlightState::TakeoffOffboardRequest;
        else if constexpr (std::is_same_v<S, TakeoffArmRequestState>) return FlightState::TakeoffArmRequest;
        else if constexpr (std::is_same_v<S, TakeoffAscendingState>) return FlightState::TakeoffAscending;
        else if constexpr (std::is_same_v<S, HoverState>) return FlightState::Hover;
        else if constexpr (std::is_same_v<S, Custom1State>) return FlightState::Custom1;
        else return FlightState::Landing;
    }, state_.state());
}

FlightSpec::Outcome FlightSpec::handle(SelfCheckState&, const FlightChoose&, FlightRuntime& rt) {
    const auto& sensor = rt.controller_.getSensorData();
    if (sensor_checks::areSensorsReady(sensor, rt.controller_.getConfig().tracking_backend))
        return Enter{ReadyState(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(ReadyState&, const FlightChoose&, FlightRuntime& rt) {
    if (auto normal = rt.normalPreemption()) return std::move(*normal);
    const auto& sensor = rt.controller_.getSensorData();
    const auto backend = rt.controller_.getConfig().tracking_backend;
    if (sensor_checks::isAirborne(sensor, backend)) return Enter{LandingState(rt.controller_)};
    const auto& distance = rt.health_.positionDistance();
    if (rt.hasCommand(FlightCommand::Takeoff) && distance.available && !distance.exceeded) {
        rt.selected_command_ = true;
        return Enter{TakeoffInitState(rt.controller_)};
    }
    if (!sensor_checks::areSensorsReady(sensor, backend)) return Enter{SelfCheckState(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(TakeoffInitState&, const FlightChoose&, FlightRuntime& rt) {
    if (auto preemption = rt.takeoffPreemption()) return std::move(*preemption);
    if (sensor_checks::isFcuArmed(rt.controller_.getSensorData())) return Enter{ReadyState(rt.controller_)};
    if (rt.hasSignal(FlightSignal::AltctlReady)) {
        rt.selectSignal(FlightSignal::AltctlReady);
        return Enter{TakeoffOffboardRequestState(rt.controller_)};
    }
    if (std::chrono::duration<double>(rt.elapsed()).count() > 10.0) return Enter{LandingState(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(TakeoffOffboardRequestState&, const FlightChoose&, FlightRuntime& rt) {
    if (auto preemption = rt.takeoffPreemption()) return std::move(*preemption);
    if (sensor_checks::isFcuArmed(rt.controller_.getSensorData())) return Enter{ReadyState(rt.controller_)};
    if (rt.hasSignal(FlightSignal::OffboardReady)) {
        rt.selectSignal(FlightSignal::OffboardReady);
        return Enter{TakeoffArmRequestState(rt.controller_)};
    }
    if (std::chrono::duration<double>(rt.elapsed()).count() > 10.0) return Enter{LandingState(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(TakeoffArmRequestState&, const FlightChoose&, FlightRuntime& rt) {
    if (auto preemption = rt.takeoffPreemption()) return std::move(*preemption);
    if (rt.hasSignal(FlightSignal::ArmReady)) {
        rt.selectSignal(FlightSignal::ArmReady);
        return Enter{TakeoffAscendingState(rt.controller_)};
    }
    if (std::chrono::duration<double>(rt.elapsed()).count() > 10.0) return Enter{LandingState(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(TakeoffAscendingState&, const FlightChoose&, FlightRuntime& rt) {
    if (auto preemption = rt.takeoffPreemption()) return std::move(*preemption);
    if (rt.hasCommand(FlightCommand::Hover)) {
        rt.selected_command_ = true;
        return Enter{HoverState(rt.controller_)};
    }
    if (rt.hasCommand(FlightCommand::Track) && rt.controller_.custom1ReferenceReady()) {
        rt.selected_command_ = true;
        return Enter{Custom1State(rt.controller_)};
    }
    if (rt.hasSignal(FlightSignal::AltitudeReached)) {
        rt.selectSignal(FlightSignal::AltitudeReached);
        return Enter{HoverState(rt.controller_)};
    }
    if (rt.hasSignal(FlightSignal::TakeoffTimeout)) {
        rt.selectSignal(FlightSignal::TakeoffTimeout);
        return Enter{LandingState(rt.controller_)};
    }
    if (rt.controller_.custom1Requested() && rt.controller_.custom1ReferenceReady())
        return Enter{Custom1State(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(HoverState&, const FlightChoose&, FlightRuntime& rt) {
    if (auto normal = rt.normalPreemption()) return std::move(*normal);
    if (rt.health_.positionDistance().exceeded) return Enter{LandingState(rt.controller_)};
    if (rt.hasCommand(FlightCommand::Track) && rt.controller_.custom1ReferenceReady()) {
        rt.selected_command_ = true;
        return Enter{Custom1State(rt.controller_)};
    }
    if (rt.controller_.custom1Requested() && rt.controller_.custom1ReferenceReady())
        return Enter{Custom1State(rt.controller_)};
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(Custom1State&, const FlightChoose&, FlightRuntime& rt) {
    if (auto normal = rt.normalPreemption()) return std::move(*normal);
    if (rt.health_.positionDistance().exceeded) return Enter{LandingState(rt.controller_)};
    if (rt.hasCommand(FlightCommand::Hover)) {
        rt.selected_command_ = true;
        return Enter{HoverState(rt.controller_)};
    }
    if (rt.hasSignal(FlightSignal::ReferenceFinished)) {
        rt.selectSignal(FlightSignal::ReferenceFinished);
        return Enter{HoverState(rt.controller_)};
    }
    return Stay{};
}

FlightSpec::Outcome FlightSpec::handle(LandingState&, const FlightChoose&, FlightRuntime& rt) {
    if (rt.hasSignal(FlightSignal::Touchdown)) {
        rt.selectSignal(FlightSignal::Touchdown);
        return Enter{SelfCheckState(rt.controller_)};
    }
    return Stay{};
}

}  // namespace px4_multirotor_controller
