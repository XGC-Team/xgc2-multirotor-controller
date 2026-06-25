#include "multirotor_reference_trajectory/state_machine/planning_state.h"

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

namespace multirotor_reference_trajectory {

PlanningState::PlanningState(ReferenceTrajectoryRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult PlanningState::onEnter(::state_machine::StateContext& ctx) {
    runtime_.enterState(ReferenceStatus::STATE_PLANNING);
    status_gate_.reset();
    if (!runtime_.requestPendingWaypointPlan()) {
        ::state_machine::Event event(event_type::PLAN_FAILED,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kInternal;
        event.source = "planning_state";
        ctx.postInternalEvent(std::move(event));
        return {};
    }
    publishStatusIfDue(ctx);
    return {};
}

::state_machine::ActionResult PlanningState::onTick(::state_machine::StateContext& ctx) {
    publishStatusIfDue(ctx);
    return {};
}

void PlanningState::publishStatusIfDue(::state_machine::StateContext& ctx) {
    if (!status_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().status_rate_hz)) {
        return;
    }
    ::state_machine::Event event(output_event_type::PUBLISH_STATUS,
                                 ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kOutput;
    event.source = "planning_state";
    ctx.emitOutput(std::move(event));
}

}  // namespace multirotor_reference_trajectory
