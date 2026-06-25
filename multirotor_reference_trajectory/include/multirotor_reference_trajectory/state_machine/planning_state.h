#pragma once

#include <state_machine/state_machine.hpp>

#include "multirotor_reference_trajectory/state_machine/periodic_gate.h"

namespace multirotor_reference_trajectory {

class ReferenceTrajectoryRuntime;

class PlanningState final : public ::state_machine::State {
   public:
    explicit PlanningState(ReferenceTrajectoryRuntime& runtime);
    std::string name() const override {
        return "Planning";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void publishStatusIfDue(::state_machine::StateContext& ctx);
    ReferenceTrajectoryRuntime& runtime_;
    PeriodicGate status_gate_;
};

}  // namespace multirotor_reference_trajectory
