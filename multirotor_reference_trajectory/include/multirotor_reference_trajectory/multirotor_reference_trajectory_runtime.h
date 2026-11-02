#pragma once

#include <cstdint>
#include <memory>
#include <state_machine/adt.hpp>
#include <state_machine/state_machine.hpp>
#include <variant>
#include <xgc2_math/trajectory.hpp>

#include "multirotor_reference_trajectory/reference_types.h"
#include "multirotor_reference_trajectory/state_machine/event_types.h"

namespace multirotor_reference_trajectory {

namespace trajectory = xgc2_math::trajectory;

struct ReferenceTrajectoryConfig {
    double status_rate_hz{10.0};
    double active_publish_rate_hz{10.0};
    double validation_sample_dt{0.02};
    double trajectory_timeout{0.5};
    double min_lead_time{0.2};
    trajectory::TrajectoryLimits3 limits{};
};

class ReferenceTrajectoryRuntime {
   public:
    struct ActiveAnalyticSpec {
        reference::AnalyticReference message{};
        std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator{};
        double duration{0.0};
    };

    struct ActiveSampledSpec {
        reference::SampledReference message{};
        std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator{};
        double duration{0.0};
    };

    using PendingSpecVariant =
        std::variant<std::monostate, reference::AnalyticReference, reference::SampledReference>;
    using ActiveSpecVariant =
        std::variant<std::monostate, ActiveAnalyticSpec, ActiveSampledSpec>;

    ReferenceTrajectoryRuntime();
    ReferenceTrajectoryRuntime(const ReferenceTrajectoryRuntime&) = delete;
    ReferenceTrajectoryRuntime& operator=(const ReferenceTrajectoryRuntime&) = delete;

    void setConfig(const ReferenceTrajectoryConfig& config);
    void reset();
    ::state_machine::Status postEvent(::state_machine::Event event);
    void update(double now_sec);

    bool acceptAnalytic(const reference::AnalyticReference& msg);
    bool acceptSampled(const reference::SampledReference& msg);

    bool activatePending();
    bool activeExpired(double now_sec) const;
    bool hasPendingReference() const;

    void enterState(uint8_t state);
    uint8_t currentState() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    const ReferenceTrajectoryConfig& config() const {
        return config_;
    }
    trajectory::TrajectoryModelType activeType() const;
    uint32_t activeTrajectoryId() const;
    uint32_t activeRevision() const;
    uint32_t flags() const {
        return flags_;
    }

    const reference::AnalyticReference& activeAnalyticMessage() const;
    const reference::SampledReference& activeSampledMessage() const;
    reference::ReferenceStatus makeStatus(double stamp_sec) const;
    const trajectory::TrajectoryEvaluator3* evaluator() const;
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }
    ::state_machine::adt::TickPolicy tickPolicy() const;

   private:
    void setupMachine();
    std::unique_ptr<trajectory::TrajectoryEvaluator3> buildAnalyticEvaluator(
        const reference::AnalyticReference& msg, uint32_t& flags) const;
    bool buildSampledEvaluator(const reference::SampledReference& msg,
                               trajectory::SampledEvaluator3& evaluator, uint32_t& flags) const;
    void setActiveAnalytic(const reference::AnalyticReference& msg,
                           std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
                           uint32_t flags);
    void setActiveSampled(const reference::SampledReference& msg,
                          std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
                          uint32_t flags);

    ReferenceTrajectoryConfig config_{};
    std::unique_ptr<::state_machine::StateMachine> machine_;
    uint8_t state_{reference::ReferenceStatus::STATE_SELF_CHECK};
    double current_time_sec_{0.0};
    uint32_t flags_{0U};

    PendingSpecVariant pending_spec_{std::monostate{}};
    ActiveSpecVariant active_spec_{std::monostate{}};
    reference::AnalyticReference empty_analytic_{};
    reference::SampledReference empty_sampled_{};
};

}  // namespace multirotor_reference_trajectory
