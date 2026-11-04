#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <variant>
#include <vector>
#include <xgc2_math/trajectory.hpp>

#include "multirotor_reference_trajectory/reference_types.h"

namespace multirotor_reference_trajectory {

namespace trajectory = xgc2_math::trajectory;

class ReferenceMachine;

enum class ReferenceEffectKind { Status, ActiveAnalytic, ActiveSampled };
struct ReferenceEffect {
    ReferenceEffectKind kind;
    double timestamp;
};

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
    ReferenceTrajectoryRuntime();
    ~ReferenceTrajectoryRuntime();
    ReferenceTrajectoryRuntime(const ReferenceTrajectoryRuntime&) = delete;
    ReferenceTrajectoryRuntime& operator=(const ReferenceTrajectoryRuntime&) = delete;

    void setConfig(const ReferenceTrajectoryConfig& config);
    void reset();
    void update(double now_sec);
    std::optional<double> nextWake() const;
    const std::vector<ReferenceEffect>& effects() const { return effects_; }

    bool acceptAnalytic(const reference::AnalyticReference& msg, double receipt_sec);
    bool acceptSampled(const reference::SampledReference& msg, double receipt_sec);

    bool activeExpired(double now_sec) const;
    bool hasPendingReference() const;

    uint8_t currentState() const;
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
   private:
    friend class ReferenceMachine;

    struct AnalyticReference {
        reference::AnalyticReference message;
        std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator;
        uint32_t flags;
    };
    struct SampledReference {
        reference::SampledReference message;
        std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator;
        uint32_t flags;
    };

    using Reference = std::variant<std::monostate, AnalyticReference, SampledReference>;
    bool activatePending();
    void emit(ReferenceEffectKind kind);
    void emitActive();
    std::optional<double> activeExpiry() const;
    std::unique_ptr<trajectory::TrajectoryEvaluator3> buildAnalyticEvaluator(
        const reference::AnalyticReference& msg, uint32_t& flags) const;
    bool buildSampledEvaluator(const reference::SampledReference& msg,
                               trajectory::SampledEvaluator3& evaluator, uint32_t& flags) const;
    ReferenceTrajectoryConfig config_{};
    std::unique_ptr<ReferenceMachine> machine_;
    double current_time_sec_{0.0};
    uint32_t flags_{0U};

    Reference pending_;
    // The last reference remains visible in status after Active expires to Ready.
    Reference active_;
    std::vector<ReferenceEffect> effects_;
};

}  // namespace multirotor_reference_trajectory
