#pragma once

#include <multirotor_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/ReferenceStatus.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <multirotor_reference_trajectory_msgs/WaypointReferenceRequest.h>

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <state_machine/state_machine.hpp>
#include <thread>
#include <xgc2_math/trajectory.hpp>

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
    ReferenceTrajectoryRuntime();
    ~ReferenceTrajectoryRuntime();
    ReferenceTrajectoryRuntime(const ReferenceTrajectoryRuntime&) = delete;
    ReferenceTrajectoryRuntime& operator=(const ReferenceTrajectoryRuntime&) = delete;

    void setConfig(const ReferenceTrajectoryConfig& config);
    void reset();
    ::state_machine::Status postEvent(::state_machine::Event event);
    void update(double now_sec);

    bool acceptAnalytic(const multirotor_reference_trajectory_msgs::AnalyticReference& msg);
    bool acceptSampled(const multirotor_reference_trajectory_msgs::SampledReference& msg);
    bool acceptWaypoint(const multirotor_reference_trajectory_msgs::WaypointReferenceRequest& msg);

    bool activatePending();
    bool requestPendingWaypointPlan();
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
    trajectory::TrajectoryModelType activeType() const {
        return active_type_;
    }
    uint32_t activeTrajectoryId() const {
        return active_trajectory_id_;
    }
    uint32_t activeRevision() const {
        return active_revision_;
    }
    uint32_t flags() const {
        return flags_;
    }

    const multirotor_reference_trajectory_msgs::AnalyticReference& activeAnalyticMessage() const {
        return active_analytic_;
    }
    const multirotor_reference_trajectory_msgs::SampledReference& activeSampledMessage() const {
        return active_sampled_;
    }
    const multirotor_reference_trajectory_msgs::ActivePolynomialReference& activePolynomialMessage()
        const {
        return active_polynomial_;
    }
    multirotor_reference_trajectory_msgs::ReferenceStatus makeStatus(double stamp_sec) const;
    const trajectory::TrajectoryEvaluator3* evaluator() const {
        return active_evaluator_.get();
    }
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }

   private:
    enum class PendingKind { kNone, kAnalytic, kSampled, kWaypoint };

    struct PlanningRequest {
        uint64_t sequence{0U};
        uint64_t generation{0U};
        double now_sec{0.0};
        uint32_t active_revision{0U};
        ReferenceTrajectoryConfig config{};
        multirotor_reference_trajectory_msgs::WaypointReferenceRequest msg{};
    };

    struct PlanningResult {
        uint64_t sequence{0U};
        uint64_t generation{0U};
        bool success{false};
        uint32_t flags{0U};
        multirotor_reference_trajectory_msgs::ActivePolynomialReference msg{};
        std::unique_ptr<trajectory::PiecewisePolynomialEvaluator3> evaluator;
    };

    void setupMachine();
    void planningWorkerLoop();
    PlanningResult solveWaypointPlan(const PlanningRequest& request) const;
    void drainPlanningResults(double now_sec);
    void clearPlanningQueuesLocked();
    std::unique_ptr<trajectory::TrajectoryEvaluator3> buildAnalyticEvaluator(
        const multirotor_reference_trajectory_msgs::AnalyticReference& msg, uint32_t& flags) const;
    bool buildSampledEvaluator(const multirotor_reference_trajectory_msgs::SampledReference& msg,
                               trajectory::SampledEvaluator3& evaluator, uint32_t& flags) const;
    bool buildWaypointProblem(
        const multirotor_reference_trajectory_msgs::WaypointReferenceRequest& msg,
        trajectory::WaypointProblem3& problem, uint32_t& flags) const;
    void setActiveAnalytic(const multirotor_reference_trajectory_msgs::AnalyticReference& msg,
                           std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
                           uint32_t flags);
    void setActiveSampled(const multirotor_reference_trajectory_msgs::SampledReference& msg,
                          std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
                          uint32_t flags);
    void setActivePolynomial(multirotor_reference_trajectory_msgs::ActivePolynomialReference msg,
                             std::unique_ptr<trajectory::TrajectoryEvaluator3> evaluator,
                             uint32_t flags);

    ReferenceTrajectoryConfig config_{};
    std::unique_ptr<::state_machine::StateMachine> machine_;
    uint8_t state_{multirotor_reference_trajectory_msgs::ReferenceStatus::STATE_SELF_CHECK};
    double current_time_sec_{0.0};
    uint32_t flags_{0U};

    PendingKind pending_kind_{PendingKind::kNone};
    multirotor_reference_trajectory_msgs::AnalyticReference pending_analytic_;
    multirotor_reference_trajectory_msgs::SampledReference pending_sampled_;
    multirotor_reference_trajectory_msgs::WaypointReferenceRequest pending_waypoint_;

    mutable std::mutex planning_mutex_;
    std::condition_variable planning_condition_;
    std::thread planning_worker_;
    bool planning_stop_{false};
    std::queue<PlanningRequest> planning_requests_;
    std::queue<PlanningResult> planning_results_;
    uint64_t planning_generation_{0U};
    uint64_t planning_sequence_{0U};
    uint64_t expected_planning_sequence_{0U};
    bool has_completed_plan_{false};
    PlanningResult completed_plan_;

    trajectory::TrajectoryModelType active_type_{trajectory::TrajectoryModelType::kNone};
    uint32_t active_trajectory_id_{0U};
    uint32_t active_revision_{0U};
    double active_start_sec_{0.0};
    double active_duration_{0.0};
    std::unique_ptr<trajectory::TrajectoryEvaluator3> active_evaluator_;
    multirotor_reference_trajectory_msgs::AnalyticReference active_analytic_;
    multirotor_reference_trajectory_msgs::SampledReference active_sampled_;
    multirotor_reference_trajectory_msgs::ActivePolynomialReference active_polynomial_;
};

}  // namespace multirotor_reference_trajectory
