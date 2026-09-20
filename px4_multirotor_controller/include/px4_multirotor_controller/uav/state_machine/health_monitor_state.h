#pragma once

#include <limits>
#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

class DroneController;

class HealthMonitorState final : public ::state_machine::State {
   public:
    // One cached comparison of canonical pose and MAVROS local pose, in metres.
    struct PositionDistance {
        double metres{std::numeric_limits<double>::quiet_NaN()};
        bool available{false};
        bool exceeded{false};
    };
    static constexpr double kPositionDistanceLimitMetres = 1.0;

    explicit HealthMonitorState(DroneController& controller);

    const PositionDistance& positionDistance() const {
        return position_distance_;
    }

    std::string name() const override {
        return "HealthMonitor";
    }

   protected:
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    struct SafetyState {
        bool was_uav_state_estimate_active{false};
        bool was_local_pos_active{false};
        bool was_local_velocity_active{false};
        bool was_imu_active{false};
        bool was_state_active{false};

        bool geofence_violated{false};
        bool velocity_xy_exceeded{false};
        bool velocity_z_exceeded{false};
        bool control_saturated_xy{false};
        bool control_saturated_z{false};
        bool state_estimate_unusable{false};
        double state_estimate_unusable_since{-1.0};
    };

    void postSafetyEvent(::state_machine::StateContext& ctx, ::state_machine::EventId event_id,
                         const char* operation) const;
    void checkSensorActiveEdge(::state_machine::StateContext& ctx,
                               const SensorData::TopicStats& stats, bool& was_active,
                               ::state_machine::EventId event_id) const;

    DroneController& controller_;
    SafetyState safety_state_;
    PositionDistance position_distance_;
    bool received_canonical_pose_{false};
    bool received_local_pose_{false};
};

}  // namespace px4_multirotor_controller
