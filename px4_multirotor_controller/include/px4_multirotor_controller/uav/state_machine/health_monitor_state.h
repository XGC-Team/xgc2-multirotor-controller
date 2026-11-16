#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include "px4_multirotor_controller/uav/flight_signal.h"

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

class DroneController;

class HealthMonitorState final {
   public:
    // One cached comparison of aligned canonical pose and MAVROS local pose, in metres.
    struct PositionDistance {
        double metres{std::numeric_limits<double>::quiet_NaN()};
        bool available{false};
        bool exceeded{false};
    };
    struct SafetyTriggerEvidence {
        bool triggered{false};
        uint32_t event_id{0};
        FlightSignal signal{FlightSignal::Geofence};
        std::string reason;
        double measured{0.0};
        double threshold{0.0};
        double dt_s{0.0};
        double timestamp_s{0.0};
    };
    static constexpr double kPositionDistanceLimitMetres = 0.5;

    explicit HealthMonitorState(DroneController& controller);

    const PositionDistance& positionDistance() const {
        return position_distance_;
    }
    const SafetyTriggerEvidence& lastSafetyTrigger() const {
        return last_safety_trigger_;
    }
    bool isPoseTimedOut() const;
    bool isLocalPositionFaulted() const;
    void resetSafetyLatches();
    uint64_t mocapStampAnomalyCount() const {
        return vrpn_tracker_.stamp_anomaly_count;
    }
    uint64_t localStampAnomalyCount() const {
        return local_tracker_.stamp_anomaly_count;
    }

    void onTick();
    void onFreshImu(std::vector<FlightSignal>& signals);
    std::optional<double> nextWake() const;

   private:
    struct PendingVrpnSample {
        SensorData::PoseSample sample;
        int64_t t_target_ns{0};
    };
    struct PoseStreamTracker {
        std::optional<SensorData::PoseSample> prev_valid;
        std::optional<uint64_t> last_pairing_stamp_ns;
        uint64_t stamp_anomaly_count{0};
    };
    struct SafetyState {
        bool was_uav_state_estimate_active{false};
        bool was_local_pos_active{false};
        bool was_local_velocity_active{false};
        bool was_imu_active{false};
        bool was_state_active{false};
        bool was_vrpn_pose_active{false};

        bool local_pose_timed_out{false};
        bool vrpn_pose_timed_out{false};
        bool local_pose_faulted{false};
        bool was_vrpn_pose_invalid{false};
        bool was_local_pose_invalid{false};
        uint64_t last_seen_vrpn_invalid_count{0};
        uint64_t last_seen_local_invalid_count{0};

        bool geofence_violated{false};
        bool mocap_gross_bounds_violated{false};
        bool local_gross_bounds_violated{false};
        bool velocity_xy_exceeded{false};
        bool velocity_z_exceeded{false};
        bool state_estimate_unusable{false};
        double state_estimate_unusable_since{-1.0};
    };

    void checkSensorActiveEdge(const SensorData::TopicStats& stats, bool& was_active,
                               FlightSignal signal, const char* reason, double threshold);
    void emitSafetySignal(FlightSignal signal, const char* reason, double measured,
                          double threshold, double dt_s, bool mark_local_fault = false);
    bool evaluatePoseSampleSafety(const SensorData::PoseSample& sample,
                                  PoseStreamTracker& tracker, bool& gross_violated_latch,
                                  FlightSignal gross_signal, const char* gross_reason,
                                  FlightSignal jump_signal, const char* pos_jump_reason,
                                  const char* att_jump_reason, bool clear_pairing_on_reset,
                                  bool mark_local_fault);

    DroneController& controller_;
    SafetyState safety_state_;
    PositionDistance position_distance_;
    SafetyTriggerEvidence last_safety_trigger_;
    uint64_t last_processed_local_seq_{0};
    uint64_t last_processed_vrpn_seq_{0};
    uint32_t last_error_event_id_{0};
    double last_error_log_s_{-1.0e9};
    std::deque<SensorData::PoseSample> local_history_;
    std::deque<PendingVrpnSample> pending_vrpn_;
    PoseStreamTracker vrpn_tracker_;
    PoseStreamTracker local_tracker_;
};

}  // namespace px4_multirotor_controller
