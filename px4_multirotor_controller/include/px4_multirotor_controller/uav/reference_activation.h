#pragma once

#include <cmath>
#include <cstdint>

#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/uav/reference_types.h"

namespace px4_multirotor_controller {

// The analytic reference request that Custom1 sends when it latches the NMPC
// or DFBC backend (output event PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION): a
// torus knot (with entry) or a circle entry that starts at the current
// estimate, from the nmpc/reference_* configuration. Each request gets new
// request, trajectory and revision numbers. The edge publishes it to the
// reference trajectory generator's analytic request input, frame "map".
class ReferenceActivation {
   public:
    // stamp: the event's timestamp (or the current time when it has none).
    reference::AnalyticReference make(double stamp, const SensorData& sensor, const ControllerConfig& config) {
        reference::AnalyticReference msg;
        msg.header.stamp = Time(stamp);
        msg.request_id = ++request_id_;
        msg.trajectory_id = ++trajectory_id_;
        msg.revision = ++revision_;
        msg.analytic_type = static_cast<uint16_t>(config.nmpc.reference_analytic_type);
        msg.flags = 0U;
        msg.start_time = Time(stamp + config.nmpc.reference_start_delay);
        msg.duration = config.nmpc.reference_duration;

        msg.origin.position.x = finiteOr(sensor.x, 0.0);
        msg.origin.position.y = finiteOr(sensor.y, 0.0);
        msg.origin.position.z = finiteOr(sensor.z, config.nmpc.reference_height);
        msg.origin.orientation.x = finiteOr(sensor.qx, 0.0);
        msg.origin.orientation.y = finiteOr(sensor.qy, 0.0);
        msg.origin.orientation.z = finiteOr(sensor.qz, 0.0);
        msg.origin.orientation.w = finiteOr(sensor.qw, 1.0);

        const double q_norm = std::sqrt(msg.origin.orientation.x * msg.origin.orientation.x +
                                        msg.origin.orientation.y * msg.origin.orientation.y +
                                        msg.origin.orientation.z * msg.origin.orientation.z +
                                        msg.origin.orientation.w * msg.origin.orientation.w);
        if (!std::isfinite(q_norm) || q_norm < 1e-9) {
            msg.origin.orientation.x = 0.0;
            msg.origin.orientation.y = 0.0;
            msg.origin.orientation.z = 0.0;
            msg.origin.orientation.w = 1.0;
        } else {
            msg.origin.orientation.x /= q_norm;
            msg.origin.orientation.y /= q_norm;
            msg.origin.orientation.z /= q_norm;
            msg.origin.orientation.w /= q_norm;
        }

        if (msg.analytic_type == reference::AnalyticReference::ANALYTIC_TORUS_KNOT) {
            const double scale = std::abs(config.nmpc.reference_torus_scale);
            const double start_x = finiteOr(sensor.x, 0.0);
            const double start_y = finiteOr(sensor.y, 0.0);
            const double start_z = finiteOr(sensor.z, config.nmpc.reference_height);
            msg.origin.position.x = start_x;
            msg.origin.position.y = start_y;
            msg.origin.position.z = start_z;
            msg.params = {config.nmpc.reference_torus_omega, scale, config.nmpc.reference_entry_duration,
                          start_x, start_y, start_z - 4.0 * scale};
        } else {
            msg.analytic_type = reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY;
            msg.params = {config.nmpc.reference_radius,      config.nmpc.reference_line_speed,
                          config.nmpc.reference_height,      config.nmpc.reference_z_amplitude,
                          config.nmpc.reference_z_frequency, config.nmpc.reference_entry_duration,
                          finiteOr(sensor.x, 0.0),           finiteOr(sensor.y, 0.0)};
        }
        return msg;
    }

   private:
    static double finiteOr(double value, double fallback) { return std::isfinite(value) ? value : fallback; }

    uint32_t request_id_{0U};
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
};

}  // namespace px4_multirotor_controller
