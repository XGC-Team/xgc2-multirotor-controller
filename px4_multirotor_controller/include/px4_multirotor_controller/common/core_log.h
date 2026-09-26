#pragma once

#include <cstdarg>
#include <cstdio>

#include "px4_multirotor_controller/common/controller_clock.h"

namespace px4_multirotor_controller {

// Logging for the controller core, without ROS. The core formats a line and
// hands it to one process-wide sink: stderr by default, rosconsole when the
// ROS node installs its sink (ros_log_sink.h), or whatever an aggregator
// module installs.
enum class LogLevel { kInfo, kWarn, kError };
using LogSink = void (*)(LogLevel level, const char* message);

void setLogSink(LogSink sink);
void logMessage(LogLevel level, const char* message);
void logFormat(LogLevel level, const char* format, ...) __attribute__((format(printf, 2, 3)));

// True at most once per `period_s` of controller time for one call site
// (`last_s` is that call site's own state). ROS_*_THROTTLE throttled on ROS
// time, which is what the controller clock follows.
inline bool logThrottleDue(double period_s, double& last_s) {
    const double now = ControllerClock::now().time_since_epoch().count();
    if (now - last_s < period_s) {
        return false;
    }
    last_s = now;
    return true;
}

}  // namespace px4_multirotor_controller

#define PMC_LOG_INFO(...) ::px4_multirotor_controller::logFormat(::px4_multirotor_controller::LogLevel::kInfo, __VA_ARGS__)
#define PMC_LOG_WARN(...) ::px4_multirotor_controller::logFormat(::px4_multirotor_controller::LogLevel::kWarn, __VA_ARGS__)
#define PMC_LOG_ERROR(...) ::px4_multirotor_controller::logFormat(::px4_multirotor_controller::LogLevel::kError, __VA_ARGS__)
#define PMC_LOG_WARN_THROTTLE(period, ...)                                                   \
    do {                                                                                     \
        static thread_local double pmc_log_last_ = -1.0e300;                                 \
        if (::px4_multirotor_controller::logThrottleDue((period), pmc_log_last_)) {          \
            PMC_LOG_WARN(__VA_ARGS__);                                                       \
        }                                                                                    \
    } while (0)
