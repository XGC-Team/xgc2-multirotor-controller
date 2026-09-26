#pragma once

namespace px4_multirotor_controller {

// Route the controller core's log (core_log.h) to rosconsole. The ROS node
// calls this once, before it constructs the controller.
void installRosLogSink();

}  // namespace px4_multirotor_controller
