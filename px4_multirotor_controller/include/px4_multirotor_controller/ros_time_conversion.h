#pragma once

#include <ros/time.h>

#include "px4_multirotor_controller/common/time.h"

namespace px4_multirotor_controller {

// ROS edge only: the core uses px4_multirotor_controller::Time, whose
// representation is rostime's, so conversion is an exact field copy.
inline Time toCoreTime(const ros::Time& t) { return Time(t.sec, t.nsec); }
inline ros::Time toRosTime(const Time& t) { return ros::Time(t.sec, t.nsec); }

}  // namespace px4_multirotor_controller
