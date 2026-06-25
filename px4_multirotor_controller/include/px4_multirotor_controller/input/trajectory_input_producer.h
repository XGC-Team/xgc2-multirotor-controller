#pragma once

#include <hover_thrust_estimator/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <multirotor_reference_trajectory/ActivePolynomialReference.h>
#include <multirotor_reference_trajectory/AnalyticReference.h>
#include <multirotor_reference_trajectory/SampledReference.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/uav/active_trajectory_cache.h"

namespace px4_multirotor_controller {

class TrajectoryInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;
    using TrajectorySink = std::function<void(const MpcTrajectoryState&)>;
    using ConfigProvider = std::function<ControllerConfig()>;

    TrajectoryInputProducer(ros::NodeHandle& nh, SensorData& sensor_data,
                            ActiveTrajectoryCache& active_trajectory_cache,
                            ConfigProvider config_provider, EventSink event_sink,
                            TrajectorySink trajectory_sink, uint32_t queue_size);

   private:
    void algSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void activeAnalyticCallback(
        const multirotor_reference_trajectory::AnalyticReference::ConstPtr& msg);
    void activePolynomialCallback(
        const multirotor_reference_trajectory::ActivePolynomialReference::ConstPtr& msg);
    void activeSampledCallback(
        const multirotor_reference_trajectory::SampledReference::ConstPtr& msg);
    void hoverThrustCallback(const hover_thrust_estimator::HoverThrustEstimate::ConstPtr& msg);
    void postInputEvent(::state_machine::EventId event_id, const char* source);

    SensorData& sensor_data_;
    ActiveTrajectoryCache& active_trajectory_cache_;
    ConfigProvider config_provider_;
    EventSink event_sink_;
    TrajectorySink trajectory_sink_;
    ros::Subscriber alg_setpoint_sub_;
    ros::Subscriber active_analytic_sub_;
    ros::Subscriber active_polynomial_sub_;
    ros::Subscriber active_sampled_sub_;
    ros::Subscriber hover_thrust_sub_;
};

}  // namespace px4_multirotor_controller
