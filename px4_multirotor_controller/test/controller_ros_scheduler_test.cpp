#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gtest/gtest.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>
#include <vector>

#include "px4_multirotor_controller/drone_ros_node.h"

TEST(ControllerRosScheduler, RealNodeUsesInputAndDeadlineWithoutPausedClockPump) {
    ros::NodeHandle private_nh("~");
    private_nh.setParam("world_boundary_json", std::string("null"));
    private_nh.setParam("feedback_source", std::string("mavros_local"));
    private_nh.setParam("tracking_backend", std::string("px4_local"));
    private_nh.setParam("skip_takeoff_init_disarm", true);
    private_nh.setParam("debug_print", false);
    // NodeHandle starts roscpp and initializes its clock, so set the test's
    // simulated clock after that initialization rather than before it.
    ros::Time::setNow(ros::Time(100.0));

    std::promise<void> constructed;
    auto ready = constructed.get_future();
    std::thread node_thread([&] {
        try {
            ros::NodeHandle node_nh("scheduler");
            px4_multirotor_controller::DroneRosNode node(node_nh);
            constructed.set_value();
            node.run();
        } catch (...) {
            constructed.set_exception(std::current_exception());
        }
    });
    struct Join {
        std::thread& thread;
        ~Join() { ros::shutdown(); if (thread.joinable()) thread.join(); }
    } join{node_thread};
    ASSERT_EQ(ready.wait_for(std::chrono::seconds(4)), std::future_status::ready);
    ASSERT_NO_THROW(ready.get());

    ros::NodeHandle nh("scheduler");
    ros::CallbackQueue observation_queue;
    ros::NodeHandle observer(nh);
    observer.setCallbackQueue(&observation_queue);
    std::vector<uint32_t> events;
    std::vector<mavros_msgs::PositionTarget> setpoints;
    auto events_sub = observer.subscribe<std_msgs::UInt32MultiArray>(
        "alg/state_machine_events", 20, [&](const std_msgs::UInt32MultiArray::ConstPtr& m) {
            events.insert(events.end(), m->data.begin(), m->data.end());
        });
    auto setpoint_sub = observer.subscribe<mavros_msgs::PositionTarget>(
        "mavros/setpoint_raw/local", 20, [&](const mavros_msgs::PositionTarget::ConstPtr& m) {
            setpoints.push_back(*m);
        });
    auto local = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 20);
    auto pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 20);
    auto velocity = nh.advertise<geometry_msgs::TwistStamped>(
        "mavros/local_position/velocity_local", 20);
    auto imu = nh.advertise<sensor_msgs::Imu>("mavros/imu/data", 20);
    auto state = nh.advertise<mavros_msgs::State>("mavros/state", 20);
    auto command = nh.advertise<std_msgs::String>("/command", 20);
    auto wait = [&](const auto& condition, double seconds) {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(seconds);
        while (!condition() && ros::ok() && ros::WallTime::now() < deadline)
            observation_queue.callAvailable(ros::WallDuration(0.01));
        return condition();
    };
    ASSERT_TRUE(wait([&] {
        return local.getNumSubscribers() && pose.getNumSubscribers() &&
               velocity.getNumSubscribers() && imu.getNumSubscribers() &&
               state.getNumSubscribers() && command.getNumSubscribers() &&
               events_sub.getNumPublishers() && setpoint_sub.getNumPublishers();
    }, 4.0));

    geometry_msgs::PoseStamped p;
    p.pose.orientation.w = 1.0;
    geometry_msgs::TwistStamped v;
    sensor_msgs::Imu sample;
    sample.orientation.w = 1.0;
    sample.linear_acceleration.z = 9.8066;
    mavros_msgs::State fcu;
    fcu.connected = true;
    fcu.mode = "ALTCTL";
    local.publish(p);
    pose.publish(p);
    velocity.publish(v);
    imu.publish(sample);
    state.publish(fcu);
    ASSERT_TRUE(wait([&] { return std::find(events.begin(), events.end(), 0U) != events.end(); },
                     4.0));  // Real SelfCheck -> Ready.
    std_msgs::String takeoff;
    takeoff.data = "takeoff";
    command.publish(takeoff);
    ASSERT_TRUE(wait([&] { return setpoints.size() >= 2; }, 4.0));
    const size_t before = setpoints.size();
    for (int i = 0; i < 5; ++i) imu.publish(sample);
    ros::WallDuration(0.15).sleep();
    observation_queue.callAvailable(ros::WallDuration(0.01));
    EXPECT_EQ(setpoints.size(), before);  // No early numerical/publish turn.

    ros::Time::setNow(ros::Time(100.02));
    ros::WallDuration(0.15).sleep();
    observation_queue.callAvailable(ros::WallDuration(0.01));
    EXPECT_EQ(setpoints.size(), before);
    ros::Time::setNow(ros::Time(100.034));
    EXPECT_TRUE(wait([&] { return setpoints.size() > before; }, 3.0));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_ros_scheduler_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
