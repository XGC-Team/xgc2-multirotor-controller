// The real ROS input producer against the module: the same messages must lead the controller
// through the same flight states. A SensorInputProducer subscribes to real topics (a private ROS
// master, see controller_transport_parity.test), while the module gets the same messages as
// payloads, mapped as the ROS edge maps them (replay/ros_payload_mapping.h). Time is simulated: the
// producer's receipt time is ros::Time::now(), the module's is the sample stamp, and both are set
// to the tick's time.
//
// The scenario: the sensors speak twice, then fall silent. After 2.5 s without messages they time
// out and the controller leaves Ready (SAFE_TIMEOUT_STATE), on both paths at the same tick.

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gtest/gtest.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <module_support/test_host.hpp>
#include <string>

#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/input/sensor_input_producer.h"
#include "replay/ros_payload_mapping.h"

using namespace px4_multirotor_controller;
namespace edge = ros_payload_mapping;

TEST(ControllerTransports, SameMessagesGiveTheSameStateStampHeartbeatAndStop) {
    module_support::ModuleLibrary library(CONTROLLER_MODULE_PATH);
    module_support::TestHost host(library.desc());
    host.setNow(static_cast<int64_t>(Time(100.0).toNSec()));
    ASSERT_EQ(host.create("{\"world_boundary_json\": \"null\", \"tracking_backend\": \"smc\"}"),
              XGC2_OK);
    ASSERT_EQ(host.start(), XGC2_OK);

    ros::Time::setNow(ros::Time(100.0));
    ros::NodeHandle nh("controller_parity");
    SensorData data;
    ControllerDriver ros_driver(data);
    ControllerParameters parameters([](const std::string& key, ControllerParameterValue& out) {
        if (key == "world_boundary_json") {
            out = std::string("null");
            return true;
        }
        if (key == "tracking_backend") {
            out = std::string("smc");
            return true;
        }
        return false;
    });
    ros_driver.controller().setConfig(readControllerConfig(parameters));
    ros1_utils::PositionQualityStats quality;
    unsigned callbacks = 0;
    SensorInputProducer producer(
        nh, data, quality, ros_driver.statistics(), 20,
        [&](::state_machine::Event event) {
            ++callbacks;
            return ros_driver.controller().getStateMachine().postEvent(std::move(event));
        },
        [] {});
    producer.start();
    ros_driver.start(Time(100.0));
    auto local = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 20);
    auto pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 20);
    auto velocity =
        nh.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 20);
    auto imu = nh.advertise<sensor_msgs::Imu>("mavros/imu/data", 20);
    auto state = nh.advertise<mavros_msgs::State>("mavros/state", 20);
    const auto connected_deadline = ros::WallTime::now() + ros::WallDuration(3.0);
    while ((!local.getNumSubscribers() || !pose.getNumSubscribers() ||
            !velocity.getNumSubscribers() || !imu.getNumSubscribers() ||
            !state.getNumSubscribers()) &&
           ros::WallTime::now() < connected_deadline) {
        ros::spinOnce();
        ros::WallDuration(0.002).sleep();
    }
    ASSERT_TRUE(local.getNumSubscribers() && pose.getNumSubscribers() &&
                velocity.getNumSubscribers() && imu.getNumSubscribers() &&
                state.getNumSubscribers());

    auto tick = [&](double time, bool send) {
        ros::Time::setNow(ros::Time(time));
        host.setNow(static_cast<int64_t>(Time(time).toNSec()));
        if (send) {
            const auto before = callbacks;
            geometry_msgs::PoseStamped p;
            p.header.stamp = ros::Time(time);
            p.pose.orientation.w = 1.0;
            geometry_msgs::TwistStamped v;
            v.header.stamp = p.header.stamp;
            sensor_msgs::Imu i;
            i.header.stamp = p.header.stamp;
            i.orientation.w = 1.0;
            i.linear_acceleration.z = 9.8066;
            mavros_msgs::State s;
            s.header.stamp = p.header.stamp;
            s.connected = true;
            s.mode = "ALTCTL";
            local.publish(p);
            pose.publish(p);
            velocity.publish(v);
            imu.publish(i);
            state.publish(s);
            // The same messages, as the ROS edge hands them to the module.
            const int64_t receipt = host.now();
            ASSERT_TRUE(host.push("local_pose", edge::map(p), receipt));
            ASSERT_TRUE(host.push("vrpn_pose", edge::map(p), receipt));
            ASSERT_TRUE(host.push("local_velocity", edge::map(v), receipt));
            ASSERT_TRUE(host.push("imu", edge::imuSample(i.header.stamp), receipt));
            ASSERT_TRUE(host.push("fcu_state", edge::map(s), receipt));
            const auto delivery_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
            while (callbacks < before + 5 && ros::WallTime::now() < delivery_deadline) {
                ros::spinOnce();
                ros::WallDuration(0.002).sleep();
            }
            EXPECT_EQ(callbacks, before + 5);
            EXPECT_EQ(data.local_pos_stats.last_message_time.toNSec(),
                      static_cast<uint64_t>(host.now()));
        }
        ros_driver.update(Time(time));
        EXPECT_EQ(host.step(), XGC2_OK);
        EXPECT_EQ(ros_driver.controller().getStateMachine().currentStateName(region_type::CONTROL),
                  host.lastReportDetail())
            << "at t=" << time;
        ros_driver.statistics().resetNewFlags();
    };
    tick(100.0, true);
    tick(100.02, true);
    tick(101.0, false);
    tick(102.0, false);
    EXPECT_EQ(host.lastReportDetail(), "Ready");
    tick(102.6, false);
    EXPECT_EQ(host.lastReportDetail(), "Ready");
    tick(103.0, false);
    EXPECT_EQ(host.lastReportDetail(), "Landing");  // the original SAFE_TIMEOUT_STATE wins
    ASSERT_EQ(host.stop(), XGC2_OK);
    ros_driver.stop();
    EXPECT_EQ(host.step(), XGC2_ERR_STATE);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_transport_parity");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
