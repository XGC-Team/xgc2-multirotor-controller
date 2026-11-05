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
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32MultiArray.h>

#include <xgc2/sdk/test_host.hpp>
#include <string>

#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/input/sensor_input_producer.h"
#include "px4_multirotor_controller/output/debug_output_consumer.h"
#include "replay/ros_payload_mapping.h"

using namespace px4_multirotor_controller;
namespace edge = ros_payload_mapping;

TEST(ControllerTransports, SameMessagesGiveTheSameStateStampHeartbeatAndStop) {
    xgc2::sdk::test::ModuleLibrary library(CONTROLLER_MODULE_PATH);
    xgc2::sdk::test::TestHost host(library.desc(), xgc2::sdk::test::Threading::kHomeWorker);
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
    SensorInputProducer producer(nh, data, quality, ros_driver.statistics(), 20, [] {});
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
    EXPECT_EQ(local.getNumSubscribers(), 1U);
    EXPECT_EQ(velocity.getNumSubscribers(), 1U);

    auto tick = [&](double time, bool send) {
        ros::Time::setNow(ros::Time(time));
        host.setNow(static_cast<int64_t>(Time(time).toNSec()));
        if (send) {
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
            auto all_received = [&] {
                return data.local_pos_stats.is_new && data.vrpn_pose_stats.is_new &&
                       data.local_velocity_stats.is_new && data.imu_stats.is_new &&
                       data.state_stats.is_new;
            };
            while (!all_received() && ros::WallTime::now() < delivery_deadline) {
                ros::spinOnce();
                ros::WallDuration(0.002).sleep();
            }
            EXPECT_TRUE(all_received());
            EXPECT_EQ(data.local_pos_stats.last_message_time.toNSec(),
                      static_cast<uint64_t>(host.now()));
        }
        ros_driver.update(Time(time));
        EXPECT_EQ(host.step(), XGC2_OK);
        EXPECT_EQ(ros_driver.controller().flightStateName(),
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

TEST(ControllerTransports, RosLocalPvUsesActualExactTimePairs) {
    ros::CallbackQueue queue;
    ros::NodeHandle nh("controller_exact_pair");
    nh.setCallbackQueue(&queue);
    SensorData data;
    ControllerDriver driver(data);
    ControllerConfig config;
    config.tracking_backend = TrackingBackend::PVA_SMC;
    config.hover_thrust.source = HoverThrustSource::INTERNAL;
    driver.controller().setConfig(config);
    driver.start(Time(200.0));
    ros1_utils::PositionQualityStats quality;
    SensorInputProducer producer(nh, data, quality, driver.statistics(), 20, [] {}, &driver);
    producer.start();
    auto local = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 20);
    auto velocity = nh.advertise<geometry_msgs::TwistStamped>(
        "mavros/local_position/velocity_local", 20);
    const auto connected = ros::WallTime::now() + ros::WallDuration(3.0);
    while ((!local.getNumSubscribers() || !velocity.getNumSubscribers()) &&
           ros::WallTime::now() < connected) {
        queue.callAvailable();
        ros::WallDuration(0.002).sleep();
    }
    ASSERT_EQ(local.getNumSubscribers(), 1U);
    ASSERT_EQ(velocity.getNumSubscribers(), 1U);
    RecordProperty("pose_publisher_connections", std::to_string(local.getNumSubscribers()));
    RecordProperty("velocity_publisher_connections", std::to_string(velocity.getNumSubscribers()));

    // The private ROS callback queue measures subscription callbacks rather than
    // publisher links, which roscpp may share for same-node subscriptions.
    auto callbacksForOneMessage = [&] {
        unsigned calls = 0;
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while (ros::WallTime::now() < deadline) {
            const auto result = queue.callOne(ros::WallDuration(0.05));
            if (result == ros::CallbackQueue::Called) {
                ++calls;
            } else if (calls && result == ros::CallbackQueue::Empty) {
                break;
            }
        }
        return calls;
    };

    geometry_msgs::PoseStamped position;
    position.header.stamp = ros::Time(100, 20);
    position.pose.position.x = 3.0;
    position.pose.position.z = 1.2;
    geometry_msgs::TwistStamped speed;
    speed.header.stamp = ros::Time(100, 21);
    speed.twist.linear.x = 9.0;
    ros::Time::setNow(ros::Time(201, 0));
    local.publish(position);
    EXPECT_EQ(callbacksForOneMessage(), 1U);
    velocity.publish(speed);
    EXPECT_EQ(callbacksForOneMessage(), 1U);
    ASSERT_TRUE(data.local_pos_stats.is_new && data.local_velocity_stats.is_new);
    EXPECT_FALSE(data.paired_local_pv.has_value());
    EXPECT_DOUBLE_EQ(data.local_x, 3.0);
    EXPECT_DOUBLE_EQ(data.local_vx, 9.0);

    speed.header.stamp = position.header.stamp;
    speed.twist.linear.x = 5.0;
    ros::Time::setNow(ros::Time(201, 20000000));
    velocity.publish(speed);
    EXPECT_EQ(callbacksForOneMessage(), 1U);
    ASSERT_TRUE(data.paired_local_pv);
    EXPECT_DOUBLE_EQ(data.paired_local_pv->position.x(), 3.0);
    EXPECT_DOUBLE_EQ(data.paired_local_pv->velocity.x(), 5.0);
    EXPECT_DOUBLE_EQ(data.local_vx, 5.0);

    ros::Time::setNow(ros::Time(201, 40000000));
    position.header.stamp = ros::Time(100, 22);
    position.pose.position.x = 4.0;
    position.pose.position.z = 1.4;
    local.publish(position);
    EXPECT_EQ(callbacksForOneMessage(), 1U);
    EXPECT_DOUBLE_EQ(data.local_x, 4.0);
    EXPECT_DOUBLE_EQ(driver.hoverThrustRuntime()->input().altitude.value, 1.4);
    EXPECT_DOUBLE_EQ(data.paired_local_pv->position.x(), 3.0);
    driver.statistics().advance(Time(201.2));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 25.0, 1e-6);
    EXPECT_NEAR(data.local_velocity_stats.frequency_hz, 50.0, 1e-6);
    driver.stop();
}

TEST(ControllerTransports, RosDiagnosticsKeepDistinctFactsWithoutEmptyTurns) {
    ros::CallbackQueue queue;
    ros::NodeHandle nh("controller_diagnostic_facts");
    nh.setCallbackQueue(&queue);
    std::vector<std::vector<uint32_t>> received;
    auto subscriber = nh.subscribe<std_msgs::UInt32MultiArray>(
        "alg/state_machine_events", 20,
        [&](const std_msgs::UInt32MultiArray::ConstPtr& msg) { received.push_back(msg->data); });

    SensorData data;
    data.local_pos_stats.is_active = data.local_velocity_stats.is_active = true;
    data.imu_stats.is_active = data.state_stats.is_active = true;
    data.local_pos_stats.is_new = data.vrpn_pose_stats.is_new = true;
    data.local_z = data.vrpn_z = 1.0;
    data.fcu_connected = true;
    DroneController controller(data);
    ControllerConfig config;
    config.takeoff_altitude = 1.0;
    controller.setConfig(config);
    ros1_utils::PositionQualityStats quality;
    const bool debug_print = false;
    state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> worker(nh);
    DebugOutputConsumer consumer(nh, worker, controller, data, quality, debug_print, 20);
    worker.start();

    const auto connect_deadline = ros::WallTime::now() + ros::WallDuration(3.0);
    while (!subscriber.getNumPublishers() && ros::WallTime::now() < connect_deadline) {
        queue.callAvailable(ros::WallDuration(0.01));
    }
    ASSERT_EQ(subscriber.getNumPublishers(), 1U);
    // Wait for the actual TCPROS link, rather than sending a duplicate command
    // while the publisher is still establishing its subscriber connection.
    ros::WallDuration(0.05).sleep();

    auto publishEffects = [&] {
        for (const auto& effect : controller.flightEffects()) {
            if (std::holds_alternative<FlightEvents>(effect))
                EXPECT_TRUE(consumer.handle(effect));
        }
    };
    auto waitForCount = [&](size_t count) {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while (received.size() < count && ros::WallTime::now() < deadline)
            queue.callAvailable(ros::WallDuration(0.01));
        ASSERT_EQ(received.size(), count);
    };

    controller.update(100.0);  // The real SelfCheck -> Ready guard.
    publishEffects();
    waitForCount(1);
    EXPECT_EQ(received[0], (std::vector<uint32_t>{0}));

    ASSERT_TRUE(controller.postCommand(FlightCommand::Takeoff, 100.01));
    ASSERT_TRUE(controller.postCommand(FlightCommand::Takeoff, 100.01));
    controller.update(100.01);
    publishEffects();
    waitForCount(2);
    EXPECT_EQ(received[1], (std::vector<uint32_t>{event_type::TAKEOFF_REQUESTED,
                                                  event_type::TAKEOFF_REQUESTED}));

    data.fcu_mode = "ALTCTL";
    data.state_stats.is_new = true;
    controller.update(100.02);  // First of the original two ALTCTL frames.
    publishEffects();
    queue.callAvailable(ros::WallDuration(0.05));
    EXPECT_EQ(received.size(), 2U);

    data.state_stats.is_new = true;
    controller.update(100.03);  // The actual ALTCTL_READY completion.
    publishEffects();
    waitForCount(3);
    EXPECT_EQ(received[2], (std::vector<uint32_t>{event_type::ALTCTL_READY}));

    ASSERT_TRUE(controller.postCommand(FlightCommand::Land, 100.04));
    ASSERT_TRUE(controller.postCommand(FlightCommand::Track, 100.04));
    controller.update(100.04);
    publishEffects();
    waitForCount(4);
    EXPECT_EQ(received[3], (std::vector<uint32_t>{event_type::LANDING_REQUESTED,
                                                  event_type::TRAJECTORY_TRACKING_REQUESTED}));
    worker.stop();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_transport_parity");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
