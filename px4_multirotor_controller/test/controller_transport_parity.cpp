#include <gtest/gtest.h>
#include <dlfcn.h>
#include <deque>
#include <map>
#include <cstring>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <xgc_rt.h>
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>
#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/input/sensor_input_producer.h"
using namespace px4_multirotor_controller;
struct Host {
    uint64_t now{100000000000ULL};
    std::map<uint32_t, std::deque<std::vector<uint8_t>>> inputs;
    std::vector<uint8_t> held;
    template<class T> void input(uint32_t port, const T& data) {
        const auto* bytes = reinterpret_cast<const uint8_t*>(&data);
        inputs[port].emplace_back(bytes, bytes + sizeof data);
    }
    static xgc_status publish(void*, uint32_t, uint64_t, const uint8_t*, uint32_t) { return XGC_OK; }
    static xgc_status next(void* pointer, uint32_t port, xgc_sample_view* view) {
        auto& host = *static_cast<Host*>(pointer);
        auto& queue = host.inputs[port];
        if (queue.empty()) return XGC_ERR_AGAIN;
        host.held = std::move(queue.front()); queue.pop_front();
        *view = {}; view->data = host.held.data(); view->len = host.held.size();
        view->t_produce = view->t_rx = static_cast<int64_t>(host.now);
        return XGC_OK;
    }
    static int64_t clock(void* pointer) { return static_cast<Host*>(pointer)->now; }
    static void log(void*, xgc_log_level, const char*) {}
};
TEST(ControllerTransports, SameProfileInputStampHeartbeatAndStop) {
    ASSERT_GT(::testing::FLAGS_gtest_filter.size(), 0U);
    const char* elf = std::getenv("XGC_CONTROLLER_NATIVE_ELF"); ASSERT_NE(elf, nullptr);
    void* library = dlopen(elf, RTLD_NOW | RTLD_LOCAL); ASSERT_NE(library, nullptr) << dlerror();
    auto get = reinterpret_cast<xgc_rt_plugin_v1_fn>(dlsym(library, "xgc_rt_plugin_v1")); ASSERT_NE(get, nullptr);
    const auto* descriptor = get(); Host host;
    xgc_host_api api{}; api.abi_version = 1; api.abi_minor = 2; api.host = &host;
    api.publish = Host::publish; api.next = Host::next; api.now = Host::clock; api.log = Host::log;
    void* native = descriptor->vtbl->create(&api); ASSERT_NE(native, nullptr);
    ASSERT_EQ(descriptor->vtbl->configure(native, "tracking_backend = \"smc\"\n"), XGC_OK);
    ASSERT_EQ(descriptor->vtbl->activate(native), XGC_OK);
    ros::Time::setNow(ros::Time(100.0));
    ros::NodeHandle nh("controller_parity"); SensorData data; ControllerDriver ros_driver(data);
    ControllerParameters parameters([](const std::string& key, ControllerParameterValue& out) {
        if (key == "world_boundary_json") { out = std::string("null"); return true; }
        if (key == "tracking_backend") { out = std::string("smc"); return true; }
        return false;
    });
    ros_driver.controller().setConfig(readControllerConfig(parameters));
    ros1_utils::PositionQualityStats quality;
    unsigned callbacks = 0;
    SensorInputProducer producer(nh, data, quality, ros_driver.statistics(), 20,
        [&](::state_machine::Event event) { ++callbacks; return ros_driver.controller().getStateMachine().postEvent(std::move(event)); }, [] {});
    producer.start(); ros_driver.start(Time(100.0));
    auto local = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 20);
    auto pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 20);
    auto velocity = nh.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 20);
    auto imu = nh.advertise<sensor_msgs::Imu>("mavros/imu/data", 20);
    auto state = nh.advertise<mavros_msgs::State>("mavros/state", 20);
    const auto connected_deadline = ros::WallTime::now() + ros::WallDuration(3.0);
    while ((!local.getNumSubscribers() || !pose.getNumSubscribers() || !velocity.getNumSubscribers() || !imu.getNumSubscribers() || !state.getNumSubscribers()) && ros::WallTime::now() < connected_deadline) {
        ros::spinOnce(); ros::WallDuration(0.002).sleep();
    }
    ASSERT_TRUE(local.getNumSubscribers() && pose.getNumSubscribers() && velocity.getNumSubscribers() && imu.getNumSubscribers() && state.getNumSubscribers());
    auto tick = [&](double time, bool send) {
        ros::Time::setNow(ros::Time(time)); host.now = Time(time).toNSec();
        if (send) {
            const auto before = callbacks;
            geometry_msgs::PoseStamped p; p.header.stamp = ros::Time(time); p.pose.orientation.w = 1.0;
            geometry_msgs::TwistStamped v; v.header.stamp = p.header.stamp;
            sensor_msgs::Imu i; i.header.stamp = p.header.stamp; i.orientation.w = 1.0; i.linear_acceleration.z = 9.8066;
            mavros_msgs::State s; s.connected = true; s.mode = "ALTCTL";
            local.publish(p); pose.publish(p); velocity.publish(v); imu.publish(i); state.publish(s);
            xgc_pose_v1 np{}; np.stamp = time; np.q_wxyz[0] = 1.0;
            xgc_twist_v1 nv{}; nv.stamp = time;
            xgc_imu_v1 ni{}; ni.stamp = time; ni.accel[2] = 9.8066;
            xgc_fcu_state_v1 ns{}; ns.stamp = time; ns.connected = 1; std::strcpy(ns.mode, "ALTCTL");
            host.input(1, np); host.input(2, nv); host.input(3, ni); host.input(4, ns); host.input(6, np);
            const auto delivery_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
            while (callbacks < before + 5 && ros::WallTime::now() < delivery_deadline) { ros::spinOnce(); ros::WallDuration(0.002).sleep(); }
            EXPECT_EQ(callbacks, before + 5);
            EXPECT_EQ(data.local_pos_stats.last_message_time.toNSec(), host.now);
        }
        ros_driver.update(Time(time));
        xgc_step_ctx context{}; context.now = host.now; context.round = static_cast<uint64_t>(time * 1000); context.round_advanced = 1;
        EXPECT_EQ(descriptor->vtbl->step(native, &context), XGC_OK);
        EXPECT_EQ(ros_driver.controller().getStateMachine().currentStateName(region_type::CONTROL), descriptor->vtbl->domain_state(native));
        ros_driver.statistics().resetNewFlags();
    };
    tick(100.0, true); tick(100.02, true); tick(101.0, false); tick(102.0, false);
    EXPECT_STREQ(descriptor->vtbl->domain_state(native), "Ready");
    tick(102.6, false); EXPECT_STREQ(descriptor->vtbl->domain_state(native), "Ready");
    tick(103.0, false); EXPECT_STREQ(descriptor->vtbl->domain_state(native), "Landing"); // original SAFE_TIMEOUT_STATE wins
    ASSERT_EQ(descriptor->vtbl->deactivate(native), XGC_OK); ros_driver.stop();
    xgc_step_ctx context{}; EXPECT_EQ(descriptor->vtbl->step(native, &context), XGC_ERR_INVALID);
    descriptor->vtbl->destroy(native); dlclose(library);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_transport_parity"); ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
