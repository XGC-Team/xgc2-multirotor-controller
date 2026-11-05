#include <ros/ros.h>

#include "px4_multirotor_controller/drone_ros_node.h"
#include "px4_multirotor_controller/ros_log_sink.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "px4_multirotor_controller_node");
    // The controller core logs through core_log.h; send it to rosconsole.
    px4_multirotor_controller::installRosLogSink();
    ros::NodeHandle nh;  // 使用全局命名空间，让launch文件的ns自动生效

    ROS_INFO("========================================");
    ROS_INFO("  Multirotor Controller Node Starting");
    ROS_INFO("========================================");

    try {
        // 创建ROS节点
        px4_multirotor_controller::DroneRosNode node(nh);

        ROS_INFO("Multirotor Controller Node initialized successfully");
        ROS_INFO("All drones subscribe to: /command (broadcast control)");
        ROS_INFO("Publishing controller status on topic: controller_status");
        ROS_INFO("========================================");

        node.run();

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }

    return 0;
}
