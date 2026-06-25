#include <ros/ros.h>

#include "px4_multirotor_controller/drone_ros_node.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "px4_multirotor_controller_node");
    ros::NodeHandle nh;  // 使用全局命名空间，让launch文件的ns自动生效

    ROS_INFO("========================================");
    ROS_INFO("  Multirotor Controller Node Starting");
    ROS_INFO("========================================");

    try {
        // 硬编码控制频率
        const double control_frequency = 1000.0;
        ROS_INFO("Control frequency: %.1f Hz (hardcoded)", control_frequency);

        // 创建ROS节点
        px4_multirotor_controller::DroneRosNode node(nh);

        ROS_INFO("Multirotor Controller Node initialized successfully");
        ROS_INFO("All drones subscribe to: /command (broadcast control)");
        ROS_INFO("Publishing controller status on topic: controller_status");
        ROS_INFO("========================================");

        // 运行节点（硬编码控制频率）
        node.run(control_frequency);

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }

    return 0;
}
