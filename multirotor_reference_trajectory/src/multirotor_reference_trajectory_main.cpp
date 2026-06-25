#include <ros/ros.h>

#include <cmath>

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "multirotor_reference_trajectory_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double main_frequency_hz = 100.0;
    private_nh.param("main_frequency", main_frequency_hz, main_frequency_hz);
    if (!std::isfinite(main_frequency_hz) || main_frequency_hz <= 0.0) {
        ROS_WARN("[ReferenceTrajectoryNode] Invalid main_frequency %.3f; using 100.0 Hz",
                 main_frequency_hz);
        main_frequency_hz = 100.0;
    }

    multirotor_reference_trajectory::ReferenceTrajectoryNode node(nh);
    node.run(main_frequency_hz);
    return 0;
}
