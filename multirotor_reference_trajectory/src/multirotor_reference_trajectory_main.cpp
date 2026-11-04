#include <ros/ros.h>

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "multirotor_reference_trajectory_node");
    ros::NodeHandle nh;
    multirotor_reference_trajectory::ReferenceTrajectoryNode node(nh);
    node.run();
    return 0;
}
