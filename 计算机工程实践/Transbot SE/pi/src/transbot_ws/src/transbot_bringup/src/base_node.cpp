#include <ros/ros.h>
#include "base.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_node");
    RobotBase Robot;
    ros::spin();
    return 0;
}
