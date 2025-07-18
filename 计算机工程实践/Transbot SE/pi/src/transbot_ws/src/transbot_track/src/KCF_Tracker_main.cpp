#include <iostream>
#include <ros/ros.h>
#include "KCF_Tracker.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "KCF_Tracker");
    ros::NodeHandle n;
    ImageConverter imageConverter(n);
    ros::spin();
    return 0;
}