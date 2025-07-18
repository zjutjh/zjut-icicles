//
// Created by yahboom on 2021/4/29.
//

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void RGB_Callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imshow("color_image", cv_ptr->image);
        waitKey(1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    //ROS节点初始化
    // The ROS node is initialized
    ros::init(argc, argv, "usb_cam_image_cpp");
    //创建节点句柄
    // Create a node handle
    ros::NodeHandle n;
    //创建一个接收者.
    // Create a receiver.
    ros::Subscriber subscriber = n.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 10, RGB_Callback);
    //按照循环频率延时
    // According to the cycle frequency delay
    ros::spin();
    return 0;
}

