//
// Created by yahboom on 2021/7/30.
//

#ifndef TRANSBOT_TRACK_KCF_TRACKER_H
#define TRANSBOT_TRACK_KCF_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kcftracker.h"
#include "PID.h"
#include <dynamic_reconfigure/server.h>
#include <transbot_track/KCFTrackerConfig.h>
#include <transbot_msgs/JoyState.h>
#include <time.h>

using namespace std;
using namespace cv;

class ImageConverter {
public:
    ros::Publisher pub;
    PID *linear_PID;
    PID *angular_PID;
    ros::NodeHandle n;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber Joy_sub_;
    ros::Publisher image_pub_;
    const char *RGB_WINDOW = "rgb_img";
    const char *DEPTH_WINDOW = "depth_img";
    float minDist = 1.0;
    float linear_speed = 0;
    float rotation_speed = 0;
    bool enable_get_depth = false;
    float dist_val[5];
    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool LAB = false;
    int center_x;
    KCFTracker tracker;
    dynamic_reconfigure::Server<transbot_track::KCFTrackerConfig> server;
    dynamic_reconfigure::Server<transbot_track::KCFTrackerConfig>::CallbackType f;

    ImageConverter(ros::NodeHandle &n);

    ~ImageConverter();

    void PIDcallback(transbot_track::KCFTrackerConfig &config, uint32_t level);

    void Reset();

    void Cancel();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    void depthCb(const sensor_msgs::ImageConstPtr &msg);

    void JoyCb(const transbot_msgs::JoyStateConstPtr &msg);

};


#endif //TRANSBOT_TRACK_KCF_TRACKER_H
