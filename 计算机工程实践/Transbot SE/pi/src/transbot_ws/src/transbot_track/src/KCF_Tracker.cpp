//
// Created by yahboom on 2021/7/30.
//

#include "KCF_Tracker.h"

Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;

const int &ACTION_ESC = 27;
const int &ACTION_SPACE = 32;

void onMouse(int event, int x, int y, int, void *) {
    if (select_flag) {
        selectRect.x = MIN(origin.x, x);
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);
        selectRect.height = abs(y - origin.y);
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == 1) {
//    if (event == CV_EVENT_LBUTTONDOWN) {
        bBeginKCF = false;
        select_flag = true;
        origin = Point(x, y);
        selectRect = Rect(x, y, 0, 0);
    } else if (event == 4) {
//    } else if (event == CV_EVENT_LBUTTONUP) {
        select_flag = false;
        bRenewROI = true;
    }
}

ImageConverter::ImageConverter(ros::NodeHandle &n) {
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
    float linear_KP=0.9;
    float linear_KI=0.0;
    float linear_KD=0.1;
    float angular_KP=0.5;
    float angular_KI=0.0;
    float angular_KD=0.2;
    this->linear_PID = new PID(linear_KP, linear_KI, linear_KD);
    this->angular_PID = new PID(angular_KP, angular_KI, angular_KD);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = n.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    depth_sub_ = n.subscribe("/camera/depth/image", 1, &ImageConverter::depthCb, this);
    Joy_sub_ = n.subscribe("/JoyState", 1, &ImageConverter::JoyCb, this);
    image_pub_ = n.advertise<sensor_msgs::Image>("/KCF_image", 1);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    f = boost::bind(&ImageConverter::PIDcallback, this, _1, _2);
    pub.publish(geometry_msgs::Twist());
    server.setCallback(f);
    namedWindow(RGB_WINDOW);
//        namedWindow(DEPTH_WINDOW);
    this->linear_PID->Set_PID(linear_KP, linear_KI, linear_KD);
    this->angular_PID->Set_PID(angular_KP, angular_KI, angular_KD);
}

ImageConverter::~ImageConverter() {
    n.shutdown();
    pub.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    delete this->linear_PID;
    delete this->angular_PID;
    destroyWindow(RGB_WINDOW);
//        destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::PIDcallback(transbot_track::KCFTrackerConfig &config, uint32_t level) {
    ROS_INFO("linear_PID: %f %f %f", config.linear_Kp, config.linear_Ki, config.linear_Kd);
    ROS_INFO("angular_PID: %f %f %f", config.angular_Kp, config.angular_Ki, config.angular_Kd);
    minDist=config.minDist;
    this->linear_PID->Set_PID(float(config.linear_Kp), float(config.linear_Ki), float(config.linear_Kd));
    this->angular_PID->Set_PID(float(config.angular_Kp), float(config.angular_Ki), float(config.angular_Kd));
    this->linear_PID->reset();
    this->angular_PID->reset();
}

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    this->linear_PID->reset();
    this->angular_PID->reset();
    pub.publish(geometry_msgs::Twist());
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(rgbimage);
    setMouseCallback(RGB_WINDOW, onMouse, 0);
    if (bRenewROI) {
         if (selectRect.width <= 0 || selectRect.height <= 0)
         {
             bRenewROI = false;
             return;
         }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }
    if (bBeginKCF) {
        result = tracker.update(rgbimage);
        rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
        circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
    } else rectangle(rgbimage, selectRect, Scalar(255, 0, 0), 2, 8, 0);
    sensor_msgs::ImagePtr kcf_imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbimage).toImageMsg();
    image_pub_.publish(kcf_imagemsg);
    imshow(RGB_WINDOW, rgbimage);
    int action = waitKey(1) & 0xFF;
    if (action == 'q' || action == ACTION_ESC) this->Cancel();
    else if (action == 'r')  this->Reset();
    else if (action == ACTION_SPACE) enable_get_depth = true;
}

void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
    if (enable_get_depth) {
        int center_x = (int)(result.x + result.width / 2);
        int center_y = (int)(result.y + result.height / 2);
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5);
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5);
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5);
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5);
        dist_val[4] = depthimage.at<float>(center_y, center_x);
        float distance = 0;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
            if (dist_val[i] > 0.4 && dist_val[i] < 10.0) distance += dist_val[i];
            else num_depth_points--;
        }
        distance /= num_depth_points;
        if (num_depth_points != 0) {
            if (abs(distance - minDist) < 0.1) linear_speed = 0;
            else linear_speed = -linear_PID->compute(minDist, distance);
        }
        rotation_speed = angular_PID->compute(320 / 100.0, center_x / 100.0);
        if (abs(rotation_speed) < 0.1)rotation_speed = 0;
        geometry_msgs::Twist twist;
        twist.linear.x = linear_speed;
        twist.angular.z = rotation_speed;
        pub.publish(twist);
        ROS_WARN("linear = %.3f,angular =  %.3f", linear_speed, rotation_speed);
        ROS_ERROR("distance = %.3f,center_x =  %d", distance, center_x);
    }
//        imshow(DEPTH_WINDOW, depthimage);
    waitKey(1);
}

void ImageConverter::Cancel() {
    this->Reset();
    ros::Duration(0.5).sleep();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    delete this->linear_PID;
    delete this->angular_PID;
    n.shutdown();
    pub.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    destroyWindow(RGB_WINDOW);
//        destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::JoyCb(const transbot_msgs::JoyStateConstPtr &msg) {
    if (msg->state) enable_get_depth = false;
    else { if (bBeginKCF)enable_get_depth = true;}
}




