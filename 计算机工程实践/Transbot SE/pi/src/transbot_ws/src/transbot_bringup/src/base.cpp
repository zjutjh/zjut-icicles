#include "base.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
std::string str1;
RobotBase::RobotBase() :
        linear_velocity_x_(0),
        linear_velocity_y_(0),
        angular_velocity_z_(0),
        last_vel_time_(0),
        vel_dt_(0),
        x_pos_(0),
        y_pos_(0),
        heading_(0) {
    ros::NodeHandle nh_private("~");
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom_raw", 50);
    velocity_subscriber_ = nh_.subscribe("/transbot/get_vel", 50, &RobotBase::velCallback, this);
    nh_private.getParam("linear_scale", linear_scale_);
    nh_private.getParam("is_namespace", namespace_);
    nh_private.param<bool>("is_multi_robot", is_multi_robot_,false);
    str1 = namespace_;
}

void RobotBase::velCallback(const geometry_msgs::Twist twist) {
//    ROS_INFO("ODOM PUBLISH %.2f,%.2f,%.2f", twist.linear.x, twist.linear.y, twist.angular.z);
    ros::Time current_time = ros::Time::now();
    linear_velocity_x_ = twist.linear.x * linear_scale_;
    linear_velocity_y_ = twist.linear.y * linear_scale_;
    angular_velocity_z_ = twist.angular.z;
    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;
    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m
    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    //odom.header.frame_id = str1+"/odom";
    //odom.child_frame_id = str1+"/base_footprint";
    if(is_multi_robot_==false)
    {
	odom.header.frame_id = "odom";
        odom.child_frame_id = "dummy";
    }
    else
    {
	odom.header.frame_id = str1+"/odom";
        odom.child_frame_id = str1+"/dummy";
    }
    std::cout<<odom.header.frame_id;
    //robot's position in x,y and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
    ROS_INFO("ODOM PUBLISH");
    odom_publisher_.publish(odom);
}
